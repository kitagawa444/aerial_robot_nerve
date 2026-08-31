#!/usr/bin/env python3
"""Read a Linux joystick directly and send selected controls over CRSF.

This program deliberately has no ROS dependency.  It transports the four
motion axes and the same discrete controls used by aerial_robot_navigation for
arm, takeoff, landing, force landing and halt.
"""

from __future__ import annotations

import argparse
import array
import fcntl
import glob
import os
import select
import signal
import struct
import sys
import time
from typing import Sequence

CRSF_SYNC_BYTE = 0xC8
CRSF_FRAME_TYPE_RC_CHANNELS_PACKED = 0x16
CRSF_CHANNEL_COUNT = 16
CRSF_CHANNEL_LOW = 172
CRSF_CHANNEL_CENTER = 992
CRSF_CHANNEL_HIGH = 1811
CRSF_PAYLOAD_SIZE = 22

CHANNEL_LATERAL = 0
CHANNEL_FORWARD = 1
CHANNEL_VERTICAL = 2
CHANNEL_YAW = 3
CHANNEL_ARM = 4
CHANNEL_TAKEOFF_MODIFIER = 5
CHANNEL_TAKEOFF_ACTION = 6
CHANNEL_STOP = 7
CHANNEL_LAND_MODIFIER = 8
CHANNEL_LAND_ACTION = 9

JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
JSIOCGAXES = 0x80016A11
JSIOCGBUTTONS = 0x80016A12
JS_EVENT = struct.Struct("IhBB")
DS4_USB_REPORT_ID = 0x01
DS4_USB_REPORT_MIN_SIZE = 7


def crc8_dvb_s2(data: bytes) -> int:
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def pack_channels(channels: Sequence[int]) -> bytes:
    if len(channels) != CRSF_CHANNEL_COUNT:
        raise ValueError(f"expected {CRSF_CHANNEL_COUNT} channels, got {len(channels)}")
    packed = 0
    for index, value in enumerate(channels):
        if not 0 <= value <= 0x7FF:
            raise ValueError(
                f"channel {index + 1} is outside the 11-bit range: {value}"
            )
        packed |= value << (index * 11)
    return packed.to_bytes(CRSF_PAYLOAD_SIZE, byteorder="little")


def build_rc_channels_frame(channels: Sequence[int]) -> bytes:
    body = bytes((CRSF_FRAME_TYPE_RC_CHANNELS_PACKED,)) + pack_channels(channels)
    return bytes((CRSF_SYNC_BYTE, len(body) + 1)) + body + bytes((crc8_dvb_s2(body),))


def safe_channels() -> list[int]:
    channels = [CRSF_CHANNEL_CENTER] * CRSF_CHANNEL_COUNT
    channels[2] = CRSF_CHANNEL_LOW
    channels[4:] = [CRSF_CHANNEL_LOW] * (CRSF_CHANNEL_COUNT - 4)
    return channels


def detect_layout(axis_count: int, button_count: int) -> str:
    layouts = {
        (29, 17): "general",
        (14, 14): "ps4",
        (8, 13): "bluetooth",
        (8, 11): "rog1",
    }
    try:
        return layouts[(axis_count, button_count)]
    except KeyError as error:
        raise ValueError(
            f"unsupported joystick shape: axes={axis_count}, buttons={button_count}; "
            "select --layout and verify the device mapping"
        ) from error


def navigation_controls(
    axes: Sequence[float], buttons: Sequence[int], layout: str = "auto"
) -> dict[str, bool]:
    if layout == "auto":
        layout = detect_layout(len(axes), len(buttons))

    if layout == "general":
        require_shape(axes, buttons, 29, 17, layout)
        return {
            "arm": bool(buttons[3]),
            "stop": bool(buttons[0]),
            "dpad_left": bool(buttons[7]),
            "dpad_right": bool(buttons[5]),
            "circle": bool(buttons[13]),
            "square": bool(buttons[15]),
        }
    if layout == "ps4":
        require_shape(axes, buttons, 14, 14, layout)
        return {
            "arm": bool(buttons[9]),
            "stop": bool(buttons[8]),
            "dpad_left": axes[9] > 0.5,
            "dpad_right": axes[9] < -0.5,
            "circle": bool(buttons[2]),
            "square": bool(buttons[0]),
        }
    if layout == "bluetooth":
        require_shape(axes, buttons, 8, 13, layout)
        return {
            "arm": bool(buttons[9]),
            "stop": bool(buttons[8]),
            "dpad_left": axes[6] > 0.5,
            "dpad_right": axes[6] < -0.5,
            "circle": bool(buttons[1]),
            "square": bool(buttons[3]),
        }
    if layout == "rog1":
        require_shape(axes, buttons, 8, 11, layout)
        return {
            "arm": bool(buttons[7]),
            "stop": bool(buttons[6]),
            "dpad_left": axes[6] > 0.5,
            "dpad_right": axes[6] < -0.5,
            "circle": bool(buttons[1]),
            "square": bool(buttons[2]),
        }
    raise ValueError(f"unknown joystick layout: {layout}")


def navigation_motion(
    axes: Sequence[float], buttons: Sequence[int], layout: str = "auto"
) -> tuple[float, ...]:
    """Return lateral, forward, vertical and yaw commands in [-1, 1]."""
    if layout == "auto":
        layout = detect_layout(len(axes), len(buttons))

    if layout == "general":
        require_shape(axes, buttons, 29, 17, layout)
        indices = (0, 1, 3, 2)
    elif layout == "ps4":
        require_shape(axes, buttons, 14, 14, layout)
        indices = (0, 1, 5, 2)
    elif layout in ("bluetooth", "rog1"):
        minimum_buttons = 13 if layout == "bluetooth" else 11
        require_shape(axes, buttons, 8, minimum_buttons, layout)
        indices = (0, 1, 4, 3)
    else:
        raise ValueError(f"unknown joystick layout: {layout}")

    return tuple(max(-1.0, min(1.0, float(axes[index]))) for index in indices)


def ds4_usb_navigation_controls(report: bytes) -> dict[str, bool]:
    """Decode the controls used by navigation from a wired DS4 HID report."""
    if len(report) < DS4_USB_REPORT_MIN_SIZE:
        raise ValueError(f"short DualShock 4 USB report: {len(report)} bytes")
    if report[0] != DS4_USB_REPORT_ID:
        raise ValueError(f"unexpected DualShock 4 USB report ID: 0x{report[0]:02x}")

    face_and_dpad = report[5]
    auxiliary_buttons = report[6]
    dpad = face_and_dpad & 0x0F
    return {
        "arm": bool(auxiliary_buttons & 0x20),  # OPTIONS / START
        "stop": bool(auxiliary_buttons & 0x10),  # SHARE / STOP
        "dpad_left": dpad in (5, 6, 7),
        "dpad_right": dpad in (1, 2, 3),
        "circle": bool(face_and_dpad & 0x40),
        "square": bool(face_and_dpad & 0x10),
    }


def ds4_usb_navigation_motion(report: bytes) -> tuple[float, ...]:
    """Decode lateral, forward, vertical and yaw from a wired DS4 report."""
    if len(report) < DS4_USB_REPORT_MIN_SIZE:
        raise ValueError(f"short DualShock 4 USB report: {len(report)} bytes")
    if report[0] != DS4_USB_REPORT_ID:
        raise ValueError(f"unexpected DualShock 4 USB report ID: 0x{report[0]:02x}")

    # DS4 bytes increase toward right/down.  Navigation axes are positive
    # toward left/up, matching the Linux joystick layouts above.
    def left_or_up(value: int) -> float:
        return max(-1.0, min(1.0, (127.5 - value) / 127.5))

    return (
        left_or_up(report[1]),
        left_or_up(report[2]),
        left_or_up(report[4]),
        left_or_up(report[3]),
    )


def require_shape(
    axes: Sequence[float],
    buttons: Sequence[int],
    minimum_axes: int,
    minimum_buttons: int,
    layout: str,
) -> None:
    if len(axes) < minimum_axes or len(buttons) < minimum_buttons:
        raise ValueError(
            f"{layout} needs at least {minimum_axes} axes and {minimum_buttons} buttons; "
            f"got {len(axes)} and {len(buttons)}"
        )


def normalized_channel(value: float) -> int:
    value = max(-1.0, min(1.0, float(value)))
    span = (
        CRSF_CHANNEL_HIGH - CRSF_CHANNEL_CENTER
        if value >= 0.0
        else CRSF_CHANNEL_CENTER - CRSF_CHANNEL_LOW
    )
    return round(CRSF_CHANNEL_CENTER + value * span)


def apply_controls(
    channels: list[int],
    controls: dict[str, bool],
    motion: Sequence[float] | None = None,
) -> None:
    def discrete(active: bool) -> int:
        return CRSF_CHANNEL_HIGH if active else CRSF_CHANNEL_LOW

    if motion is not None:
        if len(motion) != 4:
            raise ValueError(f"expected four motion axes, got {len(motion)}")
        channels[CHANNEL_LATERAL] = normalized_channel(motion[0])
        channels[CHANNEL_FORWARD] = normalized_channel(motion[1])
        channels[CHANNEL_VERTICAL] = normalized_channel(motion[2])
        channels[CHANNEL_YAW] = normalized_channel(motion[3])

    channels[CHANNEL_ARM] = discrete(controls["arm"])
    channels[CHANNEL_TAKEOFF_MODIFIER] = discrete(controls["dpad_left"])
    channels[CHANNEL_TAKEOFF_ACTION] = discrete(controls["circle"])
    channels[CHANNEL_STOP] = discrete(controls["stop"])
    channels[CHANNEL_LAND_MODIFIER] = discrete(controls["dpad_right"])
    channels[CHANNEL_LAND_ACTION] = discrete(controls["square"])


def joystick_shape(device: int) -> tuple[int, int]:
    axes = array.array("B", [0])
    buttons = array.array("B", [0])
    fcntl.ioctl(device, JSIOCGAXES, axes, True)
    fcntl.ioctl(device, JSIOCGBUTTONS, buttons, True)
    return axes[0], buttons[0]


def discover_serial_port() -> str:
    patterns = (
        "/dev/serial/by-id/*1a86*USB_Single_Serial*",
        "/dev/serial/by-id/*QinHeng*USB*Serial*",
    )
    matches = sorted({path for pattern in patterns for path in glob.glob(pattern)})
    if len(matches) == 1:
        return matches[0]
    if len(matches) > 1:
        raise RuntimeError(
            "multiple Nano TX serial ports found; select one with --port"
        )
    if os.path.exists("/dev/ttyACM0"):
        return "/dev/ttyACM0"
    raise RuntimeError("Nano TX serial port not found; specify it with --port")


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--joy", default="/dev/input/js0", help="Linux joystick device")
    parser.add_argument("--port", help="Nano TX serial device")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--rate", type=float, default=250.0, metavar="HZ")
    parser.add_argument(
        "--layout",
        choices=("auto", "general", "ps4", "bluetooth", "rog1", "ds4-hidraw"),
        default="auto",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    if args.rate <= 0:
        print("error: --rate must be greater than zero", file=sys.stderr)
        return 2

    try:
        import serial
    except ImportError:
        print("error: pyserial is missing; install python3-serial", file=sys.stderr)
        return 1

    try:
        serial_port = args.port or discover_serial_port()
        joy_fd = os.open(args.joy, os.O_RDONLY | os.O_NONBLOCK)
    except (OSError, RuntimeError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1

    stop_requested = False

    def stop(_signum: int, _frame: object) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    try:
        hidraw_input = os.path.basename(args.joy).startswith("hidraw")
        if hidraw_input:
            if args.layout not in ("auto", "ds4-hidraw"):
                raise ValueError("a hidraw joystick requires --layout ds4-hidraw")
            layout = "ds4-hidraw"
            axis_count = 0
            button_count = 0
            neutral_report = bytes((1, 128, 128, 128, 128, 8, 0))
            controls = ds4_usb_navigation_controls(neutral_report)
            motion = ds4_usb_navigation_motion(neutral_report)
        else:
            if args.layout == "ds4-hidraw":
                raise ValueError("--layout ds4-hidraw requires a /dev/hidraw device")
            axis_count, button_count = joystick_shape(joy_fd)
            layout = (
                detect_layout(axis_count, button_count)
                if args.layout == "auto"
                else args.layout
            )
            axes = [0.0] * axis_count
            buttons = [0] * button_count
        period = 1.0 / args.rate
        next_send = time.monotonic()

        shape = (
            layout
            if hidraw_input
            else f"{axis_count} axes, {button_count} buttons, {layout}"
        )
        print(f"joy={args.joy} ({shape}); tx={serial_port} at {args.baud} baud")
        with serial.Serial(
            port=serial_port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0,
            write_timeout=1.0,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
            exclusive=True,
        ) as tx:
            while not stop_requested:
                now = time.monotonic()
                timeout = max(0.0, min(next_send - now, 0.1))
                readable, _, _ = select.select([joy_fd], [], [], timeout)
                if readable:
                    data = os.read(joy_fd, 256 if hidraw_input else JS_EVENT.size * 64)
                    if not data:
                        raise OSError("joystick disconnected")
                    if hidraw_input:
                        controls = ds4_usb_navigation_controls(data)
                        motion = ds4_usb_navigation_motion(data)
                    else:
                        for offset in range(
                            0, len(data) - JS_EVENT.size + 1, JS_EVENT.size
                        ):
                            _timestamp, value, event_type, number = (
                                JS_EVENT.unpack_from(data, offset)
                            )
                            event_type &= ~JS_EVENT_INIT
                            if event_type == JS_EVENT_AXIS and number < len(axes):
                                axes[number] = max(-1.0, min(1.0, value / 32767.0))
                            elif event_type == JS_EVENT_BUTTON and number < len(
                                buttons
                            ):
                                buttons[number] = 1 if value else 0

                now = time.monotonic()
                if now >= next_send:
                    channels = safe_channels()
                    if not hidraw_input:
                        controls = navigation_controls(axes, buttons, layout)
                        motion = navigation_motion(axes, buttons, layout)
                    apply_controls(channels, controls, motion)
                    frame = build_rc_channels_frame(channels)
                    if tx.write(frame) != len(frame):
                        raise serial.SerialTimeoutException("short CRSF serial write")
                    next_send += period
                    if next_send < now - period:
                        next_send = now + period
    except (OSError, ValueError, serial.SerialException) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    finally:
        os.close(joy_fd)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
