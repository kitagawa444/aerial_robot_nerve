#!/usr/bin/env python3
"""Send fixed CRSF RC channel frames to an ExpressLRS TX module.

This utility lets a Linux PC or VIM4 act as the handset-side CRSF source for
an ExpressLRS transmitter module.  It intentionally has no ROS dependency.
"""

from __future__ import annotations

import argparse
import glob
import os
import signal
import sys
import time
from typing import Iterable, Sequence


CRSF_TRANSMITTER_ADDRESS = 0xEE
CRSF_FRAME_TYPE_RC_CHANNELS_PACKED = 0x16
CRSF_CHANNEL_COUNT = 16
CRSF_CHANNEL_MIN = 172
CRSF_CHANNEL_CENTER = 992
CRSF_CHANNEL_MAX = 1811
CRSF_PAYLOAD_SIZE = 22
# ExpressLRS TX modules auto-detect 115200.  Use it for the Nano TX V2 USB
# bridge because its QinHeng CDC ACM interface stalls at a requested 400000
# baud on the tested Ubuntu host, even though the module-bay UART supports it.
CRSF_DEFAULT_BAUD = 115_200
CRSF_DEFAULT_RATE_HZ = 250.0


def crc8_dvb_s2(data: bytes) -> int:
    """Return the CRSF CRC-8/DVB-S2 checksum (polynomial 0xD5)."""
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def pack_channels(channels: Sequence[int]) -> bytes:
    """Pack 16 unsigned 11-bit CRSF channel values into 22 bytes."""
    if len(channels) != CRSF_CHANNEL_COUNT:
        raise ValueError(f"expected {CRSF_CHANNEL_COUNT} channels, got {len(channels)}")

    packed = 0
    for index, value in enumerate(channels):
        if not 0 <= value <= 0x7FF:
            raise ValueError(f"channel {index + 1} is outside the 11-bit range: {value}")
        packed |= value << (index * 11)
    return packed.to_bytes(CRSF_PAYLOAD_SIZE, byteorder="little")


def build_rc_channels_frame(channels: Sequence[int]) -> bytes:
    """Build one handset-to-transmitter CRSF 0x16 frame."""
    body = bytes((CRSF_FRAME_TYPE_RC_CHANNELS_PACKED,)) + pack_channels(channels)
    # CRSF length covers type, payload and CRC, but not address or length itself.
    length = len(body) + 1
    return bytes((CRSF_TRANSMITTER_ADDRESS, length)) + body + bytes((crc8_dvb_s2(body),))


def safe_default_channels() -> list[int]:
    """Return AETR-style safe defaults: throttle/AUX low, other sticks centred."""
    channels = [CRSF_CHANNEL_CENTER] * CRSF_CHANNEL_COUNT
    channels[2] = CRSF_CHANNEL_MIN  # CH3: throttle in the conventional AETR order
    channels[4:] = [CRSF_CHANNEL_MIN] * (CRSF_CHANNEL_COUNT - 4)
    return channels


def apply_channel_overrides(channels: list[int], overrides: Iterable[str]) -> None:
    for override in overrides:
        try:
            channel_text, value_text = override.split("=", maxsplit=1)
            channel = int(channel_text)
            value = int(value_text)
        except ValueError as error:
            raise ValueError(f"invalid --set value {override!r}; expected CHANNEL=VALUE") from error

        if not 1 <= channel <= CRSF_CHANNEL_COUNT:
            raise ValueError(f"channel must be 1..{CRSF_CHANNEL_COUNT}: {channel}")
        if not 0 <= value <= 0x7FF:
            raise ValueError(f"channel value must be 0..2047: {value}")
        channels[channel - 1] = value


def discover_serial_port() -> str:
    patterns = (
        "/dev/serial/by-id/*1a86*USB_Single_Serial*",
        "/dev/serial/by-id/*QinHeng*USB*Serial*",
    )
    matches = sorted({path for pattern in patterns for path in glob.glob(pattern)})
    if len(matches) == 1:
        return matches[0]
    if len(matches) > 1:
        raise RuntimeError("multiple Nano TX serial ports found; select one with --port")
    if os.path.exists("/dev/ttyACM0"):
        return "/dev/ttyACM0"
    raise RuntimeError("Nano TX serial port not found; specify it with --port")


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Continuously send fixed CRSF channels to an ExpressLRS Nano TX module."
    )
    parser.add_argument(
        "--port",
        help="serial device (auto-detects the QinHeng USB serial device when omitted)",
    )
    parser.add_argument("--baud", type=int, default=CRSF_DEFAULT_BAUD)
    parser.add_argument("--rate", type=float, default=CRSF_DEFAULT_RATE_HZ, metavar="HZ")
    parser.add_argument(
        "--set",
        action="append",
        default=[],
        metavar="CHANNEL=VALUE",
        help="override a raw 11-bit channel value; may be repeated",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        metavar="SECONDS",
        help="stop after this duration; zero means run until Ctrl-C",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print the channel values and encoded frame without opening a serial port",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    if args.rate <= 0:
        print("error: --rate must be greater than zero", file=sys.stderr)
        return 2
    if args.duration < 0:
        print("error: --duration must not be negative", file=sys.stderr)
        return 2

    channels = safe_default_channels()
    try:
        apply_channel_overrides(channels, args.set)
        frame = build_rc_channels_frame(channels)
    except ValueError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    print("channels:", " ".join(f"CH{i + 1}={value}" for i, value in enumerate(channels)))
    print("frame:", frame.hex(" "))
    if args.dry_run:
        return 0

    try:
        port = args.port or discover_serial_port()
    except RuntimeError as error:
        print(f"error: {error}", file=sys.stderr)
        return 1

    try:
        import serial
    except ImportError:
        print("error: pyserial is missing; install the python3-serial apt package", file=sys.stderr)
        return 1

    stop_requested = False

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    period = 1.0 / args.rate
    frames_sent = 0
    print(f"sending on {port} at {args.baud} baud, {args.rate:g} Hz; Ctrl-C stops output")

    try:
        with serial.Serial(
            port=port,
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
        ) as device:
            # Opening some USB serial bridges can take noticeable time.  Start
            # a requested duration only after the port is ready.
            started_at = time.monotonic()
            next_send = started_at
            while not stop_requested:
                now = time.monotonic()
                if args.duration and now - started_at >= args.duration:
                    break
                if now < next_send:
                    time.sleep(next_send - now)
                    continue

                written = device.write(frame)
                if written != len(frame):
                    raise serial.SerialTimeoutException(
                        f"short serial write: wrote {written} of {len(frame)} bytes"
                    )
                frames_sent += 1
                next_send += period
                if next_send < now - period:
                    next_send = now + period
    except (OSError, serial.SerialException) as error:
        print(f"error: serial output failed: {error}", file=sys.stderr)
        return 1

    print(f"stopped after {frames_sent} frames")
    return 0


if __name__ == "__main__":
    sys.exit(main())
