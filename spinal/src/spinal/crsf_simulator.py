#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, DRAGON Laboratory, The University of Tokyo

"""CRSF receiver simulator backed by a POSIX pseudo-terminal."""

from __future__ import annotations

import errno
import os
import pty
import time
import tty
from pathlib import Path
from typing import Iterable

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool, Trigger

CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8
CRSF_FRAME_TYPE_LINK_STATISTICS = 0x14
CRSF_FRAME_TYPE_RC_CHANNELS_PACKED = 0x16
CRSF_CHANNEL_COUNT = 16
CRSF_CHANNEL_MIN = 172
CRSF_CHANNEL_CENTER = 992
CRSF_CHANNEL_MAX = 1812


def crsf_crc8(data: Iterable[int]) -> int:
    """Return the CRSF DVB-S2 CRC-8 used by the Spinal parser."""
    crc = 0
    for value in data:
        crc ^= int(value)
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def make_crsf_frame(frame_type: int, payload: bytes) -> bytes:
    body = bytes([frame_type]) + payload
    length = len(body) + 1
    return bytes([CRSF_ADDRESS_FLIGHT_CONTROLLER, length]) + body + bytes([crsf_crc8(body)])


def pack_rc_channels(channels: Iterable[int]) -> bytes:
    """Pack sixteen unsigned 11-bit channels into a CRSF RC frame."""
    values = [int(value) for value in channels]
    if len(values) != CRSF_CHANNEL_COUNT:
        raise ValueError(f"expected {CRSF_CHANNEL_COUNT} channels, got {len(values)}")
    if any(value < 0 or value > 0x7FF for value in values):
        raise ValueError("CRSF channel values must be in the range 0..2047")

    payload = bytearray(22)
    for channel, value in enumerate(values):
        bit_offset = channel * 11
        byte_offset = bit_offset // 8
        shift = bit_offset % 8
        packed = value << shift
        payload[byte_offset] |= packed & 0xFF
        if byte_offset + 1 < len(payload):
            payload[byte_offset + 1] |= (packed >> 8) & 0xFF
        if byte_offset + 2 < len(payload):
            payload[byte_offset + 2] |= (packed >> 16) & 0xFF
    return make_crsf_frame(CRSF_FRAME_TYPE_RC_CHANNELS_PACKED, bytes(payload))


def pack_link_statistics(link_quality: int = 100) -> bytes:
    quality = max(0, min(100, int(link_quality)))
    payload = bytes([50, 50, quality, 0, 0, 2, 1, 0, 0, 0])
    return make_crsf_frame(CRSF_FRAME_TYPE_LINK_STATISTICS, payload)


def normalized_to_crsf(value: float) -> int:
    normalized = max(-1.0, min(1.0, float(value)))
    return round(CRSF_CHANNEL_CENTER + (CRSF_CHANNEL_MAX - CRSF_CHANNEL_CENTER) * normalized)


class CrsfSimulator(Node):
    """Continuously write valid CRSF frames into the Gazebo serial input."""

    def __init__(self) -> None:
        super().__init__("crsf_simulator")
        self.declare_parameter("device_link", "/tmp/spinal_crsf_sim")
        self.declare_parameter("frame_rate", 100.0)
        self.declare_parameter("joy_timeout", 0.5)

        self._device_link = Path(self.get_parameter("device_link").value)
        frame_rate = float(self.get_parameter("frame_rate").value)
        if frame_rate <= 10.0:
            raise ValueError("frame_rate must be greater than 10 Hz to avoid the 100 ms CRSF timeout")
        self._joy_timeout = max(0.0, float(self.get_parameter("joy_timeout").value))

        self._channels = [CRSF_CHANNEL_CENTER] * 4 + [CRSF_CHANNEL_MIN] * 12
        self._pulse_deadlines: dict[int, float] = {}
        self._last_joy_time = 0.0
        self._link_enabled = True
        self._frame_sequence = 0
        self._master_fd = -1
        self._slave_fd = -1
        self._slave_path = ""
        self._open_pty()

        self.create_subscription(Joy, "rc/sim/joy", self._joy_callback, 10)
        self.create_service(Trigger, "rc/sim/arm", self._arm_callback)
        self.create_service(Trigger, "rc/sim/takeoff", self._takeoff_callback)
        self.create_service(Trigger, "rc/sim/land", self._land_callback)
        self.create_service(Trigger, "rc/sim/force_landing", self._force_landing_callback)
        self.create_service(Trigger, "rc/sim/halt", self._halt_callback)
        self.create_service(Trigger, "rc/sim/reset", self._reset_callback)
        self.create_service(SetBool, "rc/sim/link", self._link_callback)
        self.create_timer(1.0 / frame_rate, self._write_frame)

        self.get_logger().info(
            f"Simulated CRSF receiver ready: {self._device_link} -> {self._slave_path} at {frame_rate:.1f} Hz"
        )

    def _open_pty(self) -> None:
        self._device_link.parent.mkdir(parents=True, exist_ok=True)
        if self._device_link.exists() and not self._device_link.is_symlink():
            raise RuntimeError(f"refusing to replace non-symlink device path: {self._device_link}")
        if self._device_link.is_symlink():
            self._device_link.unlink()

        self._master_fd, self._slave_fd = pty.openpty()
        tty.setraw(self._slave_fd)
        os.set_blocking(self._master_fd, False)
        self._slave_path = os.ttyname(self._slave_fd)
        self._device_link.symlink_to(self._slave_path)

    def _joy_callback(self, message: Joy) -> None:
        for index in range(min(4, len(message.axes))):
            self._channels[index] = normalized_to_crsf(message.axes[index])
        self._last_joy_time = time.monotonic()

    def _pulse(self, indices: Iterable[int], duration: float, response: Trigger.Response) -> Trigger.Response:
        deadline = time.monotonic() + duration
        for index in indices:
            self._pulse_deadlines[index] = deadline
        response.success = True
        response.message = f"scheduled {duration:.2f} s CRSF channel pulse"
        return response

    def _arm_callback(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        return self._pulse([4], 0.25, response)

    def _takeoff_callback(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        return self._pulse([5, 6], 0.25, response)

    def _land_callback(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        return self._pulse([8, 9], 0.25, response)

    def _force_landing_callback(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        return self._pulse([7], 0.25, response)

    def _halt_callback(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        return self._pulse([7], 1.25, response)

    def _reset_callback(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._channels = [CRSF_CHANNEL_CENTER] * 4 + [CRSF_CHANNEL_MIN] * 12
        self._pulse_deadlines.clear()
        self._last_joy_time = 0.0
        self._link_enabled = True
        response.success = True
        response.message = "CRSF controls reset to neutral and link enabled"
        return response

    def _link_callback(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        self._link_enabled = request.data
        response.success = True
        response.message = "CRSF link enabled" if request.data else "CRSF link disabled"
        return response

    def _write_frame(self) -> None:
        if not self._link_enabled:
            return

        now = time.monotonic()
        if self._joy_timeout > 0.0 and self._last_joy_time > 0.0 and now - self._last_joy_time > self._joy_timeout:
            self._channels[:4] = [CRSF_CHANNEL_CENTER] * 4
            self._last_joy_time = 0.0

        channels = list(self._channels)
        expired = []
        for index, deadline in self._pulse_deadlines.items():
            if now < deadline:
                channels[index] = CRSF_CHANNEL_MAX
            else:
                expired.append(index)
        for index in expired:
            del self._pulse_deadlines[index]

        try:
            os.write(self._master_fd, pack_rc_channels(channels))
            self._frame_sequence += 1
            if self._frame_sequence % 10 == 0:
                os.write(self._master_fd, pack_link_statistics())
        except OSError as error:
            if error.errno not in (errno.EAGAIN, errno.EWOULDBLOCK, errno.EIO):
                raise

    def destroy_node(self) -> bool:
        if self._device_link.is_symlink() and os.readlink(self._device_link) == self._slave_path:
            self._device_link.unlink()
        if self._master_fd >= 0:
            os.close(self._master_fd)
            self._master_fd = -1
        if self._slave_fd >= 0:
            os.close(self._slave_fd)
            self._slave_fd = -1
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = CrsfSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
