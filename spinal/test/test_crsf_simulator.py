# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, DRAGON Laboratory, The University of Tokyo

import pytest

from spinal.crsf_simulator import (
    CRSF_CHANNEL_CENTER,
    CRSF_CHANNEL_COUNT,
    CRSF_FRAME_TYPE_RC_CHANNELS_PACKED,
    crsf_crc8,
    normalized_to_crsf,
    pack_rc_channels,
)


def decode_channels(frame):
    payload = frame[3:-1]
    channels = []
    for channel in range(CRSF_CHANNEL_COUNT):
        bit_offset = channel * 11
        byte_offset = bit_offset // 8
        shift = bit_offset % 8
        packed = payload[byte_offset]
        if byte_offset + 1 < len(payload):
            packed |= payload[byte_offset + 1] << 8
        if byte_offset + 2 < len(payload):
            packed |= payload[byte_offset + 2] << 16
        channels.append((packed >> shift) & 0x7FF)
    return channels


def test_pack_rc_channels_round_trip():
    channels = [172, 992, 1812, 500, 1500, 2047, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
    frame = pack_rc_channels(channels)

    assert frame[0] == 0xC8
    assert frame[1] == 24
    assert frame[2] == CRSF_FRAME_TYPE_RC_CHANNELS_PACKED
    assert frame[-1] == crsf_crc8(frame[2:-1])
    assert decode_channels(frame) == channels


def test_normalized_axes_use_spinal_channel_center():
    assert normalized_to_crsf(0.0) == CRSF_CHANNEL_CENTER
    assert normalized_to_crsf(-2.0) == 172
    assert normalized_to_crsf(2.0) == 1812


def test_pack_rejects_wrong_channel_count():
    with pytest.raises(ValueError):
        pack_rc_channels([CRSF_CHANNEL_CENTER] * (CRSF_CHANNEL_COUNT - 1))
