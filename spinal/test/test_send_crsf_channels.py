import importlib.util
from pathlib import Path
import unittest


SCRIPT_PATH = Path(__file__).parents[1] / "scripts" / "send_crsf_channels.py"
SPEC = importlib.util.spec_from_file_location("send_crsf_channels", SCRIPT_PATH)
CRSF_TX = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CRSF_TX)


class SendCrsfChannelsTest(unittest.TestCase):
    def test_safe_defaults(self):
        channels = CRSF_TX.safe_default_channels()

        self.assertEqual(len(channels), 16)
        self.assertEqual(channels[:4], [992, 992, 172, 992])
        self.assertEqual(channels[4:], [172] * 12)

    def test_rc_frame_round_trip(self):
        channels = [172, 992, 1811, 0, 2047, 1, 1024, 511] + [992] * 8
        frame = CRSF_TX.build_rc_channels_frame(channels)

        self.assertEqual(len(frame), 26)
        self.assertEqual(frame[:3], bytes((0xEE, 24, 0x16)))
        self.assertEqual(frame[-1], CRSF_TX.crc8_dvb_s2(frame[2:-1]))

        packed = int.from_bytes(frame[3:-1], byteorder="little")
        decoded = [(packed >> (index * 11)) & 0x7FF for index in range(16)]
        self.assertEqual(decoded, channels)

    def test_rejects_invalid_channel_count_and_values(self):
        with self.assertRaises(ValueError):
            CRSF_TX.pack_channels([992] * 15)
        with self.assertRaises(ValueError):
            CRSF_TX.pack_channels([992] * 15 + [2048])

    def test_channel_override_uses_one_based_indices(self):
        channels = CRSF_TX.safe_default_channels()
        CRSF_TX.apply_channel_overrides(channels, ["3=200", "5=1811"])

        self.assertEqual(channels[2], 200)
        self.assertEqual(channels[4], 1811)

if __name__ == "__main__":
    unittest.main()
