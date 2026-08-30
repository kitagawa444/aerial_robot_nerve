import importlib.util
from pathlib import Path
import unittest


SCRIPT_PATH = Path(__file__).parents[1] / "scripts" / "joy_to_crsf.py"
SPEC = importlib.util.spec_from_file_location("joy_to_crsf", SCRIPT_PATH)
JOY_TX = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(JOY_TX)


class JoyToCrsfTest(unittest.TestCase):
    def test_ps4_navigation_controls(self):
        axes = [0.0] * 14
        buttons = [0] * 14
        buttons[9] = 1
        buttons[2] = 1
        axes[9] = 1.0

        controls = JOY_TX.navigation_controls(axes, buttons, "ps4")

        self.assertTrue(controls["arm"])
        self.assertTrue(controls["dpad_left"])
        self.assertTrue(controls["circle"])
        self.assertFalse(controls["stop"])

    def test_landing_and_stop_channel_mapping(self):
        axes = [0.0] * 8
        buttons = [0] * 13
        axes[6] = -1.0
        buttons[3] = 1
        buttons[8] = 1
        channels = JOY_TX.safe_channels()

        JOY_TX.apply_controls(
            channels, JOY_TX.navigation_controls(axes, buttons, "bluetooth")
        )

        self.assertEqual(channels[JOY_TX.CHANNEL_STOP], JOY_TX.CRSF_CHANNEL_HIGH)
        self.assertEqual(channels[JOY_TX.CHANNEL_LAND_MODIFIER], JOY_TX.CRSF_CHANNEL_HIGH)
        self.assertEqual(channels[JOY_TX.CHANNEL_LAND_ACTION], JOY_TX.CRSF_CHANNEL_HIGH)

    def test_frame_uses_expresslrs_handset_sync_byte(self):
        frame = JOY_TX.build_rc_channels_frame(JOY_TX.safe_channels())
        self.assertEqual(frame[0], 0xC8)
        self.assertEqual(len(frame), 26)

    def test_ds4_usb_arm_takeoff_controls(self):
        report = bytearray((1, 128, 128, 128, 128, 0x46, 0x20))

        controls = JOY_TX.ds4_usb_navigation_controls(report)

        self.assertTrue(controls["arm"])
        self.assertTrue(controls["dpad_left"])
        self.assertTrue(controls["circle"])
        self.assertFalse(controls["stop"])

    def test_ds4_usb_land_and_stop_controls(self):
        report = bytearray((1, 128, 128, 128, 128, 0x12, 0x10))

        controls = JOY_TX.ds4_usb_navigation_controls(report)

        self.assertTrue(controls["stop"])
        self.assertTrue(controls["dpad_right"])
        self.assertTrue(controls["square"])

    def test_ds4_usb_rejects_non_usb_report(self):
        with self.assertRaisesRegex(ValueError, "report ID"):
            JOY_TX.ds4_usb_navigation_controls(bytes((0x11, 0, 0, 0, 0, 8, 0)))


if __name__ == "__main__":
    unittest.main()
