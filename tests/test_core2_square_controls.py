import unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
MAIN = (ROOT / "firmware/core2_panel/src/main.cpp").read_text(encoding="utf-8")
DIRECT = (ROOT / "firmware/core2_panel/src/direct_camera.cpp").read_text(encoding="utf-8")
class TestSquareControls(unittest.TestCase):
    def test_geometry_has_separate_thumb_gears_and_square_pads(self):
        for pad in ("jogPad", "dial"):
            self.assertIn(f"lv_obj_set_size({pad}, 168, PAGE_H)", MAIN)
            self.assertIn(f"lv_obj_set_pos({pad}, 76, 0)", MAIN)
        for index in range(4):
            self.assertIn(f"&easeGearLabels[{index}], openEase", MAIN)
        for x in (6,250):
            self.assertIn(f"{x}, 112, 64, 56, FONT_22", MAIN)
        self.assertLessEqual(6+64+6,76)
        self.assertLessEqual(76+168+6,250)
        self.assertEqual(112+56,168)
    def test_opening_a_gear_stops_and_blocks_underlying_control(self):
        block=MAIN.split("static void openEase(",1)[1].split("static void easeAdjust",1)[0]
        self.assertIn("stopPressed(nullptr)",block)
        self.assertIn("!handFeelOpen && !easeOpen && screenClutchHeld", MAIN)
        self.assertIn("if (easeOpen) return;", MAIN)
        self.assertIn('if (easeOpen) { closeEasePane(); return; }', MAIN)
    def test_preferences_validate_restore_and_inherit(self):
        for code in ('getBytesLength("ease_v1")', 'storedEase[m][i] < 6',
                     'storedEase[m][6] < 4', 'memcpy(easeValues, previous',
                     'easeValues[m][6] & (1 << axis)', 'easeDirty = false'):
            self.assertIn(code, MAIN)
        self.assertIn("uint16_t milliseconds[2][4]", MAIN)
        self.assertIn("setManualEase(milliseconds)", MAIN)
    def test_release_keeps_fault_and_stop_paths_immediate(self):
        self.assertIn("releasing_ && (!telemetryFresh", DIRECT)
        release = DIRECT.split("bool commandRelease(",1)[1].split("bool sendRecordRequest",1)[0]
        self.assertIn("millis() - releaseAtMs_", release)
        self.assertIn("releaseEaseRate(releasePitch_", release)
        self.assertIn("brakePitch(", release)
        self.assertIn("pitchShaper_.reset(pitch)", release)
        abort = DIRECT.split("if (in.abortEpoch != seenAbort)",1)[1].split("if (in.reconnectEpoch",1)[0]
        self.assertIn("sendCentre()", abort)
        self.assertIn("resetMotion()", abort)
        self.assertNotIn("commandRelease", abort)
    def test_camera_is_in_shooting_cycle_without_fake_follow_focus(self):
        home=MAIN.split("static void homeOrToggleControl()",1)[1].split("static void softMidEvent",1)[0]
        self.assertIn("else if (page == jog)", home)
        self.assertIn("showCameraView(CAM_TOOLS)", home)
        self.assertIn("easeOpen || handFeelOpen || jogFeelOpen", home)
        self.assertIn("LENS POSITION NOT AVAILABLE", MAIN)
