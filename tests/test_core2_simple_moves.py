import unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
MAIN = (ROOT / "firmware/core2_panel/src/main.cpp").read_text(encoding="utf-8")
DIRECT = (ROOT / "firmware/core2_panel/src/direct_camera.cpp").read_text(encoding="utf-8")
class TestSimpleMoves(unittest.TestCase):
    def test_start_is_one_runtime_command_with_short_circuit_validation(self):
        block = DIRECT.split("case DirectCamera::MotionCommand::Play:", 1)[1].split("break;", 1)[0]
        self.assertRegex(block, r"armProgram\(in, telemetryFresh\) &&\s+goProgram\(in, telemetryFresh, now\)")
        public = DIRECT.split("bool DirectCamera::requestMotionPlay()", 1)[1].split("bool DirectCamera::requestMotionGo()", 1)[0]
        for guard in ("TELEMETRY_FRESH_MS", "program.count < 2", "evidence.motionProgramActive", "!evidence.motionAtStart"):
            self.assertIn(guard, public)
    def test_hold_cannot_chain_positioning_into_play(self):
        block = MAIN.split("static void motionRunEvent(lv_event_t *e)", 1)[1].split("static void deviceCamerasPressed", 1)[0]
        for guard in ("action != pressedAction", "LV_EVENT_PRESS_LOST", "lv_indev_get_scroll_obj", "HOLD_ACT_MS"):
            self.assertIn(guard, block)
        self.assertIn("requestMotionGoto(0)", block)
        self.assertIn("requestMotionPlay()", block)
        self.assertNotIn("requestMotionArm()", block)
    def test_plain_language_and_recording_separation(self):
        start = MAIN.index("static void refreshMotionWorkspace()")
        block = MAIN[start:MAIN.index("static void refreshDeviceWorkspace()", start)]
        for text in ("GO TO START", "PLAY", "CONTINUE", "STOP MOVE", "CAMERA REC IS SEPARATE"):
            self.assertIn(text, block)
        for text in ('"ARM"', '"DISARM"', "READY TO ARM", "HOLD TO ARM"):
            self.assertNotIn(text, block)
