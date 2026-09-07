"""Offline contract fixtures keep the firmware independently testable."""
import json
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[1]
WIRE = json.loads((ROOT / "contracts/camera-control-v1.json").read_text(encoding="utf-8"))
MAIN = (ROOT / "firmware/core2_panel/src/main.cpp").read_text(encoding="utf-8")
DIRECT = (ROOT / "firmware/core2_panel/src/direct_camera.cpp").read_text(encoding="utf-8")


class TestWireContract(unittest.TestCase):
    def test_firmware_actions_match_the_versioned_contract(self):
        names = re.findall(r'say\("B ([A-Za-z_%][A-Za-z0-9_%]*)', MAIN)
        self.assertTrue(names)
        emitted = set()
        for name in names:
            if "%" in name:
                self.assertIn(name, WIRE["action_templates"], name)
            emitted.update(WIRE["action_templates"].get(name, [name]))
        self.assertEqual(sorted(emitted), WIRE["emitted_actions"])

    def test_link_deadman_matches_contract(self):
        self.assertIn(f'now - lastPacketMs_ > {int(WIRE["link_silent_s"] * 1000)}', DIRECT)
        self.assertEqual(WIRE["revision"], 1)
