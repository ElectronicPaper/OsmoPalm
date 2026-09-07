"""Source contracts for the standalone Core2 motion-program store."""

import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "firmware" / "core2_panel" / "src"
HEADER = (SRC / "motion_program.h").read_text(encoding="utf-8")
CODE = (SRC / "motion_program.cpp").read_text(encoding="utf-8")


def function(text: str, signature: str) -> str:
    start = text.index(signature)
    opening = text.index("{", start)
    depth = 0
    for at in range(opening, len(text)):
        if text[at] == "{":
            depth += 1
        elif text[at] == "}":
            depth -= 1
            if depth == 0:
                return text[start : at + 1]
    raise AssertionError(f"unterminated function {signature}")


class TestCore2MotionProgramContract(unittest.TestCase):
    def test_program_is_bounded_and_has_nonsecret_diagnostics(self):
        self.assertIn("MOTION_POINT_CAPACITY = 24", HEADER)
        program = HEADER[HEADER.index("struct MotionProgram"):
                         HEADER.index("class MotionProgramStore")]
        for field in ("version", "count", "points[MOTION_POINT_CAPACITY]",
                      "error[48]"):
            self.assertIn(field, program)
        self.assertNotRegex(CODE.lower(),
                            r"password|passphrase|credential|\\bble\\b|\\bwifi\\b")

    def test_capture_is_contiguous_and_clearing_truncates_the_tail(self):
        capture = function(CODE, "bool MotionProgramStore::capture(")
        clear = function(CODE, "bool MotionProgramStore::clear(")
        self.assertIn("index > program_.count", capture)
        self.assertIn("index == program_.count", capture)
        self.assertIn("++program_.count", capture)
        self.assertIn("for (uint8_t i = index; i < MOTION_POINT_CAPACITY; ++i)", clear)
        self.assertIn("program_.count = index", clear)

    def test_angles_and_timing_are_validated_before_persisting(self):
        capture = function(CODE, "bool MotionProgramStore::capture(")
        timing = function(CODE, "bool MotionProgramStore::setTiming(")
        self.assertIn("std::isfinite(pitch)", capture)
        self.assertIn("pitch < -180.0f", capture)
        self.assertIn("yaw > 180.0f", capture)
        self.assertIn("MIN_MOVE_MS = 500", CODE)
        self.assertIn("MAX_MOVE_MS = 30000", CODE)
        self.assertIn("MAX_DWELL_MS = 10000", CODE)
        self.assertIn("!validTiming(moveMs, dwellMs)", timing)

    def test_persistence_is_namespaced_versioned_and_checksum_guarded(self):
        # begin() now only reads the slot choice; load() owns validation.
        begin = function(CODE, "bool MotionProgramStore::load(")
        self.assertIn('NVS_NAMESPACE[] = "osmo-motion"', CODE)
        self.assertIn("MOTION_MAGIC", CODE)
        self.assertIn("MOTION_SCHEMA", CODE)
        self.assertIn("uint32_t checksum", CODE)
        self.assertIn("stored.checksum != checksum(stored)", CODE)
        self.assertNotIn("__attribute__((packed))", CODE)
        self.assertIn("StoredMotionProgram stored = {}", begin)
        self.assertIn("prefs.begin(NVS_NAMESPACE, false)", begin)
        self.assertIn("program_ = emptyProgram(slot)", begin)
        self.assertIn('setError("motion data invalid")', begin)
        self.assertIn("LEGACY_NVS_KEY_PREFIX", begin)
        self.assertIn("LegacyStoredMotionProgram legacy = {}", begin)
        self.assertIn("validLegacyStored(legacy)", begin)
        self.assertIn("MotionTransition::Smooth", begin)
        self.assertIn("if (save()) return true", begin)
        self.assertIn('setError("motion migration failed")', begin)

    def test_cues_repeat_and_slots_persist_with_rollback(self):
        self.assertIn("bool holdForGo = false;", HEADER)
        self.assertIn("enum class MotionRepeat : uint8_t { Once = 0, Loop, Bounce, Count };",
                      HEADER)
        self.assertIn("MOTION_SLOT_COUNT = 2", HEADER)
        self.assertIn("MOTION_SCHEMA = 3", CODE)
        self.assertIn('NVS_KEY_PREFIX[] = "program-v3-"', CODE)
        self.assertIn('LEGACY_NVS_KEY_PREFIX[] = "program-v2-"', CODE)
        self.assertIn('NVS_SLOT_KEY[] = "slot"', CODE)
        stored = CODE[CODE.index("struct StoredMotionProgram"):CODE.index("uint32_t checksum(")]
        self.assertIn("uint8_t repeat;", stored)
        for signature in ("bool MotionProgramStore::setHold(",
                          "bool MotionProgramStore::setRepeat("):
            body = function(CODE, signature)
            self.assertIn("const MotionProgram previous = program_", body)
            self.assertIn("if (save()) return true", body)
            self.assertIn("program_ = previous", body)
        select = function(CODE, "bool MotionProgramStore::selectSlot(")
        # A damaged record opens empty with its error attached, so the first
        # capture repairs it; only unavailable storage sends the operator back.
        self.assertIn('std::strcmp(program_.error, "motion data invalid") != 0', select)
        self.assertIn("return loaded;", select)
        # Switching never writes the programme being left; it only records
        # the choice, and a failed load puts the visible programme back.
        self.assertNotIn("save()", select.replace("saveSlotChoice()", ""))
        self.assertIn("const bool loaded = load(slot);", select)
        self.assertIn("program_ = previous", select)
        self.assertIn("saveSlotChoice()", select)

    def test_transitions_are_bounded_persisted_and_migrate_from_v2(self):
        self.assertIn("enum class MotionTransition : uint8_t", HEADER)
        self.assertIn("Smooth = 0", HEADER)
        self.assertIn("EaseIn", HEADER)
        self.assertIn("EaseOut", HEADER)
        self.assertIn("MotionTransition transition = MotionTransition::Smooth", HEADER)
        self.assertIn("bool setTransition(uint8_t index, MotionTransition transition);", HEADER)
        setter = function(CODE, "bool MotionProgramStore::setTransition(")
        self.assertIn("transition >= MotionTransition::Count", setter)
        self.assertIn("const MotionProgram previous = program_", setter)
        self.assertIn("program_ = previous", setter)
        self.assertIn("struct LegacyMotionPoint", CODE)
        self.assertIn("struct LegacyStoredMotionProgram", CODE)
        self.assertIn("LEGACY_MOTION_SCHEMA = 2", CODE)
        self.assertIn("validLegacyPoint", CODE)
        self.assertIn("legacyChecksum", CODE)
        self.assertIn("point.transition = MotionTransition::Smooth", CODE)

    def test_all_mutations_rollback_ram_if_nvs_save_fails(self):
        for signature in ("bool MotionProgramStore::capture(",
                          "bool MotionProgramStore::clear(",
                          "bool MotionProgramStore::setTiming(",
                          "bool MotionProgramStore::reset()"):
            body = function(CODE, signature)
            self.assertIn("const MotionProgram previous = program_", body)
            self.assertIn("if (save()) return true", body)
            self.assertIn("program_ = previous", body)
            self.assertIn('setError("motion save failed")', body)


if __name__ == "__main__":
    unittest.main()
