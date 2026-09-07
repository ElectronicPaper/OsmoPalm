"""Safety contracts for Core2's single-owner P1..P24 motion runtime.

A unittest class, not bare pytest functions: unittest discovery found
nothing here and reported OK, so these contracts were never run by the
suite everyone runs.
"""

import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "firmware" / "core2_panel" / "src"
DIRECT = (SRC / "direct_camera.cpp").read_text(encoding="utf-8")
DIRECT += (SRC / "control_math.h").read_text(encoding="utf-8")
HEADER = (SRC / "direct_camera.h").read_text(encoding="utf-8")
MAIN = (SRC / "main.cpp").read_text(encoding="utf-8")


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



class TestCore2MotionRuntime(unittest.TestCase):
    def test_arm_and_go_are_distinct_and_legacy_run_only_arms(self):
        assert "Armed," in HEADER
        assert "requestMotionArm()" in HEADER
        assert "requestMotionGo()" in HEADER
        legacy = function(DIRECT, "bool DirectCamera::requestMotionRun()")
        assert "return requestMotionArm();" in legacy
        assert "requestMotionGo" not in legacy
        # Canvas run surface states the concrete guarded consequence instead
        # of the old R&D-prefixed placeholder.
        assert 'LV_SYMBOL_PLAY " PLAY"' in MAIN



    def test_runtime_owns_abort_program_and_manual_takeover_priority(self):
        ready = function(DIRECT, "bool readyLoop()")
        abort = ready.index("if (in.abortEpoch != seenAbort)")
        request = ready.index("if (in.motionEpoch != seenMotionEpoch)")
        takeover = ready.index("manual takeover: release first")
        rate_tick = ready.index("if (now - lastStick >= STICK_INTERVAL_MS)")
        assert abort < request < takeover < rate_tick
        assert "requireClutchRelease = in.clutch" in ready
        assert "requireJogRelease = takeoverJog.held" in ready



    def test_jog_needs_camera_telemetry_but_not_imu_freshness(self):
        ready = function(DIRECT, "bool readyLoop()")
        assert "(in.jogActive && !telemetryFresh)" in ready
        jog_branch = ready[ready.index("else if (in.jogActive") :]
        jog_branch = jog_branch[: jog_branch.index("commandOk = commandJog") + 32]
        assert "telemetryFresh" in jog_branch
        assert "imuFresh" not in jog_branch



    def test_program_stale_telemetry_stops_at_the_350ms_gate(self):
        assert "TELEMETRY_FRESH_MS = 350" in DIRECT
        ready = function(DIRECT, "bool readyLoop()")
        stale = ready.index("if (programActive_ && !telemetryFresh)")
        reconnect = ready.index("now - noTelemetrySince > 5000")
        assert stale < reconnect
        assert 'faultProgram("program stopped: stale telemetry")' in ready



    def test_closed_loop_path_uses_measured_progress_and_shared_rate_path(self):
        command = function(DIRECT, "bool commandProgram(")
        for term in ("easeRate", "desiredPitch", "desiredYaw", "cameraPitch_",
                     "cameraYaw_", "capVector(", "brakePitch(",
                     "pitchShaper_.step", "yawShaper_.step", "sendRates("):
            assert term in command
        assert "measuredPitch * programPitchLeg_" in command
        assert "measuredYaw * programYawLeg_" in command
        progress = command[command.index("status_.motionProgress") :]
        assert "elapsed / duration" not in progress[:400]



    def test_unqualified_yaw_routes_and_infeasible_timing_fail_closed(self):
        validate = function(DIRECT, "bool validateProgramLeg(")
        assert "PROGRAM_YAW_LEG_LIMIT" in validate
        assert 'reason = "yaw route needs qualification"' in validate
        assert "peakDps > PROGRAM_MAX_DPS" in validate
        assert "transition == MotionTransition::Linear ? 1.0f" in validate
        assert "transition == MotionTransition::Smooth ? 1.5f : 2.0f" in validate
        assert 'reason = "segment too fast; add time"' in validate



    def test_cued_points_hold_until_a_human_go(self):
        assert "Cued," in HEADER
        assert "uint8_t cuePoint = 0;" in HEADER
        go = function(DIRECT, "bool goProgram(")
        # Standing at a cued point, GO means continue -- checked before the
        # "ARM before GO" rejection, or the release would be refused.
        assert go.index("MotionPhase::Cued") < go.index('rejectProgram("ARM before GO")')
        assert "return proceedFromPoint(in, now);" in go
        after = function(DIRECT, "bool afterDwell(")
        assert "here.holdForGo" in after
        assert "status_.motionPhase = MotionPhase::Cued" in after
        assert "status_.cuePoint = static_cast<uint8_t>(programTarget_ + 1)" in after
        command = function(DIRECT, "bool commandProgram(")
        # Holding at a cue sends nothing; the stale-telemetry fault still applies
        # because programActive_ stays true.
        assert "if (status_.motionPhase == MotionPhase::Cued) return true;" in command
        public_go = function(DIRECT, "bool DirectCamera::requestMotionGo()")
        assert "evidence.cuePoint != 0 && evidence.motionProgramActive" in public_go
        stop = function(DIRECT, "void stopProgram(")
        assert "status_.cuePoint = 0;" in stop



    def test_loop_and_bounce_are_runtime_modes_with_validated_legs(self):
        proceed = function(DIRECT, "bool proceedFromPoint(")
        assert "case MotionRepeat::Loop:" in proceed
        assert "case MotionRepeat::Bounce:" in proceed
        # LOOP returns on P1's own incoming transition; BOUNCE reuses the
        # forward twin's time and curve so reverse travel is visually paired.
        assert "beginProgramSegment(0, points[0].moveMs," in proceed
        assert "points[0].transition" in proceed
        assert "const MotionPoint &forwardTwin" in proceed
        assert "forwardTwin.moveMs" in proceed
        assert "forwardTwin.transition" in proceed
        assert "programDir_ = -1" in proceed
        arm = function(DIRECT, "bool armProgram(")
        assert "candidate.repeat == MotionRepeat::Loop" in arm
        assert "candidate.points[0].moveMs" in arm
        # Every leg is validated with the time it will actually be flown at.
        validate = function(DIRECT, "bool validateProgramLeg(")
        assert "uint32_t moveMs," in validate
        assert "MotionTransition transition) const" in validate
        assert "target.moveMs" not in validate
        command = function(DIRECT, "bool commandProgram(")
        assert "programSegmentMs_ / 1000.0f" in command
        assert "target.moveMs / 1000.0f" not in command

    def test_each_leg_freezes_its_incoming_transition_and_curve_derivative(self):
        begin = function(DIRECT, "bool beginProgramSegment(")
        assert "MotionTransition transition" in begin
        assert "programTransition_ = transition < MotionTransition::Count" in begin
        go = function(DIRECT, "bool goProgram(")
        goto = function(DIRECT, "bool gotoProgram(")
        proceed = function(DIRECT, "bool proceedFromPoint(")
        assert "programFrozen_.points[1].transition" in go
        assert "programFrozen_.points[in.motionPoint].transition" in goto
        assert "points[here + 1].transition" in proceed
        command = function(DIRECT, "bool commandProgram(")
        assert "sampleProgramCurve(" in command
        assert "programTransition_" in command
        curve_start = DIRECT.index("ProgramCurveSample sampleProgramCurve(")
        curve = DIRECT[curve_start : DIRECT.index("float wrap180(", curve_start)]
        for name in ("MotionTransition::Smooth", "MotionTransition::Linear",
                     "MotionTransition::EaseIn", "MotionTransition::EaseOut"):
            assert name in curve
        assert "6.0f * s * (1.0f - s) * inverseDuration" in curve
        assert "2.0f * s * inverseDuration" in curve
        assert "2.0f * (1.0f - s) * inverseDuration" in curve

    def test_transition_edits_use_the_same_frozen_program_contract(self):
        assert "setMotionTransition(uint8_t index, MotionTransition transition)" in HEADER
        edit = function(DIRECT, "bool DirectCamera::setMotionTransition(")
        assert "motionStore_.setTransition(index, transition)" in edit
        assert "motionProgramArmed" in edit
        assert "motionProgramActive" in edit



    def test_link_loss_kills_a_programme_through_the_one_owner(self):
        # fail() had its own copy of "the programme died" and forgot the cue,
        # so a cued point survived link loss and the panel kept offering GO.
        fail = function(DIRECT, "bool fail(const char *detail, bool controlFault = false)")
        self.assertIn("stopProgram(MotionPhase::Fault, nullptr)", fail)
        self.assertNotIn("status_.motionPhase = MotionPhase::Fault;", fail)

    def test_the_publish_tells_the_truth_about_the_store(self):
        ready = function(DIRECT, "bool readyLoop()")
        publish = ready[ready.index("if (now - lastStatus >= 100)"):]
        publish = publish[:publish.index("owner_.publish(status_)")]
        for line in ("status_.motionRepeat = static_cast<uint8_t>(stored.repeat);",
                     "status_.motionSlot = stored.slot;",
                     "atPoint(stored.points[0])"):
            self.assertIn(line, publish)
        # Store errors travel in the snapshot; parking them in status_ was
        # clobbered by this very publish before the UI ever saw them.
        for signature in ("bool DirectCamera::captureMotionPoint(",
                          "bool DirectCamera::setMotionRepeat(",
                          "bool DirectCamera::selectMotionSlot("):
            self.assertNotIn("status_.detail", function(DIRECT, signature))

    def test_the_host_is_sent_attitude_not_raw_gyro(self):
        host = function(DIRECT, "HostImu DirectCamera::hostImu() const")
        self.assertIn("gravityBody(q)", host)
        self.assertIn("atan2f(-g.x, sqrtf(g.y * g.y + g.z * g.z))", host)
        sample = function(MAIN, "static void sampleImu()")
        self.assertIn("const osmo::HostImu att = osmo::directCamera.hostImu()", sample)
        self.assertIn('"I %.2f %.2f %.2f %.2f\\r\\n", att.pitch, att.roll, gz, 0.0f', sample)
        self.assertNotIn('gx, gy, gz, 0.0f', sample)

    def test_store_edits_freeze_as_soon_as_arm_or_goto_is_accepted(self):
        arm = function(DIRECT, "bool DirectCamera::requestMotionArm()")
        goto = function(DIRECT, "bool DirectCamera::requestMotionGoto(")
        for body in (arm, goto):
            assert "status_.motionProgram" in body
            assert "++inputs_.motionEpoch" in body
        for signature in ("bool DirectCamera::captureMotionPoint(",
                          "bool DirectCamera::clearMotionPoint(",
                          "bool DirectCamera::setMotionTiming("):
            edit = function(DIRECT, signature)
            assert "motionProgramArmed" in edit
            assert "motionProgramActive" in edit


if __name__ == "__main__":
    unittest.main()
