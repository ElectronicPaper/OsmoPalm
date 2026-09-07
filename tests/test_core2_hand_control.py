"""Deterministic reference checks for the Core2 hand-control laws.

These tests deliberately avoid camera I/O.  They bind the safety-relevant
constants to the firmware source and exercise the same equations over jittered
control intervals, arbitrary grab attitudes, and endpoint headroom.
"""

from __future__ import annotations

import math
import pathlib
import re
import statistics
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "firmware/core2_panel/src/direct_camera.cpp").read_text(
    encoding="utf-8"
)


SOURCE += (ROOT / "firmware/core2_panel/src/control_math.h").read_text(encoding="utf-8")


def _float_array(name: str) -> tuple[float, ...]:
    match = re.search(
        rf"constexpr float {name}\[\]\s*=\s*\{{([^}}]+)\}};", SOURCE
    )
    if not match:
        raise AssertionError(f"firmware array {name} was not found")
    return tuple(float(token.rstrip("f")) for token in match.group(1).split(","))


SENSITIVITIES = _float_array("RESPONSE_GAINS")
SPEED_CAPS = _float_array("SPEED_CAPS")
AXIS_RESPONSE = SENSITIVITIES
AXIS_STABILITY = _float_array("STABILITY_FACTORS")
MAX_ACCEL = _float_array("maxAccel")
MAX_JERK = _float_array("maxJerk")
COAST = float(
    re.search(r"constexpr float CAMERA_COAST_S\s*=\s*([0-9.]+)f;", SOURCE).group(1)
)
AIR_MOUSE_MIN_CUTOFF = _float_array("AIR_MOUSE_MIN_CUTOFF")
AIR_MOUSE_BETA = _float_array("AIR_MOUSE_BETA")
PRECISION_MIN_CUTOFF = _float_array("PRECISION_MIN_CUTOFF")
PRECISION_BETA = _float_array("PRECISION_BETA")
PRECISION_GATE_ENTER_DPS = _float_array("PRECISION_GATE_ENTER_DPS")
PRECISION_GATE_EXIT_DPS = _float_array("PRECISION_GATE_EXIT_DPS")


def _float_constant(name: str) -> float:
    match = re.search(rf"constexpr float {name}\s*=\s*([0-9.]+)f;", SOURCE)
    if not match:
        raise AssertionError(f"firmware constant {name} was not found")
    return float(match.group(1))


AIR_MOUSE_DERIVATIVE_CUTOFF = _float_constant(
    "AIR_MOUSE_DERIVATIVE_CUTOFF"
)
AIR_MOUSE_RATE_GATE_ENTER_DPS = _float_constant(
    "AIR_MOUSE_RATE_GATE_ENTER_DPS"
)
AIR_MOUSE_RATE_GATE_EXIT_DPS = _float_constant(
    "AIR_MOUSE_RATE_GATE_EXIT_DPS"
)
PRECISION_DERIVATIVE_CUTOFF = _float_constant(
    "PRECISION_DERIVATIVE_CUTOFF"
)


class AxisShaper:
    """Python reference for the firmware's bounded velocity shaper."""

    def __init__(self) -> None:
        self.value = 0.0
        self.accel = 0.0

    def step(self, target: float, dt: float, smooth: int,
             stability: int = 1) -> float:
        dt = max(0.015, min(0.08, dt))
        factor = AXIS_STABILITY[stability]
        max_accel = MAX_ACCEL[smooth] * factor
        max_jerk = MAX_JERK[smooth] * factor
        wanted = max(-max_accel, min(max_accel,
                                             (target - self.value) / dt))
        accel_step = max_jerk * dt
        self.accel += max(-accel_step, min(accel_step, wanted - self.accel))
        self.accel = max(-max_accel, min(max_accel, self.accel))
        next_value = self.value + self.accel * dt
        if (target - self.value) * (target - next_value) <= 0.0:
            next_value = target
            self.accel = 0.0
        self.value = next_value
        return self.value


class PrecisionIntentGate:
    """Reference for Fine's adaptive Schmitt intent conditioner."""

    MINIMUM_DPS = 0.35

    def __init__(self, stability: int = 1) -> None:
        self.stability = stability
        self.reset()

    def reset(self) -> None:
        self.initialized = False
        self.live = False
        self.direction = 0
        self.raw = self.derivative = self.filtered = 0.0

    @staticmethod
    def alpha(cutoff: float, dt: float) -> float:
        return 1.0 / (1.0 + 1.0 / (2.0 * math.pi * cutoff * dt))

    def step(self, rate_dps: float, dt: float, enabled: bool = True) -> float:
        if not enabled:
            self.reset()
            return rate_dps
        if not math.isfinite(rate_dps) or abs(rate_dps) < 0.0001:
            self.reset()
            return 0.0
        dt = max(0.015, min(0.08, dt))
        if not self.initialized:
            self.initialized = True
            self.raw = self.derivative = self.filtered = 0.0
        d_alpha = self.alpha(PRECISION_DERIVATIVE_CUTOFF, dt)
        self.derivative += d_alpha * (
            (rate_dps - self.raw) / dt - self.derivative
        )
        self.raw = rate_dps
        cutoff = (
            PRECISION_MIN_CUTOFF[self.stability]
            + PRECISION_BETA[self.stability] * abs(self.derivative)
        )
        self.filtered += self.alpha(cutoff, dt) * (
            rate_dps - self.filtered
        )
        magnitude = abs(self.filtered)
        next_direction = -1 if self.filtered < 0.0 else 1
        if self.live and next_direction != self.direction:
            self.live = False
            self.direction = 0
        if not self.live:
            if magnitude < PRECISION_GATE_ENTER_DPS[self.stability]:
                return 0.0
            self.live = True
            self.direction = next_direction
        elif magnitude <= PRECISION_GATE_EXIT_DPS[self.stability]:
            self.live = False
            self.direction = 0
            return 0.0
        return self.direction * max(magnitude, self.MINIMUM_DPS)


class AirMouseFilter:
    """Reference for the firmware's shared-cutoff two-axis 1 Euro filter."""

    def __init__(self, smooth: int = 1, beta: float | None = None) -> None:
        self.smooth = smooth
        self.beta = AIR_MOUSE_BETA[smooth] if beta is None else beta
        self.reset()

    def reset(self) -> None:
        self.initialized = False
        self.rate_live = False
        self.raw_tilt = self.raw_pan = 0.0
        self.tilt = self.pan = 0.0
        self.d_tilt = self.d_pan = 0.0
        self.tilt_rate = self.pan_rate = 0.0

    def prime(
        self,
        tilt: float = 0.0,
        pan: float = 0.0,
        tilt_rate: float = 0.0,
        pan_rate: float = 0.0,
    ) -> None:
        self.reset()
        self.initialized = True
        self.raw_tilt = self.tilt = tilt
        self.raw_pan = self.pan = pan
        self.tilt_rate = tilt_rate
        self.pan_rate = pan_rate

    @staticmethod
    def unwrap_near(value: float, reference: float) -> float:
        while value - reference > 180.0:
            value -= 360.0
        while value - reference < -180.0:
            value += 360.0
        return value

    @staticmethod
    def alpha(cutoff: float, dt: float) -> float:
        return 1.0 / (1.0 + 1.0 / (2.0 * math.pi * cutoff * dt))

    def step(
        self,
        tilt: float,
        pan: float,
        tilt_rate: float,
        pan_rate: float,
        dt: float = 0.04,
    ) -> tuple[float, float, float, float]:
        dt = max(0.015, min(0.08, dt))
        if not self.initialized:
            self.prime(tilt, pan, tilt_rate, pan_rate)
        else:
            tilt = self.unwrap_near(tilt, self.raw_tilt)
            pan = self.unwrap_near(pan, self.raw_pan)
            derivative_alpha = self.alpha(AIR_MOUSE_DERIVATIVE_CUTOFF, dt)
            raw_d_tilt = (tilt - self.raw_tilt) / dt
            raw_d_pan = (pan - self.raw_pan) / dt
            self.d_tilt += derivative_alpha * (raw_d_tilt - self.d_tilt)
            self.d_pan += derivative_alpha * (raw_d_pan - self.d_pan)
            self.raw_tilt, self.raw_pan = tilt, pan
            motion = math.hypot(self.d_tilt, self.d_pan)
            cutoff = AIR_MOUSE_MIN_CUTOFF[self.smooth] + self.beta * motion
            signal_alpha = self.alpha(cutoff, dt)
            self.tilt += signal_alpha * (tilt - self.tilt)
            self.pan += signal_alpha * (pan - self.pan)
            self.tilt_rate += signal_alpha * (tilt_rate - self.tilt_rate)
            self.pan_rate += signal_alpha * (pan_rate - self.pan_rate)

        out_tilt_rate, out_pan_rate = self.tilt_rate, self.pan_rate
        magnitude = math.hypot(out_tilt_rate, out_pan_rate)
        if not self.rate_live:
            if magnitude < AIR_MOUSE_RATE_GATE_ENTER_DPS:
                return self.tilt, self.pan, 0.0, 0.0
            self.rate_live = True
        elif magnitude <= AIR_MOUSE_RATE_GATE_EXIT_DPS:
            self.rate_live = False
            return self.tilt, self.pan, 0.0, 0.0
        if magnitude:
            scale = max(0.0, magnitude - AIR_MOUSE_RATE_GATE_EXIT_DPS) / magnitude
            out_tilt_rate *= scale
            out_pan_rate *= scale
        return self.tilt, self.pan, out_tilt_rate, out_pan_rate


def quat_mul(a: tuple[float, ...], b: tuple[float, ...]) -> tuple[float, ...]:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_conj(q: tuple[float, ...]) -> tuple[float, ...]:
    return q[0], -q[1], -q[2], -q[3]


def axis_angle(axis: tuple[float, float, float], angle: float) -> tuple[float, ...]:
    length = math.sqrt(sum(component * component for component in axis))
    half = angle / 2.0
    scale = math.sin(half) / length
    return math.cos(half), *(component * scale for component in axis)


def rotation_vector(q: tuple[float, ...]) -> tuple[float, float, float]:
    if q[0] < 0:
        q = tuple(-component for component in q)
    vector_length = math.sqrt(sum(component * component for component in q[1:]))
    if vector_length < 1e-9:
        return tuple(2.0 * component for component in q[1:])
    angle = 2.0 * math.atan2(vector_length, max(-1.0, min(1.0, q[0])))
    return tuple(component * angle / vector_length for component in q[1:])


class TestProfessionalControlPresets(unittest.TestCase):
    def test_speed_sensitivity_and_ramp_presets_are_independent(self) -> None:
        self.assertEqual(SENSITIVITIES, (0.25, 0.5, 1.0))
        self.assertEqual(SPEED_CAPS, (6.0, 18.0, 42.0))
        self.assertEqual(MAX_ACCEL, (330.0, 130.0, 55.0))
        self.assertEqual(MAX_JERK, (3600.0, 900.0, 230.0))

    def test_crisp_fluid_and_glide_have_ordered_response(self) -> None:
        reach_times = []
        for smooth in range(3):
            shaper = AxisShaper()
            reached = None
            for step in range(100):
                value = shaper.step(18.0, 0.04, smooth)
                self.assertLessEqual(value, 18.0 + 1e-9)
                if reached is None and value >= 16.2:
                    reached = (step + 1) * 0.04
            self.assertIsNotNone(reached)
            reach_times.append(reached)
        self.assertLess(reach_times[0], reach_times[1])
        self.assertLess(reach_times[1], reach_times[2])

    def test_jittered_updates_bound_the_ramp_until_exact_settle(self) -> None:
        jitter = (0.017, 0.041, 0.063, 0.025, 0.079)
        for smooth in range(3):
            shaper = AxisShaper()
            prior_accel = 0.0
            for step in range(20):
                dt = jitter[step % len(jitter)]
                shaper.step(42.0, dt, smooth)
                bounded_dt = max(0.015, min(0.08, dt))
                self.assertLessEqual(abs(shaper.accel), MAX_ACCEL[smooth] + 1e-6)
                # The implementation deliberately snaps its internal
                # acceleration to zero on the exact target sample.  Before
                # that terminal sample, every increment remains jerk-limited.
                if not math.isclose(shaper.value, 42.0, abs_tol=1e-9):
                    self.assertLessEqual(
                        abs(shaper.accel - prior_accel),
                        MAX_JERK[smooth] * bounded_dt + 1e-6,
                    )
                self.assertLessEqual(shaper.value, 42.0 + 1e-9)
                prior_accel = shaper.accel


class TestAxisTune(unittest.TestCase):
    """Axis Tune is a Hand-only post-estimation control law.  It must not
    create a second attitude estimator or let a diagonal overrun the master
    speed cap."""

    def test_response_and_stability_vocabularies_have_neutral_middle(self) -> None:
        self.assertEqual(AXIS_RESPONSE, (0.25, 0.5, 1.0))
        self.assertEqual(AXIS_STABILITY, (0.55, 1.0, 1.6))

    def test_balanced_axis_tune_preserves_legacy_shaper_trace(self) -> None:
        legacy = AxisShaper()
        tuned = AxisShaper()
        for _ in range(25):
            self.assertAlmostEqual(
                legacy.step(18.0, 0.04, 1),
                tuned.step(18.0, 0.04, 1, 1),
                places=7,
            )

    def test_quiet_and_responsive_change_only_the_selected_axis_dynamics(self) -> None:
        quiet = AxisShaper()
        responsive = AxisShaper()
        balanced = AxisShaper()
        for _ in range(2):
            quiet_value = quiet.step(42.0, 0.04, 1, 0)
            responsive_value = responsive.step(42.0, 0.04, 1, 2)
            balanced_value = balanced.step(42.0, 0.04, 1, 1)
        self.assertLess(quiet_value, balanced_value)
        self.assertLess(balanced_value, responsive_value)

    def test_response_scales_each_axis_before_the_final_vector_cap(self) -> None:
        base = SENSITIVITIES[1]
        tilt = 24.0 * base * AXIS_RESPONSE[2]
        pan = 24.0 * base * AXIS_RESPONSE[0]
        self.assertGreater(tilt, pan)
        cap = SPEED_CAPS[1]
        length = math.hypot(tilt, pan)
        scale = min(1.0, cap / length)
        self.assertLessEqual(math.hypot(tilt * scale, pan * scale), cap + 1e-9)

    def test_native_clutch_applies_axis_response_before_cap_and_recaps_all_templates(self) -> None:
        clutch = SOURCE[SOURCE.index("bool commandClutch("):
                        SOURCE.index("bool commandJog(")]
        self.assertIn("RESPONSE_GAINS", clutch)
        self.assertIn("STABILITY_FACTORS", SOURCE)
        self.assertLess(clutch.index("tiltResponse"), clutch.index("capVector(wantedPitch"))
        self.assertLess(clutch.index("panResponse"), clutch.index("capVector(wantedPitch"))
        shaper = clutch.index("pitchShaper_.step")
        recapped = clutch.index("capVector(pitch, yaw, cap);", shaper)
        self.assertGreater(recapped, shaper)
        self.assertNotIn("if (in.templateIndex == HAND_TEMPLATE_AIR_MOUSE)",
                         clutch[shaper:recapped])

    def test_air_mouse_filter_remains_vector_coupled(self) -> None:
        conditioner = SOURCE[SOURCE.index("struct AirMouseFilter"):
                             SOURCE.index("struct ResponsePoint")]
        self.assertIn("const float motion = std::sqrt(derivativeTilt * derivativeTilt +", conditioner)
        self.assertNotIn("tiltStabilityIndex", conditioner)
        self.assertNotIn("panStabilityIndex", conditioner)

    def test_fine_stationary_tremor_never_acquires_motion(self) -> None:
        gate = PrecisionIntentGate(stability=1)
        samples = []
        for i in range(250):
            jitter = 0.18 * math.sin(2 * math.pi * 5.0 * i * 0.04)
            jitter += 0.08 * math.sin(2 * math.pi * 8.0 * i * 0.04)
            samples.append(gate.step(jitter, 0.04))
        self.assertEqual(set(samples), {0.0})

    def test_fine_gesture_is_continuous_not_pulse_density(self) -> None:
        gate = PrecisionIntentGate(stability=1)
        outputs = [gate.step(1.2, 0.04) for _ in range(20)]
        first = next(i for i, value in enumerate(outputs) if value)
        active = outputs[first:]
        self.assertTrue(all(value >= gate.MINIMUM_DPS for value in active))
        self.assertNotIn(0.0, active)

    def test_fine_hysteresis_releases_cleanly_and_does_not_replay_credit(self) -> None:
        gate = PrecisionIntentGate(stability=1)
        acquired = [gate.step(1.0, 0.04) for _ in range(12)]
        self.assertTrue(any(value >= gate.MINIMUM_DPS for value in acquired))
        self.assertTrue(all(value >= gate.MINIMUM_DPS for value in acquired[2:]))
        held = [gate.step(0.25, 0.04) for _ in range(3)]
        self.assertTrue(all(value >= gate.MINIMUM_DPS for value in held))
        self.assertEqual(gate.step(0.0, 0.04), 0.0)
        self.assertEqual(gate.step(-0.25, 0.04), 0.0)

    def test_damping_presets_trade_noise_rejection_for_response(self) -> None:
        self.assertEqual(PRECISION_MIN_CUTOFF, (1.0, 1.6, 2.4))
        self.assertEqual(PRECISION_BETA, (0.05, 0.1, 0.16))
        self.assertEqual(PRECISION_GATE_ENTER_DPS, (0.42, 0.36, 0.35))
        self.assertEqual(PRECISION_GATE_EXIT_DPS, (0.2, 0.16, 0.12))
        for enter, leave in zip(
            PRECISION_GATE_ENTER_DPS, PRECISION_GATE_EXIT_DPS
        ):
            self.assertGreater(enter, leave)
        gates = [PrecisionIntentGate(stability=i) for i in range(3)]
        acquire = []
        for gate in gates:
            outputs = [gate.step(0.65, 0.04) for _ in range(20)]
            acquire.append(next(i for i, value in enumerate(outputs) if value))
        self.assertGreaterEqual(acquire[0], acquire[1])
        self.assertGreaterEqual(acquire[1], acquire[2])

    def test_native_precision_conditioner_precedes_shaping_and_is_fine_only(self) -> None:
        clutch = SOURCE[SOURCE.index("bool commandClutch("):
                        SOURCE.index("bool commandJog(")]
        conditioned = clutch.index("pitchPrecision_.step")
        shaped = clutch.index("pitchShaper_.step")
        sent = clutch.index("return sendRates", shaped)
        self.assertLess(conditioned, shaped)
        self.assertLess(shaped, sent)
        self.assertIn("in.tiltResponseIndex == 0", clutch)
        self.assertIn("in.panResponseIndex == 0", clutch)
        reset = SOURCE[SOURCE.index("void resetMotion()"):
                       SOURCE.index("void cleanup()")]
        self.assertIn("pitchPrecision_.reset()", reset)
        self.assertIn("yawPrecision_.reset()", reset)
        self.assertNotIn("LowRateInterpolator", SOURCE)


class TestAirMouseConditioning(unittest.TestCase):
    def test_presets_trade_quiet_precision_for_response_without_axis_skew(self) -> None:
        self.assertEqual(AIR_MOUSE_MIN_CUTOFF, (2.5, 1.5, 0.8))
        self.assertEqual(AIR_MOUSE_BETA, (0.03, 0.015, 0.008))
        self.assertIn("const float motion = std::sqrt(derivativeTilt * derivativeTilt +",
                      SOURCE)
        self.assertEqual(AIR_MOUSE_DERIVATIVE_CUTOFF, 1.0)

    def test_stationary_hand_tremor_is_reduced_without_a_fixed_lag_filter(self) -> None:
        conditioned = AirMouseFilter(smooth=1)
        raw_samples: list[float] = []
        filtered_samples: list[float] = []
        for i in range(250):
            t = i * 0.04
            angle = 0.22 * math.sin(2 * math.pi * 4.7 * t)
            angle += 0.08 * math.sin(2 * math.pi * 9.1 * t)
            filtered, _, _, _ = conditioned.step(angle, 0.0, 0.0, 0.0)
            if i >= 50:
                raw_samples.append(angle)
                filtered_samples.append(filtered)
        self.assertLess(
            statistics.pstdev(filtered_samples),
            statistics.pstdev(raw_samples) * 0.55,
        )

    def test_cutoff_opens_for_fast_intent_and_reduces_ramp_lag(self) -> None:
        adaptive = AirMouseFilter(smooth=1)
        fixed = AirMouseFilter(smooth=1, beta=0.0)
        adaptive.step(0.0, 0.0, 30.0, 0.0)
        fixed.step(0.0, 0.0, 30.0, 0.0)
        target = 0.0
        for i in range(1, 26):
            target = i * 30.0 * 0.04
            adaptive_value, *_ = adaptive.step(target, 0.0, 30.0, 0.0)
            fixed_value, *_ = fixed.step(target, 0.0, 30.0, 0.0)
        self.assertLess(target - adaptive_value, target - fixed_value)
        self.assertLess(target - adaptive_value, 4.0)

    def test_slow_pitch_pose_is_continuous_while_rate_tremor_is_gated(self) -> None:
        conditioned = AirMouseFilter(smooth=1)
        last = (0.0, 0.0, 0.0, 0.0)
        for i in range(26):
            angle = i * 0.04  # deliberate 1 degree/second precision move
            last = conditioned.step(angle, 0.0, 0.18, 0.12)
        self.assertGreater(last[0], 0.70)
        self.assertEqual(last[2:], (0.0, 0.0))

    def test_reacquire_resets_all_filter_and_rate_gate_state(self) -> None:
        conditioned = AirMouseFilter(smooth=0)
        for i in range(12):
            conditioned.step(i * 0.8, -i * 0.4, 20.0, -10.0)
        conditioned.reset()
        reacquired = conditioned.step(0.0, 0.0, 0.0, 0.0)
        self.assertEqual(reacquired, (0.0, 0.0, 0.0, 0.0))
        self.assertIn("airMouseFilter_.reset();", SOURCE)

    def test_press_edge_prime_conditions_the_first_acquisition_command(self) -> None:
        conditioned = AirMouseFilter(smooth=1)
        conditioned.prime()  # physical clutch edge
        tilt, pan, tilt_rate, pan_rate = conditioned.step(
            12.0, -8.0, 30.0, -20.0, 0.04
        )
        self.assertGreater(tilt, 0.0)
        self.assertLess(tilt, 12.0)
        self.assertLess(abs(pan), 8.0)
        self.assertLess(abs(tilt_rate), 30.0)
        self.assertLess(abs(pan_rate), 20.0)
        self.assertIn("airMouseFilter_.prime();", SOURCE)

    def test_rotation_vector_seam_is_unwrapped_continuously(self) -> None:
        conditioned = AirMouseFilter(smooth=0)
        conditioned.prime(178.0, -178.0)
        conditioned.step(179.0, -179.0, 2.0, -2.0)
        conditioned.step(-179.0, 179.0, 2.0, -2.0)
        self.assertGreater(conditioned.raw_tilt, 180.0)
        self.assertLess(conditioned.raw_pan, -180.0)
        self.assertLess(abs(conditioned.raw_tilt - 181.0), 1e-6)
        self.assertLess(abs(conditioned.raw_pan + 181.0), 1e-6)

    def test_filter_and_shaper_chain_responds_without_breaking_acceleration(self) -> None:
        conditioned = AirMouseFilter(smooth=1)
        conditioned.prime()
        shaper = AxisShaper()
        outputs = []
        prior = 0.0
        for i in range(1, 26):
            angle = i * 20.0 * 0.04
            filtered, _, rate, _ = conditioned.step(angle, 0.0, 20.0, 0.0)
            wanted = filtered * 2.0 + rate
            value = shaper.step(min(18.0, wanted), 0.04, 1)
            outputs.append(value)
            self.assertLessEqual(abs(value - prior) / 0.04,
                                 MAX_ACCEL[1] + 1e-6)
            prior = value
        self.assertGreater(outputs[4], 1.0)  # useful response within 200 ms
        self.assertLessEqual(max(outputs), 18.0)

    def test_air_mouse_is_opt_in_and_legacy_equations_remain_available(self) -> None:
        self.assertIn("HAND_TEMPLATE_FOLLOW = 0", SOURCE)
        self.assertIn("HAND_TEMPLATE_RATE = 1", SOURCE)
        self.assertIn("HAND_TEMPLATE_AIR_MOUSE = 2", SOURCE)
        self.assertIn(
            "wantedPitch = (targetPitch - cameraPitch_) * 2.0f +",
            SOURCE,
        )
        self.assertIn("tiltRate * tiltResponse;", SOURCE)
        self.assertIn("wantedYaw = panRate * panResponse;", SOURCE)
        self.assertIn("airMouseFilter_.step(", SOURCE)
        self.assertNotIn("targetYaw", SOURCE[SOURCE.index("bool commandClutch("):
                                              SOURCE.index("bool commandJog(")])

    def test_delivered_hand_vector_is_recapped_after_axis_shaping(self) -> None:
        clutch = SOURCE[SOURCE.index("bool commandClutch("):
                        SOURCE.index("bool commandJog(")]
        shaper = clutch.index("pitchShaper_.step")
        recapped = clutch.index("capVector(pitch, yaw, cap);", shaper)
        self.assertGreater(recapped, shaper)
        self.assertNotIn("in.templateIndex == HAND_TEMPLATE_AIR_MOUSE",
                         clutch[shaper:recapped])


class TestOneThumbJogLaw(unittest.TestCase):
    @staticmethod
    def shape(x: float, y: float) -> tuple[float, float]:
        magnitude = math.hypot(x, y)
        deadzone = 0.12
        if magnitude <= deadzone:
            return 0.0, 0.0
        shaped = ((magnitude - deadzone) / (1.0 - deadzone)) ** 1.6
        return x * shaped / magnitude, y * shaped / magnitude

    def test_deadzone_is_quiet_and_diagonal_speed_is_radially_capped(self) -> None:
        self.assertEqual(self.shape(0.08, 0.08), (0.0, 0.0))
        x, y = self.shape(1 / math.sqrt(2), 1 / math.sqrt(2))
        self.assertAlmostEqual(math.hypot(x, y), 1.0, places=6)
        for cap in SPEED_CAPS:
            self.assertLessEqual(math.hypot(x * cap, y * cap), cap + 1e-6)

    def test_expo_preserves_the_finger_direction(self) -> None:
        x, y = self.shape(0.3, -0.6)
        self.assertAlmostEqual(x / y, -0.5, places=6)


class TestArbitraryPoseAcquisition(unittest.TestCase):
    def test_every_initial_controller_pose_is_a_zero_command(self) -> None:
        poses = (
            axis_angle((1, 0, 0), math.radians(89)),
            axis_angle((0, 1, 1), math.radians(137)),
            axis_angle((1, -2, 0.5), math.radians(173)),
        )
        for pose in poses:
            relative = quat_mul(pose, quat_conj(pose))
            for component in rotation_vector(relative):
                self.assertAlmostEqual(component, 0.0, places=6)

    def test_only_motion_after_grab_survives_the_relative_transform(self) -> None:
        delta = axis_angle((0, 0, 1), math.radians(12))
        for pose in (
            axis_angle((1, 0, 0), 0.9),
            axis_angle((0.2, 1, -0.4), 2.1),
        ):
            current = quat_mul(delta, pose)
            measured = rotation_vector(quat_mul(current, quat_conj(pose)))
            self.assertAlmostEqual(measured[0], 0.0, places=6)
            self.assertAlmostEqual(measured[1], 0.0, places=6)
            self.assertAlmostEqual(measured[2], math.radians(12), places=6)


class TestGravityCorrection(unittest.TestCase):
    def test_small_attitude_error_is_damped_not_amplified(self) -> None:
        self.assertIn("const Vec3 error = cross(measured, estimated);", SOURCE)
        self.assertNotIn("const Vec3 error = cross(estimated, measured);", SOURCE)

        # Level is measured as +Z. If the estimate has a positive roll error,
        # measured x estimated must produce a negative X correction.
        roll_error = math.radians(8)
        measured = (0.0, 0.0, 1.0)
        estimated = (0.0, math.sin(roll_error), math.cos(roll_error))
        correction_x = (
            measured[1] * estimated[2] - measured[2] * estimated[1]
        )
        self.assertLess(correction_x, 0.0)
        corrected_error = roll_error + 1.7 * correction_x * 0.01
        self.assertLess(abs(corrected_error), abs(roll_error))


class TestEndpointBraking(unittest.TestCase):
    def test_braking_includes_camera_coast_and_never_spends_headroom_twice(self) -> None:
        self.assertAlmostEqual(COAST, 0.47)
        for accel in MAX_ACCEL:
            for headroom in (1.5, 2.0, 5.0, 20.0, 80.0):
                safe = max(0.0, headroom - 1.5)
                speed = max(
                    0.0,
                    -accel * COAST
                    + math.sqrt(accel * accel * COAST * COAST + 2 * accel * safe),
                )
                stopping_distance = speed * speed / (2 * accel) + speed * COAST
                self.assertLessEqual(stopping_distance, safe + 1e-6)

    def test_endpoint_rebase_is_present_for_immediate_reversal(self) -> None:
        self.assertIn(
            "cameraPitchRef_ += wrap180(targetPitch - rawTarget);", SOURCE
        )
        self.assertLess(
            SOURCE.index("cameraPitchRef_ += wrap180(targetPitch - rawTarget);"),
            SOURCE.index("wantedPitch = (targetPitch - cameraPitch_)"),
        )


if __name__ == "__main__":
    unittest.main()
