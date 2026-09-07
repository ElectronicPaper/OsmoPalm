#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>
#include "motion_program.h"

// Shared production math: no LVGL, RTOS, radio, I2C or persistence calls.
// Keep the accepted HAND equations unchanged when relocating them.
namespace osmo { namespace control {

constexpr int STICK_CENTER = 1024;
constexpr int STICK_TRAVEL = 550;

constexpr float RESPONSE_GAINS[] = {0.25f, 0.5f, 1.0f};
// Per-axis trim on the final jerk-limited output governor. The adaptive
// Air Mouse filter intentionally remains vector-coupled so diagonal gestures
// cannot acquire different phase lag on each axis.
constexpr float STABILITY_FACTORS[] = {0.55f, 1.0f, 1.6f};
constexpr float SPEED_CAPS[] = {6.0f, 18.0f, 42.0f};
constexpr uint8_t HAND_TEMPLATE_FOLLOW = 0;
constexpr uint8_t HAND_TEMPLATE_RATE = 1;
constexpr uint8_t HAND_TEMPLATE_AIR_MOUSE = 2;
// Casiez et al.'s 1 Euro filter raises its cutoff with measured motion: quiet
// hands receive more tremor rejection, while a deliberate turn spends less
// time behind a fixed low-pass delay.  These presets run at the camera command
// boundary (nominally 25 Hz), not inside quaternion integration.  A single
// derivative magnitude drives both axes so a diagonal gesture cannot acquire
// different phase lag on tilt and pan.
constexpr float AIR_MOUSE_MIN_CUTOFF[] = {2.5f, 1.5f, 0.8f};
constexpr float AIR_MOUSE_BETA[] = {0.030f, 0.015f, 0.008f};
constexpr float AIR_MOUSE_DERIVATIVE_CUTOFF = 1.0f;
constexpr float AIR_MOUSE_RATE_GATE_ENTER_DPS = 0.35f;
constexpr float AIR_MOUSE_RATE_GATE_EXIT_DPS = 0.20f;
constexpr float AIR_MOUSE_P_GAIN = 2.0f;
constexpr float AIR_MOUSE_LEAD_MIN = 0.55f;
constexpr float AIR_MOUSE_LEAD_MAX = 1.0f;
// Fine response needs a different answer at the Pocket's measured 0.35 dps
// breakaway point than the old pulse-density approximation. Alternating
// minimum-speed and centre frames made a mathematically correct average but
// a physically visible series of starts and stops. Condition the operator's
// requested rate instead: the 1 Euro cutoff opens as intent accelerates, a
// Schmitt gate keeps stationary IMU noise out, and an acquired gesture stays
// continuously at or above the slowest rate the head can actually sustain.
//
// Axis STABILITY is deliberately reused as damping strength. QUIET has the
// lowest cutoff and widest intent gate; RESPONSIVE opens earlier and follows
// faster. These are controller-side settings only -- no unsupported claim is
// made about changing the Pocket's internal SmoothTrack or motor tuning.
constexpr float PRECISION_MIN_CUTOFF[] = {1.00f, 1.60f, 2.40f};
constexpr float PRECISION_BETA[] = {0.050f, 0.100f, 0.160f};
constexpr float PRECISION_GATE_ENTER_DPS[] = {0.42f, 0.36f, 0.35f};
constexpr float PRECISION_GATE_EXIT_DPS[] = {0.20f, 0.16f, 0.12f};
constexpr float PRECISION_DERIVATIVE_CUTOFF = 1.0f;
// Core2's body frame is opposite the operator's natural HAND framing frame.
// Keep this explicit and local to HAND: JOG already uses the correct screen
// basis and must not change when the operator fixes a HAND direction.
constexpr float HAND_TILT_BASE_SIGN = -1.0f;
constexpr float HAND_PAN_BASE_SIGN = -1.0f;

struct Vec3 {
    float x = 0, y = 0, z = 0;
};

inline float dot(const Vec3 &a, const Vec3 &b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
inline Vec3 cross(const Vec3 &a, const Vec3 &b) {
    return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x};
}
inline float norm(const Vec3 &v) { return std::sqrt(dot(v, v)); }
inline Vec3 normalized(Vec3 v, const Vec3 &fallback = {0, 0, 1}) {
    const float n = norm(v);
    if (n < 0.0001f || !std::isfinite(n)) return fallback;
    v.x /= n; v.y /= n; v.z /= n;
    return v;
}

struct Quat { float w = 1, x = 0, y = 0, z = 0; };
inline Quat conjugate(const Quat &q) { return {q.w, -q.x, -q.y, -q.z}; }
inline Quat multiply(const Quat &a, const Quat &b) {
    return {a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z,
            a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,
            a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,
            a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w};
}
inline Quat unit(Quat q) {
    const float n = std::sqrt(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z);
    if (n < 0.0001f || !std::isfinite(n)) return {};
    q.w/=n; q.x/=n; q.y/=n; q.z/=n; return q;
}
inline Vec3 rotateVector(const Quat &q, const Vec3 &v) {
    const Quat rotated = multiply(multiply(q, {0, v.x, v.y, v.z}),
                                  conjugate(q));
    return {rotated.x, rotated.y, rotated.z};
}
inline Vec3 gravityBody(const Quat &q) {
    // World-down projected into the sensor frame.
    return {2.0f*(q.x*q.z-q.w*q.y), 2.0f*(q.w*q.x+q.y*q.z),
            q.w*q.w-q.x*q.x-q.y*q.y+q.z*q.z};
}
inline Vec3 rotationVector(Quat q) {
    q = unit(q);
    if (q.w < 0.0f) { q.w=-q.w; q.x=-q.x; q.y=-q.y; q.z=-q.z; }
    const float s = std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z);
    if (s < 0.0001f) return {2*q.x, 2*q.y, 2*q.z};
    const float a = 2.0f*std::atan2(s, std::max(-1.0f, std::min(1.0f, q.w)));
    return {q.x*a/s, q.y*a/s, q.z*a/s};
}

// Bounded ease-out: expired intervals return exact neutral, including timer wrap.
inline float releaseEaseRate(float initial, uint32_t elapsedMs, uint16_t durationMs) {
    durationMs = std::min<uint16_t>(durationMs, 750);
    if (!durationMs || elapsedMs >= durationMs) return 0.0f;
    const float t = static_cast<float>(elapsedMs) / durationMs;
    return initial * (1.0f - t*t*(3.0f - 2.0f*t));
}

struct AxisShaper {
    float value = 0.0f;
    float accel = 0.0f;
    void reset(float v = 0.0f) { value = v; accel = 0.0f; }
    float step(float target, float dt, uint8_t smooth,
               uint8_t stability = 1, uint16_t startEaseMs = 0,
               uint16_t endEaseMs = 0, float speedCap = 60.0f) {
        static constexpr float maxAccel[] = {330.0f, 130.0f, 55.0f};
        static constexpr float maxJerk[] = {3600.0f, 900.0f, 230.0f};
        const uint8_t i = std::min<uint8_t>(smooth, 2);
        const float stabilityFactor =
            STABILITY_FACTORS[std::min<uint8_t>(stability, 2)];
        dt = std::max(0.015f, std::min(0.08f, dt));
        const uint16_t easeMs = std::fabs(target) < std::fabs(value)
            ? endEaseMs : startEaseMs;
        const float seconds = std::max(0.04f, std::min(0.75f, easeMs / 1000.0f));
        const float accelLimit = (easeMs ? 1.5f * speedCap / seconds : maxAccel[i]) * stabilityFactor;
        const float jerkLimit = (easeMs ? 6.0f * speedCap / (seconds * seconds) : maxJerk[i]) * stabilityFactor;
        const float wantedAccel = std::max(-accelLimit,
            std::min(accelLimit, (target-value)/dt));
        const float aStep = jerkLimit*dt;
        accel += std::max(-aStep, std::min(aStep, wantedAccel-accel));
        accel = std::max(-accelLimit, std::min(accelLimit, accel));
        float next = value + accel*dt;
        if ((target-value)*(target-next) <= 0.0f) { next=target; accel=0.0f; }
        value = next;
        return value;
    }
};

struct AirMouseSample {
    float tilt = 0.0f;
    float pan = 0.0f;
    float tiltRate = 0.0f;
    float panRate = 0.0f;
    float motionDps = 0.0f;
};

// Vector-coupled 1 Euro conditioner for the opt-in AIR MOUSE template.  Angle
// and gyro-rate channels share the same adaptive cutoff.  The angle path stays
// continuous all the way to zero for fine pitch framing; only the rate path
// gets a tiny radial Schmitt gate, which prevents a stationary wrist from
// continuously walking pan without making slow pitch adjustments sticky.
struct AirMouseFilter {
    bool initialized = false;
    bool rateLive = false;
    float rawTilt = 0.0f;
    float rawPan = 0.0f;
    float filteredTilt = 0.0f;
    float filteredPan = 0.0f;
    float derivativeTilt = 0.0f;
    float derivativePan = 0.0f;
    float filteredTiltRate = 0.0f;
    float filteredPanRate = 0.0f;

    static float alpha(float cutoff, float dt) {
        return 1.0f / (1.0f + 1.0f / (6.283185307f * cutoff * dt));
    }

    static float unwrapNear(float value, float reference) {
        while (value - reference > 180.0f) value -= 360.0f;
        while (value - reference < -180.0f) value += 360.0f;
        return value;
    }

    void reset() { *this = AirMouseFilter{}; }

    void prime(float tilt = 0.0f, float pan = 0.0f,
               float tiltRate = 0.0f, float panRate = 0.0f) {
        reset();
        initialized = true;
        rawTilt = filteredTilt = tilt;
        rawPan = filteredPan = pan;
        filteredTiltRate = tiltRate;
        filteredPanRate = panRate;
    }

    void rebaseAngles(float tilt = 0.0f, float pan = 0.0f) {
        if (!initialized) {
            prime(tilt, pan);
            return;
        }
        rawTilt = filteredTilt = tilt;
        rawPan = filteredPan = pan;
        derivativeTilt = derivativePan = 0.0f;
    }

    AirMouseSample step(float tilt, float pan, float tiltRate, float panRate,
                        float dt, uint8_t smooth) {
        if (!std::isfinite(tilt) || !std::isfinite(pan) ||
            !std::isfinite(tiltRate) || !std::isfinite(panRate) ||
            !std::isfinite(dt)) {
            reset();
            return {};
        }
        dt = std::max(0.015f, std::min(0.08f, dt));
        if (!initialized) {
            prime(tilt, pan, tiltRate, panRate);
        } else {
            tilt = unwrapNear(tilt, rawTilt);
            pan = unwrapNear(pan, rawPan);
            const float derivativeAlpha = alpha(AIR_MOUSE_DERIVATIVE_CUTOFF, dt);
            const float rawTiltDerivative = (tilt - rawTilt) / dt;
            const float rawPanDerivative = (pan - rawPan) / dt;
            derivativeTilt += derivativeAlpha *
                (rawTiltDerivative - derivativeTilt);
            derivativePan += derivativeAlpha *
                (rawPanDerivative - derivativePan);
            rawTilt = tilt;
            rawPan = pan;

            const uint8_t profile = std::min<uint8_t>(smooth, 2);
            const float motion = std::sqrt(derivativeTilt * derivativeTilt +
                                           derivativePan * derivativePan);
            const float cutoff = AIR_MOUSE_MIN_CUTOFF[profile] +
                                 AIR_MOUSE_BETA[profile] * motion;
            const float signalAlpha = alpha(cutoff, dt);
            filteredTilt += signalAlpha * (tilt - filteredTilt);
            filteredPan += signalAlpha * (pan - filteredPan);
            filteredTiltRate += signalAlpha * (tiltRate - filteredTiltRate);
            filteredPanRate += signalAlpha * (panRate - filteredPanRate);
        }

        AirMouseSample out{filteredTilt, filteredPan,
                           filteredTiltRate, filteredPanRate,
                           std::sqrt(derivativeTilt * derivativeTilt +
                                     derivativePan * derivativePan)};
        const float rateMagnitude = std::sqrt(out.tiltRate * out.tiltRate +
                                              out.panRate * out.panRate);
        if (!rateLive) {
            if (rateMagnitude < AIR_MOUSE_RATE_GATE_ENTER_DPS) {
                out.tiltRate = out.panRate = 0.0f;
                return out;
            }
            rateLive = true;
        } else if (rateMagnitude <= AIR_MOUSE_RATE_GATE_EXIT_DPS) {
            rateLive = false;
            out.tiltRate = out.panRate = 0.0f;
            return out;
        }
        if (rateMagnitude > 0.0001f) {
            const float scale = std::max(0.0f,
                rateMagnitude - AIR_MOUSE_RATE_GATE_EXIT_DPS) / rateMagnitude;
            out.tiltRate *= scale;
            out.panRate *= scale;
        }
        return out;
    }
};

struct ResponsePoint { float deflection; float dps; };
constexpr ResponsePoint RESPONSE[] = {
    {0.00f, 0.00f}, {0.10f, 0.35f}, {0.15f, 0.85f},
    {0.20f, 1.75f}, {0.30f, 4.40f}, {0.40f, 8.60f},
    {0.55f, 14.60f}, {0.70f, 18.20f}, {0.85f, 21.00f},
    {1.00f, 42.00f},
};

inline float clampf(float value, float low, float high) {
    return value < low ? low : value > high ? high : value;
}

// Controller-side precision law for an axis whose selected response is FINE.
// The filter acts on continuous intent before the jerk governor; it never
// filters or dithers already-quantised stick frames. Once a gesture crosses
// the enter threshold, hysteresis holds that intent until it genuinely comes
// to rest. The minimum-rate floor is therefore continuous, not pulsed.
struct PrecisionIntentGate {
    bool initialized = false;
    bool live = false;
    int8_t direction = 0;
    float raw = 0.0f;
    float derivative = 0.0f;
    float filtered = 0.0f;

    static float alpha(float cutoff, float dt) {
        return 1.0f / (1.0f + 1.0f / (6.283185307f * cutoff * dt));
    }

    void reset() { *this = PrecisionIntentGate{}; }

    float step(float rateDps, float dt, bool enabled, uint8_t stability) {
        if (!enabled) {
            reset();
            return rateDps;
        }
        if (!std::isfinite(rateDps) || std::fabs(rateDps) < 0.0001f) {
            reset();
            return 0.0f;
        }

        const uint8_t profile = std::min<uint8_t>(stability, 2);
        dt = std::max(0.015f, std::min(0.08f, dt));
        if (!initialized) {
            initialized = true;
            // A clutch acquire is a neutral reference. Begin at zero so one
            // noisy first sample cannot bypass the adaptive conditioner.
            raw = derivative = filtered = 0.0f;
        }
        const float derivativeAlpha = alpha(PRECISION_DERIVATIVE_CUTOFF, dt);
        const float rawDerivative = (rateDps - raw) / dt;
        derivative += derivativeAlpha * (rawDerivative - derivative);
        raw = rateDps;
        const float cutoff = PRECISION_MIN_CUTOFF[profile] +
                             PRECISION_BETA[profile] * std::fabs(derivative);
        filtered += alpha(cutoff, dt) * (rateDps - filtered);

        const float magnitude = std::fabs(filtered);
        const int8_t nextDirection = filtered < 0.0f ? -1 : 1;
        if (live && nextDirection != direction) {
            // Reversal first releases the old gesture. The opposite direction
            // must independently cross its enter threshold; there is no
            // stored displacement or delayed catch-up movement to release.
            live = false;
            direction = 0;
        }
        if (!live) {
            if (magnitude < PRECISION_GATE_ENTER_DPS[profile]) return 0.0f;
            live = true;
            direction = nextDirection;
        } else if (magnitude <= PRECISION_GATE_EXIT_DPS[profile]) {
            live = false;
            direction = 0;
            return 0.0f;
        }

        const float sustainable = std::max(magnitude, RESPONSE[1].dps);
        return direction * sustainable;
    }
};

// A transition is owned by the point it arrives at. Keep the position and
// derivative together: the feed-forward rate must always match the reference
// curve, while AxisShaper remains the final acceleration/jerk safety bound.
struct ProgramCurveSample {
    float position = 0.0f;
    float rate = 0.0f;
};

inline ProgramCurveSample sampleProgramCurve(MotionTransition transition, float s,
                                      float duration) {
    s = clampf(s, 0.0f, 1.0f);
    const float inverseDuration = duration > 0.001f ? 1.0f / duration : 0.0f;
    switch (transition) {
        case MotionTransition::Linear:
            return {s, inverseDuration};
        case MotionTransition::EaseIn:
            return {s * s, 2.0f * s * inverseDuration};
        case MotionTransition::EaseOut:
            return {s * (2.0f - s), 2.0f * (1.0f - s) * inverseDuration};
        case MotionTransition::Smooth:
        case MotionTransition::Count:
        default:
            return {s * s * (3.0f - 2.0f * s),
                    6.0f * s * (1.0f - s) * inverseDuration};
    }
}

inline float wrap180(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

inline float positive360(float angle) {
    while (angle < 0.0f) angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    return angle;
}

inline float deflectionForRate(float dps) {
    const float sign = dps < 0.0f ? -1.0f : 1.0f;
    const float wanted = std::fabs(dps);
    if (wanted < RESPONSE[1].dps) return 0.0f;
    if (wanted >= RESPONSE[std::size(RESPONSE) - 1].dps) return sign;
    for (size_t i = 1; i < std::size(RESPONSE); ++i) {
        if (wanted <= RESPONSE[i].dps) {
            const auto &a = RESPONSE[i - 1];
            const auto &b = RESPONSE[i];
            const float t = (wanted - a.dps) / (b.dps - a.dps);
            return sign * (a.deflection + (b.deflection - a.deflection) * t);
        }
    }
    return sign;
}

inline float rateForInput(float value, float cap) {
    value = clampf(value, -1.0f, 1.0f);
    const float sign = value < 0.0f ? -1.0f : 1.0f;
    return sign * cap * std::pow(std::fabs(value), 1.6f);
}

inline uint8_t limitWarning(float remaining, float closingDps) {
    // Same operator model as driver/limits.py: a quiet static notch near a
    // stop, a full warning at the stop, and an earlier warning only when the
    // current command would arrive in under one second.
    if (remaining <= 1.5f) return 100;
    const float staticPart = 0.35f *
        clampf((8.0f - remaining) / 8.0f, 0.0f, 1.0f);
    float timed = 0.0f;
    if (closingDps > 1.5f) {
        timed = clampf(1.0f - remaining / closingDps, 0.0f, 1.0f);
    }
    return static_cast<uint8_t>(std::lround(
        100.0f * std::max(staticPart, timed)));
}

inline uint16_t stickAxis(float deflection) {
    deflection = clampf(deflection, -1.0f, 1.0f);
    if (std::fabs(deflection) < 0.08f) return STICK_CENTER;
    const int value = static_cast<int>(std::lround(
        STICK_CENTER + STICK_TRAVEL * deflection));
    return static_cast<uint16_t>(std::max(STICK_CENTER - STICK_TRAVEL,
        std::min(STICK_CENTER + STICK_TRAVEL, value)));
}


} } // namespace osmo::control
