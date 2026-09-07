#pragma once

#include <cstdint>

namespace osmo {

// The gallery holds four rows of six points. The UI may use a larger layout
// for short programmes, but persistence and playback always share this bound.
constexpr uint8_t MOTION_POINT_CAPACITY = 24;
// Two independent programmes. Switching loads the other record; the one being
// left is already on flash, so a wrong tap costs nothing.
constexpr uint8_t MOTION_SLOT_COUNT = 2;
constexpr uint32_t MOTION_DEFAULT_MOVE_MS = 3000;
constexpr uint32_t MOTION_DEFAULT_DWELL_MS = 0;

// How a run continues after the last captured point. LOOP returns to P1 using
// P1's own move time, which is otherwise unused; BOUNCE reverses and a leg
// keeps the time its forward twin was given.
enum class MotionRepeat : uint8_t { Once = 0, Loop, Bounce, Count };

// The curve belongs to the incoming leg: P3's transition describes P2 -> P3.
// P1's curve is used only for a loop return to P1.
enum class MotionTransition : uint8_t {
    Smooth = 0,
    Linear,
    EaseIn,
    EaseOut,
    Count,
};

struct MotionPoint {
    bool captured = false;
    // Wait here, after any dwell, until a human releases the cue. A fixed
    // dwell suits a product pass; drama runs on the actor.
    bool holdForGo = false;
    MotionTransition transition = MotionTransition::Smooth;
    float pitch = 0.0f;
    float yaw = 0.0f;
    uint32_t moveMs = MOTION_DEFAULT_MOVE_MS;
    uint32_t dwellMs = MOTION_DEFAULT_DWELL_MS;
};

// Error text is local diagnostic state only. It is deliberately bounded and
// never carries pairing, Wi-Fi, BLE, or camera-transport material.
struct MotionProgram {
    uint16_t version = 3;
    uint8_t count = 0;
    uint8_t slot = 0;
    MotionRepeat repeat = MotionRepeat::Once;
    MotionPoint points[MOTION_POINT_CAPACITY] = {};
    char error[48] = "";
};

class MotionProgramStore {
public:
    void begin();
    MotionProgram snapshot() const;

    // index is zero-based. Capturing can replace an existing point or append
    // exactly the next point; callers cannot manufacture P1/P3 gaps.
    bool capture(uint8_t index, float pitch, float yaw);
    // Clearing Pi also clears every later point, retaining P1..P(i-1).
    bool clear(uint8_t index);
    bool remove(uint8_t index);
    bool setTiming(uint8_t index, uint32_t moveMs, uint32_t dwellMs);
    bool setHold(uint8_t index, bool holdForGo);
    bool setTransition(uint8_t index, MotionTransition transition);
    bool setRepeat(MotionRepeat repeat);
    bool selectSlot(uint8_t slot);
    bool reset();

private:
    bool load(uint8_t slot);
    bool save();
    bool saveSlotChoice();
    void clearError();
    void setError(const char *message);
    MotionProgram program_ = {};
};

}  // namespace osmo
