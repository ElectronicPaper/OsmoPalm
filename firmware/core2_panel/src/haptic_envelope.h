#pragma once

#include <algorithm>
#include <cstdint>

namespace osmo {

// AXP192 LDO3 is OFF below 1800 mV. M5Unified's byte level is NOT PWM:
// level=110 is its first usable voltage. Softness below this floor must come
// from bounded pulse energy, not an imaginary low-voltage ramp.
class HapticEnvelope {
public:
    struct Profile { uint16_t peakMv, maxOnMs, gapMs; };
    static Profile profile(uint8_t level) {
        switch (level) {
            case 1: return {1800, 20, 140};
            case 2: return {2000, 36, 120};
            case 3: return {2300, 55, 100};
            default: return {0, 0, 0};
        }
    }

    void cancel() { running_ = false; }
    bool schedule(uint32_t now, uint8_t strength, uint16_t requestedMs,
                  uint8_t level, bool mounted) {
        if (mounted || !level) { cancel(); return false; }
        const Profile p = profile(level);
        if (!p.peakMv || !strength || !requestedMs) return false;
        // Repeated menu/limit events cannot extend the current pulse into a buzz.
        if (seen_ && now - started_ < gapMs_) return false;
        const uint32_t energy = std::min<uint16_t>(requestedMs, 120) *
            p.maxOnMs * std::min<uint8_t>(strength, 180);
        onMs_ = static_cast<uint16_t>(std::max<uint32_t>(6,
            std::min<uint32_t>(p.maxOnMs, (energy + 12599) / 12600)));
        peakMv_ = p.peakMv;
        gapMs_ = p.gapMs;
        started_ = now;
        running_ = seen_ = true;
        return true;
    }

    uint8_t sample(uint32_t now) {
        if (!running_) return 0;
        const uint32_t age = now - started_;
        if (age >= onMs_) { running_ = false; return 0; }
        // Soft stays at the physical minimum. Stronger profiles move through
        // valid 100 mV steps only. A delayed UI tick skips expired output.
        const uint32_t edge = std::min<uint32_t>(100,
            std::min(age * 100 / 6, (onMs_ - age) * 100 / 10));
        const uint16_t mv = 1800 + ((peakMv_ - 1800) * edge / 100 / 100) * 100;
        return static_cast<uint8_t>((mv - 480 + 11) / 12);
    }

private:
    bool running_ = false, seen_ = false;
    uint32_t started_ = 0;
    uint16_t onMs_ = 0, peakMv_ = 1800, gapMs_ = 140;
};

} // namespace osmo
