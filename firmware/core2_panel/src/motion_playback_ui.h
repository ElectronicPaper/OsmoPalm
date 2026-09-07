#pragma once
#include <cstdint>
namespace osmo {
enum class PlaybackAction : uint8_t { None, GoToStart, Play, Continue, Stop };
struct PlaybackContext {
    bool direct, fresh, batteryBlocked, active, cue, atStart;
    uint8_t points;
};
// Priority matters: STOP stays available with lost telemetry or low battery.
// Moving to P1 never authorizes subsequent playback.
inline PlaybackAction playbackAction(const PlaybackContext &c) {
    if (c.active) {
        if (c.cue && c.direct && c.fresh) return PlaybackAction::Continue;
        return PlaybackAction::Stop;
    }
    if (!c.direct || !c.fresh || c.batteryBlocked || c.points < 2)
        return PlaybackAction::None;
    return c.atStart ? PlaybackAction::Play : PlaybackAction::GoToStart;
}
} // namespace osmo
