#include "motion_program.h"
#include "motion_edit.h"

#include <Preferences.h>

#include <cmath>
#include <cstring>

namespace osmo {
namespace {

constexpr char NVS_NAMESPACE[] = "osmo-motion";
// One record per slot. Schema 1 is left untouched. Schema 2 is read once and
// migrated into this schema without deleting the recoverable source record.
constexpr char NVS_KEY_PREFIX[] = "program-v3-";
constexpr char LEGACY_NVS_KEY_PREFIX[] = "program-v2-";
constexpr char NVS_SLOT_KEY[] = "slot";
constexpr uint32_t MOTION_MAGIC = 0x4F4D5031;  // OMP1
constexpr uint16_t MOTION_SCHEMA = 3;
constexpr uint16_t LEGACY_MOTION_SCHEMA = 2;
constexpr uint32_t MIN_MOVE_MS = 500;
constexpr uint32_t MAX_MOVE_MS = 30000;
constexpr uint32_t MAX_DWELL_MS = 10000;

// Keep the natural ESP32 layout: packing would place floats after a nine-byte
// header and can create unaligned accesses. The record is always zeroed before
// population, and schema plus checksum keep its on-flash representation safe.
struct StoredMotionProgram {
    uint32_t magic;
    uint16_t schema;
    uint16_t version;
    uint8_t count;
    uint8_t repeat;
    MotionPoint points[MOTION_POINT_CAPACITY];
    uint32_t checksum;
};

// This is deliberately a separate legacy layout rather than a reinterpret
// cast of MotionPoint. Adding a curve must never shift old on-flash floats.
struct LegacyMotionPoint {
    bool captured;
    bool holdForGo;
    float pitch;
    float yaw;
    uint32_t moveMs;
    uint32_t dwellMs;
};

struct LegacyStoredMotionProgram {
    uint32_t magic;
    uint16_t schema;
    uint16_t version;
    uint8_t count;
    uint8_t repeat;
    LegacyMotionPoint points[6];
    uint32_t checksum;
};

template <typename Record>
uint32_t checksumRecord(const Record &stored) {
    // FNV-1a over every persisted field except checksum itself. This catches
    // interrupted/garbled NVS records before any point reaches the UI.
    uint32_t value = 2166136261UL;
    const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&stored);
    constexpr size_t payloadSize = offsetof(Record, checksum);
    for (size_t i = 0; i < payloadSize; ++i) {
        value ^= bytes[i];
        value *= 16777619UL;
    }
    return value;
}

uint32_t checksum(const StoredMotionProgram &stored) {
    return checksumRecord(stored);
}

uint32_t legacyChecksum(const LegacyStoredMotionProgram &stored) {
    return checksumRecord(stored);
}

bool validTiming(uint32_t moveMs, uint32_t dwellMs) {
    return moveMs >= MIN_MOVE_MS && moveMs <= MAX_MOVE_MS &&
           dwellMs <= MAX_DWELL_MS;
}

bool validPoint(const MotionPoint &point, bool shouldBeCaptured) {
    if (point.captured != shouldBeCaptured) return false;
    if (point.transition >= MotionTransition::Count) return false;
    if (!validTiming(point.moveMs, point.dwellMs)) return false;
    return !shouldBeCaptured ||
           (std::isfinite(point.pitch) && std::isfinite(point.yaw) &&
            point.pitch >= -180.0f && point.pitch <= 180.0f &&
            point.yaw >= -180.0f && point.yaw <= 180.0f);
}

bool validLegacyPoint(const LegacyMotionPoint &point, bool shouldBeCaptured) {
    if (point.captured != shouldBeCaptured) return false;
    if (!validTiming(point.moveMs, point.dwellMs)) return false;
    return !shouldBeCaptured ||
           (std::isfinite(point.pitch) && std::isfinite(point.yaw) &&
            point.pitch >= -180.0f && point.pitch <= 180.0f &&
            point.yaw >= -180.0f && point.yaw <= 180.0f);
}

bool validStored(const StoredMotionProgram &stored) {
    if (stored.magic != MOTION_MAGIC || stored.schema != MOTION_SCHEMA ||
        stored.version != 3 || stored.count > MOTION_POINT_CAPACITY ||
        stored.repeat >= static_cast<uint8_t>(MotionRepeat::Count) ||
        stored.checksum != checksum(stored)) return false;
    for (uint8_t i = 0; i < MOTION_POINT_CAPACITY; ++i) {
        if (!validPoint(stored.points[i], i < stored.count)) return false;
    }
    return true;
}

bool validLegacyStored(const LegacyStoredMotionProgram &stored) {
    if (stored.magic != MOTION_MAGIC || stored.schema != LEGACY_MOTION_SCHEMA ||
        stored.version != 2 || stored.count > 6 ||
        stored.repeat >= static_cast<uint8_t>(MotionRepeat::Count) ||
        stored.checksum != legacyChecksum(stored)) return false;
    for (uint8_t i = 0; i < 6; ++i) {
        if (!validLegacyPoint(stored.points[i], i < stored.count)) return false;
    }
    return true;
}

StoredMotionProgram toStored(const MotionProgram &program) {
    StoredMotionProgram stored = {};
    stored.magic = MOTION_MAGIC;
    stored.schema = MOTION_SCHEMA;
    stored.version = program.version;
    stored.count = program.count;
    stored.repeat = static_cast<uint8_t>(program.repeat);
    std::memcpy(stored.points, program.points, sizeof(stored.points));
    stored.checksum = checksum(stored);
    return stored;
}

MotionProgram emptyProgram(uint8_t slot) {
    MotionProgram empty;
    empty.slot = slot;
    for (auto &point : empty.points) {
        point = MotionPoint{};
    }
    return empty;
}

void slotKey(char *out, size_t n, const char *prefix, uint8_t slot) {
    snprintf(out, n, "%s%u", prefix, static_cast<unsigned>(slot));
}

}  // namespace

void MotionProgramStore::begin() {
    uint8_t slot = 0;
    Preferences prefs;
    if (prefs.begin(NVS_NAMESPACE, false)) {
        slot = prefs.getUChar(NVS_SLOT_KEY, 0);
        prefs.end();
    }
    if (slot >= MOTION_SLOT_COUNT) slot = 0;
    load(slot);
}

bool MotionProgramStore::load(uint8_t slot) {
    program_ = emptyProgram(slot);
    Preferences prefs;
    // Read-only open can fail before the namespace has ever been created.
    // Opening writable creates the empty namespace without manufacturing a
    // program record, so first boot remains a valid empty program.
    if (!prefs.begin(NVS_NAMESPACE, false)) {
        setError("motion storage unavailable");
        return false;
    }
    char key[24];
    slotKey(key, sizeof(key), NVS_KEY_PREFIX, slot);
    StoredMotionProgram stored = {};
    const size_t got = prefs.getBytes(key, &stored, sizeof(stored));
    if (got == sizeof(stored) && validStored(stored)) {
        prefs.end();
        program_.version = stored.version;
        program_.count = stored.count;
        program_.repeat = static_cast<MotionRepeat>(stored.repeat);
        std::memcpy(program_.points, stored.points, sizeof(program_.points));
        clearError();
        return true;
    }
    if (got != 0) {
        prefs.end();
        program_ = emptyProgram(slot);
        setError("motion data invalid");
        return false;
    }

    // There is no v3 record. A valid v2 record migrates into v3; v2 stays in
    // NVS as a recoverable source until a future explicit cleanup policy.
    slotKey(key, sizeof(key), LEGACY_NVS_KEY_PREFIX, slot);
    LegacyStoredMotionProgram legacy = {};
    const size_t legacyGot = prefs.getBytes(key, &legacy, sizeof(legacy));
    prefs.end();
    if (legacyGot == 0) return true;  // First use is a valid empty program.
    if (legacyGot != sizeof(legacy) || !validLegacyStored(legacy)) {
        program_ = emptyProgram(slot);
        setError("motion data invalid");
        return false;
    }

    program_.count = legacy.count;
    program_.repeat = static_cast<MotionRepeat>(legacy.repeat);
    for (uint8_t i = 0; i < legacy.count; ++i) {
        MotionPoint &point = program_.points[i];
        point.captured = legacy.points[i].captured;
        point.holdForGo = legacy.points[i].holdForGo;
        point.transition = MotionTransition::Smooth;
        point.pitch = legacy.points[i].pitch;
        point.yaw = legacy.points[i].yaw;
        point.moveMs = legacy.points[i].moveMs;
        point.dwellMs = legacy.points[i].dwellMs;
    }
    clearError();
    if (save()) return true;
    setError("motion migration failed");
    return false;
}

MotionProgram MotionProgramStore::snapshot() const {
    return program_;
}

bool MotionProgramStore::capture(uint8_t index, float pitch, float yaw) {
    if (index > program_.count || index >= MOTION_POINT_CAPACITY ||
        !std::isfinite(pitch) || !std::isfinite(yaw) || pitch < -180.0f ||
        pitch > 180.0f || yaw < -180.0f || yaw > 180.0f) {
        setError("invalid motion point");
        return false;
    }
    const MotionProgram previous = program_;
    MotionPoint &point = program_.points[index];
    if (index == program_.count) ++program_.count;
    point.captured = true;
    point.pitch = pitch;
    point.yaw = yaw;
    if (!validTiming(point.moveMs, point.dwellMs)) {
        point.moveMs = MOTION_DEFAULT_MOVE_MS;
        point.dwellMs = MOTION_DEFAULT_DWELL_MS;
    }
    clearError();
    if (save()) return true;
    program_ = previous;
    setError("motion save failed");
    return false;
}

bool MotionProgramStore::clear(uint8_t index) {
    if (index >= program_.count) {
        setError("motion point unavailable");
        return false;
    }
    const MotionProgram previous = program_;
    for (uint8_t i = index; i < MOTION_POINT_CAPACITY; ++i) {
        program_.points[i] = MotionPoint{};
    }
    program_.count = index;
    clearError();
    if (save()) return true;
    program_ = previous;
    setError("motion save failed");
    return false;
}

bool MotionProgramStore::remove(uint8_t index) {
    const MotionProgram previous = program_;
    if (!eraseMotionPoint(program_, index)) {
        setError("motion point unavailable");
        return false;
    }
    clearError();
    if (save()) return true;
    program_ = previous;
    setError("motion save failed");
    return false;
}

bool MotionProgramStore::setTiming(uint8_t index, uint32_t moveMs,
                                   uint32_t dwellMs) {
    if (index >= program_.count || !validTiming(moveMs, dwellMs)) {
        setError("invalid motion timing");
        return false;
    }
    const MotionProgram previous = program_;
    program_.points[index].moveMs = moveMs;
    program_.points[index].dwellMs = dwellMs;
    clearError();
    if (save()) return true;
    program_ = previous;
    setError("motion save failed");
    return false;
}

bool MotionProgramStore::setHold(uint8_t index, bool holdForGo) {
    if (index >= program_.count) {
        setError("motion point unavailable");
        return false;
    }
    const MotionProgram previous = program_;
    program_.points[index].holdForGo = holdForGo;
    clearError();
    if (save()) return true;
    program_ = previous;
    setError("motion save failed");
    return false;
}

bool MotionProgramStore::setTransition(uint8_t index,
                                       MotionTransition transition) {
    if (index >= program_.count || transition >= MotionTransition::Count) {
        setError("invalid motion transition");
        return false;
    }
    const MotionProgram previous = program_;
    program_.points[index].transition = transition;
    clearError();
    if (save()) return true;
    program_ = previous;
    setError("motion save failed");
    return false;
}

bool MotionProgramStore::setRepeat(MotionRepeat repeat) {
    if (repeat >= MotionRepeat::Count) {
        setError("invalid repeat mode");
        return false;
    }
    const MotionProgram previous = program_;
    program_.repeat = repeat;
    clearError();
    if (save()) return true;
    program_ = previous;
    setError("motion save failed");
    return false;
}

bool MotionProgramStore::selectSlot(uint8_t slot) {
    if (slot >= MOTION_SLOT_COUNT) {
        setError("invalid programme slot");
        return false;
    }
    if (slot == program_.slot) return true;
    // The programme being left is already on flash; only the choice needs
    // writing. A damaged record still opens -- as an empty programme with
    // the error attached -- because a slot nobody can select is a slot
    // nobody can repair: the first capture overwrites it. Only storage
    // itself being unavailable sends the operator back where they were.
    const MotionProgram previous = program_;
    const bool loaded = load(slot);
    if (!loaded && std::strcmp(program_.error, "motion data invalid") != 0) {
        char why[48];
        std::strncpy(why, program_.error, sizeof(why) - 1);
        why[sizeof(why) - 1] = '\0';
        program_ = previous;
        setError(why[0] ? why : "motion slot unavailable");
        return false;
    }
    if (!saveSlotChoice()) {
        program_ = previous;
        setError("motion save failed");
        return false;
    }
    return loaded;
}

bool MotionProgramStore::reset() {
    const MotionProgram previous = program_;
    program_ = emptyProgram(program_.slot);
    clearError();
    if (save()) return true;
    program_ = previous;
    setError("motion save failed");
    return false;
}

bool MotionProgramStore::save() {
    const StoredMotionProgram stored = toStored(program_);
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, false)) return false;
    char key[24];
    slotKey(key, sizeof(key), NVS_KEY_PREFIX, program_.slot);
    const size_t saved = prefs.putBytes(key, &stored, sizeof(stored));
    prefs.end();
    return saved == sizeof(stored);
}

bool MotionProgramStore::saveSlotChoice() {
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, false)) return false;
    const size_t saved = prefs.putUChar(NVS_SLOT_KEY, program_.slot);
    prefs.end();
    return saved == 1;
}

void MotionProgramStore::clearError() {
    program_.error[0] = '\0';
}

void MotionProgramStore::setError(const char *message) {
    if (!message) {
        clearError();
        return;
    }
    std::strncpy(program_.error, message, sizeof(program_.error) - 1);
    program_.error[sizeof(program_.error) - 1] = '\0';
}

}  // namespace osmo
