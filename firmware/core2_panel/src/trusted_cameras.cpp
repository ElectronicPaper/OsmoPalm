#include "trusted_cameras.h"

#include <Preferences.h>

#include <cctype>
#include <cstring>

namespace osmo {
namespace {

constexpr char NVS_NAMESPACE[] = "osmo-cameras";
constexpr char NVS_KEY[] = "registry-v1";
constexpr uint32_t MAGIC = 0x4F434131;  // OCA1

struct StoredRegistry {
    uint32_t magic = MAGIC;
    TrustedCamera cameras[TrustedCameras::kMax] = {};
    int8_t selected = -1;
};

bool sameIdentity(const TrustedCamera &a, const char *address, uint8_t type) {
    return a.used && address && a.addressType == type &&
           std::strncmp(a.address, address, sizeof(a.address)) == 0;
}

bool validAddress(const char *address) {
    if (!address || !std::memchr(address, '\0', sizeof(TrustedCamera::address)) ||
        std::strlen(address) != 17) return false;
    for (uint8_t i = 0; i < 17; ++i) {
        const bool separator = i == 2 || i == 5 || i == 8 ||
                               i == 11 || i == 14;
        if (separator ? address[i] != ':'
                      : !std::isxdigit(static_cast<unsigned char>(address[i]))) {
            return false;
        }
    }
    return true;
}

bool validRegistry(const StoredRegistry &stored) {
    if (stored.selected < -1 || stored.selected >= TrustedCameras::kMax)
        return false;
    for (uint8_t i = 0; i < TrustedCameras::kMax; ++i) {
        const TrustedCamera &camera = stored.cameras[i];
        if (!camera.used) continue;
        if (camera.addressType > 3 || !validAddress(camera.address) ||
            !std::memchr(camera.name, '\0', sizeof(camera.name))) return false;
        for (uint8_t prior = 0; prior < i; ++prior) {
            if (sameIdentity(stored.cameras[prior], camera.address,
                             camera.addressType)) return false;
        }
    }
    return stored.selected < 0 || stored.cameras[stored.selected].used;
}

}  // namespace

void TrustedCameras::begin() {
    std::memset(cameras_, 0, sizeof(cameras_));
    selected_ = -1;
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, true)) return;
    StoredRegistry stored;
    const size_t got = prefs.getBytes(NVS_KEY, &stored, sizeof(stored));
    prefs.end();
    if (got != sizeof(stored) || stored.magic != MAGIC ||
        !validRegistry(stored)) return;
    std::memcpy(cameras_, stored.cameras, sizeof(cameras_));
    selected_ = stored.selected;
}

uint8_t TrustedCameras::count() const {
    uint8_t result = 0;
    for (const auto &camera : cameras_) if (camera.used) ++result;
    return result;
}

bool TrustedCameras::get(uint8_t index, TrustedCamera &out) const {
    if (index >= kMax || !cameras_[index].used) return false;
    out = cameras_[index];
    return true;
}

bool TrustedCameras::contains(const char *address, uint8_t addressType) const {
    for (const auto &camera : cameras_) {
        if (sameIdentity(camera, address, addressType)) return true;
    }
    return false;
}

bool TrustedCameras::upsert(const TrustedCamera &camera) {
    if (!camera.used || !camera.address[0]) return false;
    int8_t slot = -1;
    for (uint8_t i = 0; i < kMax; ++i) {
        if (sameIdentity(cameras_[i], camera.address, camera.addressType)) {
            slot = i;
            break;
        }
        if (slot < 0 && !cameras_[i].used) slot = i;
    }
    if (slot < 0) return false;
    const TrustedCamera previous = cameras_[slot];
    const int8_t previousSelected = selected_;
    cameras_[slot] = camera;
    cameras_[slot].used = true;
    selected_ = slot;
    if (save()) return true;
    // A failed NVS write must not create a RAM-only truth that disappears on
    // reboot while the UI claims the camera was saved and selected.
    cameras_[slot] = previous;
    selected_ = previousSelected;
    return false;
}

bool TrustedCameras::erase(const char *address, uint8_t addressType) {
    for (uint8_t i = 0; i < kMax; ++i) {
        if (!sameIdentity(cameras_[i], address, addressType)) continue;
        const TrustedCamera previous = cameras_[i];
        const int8_t previousSelected = selected_;
        std::memset(&cameras_[i], 0, sizeof(cameras_[i]));
        if (selected_ == static_cast<int8_t>(i)) selected_ = -1;
        if (save()) return true;
        cameras_[i] = previous;
        selected_ = previousSelected;
        return false;
    }
    return false;
}

bool TrustedCameras::selected(TrustedCamera &out) const {
    return selected_ >= 0 && selected_ < kMax &&
           get(static_cast<uint8_t>(selected_), out);
}

bool TrustedCameras::setSelected(const char *address, uint8_t addressType) {
    for (uint8_t i = 0; i < kMax; ++i) {
        if (sameIdentity(cameras_[i], address, addressType)) {
            if (selected_ == static_cast<int8_t>(i)) return true;
            const int8_t previousSelected = selected_;
            selected_ = i;
            if (save()) return true;
            selected_ = previousSelected;
            return false;
        }
    }
    return false;
}

bool TrustedCameras::clearSelected() {
    if (selected_ < 0) return true;
    const int8_t previousSelected = selected_;
    selected_ = -1;
    if (save()) return true;
    selected_ = previousSelected;
    return false;
}

bool TrustedCameras::save() const {
    StoredRegistry stored;
    std::memcpy(stored.cameras, cameras_, sizeof(cameras_));
    stored.selected = selected_;
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, false)) return false;
    const size_t saved = prefs.putBytes(NVS_KEY, &stored, sizeof(stored));
    prefs.end();
    return saved == sizeof(stored);
}

}  // namespace osmo
