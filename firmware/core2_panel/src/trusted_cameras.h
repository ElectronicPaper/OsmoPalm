#pragma once

#include <Arduino.h>

namespace osmo {

// This registry deliberately contains only a camera's public BLE identity.
// Wi-Fi credentials, app pairing material and advertising payloads never cross
// this boundary or reach NVS.
struct TrustedCamera {
    char name[24] = "";
    char address[18] = "";
    uint8_t addressType = 0;
    uint8_t model = 0;
    bool used = false;
};

class TrustedCameras {
public:
    // Keep two catalog slots free for newly discovered cameras even when the
    // local history is full. The UI catalog itself remains six items.
    static constexpr uint8_t kMax = 4;

    void begin();
    uint8_t count() const;
    bool get(uint8_t index, TrustedCamera &out) const;
    bool contains(const char *address, uint8_t addressType) const;
    bool upsert(const TrustedCamera &camera);
    bool erase(const char *address, uint8_t addressType);
    bool selected(TrustedCamera &out) const;
    bool setSelected(const char *address, uint8_t addressType);
    bool clearSelected();

private:
    bool save() const;
    TrustedCamera cameras_[kMax] = {};
    int8_t selected_ = -1;
};

}  // namespace osmo
