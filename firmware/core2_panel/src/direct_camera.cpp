#include "direct_camera.h"
#include "control_math.h"

#include "duml_codec.h"
#include "trusted_cameras.h"

#include <NimBLEDevice.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_system.h>
#include <esp_wifi.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

namespace osmo {
namespace {
using namespace control;

constexpr char SERVICE_UUID[] = "0000fff0-0000-1000-8000-00805f9b34fb";
constexpr char NOTIFY_UUID[] = "0000fff4-0000-1000-8000-00805f9b34fb";
constexpr char WRITE_UUID[] = "0000fff5-0000-1000-8000-00805f9b34fb";

constexpr char CAMERA_HOST[] = "192.168.2.1";
constexpr uint16_t CAMERA_TCP_PORT = 7001;
constexpr uint16_t CAMERA_UDP_PORT = 9004;

constexpr uint8_t SENDER_APP = 0x02;
constexpr uint8_t RX_CAMERA = 0x01;
constexpr uint8_t RX_GIMBAL = 0x04;
constexpr uint8_t RX_WIFI = 0x07;
constexpr uint8_t RX_SESSION = 0xF0;
constexpr uint8_t RX_WAKE = 0x1C;
constexpr uint8_t RX_DM368_1 = 0x28;
constexpr uint8_t RX_DM368_2 = 0x48;
constexpr uint8_t RX_GIMBAL_INIT = 0x03;

constexpr uint8_t FLAG_NOTIFY = 0x00;
constexpr uint8_t FLAG_REQUEST = 0x40;
constexpr uint8_t FLAG_ACK80 = 0x80;
constexpr uint8_t FLAG_RESPONSE = 0xC0;

constexpr uint8_t PKT_HANDSHAKE = 0x00;
constexpr uint8_t PKT_TELEMETRY = 0x01;
constexpr uint8_t PKT_VIDEO = 0x02;
constexpr uint8_t PKT_ACKED_DATA = 0x03;
constexpr uint8_t PKT_ACK = 0x04;
constexpr uint8_t PKT_COMMAND = 0x05;

constexpr uint16_t INITIAL_DUML_SEQ = 0xA000;
constexpr uint32_t BLE_WRITE_PACING_MS = 250;
// NimBLE 1.4.2 treats BLE_HS_EALREADY as a successful start.  A stale GAP
// discovery can therefore leave isScanning() true forever unless the caller
// owns a second, wall-clock deadline.  Keep each nominal one-second slice
// bounded and give GAP a short window to acknowledge cancellation.
constexpr uint32_t BLE_SCAN_SLICE_TIMEOUT_MS = 1800;
constexpr uint32_t BLE_SCAN_STOP_TIMEOUT_MS = 300;
// The Pocket can accept the first BLE connection and then drop it while the
// central is discovering GATT.  Re-probe only the explicitly selected peer,
// with a fresh client, before falling back to the outer reconnect loop.
constexpr uint8_t BLE_LINK_ATTEMPTS = 4;
constexpr uint32_t BLE_LINK_RETRY_BASE_MS = 750;
constexpr uint8_t WIFI_JOIN_ATTEMPTS = 3;
// Host parity: a newly-woken Pocket AP can take well beyond six seconds to
// beacon. A shorter window repeatedly tore BLE down and restarted PAIR before
// the camera's Wi-Fi radio became visible.
constexpr uint32_t WIFI_SCAN_WINDOW_MS = 25000;
constexpr uint32_t WIFI_ASSOCIATE_WINDOW_MS = 12000;
constexpr uint32_t TELEMETRY_FRESH_MS = 350;
constexpr uint32_t IMU_FRESH_MS = 250;
constexpr uint32_t RETRY_MS = 5000;
constexpr uint32_t STICK_INTERVAL_MS = 40;
// Keep the receive path cooperative.  Unlike the host implementation, which
// has independent RX and ACK workers, this task services both jobs.  A camera
// burst must not keep the 25 ms receive-window ACK waiting behind an unbounded
// socket drain.
constexpr uint8_t MAX_RX_PACKETS_PER_PUMP = 8;
constexpr uint16_t BLE_REQUIRED_MTU = 54;
constexpr float MAX_DPS = 42.0f;
// Programmed motion is intentionally capped below the manual FAST profile
// until the Pocket 4 Pro yaw envelope has been camera-body-qualified.
constexpr float PROGRAM_MAX_DPS = 18.0f;
constexpr float PROGRAM_START_PITCH_TOL = 1.0f;
constexpr float PROGRAM_START_YAW_TOL = 1.5f;
constexpr float PROGRAM_SETTLE_PITCH_TOL = 0.65f;
constexpr float PROGRAM_SETTLE_YAW_TOL = 0.9f;
constexpr uint32_t PROGRAM_SETTLE_HOLD_MS = 240;
constexpr uint32_t PROGRAM_SETTLE_BUDGET_MS = 3000;
constexpr float PROGRAM_YAW_LEG_LIMIT = 120.0f;
constexpr float CAMERA_COAST_S = 0.47f;

constexpr char APP_IDENTIFIER[] = "284ae5b8d76b3375a04a6417ad71bea3";
constexpr char PAIRING_PIN[] = "osmo";

TrustedCameras trustedCameras;

constexpr char SUBSCRIPTION_KEYS[][19] = {
    "cam_status",
    "cam_storage",
    "cam_expo_param",
    "cam_video_param_v2",
    "cam_record_time",
};
constexpr uint32_t FIRST_SUB_ID = 0x69DF;


void putLe16(uint8_t *out, uint16_t value) {
    out[0] = static_cast<uint8_t>(value);
    out[1] = static_cast<uint8_t>(value >> 8);
}

void putLe32(uint8_t *out, uint32_t value) {
    out[0] = static_cast<uint8_t>(value);
    out[1] = static_cast<uint8_t>(value >> 8);
    out[2] = static_cast<uint8_t>(value >> 16);
    out[3] = static_cast<uint8_t>(value >> 24);
}

int16_t readI16(const uint8_t *p) {
    return static_cast<int16_t>(p[0] | (p[1] << 8));
}

bool containsCameraName(const std::string &name) {
    std::string lower = name;
    std::transform(lower.begin(), lower.end(), lower.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return lower.find("osmo") != std::string::npos ||
           lower.find("dji") != std::string::npos;
}

bool isPocketManufacturerData(const std::string &data) {
    if (data.size() < 3) return false;
    const uint16_t company = static_cast<uint8_t>(data[0]) |
        (static_cast<uint16_t>(static_cast<uint8_t>(data[1])) << 8);
    if (company != 0x08AA && company != 0xF7AA) return false;

    // NimBLE retains the two-byte company id that Bleak exposes separately.
    // Match the host driver's bounded model scan after those two bytes.
    const size_t end = std::min(data.size(), static_cast<size_t>(10));
    for (size_t i = 2; i < end; ++i) {
        const uint8_t model = static_cast<uint8_t>(data[i]);
        if (model == 0x20 || model == 0x21 || model == 0x22) return true;
    }
    return false;
}

bool hasPocketManufacturerData(NimBLEAdvertisedDevice &device) {
    for (uint8_t i = 0; i < device.getManufacturerDataCount(); ++i) {
        if (isPocketManufacturerData(device.getManufacturerData(i))) return true;
    }
    return false;
}

uint32_t cameraId(const char *address, uint8_t addressType) {
    // FNV-1a, stable across scans but not a credential or a camera command.
    uint32_t hash = 2166136261u ^ addressType;
    for (const char *p = address; p && *p; ++p) {
        hash ^= static_cast<uint8_t>(*p);
        hash *= 16777619u;
    }
    return hash ? hash : 1;
}

uint8_t pocketModel(NimBLEAdvertisedDevice &device) {
    for (uint8_t i = 0; i < device.getManufacturerDataCount(); ++i) {
        const std::string data = device.getManufacturerData(i);
        for (size_t at = 2; at < std::min(data.size(), static_cast<size_t>(10)); ++at) {
            const uint8_t value = static_cast<uint8_t>(data[at]);
            if (value == 0x20 || value == 0x21 || value == 0x22) return value;
        }
    }
    return 0;
}

// The UI owns raw touch debouncing and calls setJog(false) only for its physical
// release path. Keep that raw edge separate from Inputs::jogActive: STOP and
// reconnect may neutralise the logical command, but must not pretend the finger
// was lifted. The epoch lets a release that happens during reconnect satisfy
// the next runtime without an undocumented extra press/release gesture.
std::atomic<bool> rawJogHeld{false};
std::atomic<uint32_t> rawJogReleaseEpoch{0};

struct RawJogSnapshot {
    bool held;
    uint32_t releaseEpoch;
};

RawJogSnapshot snapshotRawJog() {
    // A release stores held=false before advancing the epoch. If it lands
    // between the two epoch reads, retry; observing false with the old epoch is
    // also safe because the physical release has already happened.
    while (true) {
        const uint32_t before =
            rawJogReleaseEpoch.load(std::memory_order_acquire);
        const bool held = rawJogHeld.load(std::memory_order_acquire);
        const uint32_t after =
            rawJogReleaseEpoch.load(std::memory_order_acquire);
        if (before == after) return {held, after};
    }
}

}  // namespace

DirectCamera directCamera;

class DirectRuntime {
public:
    explicit DirectRuntime(DirectCamera &owner, uint32_t expectedReconnectEpoch)
        : owner_(owner), reconnectAtStart_(expectedReconnectEpoch) {
        // The UI may cancel or replace an action between taskLoop's final
        // launch snapshot and this constructor. Bind the runtime to the epoch
        // that was actually approved for launch; never adopt a newer Cancel
        // epoch and accidentally start work after cancellation.
        if (!wanted()) return;
        status_ = owner_.status();
        const MotionProgram stored = owner_.motionProgram();
        status_.motionCount = stored.count;
        status_.motionPhase = stored.count ? MotionPhase::Ready
                                           : MotionPhase::Empty;
        status_.motionProgramArmed = false;
        status_.motionProgramActive = false;
        status_.motionProgramFault = false;
        status_.enabled = true;
        status_.phase = DirectPhase::Starting;
        std::strncpy(status_.detail, "direct camera starting",
                     sizeof(status_.detail) - 1);
        owner_.publish(status_);
    }

    ~DirectRuntime() { cleanup(); }

    bool run() {
        char ssid[34] = {};
        char password[65] = {};
        // One explicit connect epoch owns one BLE pair/wake/credential leg.
        // The proven host then gives the sleepy AP a full visibility window
        // and retries association without returning to PAIR. Re-provisioning
        // every 6.5 seconds could reset the very AP we were waiting to join.
        if (!provisionOverBle(ssid, sizeof(ssid), password,
                             sizeof(password))) {
            secureZero(password, sizeof(password));
            return false;
        }
        const bool joined = wanted() && joinWifi(ssid, password);
        secureZero(password, sizeof(password));
        if (!joined) return failIfWanted(lastError_);
        if (!openDatalink()) return false;
        return readyLoop();
    }

    const char *lastError() const { return lastError_; }
    bool wasControlFault() const { return controlFault_; }
    bool needsManualWifiRetry() const { return manualWifiRetry_; }

private:
    struct BleFrame {
        uint16_t seq = 0;
        uint8_t flags = 0;
        uint8_t cmdSet = 0;
        uint8_t cmdId = 0;
        uint8_t payload[192] = {};
        size_t payloadLen = 0;
    };

    static DirectRuntime *activeBle_;

    static void notifyCallback(NimBLERemoteCharacteristic *, uint8_t *data,
                               size_t len, bool) {
        DirectRuntime *self = activeBle_;
        if (!self || !data || !len) return;
        portENTER_CRITICAL(&self->bleMux_);
        if (len > sizeof(self->bleRx_) - self->bleRxLen_) {
            // Never retain a truncated frame. Drop the old partial buffer and
            // accept the new notification if it fits by itself.
            self->bleRxLen_ = 0;
        }
        if (len <= sizeof(self->bleRx_) - self->bleRxLen_) {
            std::memcpy(self->bleRx_ + self->bleRxLen_, data, len);
            self->bleRxLen_ += len;
        }
        portEXIT_CRITICAL(&self->bleMux_);
    }

    bool wanted() const {
        const auto in = owner_.inputs();
        return in.wantEnabled && in.reconnectEpoch == reconnectAtStart_;
    }

    bool waitWhileWanted(uint32_t durationMs) const {
        const uint32_t started = millis();
        while (wanted() && millis() - started < durationMs) delay(20);
        return wanted();
    }

    static bool stopBleScan(NimBLEScan *scan) {
        if (!scan || !scan->isScanning()) return true;
        if (!scan->stop()) return false;
        const uint32_t stoppedAt = millis();
        while (scan->isScanning() &&
               millis() - stoppedAt < BLE_SCAN_STOP_TIMEOUT_MS) {
            delay(10);
        }
        return !scan->isScanning();
    }

    void setPhase(DirectPhase phase, const char *detail) {
        status_.enabled = true;
        status_.phase = phase;
        if (phase != DirectPhase::RetryWait) {
            status_.blockedAt = DirectPhase::Off;
            status_.retryAtMs = 0;
        }
        status_.ready = (phase == DirectPhase::Ready);
        if (detail) {
            std::strncpy(status_.detail, detail, sizeof(status_.detail) - 1);
            status_.detail[sizeof(status_.detail) - 1] = '\0';
        }
        owner_.publish(status_);
    }

    bool fail(const char *detail, bool controlFault = false) {
        std::strncpy(lastError_, detail ? detail : "direct link failed",
                     sizeof(lastError_) - 1);
        lastError_[sizeof(lastError_) - 1] = '\0';
        controlFault_ = controlFault_ || controlFault || motionActive_;
        status_.ready = false;
        status_.telemetry = false;
        status_.moving = false;
        status_.clutch = false;
        // stopProgram owns every field a dead programme must clear -- the
        // inline copy here forgot cuePoint, so a cue survived link loss and
        // the panel kept offering GO on a programme that no longer existed.
        if (programArmed_ || programActive_)
            stopProgram(MotionPhase::Fault, nullptr);
        status_.controlFault = controlFault_;
        status_.blockedAt = status_.phase;
        setPhase(DirectPhase::RetryWait, lastError_);
        return false;
    }

    bool failIfWanted(const char *detail, bool controlFault = false) {
        return wanted() ? fail(detail, controlFault) : false;
    }

    void setWifiEvidence(uint8_t attempt, wl_status_t wifiStatus,
                         uint8_t channel, const uint8_t *bssid,
                         const char *reason, const char *detail) {
        status_.wifiAttempt = attempt;
        status_.wifiStatus = static_cast<uint8_t>(wifiStatus);
        status_.wifiChannel = channel;
        if (bssid) std::memcpy(status_.wifiBssid, bssid, sizeof(status_.wifiBssid));
        else std::memset(status_.wifiBssid, 0, sizeof(status_.wifiBssid));
        std::strncpy(status_.wifiReason, reason ? reason : "",
                     sizeof(status_.wifiReason) - 1);
        status_.wifiReason[sizeof(status_.wifiReason) - 1] = '\0';
        setPhase(DirectPhase::WifiJoin, detail);
    }

    static void secureZero(void *ptr, size_t len) {
        volatile uint8_t *p = static_cast<volatile uint8_t *>(ptr);
        while (len--) *p++ = 0;
    }

    void publishNearby(NimBLEScanResults &results,
                       const DirectCamera::Inputs &in, bool scanning = false) {
        CameraCatalog next;
        uint8_t addressTypes[6] = {};
        next.version = owner_.catalogSnapshot().version + 1;
        next.scanning = scanning;
        next.actionPending = owner_.catalogSnapshot().actionPending;
        // Retained trusted identities are intentionally included even when
        // their advertiser is absent, so an operator can see and locally
        // forget a former camera without a computer.
        for (uint8_t i = 0; i < TrustedCameras::kMax && next.count < 6; ++i) {
            TrustedCamera saved;
            if (!trustedCameras.get(i, saved)) continue;
            const uint8_t slot = next.count++;
            CameraSummary &item = next.items[slot];
            item.id = cameraId(saved.address, saved.addressType);
            addressTypes[slot] = saved.addressType;
            item.model = saved.model;
            item.rssi = -127;
            item.trusted = true;
            item.selected = std::strncmp(in.requestedAddress, saved.address,
                sizeof(item.address)) == 0 && in.requestedAddressType == saved.addressType;
            std::strncpy(item.name, saved.name, sizeof(item.name) - 1);
            std::strncpy(item.address, saved.address, sizeof(item.address) - 1);
        }
        const NimBLEUUID serviceUuid(SERVICE_UUID);
        for (int i = 0; i < results.getCount(); ++i) {
            NimBLEAdvertisedDevice device = results.getDevice(i);
            const std::string name = device.haveName() ? device.getName() : "";
            if (!containsCameraName(name) && !hasPocketManufacturerData(device) &&
                !device.isAdvertisingService(serviceUuid)) continue;
            const NimBLEAddress address = device.getAddress();
            const std::string addressText = address.toString();
            const uint8_t addressType = address.getType();
            const int8_t observedRssi = static_cast<int8_t>(
                std::max(-127, std::min(0, device.getRSSI())));
            CameraSummary *item = nullptr;
            uint8_t itemIndex = 0;
            for (uint8_t at = 0; at < next.count; ++at) {
                if (next.items[at].id == cameraId(addressText.c_str(), addressType)) {
                    item = &next.items[at];
                    itemIndex = at;
                    break;
                }
            }
            if (!item) {
                if (next.count < 6) {
                    itemIndex = next.count++;
                } else {
                    // Four saved slots leave two discovery slots. Keep those
                    // two as the strongest nearby candidates rather than the
                    // first two advertisers returned by the controller.
                    int8_t weakestRssi = 1;
                    int8_t weakest = -1;
                    for (uint8_t at = 0; at < next.count; ++at) {
                        if (next.items[at].trusted) continue;
                        if (weakest < 0 || next.items[at].rssi < weakestRssi) {
                            weakest = static_cast<int8_t>(at);
                            weakestRssi = next.items[at].rssi;
                        }
                    }
                    if (weakest < 0 || observedRssi <= weakestRssi) continue;
                    itemIndex = static_cast<uint8_t>(weakest);
                    next.items[itemIndex] = CameraSummary{};
                }
                item = &next.items[itemIndex];
                addressTypes[itemIndex] = addressType;
                item->id = cameraId(addressText.c_str(), addressType);
                std::strncpy(item->address, addressText.c_str(), sizeof(item->address) - 1);
                item->trusted = trustedCameras.contains(addressText.c_str(), addressType);
            }
            item->nearby = true;
            item->rssi = observedRssi;
            item->model = pocketModel(device);
            item->selected = std::strncmp(in.requestedAddress, addressText.c_str(),
                sizeof(item->address)) == 0 && in.requestedAddressType == addressType;
            if (!name.empty()) std::strncpy(item->name, name.c_str(), sizeof(item->name) - 1);
            if (!item->name[0]) std::strncpy(item->name, "DJI Pocket", sizeof(item->name) - 1);
        }
        for (uint8_t i = 0; i < next.count; ++i) for (uint8_t j = i + 1; j < next.count; ++j) {
            const bool swap = (!next.items[i].nearby && next.items[j].nearby) ||
                (next.items[i].nearby == next.items[j].nearby && next.items[j].rssi > next.items[i].rssi);
            if (swap) { std::swap(next.items[i], next.items[j]); std::swap(addressTypes[i], addressTypes[j]); }
        }
        owner_.publishCatalogWithAddressTypes(next, addressTypes);
    }

    bool provisionOverBle(char *ssid, size_t ssidCap,
                          char *password, size_t passwordCap) {
        setPhase(DirectPhase::BleScan, "scanning for camera");
        NimBLEDevice::init("Osmo Core2");
        bleInitialised_ = true;
        NimBLEDevice::setPower(ESP_PWR_LVL_P9);
        // The capability probe uses 255: it easily carries the 51-byte pairing
        // frame while being more likely to be accepted by a conservative peer.
        NimBLEDevice::setMTU(255);

        NimBLEScan *scan = NimBLEDevice::getScan();
        scan->setActiveScan(true);
        scan->setInterval(45);
        scan->setWindow(15);
        // A failed deinit in an earlier runtime can leave GAP discovery alive.
        // Never accept NimBLE's EALREADY-as-success path as a new scan epoch.
        if (!stopBleScan(scan))
            return failIfWanted("BLE scan reset failed");
        scan->clearResults();
        // One-second bounded slices make Cancel responsive without exposing
        // NimBLE objects to LVGL. `is_continue` retains previous results, and
        // our wall-clock guard contains a missing GAP completion callback.
        NimBLEScanResults results;
        for (uint8_t second = 0; second < 8 && wanted(); ++second) {
            const uint32_t sliceStartedAt = millis();
            const bool started = scan->start(1, nullptr, second != 0);
            if (!started) {
                scan->clearResults();
                return failIfWanted("BLE scan start failed");
            }
            while (scan->isScanning() && wanted() &&
                   millis() - sliceStartedAt < BLE_SCAN_SLICE_TIMEOUT_MS) {
                delay(20);
            }
            if (!wanted()) {
                if (stopBleScan(scan)) scan->clearResults();
                return false;
            }
            if (scan->isScanning()) {
                const bool stopped = stopBleScan(scan);
                if (stopped) scan->clearResults();
                return failIfWanted(stopped ? "BLE scan timed out"
                                            : "BLE scan stop failed");
            }
            results = scan->getResults();
            if (wanted()) publishNearby(results, owner_.inputs(), true);
        }
        if (!wanted()) return false;

        const auto selected = owner_.inputs();
        publishNearby(results, selected, false);
        if (selected.deviceMode == DirectCamera::DeviceMode::ScanOnly) {
            CameraCatalog complete = owner_.catalogSnapshot();
            complete.scanning = false;
            complete.actionPending = false;
            ++complete.version;
            owner_.publishCatalog(complete);
            scan->clearResults();
            return false;  // catalog-only request: never pair as a side effect
        }
        if (!selected.requestedAddress[0]) {
            scan->clearResults();
            return failIfWanted("choose a camera on Core2");
        }
        if (!trustedCameras.contains(selected.requestedAddress,
                                     selected.requestedAddressType) &&
            trustedCameras.count() >= TrustedCameras::kMax) {
            scan->clearResults();
            return failIfWanted("saved list full; forget one first");
        }

        const NimBLEUUID serviceUuid(SERVICE_UUID);
        bool sawSelectedAdvertisement = false;
        bool sawConnectionFailure = false;
        bool sawGattDisconnect = false;
        bool sawMissingService = false;
        bool sawMissingControls = false;
        bool sawLowMtu = false;
        bool sawBadProperties = false;
        for (int i = 0; i < results.getCount() && !client_; ++i) {
            NimBLEAdvertisedDevice device = results.getDevice(i);
            const std::string name = device.haveName() ? device.getName() : "";
            if (!containsCameraName(name) &&
                !hasPocketManufacturerData(device) &&
                !device.isAdvertisingService(serviceUuid)) {
                continue;
            }
            const NimBLEAddress address = device.getAddress();
            if (address.getType() != selected.requestedAddressType ||
                address.toString() != selected.requestedAddress) {
                continue;  // explicit choice: never connect first compatible peer
            }
            sawSelectedAdvertisement = true;

            // A broad DJI/name match is only discovery. Do not select it until
            // this exact peer proves it owns the complete camera GATT spine.
            // A failed discovery can leave partial service objects behind, so
            // every retry gets a new client and therefore a clean cache.
            for (uint8_t attempt = 0;
                 attempt < BLE_LINK_ATTEMPTS && wanted() && !client_;
                 ++attempt) {
                char detail[48] = {};
                snprintf(detail, sizeof(detail), "camera BLE link %u/%u",
                         attempt + 1, BLE_LINK_ATTEMPTS);
                setPhase(DirectPhase::BlePair, detail);

                NimBLEClient *candidate = NimBLEDevice::createClient();
                candidate->setConnectTimeout(6);
                const bool connected = attempt == 0
                    ? candidate->connect(&device)
                    : candidate->connect(address);
                if (!wanted()) {
                    NimBLEDevice::deleteClient(candidate);
                    scan->clearResults();
                    return false;
                }
                if (!connected) {
                    sawConnectionFailure = true;
                    NimBLEDevice::deleteClient(candidate);
                    if (attempt + 1 < BLE_LINK_ATTEMPTS) {
                        setPhase(DirectPhase::BlePair,
                                 "camera BLE busy; retrying");
                        const uint32_t backoff =
                            BLE_LINK_RETRY_BASE_MS * (attempt + 1);
                        if (!waitWhileWanted(backoff)) {
                            scan->clearResults();
                            return false;
                        }
                        continue;
                    }
                    break;
                }

                NimBLERemoteService *service =
                    candidate->getService(SERVICE_UUID);
                const int serviceError = candidate->getLastError();
                if (!wanted()) {
                    NimBLEDevice::deleteClient(candidate);
                    scan->clearResults();
                    return false;
                }
                if (!service) {
                    const bool transient = !candidate->isConnected() ||
                        serviceError == BLE_HS_ENOTCONN ||
                        serviceError == BLE_HS_EAGAIN ||
                        serviceError == BLE_HS_ETIMEOUT;
                    sawGattDisconnect = sawGattDisconnect || transient;
                    sawMissingService = sawMissingService || !transient;
                    NimBLEDevice::deleteClient(candidate);
                    if (transient && attempt + 1 < BLE_LINK_ATTEMPTS) {
                        setPhase(DirectPhase::BlePair,
                                 "camera BLE dropped; retrying");
                        const uint32_t backoff =
                            BLE_LINK_RETRY_BASE_MS * (attempt + 1);
                        if (!waitWhileWanted(backoff)) {
                            scan->clearResults();
                            return false;
                        }
                        continue;
                    }
                    break;
                }

                NimBLERemoteCharacteristic *notify =
                    service->getCharacteristic(NOTIFY_UUID);
                NimBLERemoteCharacteristic *writer =
                    service->getCharacteristic(WRITE_UUID);
                if (!wanted()) {
                    NimBLEDevice::deleteClient(candidate);
                    scan->clearResults();
                    return false;
                }
                if (!notify || !writer) {
                    const bool transient = !candidate->isConnected();
                    sawGattDisconnect = sawGattDisconnect || transient;
                    sawMissingControls = sawMissingControls || !transient;
                    NimBLEDevice::deleteClient(candidate);
                    if (transient && attempt + 1 < BLE_LINK_ATTEMPTS) {
                        setPhase(DirectPhase::BlePair,
                                 "camera BLE dropped; retrying");
                        const uint32_t backoff =
                            BLE_LINK_RETRY_BASE_MS * (attempt + 1);
                        if (!waitWhileWanted(backoff)) {
                            scan->clearResults();
                            return false;
                        }
                        continue;
                    }
                    break;
                }
                if (candidate->getMTU() < BLE_REQUIRED_MTU) {
                    sawLowMtu = true;
                    NimBLEDevice::deleteClient(candidate);
                    break;
                }
                if (!notify->canNotify() || !notify->canWrite() ||
                    !writer->canWriteNoResponse()) {
                    sawBadProperties = true;
                    NimBLEDevice::deleteClient(candidate);
                    break;
                }

                client_ = candidate;
                notify_ = notify;
                writer_ = writer;
            }
            break;  // the selected identity is singular; never try another peer
        }
        scan->clearResults();
        if (!wanted()) return false;
        if (!client_) {
            if (sawLowMtu) return failIfWanted("camera BLE MTU below 54");
            if (sawBadProperties) return failIfWanted("camera BLE properties incompatible");
            if (sawMissingService || sawMissingControls) {
                return failIfWanted("camera BLE service/controls missing");
            }
            if (sawGattDisconnect)
                return failIfWanted("camera BLE dropped during GATT");
            if (sawConnectionFailure)
                return failIfWanted("camera BLE connection failed");
            if (sawSelectedAdvertisement)
                return failIfWanted("camera BLE probe failed");
            return failIfWanted("camera not found over BLE");
        }

        if (!wanted()) return false;
        activeBle_ = this;
        // Require the CCCD write response. The no-response default can report
        // success before the camera has actually armed the notification path.
        const bool subscribed = notify_->subscribe(true, notifyCallback, true);
        if (!wanted()) return false;
        if (!subscribed) {
            return failIfWanted("camera BLE notify failed");
        }
        const uint8_t arm[] = {0x01, 0x00};
        if (!wanted()) return false;
        const bool armed = notify_->writeValue(arm, sizeof(arm), true);
        if (!wanted()) return false;
        if (!armed) {
            return failIfWanted("camera pairing arm failed");
        }
        delay(200);

        setPhase(DirectPhase::BlePair, "pairing with camera");
        const uint8_t wakePayload[] = {0x04, 0x00};
        if (!bleSend(RX_SESSION, 0x802B, FLAG_REQUEST, 0x00, 0x2B,
                     wakePayload, sizeof(wakePayload))) {
            return failIfWanted("BLE session wake failed");
        }

        uint8_t pairPayload[1 + sizeof(APP_IDENTIFIER) - 1 +
                            1 + sizeof(PAIRING_PIN) - 1] = {};
        size_t at = 0;
        pairPayload[at++] = sizeof(APP_IDENTIFIER) - 1;
        std::memcpy(pairPayload + at, APP_IDENTIFIER,
                    sizeof(APP_IDENTIFIER) - 1);
        at += sizeof(APP_IDENTIFIER) - 1;
        pairPayload[at++] = sizeof(PAIRING_PIN) - 1;
        std::memcpy(pairPayload + at, PAIRING_PIN, sizeof(PAIRING_PIN) - 1);
        at += sizeof(PAIRING_PIN) - 1;
        if (!bleSend(RX_WIFI, 0x8092, FLAG_REQUEST, 0x07, 0x45,
                     pairPayload, at)) {
            return failIfWanted("camera pairing request failed");
        }

        BleFrame reply;
        if (!waitBle(0x07, 0x45, reply, 8000)) {
            return failIfWanted("camera pairing reply timed out");
        }
        if (!wanted()) return false;
        const uint8_t pairingStatus =
            reply.payloadLen > 1 ? reply.payload[1] : 0xFF;
        if (pairingStatus == 0x02) {
            setPhase(DirectPhase::AwaitApproval, "approve on camera screen");
            BleFrame approval;
            if (!waitBle(0x07, 0x46, approval, 60000)) {
                return failIfWanted("camera approval timed out");
            }
            const uint8_t ok[] = {0x00};
            if (!bleSend(RX_WIFI, approval.seq, FLAG_RESPONSE,
                         0x07, 0x46, ok, sizeof(ok))) {
                return failIfWanted("camera approval ACK failed");
            }
        } else if (pairingStatus == 0x06) {
            return failIfWanted("close Mimo or other camera controller");
        } else if (pairingStatus != 0x01) {
            return failIfWanted("camera rejected app pairing");
        }

        const uint8_t wakeApPayload[] = {0, 0, 0, 0};
        if (!bleSend(RX_WAKE, 0x8053, FLAG_REQUEST, 0x53, 0x10,
                     wakeApPayload, sizeof(wakeApPayload))) {
            return failIfWanted("camera AP wake failed");
        }
        delay(500);

        if (!bleSend(RX_WIFI, 0x8007, FLAG_REQUEST, 0x07, 0x07,
                     nullptr, 0) ||
            !waitBle(0x07, 0x07, reply, 8000) ||
            !unpackStatusString(reply, ssid, ssidCap)) {
            return failIfWanted("camera SSID unavailable");
        }
        if (!bleSend(RX_WIFI, 0x800E, FLAG_REQUEST, 0x07, 0x0E,
                     nullptr, 0) ||
            !waitBle(0x07, 0x0E, reply, 8000)) {
            return failIfWanted("camera Wi-Fi key unavailable");
        }
        const bool passwordUnpacked =
            unpackStatusString(reply, password, passwordCap);
        // The decoded BLE reply held the same key as the destination buffer;
        // scrub that second copy immediately, whether decoding succeeded or
        // not. The caller separately scrubs password after each join attempt.
        secureZero(reply.payload, sizeof(reply.payload));
        reply.payloadLen = 0;
        if (!passwordUnpacked) {
            return failIfWanted("camera Wi-Fi key unavailable");
        }
        if (!ssid[0] || !password[0]) {
            return failIfWanted("camera sent empty Wi-Fi key");
        }

        // App authorization and both credential reads have succeeded. Only at
        // this edge does a discovered camera become trusted in Core2's local,
        // non-secret registry. The WPA key remains in the stack buffer only.
        TrustedCamera trusted;
        trusted.used = true;
        trusted.addressType = selected.requestedAddressType;
        std::strncpy(trusted.address, selected.requestedAddress,
                     sizeof(trusted.address) - 1);
        for (uint8_t i = 0; i < owner_.catalogSnapshot().count; ++i) {
            const CameraSummary &summary = owner_.catalogSnapshot().items[i];
            if (summary.id != selected.requestedDeviceId) continue;
            trusted.model = summary.model;
            std::strncpy(trusted.name, summary.name, sizeof(trusted.name) - 1);
            break;
        }
        if (!trusted.name[0]) std::strncpy(trusted.name, "DJI Pocket", sizeof(trusted.name) - 1);
        if (!trustedCameras.upsert(trusted))
            return failIfWanted("camera registry save failed");
        CameraCatalog verified = owner_.catalogSnapshot();
        for (uint8_t i = 0; i < verified.count; ++i) {
            if (verified.items[i].id == selected.requestedDeviceId) {
                verified.items[i].trusted = true;
                verified.items[i].verified = true;
                break;
            }
        }
        ++verified.version;
        owner_.publishCatalog(verified);

        std::strncpy(status_.ssid, ssid, sizeof(status_.ssid) - 1);
        status_.ssid[sizeof(status_.ssid) - 1] = '\0';
        owner_.publish(status_);

        closeBle();  // shared radio: BLE has finished its only job
        return true;
    }

    bool bleSend(uint8_t receiver, uint16_t seq, uint8_t flags,
                 uint8_t cmdSet, uint8_t cmdId,
                 const uint8_t *payload, size_t payloadLen) {
        if (!wanted()) return false;
        if (!writer_ || !client_ || !client_->isConnected()) return false;
        uint8_t frame[256];
        const size_t len = wire::encodeFrame(frame, sizeof(frame), SENDER_APP,
            receiver, seq, flags, cmdSet, cmdId, payload, payloadLen);
        if (!wanted()) return false;
        if (!len || !writer_->writeValue(frame, len, false)) return false;
        delay(BLE_WRITE_PACING_MS);
        return wanted();
    }

    bool popBleFrame(BleFrame &out) {
        uint8_t local[sizeof(bleRx_)];
        size_t localLen;
        portENTER_CRITICAL(&bleMux_);
        localLen = bleRxLen_;
        std::memcpy(local, bleRx_, localLen);
        portEXIT_CRITICAL(&bleMux_);
        if (!localLen) return false;

        size_t start = 0;
        while (start < localLen && local[start] != 0x55) ++start;
        if (start == localLen) {
            portENTER_CRITICAL(&bleMux_);
            bleRxLen_ = 0;
            portEXIT_CRITICAL(&bleMux_);
            return false;
        }
        if (localLen - start < 13) {
            if (start) consumeBle(start);
            return false;
        }
        const size_t total = local[start + 1] |
                             ((local[start + 2] & 0x03u) << 8);
        if (total < 13 || total > sizeof(local)) {
            consumeBle(start + 1);
            return false;
        }
        if (localLen - start < total) {
            if (start) consumeBle(start);
            return false;
        }
        wire::FrameView view;
        if (!wire::decodeFrame(local + start, localLen - start, view)) {
            consumeBle(start + 1);
            return false;
        }
        out.seq = view.seq;
        out.flags = view.flags;
        out.cmdSet = view.cmdSet;
        out.cmdId = view.cmdId;
        out.payloadLen = std::min(view.payloadLen, sizeof(out.payload));
        std::memcpy(out.payload, view.payload, out.payloadLen);
        consumeBle(start + view.totalLen);
        return true;
    }

    void consumeBle(size_t count) {
        portENTER_CRITICAL(&bleMux_);
        count = std::min(count, bleRxLen_);
        if (count < bleRxLen_) {
            std::memmove(bleRx_, bleRx_ + count, bleRxLen_ - count);
        }
        bleRxLen_ -= count;
        portEXIT_CRITICAL(&bleMux_);
    }

    bool waitBle(uint8_t cmdSet, uint8_t cmdId, BleFrame &out,
                 uint32_t timeoutMs) {
        const uint32_t started = millis();
        while (wanted() && millis() - started < timeoutMs) {
            BleFrame got;
            while (popBleFrame(got)) {
                if (got.cmdSet == cmdSet && got.cmdId == cmdId) {
                    if (!wanted()) return false;
                    out = got;
                    return true;
                }
            }
            delay(10);
        }
        return false;
    }

    static bool unpackStatusString(const BleFrame &frame, char *out,
                                   size_t capacity) {
        if (!out || !capacity || frame.payloadLen < 2) return false;
        const size_t declared = frame.payload[1];
        if (!declared || declared > frame.payloadLen - 2 ||
            declared >= capacity) return false;
        std::memcpy(out, frame.payload + 2, declared);
        out[declared] = '\0';
        return true;
    }

    void closeBle() {
        activeBle_ = nullptr;
        notify_ = nullptr;
        writer_ = nullptr;
        if (client_) {
            if (client_->isConnected()) client_->disconnect();
            NimBLEDevice::deleteClient(client_);
            client_ = nullptr;
        }
        if (bleInitialised_) {
            // closeBle also runs after cancellation and failed provisioning.
            // Stop discovery explicitly because deinit(true) may fail to tear
            // down a still-active NimBLE host and poison the next retry.
            NimBLEScan *scan = NimBLEDevice::getScan();
            stopBleScan(scan);  // best effort; this helper is itself bounded
            NimBLEDevice::deinit(true);
            bleInitialised_ = false;
        }
        portENTER_CRITICAL(&bleMux_);
        secureZero(bleRx_, sizeof(bleRx_));
        bleRxLen_ = 0;
        portEXIT_CRITICAL(&bleMux_);
    }

    bool joinWifi(const char *ssid, const char *password) {
        // Core2's ESP32 radio is 2.4 GHz only.  Scan first, both to avoid
        // racing the AP startup and to lock association to the exact BSSID
        // and channel the camera is actually advertising.
        char scanning[48] = {};
        snprintf(scanning, sizeof(scanning), "Wi-Fi: waiting up to 25s for AP");
        setWifiEvidence(1, WiFi.status(), 0, nullptr,
                        "waiting for AP", scanning);
        WiFi.persistent(false);
        WiFi.mode(WIFI_STA);
        esp_wifi_set_storage(WIFI_STORAGE_RAM);
        WiFi.setAutoReconnect(false);
        wifiStarted_ = true;

        uint8_t bssid[6] = {};
        uint8_t channel = 0;
        bool found = false;
        const uint32_t scanStarted = millis();
        while (wanted() && millis() - scanStarted < WIFI_SCAN_WINDOW_MS) {
            // Async scanning keeps STOP/reconnect cancellation responsive and
            // makes WIFI_SCAN_WINDOW_MS a real bound.  The Arduino core's
            // synchronous path can otherwise wait an extra ten seconds.
            // Use an ordinary scan like the proven host, then match the exact
            // SSID in the completed result set. A directed/filter scan issued
            // while the AP is still waking can miss its first beacon cycle.
            const int launched = WiFi.scanNetworks(
                true, true, false, 120, 0, nullptr, nullptr);
            if (launched != WIFI_SCAN_RUNNING && launched < 0) {
                setWifiEvidence(1, WiFi.status(), 0, nullptr,
                                "scan start failed", "Wi-Fi scan could not start");
                delay(150);
                continue;
            }

            bool scanFinished = false;
            while (wanted() && millis() - scanStarted < WIFI_SCAN_WINDOW_MS) {
                const int count = WiFi.scanComplete();
                if (count == WIFI_SCAN_RUNNING) {
                    delay(40);
                    continue;
                }
                scanFinished = true;
                if (count >= 0) {
                    for (int i = 0; i < count; ++i) {
                        if (WiFi.SSID(i) != ssid) continue;
                        const int candidateChannel = WiFi.channel(i);
                        const uint8_t *candidateBssid = WiFi.BSSID(i);
                        // A 5 GHz-only AP cannot be joined by an ESP32.  In
                        // practice it will not appear here, but reject
                        // impossible metadata before association as well.
                        if (!candidateBssid || candidateChannel < 1 ||
                            candidateChannel > 14) continue;
                        std::memcpy(bssid, candidateBssid, sizeof(bssid));
                        channel = static_cast<uint8_t>(candidateChannel);
                        found = true;
                        break;
                    }
                }
                WiFi.scanDelete();
                break;
            }
            if (found || !wanted()) break;
            if (!scanFinished) WiFi.scanDelete();
            setWifiEvidence(1, WiFi.status(), 0, nullptr,
                            "waiting for AP", "Wi-Fi scan: AP not visible yet");
            delay(150);
        }
        if (!wanted()) return false;
        if (!found) {
            // This is an operator-remediable radio mismatch, not a transient
            // link error. Let taskLoop hold the evidence until RETRY instead
            // of restarting BLE pairing and waking the AP over and over.
            manualWifiRetry_ = true;
            std::strncpy(lastError_, "camera AP not visible; use 2.4GHz",
                         sizeof(lastError_) - 1);
            lastError_[sizeof(lastError_) - 1] = '\0';
            setWifiEvidence(1, WiFi.status(), 0, nullptr,
                            "AP not visible", lastError_);
            return false;
        }

        // Keep the successful BLE-derived credential in this runtime's stack
        // for bounded association retries, then scrub it immediately in run().
        // Do not return to BLE/PAIR between retries.
        for (uint8_t attempt = 1; wanted() &&
             attempt <= WIFI_JOIN_ATTEMPTS; ++attempt) {
            char joining[48] = {};
            snprintf(joining, sizeof(joining), "Wi-Fi %u/%u: joining ch%u",
                     attempt, WIFI_JOIN_ATTEMPTS, channel);
            setWifiEvidence(attempt, WiFi.status(), channel, bssid,
                            "associating", joining);
            WiFi.disconnect(false, false);
            const wl_status_t beginStatus =
                WiFi.begin(ssid, password, channel, bssid, true);
            if (beginStatus == WL_CONNECT_FAILED) {
                snprintf(lastError_, sizeof(lastError_),
                         "Wi-Fi %u/%u: join request failed",
                         attempt, WIFI_JOIN_ATTEMPTS);
                setWifiEvidence(attempt, beginStatus, channel, bssid,
                                "join request failed", lastError_);
            } else {
                const uint32_t associateStarted = millis();
                while (wanted() &&
                       millis() - associateStarted < WIFI_ASSOCIATE_WINDOW_MS) {
                    const wl_status_t current = WiFi.status();
                    if (current == WL_CONNECTED) {
                        // The camera link is a low-latency local UDP session.
                        // ESP32 modem sleep can defer receive/ACK traffic long
                        // enough for the Pocket's small windows to close.
                        WiFi.setSleep(false);
                        setWifiEvidence(attempt, current, channel, bssid,
                                        "connected",
                                        "Wi-Fi connected; getting camera link");
                        return true;
                    }
                    delay(150);
                }
                if (!wanted()) return false;
                const wl_status_t current = WiFi.status();
                const char *reason = current == WL_CONNECT_FAILED
                    ? "association failed" : current == WL_NO_SSID_AVAIL
                    ? "AP disappeared" : current == WL_CONNECTION_LOST
                    ? "connection lost" : "association timed out";
                snprintf(lastError_, sizeof(lastError_), "Wi-Fi %u/%u: %s",
                         attempt, WIFI_JOIN_ATTEMPTS, reason);
                setWifiEvidence(attempt, current, channel, bssid,
                                reason, lastError_);
            }
            WiFi.disconnect(false, false);
            if (attempt < WIFI_JOIN_ATTEMPTS) {
                char retrying[48] = {};
                snprintf(retrying, sizeof(retrying),
                         "Wi-Fi %u/%u failed; retrying join",
                         attempt, WIFI_JOIN_ATTEMPTS);
                setWifiEvidence(attempt, WiFi.status(), channel, bssid,
                                "retrying association", retrying);
                delay(500);
            }
        }
        return false;
    }

    bool openDatalink() {
        setPhase(DirectPhase::TcpPoke, "opening camera control");
        tcp_.setTimeout(2);
        const bool tcpConnected = tcp_.connect(cameraIp_, CAMERA_TCP_PORT, 2500);
        if (!wanted()) return false;
        if (!tcpConnected) {
            return failIfWanted("camera TCP 7001 unavailable");
        }
        tcp_.setNoDelay(true);

        uint8_t pairPayload[1 + sizeof(APP_IDENTIFIER) - 1 +
                            1 + sizeof(PAIRING_PIN) - 1] = {};
        size_t at = 0;
        pairPayload[at++] = sizeof(APP_IDENTIFIER) - 1;
        std::memcpy(pairPayload + at, APP_IDENTIFIER,
                    sizeof(APP_IDENTIFIER) - 1);
        at += sizeof(APP_IDENTIFIER) - 1;
        pairPayload[at++] = sizeof(PAIRING_PIN) - 1;
        std::memcpy(pairPayload + at, PAIRING_PIN, sizeof(PAIRING_PIN) - 1);
        at += sizeof(PAIRING_PIN) - 1;
        uint8_t frame[128];
        const size_t frameLen = wire::encodeFrame(frame, sizeof(frame),
            SENDER_APP, RX_WIFI, 0x8092, FLAG_REQUEST, 0x07, 0x45,
            pairPayload, at);
        const size_t tcpWritten = frameLen ? tcp_.write(frame, frameLen) : 0;
        if (!wanted()) return false;
        if (!frameLen || tcpWritten != frameLen) {
            return failIfWanted("camera TCP pairing poke failed");
        }
        delay(400);

        if (!udp_.begin(0)) return failIfWanted("camera UDP socket failed");
        udpStarted_ = true;
        resetTransport();
        setPhase(DirectPhase::UdpHandshake, "negotiating camera link");

        uint8_t handshake[40] = {
            0x00,0x00,0x64,0x00,0x64,0x00,0xC0,0x05,0x14,0x00,
            0x00,0x64,0x00,0x00,0x01,0x90,0x01,0xC0,0x05,0x14,
            0x00,0x00,0x64,0x00,0x14,0x00,0x64,0x00,0xC0,0x05,
            0x14,0x00,0x00,0x64,0x00,0x01,0x01,0x04,0x01,0x02,
        };
        putLe16(handshake, baseSeq_);
        for (uint8_t attempt = 0; attempt < 8 && wanted() &&
             !handshakeSeen_; ++attempt) {
            if (!sendRaw(PKT_HANDSHAKE, handshake, sizeof(handshake))) {
                return failIfWanted("camera handshake send failed");
            }
            pumpFor(250);
        }
        if (!handshakeSeen_) return failIfWanted("camera handshake timed out");

        const uint32_t channelStarted = millis();
        while (wanted() && !channelSeen_ &&
               millis() - channelStarted < 3000) {
            pumpIncoming();
            delay(5);
        }
        // Unlike the host's bring-up probe, the standalone controller must
        // fail closed here. Guessing a command window produces a link that
        // looks healthy while silently dropping every STOP and stick packet.
        if (!channelSeen_) return failIfWanted("camera sequence window missing");
        udpSeq_ = static_cast<uint16_t>(cameraChannel_ + 8);

        if (!sendAck()) return failIfWanted("camera ACK failed");
        setPhase(DirectPhase::Registering, "registering controller");
        if (!registerController()) return failIfWanted("camera registration failed");
        pumpFor(400);

        status_.ready = true;
        status_.controlFault = false;
        setPhase(DirectPhase::Ready, "waiting for telemetry");
        return true;
    }

    void resetTransport() {
        sessionId_ = static_cast<uint16_t>(0x1000 +
            (esp_random() % (0xFFFE - 0x1000)));
        baseSeq_ = static_cast<uint16_t>((0x1000 +
            (esp_random() % (0xF000 - 0x1000))) & 0xFFF8);
        cameraChannel_ = baseSeq_;
        udpSeq_ = 0;
        dumlSeq_ = INITIAL_DUML_SEQ;
        cmdCounter_ = 0;
        videoCursor_ = 0;
        ackedDataCursor_ = 0;
        extraCursor_ = 0;
        handshakeSeen_ = false;
        channelSeen_ = false;
        lastPacketMs_ = 0;
        lastTelemetryMs_ = 0;
    }

    bool registerController() {
        uint8_t deviceInfo[62] = {};
        deviceInfo[1] = 0x41; deviceInfo[2] = 0x50; deviceInfo[3] = 0x50;
        deviceInfo[41] = 0x02;
        deviceInfo[50] = 0x02; deviceInfo[51] = 0x08;
        if (!sendDuml(RX_DM368_2, FLAG_ACK80, 0x00, 0x81,
                      deviceInfo, sizeof(deviceInfo)) || !sendAck()) return false;

        const uint8_t presence[] = {
            0x17,0x00,0x46,0x23,0x7C,0x41,0x50,0x50,
            0x00,0x00,0x00,0x00,0x00,0x02,
        };
        if (!sendDuml(RX_DM368_1, FLAG_REQUEST, 0x00, 0x88,
                      presence, sizeof(presence)) || !sendAck()) return false;

        const uint8_t gimbalInit[] = {0x05,0xFF,0xFF,0xFF,0xFF};
        if (!sendDuml(RX_GIMBAL_INIT, FLAG_REQUEST, 0x03, 0xDA,
                      gimbalInit, sizeof(gimbalInit)) || !sendAck()) return false;

        for (size_t i = 0; i < std::size(SUBSCRIPTION_KEYS); ++i) {
            const size_t nameLen = std::strlen(SUBSCRIPTION_KEYS[i]);
            uint8_t payload[64] = {0x02,0x02,0x00,0x00};
            putLe32(payload + 4, FIRST_SUB_ID + i);
            putLe16(payload + 11, static_cast<uint16_t>(nameLen + 6));
            putLe16(payload + 13, static_cast<uint16_t>(nameLen));
            std::memcpy(payload + 15, SUBSCRIPTION_KEYS[i], nameLen);
            const size_t payloadLen = 15 + nameLen + 4;
            if (!sendDuml(RX_DM368_1, FLAG_REQUEST, 0x00, 0x99,
                          payload, payloadLen)) return false;
        }
        return sendAck();
    }

    uint16_t nextDumlSeq() {
        return ++dumlSeq_;
    }

    bool sendDuml(uint8_t receiver, uint8_t flags, uint8_t cmdSet,
                  uint8_t cmdId, const uint8_t *payload, size_t payloadLen,
                  uint16_t explicitSeq = 0) {
        uint8_t frame[256];
        const uint16_t seq = explicitSeq ? explicitSeq : nextDumlSeq();
        const size_t frameLen = wire::encodeFrame(frame, sizeof(frame),
            SENDER_APP, receiver, seq, flags, cmdSet, cmdId,
            payload, payloadLen);
        if (!frameLen) return false;

        uint8_t body[12 + sizeof(frame)];
        const uint16_t ack = static_cast<uint16_t>(udpSeq_ - 8);
        putLe16(body, ack);
        putLe16(body + 2, udpSeq_);
        std::memset(body + 4, 0, 4);
        body[8] = ++cmdCounter_;
        body[9] = 0x01;
        body[10] = 0x00;
        body[11] = 0x00;
        std::memcpy(body + 12, frame, frameLen);
        return sendRaw(PKT_COMMAND, body, 12 + frameLen);
    }

    bool sendRaw(uint8_t packetType, const uint8_t *payload, size_t payloadLen) {
        if (!udpStarted_) return false;
        uint8_t header[8];
        const uint16_t total = static_cast<uint16_t>(8 + payloadLen);
        const uint16_t word = static_cast<uint16_t>(0x8000 | (total & 0x3FFF));
        putLe16(header, word);
        putLe16(header + 2, sessionId_);
        putLe16(header + 4, udpSeq_);
        header[6] = packetType;
        header[7] = 0;
        for (size_t i = 0; i < 7; ++i) header[7] ^= header[i];

        if (!udp_.beginPacket(cameraIp_, CAMERA_UDP_PORT) ||
            udp_.write(header, sizeof(header)) != sizeof(header) ||
            (payloadLen && udp_.write(payload, payloadLen) != payloadLen) ||
            !udp_.endPacket()) return false;
        udpSeq_ = static_cast<uint16_t>(udpSeq_ + 8);
        return true;
    }

    bool sendAck() {
        uint8_t payload[26] = {};
        auto group = [&](size_t at, uint16_t value) {
            putLe16(payload + at, value);
            putLe16(payload + at + 2, value);
        };
        group(0, videoCursor_);
        group(8, ackedDataCursor_ ? ackedDataCursor_ : baseSeq_);
        group(16, extraCursor_ ? extraCursor_ : baseSeq_);

        uint8_t header[8];
        const uint16_t word = 0x8000 | (8 + sizeof(payload));
        putLe16(header, word);
        putLe16(header + 2, sessionId_);
        putLe16(header + 4, 0);  // ACKs never consume the command window
        header[6] = PKT_ACK;
        header[7] = 0;
        for (size_t i = 0; i < 7; ++i) header[7] ^= header[i];
        if (!udp_.beginPacket(cameraIp_, CAMERA_UDP_PORT) ||
            udp_.write(header, sizeof(header)) != sizeof(header) ||
            udp_.write(payload, sizeof(payload)) != sizeof(payload) ||
            !udp_.endPacket()) return false;
        return true;
    }

    void pumpFor(uint32_t durationMs) {
        const uint32_t started = millis();
        while (wanted() && millis() - started < durationMs) {
            pumpIncoming();
            delay(3);
        }
    }

    void pumpIncoming() {
        int packetLen;
        uint8_t processed = 0;
        while (processed < MAX_RX_PACKETS_PER_PUMP &&
               (packetLen = udp_.parsePacket()) > 0) {
            ++processed;
            const IPAddress remoteIp = udp_.remoteIP();
            const uint16_t remotePort = udp_.remotePort();
            uint8_t packet[1536];
            const int readLen = udp_.read(packet,
                std::min(packetLen, static_cast<int>(sizeof(packet))));
            // WiFiUDP is not a connected socket. Match the host driver's
            // connected-UDP semantics explicitly so another station cannot
            // fabricate a valid-looking handshake, channel, or attitude.
            if (remoteIp == cameraIp_ && remotePort == CAMERA_UDP_PORT) {
                status_.rxPeerPackets++;
                const int boundedReadLen = std::max(0,
                    std::min(readLen, static_cast<int>(UINT16_MAX)));
                status_.lastRxLen = static_cast<uint16_t>(boundedReadLen);
                status_.lastRxType = readLen > 6 ? packet[6] : 0xFF;
                handleDatagram(packet, static_cast<size_t>(std::max(readLen, 0)));
            } else {
                status_.rxForeignPackets++;
            }
            while (udp_.available()) udp_.read();
        }
    }

    void handleDatagram(const uint8_t *data, size_t len) {
        if (!data || len < 8) {
            status_.rxHeaderRejects++;
            return;
        }
        uint8_t x = 0;
        for (size_t i = 0; i < 7; ++i) x ^= data[i];
        const size_t declared = (data[0] | (data[1] << 8)) & 0x3FFF;
        if (!(data[1] & 0x80) || data[7] != x || declared < 8 || declared > len) {
            status_.rxHeaderRejects++;
            return;
        }

        lastPacketMs_ = millis();
        const uint8_t packetType = data[6];
        if (packetType == PKT_HANDSHAKE) handshakeSeen_ = true;
        if (packetType == PKT_TELEMETRY && len >= 10) {
            status_.rxWindowPackets++;
            const uint16_t channel = data[8] | (data[9] << 8);
            if (channel) {
                cameraChannel_ = channel;
                channelSeen_ = true;
            }
        }
        // The receive ACK is three independent windows. Pocket 4 Pro stops
        // each stream once its credit is exhausted; repeating handshake
        // baseSeq in groups 1/2 only survives the first short status burst.
        if (packetType == PKT_TELEMETRY && len >= 34) {
            if (!ackedDataCursor_) {
                ackedDataCursor_ = data[18] | (data[19] << 8);
            }
            extraCursor_ = data[26] | (data[27] << 8);
        } else if (packetType == PKT_ACKED_DATA) {
            status_.rxAckedDataPackets++;
            ackedDataCursor_ = data[4] | (data[5] << 8);
        } else if (packetType == PKT_VIDEO) {
            status_.rxVideoPackets++;
            videoCursor_ = data[4] | (data[5] << 8);
        }
        status_.ackVideoCursor = videoCursor_;
        status_.ackDataCursor = ackedDataCursor_ ? ackedDataCursor_ : baseSeq_;
        status_.ackExtraCursor = extraCursor_ ? extraCursor_ : baseSeq_;

        size_t search = 0;
        while (search + 13 <= len) {
            size_t offset = 0;
            wire::FrameView frame;
            if (!wire::scanFrame(data + search, len - search, offset, frame)) break;
            status_.rxFrames++;
            status_.lastFrameCmdSet = frame.cmdSet;
            status_.lastFrameCmdId = frame.cmdId;
            status_.lastFramePayloadLen = static_cast<uint16_t>(
                std::min(frame.payloadLen, static_cast<size_t>(UINT16_MAX)));
            if (frame.cmdSet == 0x04 && frame.cmdId == 0x05) {
                status_.rxAttitudeFrames++;
                if (frame.payloadLen >= 40) {
                    status_.lastAttitudeRxLen = static_cast<uint16_t>(
                        std::min(len, static_cast<size_t>(UINT16_MAX)));
                    status_.lastAttitudeSeq = static_cast<uint16_t>(
                        data[4] | (data[5] << 8));
                    status_.lastAttitudeType = packetType;
                }
            }
            handleFrame(frame);
            // Match the proven host scanner: advance one byte past the valid
            // frame start, not past its end. A later CRC-valid frame can be
            // embedded after (or unusually within) an earlier frame.
            search += offset + 1;
        }
    }

    void handleFrame(const wire::FrameView &frame) {
        if (frame.cmdSet != 0x04 || frame.cmdId != 0x05 ||
            frame.payloadLen < 40) return;
        cameraPitch_ = readI16(frame.payload) / 10.0f;
        cameraYaw_ = readI16(frame.payload + 4) / 10.0f;
        status_.rxAcceptedAttitudes++;
        lastTelemetryMs_ = millis();
        updatePitchHeadroom(cameraPitch_);
    }

    void updatePitchHeadroom(float pitch) {
        constexpr float start = 64.5f + 3.0f;
        constexpr float usable = 158.6f - 6.0f;
        const float offset = positive360(pitch - start);
        if (offset <= usable) {
            status_.pitchHeadLow = offset;
            status_.pitchHeadHigh = usable - offset;
        } else {
            const float pastHigh = offset - usable;
            const float pastLow = 360.0f - offset;
            if (pastLow <= pastHigh) {
                status_.pitchHeadLow = 0.0f;
                status_.pitchHeadHigh = usable;
            } else {
                status_.pitchHeadLow = usable;
                status_.pitchHeadHigh = 0.0f;
            }
        }
    }

    float clampPitch(float pitch) const {
        constexpr float start = 64.5f + 3.0f;
        constexpr float usable = 158.6f - 6.0f;
        const float end = wrap180(start + usable);
        const float offset = positive360(pitch - start);
        if (offset <= usable) return pitch;
        const float toStart = positive360(start - pitch);
        const float toEnd = positive360(pitch - end);
        return toStart <= toEnd ? start : end;
    }

    bool readyLoop() {
        uint32_t lastAck = millis();
        uint32_t lastKeepalive = millis();
        uint32_t lastStick = 0;
        uint32_t lastStatus = 0;
        uint32_t seenAbort = owner_.inputs().abortEpoch;
        const uint32_t reconnectAtStart = reconnectAtStart_;
        bool requireClutchRelease = true;
        const RawJogSnapshot initialJog = snapshotRawJog();
        bool requireJogRelease = initialJog.held;
        uint32_t seenJogReleaseEpoch = initialJog.releaseEpoch;
        bool priorClutch = owner_.inputs().clutch;
        bool priorJog = false;
        bool priorLockTilt = owner_.inputs().lockTilt;
        bool priorLockPan = owner_.inputs().lockPan;
        uint32_t seenRecordEpoch = owner_.inputs().recordEpoch;
        uint32_t seenCameraActionEpoch = owner_.inputs().cameraActionEpoch;
        uint32_t seenMotionEpoch = owner_.inputs().motionEpoch;
        uint32_t noTelemetrySince = millis();

        while (wanted()) {
            const auto in = owner_.inputs();
            uint32_t now = millis();

            // Service an already-due ACK before touching a receive burst, then
            // check again using a fresh clock after the bounded batch.  This
            // gives the single embedded task the same scheduling guarantee as
            // the host's dedicated 40 Hz ACK worker.
            if (now - lastAck >= 25) {
                if (!sendAck()) {
                    return failIfWanted("camera ACK pump failed", motionActive_);
                }
                lastAck = now;
            }
            pumpIncoming();
            now = millis();

            // requestReconnect() also advances abortEpoch. Neutralise first;
            // returning first would leave the last stick packet active until
            // the runtime destructor eventually closed the socket.
            if (in.abortEpoch != seenAbort) {
                seenAbort = in.abortEpoch;
                if (!sendCentre()) {
                    noteStopUnconfirmed();
                    return fail("STOP delivery unconfirmed", true);
                }
                if (programArmed_ || programActive_)
                    stopProgram(MotionPhase::Aborted, "program aborted");
                resetMotion();
                requireClutchRelease = true;
                const RawJogSnapshot abortedJog = snapshotRawJog();
                requireJogRelease = abortedJog.held;
                seenJogReleaseEpoch = abortedJog.releaseEpoch;
                priorJog = false;
            }
            if (in.reconnectEpoch != reconnectAtStart) return false;
            if (!in.clutch) requireClutchRelease = false;
            const uint32_t releaseEpoch = rawJogReleaseEpoch.load(
                std::memory_order_acquire);
            if (releaseEpoch != seenJogReleaseEpoch) {
                seenJogReleaseEpoch = releaseEpoch;
                requireJogRelease = false;
            }

            if (now - lastAck >= 25) {
                if (!sendAck()) {
                    return failIfWanted("camera ACK pump failed", motionActive_);
                }
                lastAck = now;
            }
            if (now - lastKeepalive >= 1000) {
                lastKeepalive = now;
                const uint8_t presence[] = {
                    0x17,0x00,0x46,0x23,0x7C,0x41,0x50,0x50,
                    0x00,0x00,0x00,0x00,0x00,0x02,
                };
                if (!sendDuml(RX_DM368_1, FLAG_REQUEST, 0x00, 0x88,
                              presence, sizeof(presence))) {
                    return failIfWanted("camera keepalive failed", motionActive_);
                }
                // Keep maintenance ordering identical to the proven host:
                // every 1 Hz presence command is followed immediately by a
                // window ACK. Waiting for the next periodic ACK lets the
                // Pocket close a newly established downlink after its first
                // status burst on the embedded Wi-Fi path.
                if (!sendAck()) {
                    return failIfWanted("camera keepalive ACK failed",
                                        motionActive_);
                }
            }

            const bool telemetryFresh = lastTelemetryMs_ &&
                now - lastTelemetryMs_ <= TELEMETRY_FRESH_MS;
            const bool imuFresh = in.imuAtMs &&
                now - in.imuAtMs <= IMU_FRESH_MS;
            if (telemetryFresh) noTelemetrySince = now;
            if (!telemetryFresh && catalogCurrentPublished_) {
                CameraCatalog catalog = owner_.catalogSnapshot();
                for (uint8_t i = 0; i < catalog.count; ++i)
                    catalog.items[i].current = false;
                ++catalog.version;
                catalog.actionPending = true;
                owner_.publishCatalog(catalog);
                catalogCurrentPublished_ = false;
            }
            if (telemetryFresh && !catalogCurrentPublished_) {
                CameraCatalog catalog = owner_.catalogSnapshot();
                for (uint8_t i = 0; i < catalog.count; ++i) {
                    catalog.items[i].current = catalog.items[i].id == in.requestedDeviceId;
                }
                ++catalog.version;
                catalog.actionPending = false;
                owner_.publishCatalog(catalog);
                catalogCurrentPublished_ = true;
            }

            if (in.recordEpoch != seenRecordEpoch) {
                seenRecordEpoch = in.recordEpoch;
                if (telemetryFresh && sendRecordRequest(in.recordStart)) {
                    status_.recordIntent = in.recordStart;
                    status_.recordPending = false;
                    status_.recordUnconfirmed = true;
                    status_.recordCommandFault = false;
                } else {
                    status_.recordPending = false;
                    status_.recordCommandFault = true;
                    std::strncpy(status_.detail, "record command failed",
                                 sizeof(status_.detail) - 1);
                }
            }

            if (in.cameraActionEpoch != seenCameraActionEpoch) {
                seenCameraActionEpoch = in.cameraActionEpoch;
                status_.cameraAction = in.cameraAction;
                status_.cameraActionPending = true;
                status_.cameraActionUnconfirmed = false;
                status_.cameraActionFault = false;
                owner_.publish(status_);
                const bool sent = telemetryFresh &&
                    sendCameraActionRequest(in.cameraAction);
                status_.cameraActionPending = false;
                status_.cameraActionUnconfirmed = sent;
                status_.cameraActionFault = !sent;
                snprintf(status_.detail, sizeof(status_.detail), "%s %s",
                         DirectCamera::actionName(in.cameraAction),
                         sent ? "request sent; verify camera"
                              : "request failed");
            }

            if (in.motionEpoch != seenMotionEpoch) {
                seenMotionEpoch = in.motionEpoch;
                bool accepted = false;
                switch (in.motionCommand) {
                    case DirectCamera::MotionCommand::Arm:
                        accepted = armProgram(in, telemetryFresh);
                        break;
                    case DirectCamera::MotionCommand::Play:
                        // One deliberate PLAY, one runtime owner. Never queue a
                        // second UI command or start after a failed validation.
                        accepted = armProgram(in, telemetryFresh) &&
                                   goProgram(in, telemetryFresh, now);
                        break;
                    case DirectCamera::MotionCommand::Go:
                        accepted = goProgram(in, telemetryFresh, now);
                        break;
                    case DirectCamera::MotionCommand::GotoPoint:
                        accepted = gotoProgram(in, telemetryFresh, now);
                        break;
                    case DirectCamera::MotionCommand::None:
                    default:
                        accepted = false;
                        break;
                }
                if (!accepted && !status_.motionProgramFault) {
                    rejectProgram("motion request refused");
                }
            }

            // Manual intent always wins, but a held finger cannot flow directly
            // from programmed ownership into a live manual command. Abort,
            // neutralise, and require a real release before the next acquire.
            if ((programArmed_ || programActive_) &&
                (in.clutch || in.jogActive)) {
                if (!sendCentre()) {
                    noteStopUnconfirmed();
                    return fail("STOP delivery unconfirmed", true);
                }
                stopProgram(MotionPhase::Aborted, "manual takeover: release first");
                resetMotion();
                requireClutchRelease = in.clutch;
                const RawJogSnapshot takeoverJog = snapshotRawJog();
                requireJogRelease = takeoverJog.held;
                seenJogReleaseEpoch = takeoverJog.releaseEpoch;
                priorClutch = in.clutch;
                priorJog = in.jogActive;
            }

            if (in.clutch && !priorClutch && !requireClutchRelease) {
                if (!telemetryFresh || !imuFresh) {
                    sendCentre();
                    resetMotion();
                    requireClutchRelease = true;
                    controlFault_ = true;
                    std::strncpy(status_.detail, "no fresh data: clutch refused",
                                 sizeof(status_.detail) - 1);
                } else {
                    // AIR MOUSE owns controller intent from the physical press
                    // edge.  Camera attitude still has to be sampled after
                    // that edge before output is enabled, but a fast operator
                    // must not lose the hand motion made during that wait.
                    clutchPressQ_ = {in.qw, in.qx, in.qy, in.qz};
                    clutchPressQValid_ = in.templateIndex == HAND_TEMPLATE_AIR_MOUSE;
                    if (clutchPressQValid_) airMouseFilter_.prime();
                    else airMouseFilter_.reset();
                    // Neutralise immediately, then wait for camera telemetry
                    // sampled after the press. A merely "fresh" pre-press
                    // frame can still be 349 ms old and is not a grab pose.
                    if (!sendCentre()) return failIfWanted("clutch neutral failed", true);
                    clutchAcquirePending_ = true;
                    clutchAcquireAfterMs_ = now;
                    motionActive_ = false;
                    pitchShaper_.reset();
                    yawShaper_.reset();
                    pitchPrecision_.reset();
                    yawPrecision_.reset();
                    lastControlMs_ = now;
                    releasing_ = false;
                    controlFault_ = false;
                }
            }
            priorClutch = in.clutch;

            if (clutchAcquirePending_) {
                if (!in.clutch) {
                    clutchAcquirePending_ = false;
                    clutchPressQValid_ = false;
                    airMouseFilter_.reset();
                } else if (lastTelemetryMs_ > clutchAcquireAfterMs_ && imuFresh) {
                    clutchAcquirePending_ = false;
                    clutchActive_ = true;
                    clutchQ0_ = in.templateIndex == HAND_TEMPLATE_AIR_MOUSE &&
                                    clutchPressQValid_
                        ? clutchPressQ_ : Quat{in.qw, in.qx, in.qy, in.qz};
                    const Vec3 worldDown{0, 0, 1};
                    const Vec3 screenForward = rotateVector(clutchQ0_, {0, 0, 1});
                    const Vec3 screenRight = rotateVector(clutchQ0_, {1, 0, 0});
                    clutchGravity_ = worldDown;
                    clutchTiltAxis_ = normalized(
                        cross(worldDown, screenForward), screenRight);
                    cameraPitchRef_ = cameraPitch_;
                    pitchShaper_.reset();
                    yawShaper_.reset();
                    pitchPrecision_.reset();
                    yawPrecision_.reset();
                    if (in.templateIndex != HAND_TEMPLATE_AIR_MOUSE ||
                        !clutchPressQValid_) airMouseFilter_.reset();
                    lastControlMs_ = now;
                } else if (now - clutchAcquireAfterMs_ > TELEMETRY_FRESH_MS) {
                    clutchAcquirePending_ = false;
                    clutchPressQValid_ = false;
                    airMouseFilter_.reset();
                    requireClutchRelease = true;
                    controlFault_ = true;
                    std::strncpy(status_.detail, "clutch pose not confirmed",
                                 sizeof(status_.detail) - 1);
                }
            }

            // A lock toggle is a constraint change, not a command to revisit
            // the pose captured when the clutch was first grabbed. Rebase the
            // affected axis at the measured pose on both lock and unlock so
            // the next 40 ms tick is continuous rather than a 42 dps snap.
            if (clutchActive_ && in.lockTilt != priorLockTilt) {
                clutchQ0_ = {in.qw, in.qx, in.qy, in.qz};
                const Vec3 screenForward = rotateVector(clutchQ0_, {0, 0, 1});
                const Vec3 screenRight = rotateVector(clutchQ0_, {1, 0, 0});
                clutchTiltAxis_ = normalized(
                    cross(clutchGravity_, screenForward), screenRight);
                cameraPitchRef_ = cameraPitch_;
                pitchShaper_.reset(lastPitchDps_);
                pitchPrecision_.reset();
                if (in.templateIndex == HAND_TEMPLATE_AIR_MOUSE)
                    airMouseFilter_.rebaseAngles();
            }
            if (clutchActive_ && in.lockPan != priorLockPan) {
                yawShaper_.reset(lastYawDps_);
                yawPrecision_.reset();
            }
            priorLockTilt = in.lockTilt;
            priorLockPan = in.lockPan;

            const bool staleManual =
                (clutchActive_ && (!telemetryFresh || !imuFresh)) ||
                (in.jogActive && !telemetryFresh) ||
                (releasing_ && (!telemetryFresh || (!releaseWasJog_ && !imuFresh)));
            if (staleManual) {
                if (motionActive_ && !sendCentre()) {
                    // Do not erase motion evidence when the one packet that
                    // makes stale control safe could not be delivered.
                    noteStopUnconfirmed();
                    return fail("STOP delivery unconfirmed", true);
                }
                if (!motionActive_) sendCentre();
                resetMotion();
                requireClutchRelease = true;
                const RawJogSnapshot staleJog = snapshotRawJog();
                requireJogRelease = staleJog.held;
                seenJogReleaseEpoch = staleJog.releaseEpoch;
                controlFault_ = true;
                std::strncpy(status_.detail, "control stopped: stale data",
                             sizeof(status_.detail) - 1);
            }
            if (programActive_ && !telemetryFresh) {
                if (!faultProgram("program stopped: stale telemetry"))
                    return fail("STOP delivery unconfirmed", true);
            }

            if (now - lastStick >= STICK_INTERVAL_MS) {
                const float dt = lastControlMs_ ?
                    std::min(0.2f, (now - lastControlMs_) / 1000.0f) : 0.04f;
                lastControlMs_ = now;

                bool commandOk = true;
                if (programActive_) {
                    commandOk = commandProgram(in, now, dt);
                    releasing_ = false;
                } else if (clutchActive_ && in.clutch && telemetryFresh && imuFresh) {
                    commandOk = commandClutch(in, dt);
                    releasing_ = false;
                } else if (in.jogActive && !requireJogRelease &&
                           telemetryFresh) {
                    commandOk = commandJog(in, dt);
                    releasing_ = false;
                } else if ((clutchActive_ && !in.clutch) ||
                           (priorJog && !in.jogActive)) {
                    clutchActive_ = false;
                    airMouseFilter_.reset();
                    pitchPrecision_.reset();
                    yawPrecision_.reset();
                    clutchPressQValid_ = false;
                    releasing_ = true;
                    releaseWasJog_ = priorJog;
                    releaseAtMs_ = now;
                    releasePitch_ = lastPitchDps_;
                    releaseYaw_ = lastYawDps_;
                    releasePitchMs_ = in.manualEase[priorJog ? 1 : 0][1];
                    releaseYawMs_ = in.manualEase[priorJog ? 1 : 0][3];
                    releaseSmooth_ = priorJog ? in.jogSmoothIndex
                                              : in.smoothIndex;
                    releaseTiltStability_ = priorJog ? 1 : in.tiltStabilityIndex;
                    releasePanStability_ = priorJog ? 1 : in.panStabilityIndex;
                    commandOk = commandRelease(dt, releaseSmooth_,
                                               releaseTiltStability_,
                                               releasePanStability_);
                } else if (releasing_) {
                    commandOk = commandRelease(dt, releaseSmooth_,
                                               releaseTiltStability_,
                                               releasePanStability_);
                }
                if (!commandOk) {
                    sendCentre();
                    return failIfWanted("gimbal command send failed", true);
                }
                lastStick = now;
            }
            priorJog = in.jogActive;

            // A ready transport that never produces attitude cannot be a
            // controller. Re-negotiate instead of showing READY indefinitely.
            if (!telemetryFresh && now - noTelemetrySince > 5000) {
                sendCentre();
                const char *reason = lastTelemetryMs_ ? "camera telemetry stopped" :
                    status_.rxPeerPackets == 0 ? "no camera UDP downlink" :
                    status_.rxFrames == 0 ? "no valid camera DUML frames" :
                    status_.rxAttitudeFrames == 0 ? "camera sent no attitude telemetry" :
                    "camera attitude payload unsupported";
                return failIfWanted(reason, motionActive_);
            }
            if (lastPacketMs_ && now - lastPacketMs_ > 2000) {
                sendCentre();
                return failIfWanted("camera link timed out", motionActive_);
            }

            if (now - lastStatus >= 100) {
                lastStatus = now;
                status_.ready = true;
                status_.telemetry = telemetryFresh;
                status_.moving = motionActive_;
                status_.clutch = clutchActive_;
                status_.motionProgramArmed = programArmed_;
                status_.motionProgramActive = programActive_;
                {
                    // Store-owned fields come from the store, every tick.
                    // Every setter refuses while armed/active, so this can
                    // never disagree with the frozen copy mid-run.
                    const MotionProgram stored = owner_.motionProgram();
                    status_.motionCount = programFrozen_.count
                        ? programFrozen_.count : stored.count;
                    status_.motionRepeat = static_cast<uint8_t>(stored.repeat);
                    status_.motionSlot = stored.slot;
                    status_.motionAtStart = stored.count > 0 &&
                        atPoint(stored.points[0]);
                }
                status_.controlFault = controlFault_;
                status_.pitch = cameraPitch_;
                status_.yaw = cameraYaw_;
                status_.pitchWarnLow = limitWarning(
                    status_.pitchHeadLow, std::max(0.0f, -lastPitchDps_));
                status_.pitchWarnHigh = limitWarning(
                    status_.pitchHeadHigh, std::max(0.0f, lastPitchDps_));
                status_.telemetryAgeMs = lastTelemetryMs_ ?
                    now - lastTelemetryMs_ : UINT32_MAX;
                if (!controlFault_ && !programArmed_ && !programActive_ &&
                    status_.motionPhase != MotionPhase::Complete &&
                    status_.motionPhase != MotionPhase::Aborted &&
                    status_.motionPhase != MotionPhase::Fault) {
                    std::strncpy(status_.detail,
                        telemetryFresh ? "direct control ready" : "waiting for telemetry",
                        sizeof(status_.detail) - 1);
                }
                owner_.publish(status_);
            }
            delay(3);
        }
        if (!sendCentre()) {
            // Keep motionActive_ true so cleanup makes one final STOP attempt.
            noteStopUnconfirmed();
            return false;
        }
        if (programArmed_ || programActive_)
            stopProgram(MotionPhase::Aborted, "program stopped with link");
        resetMotion();
        return false;
    }

    void capVector(float &pitch, float &yaw, float cap) const {
        const float length = std::sqrt(pitch*pitch + yaw*yaw);
        if (length > cap && length > 0.001f) {
            const float scale = cap / length;
            pitch *= scale; yaw *= scale;
        }
    }

    float brakePitch(float rate, uint8_t smooth,
                     uint8_t stability = 1) const {
        // Account for measured camera-side coast before the endpoint, instead
        // of letting a position controller repeatedly wind into the stop.
        const float headroom = rate < 0.0f ? status_.pitchHeadLow : status_.pitchHeadHigh;
        const float safe = std::max(0.0f, headroom - 1.5f);
        static constexpr float decel[] = {330.0f, 130.0f, 55.0f};
        // QUIET genuinely has a smaller deceleration budget. RESPONSIVE may
        // stop sooner, but never spends that optimistic extra headroom here.
        const float stabilityFactor = std::min(1.0f,
            STABILITY_FACTORS[std::min<uint8_t>(stability, 2)]);
        const float a = decel[std::min<uint8_t>(smooth, 2)] * stabilityFactor;
        // Solve v^2/(2a) + v*coast <= remaining headroom.
        const float vSafe = std::max(0.0f,
            -a * CAMERA_COAST_S + std::sqrt(
                a * a * CAMERA_COAST_S * CAMERA_COAST_S + 2.0f * a * safe));
        return std::copysign(std::min(std::fabs(rate), vSafe), rate);
    }

    bool commandClutch(const DirectCamera::Inputs &in, float dt) {
        if (!std::isfinite(in.qw) || !std::isfinite(in.qx) ||
            !std::isfinite(in.qy) || !std::isfinite(in.qz) ||
            !std::isfinite(in.gx) || !std::isfinite(in.gy) ||
            !std::isfinite(in.gz) || !std::isfinite(dt)) {
            controlFault_ = true;
            std::strncpy(status_.detail, "invalid hand-control sample",
                         sizeof(status_.detail) - 1);
            return false;
        }
        const float tiltResponse =
            RESPONSE_GAINS[std::min<uint8_t>(in.tiltResponseIndex, 2)];
        const float panResponse =
            RESPONSE_GAINS[std::min<uint8_t>(in.panResponseIndex, 2)];
        const float cap = SPEED_CAPS[std::min<uint8_t>(in.speedIndex, 2)];
        const Quat current{in.qw, in.qx, in.qy, in.qz};
        const Quat relative = multiply(current, conjugate(clutchQ0_));
        const Vec3 turn = rotationVector(relative);
        const Vec3 gyro = rotateVector(current, {in.gx, in.gy, in.gz});
        const float tiltSign = HAND_TILT_BASE_SIGN *
            (in.invertTilt ? -1.0f : 1.0f);
        const float panSign = HAND_PAN_BASE_SIGN *
            (in.invertPan ? -1.0f : 1.0f);
        const float tiltRate = dot(gyro, clutchTiltAxis_) * tiltSign;
        const float panRate = dot(gyro, clutchGravity_) * panSign;
        const float tiltDelta = dot(turn, clutchTiltAxis_) * tiltSign * 57.29578f;
        const float panDelta = dot(turn, clutchGravity_) * panSign * 57.29578f;
        status_.handDeltaPitch = tiltDelta;
        status_.handDeltaPan = panDelta;
        float wantedPitch = 0.0f;
        float wantedYaw = 0.0f;

        if (in.templateIndex == HAND_TEMPLATE_FOLLOW) {
            // HAND FOLLOW is position-following only on pitch. Pan remains a
            // gravity-aligned rate path, avoiding an unobservable yaw target.
            const float rawTarget = cameraPitchRef_ + tiltDelta * tiltResponse;
            const float targetPitch = clampPitch(rawTarget);
            if (std::fabs(wrap180(targetPitch - rawTarget)) > 0.01f) {
                // Rebase at the endpoint so reversing the hand responds
                // immediately instead of paying back unreachable angle debt.
                cameraPitchRef_ += wrap180(targetPitch - rawTarget);
            }
            wantedPitch = (targetPitch - cameraPitch_) * 2.0f +
                          tiltRate * tiltResponse;
            wantedYaw = panRate * panResponse;
        } else if (in.templateIndex == HAND_TEMPLATE_AIR_MOUSE) {
            // AIR MOUSE keeps HAND FOLLOW's arbitrary-pose pitch target and
            // the drift-safe rate-only pan contract.  It conditions both
            // axes with one motion-adaptive cutoff, then blends precise pose
            // following with progressively stronger rate lead as the hand
            // accelerates.  No magnetometer means no honest absolute yaw
            // target can be invented here.
            const AirMouseSample air = airMouseFilter_.step(
                tiltDelta, panDelta, tiltRate, panRate, dt, in.smoothIndex);
            status_.handDeltaPitch = air.tilt;
            status_.handDeltaPan = air.pan;
            const float rawTarget = cameraPitchRef_ + air.tilt * tiltResponse;
            const float targetPitch = clampPitch(rawTarget);
            if (std::fabs(wrap180(targetPitch - rawTarget)) > 0.01f) {
                cameraPitchRef_ += wrap180(targetPitch - rawTarget);
            }
            const float leadMix = clampf(air.motionDps / 20.0f, 0.0f, 1.0f);
            const float lead = AIR_MOUSE_LEAD_MIN +
                (AIR_MOUSE_LEAD_MAX - AIR_MOUSE_LEAD_MIN) *
                leadMix * leadMix * (3.0f - 2.0f * leadMix);
            wantedPitch = (targetPitch - cameraPitch_) * AIR_MOUSE_P_GAIN +
                          air.tiltRate * tiltResponse * lead;
            wantedYaw = air.panRate * panResponse;
        } else if (in.templateIndex == HAND_TEMPLATE_RATE) {
            // GYRO RATE has no pose term: any controller/camera pose acquires
            // cleanly because only motion after the hold edge has an effect.
            wantedPitch = tiltRate * tiltResponse;
            wantedYaw = panRate * panResponse;
        } else {
            controlFault_ = true;
            std::strncpy(status_.detail, "invalid hand template",
                         sizeof(status_.detail) - 1);
            return false;
        }
        if (in.lockTilt) wantedPitch = 0.0f;
        if (in.lockPan) wantedYaw = 0.0f;
        // Fine is an intent/damping mode, not a request for impossible
        // sub-breakaway stick speeds. Filter and acquire each axis before the
        // jerk limiter, then let that limiter own the continuous onset and
        // release. BALANCED and DIRECT remain bit-for-bit pass-through here.
        wantedPitch = pitchPrecision_.step(
            wantedPitch, dt, in.tiltResponseIndex == 0,
            in.tiltStabilityIndex);
        wantedYaw = yawPrecision_.step(
            wantedYaw, dt, in.panResponseIndex == 0,
            in.panStabilityIndex);
        capVector(wantedPitch, wantedYaw, cap);
        wantedPitch = brakePitch(wantedPitch, in.smoothIndex,
                                  in.tiltStabilityIndex);
        float pitch = pitchShaper_.step(wantedPitch, dt, in.smoothIndex,
                                        in.tiltStabilityIndex, in.manualEase[0][0], in.manualEase[0][1], cap);
        float yaw = yawShaper_.step(wantedYaw, dt, in.smoothIndex,
                                    in.panStabilityIndex, in.manualEase[0][2], in.manualEase[0][3], cap);
        // Independent axis shapers can briefly add an old command on one axis
        // to a new command on the other during reversal. Re-cap the delivered
        // vector so the selected speed ceiling remains true at the wire.
        capVector(pitch, yaw, cap);
        status_.handOutputPct = cap > 0.01f ? static_cast<uint8_t>(std::lround(
            std::min(1.0f, std::sqrt(pitch * pitch + yaw * yaw) / cap) * 100.0f)) : 0;
        return sendRates(clampf(pitch, -MAX_DPS, MAX_DPS),
                         clampf(yaw, -MAX_DPS, MAX_DPS));
    }

    bool commandJog(const DirectCamera::Inputs &in, float dt) {
        const float cap = SPEED_CAPS[std::min<uint8_t>(in.jogSpeedIndex, 2)];
        float x = clampf(in.jogPan, -1.0f, 1.0f);
        float y = clampf(in.jogTilt, -1.0f, 1.0f);
        const float magnitude = std::sqrt(x*x + y*y);
        const float shaped = DirectCamera::jogResponse(magnitude);
        if (shaped <= 0.0f) { x = 0.0f; y = 0.0f; }
        else {
            x *= shaped/magnitude; y *= shaped/magnitude;
        }
        float wantedPitch = in.lockTilt ? 0.0f :
            -y * cap * (in.invertTilt ? -1.0f : 1.0f);
        float wantedYaw = in.lockPan ? 0.0f :
            x * cap * (in.invertPan ? -1.0f : 1.0f);
        capVector(wantedPitch, wantedYaw, cap);
        wantedPitch = brakePitch(wantedPitch, in.jogSmoothIndex);
        return sendRates(pitchShaper_.step(wantedPitch, dt, in.jogSmoothIndex, 1,
                             in.manualEase[1][0], in.manualEase[1][1], cap),
                         yawShaper_.step(wantedYaw, dt, in.jogSmoothIndex, 1,
                             in.manualEase[1][2], in.manualEase[1][3], cap));
    }

    // Within ARM tolerance of a stored point. Shared by the ARM gate and the
    // BACK TO P1 key so the two can never disagree about "at one".
    bool atPoint(const MotionPoint &point) const {
        return std::fabs(wrap180(point.pitch - cameraPitch_)) <= PROGRAM_START_PITCH_TOL &&
               std::fabs(wrap180(point.yaw - cameraYaw_)) <= PROGRAM_START_YAW_TOL;
    }

    bool pointUsesPitchArc(const MotionPoint &point) const {
        return point.captured && std::isfinite(point.pitch) &&
            std::isfinite(point.yaw) &&
            std::fabs(wrap180(clampPitch(point.pitch) - point.pitch)) <= 0.25f;
    }

    bool validateProgramLeg(float startPitch, float startYaw,
                            const MotionPoint &target,
                            const DirectCamera::Inputs &in,
                            const char *&reason, uint32_t moveMs,
                            MotionTransition transition) const {
        if (!pointUsesPitchArc(target)) {
            reason = "point outside pitch arc";
            return false;
        }
        const float pitchLeg = wrap180(target.pitch - startPitch);
        const float yawLeg = wrap180(target.yaw - startYaw);
        // Wrapped telemetry alone cannot prove which side of the physical yaw
        // arc is safe. Reject long legs and seam crossings until camera-on yaw
        // characterization supplies an unwrapped route.
        if (std::fabs(yawLeg) > PROGRAM_YAW_LEG_LIMIT ||
            (startYaw * target.yaw < 0.0f && std::fabs(startYaw) > 150.0f &&
             std::fabs(target.yaw) > 150.0f)) {
            reason = "yaw route needs qualification";
            return false;
        }
        if ((in.lockTilt && std::fabs(pitchLeg) > PROGRAM_SETTLE_PITCH_TOL) ||
            (in.lockPan && std::fabs(yawLeg) > PROGRAM_SETTLE_YAW_TOL)) {
            reason = "unlock programmed axis";
            return false;
        }
        const float seconds = moveMs / 1000.0f;
        // Peak derivative differs by curve. Validate the curve that will
        // actually fly; otherwise EASE IN/OUT can pass a smoothstep gate and
        // immediately saturate the rate cap at twice the average speed.
        const float peakFactor = transition == MotionTransition::Linear ? 1.0f
            : transition == MotionTransition::Smooth ? 1.5f : 2.0f;
        const float peakDps = peakFactor * std::sqrt(
            pitchLeg * pitchLeg + yawLeg * yawLeg) / std::max(0.001f, seconds);
        if (moveMs < 500 || peakDps > PROGRAM_MAX_DPS) {
            reason = "segment too fast; add time";
            return false;
        }
        return true;
    }

    void stopProgram(MotionPhase phase, const char *detail) {
        programArmed_ = false;
        programActive_ = false;
        programRunAll_ = false;
        programSettledSinceMs_ = 0;
        status_.motionPhase = phase;
        status_.motionProgramArmed = false;
        status_.motionProgramActive = false;
        status_.motionProgramFault = phase == MotionPhase::Fault;
        status_.cuePoint = 0;
        status_.motionLap = programLap_;
        programDir_ = 1;
        if (phase == MotionPhase::Complete) status_.motionProgress = 1000;
        if (detail) {
            std::strncpy(status_.detail, detail, sizeof(status_.detail) - 1);
            status_.detail[sizeof(status_.detail) - 1] = '\0';
        }
    }

    bool rejectProgram(const char *detail) {
        stopProgram(MotionPhase::Fault, detail);
        return false;
    }

    bool faultProgram(const char *detail) {
        if (motionActive_ && !sendCentre()) {
            noteStopUnconfirmed();
            return false;
        }
        motionActive_ = false;
        lastPitchDps_ = lastYawDps_ = 0.0f;
        pitchShaper_.reset();
        yawShaper_.reset();
        stopProgram(MotionPhase::Fault, detail);
        return true;
    }

    bool armProgram(const DirectCamera::Inputs &in, bool telemetryFresh) {
        if (!telemetryFresh) return rejectProgram("fresh telemetry required");
        const MotionProgram candidate = owner_.motionProgram();
        if (candidate.count < 2) return rejectProgram("capture at least P1 + P2");
        const char *reason = nullptr;
        for (uint8_t i = 0; i < candidate.count; ++i) {
            if (!pointUsesPitchArc(candidate.points[i]))
                return rejectProgram("point outside pitch arc");
            if (i && !validateProgramLeg(candidate.points[i - 1].pitch,
                                         candidate.points[i - 1].yaw,
                                         candidate.points[i], in, reason,
                                         candidate.points[i].moveMs,
                                         candidate.points[i].transition))
                return rejectProgram(reason);
        }
        // LOOP flies one leg the forward pass never does: last point back to
        // P1, on P1's own move time. It must pass the same gate.
        if (candidate.repeat == MotionRepeat::Loop) {
            const MotionPoint &last = candidate.points[candidate.count - 1];
            if (!validateProgramLeg(last.pitch, last.yaw, candidate.points[0],
                                    in, reason, candidate.points[0].moveMs,
                                    candidate.points[0].transition))
                return rejectProgram(reason);
        }
        const float pitchError = wrap180(candidate.points[0].pitch - cameraPitch_);
        const float yawError = wrap180(candidate.points[0].yaw - cameraYaw_);
        if (std::fabs(pitchError) > PROGRAM_START_PITCH_TOL ||
            std::fabs(yawError) > PROGRAM_START_YAW_TOL)
            return rejectProgram("go to start P1, then PLAY");
        if (!sendCentre()) return rejectProgram("arm neutral failed");
        programFrozen_ = candidate;
        programArmed_ = true;
        programActive_ = false;
        programRunAll_ = true;
        programDir_ = 1;
        programLap_ = 0;
        motionActive_ = false;
        pitchShaper_.reset();
        yawShaper_.reset();
        status_.motionPhase = MotionPhase::Armed;
        status_.motionPoint = 1;
        status_.motionCount = candidate.count;
        status_.motionProgress = 0;
        status_.motionLap = 0;
        status_.cuePoint = 0;
        status_.motionRepeat = static_cast<uint8_t>(candidate.repeat);
        status_.motionProgramArmed = true;
        status_.motionProgramActive = false;
        status_.motionProgramFault = false;
        std::strncpy(status_.detail, "validated at P1; ready to play",
                     sizeof(status_.detail) - 1);
        return true;
    }

    bool beginProgramSegment(uint8_t targetIndex, uint32_t moveMs,
                             MotionTransition transition,
                             const DirectCamera::Inputs &in,
                             uint32_t now) {
        if (targetIndex >= programFrozen_.count)
            return rejectProgram("program point unavailable");
        const MotionPoint &target = programFrozen_.points[targetIndex];
        const char *reason = nullptr;
        if (!validateProgramLeg(cameraPitch_, cameraYaw_, target, in, reason,
                                moveMs, transition))
            return rejectProgram(reason);
        programTarget_ = targetIndex;
        programSegmentMs_ = moveMs;
        programTransition_ = transition < MotionTransition::Count
            ? transition : MotionTransition::Smooth;
        programStartedAtMs_ = now;
        programSettledSinceMs_ = 0;
        programStartPitch_ = cameraPitch_;
        programStartYaw_ = cameraYaw_;
        programPitchLeg_ = wrap180(target.pitch - programStartPitch_);
        programYawLeg_ = wrap180(target.yaw - programStartYaw_);
        pitchShaper_.reset();
        yawShaper_.reset();
        lastControlMs_ = now;
        status_.motionPhase = MotionPhase::Running;
        status_.motionPoint = static_cast<uint8_t>(targetIndex + 1);
        status_.motionCount = programFrozen_.count;
        status_.motionProgress = 0;
        status_.motionProgramArmed = false;
        status_.motionProgramActive = true;
        status_.motionProgramFault = false;
        programArmed_ = false;
        programActive_ = true;
        return true;
    }

    bool goProgram(const DirectCamera::Inputs &in, bool telemetryFresh,
                   uint32_t now) {
        // Standing at a cued point, GO means "continue", not "start".
        if (programActive_ && status_.motionPhase == MotionPhase::Cued) {
            if (!telemetryFresh) return rejectProgram("fresh telemetry required");
            status_.cuePoint = 0;
            return proceedFromPoint(in, now);
        }
        if (!programArmed_ || programFrozen_.count < 2)
            return rejectProgram("ARM before GO");
        if (!telemetryFresh) return rejectProgram("fresh telemetry required");
        const float pitchError = wrap180(programFrozen_.points[0].pitch - cameraPitch_);
        const float yawError = wrap180(programFrozen_.points[0].yaw - cameraYaw_);
        if (std::fabs(pitchError) > PROGRAM_START_PITCH_TOL ||
            std::fabs(yawError) > PROGRAM_START_YAW_TOL)
            return rejectProgram("P1 moved; go to start again");
        programRunAll_ = true;
        return beginProgramSegment(1, programFrozen_.points[1].moveMs,
                                   programFrozen_.points[1].transition, in, now);
    }

    bool gotoProgram(const DirectCamera::Inputs &in, bool telemetryFresh,
                     uint32_t now) {
        if (!telemetryFresh) return rejectProgram("fresh telemetry required");
        programFrozen_ = owner_.motionProgram();
        if (in.motionPoint >= programFrozen_.count)
            return rejectProgram("program point unavailable");
        programRunAll_ = false;
        return beginProgramSegment(in.motionPoint,
                                   programFrozen_.points[in.motionPoint].moveMs,
                                   programFrozen_.points[in.motionPoint].transition,
                                   in, now);
    }

    bool completeProgramSegment(const DirectCamera::Inputs &in, uint32_t now) {
        if (!sendCentre()) return false;
        motionActive_ = false;
        lastPitchDps_ = lastYawDps_ = 0.0f;
        pitchShaper_.reset();
        yawShaper_.reset();
        const MotionPoint &target = programFrozen_.points[programTarget_];
        if (!programRunAll_) {
            stopProgram(MotionPhase::Complete, "R&D goto complete; holding");
            return true;
        }
        if (target.dwellMs) {
            programDwellAtMs_ = now;
            status_.motionPhase = MotionPhase::Dwelling;
            status_.motionProgress = 1000;
            std::snprintf(status_.detail, sizeof(status_.detail),
                          "R&D P%u dwell", programTarget_ + 1);
            return true;
        }
        return afterDwell(in, now);
    }

    // Dwell first, then the cue: a dwell is "hold this framing for N
    // seconds"; a cue is "and then wait for the line". Both may be set.
    bool afterDwell(const DirectCamera::Inputs &in, uint32_t now) {
        const MotionPoint &here = programFrozen_.points[programTarget_];
        if (here.holdForGo) {
            status_.motionPhase = MotionPhase::Cued;
            status_.cuePoint = static_cast<uint8_t>(programTarget_ + 1);
            status_.motionProgress = 1000;
            std::snprintf(status_.detail, sizeof(status_.detail),
                          "R&D P%u cued; GO to continue", programTarget_ + 1);
            return true;
        }
        return proceedFromPoint(in, now);
    }

    // The next leg, by repeat mode. The programme never auto-returns at the
    // end of a ONCE run: the camera holds where it landed.
    bool proceedFromPoint(const DirectCamera::Inputs &in, uint32_t now) {
        const uint8_t count = programFrozen_.count;
        const MotionPoint *points = programFrozen_.points;
        const uint8_t here = programTarget_;
        switch (static_cast<MotionRepeat>(status_.motionRepeat)) {
            case MotionRepeat::Loop:
                if (here + 1 < count)
                    return beginProgramSegment(here + 1, points[here + 1].moveMs,
                                               points[here + 1].transition,
                                               in, now);
                ++programLap_;
                status_.motionLap = programLap_;
                // P1's move time is otherwise unused; it times the return.
                return beginProgramSegment(0, points[0].moveMs,
                                           points[0].transition, in, now);
            case MotionRepeat::Bounce: {
                if (count < 2) break;
                if (programDir_ > 0 && here + 1 >= count) {
                    programDir_ = -1;
                    ++programLap_;
                } else if (programDir_ < 0 && here == 0) {
                    programDir_ = 1;
                    ++programLap_;
                }
                status_.motionLap = programLap_;
                const uint8_t next = static_cast<uint8_t>(here + programDir_);
                // A reverse leg keeps the time its forward twin was given.
                const MotionPoint &forwardTwin = points[here > next ? here : next];
                return beginProgramSegment(next, forwardTwin.moveMs,
                                           forwardTwin.transition, in, now);
            }
            case MotionRepeat::Once:
            default:
                if (here + 1 < count)
                    return beginProgramSegment(here + 1, points[here + 1].moveMs,
                                               points[here + 1].transition,
                                               in, now);
                break;
        }
        stopProgram(MotionPhase::Complete, "R&D program complete; holding");
        return true;
    }

    bool commandProgram(const DirectCamera::Inputs &in, uint32_t now, float dt) {
        if (status_.motionPhase == MotionPhase::Dwelling) {
            const MotionPoint &target = programFrozen_.points[programTarget_];
            if (now - programDwellAtMs_ < target.dwellMs) return true;
            return afterDwell(in, now);
        }
        // Cued: neutral was sent on arrival and the camera holds. Nothing to
        // command until a human releases it; stale telemetry still faults.
        if (status_.motionPhase == MotionPhase::Cued) return true;
        if (status_.motionPhase != MotionPhase::Running ||
            programTarget_ >= programFrozen_.count)
            return faultProgram("program state invalid");

        const MotionPoint &target = programFrozen_.points[programTarget_];
        const float duration = programSegmentMs_ / 1000.0f;
        const float elapsed = (now - programStartedAtMs_) / 1000.0f;
        const float s = clampf(elapsed / duration, 0.0f, 1.0f);
        const ProgramCurveSample curve = sampleProgramCurve(
            programTransition_, s, duration);
        const float ease = curve.position;
        const float easeRate = curve.rate;
        const float desiredPitch = wrap180(programStartPitch_ +
                                             programPitchLeg_ * ease);
        const float desiredYaw = wrap180(programStartYaw_ +
                                           programYawLeg_ * ease);
        float wantedPitch = programPitchLeg_ * easeRate +
                            2.2f * wrap180(desiredPitch - cameraPitch_);
        float wantedYaw = programYawLeg_ * easeRate +
                          2.2f * wrap180(desiredYaw - cameraYaw_);
        if (in.lockTilt) wantedPitch = 0.0f;
        if (in.lockPan) wantedYaw = 0.0f;
        capVector(wantedPitch, wantedYaw, PROGRAM_MAX_DPS);
        wantedPitch = brakePitch(wantedPitch, 1);

        const float travel2 = programPitchLeg_ * programPitchLeg_ +
                              programYawLeg_ * programYawLeg_;
        const float measuredPitch = wrap180(cameraPitch_ - programStartPitch_);
        const float measuredYaw = wrap180(cameraYaw_ - programStartYaw_);
        const float measured = travel2 > 0.01f
            ? (measuredPitch * programPitchLeg_ +
               measuredYaw * programYawLeg_) / travel2 : 1.0f;
        status_.motionProgress = static_cast<uint16_t>(std::lround(
            1000.0f * clampf(measured, 0.0f, 1.0f)));
        std::snprintf(status_.detail, sizeof(status_.detail),
                      "R&D P%u/%u %u%%", programTarget_ + 1,
                      programFrozen_.count, status_.motionProgress / 10);

        const float pitchError = wrap180(target.pitch - cameraPitch_);
        const float yawError = wrap180(target.yaw - cameraYaw_);
        const bool settled = s >= 1.0f &&
            std::fabs(pitchError) <= PROGRAM_SETTLE_PITCH_TOL &&
            std::fabs(yawError) <= PROGRAM_SETTLE_YAW_TOL;
        if (settled) {
            if (!programSettledSinceMs_) programSettledSinceMs_ = now;
            if (!sendCentre()) return false;
            motionActive_ = false;
            lastPitchDps_ = lastYawDps_ = 0.0f;
            pitchShaper_.reset();
            yawShaper_.reset();
            if (now - programSettledSinceMs_ >= PROGRAM_SETTLE_HOLD_MS)
                return completeProgramSegment(in, now);
            return true;
        }
        programSettledSinceMs_ = 0;
        if (elapsed * 1000.0f > programSegmentMs_ + PROGRAM_SETTLE_BUDGET_MS)
            return faultProgram("program off path; stopped");

        const float pitch = pitchShaper_.step(wantedPitch, dt, 1);
        const float yaw = yawShaper_.step(wantedYaw, dt, 1);
        return sendRates(clampf(pitch, -PROGRAM_MAX_DPS, PROGRAM_MAX_DPS),
                         clampf(yaw, -PROGRAM_MAX_DPS, PROGRAM_MAX_DPS));
    }

    bool commandRelease(float dt, uint8_t smooth, uint8_t tiltStability,
                        uint8_t panStability) {
        (void)dt; (void)smooth; (void)tiltStability; (void)panStability;
        const uint32_t elapsed = millis() - releaseAtMs_;
        // Snapshot the actual delivered velocity, never replay input samples.
        const float pitch = brakePitch(releaseEaseRate(releasePitch_, elapsed, releasePitchMs_),
                                       smooth, tiltStability);
        const float yaw = releaseEaseRate(releaseYaw_, elapsed, releaseYawMs_);
        pitchShaper_.reset(pitch);
        yawShaper_.reset(yaw);
        if (std::fabs(pitch) < 0.12f && std::fabs(yaw) < 0.12f) {
            if (!sendCentre()) return false;
            releasing_ = false;
            motionActive_ = false;
            lastPitchDps_ = lastYawDps_ = 0.0f;
            return true;
        }
        return sendRates(pitch, yaw, false);
    }

    bool sendRecordRequest(bool start) {
        const uint8_t payload[] = {static_cast<uint8_t>(start ? 1 : 0)};
        return sendDuml(RX_CAMERA, FLAG_REQUEST, 0x02, 0x02,
                        payload, sizeof(payload));
    }

    bool sendCameraActionRequest(CameraAction action) {
        switch (action) {
            case CameraAction::FocusSingle: {
                const uint8_t payload[] = {0x01};
                return sendDuml(RX_CAMERA, FLAG_REQUEST, 0x02, 0x24,
                                payload, sizeof(payload));
            }
            case CameraAction::FocusContinuous: {
                const uint8_t payload[] = {0x02};
                return sendDuml(RX_CAMERA, FLAG_REQUEST, 0x02, 0x24,
                                payload, sizeof(payload));
            }
            case CameraAction::GimbalRecenter: {
                const uint8_t payload[] = {0xFE, 0x08};
                return sendDuml(RX_GIMBAL, FLAG_REQUEST, 0x04, 0x4C,
                                payload, sizeof(payload));
            }
            case CameraAction::GimbalFlip: {
                const uint8_t payload[] = {0xFE, 0x09};
                return sendDuml(RX_GIMBAL, FLAG_REQUEST, 0x04, 0x4C,
                                payload, sizeof(payload));
            }
            case CameraAction::GimbalFollow: {
                const uint8_t payload[] = {0x02, 0x08};
                return sendDuml(RX_GIMBAL, FLAG_REQUEST, 0x04, 0x4C,
                                payload, sizeof(payload));
            }
            case CameraAction::GimbalFpv: {
                const uint8_t payload[] = {0x01, 0x08};
                return sendDuml(RX_GIMBAL, FLAG_REQUEST, 0x04, 0x4C,
                                payload, sizeof(payload));
            }
            case CameraAction::None:
            default:
                return false;
        }
    }

    bool sendRates(float pitchDps, float yawDps, bool remember = true) {
        // Telemetry frame -> operator/wire frame. Measured sign: a positive
        // tilt stick makes reported pitch decrease; yaw tracked with +1.
        const uint16_t tilt = stickAxis(deflectionForRate(-pitchDps));
        const uint16_t pan = stickAxis(deflectionForRate(yawDps));
        uint8_t payload[10] = {};
        putLe16(payload, tilt);
        putLe16(payload + 4, pan);
        payload[7] = 0x80;
        payload[8] = 0x22;
        const bool ok = sendDuml(RX_GIMBAL, FLAG_NOTIFY, 0x04, 0x01,
                                 payload, sizeof(payload));
        if (!ok) return false;  // preserve prior-motion evidence for fail()
        if (remember) {
            lastPitchDps_ = pitchDps;
            lastYawDps_ = yawDps;
        }
        motionActive_ = ok && (tilt != STICK_CENTER || pan != STICK_CENTER);
        return ok;
    }

    bool sendCentre() {
        uint8_t payload[10] = {};
        putLe16(payload, STICK_CENTER);
        putLe16(payload + 4, STICK_CENTER);
        payload[7] = 0x80;
        payload[8] = 0x22;
        if (udpStarted_ && channelSeen_) {
            return sendDuml(RX_GIMBAL, FLAG_NOTIFY, 0x04, 0x01,
                            payload, sizeof(payload));
        }
        return true;
    }

    void noteStopUnconfirmed() {
        controlFault_ = true;
        std::strncpy(lastError_, "STOP delivery unconfirmed",
                     sizeof(lastError_) - 1);
        lastError_[sizeof(lastError_) - 1] = '\0';
        std::strncpy(status_.detail, lastError_, sizeof(status_.detail) - 1);
        status_.detail[sizeof(status_.detail) - 1] = '\0';
    }

    void resetMotion() {
        clutchActive_ = false;
        clutchAcquirePending_ = false;
        motionActive_ = false;
        releasing_ = false;
        lastPitchDps_ = lastYawDps_ = 0.0f;
        status_.handDeltaPitch = 0.0f;
        status_.handDeltaPan = 0.0f;
        status_.handOutputPct = 0;
        pitchShaper_.reset();
        yawShaper_.reset();
        pitchPrecision_.reset();
        yawPrecision_.reset();
        airMouseFilter_.reset();
        clutchPressQValid_ = false;
    }

    void cleanup() {
        if (motionActive_) sendCentre();
        if (programArmed_ || programActive_)
            stopProgram(MotionPhase::Aborted, "program stopped with link");
        resetMotion();
        closeBle();
        if (udpStarted_) {
            udp_.stop();
            udpStarted_ = false;
        }
        if (tcp_.connected()) tcp_.stop();
        if (wifiStarted_) {
            // WIFI_STORAGE_RAM is selected before association. Erase the STA
            // profile as the radio is stopped so the camera key is not retained
            // in the ESP Wi-Fi configuration after this runtime ends.
            WiFi.disconnect(true, true);
            WiFi.mode(WIFI_OFF);
            wifiStarted_ = false;
        }
    }

    DirectCamera &owner_;
    uint32_t reconnectAtStart_ = 0;
    DirectStatus status_;
    char lastError_[48] = "direct link ended";
    bool controlFault_ = false;
    bool manualWifiRetry_ = false;

    NimBLEClient *client_ = nullptr;
    NimBLERemoteCharacteristic *notify_ = nullptr;
    NimBLERemoteCharacteristic *writer_ = nullptr;
    bool bleInitialised_ = false;
    portMUX_TYPE bleMux_ = portMUX_INITIALIZER_UNLOCKED;
    uint8_t bleRx_[1024] = {};
    size_t bleRxLen_ = 0;

    WiFiClient tcp_;
    WiFiUDP udp_;
    IPAddress cameraIp_{192, 168, 2, 1};
    bool wifiStarted_ = false;
    bool udpStarted_ = false;

    uint16_t sessionId_ = 0;
    uint16_t baseSeq_ = 0;
    uint16_t cameraChannel_ = 0;
    uint16_t udpSeq_ = 0;
    uint16_t dumlSeq_ = INITIAL_DUML_SEQ;
    uint16_t videoCursor_ = 0;
    uint16_t ackedDataCursor_ = 0;
    uint16_t extraCursor_ = 0;
    uint8_t cmdCounter_ = 0;
    bool handshakeSeen_ = false;
    bool channelSeen_ = false;
    uint32_t lastPacketMs_ = 0;
    uint32_t lastTelemetryMs_ = 0;

    float cameraPitch_ = 0.0f;
    float cameraYaw_ = 0.0f;
    float cameraPitchRef_ = 0.0f;
    Quat clutchPressQ_;
    bool clutchPressQValid_ = false;
    Quat clutchQ0_;
    Vec3 clutchGravity_{0, 0, 1};
    Vec3 clutchTiltAxis_{1, 0, 0};
    AxisShaper pitchShaper_;
    AxisShaper yawShaper_;
    PrecisionIntentGate pitchPrecision_;
    PrecisionIntentGate yawPrecision_;
    AirMouseFilter airMouseFilter_;
    MotionProgram programFrozen_;
    float lastPitchDps_ = 0.0f;
    float lastYawDps_ = 0.0f;
    float programStartPitch_ = 0.0f;
    float programStartYaw_ = 0.0f;
    float programPitchLeg_ = 0.0f;
    float programYawLeg_ = 0.0f;
    uint32_t lastControlMs_ = 0;
    uint32_t programStartedAtMs_ = 0;
    uint32_t programDwellAtMs_ = 0;
    uint32_t programSettledSinceMs_ = 0;
    // The leg being flown is timed by whichever point owns it, which for a
    // reverse or return leg is not the target. Frozen per segment.
    uint32_t programSegmentMs_ = 0;
    // Frozen with the segment so a store edit can never alter an armed run.
    MotionTransition programTransition_ = MotionTransition::Smooth;
    int8_t programDir_ = 1;
    uint16_t programLap_ = 0;
    uint8_t programTarget_ = 0;
    bool releaseWasJog_ = false;
    uint32_t releaseAtMs_ = 0;
    float releasePitch_ = 0, releaseYaw_ = 0;
    uint16_t releasePitchMs_ = 0, releaseYawMs_ = 0;
    uint8_t releaseSmooth_ = 1;
    uint8_t releaseTiltStability_ = 1;
    uint8_t releasePanStability_ = 1;
    bool clutchActive_ = false;
    bool clutchAcquirePending_ = false;
    uint32_t clutchAcquireAfterMs_ = 0;
    bool motionActive_ = false;
    bool programArmed_ = false;
    bool programActive_ = false;
    bool programRunAll_ = false;
    bool releasing_ = false;
    bool catalogCurrentPublished_ = false;
};

DirectRuntime *DirectRuntime::activeBle_ = nullptr;

void DirectCamera::begin() {
    static bool registryLoaded = false;
    if (!motionStoreReady_) {
        // NVS work must never run inside the FreeRTOS spinlock. The UI task
        // owns edits; DirectRuntime only consumes immutable snapshots.
        motionStore_.begin();
        const MotionProgram storedMotion = motionStore_.snapshot();
        portENTER_CRITICAL(&mux_);
        motionProgram_ = storedMotion;
        motionStoreReady_ = true;
        status_.motionCount = storedMotion.count;
        status_.motionSlot = storedMotion.slot;
        status_.motionRepeat = static_cast<uint8_t>(storedMotion.repeat);
        status_.motionPhase = storedMotion.count ? MotionPhase::Ready
                                                  : MotionPhase::Empty;
        portEXIT_CRITICAL(&mux_);
    }
    TrustedCamera persistedSelection;
    bool hasPersistedSelection = false;
    CameraCatalog persistedCatalog;
    uint8_t persistedAddressTypes[CAMERA_CATALOG_CAPACITY] = {};
    bool publishPersistedCatalog = false;
    if (!registryLoaded) {
        trustedCameras.begin();
        hasPersistedSelection = trustedCameras.selected(persistedSelection);
        persistedCatalog.version = 1;
        persistedCatalog.actionPending = hasPersistedSelection;
        for (uint8_t i = 0; i < TrustedCameras::kMax; ++i) {
            TrustedCamera saved;
            if (!trustedCameras.get(i, saved)) continue;
            const uint8_t slot = persistedCatalog.count++;
            CameraSummary &item = persistedCatalog.items[slot];
            item.id = cameraId(saved.address, saved.addressType);
            item.model = saved.model;
            item.trusted = true;
            item.selected = hasPersistedSelection &&
                saved.addressType == persistedSelection.addressType &&
                std::strncmp(saved.address, persistedSelection.address,
                             sizeof(saved.address)) == 0;
            std::strncpy(item.name, saved.name, sizeof(item.name) - 1);
            std::strncpy(item.address, saved.address, sizeof(item.address) - 1);
            persistedAddressTypes[slot] = saved.addressType;
        }
        publishPersistedCatalog = true;
        registryLoaded = true;
    }
    portENTER_CRITICAL(&mux_);
    if (publishPersistedCatalog) {
        catalog_ = persistedCatalog;
        std::memcpy(catalogAddressTypes_, persistedAddressTypes,
                    sizeof(catalogAddressTypes_));
    }
    if (hasPersistedSelection) {
            std::strncpy(inputs_.requestedAddress, persistedSelection.address,
                         sizeof(inputs_.requestedAddress) - 1);
            inputs_.requestedAddressType = persistedSelection.addressType;
            inputs_.requestedDeviceId = cameraId(persistedSelection.address, persistedSelection.addressType);
    } else if (!inputs_.requestedAddress[0]) {
        inputs_.deviceMode = DeviceMode::Idle;  // wait for explicit SCAN
    }
    inputs_.wantEnabled = true;
    const bool needsTask = task_ == nullptr;
    portEXIT_CRITICAL(&mux_);
    if (needsTask) {
        TaskHandle_t created = nullptr;
        const BaseType_t result = xTaskCreatePinnedToCore(
            taskEntry, "osmo-direct", 16384, this, 1, &created, 0);
        if (result == pdPASS) {
            portENTER_CRITICAL(&mux_);
            task_ = created;
            portEXIT_CRITICAL(&mux_);
        } else {
            // Keep the requested mode visible so the reconnect button can
            // retry task creation, but never leave an empty OFF screen that
            // suggests the connection attempt is merely slow.
            DirectStatus failed;
            failed.enabled = true;
            failed.controlFault = true;
            failed.phase = DirectPhase::RetryWait;
            std::strncpy(failed.detail, "direct task allocation failed",
                         sizeof(failed.detail) - 1);
            publish(failed);
        }
    }
}

void DirectCamera::requestDeviceScan() {
    portENTER_CRITICAL(&mux_);
    if (catalog_.forgetPending) {
        portEXIT_CRITICAL(&mux_);
        return;
    }
    ++inputs_.deviceScanEpoch;
    inputs_.deviceMode = DeviceMode::ScanOnly;
    ++inputs_.reconnectEpoch;
    ++inputs_.abortEpoch;
    inputs_.clutch = false;
    inputs_.jogActive = false;
    catalog_.scanning = true;
    catalog_.actionPending = true;
    catalog_.forgetPending = false;
    for (uint8_t i = 0; i < catalog_.count; ++i) catalog_.items[i].current = false;
    catalog_.error[0] = '\0';
    ++catalog_.version;
    portEXIT_CRITICAL(&mux_);
    if (task_) xTaskNotifyGive(task_);
}

void DirectCamera::cancelDeviceAction() {
    portENTER_CRITICAL(&mux_);
    if (catalog_.forgetPending) {
        // The explicit hold-to-forget is the commit point. Pretending it can
        // be cancelled after that point races the NVS mutation and lies to the
        // operator about whether the identity will survive a reboot.
        portEXIT_CRITICAL(&mux_);
        return;
    }
    ++inputs_.deviceCancelEpoch;
    inputs_.deviceMode = DeviceMode::Idle;
    ++inputs_.abortEpoch;
    ++inputs_.reconnectEpoch;
    inputs_.clutch = false;
    inputs_.jogActive = false;
    catalog_.scanning = false;
    catalog_.actionPending = false;
    catalog_.forgetPending = false;
    catalog_.error[0] = '\0';
    ++catalog_.version;
    portEXIT_CRITICAL(&mux_);
    if (task_) xTaskNotifyGive(task_);
}

void DirectCamera::selectDevice(uint32_t id) {
    portENTER_CRITICAL(&mux_);
    if (catalog_.forgetPending) {
        portEXIT_CRITICAL(&mux_);
        return;
    }
    for (uint8_t i = 0; i < catalog_.count; ++i) {
        if (catalog_.items[i].id != id || !catalog_.items[i].address[0]) continue;
        inputs_.requestedDeviceId = id;
        inputs_.deviceMode = DeviceMode::ConnectSelected;
        std::strncpy(inputs_.requestedAddress, catalog_.items[i].address,
                     sizeof(inputs_.requestedAddress) - 1);
        inputs_.requestedAddressType = catalogAddressTypes_[i];
        ++inputs_.deviceSelectEpoch;
        ++inputs_.reconnectEpoch;
        ++inputs_.abortEpoch;
        inputs_.clutch = false;
        inputs_.jogActive = false;
        catalog_.actionPending = true;
        catalog_.scanning = false;
        catalog_.forgetPending = false;
        for (uint8_t j = 0; j < catalog_.count; ++j) catalog_.items[j].current = false;
        catalog_.error[0] = '\0';
        for (uint8_t j = 0; j < catalog_.count; ++j)
            catalog_.items[j].selected = catalog_.items[j].id == id;
        ++catalog_.version;
        break;
    }
    portEXIT_CRITICAL(&mux_);
    if (task_) xTaskNotifyGive(task_);
}

void DirectCamera::forgetDevice(uint32_t id) {
    portENTER_CRITICAL(&mux_);
    if (catalog_.forgetPending) {
        portEXIT_CRITICAL(&mux_);
        return;
    }
    for (uint8_t i = 0; i < catalog_.count; ++i) {
        if (catalog_.items[i].id != id || !catalog_.items[i].address[0]) continue;
        inputs_.deviceMode = DeviceMode::Idle;
        inputs_.forgetDeviceId = id;
        std::strncpy(inputs_.forgetAddress, catalog_.items[i].address,
                     sizeof(inputs_.forgetAddress) - 1);
        inputs_.forgetAddress[sizeof(inputs_.forgetAddress) - 1] = '\0';
        inputs_.forgetAddressType = catalogAddressTypes_[i];
        ++inputs_.deviceForgetEpoch;
        ++inputs_.reconnectEpoch;
        ++inputs_.abortEpoch;
        inputs_.clutch = false;
        inputs_.jogActive = false;
        catalog_.actionPending = true;
        catalog_.scanning = false;
        catalog_.forgetPending = true;
        for (uint8_t j = 0; j < catalog_.count; ++j) catalog_.items[j].current = false;
        catalog_.error[0] = '\0';
        ++catalog_.version;
        break;
    }
    portEXIT_CRITICAL(&mux_);
    if (task_) xTaskNotifyGive(task_);
}

void DirectCamera::disable() {
    portENTER_CRITICAL(&mux_);
    inputs_.wantEnabled = false;
    ++inputs_.abortEpoch;
    inputs_.clutch = false;
    inputs_.jogActive = false;
    portEXIT_CRITICAL(&mux_);
    if (task_) xTaskNotifyGive(task_);
}

void DirectCamera::requestReconnect() {
    portENTER_CRITICAL(&mux_);
    if (catalog_.forgetPending) {
        portEXIT_CRITICAL(&mux_);
        return;
    }
    if (inputs_.requestedAddress[0] && inputs_.deviceMode == DeviceMode::Idle)
        inputs_.deviceMode = DeviceMode::ConnectSelected;
    ++inputs_.reconnectEpoch;
    ++inputs_.abortEpoch;
    inputs_.clutch = false;
    inputs_.jogActive = false;
    portEXIT_CRITICAL(&mux_);
    if (task_) xTaskNotifyGive(task_);
}

void DirectCamera::abort() {
    portENTER_CRITICAL(&mux_);
    ++inputs_.abortEpoch;
    inputs_.clutch = false;
    inputs_.jogActive = false;
    portEXIT_CRITICAL(&mux_);
    if (task_) xTaskNotifyGive(task_);
}

void DirectCamera::setImu(float ax, float ay, float az, float gx, float gy,
                          float gz, uint32_t sampleAtMs) {
    portENTER_CRITICAL(&mux_);
    float dt = imu_.lastAtMs ? (sampleAtMs - imu_.lastAtMs) / 1000.0f : 0.01f;
    dt = std::max(0.004f, std::min(0.03f, dt));
    imu_.lastAtMs = sampleAtMs;
    const Vec3 accel{ax, ay, az};
    const float accelLength = norm(accel);
    const float accelDelta = norm({ax - imu_.lastAx, ay - imu_.lastAy,
                                   az - imu_.lastAz});
    const float gyroLength = std::sqrt(gx*gx + gy*gy + gz*gz);
    const bool inputBusy = inputs_.clutch || inputs_.jogActive ||
        sampleAtMs < imu_.freezeUntilMs;
    const bool stationary = !inputBusy &&
        std::fabs(accelLength - 1.0f) < 0.12f &&
        accelDelta < 0.035f && gyroLength < 1.2f;
    if (stationary) {
        if (!imu_.stationarySinceMs) imu_.stationarySinceMs = sampleAtMs;
        if (sampleAtMs - imu_.stationarySinceMs >= 900) {
            constexpr float alpha = 0.01f;
            imu_.bx += alpha * (gx - imu_.bx);
            imu_.by += alpha * (gy - imu_.by);
            imu_.bz += alpha * (gz - imu_.bz);
        }
    } else {
        imu_.stationarySinceMs = 0;
    }

    Quat q{imu_.qw, imu_.qx, imu_.qy, imu_.qz};
    Vec3 gyro{(gx-imu_.bx)*0.0174532925f, (gy-imu_.by)*0.0174532925f,
              (gz-imu_.bz)*0.0174532925f};
    if (std::fabs(accelLength - 1.0f) < 0.35f) {
        const Vec3 measured = normalized(accel);
        const Vec3 estimated = normalized(gravityBody(q));
        // Mahony-style innovation is measured x estimated. Reversing these
        // operands creates positive feedback: a small roll/pitch estimate
        // error is driven farther from gravity instead of being damped.
        const Vec3 error = cross(measured, estimated);
        // A small proportional gravity correction stabilises roll/pitch. It
        // intentionally cannot invent an absolute yaw without a magnetometer.
        constexpr float kp = 1.7f;
        gyro.x += kp * error.x; gyro.y += kp * error.y; gyro.z += kp * error.z;
    }
    const Quat omega{0, gyro.x, gyro.y, gyro.z};
    const Quat derivative = multiply(q, omega);
    q.w += 0.5f * derivative.w * dt;
    q.x += 0.5f * derivative.x * dt;
    q.y += 0.5f * derivative.y * dt;
    q.z += 0.5f * derivative.z * dt;
    q = unit(q);
    imu_.qw=q.w; imu_.qx=q.x; imu_.qy=q.y; imu_.qz=q.z;
    imu_.lastAx=ax; imu_.lastAy=ay; imu_.lastAz=az;
    inputs_.ax = ax; inputs_.ay = ay; inputs_.az = az;
    inputs_.gx = gx - imu_.bx; inputs_.gy = gy - imu_.by;
    inputs_.gz = gz - imu_.bz;
    inputs_.qw=q.w; inputs_.qx=q.x; inputs_.qy=q.y; inputs_.qz=q.z;
    inputs_.imuAtMs = sampleAtMs;
    portEXIT_CRITICAL(&mux_);
}

void DirectCamera::setClutch(bool held) {
    portENTER_CRITICAL(&mux_);
    inputs_.clutch = held;
    if (!held) imu_.freezeUntilMs = millis() + 800;
    portEXIT_CRITICAL(&mux_);
}

void DirectCamera::setJog(float tilt, float pan, bool active) {
    portENTER_CRITICAL(&mux_);
    inputs_.jogTilt = clampf(tilt, -1.0f, 1.0f);
    inputs_.jogPan = clampf(pan, -1.0f, 1.0f);
    inputs_.jogActive = active;
    if (!active) imu_.freezeUntilMs = millis() + 800;
    portEXIT_CRITICAL(&mux_);

    if (active) {
        rawJogHeld.store(true, std::memory_order_release);
    } else if (rawJogHeld.exchange(false, std::memory_order_acq_rel)) {
        rawJogReleaseEpoch.fetch_add(1, std::memory_order_acq_rel);
    }
}

void DirectCamera::setControlConfig(uint8_t tiltResponseIndex,
                                    uint8_t panResponseIndex,
                                    uint8_t tiltStabilityIndex,
                                    uint8_t panStabilityIndex,
                                    uint8_t speedIndex, uint8_t smoothIndex,
                                    uint8_t templateIndex, bool lockTilt,
                                    bool lockPan, bool invertTilt,
                                    bool invertPan) {
    portENTER_CRITICAL(&mux_);
    inputs_.tiltResponseIndex = std::min<uint8_t>(tiltResponseIndex, 2);
    inputs_.panResponseIndex = std::min<uint8_t>(panResponseIndex, 2);
    inputs_.tiltStabilityIndex = std::min<uint8_t>(tiltStabilityIndex, 2);
    inputs_.panStabilityIndex = std::min<uint8_t>(panStabilityIndex, 2);
    inputs_.speedIndex = std::min<uint8_t>(speedIndex, 2);
    inputs_.smoothIndex = std::min<uint8_t>(smoothIndex, 2);
    inputs_.templateIndex = std::min<uint8_t>(templateIndex,
                                              HAND_TEMPLATE_AIR_MOUSE);
    inputs_.lockTilt = lockTilt;
    inputs_.lockPan = lockPan;
    inputs_.invertTilt = invertTilt;
    inputs_.invertPan = invertPan;
    portEXIT_CRITICAL(&mux_);
}

void DirectCamera::setManualEase(const uint16_t (&milliseconds)[2][4]) {
    portENTER_CRITICAL(&mux_);
    for (uint8_t mode = 0; mode < 2; ++mode)
        for (uint8_t value = 0; value < 4; ++value)
            inputs_.manualEase[mode][value] = std::min<uint16_t>(milliseconds[mode][value], 750);
    portEXIT_CRITICAL(&mux_);
}

void DirectCamera::setJogConfig(uint8_t speedIndex, uint8_t smoothIndex) {
    portENTER_CRITICAL(&mux_);
    inputs_.jogSpeedIndex = std::min<uint8_t>(speedIndex, 2);
    inputs_.jogSmoothIndex = std::min<uint8_t>(smoothIndex, 2);
    portEXIT_CRITICAL(&mux_);
}

void DirectCamera::requestRecord(bool start) {
    portENTER_CRITICAL(&mux_);
    inputs_.recordStart = start;
    ++inputs_.recordEpoch;
    status_.recordIntent = start;
    status_.recordPending = true;
    status_.recordCommandFault = false;
    portEXIT_CRITICAL(&mux_);
}

void DirectCamera::requestCameraAction(CameraAction action) {
    if (action == CameraAction::None) return;
    portENTER_CRITICAL(&mux_);
    inputs_.cameraAction = action;
    ++inputs_.cameraActionEpoch;
    // Gimbal actions can move the head. They first take the same immediate
    // neutralisation path as STOP/manual takeover; focus-mode writes do not.
    if (action == CameraAction::GimbalRecenter ||
        action == CameraAction::GimbalFlip ||
        action == CameraAction::GimbalFollow ||
        action == CameraAction::GimbalFpv) {
        ++inputs_.abortEpoch;
        inputs_.clutch = false;
        inputs_.jogActive = false;
    }
    portEXIT_CRITICAL(&mux_);
    if (task_) xTaskNotifyGive(task_);
}

MotionProgram DirectCamera::motionProgram() const {
    portENTER_CRITICAL(&mux_);
    const MotionProgram copy = motionProgram_;
    portEXIT_CRITICAL(&mux_);
    return copy;
}

bool DirectCamera::captureMotionPoint(uint8_t index) {
    const DirectStatus evidence = status();
    if (!motionStoreReady_ || !evidence.ready || !evidence.telemetry ||
        evidence.telemetryAgeMs > TELEMETRY_FRESH_MS ||
        evidence.motionProgramArmed || evidence.motionProgramActive) {
        return false;
    }
    const bool saved = motionStore_.capture(index, evidence.pitch, evidence.yaw);
    const MotionProgram updated = motionStore_.snapshot();
    portENTER_CRITICAL(&mux_);
    motionProgram_ = updated;
    status_.motionCount = updated.count;
    status_.motionRepeat = static_cast<uint8_t>(updated.repeat);
    status_.motionPoint = saved ? static_cast<uint8_t>(index + 1) : 0;
    status_.motionPhase = updated.count ? MotionPhase::Ready : MotionPhase::Empty;
    // The store's own error travels in the MotionProgram snapshot the UI
    // reads directly. Parking it in status_ was useless: the runtime
    // republishes its private copy every 100 ms and the UI polls at the
    // same cadence, so the flag was clobbered before it was ever seen.
    portEXIT_CRITICAL(&mux_);
    return saved;
}

bool DirectCamera::clearMotionPoint(uint8_t index) {
    const DirectStatus evidence = status();
    if (!motionStoreReady_ || evidence.motionProgramArmed ||
        evidence.motionProgramActive) return false;
    const bool saved = motionStore_.clear(index);
    const MotionProgram updated = motionStore_.snapshot();
    portENTER_CRITICAL(&mux_);
    motionProgram_ = updated;
    status_.motionCount = updated.count;
    status_.motionPoint = 0;
    status_.motionPhase = updated.count ? MotionPhase::Ready : MotionPhase::Empty;
    // The store's own error travels in the MotionProgram snapshot the UI
    // reads directly. Parking it in status_ was useless: the runtime
    // republishes its private copy every 100 ms and the UI polls at the
    // same cadence, so the flag was clobbered before it was ever seen.
    portEXIT_CRITICAL(&mux_);
    return saved;
}

bool DirectCamera::removeMotionPoint(uint8_t index) {
    const DirectStatus evidence = status();
    if (!motionStoreReady_ || evidence.motionProgramArmed ||
        evidence.motionProgramActive) return false;
    const bool saved = motionStore_.remove(index);
    const MotionProgram updated = motionStore_.snapshot();
    portENTER_CRITICAL(&mux_);
    motionProgram_ = updated;
    status_.motionCount = updated.count;
    status_.motionPoint = 0;
    status_.motionPhase = updated.count ? MotionPhase::Ready : MotionPhase::Empty;
    // The store's own error travels in the MotionProgram snapshot the UI
    // reads directly. Parking it in status_ was useless: the runtime
    // republishes its private copy every 100 ms and the UI polls at the
    // same cadence, so the flag was clobbered before it was ever seen.
    portEXIT_CRITICAL(&mux_);
    return saved;
}

bool DirectCamera::setMotionTiming(uint8_t index, uint32_t moveMs,
                                   uint32_t dwellMs) {
    const DirectStatus evidence = status();
    if (!motionStoreReady_ || evidence.motionProgramArmed ||
        evidence.motionProgramActive) return false;
    const bool saved = motionStore_.setTiming(index, moveMs, dwellMs);
    const MotionProgram updated = motionStore_.snapshot();
    portENTER_CRITICAL(&mux_);
    motionProgram_ = updated;
    status_.motionCount = updated.count;
    // The store's own error travels in the MotionProgram snapshot the UI
    // reads directly. Parking it in status_ was useless: the runtime
    // republishes its private copy every 100 ms and the UI polls at the
    // same cadence, so the flag was clobbered before it was ever seen.
    portEXIT_CRITICAL(&mux_);
    return saved;
}

bool DirectCamera::setMotionHold(uint8_t index, bool holdForGo) {
    const DirectStatus evidence = status();
    if (!motionStoreReady_ || evidence.motionProgramArmed ||
        evidence.motionProgramActive) return false;
    const bool saved = motionStore_.setHold(index, holdForGo);
    const MotionProgram updated = motionStore_.snapshot();
    portENTER_CRITICAL(&mux_);
    motionProgram_ = updated;
    // The store's own error travels in the MotionProgram snapshot the UI
    // reads directly. Parking it in status_ was useless: the runtime
    // republishes its private copy every 100 ms and the UI polls at the
    // same cadence, so the flag was clobbered before it was ever seen.
    portEXIT_CRITICAL(&mux_);
    return saved;
}

bool DirectCamera::setMotionTransition(uint8_t index,
                                       MotionTransition transition) {
    const DirectStatus evidence = status();
    if (!motionStoreReady_ || transition >= MotionTransition::Count ||
        evidence.motionProgramArmed || evidence.motionProgramActive) return false;
    const bool saved = motionStore_.setTransition(index, transition);
    const MotionProgram updated = motionStore_.snapshot();
    portENTER_CRITICAL(&mux_);
    motionProgram_ = updated;
    portEXIT_CRITICAL(&mux_);
    return saved;
}

bool DirectCamera::setMotionRepeat(uint8_t repeat) {
    const DirectStatus evidence = status();
    if (!motionStoreReady_ || evidence.motionProgramArmed ||
        evidence.motionProgramActive) return false;
    const bool saved = motionStore_.setRepeat(static_cast<MotionRepeat>(repeat));
    const MotionProgram updated = motionStore_.snapshot();
    portENTER_CRITICAL(&mux_);
    motionProgram_ = updated;
    status_.motionRepeat = static_cast<uint8_t>(updated.repeat);
    // The store's own error travels in the MotionProgram snapshot the UI
    // reads directly. Parking it in status_ was useless: the runtime
    // republishes its private copy every 100 ms and the UI polls at the
    // same cadence, so the flag was clobbered before it was ever seen.
    portEXIT_CRITICAL(&mux_);
    return saved;
}

bool DirectCamera::selectMotionSlot(uint8_t slot) {
    const DirectStatus evidence = status();
    if (!motionStoreReady_ || evidence.motionProgramArmed ||
        evidence.motionProgramActive) return false;
    const bool loaded = motionStore_.selectSlot(slot);
    const MotionProgram updated = motionStore_.snapshot();
    portENTER_CRITICAL(&mux_);
    motionProgram_ = updated;
    status_.motionCount = updated.count;
    status_.motionSlot = updated.slot;
    status_.motionRepeat = static_cast<uint8_t>(updated.repeat);
    status_.motionPoint = 0;
    status_.motionPhase = updated.count ? MotionPhase::Ready : MotionPhase::Empty;
    portEXIT_CRITICAL(&mux_);
    return loaded;
}

bool DirectCamera::requestMotionGoto(uint8_t index) {
    const DirectStatus evidence = status();
    const MotionProgram program = motionProgram();
    if (!evidence.ready || !evidence.telemetry ||
        evidence.telemetryAgeMs > TELEMETRY_FRESH_MS || index >= program.count)
        return false;
    portENTER_CRITICAL(&mux_);
    ++inputs_.abortEpoch;
    inputs_.clutch = false;
    inputs_.jogActive = false;
    inputs_.motionCommand = MotionCommand::GotoPoint;
    inputs_.motionPoint = index;
    ++inputs_.motionEpoch;
    status_.motionPhase = MotionPhase::Positioning;
    status_.motionPoint = static_cast<uint8_t>(index + 1);
    status_.motionProgramActive = true;  // accepted request, not proof of motion
    status_.motionProgramArmed = false;
    status_.motionProgramFault = false;
    portEXIT_CRITICAL(&mux_);
    if (task_) xTaskNotifyGive(task_);
    return true;
}

bool DirectCamera::requestMotionArm() {
    const DirectStatus evidence = status();
    const MotionProgram program = motionProgram();
    if (!evidence.ready || !evidence.telemetry ||
        evidence.telemetryAgeMs > TELEMETRY_FRESH_MS || program.count < 2)
        return false;
    portENTER_CRITICAL(&mux_);
    ++inputs_.abortEpoch;
    inputs_.clutch = false;
    inputs_.jogActive = false;
    inputs_.motionCommand = MotionCommand::Arm;
    ++inputs_.motionEpoch;
    status_.motionPhase = MotionPhase::Armed;
    status_.motionProgramArmed = true;  // pending runtime validation
    status_.motionProgramActive = false;
    status_.motionProgramFault = false;
    portEXIT_CRITICAL(&mux_);
    if (task_) xTaskNotifyGive(task_);
    return true;
}

bool DirectCamera::requestMotionPlay() {
    const DirectStatus evidence = status();
    const MotionProgram program = motionProgram();
    if (!evidence.ready || !evidence.telemetry ||
        evidence.telemetryAgeMs > TELEMETRY_FRESH_MS || program.count < 2 ||
        evidence.motionProgramActive || !evidence.motionAtStart)
        return false;
    portENTER_CRITICAL(&mux_);
    ++inputs_.abortEpoch;
    inputs_.clutch = false;
    inputs_.jogActive = false;
    inputs_.motionCommand = MotionCommand::Play;
    ++inputs_.motionEpoch;
    status_.motionPhase = MotionPhase::Armed;  // preparing, not proof of motion
    status_.motionProgramArmed = true;
    status_.motionProgramActive = false;
    status_.motionProgramFault = false;
    portEXIT_CRITICAL(&mux_);
    if (task_) xTaskNotifyGive(task_);
    return true;
}

bool DirectCamera::requestMotionGo() {
    const DirectStatus evidence = status();
    // Two meanings, one deliberate key: start an armed programme, or release
    // a programme that is holding at a cued point.
    const bool cued = evidence.cuePoint != 0 && evidence.motionProgramActive;
    if (!evidence.ready || !evidence.telemetry ||
        evidence.telemetryAgeMs > TELEMETRY_FRESH_MS ||
        (!cued && (!evidence.motionProgramArmed || evidence.motionProgramActive)))
        return false;
    portENTER_CRITICAL(&mux_);
    inputs_.motionCommand = MotionCommand::Go;
    ++inputs_.motionEpoch;
    status_.cuePoint = 0;
    status_.motionPhase = MotionPhase::Running;
    status_.motionProgramArmed = false;
    status_.motionProgramActive = true;  // pending first bounded rate packet
    status_.motionProgramFault = false;
    portEXIT_CRITICAL(&mux_);
    if (task_) xTaskNotifyGive(task_);
    return true;
}

bool DirectCamera::requestMotionRun() {
    // Legacy callers get the safe first half of the two-step contract. A
    // separate deliberate GO is always required before any motion packet.
    return requestMotionArm();
}

DirectStatus DirectCamera::status() const {
    portENTER_CRITICAL(&mux_);
    DirectStatus copy = status_;
    const bool enabled = inputs_.wantEnabled;
    portEXIT_CRITICAL(&mux_);
    // disable() is an input published to the networking task.  That task may
    // need a scheduler turn to close sockets and publish DirectPhase::Off, but
    // callers must never observe the old READY/motion evidence in that gap.
    // Preserve stored-program metadata while masking every live-session fact.
    if (!enabled) {
        copy.enabled = false;
        copy.ready = false;
        copy.telemetry = false;
        copy.moving = false;
        copy.clutch = false;
        copy.handDeltaPitch = 0.0f;
        copy.handDeltaPan = 0.0f;
        copy.handOutputPct = 0;
        copy.controlFault = false;
        copy.recordIntent = false;
        copy.recordPending = false;
        copy.recordUnconfirmed = false;
        copy.recordCommandFault = false;
        copy.cameraAction = CameraAction::None;
        copy.cameraActionPending = false;
        copy.cameraActionUnconfirmed = false;
        copy.cameraActionFault = false;
        copy.motionProgramActive = false;
        copy.motionProgramArmed = false;
        copy.cuePoint = 0;
        copy.motionProgress = 0;
        copy.motionAtStart = false;
        if (!copy.motionProgramFault)
            copy.motionPhase = copy.motionCount ? MotionPhase::Ready
                                                : MotionPhase::Empty;
        copy.phase = DirectPhase::Off;
        copy.blockedAt = DirectPhase::Off;
        copy.pitchHeadLow = 0.0f;
        copy.pitchHeadHigh = 0.0f;
        copy.pitchWarnLow = 0;
        copy.pitchWarnHigh = 0;
        copy.wifiAttempt = 0;
        copy.wifiStatus = 0;
        copy.wifiChannel = 0;
        std::memset(copy.wifiBssid, 0, sizeof(copy.wifiBssid));
        copy.retryAtMs = 0;
        copy.telemetryAgeMs = UINT32_MAX;
        copy.ssid[0] = '\0';
        copy.wifiReason[0] = '\0';
        std::strncpy(copy.detail, "direct camera off",
                     sizeof(copy.detail) - 1);
        copy.detail[sizeof(copy.detail) - 1] = '\0';
    }
    return copy;
}

CameraCatalog DirectCamera::catalog() const { return catalogSnapshot(); }

bool DirectCamera::enabled() const {
    portENTER_CRITICAL(&mux_);
    const bool value = inputs_.wantEnabled;
    portEXIT_CRITICAL(&mux_);
    return value;
}

HostImu DirectCamera::hostImu() const {
    Quat q;
    portENTER_CRITICAL(&mux_);
    q = {inputs_.qw, inputs_.qx, inputs_.qy, inputs_.qz};
    portEXIT_CRITICAL(&mux_);
    // World-down in the sensor frame: at rest this is the normalised
    // accelerometer, and the angles below are exactly the old firmware's.
    const Vec3 g = normalized(gravityBody(q));
    HostImu out;
    out.pitch = atan2f(-g.x, sqrtf(g.y * g.y + g.z * g.z)) * 57.2957795f;
    out.roll = atan2f(g.y, g.z) * 57.2957795f;
    return out;
}

DirectCamera::Inputs DirectCamera::inputs() const {
    portENTER_CRITICAL(&mux_);
    const Inputs copy = inputs_;
    portEXIT_CRITICAL(&mux_);
    return copy;
}

void DirectCamera::publish(const DirectStatus &next) {
    portENTER_CRITICAL(&mux_);
    status_ = next;
    portEXIT_CRITICAL(&mux_);
}

void DirectCamera::publishCatalog(const CameraCatalog &next) {
    portENTER_CRITICAL(&mux_);
    catalog_ = next;
    portEXIT_CRITICAL(&mux_);
}

void DirectCamera::publishCatalogWithAddressTypes(
    const CameraCatalog &next, const uint8_t addressTypes[6]) {
    portENTER_CRITICAL(&mux_);
    catalog_ = next;
    std::memcpy(catalogAddressTypes_, addressTypes, sizeof(catalogAddressTypes_));
    portEXIT_CRITICAL(&mux_);
}

CameraCatalog DirectCamera::catalogSnapshot() const {
    portENTER_CRITICAL(&mux_);
    const CameraCatalog copy = catalog_;
    portEXIT_CRITICAL(&mux_);
    return copy;
}

void DirectCamera::taskEntry(void *arg) {
    static_cast<DirectCamera *>(arg)->taskLoop();
}

void DirectCamera::taskLoop() {
    uint32_t seenForgetEpoch = 0;
    uint32_t seenSelectEpoch = 0;
    while (true) {
        const Inputs management = inputs();
        if (management.deviceSelectEpoch != seenSelectEpoch) {
            seenSelectEpoch = management.deviceSelectEpoch;
            // Selecting an already trusted camera is durable operator intent,
            // even if that camera is currently offline. New discoveries only
            // become durable after authorization and credential retrieval.
            if (trustedCameras.contains(management.requestedAddress,
                                        management.requestedAddressType) &&
                !trustedCameras.setSelected(management.requestedAddress,
                                            management.requestedAddressType)) {
                TrustedCamera retained;
                const bool hasRetained = trustedCameras.selected(retained);
                portENTER_CRITICAL(&mux_);
                inputs_.deviceMode = DeviceMode::Idle;
                if (hasRetained) {
                    std::strncpy(inputs_.requestedAddress, retained.address,
                                 sizeof(inputs_.requestedAddress) - 1);
                    inputs_.requestedAddress[sizeof(inputs_.requestedAddress) - 1] = '\0';
                    inputs_.requestedAddressType = retained.addressType;
                    inputs_.requestedDeviceId = cameraId(retained.address,
                                                         retained.addressType);
                } else {
                    inputs_.requestedAddress[0] = '\0';
                    inputs_.requestedAddressType = 0;
                    inputs_.requestedDeviceId = 0;
                }
                catalog_.scanning = false;
                catalog_.actionPending = false;
                catalog_.forgetPending = false;
                for (uint8_t i = 0; i < catalog_.count; ++i) {
                    catalog_.items[i].current = false;
                    catalog_.items[i].selected = hasRetained &&
                        std::strncmp(catalog_.items[i].address, retained.address,
                                     sizeof(catalog_.items[i].address)) == 0 &&
                        catalogAddressTypes_[i] == retained.addressType;
                }
                std::strncpy(catalog_.error, "camera selection save failed",
                             sizeof(catalog_.error) - 1);
                catalog_.error[sizeof(catalog_.error) - 1] = '\0';
                ++catalog_.version;
                portEXIT_CRITICAL(&mux_);
                continue;
            }
        }
        if (management.deviceForgetEpoch != seenForgetEpoch) {
            seenForgetEpoch = management.deviceForgetEpoch;
            // This is intentionally a local operation. DJI's app-level
            // protocol in this controller has no proven remote-unpair opcode.
            // Remove only the exact local identity and an exact NimBLE bond if
            // one exists; never delete all bonds as a side effect.
            const bool erased = trustedCameras.erase(
                management.forgetAddress, management.forgetAddressType);
            if (!erased) {
                portENTER_CRITICAL(&mux_);
                inputs_.forgetAddress[0] = '\0';
                inputs_.forgetAddressType = 0;
                inputs_.forgetDeviceId = 0;
                catalog_.actionPending = false;
                catalog_.scanning = false;
                catalog_.forgetPending = false;
                std::strncpy(catalog_.error, "camera forget save failed",
                             sizeof(catalog_.error) - 1);
                catalog_.error[sizeof(catalog_.error) - 1] = '\0';
                ++catalog_.version;
                portEXIT_CRITICAL(&mux_);
                continue;
            }
            NimBLEDevice::init("Osmo Core2");
            const NimBLEAddress address(std::string(management.forgetAddress),
                                        management.forgetAddressType);
            const bool bondRemoved = !NimBLEDevice::isBonded(address) ||
                                     NimBLEDevice::deleteBond(address);
            NimBLEDevice::deinit(true);
            portENTER_CRITICAL(&mux_);
            if (std::strncmp(inputs_.requestedAddress, management.forgetAddress,
                             sizeof(inputs_.requestedAddress)) == 0 &&
                inputs_.requestedAddressType == management.forgetAddressType) {
                inputs_.requestedAddress[0] = '\0';
                inputs_.requestedDeviceId = 0;
                inputs_.requestedAddressType = 0;
            }
            for (uint8_t i = 0; i < catalog_.count; ++i) {
                if (catalog_.items[i].id != management.forgetDeviceId) continue;
                for (uint8_t j = i + 1; j < catalog_.count; ++j) {
                    catalog_.items[j - 1] = catalog_.items[j];
                    catalogAddressTypes_[j - 1] = catalogAddressTypes_[j];
                }
                --catalog_.count;
                break;
            }
            inputs_.forgetAddress[0] = '\0';
            inputs_.forgetAddressType = 0;
            inputs_.forgetDeviceId = 0;
            catalog_.actionPending = false;
            catalog_.forgetPending = false;
            if (bondRemoved) {
                catalog_.error[0] = '\0';
            } else {
                std::strncpy(catalog_.error,
                             "saved camera removed; BLE bond remains",
                             sizeof(catalog_.error) - 1);
                catalog_.error[sizeof(catalog_.error) - 1] = '\0';
            }
            ++catalog_.version;
            portEXIT_CRITICAL(&mux_);
            continue;
        }
        // Re-snapshot after any NVS selection work. A UI Cancel or replacement
        // action can land while NVS commits; using the older snapshot
        // here could launch BLE/Wi-Fi for an action the operator cancelled.
        const Inputs launch = inputs();
        if (launch.deviceMode == DeviceMode::Idle) {
            // Device manager owns the next step.  Do not turn a cancelled or
            // completed scan into a background reconnect attempt.
            const DirectStatus prior = status();
            const bool awaitingExplicitRetry =
                prior.phase == DirectPhase::RetryWait && prior.retryAtMs == 0;
            if (prior.controlFault || awaitingExplicitRetry) {
                ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(250));
                continue;  // explicit STOP/AP remediation owns the next action
            }
            if (prior.phase != DirectPhase::DeviceIdle) {
                DirectStatus idle;
                idle.enabled = true;
                idle.phase = DirectPhase::DeviceIdle;
                const CameraCatalog devices = catalogSnapshot();
                const char *detail = devices.error[0] ? devices.error
                    : devices.count ? "choose a camera on Core2"
                                    : "open CAMERAS and tap SCAN";
                std::strncpy(idle.detail, detail, sizeof(idle.detail) - 1);
                publish(idle);
            }
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(250));
            continue;
        }
        if (!launch.wantEnabled) {
            DirectStatus off;
            off.phase = DirectPhase::Off;
            publish(off);
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(250));
            continue;
        }

        const uint32_t attemptReconnectEpoch = launch.reconnectEpoch;
        char runtimeError[48] = "direct link ended";
        bool runtimeControlFault = false;
        bool runtimeNeedsManualWifiRetry = false;
        DirectStatus runtimeStatus;
        {
            DirectRuntime runtime(*this, attemptReconnectEpoch);
            runtime.run();
            runtimeControlFault = runtime.wasControlFault();
            runtimeNeedsManualWifiRetry = runtime.needsManualWifiRetry();
            std::strncpy(runtimeError, runtime.lastError(),
                         sizeof(runtimeError) - 1);
            runtimeError[sizeof(runtimeError) - 1] = '\0';
            // Preserve the exact SSID and Wi-Fi evidence before runtime
            // cleanup. A new blank status would erase the 2.4 GHz guidance.
            runtimeStatus = status();
        }  // close sockets/radios and centre motion before any retry wait
        if (!inputs().wantEnabled) continue;
        if (runtimeControlFault) {
            // A management request must never erase evidence that the old
            // camera may still have its last non-neutral stick command. Abort
            // the pending scan/switch/forget and require an explicit retry.
            const Inputs failed = inputs();
            seenForgetEpoch = failed.deviceForgetEpoch;
            seenSelectEpoch = failed.deviceSelectEpoch;
            TrustedCamera retained;
            const bool hasRetained = trustedCameras.selected(retained);
            portENTER_CRITICAL(&mux_);
            inputs_.deviceMode = DeviceMode::Idle;
            inputs_.forgetAddress[0] = '\0';
            inputs_.forgetAddressType = 0;
            inputs_.forgetDeviceId = 0;
            if (hasRetained) {
                std::strncpy(inputs_.requestedAddress, retained.address,
                             sizeof(inputs_.requestedAddress) - 1);
                inputs_.requestedAddress[sizeof(inputs_.requestedAddress) - 1] = '\0';
                inputs_.requestedAddressType = retained.addressType;
                inputs_.requestedDeviceId = cameraId(retained.address,
                                                     retained.addressType);
            } else {
                inputs_.requestedAddress[0] = '\0';
                inputs_.requestedAddressType = 0;
                inputs_.requestedDeviceId = 0;
            }
            catalog_.scanning = false;
            catalog_.actionPending = false;
            catalog_.forgetPending = false;
            for (uint8_t i = 0; i < catalog_.count; ++i) {
                catalog_.items[i].current = false;
                catalog_.items[i].selected = hasRetained &&
                    std::strncmp(catalog_.items[i].address, retained.address,
                                 sizeof(catalog_.items[i].address)) == 0 &&
                    catalogAddressTypes_[i] == retained.addressType;
            }
            std::strncpy(catalog_.error, runtimeError,
                         sizeof(catalog_.error) - 1);
            catalog_.error[sizeof(catalog_.error) - 1] = '\0';
            ++catalog_.version;
            portEXIT_CRITICAL(&mux_);

            DirectStatus fault = status();
            fault.enabled = true;
            fault.ready = false;
            fault.telemetry = false;
            fault.moving = false;
            fault.clutch = false;
            fault.controlFault = true;
            if (fault.phase != DirectPhase::RetryWait)
                fault.blockedAt = fault.phase;
            fault.phase = DirectPhase::RetryWait;
            fault.retryAtMs = 0;
            std::strncpy(fault.detail, runtimeError,
                         sizeof(fault.detail) - 1);
            fault.detail[sizeof(fault.detail) - 1] = '\0';
            publish(fault);
            continue;
        }
        if (inputs().deviceMode == DeviceMode::ScanOnly &&
            inputs().reconnectEpoch == attemptReconnectEpoch) {
            // provisionOverBle published the bounded scan result.  Leave the
            // device manager idle until the operator explicitly selects one.
            // The epoch equality is essential: a SCAN requested while an old
            // READY runtime is stopping belongs to the *next* runtime and must
            // not be mistaken for a scan that the old runtime completed.
            portENTER_CRITICAL(&mux_);
            inputs_.deviceMode = DeviceMode::Idle;
            catalog_.scanning = false;
            catalog_.actionPending = false;
            catalog_.forgetPending = false;
            ++catalog_.version;
            portEXIT_CRITICAL(&mux_);
            continue;
        }
        if (inputs().deviceMode == DeviceMode::Idle) continue;
        // A reconnect is an instruction to retry now, not an error that should
        // sit through the ordinary five-second backoff. Capturing the epoch
        // before construction also prevents a request during BLE/Wi-Fi setup
        // from being mistaken for the starting state of a later phase.
        if (inputs().reconnectEpoch != attemptReconnectEpoch) continue;

        if (runtimeNeedsManualWifiRetry) {
            // The ESP32 cannot see a 5 GHz-only camera AP. Keep this connect
            // operation pending but idle: RETRY starts one new BLE epoch after
            // the operator selects 2.4 GHz; BACK/CANCEL can still end it.
            portENTER_CRITICAL(&mux_);
            inputs_.deviceMode = DeviceMode::Idle;
            for (uint8_t i = 0; i < catalog_.count; ++i)
                catalog_.items[i].current = false;
            catalog_.scanning = false;
            ++catalog_.version;
            portEXIT_CRITICAL(&mux_);

            DirectStatus retry = runtimeStatus;
            retry.enabled = true;
            retry.ready = false;
            retry.telemetry = false;
            retry.moving = false;
            retry.clutch = false;
            retry.controlFault = false;
            retry.phase = DirectPhase::RetryWait;
            retry.blockedAt = DirectPhase::WifiJoin;
            retry.retryAtMs = 0;
            publish(retry);
            continue;
        }

        portENTER_CRITICAL(&mux_);
        for (uint8_t i = 0; i < catalog_.count; ++i)
            catalog_.items[i].current = false;
        catalog_.actionPending = true;
        catalog_.scanning = false;
        ++catalog_.version;
        portEXIT_CRITICAL(&mux_);

        DirectStatus retry = status();
        retry.enabled = true;
        retry.ready = false;
        retry.telemetry = false;
        retry.moving = false;
        retry.clutch = false;
        retry.phase = DirectPhase::RetryWait;
        retry.controlFault = runtimeControlFault;
        retry.retryAtMs = millis() + RETRY_MS;
        std::strncpy(retry.detail, runtimeError, sizeof(retry.detail) - 1);
        retry.detail[sizeof(retry.detail) - 1] = '\0';
        publish(retry);

        const uint32_t reconnectEpoch = inputs().reconnectEpoch;
        const uint32_t waitStarted = millis();
        while (inputs().wantEnabled &&
               inputs().reconnectEpoch == reconnectEpoch &&
               millis() - waitStarted < RETRY_MS) {
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
        }
    }
}

float DirectCamera::jogResponse(float magnitude) {
    magnitude = clampf(magnitude, 0.0f, 1.0f);
    constexpr float deadzone = 0.12f;
    if (magnitude <= deadzone) return 0.0f;
    return std::pow((magnitude - deadzone) / (1.0f - deadzone), 1.6f);
}

const char *DirectCamera::phaseName(DirectPhase phase) {
    switch (phase) {
        case DirectPhase::Off: return "off";
        case DirectPhase::Starting: return "starting";
        case DirectPhase::DeviceIdle: return "choose camera";
        case DirectPhase::BleScan: return "BLE scan";
        case DirectPhase::BlePair: return "pairing";
        case DirectPhase::AwaitApproval: return "approve";
        case DirectPhase::WifiJoin: return "Wi-Fi join";
        case DirectPhase::TcpPoke: return "TCP 7001";
        case DirectPhase::UdpHandshake: return "UDP 9004";
        case DirectPhase::Registering: return "registering";
        case DirectPhase::Ready: return "ready";
        case DirectPhase::RetryWait: return "retry wait";
        case DirectPhase::Stopping: return "stopping";
        default: return "unknown";
    }
}

const char *DirectCamera::actionName(CameraAction action) {
    switch (action) {
        case CameraAction::FocusSingle: return "AF-S";
        case CameraAction::FocusContinuous: return "AF-C";
        case CameraAction::GimbalRecenter: return "recenter";
        case CameraAction::GimbalFlip: return "flip";
        case CameraAction::GimbalFollow: return "follow mode";
        case CameraAction::GimbalFpv: return "FPV mode";
        case CameraAction::None:
        default: return "camera action";
    }
}

}  // namespace osmo
