#pragma once

#include <Arduino.h>

#include "motion_program.h"

namespace osmo {

class DirectRuntime;
constexpr uint8_t CAMERA_CATALOG_CAPACITY = 6;

// A stable UI-facing camera identity.  `trusted` means Core2 previously
// completed DJI app authorization and retrieved that camera's credentials;
// it does not mean the camera reports a BLE bond. `current` is reserved for a
// live READY session with fresh telemetry.
struct CameraSummary {
    uint32_t id = 0;
    int8_t rssi = -127;
    uint8_t model = 0;
    bool nearby = false;
    bool trusted = false;
    bool selected = false;
    bool current = false;
    bool verified = false;
    char name[24] = "";
    char address[18] = "";
};

struct CameraCatalog {
    uint32_t version = 0;
    uint8_t count = 0;
    bool scanning = false;
    bool actionPending = false;
    // A confirmed destructive action is intentionally not cancellable after
    // the 750 ms hold crosses its commit point.
    bool forgetPending = false;
    char error[40] = "";
    CameraSummary items[CAMERA_CATALOG_CAPACITY] = {};
};

enum class DirectPhase : uint8_t {
    Off = 0,
    Starting,
    DeviceIdle,
    BleScan,
    BlePair,
    AwaitApproval,
    WifiJoin,
    TcpPoke,
    UdpHandshake,
    Registering,
    Ready,
    RetryWait,
    Stopping,
};

// Camera writes whose payloads exist in the captured command catalog but are
// not yet camera-body-qualified through this standalone transport. The UI must
// present successful delivery as an amber request, never as a confirmed state.
enum class CameraAction : uint8_t {
    None = 0,
    FocusSingle,
    FocusContinuous,
    GimbalRecenter,
    GimbalFlip,
    GimbalFollow,
    GimbalFpv,
};

enum class MotionPhase : uint8_t {
    Empty = 0,
    Ready,
    Armed,
    Positioning,
    Running,
    Dwelling,
    Cued,        // holding at a point until a human GO
    Complete,
    Aborted,
    Fault,
};

// What the USB host used to be told was attitude and was in fact raw gyro:
// the `I` line carried gx/gy/gz and the host read them as pitch/roll. This
// is the estimator's gravity-referenced answer in the old core2_imu.ino
// convention, so the host clutch's sign and gain tests keep meaning.
struct HostImu {
    float pitch = 0.0f;
    float roll = 0.0f;
};

struct DirectStatus {
    bool enabled = false;
    bool ready = false;
    bool telemetry = false;
    bool moving = false;
    bool clutch = false;
    // Operator-relative orientation captured at the current clutch edge.
    // These are measured Core2 deltas, not guessed camera pose or tracking
    // error. Output is the shaped command magnitude as a percentage of the
    // selected speed cap.
    float handDeltaPitch = 0.0f;
    float handDeltaPan = 0.0f;
    uint8_t handOutputPct = 0;
    bool controlFault = false;
    // Direct DUML has no decoded authoritative record tally. These fields
    // deliberately distinguish a requested command from camera confirmation.
    bool recordIntent = false;
    bool recordPending = false;
    bool recordUnconfirmed = false;
    bool recordCommandFault = false;
    CameraAction cameraAction = CameraAction::None;
    bool cameraActionPending = false;
    bool cameraActionUnconfirmed = false;
    bool cameraActionFault = false;
    MotionPhase motionPhase = MotionPhase::Empty;
    uint8_t motionPoint = 0;
    uint8_t motionCount = 0;
    uint16_t motionProgress = 0;  // 0..1000, never a guessed countdown
    bool motionProgramActive = false;
    bool motionProgramArmed = false;
    bool motionProgramFault = false;
    // A cued point holds the head until a human GO. 1-based; 0 = not cued.
    uint8_t cuePoint = 0;
    uint16_t motionLap = 0;    // ends reached in LOOP/BOUNCE this run
    uint8_t motionRepeat = 0;  // MotionRepeat of the loaded programme
    uint8_t motionSlot = 0;    // which stored programme is loaded
    // The head is within ARM tolerance of P1. What "back to one" is for.
    bool motionAtStart = false;
    DirectPhase phase = DirectPhase::Off;
    // When phase is RetryWait this preserves the exact stage that failed.  The
    // progress screen can therefore remain truthful instead of erasing all
    // completed work as soon as a retry begins.
    DirectPhase blockedAt = DirectPhase::Off;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float pitchHeadLow = 0.0f;
    float pitchHeadHigh = 0.0f;
    uint8_t pitchWarnLow = 0;
    uint8_t pitchWarnHigh = 0;
    // Wi-Fi evidence is deliberately public to the UI, but never includes the
    // WPA key.  It distinguishes a sleeping AP from a rejected association.
    uint8_t wifiAttempt = 0;
    uint8_t wifiStatus = 0;
    uint8_t wifiChannel = 0;
    uint8_t wifiBssid[6] = {};
    uint32_t retryAtMs = 0;
    uint32_t telemetryAgeMs = UINT32_MAX;
    // Receive census for safe, no-motion diagnosis of the final datalink
    // hand-off. Registration being sent is not evidence that the camera
    // accepted it or that a parseable attitude frame arrived.
    uint32_t rxPeerPackets = 0;
    uint32_t rxForeignPackets = 0;
    uint32_t rxHeaderRejects = 0;
    uint32_t rxWindowPackets = 0;
    uint32_t rxAckedDataPackets = 0;
    uint32_t rxVideoPackets = 0;
    uint32_t rxFrames = 0;
    uint32_t rxAttitudeFrames = 0;
    uint32_t rxAcceptedAttitudes = 0;
    uint16_t lastRxLen = 0;
    uint16_t lastFramePayloadLen = 0;
    uint16_t lastAttitudeRxLen = 0;
    uint16_t lastAttitudeSeq = 0;
    uint16_t ackVideoCursor = 0;
    uint16_t ackDataCursor = 0;
    uint16_t ackExtraCursor = 0;
    uint8_t lastRxType = 0xFF;
    uint8_t lastFrameCmdSet = 0xFF;
    uint8_t lastFrameCmdId = 0xFF;
    uint8_t lastAttitudeType = 0xFF;
    char ssid[34] = "";
    char wifiReason[24] = "";
    char detail[48] = "";
};

// Complete host-free connection spine for the Core2:
//
//   BLE app pairing / AP wake / credential read
//     -> BLE off -> Wi-Fi STA -> TCP :7001 held open
//     -> UDP :9004 handshake / register / subscriptions
//     -> fresh telemetry gate -> clutch or momentary jog motion
//
// The networking task owns every BLE/Wi-Fi/socket object. The UI core only
// publishes input snapshots and reads status, so a slow scan or association
// cannot starve LVGL, touch handling, the screen watchdog, or the STOP input.
class DirectCamera {
public:
    void begin();
    void disable();
    void requestReconnect();
    void abort();

    // Standalone device manager. These calls merely publish an operation epoch
    // for the networking task; they never touch BLE, Wi-Fi or sockets from the
    // UI/LVGL task.
    void requestDeviceScan();
    void cancelDeviceAction();
    void selectDevice(uint32_t id);
    void forgetDevice(uint32_t id);  // local Core2 forget; not camera unpair
    CameraCatalog catalog() const;

    void setImu(float ax, float ay, float az, float gx, float gy, float gz,
                uint32_t sampleAtMs);
    void setClutch(bool held);
    void setJog(float tilt, float pan, bool active);
    void setControlConfig(uint8_t tiltResponseIndex,
                          uint8_t panResponseIndex,
                          uint8_t tiltStabilityIndex,
                          uint8_t panStabilityIndex,
                          uint8_t speedIndex, uint8_t smoothIndex,
                          uint8_t templateIndex,
                          bool lockTilt, bool lockPan,
                          bool invertTilt, bool invertPan);
    void setManualEase(const uint16_t (&milliseconds)[2][4]);
    void setJogConfig(uint8_t speedIndex, uint8_t smoothIndex);
    void requestRecord(bool start);
    void requestCameraAction(CameraAction action);

    // Bounded, persisted P1..Pn programme. Capture is allowed only from fresh
    // camera attitude; a run owns the same single motion path as hand and jog.
    MotionProgram motionProgram() const;
    bool captureMotionPoint(uint8_t index);
    bool clearMotionPoint(uint8_t index);
    bool removeMotionPoint(uint8_t index);
    bool setMotionTiming(uint8_t index, uint32_t moveMs, uint32_t dwellMs);
    bool setMotionHold(uint8_t index, bool holdForGo);
    bool setMotionTransition(uint8_t index, MotionTransition transition);
    bool setMotionRepeat(uint8_t repeat);
    bool selectMotionSlot(uint8_t slot);
    bool requestMotionGoto(uint8_t index);
    bool requestMotionArm();
    bool requestMotionGo();
    bool requestMotionPlay();  // deliberate hold: validate and start at P1
    bool requestMotionRun();  // compatibility alias: arms, never starts motion

    DirectStatus status() const;
    HostImu hostImu() const;
    bool enabled() const;
    // Single owner for the JOG deadzone/expo curve. The UI uses this same
    // response to display effective output rather than raw thumb travel.
    static float jogResponse(float magnitude);
    static const char *phaseName(DirectPhase phase);
    static const char *actionName(CameraAction action);

private:
    friend class DirectRuntime;
    enum class DeviceMode : uint8_t {
        AutoConnect = 0,
        ScanOnly,
        ConnectSelected,
        Idle,
    };
    enum class MotionCommand : uint8_t {
        None = 0,
        GotoPoint,
        Arm,
        Go,
        Play,
    };
    struct Inputs {
        float ax = 0.0f;
        float ay = 0.0f;
        float az = 1.0f;
        float gx = 0.0f;
        float gy = 0.0f;
        float gz = 0.0f;
        // Attitude is an IMU-to-world quaternion updated at the 100 Hz input
        // boundary; the network task only snapshots it.
        float qw = 1.0f;
        float qx = 0.0f;
        float qy = 0.0f;
        float qz = 0.0f;
        float jogTilt = 0.0f;
        float jogPan = 0.0f;
        uint32_t imuAtMs = 0;
        uint32_t abortEpoch = 0;
        uint32_t reconnectEpoch = 0;
        uint32_t deviceScanEpoch = 0;
        uint32_t deviceCancelEpoch = 0;
        uint32_t deviceSelectEpoch = 0;
        uint32_t deviceForgetEpoch = 0;
        uint32_t requestedDeviceId = 0;
        uint8_t requestedAddressType = 0;
        DeviceMode deviceMode = DeviceMode::AutoConnect;
        char requestedAddress[18] = "";
        // Forget is a distinct operation target. Reusing requestedAddress
        // would silently replace the selected/current camera when deleting a
        // different saved identity.
        uint32_t forgetDeviceId = 0;
        uint8_t forgetAddressType = 0;
        char forgetAddress[18] = "";
        uint8_t tiltResponseIndex = 1;
        uint8_t panResponseIndex = 1;
        uint8_t tiltStabilityIndex = 1;
        uint8_t panStabilityIndex = 1;
        uint8_t speedIndex = 1;
        uint8_t smoothIndex = 1;
        uint8_t jogSpeedIndex = 1;
        uint8_t jogSmoothIndex = 1;
        uint16_t manualEase[2][4] = {}; // HAND/JOG: tilt start/end, pan start/end
        uint8_t templateIndex = 0;
        uint32_t recordEpoch = 0;
        uint32_t cameraActionEpoch = 0;
        CameraAction cameraAction = CameraAction::None;
        uint32_t motionEpoch = 0;
        MotionCommand motionCommand = MotionCommand::None;
        uint8_t motionPoint = 0;
        bool recordStart = false;
        bool clutch = false;
        bool jogActive = false;
        bool lockTilt = false;
        bool lockPan = false;
        bool invertTilt = false;
        bool invertPan = false;
        bool wantEnabled = false;
    };

    static void taskEntry(void *arg);
    void taskLoop();
    void publish(const DirectStatus &next);
    void publishCatalog(const CameraCatalog &next);
    void publishCatalogWithAddressTypes(const CameraCatalog &next,
                                        const uint8_t addressTypes[6]);
    Inputs inputs() const;
    CameraCatalog catalogSnapshot() const;

    mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
    Inputs inputs_;
    DirectStatus status_;
    CameraCatalog catalog_;
    MotionProgramStore motionStore_;
    MotionProgram motionProgram_;
    bool motionStoreReady_ = false;
    uint8_t catalogAddressTypes_[6] = {};
    struct ImuEstimator {
        float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
        float bx = 0.0f, by = 0.0f, bz = 0.0f;
        float lastAx = 0.0f, lastAy = 0.0f, lastAz = 1.0f;
        uint32_t lastAtMs = 0;
        uint32_t stationarySinceMs = 0;
        uint32_t freezeUntilMs = 0;
    } imu_;
    TaskHandle_t task_ = nullptr;
};

extern DirectCamera directCamera;

}  // namespace osmo
