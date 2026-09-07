// M5Stack Core2 control surface for the Osmo Pocket rig.
//
// This is a one-thumb camera console: HAND, JOG, bounded MOVES, DEVICE and
// SETTINGS workspaces share one control owner and one persistent STOP. It does
// not imitate a phone UI; every function must remain operable on 320x240 and
// every camera action must state whether it is proven or still R&D-only.
//
// HONEST LIMITATION: the three "buttons" are capacitive touch zones, not
// switches. They fail through gloves and rain covers and can ghost-trigger
// when wet. This is a bare-hand prototype. A set-ready build needs a real
// spring-return clutch paddle and an unmistakable physical STOP.
//
// Transport: standalone BLE/Wi-Fi camera link, or USB serial to driver/core2.py.

#include <M5Unified.h>
#include <FastLED.h>
#include <Preferences.h>
#include <esp_heap_caps.h>
#include <lvgl.h>
#include <cstring>

#include "direct_camera.h"
#include "haptic_envelope.h"
#include "motion_playback_ui.h"

// LVGL 8 invalidates and reallocates a label even when the new bytes are
// identical.  The panel has many live summaries, so the old 10 Hz refresh
// repeatedly repainted hidden/static text through a 24-line display buffer.
// Keep the existing call sites readable while making unchanged text free.
static void setLabelTextIfChanged(lv_obj_t *label, const char *text) {
    if (!label || !text) return;
    const char *current = lv_label_get_text(label);
    if (!current || std::strcmp(current, text) != 0)
        lv_label_set_text(label, text);
}
#define lv_label_set_text(label, text) \
    setLabelTextIfChanged((label), (text))

// Forward declarations: the LVGL event callbacks are defined above the
// serial helpers they use.
static void say(const char *fmt, ...);
static void tick(uint16_t ms);

// ---------------------------------------------------------------- LED beacon
// Ten SK6812s on the M5GO Bottom2. Treated as ONE prioritised beacon, never as
// ten separate indicators: peripheral vision reads a single state and a single
// urgency. Colour rules, in priority order:
//
//   magenta double-pulse  control fault (never mistaken for tally)
//   red solid             RECORDING -- and only when the rig can prove it
//   cyan                  manual clutch has the head
//   amber moving          programmed motion actually executing
//   amber breathe         armed, waiting for a human GO
//   amber solid           armed but stationary
//   off                   safely disarmed
//
// Deliberately absent: battery, signal strength, angles, progress, tracking
// error. All ambiguous at the edge of vision.

#define LED_PIN     25
#define LED_COUNT   10
static CRGB leds[LED_COUNT];

// The ten LEDs are two vertical bars of five, one down each side of the box.
// That geometry is worth using rather than lighting all ten the same colour:
// the bars can say WHICH way the head is running out of travel, which is the
// thing the operator cannot see while looking at the subject.
//
// Vertical extremes read as tilt and the sides read as pan, because that is
// how the hand already thinks about the box:
//
//     top of both bars     tilt is running out upward
//     bottom of both bars  tilt is running out downward
//     middle of the left   pan is running out to the left
//     middle of the right  pan is running out to the right
//
// This is an overlay. The beacon still owns the bars and paints first; only
// the LEDs a live warning needs are taken, and only while it is live. Arming,
// recording and fault states are never masked by a limit.
#define LED_LEFT_BASE   0
#define LED_RIGHT_BASE  5
#define LED_PER_BAR     5

// Below this a limit is a passive notch rather than something to shout about,
// and lighting the bars for it would make them permanent furniture.
static const uint8_t LIMIT_LED_FLOOR = 30;

static bool limitLedsEnabled = true;
static bool beaconEnabled = true;

enum Beacon : uint8_t {
    BEACON_OFF = 0, BEACON_ARMED, BEACON_WAIT_GO,
    BEACON_MOVING, BEACON_CLUTCH, BEACON_REC, BEACON_FAULT
};

static Beacon   beacon        = BEACON_OFF;
static uint8_t  ledBrightness = 60;      // day/interior/night/blackout
static bool     blackout      = false;

// 0..100 per direction, pushed down from the panel on the S line.
static uint8_t limUpLed = 0, limDownLed = 0, limLeftLed = 0, limRightLed = 0;

// Amber at the first hint, red at the stop -- the same grammar the screen
// gauges and the web panel use, so one colour means one thing everywhere.
static CRGB limitColour(uint8_t v) {
    const float t = (v - LIMIT_LED_FLOOR) / (float)(100 - LIMIT_LED_FLOOR);
    const uint8_t g = (uint8_t)(176 * (1.0f - t));
    return CRGB(255, g, 0);
}

static void paintLimit(int idx, uint8_t v) {
    if (v < LIMIT_LED_FLOOR) return;
    leds[idx] = limitColour(v);
}

// Called after the beacon has painted, so a limit warning sits on top of the
// state colour rather than replacing it.
static void overlayLimitLeds() {
    if (!limitLedsEnabled || blackout) return;

    const int lTop = LED_LEFT_BASE + LED_PER_BAR - 1;
    const int rTop = LED_RIGHT_BASE + LED_PER_BAR - 1;
    paintLimit(lTop, limUpLed);
    paintLimit(rTop, limUpLed);
    paintLimit(LED_LEFT_BASE, limDownLed);
    paintLimit(LED_RIGHT_BASE, limDownLed);

    for (int i = 1; i < LED_PER_BAR - 1; i++) {
        paintLimit(LED_LEFT_BASE + i, limLeftLed);
        paintLimit(LED_RIGHT_BASE + i, limRightLed);
    }
}

static void renderBeacon(uint32_t now) {
    CRGB base = CRGB::Black;
    if (!beaconEnabled) {
        // The bars still carry travel warnings; only the state colour goes.
        fill_solid(leds, LED_COUNT, CRGB::Black);
        overlayLimitLeds();
        FastLED.show();
        return;
    }
    float wave = 1.0f;

    switch (beacon) {
        case BEACON_FAULT: {
            // Double-pulse: unmistakable, and not a colour anything else uses.
            uint32_t t = now % 1200;
            wave = (t < 120 || (t > 240 && t < 360)) ? 1.0f : 0.05f;
            base = CRGB(255, 0, 140);
            break;
        }
        case BEACON_REC:
            base = CRGB::Red;             // authoritative tally only
            break;
        case BEACON_CLUTCH:
            base = CRGB(0, 200, 255);
            break;
        case BEACON_MOVING: {
            // Motion means the head is actually moving. Nothing else animates
            // like this -- not network traffic, not a heartbeat.
            uint32_t t = now % 900;
            wave = 0.35f + 0.65f * (t < 450 ? t : 900 - t) / 450.0f;
            base = CRGB(255, 150, 0);
            break;
        }
        case BEACON_WAIT_GO: {
            uint32_t t = now % 2000;      // slow breathe: a human cue is pending
            wave = 0.25f + 0.75f * (t < 1000 ? t : 2000 - t) / 1000.0f;
            base = CRGB(255, 150, 0);
            break;
        }
        case BEACON_ARMED:
            base = CRGB(255, 150, 0);
            break;
        case BEACON_OFF:
        default:
            base = CRGB::Black;
            break;
    }

    uint8_t level = blackout ? 3 : ledBrightness;
    CRGB c = base;
    c.nscale8_video((uint8_t)(level * wave));
    for (int i = 0; i < LED_COUNT; i++) leds[i] = c;
    overlayLimitLeds();
    FastLED.show();
}

// A short verdict flash after a take, then straight back to the running state.
// The full diagnosis lives in the log, not on the LEDs.
static uint32_t flashUntil = 0;
static CRGB     flashColour = CRGB::Black;
static uint8_t  flashPulses = 0;

static void startFlash(CRGB colour, uint8_t pulses) {
    flashColour = colour;
    flashPulses = pulses;
    flashUntil  = millis() + pulses * 260;
}

static bool renderFlash(uint32_t now) {
    if (now >= flashUntil) return false;
    uint32_t t = (flashUntil - now) % 260;
    CRGB c = (t < 130) ? flashColour : CRGB::Black;
    c.nscale8_video(blackout ? 3 : ledBrightness);
    for (int i = 0; i < LED_COUNT; i++) leds[i] = c;
    FastLED.show();
    return true;
}

// ------------------------------------------------------------------- haptics
// Speaker is muted by default and stays muted across reconnects: most beeps
// are unusable on a set with live sound. Haptics are for the hand only, and
// even those are suppressed when the operator is holding the camera, where
// vibration reaches the microphone and shows in the frame.

// The box sits on a tripod for hours during a timelapse. The screen is the
// biggest draw on the battery and the brightest thing on a dark set, and
// nobody is looking at it between takes.
//
// Sleeping is suppressed while a fault is up or a travel limit is being hit --
// those are precisely the moments somebody glances over, and a dark screen
// then reads as a dead rig.
static uint8_t  screenTimeoutIdx = 2;                 // 1 min
static const char *SCREEN_TIMEOUT_NAMES[] = {"NEVER", "30s", "1min", "5min"};
static const uint32_t SCREEN_TIMEOUT_MS[] = {0, 30000UL, 60000UL, 300000UL};
static uint32_t screenOffAt   = 0;
static bool     screenAsleep  = false;
// The touch that WAKES the screen must not also press whatever sits under the
// finger. Waking onto the clutch page and engaging the clutch because the
// thumb landed there is the failure that makes people switch a timeout off.
static bool     swallowWake   = false;
// Bezel keys need the same wake-only first press as the touchscreen. Without
// this latch BtnB can wake and grab in one loop, while A/C can wake and change
// pages before the operator has even seen the screen.
static bool     swallowHardwareWake = false;

// Declared here, beside the state it owns, because touchCb is defined well
// above it and one translation unit means order is the compiler's whole view.
static void noteScreenActivity(uint32_t now);

static bool soundEnabled  = false;

// setVolume(0) at boot and nothing ever raised it: the SOUND ON toggle flipped
// a label and a boolean against a permanently muted speaker. Every beep this
// firmware has ever "made" went nowhere. Routed through one setter so the flag
// and the hardware cannot disagree again.
static void setSoundEnabled(bool on);
static bool cameraInHand  = false;

// Hardware-correct, non-blocking haptics. Voltage quantization and pulse
// energy live in a pure module executed by the native C++ regression tests.
enum HapticLevel : uint8_t { HAPTIC_OFF = 0, HAPTIC_SOFT, HAPTIC_MED, HAPTIC_FIRM };
static const char *HAPTIC_LEVEL_NAMES[] = {"OFF", "SOFT", "MED", "FIRM"};
static uint8_t hapticLevelIdx = HAPTIC_SOFT;
static osmo::HapticEnvelope hapticEnvelope;
static uint8_t hapticApplied = 0;
static bool uiRefreshRequested = true;

static void scheduleHaptic(uint8_t strength, uint16_t durationMs) {
    hapticEnvelope.schedule(millis(), strength, durationMs,
                            hapticLevelIdx, cameraInHand);
}

static void tick(uint16_t ms) {
    uiRefreshRequested = true;
    scheduleHaptic(180, ms);
}

static void pumpHaptic(uint32_t now) {
    if (cameraInHand || hapticLevelIdx == HAPTIC_OFF)
        hapticEnvelope.cancel();
    const uint8_t target = hapticEnvelope.sample(now);
    // Write only when the actual PMIC voltage changes, not every ramp tick.
    if (target != hapticApplied) {
        M5.Power.setVibration(target);
        hapticApplied = target;
    }
}
static void hapticGrab()   { tick(28); }                       // clutch acquired
static void hapticGo()     { tick(45); }                       // GO accepted
// ---------------------------------------------------------------- run cues
// On a set the crew has to know the head is about to move WITHOUT looking at a
// screen -- the operator is watching the subject and the AC is watching the
// lens. Dragonframe and MRMC Flair both beep; so does this.
//
// Every tone is scheduled, never played inline. applyState() runs on the
// serial reader, and a countdown that slept between beeps would stall the link
// and the IMU stream for three seconds -- the rig would appear to freeze
// exactly as it was about to move.
//
// M5.Speaker.tone() returns immediately; what needs sequencing is the GAP
// between tones, and that is a deadline compared in loop() rather than a wait.

static const uint8_t SPEAKER_VOLUME = 96;   // of 255

static void setSoundEnabled(bool on) {
    soundEnabled = on;
    M5.Speaker.setVolume(on ? SPEAKER_VOLUME : 0);
}

struct RunCueTone { uint16_t hz; uint16_t ms; uint16_t gap; };

// Descending for the countdown, because a falling pitch reads as "approaching"
// and a rising one reads as "finished". The end pair is low and quick so it
// cannot be mistaken for the start.
static const RunCueTone CUE_ARM[]   = {{1760, 110, 0}};
static const RunCueTone CUE_COUNT[] = {{1320, 120, 1000},
                                       {1040, 120, 1000},
                                       { 780, 120, 0}};
static const RunCueTone CUE_GO[]    = {{1040, 350, 0}};
static const RunCueTone CUE_END[]   = {{440, 90, 170}, {440, 90, 0}};

static const RunCueTone *runCue = nullptr;
static uint8_t  runCueStep = 0, runCueCount = 0;
static uint32_t runCueAt = 0;
static bool     runCueBuzz = false;

static void scheduleRunCue(const char *name) {
    if (!name || !name[0]) return;
    if      (!strcmp(name, "arm"))   { runCue = CUE_ARM;   runCueCount = 1;
                                       runCueBuzz = true; }
    else if (!strcmp(name, "count")) { runCue = CUE_COUNT; runCueCount = 3;
                                       runCueBuzz = false; }
    else if (!strcmp(name, "go"))    { runCue = CUE_GO;    runCueCount = 1;
                                       runCueBuzz = true; }
    else if (!strcmp(name, "end"))   { runCue = CUE_END;   runCueCount = 2;
                                       runCueBuzz = false; }
    else return;                      // an unknown cue is ignored, not guessed
    runCueStep = 0;
    runCueAt = millis();
}

static void pumpRunCue(uint32_t now) {
    if (!runCue) return;
    // Signed compare: millis() wraps after 49 days and an unsigned test would
    // stop the cues dead for the rest of the shoot.
    if ((int32_t)(now - runCueAt) < 0) return;

    const RunCueTone &t = runCue[runCueStep];
    if (soundEnabled) M5.Speaker.tone(t.hz, t.ms);
    // Haptic on the cues that mean "it is about to move", and only on the
    // first tone: a buzz per beep during a countdown is just noise in the hand.
    if (runCueBuzz) { tick(28); runCueBuzz = false; }

    if (++runCueStep >= runCueCount) runCue = nullptr;
    else                             runCueAt = now + t.gap;
}

// Approaching a stop should be felt before it is hit, and the warning has to
// grow or it becomes wallpaper. `near` is 0..100 (100 = at the stop): the
// pulses get shorter-spaced and firmer as it climbs, and stop entirely once
// the operator moves away. Never a continuous buzz -- that numbs the cue,
// shakes the hand and invites the exact oscillation it is warning about.
static uint8_t limitNear = 0;

// A head simply parked near a stop reports a capped notch -- 35, quantised to
// 40 on the wire -- because being somewhere is not the same as heading there.
// The buzz must start above that or a rig set down near its yaw stop hums
// forever, which is precisely the nuisance this was meant to prevent.
static const uint8_t HAPTIC_FLOOR = 50;

static void pumpLimitHaptic(uint32_t now) {
    static uint32_t nextAt = 0;
    if (cameraInHand || limitNear < HAPTIC_FLOOR) { nextAt = 0; return; }
    if (nextAt && now < nextAt) return;
    // 50% -> every 900 ms and barely there; 100% -> every 160 ms and firm.
    const float f = (limitNear - HAPTIC_FLOOR) / (float)(100 - HAPTIC_FLOOR);
    const uint16_t gap = (uint16_t)(900 - 740 * f);
    const uint16_t dur = (uint16_t)(12 + 26 * f);
    nextAt = now + gap;
    scheduleHaptic(90 + (uint8_t)(120 * f), dur);
}

static void hapticLimit()  { if (!cameraInHand) { tick(70); } } // hard stop
static void hapticFault()  { if (!cameraInHand) { tick(120); } }

// ---------------------------------------------------------------- rig state
struct RigState {
    bool     linked      = false;   // PC driver is talking to us
    bool     camera      = false;   // camera datalink up
    bool     armed       = false;
    bool     moving      = false;
    bool     recording   = false;   // only ever set when provable
    // Direct record requests are deliberately separate from `recording`.
    // A successful command write is not camera-body tally evidence.
    bool     recordIntent = false;
    bool     recordPending = false;
    bool     recordUnconfirmed = false;
    bool     recordCommandFault = false;
    bool     telemetry   = false;
    // Received and then thrown away until now: absolute pose is what makes a
    // frame repeatable, and telemetry age is the difference between a link
    // that is working and one that stopped a second ago.
    uint32_t telemetryAgeMs = UINT32_MAX;
    float    pitchNow    = 0.0f;
    float    yawNow      = 0.0f;
    bool     poseValid   = false;
    int      waitingCue  = -1;
    bool     lockTilt    = false;   // server owns these; we only display them
    bool     lockPan     = false;
    bool     nearLimit   = false;
    char     owner[10]   = "none";  // program / core2 / phone / none
    char     move[24]    = "";
    float    elapsed     = 0.0f;
    // A timelapse runs for hours and the operator is not at the browser.
    int      tlFrame     = 0;
    int      tlFrames    = 0;
    float    total       = 0.0f;
    float    headUp      = 0.0f;    // degrees of travel left, each way
    float    headDown    = 0.0f;
    // How close each individual end of travel is, 0..100. Four numbers, not
    // one: a single figure for the whole rig can only light the whole control,
    // which says something is wrong without saying what or which way.
    uint8_t  limUp       = 0;       // pitch toward the arc low end
    uint8_t  limDown     = 0;       // pitch toward the arc high end
    uint8_t  limLeft     = 0;       // yaw toward the arc low end
    uint8_t  limRight    = 0;       // yaw toward the arc high end
    char     fault[48]   = "";
    char     ssid[34]    = "";      // whatever access point the host is on
    char     detail[48]  = "";      // connection progress or offline reason
    bool     direct      = false;   // Core2 owns the camera transport
    osmo::DirectPhase directPhase = osmo::DirectPhase::Off;
    osmo::DirectPhase directBlockedAt = osmo::DirectPhase::Off;
    uint8_t  wifiAttempt = 0;
    uint8_t  wifiStatus = 0;
    uint8_t  wifiChannel = 0;
    uint8_t  wifiBssid[6] = {};
    uint32_t retryAtMs = 0;
    char     wifiReason[24] = "";
};
static RigState rig;

// Motion feedback and input availability share the same evidence gate. A BLE
// association, a USB serial line, or a UDP socket by itself is not permission
// to imply that a clutch or jog command can reach a measured camera head.
static bool controlEvidenceReady() {
    return rig.linked && rig.camera && rig.telemetry && !rig.fault[0];
}

static bool     clutchHeld   = false;
// STOP and link loss require a physical release before another grab. A thumb
// still holding BtnB must not undo the stop on the next 2 ms loop.
static bool     clutchReleaseRequired = false;
// Local contact is intent; feedback waits for camera-owner evidence.
static bool     clutchFeedbackConfirmed = false;
static uint32_t clutchReleaseCandidateMs = 0;
// The touch jog needs the same edge rule, but must also remember raw contact.
// "inactive" is a command state; it is not proof that the operator lifted a
// finger. Keeping those facts separate prevents a held stick from waking up
// when telemetry or transport authority returns.
static bool     jogRawContact = false;
static bool     jogReleaseRequired = false;
static uint32_t jogReleaseCandidateMs = 0;
static float    jogX = 0, jogY = 0;
static bool     jogActive = false;
static bool     touchDiag    = true;
// LVGL redraws and synchronous haptics can occupy the Arduino loop for tens
// of milliseconds. IMU sampling therefore has its own periodic task; the
// loop remains only a fallback if task allocation fails at boot.
static TaskHandle_t imuTaskHandle = nullptr;
static volatile uint32_t imuSampleCount = 0;
static volatile uint32_t imuSerialDrops = 0;
static volatile uint32_t imuReadFailures = 0;
static volatile UBaseType_t imuStackWords = 0;
static uint32_t lastRxMs     = 0;
// Set only after a syntactically valid serial command. Serial.available()
// cannot tell us whether a host spoke because pumpSerial() deliberately drains
// the buffer before transport ownership is selected.
static bool     usbHostSeen  = false;
static uint32_t hostClaimDeadlineMs = 0;
static const uint32_t LINK_TIMEOUT_MS = 1500;

// --------------------------------------------------------------------- LVGL
static lv_disp_draw_buf_t drawBuf;
// BLE + Wi-Fi make the standalone image too large for the ESP32's small
// linker-time DRAM segment if LVGL also parks both scanline buffers there.
// Core2 has PSRAM, so allocate these runtime-only pixels there after M5.begin.
static constexpr size_t DRAW_PIXELS = 320 * 24;
static lv_color_t *buf1 = nullptr;
static lv_color_t *buf2 = nullptr;

static lv_obj_t *screenMain, *pager;
static lv_obj_t *pgControl, *pgJog, *pgClutch, *pgDevice, *pgSetup;

// SETTINGS is a compact launcher, not a long configuration form. The root is
// a two-column icon grid; CONTROL opens one small sub-grid and every other tile
// opens a focused leaf or an existing canonical workspace. This keeps setup
// discoverable on 320x240 without burying the controls changed while shooting.
// LEFT is BACK inside a sub-page and the root always reopens at CAMERA.
enum SetSub : uint8_t {
    SET_MENU = 0, SET_CONTROLS, SET_DIRECTION, SET_DISPLAY, SET_FEEDBACK,
    SET_HAPTIC, SET_SYSTEM, SET_COUNT
};
static const char *SET_NAMES[SET_COUNT] = {
    "SETTINGS", "CONTROLS", "DIRECTION", "DISPLAY", "FEEDBACK", "HAPTIC",
    "SYSTEM"
};
static uint8_t setSub = SET_MENU;
// DIRECTION is shared by persistent Settings and both shot-time FEEL sheets.
// Remember only the live-sheet origins so BACK restores the operator's exact
// shooting context instead of dropping them into an unrelated Settings grid.
enum class SettingsReturn : uint8_t { None = 0, HandFeel, JogFeel };
static SettingsReturn settingsReturn = SettingsReturn::None;
static lv_obj_t *pgSetPanes[SET_COUNT];
static lv_obj_t *btnLimitLeds, *lblLimitLedsTxt;
static lv_obj_t *btnBeaconOn, *lblBeaconTxt;
static lv_obj_t *btnHapticSettings, *lblHapticSettings;
static lv_obj_t *btnHapticLevel, *lblHapticLevel;
static lv_obj_t *btnHapticMounting, *lblHapticMounting;
static lv_obj_t *btnHapticTest, *lblHapticTest;
static lv_obj_t *btnSettingsCamera, *lblSettingsCameraState;
static lv_obj_t *btnSettingsControls, *lblSettingsControlsState;
static lv_obj_t *btnSettingsDisplay, *lblSettingsDisplayState;
static lv_obj_t *btnSettingsFeedback, *lblSettingsFeedbackState;
static lv_obj_t *btnSettingsSystem, *lblSettingsSystemState;
static lv_obj_t *btnSettingsHand, *lblSettingsHandState;
static lv_obj_t *btnSettingsJog, *lblSettingsJogState;
static lv_obj_t *btnSettingsDirection, *lblSettingsDirectionState;
static lv_obj_t *btnSettingsPower, *lblSettingsPower;
// Camera discovery/selection and link evidence are separate views. A bounded
// list makes the Core2 a standalone device manager; the progress view remains
// available without exposing the camera's Wi-Fi key.
enum CameraView : uint8_t {
    CAM_DEVICE_HOME = 0, CAM_HUB, CAM_PROGRESS, CAM_SCAN_CONFIRM,
    CAM_SWITCH_CONFIRM, CAM_FORGET_CONFIRM, CAM_TOOLS, CAM_POWER
};
// The header names the DEVICE sub-view for the same reason it names a
// settings sub-page: the left key becomes BACK, and BACK must never be
// ambiguous about what it is leaving.
static const char *CAMERA_VIEW_NAMES[] = {
    "DEVICE", "CAMERAS", "PAIR CAMERA", "SCAN?", "SWITCH?", "FORGET?", "CAMERA",
    "POWER"
};
static CameraView cameraView = CAM_DEVICE_HOME;
// Only a navigation breadcrumb for Settings shortcuts. The DEVICE workspace
// remains the one camera manager and owns all connection state and actions.
static bool cameraReturnToSettings = false;
static osmo::CameraCatalog cameraCatalogUi;
static uint32_t cameraHighlightedId = 0;
static uint32_t cameraConfirmId = 0;
static uint32_t cameraForgetHoldStartedMs = 0;
static bool cameraForgetTriggered = false;
static lv_obj_t *cameraHub, *cameraProgress, *cameraConfirm;
static lv_obj_t *deviceHome, *deviceTools, *devicePower;
static lv_obj_t *lblDeviceSummary, *lblDeviceCapability, *lblToolResult;
static lv_obj_t *deviceEvidencePanel;
static lv_obj_t *lblDeviceEvidenceName[5], *lblDeviceEvidenceState[5];
static lv_obj_t *btnDeviceCameras, *lblDeviceCameras;
static lv_obj_t *btnDeviceTools, *lblDeviceTools;
static lv_obj_t *btnDevicePower, *lblDevicePower;
static lv_obj_t *btnFocusAfS, *lblFocusAfS, *btnFocusAfC, *lblFocusAfC;
static lv_obj_t *btnGimbalCenter, *lblGimbalCenter;
static lv_obj_t *btnGimbalFollow, *lblGimbalFollow;
static lv_obj_t *focusTrack, *lblFocusMarks;
static lv_obj_t *btnLinkDisconnect, *lblLinkDisconnect;
static lv_obj_t *btnLinkReconnect, *lblLinkReconnect;
static lv_obj_t *btnPowerOff, *lblPowerOff;
static lv_obj_t *cameraList, *lblCameraEmpty;
static lv_obj_t *cameraRows[osmo::CAMERA_CATALOG_CAPACITY];
static lv_obj_t *lblCameraRowName[osmo::CAMERA_CATALOG_CAPACITY];
static lv_obj_t *lblCameraRowState[osmo::CAMERA_CATALOG_CAPACITY];
static lv_obj_t *btnCameraScan, *lblCameraScan;
static lv_obj_t *btnCameraConnect, *lblCameraConnect;
static lv_obj_t *btnCameraForget, *lblCameraForget;
static lv_obj_t *lblCameraConfirmTitle, *lblCameraConfirmBody;
static lv_obj_t *btnCameraConfirmCancel, *lblCameraConfirmCancel;
static lv_obj_t *btnCameraConfirmAction, *lblCameraConfirmAction;
static lv_obj_t *btnCameraProgressBack, *lblCameraProgressBack;
// Wi-Fi is one stage of a selected camera's connection workflow, not a
// generic network setting. Keep its evidence visible as an ordered rail.
static lv_obj_t *lblWifiSteps, *lblWifiState, *lblWifiSsid, *lblWifiDetail;
static lv_obj_t *btnWifiRetry, *lblWifiRetry, *lblSysUptime, *lblBattery;

// Defined beside showPage; the menu rows and the back key both need it.
static void showSettings(uint8_t which);
static void showCameraView(CameraView which);
static void showPage(uint8_t idx);
static void updatePageTitle();
static void updateSoftKeys();
static lv_obj_t *headerContext, *btnJogFeel, *lblPageName;
// A fault is global, but the only FAULT word on the panel lived on the HAND
// dial, so lost authority on JOG, MOVES, DEVICE or SETTINGS showed on the LED
// bars and nowhere on the screen. This strip belongs to the screen, not a page.
static lv_obj_t *faultBar, *lblFaultTxt;
static lv_obj_t *softL, *softM, *softR, *softLTxt, *softMTxt, *softRTxt;
static lv_obj_t *softReturnIcon;
static const lv_point_t RETURN_ICON_POINTS[] = {
    {22, 0}, {22, 10}, {0, 10}, {6, 4}, {0, 10}, {6, 16}
};
// Retained as a hidden compatibility label while the middle rail is the large
// HOME/HAND/JOG action on every page.
static lv_obj_t *lblRail;
// One immutable 320x240 canvas: 40 px evidence header, 168 px workspace and
// 32 px navigation rail. Keeping the arithmetic named prevents a page from
// drifting beneath the global STOP or the bezel-aligned navigation targets.
#define HEADER_H  40
#define PAGE_H   168
#define NAV_Y    208

enum class Workspace : uint8_t {
    Jog = 0, Hand, Moves, Device, Settings, Count
};
static const char *WORKSPACE_NAMES[] = {
    "JOG", "HAND", "MOVES", "DEVICE", "SETTINGS"
};
static uint8_t page = static_cast<uint8_t>(Workspace::Hand);
static const uint8_t PAGE_COUNT = 5;
static lv_obj_t *lblCtrl, *lblBatteryHeader;
static lv_obj_t *btnRecord, *lblRecordTxt;

// -- follow page ---------------------------------------------------------
static lv_obj_t *dial, *lblState, *lblSub;
static lv_obj_t *lblHandDelta, *lblHandMode, *handCommandBar;
static lv_obj_t *handReticle, *handReticleH, *handReticleV, *handImuDot;
static lv_obj_t *handFeelSheet, *handAxisTuneSheet;
// Locking an axis for a clean pan is a shot-time decision. It lives on the
// FEEL sheets, one tap from the shot, not three taps deep in SETTINGS.
static lv_obj_t *btnHandAxisTune, *lblHandAxisTune;
static lv_obj_t *btnTiltResponse, *lblTiltResponse;
static lv_obj_t *btnPanResponse, *lblPanResponse;
static lv_obj_t *btnTiltStability, *lblTiltStability;
static lv_obj_t *btnPanStability, *lblPanStability;
static lv_obj_t *btnHandDirection, *lblHandDirection;
static lv_obj_t *btnHandCenter, *lblHandCenter;
// The shot surface keeps one narrow, full-height axis preset on either side.
// These are modes (TILT ONLY / PAN ONLY), not tiny lock toggles: selecting the
// active mode again returns to BOTH axes live.
static lv_obj_t *btnHandTiltOnly, *lblHandTiltOnly;
static lv_obj_t *btnHandPanOnly, *lblHandPanOnly;
static bool handFeelOpen = false;
enum class HandFeelView : uint8_t { Main = 0, AxisTune };
static HandFeelView handFeelView = HandFeelView::Main;
static lv_obj_t *btnFollowTemplate, *lblFollowTemplateTxt;
static lv_obj_t *btnFollowSensitivity, *lblFollowSensitivityTxt;
static lv_obj_t *btnFollowSmooth, *lblFollowSmoothTxt;

// -- direction page ------------------------------------------------------
static lv_obj_t *btnInvT, *lblInvTTxt, *btnInvP, *lblInvPTxt;
static lv_obj_t *btnResetFeel, *lblResetFeelTxt;
static lv_obj_t *btnStop, *lblStopTxt;      // pinned, drawn over every page

// -- drive page ----------------------------------------------------------
static lv_obj_t *jogPad, *jogKnob, *jogDeadzone, *jogHomeDot;
static lv_obj_t *jogCrossH, *jogCrossV, *lblJog;
static lv_obj_t *jogFeelSheet, *lblJogEvidence, *lblJogPanEvidence;
static lv_obj_t *lblJogTravel;
static lv_obj_t *btnJogTilt, *lblJogTiltTxt, *btnJogPan, *lblJogPanTxt;
static lv_obj_t *btnJogCenter, *lblJogCenter;
static lv_obj_t *btnJogDirection, *lblJogDirection;
static lv_obj_t *btnJogTiltOnly, *lblJogTiltOnly;
static lv_obj_t *btnJogPanOnly, *lblJogPanOnly;
// The idle lesson on the pad is for the first shoots. After enough pickups it
// is noise over the surface, so it fades to the one line that still matters.
static uint8_t jogPickups = 0;
static const uint8_t JOG_LESSON_PICKUPS = 20;
static lv_obj_t *btnJogSpeed, *lblJogSpeedTxt;
static lv_obj_t *btnJogSmooth, *lblJogSmoothTxt;
static lv_obj_t *driveLimitUp, *driveLimitDown, *driveLimitLeft, *driveLimitRight;
static bool jogFeelOpen = false;
static bool jogOriginValid = false;
static float jogOriginX = 0.0f, jogOriginY = 0.0f;

// -- moves page ----------------------------------------------------------
static lv_obj_t *btnMotionPrev, *lblMotionPrev;
static lv_obj_t *btnMotionSlot, *lblMotionSlot;
static lv_obj_t *btnMotionRepeat, *lblMotionRepeat;
static lv_obj_t *btnMotionSelected, *lblMotionSelected;
static lv_obj_t *btnMotionNext, *lblMotionNext;
static lv_obj_t *btnMotionCapture, *lblMotionCapture;
static lv_obj_t *btnMotionClear, *lblMotionClear;
static lv_obj_t *btnMotionGoto, *lblMotionGoto;
static lv_obj_t *btnMotionTime, *lblMotionTime;
static lv_obj_t *btnMotionDwell, *lblMotionDwell;
static lv_obj_t *btnMotionRun, *lblMotionRun;
static lv_obj_t *btnMotionHome, *lblMotionHome;
static lv_obj_t *btnMotionOpenRun, *lblMotionOpenRun;
static lv_obj_t *motionRunPane, *lblMotionRunSummary, *lblMotionRunSub;
static lv_obj_t *motionProgressBar;
static lv_obj_t *lblMotionElapsed, *lblMotionLegProgress, *lblMotionCueCount;
static lv_obj_t *btnMotionEdit, *lblMotionEdit;
static lv_obj_t *batteryRefusalPane, *lblBatteryRefusalPct;
static lv_obj_t *btnRefusalHand, *lblRefusalHand;
// MOVES has three deliberate surfaces: a glanceable gallery, one spacious
// point editor, and the existing sparse runner. A point is never edited in a
// cramped grid cell.
enum class MotionUiView : uint8_t { Gallery = 0, Point, Run };
static MotionUiView motionUiView = MotionUiView::Gallery;
static void setMotionUiView(MotionUiView view);
static lv_obj_t *motionGalleryPane, *motionPointPane;
static lv_obj_t *motionTiles[osmo::MOTION_POINT_CAPACITY];
static lv_obj_t *lblMotionTiles[osmo::MOTION_POINT_CAPACITY];
static lv_obj_t *lblMotionPointPose, *lblMotionPointLeg;
static lv_obj_t *btnMotionPointCapture, *lblMotionPointCapture;
static lv_obj_t *btnMotionPointTransition, *lblMotionPointTransition;
static lv_obj_t *btnMotionPointTime, *lblMotionPointTime;
static lv_obj_t *btnMotionPointDwell, *lblMotionPointDwell;
static lv_obj_t *btnMotionPointGoto, *lblMotionPointGoto;
static lv_obj_t *btnMotionPointClear, *lblMotionPointClear;
static lv_obj_t *btnMotionPointRemove, *lblMotionPointRemove;
static bool removeSingleHold = false;
static uint8_t motionSelected = 0;
static bool motionRunView = false;
static uint32_t motionHoldStartedMs = 0;
static bool motionHoldTriggered = false;
static bool motionHoldIsRun = false;
// Standalone motion exposes no elapsed field, but the Core2 owns the runtime
// clock. Count only active, non-cued milliseconds; freeze while a human cue
// holds the frame. This is wall-clock evidence, never tracking error.
static uint32_t motionElapsedMs = 0;
static uint32_t motionElapsedTickMs = 0;
static uint16_t motionCueCount = 0;
static uint8_t motionLastCuePoint = 0;
static bool motionWasActive = false;
static bool motionWasArmed = false;

// -- setup page ----------------------------------------------------------
static lv_obj_t *btnScreenTo, *lblScreenToTxt;
static lv_obj_t *btnBright, *lblBrightTxt, *btnSilent, *lblSilentTxt;
static lv_obj_t *lblFw;
static lv_obj_t *faultPane, *lblFaultReason, *lblFaultRetry, *lblFaultPath;
static lv_obj_t *btnFaultDevice, *lblFaultDevice;

static bool screenClutchHeld = false;
static uint32_t recordHoldStartedMs = 0;
static bool recordHoldTriggered = false;
// The transition into a rolling state is observable locally, so a take timer
// needs no protocol work at all.
static uint32_t recordRollingSinceMs = 0;
// STOP is styled from this: a press that changes nothing else still has to be
// visibly received.
static uint32_t stopPressedAtMs = 0;
// Clearing a captured waypoint and wiping the feel profile both cost real
// setup time. Every other destructive action on this panel is a hold.
static uint32_t clearHoldStartedMs = 0;
static bool clearHoldTriggered = false;
static uint32_t resetHoldStartedMs = 0;
static bool resetHoldTriggered = false;
static uint32_t centerHoldStartedMs = 0;
static bool centerHoldTriggered = false;
static uint32_t homeHoldStartedMs = 0;
static bool homeHoldTriggered = false;
// Seven different hold lengths taught the thumb nothing. Three classes:
// an action that moves or changes the head, an action that destroys setup,
// and the one hold that tears the radio down. REC keeps its own, shorter
// hold: a tally that lags the slate by most of a second is a missed slate.
static const uint32_t HOLD_ACT_MS = 700;      // GOTO, ARM, GO, CENTER, P1, TL STOP
static const uint32_t HOLD_DESTROY_MS = 800;  // CLEAR, FORGET, RESET FEEL
static const uint32_t HOLD_POWER_MS = 1200;   // POWER OFF
static const uint32_t HOLD_RECORD_MS = 450;
// A brownout mid-move is a safety event on a rig that moves. Below this the
// panel refuses to ARM a programme; the clutch stays available because a
// hand on the box is not a promise about where the head will be in 30 s.
static const int BATTERY_ARM_FLOOR = 10;
static int batteryPct = -1;
static bool batteryCharging = false;
static uint32_t powerOffHoldStartedMs = 0;
static bool powerOffTriggered = false;
static uint32_t powerOffAtMs = 0;

// LVGL does not promise RELEASED or PRESS_LOST when a pressed parent is
// hidden.  Never let elapsed time from an abandoned page-local hold carry
// into the next press.  REC is deliberately excluded: its header control
// remains visible across page changes and owns its complete event lifecycle.
static void cancelPageLocalHolds() {
    clearHoldStartedMs = 0;
    clearHoldTriggered = false;
    resetHoldStartedMs = 0;
    resetHoldTriggered = false;
    centerHoldStartedMs = 0;
    centerHoldTriggered = false;
    homeHoldStartedMs = 0;
    homeHoldTriggered = false;
    motionHoldStartedMs = 0;
    motionHoldTriggered = false;
    motionHoldIsRun = false;
    cameraForgetHoldStartedMs = 0;
    cameraForgetTriggered = false;
    powerOffHoldStartedMs = 0;
    if (!powerOffAtMs) powerOffTriggered = false;
}

// A capacitive panel reports brief dropouts as a finger flexes, and LVGL turns
// each into PRESS_LOST -> PRESSED. Undebounced that became grab/release
// chatter, which on the rig means the head repeatedly seizing and letting go.
static const uint32_t CLUTCH_RELEASE_DEBOUNCE_MS = 140;
static uint32_t clutchLastContactMs = 0;

// -- palette --------------------------------------------------------------
// Canvas semantic tokens. Passive selection stays white; green is reserved
// for released cues and affirmative connection evidence, never mere health.
#define C_BG        0x05070A
#define C_TEXT      0xF2F4F5
#define C_DIM       0x8D969D
#define C_FAINT     0x39414A
#define C_CYAN      0x3DD6F5   // live control or commanded motion
#define C_AMBER     0xFFB020   // armed, constrained, waiting, near a limit
#define C_GREEN     0x4CE05B   // cue release or affirmative system evidence
#define C_MAGENTA   0xE04CFF   // fault, lost authority, invalid state
#define C_REC       0xFF453A   // authoritative REC tally, nothing else
#define C_PANEL     0x161C23
#define C_EDGE      0x232B34
#define C_SURF_HI   0x1F262E   // raised surface
#define C_SURF_LO   0x11161C   // recessed well

// The canvas references DM Mono / IBM Plex sizes. Built-in Montserrat avoids
// generated-font dependencies; odd sizes round upward exactly once here.
#define FONT_10 &lv_font_montserrat_10
#define FONT_12 &lv_font_montserrat_12
#define FONT_14 &lv_font_montserrat_14
#define FONT_16 &lv_font_montserrat_16
#define FONT_18 &lv_font_montserrat_18
#define FONT_20 &lv_font_montserrat_20
#define FONT_22 &lv_font_montserrat_22
#define FONT_26 &lv_font_montserrat_26
#define FONT_30 &lv_font_montserrat_30
#define FONT_40 &lv_font_montserrat_40

// One place, so the hello line and the setup page can never disagree.
#define FW_VERSION "7.9.1-return-icon"

// Top speed for the head, named rather than numbered. These mirror
// driver/response.py SPEED_CAPS; the box only sends the name and the panel
// owns the actual degrees per second.
static const char *SPEED_NAMES[] = {"SLOW", "NORMAL", "FAST"};
static uint8_t speedIdx = 1;
static const char *SMOOTH_NAMES[] = {"CRISP", "FLUID", "GLIDE"};
static uint8_t smoothIdx = 1;
static uint8_t jogSpeedIdx = 1;
static uint8_t jogSmoothIdx = 1;
// Response is the axis leverage the operator feels. Stability is the amount
// of acceleration/jerk authority that axis receives after the shared motion
// conditioner: QUIET rejects more hand tremor, RESPONSIVE follows intent
// sooner, and BALANCED is bit-for-bit the established control law.
static const char *AXIS_RESPONSE_NAMES[] = {"FINE", "BALANCED", "DIRECT"};
static const char *AXIS_GAIN_LABELS[] = {"0.25x / FINE", "0.50x", "1.00x"};
static const char *AXIS_RESPONSE_SHORT_NAMES[] = {"FINE", "BAL", "DIRECT"};
static const char *AXIS_DAMPING_NAMES[] = {"STRONG", "MEDIUM", "LIGHT"};
// Preserve the established USB action vocabulary while the screen uses the
// clearer operator-facing word DAMPING.
static const char *AXIS_STABILITY_WIRE_NAMES[] = {"QUIET", "BALANCED", "RESPONSIVE"};
static const char *REPEAT_NAMES[] = {"ONCE", "LOOP", "BOUNCE"};
static const char *SLOT_NAMES[] = {"A", "B"};
static uint8_t tiltResponseIdx = 1;
static uint8_t panResponseIdx = 1;
static uint8_t tiltStabilityIdx = 1;
static uint8_t panStabilityIdx = 1;
static const char *TEMPLATE_NAMES[] = {"HAND FOLLOW", "GYRO RATE", "AIR MOUSE"};
static const char *TEMPLATE_SHORT_NAMES[] = {"FOLLOW", "RATE", "AIR"};
static const char *TEMPLATE_LOG_NAMES[] = {"follow", "rate", "air_mouse"};
static const uint8_t TEMPLATE_COUNT = 3;
static uint8_t templateIdx = 0;

// Per-axis inversion. Whether a rotation of the hand should raise or lower the
// frame is a preference, not a fact -- it depends on whether the operator
// thinks of the box as the camera or as a window onto it. Both camps exist and
// neither is wrong, so it is a switch.
static bool invertTilt = false;
static bool invertPan  = false;

// Operator feel is equipment setup, not session state. Keep it in a dedicated
// non-secret NVS namespace so a battery swap does not silently change muscle
// memory. Axis locks are intentionally excluded: they are transient safety
// constraints and always boot LIVE.
static Preferences feelPreferences;
static bool feelPreferencesReady = false;
static const uint16_t EASE_MS[] = {0, 100, 200, 350, 500, 750};
// Mode: HAND/JOG. Global start/end, tilt start/end, pan start/end, override bits.
static uint8_t easeValues[2][7] = {{0,2,0,2,0,2,0},{0,2,0,2,0,2,0}};
static bool easeDirty = true, easeOpen = false, easeSaveError = false;
static uint8_t easeMode = 0;
static int8_t easeAxis = -1;
static lv_obj_t *easePane, *easeStartLabel, *easeEndLabel, *easeSourceLabel;
static lv_obj_t *easeScopeLabel, *easeHelpLabel, *easeSourceButton;
static lv_obj_t *easeGearLabels[4], *easeGlobalLabels[2];
static void refreshEasePane();
static void closeEasePane();


static void loadFeelPreferences() {
    feelPreferencesReady = feelPreferences.begin("core2-feel", false);
    if (!feelPreferencesReady) return;
    uint8_t storedEase[2][7];
    if (feelPreferences.getBytesLength("ease_v1") == sizeof(storedEase) &&
        feelPreferences.getBytes("ease_v1", storedEase, sizeof(storedEase)) == sizeof(storedEase)) {
        bool valid = true;
        for (uint8_t m=0; m<2; ++m) {
            for (uint8_t i=0; i<6; ++i) valid &= storedEase[m][i] < 6;
            valid &= storedEase[m][6] < 4;
        }
        if (valid) memcpy(easeValues, storedEase, sizeof(easeValues));
    }
    // Upgrade without changing muscle memory: the former global sensitivity
    // seeds both axes until an operator deliberately tunes either one.
    const uint8_t legacySensitivity =
        min<uint8_t>(feelPreferences.getUChar("sens", 1), 2);
    tiltResponseIdx = min<uint8_t>(
        feelPreferences.getUChar("tilt_rsp", legacySensitivity), 2);
    panResponseIdx = min<uint8_t>(
        feelPreferences.getUChar("pan_rsp", legacySensitivity), 2);
    tiltStabilityIdx = min<uint8_t>(
        feelPreferences.getUChar("tilt_stab", 1), 2);
    panStabilityIdx = min<uint8_t>(
        feelPreferences.getUChar("pan_stab", 1), 2);
    speedIdx       = min<uint8_t>(feelPreferences.getUChar("speed", 1), 2);
    smoothIdx      = min<uint8_t>(feelPreferences.getUChar("smooth", 1), 2);
    jogSpeedIdx    = min<uint8_t>(feelPreferences.getUChar("jog_spd", speedIdx), 2);
    jogSmoothIdx   = min<uint8_t>(feelPreferences.getUChar("jog_ramp", smoothIdx), 2);
    templateIdx    = min<uint8_t>(feelPreferences.getUChar("template", 0),
                                  TEMPLATE_COUNT - 1);
    invertTilt     = feelPreferences.getBool("inv_t", false);
    invertPan      = feelPreferences.getBool("inv_p", false);
    // 7.0 corrects the HAND body-frame basis. Older builds made operators
    // enable both INV switches merely to obtain natural motion, so carrying
    // those bits forward would re-introduce the bug after the math is fixed.
    // Establish one clean semantic baseline once: NORM is natural, INV is an
    // intentional per-axis reversal from then on.
    if (!feelPreferences.getBool("hand_nat_v2", false)) {
        invertTilt = false;
        invertPan = false;
        feelPreferences.putBool("inv_t", false);
        feelPreferences.putBool("inv_p", false);
        feelPreferences.putBool("hand_nat_v2", true);
    }
}

static void persistFeelByte(const char *key, uint8_t value) {
    if (feelPreferencesReady) feelPreferences.putUChar(key, value);
}

static void persistFeelBool(const char *key, bool value) {
    if (feelPreferencesReady) feelPreferences.putBool(key, value);
}

static void resetFeelPreferences() {
    tiltResponseIdx = 1;
    panResponseIdx = 1;
    tiltStabilityIdx = 1;
    panStabilityIdx = 1;
    speedIdx = 1;
    smoothIdx = 1;
    jogSpeedIdx = 1;
    jogSmoothIdx = 1;
    templateIdx = 0;
    invertTilt = false;
    invertPan = false;
    if (!feelPreferencesReady) return;
    // Keep the legacy value neutral so an older image also starts safely.
    feelPreferences.putUChar("sens", 1);
    feelPreferences.putUChar("tilt_rsp", tiltResponseIdx);
    feelPreferences.putUChar("pan_rsp", panResponseIdx);
    feelPreferences.putUChar("tilt_stab", tiltStabilityIdx);
    feelPreferences.putUChar("pan_stab", panStabilityIdx);
    feelPreferences.putUChar("speed", speedIdx);
    feelPreferences.putUChar("smooth", smoothIdx);
    feelPreferences.putUChar("jog_spd", jogSpeedIdx);
    feelPreferences.putUChar("jog_ramp", jogSmoothIdx);
    feelPreferences.putUChar("template", templateIdx);
    feelPreferences.putBool("inv_t", invertTilt);
    feelPreferences.putBool("inv_p", invertPan);
}

static uint8_t brightIdx = 1;          // 0 day, 1 interior, 2 night, 3 blackout
static const char *BRIGHT_NAMES[] = {"DAY", "INTERIOR", "NIGHT", "BLACKOUT"};

// Display, lights and sound are equipment setup in exactly the same sense as
// feel. The screen timeout exists because operators switch a timeout off when
// it misbehaves; handing it back on the next battery swap recreates the very
// problem it was added to solve. Kept in their own namespace so RESET FEEL
// DEFAULTS cannot quietly take the display with it.
static Preferences uiPreferences;
static bool uiPreferencesReady = false;

static void loadUiPreferences() {
    uiPreferencesReady = uiPreferences.begin("core2-ui", false);
    if (!uiPreferencesReady) return;
    brightIdx = min<uint8_t>(uiPreferences.getUChar("bright", brightIdx), 3);
    screenTimeoutIdx =
        min<uint8_t>(uiPreferences.getUChar("scr_to", screenTimeoutIdx), 3);
    beaconEnabled    = uiPreferences.getBool("beacon", beaconEnabled);
    limitLedsEnabled = uiPreferences.getBool("lim_led", limitLedsEnabled);
    cameraInHand     = uiPreferences.getBool("in_hand", cameraInHand);
    hapticLevelIdx   = min<uint8_t>(uiPreferences.getUChar("hap_lvl", hapticLevelIdx),
                                    HAPTIC_FIRM);
    jogPickups       = uiPreferences.getUChar("jog_pick", 0);
    blackout         = (brightIdx == 3);
    // Routed through the setter so the flag and the muted speaker cannot
    // disagree, which is the bug that made SOUND ON decorative for months.
    setSoundEnabled(uiPreferences.getBool("sound", soundEnabled));
}

static void persistUiByte(const char *key, uint8_t value) {
    if (uiPreferencesReady) uiPreferences.putUChar(key, value);
}

static void persistUiBool(const char *key, bool value) {
    if (uiPreferencesReady) uiPreferences.putBool(key, value);
}
static const uint8_t BRIGHT_LCD[]  = {255, 160, 70, 18};

static void flushCb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *px) {
    const int32_t w = area->x2 - area->x1 + 1;
    const int32_t h = area->y2 - area->y1 + 1;
    M5.Display.startWrite();
    M5.Display.setAddrWindow(area->x1, area->y1, w, h);
    M5.Display.writePixels((uint16_t *)px, w * h, true);
    M5.Display.endWrite();
    lv_disp_flush_ready(drv);
}

static void touchCb(lv_indev_drv_t *, lv_indev_data_t *data) {
    auto t = M5.Touch.getDetail();
    if (t.isPressed()) {
        noteScreenActivity(millis());
        if (swallowWake) {
            // LVGL never sees this press at all, so nothing under the finger
            // can fire. Cleared on finger-up below, so the swallow lasts
            // exactly one touch and the screen is not left dead.
            data->state = LV_INDEV_STATE_RELEASED;
            return;
        }
        data->state   = LV_INDEV_STATE_PRESSED;
        data->point.x = t.x;
        data->point.y = t.y;
    } else {
        swallowWake = false;
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

// ---------------------------------------------------------------- handlers
static void stopPressed(lv_event_t *) {
    stopPressedAtMs = millis();
    clutchReleaseRequired = true;
    clutchReleaseCandidateMs = 0;
    if (clutchHeld) {
        clutchHeld = false;
        osmo::directCamera.setClutch(false);
        say("E 0 reason=stop");
    }
    if (jogRawContact || jogActive) {
        jogReleaseRequired = true;
        jogReleaseCandidateMs = 0;
    }
    jogActive = false;
    jogX = jogY = 0;
    jogOriginValid = false;
    if (jogKnob) lv_obj_add_flag(jogKnob, LV_OBJ_FLAG_HIDDEN);
    if (jogDeadzone) lv_obj_add_flag(jogDeadzone, LV_OBJ_FLAG_HIDDEN);
    osmo::directCamera.abort();
    say("B abort");
    tick(90);
}
static void tiltLockPressed(lv_event_t *) {
    if (rig.direct) rig.lockTilt = !rig.lockTilt;
    say("B tilt_lock");
    tick(20);
}
static void panLockPressed(lv_event_t *)  {
    if (rig.direct) rig.lockPan = !rig.lockPan;
    say("B pan_lock");
    tick(20);
}

// Apply an axis preset atomically from the operator's point of view. The
// direct runtime sees the local state in this frame; a USB host receives only
// the lock toggles whose state actually needs to change.
static void applyAxisLocks(bool lockTilt, bool lockPan) {
    const osmo::DirectStatus motion = osmo::directCamera.status();
    if (motion.motionProgramArmed || motion.motionProgramActive) return;
    if (rig.lockTilt != lockTilt) {
        if (rig.direct) rig.lockTilt = lockTilt;
        say("B tilt_lock");
    }
    if (rig.lockPan != lockPan) {
        if (rig.direct) rig.lockPan = lockPan;
        say("B pan_lock");
    }
    tick(24);
}

static void tiltOnlyPressed(lv_event_t *) {
    const bool alreadyTiltOnly = !rig.lockTilt && rig.lockPan;
    applyAxisLocks(false, alreadyTiltOnly ? false : true);
}

static void panOnlyPressed(lv_event_t *) {
    const bool alreadyPanOnly = rig.lockTilt && !rig.lockPan;
    applyAxisLocks(alreadyPanOnly ? false : true, false);
}

static void clutchEvent(lv_event_t *e) {
    const lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_PRESSED || code == LV_EVENT_PRESSING) {
        screenClutchHeld = true;
    } else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        screenClutchHeld = false;
    }
}

static void tiltResponsePressed(lv_event_t *) {
    if (screenClutchHeld || clutchHeld) return;
    tiltResponseIdx = (tiltResponseIdx + 1) % 3;
    persistFeelByte("tilt_rsp", tiltResponseIdx);
    say("B tilt_response_%s", AXIS_RESPONSE_NAMES[tiltResponseIdx]);
    tick(20);
}

static void panResponsePressed(lv_event_t *) {
    if (screenClutchHeld || clutchHeld) return;
    panResponseIdx = (panResponseIdx + 1) % 3;
    persistFeelByte("pan_rsp", panResponseIdx);
    say("B pan_response_%s", AXIS_RESPONSE_NAMES[panResponseIdx]);
    tick(20);
}

static void tiltStabilityPressed(lv_event_t *) {
    if (screenClutchHeld || clutchHeld) return;
    tiltStabilityIdx = (tiltStabilityIdx + 1) % 3;
    persistFeelByte("tilt_stab", tiltStabilityIdx);
    say("B tilt_stability_%s", AXIS_STABILITY_WIRE_NAMES[tiltStabilityIdx]);
    tick(20);
}

static void panStabilityPressed(lv_event_t *) {
    if (screenClutchHeld || clutchHeld) return;
    panStabilityIdx = (panStabilityIdx + 1) % 3;
    persistFeelByte("pan_stab", panStabilityIdx);
    say("B pan_stability_%s", AXIS_STABILITY_WIRE_NAMES[panStabilityIdx]);
    tick(20);
}

static void speedPressed(lv_event_t *) {
    speedIdx = (speedIdx + 1) % 3;
    persistFeelByte("speed", speedIdx);
    say("B speed_%s", SPEED_NAMES[speedIdx]);
    tick(20);
}

static void smoothPressed(lv_event_t *) {
    smoothIdx = (smoothIdx + 1) % 3;
    persistFeelByte("smooth", smoothIdx);
    say("B smooth_%s", SMOOTH_NAMES[smoothIdx]);
    tick(20);
}

static void jogSpeedPressed(lv_event_t *) {
    jogSpeedIdx = (jogSpeedIdx + 1) % 3;
    persistFeelByte("jog_spd", jogSpeedIdx);
    say("B jog_speed_%s", SPEED_NAMES[jogSpeedIdx]);
    tick(20);
}

static void jogSmoothPressed(lv_event_t *) {
    jogSmoothIdx = (jogSmoothIdx + 1) % 3;
    persistFeelByte("jog_ramp", jogSmoothIdx);
    say("B jog_smooth_%s", SMOOTH_NAMES[jogSmoothIdx]);
    tick(20);
}

static void templatePressed(lv_event_t *) {
    templateIdx = (templateIdx + 1) % TEMPLATE_COUNT;
    persistFeelByte("template", templateIdx);
    say("B template_%s", TEMPLATE_LOG_NAMES[templateIdx]);
    tick(20);
}

// Shooting pages devote the entire content rectangle to live control.  Feel
// changes live in an explicit sheet opened from the large left header target;
// they never steal a corner of the movement surface or fire under a moving
// thumb.
static void setJogFeelOpen(bool open) {
    if (!jogFeelSheet || !jogPad) return;
    if (open && (jogRawContact || jogActive)) return;
    if (jogFeelOpen != open) cancelPageLocalHolds();
    jogFeelOpen = open;
    if (open) {
        lv_obj_add_flag(jogPad, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(jogFeelSheet, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(jogFeelSheet, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(jogPad, LV_OBJ_FLAG_HIDDEN);
    }
    updatePageTitle();
    updateSoftKeys();
}

static void setHandFeelOpen(bool open) {
    if (!handFeelSheet || !handAxisTuneSheet || !dial) return;
    if (open && (screenClutchHeld || clutchHeld)) return;
    if (handFeelOpen != open) cancelPageLocalHolds();
    handFeelOpen = open;
    if (open) {
        handFeelView = HandFeelView::Main;
        lv_obj_add_flag(dial, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(handFeelSheet, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(handAxisTuneSheet, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(handFeelSheet, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(handAxisTuneSheet, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(dial, LV_OBJ_FLAG_HIDDEN);
        handFeelView = HandFeelView::Main;
    }
    updatePageTitle();
    updateSoftKeys();
}

static void setHandFeelView(HandFeelView view) {
    if (!handFeelOpen || !handFeelSheet || !handAxisTuneSheet) return;
    if (screenClutchHeld || clutchHeld) return;
    if (handFeelView != view) cancelPageLocalHolds();
    handFeelView = view;
    if (view == HandFeelView::AxisTune) {
        lv_obj_add_flag(handFeelSheet, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(handAxisTuneSheet, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(handAxisTuneSheet, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(handFeelSheet, LV_OBJ_FLAG_HIDDEN);
    }
    updatePageTitle();
    updateSoftKeys();
}

static void handAxisTunePressed(lv_event_t *) {
    setHandFeelView(HandFeelView::AxisTune);
    tick(18);
}

static void headerContextPressed(lv_event_t *) {
    if (easeOpen) return;
    if (rig.fault[0]) return;
    if (page == static_cast<uint8_t>(Workspace::Jog)) {
        setJogFeelOpen(!jogFeelOpen);
        tick(16);
    } else if (page == static_cast<uint8_t>(Workspace::Hand)) {
        if (handFeelOpen && handFeelView == HandFeelView::AxisTune)
            setHandFeelView(HandFeelView::Main);
        else
            setHandFeelOpen(!handFeelOpen);
        tick(16);
    } else if (page == static_cast<uint8_t>(Workspace::Moves)) {
        const osmo::DirectStatus status = osmo::directCamera.status();
        if (motionUiView == MotionUiView::Gallery) {
            setMotionUiView(MotionUiView::Run);
        } else if (!(status.motionProgramArmed || status.motionProgramActive)) {
            setMotionUiView(MotionUiView::Gallery);
        }
        tick(16);
    }
}

static void setMotionUiView(MotionUiView view) {
    if (motionUiView != view) cancelPageLocalHolds();
    motionUiView = view;
    motionRunView = view == MotionUiView::Run;
    if (motionGalleryPane) {
        if (view == MotionUiView::Gallery)
            lv_obj_clear_flag(motionGalleryPane, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_add_flag(motionGalleryPane, LV_OBJ_FLAG_HIDDEN);
    }
    if (motionPointPane) {
        if (view == MotionUiView::Point)
            lv_obj_clear_flag(motionPointPane, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_add_flag(motionPointPane, LV_OBJ_FLAG_HIDDEN);
    }
    if (motionRunPane) {
        if (view == MotionUiView::Run)
            lv_obj_clear_flag(motionRunPane, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_add_flag(motionRunPane, LV_OBJ_FLAG_HIDDEN);
    }
    updatePageTitle();
    updateSoftKeys();
}

static const char *motionTransitionName(osmo::MotionTransition transition) {
    switch (transition) {
        case osmo::MotionTransition::Linear:  return "LINEAR";
        case osmo::MotionTransition::EaseIn:  return "EASE IN";
        case osmo::MotionTransition::EaseOut: return "EASE OUT";
        case osmo::MotionTransition::Smooth:
        default:                              return "SMOOTH";
    }
}

static const char *motionTransitionShort(osmo::MotionTransition transition) {
    switch (transition) {
        case osmo::MotionTransition::Linear:  return "LIN";
        case osmo::MotionTransition::EaseIn:  return "IN";
        case osmo::MotionTransition::EaseOut: return "OUT";
        case osmo::MotionTransition::Smooth:
        default:                              return "SM";
    }
}

static void motionTilePressed(lv_event_t *e) {
    const uint8_t selected = static_cast<uint8_t>(
        reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));
    const osmo::MotionProgram program = osmo::directCamera.motionProgram();
    if (selected > program.count || selected >= osmo::MOTION_POINT_CAPACITY)
        return;
    motionSelected = selected;
    setMotionUiView(MotionUiView::Point);
    tick(16);
}

static void motionPrevPressed(lv_event_t *) {
    if (motionSelected > 0) --motionSelected;
    tick(12);
}

static void motionNextPressed(lv_event_t *) {
    const osmo::MotionProgram program = osmo::directCamera.motionProgram();
    const uint8_t last = min<uint8_t>(program.count,
        osmo::MOTION_POINT_CAPACITY - 1);
    if (motionSelected < last) ++motionSelected;
    tick(12);
}

static void motionOpenRunPressed(lv_event_t *) {
    setMotionUiView(MotionUiView::Run);
    tick(16);
}

static void motionEditPressed(lv_event_t *) {
    const osmo::DirectStatus status = osmo::directCamera.status();
    if (status.motionProgramArmed || status.motionProgramActive) return;
    setMotionUiView(MotionUiView::Gallery);
    tick(16);
}

static void motionCapturePressed(lv_event_t *) {
    const osmo::MotionProgram program = osmo::directCamera.motionProgram();
    if (motionSelected > program.count) motionSelected = program.count;
    if (motionSelected >= osmo::MOTION_POINT_CAPACITY) return;
    if (osmo::directCamera.captureMotionPoint(motionSelected)) {
        tick(35);
    }
}

// A captured waypoint cost the operator a physical framing to set. FORGET,
// GOTO, ARM and POWER OFF are all holds; a single tap wiping a point was the
// one destructive action on this panel that could happen by brushing it.
static void motionClearPressed(lv_event_t *e) {
    const lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        clearHoldStartedMs = 0;
        clearHoldTriggered = false;
        return;
    }
    if (code != LV_EVENT_PRESSED && code != LV_EVENT_PRESSING) return;
    const uint32_t now = millis();
    if (lv_indev_get_act() && lv_indev_get_scroll_obj(lv_indev_get_act())) {
        clearHoldStartedMs = 0;
        clearHoldTriggered = false;
        return;
    }
    if (!clearHoldStartedMs) {
        clearHoldStartedMs = now;
        removeSingleHold = lv_event_get_target(e) == btnMotionPointRemove;
    }
    if (clearHoldTriggered || now - clearHoldStartedMs < HOLD_DESTROY_MS) return;
    clearHoldTriggered = true;
    const bool saved = removeSingleHold
        ? osmo::directCamera.removeMotionPoint(motionSelected)
        : osmo::directCamera.clearMotionPoint(motionSelected);
    if (saved) {
        const osmo::MotionProgram program = osmo::directCamera.motionProgram();
        motionSelected = program.count
            ? min<uint8_t>(motionSelected, program.count - 1) : 0;
        setMotionUiView(MotionUiView::Gallery);
        tick(35);
    }
}

// The cycle wraps. It used to stick at the top value, and the only way back
// down was HOLD CLEAR, which throws away the framing to change a number.
static void motionTimePressed(lv_event_t *) {
    static const uint32_t choices[] = {1000, 2000, 3000, 5000, 8000, 12000};
    static const uint8_t N = sizeof(choices) / sizeof(choices[0]);
    const osmo::MotionProgram program = osmo::directCamera.motionProgram();
    if (motionSelected >= program.count) return;
    uint8_t next = 0;
    while (next < N && choices[next] <= program.points[motionSelected].moveMs)
        ++next;
    if (next >= N) next = 0;
    osmo::directCamera.setMotionTiming(motionSelected, choices[next],
        program.points[motionSelected].dwellMs);
    tick(18);
}

// DWELL runs 0 > 0.5 > 1 > 2 > 5 s > GO > 0. GO is a cue: the head holds at
// this point, after any dwell, until a human releases it. Drama runs on the
// actor; a fixed dwell suits a product pass.
static void motionDwellPressed(lv_event_t *) {
    static const uint32_t choices[] = {0, 500, 1000, 2000, 5000};
    static const uint8_t N = sizeof(choices) / sizeof(choices[0]);
    const osmo::MotionProgram program = osmo::directCamera.motionProgram();
    if (motionSelected >= program.count) return;
    const osmo::MotionPoint &point = program.points[motionSelected];
    if (point.holdForGo) {
        osmo::directCamera.setMotionHold(motionSelected, false);
        osmo::directCamera.setMotionTiming(motionSelected, point.moveMs, 0);
        tick(18);
        return;
    }
    uint8_t next = 0;
    while (next < N && choices[next] <= point.dwellMs) ++next;
    if (next >= N) {
        osmo::directCamera.setMotionHold(motionSelected, true);
    } else {
        osmo::directCamera.setMotionTiming(motionSelected, point.moveMs,
                                           choices[next]);
    }
    tick(18);
}

static void motionTransitionPressed(lv_event_t *) {
    const osmo::MotionProgram program = osmo::directCamera.motionProgram();
    if (motionSelected >= program.count) return;
    const auto current = static_cast<uint8_t>(
        program.points[motionSelected].transition);
    const auto next = static_cast<osmo::MotionTransition>(
        (current + 1) % static_cast<uint8_t>(osmo::MotionTransition::Count));
    osmo::directCamera.setMotionTransition(motionSelected, next);
    tick(18);
}

static void motionRepeatPressed(lv_event_t *) {
    const osmo::DirectStatus status = osmo::directCamera.status();
    if (status.motionProgramArmed || status.motionProgramActive) return;
    const uint8_t next = (status.motionRepeat + 1) %
        static_cast<uint8_t>(osmo::MotionRepeat::Count);
    osmo::directCamera.setMotionRepeat(next);
    tick(18);
}

static void motionSlotPressed(lv_event_t *) {
    const osmo::DirectStatus status = osmo::directCamera.status();
    if (status.motionProgramArmed || status.motionProgramActive) return;
    const uint8_t next = (status.motionSlot + 1) % osmo::MOTION_SLOT_COUNT;
    osmo::directCamera.selectMotionSlot(next);
    // A damaged slot opens as an empty programme with its error shown; the
    // selector must not be left pointing past it.
    if (osmo::directCamera.status().motionSlot != status.motionSlot)
        motionSelected = 0;
    tick(18);
}

// Battery floor for programmed motion only. Reported, never silent.
static bool armBlockedByBattery() {
    return batteryPct >= 0 && batteryPct <= BATTERY_ARM_FLOOR && !batteryCharging;
}

// Recenter from the shot, not from DEVICE > TOOLS three taps away. Direct
// mode sends the R&D gimbal request; USB asks the host, which owns it.
static void centerHoldEvent(lv_event_t *e) {
    const lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        centerHoldStartedMs = 0;
        centerHoldTriggered = false;
        return;
    }
    if (code != LV_EVENT_PRESSED && code != LV_EVENT_PRESSING) return;
    if (!controlEvidenceReady()) return;
    const uint32_t now = millis();
    if (!centerHoldStartedMs) centerHoldStartedMs = now;
    if (centerHoldTriggered || now - centerHoldStartedMs < HOLD_ACT_MS) return;
    centerHoldTriggered = true;
    if (rig.direct)
        osmo::directCamera.requestCameraAction(osmo::CameraAction::GimbalRecenter);
    else
        say("B recenter");
    tick(35);
}

static void motionHoldEvent(lv_event_t *e, bool runControl) {
    const lv_event_code_t code = lv_event_get_code(e);
    if (lv_indev_get_act() && lv_indev_get_scroll_obj(lv_indev_get_act())) {
        motionHoldStartedMs = 0;
        motionHoldTriggered = false;
        return;
    }
    // A cued waypoint holds until a human says go. The state word has always
    // been able to say "GO?" but nothing on the box could answer it, so the
    // release had to happen at the browser. A cue release is a tap, not a
    // hold: the deliberation already happened when the move was armed.
    if (runControl && rig.waitingCue >= 0) {
        if (code == LV_EVENT_RELEASED) {
            // Direct mode owns its own cue; the host owns the USB one.
            if (rig.direct) osmo::directCamera.requestMotionGo();
            else            say("B go");
            hapticGo();
        }
        motionHoldStartedMs = 0;
        motionHoldTriggered = false;
        return;
    }
    // A host timelapse is the only thing running. The run key stops it
    // between frames, which is gentler than STOP and leaves a resumable
    // progress file behind.
    if (runControl && rig.tlFrames > 0) {
        if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
            motionHoldStartedMs = 0;
            motionHoldTriggered = false;
            return;
        }
        if (code != LV_EVENT_PRESSED && code != LV_EVENT_PRESSING) return;
        const uint32_t now = millis();
        if (!motionHoldStartedMs) { motionHoldStartedMs = now; motionHoldIsRun = true; }
        if (motionHoldTriggered || now - motionHoldStartedMs < HOLD_ACT_MS) return;
        motionHoldTriggered = true;
        say("B tl_stop");
        tick(45);
        return;
    }
    const osmo::DirectStatus gate = osmo::directCamera.status();
    const osmo::MotionProgram program = osmo::directCamera.motionProgram();
    // The P1..P6 programme is the standalone runtime's. Under a USB host
    // the keys used to light, count to 100% and buzz for a request nothing
    // would run. The battery floor gates ARM only: an armed programme that
    // dips to 9% still gets its GO, because refusing it mid-blocking is
    // worse than finishing a move the operator has already committed to.
    const bool allowed = runControl
        ? (rig.direct && !gate.motionProgramActive &&
           (gate.motionProgramArmed ||
            (!armBlockedByBattery() && controlEvidenceReady() &&
             program.count >= 2)))
        : (rig.direct && controlEvidenceReady() &&
           motionSelected < program.count &&
           !gate.motionProgramArmed && !gate.motionProgramActive);
    if (!allowed) {
        motionHoldStartedMs = 0;
        motionHoldTriggered = false;
        return;
    }
    if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        motionHoldStartedMs = 0;
        motionHoldTriggered = false;
        return;
    }
    if (code != LV_EVENT_PRESSED && code != LV_EVENT_PRESSING) return;
    const uint32_t now = millis();
    if (!motionHoldStartedMs) {
        motionHoldStartedMs = now;
        motionHoldIsRun = runControl;
    }
    if (motionHoldIsRun != runControl || motionHoldTriggered ||
        now - motionHoldStartedMs < HOLD_ACT_MS) return;
    motionHoldTriggered = true;
    bool accepted = false;
    if (runControl) {
        const osmo::DirectStatus status = osmo::directCamera.status();
        if (status.motionProgramActive) {
            motionHoldStartedMs = 0;
            motionHoldTriggered = false;
            return;
        }
        accepted = status.motionProgramArmed
            ? osmo::directCamera.requestMotionGo()
            : osmo::directCamera.requestMotionArm();
    } else {
        accepted = osmo::directCamera.requestMotionGoto(motionSelected);
    }
    // The buzz means "the runtime took it". A refused request (stale
    // telemetry, host mode) used to buzz exactly like an accepted one.
    if (accepted) {
        tick(45);
    } else {
        motionHoldStartedMs = 0;
        motionHoldTriggered = false;
    }
}

static void motionGotoEvent(lv_event_t *e) { motionHoldEvent(e, false); }

// "Back to one" is the most frequent thing anyone does between takes. It was
// six actions here: prev, prev, prev, then a hold. One hold on the run pane,
// reusing the goto path, refused while the head is already at P1.
static void motionHomeEvent(lv_event_t *e) {
    const lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        homeHoldStartedMs = 0;
        homeHoldTriggered = false;
        return;
    }
    if (code != LV_EVENT_PRESSED && code != LV_EVENT_PRESSING) return;
    const osmo::DirectStatus gate = osmo::directCamera.status();
    const osmo::MotionProgram program = osmo::directCamera.motionProgram();
    if (!rig.direct || !controlEvidenceReady() || program.count < 1 ||
        gate.motionProgramArmed || gate.motionProgramActive ||
        gate.motionAtStart) {
        homeHoldStartedMs = 0;
        homeHoldTriggered = false;
        return;
    }
    const uint32_t now = millis();
    if (!homeHoldStartedMs) homeHoldStartedMs = now;
    if (homeHoldTriggered || now - homeHoldStartedMs < HOLD_ACT_MS) return;
    homeHoldTriggered = true;
    osmo::directCamera.requestMotionGoto(0);
    tick(45);
}
static osmo::PlaybackAction currentPlaybackAction() {
    const auto st = osmo::directCamera.status();
    const auto program = osmo::directCamera.motionProgram();
    return osmo::playbackAction({rig.direct, controlEvidenceReady(),
        armBlockedByBattery(), st.motionProgramActive,
        st.cuePoint != 0, st.motionAtStart, program.count});
}

static void motionRunEvent(lv_event_t *e) {
    if (!rig.direct) { motionHoldEvent(e, true); return; }
    const auto code = lv_event_get_code(e);
    static osmo::PlaybackAction pressedAction = osmo::PlaybackAction::None;
    const auto action = currentPlaybackAction();
    if (code == LV_EVENT_PRESS_LOST ||
        (lv_indev_get_act() && lv_indev_get_scroll_obj(lv_indev_get_act()))) {
        pressedAction = osmo::PlaybackAction::None;
        motionHoldStartedMs = 0;
        motionHoldTriggered = false;
        return;
    }
    if (code == LV_EVENT_PRESSED) {
        pressedAction = action;
        motionHoldStartedMs = millis();
        motionHoldIsRun = true;
        motionHoldTriggered = false;
        if (action == osmo::PlaybackAction::Stop) {
            stopPressed(nullptr);
            motionHoldTriggered = true;
        }
    } else if (code == LV_EVENT_RELEASED) {
        if (pressedAction == osmo::PlaybackAction::Continue &&
            action == pressedAction && !motionHoldTriggered) {
            if (osmo::directCamera.requestMotionGo()) hapticGo();
        }
        pressedAction = osmo::PlaybackAction::None;
        motionHoldStartedMs = 0;
        motionHoldTriggered = false;
        return;
    } else if (code != LV_EVENT_PRESSING) return;

    // A state change consumes the gesture; it never chains GO TO START into
    // PLAY or reuses a held finger when the connection becomes ready.
    if (action != pressedAction) {
        pressedAction = osmo::PlaybackAction::None;
        motionHoldStartedMs = 0;
        return;
    }
    if (motionHoldTriggered || !motionHoldStartedMs ||
        millis() - motionHoldStartedMs < HOLD_ACT_MS) return;
    bool accepted = false;
    if (action == osmo::PlaybackAction::GoToStart)
        accepted = osmo::directCamera.requestMotionGoto(0);
    else if (action == osmo::PlaybackAction::Play)
        accepted = osmo::directCamera.requestMotionPlay();
    else return;
    motionHoldTriggered = true;
    if (accepted) tick(45);
}

static void deviceCamerasPressed(lv_event_t *) { showCameraView(CAM_HUB); tick(16); }
static void deviceToolsPressed(lv_event_t *) { showCameraView(CAM_TOOLS); tick(16); }
static void devicePowerPressed(lv_event_t *) { showCameraView(CAM_POWER); tick(16); }

static void refusalHandPressed(lv_event_t *) {
    showPage(static_cast<uint8_t>(Workspace::Hand));
    tick(18);
}

static void faultDevicePressed(lv_event_t *) {
    showPage(static_cast<uint8_t>(Workspace::Device));
    showCameraView(rig.direct ? CAM_PROGRESS : CAM_DEVICE_HOME);
    tick(18);
}

static void cameraToolPressed(lv_event_t *e) {
    const auto action = static_cast<osmo::CameraAction>(
        (intptr_t)lv_event_get_user_data(e));
    if (!controlEvidenceReady()) return;
    osmo::directCamera.requestCameraAction(action);
    tick(28);
}

// DirectCamera::disable() tells the networking task to tear down, but that
// task publishes its final OFF snapshot asynchronously.  The UI's evidence
// gate must close in the same interaction that disables the link: otherwise a
// stale READY/telemetry sample can leave REC, JOG and HAND looking available
// after the operator has deliberately disconnected.
static void clearDirectRigEvidence() {
    rig.direct = false;
    rig.directPhase = osmo::DirectPhase::Off;
    rig.directBlockedAt = osmo::DirectPhase::Off;
    rig.linked = false;
    rig.camera = false;
    rig.telemetry = false;
    rig.telemetryAgeMs = UINT32_MAX;
    rig.poseValid = false;
    rig.moving = false;
    rig.armed = false;
    rig.recording = false;
    rig.recordIntent = false;
    rig.recordPending = false;
    rig.recordUnconfirmed = false;
    rig.recordCommandFault = false;
    rig.waitingCue = -1;
    rig.nearLimit = false;
    rig.headUp = rig.headDown = 0.0f;
    rig.limUp = rig.limDown = rig.limLeft = rig.limRight = 0;
    limUpLed = limDownLed = limLeftLed = limRightLed = limitNear = 0;
    rig.wifiAttempt = rig.wifiStatus = rig.wifiChannel = 0;
    memset(rig.wifiBssid, 0, sizeof(rig.wifiBssid));
    rig.retryAtMs = 0;
    rig.ssid[0] = '\0';
    rig.wifiReason[0] = '\0';
    strncpy(rig.detail, "direct link off", sizeof(rig.detail) - 1);
    rig.detail[sizeof(rig.detail) - 1] = '\0';
    rig.fault[0] = '\0';
    rig.move[0] = '\0';
    strcpy(rig.owner, "none");
    clutchFeedbackConfirmed = false;
}

static void linkDisconnectPressed(lv_event_t *) {
    // Painted unavailable when the direct link is off; setKey is colour
    // only, so without this a dimmed key still fired STOP.
    if (!osmo::directCamera.enabled()) return;
    stopPressed(nullptr);
    osmo::directCamera.disable();
    clearDirectRigEvidence();
    tick(30);
}

static void linkReconnectPressed(lv_event_t *) {
    osmo::directCamera.begin();
    osmo::directCamera.requestReconnect();
    tick(30);
}

static void powerOffEvent(lv_event_t *e) {
    const lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        powerOffHoldStartedMs = 0;
        if (!powerOffAtMs) powerOffTriggered = false;
        return;
    }
    if (code != LV_EVENT_PRESSED && code != LV_EVENT_PRESSING || powerOffAtMs)
        return;
    const uint32_t now = millis();
    if (!powerOffHoldStartedMs) powerOffHoldStartedMs = now;
    if (powerOffTriggered || now - powerOffHoldStartedMs < HOLD_POWER_MS) return;
    powerOffTriggered = true;
    stopPressed(nullptr);
    osmo::directCamera.disable();
    clearDirectRigEvidence();
    powerOffAtMs = now + 500;  // non-blocking neutral/radio shutdown grace
    tick(80);
}

// Core2 cannot yet decode an authoritative record-state payload. A deliberate
// hold still makes the camera command useful, but the result remains amber and
// uncertain in direct mode. Red is reserved for a state proved by the camera
// path, never for a local toggle.
static void recordEvent(lv_event_t *e) {
    const lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        recordHoldStartedMs = 0;
        recordHoldTriggered = false;
        return;
    }
    if (code != LV_EVENT_PRESSED && code != LV_EVENT_PRESSING) return;
    if (!controlEvidenceReady() || rig.recordPending) return;
    const uint32_t now = millis();
    if (!recordHoldStartedMs) recordHoldStartedMs = now;
    if (recordHoldTriggered || now - recordHoldStartedMs < HOLD_RECORD_MS) return;
    recordHoldTriggered = true;
    const bool start = !(rig.recording || rig.recordIntent);
    if (rig.direct) osmo::directCamera.requestRecord(start);
    else            say("B record_%s", start ? "start" : "stop");
    tick(35);
}

static void invTiltPressed(lv_event_t *) {
    invertTilt = !invertTilt;
    persistFeelBool("inv_t", invertTilt);
    say("B invert_tilt_%d", invertTilt ? 1 : 0);
    tick(20);
}

static void invPanPressed(lv_event_t *) {
    invertPan = !invertPan;
    persistFeelBool("inv_p", invertPan);
    say("B invert_pan_%d", invertPan ? 1 : 0);
    tick(20);
}

// Wiping the persisted feel profile throws away tuning the operator arrived at
// by shooting with it. Same class of loss as FORGET, so the same hold.
static void resetFeelPressed(lv_event_t *e) {
    const lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        resetHoldStartedMs = 0;
        resetHoldTriggered = false;
        return;
    }
    if (code != LV_EVENT_PRESSED && code != LV_EVENT_PRESSING) return;
    const uint32_t now = millis();
    if (!resetHoldStartedMs) resetHoldStartedMs = now;
    if (resetHoldTriggered || now - resetHoldStartedMs < HOLD_DESTROY_MS) return;
    resetHoldTriggered = true;
    resetFeelPreferences();
    say("B feel_defaults");
    tick(35);
}

static void setMenuPressed(lv_event_t *e) {
    const uint8_t target = (uint8_t)(intptr_t)lv_event_get_user_data(e);
    settingsReturn = SettingsReturn::None;
    showSettings(target);
    tick(18);
}

// These are route adapters, not second implementations. CAMERA and POWER open
// the existing DEVICE views; HAND and JOG open the exact same one-tap sheets
// used on the shooting surfaces.
static void settingsCameraPressed(lv_event_t *) {
    showPage(static_cast<uint8_t>(Workspace::Device));
    cameraReturnToSettings = true;
    showCameraView(CAM_HUB);
    tick(18);
}

static void settingsHandFeelPressed(lv_event_t *) {
    showPage(static_cast<uint8_t>(Workspace::Hand));
    setHandFeelOpen(true);
    tick(18);
}

static void settingsJogFeelPressed(lv_event_t *) {
    showPage(static_cast<uint8_t>(Workspace::Jog));
    setJogFeelOpen(true);
    tick(18);
}

static void settingsDirectionPressed(lv_event_t *) {
    settingsReturn = page == static_cast<uint8_t>(Workspace::Hand) && handFeelOpen
        ? SettingsReturn::HandFeel
        : page == static_cast<uint8_t>(Workspace::Jog) && jogFeelOpen
            ? SettingsReturn::JogFeel : SettingsReturn::None;
    showPage(static_cast<uint8_t>(Workspace::Settings));
    showSettings(SET_DIRECTION);
    tick(18);
}

static void settingsPowerPressed(lv_event_t *) {
    showPage(static_cast<uint8_t>(Workspace::Device));
    cameraReturnToSettings = true;
    showCameraView(CAM_POWER);
    tick(16);
}

static void limitLedsPressed(lv_event_t *) {
    limitLedsEnabled = !limitLedsEnabled;
    persistUiBool("lim_led", limitLedsEnabled);
    say("B limitleds_%d", limitLedsEnabled ? 1 : 0);
    tick(20);
}

static void beaconPressed(lv_event_t *) {
    beaconEnabled = !beaconEnabled;
    persistUiBool("beacon", beaconEnabled);
    say("B beacon_%d", beaconEnabled ? 1 : 0);
    tick(20);
}

static const osmo::CameraSummary *cameraById(uint32_t id) {
    if (!id) return nullptr;
    for (uint8_t i = 0; i < cameraCatalogUi.count; ++i) {
        if (cameraCatalogUi.items[i].id == id) return &cameraCatalogUi.items[i];
    }
    return nullptr;
}

static const osmo::CameraSummary *highlightedCamera() {
    return cameraById(cameraHighlightedId);
}

static const osmo::CameraSummary *currentCamera() {
    for (uint8_t i = 0; i < cameraCatalogUi.count; ++i) {
        if (cameraCatalogUi.items[i].current) return &cameraCatalogUi.items[i];
    }
    return nullptr;
}

static void cameraRowPressed(lv_event_t *e) {
    const uint8_t index = (uint8_t)(intptr_t)lv_event_get_user_data(e);
    if (index >= cameraCatalogUi.count) return;
    cameraHighlightedId = cameraCatalogUi.items[index].id;
    tick(14);
}

static void cameraScanPressed(lv_event_t *) {
    if (cameraCatalogUi.forgetPending) return;
    if (cameraCatalogUi.scanning || cameraCatalogUi.actionPending) {
        osmo::directCamera.cancelDeviceAction();
        showCameraView(CAM_HUB);
        tick(25);
        return;
    }
    if (currentCamera()) {
        cameraConfirmId = 0;
        showCameraView(CAM_SCAN_CONFIRM);
        tick(18);
        return;
    }
    osmo::directCamera.requestDeviceScan();
    showCameraView(CAM_HUB);
    tick(25);
}

static void cameraConnectPressed(lv_event_t *) {
    const osmo::CameraSummary *selected = highlightedCamera();
    if (!selected || cameraCatalogUi.scanning ||
        cameraCatalogUi.forgetPending) return;
    if (selected->current || cameraCatalogUi.actionPending) {
        showCameraView(CAM_PROGRESS);
        tick(16);
        return;
    }
    const osmo::CameraSummary *current = currentCamera();
    if (current && current->id != selected->id) {
        cameraConfirmId = selected->id;
        showCameraView(CAM_SWITCH_CONFIRM);
        tick(18);
        return;
    }
    osmo::directCamera.selectDevice(selected->id);
    showCameraView(CAM_PROGRESS);
    tick(25);
}

static void cameraForgetPressed(lv_event_t *) {
    const osmo::CameraSummary *selected = highlightedCamera();
    if (cameraCatalogUi.scanning || cameraCatalogUi.actionPending ||
        !selected || (!selected->trusted && !selected->current)) return;
    cameraConfirmId = selected->id;
    cameraForgetHoldStartedMs = 0;
    cameraForgetTriggered = false;
    showCameraView(CAM_FORGET_CONFIRM);
    tick(18);
}

static void cameraConfirmCancelPressed(lv_event_t *) {
    cameraForgetHoldStartedMs = 0;
    cameraForgetTriggered = false;
    showCameraView(CAM_HUB);
    tick(16);
}

static void cameraConfirmActionEvent(lv_event_t *e) {
    const lv_event_code_t code = lv_event_get_code(e);
    if (cameraView == CAM_SCAN_CONFIRM) {
        if (code != LV_EVENT_CLICKED) return;
        osmo::directCamera.requestDeviceScan();
        showCameraView(CAM_HUB);
        tick(30);
        return;
    }
    if (cameraView == CAM_SWITCH_CONFIRM) {
        if (code != LV_EVENT_CLICKED) return;
        osmo::directCamera.selectDevice(cameraConfirmId);
        showCameraView(CAM_PROGRESS);
        tick(30);
        return;
    }
    if (cameraView != CAM_FORGET_CONFIRM) return;
    if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        cameraForgetHoldStartedMs = 0;
        cameraForgetTriggered = false;
        return;
    }
    if (code != LV_EVENT_PRESSED && code != LV_EVENT_PRESSING) return;
    const uint32_t now = millis();
    if (!cameraForgetHoldStartedMs) cameraForgetHoldStartedMs = now;
    if (cameraForgetTriggered || now - cameraForgetHoldStartedMs < HOLD_DESTROY_MS) return;
    cameraForgetTriggered = true;
    osmo::directCamera.forgetDevice(cameraConfirmId);
    cameraHighlightedId = 0;
    cameraForgetHoldStartedMs = 0;
    cameraForgetTriggered = false;
    showCameraView(CAM_HUB);
    tick(70);
}

static void wifiReconnectPressed(lv_event_t *) {
    if (cameraCatalogUi.actionPending &&
        rig.directPhase != osmo::DirectPhase::RetryWait &&
        rig.directPhase != osmo::DirectPhase::Ready) {
        osmo::directCamera.cancelDeviceAction();
        showCameraView(CAM_HUB);
        tick(25);
        return;
    }
    if (rig.direct || !rig.linked) {
        osmo::directCamera.begin();
        osmo::directCamera.requestReconnect();
    } else {
        say("B reconnect");
    }
    tick(25);
}

static void cameraProgressBackPressed(lv_event_t *) {
    // Browsing back to the camera list must not restart discovery/pairing.
    // The adjacent action key explicitly says CANCEL whenever cancellation is
    // available, so only that deliberate control ends an in-flight action.
    showCameraView(CAM_HUB);
    tick(18);
}

// One owner for the backlight. Two writers -- the brightness button and the
// timeout -- would fight, and whichever ran last would win: the screen would
// wake to the wrong level or refuse to sleep at all.
static void applyBrightness() {
    M5.Display.setBrightness(screenAsleep ? 0 : BRIGHT_LCD[brightIdx]);
}

static void noteScreenActivity(uint32_t now) {
    screenOffAt = now + SCREEN_TIMEOUT_MS[screenTimeoutIdx];
    if (!screenAsleep) return;
    screenAsleep = false;
    swallowWake = M5.Touch.getDetail().isPressed();
    swallowHardwareWake = M5.BtnA.isPressed() || M5.BtnB.isPressed() ||
                           M5.BtnC.isPressed();
    applyBrightness();
}

static void pumpScreenTimeout(uint32_t now) {
    // STOP must remain one touch away while anything can move. If motion,
    // arming or a pending cue wakes the panel automatically there is no wake
    // gesture to swallow and the persistent STOP remains immediately usable.
    // A tally that goes dark mid-take reads as a stopped recording, and the
    // touch that wakes it is swallowed, so the first tap does nothing.
    const bool safetyStateVisible = rig.fault[0] || rig.nearLimit ||
        rig.moving || rig.armed || clutchHeld || jogActive ||
        rig.waitingCue >= 0 || rig.recording || rig.recordIntent ||
        rig.recordPending;
    if (!SCREEN_TIMEOUT_MS[screenTimeoutIdx] || safetyStateVisible) {
        noteScreenActivity(now);            // hold it awake, keep the deadline fresh
        return;
    }
    // Signed: millis() wraps after 49 days, and an unsigned test would either
    // sleep the screen instantly or never again for the rest of the shoot.
    if (!screenAsleep && (int32_t)(now - screenOffAt) >= 0) {
        screenAsleep = true;
        applyBrightness();
    }
}

static void screenToPressed(lv_event_t *) {
    screenTimeoutIdx = (screenTimeoutIdx + 1) % 4;
    persistUiByte("scr_to", screenTimeoutIdx);
    noteScreenActivity(millis());
    lv_label_set_text(lblScreenToTxt, SCREEN_TIMEOUT_NAMES[screenTimeoutIdx]);
    tick(20);
}

static void brightPressed(lv_event_t *) {
    brightIdx = (brightIdx + 1) % 4;
    blackout = (brightIdx == 3);
    persistUiByte("bright", brightIdx);
    noteScreenActivity(millis());
    applyBrightness();
    lv_label_set_text(lblBrightTxt, BRIGHT_NAMES[brightIdx]);
    tick(20);
}
static void silentPressed(lv_event_t *) {
    setSoundEnabled(!soundEnabled);
    persistUiBool("sound", soundEnabled);
    lv_label_set_text(lblSilentTxt, soundEnabled ? "ON" : "OFF");
}
static void hapticLevelPressed(lv_event_t *) {
    hapticLevelIdx = (hapticLevelIdx + 1) % (HAPTIC_FIRM + 1);
    persistUiByte("hap_lvl", hapticLevelIdx);
    // Changing comfort always cancels the old pulse, including FIRM -> SOFT.
    hapticEnvelope.cancel();
    pumpHaptic(millis());
    lv_label_set_text(lblHapticLevel, HAPTIC_LEVEL_NAMES[hapticLevelIdx]);
}
static void hapticMountingPressed(lv_event_t *) {
    cameraInHand = !cameraInHand;
    persistUiBool("in_hand", cameraInHand);
    if (cameraInHand) pumpHaptic(millis());
    lv_label_set_text(lblHapticMounting, cameraInHand
        ? "MOUNTED OFF" : "REMOTE ON");
}
static void hapticTestPressed(lv_event_t *) {
    scheduleHaptic(180, 70);
    uiRefreshRequested = true;
}

// DRIVE is a large, one-thumb rate surface. The horizontal and vertical travel
// are normalized independently, then radially capped, so the useful target
// fills the small display without a diagonal overspeed.
static void jogEvent(lv_event_t *e) {
    if (easeOpen) return;
    const lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        // PRESS_LOST can mean that a still-held finger slid outside the pad.
        // Only the raw touch controller can distinguish that from a lift.
        jogRawContact = M5.Touch.getDetail().isPressed();
        jogReleaseRequired = true;
        jogReleaseCandidateMs = jogRawContact ? 0 : millis();
        jogActive = false; jogX = jogY = 0;
        jogOriginValid = false;
        lv_obj_add_flag(jogKnob, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(jogDeadzone, LV_OBJ_FLAG_HIDDEN);
        osmo::directCamera.setJog(0.0f, 0.0f, false);
        say("J 0 0");
        return;
    }
    jogRawContact = true;
    const osmo::DirectStatus motion = osmo::directCamera.status();
    if (motion.motionProgramArmed || motion.motionProgramActive) {
        osmo::directCamera.abort();
        jogReleaseRequired = true;
        jogReleaseCandidateMs = 0;
        jogActive = false; jogX = jogY = 0;
        jogOriginValid = false;
        lv_obj_add_flag(jogKnob, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(jogDeadzone, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    if (!controlEvidenceReady()) {
        // Moving the visual stick while the transport cannot accept measured
        // motion would teach the operator that a dead control is live. Do not
        // synthesize a release here: the still-held finger must be lifted
        // before a later READY state is allowed to accept it.
        jogReleaseRequired = true;
        jogReleaseCandidateMs = 0;
        jogActive = false; jogX = jogY = 0;
        jogOriginValid = false;
        lv_obj_add_flag(jogKnob, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(jogDeadzone, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    if (jogReleaseRequired) {
        jogActive = false; jogX = jogY = 0;
        jogOriginValid = false;
        lv_obj_add_flag(jogKnob, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(jogDeadzone, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    lv_point_t pt;
    lv_indev_get_point(lv_indev_get_act(), &pt);
    lv_area_t a;
    lv_obj_get_coords(jogPad, &a);
    // Floating pickup is the essential safety property: wherever the thumb
    // first lands becomes zero.  An edge touch can never jump the head.  Only
    // displacement after pickup produces a rate command.
    if (code == LV_EVENT_PRESSED || !jogOriginValid) {
        jogOriginX = pt.x;
        jogOriginY = pt.y;
        jogOriginValid = true;
        jogActive = false; jogX = jogY = 0;
        // Keep the visual zero at the exact mathematical pickup, including at
        // an edge. LVGL clips the part of the ring/knob outside the pad; moving
        // the centre inward would make a neutral touch look like a command.
        const int localX = constrain((int)(pt.x - a.x1), 0,
                                     (int)(a.x2 - a.x1));
        const int localY = constrain((int)(pt.y - a.y1), 0,
                                     (int)(a.y2 - a.y1));
        lv_obj_set_pos(jogDeadzone, localX - 37, localY - 37);
        lv_obj_set_pos(jogKnob, localX - 21, localY - 21);
        lv_obj_move_foreground(jogKnob);
        lv_obj_clear_flag(jogDeadzone, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(jogKnob, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(jogHomeDot, LV_OBJ_FLAG_HIDDEN);
        osmo::directCamera.setJog(0.0f, 0.0f, false);
        if (jogPickups < JOG_LESSON_PICKUPS) {
            ++jogPickups;
            persistUiByte("jog_pick", jogPickups);
        }
        say("J 0 0 pickup=1");
        return;
    }

    float dx = (pt.x - jogOriginX) / 96.0f;
    float dy = (pt.y - jogOriginY) / 58.0f;
    const float mag = sqrtf(dx * dx + dy * dy);
    if (mag > 1.0f) { dx /= mag; dy /= mag; }
    jogX = dx; jogY = -dy;
    const float effective = osmo::DirectCamera::jogResponse(min(1.0f, mag));
    jogActive = effective > 0.0f;
    const int knobX = constrain((int)(pt.x - a.x1), 0,
                                (int)(a.x2 - a.x1)) - 21;
    const int knobY = constrain((int)(pt.y - a.y1), 0,
                                (int)(a.y2 - a.y1)) - 21;
    lv_obj_set_pos(jogKnob, knobX, knobY);
    osmo::directCamera.setJog(jogY, jogX, jogActive);
    say("J %.2f %.2f", jogY, jogX);
}

// ------------------------------------------------------------------ builders
static lv_obj_t *label(lv_obj_t *par, const lv_font_t *f, uint32_t col,
                       int x, int y, const char *txt) {
    lv_obj_t *l = lv_label_create(par);
    lv_obj_set_style_text_font(l, f, 0);
    lv_obj_set_style_text_color(l, lv_color_hex(col), 0);
    lv_label_set_text(l, txt);
    lv_obj_set_pos(l, x, y);
    return l;
}

// Native Core2 keys are divider-led rows, not miniature app cards.  A square
// one-pixel edge survives the low-density panel and leaves the label as the
// strongest shape; active semantic state still inverts the whole surface.
static lv_obj_t *keyBtn(lv_obj_t *par, const char *txt, int x, int y,
                        int w, int h, const lv_font_t *f,
                        lv_obj_t **lblOut, lv_event_cb_t cb,
                        lv_event_code_t code, void *userData = nullptr) {
    lv_obj_t *b = lv_btn_create(par);
    lv_obj_set_size(b, w, h);
    lv_obj_set_pos(b, x, y);
    lv_obj_set_style_radius(b, 0, 0);
    lv_obj_set_style_bg_color(b, lv_color_hex(C_SURF_LO), 0);
    lv_obj_set_style_bg_grad_dir(b, LV_GRAD_DIR_NONE, 0);
    lv_obj_set_style_border_width(b, 1, 0);
    lv_obj_set_style_border_color(b, lv_color_hex(C_EDGE), 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
    lv_obj_set_style_outline_width(b, 0, 0);
    // Pressed state is instant and physical: the surface drops, nothing glows.
    lv_obj_set_style_bg_color(b, lv_color_hex(C_SURF_HI), LV_STATE_PRESSED);
    if (cb) lv_obj_add_event_cb(b, cb, code, userData);
    lv_obj_t *l = lv_label_create(b);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_font(l, f, 0);
    lv_obj_center(l);
    if (lblOut) *lblOut = l;
    return b;
}

// One configuration card is the visual language for every adjustable surface:
// a stable monochrome symbol, a short noun, and the current value. Launcher
// cards and shot-time adjustment sheets use 150x72. Overflow scrolls inside
// the clipped page; shrinking targets is not a substitute for navigation.
static lv_obj_t *configTile(lv_obj_t *par, const char *symbol,
                            const char *title, const char *state,
                            int x, int y, int w, int h,
                            lv_obj_t **stateOut, lv_event_cb_t cb,
                            void *userData = nullptr) {
    lv_obj_t *tile = lv_btn_create(par);
    lv_obj_set_size(tile, w, h);
    lv_obj_set_pos(tile, x, y);
    lv_obj_set_style_radius(tile, 0, 0);
    lv_obj_set_style_bg_color(tile, lv_color_hex(C_SURF_LO), 0);
    lv_obj_set_style_bg_color(tile, lv_color_hex(C_SURF_HI), LV_STATE_PRESSED);
    lv_obj_set_style_bg_grad_dir(tile, LV_GRAD_DIR_NONE, 0);
    lv_obj_set_style_border_width(tile, 1, 0);
    lv_obj_set_style_border_color(tile, lv_color_hex(C_EDGE), 0);
    lv_obj_set_style_pad_all(tile, 0, 0);
    lv_obj_set_style_shadow_width(tile, 0, 0);
    lv_obj_set_style_outline_width(tile, 0, 0);
    if (cb) lv_obj_add_event_cb(tile, cb, LV_EVENT_CLICKED, userData);

    const bool compact = h < 64;
    lv_obj_t *icon = label(tile, compact ? FONT_18 : FONT_22, C_TEXT, 10,
                           compact ? 15 : 22, symbol);
    lv_obj_set_size(icon, 26, 28);
    lv_obj_set_style_text_align(icon, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_clear_flag(icon, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *name = label(tile, FONT_12, C_TEXT, 42,
                           compact ? 7 : 14, title);
    lv_obj_set_size(name, w - 48, 17);
    lv_label_set_long_mode(name, LV_LABEL_LONG_DOT);
    lv_obj_clear_flag(name, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *detail = label(tile, FONT_12, C_DIM, 42,
                             compact ? 30 : 39, state);
    lv_obj_set_size(detail, w - 48, 18);
    lv_label_set_long_mode(detail, LV_LABEL_LONG_DOT);
    lv_obj_clear_flag(detail, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    if (stateOut) *stateOut = detail;
    return tile;
}

// Config state is communicated by the value and border, never by repainting a
// whole card white and losing its icon/title. Cyan remains reserved for live
// motion; ordinary selected preferences use white, holds/waits use amber.
static void cancelScrollHolds(lv_event_t *) {
    cancelPageLocalHolds();
}

static void setConfigTile(lv_obj_t *tile, lv_obj_t *state, bool active,
                          uint32_t colour, bool available) {
    if (!tile || !state) return;
    if (available) lv_obj_add_flag(tile, LV_OBJ_FLAG_CLICKABLE);
    else           lv_obj_clear_flag(tile, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_bg_color(tile,
        lv_color_hex(available ? (active ? C_SURF_HI : C_SURF_LO) : C_BG), 0);
    lv_obj_set_style_border_width(tile, active ? 2 : 1, 0);
    lv_obj_set_style_border_color(tile,
        lv_color_hex(active ? colour : (available ? C_EDGE : C_FAINT)), 0);
    lv_obj_set_style_text_color(state,
        lv_color_hex(available ? (active ? colour : C_DIM) : C_FAINT), 0);
    for (uint8_t i = 0; i < 2; ++i) {
        lv_obj_t *child = lv_obj_get_child(tile, i);
        if (child) lv_obj_set_style_text_color(child,
            lv_color_hex(available ? C_TEXT : C_FAINT), 0);
    }
}

static lv_obj_t *makeConfigPane(lv_obj_t *parent, bool scrollable) {
    lv_obj_t *pane = lv_obj_create(parent);
    lv_obj_set_size(pane, 320, PAGE_H);
    lv_obj_set_pos(pane, 0, 0);
    lv_obj_set_style_bg_color(pane, lv_color_hex(C_BG), 0);
    lv_obj_set_style_border_width(pane, 0, 0);
    lv_obj_set_style_pad_all(pane, 0, 0);
    lv_obj_set_style_radius(pane, 0, 0);
    if (scrollable) {
        lv_obj_set_scroll_dir(pane, LV_DIR_VER);
        lv_obj_add_event_cb(pane, cancelScrollHolds, LV_EVENT_SCROLL_BEGIN, nullptr);
        lv_obj_set_scrollbar_mode(pane, LV_SCROLLBAR_MODE_AUTO);
        lv_obj_set_style_pad_bottom(pane, 8, 0);
        // On 320x240 a menu is a direct thumb surface, not a phone feed:
        // deliberate drag scrolls, lift stops, and an edge never rubber-bands.
        lv_obj_clear_flag(pane,
            LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM);
    } else {
        lv_obj_clear_flag(pane, LV_OBJ_FLAG_SCROLLABLE);
    }
    return pane;
}

static void setKey(lv_obj_t *btn, lv_obj_t *lbl, bool active,
                   uint32_t colour, bool available) {
    // A widget that was declared but never built is a null here, and LVGL
    // dereferences it without checking: the box panics on the first refresh and
    // boot-loops, which looks like dead hardware rather than a missing widget.
    // One branch is cheaper than that failure mode.
    if (!btn || !lbl) return;
    lv_obj_set_style_bg_color(btn,
        lv_color_hex(active ? colour : (available ? C_SURF_LO : C_BG)), 0);
    lv_obj_set_style_text_color(lbl,
        lv_color_hex(active ? C_BG : (available ? C_TEXT : C_FAINT)), 0);
}


static uint8_t easeBase() {
    return easeAxis < 0 || !(easeValues[easeMode][6] & (1 << easeAxis))
        ? 0 : 2 + easeAxis*2;
}
static void commitEase(const uint8_t (&previous)[2][7]) {
    if (!feelPreferencesReady ||
        feelPreferences.putBytes("ease_v1", easeValues, sizeof(easeValues)) != sizeof(easeValues)) {
        memcpy(easeValues, previous, sizeof(easeValues));
        easeSaveError = true;
    } else { easeSaveError = false; easeDirty = true; }
    refreshEasePane();
}
static void closeEasePane() {
    easeOpen = false;
    if (easePane) lv_obj_add_flag(easePane, LV_OBJ_FLAG_HIDDEN);
    updatePageTitle();
    updateSoftKeys();
}
static void openEase(lv_event_t *e) {
    stopPressed(nullptr); // hard neutral; opening a gear never leaves a release tail
    const int id = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));
    easeMode = id / 3;
    easeAxis = (id % 3) - 1;
    easeOpen = true;
    easeSaveError = false;
    lv_obj_scroll_to_y(easePane, 0, LV_ANIM_OFF);
    lv_obj_clear_flag(easePane, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(easePane);
    refreshEasePane();
    updatePageTitle();
    updateSoftKeys();
}
static void easeAdjust(lv_event_t *e) {
    uint8_t previous[2][7]; memcpy(previous, easeValues, sizeof(previous));
    const int delta = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));
    uint8_t *v = easeValues[easeMode];
    if (easeAxis >= 0 && !(v[6] & (1 << easeAxis))) {
        v[2+easeAxis*2] = v[0]; v[3+easeAxis*2] = v[1];
        v[6] |= 1 << easeAxis;
    }
    const uint8_t index = easeBase() + (abs(delta) == 2 ? 1 : 0);
    v[index] = constrain(static_cast<int>(v[index]) + (delta > 0 ? 1 : -1), 0, 5);
    commitEase(previous);
}
static void easeSource(lv_event_t *) {
    if (easeAxis < 0) return;
    uint8_t previous[2][7]; memcpy(previous, easeValues, sizeof(previous));
    uint8_t *v = easeValues[easeMode];
    if (!(v[6] & (1 << easeAxis))) {
        v[2+easeAxis*2]=v[0]; v[3+easeAxis*2]=v[1];
    }
    v[6] ^= 1 << easeAxis;
    commitEase(previous);
}
static void easeGlobal(lv_event_t *) {
    easeAxis = -1;
    refreshEasePane();
    updatePageTitle();
}
static void easePreset(lv_event_t *) {
    uint8_t previous[2][7]; memcpy(previous, easeValues, sizeof(previous));
    uint8_t *v = easeValues[easeMode];
    if (easeAxis >= 0) v[6] |= 1 << easeAxis;
    const uint8_t base = easeBase();
    const uint8_t next = v[base] < 3 ? 3 : v[base] < 5 ? 5 : 0;
    v[base] = next; v[base+1] = next;
    commitEase(previous);
}
static void easeFeel(lv_event_t *) {
    closeEasePane();
    if (easeMode == 0) {
        showPage(static_cast<uint8_t>(Workspace::Hand));
        setHandFeelOpen(true);
        setHandFeelView(HandFeelView::AxisTune);
    } else {
        showPage(static_cast<uint8_t>(Workspace::Jog));
        setJogFeelOpen(true);
    }
}
static void refreshEasePane() {
    if (!easePane) return;
    const uint8_t base = easeBase();
    char value[24];
    if (easeValues[easeMode][base] == 0) snprintf(value, sizeof(value), "AUTO");
    else snprintf(value, sizeof(value), "%u ms", EASE_MS[easeValues[easeMode][base]]);
    lv_label_set_text(easeStartLabel, value);
    snprintf(value, sizeof(value), "%u ms", EASE_MS[easeValues[easeMode][base+1]]);
    lv_label_set_text(easeEndLabel, value);
    lv_label_set_text(easeSourceLabel, easeAxis < 0 ? "GLOBAL" :
        (easeValues[easeMode][6] & (1 << easeAxis)) ? "CUSTOM" : "USE GLOBAL");
    lv_label_set_text(easeScopeLabel, easeMode == 0 ? "HAND DEFAULTS" : "JOG DEFAULTS");
    setConfigTile(easeSourceButton, easeSourceLabel, easeAxis >= 0 && easeBase() != 0,
                  C_TEXT, easeAxis >= 0);
    lv_label_set_text(easeHelpLabel, easeSaveError ? "SAVE FAILED / VALUE RESTORED" :
        "DIRECT MODE ONLY\nStart: AUTO keeps existing feel.\nRelease: timed deceleration.\nSTOP and faults bypass easing.");
}
static void buildEasePane() {
    easePane = makeConfigPane(screenMain, true);
    lv_obj_set_pos(easePane, 0, HEADER_H);
    easeSourceButton = configTile(easePane, LV_SYMBOL_SETTINGS, "SOURCE", "USE GLOBAL",
        6, 4, 150, 72, &easeSourceLabel, easeSource);
    configTile(easePane, LV_SYMBOL_HOME, "GLOBAL", "DEFAULTS",
        164, 4, 150, 72, &easeScopeLabel, easeGlobal);
    label(easePane, FONT_14, C_TEXT, 8, 84, "START EASE");
    keyBtn(easePane, "-", 6, 106, 64, 56, FONT_22, nullptr, easeAdjust,
           LV_EVENT_CLICKED, reinterpret_cast<void*>(-1));
    easeStartLabel = label(easePane, FONT_22, C_TEXT, 86, 122, "0 ms");
    lv_obj_set_size(easeStartLabel, 150, 28);
    keyBtn(easePane, "+", 250, 106, 64, 56, FONT_22, nullptr, easeAdjust,
           LV_EVENT_CLICKED, reinterpret_cast<void*>(1));
    label(easePane, FONT_14, C_TEXT, 8, 180, "RELEASE EASE");
    keyBtn(easePane, "-", 6, 202, 64, 56, FONT_22, nullptr, easeAdjust,
           LV_EVENT_CLICKED, reinterpret_cast<void*>(-2));
    easeEndLabel = label(easePane, FONT_22, C_TEXT, 86, 218, "200 ms");
    lv_obj_set_size(easeEndLabel, 150, 28);
    keyBtn(easePane, "+", 250, 202, 64, 56, FONT_22, nullptr, easeAdjust,
           LV_EVENT_CLICKED, reinterpret_cast<void*>(2));
    configTile(easePane, LV_SYMBOL_LOOP, "PRESET", "3 FEELS",
        6, 276, 150, 72, nullptr, easePreset);
    configTile(easePane, LV_SYMBOL_SETTINGS, "FEEL", "GAIN / FILTER",
        164, 276, 150, 72, nullptr, easeFeel);
    easeHelpLabel = label(easePane, FONT_12, C_DIM, 8, 360, "");
    lv_obj_set_size(easeHelpLabel, 304, 76);
    lv_label_set_long_mode(easeHelpLabel, LV_LABEL_LONG_WRAP);
    lv_obj_add_flag(easePane, LV_OBJ_FLAG_HIDDEN);
}

static void updateSoftKeys();
static void showPage(uint8_t idx);

// Inside a settings sub-page the left key is "back", because leaving the page
// entirely to get out of a sub-page would lose the operator's place.
static void navigateLeft() {
    if (easeOpen) { closeEasePane(); return; }
    if (page == static_cast<uint8_t>(Workspace::Jog) && jogFeelOpen) {
        setJogFeelOpen(false);
        tick(15);
        return;
    }
    if (page == static_cast<uint8_t>(Workspace::Hand) && handFeelOpen) {
        if (handFeelView == HandFeelView::AxisTune) {
            setHandFeelView(HandFeelView::Main);
            tick(15);
            return;
        }
        setHandFeelOpen(false);
        tick(15);
        return;
    }
    if (page == static_cast<uint8_t>(Workspace::Moves) &&
        motionUiView == MotionUiView::Point) {
        setMotionUiView(MotionUiView::Gallery);
        tick(15);
        return;
    }
    if (page == static_cast<uint8_t>(Workspace::Moves) && motionRunView) {
        const osmo::DirectStatus st = osmo::directCamera.status();
        if (!(st.motionProgramArmed || st.motionProgramActive)) {
            motionEditPressed(nullptr);
            return;
        }
        // Edit is locked while armed or running; the left key is a page
        // key again rather than a dead one.
    }
    // Camera progress/confirm belongs to the camera list. A Settings shortcut
    // remembers its origin so BACK returns to the launcher instead of making
    // the operator walk the top-level page cycle. Navigation never cancels an
    // in-flight scan or pairing action.
    if (page == static_cast<uint8_t>(Workspace::Device) &&
        cameraView != CAM_DEVICE_HOME) {
        const bool cameraChild = cameraView == CAM_PROGRESS ||
            cameraView == CAM_SCAN_CONFIRM || cameraView == CAM_SWITCH_CONFIRM ||
            cameraView == CAM_FORGET_CONFIRM;
        if (cameraChild) {
            showCameraView(CAM_HUB);
        } else if (cameraReturnToSettings) {
            cameraReturnToSettings = false;
            showPage(static_cast<uint8_t>(Workspace::Settings));
            showSettings(SET_MENU);
        } else {
            showCameraView(CAM_DEVICE_HOME);
        }
        tick(15);
        return;
    }
    if (page == static_cast<uint8_t>(Workspace::Settings) &&
        setSub != SET_MENU) {
        if (setSub == SET_DIRECTION && settingsReturn != SettingsReturn::None) {
            const SettingsReturn returnTo = settingsReturn;
            settingsReturn = SettingsReturn::None;
            if (returnTo == SettingsReturn::HandFeel) {
                showPage(static_cast<uint8_t>(Workspace::Hand));
                setHandFeelOpen(true);
            } else {
                showPage(static_cast<uint8_t>(Workspace::Jog));
                setJogFeelOpen(true);
            }
            tick(15);
            return;
        }
        showSettings(setSub == SET_DIRECTION ? SET_CONTROLS :
                     setSub == SET_HAPTIC ? SET_FEEDBACK : SET_MENU);
        tick(15);
        return;
    }
    showPage((page + PAGE_COUNT - 1) % PAGE_COUNT);
    tick(15);
}
static void softLeftCb(lv_event_t *) { navigateLeft(); }
static void softRightCb(lv_event_t *) { showPage((page + 1) % PAGE_COUNT); tick(15); }
static void homeOrToggleControl() {
    const uint8_t hand = static_cast<uint8_t>(Workspace::Hand);
    const uint8_t jog = static_cast<uint8_t>(Workspace::Jog);
    if (easeOpen || handFeelOpen || jogFeelOpen) {
        closeEasePane();
        showPage(hand);
    } else if (page == hand) {
        showPage(jog);
    } else if (page == jog) {
        showPage(static_cast<uint8_t>(Workspace::Device));
        showCameraView(CAM_TOOLS);
    } else {
        showPage(hand);
    }
    tick(18);
}
static void softMidEvent(lv_event_t *) { homeOrToggleControl(); }

// Every page uses the same reserved header and content rectangle.
static void layoutPage(lv_obj_t *shown) {
    lv_obj_set_pos(shown, 0, HEADER_H);
    lv_obj_set_size(shown, 320, PAGE_H);
}

// The title had two owners and neither refreshed: showPage wrote the running
// timelapse counter once, on the page change, so `TL 4/300` then sat frozen
// for the rest of a multi-hour run and read as a stalled rig. One function,
// called from every page change AND from the refresh tick.
static void updatePageTitle() {
    if (easeOpen) {
        lv_label_set_text(lblPageName, easeAxis < 0 ? "GLOBAL EASE" :
            easeAxis == 0 ? "TILT FEEL" : "PAN FEEL");
        return;
    }
    if (page == static_cast<uint8_t>(Workspace::Settings)) {
        lv_label_set_text(lblPageName, SET_NAMES[setSub]);
        lv_obj_set_style_text_color(lblPageName, lv_color_hex(C_TEXT), 0);
        return;
    }
    if (page == static_cast<uint8_t>(Workspace::Device)) {
        // BACK is ambiguous if the header still says DEVICE from three
        // sub-views deep. SETTINGS has always named its sub-page; so does this.
        lv_label_set_text(lblPageName, CAMERA_VIEW_NAMES[cameraView]);
        lv_obj_set_style_text_color(lblPageName, lv_color_hex(C_TEXT), 0);
        return;
    }
    if (page == static_cast<uint8_t>(Workspace::Moves)) {
        static char motionTitle[24];
        if (motionUiView == MotionUiView::Run)
            snprintf(motionTitle, sizeof(motionTitle), "MOVES / RUN");
        else if (motionUiView == MotionUiView::Point)
            snprintf(motionTitle, sizeof(motionTitle), "P%u / GALLERY",
                     motionSelected + 1);
        else
            snprintf(motionTitle, sizeof(motionTitle), "MOVES %s / RUN >",
                     SLOT_NAMES[osmo::directCamera.status().motionSlot % 2]);
        lv_label_set_text(lblPageName, motionTitle);
        lv_obj_set_style_text_color(lblPageName, lv_color_hex(C_TEXT), 0);
        return;
    }
    if (page == static_cast<uint8_t>(Workspace::Jog) && jogFeelOpen) {
        lv_label_set_text(lblPageName, "JOG FEEL");
        lv_obj_set_style_text_color(lblPageName, lv_color_hex(C_TEXT), 0);
        return;
    }
    if (page == static_cast<uint8_t>(Workspace::Hand) && handFeelOpen) {
        lv_label_set_text(lblPageName,
            handFeelView == HandFeelView::AxisTune ? "AXIS TUNE" : "HAND FEEL");
        lv_obj_set_style_text_color(lblPageName, lv_color_hex(C_TEXT), 0);
        return;
    }
    if (rig.tlFrames > 0) {
        // A running timelapse takes the page name: it is the only thing
        // happening, it lasts hours, and it is what the operator walked over
        // to check. Cyan because it is commanded motion, same as everywhere
        // else -- not red, which means the tally and nothing else.
        char buf[24];
        snprintf(buf, sizeof(buf), "TL %d/%d", rig.tlFrame, rig.tlFrames);
        lv_label_set_text(lblPageName, buf);
        lv_obj_set_style_text_color(lblPageName, lv_color_hex(C_CYAN), 0);
        return;
    }
    if (page == static_cast<uint8_t>(Workspace::Hand))
        lv_label_set_text(lblPageName, "HAND / FEEL >");
    else if (page == static_cast<uint8_t>(Workspace::Jog))
        lv_label_set_text(lblPageName, "JOG / FEEL >");
    else
        lv_label_set_text(lblPageName, WORKSPACE_NAMES[page]);
    lv_obj_set_style_text_color(lblPageName, lv_color_hex(C_TEXT), 0);
}

static void showSettings(uint8_t which) {
    const uint8_t nextSub = which % SET_COUNT;
    if (setSub != nextSub) cancelPageLocalHolds();
    setSub = nextSub;
    for (uint8_t i = 0; i < SET_COUNT; i++) {
        if (i == setSub) lv_obj_clear_flag(pgSetPanes[i], LV_OBJ_FLAG_HIDDEN);
        else             lv_obj_add_flag(pgSetPanes[i], LV_OBJ_FLAG_HIDDEN);
    }
    if (setSub == SET_MENU)
        lv_obj_scroll_to_y(pgSetPanes[SET_MENU], 0, LV_ANIM_OFF);
    else if (setSub == SET_CONTROLS)
        lv_obj_scroll_to_y(pgSetPanes[SET_CONTROLS], 0, LV_ANIM_OFF);
    // The header carries the sub-page name, so there is never any doubt about
    // which screen the back key will leave.
    updatePageTitle();
    updateSoftKeys();
}

static void showPage(uint8_t idx) {
    if (easeOpen) closeEasePane();
    const uint8_t nextPage = idx % PAGE_COUNT;
    if (page != nextPage) cancelPageLocalHolds();
    // Hiding a pressed LVGL parent is not guaranteed to emit RELEASED or
    // PRESS_LOST. Neutralise the page-local live input before it disappears,
    // and keep its latch until the raw hardware has really been released.
    if (page == static_cast<uint8_t>(Workspace::Jog) &&
        nextPage != static_cast<uint8_t>(Workspace::Jog)) {
        const bool wasJogActive = jogActive;
        if (jogRawContact || wasJogActive) {
            jogReleaseRequired = true;
            jogReleaseCandidateMs = 0;
        }
        jogActive = false;
        jogX = jogY = 0;
        jogOriginValid = false;
        if (jogKnob) lv_obj_add_flag(jogKnob, LV_OBJ_FLAG_HIDDEN);
        if (jogDeadzone) lv_obj_add_flag(jogDeadzone, LV_OBJ_FLAG_HIDDEN);
        if (wasJogActive) {
            osmo::directCamera.abort();
            say("J 0 0");
        }
        setJogFeelOpen(false);
    }
    if (page == static_cast<uint8_t>(Workspace::Hand) &&
        nextPage != static_cast<uint8_t>(Workspace::Hand)) {
        const bool rawClutch = screenClutchHeld;
        if (rawClutch || clutchHeld) {
            clutchReleaseRequired = true;
            clutchReleaseCandidateMs = 0;
        }
        screenClutchHeld = false;
        if (clutchHeld) {
            clutchHeld = false;
            osmo::directCamera.setClutch(false);
            say("E 0 reason=page");
        }
        setHandFeelOpen(false);
    }
    if (page == static_cast<uint8_t>(Workspace::Moves) &&
        nextPage != static_cast<uint8_t>(Workspace::Moves)) {
        motionUiView = MotionUiView::Gallery;
        motionRunView = false;
        if (motionRunPane) lv_obj_add_flag(motionRunPane, LV_OBJ_FLAG_HIDDEN);
        if (motionPointPane) lv_obj_add_flag(motionPointPane, LV_OBJ_FLAG_HIDDEN);
        if (motionGalleryPane)
            lv_obj_clear_flag(motionGalleryPane, LV_OBJ_FLAG_HIDDEN);
    }
    // Leaving settings always drops back to the menu. Coming back later to
    // find yourself three levels in, with no memory of how you got there, is
    // worse than losing your place.
    if (page == static_cast<uint8_t>(Workspace::Settings) &&
        nextPage != static_cast<uint8_t>(Workspace::Settings)) {
        setSub = SET_MENU;
        settingsReturn = SettingsReturn::None;
    }
    // Leaving DEVICE drops any armed confirm for the same reason: a
    // destructive dialog must not be waiting when the page comes back.
    if (page == static_cast<uint8_t>(Workspace::Device) &&
        nextPage != static_cast<uint8_t>(Workspace::Device)) {
        cameraConfirmId = 0;
        cameraForgetHoldStartedMs = 0;
        cameraForgetTriggered = false;
        cameraView = CAM_DEVICE_HOME;
        cameraReturnToSettings = false;
    }
    page = nextPage;
    lv_obj_add_flag(pgControl, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(pgJog, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(pgClutch, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(pgDevice, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(pgSetup, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(page == 0 ? pgControl : page == 1 ? pgJog
                      : page == 2 ? pgClutch : page == 3 ? pgDevice
                      : pgSetup, LV_OBJ_FLAG_HIDDEN);
    lv_obj_t *shown = page == 0 ? pgControl : page == 1 ? pgJog
                      : page == 2 ? pgClutch : page == 3 ? pgDevice
                      : pgSetup;
    layoutPage(shown);
    if (page == static_cast<uint8_t>(Workspace::Settings)) {
        showSettings(setSub);
    } else if (page == static_cast<uint8_t>(Workspace::Device)) {
        showCameraView(cameraView);
    }
    updatePageTitle();
    updateSoftKeys();
}

static void showCameraView(CameraView which) {
    if (cameraView != which) cancelPageLocalHolds();
    cameraView = which;
    if (!deviceHome || !cameraHub || !cameraProgress || !cameraConfirm ||
        !deviceTools || !devicePower) return;
    if (which == CAM_DEVICE_HOME)
        lv_obj_clear_flag(deviceHome, LV_OBJ_FLAG_HIDDEN);
    else
        lv_obj_add_flag(deviceHome, LV_OBJ_FLAG_HIDDEN);
    if (which == CAM_HUB) lv_obj_clear_flag(cameraHub, LV_OBJ_FLAG_HIDDEN);
    else                  lv_obj_add_flag(cameraHub, LV_OBJ_FLAG_HIDDEN);
    if (which == CAM_PROGRESS)
        lv_obj_clear_flag(cameraProgress, LV_OBJ_FLAG_HIDDEN);
    else
        lv_obj_add_flag(cameraProgress, LV_OBJ_FLAG_HIDDEN);
    if (which == CAM_SCAN_CONFIRM || which == CAM_SWITCH_CONFIRM ||
        which == CAM_FORGET_CONFIRM)
        lv_obj_clear_flag(cameraConfirm, LV_OBJ_FLAG_HIDDEN);
    else
        lv_obj_add_flag(cameraConfirm, LV_OBJ_FLAG_HIDDEN);
    if (which == CAM_TOOLS) lv_obj_clear_flag(deviceTools, LV_OBJ_FLAG_HIDDEN);
    else                    lv_obj_add_flag(deviceTools, LV_OBJ_FLAG_HIDDEN);
    if (which == CAM_POWER) lv_obj_clear_flag(devicePower, LV_OBJ_FLAG_HIDDEN);
    else                    lv_obj_add_flag(devicePower, LV_OBJ_FLAG_HIDDEN);
    updatePageTitle();
    updateSoftKeys();
}

// Thin edge bars keep directional headroom visible without stealing the
// thumb's movement surface. They are siblings of the pad and never clickable.
static lv_obj_t *makeLimitBar(lv_obj_t *parent, int x, int y, int w, int h) {
    lv_obj_t *bar = lv_obj_create(parent);
    lv_obj_set_pos(bar, x, y);
    lv_obj_set_size(bar, w, h);
    lv_obj_set_style_radius(bar, 0, 0);
    lv_obj_set_style_border_width(bar, 0, 0);
    lv_obj_set_style_bg_color(bar, lv_color_hex(C_AMBER), 0);
    lv_obj_set_style_bg_opa(bar, LV_OPA_20, 0);
    lv_obj_set_style_pad_all(bar, 0, 0);
    lv_obj_clear_flag(bar, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t *hatch = lv_label_create(bar);
    lv_obj_set_style_text_font(hatch, FONT_10, 0);
    lv_obj_set_style_text_color(hatch, lv_color_hex(C_AMBER), 0);
    lv_obj_set_style_text_opa(hatch, LV_OPA_50, 0);
    lv_label_set_text(hatch, w > h
        ? "////////////////////////////"
        : "/\n/\n/\n/\n/\n/\n/\n/\n/");
    lv_obj_set_pos(hatch, w > h ? 1 : 0, w > h ? -4 : -3);
    lv_obj_set_size(hatch, w, h + (w > h ? 8 : 6));
    lv_label_set_long_mode(hatch, LV_LABEL_LONG_CLIP);
    lv_obj_clear_flag(hatch, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(bar, LV_OBJ_FLAG_HIDDEN);
    return bar;
}

static void setLimitBar(lv_obj_t *bar, uint8_t value) {
    if (!bar) return;
    if (value < 3) { lv_obj_add_flag(bar, LV_OBJ_FLAG_HIDDEN); return; }
    lv_obj_clear_flag(bar, LV_OBJ_FLAG_HIDDEN);
    const float t = value / 100.0f;
    const uint8_t g = static_cast<uint8_t>(176 * (1.0f - t));
    lv_obj_set_style_bg_color(bar, lv_color_make(0xFF, g, 0), 0);
    lv_obj_set_style_bg_opa(bar,
        static_cast<lv_opa_t>(LV_OPA_40 + (LV_OPA_COVER - LV_OPA_40) * t), 0);
    lv_obj_t *hatch = lv_obj_get_child(bar, 0);
    if (hatch) lv_obj_set_style_text_opa(hatch,
        static_cast<lv_opa_t>(LV_OPA_40 + (LV_OPA_COVER - LV_OPA_40) * t), 0);
}

static lv_obj_t *makePage() {
    lv_obj_t *p = lv_obj_create(screenMain);
    // Every page starts below the uniform header and ends above navigation.
    lv_obj_set_size(p, 320, PAGE_H);
    lv_obj_set_pos(p, 0, HEADER_H);
    lv_obj_set_style_bg_color(p, lv_color_hex(C_BG), 0);
    lv_obj_set_style_border_width(p, 0, 0);
    lv_obj_set_style_radius(p, 0, 0);
    lv_obj_set_style_pad_all(p, 0, 0);
    lv_obj_clear_flag(p, LV_OBJ_FLAG_SCROLLABLE);
    return p;
}

static void buildUi() {
    screenMain = lv_scr_act();
    lv_obj_set_style_bg_color(screenMain, lv_color_hex(C_BG), 0);
    lv_obj_clear_flag(screenMain, LV_OBJ_FLAG_SCROLLABLE);

    // Three fixed header cells: record, context/evidence and STOP. The
    // transport/owner subtitle never changes duty; page-local status belongs
    // in the workspace or bottom rail.
    headerContext = lv_btn_create(screenMain);
    btnJogFeel = headerContext;  // explicit JOG feel entry; shared by HAND
    lv_obj_set_pos(headerContext, 78, 0);
    lv_obj_set_size(headerContext, 184, HEADER_H);
    lv_obj_set_style_radius(headerContext, 0, 0);
    lv_obj_set_style_bg_color(headerContext, lv_color_hex(C_BG), 0);
    lv_obj_set_style_bg_color(headerContext, lv_color_hex(C_SURF_LO),
                              LV_STATE_PRESSED);
    lv_obj_set_style_border_width(headerContext, 0, 0);
    lv_obj_set_style_shadow_width(headerContext, 0, 0);
    lv_obj_set_style_pad_all(headerContext, 0, 0);
    lv_obj_clear_flag(headerContext, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(headerContext, headerContextPressed, LV_EVENT_CLICKED,
                        nullptr);
    // One bold line is the complete top-bar context. The former subtitle and
    // battery row consumed half of a 40 px header without creating another
    // useful touch target on this display.
    lblPageName = label(headerContext, FONT_16, C_TEXT, 6, 10, "HAND");
    lv_obj_set_size(lblPageName, 172, 22);
    lv_obj_set_style_text_align(lblPageName, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(lblPageName, LV_LABEL_LONG_DOT);
    lblCtrl = label(headerContext, FONT_10, C_DIM,
                    8, 23, "OFFLINE/NONE");
    lv_obj_set_size(lblCtrl, 132, 14);
    lv_label_set_long_mode(lblCtrl, LV_LABEL_LONG_DOT);
    lblBatteryHeader = label(headerContext, FONT_10, C_FAINT,
                             146, 23, "--%");
    lv_obj_add_flag(lblCtrl, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(lblBatteryHeader, LV_OBJ_FLAG_HIDDEN);

    pgControl = makePage();
    pgJog     = makePage();
    pgClutch  = makePage();
    pgDevice  = makePage();
    pgSetup   = makePage();

    // ---------------- DRIVE: one-thumb touch steering ----------------
    jogPad = lv_obj_create(pgControl);
    // Full-height control well. The 44 px edge keys are large enough for one
    // thumb while leaving a 228x168 steering field -- substantially larger
    // than the old visually implied centre square.
    lv_obj_set_size(jogPad, 168, PAGE_H);
    lv_obj_set_pos(jogPad, 76, 0);
    lv_obj_set_style_radius(jogPad, 0, 0);
    lv_obj_set_style_bg_color(jogPad, lv_color_hex(C_BG), 0);
    lv_obj_set_style_border_width(jogPad, 0, 0);
    lv_obj_set_style_shadow_width(jogPad, 0, 0);
    lv_obj_set_style_pad_all(jogPad, 0, 0);
    lv_obj_clear_flag(jogPad, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(jogPad, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(jogPad, jogEvent, LV_EVENT_PRESSED, nullptr);
    lv_obj_add_event_cb(jogPad, jogEvent, LV_EVENT_PRESSING, nullptr);
    lv_obj_add_event_cb(jogPad, jogEvent, LV_EVENT_RELEASED, nullptr);
    lv_obj_add_event_cb(jogPad, jogEvent, LV_EVENT_PRESS_LOST, nullptr);

    // No fixed joystick target: the origin ring exists only while a finger is
    // down and is centred on the exact pickup position.
    jogDeadzone = lv_obj_create(jogPad);
    lv_obj_set_size(jogDeadzone, 74, 74);
    lv_obj_set_pos(jogDeadzone, 77, 47);
    lv_obj_set_style_radius(jogDeadzone, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(jogDeadzone, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(jogDeadzone, 1, 0);
    lv_obj_set_style_border_color(jogDeadzone, lv_color_hex(C_EDGE), 0);
    lv_obj_set_style_border_opa(jogDeadzone, LV_OPA_70, 0);
    lv_obj_set_style_pad_all(jogDeadzone, 0, 0);
    lv_obj_clear_flag(jogDeadzone, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(jogDeadzone, LV_OBJ_FLAG_HIDDEN);

    jogCrossH = lv_obj_create(jogDeadzone);
    lv_obj_set_size(jogCrossH, 52, 1);
    lv_obj_set_pos(jogCrossH, 11, 36);
    lv_obj_set_style_bg_color(jogCrossH, lv_color_hex(C_EDGE), 0);
    lv_obj_set_style_border_width(jogCrossH, 0, 0);
    lv_obj_clear_flag(jogCrossH, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    jogCrossV = lv_obj_create(jogDeadzone);
    lv_obj_set_size(jogCrossV, 1, 52);
    lv_obj_set_pos(jogCrossV, 36, 11);
    lv_obj_set_style_bg_color(jogCrossV, lv_color_hex(C_EDGE), 0);
    lv_obj_set_style_border_width(jogCrossV, 0, 0);
    lv_obj_clear_flag(jogCrossV, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);

    jogHomeDot = lv_obj_create(jogDeadzone);
    lv_obj_set_size(jogHomeDot, 8, 8);
    lv_obj_set_pos(jogHomeDot, 33, 33);
    lv_obj_set_style_radius(jogHomeDot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(jogHomeDot, lv_color_hex(C_TEXT), 0);
    lv_obj_set_style_border_width(jogHomeDot, 0, 0);
    lv_obj_clear_flag(jogHomeDot, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(jogHomeDot, LV_OBJ_FLAG_HIDDEN);

    jogKnob = lv_obj_create(jogPad);
    lv_obj_set_size(jogKnob, 42, 42);
    lv_obj_set_pos(jogKnob, 93, 63);
    lv_obj_set_style_radius(jogKnob, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(jogKnob, lv_color_hex(C_CYAN), 0);
    lv_obj_set_style_border_width(jogKnob, 0, 0);
    lv_obj_set_style_shadow_width(jogKnob, 0, 0);
    lv_obj_clear_flag(jogKnob, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(jogKnob, LV_OBJ_FLAG_HIDDEN);
    // Persistent copy is limited to the action and release consequence.  Pose
    // is a single top line; live rate replaces the release line while moving.
    lblJog = label(jogPad, FONT_16, C_TEXT, 4, 63, "TOUCH + DRAG");
    lv_obj_set_size(lblJog, 160, 24);
    lv_obj_set_style_text_align(lblJog, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(lblJog, LV_LABEL_LONG_DOT);
    lv_obj_clear_flag(lblJog, LV_OBJ_FLAG_CLICKABLE);
    lblJogEvidence = label(jogPad, FONT_12, C_TEXT, 7, 8, "P  --");
    lv_obj_set_size(lblJogEvidence, 74, 20);
    lv_label_set_long_mode(lblJogEvidence, LV_LABEL_LONG_DOT);
    lv_obj_clear_flag(lblJogEvidence, LV_OBJ_FLAG_CLICKABLE);
    lblJogPanEvidence = label(jogPad, FONT_12, C_TEXT, 87, 8, "T  --");
    lv_obj_set_size(lblJogPanEvidence, 74, 20);
    lv_obj_set_style_text_align(lblJogPanEvidence, LV_TEXT_ALIGN_RIGHT, 0);
    lv_label_set_long_mode(lblJogPanEvidence, LV_LABEL_LONG_DOT);
    lblJogTravel = label(jogPad, FONT_10, C_DIM, 4, 145, "LIFT TO STOP");
    lv_obj_set_size(lblJogTravel, 160, 16);
    lv_obj_set_style_text_align(lblJogTravel, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(lblJogTravel, LV_LABEL_LONG_DOT);

    btnJogTiltOnly = keyBtn(pgControl, "TILT\nONLY", 6, 0, 64, 104,
                             FONT_12, &lblJogTiltOnly,
                             tiltOnlyPressed, LV_EVENT_CLICKED);
    btnJogPanOnly = keyBtn(pgControl, "PAN\nONLY", 250, 0, 64, 104,
                            FONT_12, &lblJogPanOnly,
                            panOnlyPressed, LV_EVENT_CLICKED);
    lv_obj_set_style_radius(btnJogTiltOnly, 0, 0);
    lv_obj_set_style_radius(btnJogPanOnly, 0, 0);
    keyBtn(pgControl, LV_SYMBOL_SETTINGS, 6, 112, 64, 56, FONT_22,
           &easeGearLabels[2], openEase, LV_EVENT_CLICKED, reinterpret_cast<void*>(4));
    keyBtn(pgControl, LV_SYMBOL_SETTINGS, 250, 112, 64, 56, FONT_22,
           &easeGearLabels[3], openEase, LV_EVENT_CLICKED, reinterpret_cast<void*>(5));

    driveLimitUp    = makeLimitBar(pgControl, 76, 0, 168, 5);
    driveLimitDown  = makeLimitBar(pgControl, 76, PAGE_H - 5, 168, 5);
    driveLimitLeft  = makeLimitBar(pgControl, 76, 0, 5, PAGE_H);
    driveLimitRight = makeLimitBar(pgControl, 239, 0, 5, PAGE_H);

    // FEEL shares the large SETTINGS cards; less frequent choices scroll.
    jogFeelSheet = makeConfigPane(pgControl, true);
    btnJogSpeed = configTile(jogFeelSheet, LV_SYMBOL_PLAY, "SPEED", "NORMAL",
                             6, 4, 150, 72, &lblJogSpeedTxt,
                             jogSpeedPressed);
    btnJogSmooth = configTile(jogFeelSheet, LV_SYMBOL_LOOP, "SMOOTH", "FLUID",
                              164, 4, 150, 72, &lblJogSmoothTxt,
                              jogSmoothPressed);
    btnJogTilt = configTile(jogFeelSheet, LV_SYMBOL_UP, "TILT LOCK", "LIVE",
                            6, 84, 150, 72, &lblJogTiltTxt,
                            tiltLockPressed);
    btnJogPan = configTile(jogFeelSheet, LV_SYMBOL_SHUFFLE, "PAN LOCK", "LIVE",
                           164, 84, 150, 72, &lblJogPanTxt,
                           panLockPressed);
    btnJogDirection = configTile(jogFeelSheet, LV_SYMBOL_GPS, "DIRECTION", "NORMAL",
                                 6, 164, 150, 72, &lblJogDirection,
                                 settingsDirectionPressed);
    btnJogCenter = configTile(jogFeelSheet, LV_SYMBOL_HOME, "RECENTER", "HOLD",
                              164, 164, 150, 72, &lblJogCenter,
                              nullptr);
    lv_obj_add_event_cb(btnJogCenter, centerHoldEvent, LV_EVENT_PRESSED, nullptr);
    lv_obj_add_event_cb(btnJogCenter, centerHoldEvent, LV_EVENT_PRESSING, nullptr);
    lv_obj_add_event_cb(btnJogCenter, centerHoldEvent, LV_EVENT_RELEASED, nullptr);
    lv_obj_add_event_cb(btnJogCenter, centerHoldEvent, LV_EVENT_PRESS_LOST, nullptr);
    configTile(jogFeelSheet, LV_SYMBOL_SETTINGS, "EASING", "GLOBAL / AXES",
        6, 244, 308, 72, &easeGlobalLabels[1], openEase, reinterpret_cast<void*>(3));
    lv_obj_add_flag(jogFeelSheet, LV_OBJ_FLAG_HIDDEN);

    // ---------------- FOLLOW: momentary inertial steering ---------------
    dial = lv_btn_create(pgJog);
    lv_obj_set_size(dial, 168, PAGE_H);
    lv_obj_set_pos(dial, 76, 0);
    lv_obj_set_style_radius(dial, 0, 0);
    lv_obj_set_style_bg_color(dial, lv_color_hex(C_BG), 0);
    lv_obj_set_style_bg_grad_dir(dial, LV_GRAD_DIR_NONE, 0);
    lv_obj_set_style_border_width(dial, 2, 0);
    lv_obj_set_style_border_color(dial, lv_color_hex(C_EDGE), 0);
    lv_obj_set_style_shadow_width(dial, 0, 0);
    lv_obj_set_style_pad_all(dial, 0, 0);
    lv_obj_add_event_cb(dial, clutchEvent, LV_EVENT_PRESSED, nullptr);
    lv_obj_add_event_cb(dial, clutchEvent, LV_EVENT_PRESSING, nullptr);
    lv_obj_add_event_cb(dial, clutchEvent, LV_EVENT_RELEASED, nullptr);
    lv_obj_add_event_cb(dial, clutchEvent, LV_EVENT_PRESS_LOST, nullptr);

    lblState = label(dial, FONT_16, C_TEXT, 6, 58, "HOLD TO MOVE");
    lv_obj_set_size(lblState, 156, 28);
    lv_obj_set_style_text_align(lblState, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(lblState, LV_LABEL_LONG_DOT);
    lblHandDelta = label(dial, FONT_14, C_TEXT, 6, 31,
                         "P --      T --");
    lv_obj_set_size(lblHandDelta, 156, 22);
    lv_label_set_long_mode(lblHandDelta, LV_LABEL_LONG_DOT);
    lv_obj_add_flag(lblHandDelta, LV_OBJ_FLAG_HIDDEN);
    handCommandBar = lv_bar_create(dial);
    lv_obj_set_pos(handCommandBar, 8, 131);
    lv_obj_set_size(handCommandBar, 152, 7);
    lv_bar_set_range(handCommandBar, 0, 100);
    lv_bar_set_value(handCommandBar, 0, LV_ANIM_OFF);
    lv_obj_set_style_radius(handCommandBar, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(handCommandBar, lv_color_hex(C_EDGE), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(handCommandBar, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(handCommandBar, 0, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(handCommandBar, lv_color_hex(C_CYAN),
                              LV_PART_INDICATOR);
    lv_obj_add_flag(handCommandBar, LV_OBJ_FLAG_HIDDEN);
    lblHandMode = label(dial, FONT_10, C_DIM, 6, 145,
                        "FOLLOW / NORMAL / FLUID");
    lv_obj_set_size(lblHandMode, 156, 14);
    lv_obj_set_style_text_align(lblHandMode, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(lblHandMode, LV_LABEL_LONG_DOT);
    lblSub = label(dial, FONT_10, C_DIM, 6, 91,
                   "RELATIVE FROM CURRENT FRAME");
    lv_obj_set_size(lblSub, 156, 16);
    lv_obj_set_style_text_align(lblSub, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(lblSub, LV_LABEL_LONG_DOT);

    handReticle = lv_obj_create(dial);
    lv_obj_set_pos(handReticle, 50, 57);
    lv_obj_set_size(handReticle, 68, 68);
    lv_obj_set_style_radius(handReticle, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(handReticle, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(handReticle, 1, 0);
    lv_obj_set_style_border_color(handReticle, lv_color_hex(C_EDGE), 0);
    lv_obj_set_style_pad_all(handReticle, 0, 0);
    lv_obj_clear_flag(handReticle, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    handReticleH = lv_obj_create(handReticle);
    lv_obj_set_pos(handReticleH, 13, 33);
    lv_obj_set_size(handReticleH, 42, 2);
    lv_obj_set_style_border_width(handReticleH, 0, 0);
    lv_obj_set_style_bg_color(handReticleH, lv_color_hex(C_CYAN), 0);
    lv_obj_clear_flag(handReticleH, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    handReticleV = lv_obj_create(handReticle);
    lv_obj_set_pos(handReticleV, 33, 13);
    lv_obj_set_size(handReticleV, 2, 42);
    lv_obj_set_style_border_width(handReticleV, 0, 0);
    lv_obj_set_style_bg_color(handReticleV, lv_color_hex(C_CYAN), 0);
    lv_obj_clear_flag(handReticleV, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    handImuDot = lv_obj_create(dial);
    lv_obj_set_pos(handImuDot, 201, 13);
    lv_obj_set_size(handImuDot, 18, 18);
    lv_obj_set_style_radius(handImuDot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(handImuDot, lv_color_hex(C_CYAN), 0);
    lv_obj_set_style_border_width(handImuDot, 0, 0);
    lv_obj_clear_flag(handImuDot, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(handReticle, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(handImuDot, LV_OBJ_FLAG_HIDDEN);

    btnHandTiltOnly = keyBtn(pgJog, "TILT\nONLY", 6, 0, 64, 104,
                              FONT_12, &lblHandTiltOnly,
                              tiltOnlyPressed, LV_EVENT_CLICKED);
    btnHandPanOnly = keyBtn(pgJog, "PAN\nONLY", 250, 0, 64, 104,
                             FONT_12, &lblHandPanOnly,
                             panOnlyPressed, LV_EVENT_CLICKED);
    lv_obj_set_style_radius(btnHandTiltOnly, 0, 0);
    lv_obj_set_style_radius(btnHandPanOnly, 0, 0);
    keyBtn(pgJog, LV_SYMBOL_SETTINGS, 6, 112, 64, 56, FONT_22,
           &easeGearLabels[0], openEase, LV_EVENT_CLICKED, reinterpret_cast<void*>(1));
    keyBtn(pgJog, LV_SYMBOL_SETTINGS, 250, 112, 64, 56, FONT_22,
           &easeGearLabels[1], openEase, LV_EVENT_CLICKED, reinterpret_cast<void*>(2));

    handFeelSheet = makeConfigPane(pgJog, true);
    btnFollowTemplate = configTile(handFeelSheet, LV_SYMBOL_SETTINGS,
        "MODE", "HAND FOLLOW", 6, 4, 150, 72, &lblFollowTemplateTxt,
        templatePressed);
    btnFollowSensitivity = configTile(handFeelSheet, LV_SYMBOL_PLAY,
        "SPEED", "NORMAL", 164, 4, 150, 72, &lblFollowSensitivityTxt,
        speedPressed);
    btnFollowSmooth = configTile(handFeelSheet, LV_SYMBOL_LOOP,
        "SMOOTH", "FLUID", 6, 84, 150, 72, &lblFollowSmoothTxt,
        smoothPressed);
    btnHandAxisTune = configTile(handFeelSheet, LV_SYMBOL_SETTINGS,
        "AXIS TUNE", "T BAL / P BAL", 164, 84, 150, 72, &lblHandAxisTune,
        handAxisTunePressed);
    btnHandDirection = configTile(handFeelSheet, LV_SYMBOL_GPS,
        "DIRECTION", "NORMAL", 6, 164, 150, 72, &lblHandDirection,
        settingsDirectionPressed);
    btnHandCenter = configTile(handFeelSheet, LV_SYMBOL_HOME,
        "RECENTER", "HOLD", 164, 164, 150, 72, &lblHandCenter,
        nullptr);
    lv_obj_add_event_cb(btnHandCenter, centerHoldEvent, LV_EVENT_PRESSED, nullptr);
    lv_obj_add_event_cb(btnHandCenter, centerHoldEvent, LV_EVENT_PRESSING, nullptr);
    lv_obj_add_event_cb(btnHandCenter, centerHoldEvent, LV_EVENT_RELEASED, nullptr);
    lv_obj_add_event_cb(btnHandCenter, centerHoldEvent, LV_EVENT_PRESS_LOST, nullptr);
    configTile(handFeelSheet, LV_SYMBOL_SETTINGS, "EASING", "GLOBAL / AXES",
        6, 244, 308, 72, &easeGlobalLabels[0], openEase, reinterpret_cast<void*>(0));
    lv_obj_add_flag(handFeelSheet, LV_OBJ_FLAG_HIDDEN);

    // One sub-page, four decisions.  Splitting this into separate pan/tilt
    // pages would make comparison slow; four 72 px cards fit the immutable
    // canvas with an 8 px gutter and remain comfortable under a thumb.
    handAxisTuneSheet = makeConfigPane(pgJog, true);
    btnTiltResponse = configTile(handAxisTuneSheet, LV_SYMBOL_UP,
        "TILT GAIN", "0.50x", 6, 4, 150, 72, &lblTiltResponse,
        tiltResponsePressed);
    btnPanResponse = configTile(handAxisTuneSheet, LV_SYMBOL_SHUFFLE,
        "PAN GAIN", "0.50x", 164, 4, 150, 72, &lblPanResponse,
        panResponsePressed);
    btnTiltStability = configTile(handAxisTuneSheet, LV_SYMBOL_LOOP,
        "TILT WEIGHT", "MEDIUM", 6, 84, 150, 72, &lblTiltStability,
        tiltStabilityPressed);
    btnPanStability = configTile(handAxisTuneSheet, LV_SYMBOL_LOOP,
        "PAN WEIGHT", "MEDIUM", 164, 84, 150, 72, &lblPanStability,
        panStabilityPressed);
    lv_obj_t *axisHelp = label(handAxisTuneSheet, FONT_12, C_DIM, 8, 166,
        "GAIN: camera travel per hand turn.\nWEIGHT: softer starts and stops;\nFine also rejects small tremors.");
    lv_obj_set_size(axisHelp, 304, 56);
    lv_label_set_long_mode(axisHelp, LV_LABEL_LONG_WRAP);
    lv_obj_add_flag(handAxisTuneSheet, LV_OBJ_FLAG_HIDDEN);

    // ---------------- MOVES: one point editor + separate live runner ------
    // Four 40 px rows fill the available 168 px. Point navigation is the top
    // strip; each following row has one clear editing decision.
    btnMotionPrev = keyBtn(pgClutch, "<", 4, 4, 40, 40,
                           FONT_16, &lblMotionPrev,
                           motionPrevPressed, LV_EVENT_CLICKED);
    // Two programmes, A and B. Switching is a tap because it destroys
    // nothing: the one being left is already on flash.
    btnMotionSlot = keyBtn(pgClutch, "A", 260, 4, 56, 40,
                           FONT_16, &lblMotionSlot,
                           motionSlotPressed, LV_EVENT_CLICKED);
    btnMotionSelected = keyBtn(pgClutch, "P1 / EMPTY", 48, 4, 164, 40,
                               FONT_14, &lblMotionSelected,
                               nullptr, LV_EVENT_CLICKED);
    lv_obj_clear_flag(btnMotionSelected, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(lblMotionSelected, 156, 34);
    lv_obj_set_style_text_align(lblMotionSelected, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(lblMotionSelected, LV_LABEL_LONG_DOT);
    lv_obj_center(lblMotionSelected);
    btnMotionNext = keyBtn(pgClutch, ">", 216, 4, 40, 40,
                           FONT_16, &lblMotionNext,
                           motionNextPressed, LV_EVENT_CLICKED);
    btnMotionCapture = keyBtn(pgClutch, "SAVE POSITION", 4, 46, 153, 40,
                              FONT_14, &lblMotionCapture,
                              motionCapturePressed, LV_EVENT_CLICKED);
    btnMotionClear = keyBtn(pgClutch, "HOLD CLEAR", 4, 128, 104, 40,
                            FONT_12, &lblMotionClear,
                            motionClearPressed, LV_EVENT_PRESSED);
    lv_obj_set_style_text_align(lblMotionClear, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_add_event_cb(btnMotionClear, motionClearPressed, LV_EVENT_PRESSING, nullptr);
    lv_obj_add_event_cb(btnMotionClear, motionClearPressed, LV_EVENT_RELEASED, nullptr);
    lv_obj_add_event_cb(btnMotionClear, motionClearPressed, LV_EVENT_PRESS_LOST, nullptr);
    btnMotionGoto = keyBtn(pgClutch, "HOLD GOTO", 163, 46, 153, 40,
                           FONT_14, &lblMotionGoto,
                           motionGotoEvent, LV_EVENT_PRESSED);
    lv_obj_set_style_text_align(lblMotionGoto, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_add_event_cb(btnMotionGoto, motionGotoEvent, LV_EVENT_PRESSING, nullptr);
    lv_obj_add_event_cb(btnMotionGoto, motionGotoEvent, LV_EVENT_RELEASED, nullptr);
    lv_obj_add_event_cb(btnMotionGoto, motionGotoEvent, LV_EVENT_PRESS_LOST, nullptr);
    btnMotionTime = keyBtn(pgClutch, "TIME  3.0s", 4, 88, 153, 40,
                           FONT_14, &lblMotionTime,
                           motionTimePressed, LV_EVENT_CLICKED);
    btnMotionDwell = keyBtn(pgClutch, "CUE  0s", 163, 88, 153, 40,
                            FONT_14, &lblMotionDwell,
                            motionDwellPressed, LV_EVENT_CLICKED);
    btnMotionOpenRun = keyBtn(pgClutch, LV_SYMBOL_PLAY " PLAYBACK", 112, 128, 204, 40,
                              FONT_14, &lblMotionOpenRun,
                              motionOpenRunPressed, LV_EVENT_CLICKED);
    lv_obj_set_style_text_align(lblMotionOpenRun, LV_TEXT_ALIGN_CENTER, 0);

    // Fixed two-column thumb grid; all 24 points remain accessible by scroll.
    motionGalleryPane = lv_obj_create(pgClutch);
    lv_obj_set_size(motionGalleryPane, 320, PAGE_H);
    lv_obj_set_pos(motionGalleryPane, 0, 0);
    lv_obj_set_style_bg_color(motionGalleryPane, lv_color_hex(C_BG), 0);
    lv_obj_set_style_border_width(motionGalleryPane, 0, 0);
    lv_obj_set_style_radius(motionGalleryPane, 0, 0);
    lv_obj_set_style_pad_all(motionGalleryPane, 0, 0);
    lv_obj_set_style_pad_bottom(motionGalleryPane, 4, 0);
    lv_obj_set_scroll_dir(motionGalleryPane, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(motionGalleryPane, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_clear_flag(motionGalleryPane,
        LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM);
    for (uint8_t i = 0; i < osmo::MOTION_POINT_CAPACITY; ++i) {
        motionTiles[i] = keyBtn(motionGalleryPane, "+", 6, 4, 150, 72,
                                FONT_10, &lblMotionTiles[i], nullptr,
                                LV_EVENT_CLICKED);
        lv_obj_add_event_cb(motionTiles[i], motionTilePressed,
                            LV_EVENT_CLICKED,
                            reinterpret_cast<void *>(static_cast<intptr_t>(i)));
        lv_obj_set_style_radius(motionTiles[i], 3, 0);
        lv_obj_add_flag(motionTiles[i], LV_OBJ_FLAG_HIDDEN);
    }

    // A selected point gets the whole content page. Curve/time describe the
    // incoming leg; P1 explicitly describes the loop-return leg.
    motionPointPane = makeConfigPane(pgClutch, true);
    lblMotionPointPose = label(motionPointPane, FONT_14, C_TEXT,
                               6, 5, "P1 / NEW FRAME");
    lv_obj_set_size(lblMotionPointPose, 308, 18);
    lv_obj_set_style_text_align(lblMotionPointPose, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(lblMotionPointPose, LV_LABEL_LONG_DOT);
    lblMotionPointLeg = label(motionPointPane, FONT_10, C_DIM,
                              6, 25, "LOOP RETURN INTO P1");
    lv_obj_set_size(lblMotionPointLeg, 308, 14);
    lv_obj_set_style_text_align(lblMotionPointLeg, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(lblMotionPointLeg, LV_LABEL_LONG_DOT);

    // Large point cards. Starting a scroll cancels all pending holds.
    btnMotionPointTransition = configTile(motionPointPane, LV_SYMBOL_LOOP,
        "CURVE", "SMOOTH", 6, 42, 150, 72, &lblMotionPointTransition,
        motionTransitionPressed);
    btnMotionPointTime = configTile(motionPointPane, LV_SYMBOL_PLAY,
        "TIME", "3.0s", 164, 42, 150, 72, &lblMotionPointTime,
        motionTimePressed);
    btnMotionPointDwell = configTile(motionPointPane, LV_SYMBOL_PAUSE,
        "ARRIVAL", "0s", 6, 122, 150, 72, &lblMotionPointDwell,
        motionDwellPressed);
    btnMotionPointCapture = configTile(motionPointPane, LV_SYMBOL_PLUS,
        "POSITION", "SAVE", 164, 122, 150, 72, &lblMotionPointCapture,
        motionCapturePressed);
    btnMotionPointGoto = configTile(motionPointPane, LV_SYMBOL_PLAY,
        "PREVIEW", "HOLD TO MOVE", 6, 202, 150, 72, &lblMotionPointGoto,
        nullptr);
    lv_obj_add_event_cb(btnMotionPointGoto, motionGotoEvent,
                        LV_EVENT_PRESSED, nullptr);
    lv_obj_add_event_cb(btnMotionPointGoto, motionGotoEvent,
                        LV_EVENT_PRESSING, nullptr);
    lv_obj_add_event_cb(btnMotionPointGoto, motionGotoEvent,
                        LV_EVENT_RELEASED, nullptr);
    lv_obj_add_event_cb(btnMotionPointGoto, motionGotoEvent,
                        LV_EVENT_PRESS_LOST, nullptr);
    btnMotionPointRemove = configTile(motionPointPane, LV_SYMBOL_TRASH,
        "DELETE ONE", "HOLD", 164, 202, 150, 72, &lblMotionPointRemove, nullptr);
    lv_obj_add_event_cb(btnMotionPointRemove, motionClearPressed, LV_EVENT_PRESSED, nullptr);
    lv_obj_add_event_cb(btnMotionPointRemove, motionClearPressed, LV_EVENT_PRESSING, nullptr);
    lv_obj_add_event_cb(btnMotionPointRemove, motionClearPressed, LV_EVENT_RELEASED, nullptr);
    lv_obj_add_event_cb(btnMotionPointRemove, motionClearPressed, LV_EVENT_PRESS_LOST, nullptr);
    btnMotionPointClear = configTile(motionPointPane, LV_SYMBOL_TRASH,
        "CLEAR TAIL", "HOLD: Pn TO END", 6, 282, 308, 72, &lblMotionPointClear,
        nullptr);
    lv_obj_add_event_cb(btnMotionPointClear, motionClearPressed,
                        LV_EVENT_PRESSED, nullptr);
    lv_obj_add_event_cb(btnMotionPointClear, motionClearPressed,
                        LV_EVENT_PRESSING, nullptr);
    lv_obj_add_event_cb(btnMotionPointClear, motionClearPressed,
                        LV_EVENT_RELEASED, nullptr);
    lv_obj_add_event_cb(btnMotionPointClear, motionClearPressed,
                        LV_EVENT_PRESS_LOST, nullptr);
    lv_obj_add_flag(motionPointPane, LV_OBJ_FLAG_HIDDEN);

    // Live execution is deliberately sparse: one evidence strip, one dominant
    // ARM/GO surface, three honest counters and a final row of 40 px tools.
    // STOP remains pinned in the header.
    motionRunPane = lv_obj_create(pgClutch);
    lv_obj_set_size(motionRunPane, 320, PAGE_H);
    lv_obj_set_pos(motionRunPane, 0, 0);
    lv_obj_set_style_bg_color(motionRunPane, lv_color_hex(C_BG), 0);
    lv_obj_set_style_border_width(motionRunPane, 0, 0);
    lv_obj_set_style_radius(motionRunPane, 0, 0);
    lv_obj_set_style_pad_all(motionRunPane, 0, 0);
    lv_obj_clear_flag(motionRunPane, LV_OBJ_FLAG_SCROLLABLE);
    motionProgressBar = lv_bar_create(motionRunPane);
    lv_obj_set_pos(motionProgressBar, 4, 4);
    lv_obj_set_size(motionProgressBar, 312, 7);
    lv_bar_set_range(motionProgressBar, 0, 1000);
    lv_bar_set_value(motionProgressBar, 0, LV_ANIM_OFF);
    lv_obj_set_style_radius(motionProgressBar, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(motionProgressBar, lv_color_hex(C_EDGE), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(motionProgressBar, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(motionProgressBar, 0, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(motionProgressBar, lv_color_hex(C_CYAN),
                              LV_PART_INDICATOR);

    btnMotionRun = keyBtn(motionRunPane, LV_SYMBOL_PLAY " PLAY", 4, 15, 312, 97,
                          FONT_26, &lblMotionRun,
                          motionRunEvent, LV_EVENT_PRESSED);
    lv_obj_set_style_radius(btnMotionRun, 0, 0);
    lv_obj_set_style_border_width(btnMotionRun, 2, 0);
    lv_obj_set_style_border_color(btnMotionRun, lv_color_hex(C_AMBER), 0);
    lblMotionRunSummary = label(btnMotionRun, FONT_12, C_AMBER,
                                6, 7, "READY / 2 POINTS");
    lv_obj_set_size(lblMotionRunSummary, 296, 17);
    lv_obj_set_style_text_align(lblMotionRunSummary, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(lblMotionRunSummary, LV_LABEL_LONG_DOT);
    lblMotionRunSub = label(btnMotionRun, FONT_12, C_DIM,
                            6, 77, "GOES TO P1 FIRST");
    lv_obj_set_size(lblMotionRunSub, 296, 14);
    lv_obj_set_style_text_align(lblMotionRunSub, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(lblMotionRunSub, LV_LABEL_LONG_DOT);

    lblMotionElapsed = label(motionRunPane, FONT_10, C_DIM,
                             4, 115, "ELAPSED 0.0s");
    lv_obj_set_size(lblMotionElapsed, 100, 16);
    lblMotionLegProgress = label(motionRunPane, FONT_10, C_DIM,
                                 108, 115, "LEG 0%");
    lv_obj_set_size(lblMotionLegProgress, 104, 16);
    lv_obj_set_style_text_align(lblMotionLegProgress, LV_TEXT_ALIGN_CENTER, 0);
    lblMotionCueCount = label(motionRunPane, FONT_10, C_DIM,
                              216, 115, "CUES 0");
    lv_obj_set_size(lblMotionCueCount, 100, 16);
    lv_obj_set_style_text_align(lblMotionCueCount, LV_TEXT_ALIGN_RIGHT, 0);

    // BACK already returns to the gallery, so the first runner tool is the
    // saved programme slot instead of a duplicate exit button.
    btnMotionEdit = keyBtn(motionRunPane, "A", 4, 128, 92, 40,
                           FONT_12, &lblMotionEdit,
                           motionSlotPressed, LV_EVENT_CLICKED);
    btnMotionRepeat = keyBtn(motionRunPane, "ONCE", 100, 128, 92, 40,
                             FONT_12, &lblMotionRepeat,
                             motionRepeatPressed, LV_EVENT_CLICKED);
    btnMotionHome = keyBtn(motionRunPane, "HOLD P1", 196, 128, 120, 40,
                           FONT_12, &lblMotionHome,
                           motionHomeEvent, LV_EVENT_PRESSED);
    lv_obj_add_event_cb(btnMotionHome, motionHomeEvent, LV_EVENT_PRESSING, nullptr);
    lv_obj_add_event_cb(btnMotionHome, motionHomeEvent, LV_EVENT_RELEASED, nullptr);
    lv_obj_add_event_cb(btnMotionHome, motionHomeEvent, LV_EVENT_PRESS_LOST, nullptr);
    lv_obj_add_event_cb(btnMotionRun, motionRunEvent, LV_EVENT_PRESSING, nullptr);
    lv_obj_add_event_cb(btnMotionRun, motionRunEvent, LV_EVENT_RELEASED, nullptr);
    lv_obj_add_event_cb(btnMotionRun, motionRunEvent, LV_EVENT_PRESS_LOST, nullptr);
    lv_obj_add_flag(motionRunPane, LV_OBJ_FLAG_HIDDEN);

    batteryRefusalPane = lv_obj_create(pgClutch);
    lv_obj_set_size(batteryRefusalPane, 320, PAGE_H);
    lv_obj_set_pos(batteryRefusalPane, 0, 0);
    lv_obj_set_style_bg_color(batteryRefusalPane, lv_color_hex(C_BG), 0);
    lv_obj_set_style_border_width(batteryRefusalPane, 0, 0);
    lv_obj_set_style_radius(batteryRefusalPane, 0, 0);
    lv_obj_set_style_pad_all(batteryRefusalPane, 0, 0);
    lv_obj_clear_flag(batteryRefusalPane, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t *refusalCard = lv_obj_create(batteryRefusalPane);
    lv_obj_set_pos(refusalCard, 4, 4);
    lv_obj_set_size(refusalCard, 312, 106);
    lv_obj_set_style_bg_color(refusalCard, lv_color_hex(C_SURF_LO), 0);
    lv_obj_set_style_border_width(refusalCard, 2, 0);
    lv_obj_set_style_border_color(refusalCard, lv_color_hex(C_AMBER), 0);
    lv_obj_set_style_radius(refusalCard, 0, 0);
    lv_obj_set_style_pad_all(refusalCard, 0, 0);
    lv_obj_clear_flag(refusalCard,
                      LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t *refusalTitle = label(refusalCard, FONT_10, C_AMBER,
                                   6, 8, "CHARGE TO PLAY");
    lv_obj_set_size(refusalTitle, 296, 14);
    lv_obj_set_style_text_align(refusalTitle, LV_TEXT_ALIGN_CENTER, 0);
    lblBatteryRefusalPct = label(refusalCard, FONT_22, C_AMBER,
                                 6, 30, "CORE2 AT --%");
    lv_obj_set_size(lblBatteryRefusalPct, 296, 28);
    lv_obj_set_style_text_align(lblBatteryRefusalPct, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_t *refusalWhy = label(refusalCard, FONT_10, C_DIM, 6, 67,
        "A MOVE THAT DIES MID-TAKE IS WORSE\nTHAN ONE THAT NEVER STARTS");
    lv_obj_set_size(refusalWhy, 296, 30);
    lv_obj_set_style_text_align(refusalWhy, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(refusalWhy, LV_LABEL_LONG_WRAP);
    btnRefusalHand = keyBtn(batteryRefusalPane,
                            "HAND CLUTCH STILL AVAILABLE  >",
                            4, 116, 312, 48, FONT_12, &lblRefusalHand,
                            refusalHandPressed, LV_EVENT_CLICKED);
    lv_obj_add_flag(batteryRefusalPane, LV_OBJ_FLAG_HIDDEN);

    // ---------------- settings: scrollable icon hub ----------------------
    // Every pane is a full-size overlay on SETTINGS. Only the two grid panes
    // scroll; focused leaves keep all actions anchored in the 168 px content
    // rectangle. The root is deliberately reset to CAMERA whenever reopened.
    for (uint8_t i = 0; i < SET_COUNT; i++) {
        pgSetPanes[i] = makeConfigPane(
            pgSetup, i == SET_MENU || i == SET_CONTROLS);
        if (i != SET_MENU) lv_obj_add_flag(pgSetPanes[i], LV_OBJ_FLAG_HIDDEN);
    }

    // Two columns use the whole width while retaining 6 px outer margins and
    // an 8 px safety gutter. Two rows are fully visible; the paired third is
    // swipe away and makes the scroll affordance self-evident.
    btnSettingsCamera = configTile(pgSetPanes[SET_MENU], LV_SYMBOL_WIFI,
        "CAMERA", "OPEN MANAGER", 6, 4, 150, 72,
        &lblSettingsCameraState, settingsCameraPressed);
    btnSettingsControls = configTile(pgSetPanes[SET_MENU], LV_SYMBOL_SETTINGS,
        "CONTROL", "HAND + JOG", 164, 4, 150, 72,
        &lblSettingsControlsState, setMenuPressed,
        (void *)(intptr_t)SET_CONTROLS);
    btnSettingsDisplay = configTile(pgSetPanes[SET_MENU], LV_SYMBOL_IMAGE,
        "DISPLAY", "BRIGHT + SLEEP", 6, 84, 150, 72,
        &lblSettingsDisplayState, setMenuPressed,
        (void *)(intptr_t)SET_DISPLAY);
    btnSettingsFeedback = configTile(pgSetPanes[SET_MENU], LV_SYMBOL_BELL,
        "FEEDBACK", "LED + SOUND", 164, 84, 150, 72,
        &lblSettingsFeedbackState, setMenuPressed,
        (void *)(intptr_t)SET_FEEDBACK);
    btnSettingsSystem = configTile(pgSetPanes[SET_MENU], LV_SYMBOL_BATTERY_FULL,
        "SYSTEM", "STATUS + ABOUT", 6, 164, 150, 72,
        &lblSettingsSystemState, setMenuPressed,
        (void *)(intptr_t)SET_SYSTEM);
    btnSettingsPower = configTile(pgSetPanes[SET_MENU], LV_SYMBOL_POWER,
        "POWER", "LINK + OFF", 164, 164, 150, 72,
        &lblSettingsPower, settingsPowerPressed);

    // CONTROL is the only nested category: the first two tiles are shortcuts
    // to the live shooting sheets, while DIRECTION is durable setup. There is
    // no second copy of the feel callbacks or state.
    btnSettingsHand = configTile(pgSetPanes[SET_CONTROLS], LV_SYMBOL_GPS,
        "HAND FEEL", "LIVE SHORTCUT", 6, 4, 150, 72,
        &lblSettingsHandState, settingsHandFeelPressed);
    btnSettingsJog = configTile(pgSetPanes[SET_CONTROLS], LV_SYMBOL_LOOP,
        "JOG FEEL", "LIVE SHORTCUT", 164, 4, 150, 72,
        &lblSettingsJogState, settingsJogFeelPressed);
    btnSettingsDirection = configTile(pgSetPanes[SET_CONTROLS], LV_SYMBOL_SHUFFLE,
        "DIRECTION", "TILT + PAN", 6, 84, 150, 72,
        &lblSettingsDirectionState, settingsDirectionPressed);

    // ---- display ----
    btnBright = configTile(pgSetPanes[SET_DISPLAY], LV_SYMBOL_EYE_OPEN,
                           "BRIGHTNESS", BRIGHT_NAMES[brightIdx],
                           6, 4, 150, 72, &lblBrightTxt, brightPressed);
    btnScreenTo = configTile(pgSetPanes[SET_DISPLAY], LV_SYMBOL_POWER,
                             "AUTO SLEEP", SCREEN_TIMEOUT_NAMES[screenTimeoutIdx],
                             164, 4, 150, 72, &lblScreenToTxt, screenToPressed);
    lv_obj_t *displayNote = label(pgSetPanes[SET_DISPLAY], FONT_12, C_DIM,
        8, 101, "First touch wakes only; it never activates a control");
    lv_obj_set_size(displayNote, 304, 34);
    lv_label_set_long_mode(displayNote, LV_LABEL_LONG_WRAP);

    // ---- feedback: status LEDs, sound and haptics belong together ----
    btnBeaconOn = configTile(pgSetPanes[SET_FEEDBACK], LV_SYMBOL_EYE_OPEN,
                             "STATUS LED", "ON", 6, 4, 150, 72,
                             &lblBeaconTxt, beaconPressed);
    btnLimitLeds = configTile(pgSetPanes[SET_FEEDBACK], LV_SYMBOL_WARNING,
                              "LIMIT LED", "ON", 164, 4, 150, 72,
                              &lblLimitLedsTxt, limitLedsPressed);
    btnSilent = configTile(pgSetPanes[SET_FEEDBACK], LV_SYMBOL_VOLUME_MAX,
                           "SOUND", "ON", 6, 84, 150, 72,
                           &lblSilentTxt, silentPressed);
    // HAPTIC leads to its own leaf: level, mounting and test need full-size
    // targets, and squeezing them into this feedback grid made a thumb menu.
    btnHapticSettings = configTile(pgSetPanes[SET_FEEDBACK], LV_SYMBOL_GPS,
                         "HAPTIC", HAPTIC_LEVEL_NAMES[hapticLevelIdx],
                         164, 84, 150, 72, &lblHapticSettings, setMenuPressed,
                         (void *)(intptr_t)SET_HAPTIC);

    // ---- haptic: comfort is persistent; camera mounting is hard-off ----
    btnHapticLevel = configTile(pgSetPanes[SET_HAPTIC], LV_SYMBOL_BELL,
                         "LEVEL", HAPTIC_LEVEL_NAMES[hapticLevelIdx],
                         6, 4, 150, 72, &lblHapticLevel, hapticLevelPressed);
    btnHapticMounting = configTile(pgSetPanes[SET_HAPTIC], LV_SYMBOL_GPS,
                         "MOUNTING", cameraInHand ? "MOUNTED OFF" : "REMOTE ON",
                         164, 4, 150, 72, &lblHapticMounting,
                         hapticMountingPressed);
    btnHapticTest = configTile(pgSetPanes[SET_HAPTIC], LV_SYMBOL_PLAY,
                         "TEST FEEL", "TAP TO PREVIEW", 6, 84, 308, 72,
                         &lblHapticTest, hapticTestPressed);

    // ---- direction: durable mapping, not transient shot-time axis locks ----
    btnInvT = configTile(pgSetPanes[SET_DIRECTION], LV_SYMBOL_UP,
                         "TILT INPUT", "NORMAL", 6, 4, 150, 72,
                         &lblInvTTxt, invTiltPressed);
    btnInvP = configTile(pgSetPanes[SET_DIRECTION], LV_SYMBOL_SHUFFLE,
                         "PAN INPUT", "NORMAL", 164, 4, 150, 72,
                         &lblInvPTxt, invPanPressed);
    btnResetFeel = configTile(pgSetPanes[SET_DIRECTION], LV_SYMBOL_REFRESH,
                              "FEEL PROFILE", "HOLD TO RESET",
                              6, 84, 308, 72, &lblResetFeelTxt, nullptr);
    lv_obj_add_event_cb(btnResetFeel, resetFeelPressed, LV_EVENT_PRESSED, nullptr);
    lv_obj_add_event_cb(btnResetFeel, resetFeelPressed, LV_EVENT_PRESSING, nullptr);
    lv_obj_add_event_cb(btnResetFeel, resetFeelPressed, LV_EVENT_RELEASED, nullptr);
    lv_obj_add_event_cb(btnResetFeel, resetFeelPressed, LV_EVENT_PRESS_LOST, nullptr);

    // ---- standalone camera device manager ----
    deviceHome = makeConfigPane(pgDevice, true);
    lblDeviceSummary = label(deviceHome, FONT_12, C_TEXT,
                             14, 17, "NO CAMERA");
    lv_obj_set_size(lblDeviceSummary, 194, 16);
    lv_label_set_long_mode(lblDeviceSummary, LV_LABEL_LONG_DOT);

    deviceEvidencePanel = lv_obj_create(deviceHome);
    lv_obj_set_pos(deviceEvidencePanel, 4, 4);
    lv_obj_set_size(deviceEvidencePanel, 312, 44);
    lv_obj_set_style_bg_color(deviceEvidencePanel, lv_color_hex(C_SURF_LO), 0);
    lv_obj_set_style_border_width(deviceEvidencePanel, 1, 0);
    lv_obj_set_style_border_color(deviceEvidencePanel, lv_color_hex(C_EDGE), 0);
    lv_obj_set_style_radius(deviceEvidencePanel, 0, 0);
    lv_obj_set_style_pad_all(deviceEvidencePanel, 0, 0);
    lv_obj_clear_flag(deviceEvidencePanel,
                      LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    static const char *evidenceNames[5] = {
        "BLE", "WI-FI", "SOCKETS", "TELEMETRY", "TALLY"
    };
    for (uint8_t i = 0; i < 5; ++i) {
        lblDeviceEvidenceName[i] = label(deviceEvidencePanel, FONT_10, C_DIM,
                                         7, 7 + i * 25,
                                         evidenceNames[i]);
        lblDeviceEvidenceState[i] = label(deviceEvidencePanel, FONT_10, C_FAINT,
                                          78, 7 + i * 25, "WAIT");
        lv_obj_set_size(lblDeviceEvidenceState[i], 104, 14);
        lv_obj_set_style_text_align(lblDeviceEvidenceState[i],
                                    LV_TEXT_ALIGN_RIGHT, 0);
        lv_label_set_long_mode(lblDeviceEvidenceState[i], LV_LABEL_LONG_DOT);
    }
    for (uint8_t i = 0; i < 5; ++i) {
        lv_obj_add_flag(lblDeviceEvidenceName[i], LV_OBJ_FLAG_HIDDEN);
        if (i) lv_obj_add_flag(lblDeviceEvidenceState[i], LV_OBJ_FLAG_HIDDEN);
    }
    lv_obj_set_pos(lblDeviceEvidenceState[0], 218, 14);
    lv_obj_set_size(lblDeviceEvidenceState[0], 82, 16);
    lv_obj_move_foreground(lblDeviceSummary);

    btnDeviceCameras = configTile(deviceHome, LV_SYMBOL_WIFI,
        "CAMERAS", "OPEN MANAGER", 6, 52, 150, 72,
        &lblDeviceCameras, deviceCamerasPressed);
    btnDeviceTools = configTile(deviceHome, LV_SYMBOL_SETTINGS,
        "R&D TOOLS", "UNCONFIRMED", 164, 52, 150, 72,
        &lblDeviceTools, deviceToolsPressed);
    btnDevicePower = configTile(deviceHome, LV_SYMBOL_POWER,
        "LINK / POWER", "DISCONNECT / CORE2 OFF", 6, 132, 308, 72,
        &lblDevicePower, devicePowerPressed);

    deviceTools = makeConfigPane(pgDevice, true);
    lv_obj_set_size(deviceTools, 320, PAGE_H);
    lv_obj_set_pos(deviceTools, 0, 0);
    lv_obj_set_style_bg_color(deviceTools, lv_color_hex(C_BG), 0);
    lv_obj_set_style_border_width(deviceTools, 0, 0);
    lv_obj_set_style_pad_all(deviceTools, 0, 0);
    lblToolResult = label(deviceTools, FONT_10, C_AMBER, 6, 8,
                          "FOCUS COMMAND NOT PROVEN BY CAMERA LINK");
    lv_obj_set_size(lblToolResult, 308, 30);
    lv_obj_set_style_text_align(lblToolResult, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(lblToolResult, LV_LABEL_LONG_WRAP);
    lblFocusMarks = label(deviceTools, FONT_12, C_DIM, 20, 48,
                          "LENS POSITION NOT AVAILABLE");
    lv_obj_set_size(lblFocusMarks, 280, 16);
    focusTrack = lv_bar_create(deviceTools);
    lv_obj_set_pos(focusTrack, 8, 70);
    lv_obj_set_size(focusTrack, 304, 8);
    lv_bar_set_range(focusTrack, 0, 100);
    lv_bar_set_value(focusTrack, 50, LV_ANIM_OFF);
    lv_obj_set_style_radius(focusTrack, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(focusTrack, lv_color_hex(C_EDGE), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(focusTrack, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(focusTrack, 0, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(focusTrack, lv_color_hex(C_CYAN),
                              LV_PART_INDICATOR);
    lv_obj_set_style_opa(focusTrack, LV_OPA_40, 0);
    lv_obj_add_flag(focusTrack, LV_OBJ_FLAG_HIDDEN);

    btnFocusAfS = configTile(deviceTools, LV_SYMBOL_EYE_OPEN, "AF-S", "REQUEST",
        6, 84, 150, 72, &lblFocusAfS, nullptr);
    lv_obj_add_event_cb(btnFocusAfS, cameraToolPressed, LV_EVENT_CLICKED,
        (void *)(intptr_t)osmo::CameraAction::FocusSingle);
    btnFocusAfC = configTile(deviceTools, LV_SYMBOL_EYE_OPEN, "AF-C", "REQUEST",
        164, 84, 150, 72, &lblFocusAfC, nullptr);
    lv_obj_add_event_cb(btnFocusAfC, cameraToolPressed, LV_EVENT_CLICKED,
        (void *)(intptr_t)osmo::CameraAction::FocusContinuous);
    btnGimbalCenter = configTile(deviceTools, LV_SYMBOL_HOME, "CENTER", "REQUEST",
        6, 164, 150, 72, &lblGimbalCenter, nullptr);
    lv_obj_add_event_cb(btnGimbalCenter, cameraToolPressed, LV_EVENT_CLICKED,
        (void *)(intptr_t)osmo::CameraAction::GimbalRecenter);
    btnGimbalFollow = configTile(deviceTools, LV_SYMBOL_REFRESH, "FOLLOW", "REQUEST",
        164, 164, 150, 72, &lblGimbalFollow, nullptr);
    lv_obj_add_event_cb(btnGimbalFollow, cameraToolPressed, LV_EVENT_CLICKED,
        (void *)(intptr_t)osmo::CameraAction::GimbalFollow);
    lv_obj_add_flag(deviceTools, LV_OBJ_FLAG_HIDDEN);

    devicePower = lv_obj_create(pgDevice);
    lv_obj_set_size(devicePower, 320, PAGE_H);
    lv_obj_set_pos(devicePower, 0, 0);
    lv_obj_set_style_bg_color(devicePower, lv_color_hex(C_BG), 0);
    lv_obj_set_style_border_width(devicePower, 0, 0);
    lv_obj_set_style_pad_all(devicePower, 0, 0);
    lv_obj_clear_flag(devicePower, LV_OBJ_FLAG_SCROLLABLE);
    lblDeviceCapability = label(devicePower, &lv_font_montserrat_14, C_DIM,
                                8, 3, "CAMERA POWER OFF: UNSUPPORTED");
    btnLinkDisconnect = keyBtn(devicePower, "DISCONNECT", 6, 28, 150, 42,
                               &lv_font_montserrat_14, &lblLinkDisconnect,
                               linkDisconnectPressed, LV_EVENT_CLICKED);
    btnLinkReconnect = keyBtn(devicePower, "RECONNECT", 164, 28, 150, 42,
                              &lv_font_montserrat_14, &lblLinkReconnect,
                              linkReconnectPressed, LV_EVENT_CLICKED);
    btnPowerOff = keyBtn(devicePower, "POWER OFF?", 6, 78, 308, 48,
                         &lv_font_montserrat_16, &lblPowerOff,
                         powerOffEvent, LV_EVENT_PRESSED);
    lv_obj_add_event_cb(btnPowerOff, powerOffEvent, LV_EVENT_PRESSING, nullptr);
    lv_obj_add_event_cb(btnPowerOff, powerOffEvent, LV_EVENT_RELEASED, nullptr);
    lv_obj_add_event_cb(btnPowerOff, powerOffEvent, LV_EVENT_PRESS_LOST, nullptr);
    label(devicePower, FONT_12, C_FAINT, 8, 134,
          "HOLD TO POWER OFF / motion stops first");
    lv_obj_add_flag(devicePower, LV_OBJ_FLAG_HIDDEN);

    // Only the list scrolls. Actions remain anchored beneath the thumb and
    // every row/button is at least 40 px high on the 320x240 panel.
    cameraHub = lv_obj_create(pgDevice);
    lv_obj_set_size(cameraHub, 320, PAGE_H);
    lv_obj_set_pos(cameraHub, 0, 0);
    lv_obj_set_style_bg_color(cameraHub, lv_color_hex(C_BG), 0);
    lv_obj_set_style_border_width(cameraHub, 0, 0);
    lv_obj_set_style_pad_all(cameraHub, 0, 0);
    lv_obj_set_style_radius(cameraHub, 0, 0);
    lv_obj_clear_flag(cameraHub, LV_OBJ_FLAG_SCROLLABLE);

    cameraList = lv_obj_create(cameraHub);
    lv_obj_set_size(cameraList, 312, 108);
    lv_obj_set_pos(cameraList, 4, 4);
    lv_obj_set_style_bg_color(cameraList, lv_color_hex(C_SURF_LO), 0);
    lv_obj_set_style_border_width(cameraList, 0, 0);
    lv_obj_set_style_pad_all(cameraList, 0, 0);
    lv_obj_set_style_radius(cameraList, 0, 0);
    lv_obj_set_scroll_dir(cameraList, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(cameraList, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_clear_flag(cameraList,
        LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM);
    for (uint8_t i = 0; i < osmo::CAMERA_CATALOG_CAPACITY; ++i) {
        cameraRows[i] = keyBtn(cameraList, "", 0, i * 46, 308, 44,
                               &lv_font_montserrat_14,
                               &lblCameraRowName[i], nullptr,
                               LV_EVENT_CLICKED);
        lv_obj_set_style_radius(cameraRows[i], 0, 0);
        lv_obj_align(lblCameraRowName[i], LV_ALIGN_LEFT_MID, 10, 0);
        lv_obj_set_size(lblCameraRowName[i], 196, 20);
        lv_label_set_long_mode(lblCameraRowName[i], LV_LABEL_LONG_DOT);
        lblCameraRowState[i] = label(cameraRows[i], &lv_font_montserrat_14,
                                     C_DIM, 208, 12, "");
        lv_obj_set_size(lblCameraRowState[i], 84, 20);
        lv_obj_set_style_text_align(lblCameraRowState[i], LV_TEXT_ALIGN_RIGHT, 0);
        lv_label_set_long_mode(lblCameraRowState[i], LV_LABEL_LONG_DOT);
        lv_obj_add_event_cb(cameraRows[i], cameraRowPressed, LV_EVENT_CLICKED,
                            (void *)(intptr_t)i);
    }
    lblCameraEmpty = label(cameraList, &lv_font_montserrat_14, C_DIM,
                           12, 20, "NO CAMERAS NEARBY\nWake camera, then tap SCAN");
    lv_obj_set_size(lblCameraEmpty, 280, 64);
    lv_label_set_long_mode(lblCameraEmpty, LV_LABEL_LONG_WRAP);

    btnCameraScan = keyBtn(cameraHub, "SCAN", 4, 116, 100, 48,
                           &lv_font_montserrat_14, &lblCameraScan,
                           cameraScanPressed, LV_EVENT_CLICKED);
    btnCameraConnect = keyBtn(cameraHub, "CONNECT", 108, 116, 100, 48,
                              &lv_font_montserrat_14, &lblCameraConnect,
                              cameraConnectPressed, LV_EVENT_CLICKED);
    btnCameraForget = keyBtn(cameraHub, "FORGET", 212, 116, 104, 48,
                             &lv_font_montserrat_14, &lblCameraForget,
                             cameraForgetPressed, LV_EVENT_CLICKED);

    // Selected-device connection evidence. Credentials remain protocol-task
    // private; the UI gets only bounded identity, SSID, and failure detail.
    cameraProgress = lv_obj_create(pgDevice);
    lv_obj_set_size(cameraProgress, 320, PAGE_H);
    lv_obj_set_pos(cameraProgress, 0, 0);
    lv_obj_set_style_bg_color(cameraProgress, lv_color_hex(C_BG), 0);
    lv_obj_set_style_border_width(cameraProgress, 0, 0);
    lv_obj_set_style_pad_all(cameraProgress, 0, 0);
    lv_obj_set_style_radius(cameraProgress, 0, 0);
    lv_obj_clear_flag(cameraProgress, LV_OBJ_FLAG_SCROLLABLE);
    lblWifiSteps = label(cameraProgress, FONT_12, C_FAINT,
                         8, 8, "1  DISCOVER");
    lv_obj_set_size(lblWifiSteps, 304, 22);
    lv_label_set_long_mode(lblWifiSteps, LV_LABEL_LONG_DOT);
    lblWifiState = label(cameraProgress, FONT_12, C_DIM,
                          8, 38, "2  PAIR");
    lv_obj_set_size(lblWifiState, 304, 22);
    lv_label_set_long_mode(lblWifiState, LV_LABEL_LONG_DOT);
    lblWifiSsid = label(cameraProgress, FONT_12, C_DIM,
                         8, 68, "3  WI-FI");
    lv_obj_set_size(lblWifiSsid, 304, 22);
    lv_label_set_long_mode(lblWifiSsid, LV_LABEL_LONG_DOT);
    lblWifiDetail = label(cameraProgress, FONT_12, C_DIM,
                          8, 98, "4  SESSION");
    lv_label_set_long_mode(lblWifiDetail, LV_LABEL_LONG_DOT);
    lv_obj_set_size(lblWifiDetail, 304, 22);
    btnWifiRetry = keyBtn(cameraProgress, "RETRY", 163, 124, 153, 40,
           FONT_12, &lblWifiRetry,
           wifiReconnectPressed, LV_EVENT_CLICKED);
    btnCameraProgressBack = keyBtn(cameraProgress, "CAMERAS", 4, 124,
           153, 40, FONT_12, &lblCameraProgressBack,
           cameraProgressBackPressed, LV_EVENT_CLICKED);
    lv_obj_add_flag(cameraProgress, LV_OBJ_FLAG_HIDDEN);

    // Destructive local forget and link-disrupting scan/switch each get a
    // full confirmation surface. Forget requires a deliberate sustained hold.
    cameraConfirm = lv_obj_create(pgDevice);
    lv_obj_set_size(cameraConfirm, 320, PAGE_H);
    lv_obj_set_pos(cameraConfirm, 0, 0);
    lv_obj_set_style_bg_color(cameraConfirm, lv_color_hex(C_BG), 0);
    lv_obj_set_style_border_width(cameraConfirm, 0, 0);
    lv_obj_set_style_pad_all(cameraConfirm, 0, 0);
    lv_obj_set_style_radius(cameraConfirm, 0, 0);
    lv_obj_clear_flag(cameraConfirm, LV_OBJ_FLAG_SCROLLABLE);
    lblCameraConfirmTitle = label(cameraConfirm, &lv_font_montserrat_16,
                                  C_AMBER, 8, 12, "CONFIRM");
    lv_obj_set_size(lblCameraConfirmTitle, 304, 22);
    lblCameraConfirmBody = label(cameraConfirm, &lv_font_montserrat_14,
                                 C_DIM, 8, 42, "");
    lv_obj_set_size(lblCameraConfirmBody, 304, 62);
    lv_label_set_long_mode(lblCameraConfirmBody, LV_LABEL_LONG_WRAP);
    btnCameraConfirmCancel = keyBtn(cameraConfirm, "CANCEL", 6, 112,
                                    150, 44, &lv_font_montserrat_14,
                                    &lblCameraConfirmCancel,
                                    cameraConfirmCancelPressed,
                                    LV_EVENT_CLICKED);
    btnCameraConfirmAction = keyBtn(cameraConfirm, "CONTINUE", 164, 112,
                                    150, 44, &lv_font_montserrat_14,
                                    &lblCameraConfirmAction,
                                    cameraConfirmActionEvent, LV_EVENT_ALL);
    lv_obj_add_flag(cameraConfirm, LV_OBJ_FLAG_HIDDEN);

    // ---- system status: read-only cards use the same hierarchy without
    // pretending to be controls. The full-width firmware card avoids clipping
    // release identity on this small panel.
    lv_obj_t *sysBatteryTile = configTile(pgSetPanes[SET_SYSTEM],
        LV_SYMBOL_BATTERY_FULL, "BATTERY", "--%", 6, 4, 150, 72,
        &lblBattery, nullptr);
    lv_obj_t *sysUptimeTile = configTile(pgSetPanes[SET_SYSTEM],
        LV_SYMBOL_PLAY, "UPTIME", "0m 00s", 164, 4, 150, 72,
        &lblSysUptime, nullptr);
    lv_obj_t *sysFirmwareTile = configTile(pgSetPanes[SET_SYSTEM],
        LV_SYMBOL_FILE, "FIRMWARE", FW_VERSION, 6, 84, 308, 72,
        &lblFw, nullptr);
    lv_obj_clear_flag(sysBatteryTile, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(sysUptimeTile, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(sysFirmwareTile, LV_OBJ_FLAG_CLICKABLE);

    // Global fault content sits above every workspace but below the immutable
    // header and rail. DEVICE itself is left visible so its recovery evidence
    // and controls can be used after tapping the action below.
    faultPane = lv_obj_create(screenMain);
    lv_obj_set_size(faultPane, 320, PAGE_H);
    lv_obj_set_pos(faultPane, 0, HEADER_H);
    lv_obj_set_style_bg_color(faultPane, lv_color_hex(C_SURF_LO), 0);
    lv_obj_set_style_border_width(faultPane, 0, 0);
    lv_obj_set_style_radius(faultPane, 0, 0);
    lv_obj_set_style_pad_all(faultPane, 0, 0);
    lv_obj_clear_flag(faultPane, LV_OBJ_FLAG_SCROLLABLE);
    lblFaultReason = label(faultPane, FONT_12, C_DIM, 8, 12,
                           "TELEMETRY SILENT / CONTROL DISARMED");
    lv_obj_set_size(lblFaultReason, 304, 18);
    lv_obj_set_style_text_align(lblFaultReason, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(lblFaultReason, LV_LABEL_LONG_DOT);
    lblFaultRetry = label(faultPane, FONT_30, C_MAGENTA, 8, 37,
                          "RETRY READY");
    lv_obj_set_size(lblFaultRetry, 304, 38);
    lv_obj_set_style_text_align(lblFaultRetry, LV_TEXT_ALIGN_CENTER, 0);
    lblFaultPath = label(faultPane, FONT_10, C_FAINT, 8, 80,
        "RADIOS AND SOCKETS RESET FIRST\nA CLEAN LINK RETURNS DISARMED");
    lv_obj_set_size(lblFaultPath, 304, 30);
    lv_obj_set_style_text_align(lblFaultPath, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(lblFaultPath, LV_LABEL_LONG_WRAP);
    btnFaultDevice = keyBtn(faultPane, "DEVICE  >", 90, 120, 140, 44,
                            FONT_12, &lblFaultDevice,
                            faultDevicePressed, LV_EVENT_CLICKED);
    lv_obj_add_flag(faultPane, LV_OBJ_FLAG_HIDDEN);

    // Compact global fault flag. It shares the title row without replacing
    // the transport/owner subtitle; the full fault treatment is in content.
    faultBar = lv_obj_create(screenMain);
    lv_obj_set_size(faultBar, 82, 18);
    lv_obj_set_pos(faultBar, 180, 1);
    lv_obj_set_style_bg_color(faultBar, lv_color_hex(C_MAGENTA), 0);
    lv_obj_set_style_border_width(faultBar, 0, 0);
    lv_obj_set_style_radius(faultBar, 0, 0);
    lv_obj_set_style_pad_all(faultBar, 0, 0);
    lv_obj_clear_flag(faultBar, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lblFaultTxt = label(faultBar, FONT_10, C_BG, 3, 2, "");
    lv_obj_set_size(lblFaultTxt, 76, 13);
    // One line, scrolling: a 48-char standalone fault must read in full on
    // the pages that have no other fault text.
    lv_label_set_long_mode(lblFaultTxt, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_add_flag(faultBar, LV_OBJ_FLAG_HIDDEN);

    btnRecord = keyBtn(screenMain, "REC", 0, 0, 78, 40,
                       FONT_14, &lblRecordTxt,
                       recordEvent, LV_EVENT_PRESSED);
    lv_obj_add_event_cb(btnRecord, recordEvent, LV_EVENT_PRESSING, nullptr);
    lv_obj_add_event_cb(btnRecord, recordEvent, LV_EVENT_RELEASED, nullptr);
    lv_obj_add_event_cb(btnRecord, recordEvent, LV_EVENT_PRESS_LOST, nullptr);
    lv_obj_set_style_radius(btnRecord, 0, 0);
    lv_obj_set_style_border_width(btnRecord, 1, 0);
    lv_obj_set_style_border_side(btnRecord, LV_BORDER_SIDE_RIGHT, 0);
    lv_obj_set_style_border_color(btnRecord, lv_color_hex(C_EDGE), 0);

    // STOP stays visible in the header's dedicated rectangle and cannot cover
    // controls in DRIVE, FOLLOW, FEEL or SYSTEM.
    btnStop = keyBtn(screenMain, "STOP", 262, 0, 58, 40,
                     FONT_16, &lblStopTxt,
                     stopPressed, LV_EVENT_PRESSED);
    // STOP was built with the default instrument surface and then never
    // restyled by anything, so the one control that must not look like its
    // neighbour read exactly like SPD NORMAL. It owns a fill of its own; the
    // refresh tick recolours it for fault and for press acknowledgement.
    lv_obj_set_style_bg_color(btnStop, lv_color_hex(C_TEXT), 0);
    lv_obj_set_style_bg_color(btnStop, lv_color_hex(C_AMBER), LV_STATE_PRESSED);
    lv_obj_set_style_text_color(lblStopTxt, lv_color_hex(C_BG), 0);
    lv_obj_set_style_text_letter_space(lblStopTxt, 2, 0);
    lv_obj_set_style_radius(btnStop, 0, 0);
    lv_obj_set_style_border_width(btnStop, 2, 0);
    lv_obj_set_style_border_side(btnStop, LV_BORDER_SIDE_LEFT, 0);
    lv_obj_set_style_border_color(btnStop, lv_color_hex(C_TEXT), 0);


    // Soft-key legends, aligned with the three capacitive zones on the bezel
    // directly below the screen. Pressing a physical button is reliable in a
    // way that hunting a 50px on-screen tab never was, and the legend keeps
    // the meaning visible so nothing is memorised.
    // The legends are also touch targets, doing exactly what the hardware
    // button beneath them does. Same action, two ways in.
    // Visually narrow chevrons free the rail for a large, contextual middle
    // action. The physical A/C zones remain the broad navigation fallbacks.
    softL = keyBtn(screenMain, "<", 0, NAV_Y, 40, 32, FONT_16,
                   &softLTxt, softLeftCb, LV_EVENT_CLICKED);
    softReturnIcon = lv_line_create(softL);
    lv_line_set_points(softReturnIcon, RETURN_ICON_POINTS, 6);
    lv_obj_center(softReturnIcon);
    lv_obj_set_style_line_width(softReturnIcon, 2, 0);
    lv_obj_set_style_line_rounded(softReturnIcon, true, 0);
    lv_obj_set_style_line_color(softReturnIcon, lv_color_hex(C_BG), 0);
    lv_obj_clear_flag(softReturnIcon, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(softReturnIcon, LV_OBJ_FLAG_HIDDEN);
    softM = keyBtn(screenMain, "JOG", 40, NAV_Y, 240, 32, FONT_12,
                   &softMTxt, softMidEvent, LV_EVENT_CLICKED);
    softR = keyBtn(screenMain, ">", 280, NAV_Y, 40, 32, FONT_16,
                   &softRTxt, softRightCb, LV_EVENT_CLICKED);
    lblRail = label(screenMain, FONT_10, C_DIM, 40, NAV_Y + 9, "");
    lv_obj_set_size(lblRail, 240, 14);
    lv_obj_set_style_text_align(lblRail, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(lblRail, LV_LABEL_LONG_DOT);
    lv_obj_clear_flag(lblRail, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(lblRail, LV_OBJ_FLAG_HIDDEN);

    showPage(static_cast<uint8_t>(Workspace::Hand));
}

// BtnB and the matching middle rail are deterministic navigation: HAND <->
// JOG on the two shooting pages and HOME (HAND) everywhere else. Motion
// acquisition belongs only to the large central touch surfaces.
static void updateSoftKeys() {
    // The nav bar is global and its meaning never changes, with one exception:
    // inside a settings sub-page the left key backs out to the menu. Leaving
    // the whole page to escape a sub-page would throw away the operator's
    // place for no reason.
    const bool onHand = page == static_cast<uint8_t>(Workspace::Hand);
    const bool onJog = page == static_cast<uint8_t>(Workspace::Jog);
    lv_obj_clear_flag(softM, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_pos(softL, 0, NAV_Y);   lv_obj_set_size(softL, 40, 32);
    lv_obj_set_pos(softM, 40, NAV_Y);  lv_obj_set_size(softM, 240, 32);
    lv_obj_set_pos(softR, 280, NAV_Y); lv_obj_set_size(softR, 40, 32);

    const bool deviceSub = page == static_cast<uint8_t>(Workspace::Device) &&
                           cameraView != CAM_DEVICE_HOME;
    const osmo::DirectStatus navStatus = osmo::directCamera.status();
    const bool runLocked = navStatus.motionProgramArmed ||
                           navStatus.motionProgramActive;
    const bool inSub =
        (page == static_cast<uint8_t>(Workspace::Jog) && jogFeelOpen) ||
        (page == static_cast<uint8_t>(Workspace::Hand) && handFeelOpen) ||
        ((motionUiView == MotionUiView::Point || motionRunView) && !runLocked &&
         page == static_cast<uint8_t>(Workspace::Moves)) ||
        easeOpen || deviceSub ||
        (page == static_cast<uint8_t>(Workspace::Settings) && setSub != SET_MENU);
    lv_label_set_text(softLTxt, inSub ? "" : "<");
    if (inSub) lv_obj_clear_flag(softReturnIcon, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_add_flag(softReturnIcon, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_text(softRTxt, ">");
    lv_label_set_text(softMTxt, onHand && !easeOpen && !handFeelOpen ? "JOG" : onJog && !easeOpen && !jogFeelOpen ? "CAMERA" : "HOME / HAND");
    setKey(softM, softMTxt, false, C_TEXT, true);
    setKey(softL, softLTxt, inSub, C_AMBER, true);
    if (lblRail) lv_obj_add_flag(lblRail, LV_OBJ_FLAG_HIDDEN);

    const bool feelContext = page == static_cast<uint8_t>(Workspace::Jog) ||
                             page == static_cast<uint8_t>(Workspace::Hand) ||
                             page == static_cast<uint8_t>(Workspace::Moves);
    if (feelContext) lv_obj_add_flag(headerContext, LV_OBJ_FLAG_CLICKABLE);
    else             lv_obj_clear_flag(headerContext, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_border_color(headerContext, lv_color_hex(
        jogFeelOpen || handFeelOpen || motionUiView != MotionUiView::Gallery
            ? C_AMBER : C_EDGE), 0);
}

struct UiPresentation { const char *word; uint32_t colour; };

// The visible state is derived from evidence, not merely from a serial host
// being present. In direct mode each connection phase says what the operator
// can do next; READY is reserved for a camera session with fresh telemetry.
static UiPresentation reducePresentation() {
    if (rig.fault[0]) return {"FAULT", C_MAGENTA};
    if (rig.direct) {
        switch (rig.directPhase) {
            case osmo::DirectPhase::Starting:
                return {"START", C_DIM};
            case osmo::DirectPhase::DeviceIdle:    return {"CHOOSE", C_AMBER};
            case osmo::DirectPhase::BleScan:       return {"SCAN", C_DIM};
            case osmo::DirectPhase::RetryWait:     return {"RETRY", C_AMBER};
            case osmo::DirectPhase::BlePair:       return {"PAIR", C_AMBER};
            case osmo::DirectPhase::AwaitApproval: return {"APPROVE", C_AMBER};
            case osmo::DirectPhase::WifiJoin:      return {"WIFI", C_AMBER};
            case osmo::DirectPhase::TcpPoke:
            case osmo::DirectPhase::UdpHandshake:
            case osmo::DirectPhase::Registering:   return {"LINK", C_AMBER};
            case osmo::DirectPhase::Ready:
                if (!rig.telemetry) return {"TELEMETRY", C_AMBER};
                break;
            case osmo::DirectPhase::Off:
            case osmo::DirectPhase::Stopping:      return {"OFFLINE", C_DIM};
        }
    } else {
        if (!rig.linked) return {"OFFLINE", C_DIM};
        if (!rig.camera) return {"CAMERA?", C_AMBER};
        if (!rig.telemetry) return {"TELEMETRY", C_AMBER};
    }

    const bool clutchIndicated = clutchHeld && rig.linked &&
        !strcmp(rig.owner, "core2");
    if (clutchIndicated)     return {"STEERING", C_CYAN};
    if (rig.moving && !strcmp(rig.owner, "program"))
                              return {"PROGRAM", C_CYAN};
    if (rig.waitingCue >= 0) return {"CUED", C_AMBER};
    if (rig.moving)          return {"SETTLING", C_CYAN};
    if (rig.armed)           return {"ARMED", C_AMBER};
    return {"HOLD TO STEER", C_TEXT};
}

static uint8_t wifiStepForPhase(osmo::DirectPhase phase) {
    switch (phase) {
        case osmo::DirectPhase::Starting:
        case osmo::DirectPhase::BleScan:       return 1;
        case osmo::DirectPhase::DeviceIdle:    return 0;
        case osmo::DirectPhase::BlePair:       return 2;
        case osmo::DirectPhase::AwaitApproval: return 3;
        case osmo::DirectPhase::WifiJoin:      return 4;
        // Wi-Fi already has an address before TCP :7001 is opened.  Treat the
        // TCP poke as the first camera-session step, not as IP acquisition.
        case osmo::DirectPhase::TcpPoke:       return 6;
        case osmo::DirectPhase::UdpHandshake:
        case osmo::DirectPhase::Registering:   return 6;
        case osmo::DirectPhase::Ready:         return 7;
        default:                               return 0;
    }
}

// Seven concrete hand-off steps. Completed work remains visible after a
// failure because DirectStatus preserves the phase that entered RetryWait.
// Wi-Fi attempt/result/channel evidence comes from the networking task; the UI
// never invents a countdown and never receives the WPA key.
static void wifiProgress(uint8_t &step, const char *&state,
                         uint32_t &colour, bool &blocked) {
    step = 0;
    state = "OFFLINE - TAP RETRY";
    colour = C_DIM;
    blocked = false;

    if (!rig.direct) {
        if (!rig.linked) { blocked = true; return; }
        if (!rig.camera) { step = 6; state = "6/7 CAMERA LINK"; colour = C_AMBER; return; }
        if (!rig.telemetry) { step = 7; state = "7/7 WAIT TELEMETRY"; colour = C_AMBER; return; }
        step = 8; state = "7/7 CAMERA READY"; colour = C_GREEN; return;
    }

    switch (rig.directPhase) {
        case osmo::DirectPhase::DeviceIdle:
            state = "CHOOSE A CAMERA"; colour = C_AMBER; break;
        case osmo::DirectPhase::Starting:
        case osmo::DirectPhase::BleScan:
            step = 1; state = "1/7 DISCOVER CAMERA"; colour = C_AMBER; break;
        case osmo::DirectPhase::BlePair:
            step = 2; state = "2/7 PAIR CAMERA"; colour = C_AMBER; break;
        case osmo::DirectPhase::AwaitApproval:
            step = 3; state = "3/7 APPROVE / CREDS"; colour = C_AMBER; break;
        case osmo::DirectPhase::WifiJoin:
            step = 4; state = "4/7 JOIN 2.4 GHz"; colour = C_AMBER; break;
        case osmo::DirectPhase::TcpPoke:
            step = 6; state = "6/7 OPEN CAMERA SESSION"; colour = C_AMBER; break;
        case osmo::DirectPhase::UdpHandshake:
        case osmo::DirectPhase::Registering:
            step = 6; state = "6/7 CAMERA LINK"; colour = C_AMBER; break;
        case osmo::DirectPhase::Ready:
            step = 7;
            if (rig.telemetry) { step = 8; state = "7/7 CAMERA READY"; colour = C_GREEN; }
            else { state = "7/7 WAIT TELEMETRY"; colour = C_AMBER; }
            break;
        case osmo::DirectPhase::RetryWait:
            step = wifiStepForPhase(rig.directBlockedAt);
            blocked = true;
            colour = C_MAGENTA;
            switch (rig.directBlockedAt) {
                case osmo::DirectPhase::Starting:
                case osmo::DirectPhase::BleScan:
                    state = "1/7 CAMERA NOT FOUND"; break;
                case osmo::DirectPhase::BlePair:
                    state = "2/7 PAIRING FAILED"; break;
                case osmo::DirectPhase::AwaitApproval:
                    state = "3/7 APPROVAL NEEDED"; break;
                case osmo::DirectPhase::WifiJoin:
                    state = !strcmp(rig.wifiReason, "AP not visible")
                        ? "4/7 2.4G AP NOT FOUND"
                        : !strcmp(rig.wifiReason, "authentication failed")
                            ? "4/7 WI-FI AUTH FAILED"
                            : "4/7 WI-FI JOIN FAILED";
                    break;
                case osmo::DirectPhase::TcpPoke:
                    state = "6/7 SESSION OPEN FAILED"; break;
                case osmo::DirectPhase::UdpHandshake:
                case osmo::DirectPhase::Registering:
                    state = "6/7 CAMERA LINK FAILED"; break;
                case osmo::DirectPhase::Ready:
                    state = "7/7 TELEMETRY LOST"; break;
                default:
                    state = "BLOCKED - TAP RETRY"; break;
            }
            break;
        case osmo::DirectPhase::Stopping:
            state = "STOPPING CAMERA LINK"; colour = C_DIM; break;
        case osmo::DirectPhase::Off:
            blocked = true; break;
    }
}

// DirectStatus.detail is bounded, protocol-owned operator text. Credentials
// never enter it; this extra guard also prevents a future protocol regression
// from turning the panel into a password display.
static char asciiLower(char ch) {
    return ch >= 'A' && ch <= 'Z' ? static_cast<char>(ch + ('a' - 'A')) : ch;
}

static bool containsAsciiNoCase(const char *text, const char *needle) {
    if (!text || !needle || !needle[0]) return false;
    for (const char *start = text; *start; ++start) {
        const char *a = start;
        const char *b = needle;
        while (*a && *b && asciiLower(*a) == asciiLower(*b)) {
            ++a;
            ++b;
        }
        if (!*b) return true;
    }
    return false;
}

static const char *safeWifiDetail(const char *detail) {
    if (!detail || !detail[0]) return "waiting for camera evidence";
    if (containsAsciiNoCase(detail, "pass") ||
        containsAsciiNoCase(detail, "credential") ||
        containsAsciiNoCase(detail, "key"))
        return "camera credentials received (hidden)";
    return detail;
}

// -72 dBm is an engineer's unit. The list is an operator surface, and the
// only decision it supports is "is this one close enough to pair with".
static const char *signalWord(int rssi) {
    if (rssi >= -55) return "STRONG";
    if (rssi >= -70) return "GOOD";
    if (rssi >= -80) return "FAIR";
    return "WEAK";
}

static void chooseCameraHighlight() {
    if (cameraById(cameraHighlightedId)) return;
    cameraHighlightedId = 0;
    // Preserve operator intent first, then live evidence, then a saved device,
    // then the strongest remaining nearby result (catalog order is RSSI-first).
    for (uint8_t pass = 0; pass < 4 && !cameraHighlightedId; ++pass) {
        for (uint8_t i = 0; i < cameraCatalogUi.count; ++i) {
            const osmo::CameraSummary &item = cameraCatalogUi.items[i];
            const bool match = pass == 0 ? item.selected
                             : pass == 1 ? item.current
                             : pass == 2 ? item.trusted
                                         : true;
            if (match) {
                cameraHighlightedId = item.id;
                break;
            }
        }
    }
}

static void refreshCameraHub() {
    chooseCameraHighlight();
    const bool hasManagerError = cameraCatalogUi.error[0] != '\0';
    static char rowName[osmo::CAMERA_CATALOG_CAPACITY][32];
    static char rowState[osmo::CAMERA_CATALOG_CAPACITY][20];
    for (uint8_t i = 0; i < osmo::CAMERA_CATALOG_CAPACITY; ++i) {
        if (i >= cameraCatalogUi.count) {
            lv_obj_add_flag(cameraRows[i], LV_OBJ_FLAG_HIDDEN);
            continue;
        }
        lv_obj_clear_flag(cameraRows[i], LV_OBJ_FLAG_HIDDEN);
        // Keep a persisted-action failure visible without sacrificing access
        // to the saved cameras below it. The list is the only scroll surface.
        lv_obj_set_y(cameraRows[i], (i + (hasManagerError ? 1 : 0)) * 46);
        const osmo::CameraSummary &item = cameraCatalogUi.items[i];
        const bool highlighted = item.id == cameraHighlightedId;
        const char *name = item.name[0] ? item.name : "Unnamed DJI camera";
        const size_t addressLen = strlen(item.address);
        const char *addressTail = addressLen > 5
            ? item.address + addressLen - 5 : item.address;
        if (addressTail[0]) {
            snprintf(rowName[i], sizeof(rowName[i]), "%s%.13s  %s",
                     highlighted ? "> " : "  ", name, addressTail);
        } else {
            snprintf(rowName[i], sizeof(rowName[i]), "%s%.20s",
                     highlighted ? "> " : "  ", name);
        }
        if (item.current) {
            snprintf(rowState[i], sizeof(rowState[i]), "READY");
        } else if (item.selected && cameraCatalogUi.actionPending) {
            snprintf(rowState[i], sizeof(rowState[i]), "LINKING");
        } else if (item.trusted && item.nearby) {
            snprintf(rowState[i], sizeof(rowState[i]), "SAVED %s",
                     signalWord(item.rssi));
        } else if (item.trusted) {
            snprintf(rowState[i], sizeof(rowState[i]), "SAVED");
        } else if (item.nearby) {
            snprintf(rowState[i], sizeof(rowState[i]), "%s",
                     signalWord(item.rssi));
        } else if (item.verified) {
            snprintf(rowState[i], sizeof(rowState[i]), "VERIFIED");
        } else {
            snprintf(rowState[i], sizeof(rowState[i]), "OFFLINE");
        }
        lv_label_set_text(lblCameraRowName[i], rowName[i]);
        lv_label_set_text(lblCameraRowState[i], rowState[i]);
        lv_obj_set_style_bg_color(cameraRows[i],
            lv_color_hex(highlighted ? C_SURF_HI : C_SURF_LO), 0);
        lv_obj_set_style_text_color(lblCameraRowName[i], lv_color_hex(C_TEXT), 0);
        const uint32_t stateColour = item.current ? C_TEXT
            : (item.selected && cameraCatalogUi.actionPending) ? C_AMBER
            : C_DIM;
        lv_obj_set_style_text_color(lblCameraRowState[i],
                                    lv_color_hex(stateColour), 0);
    }

    static char empty[96];
    if (cameraCatalogUi.scanning) {
        snprintf(empty, sizeof(empty), "SCANNING NEARBY...\nResults appear here");
    } else if (hasManagerError) {
        snprintf(empty, sizeof(empty), "CAMERA ACTION FAILED\n%.64s",
                 safeWifiDetail(cameraCatalogUi.error));
    } else {
        snprintf(empty, sizeof(empty),
                 "NO CAMERAS NEARBY\nWake camera, then tap SCAN");
    }
    lv_label_set_text(lblCameraEmpty, empty);
    lv_obj_set_pos(lblCameraEmpty, 12, hasManagerError ? 4 : 20);
    lv_obj_set_style_text_color(lblCameraEmpty,
        lv_color_hex(hasManagerError ? C_MAGENTA : C_DIM), 0);
    if (cameraCatalogUi.count && !hasManagerError)
        lv_obj_add_flag(lblCameraEmpty, LV_OBJ_FLAG_HIDDEN);
    else
        lv_obj_clear_flag(lblCameraEmpty, LV_OBJ_FLAG_HIDDEN);

    const osmo::CameraSummary *selected = highlightedCamera();
    const bool busy = cameraCatalogUi.scanning || cameraCatalogUi.actionPending;
    lv_label_set_text(lblCameraScan, cameraCatalogUi.forgetPending
        ? "SAVING" : busy ? "CANCEL" : "SCAN");
    setKey(btnCameraScan, lblCameraScan,
           busy && !cameraCatalogUi.forgetPending, C_AMBER,
           !cameraCatalogUi.forgetPending);

    const bool connectAvailable = selected && !cameraCatalogUi.scanning &&
                                  !cameraCatalogUi.forgetPending;
    const char *connectWord = !selected ? "SELECT"
        : (selected->current || cameraCatalogUi.actionPending) ? "DETAILS"
        : currentCamera() ? "SWITCH" : "CONNECT";
    lv_label_set_text(lblCameraConnect, connectWord);
    setKey(btnCameraConnect, lblCameraConnect, false, C_TEXT, connectAvailable);

    const bool forgetAvailable = selected && !busy &&
        (selected->trusted || selected->current);
    lv_label_set_text(lblCameraForget, "FORGET");
    setKey(btnCameraForget, lblCameraForget, false, C_TEXT, forgetAvailable);
}

static void refreshCameraConfirm() {
    if (cameraView != CAM_SCAN_CONFIRM && cameraView != CAM_SWITCH_CONFIRM &&
        cameraView != CAM_FORGET_CONFIRM) return;
    const osmo::CameraSummary *target = cameraById(cameraConfirmId);
    const char *targetName = target && target->name[0]
        ? target->name : "selected camera";
    static char body[128];
    if (cameraView == CAM_SCAN_CONFIRM) {
        lv_label_set_text(lblCameraConfirmTitle, "SCAN FOR CAMERAS?");
        snprintf(body, sizeof(body),
                 "The current link will close.\nMotion stops before the radio scan.");
        lv_label_set_text(lblCameraConfirmAction, "STOP + SCAN");
    } else if (cameraView == CAM_SWITCH_CONFIRM) {
        const osmo::CameraSummary *current = currentCamera();
        const char *currentName = current && current->name[0]
            ? current->name : "current camera";
        lv_label_set_text(lblCameraConfirmTitle, "SWITCH CAMERA?");
        snprintf(body, sizeof(body), "%s  >  %s\nMotion stops before switching.",
                 currentName, targetName);
        lv_label_set_text(lblCameraConfirmAction, "SWITCH");
    } else {
        lv_label_set_text(lblCameraConfirmTitle, "FORGET FROM CORE2?");
        if (currentCamera()) {
            snprintf(body, sizeof(body),
                     "%s\nLink/motion stop. Camera may retain approval.",
                     targetName);
        } else {
            snprintf(body, sizeof(body),
                     "%s\nLocal only. Camera may retain approval.", targetName);
        }
        uint32_t held = cameraForgetHoldStartedMs
            ? (millis() - cameraForgetHoldStartedMs) * 100 / HOLD_DESTROY_MS : 0;
        if (held > 100) held = 100;
        static char hold[24];
        if (held) snprintf(hold, sizeof(hold), "HOLD %lu%%",
                           static_cast<unsigned long>(held));
        else      snprintf(hold, sizeof(hold), "HOLD TO FORGET");
        lv_label_set_text(lblCameraConfirmAction, hold);
    }
    lv_label_set_text(lblCameraConfirmBody, body);
    setKey(btnCameraConfirmAction, lblCameraConfirmAction,
           cameraView == CAM_FORGET_CONFIRM && cameraForgetHoldStartedMs,
           C_AMBER, true);
}

static const char *motionPhaseWord(osmo::MotionPhase phase) {
    switch (phase) {
        case osmo::MotionPhase::Empty:       return "EMPTY";
        case osmo::MotionPhase::Ready:       return "EDIT";
        case osmo::MotionPhase::Armed:       return "READY";
        case osmo::MotionPhase::Positioning: return "GOTO";
        case osmo::MotionPhase::Running:     return "RUNNING";
        case osmo::MotionPhase::Dwelling:    return "WAITING";
        case osmo::MotionPhase::Cued:        return "PAUSED";
        case osmo::MotionPhase::Complete:    return "COMPLETE";
        case osmo::MotionPhase::Aborted:     return "STOPPED";
        case osmo::MotionPhase::Fault:       return "FAULT";
        default:                             return "--";
    }
}

static void refreshMotionEditor(const osmo::MotionProgram &program,
                                const osmo::DirectStatus &status) {
    if (!motionGalleryPane || !motionPointPane) return;
    const bool editing = !status.motionProgramArmed &&
                         !status.motionProgramActive;
    const bool fresh = rig.direct && controlEvidenceReady();
    // More points add scrolling, never smaller hit targets.
    const uint8_t visible = min<uint8_t>(osmo::MOTION_POINT_CAPACITY,
        program.count + (program.count < osmo::MOTION_POINT_CAPACITY ? 1 : 0));
    const uint8_t columns = 2;
    const int tileW = 150;
    const int tileH = 72;
    const int stepX = 158;
    const int stepY = 80;

    for (uint8_t i = 0; i < osmo::MOTION_POINT_CAPACITY; ++i) {
        if (i >= visible) {
            lv_obj_add_flag(motionTiles[i], LV_OBJ_FLAG_HIDDEN);
            continue;
        }
        lv_obj_clear_flag(motionTiles[i], LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_pos(motionTiles[i], 6 + (i % columns) * stepX,
                       4 + (i / columns) * stepY);
        lv_obj_set_size(motionTiles[i], tileW, tileH);
        lv_obj_set_style_text_font(lblMotionTiles[i],
                                   FONT_14, 0);
        lv_obj_set_style_text_align(lblMotionTiles[i], LV_TEXT_ALIGN_CENTER, 0);
        static char tileText[osmo::MOTION_POINT_CAPACITY][32];
        if (i == program.count) {
            snprintf(tileText[i], sizeof(tileText[i]), "+\nNEW POINT");
            setKey(motionTiles[i], lblMotionTiles[i], false, C_AMBER,
                   editing && fresh && rig.poseValid);
            lv_obj_set_style_text_color(lblMotionTiles[i], lv_color_hex(
                editing && fresh && rig.poseValid ? C_AMBER : C_FAINT), 0);
        } else {
            const osmo::MotionPoint &point = program.points[i];
            snprintf(tileText[i], sizeof(tileText[i]),
                     i == 0 ? "P%u\nLOOP / %s" : "P%u\n%s / %.1fs",
                     i + 1, motionTransitionName(point.transition),
                     point.moveMs / 1000.0f);
            const bool selected = i == motionSelected;
            setKey(motionTiles[i], lblMotionTiles[i], selected, C_TEXT, true);
        }
        lv_label_set_text(lblMotionTiles[i], tileText[i]);
        lv_obj_center(lblMotionTiles[i]);
    }

    if (motionSelected > program.count)
        motionSelected = min<uint8_t>(program.count,
            osmo::MOTION_POINT_CAPACITY - 1);
    const bool existing = motionSelected < program.count;
    static char poseText[64], legText[64], curveText[20], timeText[16];
    static char dwellText[20], captureText[20], gotoText[20], clearText[20];
    if (existing) {
        const osmo::MotionPoint &point = program.points[motionSelected];
        snprintf(poseText, sizeof(poseText), "P%u  PAN %+.1f / TILT %+.1f",
                 motionSelected + 1, point.yaw, point.pitch);
        snprintf(legText, sizeof(legText), motionSelected == 0
                 ? "LOOP RETURN INTO P1" : "P%u -> P%u / INCOMING LEG",
                 motionSelected, motionSelected + 1);
        snprintf(curveText, sizeof(curveText), "%s",
                 motionTransitionName(point.transition));
        snprintf(timeText, sizeof(timeText), "%.1fs", point.moveMs / 1000.0f);
        if (point.holdForGo)
            snprintf(dwellText, sizeof(dwellText), "WAIT/GO");
        else
            snprintf(dwellText, sizeof(dwellText), "%.1fs",
                      point.dwellMs / 1000.0f);
        snprintf(captureText, sizeof(captureText), "RECAP P%u",
                 motionSelected + 1);
    } else {
        snprintf(poseText, sizeof(poseText), "P%u  NEW FRAME",
                 motionSelected + 1);
        snprintf(legText, sizeof(legText), motionSelected == 0
                 ? "CAPTURE P1 / LOOP CURVE FOLLOWS" : "AFTER P%u / CAPTURE FIRST",
                 motionSelected);
        snprintf(curveText, sizeof(curveText), "--");
        snprintf(timeText, sizeof(timeText), "--");
        snprintf(dwellText, sizeof(dwellText), "--");
        snprintf(captureText, sizeof(captureText), "SAVE P%u",
                 motionSelected + 1);
    }
    lv_label_set_text(lblMotionPointPose, poseText);
    lv_label_set_text(lblMotionPointLeg, legText);
    lv_label_set_text(lblMotionPointTransition, curveText);
    lv_label_set_text(lblMotionPointTime, timeText);
    lv_label_set_text(lblMotionPointDwell, dwellText);
    lv_label_set_text(lblMotionPointCapture, captureText);
    setConfigTile(btnMotionPointTransition, lblMotionPointTransition, false,
                  C_TEXT, editing && existing);
    setConfigTile(btnMotionPointTime, lblMotionPointTime, false,
                  C_TEXT, editing && existing);
    setConfigTile(btnMotionPointDwell, lblMotionPointDwell, false,
                  C_TEXT, editing && existing);
    setConfigTile(btnMotionPointCapture, lblMotionPointCapture, false, C_TEXT,
           editing && fresh && rig.poseValid && motionSelected <= program.count);

    const uint32_t gotoHeld = motionHoldStartedMs && !motionHoldIsRun
        ? min<uint32_t>(100, (millis() - motionHoldStartedMs) * 100 / HOLD_ACT_MS)
        : 0;
    snprintf(gotoText, sizeof(gotoText), gotoHeld ? "GOTO %lu%%" : "HOLD GOTO",
             static_cast<unsigned long>(gotoHeld));
    lv_label_set_text(lblMotionPointGoto, gotoText);
    setConfigTile(btnMotionPointGoto, lblMotionPointGoto, gotoHeld != 0, C_AMBER,
           editing && existing && fresh);
    const uint32_t clearHeld = clearHoldStartedMs
        ? min<uint32_t>(100, (millis() - clearHoldStartedMs) * 100 / HOLD_DESTROY_MS)
        : 0;
    if (clearHeld)
        snprintf(clearText, sizeof(clearText), "CLEAR %lu%%",
                 static_cast<unsigned long>(clearHeld));
    else
        snprintf(clearText, sizeof(clearText), "CLEAR P%u+ / HOLD",
                 motionSelected + 1);
    lv_label_set_text(lblMotionPointClear, removeSingleHold ? "HOLD: Pn TO END" : clearText);
    char removeText[20];
    if (clearHeld && removeSingleHold)
        snprintf(removeText, sizeof(removeText), "DELETE %lu%%", static_cast<unsigned long>(clearHeld));
    else
        snprintf(removeText, sizeof(removeText), "HOLD: P%u ONLY", motionSelected + 1);
    lv_label_set_text(lblMotionPointRemove, removeText);
    setConfigTile(btnMotionPointRemove, lblMotionPointRemove,
                  clearHeld && removeSingleHold, C_AMBER, editing && existing);
    setConfigTile(btnMotionPointClear, lblMotionPointClear, clearHeld != 0 && !removeSingleHold, C_AMBER,
           editing && existing);
}

static void refreshMotionWorkspace() {
    const osmo::MotionProgram program = osmo::directCamera.motionProgram();
    const osmo::DirectStatus status = osmo::directCamera.status();
    refreshMotionEditor(program, status);
    const uint32_t motionNow = millis();
    if (status.motionProgramArmed && !motionWasArmed) {
        motionElapsedMs = 0;
        motionElapsedTickMs = 0;
        motionCueCount = 0;
        motionLastCuePoint = 0;
    }
    if (status.motionProgramActive) {
        if (!motionWasActive) {
            if (!motionWasArmed) {
                motionElapsedMs = 0;
                motionCueCount = 0;
                motionLastCuePoint = 0;
            }
            motionElapsedTickMs = motionNow;
        }
        if (motionElapsedTickMs &&
            status.motionPhase != osmo::MotionPhase::Cued)
            motionElapsedMs += motionNow - motionElapsedTickMs;
        motionElapsedTickMs = motionNow;
        // Green and the counter both mean a cue was actually released, not
        // merely that the programme reached one and is still waiting.
        if (motionLastCuePoint && !status.cuePoint)
            ++motionCueCount;
        motionLastCuePoint = status.cuePoint;
    } else {
        // A final-point cue can transition directly to COMPLETE between UI
        // samples. Count that proven release, but never count STOP or fault.
        if (motionLastCuePoint &&
            status.motionPhase == osmo::MotionPhase::Complete)
            ++motionCueCount;
        motionElapsedTickMs = 0;
        motionLastCuePoint = 0;
    }
    motionWasActive = status.motionProgramActive;
    motionWasArmed = status.motionProgramArmed;
    if (motionSelected > program.count)
        motionSelected = min<uint8_t>(program.count,
            osmo::MOTION_POINT_CAPACITY - 1);

    const bool editingNow = !status.motionProgramArmed && !status.motionProgramActive;
    lv_label_set_text(lblMotionSlot, SLOT_NAMES[status.motionSlot % 2]);
    setKey(btnMotionSlot, lblMotionSlot, false, C_TEXT, editingNow);
    setKey(btnMotionPrev, lblMotionPrev, false, C_TEXT, motionSelected > 0);
    setKey(btnMotionNext, lblMotionNext, false, C_TEXT,
           motionSelected < min<uint8_t>(program.count,
               osmo::MOTION_POINT_CAPACITY - 1));

    // The top picker is deliberately one line. Timing and cue state live on
    // their own full-width controls below, so the selected point never turns
    // into a tiny paragraph.
    static char selectedText[112];
    if (program.error[0]) {
        snprintf(selectedText, sizeof(selectedText), "P%u  STORE ERROR",
                 motionSelected + 1);
    } else if (status.motionProgramFault) {
        snprintf(selectedText, sizeof(selectedText), "P%u  MOTION FAULT",
                 motionSelected + 1);
    } else if (status.motionProgramArmed || status.motionProgramActive ||
               status.motionPhase == osmo::MotionPhase::Complete) {
        snprintf(selectedText, sizeof(selectedText), "P%u/%u  %s  %u%%",
                 status.motionPoint, status.motionCount,
                 motionPhaseWord(status.motionPhase), status.motionProgress / 10);
    } else if (motionSelected < program.count) {
        const auto &point = program.points[motionSelected];
        snprintf(selectedText, sizeof(selectedText),
                 "P%u/%u  %+.0f / %+.0f",
                 motionSelected + 1, program.count, point.yaw, point.pitch);
    } else if (rig.poseValid) {
        snprintf(selectedText, sizeof(selectedText), "P%u  + NEW",
                 motionSelected + 1);
    } else {
        snprintf(selectedText, sizeof(selectedText), "P%u  EMPTY",
                 motionSelected + 1);
    }
    lv_label_set_text(lblMotionSelected, selectedText);
    const bool selectedAlarm = program.error[0] || status.motionProgramFault;
    const bool selectedLive = status.motionProgramActive ||
                              status.motionPhase == osmo::MotionPhase::Complete;
    const uint32_t selectedColour = selectedAlarm ? C_MAGENTA :
        selectedLive ? C_CYAN : status.motionProgramArmed ? C_AMBER : C_TEXT;
    // This is the selected point even while nothing is moving, so its passive
    // state is the canvas's white knockout selection. Semantic colours replace
    // white only when the programme is armed, live, complete, or faulted.
    setKey(btnMotionSelected, lblMotionSelected, true, selectedColour, true);

    const bool editing = !status.motionProgramArmed && !status.motionProgramActive;
    const bool existing = motionSelected < program.count;
    // Capture, goto and run are the standalone runtime's; under a USB host
    // they would light for a request nothing could execute.
    const bool fresh = rig.direct && controlEvidenceReady();
    setKey(btnMotionCapture, lblMotionCapture, false, C_TEXT,
           editing && fresh && motionSelected <= program.count);
    const uint32_t clearHeld = clearHoldStartedMs
        ? min<uint32_t>(100, (millis() - clearHoldStartedMs) * 100 / HOLD_DESTROY_MS) : 0;
    static char clearText[20];
    if (clearHeld) snprintf(clearText, sizeof(clearText), "CLEAR %lu%%",
                            static_cast<unsigned long>(clearHeld));
    else           snprintf(clearText, sizeof(clearText), "HOLD CLEAR");
    lv_label_set_text(lblMotionClear, clearText);
    setKey(btnMotionClear, lblMotionClear, clearHeld != 0, C_AMBER,
           editing && existing);
    setKey(btnMotionTime, lblMotionTime, false, C_TEXT, editing && existing);
    setKey(btnMotionDwell, lblMotionDwell, false, C_TEXT, editing && existing);
    const bool cueWaiting = rig.waitingCue >= 0;
    const bool hostTimelapse = rig.tlFrames > 0;
    const bool batteryBlocked = armBlockedByBattery();
    const bool showBatteryRefusal = motionRunView && batteryBlocked &&
        !status.motionProgramArmed && !status.motionProgramActive;
    if (showBatteryRefusal) {
        static char refusalPct[28];
        snprintf(refusalPct, sizeof(refusalPct), "CORE2 AT %d%%", batteryPct);
        lv_label_set_text(lblBatteryRefusalPct, refusalPct);
        lv_obj_clear_flag(batteryRefusalPane, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(batteryRefusalPane, LV_OBJ_FLAG_HIDDEN);
    }
    static char openRunText[16];
    if (cueWaiting)
        snprintf(openRunText, sizeof(openRunText), "GO P%d  >", rig.waitingCue + 1);
    else if (hostTimelapse)
        snprintf(openRunText, sizeof(openRunText), "TL %d/%d  >", rig.tlFrame, rig.tlFrames);
    else
        snprintf(openRunText, sizeof(openRunText), "RUN  >");
    lv_label_set_text(lblMotionOpenRun, openRunText);
    setKey(btnMotionOpenRun, lblMotionOpenRun, cueWaiting, C_GREEN,
           program.count >= 2 || cueWaiting || hostTimelapse);

    static char timeText[20], dwellText[20], gotoText[24], runText[32];
    if (existing) {
        snprintf(timeText, sizeof(timeText), "TIME %.1fs",
                 program.points[motionSelected].moveMs / 1000.0f);
        if (program.points[motionSelected].holdForGo)
            snprintf(dwellText, sizeof(dwellText), "CUE  GO");
        else
            snprintf(dwellText, sizeof(dwellText), "CUE  %.1fs",
                     program.points[motionSelected].dwellMs / 1000.0f);
    } else {
        snprintf(timeText, sizeof(timeText), "TIME --");
        snprintf(dwellText, sizeof(dwellText), "CUE --");
    }
    lv_label_set_text(lblMotionTime, timeText);
    lv_label_set_text(lblMotionDwell, dwellText);
    const uint32_t held = motionHoldStartedMs
        ? min<uint32_t>(100, (millis() - motionHoldStartedMs) * 100 / HOLD_ACT_MS) : 0;
    snprintf(gotoText, sizeof(gotoText), held && !motionHoldIsRun
             ? "GOTO %lu%%" : "HOLD GOTO", static_cast<unsigned long>(held));
    lv_label_set_text(lblMotionGoto, gotoText);
    setKey(btnMotionGoto, lblMotionGoto, held && !motionHoldIsRun,
           C_AMBER, editing && existing && fresh);
    const auto playAction = currentPlaybackAction();
    if (hostTimelapse) {
        snprintf(runText, sizeof(runText), "HOLD TL STOP");
    } else if (!rig.direct && cueWaiting) {
        snprintf(runText, sizeof(runText), LV_SYMBOL_PLAY " CONTINUE");
    } else if (playAction == osmo::PlaybackAction::Continue) {
        snprintf(runText, sizeof(runText), LV_SYMBOL_PLAY " CONTINUE");
    } else if (playAction == osmo::PlaybackAction::Stop) {
        snprintf(runText, sizeof(runText), LV_SYMBOL_STOP " STOP MOVE");
    } else if (playAction == osmo::PlaybackAction::GoToStart) {
        snprintf(runText, sizeof(runText), LV_SYMBOL_HOME " GO TO START");
    } else if (playAction == osmo::PlaybackAction::Play) {
        snprintf(runText, sizeof(runText), LV_SYMBOL_PLAY " PLAY");
    } else if (!fresh) {
        snprintf(runText, sizeof(runText), LV_SYMBOL_WARNING " CONNECT CAMERA");
    } else if (program.count < 2) {
        snprintf(runText, sizeof(runText), LV_SYMBOL_PLUS " SAVE 2 POINTS");
    } else {
        snprintf(runText, sizeof(runText), LV_SYMBOL_CHARGE " CHARGE CORE2");
    }
    lv_label_set_text(lblMotionRun, runText);
    lv_obj_set_style_text_font(lblMotionRun, FONT_22, 0);
    setKey(btnMotionRun, lblMotionRun,
           cueWaiting || hostTimelapse || status.motionProgramActive ||
               (held && motionHoldIsRun),
           cueWaiting ? C_GREEN : status.motionProgramActive ? C_CYAN : C_AMBER,
           playAction != osmo::PlaybackAction::None || hostTimelapse ||
               (!rig.direct && cueWaiting));
    // Back to one: dim when already there, when nothing is captured, or
    // while a programme owns the head.
    const uint32_t homeHeld = homeHoldStartedMs
        ? min<uint32_t>(100, (millis() - homeHoldStartedMs) * 100 / HOLD_ACT_MS) : 0;
    static char homeText[24];
    if (homeHeld) snprintf(homeText, sizeof(homeText), "P1 %lu%%",
                           static_cast<unsigned long>(homeHeld));
    else if (status.motionAtStart && program.count) snprintf(homeText, sizeof(homeText), "AT P1");
    else snprintf(homeText, sizeof(homeText), "HOLD P1");
    lv_label_set_text(lblMotionHome, homeText);
    setKey(btnMotionHome, lblMotionHome, homeHeld != 0, C_AMBER,
           fresh && program.count >= 1 && editing && !status.motionAtStart);
    lv_label_set_text(lblMotionRepeat, REPEAT_NAMES[status.motionRepeat % 3]);
    setKey(btnMotionRepeat, lblMotionRepeat, status.motionRepeat != 0, C_TEXT,
           editing);

    uint32_t totalMs = 0;
    for (uint8_t i = 1; i < program.count; ++i)
        totalMs += program.points[i].moveMs + program.points[i].dwellMs;
    static char runSummary[72], runSub[80];
    if (program.error[0]) {
        snprintf(runSummary, sizeof(runSummary), "STORE / %.56s",
                 program.error);
        snprintf(runSub, sizeof(runSub), "EDIT OR RELOAD THE PROGRAMME");
    } else if (status.motionProgramFault) {
        snprintf(runSummary, sizeof(runSummary), "FAULT / %.56s",
                 status.detail);
        snprintf(runSub, sizeof(runSub), "STOP / CHECK DEVICE LINK");
    } else if (hostTimelapse) {
        snprintf(runSummary, sizeof(runSummary), "TIMELAPSE %d/%d",
                 rig.tlFrame, rig.tlFrames);
        snprintf(runSub, sizeof(runSub), "HOLD TO STOP BETWEEN FRAMES");
    } else if (cueWaiting) {
        snprintf(runSummary, sizeof(runSummary), "HOLDING FRAME AT P%d",
                 rig.waitingCue + 1);
        const int nextPoint = min<int>(rig.waitingCue + 2, status.motionCount);
        snprintf(runSub, sizeof(runSub), "RELEASES LEG TO P%d", nextPoint);
    } else if (status.motionProgramActive) {
        snprintf(runSummary, sizeof(runSummary), "%s / P%u OF %u",
                 motionPhaseWord(status.motionPhase), status.motionPoint,
                 status.motionCount);
        snprintf(runSub, sizeof(runSub), "%s / TAP TO STOP",
                 REPEAT_NAMES[status.motionRepeat % 3]);
    } else if (status.motionProgramArmed) {
        snprintf(runSummary, sizeof(runSummary), "READY AT START");
        snprintf(runSub, sizeof(runSub), "AT P1 / %.1fs TOTAL / %s",
                 totalMs / 1000.0f, REPEAT_NAMES[status.motionRepeat % 3]);
    } else if (!rig.direct) {
        snprintf(runSummary, sizeof(runSummary), "HOST OWNS MOTION");
        snprintf(runSub, sizeof(runSub), "PROGRAMME RUNS STANDALONE ONLY");
    } else if (batteryBlocked) {
        snprintf(runSummary, sizeof(runSummary), "CHARGE TO PLAY / %d%%",
                 batteryPct);
        snprintf(runSub, sizeof(runSub), "HAND CLUTCH REMAINS AVAILABLE");
    } else {
        snprintf(runSummary, sizeof(runSummary), "%s / %u POINTS / %.1fs",
                 SLOT_NAMES[status.motionSlot % 2], program.count,
                 totalMs / 1000.0f);
        snprintf(runSub, sizeof(runSub), "%s%s",
                 REPEAT_NAMES[status.motionRepeat % 3],
                 program.count >= 2 ? " / CAMERA REC IS SEPARATE" : " / NEED 2+ POINTS");
    }
    lv_label_set_text(lblMotionRunSummary, runSummary);
    lv_label_set_text(lblMotionRunSub, runSub);
    // Keep the consequence visible above/below the large thumb target.
    lv_obj_clear_flag(lblMotionRunSummary, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(lblMotionRunSub, LV_OBJ_FLAG_HIDDEN);
    if (!hostTimelapse && rig.direct) {
        if (held && motionHoldIsRun && !status.motionProgramActive) {
            snprintf(runSub, sizeof(runSub), "HOLD %lu%% / RELEASE TO CANCEL",
                     static_cast<unsigned long>(held));
        } else if (playAction == osmo::PlaybackAction::GoToStart) {
            snprintf(runSub, sizeof(runSub), "HOLD / MOVES TO P1 ONLY");
        } else if (playAction == osmo::PlaybackAction::Play) {
            snprintf(runSub, sizeof(runSub), "HOLD / CAMERA REC IS SEPARATE");
        } else if (playAction == osmo::PlaybackAction::Continue) {
            snprintf(runSub, sizeof(runSub), "TAP / CONTINUE SAVED MOVE");
        }
        lv_label_set_text(lblMotionRunSub, runSub);
    }
    const bool filledRun = cueWaiting || hostTimelapse ||
        status.motionProgramArmed || status.motionProgramActive ||
        (held && motionHoldIsRun);
    const uint32_t summaryColour = filledRun ? C_BG :
        program.error[0] || status.motionProgramFault ? C_MAGENTA :
        batteryBlocked ? C_AMBER : C_DIM;
    lv_obj_set_style_text_color(lblMotionRunSummary,
                                lv_color_hex(summaryColour), 0);
    lv_obj_set_style_text_color(lblMotionRunSub,
                                lv_color_hex(filledRun ? C_BG : C_DIM), 0);
    lv_obj_set_style_border_color(btnMotionRun, lv_color_hex(
        cueWaiting ? C_GREEN : status.motionProgramActive ? C_CYAN :
        (status.motionProgramArmed || batteryBlocked) ? C_AMBER : C_EDGE), 0);
    lv_bar_set_value(motionProgressBar, status.motionProgress, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(motionProgressBar, lv_color_hex(
        cueWaiting ? C_GREEN : status.motionProgramActive ? C_CYAN : C_AMBER),
        LV_PART_INDICATOR);
    static char elapsedText[24], progressText[24], cueText[20];
    const float elapsedSeconds = rig.direct
        ? motionElapsedMs / 1000.0f : rig.elapsed;
    snprintf(elapsedText, sizeof(elapsedText), "ELAPSED %.1fs", elapsedSeconds);
    snprintf(progressText, sizeof(progressText), "LEG %u%%",
             status.motionProgress / 10);
    snprintf(cueText, sizeof(cueText), "CUES %u", motionCueCount);
    lv_label_set_text(lblMotionElapsed, elapsedText);
    lv_label_set_text(lblMotionLegProgress, progressText);
    lv_label_set_text(lblMotionCueCount, cueText);
    lv_obj_set_style_text_color(lblMotionCueCount,
        lv_color_hex(motionCueCount ? C_GREEN : C_DIM), 0);
    lv_label_set_text(lblMotionEdit, SLOT_NAMES[status.motionSlot % 2]);
    setKey(btnMotionEdit, lblMotionEdit, false, C_TEXT, editing);
    if ((status.motionProgramArmed || status.motionProgramActive) &&
        !motionRunView) {
        setMotionUiView(MotionUiView::Run);
    }
}

static void refreshDeviceWorkspace() {
    const osmo::DirectStatus status = osmo::directCamera.status();
    const osmo::CameraSummary *current = currentCamera();
    static char summary[80];
    const char *cameraIdentity = current && current->name[0] ? current->name :
        (!rig.direct && rig.camera ? "CAMERA VIA USB" : "NO CAMERA");
    if (controlEvidenceReady() && rig.telemetryAgeMs != UINT32_MAX)
        snprintf(summary, sizeof(summary), "%s / %lums", cameraIdentity,
                 static_cast<unsigned long>(rig.telemetryAgeMs));
    else
        snprintf(summary, sizeof(summary), "%s", cameraIdentity);
    lv_label_set_text(lblDeviceSummary, summary);

    uint8_t evidenceStep = 0;
    const char *evidenceWord = nullptr;
    uint32_t evidenceColour = C_DIM;
    bool evidenceBlocked = false;
    wifiProgress(evidenceStep, evidenceWord, evidenceColour, evidenceBlocked);
    const bool ready = controlEvidenceReady();
    const char *sessionWord = ready ? "READY" : evidenceBlocked ? "FAILED" :
        status.telemetry ? "TELEM" : rig.camera ? "LINK" :
        evidenceStep >= 4 ? "WI-FI" : evidenceStep >= 2 ? "PAIR" :
        evidenceStep ? "SCAN" : "OFFLINE";
    const uint32_t sessionColour = ready ? C_GREEN :
        evidenceBlocked ? C_MAGENTA : evidenceStep ? C_AMBER : C_DIM;
    lv_label_set_text(lblDeviceEvidenceState[0], sessionWord);
    lv_obj_set_style_text_color(lblDeviceEvidenceState[0],
                                lv_color_hex(sessionColour), 0);
    lv_obj_set_style_border_color(deviceEvidencePanel,
                                  lv_color_hex(sessionColour), 0);

    uint8_t savedCameras = 0;
    for (uint8_t i = 0; i < cameraCatalogUi.count; ++i)
        if (cameraCatalogUi.items[i].trusted) ++savedCameras;
    static char cameraRow[40], toolsRow[40], powerRow[40];
    snprintf(cameraRow, sizeof(cameraRow), "%u SAVED", savedCameras);
    snprintf(toolsRow, sizeof(toolsRow), "UNCONFIRMED");
    snprintf(powerRow, sizeof(powerRow), "%s",
             rig.direct ? "DIRECT" : rig.linked ? "USB" : "OFFLINE");
    lv_label_set_text(lblDeviceCameras, cameraRow);
    lv_label_set_text(lblDeviceTools, toolsRow);
    lv_label_set_text(lblDevicePower, powerRow);
    setConfigTile(btnDeviceCameras, lblDeviceCameras, cameraCatalogUi.actionPending,
           C_AMBER, true);
    setConfigTile(btnDeviceTools, lblDeviceTools, false, C_TEXT, true);
    setConfigTile(btnDevicePower, lblDevicePower, false, C_TEXT, true);
    setConfigTile(btnFocusAfS, lblFocusAfS, false, C_AMBER, ready);
    setConfigTile(btnFocusAfC, lblFocusAfC, false, C_AMBER, ready);
    setConfigTile(btnGimbalCenter, lblGimbalCenter, false, C_AMBER, ready);
    setConfigTile(btnGimbalFollow, lblGimbalFollow, false, C_AMBER, ready);
    static char tool[96];
    if (status.cameraActionPending) {
        snprintf(tool, sizeof(tool), "%s REQUESTING / NOT CONFIRMED",
                 osmo::DirectCamera::actionName(status.cameraAction));
    } else if (status.cameraActionFault) {
        snprintf(tool, sizeof(tool), "%s REQUEST FAILED",
                 osmo::DirectCamera::actionName(status.cameraAction));
    } else if (status.cameraActionUnconfirmed) {
        snprintf(tool, sizeof(tool), "%s SENT / VERIFY ON CAMERA",
                 osmo::DirectCamera::actionName(status.cameraAction));
    } else {
        snprintf(tool, sizeof(tool),
                 "FOCUS COMMAND NOT PROVEN\nA-B PULL REMAINS DISABLED");
    }
    lv_label_set_text(lblToolResult, tool);
    lv_obj_set_style_text_color(lblToolResult, lv_color_hex(
        status.cameraActionFault ? C_MAGENTA :
        (status.cameraActionPending || status.cameraActionUnconfirmed) ? C_AMBER : C_DIM), 0);

    setKey(btnLinkDisconnect, lblLinkDisconnect, false, C_TEXT,
           osmo::directCamera.enabled());
    setKey(btnLinkReconnect, lblLinkReconnect, false, C_AMBER,
           !cameraCatalogUi.forgetPending);
    static char power[24];
    if (powerOffAtMs) snprintf(power, sizeof(power), "STOPPING...");
    else if (powerOffHoldStartedMs) {
        const uint32_t pct = min<uint32_t>(100,
            (millis() - powerOffHoldStartedMs) * 100 / HOLD_POWER_MS);
        snprintf(power, sizeof(power), "POWER OFF %lu%%",
                 static_cast<unsigned long>(pct));
    } else snprintf(power, sizeof(power), "POWER OFF?");
    lv_label_set_text(lblPowerOff, power);
    setKey(btnPowerOff, lblPowerOff, powerOffHoldStartedMs || powerOffAtMs,
           C_AMBER, !powerOffAtMs);
}

static void refreshUi() {
    if (easeOpen) refreshEasePane();
    const bool onMoves = page == static_cast<uint8_t>(Workspace::Moves);
    const bool onDevice = page == static_cast<uint8_t>(Workspace::Device);
    const bool onSettings = page == static_cast<uint8_t>(Workspace::Settings);
    // These are the three largest workspace painters.  They used to redraw
    // their hidden trees ten times a second regardless of which page was on
    // glass, starving LVGL's next input read and display flush.
    if (onDevice || onSettings)
        cameraCatalogUi = osmo::directCamera.catalog();
    if (onDevice) {
        refreshCameraHub();
        refreshCameraConfirm();
        refreshDeviceWorkspace();
    }
    if (onMoves) refreshMotionWorkspace();
    // The title is refreshed, not written once per page change: a running
    // timelapse counter that only moved when the operator changed pages read
    // as a stalled rig.
    updatePageTitle();

    // A fault is global. The only FAULT word on the panel was a child of the
    // HAND dial, so on JOG, MOVES, DEVICE and SETTINGS lost authority showed
    // on the LED bars, buzzed once, and said nothing on screen.
    if (rig.fault[0]) {
        lv_label_set_text(lblFaultTxt, rig.fault);
        lv_obj_clear_flag(faultBar, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_style_text_color(lblPageName, lv_color_hex(C_MAGENTA), 0);
        static char faultReason[80], retryText[28], faultPath[80];
        snprintf(faultReason, sizeof(faultReason), "%.52s / DISARMED",
                 rig.fault);
        lv_label_set_text(lblFaultReason, faultReason);
        const osmo::DirectStatus faultStatus = osmo::directCamera.status();
        if (faultStatus.retryAtMs) {
            const int32_t left = static_cast<int32_t>(
                faultStatus.retryAtMs - millis());
            const uint32_t seconds = left > 0
                ? (static_cast<uint32_t>(left) + 999) / 1000 : 0;
            if (seconds)
                snprintf(retryText, sizeof(retryText), "RETRY %lus",
                         static_cast<unsigned long>(seconds));
            else
                snprintf(retryText, sizeof(retryText), "RETRY READY");
        } else {
            snprintf(retryText, sizeof(retryText), "LINK LOST");
        }
        lv_label_set_text(lblFaultRetry, retryText);
        if (rig.direct) {
            snprintf(faultPath, sizeof(faultPath), "FAILED AT %s\nRADIOS AND SOCKETS RESET FIRST",
                     osmo::DirectCamera::phaseName(faultStatus.blockedAt));
        } else {
            snprintf(faultPath, sizeof(faultPath),
                     "USB OWNER WENT SILENT\nRECONNECT HOST OR OPEN DEVICE");
        }
        lv_label_set_text(lblFaultPath, faultPath);
        setKey(btnFaultDevice, lblFaultDevice, false, C_TEXT, true);
        if (page == static_cast<uint8_t>(Workspace::Device))
            lv_obj_add_flag(faultPane, LV_OBJ_FLAG_HIDDEN);
        else
            lv_obj_clear_flag(faultPane, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(faultBar, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(faultPane, LV_OBJ_FLAG_HIDDEN);
    }

    // STOP: white at rest so it can never be mistaken for the key beside it,
    // magenta while the rig reports a fault -- which is where a STOP that
    // could not be delivered lands -- and a brief amber acknowledgement so a
    // press that changed nothing else is still visibly received.
    const bool stopAcknowledging = stopPressedAtMs &&
        (millis() - stopPressedAtMs) < 600;
    lv_label_set_text(lblStopTxt, stopAcknowledging ? "STOP!" : "STOP");
    lv_obj_set_style_bg_color(btnStop, lv_color_hex(
        rig.fault[0] ? C_MAGENTA : stopAcknowledging ? C_AMBER : C_TEXT), 0);

    const UiPresentation presentation = reducePresentation();
    const char *word = presentation.word;
    const uint32_t colour = presentation.colour;
    const bool clutchIndicated = clutchHeld && rig.linked &&
        !strcmp(rig.owner, "core2");

    const bool handReady = controlEvidenceReady();
    lv_label_set_text(lblState, clutchIndicated ? "GRABBED" :
                      handReady ? "HOLD TO MOVE" : word);

    // HAND changes hierarchy when grabbed: the idle screen is one large
    // invitation, then the same surface becomes sparse measured feedback.
    static char followDetail[64];
    if (rig.fault[0]) {
        snprintf(followDetail, sizeof(followDetail), "%s", rig.fault);
    } else if (!controlEvidenceReady()) {
        snprintf(followDetail, sizeof(followDetail), "%s",
                 rig.detail[0] ? rig.detail : "waiting for camera evidence");
    } else if (rig.waitingCue >= 0) {
        // The dial cannot answer a cue; a grab here aborts the programme.
        // Say both, because "GO?" over a clutch reads as "grab to go".
        snprintf(followDetail, sizeof(followDetail),
                 "GO P%d on MOVES   /   grab = abort", rig.waitingCue + 1);
    } else if (clutchIndicated) {
        snprintf(followDetail, sizeof(followDetail),
                 "RELEASE = HOLD FRAME");
    } else {
        snprintf(followDetail, sizeof(followDetail),
                 "RELATIVE FROM CURRENT FRAME");
    }
    lv_label_set_text(lblSub, followDetail);
    const osmo::DirectStatus handStatus = osmo::directCamera.status();
    static char handDelta[56], handMode[64];
    const bool measuredHand = rig.direct && clutchIndicated;
    if (measuredHand) {
        snprintf(handDelta, sizeof(handDelta), "P %+.1f deg    T %+.1f deg",
                 handStatus.handDeltaPan, handStatus.handDeltaPitch);
    } else {
        snprintf(handDelta, sizeof(handDelta), "P --      T --");
    }
    lv_label_set_text(lblHandDelta, handDelta);
    const uint8_t outputPct = measuredHand ? handStatus.handOutputPct : 0;
    lv_bar_set_value(handCommandBar, outputPct, LV_ANIM_OFF);
    snprintf(handMode, sizeof(handMode), "%s / T %s / P %s",
             TEMPLATE_SHORT_NAMES[templateIdx],
             AXIS_RESPONSE_SHORT_NAMES[tiltResponseIdx],
             AXIS_RESPONSE_SHORT_NAMES[panResponseIdx]);
    lv_label_set_text(lblHandMode, handMode);
    lv_obj_set_style_bg_color(dial, lv_color_hex(C_BG), 0);
    lv_obj_set_style_border_color(dial, lv_color_hex(
        rig.fault[0] ? C_MAGENTA : clutchIndicated ? C_CYAN : C_EDGE), 0);
    lv_obj_set_style_text_color(lblHandDelta,
        lv_color_hex(clutchIndicated ? C_TEXT : C_DIM), 0);
    lv_obj_set_style_text_color(lblSub,
        lv_color_hex(clutchIndicated ? C_CYAN : C_DIM), 0);
    lv_obj_set_style_text_color(lblState, lv_color_hex(
        rig.fault[0] ? C_MAGENTA : clutchIndicated ? C_CYAN :
        handReady ? C_TEXT : colour), 0);
    if (clutchIndicated) {
        lv_obj_set_pos(lblState, 6, 7);
        lv_obj_set_size(lblState, 156, 16);
        lv_obj_set_style_text_font(lblState, FONT_12, 0);
        lv_obj_set_style_text_align(lblState, LV_TEXT_ALIGN_LEFT, 0);
        lv_obj_set_pos(lblSub, 6, 147);
        lv_obj_set_style_text_align(lblSub, LV_TEXT_ALIGN_RIGHT, 0);
        lv_obj_clear_flag(lblHandDelta, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(handCommandBar, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(handReticle, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(handImuDot, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(lblHandMode, LV_OBJ_FLAG_HIDDEN);
        const int imuX = constrain(84 + static_cast<int>(
            handStatus.handDeltaPan * 2.2f), 18, 150);
        const int imuY = constrain(91 - static_cast<int>(
            handStatus.handDeltaPitch * 2.2f), 45, 122);
        lv_obj_set_pos(handImuDot, imuX - 9, imuY - 9);
    } else {
        lv_obj_set_pos(lblState, 6, 58);
        lv_obj_set_size(lblState, 156, 28);
        lv_obj_set_style_text_font(lblState, handReady ? FONT_18 : FONT_14, 0);
        lv_obj_set_style_text_align(lblState, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_pos(lblSub, 6, 91);
        lv_obj_set_style_text_align(lblSub, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_add_flag(lblHandDelta, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(handCommandBar, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(handReticle, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(handImuDot, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(lblHandMode, LV_OBJ_FLAG_HIDDEN);
    }

    setLimitBar(driveLimitUp,    rig.limUp);
    setLimitBar(driveLimitDown,  rig.limDown);
    setLimitBar(driveLimitLeft,  rig.limLeft);
    setLimitBar(driveLimitRight, rig.limRight);

    // Transport and owner, on every page. The two pages where the head is
    // actually steered are the last place to hide who is steering it, and a
    // permanently amber header line dilutes the colour that means "armed".
    // Feel summaries live on the JOG rail and the HAND dial instead.
    static char ctrl[32];
    const char *transport = rig.direct ? "DIRECT" : rig.linked ? "USB" : "OFFLINE";
    const char *owner = !strcmp(rig.owner, "core2") ? "CORE2"
                      : !strcmp(rig.owner, "program") ? "PROGRAM"
                      : !strcmp(rig.owner, "phone") ? "PHONE" : "NONE";
    snprintf(ctrl, sizeof(ctrl), "%s / %s", transport, owner);
    lv_label_set_text(lblCtrl, ctrl);
    lv_obj_set_style_text_color(lblCtrl,
        lv_color_hex(!strcmp(owner, "CORE2") ? C_CYAN : C_DIM), 0);
    const bool recordAvailable = controlEvidenceReady();
    const bool recordHolding = recordHoldStartedMs && !recordHoldTriggered;
    // How long have we been on is the question an operator asks most, and the
    // transition into a rolling state is observable locally: no protocol work,
    // no decoded tally required.
    const bool rollingNow = rig.recording || (rig.direct && rig.recordIntent);
    if (rollingNow && !recordRollingSinceMs) recordRollingSinceMs = millis();
    if (!rollingNow) recordRollingSinceMs = 0;
    const char *recordWord = rig.recording ? "STOP"
        : rig.recordCommandFault ? "REC !"
        : rig.recordPending ? "WAIT"
        : recordHolding ? "HOLD"
        : rig.recordUnconfirmed && rig.recordIntent ? "STOP?"
        : rig.recordUnconfirmed ? "REC?" : "REC";
    const uint32_t recordColour = rig.recording ? C_REC
        : rig.recordCommandFault ? C_MAGENTA : C_AMBER;
    // GOTO, FORGET and POWER OFF all show a hold percentage; the record hold
    // showed a static word for 450 ms and read as an unresponsive button.
    static char recordLabel[20];
    if (recordHolding) {
        const uint32_t pct = min<uint32_t>(100,
            (millis() - recordHoldStartedMs) * 100 / HOLD_RECORD_MS);
        snprintf(recordLabel, sizeof(recordLabel), "HOLD\n%lu%%",
                 static_cast<unsigned long>(pct));
    } else if (recordRollingSinceMs) {
        const uint32_t took = (millis() - recordRollingSinceMs) / 1000;
        if (rig.recording) {
            snprintf(recordLabel, sizeof(recordLabel), "%s\n%lu:%02lu", recordWord,
                     static_cast<unsigned long>(took / 60),
                     static_cast<unsigned long>(took % 60));
        } else {
            // Direct DUML has no decoded tally. This is time since the local
            // request, not a take timer, and the cell says so explicitly.
            snprintf(recordLabel, sizeof(recordLabel), "STOP?\nREQ %lu:%02lu",
                     static_cast<unsigned long>(took / 60),
                     static_cast<unsigned long>(took % 60));
        }
    } else {
        snprintf(recordLabel, sizeof(recordLabel), "%s\n%s", recordWord,
                 recordAvailable ? "HOLD" : "OFF");
    }
    lv_label_set_text(lblRecordTxt, recordLabel);
    setKey(btnRecord, lblRecordTxt,
           rig.recording || rig.recordPending || rig.recordUnconfirmed ||
               rig.recordCommandFault || recordHolding,
           recordColour, recordAvailable);
    if (!rig.recording && !rig.recordPending && !rig.recordUnconfirmed &&
        !rig.recordCommandFault && !recordHolding) {
        lv_obj_set_style_bg_color(btnRecord, lv_color_hex(C_BG), 0);
        lv_obj_set_style_text_color(lblRecordTxt,
            lv_color_hex(recordAvailable ? C_AMBER : C_FAINT), 0);
    }

    // Transient axis locks live only on the shooting surface and JOG sheet;
    // durable inversion lives in SETTINGS > CONTROL > DIRECTION.
    lv_label_set_text(lblJogTiltTxt, rig.lockTilt ? "LOCKED" : "LIVE");
    lv_label_set_text(lblJogPanTxt, rig.lockPan ? "LOCKED" : "LIVE");
    setConfigTile(btnJogTilt, lblJogTiltTxt, rig.lockTilt, C_AMBER, true);
    setConfigTile(btnJogPan,  lblJogPanTxt,  rig.lockPan,  C_AMBER, true);
    const bool tiltOnly = !rig.lockTilt && rig.lockPan;
    const bool panOnly = rig.lockTilt && !rig.lockPan;
    setKey(btnJogTiltOnly, lblJogTiltOnly, tiltOnly, C_CYAN, true);
    setKey(btnJogPanOnly, lblJogPanOnly, panOnly, C_CYAN, true);
    setKey(btnHandTiltOnly, lblHandTiltOnly, tiltOnly, C_CYAN, true);
    setKey(btnHandPanOnly, lblHandPanOnly, panOnly, C_CYAN, true);

    // Recenter is an R&D request in direct mode: say so on the key itself
    // rather than only on DEVICE > TOOLS, where the operator is not.
    const osmo::DirectStatus centerStatus = osmo::directCamera.status();
    const bool centerSent = rig.direct &&
        centerStatus.cameraAction == osmo::CameraAction::GimbalRecenter &&
        (centerStatus.cameraActionPending || centerStatus.cameraActionUnconfirmed);
    const bool centerAvailable = controlEvidenceReady();
    if (!centerAvailable) {
        centerHoldStartedMs = 0;
        centerHoldTriggered = false;
    }
    const uint32_t centerHeld = centerHoldStartedMs
        ? min<uint32_t>(100, (millis() - centerHoldStartedMs) * 100 / HOLD_ACT_MS) : 0;
    static char centerText[16];
    if (centerHeld && !centerHoldTriggered)
        snprintf(centerText, sizeof(centerText), "%lu%%",
                 static_cast<unsigned long>(centerHeld));
    else if (centerSent)
        snprintf(centerText, sizeof(centerText), "SENT?");
    else
        snprintf(centerText, sizeof(centerText), "HOLD");
    lv_label_set_text(lblJogCenter, centerText);
    lv_label_set_text(lblHandCenter, centerText);
    setConfigTile(btnJogCenter, lblJogCenter, centerHeld != 0 || centerSent,
                  C_AMBER, centerAvailable);
    setConfigTile(btnHandCenter, lblHandCenter, centerHeld != 0 || centerSent,
                  C_AMBER, centerAvailable);

    static char jg[48], jogFooter[72];
    if (jogActive) {
        const float amount = osmo::DirectCamera::jogResponse(
            min(1.0f, sqrtf(jogX * jogX + jogY * jogY)));
        jg[0] = '\0';
        snprintf(jogFooter, sizeof(jogFooter), "PAN %+.0f%% / TILT %+.0f%% / %u%%",
                 jogX * 100.0f, jogY * 100.0f,
                 static_cast<unsigned>(amount * 100.0f));
    } else if (jogOriginValid) {
        jg[0] = '\0';
        snprintf(jogFooter, sizeof(jogFooter), "ZERO SET / DRAG / LIFT TO STOP");
    } else if (controlEvidenceReady()) {
        snprintf(jg, sizeof(jg), "TOUCH + DRAG");
        snprintf(jogFooter, sizeof(jogFooter), "LIFT TO STOP");
    } else {
        snprintf(jg, sizeof(jg), "CAMERA NOT READY");
        snprintf(jogFooter, sizeof(jogFooter), "OPEN DEVICE TO CONNECT");
    }
    lv_label_set_text(lblJog, jg);
    lv_label_set_text(lblJogTravel, jogFooter);
    if (jogOriginValid)
        lv_obj_clear_flag(jogDeadzone, LV_OBJ_FLAG_HIDDEN);
    else
        lv_obj_add_flag(jogDeadzone, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(jogHomeDot, LV_OBJ_FLAG_HIDDEN);
    static char jogTilt[24], jogPan[24];
    if (rig.poseValid) {
        snprintf(jogPan, sizeof(jogPan), "P %+.1f deg", rig.yawNow);
        snprintf(jogTilt, sizeof(jogTilt), "T %+.1f deg", rig.pitchNow);
    } else {
        snprintf(jogPan, sizeof(jogPan), "P  --");
        snprintf(jogTilt, sizeof(jogTilt), "T  --");
    }
    lv_label_set_text(lblJogEvidence, jogPan);
    lv_label_set_text(lblJogPanEvidence, jogTilt);
    lv_obj_set_style_text_color(lblJogEvidence,
        lv_color_hex(rig.poseValid ? C_TEXT : C_DIM), 0);
    lv_obj_set_style_text_color(lblJogPanEvidence,
        lv_color_hex(rig.poseValid ? C_TEXT : C_DIM), 0);
    lv_obj_set_style_text_color(lblJogTravel,
        lv_color_hex(jogActive ? C_CYAN : C_DIM), 0);
    lv_obj_set_style_text_color(lblJog,
        lv_color_hex(controlEvidenceReady() ? C_TEXT : C_AMBER), 0);

    lv_label_set_text(lblFollowTemplateTxt, TEMPLATE_NAMES[templateIdx]);
    lv_label_set_text(lblFollowSensitivityTxt, SPEED_NAMES[speedIdx]);
    // Feel changed on the sheet or from the host: keep the JOG rail honest.
    updateSoftKeys();
    lv_label_set_text(lblFollowSmoothTxt, SMOOTH_NAMES[smoothIdx]);
    static char axisTuneSummary[24];
    if (tiltStabilityIdx == 1 && panStabilityIdx == 1) {
        snprintf(axisTuneSummary, sizeof(axisTuneSummary), "T %s / P %s",
                 AXIS_RESPONSE_SHORT_NAMES[tiltResponseIdx],
                 AXIS_RESPONSE_SHORT_NAMES[panResponseIdx]);
    } else {
        snprintf(axisTuneSummary, sizeof(axisTuneSummary), "T/P CUSTOM");
    }
    lv_label_set_text(lblHandAxisTune, axisTuneSummary);
    lv_label_set_text(lblTiltResponse, AXIS_GAIN_LABELS[tiltResponseIdx]);
    lv_label_set_text(lblPanResponse, AXIS_GAIN_LABELS[panResponseIdx]);
    lv_label_set_text(lblTiltStability, AXIS_DAMPING_NAMES[tiltStabilityIdx]);
    lv_label_set_text(lblPanStability, AXIS_DAMPING_NAMES[panStabilityIdx]);
    lv_label_set_text(lblJogSpeedTxt, SPEED_NAMES[jogSpeedIdx]);
    lv_label_set_text(lblJogSmoothTxt, SMOOTH_NAMES[jogSmoothIdx]);
    static char directionShortcut[16];
    if (invertTilt && invertPan)
        snprintf(directionShortcut, sizeof(directionShortcut), "T INV / P INV");
    else if (invertTilt)
        snprintf(directionShortcut, sizeof(directionShortcut), "T INV");
    else if (invertPan)
        snprintf(directionShortcut, sizeof(directionShortcut), "P INV");
    else
        snprintf(directionShortcut, sizeof(directionShortcut), "NORMAL");
    lv_label_set_text(lblHandDirection, directionShortcut);
    lv_label_set_text(lblJogDirection, directionShortcut);
    setConfigTile(btnHandDirection, lblHandDirection,
                  invertTilt || invertPan, C_TEXT, true);
    setConfigTile(btnJogDirection, lblJogDirection,
                  invertTilt || invertPan, C_TEXT, true);
    setConfigTile(btnFollowTemplate, lblFollowTemplateTxt, false, C_TEXT, true);
    setConfigTile(btnFollowSensitivity, lblFollowSensitivityTxt, false, C_TEXT, true);
    setConfigTile(btnFollowSmooth, lblFollowSmoothTxt, false, C_TEXT, true);
    setConfigTile(btnHandAxisTune, lblHandAxisTune,
                  tiltResponseIdx != 1 || panResponseIdx != 1 ||
                  tiltStabilityIdx != 1 || panStabilityIdx != 1,
                  C_TEXT, true);
    setConfigTile(btnTiltResponse, lblTiltResponse,
                  tiltResponseIdx != 1, C_TEXT, true);
    setConfigTile(btnPanResponse, lblPanResponse,
                  panResponseIdx != 1, C_TEXT, true);
    setConfigTile(btnTiltStability, lblTiltStability,
                  tiltStabilityIdx != 1, C_TEXT, true);
    setConfigTile(btnPanStability, lblPanStability,
                  panStabilityIdx != 1, C_TEXT, true);
    setConfigTile(btnJogSpeed, lblJogSpeedTxt, false, C_TEXT, true);
    setConfigTile(btnJogSmooth, lblJogSmoothTxt, false, C_TEXT, true);

    // Settings panes. Cheap enough to paint every frame and it keeps the
    // toggles honest if anything changes them from the host side.
    lv_label_set_text(lblLimitLedsTxt, limitLedsEnabled ? "ON" : "OFF");
    setConfigTile(btnLimitLeds, lblLimitLedsTxt, limitLedsEnabled, C_TEXT, true);
    lv_label_set_text(lblBeaconTxt, beaconEnabled ? "ON" : "OFF");
    setConfigTile(btnBeaconOn, lblBeaconTxt, beaconEnabled, C_TEXT, true);
    lv_label_set_text(lblSilentTxt, soundEnabled ? "ON" : "OFF");
    setConfigTile(btnSilent, lblSilentTxt, soundEnabled, C_TEXT, true);
    lv_label_set_text(lblHapticSettings, HAPTIC_LEVEL_NAMES[hapticLevelIdx]);
    setConfigTile(btnHapticSettings, lblHapticSettings,
                  hapticLevelIdx != HAPTIC_OFF, C_TEXT, true);
    lv_label_set_text(lblHapticLevel, HAPTIC_LEVEL_NAMES[hapticLevelIdx]);
    setConfigTile(btnHapticLevel, lblHapticLevel,
                  hapticLevelIdx != HAPTIC_OFF, C_TEXT, true);
    lv_label_set_text(lblHapticMounting, cameraInHand ? "MOUNTED OFF" : "REMOTE ON");
    setConfigTile(btnHapticMounting, lblHapticMounting, !cameraInHand, C_TEXT, true);
    lv_label_set_text(lblHapticTest,
                      cameraInHand ? "MOUNTED: SILENT" :
                      hapticLevelIdx == HAPTIC_OFF ? "LEVEL IS OFF" : "TAP TO PREVIEW");
    const bool hapticTestAvailable =
        !cameraInHand && hapticLevelIdx != HAPTIC_OFF;
    setConfigTile(btnHapticTest, lblHapticTest, false, C_TEXT,
                  hapticTestAvailable);

    const uint32_t resetHeld = resetHoldStartedMs
        ? min<uint32_t>(100, (millis() - resetHoldStartedMs) * 100 / HOLD_DESTROY_MS) : 0;
    static char resetText[28];
    if (resetHeld) snprintf(resetText, sizeof(resetText), "%lu%%",
                            static_cast<unsigned long>(resetHeld));
    else snprintf(resetText, sizeof(resetText), "HOLD TO RESET");
    lv_label_set_text(lblResetFeelTxt, resetText);
    setConfigTile(btnResetFeel, lblResetFeelTxt, resetHeld != 0, C_AMBER, true);

    uint8_t wifiStep;
    const char *wifiState;
    uint32_t wifiColour;
    bool wifiBlocked;
    wifiProgress(wifiStep, wifiState, wifiColour, wifiBlocked);
    const osmo::CameraSummary *linkTarget = nullptr;
    for (uint8_t i = 0; i < cameraCatalogUi.count; ++i) {
        if (cameraCatalogUi.items[i].selected || cameraCatalogUi.items[i].current) {
            linkTarget = &cameraCatalogUi.items[i];
            break;
        }
    }
    const char *linkName = linkTarget && linkTarget->name[0]
        ? linkTarget->name : "camera";
    static char ap[96];
    if (rig.direct && rig.wifiAttempt && rig.wifiChannel) {
        snprintf(ap, sizeof(ap), "%s / %s  TRY %u/3 CH%u", linkName,
                 rig.ssid[0] ? rig.ssid : "camera AP",
                 rig.wifiAttempt, rig.wifiChannel);
    } else if (rig.direct && rig.wifiAttempt) {
        snprintf(ap, sizeof(ap), "%s / %s  TRY %u/3", linkName,
                 rig.ssid[0] ? rig.ssid : "camera AP", rig.wifiAttempt);
    } else {
        snprintf(ap, sizeof(ap), "%s / AP: %s", linkName,
                 rig.ssid[0] ? rig.ssid : "not reported");
    }
    const char *wifiDetail = safeWifiDetail(rig.detail);
    const bool missing24 = rig.direct &&
        rig.directPhase == osmo::DirectPhase::RetryWait &&
        rig.directBlockedAt == osmo::DirectPhase::WifiJoin &&
        !strcmp(rig.wifiReason, "AP not visible");
    if (missing24) wifiDetail = "Set camera Wi-Fi frequency to 2.4 GHz";

    // Four retained stages replace the reset-prone spinner/flat breadcrumb.
    // The failed stage remains highlighted and every completed stage remains
    // green, so retry never looks as if discovery started from zero.
    const uint8_t linkGroup = wifiStep <= 1 ? 0 : wifiStep <= 3 ? 1 :
                              wifiStep <= 5 ? 2 : 3;
    static char discoverRow[64], pairRow[64], wifiRow[80], sessionRow[64];
    snprintf(discoverRow, sizeof(discoverRow), "1  DISCOVER        %s",
             wifiStep > 1 ? "DONE" :
             (wifiBlocked && linkGroup == 0) ? "FAILED" :
             wifiStep ? "SCANNING" : "CHOOSE CAMERA");
    if (wifiStep > 3) {
        snprintf(pairRow, sizeof(pairRow), "2  PAIR            DONE");
    } else if (linkGroup == 1) {
        snprintf(pairRow, sizeof(pairRow), "2  PAIR            %.34s",
                 wifiBlocked ? wifiState : wifiStep == 3
                     ? "APPROVE ON CAMERA" : "PAIRING");
    } else {
        snprintf(pairRow, sizeof(pairRow), "2  PAIR            NOT STARTED");
    }
    if (wifiStep > 5) {
        snprintf(wifiRow, sizeof(wifiRow), "3  WI-FI           JOINED");
    } else if (linkGroup == 2) {
        const char *wifiNow = missing24 ? "2.4 GHz AP NOT FOUND" :
            wifiBlocked ? wifiDetail : wifiStep == 5 ? "GETTING CAMERA IP" : ap;
        snprintf(wifiRow, sizeof(wifiRow), "3  WI-FI           %.42s", wifiNow);
    } else {
        snprintf(wifiRow, sizeof(wifiRow), "3  WI-FI           NOT STARTED");
    }
    if (wifiStep >= 8) {
        snprintf(sessionRow, sizeof(sessionRow), "4  SESSION         READY");
    } else if (linkGroup == 3) {
        snprintf(sessionRow, sizeof(sessionRow), "4  SESSION         %.32s",
                 wifiBlocked ? wifiState : wifiStep >= 7
                     ? "WAIT TELEMETRY" : "LINKING");
    } else {
        snprintf(sessionRow, sizeof(sessionRow), "4  SESSION         NOT STARTED");
    }
    lv_obj_t *stepLabels[4] = {
        lblWifiSteps, lblWifiState, lblWifiSsid, lblWifiDetail
    };
    const char *stepRows[4] = {
        discoverRow, pairRow, wifiRow, sessionRow
    };
    const bool completed[4] = {
        wifiStep > 1, wifiStep > 3, wifiStep > 5, wifiStep >= 8
    };
    for (uint8_t i = 0; i < 4; ++i) {
        const bool current = i == linkGroup && !completed[i];
        const uint32_t stepColour = completed[i] ? C_GREEN :
            current ? (wifiBlocked ? C_MAGENTA : wifiColour) : C_FAINT;
        lv_label_set_text(stepLabels[i], stepRows[i]);
        lv_obj_set_style_text_color(stepLabels[i], lv_color_hex(stepColour), 0);
        lv_obj_set_style_bg_color(stepLabels[i], lv_color_hex(stepColour), 0);
        lv_obj_set_style_bg_opa(stepLabels[i], current ? LV_OPA_10 : LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(stepLabels[i], current ? 2 : 0, 0);
        lv_obj_set_style_border_side(stepLabels[i], LV_BORDER_SIDE_LEFT, 0);
        lv_obj_set_style_border_color(stepLabels[i], lv_color_hex(stepColour), 0);
        lv_obj_set_style_pad_left(stepLabels[i], current ? 4 : 0, 0);
    }
    const bool cancellable = cameraCatalogUi.actionPending && !wifiBlocked &&
        rig.directPhase != osmo::DirectPhase::Ready;
    static char retryWord[24];
    if (missing24) {
        snprintf(retryWord, sizeof(retryWord), "RETRY AFTER 2.4G");
    } else if (wifiBlocked && rig.retryAtMs) {
        const int32_t left = static_cast<int32_t>(rig.retryAtMs - millis());
        const uint32_t seconds = left > 0
            ? (static_cast<uint32_t>(left) + 999) / 1000 : 0;
        snprintf(retryWord, sizeof(retryWord), "RETRY NOW  %lus",
                 static_cast<unsigned long>(seconds));
    } else {
        snprintf(retryWord, sizeof(retryWord), "%s",
                 wifiBlocked ? "RETRY NOW" : cancellable ? "CANCEL"
                 : rig.directPhase == osmo::DirectPhase::Ready
                     ? "RECONNECT" : "RESTART LINK");
    }
    lv_label_set_text(lblWifiRetry, retryWord);
    setKey(btnWifiRetry, lblWifiRetry, wifiBlocked || cancellable,
           wifiBlocked ? C_MAGENTA : C_AMBER, true);

    static char up[40];
    const uint32_t secs = millis() / 1000;
    snprintf(up, sizeof(up), "%lum %02lus", (unsigned long)(secs / 60),
             (unsigned long)(secs % 60));
    lv_label_set_text(lblSysUptime, up);

    lv_label_set_text(lblInvTTxt, invertTilt ? "INVERTED" : "NORMAL");
    lv_label_set_text(lblInvPTxt, invertPan ? "INVERTED" : "NORMAL");
    setConfigTile(btnInvT, lblInvTTxt, invertTilt, C_TEXT, true);
    setConfigTile(btnInvP, lblInvPTxt, invertPan, C_TEXT, true);
    setConfigTile(btnBright, lblBrightTxt, false, C_TEXT, true);
    setConfigTile(btnScreenTo, lblScreenToTxt, false, C_TEXT, true);

    // Grid subtitles are live summaries, never controls. They make the hub
    // useful at a glance without shrinking the tile titles or adding rows.
    static char tileState[28];
    const bool directReady = rig.direct &&
        rig.directPhase == osmo::DirectPhase::Ready && rig.telemetry;
    const char *cameraState = directReady ? "READY" :
        cameraCatalogUi.scanning ? "SCANNING" :
        cameraCatalogUi.actionPending ? "CONNECTING" :
        rig.fault[0] ? "FAULT" : rig.direct ? word : rig.linked ? "USB LINK" : "OFFLINE";
    lv_label_set_text(lblSettingsCameraState, cameraState);
    lv_obj_set_style_text_color(lblSettingsCameraState, lv_color_hex(
        directReady ? C_GREEN : rig.fault[0] ? C_MAGENTA :
        (cameraCatalogUi.scanning || cameraCatalogUi.actionPending) ? C_AMBER : C_DIM), 0);

    snprintf(tileState, sizeof(tileState), "%s / %s",
             SPEED_NAMES[speedIdx], SPEED_NAMES[jogSpeedIdx]);
    lv_label_set_text(lblSettingsControlsState, tileState);
    snprintf(tileState, sizeof(tileState), "%s / %s",
             TEMPLATE_SHORT_NAMES[templateIdx], SPEED_NAMES[speedIdx]);
    lv_label_set_text(lblSettingsHandState, tileState);
    snprintf(tileState, sizeof(tileState), "%s / %s",
             SPEED_NAMES[jogSpeedIdx], SMOOTH_NAMES[jogSmoothIdx]);
    lv_label_set_text(lblSettingsJogState, tileState);
    snprintf(tileState, sizeof(tileState), "T %s / P %s",
             invertTilt ? "INV" : "NORM", invertPan ? "INV" : "NORM");
    lv_label_set_text(lblSettingsDirectionState, tileState);
    snprintf(tileState, sizeof(tileState), "%s / %s",
             BRIGHT_NAMES[brightIdx], SCREEN_TIMEOUT_NAMES[screenTimeoutIdx]);
    lv_label_set_text(lblSettingsDisplayState, tileState);
    snprintf(tileState, sizeof(tileState), "%s%s / %s",
             beaconEnabled ? "B" : "-", limitLedsEnabled ? "L" : "-",
             soundEnabled ? "SOUND" : "SILENT");
    lv_label_set_text(lblSettingsFeedbackState, tileState);
    lv_label_set_text(lblSettingsPower, "HOLD ON PAGE");

    const int battery = M5.Power.getBatteryLevel();
    const bool charging = static_cast<int>(M5.Power.isCharging()) == 1;
    batteryPct = (battery >= 0 && battery <= 100) ? battery : -1;
    batteryCharging = charging;
    static char batteryText[16];
    static char batteryShort[8];
    if (battery >= 0 && battery <= 100) {
        snprintf(batteryText, sizeof(batteryText), "%d%%%s", battery,
                 charging ? " +" : "");
        snprintf(batteryShort, sizeof(batteryShort), "%d%%%s", battery,
                 charging ? "+" : "");
        const uint32_t batteryColour = charging ? C_GREEN :
            battery <= BATTERY_ARM_FLOOR ? C_AMBER : C_TEXT;
        lv_obj_set_style_text_color(lblBattery, lv_color_hex(batteryColour), 0);
        lv_obj_set_style_text_color(lblBatteryHeader, lv_color_hex(
            charging ? C_GREEN : battery <= BATTERY_ARM_FLOOR
                ? C_AMBER : C_FAINT), 0);
    } else {
        snprintf(batteryText, sizeof(batteryText), "UNKNOWN");
        snprintf(batteryShort, sizeof(batteryShort), "--%%");
        lv_obj_set_style_text_color(lblBattery, lv_color_hex(C_DIM), 0);
        lv_obj_set_style_text_color(lblBatteryHeader, lv_color_hex(C_FAINT), 0);
    }
    lv_label_set_text(lblBattery, batteryText);
    lv_label_set_text(lblBatteryHeader, batteryShort);

    if (batteryPct >= 0)
        snprintf(tileState, sizeof(tileState), "%d%% / FW %.3s", batteryPct,
                 FW_VERSION);
    else
        snprintf(tileState, sizeof(tileState), "FW %.3s", FW_VERSION);
    lv_label_set_text(lblSettingsSystemState, tileState);

    static char fw[40];
    snprintf(fw, sizeof(fw), "%s / %s", FW_VERSION, transport);
    lv_label_set_text(lblFw, fw);
}

// ------------------------------------------------------------------ protocol
static void say(const char *fmt, ...) {
    char line[128];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    // One write keeps lines atomic when the independent IMU task is also
    // reporting. HardwareSerial serialises each buffer at the UART boundary.
    Serial.printf("%s\r\n", line);
}

static void hello() {
    say("H " FW_VERSION " imu,touch,leds,haptic,speaker,lvgl,direct");
    // Board identity and input capability. If touch is not enabled here, no
    // amount of button-mapping work will help.
    say("D board=%d touchEnabled=%d touchPoints=%d disp=%dx%d",
        (int)M5.getBoard(),
        M5.Touch.isEnabled() ? 1 : 0,
        (int)M5.Touch.getCount(),
        M5.Display.width(), M5.Display.height());
}

static const char *findKey(const char *s, const char *key) {
    size_t n = strlen(key);
    for (const char *p = s; (p = strstr(p, key)); p += n)
        if ((p == s || p[-1] == ' ') && p[n] == '=') return p + n + 1;
    return nullptr;
}
static bool keyBool(const char *s, const char *key, bool dflt = false) {
    const char *v = findKey(s, key);
    return v ? (*v == '1') : dflt;
}
static float keyFloat(const char *s, const char *key, float dflt = 0.0f) {
    const char *v = findKey(s, key);
    return v ? atof(v) : dflt;
}
static void keyStr(const char *s, const char *key, char *out, size_t n) {
    const char *v = findKey(s, key);
    if (!v) { out[0] = '\0'; return; }
    size_t i = 0;
    while (v[i] && v[i] != ' ' && i < n - 1) { out[i] = v[i] == '_' ? ' ' : v[i]; i++; }
    out[i] = '\0';
}

static uint8_t responseIndexFromWire(const char *name, uint8_t fallback) {
    if (!strcmp(name, "fine")) return 0;
    if (!strcmp(name, "normal") || !strcmp(name, "balanced")) return 1;
    if (!strcmp(name, "fast") || !strcmp(name, "direct")) return 2;
    return fallback;
}

static uint8_t stabilityIndexFromWire(const char *name, uint8_t fallback) {
    if (!strcmp(name, "quiet")) return 0;
    if (!strcmp(name, "balanced")) return 1;
    if (!strcmp(name, "responsive")) return 2;
    return fallback;
}

static void applyState(const char *s) {
    bool wasFault = rig.fault[0] != '\0';
    rig.linked    = true;
    rig.direct    = false;
    rig.detail[0] = '\0';
    rig.camera    = keyBool(s, "cam");
    rig.armed     = keyBool(s, "armed");
    rig.moving    = keyBool(s, "moving");
    rig.recording = keyBool(s, "rec");
    // The host asserts `rec` only when it can prove the tally, which it
    // cannot yet, so it was always false and the REC key could start but
    // never stop. `recint` is the host's own intent; an older host omits it
    // and the key behaves as before.
    rig.recordIntent = keyBool(s, "recint");
    rig.recordPending = false;
    rig.recordUnconfirmed = rig.recordIntent && !rig.recording;
    rig.recordCommandFault = false;
    rig.telemetry = keyBool(s, "tel");
    // The USB state line carries travel headroom but never absolute pose, so
    // the pose readouts must say unknown rather than repeat a stale direct one.
    rig.poseValid = false;
    rig.telemetryAgeMs = UINT32_MAX;
    rig.waitingCue = (int)keyFloat(s, "cue", -1.0f);
    rig.lockTilt   = keyBool(s, "lockt");
    rig.lockPan    = keyBool(s, "lockp");
    char controlName[16] = {};
    if (findKey(s, "gain")) {
        keyStr(s, "gain", controlName, sizeof(controlName));
        const uint8_t legacy = responseIndexFromWire(controlName, 1);
        tiltResponseIdx = legacy;
        panResponseIdx = legacy;
    }
    if (findKey(s, "trsp")) {
        keyStr(s, "trsp", controlName, sizeof(controlName));
        tiltResponseIdx = responseIndexFromWire(controlName, tiltResponseIdx);
    }
    if (findKey(s, "prsp")) {
        keyStr(s, "prsp", controlName, sizeof(controlName));
        panResponseIdx = responseIndexFromWire(controlName, panResponseIdx);
    }
    if (findKey(s, "tstab")) {
        keyStr(s, "tstab", controlName, sizeof(controlName));
        tiltStabilityIdx = stabilityIndexFromWire(controlName, tiltStabilityIdx);
    }
    if (findKey(s, "pstab")) {
        keyStr(s, "pstab", controlName, sizeof(controlName));
        panStabilityIdx = stabilityIndexFromWire(controlName, panStabilityIdx);
    }
    if (findKey(s, "speed")) {
        keyStr(s, "speed", controlName, sizeof(controlName));
        if      (!strcmp(controlName, "slow"))   speedIdx = 0;
        else if (!strcmp(controlName, "normal")) speedIdx = 1;
        else if (!strcmp(controlName, "fast"))   speedIdx = 2;
    }
    if (findKey(s, "invt")) invertTilt = keyBool(s, "invt");
    if (findKey(s, "invp")) invertPan = keyBool(s, "invp");
    rig.nearLimit  = keyBool(s, "limit");
    // A timelapse runs unattended for hours. The operator is not sitting at a
    // browser, they are somewhere else in the room, so the box has to be able
    // to answer "is it still going, and how far in" from across it.
    rig.tlFrame  = (int)keyFloat(s, "tlf", 0.0f);
    rig.tlFrames = (int)keyFloat(s, "tln", 0.0f);
    rig.elapsed   = keyFloat(s, "t");
    rig.total     = keyFloat(s, "total");
    rig.headUp    = keyFloat(s, "up");
    rig.headDown  = keyFloat(s, "dn");
    keyStr(s, "owner", rig.owner, sizeof(rig.owner));
    keyStr(s, "move",  rig.move,  sizeof(rig.move));
    keyStr(s, "fault", rig.fault, sizeof(rig.fault));
    if (!rig.owner[0]) strcpy(rig.owner, "none");

    const char *v = findKey(s, "verdict");
    if (v) {
        if      (!strncmp(v, "clean", 5)) startFlash(CRGB(0, 255, 90), 1);
        else if (!strncmp(v, "abort", 5) || !strncmp(v, "telemetry", 9))
            startFlash(CRGB(255, 0, 140), 3);
        else startFlash(CRGB(255, 150, 0), 3);
    }
    // Per-direction figures drive the side bars as well as the screen.
    limUpLed    = (uint8_t)keyFloat(s, "ptl", 0.0f);
    limDownLed  = (uint8_t)keyFloat(s, "pth", 0.0f);
    limLeftLed  = (uint8_t)keyFloat(s, "ywl", 0.0f);
    limRightLed = (uint8_t)keyFloat(s, "ywh", 0.0f);
    keyStr(s, "ssid", rig.ssid, sizeof(rig.ssid));
    limitNear    = (uint8_t)keyFloat(s, "near", 0.0f);
    rig.limUp    = (uint8_t)keyFloat(s, "ptl", 0.0f);
    rig.limDown  = (uint8_t)keyFloat(s, "pth", 0.0f);
    rig.limLeft  = (uint8_t)keyFloat(s, "ywl", 0.0f);
    rig.limRight = (uint8_t)keyFloat(s, "ywh", 0.0f);
    if (keyBool(s, "limit") && limitNear >= 99) hapticLimit();
    if (rig.fault[0] && !wasFault) hapticFault();
    if (findKey(s, "sound"))    setSoundEnabled(keyBool(s, "sound"));
    // "rcue", not "cue": that key already exists and carries waitingCue, a
    // waypoint index. Sharing one key between an integer and a name means
    // guessing which is meant from the value, and both readings then rest on
    // a coincidence of formatting.
    char rcue[8] = {0};
    keyStr(s, "rcue", rcue, sizeof(rcue));
    scheduleRunCue(rcue);
    if (findKey(s, "inhand"))   cameraInHand = keyBool(s, "inhand");
    if (findKey(s, "blackout")) blackout     = keyBool(s, "blackout");
}

static void handleLine(char *line) {
    const bool valid = (line[0] == '?' && line[1] == '\0') ||
        ((line[0] == 'S' || line[0] == 'Z' || line[0] == 'D') &&
         line[1] == ' ');
    if (!valid) return;
    // Only an authoritative state line claims USB camera ownership. Diagnostic
    // hello/touch/haptic traffic must not disable the standalone controller.
    const bool hadControlEvidence = controlEvidenceReady();
    const bool authorityChanged = line[0] == 'S' &&
        (rig.direct || osmo::directCamera.enabled());
    if (line[0] == 'S') {
        lastRxMs = millis();
        usbHostSeen = true;
        // Only the first S at a real DIRECT -> USB transition is a hand-off.
        // Recurring USB state packets are evidence updates, not release edges.
        if (authorityChanged) {
            clutchReleaseRequired = true;
            clutchReleaseCandidateMs = 0;
            if (clutchHeld) {
                clutchHeld = false;
                say("E 0 reason=handoff");
            }
            const bool wasJogActive = jogActive;
            if (jogRawContact || wasJogActive) {
                jogReleaseRequired = true;
                jogReleaseCandidateMs = 0;
            }
            jogActive = false;
            jogX = jogY = 0;
            jogOriginValid = false;
            if (jogKnob) lv_obj_add_flag(jogKnob, LV_OBJ_FLAG_HIDDEN);
            if (jogDeadzone) lv_obj_add_flag(jogDeadzone, LV_OBJ_FLAG_HIDDEN);
            if (wasJogActive) say("J 0 0");
            osmo::directCamera.disable();
        }
        rig.direct = false;
        rig.directPhase = osmo::DirectPhase::Off;
    }
    switch (line[0]) {
        case 'S':
            applyState(line + 2);
            if (hadControlEvidence && !controlEvidenceReady()) {
                clutchReleaseRequired = true;
                clutchReleaseCandidateMs = 0;
                if (clutchHeld) {
                    clutchHeld = false;
                    osmo::directCamera.setClutch(false);
                    say("E 0 reason=evidence");
                }
                const bool wasJogActive = jogActive;
                if (jogRawContact || wasJogActive) {
                    jogReleaseRequired = true;
                    jogReleaseCandidateMs = 0;
                }
                jogActive = false;
                jogX = jogY = 0;
                jogOriginValid = false;
                if (jogKnob) lv_obj_add_flag(jogKnob, LV_OBJ_FLAG_HIDDEN);
                if (jogDeadzone) lv_obj_add_flag(jogDeadzone, LV_OBJ_FLAG_HIDDEN);
                if (wasJogActive) say("J 0 0");
            }
            break;
        case 'Z': { int ms = atoi(line + 2); if (ms > 0) tick((uint16_t)min(ms, 300)); break; }
        case '?': hello(); break;
        case 'D': touchDiag = (line[2] == '1'); break;
        default: break;
    }
}

static void pumpSerial() {
    static char buf[192];
    static size_t len = 0;
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (len) { buf[len] = '\0'; handleLine(buf); len = 0; }
        } else if (len < sizeof(buf) - 1) {
            buf[len++] = c;
        } else {
            len = 0;                        // overlong line: drop, resync
        }
    }
}

// -------------------------------------------------------------------- inputs
// Deliberately no gestures. Handheld camera work contains every flick, shake
// and twist accidentally, so a gesture vocabulary would fire during shots.

// Diagnostic: report every raw touch and button transition so a dead input
// path can be told apart from a mis-mapped one.
static void pumpTouchDiag() {
    if (!touchDiag) return;
    static uint32_t lastBeat = 0;
    if (millis() - lastBeat > 2000) {
        lastBeat = millis();
        auto raw = M5.Touch.getDetail();
        say("D beat count=%d pressed=%d x=%d y=%d imu_n=%lu drop=%lu fail=%lu stack=%lu",
            (int)M5.Touch.getCount(), raw.isPressed() ? 1 : 0, raw.x, raw.y,
            static_cast<unsigned long>(imuSampleCount),
            static_cast<unsigned long>(imuSerialDrops),
            static_cast<unsigned long>(imuReadFailures),
            static_cast<unsigned long>(imuStackWords));
    }
    static bool wasDown = false;
    auto t = M5.Touch.getDetail();
    bool down = t.isPressed();
    if (down != wasDown) {
        wasDown = down;
        say("D touch %d x=%d y=%d btnA=%d btnB=%d btnC=%d",
            down ? 1 : 0, t.x, t.y,
            M5.BtnA.isPressed() ? 1 : 0,
            M5.BtnB.isPressed() ? 1 : 0,
            M5.BtnC.isPressed() ? 1 : 0);
    }
}

static void pumpButtons() {
    // Bezel A / B / C map to the three legends under them. Navigation on
    // hardware, because the on-screen tabs were too small to hit reliably
    // while holding the device.
    const uint32_t now = millis();
    const bool anyHardwareDown = M5.BtnA.isPressed() || M5.BtnB.isPressed() ||
                                 M5.BtnC.isPressed();
    if (swallowHardwareWake) {
        // Consume the entire press, not just its first loop. `wasPressed()` is
        // already gone by the time all three keys are up, so the wake gesture
        // cannot leak through as navigation or clutch acquisition.
        if (anyHardwareDown) return;
        swallowHardwareWake = false;
    }

    const bool touchDown = M5.Touch.getDetail().isPressed();
    if (jogReleaseRequired) {
        if (touchDown) {
            jogReleaseCandidateMs = 0;
        } else {
            if (!jogReleaseCandidateMs) jogReleaseCandidateMs = now;
            if (now - jogReleaseCandidateMs >= CLUTCH_RELEASE_DEBOUNCE_MS) {
                jogReleaseRequired = false;
                jogReleaseCandidateMs = 0;
                jogRawContact = false;
                osmo::directCamera.setJog(0.0f, 0.0f, false);
            }
        }
    }

    // The centre bezel key is the reliable eyes-off HOME key. On the two live
    // control pages it toggles HAND/JOG; elsewhere it always returns to HAND.
    // Give it priority if two capacitive zones happen to report together.
    if (M5.BtnB.wasPressed()) {
        homeOrToggleControl();
    } else {
        if (M5.BtnA.wasPressed()) navigateLeft();
        if (M5.BtnC.wasPressed()) {
            showPage((page + 1) % PAGE_COUNT);
            tick(15);
        }
    }

    const bool rawContact = (page == static_cast<uint8_t>(Workspace::Hand) &&
        !handFeelOpen && !easeOpen && screenClutchHeld);
    const osmo::DirectStatus motion = osmo::directCamera.status();
    if (rawContact && (motion.motionProgramArmed ||
                       motion.motionProgramActive)) {
        osmo::directCamera.abort();
        clutchReleaseRequired = true;
        clutchReleaseCandidateMs = 0;
    }
    const bool readyForMotion = controlEvidenceReady();
    // If the operator is already touching the clutch while evidence becomes
    // ready, require a release instead of acquiring the head under a held
    // finger. Offline presses provide no positive grab feedback.
    if (!readyForMotion && rawContact) {
        clutchReleaseRequired = true;
        clutchReleaseCandidateMs = 0;
    }
    if (clutchReleaseRequired) {
        const bool physicalContact = touchDown;
        if (physicalContact) {
            clutchReleaseCandidateMs = 0;
        } else {
            if (!clutchReleaseCandidateMs) clutchReleaseCandidateMs = now;
            if (now - clutchReleaseCandidateMs >= CLUTCH_RELEASE_DEBOUNCE_MS) {
                clutchReleaseRequired = false;
                clutchReleaseCandidateMs = 0;
                screenClutchHeld = false;
            }
        }
    }
    const bool contact = rawContact && readyForMotion && !clutchReleaseRequired;
    if (contact) clutchLastContactMs = now;
    const bool held = contact ||
        (clutchHeld && (now - clutchLastContactMs) < CLUTCH_RELEASE_DEBOUNCE_MS);

    if (held != clutchHeld) {
        clutchHeld = held;
        osmo::directCamera.setClutch(held);
        say("E %d touch=%d since=%lu",
            held ? 1 : 0, screenClutchHeld ? 1 : 0,
            (unsigned long)(now - clutchLastContactMs));
        updateSoftKeys();
    }

    // Haptic and cyan state mean acquired authority, not merely a local press.
    // USB must echo owner=core2; direct mode publishes the same ownership.
    const bool confirmed = clutchHeld && controlEvidenceReady() &&
                           !strcmp(rig.owner, "core2");
    if (confirmed && !clutchFeedbackConfirmed) hapticGrab();
    clutchFeedbackConfirmed = confirmed;

}

static bool configureImuControlBandwidth() {
    // M5Unified configures the MPU6886 for 176 Hz gyro / 218 Hz accel
    // bandwidth, then this application samples at 100 Hz.  DLPF_CFG=3 and
    // A_DLPF_CFG=3 give 41 Hz / 44.8 Hz anti-alias filtering while preserving
    // the library's ranges and scale conversion. Other IMU variants keep
    // their library-owned settings until their register maps are qualified.
    if (M5.Imu.getType() != m5::imu_mpu6886) {
        say("D imu_lpf=library-default type=%d", (int)M5.Imu.getType());
        return false;
    }
    constexpr uint8_t address = 0x68;
    constexpr uint8_t gyroFilterRegister = 0x1A;
    constexpr uint8_t accelFilterRegister = 0x1D;
    constexpr uint32_t i2cHz = 400000;
    uint8_t gyroConfig = 0;
    uint8_t accelConfig = 0;
    const bool readCurrent = M5.In_I2C.readRegister(
        address, gyroFilterRegister, &gyroConfig, 1, i2cHz) &&
        M5.In_I2C.readRegister(
            address, accelFilterRegister, &accelConfig, 1, i2cHz);
    if (!readCurrent) {
        say("D imu_lpf=read-failed type=%d", (int)M5.Imu.getType());
        return false;
    }
    const uint8_t wantedGyro = (gyroConfig & 0xF8u) | 0x03u;
    const uint8_t wantedAccel = (accelConfig & 0xF8u) | 0x03u;
    const bool wrote = M5.In_I2C.writeRegister8(
        address, gyroFilterRegister, wantedGyro, i2cHz) &&
        M5.In_I2C.writeRegister8(
            address, accelFilterRegister, wantedAccel, i2cHz);
    uint8_t verifiedGyro = 0;
    uint8_t verifiedAccel = 0;
    const bool verified = wrote && M5.In_I2C.readRegister(
        address, gyroFilterRegister, &verifiedGyro, 1, i2cHz) &&
        M5.In_I2C.readRegister(
            address, accelFilterRegister, &verifiedAccel, 1, i2cHz) &&
        (verifiedGyro & 0x07u) == 0x03u &&
        (verifiedAccel & 0x07u) == 0x03u;
    say("D imu_lpf=%s gyro=0x%02X accel=0x%02X",
        verified ? "41/44.8Hz" : "verify-failed",
        verifiedGyro, verifiedAccel);
    return verified;
}

static void sampleImu() {
    // One hardware update owns both halves of the 6-DoF sample. Calling
    // getAccel()/getGyro() separately can stamp a partial/stale pair as fresh
    // because each accessor conditionally updates the sensor on its own.
    const int mask = static_cast<int>(M5.Imu.update());
    const int required = m5::IMU_Class::sensor_mask_accel |
                         m5::IMU_Class::sensor_mask_gyro;
    if ((mask & required) != required) {
        ++imuReadFailures;
        return;
    }
    m5::IMU_Class::imu_data_t data;
    M5.Imu.getImuData(&data);
    const float ax = data.accel.x, ay = data.accel.y, az = data.accel.z;
    const float gx = data.gyro.x, gy = data.gyro.y, gz = data.gyro.z;
    // Timestamp only a coherent successful acquisition. Invalid reads then
    // age naturally into the controller's existing 250 ms fail-closed gate.
    const uint32_t sampleAt = millis();
    // The direct controller owns the full quaternion estimator and all-axis
    // bias learning. Raw samples keep inversion and hand templates out of the
    // estimator state and preserve the actual scheduler timestamp.
    osmo::directCamera.setImu(ax, ay, az, gx, gy, gz, sampleAt);
    ++imuSampleCount;
    // The host reads this line as pitch, roll, yaw rate. Keep reporting on the
    // same 100 Hz boundary, but never let a full UART stall estimator input.
    const osmo::HostImu att = osmo::directCamera.hostImu();
    char line[64];
    const int len = snprintf(line, sizeof(line),
        "I %.2f %.2f %.2f %.2f\r\n", att.pitch, att.roll, gz, 0.0f);
    if (len > 0 && len < static_cast<int>(sizeof(line)) &&
        Serial.availableForWrite() >= len) {
        Serial.write(reinterpret_cast<const uint8_t *>(line), len);
    } else {
        ++imuSerialDrops;
    }
}

static void pumpImu(uint32_t now) {
    // Allocation failure fallback. The normal path is imuTaskEntry below and
    // is independent of UI redraw, touch, haptic and networking cadence.
    static uint32_t last = 0;
    if (now - last < 10) return;
    last = now;
    sampleImu();
}

static void imuTaskEntry(void *) {
    const TickType_t period = pdMS_TO_TICKS(10);
    TickType_t previousWake = xTaskGetTickCount();
    for (;;) {
        sampleImu();
        if ((imuSampleCount % 100) == 0)
            imuStackWords = uxTaskGetStackHighWaterMark(nullptr);
        const TickType_t now = xTaskGetTickCount();
        // vTaskDelayUntil repays missed deadlines with immediate iterations.
        // Rebase after a full-period overrun so an I2C timeout cannot create a
        // high-priority catch-up burst that starves touch, LVGL or STOP.
        if (now - previousWake >= period) previousWake = now;
        vTaskDelayUntil(&previousWake, period);
    }
}

static bool startImuTask() {
    if (imuTaskHandle) return true;
    TaskHandle_t created = nullptr;
    const BaseType_t result = xTaskCreatePinnedToCore(
        imuTaskEntry, "core2-imu", 4096, nullptr, 2, &created, 1);
    if (result != pdPASS || !created) {
        say("D imu task allocation failed / loop fallback");
        return false;
    }
    imuTaskHandle = created;
    say("D imu sampler=100Hz core=1 priority=2");
    return true;
}

// Mirror direct-task evidence into the existing single UI state. Wi-Fi
// association alone is never called a camera link: ready requires the UDP
// session, and telemetry requires a fresh 0x04/0x05 attitude push.
static void pumpDirectState() {
    if (!osmo::directCamera.enabled()) {
        // Catch every future disable path, not only the two UI controls above.
        // A USB-owned state has rig.direct == false and is intentionally left
        // alone; only stale standalone evidence is invalidated here.
        if (rig.direct) clearDirectRigEvidence();
        return;
    }
    const bool hadControlEvidence = controlEvidenceReady();
    const osmo::DirectStatus st = osmo::directCamera.status();
    rig.direct = true;
    rig.directPhase = st.phase;
    rig.directBlockedAt = st.blockedAt;
    rig.linked = st.ready;
    rig.camera = st.ready;
    rig.telemetry = st.telemetry;
    rig.telemetryAgeMs = st.telemetryAgeMs;
    rig.pitchNow = st.pitch;
    rig.yawNow = st.yaw;
    rig.poseValid = st.telemetry;
    rig.moving = st.moving;
    rig.armed = st.motionProgramArmed;
    rig.recording = false;
    rig.recordIntent = st.recordIntent;
    rig.recordPending = st.recordPending;
    rig.recordUnconfirmed = st.recordUnconfirmed;
    rig.recordCommandFault = st.recordCommandFault;
    // A standalone cue is the same fact as a host cue: the head is holding
    // for a human. One field drives the GO? word, the amber breathe on the
    // bars, the wake gate and the run key.
    rig.waitingCue = st.cuePoint ? st.cuePoint - 1 : -1;
    // Direct mode has no host program/timelapse presentation. Leaving those
    // fields populated lets stale USB text conceal pairing or retry guidance.
    if (st.motionProgramArmed || st.motionProgramActive) {
        snprintf(rig.move, sizeof(rig.move), "P%u/%u",
                 st.motionPoint, st.motionCount);
    } else {
        rig.move[0] = '\0';
    }
    rig.elapsed = rig.total = 0.0f;
    rig.tlFrame = rig.tlFrames = 0;
    rig.headUp = st.pitchHeadLow;
    rig.headDown = st.pitchHeadHigh;
    rig.limUp = limUpLed = st.pitchWarnLow;
    rig.limDown = limDownLed = st.pitchWarnHigh;
    // The Pocket 4 yaw envelope is still contradictory in the captures. Do
    // not invent direct-mode pan warnings until it is re-characterised.
    rig.limLeft = limLeftLed = 0;
    rig.limRight = limRightLed = 0;
    limitNear = max(st.pitchWarnLow, st.pitchWarnHigh);
    rig.nearLimit = limitNear >= 99;
    strncpy(rig.ssid, st.ssid, sizeof(rig.ssid) - 1);
    rig.ssid[sizeof(rig.ssid) - 1] = '\0';
    strncpy(rig.detail, st.detail, sizeof(rig.detail) - 1);
    rig.detail[sizeof(rig.detail) - 1] = '\0';
    rig.wifiAttempt = st.wifiAttempt;
    rig.wifiStatus = st.wifiStatus;
    rig.wifiChannel = st.wifiChannel;
    memcpy(rig.wifiBssid, st.wifiBssid, sizeof(rig.wifiBssid));
    rig.retryAtMs = st.retryAtMs;
    strncpy(rig.wifiReason, st.wifiReason, sizeof(rig.wifiReason) - 1);
    rig.wifiReason[sizeof(rig.wifiReason) - 1] = '\0';

    // One change-driven line makes the exact standalone connection frontier
    // observable during bench verification without ever logging the WPA key.
    static uint8_t lastPhase = 0xFF;
    static uint8_t lastBlockedAt = 0xFF;
    static uint8_t lastAttempt = 0xFF;
    static uint8_t lastStatus = 0xFF;
    static uint8_t lastChannel = 0xFF;
    static uint8_t lastBssid[6] = {};
    static char lastReason[24] = "";
    static char lastDetail[48] = "";
    static bool lastTelemetry = false;
    static bool haveLastTelemetry = false;
    if (lastPhase != static_cast<uint8_t>(st.phase) ||
        lastBlockedAt != static_cast<uint8_t>(st.blockedAt) ||
        lastAttempt != st.wifiAttempt || lastStatus != st.wifiStatus ||
        lastChannel != st.wifiChannel ||
        memcmp(lastBssid, st.wifiBssid, sizeof(lastBssid)) ||
        strcmp(lastReason, st.wifiReason) ||
        strcmp(lastDetail, st.detail) || !haveLastTelemetry ||
        lastTelemetry != st.telemetry) {
        say("D direct=%s block=%s try=%u/3 wl=%u ch=%u telem=%u",
            osmo::DirectCamera::phaseName(st.phase),
            osmo::DirectCamera::phaseName(st.blockedAt), st.wifiAttempt,
            st.wifiStatus, st.wifiChannel, st.telemetry ? 1 : 0);
        say("D wifi bssid=%02X:%02X:%02X:%02X:%02X:%02X why=%s",
            st.wifiBssid[0], st.wifiBssid[1], st.wifiBssid[2],
            st.wifiBssid[3], st.wifiBssid[4], st.wifiBssid[5],
            st.wifiReason[0] ? st.wifiReason : "-");
        say("D detail=%s", safeWifiDetail(st.detail));
        say("D rx peer=%lu foreign=%lu hdrbad=%lu win=%lu p3=%lu vid=%lu",
            static_cast<unsigned long>(st.rxPeerPackets),
            static_cast<unsigned long>(st.rxForeignPackets),
            static_cast<unsigned long>(st.rxHeaderRejects),
            static_cast<unsigned long>(st.rxWindowPackets),
            static_cast<unsigned long>(st.rxAckedDataPackets),
            static_cast<unsigned long>(st.rxVideoPackets));
        say("D frame valid=%lu att=%lu ok=%lu last=%u/%02X op=%02X/%02X plen=%u",
            static_cast<unsigned long>(st.rxFrames),
            static_cast<unsigned long>(st.rxAttitudeFrames),
            static_cast<unsigned long>(st.rxAcceptedAttitudes),
            st.lastRxLen, st.lastRxType, st.lastFrameCmdSet,
            st.lastFrameCmdId, st.lastFramePayloadLen);
        say("D att rx=%u/%02X seq=%04X", st.lastAttitudeRxLen,
            st.lastAttitudeType, st.lastAttitudeSeq);
        say("D ack v=%04X d=%04X x=%04X", st.ackVideoCursor,
            st.ackDataCursor, st.ackExtraCursor);
        lastPhase = static_cast<uint8_t>(st.phase);
        lastBlockedAt = static_cast<uint8_t>(st.blockedAt);
        lastAttempt = st.wifiAttempt;
        lastStatus = st.wifiStatus;
        lastChannel = st.wifiChannel;
        memcpy(lastBssid, st.wifiBssid, sizeof(lastBssid));
        strncpy(lastReason, st.wifiReason, sizeof(lastReason) - 1);
        lastReason[sizeof(lastReason) - 1] = '\0';
        strncpy(lastDetail, st.detail, sizeof(lastDetail) - 1);
        lastDetail[sizeof(lastDetail) - 1] = '\0';
        lastTelemetry = st.telemetry;
        haveLastTelemetry = true;
    }
    strcpy(rig.owner, (st.motionProgramArmed || st.motionProgramActive)
        ? "program" : (st.clutch || st.moving) ? "core2" : "none");
    const bool wasFault = rig.fault[0] != '\0';
    if (st.controlFault) {
        strncpy(rig.fault, st.detail, sizeof(rig.fault) - 1);
        rig.fault[sizeof(rig.fault) - 1] = '\0';
    } else {
        rig.fault[0] = '\0';
    }
    if (rig.fault[0] && !wasFault) hapticFault();
    // The host cues ARM / GO / END on edges (server.py _cue_edges). A
    // standalone run gave the crew nothing: same edges, same tones.
    static bool cuedArmed = false, cuedActive = false;
    if (st.motionProgramArmed && !cuedArmed) scheduleRunCue("arm");
    if (st.motionProgramActive && !cuedActive) scheduleRunCue("go");
    else if (cuedActive && !st.motionProgramActive) scheduleRunCue("end");
    cuedArmed = st.motionProgramArmed;
    cuedActive = st.motionProgramActive;
    if (hadControlEvidence && !controlEvidenceReady()) {
        clutchReleaseRequired = true;
        clutchReleaseCandidateMs = 0;
        if (clutchHeld) {
            clutchHeld = false;
            osmo::directCamera.setClutch(false);
            say("E 0 reason=evidence");
        }
        if (jogRawContact || jogActive) {
            jogReleaseRequired = true;
            jogReleaseCandidateMs = 0;
        }
        jogActive = false;
        jogX = jogY = 0;
        jogOriginValid = false;
        if (jogKnob) lv_obj_add_flag(jogKnob, LV_OBJ_FLAG_HIDDEN);
        if (jogDeadzone) lv_obj_add_flag(jogDeadzone, LV_OBJ_FLAG_HIDDEN);
    }
}

// ---------------------------------------------------------------------- main
void setup() {
    auto cfg = M5.config();
    cfg.internal_imu = true;
    cfg.internal_spk = true;
    M5.begin(cfg);
    M5.Speaker.setVolume(0);                // silent until explicitly enabled
    M5.Display.setRotation(1);
    M5.Display.fillScreen(TFT_BLACK);

    loadFeelPreferences();
    loadUiPreferences();

    Serial.begin(115200);

    FastLED.addLeds<SK6812, LED_PIN, GRB>(leds, LED_COUNT);
    FastLED.setBrightness(255);             // scaled per-frame in renderBeacon
    fill_solid(leds, LED_COUNT, CRGB::Black);
    FastLED.show();

    buf1 = static_cast<lv_color_t *>(heap_caps_malloc(
        DRAW_PIXELS * sizeof(lv_color_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    buf2 = static_cast<lv_color_t *>(heap_caps_malloc(
        DRAW_PIXELS * sizeof(lv_color_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!buf1 || !buf2) {
        M5.Display.setTextColor(TFT_MAGENTA, TFT_BLACK);
        M5.Display.setTextSize(2);
        M5.Display.drawString("PSRAM ERROR", 84, 104);
        while (true) delay(1000);  // no UI/control is safer than partial boot
    }

    lv_init();
    lv_disp_draw_buf_init(&drawBuf, buf1, buf2, DRAW_PIXELS);

    static lv_disp_drv_t dispDrv;
    lv_disp_drv_init(&dispDrv);
    dispDrv.hor_res  = 320;
    dispDrv.ver_res  = 240;
    dispDrv.flush_cb = flushCb;
    dispDrv.draw_buf = &drawBuf;
    lv_disp_drv_register(&dispDrv);

    static lv_indev_drv_t indevDrv;
    lv_indev_drv_init(&indevDrv);
    indevDrv.type    = LV_INDEV_TYPE_POINTER;
    indevDrv.read_cb = touchCb;
    // A thumb can wander ten pixels while intending to tap.  Require a real
    // drag before a menu scrolls and stop it on lift; long phone-like coast is
    // actively harmful when the entire viewport is only 168 px high.
    indevDrv.scroll_limit = 18;
    indevDrv.scroll_throw = 100;
    lv_indev_t *touchInput = lv_indev_drv_register(&indevDrv);
    if (touchInput && indevDrv.read_timer)
        lv_timer_set_period(indevDrv.read_timer, 15);

    buildUi();
    buildEasePane();
    // Nothing applied the stored DAY/INTERIOR/NIGHT choice at boot: the panel
    // came up at whatever brightness M5Unified left it at, and only the first
    // press of the toggle made the setting real.
    applyBrightness();
    refreshUi();
    hello();
    configureImuControlBandwidth();
    // Start after M5/Serial/IMU initialisation and before the host claim
    // window. The fallback remains in loop() if allocation is unavailable.
    startImuTask();
    // screenOffAt is zero until activity is recorded. Without this the first
    // loop immediately interprets the timeout as expired and blanks the LCD.
    noteScreenActivity(millis());
    // Give an attached host a short, explicit claim window. The continuous IMU
    // estimator no longer blocks boot for a one-pose calibration.
    hostClaimDeadlineMs = millis() + 750;
}

// Configuration is unchanged for most frames. Avoid two cross-core spinlock
// acquisitions per UI iteration; input/freshness/STOP remain unthrottled.
static void syncControlConfig() {
    if (easeDirty) {
        uint16_t milliseconds[2][4] = {};
        for (uint8_t m=0; m<2; ++m)
            for (uint8_t axis=0; axis<2; ++axis) {
                const uint8_t base = (easeValues[m][6] & (1 << axis)) ? 2 + axis*2 : 0;
                milliseconds[m][axis*2] = EASE_MS[easeValues[m][base]];
                milliseconds[m][axis*2+1] = EASE_MS[easeValues[m][base+1]];
            }
        osmo::directCamera.setManualEase(milliseconds);
        easeDirty = false;
    }
    const uint8_t current[] = {tiltResponseIdx, panResponseIdx,
        tiltStabilityIdx, panStabilityIdx, speedIdx, smoothIdx, templateIdx,
        static_cast<uint8_t>(rig.lockTilt), static_cast<uint8_t>(rig.lockPan),
        static_cast<uint8_t>(invertTilt), static_cast<uint8_t>(invertPan),
        jogSpeedIdx, jogSmoothIdx};
    static uint8_t previous[sizeof(current)] = {};
    static bool sent = false;
    if (sent && std::memcmp(current, previous, sizeof(current)) == 0) return;
    osmo::directCamera.setControlConfig(
        tiltResponseIdx, panResponseIdx,
        tiltStabilityIdx, panStabilityIdx,
        speedIdx, smoothIdx, templateIdx,
        rig.lockTilt, rig.lockPan, invertTilt, invertPan);
    osmo::directCamera.setJogConfig(jogSpeedIdx, jogSmoothIdx);
    std::memcpy(previous, current, sizeof(current));
    sent = true;
}

void loop() {
    const uint32_t now = millis();
    M5.update();

    pumpSerial();
    // Give an attached host one short boot window to claim the serial path.
    // With no valid host command, Core2 starts its own BLE -> Wi-Fi -> UDP
    // camera session on a separate FreeRTOS task; LVGL stays responsive.
    static bool transportSelected = false;
    if (!transportSelected && (usbHostSeen ||
        (int32_t)(now - hostClaimDeadlineMs) >= 0)) {
        transportSelected = true;
        if (!usbHostSeen) osmo::directCamera.begin();
    }
    pumpDirectState();
    syncControlConfig();
    if (powerOffAtMs && static_cast<int32_t>(now - powerOffAtMs) >= 0) {
        powerOffAtMs = 0;
        fill_solid(leds, LED_COUNT, CRGB::Black);
        FastLED.show();
        M5.Display.sleep();
        M5.Display.waitDisplay();
        // Core2 is a fixed AXP192 board. Calling the generic
        // M5.Power.powerOff() pulls the ESP deep-sleep fallback into IRAM and
        // overflows this BLE + Wi-Fi image; the PMIC command is the exact
        // hardware shutdown operation the generic helper selects for Core2.
        M5.Power.Axp192.powerOff();
    }
    pumpRunCue(now);
    pumpScreenTimeout(now);
    // A hardware button is activity as much as a touch is; without this the
    // screen sleeps in the middle of someone driving the rig from the keys.
    if (M5.BtnA.isPressed() || M5.BtnB.isPressed() || M5.BtnC.isPressed()) {
        noteScreenActivity(now);
    }
    pumpTouchDiag();
    pumpButtons();
    pumpLimitHaptic(now);
    pumpHaptic(now);
    if (!imuTaskHandle) pumpImu(now);

    // Losing the PC is a fault, and it latches. Silence from the driver means
    // we do not know what the head is doing, which is exactly when the box
    // must stop implying everything is fine.
    //
    // Re-read the clock rather than reusing `now` from the top of the loop:
    // pumpSerial() above may have stamped lastRxMs at a LATER millis(), and
    // `now - lastRxMs` is unsigned, so it wrapped to ~4e9 and tripped the
    // timeout on every single inbound message -- which dropped the clutch
    // 2.5 times a second while a finger was still on the button.
    const uint32_t sinceRx = (millis() >= lastRxMs) ? (millis() - lastRxMs) : 0;
    if (rig.linked && !rig.direct && sinceRx > LINK_TIMEOUT_MS) {
        rig.linked = false;
        rig.armed = rig.moving = rig.recording = false;
        rig.waitingCue = -1;
        strcpy(rig.owner, "none");
        strcpy(rig.fault, "USB link lost");
        clutchReleaseRequired = true;
        clutchReleaseCandidateMs = 0;
        if (jogRawContact || jogActive) {
            jogReleaseRequired = true;
            jogReleaseCandidateMs = 0;
        }
        jogActive = false;
        jogX = jogY = 0;
        jogOriginValid = false;
        if (jogKnob) lv_obj_add_flag(jogKnob, LV_OBJ_FLAG_HIDDEN);
        if (jogDeadzone) lv_obj_add_flag(jogDeadzone, LV_OBJ_FLAG_HIDDEN);
        if (clutchHeld) { clutchHeld = false; say("E 0 reason=linklost"); }
        hapticFault();
    }

    const bool clutchIndicated = clutchHeld && rig.linked &&
        !strcmp(rig.owner, "core2");
    if (rig.fault[0])            beacon = BEACON_FAULT;
    else if (rig.recording)      beacon = BEACON_REC;
    else if (clutchIndicated)    beacon = BEACON_CLUTCH;
    else if (rig.moving)         beacon = BEACON_MOVING;
    else if (rig.waitingCue >= 0)beacon = BEACON_WAIT_GO;
    else if (rig.armed)          beacon = BEACON_ARMED;
    else                         beacon = BEACON_OFF;

    static uint32_t lastLed = 0;
    if (now - lastLed >= 33) {
        lastLed = now;
        if (!renderFlash(now)) renderBeacon(now);
    }

    // Service touch before any optional repaint.  Settings are mostly static;
    // an event asks for an immediate refresh, while the slow heartbeat keeps
    // battery/link summaries honest without continuously repainting a menu.
    lv_timer_handler();
    static uint32_t lastUi = 0;
    const uint32_t uiNow = millis();
    const uint32_t uiInterval =
        page == static_cast<uint8_t>(Workspace::Settings) ? 500 : 100;
    if (uiRefreshRequested || uiNow - lastUi >= uiInterval) {
        lastUi = uiNow;
        uiRefreshRequested = false;
        refreshUi();
    }
    delay(2);
}
