"""Canvas-adoption contracts for the 320x240 Core2 panel.

These checks pin the safety-preserving decisions made while porting the
Claude Design canvas.  They deliberately check semantic geometry and evidence
ownership rather than trying to reproduce an LVGL screenshot in Python.
"""

import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MAIN = (ROOT / "firmware" / "core2_panel" / "src" / "main.cpp").read_text(
    encoding="utf-8"
)
DIRECT = (ROOT / "firmware" / "core2_panel" / "src" / "direct_camera.cpp").read_text(
    encoding="utf-8"
)
LV_CONFS = [
    (ROOT / "firmware" / "core2_panel" / "include" / "lv_conf.h").read_text(
        encoding="utf-8"
    ),
]


def key_geometry(name: str) -> tuple[int, int, int, int]:
    match = re.search(
        rf"{name}\s*=\s*keyBtn\([^,]+,\s*[^,]+,\s*"
        r"(\d+),\s*(\d+),\s*(\d+),\s*(\d+)",
        MAIN,
    )
    assert match is not None, f"missing {name} geometry"
    return tuple(map(int, match.groups()))


class TestCanvasChrome(unittest.TestCase):
    def test_d1_grid_is_40_168_32_without_a_gap(self):
        values = {
            name: int(re.search(rf"#define {name}\s+(\d+)", MAIN).group(1))
            for name in ("HEADER_H", "PAGE_H", "NAV_Y")
        }
        self.assertEqual(values, {"HEADER_H": 40, "PAGE_H": 168, "NAV_Y": 208})
        self.assertEqual(values["HEADER_H"] + values["PAGE_H"], values["NAV_Y"])

    def test_d2_header_cells_keep_rec_and_stop_separate(self):
        self.assertEqual(key_geometry("btnRecord"), (0, 0, 78, 40))
        self.assertEqual(key_geometry("btnStop"), (262, 0, 58, 40))
        self.assertIn("lv_obj_set_pos(headerContext, 78, 0);", MAIN)
        self.assertIn("lv_obj_set_size(headerContext, 184, HEADER_H);", MAIN)
        stop = MAIN.index('btnStop = keyBtn(screenMain, "STOP"')
        self.assertGreater(stop, MAIN.index("faultPane = lv_obj_create(screenMain)"))
        build = MAIN[stop:MAIN.index("softL = keyBtn", stop)]
        for pinned in (
            "LV_EVENT_PRESSED",
            "lv_obj_set_style_bg_color(btnStop, lv_color_hex(C_TEXT), 0)",
            "lv_obj_set_style_border_width(btnStop, 2, 0)",
            "lv_obj_set_style_border_side(btnStop, LV_BORDER_SIDE_LEFT, 0)",
            "lv_obj_set_style_text_letter_space(lblStopTxt, 2, 0)",
        ):
            self.assertIn(pinned, build)
        self.assertNotRegex(MAIN, r"style_(?:bg_)?opa\(btnStop")

    def test_d7_keeps_both_narrow_arrow_targets_on_every_state(self):
        self.assertRegex(MAIN, r"softL\s*=\s*keyBtn\(screenMain,\s*\"<\",\s*0,\s*NAV_Y,\s*40,\s*32")
        self.assertRegex(MAIN, r"softR\s*=\s*keyBtn\(screenMain,\s*\">\",\s*280,\s*NAV_Y,\s*40,\s*32")
        keys = MAIN[MAIN.index("static void updateSoftKeys() {"):]
        self.assertIn("lv_obj_clear_flag(softM, LV_OBJ_FLAG_HIDDEN)", keys)
        self.assertIn('onHand && !easeOpen && !handFeelOpen ? "JOG" : onJog && !easeOpen && !jogFeelOpen ? "CAMERA" : "HOME / HAND"', keys)
        self.assertIn("lv_obj_add_flag(lblRail, LV_OBJ_FLAG_HIDDEN)", keys)

    def test_d8_header_is_one_bold_title_line(self):
        start = MAIN.index("headerContext = lv_btn_create")
        build = MAIN[start:MAIN.index("pgControl = makePage", start)]
        self.assertIn("FONT_16", build)
        self.assertIn("lv_obj_add_flag(lblCtrl, LV_OBJ_FLAG_HIDDEN)", build)
        self.assertIn("lv_obj_add_flag(lblBatteryHeader, LV_OBJ_FLAG_HIDDEN)", build)
        title = MAIN[MAIN.index("static void updatePageTitle() {"):]
        self.assertIn("lv_label_set_text(lblPageName", title)
        self.assertIn("lv_obj_set_style_text_color(lblPageName", title)


class TestCanvasEvidence(unittest.TestCase):
    def test_d3_palette_and_green_mean_only_released_or_good_evidence(self):
        for name, value in (
            ("C_CYAN", "0x3DD6F5"), ("C_AMBER", "0xFFB020"),
            ("C_GREEN", "0x4CE05B"), ("C_MAGENTA", "0xE04CFF"),
            ("C_REC", "0xFF453A"),
        ):
            self.assertRegex(MAIN, rf"#define {name}\s+{value}")
        self.assertIn('"7/7 CAMERA READY"; colour = C_GREEN', MAIN)
        self.assertIn("cueWaiting ? C_GREEN", MAIN)

    def test_d4_red_tally_requires_authoritative_recording(self):
        refresh = MAIN[MAIN.index("const char *recordWord ="):]
        refresh = refresh[:refresh.index("lv_label_set_text(lblRecordTxt", 0)]
        self.assertIn("const uint32_t recordColour = rig.recording ? C_REC", refresh)
        self.assertNotIn("rig.recordIntent ? C_REC", refresh)
        self.assertIn("rig.recordUnconfirmed ? \"REC?\"", refresh)
        self.assertIn('"STOP?\\nREQ %lu:%02lu"', refresh)

    def test_d5_limit_bars_use_only_real_axes_and_direct_pan_is_zero(self):
        for bar, rig_limit in (
            ("driveLimitUp", "rig.limUp"), ("driveLimitDown", "rig.limDown"),
            ("driveLimitLeft", "rig.limLeft"), ("driveLimitRight", "rig.limRight"),
        ):
            self.assertIn(f"setLimitBar({bar},", MAIN)
            self.assertIn(rig_limit, MAIN)
        self.assertIn('"LIFT TO STOP"', MAIN)
        self.assertNotIn('PAN %.0f deg', MAIN)
        direct = MAIN[MAIN.index("static void pumpDirectState() {"):]
        self.assertIn("rig.limLeft = limLeftLed = 0;", direct)
        self.assertIn("rig.limRight = limRightLed = 0;", direct)

    def test_d6_counts_local_elapsed_and_cues_and_uses_signed_retry_time(self):
        workspace = MAIN[MAIN.index("static void refreshMotionWorkspace() {"):]
        workspace = workspace[:workspace.index("static void refreshDeviceWorkspace")]
        for fact in ("motionElapsedMs +=", "motionCueCount", '"ELAPSED %.1fs"', '"CUES %u"'):
            self.assertIn(fact, workspace)
        self.assertIn("if (motionLastCuePoint && !status.cuePoint)", workspace)
        self.assertNotIn(
            "if (status.cuePoint && status.cuePoint != motionLastCuePoint)",
            workspace,
        )
        self.assertNotIn('"ERR ', workspace)
        fault = MAIN[MAIN.index("if (faultStatus.retryAtMs)"):]
        self.assertIn("const int32_t left = static_cast<int32_t>", fault[:500])
        self.assertIn("faultStatus.retryAtMs - millis()", fault[:500])

    def test_tcp_open_is_a_session_step_after_wifi_joined(self):
        progress = MAIN[MAIN.index("static uint8_t wifiStepForPhase"):
                        MAIN.index("static const char *safeWifiDetail")]
        self.assertIn("case osmo::DirectPhase::TcpPoke:       return 6;", progress)
        self.assertIn('step = 6; state = "6/7 OPEN CAMERA SESSION"', progress)
        self.assertIn('state = "6/7 SESSION OPEN FAILED"', progress)
        self.assertNotIn("GET CAMERA IP", progress)
        self.assertNotIn("CAMERA IP FAILED", progress)

    def test_usb_camera_identity_never_contradicts_usb_ready(self):
        device = MAIN[MAIN.index("static void refreshDeviceWorkspace()"):
                      MAIN.index("static void refreshUi()")]
        self.assertIn('!rig.direct && rig.camera ? "CAMERA VIA USB"', device)


class TestCanvasSurfaceAndSafety(unittest.TestCase):
    def test_device_callback_navigation_uses_the_view_router(self):
        """A callback that writes cameraView directly leaves old overlays
        visible.  Only initialization/showPage reset and showCameraView own
        assignment; user-triggered DEVICE transitions go through the router."""
        callbacks = MAIN[MAIN.index("static void deviceCamerasPressed"):
                         MAIN.index("static void navigateLeft()")]
        self.assertNotRegex(callbacks, r"cameraView\s*=(?!=)")
        for target in ("CAM_HUB", "CAM_PROGRESS", "CAM_SCAN_CONFIRM",
                       "CAM_SWITCH_CONFIRM", "CAM_FORGET_CONFIRM"):
            self.assertIn(f"showCameraView({target})", callbacks)

    def test_back_browses_without_cancelling_an_inflight_pair(self):
        back = MAIN[MAIN.index("static void cameraProgressBackPressed"):
                    MAIN.index("// One owner for the backlight")]
        nav = MAIN[MAIN.index("static void navigateLeft()"):
                   MAIN.index("static void softLeftCb")]
        retry = MAIN[MAIN.index("static void wifiReconnectPressed"):
                     MAIN.index("static void cameraProgressBackPressed")]
        self.assertNotIn("cancelDeviceAction", back)
        self.assertNotIn("cancelDeviceAction", nav)
        self.assertIn("cancelDeviceAction", retry)

    def test_disconnect_closes_live_evidence_synchronously(self):
        clear = MAIN[MAIN.index("static void clearDirectRigEvidence()"):
                     MAIN.index("static void linkDisconnectPressed")]
        for fact in (
            "rig.direct = false", "rig.linked = false", "rig.camera = false",
            "rig.telemetry = false", "rig.poseValid = false",
            "rig.moving = false", "rig.armed = false",
            "rig.recordIntent = false", "rig.waitingCue = -1",
        ):
            self.assertIn(fact, clear)
        disconnect = MAIN[MAIN.index("static void linkDisconnectPressed"):
                          MAIN.index("static void linkReconnectPressed")]
        power = MAIN[MAIN.index("static void powerOffEvent"):
                     MAIN.index("static void recordEvent")]
        pump = MAIN[MAIN.index("static void pumpDirectState()"):
                    MAIN.index("// ---------------------------------------------------------------------- main")]
        self.assertIn("clearDirectRigEvidence();", disconnect)
        self.assertIn("clearDirectRigEvidence();", power)
        self.assertIn("if (rig.direct) clearDirectRigEvidence();", pump)

        status = DIRECT[DIRECT.index("DirectStatus DirectCamera::status() const"):
                        DIRECT.index("CameraCatalog DirectCamera::catalog()")]
        self.assertIn("const bool enabled = inputs_.wantEnabled;", status)
        self.assertIn("if (!enabled)", status)
        for fact in ("copy.ready = false", "copy.telemetry = false",
                     "copy.moving = false", "copy.motionProgramActive = false",
                     "copy.phase = DirectPhase::Off"):
            self.assertIn(fact, status)

    def test_d9_preserves_device_destructive_guards(self):
        for view in ("CAM_SCAN_CONFIRM", "CAM_SWITCH_CONFIRM", "CAM_FORGET_CONFIRM"):
            self.assertIn(view, MAIN)
        self.assertIn("HOLD_DESTROY_MS", MAIN)
        scan = MAIN[MAIN.index("static void cameraScanPressed("):
                    MAIN.index("static void cameraConnectPressed(")]
        switch = MAIN[MAIN.index("static void cameraConnectPressed("):
                      MAIN.index("static void cameraForgetPressed(")]
        forget = MAIN[MAIN.index("static void cameraForgetPressed("):
                      MAIN.index("static void cameraConfirmCancelPressed(")]
        confirm = MAIN[MAIN.index("static void cameraConfirmActionEvent("):
                       MAIN.index("static void cameraProgressBackPressed(")]
        self.assertIn("CAM_SCAN_CONFIRM", scan)
        self.assertIn("CAM_SWITCH_CONFIRM", switch)
        self.assertIn("CAM_FORGET_CONFIRM", forget)
        self.assertIn("code != LV_EVENT_PRESSED && code != LV_EVENT_PRESSING", confirm)
        self.assertIn("now - cameraForgetHoldStartedMs < HOLD_DESTROY_MS", confirm)

    def test_d10_canonical_font_config_is_legible(self):
        for size in (10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 40):
            self.assertIn(f"#define LV_FONT_MONTSERRAT_{size}   1", LV_CONFS[0])
        for size in (10, 12, 14, 16, 18, 20, 22, 26, 30, 40):
            self.assertIn(f"FONT_{size}", MAIN)
        self.assertIn("LV_FONT_MONTSERRAT_48   0", LV_CONFS[0])

    def test_d11_moves_is_an_adaptive_gallery_with_real_leg_controls(self):
        self.assertIn("MOTION_POINT_CAPACITY = 24", (ROOT / "firmware" / "core2_panel" / "src" / "motion_program.h").read_text(encoding="utf-8"))
        build = MAIN[MAIN.index("motionGalleryPane = lv_obj_create"):
                     MAIN.index("motionPointPane = makeConfigPane")]
        self.assertIn("lv_obj_set_scroll_dir(motionGalleryPane, LV_DIR_VER)", build)
        self.assertIn("motionTiles[i] = keyBtn", build)
        motion = MAIN[MAIN.index("static void refreshMotionEditor"):
                      MAIN.index("static void refreshMotionWorkspace")]
        for fact in (
            "// More points add scrolling, never smaller hit targets.",
            "const uint8_t columns = 2;",
            "const int tileW = 150;",
            "const int tileH = 72;",
            "motionTransitionName(point.transition)",
            "btnMotionPointTransition", "btnMotionPointTime",
            "btnMotionPointDwell", "btnMotionPointCapture",
            "btnMotionPointGoto", "btnMotionPointClear",
        ):
            self.assertIn(fact, motion)
        self.assertIn("MotionUiView::Point", MAIN)
        self.assertIn("MOVES %s / RUN >", MAIN)

    def test_d11_jog_edge_pickup_draws_the_same_zero_the_math_uses(self):
        jog = MAIN[MAIN.index("static void jogEvent("):
                   MAIN.index("// ------------------------------------------------------------------ builders")]
        self.assertIn("jogOriginX = pt.x;", jog)
        self.assertIn("jogOriginY = pt.y;", jog)
        self.assertRegex(
            jog,
            r"const int localX = constrain\(\(int\)\(pt\.x - a\.x1\), 0,",
        )
        self.assertIn("lv_obj_set_pos(jogDeadzone, localX - 37, localY - 37);", jog)
        self.assertIn("lv_obj_set_pos(jogKnob, localX - 21, localY - 21);", jog)
        self.assertNotRegex(jog, r"local[XY] = constrain\([^;]+, 56,")

    def test_d11_large_cue_word_does_not_overlap_helper_copy(self):
        motion = MAIN[MAIN.index("static void refreshMotionWorkspace() {"):]
        motion = motion[:motion.index("static void refreshDeviceWorkspace")]
        cue_visibility = motion[motion.index("if (cueWaiting) {"):
                                motion.index("const bool filledRun")]
        self.assertIn(
            "lv_obj_clear_flag(lblMotionRunSummary, LV_OBJ_FLAG_HIDDEN)",
            cue_visibility,
        )
        self.assertIn(
            "lv_obj_clear_flag(lblMotionRunSub, LV_OBJ_FLAG_HIDDEN)",
            cue_visibility,
        )

        self.assertIn("lv_obj_set_style_text_font(lblMotionRun, FONT_22, 0)", motion)
        self.assertNotIn("cueWaiting ? FONT_40", motion)

    def test_d11_battery_refusal_is_condition_based_not_page_based(self):
        self.assertIn("battery <= BATTERY_ARM_FLOOR", MAIN)
        self.assertIn("charging ? C_GREEN", MAIN)
        self.assertIn("batteryRefusalPane", MAIN)
        self.assertIn("HAND CLUTCH STILL AVAILABLE", MAIN)

    def test_d11_settings_use_white_for_passive_choices_without_legacy_rows(self):
        for contract in (
            "setConfigTile(btnLimitLeds, lblLimitLedsTxt, limitLedsEnabled, C_TEXT, true)",
            "setConfigTile(btnInvT, lblInvTTxt, invertTilt, C_TEXT, true)",
            "setConfigTile(btnInvP, lblInvPTxt, invertPan, C_TEXT, true)",
        ):
            self.assertIn(contract, MAIN)
        # The new grid owns discovery and state presentation.  Retaining
        # hidden quick rows duplicates settings state and makes a future
        # refresh regression look like a working control.
        self.assertNotIn("btnQuickTilt", MAIN)
        self.assertNotIn("lblSettingsSaved", MAIN)


if __name__ == "__main__":
    unittest.main()
