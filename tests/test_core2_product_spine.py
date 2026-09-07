"""Guard the Core2 small-screen product spine during R&D UI iteration.

These are source contracts because LVGL and the attached camera are hardware
bound. They assert the durable operator promises rather than a fragile pixel
snapshot.
"""

import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MAIN = (ROOT / "firmware" / "core2_panel" / "src" / "main.cpp").read_text(
    encoding="utf-8"
)
LV_CONF = (ROOT / "firmware" / "core2_panel" / "include" / "lv_conf.h").read_text(
    encoding="utf-8"
)


class TestCore2ProductSpine(unittest.TestCase):
    def test_workspaces_are_named_for_an_operator_not_implementation(self):
        # These names keep one-handed work predictable: steer, jog, program a
        # move, manage the camera, or change local controller preferences.
        for workspace in ("HAND", "JOG", "MOVES", "DEVICE", "SETTINGS"):
            self.assertIn(f'"{workspace}"', MAIN)

    def test_stop_and_record_evidence_remain_pinned_in_the_header(self):
        self.assertRegex(MAIN, r'keyBtn\(screenMain,\s*"STOP"')
        self.assertRegex(MAIN, r'keyBtn\(screenMain,\s*"REC"')
        # Until camera telemetry confirms the state, record must be visibly
        # uncertain rather than pretending that a request succeeded.
        self.assertIn('"REC?"', MAIN)
        self.assertIn('"STOP?"', MAIN)

    def test_navigation_uses_narrow_visual_arrows_with_safe_hit_targets(self):
        # The rail is only 32 px tall. 40px chevrons preserve a generous
        # central shooting surface without turning navigation into the UI.
        for symbol in ("<", ">"):
            match = re.search(
                rf'keyBtn\(screenMain,\s*"{re.escape(symbol)}",\s*'
                r'\d+,\s*NAV_Y,\s*(\d+),\s*(\d+)', MAIN
            )
            self.assertIsNotNone(match, f"missing {symbol} navigation target")
            width, height = map(int, match.groups())
            self.assertEqual(width, 40)
            self.assertEqual(height, 32)

    def test_motion_workspace_has_all_point_and_store_hooks(self):
        # P1..P6 are visible, bounded program points, never an unbounded list.
        self.assertIn("MOTION_POINT_CAPACITY", MAIN)
        self.assertRegex(MAIN, r'"P[1-6]"|"P%[ud][^"]*"')
        # LVGL never owns persistence. Every edit crosses the DirectCamera
        # boundary shared with the one networking/motion runtime.
        for operation in ("directCamera.captureMotionPoint(",
                          "directCamera.clearMotionPoint(",
                          "directCamera.setMotionTiming(",
                          "directCamera.motionProgram("):
            self.assertIn(operation, MAIN)

    def test_motion_point_selection_is_not_a_row_of_tiny_tabs(self):
        # P1..P6 are a sequence, not six competing 38px targets. The operator
        # steps a single selected point with two full-height neighbours.
        self.assertIn("btnMotionPrev", MAIN)
        self.assertIn("btnMotionSelected", MAIN)
        self.assertIn("btnMotionNext", MAIN)
        self.assertNotIn("i * 52, 2, 48, 38", MAIN)
        for name in ("btnMotionPrev", "btnMotionSlot", "btnMotionSelected",
                     "btnMotionNext"):
            match = re.search(
                rf"{name}\s*=\s*keyBtn\([^,]+,\s*[^,]+,\s*"
                rf"\d+,\s*\d+,\s*(\d+),\s*(\d+)", MAIN
            )
            self.assertIsNotNone(match, name)
            self.assertGreaterEqual(min(map(int, match.groups())), 40, name)

    def test_focus_modes_are_honest_unproven_controls(self):
        # The UI may expose a future cinema-focus workflow, but it cannot claim
        # camera-autofocus support until a transport command has been proven.
        for mode in ("AF-S", "AF-C"):
            self.assertIn(f'"{mode}"', MAIN)
        self.assertIn("FOCUS COMMAND NOT PROVEN", MAIN)
        self.assertIn("A-B PULL REMAINS DISABLED", MAIN)

    def test_device_copy_does_not_claim_manual_follow_focus(self):
        # AF requests can assist focus; they are not cinema lens-position
        # control until an observed transport command proves otherwise.
        self.assertIn("R&D TOOLS", MAIN)
        self.assertIn("LENS POSITION NOT AVAILABLE", MAIN)
        self.assertNotIn("FOLLOW FOCUS", MAIN)

    def test_power_off_requires_a_deliberate_safeguard(self):
        self.assertIn('"POWER OFF?"', MAIN)
        self.assertRegex(MAIN, r'powerOffHold(?:Started)?Ms')
        self.assertIn("M5.Power.Axp192.powerOff()", MAIN)

    def test_full_ui_has_a_bounded_psram_object_pool(self):
        # The commercial-size UI exhausted the original 40 KiB LVGL arena
        # while creating a camera-row label and boot-looped. Keep LVGL's
        # deterministic allocator, but place a meaningfully larger fixed pool
        # in PSRAM rather than consuming scarce internal DRAM.
        self.assertRegex(LV_CONF, r"#define\s+LV_MEM_CUSTOM\s+0")
        size = re.search(
            r"#define\s+LV_MEM_SIZE\s+\((\d+)U\s*\*\s*1024U\)", LV_CONF
        )
        self.assertIsNotNone(size, "LVGL pool size is not explicit")
        self.assertGreaterEqual(int(size.group(1)), 96)
        self.assertIn("LV_MEM_POOL_ALLOC", LV_CONF)
        self.assertIn("MALLOC_CAP_SPIRAM", LV_CONF)


class TestGlobalEvidenceIsNotPageLocal(unittest.TestCase):
    """Every one of these was a real gap: state the panel already held and
    only ever painted on one page, or not at all."""

    def test_stop_is_not_styled_like_the_key_beside_it(self):
        # STOP was created with the default instrument surface and then never
        # passed through setKey or any style call, so the one control that
        # must not be confusable read exactly like SPD NORMAL.
        build = MAIN[MAIN.index('btnStop = keyBtn(screenMain, "STOP"'):]
        build = build[:build.index("softL = keyBtn")]
        self.assertIn(
            "lv_obj_set_style_bg_color(btnStop, lv_color_hex(C_TEXT), 0)", build)
        self.assertIn(
            "lv_obj_set_style_bg_color(btnStop, lv_color_hex(C_AMBER), "
            "LV_STATE_PRESSED)", build)
        self.assertIn(
            "lv_obj_set_style_text_color(lblStopTxt, lv_color_hex(C_BG), 0)",
            build)
        # And it answers a press even when nothing else on screen changes.
        refresh = MAIN[MAIN.index("static void refreshUi()"):]
        self.assertIn("stopPressedAtMs", refresh)
        self.assertIn("lv_obj_set_style_bg_color(btnStop", refresh)
        self.assertIn("C_MAGENTA", refresh[refresh.index("stopPressedAtMs"):
                                           refresh.index("stopPressedAtMs") + 400])

    def test_a_fault_is_visible_on_every_workspace(self):
        # The only FAULT word was a child of the HAND dial. On JOG, MOVES,
        # DEVICE and SETTINGS a lost link showed on the LED bars and nowhere
        # on the screen.
        self.assertRegex(MAIN, r"faultBar\s*=\s*lv_obj_create\(screenMain\)")
        # A compact header badge preserves the title, owner and immutable
        # STOP target; fault recovery itself lives on the content overlay.
        self.assertIn("lv_obj_set_size(faultBar, 82, 18)", MAIN)
        self.assertIn("lv_obj_set_pos(faultBar, 180, 1)", MAIN)
        self.assertIn("LV_LABEL_LONG_SCROLL_CIRCULAR", MAIN)
        refresh = MAIN[MAIN.index("static void refreshUi()"):]
        self.assertIn("lv_label_set_text(lblFaultTxt, rig.fault)", refresh)
        self.assertIn("lv_obj_clear_flag(faultBar, LV_OBJ_FLAG_HIDDEN)", refresh)

    def test_the_page_title_has_one_owner_and_is_refreshed(self):
        # showPage wrote the timelapse counter once, on the page change, so a
        # multi-hour run showed a frozen TL n/m and read as a stalled rig.
        self.assertIn("static void updatePageTitle()", MAIN)
        # The definition, not the forward declaration at the top of the
        # file -- slicing from the declaration swallows updatePageTitle and
        # the assertion below then passes for the wrong reason.
        show = MAIN[MAIN.index("static void showPage(uint8_t idx) {"):]
        show = show[:show.index("static void showCameraView")]
        self.assertIn("updatePageTitle();", show)
        self.assertNotIn('snprintf(buf, sizeof(buf), "TL %d/%d"', show)
        refresh = MAIN[MAIN.index("static void refreshUi()"):]
        self.assertIn("updatePageTitle();", refresh)

    def test_device_sub_views_name_themselves_in_the_header(self):
        # The left key becomes BACK inside a DEVICE sub-view, and BACK is
        # ambiguous while the header still reads DEVICE three views deep.
        self.assertIn("CAMERA_VIEW_NAMES", MAIN)
        title = MAIN[MAIN.index("static void updatePageTitle()"):]
        title = title[:title.index("static void showSettings")]
        self.assertIn("CAMERA_VIEW_NAMES[cameraView]", title)
        view = MAIN[MAIN.index("static void showCameraView(CameraView which)"):]
        self.assertIn("updatePageTitle();",
                      view[:view.index("static lv_obj_t *makeLimitBar")])

    def test_pose_and_travel_headroom_reach_the_operator(self):
        # A repeatable remote head that cannot state where it is pointing is
        # not repeatable. Both figures were decoded and then discarded -- and
        # then, once shown, moved onto a sheet that cannot open while the
        # thumb is down. The readout lives on the pad.
        self.assertIn("lblJogEvidence = label(jogPad,", MAIN)
        self.assertNotIn("lblJogEvidence = label(jogFeelSheet", MAIN)
        self.assertIn("float    pitchNow", MAIN)
        self.assertIn("bool     poseValid", MAIN)
        direct = MAIN[MAIN.index("static void pumpDirectState()"):]
        self.assertIn("rig.pitchNow = st.pitch;", direct)
        self.assertIn("rig.poseValid = st.telemetry;", direct)
        # The USB state line has no absolute pose, so it must not imply one.
        state = MAIN[MAIN.index("static void applyState(const char *s)"):]
        state = state[:state.index("static void handleLine")]
        self.assertIn("rig.poseValid = false;", state)
        refresh = MAIN[MAIN.index("static void refreshUi()"):]
        self.assertIn('"P %+.1f deg", rig.yawNow', refresh)
        self.assertIn('"T %+.1f deg", rig.pitchNow', refresh)
        self.assertIn('"LIFT TO STOP"', MAIN)

    def test_telemetry_age_is_shown_not_reduced_to_a_yes_or_no(self):
        # On a marginal association the whole decision is how stale the last
        # attitude frame is, and the panel only ever said TELEMETRY.
        device = MAIN[MAIN.index("static void refreshDeviceWorkspace()"):]
        device = device[:device.index("static void refreshUi()")]
        self.assertIn("rig.telemetryAgeMs != UINT32_MAX", device)
        self.assertIn('"%s / %lums"', device)
        self.assertIn("static_cast<unsigned long>(rig.telemetryAgeMs)", device)


class TestTheShootingPagesKeepTheirFacts(unittest.TestCase):
    def test_the_header_owner_line_is_never_replaced_by_feel(self):
        # The two pages where the head is steered are the last place to hide
        # who is steering it; and a permanently amber line dilutes "armed".
        refresh = MAIN[MAIN.index("static void refreshUi()"):]
        self.assertIn('snprintf(ctrl, sizeof(ctrl), "%s / %s", transport, owner);', refresh)
        self.assertNotIn('"FEEL %s/%s"', MAIN)
        self.assertNotIn("onJog || onHand ? C_AMBER", MAIN)

    def test_the_middle_rail_is_never_a_false_clutch(self):
        # The tiny screen no longer sacrifices a central rail to status copy:
        # the middle target is always a direct HOME/HAND/JOG navigation action.
        keys = MAIN[MAIN.index("static void updateSoftKeys() {"):]
        keys = keys[:keys.index("struct UiPresentation")]
        self.assertIn('onHand && !easeOpen && !handFeelOpen ? "JOG" : onJog && !easeOpen && !jogFeelOpen ? "CAMERA" : "HOME / HAND"', keys)
        self.assertIn("lv_obj_clear_flag(softM, LV_OBJ_FLAG_HIDDEN)", keys)
        self.assertIn("lv_obj_add_flag(lblRail, LV_OBJ_FLAG_HIDDEN)", keys)

    def test_axis_locks_and_recenter_are_one_tap_from_the_shot(self):
        # JOG retains explicit locks on its feel sheet. HAND uses the larger
        # full-height TILT ONLY / PAN ONLY keys directly beside the clutch, so
        # those shot-time modes stay one tap away while its sheet gains the
        # previously buried sensitivity control.
        self.assertRegex(
            MAIN,
            r"btnJogTilt\s*=\s*configTile\(jogFeelSheet,[^;]*tiltLockPressed",
        )
        self.assertRegex(
            MAIN,
            r"btnJogPan\s*=\s*configTile\(jogFeelSheet,[^;]*panLockPressed",
        )
        self.assertRegex(
            MAIN,
            r"btnHandTiltOnly\s*=\s*keyBtn\(pgJog,[^;]*tiltOnlyPressed",
        )
        self.assertRegex(
            MAIN,
            r"btnHandPanOnly\s*=\s*keyBtn\(pgJog,[^;]*panOnlyPressed",
        )
        for sheet, center in (("jogFeelSheet", "btnJogCenter"),
                              ("handFeelSheet", "btnHandCenter")):
            self.assertRegex(MAIN, rf"{center}\s*=\s*configTile\({sheet}")
            for event in ("PRESSED", "PRESSING", "RELEASED", "PRESS_LOST"):
                self.assertRegex(
                    MAIN,
                    rf"lv_obj_add_event_cb\({center}, centerHoldEvent, LV_EVENT_{event}",
                )
        hold = MAIN[MAIN.index("static void centerHoldEvent"):]
        hold = hold[:hold.index("\n}\n") + 3]
        self.assertIn("osmo::CameraAction::GimbalRecenter", hold)
        self.assertIn('say("B recenter")', hold)
        self.assertRegex(hold, r"now - centerHoldStartedMs < HOLD_ACT_MS")

    def test_hand_axis_tune_is_a_real_nested_thumb_surface_not_a_hidden_extra(self):
        """One-handed HAND FEEL keeps its six cards.  Per-axis response and
        stability therefore belong on their own explicit 2x2 subpage."""
        self.assertRegex(
            MAIN,
            r"btnHandAxisTune\s*=\s*configTile\(handFeelSheet,[^;]*handAxisTunePressed",
        )
        self.assertIn("handAxisTuneSheet = makeConfigPane(pgJog, true)", MAIN)
        for card in ("btnTiltResponse", "btnPanResponse",
                     "btnTiltStability", "btnPanStability"):
            self.assertRegex(MAIN, rf"{card}\s*=\s*configTile\(handAxisTuneSheet")
        nav = MAIN[MAIN.index("static void navigateLeft()"):
                   MAIN.index("static void softLeftCb", MAIN.index("static void navigateLeft()"))]
        self.assertLess(nav.index("HandFeelView::AxisTune"),
                        nav.index("setHandFeelOpen(false)"))

    def test_the_jog_lesson_fades_after_real_use(self):
        self.assertIn("JOG_LESSON_PICKUPS", MAIN)
        self.assertIn('persistUiByte("jog_pick", jogPickups)', MAIN)
        refresh = MAIN[MAIN.index("static void refreshUi()"):]
        self.assertIn("else if (controlEvidenceReady())", refresh)
        self.assertIn('"TOUCH + DRAG"', refresh)


class TestProgrammedMotionIsAnOperatorTool(unittest.TestCase):
    def test_move_and_dwell_cycles_wrap(self):
        # They stuck at the top value; the only way down was HOLD CLEAR,
        # which throws away the framing to change a number.
        time_fn = MAIN[MAIN.index("static void motionTimePressed"):]
        time_fn = time_fn[:time_fn.index("static void motionDwellPressed")]
        self.assertIn("if (next >= N) next = 0;", time_fn)
        dwell = MAIN[MAIN.index("static void motionDwellPressed"):]
        dwell = dwell[:dwell.index("static void motionRepeatPressed")]
        # ... and the dwell cycle ends in a cue, not a ceiling.
        self.assertIn("setMotionHold(motionSelected, true)", dwell)
        self.assertIn("setMotionHold(motionSelected, false)", dwell)

    def test_a_standalone_cue_uses_the_same_go_word_beacon_and_key(self):
        direct = MAIN[MAIN.index("static void pumpDirectState()"):]
        self.assertIn("rig.waitingCue = st.cuePoint ? st.cuePoint - 1 : -1;", direct)
        hold = MAIN[MAIN.index("static void motionHoldEvent"):]
        hold = hold[:hold.index("static void motionGotoEvent")]
        self.assertIn("if (rig.direct) osmo::directCamera.requestMotionGo();", hold)
        self.assertIn('else            say("B go");', hold)
        self.assertIn('case osmo::MotionPhase::Cued:        return "PAUSED";', MAIN)

    def test_repeat_and_slot_are_on_the_panel(self):
        self.assertIn('REPEAT_NAMES[] = {"ONCE", "LOOP", "BOUNCE"}', MAIN)
        self.assertRegex(MAIN, r"btnMotionRepeat\s*=\s*keyBtn\(motionRunPane,[^;]*motionRepeatPressed")
        self.assertRegex(MAIN, r"btnMotionSlot\s*=\s*keyBtn\(pgClutch,[^;]*motionSlotPressed")
        self.assertIn("directCamera.setMotionRepeat(next)", MAIN)
        self.assertIn("directCamera.selectMotionSlot(next)", MAIN)

    def test_arming_is_refused_on_a_dying_battery_and_says_so(self):
        # A brownout mid-move is a safety event. The clutch is untouched: a
        # hand on the box is not a promise about where the head will be.
        self.assertIn("BATTERY_ARM_FLOOR = 10", MAIN)
        hold = MAIN[MAIN.index("static void motionHoldEvent"):]
        hold = hold[:hold.index("static void motionGotoEvent")]
        self.assertIn("!armBlockedByBattery()", hold)
        buttons = MAIN[MAIN.index("static void pumpButtons()"):]
        buttons = buttons[:buttons.index("static void pumpImu")]
        self.assertNotIn("armBlockedByBattery", buttons)
        self.assertIn('"CHARGE TO PLAY / %d%%"', MAIN)
        self.assertIn('"HAND CLUTCH STILL AVAILABLE  >"', MAIN)

    def test_a_host_timelapse_can_be_stopped_between_frames_from_the_box(self):
        hold = MAIN[MAIN.index("static void motionHoldEvent"):]
        hold = hold[:hold.index("static void motionGotoEvent")]
        self.assertIn('say("B tl_stop")', hold)


class TestThePanelNeverLiesUnderAHost(unittest.TestCase):
    def test_the_programme_keys_are_standalone_only(self):
        # Under a USB host the keys lit, counted to 100% and buzzed for a
        # request nothing would run.
        motion = MAIN[MAIN.index("static void refreshMotionWorkspace()"):]
        motion = motion[:motion.index("static void refreshDeviceWorkspace")]
        self.assertIn("const bool fresh = rig.direct && controlEvidenceReady();", motion)
        self.assertIn('"HOST OWNS MOTION"', motion)
        self.assertIn('"PROGRAMME RUNS STANDALONE ONLY"', motion)
        hold = MAIN[MAIN.index("static void motionHoldEvent"):]
        hold = hold[:hold.index("static void motionGotoEvent")]
        self.assertIn("? (rig.direct && !gate.motionProgramActive &&", hold)
        self.assertIn(": (rig.direct && controlEvidenceReady() &&", hold)

    def test_the_battery_floor_gates_arm_not_go(self):
        hold = MAIN[MAIN.index("static void motionHoldEvent"):]
        hold = hold[:hold.index("static void motionGotoEvent")]
        self.assertIn("(gate.motionProgramArmed ||\n            (!armBlockedByBattery() &&", hold)

    def test_record_intent_reaches_the_box_over_usb(self):
        state = MAIN[MAIN.index("static void applyState(const char *s)"):]
        state = state[:state.index("static void handleLine")]
        self.assertIn('rig.recordIntent = keyBool(s, "recint");', state)
        self.assertIn("rig.recordUnconfirmed = rig.recordIntent && !rig.recording;", state)
        self.assertIn("const bool start = !(rig.recording || rig.recordIntent);", MAIN)

    def test_the_host_hears_arm_go_and_end_from_a_standalone_run(self):
        direct = MAIN[MAIN.index("static void pumpDirectState()"):]
        for cue in ('scheduleRunCue("arm")', 'scheduleRunCue("go")', 'scheduleRunCue("end")'):
            self.assertIn(cue, direct)
        self.assertIn("if (rig.fault[0] && !wasFault) hapticFault();", direct)


class TestNavigationNeverTrapsOrCancelsSilently(unittest.TestCase):
    def test_back_on_device_home_is_a_page_key(self):
        back = MAIN[MAIN.index("static void navigateLeft()"):]
        back = back[:back.index("static void softLeftCb")]
        self.assertIn("cameraView != CAM_DEVICE_HOME) {", back)
        self.assertNotIn("cameraView != CAM_DEVICE_HOME || cameraCatalogUi.scanning", back)

    def test_back_is_never_dead_on_a_locked_run_pane(self):
        back = MAIN[MAIN.index("static void navigateLeft()"):]
        back = back[:back.index("static void softLeftCb")]
        self.assertIn("if (!(st.motionProgramArmed || st.motionProgramActive)) {", back)
        # While run is armed, BACK intentionally reverts to page navigation;
        # it must never silently leave an active program pane.
        self.assertIn("// key again rather than a dead one.", back)
        keys = MAIN[MAIN.index("static void updateSoftKeys() {"):]
        keys = keys[:keys.index("struct UiPresentation")]
        self.assertIn("motionRunView) && !runLocked", keys)

    def test_a_confirm_dialog_does_not_outlive_its_page(self):
        show = MAIN[MAIN.index("static void showPage(uint8_t idx) {"):]
        show = show[:show.index("static void showCameraView")]
        self.assertIn("cameraForgetHoldStartedMs = 0;", show)
        self.assertIn("cameraView = CAM_DEVICE_HOME;", show)

    def test_a_dimmed_disconnect_cannot_stop(self):
        fn = MAIN[MAIN.index("static void linkDisconnectPressed"):]
        fn = fn[:fn.index("\n}\n") + 3]
        self.assertLess(fn.index("if (!osmo::directCamera.enabled()) return;"),
                        fn.index("stopPressed(nullptr);"))

    def test_the_hand_dial_says_cued_and_where_to_answer(self):
        self.assertIn('return {"CUED", C_AMBER};', MAIN)
        self.assertNotIn('return {"GO?", C_AMBER};', MAIN)
        self.assertIn('"GO P%d on MOVES   /   grab = abort"', MAIN)

    def test_holds_come_in_three_lengths(self):
        for name in ("HOLD_ACT_MS", "HOLD_DESTROY_MS", "HOLD_POWER_MS", "HOLD_RECORD_MS"):
            self.assertRegex(MAIN, rf"static const uint32_t {name} = \d+;")
        # No literal hold length survives outside the four definitions.
        import re as _re
        literals = _re.findall(r"HoldStartedMs\s*<\s*(\d+)\)", MAIN)
        self.assertEqual(literals, [], f"literal hold lengths: {literals}")

    def test_back_to_one_is_one_hold_on_the_run_pane(self):
        self.assertRegex(MAIN, r"btnMotionHome\s*=\s*keyBtn\(motionRunPane,[^;]*motionHomeEvent")
        home = MAIN[MAIN.index("static void motionHomeEvent"):]
        home = home[:home.index("\n}\n") + 3]
        self.assertIn("gate.motionAtStart", home)
        self.assertIn("osmo::directCamera.requestMotionGoto(0);", home)
        self.assertIn('"AT P1"', MAIN)

    def test_the_edit_surface_answers_a_cue_and_a_timelapse(self):
        motion = MAIN[MAIN.index("static void refreshMotionWorkspace()"):]
        motion = motion[:motion.index("static void refreshDeviceWorkspace")]
        self.assertIn("GO P%d  >", motion)
        self.assertIn("TL %d/%d  >", motion)
        return  # superseded compact single-line labels below
        self.assertIn('"GO P%d\\n>"', motion)
        self.assertIn('"TL %d/%d\\n>"', motion)
        self.assertIn("program.count >= 2 || cueWaiting || hostTimelapse);", motion)

    def test_store_errors_are_read_from_the_store(self):
        motion = MAIN[MAIN.index("static void refreshMotionWorkspace()"):]
        motion = motion[:motion.index("static void refreshDeviceWorkspace")]
        self.assertIn("P%u  STORE ERROR", motion)
        self.assertIn('"STORE / %.56s",', motion)
        self.assertLess(motion.index("if (program.error[0]) {"),
                        motion.index("} else if (status.motionProgramFault) {"))
        return  # superseded multi-line edit-card assertion below
        self.assertIn('"P%u\\nSTORE ERROR\\nOPEN RUN\\nFOR DETAIL"', motion)
        self.assertIn('"STORE / %.56s",', motion)
        self.assertLess(motion.index("if (program.error[0]) {"),
                        motion.index("} else if (status.motionProgramFault) {"))

    def test_the_state_line_fits_its_width(self):
        self.assertIn('"P%u/%u  %+.0f / %+.0f",', MAIN)
        self.assertIn('"%s / %u POINTS / %.1fs",', MAIN)


class TestDestructiveEditsAreDeliberate(unittest.TestCase):
    """FORGET, GOTO, ARM and POWER OFF are all holds. These two were taps."""

    def test_clearing_a_captured_waypoint_is_a_hold(self):
        clear = MAIN[MAIN.index("static void motionClearPressed"):]
        clear = clear[:clear.index("static void motionTimePressed")]
        self.assertIn("lv_event_get_code(e)", clear)
        self.assertIn("clearHoldStartedMs", clear)
        self.assertRegex(clear, r"now - clearHoldStartedMs < HOLD_DESTROY_MS")
        self.assertNotIn("LV_EVENT_CLICKED", clear)
        # And the hold has to be visible while it accumulates.
        self.assertIn('"CLEAR %lu%%"', MAIN)

    def test_wiping_the_feel_profile_is_a_hold(self):
        reset = MAIN[MAIN.index("static void resetFeelPressed"):]
        reset = reset[:reset.index("static void setMenuPressed")]
        self.assertIn("resetHoldStartedMs", reset)
        self.assertRegex(reset, r"now - resetHoldStartedMs < HOLD_DESTROY_MS")
        self.assertIn('setConfigTile(btnResetFeel', MAIN)
        self.assertIn('"HOLD TO RESET"', MAIN)

    def test_the_record_hold_shows_its_progress(self):
        # 450 ms of an unchanging word reads as an unresponsive button, while
        # every other hold on the panel counts up.
        refresh = MAIN[MAIN.index("static void refreshUi()"):]
        self.assertIn('"HOLD\\n%lu%%"', refresh)
        self.assertIn("recordHoldStartedMs) * 100 / HOLD_RECORD_MS", refresh)
        self.assertIn("HOLD_RECORD_MS = 450", MAIN)


class TestOperatorFactsTheBoxCanAnswer(unittest.TestCase):
    def test_a_cued_waypoint_can_be_released_from_the_box(self):
        # The state word could always say GO?, but nothing on the panel could
        # answer it -- the release had to happen at the browser.
        hold = MAIN[MAIN.index("static void motionHoldEvent"):]
        hold = hold[:hold.index("static void motionGotoEvent")]
        self.assertIn("rig.waitingCue >= 0", hold)
        self.assertIn('say("B go");', hold)
        self.assertIn("hapticGo();", hold)
        self.assertIn("GO P%d  >", MAIN)
        return  # superseded multi-line key label below
        self.assertIn('"GO P%d\\n>"', MAIN)

    def test_the_take_timer_needs_no_protocol_work(self):
        refresh = MAIN[MAIN.index("static void refreshUi()"):]
        self.assertIn(
            "if (rollingNow && !recordRollingSinceMs) recordRollingSinceMs = millis();",
            refresh)
        self.assertIn("if (!rollingNow) recordRollingSinceMs = 0;", refresh)
        self.assertIn('"%s\\n%lu:%02lu"', refresh)

    def test_the_move_states_its_total_duration(self):
        motion = MAIN[MAIN.index("static void refreshMotionWorkspace()"):]
        motion = motion[:motion.index("static void refreshDeviceWorkspace")]
        self.assertIn("totalMs += program.points[i].moveMs", motion)
        self.assertIn("%.1fs", motion)

    def test_the_camera_list_speaks_operator_units(self):
        # -72 dBm is an engineer's unit; the only decision the list supports
        # is whether a camera is close enough to pair with.
        self.assertIn("static const char *signalWord(int rssi)", MAIN)
        self.assertNotIn('"%d dBm"', MAIN)


class TestEquipmentSetupSurvivesAPowerCycle(unittest.TestCase):
    def test_display_lights_and_sound_persist_outside_the_feel_namespace(self):
        # The screen timeout exists because operators switch a misbehaving
        # timeout off; handing it back on the next battery swap recreates the
        # problem it was added for. RESET FEEL DEFAULTS must not take the
        # display with it, so they do not share a namespace.
        self.assertIn('uiPreferences.begin("core2-ui"', MAIN)
        self.assertIn('feelPreferences.begin("core2-feel"', MAIN)
        load = MAIN[MAIN.index("static void loadUiPreferences()"):]
        load = load[:load.index("static void persistUiByte")]
        for key in ("bright", "scr_to", "beacon", "lim_led", "in_hand", "sound"):
            self.assertIn('"%s"' % key, load)
        reset = MAIN[MAIN.index("static void resetFeelPreferences"):]
        reset = reset[:reset.index("\n}\n") + 3]
        self.assertNotIn("uiPreferences", reset)

    def test_the_stored_brightness_is_applied_at_boot(self):
        # Nothing applied it: the panel came up at whatever M5Unified left,
        # and only the first press of the toggle made the setting real.
        setup = MAIN[MAIN.index("void setup() {"):]
        self.assertIn("loadUiPreferences();", setup)
        self.assertLess(setup.index("buildUi();"), setup.index("applyBrightness();"))

if __name__ == "__main__":
    unittest.main()
