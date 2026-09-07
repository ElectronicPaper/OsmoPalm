"""Static checks on the Core2 firmware source.

There is no C++ test harness here and building one for an LVGL/M5Unified
sketch is not proportionate. But the two faults this firmware has actually
shipped were both structural and both visible in the text:

* `profBtn[]` was declared, driven every frame by the refresh loop, and never
  created. LVGL dereferenced the nulls and the box boot-looped on a
  LoadProhibited panic -- which on a device with no console reads as dead
  hardware, not as a missing widget.
* Page indices drifted when a fourth page was added: the soft-key legend and
  the bezel handler still tested `page == 2` for the last page.

Neither needs a compiler to catch. These checks are deliberately narrow --
they encode the mistakes that were made, not a general C++ linter.
"""

import re
import unittest
from pathlib import Path

SRC = (Path(__file__).resolve().parents[1]
       / "firmware" / "core2_panel" / "src" / "main.cpp")

# Read at import so every class below can use it, but absence must not be an
# ImportError: a deployment that ships only the host code (the Pi build) would
# otherwise fail collection rather than skipping, and one unrunnable test would
# read as a broken suite.
if SRC.exists():
    CODE = SRC.read_text(encoding="utf-8")
    NO_SOURCE = None
else:
    CODE = ""
    NO_SOURCE = f"firmware source not present at {SRC}"


def setUpModule():
    if NO_SOURCE:
        raise unittest.SkipTest(NO_SOURCE)


def strip_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    return re.sub(r"//[^\n]*", "", text)


BODY = strip_comments(CODE)


def declared_widgets() -> set[str]:
    """Names from `static lv_obj_t *a, *b[N];` declarations.

    Skips function signatures, which start identically -- `static lv_obj_t
    *keyBtn(...)` declares a return type, not a widget, and its parameters are
    not widgets either.
    """
    names: set[str] = set()
    for line in BODY.splitlines():
        line = line.strip()
        if not line.startswith("static lv_obj_t *") or "(" in line:
            continue
        for m in re.finditer(r"\*(\w+)", line):
            names.add(m.group(1))
    return names


class TestEveryWidgetIsCreated(unittest.TestCase):
    """A widget that is used but never assigned is a null the first refresh
    dereferences, and LVGL does not check."""

    def test_used_widgets_are_assigned_somewhere(self):
        unassigned = []
        for name in sorted(declared_widgets()):
            used = re.search(rf"\b{name}\s*(\[[^\]]*\])?\s*[,)]", BODY)
            # Two creation paths, both real: a direct assignment, or the
            # address handed to a builder that fills it in (`&lblGoTxt`).
            assigned = (re.search(rf"\b{name}\s*(\[[^\]]*\])?\s*=[^=]", BODY)
                        or re.search(rf"&{name}\b", BODY))
            if used and not assigned:
                unassigned.append(name)
        self.assertEqual(unassigned, [],
                         f"declared and used but never created: {unassigned}")

    def test_set_key_tolerates_a_null(self):
        """Belt and braces for the above: the helper that panicked is the one
        every button state goes through, so it checks."""
        fn = BODY[BODY.index("static void setKey("):]
        fn = fn[:fn.index("\n}")]
        self.assertIn("if (!btn || !lbl) return;", fn)


class TestPageIndices(unittest.TestCase):
    """Adding a page must not leave a stale index behind."""

    def _page_count(self) -> int:
        m = re.search(r"PAGE_COUNT\s*=\s*(\d+)", BODY)
        self.assertIsNotNone(m, "PAGE_COUNT not found")
        return int(m.group(1))

    def test_page_count_matches_the_pages_that_exist(self):
        pages = set(re.findall(r"\b(pg[A-Z]\w+)\s*=\s*makePage\(\)", BODY))
        self.assertEqual(len(pages), self._page_count(),
                         f"PAGE_COUNT disagrees with {sorted(pages)}")

    def test_show_page_reveals_every_page(self):
        # The router deliberately uses one ternary selection for the only
        # reveal after it hides all five pages.  Pin that every workspace is
        # reachable, rather than relying on a regex that mistakes the first
        # nested `if` for the end of the C++ function.
        start = BODY.index("static void showPage(uint8_t idx) {")
        block = BODY[start:BODY.index("static void showCameraView", start)]
        reveal = re.search(r"lv_obj_clear_flag\(page == 0 \? (.*?)\s*,\s*LV_OBJ_FLAG_HIDDEN\)",
                           block, re.S)
        self.assertIsNotNone(reveal, "showPage never reveals its selected page")
        for name in ("pgControl", "pgJog", "pgClutch", "pgDevice", "pgSetup"):
            self.assertIn(name, reveal.group(1), f"{name} can never be shown")
        self.assertIn("static uint8_t page = static_cast<uint8_t>(Workspace::Hand);", BODY)
        self.assertIn("showPage(static_cast<uint8_t>(Workspace::Hand));", BODY)

    def test_nothing_still_tests_a_stale_last_page_index(self):
        """`page == 2` meant the last page when there were three. It now means
        the clutch page, and the soft key and bezel handler both used it to
        mean 'settings'."""
        last = self._page_count() - 1
        stale = re.findall(r"page\s*==\s*(\d+)", BODY)
        self.assertNotIn(str(last + 1), stale, "index beyond the last page")
        for idx in stale:
            self.assertLessEqual(int(idx), last,
                                 f"page == {idx} exceeds the last page {last}")


class TestControlVocabulary(unittest.TestCase):
    """The operator chooses behavior, never a guessed sensor orientation."""

    def test_pose_presets_are_removed(self):
        for obsolete in ("PROFILE_NAMES", "PROFILE_GLYPHS", "ImuProfile",
                         "PROF_FLAT", "hold the box like"):
            self.assertNotIn(obsolete, BODY)

    def test_templates_and_independent_controls_are_visible(self):
        self.assertIn('{"HAND FOLLOW", "GYRO RATE", "AIR MOUSE"}', BODY)
        self.assertIn('{"FOLLOW", "RATE", "AIR"}', BODY)
        self.assertIn("TEMPLATE_COUNT = 3", BODY)
        self.assertIn('{"CRISP", "FLUID", "GLIDE"}', BODY)
        self.assertIn('{"FINE", "BALANCED", "DIRECT"}', BODY)
        self.assertIn('{"QUIET", "BALANCED", "RESPONSIVE"}', BODY)
        # HAND/JOG feel now own the frequently adjusted controls directly;
        # the Settings grid only routes into these existing surfaces.
        for widget in ("btnFollowTemplate", "btnFollowSensitivity",
                       "btnFollowSmooth", "btnHandAxisTune",
                       "btnJogSpeed", "btnJogSmooth"):
            self.assertIn(widget, BODY)


class TestFeelPersistence(unittest.TestCase):
    def test_non_secret_operator_feel_survives_power_cycles(self):
        self.assertIn('#include <Preferences.h>', CODE)
        self.assertIn('feelPreferences.begin("core2-feel", false)', BODY)
        for key in ('"sens"', '"speed"', '"smooth"', '"template"',
                    '"inv_t"', '"inv_p"', '"tilt_rsp"', '"pan_rsp"',
                    '"tilt_stab"', '"pan_stab"'):
            self.assertIn(key, BODY)
        setup = BODY[BODY.index("void setup()") : BODY.index("void loop()")]
        self.assertIn("loadFeelPreferences();", setup)
        self.assertLess(setup.index("loadFeelPreferences();"),
                        setup.index("buildUi();"))

    def test_transient_axis_locks_are_never_restored(self):
        load = BODY[BODY.index("static void loadFeelPreferences") :
                    BODY.index("static void persistFeelByte")]
        self.assertNotIn("lockTilt", load)
        self.assertNotIn("lockPan", load)

    def test_hand_basis_migration_clears_old_compensating_inversions_once(self):
        load = BODY[BODY.index("static void loadFeelPreferences"):
                    BODY.index("static void persistFeelByte")]
        self.assertIn('feelPreferences.getBool("hand_nat_v2", false)', load)
        for fact in (
            "invertTilt = false;", "invertPan = false;",
            'feelPreferences.putBool("inv_t", false);',
            'feelPreferences.putBool("inv_p", false);',
            'feelPreferences.putBool("hand_nat_v2", true);',
        ):
            self.assertIn(fact, load)

    def test_direction_page_has_a_full_width_reset(self):
        """Reset belongs with the persistent direction preferences, not the
        live shot-surface axis locks.  It must remain an intentional, full
        width hold target."""
        self.assertRegex(
            BODY,
            r'btnResetFeel\s*=\s*configTile\(pgSetPanes\[SET_DIRECTION\],'
            r'[\s\S]*?\d+,\s*\d+,\s*308,\s*72',
        )
        reset = BODY[BODY.index("static void resetFeelPreferences") :
                     BODY.index("static void flushCb")]
        for default in ("speedIdx = 1",
                        "smoothIdx = 1", "templateIdx = 0",
                        "invertTilt = false", "invertPan = false",
                        "tiltResponseIdx = 1", "panResponseIdx = 1",
                        "tiltStabilityIdx = 1", "panStabilityIdx = 1"):
            self.assertIn(default, reset)

    def test_axis_tune_migrates_legacy_master_sensitivity_to_balanced(self):
        """A battery swap from a pre-axis build must retain its learned
        master gain, while the newly independent axes begin neutral."""
        load = BODY[BODY.index("static void loadFeelPreferences"):
                    BODY.index("static void persistFeelByte")]
        self.assertIn('getUChar("tilt_rsp", legacySensitivity)', load)
        self.assertIn('getUChar("pan_rsp", legacySensitivity)', load)
        self.assertIn('getUChar("tilt_stab", 1)', load)
        self.assertIn('getUChar("pan_stab", 1)', load)


class TestVersionIsSingleSourced(unittest.TestCase):
    def test_defined_once_and_used_not_retyped(self):
        """The hello line and the settings page disagreed about the version
        for several builds, because each carried its own literal."""
        self.assertEqual(len(re.findall(r"#define\s+FW_VERSION", BODY)), 1)
        self.assertNotRegex(BODY, r'"\d+\.\d+-\w+ imu,',
                            "a version literal is being retyped")

    def test_soft_haptic_build_is_identifiable(self):
        self.assertRegex(CODE, r'#define FW_VERSION "7\.9\.1-return-icon"')


class TestHapticFloor(unittest.TestCase):
    def test_it_clears_the_parked_notch(self):
        """The panel caps a merely-parked axis at 35, which the wire rounds to
        40. A floor at or below that buzzes forever on a rig set down near a
        stop -- the exact nuisance the graded warning exists to avoid."""
        m = re.search(r"HAPTIC_FLOOR\s*=\s*(\d+)", BODY)
        self.assertIsNotNone(m)
        self.assertGreater(int(m.group(1)), 40)


if __name__ == "__main__":
    unittest.main()


class TestRawImuOwnership(unittest.TestCase):
    """The UI samples hardware; the direct controller owns estimation."""

    def test_all_six_raw_axes_reach_the_controller(self):
        self.assertIn("setImu(ax, ay, az, gx, gy, gz, sampleAt)", BODY)
        sample = BODY[BODY.index("static void sampleImu()"):
                      BODY.index("static void pumpImu(uint32_t now)")]
        self.assertIn("static_cast<int>(M5.Imu.update())", sample)
        self.assertIn("sensor_mask_accel", sample)
        self.assertIn("sensor_mask_gyro", sample)
        self.assertIn("M5.Imu.getImuData(&data)", sample)
        self.assertIn("if ((mask & required) != required)", sample)
        self.assertLess(sample.index("M5.Imu.getImuData(&data)"),
                        sample.index("const uint32_t sampleAt = millis()"))

    def test_ui_does_not_mutate_attitude_for_inversion(self):
        sampling = BODY[BODY.index("static void sampleImu"):BODY.index(
            "static void pumpDirectState", BODY.index("static void sampleImu"))]
        self.assertNotIn("invertTilt", sampling)
        self.assertNotIn("invertPan", sampling)
        self.assertNotIn("mapImu", BODY)
        self.assertNotIn("calibrateGyro", BODY)

    def test_direction_preferences_cross_the_output_contract(self):
        self.assertIn("tiltResponseIdx, panResponseIdx,", BODY)
        self.assertIn("tiltStabilityIdx, panStabilityIdx,", BODY)
        self.assertIn("rig.lockTilt, rig.lockPan, invertTilt, invertPan);", BODY)
        for field in ("tiltResponseIdx", "panResponseIdx",
                      "tiltStabilityIdx", "panStabilityIdx"):
            self.assertIn(field, BODY)


class TestSettingsSubPages(unittest.TestCase):
    """The small-screen settings hub is an icon grid with deliberate routes.

    A pane that is created but never reachable from the menu is invisible; a
    pane reachable but never created is a null LVGL will dereference. The
    profBtn boot loop was exactly the second kind, so it is worth checking
    both directions here.
    """

    def test_every_pane_is_created(self):
        self.assertIn("pgSetPanes[i] = makeConfigPane(", BODY)

    def test_the_settings_enum_is_the_new_information_architecture(self):
        enum = re.search(r"enum SetSub : uint8_t \{(.*?)\};", BODY, re.S)
        self.assertIsNotNone(enum)
        members = re.findall(r"SET_[A-Z]+", enum.group(1))
        self.assertEqual(
            members,
            ["SET_MENU", "SET_CONTROLS", "SET_DIRECTION", "SET_DISPLAY",
             "SET_FEEDBACK", "SET_HAPTIC", "SET_SYSTEM", "SET_COUNT"],
        )
        for obsolete in ("SET_LIGHTS", "SET_SOUND", "SET_INPUT"):
            self.assertNotIn(obsolete, BODY)

    def test_root_grid_and_control_subgrid_reach_every_settings_leaf(self):
        """The root must be a compact discovery grid; controls get their own
        sub-grid because live feel changes route to the shot sheets instead of
        duplicating those controls in Settings."""
        for sub in ("SET_CONTROLS", "SET_DISPLAY", "SET_FEEDBACK",
                    "SET_SYSTEM"):
            self.assertIn(f"(void *)(intptr_t){sub}", BODY)
        self.assertTrue(
            "(void *)(intptr_t)SET_DIRECTION" in BODY
            or "settingsDirectionPressed" in BODY,
            "DIRECTION needs a reachable control-grid route",
        )
        for adapter in ("settingsHandFeelPressed", "settingsJogFeelPressed"):
            self.assertIn(adapter, BODY)
        self.assertIn("setHandFeelOpen(true)", BODY)
        self.assertIn("setJogFeelOpen(true)", BODY)

    def test_camera_is_the_first_settings_destination_and_reuses_device_flow(self):
        """Connection is both the first discovery tile and a canonical DEVICE
        workflow.  The grid must route there; it must not grow a second camera
        manager or an unreachable SETTINGS camera pane."""
        self.assertNotIn("SET_CAMERAS", BODY)
        self.assertRegex(
            BODY,
            r'settingsCameraPressed[\s\S]*?showPage\(static_cast<uint8_t>'
            r'\(Workspace::Device\)\)[\s\S]*?showCameraView\(CAM_HUB\)',
        )

    def test_root_grid_is_scrollable_two_column_touch_layout(self):
        """Five destinations are intentionally not squeezed into rows or tiny
        targets.  150x72 cards retain an 8px thumb gutter on 320px."""
        self.assertIn("i == SET_MENU || i == SET_CONTROLS", BODY)
        self.assertIn("lv_obj_set_scroll_dir(pane, LV_DIR_VER)", BODY)
        for geometry in ("6, 4, 150, 72", "164, 4, 150, 72",
                         "6, 84, 150, 72", "164, 84, 150, 72",
                         "6, 164, 150, 72"):
            self.assertIn(geometry, BODY)
        self.assertRegex(BODY, r'"CAMERA"[\s\S]{0,350}"CONTROL"')

    def test_direction_keeps_persistent_orientation_out_of_live_axis_locks(self):
        """Tilt/pan locks are shot-time controls on HAND/JOG.  SETTINGS only
        owns their durable inversion preferences and reset action."""
        for button in ("btnInvT", "btnInvP", "btnResetFeel"):
            self.assertRegex(
                BODY, rf"{button}\s*=\s*configTile\(pgSetPanes\[SET_DIRECTION\]"
            )
        for transient in ("btnTilt", "btnPan", "btnHandTilt", "btnHandPan",
                          "btnJogTilt", "btnJogPan"):
            self.assertNotRegex(
                BODY, rf"{transient}\s*=\s*configTile\(pgSetPanes\[SET_DIRECTION\]"
            )

    def test_direction_back_returns_to_the_feel_sheet_that_opened_it(self):
        route_start = BODY.index("static void settingsDirectionPressed")
        route = BODY[route_start:BODY.index(
            "static void settingsPowerPressed", route_start
        )]
        self.assertIn("SettingsReturn::HandFeel", route)
        self.assertIn("SettingsReturn::JogFeel", route)
        self.assertLess(route.index("settingsReturn ="), route.index("showPage("))

        nav_start = BODY.index("static void navigateLeft()")
        nav = BODY[nav_start:BODY.index("static void softLeftCb", nav_start)]
        self.assertIn("setSub == SET_DIRECTION", nav)
        self.assertIn("setHandFeelOpen(true);", nav)
        self.assertIn("setJogFeelOpen(true);", nav)
        self.assertIn("settingsReturn = SettingsReturn::None;", nav)

    def test_feedback_collects_light_sound_and_haptic_preferences(self):
        """Haptic discovery remains with feedback, but its three large
        controls get a focused leaf rather than becoming tiny grid targets."""
        for button in ("btnBeaconOn", "btnLimitLeds", "btnSilent",
                       "btnHapticSettings"):
            self.assertRegex(
                BODY, rf"{button}\s*=\s*configTile\(pgSetPanes\[SET_FEEDBACK\]"
            )
        self.assertIn("(void *)(intptr_t)SET_HAPTIC", BODY)
        for button in ("btnHapticLevel", "btnHapticMounting", "btnHapticTest"):
            self.assertRegex(
                BODY, rf"{button}\s*=\s*configTile\(pgSetPanes\[SET_HAPTIC\]"
            )

    def test_haptic_is_persisted_enveloped_and_camera_mount_remains_hard_off(self):
        self.assertIn('uiPreferences.getUChar("hap_lvl", hapticLevelIdx)', BODY)
        self.assertIn('persistUiByte("hap_lvl", hapticLevelIdx)', BODY)
        self.assertIn('#include "haptic_envelope.h"', BODY)
        self.assertIn("osmo::HapticEnvelope hapticEnvelope", BODY)
        pump = BODY[BODY.index("static void pumpHaptic(uint32_t now) {"):]
        pump = pump[:pump.index("static void hapticGrab")]
        self.assertNotIn("delay(", pump)
        self.assertIn("cameraInHand || hapticLevelIdx == HAPTIC_OFF", pump)
        self.assertIn("hapticEnvelope.cancel()", pump)
        self.assertIn("hapticEnvelope.sample(now)", pump)
        self.assertIn("M5.Power.setVibration(target)", pump)
        self.assertIn("target != hapticApplied", pump)
        # The actual physical voltage/duration behavior is executed by
        # test_core2_native, not inferred from source constants.

        nav_start = BODY.index("static void navigateLeft()")
        nav = BODY[nav_start:BODY.index("static void softLeftCb", nav_start)]
        self.assertIn("setSub == SET_HAPTIC ? SET_FEEDBACK", nav)

    def test_one_card_component_owns_every_adjustment_surface(self):
        """A visual resemblance implemented by separate widgets drifts again.
        These representatives cover both shooting sheets, persistent settings,
        the motion editor, and the read-only System surface."""
        for button, parent in (
            ("btnFollowTemplate", "handFeelSheet"),
            ("btnHandAxisTune", "handFeelSheet"),
            ("btnJogSpeed", "jogFeelSheet"),
            ("btnJogPan", "jogFeelSheet"),
            ("btnBright", r"pgSetPanes\[SET_DISPLAY\]"),
            ("btnBeaconOn", r"pgSetPanes\[SET_FEEDBACK\]"),
            ("btnInvT", r"pgSetPanes\[SET_DIRECTION\]"),
            ("btnMotionPointTransition", "motionPointPane"),
        ):
            self.assertRegex(BODY, rf"{button}\s*=\s*configTile\({parent}")
        for title in ('"BATTERY"', '"UPTIME"', '"FIRMWARE"'):
            self.assertIn(title, BODY)

    def test_axis_tune_is_a_reachable_fixed_two_by_two_hand_subpage(self):
        """The four per-axis adjustments are a deliberate, thumb-sized
        subpage, not a seventh tiny control squeezed into HAND FEEL."""
        self.assertIn("handAxisTuneSheet = makeConfigPane(pgJog, true)", BODY)
        for button, title, geometry in (
            ("btnTiltResponse", '"TILT GAIN"', "6, 4, 150, 72"),
            ("btnPanResponse", '"PAN GAIN"', "164, 4, 150, 72"),
            ("btnTiltStability", '"TILT WEIGHT"', "6, 84, 150, 72"),
            ("btnPanStability", '"PAN WEIGHT"', "164, 84, 150, 72"),
        ):
            self.assertIn(f"{button} = configTile(handAxisTuneSheet", BODY)
            self.assertIn(title, BODY)
            self.assertIn(geometry, BODY)
        self.assertIn("btnHandAxisTune", BODY)
        self.assertIn("handAxisTunePressed", BODY)
        self.assertIn('AXIS_DAMPING_NAMES[] = {"STRONG", "MEDIUM", "LIGHT"}', BODY)
        self.assertIn("AXIS_STABILITY_WIRE_NAMES", BODY)

    def test_axis_tune_back_unwinds_before_the_hand_canvas(self):
        nav_start = BODY.index("static void navigateLeft()")
        nav = BODY[nav_start:BODY.index("static void softLeftCb", nav_start)]
        self.assertIn("HandFeelView::AxisTune", nav)
        self.assertIn("setHandFeelView(HandFeelView::Main)", nav)
        self.assertLess(nav.index("HandFeelView::AxisTune"),
                        nav.index("setHandFeelOpen(false)"))

    def test_header_context_obeys_the_same_nested_back_model(self):
        start = BODY.index("static void headerContextPressed")
        route = BODY[start:BODY.index("static void setMotionUiView", start)]
        self.assertIn("handFeelView == HandFeelView::AxisTune", route)
        self.assertIn("setHandFeelView(HandFeelView::Main)", route)

    def test_launcher_never_hides_a_custom_stability_setting(self):
        refresh = BODY[BODY.index("static void refreshUi()") :]
        summary = refresh[refresh.index("static char axisTuneSummary") :]
        summary = summary[:summary.index("lv_label_set_text(lblHandAxisTune")]
        self.assertIn("tiltStabilityIdx == 1 && panStabilityIdx == 1", summary)
        self.assertIn('"T/P CUSTOM"', summary)

    def test_config_cards_keep_icon_name_value_hierarchy(self):
        card = BODY[BODY.index("static lv_obj_t *configTile"):
                    BODY.index("static void setConfigTile")]
        self.assertIn("lv_obj_t *icon = label", card)
        self.assertIn("lv_obj_t *name = label", card)
        self.assertIn("lv_obj_t *detail = label", card)
        self.assertIn("const bool compact = h < 64", card)

    def test_unavailable_config_cards_cannot_receive_touch(self):
        setter = BODY[BODY.index("static void setConfigTile"):
                      BODY.index("static lv_obj_t *makeConfigPane")]
        self.assertRegex(
            setter,
            r"if \(available\)\s+lv_obj_add_flag\(tile, LV_OBJ_FLAG_CLICKABLE\);",
        )
        self.assertRegex(
            setter,
            r"else\s+lv_obj_clear_flag\(tile, LV_OBJ_FLAG_CLICKABLE\);",
        )

    def test_page_and_subview_changes_cancel_only_page_local_holds(self):
        start = BODY.index("static void cancelPageLocalHolds()")
        cancel = BODY[start:BODY.index(
            "static const uint32_t CLUTCH_RELEASE_DEBOUNCE_MS", start
        )]
        for state in (
            "clearHoldStartedMs = 0", "clearHoldTriggered = false",
            "resetHoldStartedMs = 0", "resetHoldTriggered = false",
            "centerHoldStartedMs = 0", "centerHoldTriggered = false",
            "homeHoldStartedMs = 0", "homeHoldTriggered = false",
            "motionHoldStartedMs = 0", "motionHoldTriggered = false",
            "motionHoldIsRun = false",
            "cameraForgetHoldStartedMs = 0",
            "cameraForgetTriggered = false",
            "powerOffHoldStartedMs = 0",
        ):
            self.assertIn(state, cancel)
        self.assertNotIn("recordHoldStartedMs", cancel)

        boundaries = (
            ("static void setJogFeelOpen(bool open) {",
             "static void setHandFeelOpen(bool open) {"),
            ("static void setHandFeelOpen(bool open) {",
             "static void headerContextPressed"),
            ("static void setMotionUiView(MotionUiView view) {",
             "static const char *motionTransitionName"),
            ("static void showSettings(uint8_t which) {",
             "static void showPage(uint8_t idx) {"),
            ("static void showPage(uint8_t idx) {",
             "static void showCameraView(CameraView which) {"),
            ("static void showCameraView(CameraView which) {",
             "static lv_obj_t *makeLimitBar"),
        )
        for begin, end in boundaries:
            section_start = BODY.index(begin)
            section = BODY[section_start:BODY.index(end, section_start + len(begin))]
            self.assertIn("cancelPageLocalHolds();", section, begin)

    def test_hand_and_jog_feel_use_six_large_cards_with_scrolling(self):
        for sheet in ("handFeelSheet", "jogFeelSheet"):
            self.assertIn(f"{sheet} = makeConfigPane", BODY)
        for name in (
            "btnFollowTemplate", "btnFollowSensitivity", "btnFollowSmooth",
            "btnHandAxisTune", "btnHandDirection", "btnHandCenter",
            "btnJogSpeed", "btnJogSmooth", "btnJogTilt", "btnJogPan",
            "btnJogDirection", "btnJogCenter",
        ):
            match = re.search(
                rf"{name}\s*=\s*configTile\([^;]*?,\s*(\d+),\s*(\d+),\s*"
                r"(\d+),\s*(\d+),", BODY, re.S,
            )
            self.assertIsNotNone(match, name)
            x, y, width, height = map(int, match.groups())
            self.assertIn(x, (6, 164), name)
            self.assertIn(y, (4, 84, 164), name)
            self.assertEqual((width, height), (150, 72), name)
            self.assertGreaterEqual(height, 40, name)
            self.assertLessEqual(y + height, 236, name)
        self.assertGreaterEqual(84 - (4 + 72), 8)
        self.assertGreaterEqual(164 - (84 + 72), 8)

    def test_motion_holds_are_cards_with_scroll_cancellation(self):
        for button in ("btnMotionPointTransition", "btnMotionPointTime",
                       "btnMotionPointDwell"):
            self.assertRegex(BODY, rf"{button}\s*=\s*configTile\(motionPointPane")
        for button in ("btnMotionPointGoto", "btnMotionPointClear"):
            self.assertRegex(
                BODY,
                rf"{button}\s*=\s*configTile\(motionPointPane,[\s\S]*?LV_EVENT_PRESSED",
            )
            for event in ("PRESSED", "PRESSING", "RELEASED", "PRESS_LOST"):
                self.assertRegex(
                    BODY,
                    rf"lv_obj_add_event_cb\({button},[\s\S]*?LV_EVENT_{event}",
                )
        self.assertIn("LV_EVENT_SCROLL_BEGIN", BODY)
        self.assertIn("cancelScrollHolds", BODY)
        hold = BODY[BODY.index("static void motionHoldEvent("):BODY.index("static void motionGotoEvent(")]
        self.assertIn("lv_indev_get_scroll_obj", hold)
        self.assertIn('"WAIT/GO"', BODY)
        self.assertNotIn('"WAIT FOR GO"', BODY)

    def test_the_names_table_matches_the_enum(self):
        names = re.search(r"SET_NAMES\[SET_COUNT\] = \{(.*?)\};", BODY, re.S)
        self.assertIsNotNone(names)
        enum = re.search(r"enum SetSub : uint8_t \{(.*?)\};", BODY, re.S)
        self.assertIsNotNone(enum)
        members = [m for m in re.findall(r"SET_[A-Z]+", enum.group(1))
                   if m != "SET_COUNT"]
        # Counted, not hardcoded: a literal here goes stale the moment a
        # sub-page is added or removed, and then guards nothing.
        self.assertEqual(len(re.findall(r'"', names.group(1))) // 2,
                         len(members),
                         "a sub-page has no name, or a name has no page")
        self.assertIn("SET_DIRECTION", BODY)

    def test_there_is_a_way_back_out_of_a_sub_page(self):
        """Without this the only escape is leaving the whole settings page."""
        self.assertIn("showSettings(SET_MENU)", BODY)
        self.assertIn('lv_line_create(softL)', BODY)
        self.assertIn('lv_obj_set_style_line_color(softReturnIcon, lv_color_hex(C_BG), 0)', BODY)
        self.assertIn('lv_line_set_points(softReturnIcon, RETURN_ICON_POINTS, 6)', BODY)
        self.assertIn('lv_obj_clear_flag(softReturnIcon, LV_OBJ_FLAG_CLICKABLE)', BODY)
        self.assertIn('lv_label_set_text(softLTxt, inSub ? "" : "<")', BODY)
        self.assertIn('if (inSub) lv_obj_clear_flag(softReturnIcon, LV_OBJ_FLAG_HIDDEN)', BODY)
        self.assertIn('else lv_obj_add_flag(softReturnIcon, LV_OBJ_FLAG_HIDDEN)', BODY)

    def test_leaving_settings_resets_to_the_menu(self):
        """Returning later to find yourself deep in a sub-page, with no memory
        of how you got there, is worse than losing your place."""
        self.assertIn("setSub = SET_MENU", BODY)


class TestLimitLeds(unittest.TestCase):
    """The side bars gained a second duty and must not lose the first."""

    def test_the_overlay_runs_after_the_beacon_has_painted(self):
        """Painting limits first would just be overwritten by the state colour."""
        fill = BODY.find("for (int i = 0; i < LED_COUNT; i++) leds[i] = c;")
        overlay = BODY.find("overlayLimitLeds();", fill)
        show = BODY.find("FastLED.show();", fill)
        self.assertGreater(fill, 0)
        self.assertLess(fill, overlay, "the beacon must paint first")
        self.assertLess(overlay, show, "the overlay must land before the show")

    def test_both_bars_are_addressed_separately(self):
        """One bar for each side is the entire reason this is worth doing."""
        self.assertIn("LED_LEFT_BASE", BODY)
        self.assertIn("LED_RIGHT_BASE", BODY)

    def test_all_four_directions_drive_something(self):
        for name in ("limUpLed", "limDownLed", "limLeftLed", "limRightLed"):
            self.assertIn(name, BODY, f"{name} is never used")

    def test_a_parked_axis_does_not_light_the_bars(self):
        """The panel caps a merely-parked axis at 35, so a floor below that
        would leave the bars permanently lit and worth ignoring."""
        floor = int(re.search(r"LIMIT_LED_FLOOR = (\d+)", BODY).group(1))
        self.assertGreaterEqual(floor, 25)
        self.assertLess(floor, 60, "so high a real warning never shows")

    def test_the_beacon_can_be_switched_off_without_losing_warnings(self):
        """They are separate duties and the settings page offers them
        separately, so the code has to honour that."""
        self.assertIn("beaconEnabled", BODY)
        self.assertIn("limitLedsEnabled", BODY)


class TestDrivePage(unittest.TestCase):
    def test_it_has_four_directional_headroom_edges(self):
        for name in ("driveLimitUp", "driveLimitDown", "driveLimitLeft",
                     "driveLimitRight"):
            self.assertIn(name, BODY)

    def test_the_edges_are_fed_from_canonical_limit_evidence(self):
        for name, value in (("driveLimitUp", "rig.limUp"),
                            ("driveLimitDown", "rig.limDown"),
                            ("driveLimitLeft", "rig.limLeft"),
                            ("driveLimitRight", "rig.limRight")):
            self.assertIn(f"setLimitBar({name},", BODY)
            self.assertIn(value, BODY)

    def test_speed_and_smoothing_are_independent_on_page(self):
        # Feel controls live on an explicit sheet; they no longer steal a
        # vertical rail from the actual one-thumb movement surface.
        # Each current value is a large card and cycles independently; dense
        # three-way button strips no longer create six competing targets.
        self.assertRegex(BODY, r"btnJogSpeed\s*=\s*configTile\(jogFeelSheet")
        self.assertRegex(BODY, r"btnJogSmooth\s*=\s*configTile\(jogFeelSheet")
        self.assertIn("jogSpeedPressed", BODY)
        self.assertIn("jogSmoothPressed", BODY)
        self.assertNotIn("btnJogSpeedChoice", BODY)
        self.assertNotIn("btnJogRampChoice", BODY)

    def test_hand_feel_is_an_explicit_sheet_not_competing_with_clutch(self):
        self.assertIn("handFeelSheet", BODY)
        self.assertRegex(BODY, r"btnFollowTemplate\s*=\s*configTile\(handFeelSheet")
        self.assertRegex(BODY, r"btnFollowSensitivity\s*=\s*configTile\(handFeelSheet")
        self.assertRegex(BODY, r"btnFollowSmooth\s*=\s*configTile\(handFeelSheet")


class TestPageGeometry(unittest.TestCase):
    """The screen is 240 px tall and the nav bar is nailed to the bottom of it.

    Settings shipped with five rows running to y=168 on a 152 px page, so the
    last item sat underneath the nav bar and could not be read or pressed. The
    arithmetic that prevents that is worth asserting, because it is invisible
    until someone photographs the box.
    """

    NAV_Y = 208          # where the soft-key row starts

    def _const(self, name):
        return int(re.search(rf"#define {name}\s+(\d+)", BODY).group(1))

    def test_the_uniform_layout_ends_exactly_at_the_nav_bar(self):
        top = self._const("HEADER_H")
        height = self._const("PAGE_H")
        self.assertEqual(top + height, self.NAV_Y,
                         "the page leaves a gap or runs under navigation")

    def test_every_page_uses_the_same_reserved_header(self):
        self.assertIn("lv_obj_set_pos(shown, 0, HEADER_H);", BODY)
        self.assertIn("lv_obj_set_size(shown, 320, PAGE_H);", BODY)
        self.assertNotIn("HEADER_H_TALL", BODY)
        self.assertNotIn("PAGE_H_SLIM", BODY)


class TestSafetyControlsStayReachable(unittest.TestCase):
    """STOP is a safety action, so changing pages must never hide it."""

    def test_stop_is_parented_to_the_screen_not_a_page(self):
        self.assertRegex(
            BODY,
            r"btnStop\s*=\s*keyBtn\(screenMain,\s*\"STOP\"",
            "STOP is hidden when its page parent is hidden",
        )

    def test_stop_is_created_after_page_content(self):
        """A root-level STOP created before later page siblings can still be
        painted underneath them and become unreachable."""
        stop = BODY.find('btnStop = keyBtn(screenMain, "STOP"')
        last_page_widget = BODY.find("btnHandDirection = configTile(handFeelSheet")
        self.assertGreaterEqual(last_page_widget, 0)
        self.assertGreater(stop, last_page_widget)


class TestReservedSafetyGeometry(unittest.TestCase):
    """The persistent STOP must be a header control, never an overlay on a page."""

    NAV_Y = 208

    @staticmethod
    def _key_geometry(name: str) -> tuple[int, int, int, int]:
        match = re.search(
            rf'{name}\s*=\s*(?:keyBtn\([^,]+,\s*[^,]+|'
            rf'configTile\([^,]+,\s*[^,]+,\s*[^,]+,\s*[^,]+),\s*'
            r'(\d+),\s*(\d+),\s*(\d+),\s*(\d+)', BODY,
        )
        assert match is not None, f"could not find geometry for {name}"
        return tuple(int(value) for value in match.groups())

    def test_stop_is_fully_inside_the_reserved_header(self):
        x, y, width, height = self._key_geometry("btnStop")
        header = int(re.search(r"#define HEADER_H\s+(\d+)", BODY).group(1))
        self.assertGreaterEqual(x, 0)
        self.assertGreaterEqual(y, 0)
        self.assertLessEqual(x + width, 320)
        self.assertLessEqual(y + height, header)
        self.assertGreaterEqual(height, 40)

    def test_page_controls_cannot_intersect_header_stop(self):
        sx, sy, sw, sh = self._key_geometry("btnStop")
        header = int(re.search(r"#define HEADER_H\s+(\d+)", BODY).group(1))
        for name in ("btnFollowTemplate",
                     "btnFollowSensitivity", "btnFollowSmooth",
                     "btnHandAxisTune", "btnHandDirection",
                     "btnTiltResponse", "btnPanResponse",
                     "btnTiltStability", "btnPanStability",
                     "btnInvT", "btnInvP", "btnBright",
                     "btnScreenTo", "btnBeaconOn", "btnLimitLeds", "btnSilent",
                     "btnHapticSettings", "btnHapticLevel", "btnHapticMounting",
                     "btnHapticTest", "btnResetFeel", "btnCameraScan", "btnCameraConnect",
                     "btnCameraForget", "btnCameraConfirmCancel",
                     "btnCameraConfirmAction", "btnCameraProgressBack"):
            x, y, width, height = self._key_geometry(name)
            # Only the icon-grid hubs may extend below the clipped viewport;
            # every leaf action remains a fixed, fully visible target.
            self.assertGreaterEqual(header + y, sy + sh, name)
            if header + y + height > self.NAV_Y:
                self.assertRegex(
                    BODY,
                    rf'{name}\s*=\s*(?:keyBtn\(pgSetPanes\[(SET_MENU|SET_CONTROLS)\]|configTile\((?:handFeelSheet|jogFeelSheet)),',
                    name,
                )
            else:
                self.assertLessEqual(header + y + height, self.NAV_Y, name)

    def test_fault_bar_is_wholly_in_the_header_not_an_action_overlay(self):
        # A global fault must remain visible without covering the first row of
        # JOG, MOVES or the device manager exactly when recovery is needed.
        m = re.search(
            r"lv_obj_set_size\(faultBar,\s*(\d+),\s*(\d+)\).*?"
            r"lv_obj_set_pos\(faultBar,\s*(\d+),\s*(\d+)\)",
            BODY, re.S,
        )
        self.assertIsNotNone(m)
        width, height, x, y = map(int, m.groups())
        header = int(re.search(r"#define HEADER_H\s+(\d+)", BODY).group(1))
        # Compact badge in the title cell: it must not consume the immutable
        # STOP cell or cover the canvas content surface.
        self.assertGreaterEqual(x, 78)
        self.assertGreaterEqual(y, 0)
        self.assertLessEqual(x + width, 262)
        self.assertLessEqual(y + height, header)

    def test_touch_only_actions_meet_the_one_hand_target(self):
        for name in ("btnStop", "btnRecord",
                     "btnFollowTemplate", "btnFollowSensitivity",
                     "btnFollowSmooth", "btnHandAxisTune", "btnHandDirection",
                     "btnTiltResponse", "btnPanResponse",
                     "btnTiltStability", "btnPanStability",
                     "btnInvT",
                     "btnInvP", "btnBright", "btnScreenTo", "btnBeaconOn",
                     "btnLimitLeds", "btnSilent", "btnHapticSettings",
                     "btnHapticLevel", "btnHapticMounting", "btnHapticTest", "btnWifiRetry",
                     "btnResetFeel",
                     "btnCameraScan", "btnCameraConnect", "btnCameraForget",
                     "btnCameraConfirmCancel", "btnCameraConfirmAction",
                     "btnCameraProgressBack"):
            _x, _y, width, height = self._key_geometry(name)
            self.assertGreaterEqual(min(width, height), 40, name)
        self.assertIn("6, 4, 150, 72", BODY, "settings grid cards must remain touchable")
        self.assertIn("164, 4, 150, 72", BODY, "settings grid needs its second column")
        for name in ("btnJogSpeed", "btnJogSmooth", "btnJogTilt", "btnJogPan",
                     "btnJogDirection", "btnJogCenter"):
            _x, _y, width, height = self._key_geometry(name)
            self.assertEqual((width, height), (150, 72), name)
        self.assertRegex(BODY, r'"RETRY",\s*163, 124, 153, 40')

    def test_record_and_stop_have_disjoint_header_targets(self):
        rx, ry, rw, rh = self._key_geometry("btnRecord")
        sx, sy, sw, sh = self._key_geometry("btnStop")
        self.assertLessEqual(rx + rw, sx)
        self.assertEqual((ry, rh), (sy, sh))


class TestEvidencePresentation(unittest.TestCase):
    def test_ready_requires_camera_and_fresh_telemetry(self):
        start = BODY.index("static UiPresentation reducePresentation()")
        reducer = BODY[start:BODY.index("static void refreshUi()", start)]
        self.assertIn('if (!rig.camera) return {"CAMERA?", C_AMBER};', reducer)
        self.assertIn('if (!rig.telemetry) return {"TELEMETRY", C_AMBER};', reducer)
        self.assertLess(reducer.index('if (!rig.telemetry)'),
                        reducer.index('return {"HOLD TO STEER", C_TEXT};'))

    def test_direct_phases_have_operator_words(self):
        start = BODY.index("static UiPresentation reducePresentation()")
        reducer = BODY[start:BODY.index("static void refreshUi()", start)]
        for word in ("START", "SCAN", "PAIR", "APPROVE", "WIFI", "LINK",
                     "TELEMETRY", "RETRY"):
            self.assertIn(f'"{word}"', reducer)

    def test_connection_detail_is_readable_on_the_wifi_page(self):
        self.assertIn("lblWifiDetail = label(cameraProgress", CODE)
        self.assertIn("LV_LABEL_LONG_DOT", BODY)
        self.assertIn("lv_label_set_text(stepLabels[i], stepRows[i]);", BODY)

    def test_camera_connection_screen_has_all_evidence_stages(self):
        """Joining the AP is only one hand-off, never a false camera-ready."""
        start = BODY.index("static void wifiProgress(")
        progress = BODY[start:BODY.index("static const char *safeWifiDetail", start)]
        for state in ("DISCOVER CAMERA", "PAIR CAMERA", "APPROVE / CREDS",
                      "JOIN 2.4 GHz", "OPEN CAMERA SESSION", "CAMERA LINK",
                      "WAIT TELEMETRY", "CAMERA READY", "BLOCKED - TAP RETRY"):
            self.assertIn(state, progress)
        self.assertNotIn("GET CAMERA IP", progress)
        self.assertNotIn("CAMERA IP FAILED", progress)
        for row in ("1  DISCOVER", "2  PAIR", "3  WI-FI", "4  SESSION"):
            self.assertIn(f'"{row}', BODY)
        self.assertIn("const bool completed[4]", BODY)
        self.assertIn("completed[i] ? C_GREEN", BODY)
        return  # superseded flat breadcrumb assertions below
        self.assertIn('"SCAN%s > PAIR%s > CREDS%s\\n2.4G%s', BODY)
        self.assertIn('IP%s > LINK%s > TELEM%s', BODY)

    def test_camera_connection_screen_keeps_credentials_off_screen(self):
        start = BODY.index("static const char *safeWifiDetail")
        guard = BODY[start:BODY.index("static void refreshUi()", start)]
        for secret_word in ('"pass"', '"credential"', '"key"'):
            self.assertIn(f"containsAsciiNoCase(detail, {secret_word})", guard)
        self.assertIn('"camera credentials received (hidden)"', guard)
        self.assertNotIn("lv_label_set_text(lblWifiDetail, rig.detail);", BODY)

    def test_direct_wifi_evidence_reaches_the_progress_screen_and_serial(self):
        start = BODY.index("static void pumpDirectState()")
        pump = BODY[start:BODY.index("void setup()", start)]
        for assignment in (
                "rig.directBlockedAt = st.blockedAt",
                "rig.wifiAttempt = st.wifiAttempt",
                "rig.wifiStatus = st.wifiStatus",
                "rig.wifiChannel = st.wifiChannel",
                "memcpy(rig.wifiBssid, st.wifiBssid",
                "rig.retryAtMs = st.retryAtMs",
                "strncpy(rig.wifiReason, st.wifiReason"):
            self.assertIn(assignment, pump)
        self.assertIn('say("D direct=%s block=%s try=%u/3', pump)
        self.assertIn('"Set camera Wi-Fi frequency to 2.4 GHz"', BODY)
        self.assertIn('rig.retryAtMs = st.retryAtMs', BODY)

    def test_direct_serial_census_is_change_driven_and_includes_telemetry(self):
        start = BODY.index("static void pumpDirectState()")
        pump = BODY[start:BODY.index("void setup()", start)]
        self.assertIn("!haveLastTelemetry", pump)
        self.assertIn("lastTelemetry != st.telemetry", pump)
        self.assertIn("lastTelemetry = st.telemetry", pump)
        self.assertIn('say("D direct=%s block=%s try=%u/3 wl=%u ch=%u telem=%u",', pump)
        self.assertIn('say("D rx peer=%lu foreign=%lu hdrbad=%lu win=%lu p3=%lu vid=%lu",', pump)
        self.assertIn('say("D frame valid=%lu att=%lu ok=%lu last=%u/%02X op=%02X/%02X plen=%u",', pump)
        self.assertIn('say("D ack v=%04X d=%04X x=%04X",', pump)
        self.assertIn('say("D att rx=', pump)

    def test_passive_camera_ready_does_not_use_motion_cyan(self):
        start = BODY.index("static void wifiProgress(")
        progress = BODY[start:BODY.index("static char asciiLower", start)]
        self.assertNotIn('"7/7 CAMERA READY"; colour = C_CYAN', progress)
        self.assertIn('"7/7 CAMERA READY"; colour = C_GREEN', progress)

    def test_camera_connection_screen_uses_real_evidence_not_a_countdown(self):
        self.assertNotIn("countdown", BODY[BODY.index("static void wifiProgress("):BODY.index("static void refreshUi()")])
        self.assertIn("DirectPhase::RetryWait", BODY)
        self.assertIn("RETRY NOW", BODY)

    def test_transport_and_owner_share_one_header_field(self):
        self.assertIn('rig.direct ? "DIRECT" : rig.linked ? "USB" : "OFFLINE"', BODY)
        self.assertIn('snprintf(ctrl, sizeof(ctrl), "%s / %s", transport, owner);', BODY)

    def test_host_speed_uses_slow_normal_fast(self):
        self.assertIn('!strcmp(controlName, "slow")', BODY)
        self.assertNotIn('!strcmp(controlName, "fine"))   speedIdx', BODY)

    def test_only_state_lines_claim_usb_ownership(self):
        start = BODY.index("static void handleLine")
        handler = BODY[start:BODY.index("static void pumpSerial", start)]
        claim = handler.index("usbHostSeen = true;")
        self.assertLess(handler.index("if (line[0] == 'S')"), claim)
        self.assertGreater(handler.index("lastRxMs = millis();"),
                           handler.index("if (line[0] == 'S')"))
        self.assertNotIn("case '?': hello();\n        usbHostSeen", handler)

    def test_motion_inputs_share_the_camera_evidence_gate(self):
        self.assertIn(
            "return rig.linked && rig.camera && rig.telemetry && !rig.fault[0];",
            BODY)
        jog = BODY[BODY.index("static void jogEvent"):BODY.index(
            "static lv_obj_t *label", BODY.index("static void jogEvent"))]
        buttons = BODY[BODY.index("static void pumpButtons"):BODY.index(
            "static void pumpImu", BODY.index("static void pumpButtons"))]
        self.assertIn("if (!controlEvidenceReady())", jog)
        self.assertIn("jogReleaseRequired = true", jog)
        self.assertIn("if (jogReleaseRequired)", jog)
        offline = jog[jog.index("if (!controlEvidenceReady())"):
                      jog.index("if (jogReleaseRequired)")]
        self.assertNotIn("setJog", offline,
                         "an offline held finger is not a release edge")
        self.assertIn("readyForMotion = controlEvidenceReady()", buttons)
        self.assertIn("clutchReleaseRequired = true", buttons)

    def test_direct_mode_clears_host_only_run_copy(self):
        start = BODY.index("static void pumpDirectState()")
        pump = BODY[start:BODY.index("void setup()", start)]
        for statement in ("rig.move[0] = '\\0';",
                          "rig.elapsed = rig.total = 0.0f;",
                          "rig.tlFrame = rig.tlFrames = 0;"):
            self.assertIn(statement, pump)

    def test_authority_and_evidence_loss_latch_jog_until_release(self):
        handler_start = BODY.index("static void handleLine")
        handler = BODY[handler_start:BODY.index("static void pumpSerial", handler_start)]
        direct_start = BODY.index("static void pumpDirectState()")
        direct = BODY[direct_start:BODY.index("void setup()", direct_start)]
        self.assertIn("jogReleaseRequired = true", handler)
        self.assertIn("hadControlEvidence && !controlEvidenceReady()", direct)
        self.assertIn("jogReleaseRequired = true", direct)

    def test_usb_state_updates_do_not_repeat_the_authority_boundary(self):
        start = BODY.index("static void handleLine")
        handler = BODY[start:BODY.index("static void pumpSerial", start)]
        self.assertIn(
            "const bool authorityChanged = line[0] == 'S' &&",
            handler)
        self.assertIn("if (authorityChanged)", handler)
        self.assertIn('if (wasJogActive) say("J 0 0");', handler)
        self.assertIn('say("E 0 reason=handoff");', handler)
        self.assertLess(handler.index("if (authorityChanged)"),
                        handler.index("osmo::directCamera.disable();"))

    def test_page_exit_explicitly_neutralises_live_controls(self):
        start = BODY.index("static void showPage(uint8_t idx)")
        show = BODY[start:BODY.index("static lv_obj_t *makeLimitBar", start)]
        self.assertIn("page == static_cast<uint8_t>(Workspace::Jog)", show)
        self.assertIn("nextPage != static_cast<uint8_t>(Workspace::Jog)", show)
        self.assertIn("osmo::directCamera.abort();", show)
        self.assertIn('say("J 0 0");', show)
        self.assertIn("page == static_cast<uint8_t>(Workspace::Hand)", show)
        self.assertIn("nextPage != static_cast<uint8_t>(Workspace::Hand)", show)
        self.assertIn("screenClutchHeld = false;", show)
        self.assertIn('say("E 0 reason=page");', show)

    def test_wake_key_is_consumed_until_all_hardware_keys_are_up(self):
        activity_start = BODY.index("static void noteScreenActivity(uint32_t now) {")
        activity = BODY[activity_start:BODY.index("static void pumpScreenTimeout", activity_start)]
        self.assertIn("swallowHardwareWake = M5.BtnA.isPressed()", activity)
        buttons_start = BODY.index("static void pumpButtons()")
        buttons = BODY[buttons_start:BODY.index("static void pumpImu", buttons_start)]
        gate = buttons.index("if (swallowHardwareWake)")
        self.assertLess(gate, buttons.index("M5.BtnA.wasPressed()"))
        self.assertIn("if (anyHardwareDown) return;", buttons)

    def test_screen_never_sleeps_while_stop_may_be_needed(self):
        start = BODY.index("static void pumpScreenTimeout(uint32_t now)")
        timeout = BODY[start:BODY.index("static void screenToPressed", start)]
        for state in ("rig.fault[0]", "rig.nearLimit", "rig.moving",
                      "rig.armed", "clutchHeld", "jogActive",
                      "rig.waitingCue >= 0",
                      # A tally that goes dark mid-take reads as a stopped
                      # recording, and the wake touch is swallowed.
                      "rig.recording", "rig.recordIntent"):
            self.assertIn(state, timeout)
        self.assertIn("if (!SCREEN_TIMEOUT_MS[screenTimeoutIdx] || safetyStateVisible)",
                      timeout)

    def test_clutch_feedback_requires_confirmed_core2_ownership(self):
        self.assertNotIn('!rig.direct || !strcmp(rig.owner, "core2")', BODY)
        buttons_start = BODY.index("static void pumpButtons()")
        buttons = BODY[buttons_start:BODY.index("static void pumpImu", buttons_start)]
        self.assertIn('!strcmp(rig.owner, "core2")', buttons)
        self.assertLess(buttons.index("const bool confirmed"),
                        buttons.index("hapticGrab();"))
        self.assertNotIn("if (held) hapticGrab();", buttons)


class TestNavigationSemantics(unittest.TestCase):
    def test_middle_key_and_bezel_share_home_or_toggle_control(self):
        start = BODY.index("static void updateSoftKeys() {")
        keys = BODY[start:BODY.index("static UiPresentation", start)]
        self.assertIn("lv_obj_clear_flag(softM, LV_OBJ_FLAG_HIDDEN)", keys)
        self.assertIn('onHand && !easeOpen && !handFeelOpen ? "JOG" : onJog && !easeOpen && !jogFeelOpen ? "CAMERA" : "HOME / HAND"', keys)
        buttons_start = BODY.index("static void pumpButtons()")
        buttons = BODY[buttons_start:BODY.index("static void pumpImu", buttons_start)]
        self.assertIn("if (M5.BtnB.wasPressed())", buttons)
        self.assertIn("homeOrToggleControl();", buttons)
        home = BODY[BODY.index("static void homeOrToggleControl() {"):]
        home = home[:home.index("static void softMidEvent", 0)]
        self.assertIn("showCameraView(CAM_TOOLS);", home)
        soft = BODY[BODY.index("static void softMidEvent"):]
        self.assertIn("homeOrToggleControl();", soft[:soft.index("static void layoutPage")])

    def test_clutch_raw_contact_is_central_touch_only(self):
        buttons = BODY[BODY.index("static void pumpButtons()"):
                       BODY.index("static void sampleImu", BODY.index("static void pumpButtons()"))]
        self.assertIn("const bool rawContact = (page == static_cast<uint8_t>(Workspace::Hand) &&", buttons)
        self.assertIn("!handFeelOpen && !easeOpen && screenClutchHeld);", buttons)
        self.assertNotRegex(buttons, r"rawContact\s*=.*BtnB", "BtnB must not acquire clutch")

    def test_running_motion_cannot_be_rearmed_by_the_run_surface(self):
        start = BODY.index("static void motionHoldEvent")
        motion = BODY[start:BODY.index("static void motionGotoEvent", start)]
        self.assertIn("!gate.motionProgramActive", motion)
        self.assertIn("if (status.motionProgramActive)", motion)
        self.assertLess(motion.index("if (status.motionProgramActive)"),
                        motion.index("requestMotionArm()"))

    def test_led_copy_does_not_claim_unsynchronised_brightness(self):
        self.assertNotIn("brightness follows the display setting", CODE)
        # The LED output has its own calibrated level; only the explicit
        # blackout state may dim it, not a display-brightness metaphor.
        self.assertIn("blackout ? 3 : ledBrightness", CODE)


class TestBootAndReconnectDoNotFreezeThePanel(unittest.TestCase):
    def test_setup_initialises_the_screen_timeout_deadline(self):
        setup = re.search(r"void setup\(\)\s*\{(.*?)\n\}", BODY, re.S)
        self.assertIsNotNone(setup)
        self.assertIn("noteScreenActivity(millis());", setup.group(1))

    def test_hello_only_reports_identity(self):
        """The host asks for hello on every serial reconnect. Recalibrating
        there blocks serial and IMU work for roughly three seconds."""
        hello = re.search(r"static void hello\(\)\s*\{(.*?)\n\}", BODY, re.S)
        self.assertIsNotNone(hello)
        self.assertNotIn("calibrateGyro", hello.group(1))

    def test_boot_has_no_blocking_one_pose_calibration(self):
        setup = re.search(r"void setup\(\)\s*\{(.*?)\n\}", BODY, re.S)
        self.assertIsNotNone(setup)
        self.assertNotIn("calibrateGyro", setup.group(1))
        self.assertIn("hostClaimDeadlineMs = millis() + 750", setup.group(1))


class TestSettingsScrolls(unittest.TestCase):
    def test_the_grids_scroll(self):
        """A further category remains reachable rather than forcing a new
        layout revision on the constrained display."""
        for pane in ("SET_MENU", "SET_CONTROLS"):
            self.assertIn(pane, BODY)
        self.assertIn("i == SET_MENU || i == SET_CONTROLS", BODY)
        self.assertIn("lv_obj_set_scroll_dir(pane, LV_DIR_VER)", BODY)

    def test_the_leaf_pages_do_not(self):
        """A settings pane that scrolls by accident hides half a control."""
        self.assertIn("lv_obj_clear_flag(pane, LV_OBJ_FLAG_SCROLLABLE)", BODY)

    def test_the_panes_are_sized_from_the_same_constant_as_the_page(self):
        """A pane taller than its page clips; shorter and it leaves a gap."""
        self.assertIn("lv_obj_set_size(pane, 320, PAGE_H)", BODY)

    def test_thumb_scroll_requires_intent_and_stops_on_lift(self):
        """Tiny tap wander must not launch a phone-like inertial scroll."""
        limit = re.search(r"indevDrv\.scroll_limit\s*=\s*(\d+)", BODY)
        throw = re.search(r"indevDrv\.scroll_throw\s*=\s*(\d+)", BODY)
        self.assertIsNotNone(limit)
        self.assertIsNotNone(throw)
        self.assertGreaterEqual(int(limit.group(1)), 18)
        self.assertEqual(int(throw.group(1)), 100)
        self.assertIn("LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM",
                      BODY)

    def test_touch_is_serviced_before_the_optional_repaint(self):
        loop = BODY[BODY.index("void loop() {"):]
        handler = loop.index("lv_timer_handler();")
        refresh = loop.index("refreshUi();", handler)
        self.assertLess(handler, refresh)
        self.assertIn("lv_timer_set_period(indevDrv.read_timer, 15)", BODY)


class TestResponsiveUiWorkLoop(unittest.TestCase):
    def test_haptics_never_sleep_the_lvgl_thread(self):
        tick = BODY[BODY.index("static void tick(uint16_t ms) {"):]
        tick = tick[:tick.index("static void hapticGrab")]
        limit = BODY[BODY.index("static void pumpLimitHaptic("):]
        limit = limit[:limit.index("static void hapticLimit")]
        self.assertNotIn("delay(", tick)
        self.assertNotIn("delay(", limit)
        self.assertIn("scheduleHaptic", tick)
        self.assertIn("scheduleHaptic", limit)
        self.assertIn("pumpHaptic(now);", BODY)

    def test_hidden_large_workspaces_are_not_repainted(self):
        refresh = BODY[BODY.index("static void refreshUi() {"):]
        refresh = refresh[:refresh.index("updatePageTitle();")]
        self.assertIn("if (onDevice)", refresh)
        self.assertIn("if (onMoves) refreshMotionWorkspace();", refresh)
        self.assertLess(refresh.index("if (onDevice)"),
                        refresh.index("refreshCameraHub();"))
        self.assertLess(refresh.index("if (onMoves)"),
                        refresh.index("refreshMotionWorkspace();"))

    def test_static_settings_use_event_refresh_and_a_slow_heartbeat(self):
        loop = BODY[BODY.index("void loop() {"):]
        self.assertIn("uiRefreshRequested || uiNow - lastUi >= uiInterval", loop)
        self.assertIn("Workspace::Settings) ? 500 : 100", loop)
        tick = BODY[BODY.index("static void tick(uint16_t ms) {"):]
        tick = tick[:tick.index("static void pumpHaptic")]
        self.assertIn("uiRefreshRequested = true", tick)

    def test_unchanged_label_text_is_not_invalidated(self):
        wrapper = CODE[CODE.index("static void setLabelTextIfChanged"):]
        wrapper = wrapper[:wrapper.index("// Forward declarations")]
        self.assertIn("lv_label_get_text(label)", wrapper)
        self.assertIn("std::strcmp(current, text) != 0", wrapper)


class TestDriveSurface(unittest.TestCase):
    def test_shooting_surfaces_prioritise_the_central_touch_target(self):
        for surface in ("jogPad", "dial"):
            self.assertIn(f"lv_obj_set_size({surface}, 168, PAGE_H);", BODY)
            self.assertIn(f"lv_obj_set_pos({surface}, 76, 0);", BODY)

    def test_the_pad_and_feel_controls_fit_the_slim_page(self):
        for name in ("btnJogTiltOnly", "btnJogPanOnly", "btnHandTiltOnly", "btnHandPanOnly"):
            match = re.search(
                rf'{name}\s*=\s*keyBtn\([^,]+,\s*[^,]+,\s*'
                r'(\d+),\s*(\d+),\s*(\d+),\s*(104)', BODY)
            self.assertIsNotNone(match, name)
            x, y, width, height = map(int, match.groups())
            self.assertEqual(y, 0)
            self.assertEqual((width, height), (64, 104))
            self.assertIn(x, (6, 250))
        self.assertIn("jogFeelSheet", BODY)
        self.assertIn("btnJogFeel", BODY)

    def test_jog_is_floating_origin_not_a_absolute_screen_joystick(self):
        start = BODY.index("static void jogEvent")
        jog = BODY[start:BODY.index("static lv_obj_t *label", start)]
        for name in ("jogOriginX", "jogOriginY"):
            self.assertIn(name, jog)
        self.assertIn("LV_EVENT_PRESSED", jog)
        self.assertIn("jogX = jogY = 0", jog)
        self.assertNotIn("PICKUP_DEADZONE", jog)
        self.assertIn("DirectCamera::jogResponse", jog)

    def test_arrows_are_40px_and_middle_is_always_a_navigation_target(self):
        self.assertIn("lv_obj_set_size(softL, 40, 32)", BODY)
        self.assertIn("lv_obj_set_size(softR, 40, 32)", BODY)
        self.assertIn("lv_obj_clear_flag(softM, LV_OBJ_FLAG_HIDDEN)", BODY)

class TestTimelapseOnTheBox(unittest.TestCase):
    def test_it_parses_the_frame_counters(self):
        self.assertIn('keyFloat(s, "tlf"', CODE)
        self.assertIn('keyFloat(s, "tln"', CODE)

    def test_the_struct_has_somewhere_to_put_them(self):
        """A field parsed into nothing is the shape of the boot loop that
        cost a flash cycle here before."""
        self.assertIn("int      tlFrame", CODE)
        self.assertIn("int      tlFrames", CODE)

    def test_a_running_timelapse_takes_the_page_name(self):
        self.assertIn("rig.tlFrames > 0", CODE)
        self.assertIn('"TL %d/%d"', CODE)

    def test_it_uses_the_commanded_motion_colour(self):
        """Cyan, not red -- red means the record tally and nothing else.

        Measured against the comment-stripped source: a window of raw text is
        mostly the comment explaining the choice, which pushed the code being
        checked out of range.
        """
        bare = strip_comments(CODE)
        title = bare[bare.index("static void updatePageTitle() {"):]
        idx = title.index("rig.tlFrames > 0")
        self.assertIn("C_CYAN", title[idx:idx + 300])

    def test_the_colour_is_put_back_when_it_ends(self):
        """Otherwise CONTROL stays cyan for the rest of the session and the
        colour stops meaning anything."""
        bare = strip_comments(CODE)
        title = bare[bare.index("static void updatePageTitle() {"):]
        idx = title.index("rig.tlFrames > 0")
        self.assertIn("C_TEXT", title[idx:idx + 800])

class TestRunCuesOnTheBox(unittest.TestCase):
    def test_it_listens_on_its_own_key(self):
        """NOT "cue": that key already carries waitingCue, a waypoint index.
        Sharing one key between an integer and a name makes both readings rest
        on a coincidence of formatting."""
        self.assertIn('keyStr(s, "rcue"', CODE)
        self.assertIn('rig.waitingCue = (int)keyFloat(s, "cue"', CODE)

    def test_the_countdown_is_scheduled_not_slept(self):
        """applyState runs on the serial reader. A countdown that slept
        between beeps would stall the link and the IMU stream for three
        seconds -- exactly as the head was about to move."""
        bare = strip_comments(CODE)
        idx = bare.index("static void scheduleRunCue")
        block = bare[idx:bare.index("static void pumpRunCue")]
        self.assertNotIn("delay(", block)
        self.assertIn("runCueAt = millis()", block)

    def test_the_pump_is_called_from_the_loop(self):
        self.assertIn("pumpRunCue(now)", strip_comments(CODE))

    def test_the_rollover_compare_is_signed(self):
        """millis() wraps after 49 days; an unsigned test would kill the cues
        for the rest of the shoot."""
        self.assertIn("(int32_t)(now - runCueAt) < 0", CODE)

    def test_every_tone_respects_the_silent_switch(self):
        bare = strip_comments(CODE)
        idx = bare.index("static void pumpRunCue")
        block = bare[idx:idx + 700]
        self.assertIn("if (soundEnabled) M5.Speaker.tone", block)

    def test_the_speaker_volume_follows_the_flag(self):
        """setVolume(0) at boot and nothing ever raised it: the SOUND ON
        toggle flipped a label against a permanently muted speaker."""
        self.assertIn("M5.Speaker.setVolume(on ? SPEAKER_VOLUME : 0)", CODE)
        bare = strip_comments(CODE)
        self.assertNotIn("soundEnabled = !soundEnabled", bare,
                         "the toggle bypasses the setter and mutes itself again")

    def test_an_unknown_cue_is_ignored_rather_than_guessed(self):
        bare = strip_comments(CODE)
        idx = bare.index("static void scheduleRunCue")
        self.assertIn("else return;", bare[idx:idx + 900])

class TestScreenTimeout(unittest.TestCase):
    """The box sits on a tripod for hours. The screen is the biggest draw on
    the battery and the brightest thing on a dark set."""

    def test_it_is_adjustable_from_the_display_page(self):
        self.assertIn("btnScreenTo = configTile(pgSetPanes[SET_DISPLAY]", CODE)
        self.assertIn("screenToPressed", CODE)

    def test_the_choices_include_never(self):
        """A timeout that cannot be turned off is one people work around by
        propping something on the screen."""
        self.assertIn('SCREEN_TIMEOUT_NAMES[] = {"NEVER"', CODE)

    def test_the_new_button_clears_the_brightness_one(self):
        """6..156 is taken; anything starting below 156 overlaps it."""
        m = re.search(r"btnScreenTo = configTile\([^;]*?(\d+), (\d+), (\d+), (\d+),",
                      CODE, re.S)
        self.assertIsNotNone(m, "could not read the button geometry")
        x, _y, w, _h = (int(g) for g in m.groups())
        self.assertGreaterEqual(x, 160, "overlaps the brightness button")
        self.assertLessEqual(x + w, 320, "runs off the pane")

    def test_the_waking_touch_is_swallowed(self):
        """Waking onto the clutch page and engaging the clutch because the
        thumb landed there is the failure that makes people switch a timeout
        off."""
        bare = strip_comments(CODE)
        idx = bare.index("static void touchCb")
        block = bare[idx:idx + 700]
        self.assertIn("if (swallowWake)", block)
        self.assertIn("return;", block)

    def test_the_swallow_is_cleared_on_finger_up(self):
        """Otherwise the first wake kills the touchscreen for good: the flag
        stays set, every later press is reported as a release, and the box
        looks bricked."""
        bare = strip_comments(CODE)
        idx = bare.index("static void touchCb")
        end = bare.index("}", bare.index("data->state = LV_INDEV_STATE_RELEASED",
                                         bare.index("} else {", idx)))
        released_branch = bare[bare.index("} else {", idx):end]
        self.assertIn("swallowWake = false", released_branch,
                      "the swallow is never cleared on finger-up")

    def test_one_owner_for_the_backlight(self):
        """Two writers fight and whichever ran last wins: the screen wakes to
        the wrong level or refuses to sleep at all. There is exactly one call
        in the file and it is inside applyBrightness()."""
        bare = strip_comments(CODE)
        self.assertEqual(bare.count("M5.Display.setBrightness("), 1,
                         "the backlight is written from more than one place")
        idx = bare.index("static void applyBrightness")
        self.assertIn("M5.Display.setBrightness(", bare[idx:idx + 220],
                      "the one writer is not applyBrightness()")

    def test_a_fault_or_a_limit_holds_it_awake(self):
        """Those are precisely the moments somebody glances over, and a dark
        screen then reads as a dead rig."""
        bare = strip_comments(CODE)
        idx = bare.index("static void pumpScreenTimeout")
        block = bare[idx:idx + 500]
        self.assertIn("rig.fault[0]", block)
        self.assertIn("rig.nearLimit", block)

    def test_the_rollover_compare_is_signed(self):
        self.assertIn("(int32_t)(now - screenOffAt) >= 0", CODE)

    def test_a_hardware_button_counts_as_activity(self):
        """Otherwise the screen sleeps while someone is driving the rig from
        the keys."""
        bare = strip_comments(CODE)
        idx = bare.index("pumpScreenTimeout(now)")
        self.assertIn("M5.BtnA.isPressed()", bare[idx:idx + 400])

    def test_the_pump_runs_in_the_loop(self):
        self.assertIn("pumpScreenTimeout(now)", strip_comments(CODE))
