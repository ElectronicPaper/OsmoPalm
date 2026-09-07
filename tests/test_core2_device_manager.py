"""Source contracts for the standalone Core2 camera device manager.

The BLE stack and LVGL are hardware libraries, so these checks pin the safety
and truth boundaries that are otherwise easy to regress during UI refactors.
"""

import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "firmware" / "core2_panel" / "src"
HEADER = (SRC / "direct_camera.h").read_text(encoding="utf-8")
DIRECT = (SRC / "direct_camera.cpp").read_text(encoding="utf-8")
REGISTRY_H = (SRC / "trusted_cameras.h").read_text(encoding="utf-8")
REGISTRY_CPP = (SRC / "trusted_cameras.cpp").read_text(encoding="utf-8")
MAIN = (SRC / "main.cpp").read_text(encoding="utf-8")


def strip_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    return re.sub(r"//[^\n]*", "", text)


def function(text: str, signature: str) -> str:
    start = text.index(signature)
    opening = text.index("{", start)
    depth = 0
    for at in range(opening, len(text)):
        if text[at] == "{":
            depth += 1
        elif text[at] == "}":
            depth -= 1
            if depth == 0:
                return text[start : at + 1]
    raise AssertionError(f"unterminated function {signature}")


class TestDeviceCatalogContract(unittest.TestCase):
    def test_catalog_is_bounded_versioned_and_truthful(self):
        catalog = HEADER[HEADER.index("struct CameraSummary"):
                         HEADER.index("enum class DirectPhase")]
        self.assertIn("CAMERA_CATALOG_CAPACITY = 6", HEADER)
        for field in ("nearby", "trusted", "selected", "current", "verified",
                      "name[24]", "address[18]", "version", "count",
                      "scanning", "actionPending", "forgetPending",
                      "error[40]"):
            self.assertIn(field, catalog)
        self.assertIn("items[CAMERA_CATALOG_CAPACITY]", catalog)

    def test_ui_only_publishes_operation_epochs(self):
        for method in ("requestDeviceScan", "cancelDeviceAction", "selectDevice",
                       "forgetDevice", "catalog"):
            self.assertIn(method, HEADER)
        for method in ("requestDeviceScan", "cancelDeviceAction", "selectDevice",
                       "forgetDevice"):
            body = function(DIRECT, f"void DirectCamera::{method}(")
            self.assertNotIn("NimBLEDevice::", body)
            self.assertNotIn("WiFi.", body)

    def test_selected_identity_is_enforced_before_gatt_probe(self):
        provision = function(DIRECT, "bool provisionOverBle(")
        address_gate = provision.index("selected.requestedAddressType")
        connect = provision.index("candidate->connect(&device)")
        service = provision.index("candidate->getService(SERVICE_UUID)")
        self.assertLess(address_gate, connect)
        self.assertLess(connect, service)
        self.assertIn("never connect first compatible peer", provision)

    def test_scan_and_cancel_are_consumed_as_real_management_operations(self):
        task = function(DIRECT, "void DirectCamera::taskLoop()")
        scan = function(DIRECT, "void DirectCamera::requestDeviceScan()")
        cancel = function(DIRECT, "void DirectCamera::cancelDeviceAction()")
        select = function(DIRECT, "void DirectCamera::selectDevice(")
        self.assertIn("inputs_.deviceMode = DeviceMode::ScanOnly", scan)
        self.assertIn("inputs_.deviceMode = DeviceMode::ConnectSelected", select)
        self.assertIn("inputs_.deviceMode = DeviceMode::Idle", cancel)
        self.assertIn("launch.deviceMode == DeviceMode::Idle", task)
        self.assertIn("inputs().deviceMode == DeviceMode::ScanOnly", task)
        # Eight-second uninterruptible discovery is not acceptable on a
        # handheld controller. One-second slices bound cancellation latency.
        self.assertIn("scan->start(1", DIRECT)
        self.assertIn("BLE_SCAN_SLICE_TIMEOUT_MS", DIRECT)
        self.assertIn("millis() - sliceStartedAt < BLE_SCAN_SLICE_TIMEOUT_MS",
                      DIRECT)
        self.assertIn("actionPending = false", task)
        self.assertIn("scanning = false", task)

    def test_scan_requested_during_an_active_link_is_not_swallowed(self):
        task = function(DIRECT, "void DirectCamera::taskLoop()")
        completed_scan = task[task.index(
            "inputs().deviceMode == DeviceMode::ScanOnly"):]
        self.assertIn("inputs().reconnectEpoch == attemptReconnectEpoch",
                      completed_scan[:300])
        self.assertIn("belongs to the *next* runtime", completed_scan)

    def test_ap_frequency_remediation_preserves_pending_operation_and_evidence(self):
        task = function(DIRECT, "void DirectCamera::taskLoop()")
        manual = function(DIRECT, "if (runtimeNeedsManualWifiRetry)")
        self.assertIn("DirectStatus retry = runtimeStatus", manual)
        self.assertIn("retry.retryAtMs = 0", manual)
        self.assertNotIn("catalog_.actionPending = false", manual)
        idle = task[task.index("const bool awaitingExplicitRetry"):]
        self.assertIn("prior.phase == DirectPhase::RetryWait", idle[:300])
        self.assertIn("prior.retryAtMs == 0", idle[:300])

    def test_cancel_and_successful_scan_do_not_publish_false_errors(self):
        cancel = function(DIRECT, "void DirectCamera::cancelDeviceAction()")
        provision = function(DIRECT, "bool provisionOverBle(")
        scan_complete = provision[provision.index(
            "selected.deviceMode == DirectCamera::DeviceMode::ScanOnly"):]
        self.assertIn("catalog_.error[0] = '\\0'", cancel)
        self.assertNotIn("no compatible camera nearby", scan_complete)

    def test_scan_start_failure_is_not_reported_as_no_devices(self):
        provision = function(DIRECT, "bool provisionOverBle(")
        self.assertIn("const bool started = scan->start(1, nullptr", provision)
        self.assertIn("if (!started)", provision)
        self.assertIn('failIfWanted("BLE scan start failed")', provision)

    def test_stale_or_lost_gap_scan_cannot_wedge_reconnect(self):
        provision = function(DIRECT, "bool provisionOverBle(")
        close_ble = function(DIRECT, "void closeBle()")
        stop = function(DIRECT, "static bool stopBleScan(")
        self.assertIn("BLE_SCAN_STOP_TIMEOUT_MS", stop)
        self.assertIn("scan->stop()", stop)
        self.assertIn("return !scan->isScanning()", stop)
        self.assertLess(provision.index("stopBleScan(scan)"),
                        provision.index("scan->start(1"))
        self.assertIn('failIfWanted("BLE scan reset failed")', provision)
        self.assertIn('"BLE scan timed out"', provision)
        self.assertIn('"BLE scan stop failed"', provision)
        self.assertIn("stopBleScan(scan)", close_ble)

    def test_cancel_cannot_race_into_a_new_connection_runtime(self):
        task = function(DIRECT, "void DirectCamera::taskLoop()")
        persist = task.index("trustedCameras.setSelected")
        launch = task.index("const Inputs launch = inputs()", persist)
        attempt = task.index("attemptReconnectEpoch = launch.reconnectEpoch",
                             launch)
        runtime = task.index("DirectRuntime runtime(*this, attemptReconnectEpoch)",
                             attempt)
        self.assertLess(persist, launch)
        self.assertLess(launch, attempt)
        self.assertLess(attempt, runtime)
        constructor = function(DIRECT, "explicit DirectRuntime(")
        self.assertIn("expectedReconnectEpoch", constructor)
        self.assertIn("if (!wanted()) return", constructor)


class TestTrustedCameraBoundary(unittest.TestCase):
    def test_registry_contains_identity_only(self):
        trusted = REGISTRY_H[REGISTRY_H.index("struct TrustedCamera"):
                             REGISTRY_H.index("class TrustedCameras")]
        for forbidden in ("ssid", "password", "passphrase", "credential",
                          "manufacturerData"):
            self.assertNotIn(forbidden.lower(), strip_comments(trusted).lower())
        self.assertNotIn("Preferences", DIRECT)
        self.assertIn("Preferences", REGISTRY_CPP)

    def test_trust_is_written_only_after_authorization_and_credentials(self):
        provision = function(DIRECT, "bool provisionOverBle(")
        pair = provision.index("pairingStatus")
        password = provision.index("passwordUnpacked")
        persist = provision.index("trustedCameras.upsert")
        self.assertLess(pair, password)
        self.assertLess(password, persist)
        self.assertIn("secureZero(reply.payload", provision)

    def test_registry_mutations_roll_back_if_nvs_save_fails(self):
        upsert = function(REGISTRY_CPP, "bool TrustedCameras::upsert(")
        erase = function(REGISTRY_CPP, "bool TrustedCameras::erase(")
        selected = function(REGISTRY_CPP, "bool TrustedCameras::setSelected(")
        for body in (upsert, erase, selected):
            self.assertIn("previousSelected", body)
            self.assertIn("if (save()) return true", body)
            self.assertIn("selected_ = previousSelected", body)
        self.assertIn("cameras_[slot] = previous", upsert)
        self.assertIn("cameras_[i] = previous", erase)

    def test_saved_selection_is_persisted_before_connection_attempt(self):
        task = function(DIRECT, "void DirectCamera::taskLoop()")
        persist = task.index("trustedCameras.setSelected")
        runtime = task.index("DirectRuntime runtime")
        self.assertLess(persist, runtime)
        self.assertIn("camera selection save failed", task)

    def test_saved_cameras_are_visible_immediately_after_boot(self):
        begin = function(DIRECT, "void DirectCamera::begin()")
        self.assertIn("trustedCameras.get", begin)
        self.assertIn("persistedCatalog.items", begin)
        self.assertIn("item.trusted = true", begin)
        self.assertIn("catalog_ = persistedCatalog", begin)

    def test_loaded_registry_identity_is_structurally_validated(self):
        begin = function(REGISTRY_CPP, "void TrustedCameras::begin()")
        validation = function(REGISTRY_CPP, "bool validRegistry(")
        address = function(REGISTRY_CPP, "bool validAddress(")
        self.assertIn("validRegistry(stored)", begin)
        self.assertIn("camera.addressType > 3", validation)
        self.assertIn("std::memchr(camera.name", validation)
        self.assertIn("sameIdentity(stored.cameras[prior]", validation)
        self.assertIn("std::strlen(address) != 17", address)
        self.assertIn("std::isxdigit", address)

    def test_current_requires_fresh_camera_telemetry(self):
        ready = function(DIRECT, "bool readyLoop()")
        fresh = ready.index("const bool telemetryFresh")
        publish = ready.index("catalog.items[i].current", fresh)
        self.assertLess(fresh, publish)
        self.assertIn("telemetryFresh && !catalogCurrentPublished_", ready)
        self.assertIn("!telemetryFresh && catalogCurrentPublished_", ready)
        self.assertIn("catalogCurrentPublished_ = false", ready)

    def test_forget_is_exact_local_and_never_delete_all(self):
        request = function(DIRECT, "void DirectCamera::forgetDevice(")
        task = function(DIRECT, "void DirectCamera::taskLoop()")
        self.assertIn("forgetAddressType = catalogAddressTypes_[i]", request)
        self.assertIn("forgetAddress, catalog_.items[i].address", request)
        self.assertNotIn("inputs_.requestedAddress, catalog_.items[i].address", request)
        self.assertIn("trustedCameras.erase", task)
        self.assertIn("NimBLEDevice::deleteBond(address)", task)
        self.assertNotIn("deleteAllBonds", DIRECT)
        self.assertIn("no proven remote-unpair opcode", task)
        erase = task.index("const bool erased = trustedCameras.erase")
        failure = task.index("if (!erased)", erase)
        bond = task.index('NimBLEDevice::init("Osmo Core2")', erase)
        remove = task.index("--catalog_.count", erase)
        self.assertLess(erase, failure)
        self.assertLess(failure, bond)
        self.assertLess(bond, remove)
        self.assertIn("camera forget save failed", task[failure:bond])

    def test_stop_fault_aborts_manager_actions_without_changing_selection(self):
        task = function(DIRECT, "void DirectCamera::taskLoop()")
        fault = task[task.index("if (runtimeControlFault)"):]
        self.assertIn("seenForgetEpoch = failed.deviceForgetEpoch", fault)
        self.assertIn("seenSelectEpoch = failed.deviceSelectEpoch", fault)
        self.assertIn("inputs_.forgetAddress[0] = '\\0'", fault)
        self.assertIn("trustedCameras.selected(retained)", fault)
        self.assertIn("fault.controlFault = true", fault)


class TestSmallScreenDeviceManager(unittest.TestCase):
    def test_camera_hub_has_scroll_list_and_fixed_thumb_actions(self):
        build = function(MAIN, "static void buildUi()")
        # Cameras are the DEVICE workspace. The settings enum used to carry a
        # SET_CAMERAS pane that was allocated on every boot and that no menu
        # row could reach -- dead weight on a UI that had already exhausted
        # the LVGL object arena once and boot-looped for it.
        self.assertNotIn("SET_CAMERAS", MAIN)
        # The canvas puts connection evidence on DEVICE home and moves the
        # full scan/switch/forget flow behind one large CAMERAS target.
        self.assertIn('"DEVICE", "CAMERAS", "PAIR CAMERA"', MAIN)
        self.assertIn('"BLE", "WI-FI", "SOCKETS", "TELEMETRY", "TALLY"', MAIN)
        self.assertIn("lv_obj_set_scroll_dir(cameraList, LV_DIR_VER)", build)
        self.assertIn("lv_obj_clear_flag(cameraHub, LV_OBJ_FLAG_SCROLLABLE)", build)
        for geometry in ('"SCAN", 4, 116, 100, 48',
                         '"CONNECT", 108, 116, 100, 48',
                         '"FORGET", 212, 116, 104, 48'):
            self.assertIn(geometry, strip_comments(build))
        self.assertIn("i * 46, 308, 44", build)

    def test_scan_switch_and_forget_have_safe_confirmation(self):
        confirm = function(MAIN, "static void refreshCameraConfirm()")
        event = function(MAIN, "static void cameraConfirmActionEvent(")
        for text in ("SCAN FOR CAMERAS?", "SWITCH CAMERA?",
                     "FORGET FROM CORE2?", "Motion stops"):
            self.assertIn(text, confirm)
        self.assertIn("Camera may retain approval", confirm)
        self.assertIn("Link/motion stop", confirm)
        self.assertIn("< HOLD_DESTROY_MS", event)
        self.assertIn("forgetDevice(cameraConfirmId)", event)

    def test_back_leaves_subflow_visible_without_cancelling_the_action(self):
        back = function(MAIN, "static void navigateLeft()")
        self.assertIn("Workspace::Device", back)
        self.assertIn("cameraView != CAM_DEVICE_HOME", back)
        self.assertNotIn("cancelDeviceAction()", back)
        self.assertIn("showCameraView(CAM_DEVICE_HOME)", back)
        explicit_cancel = function(MAIN, "static void wifiReconnectPressed(")
        self.assertIn("cancelDeviceAction()", explicit_cancel)
        buttons = function(MAIN, "static void pumpButtons(")
        self.assertIn("navigateLeft()", buttons)

    def test_manager_uses_semantic_wait_fault_and_neutral_colours(self):
        hub = function(MAIN, "static void refreshCameraHub()")
        confirm = function(MAIN, "static void refreshCameraConfirm()")
        self.assertNotIn("C_CYAN", hub + confirm)
        self.assertIn("C_AMBER", hub + confirm)
        self.assertIn("C_MAGENTA", MAIN)

    def test_empty_scan_and_scan_failure_are_distinct(self):
        hub = function(MAIN, "static void refreshCameraHub()")
        self.assertIn("NO CAMERAS NEARBY", hub)
        self.assertIn("CAMERA ACTION FAILED", hub)
        self.assertIn("SCANNING NEARBY", hub)

    def test_unavailable_forget_is_not_only_visually_disabled(self):
        pressed = function(MAIN, "static void cameraForgetPressed(")
        self.assertIn("cameraCatalogUi.scanning", pressed)
        self.assertIn("cameraCatalogUi.actionPending", pressed)

    def test_committed_forget_is_truthfully_non_cancellable(self):
        cancel = function(DIRECT, "void DirectCamera::cancelDeviceAction()")
        forget = function(DIRECT, "void DirectCamera::forgetDevice(")
        scan_press = function(MAIN, "static void cameraScanPressed(")
        hub = function(MAIN, "static void refreshCameraHub()")
        self.assertIn("catalog_.forgetPending = true", forget)
        self.assertIn("if (catalog_.forgetPending)", cancel)
        self.assertIn("if (cameraCatalogUi.forgetPending) return", scan_press)
        self.assertIn('"SAVING"', hub)
        for method in ("requestDeviceScan", "selectDevice", "forgetDevice",
                       "requestReconnect"):
            body = function(DIRECT, f"void DirectCamera::{method}(")
            self.assertIn("if (catalog_.forgetPending)", body)

    def test_same_name_cameras_have_a_visible_identity_suffix(self):
        hub = function(MAIN, "static void refreshCameraHub()")
        self.assertIn("addressTail", hub)
        self.assertIn("item.address + addressLen - 5", hub)


if __name__ == "__main__":
    unittest.main()
