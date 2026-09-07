"""Offline contract checks for the host-free Core2 camera path.

The target C++ is also compiled by the delivery gate.  These tests pin the
wire constants and safety ordering that can compile perfectly while still
silently failing (or resuming motion) on a camera.
"""

import re
import unittest
from pathlib import Path

import json


ROOT = Path(__file__).resolve().parents[1]
WIRE = json.loads((ROOT / "contracts/camera-control-v1.json").read_text(encoding="utf-8"))
DIRECT_PATH = ROOT / "firmware" / "core2_panel" / "src" / "direct_camera.cpp"
HEADER_PATH = ROOT / "firmware" / "core2_panel" / "src" / "direct_camera.h"
MAIN_PATH = ROOT / "firmware" / "core2_panel" / "src" / "main.cpp"
INI_PATH = ROOT / "firmware" / "core2_panel" / "platformio.ini"

DIRECT = DIRECT_PATH.read_text(encoding="utf-8") if DIRECT_PATH.exists() else ""
DIRECT += (DIRECT_PATH.parent / "control_math.h").read_text(encoding="utf-8")
HEADER = HEADER_PATH.read_text(encoding="utf-8") if HEADER_PATH.exists() else ""
MAIN = MAIN_PATH.read_text(encoding="utf-8") if MAIN_PATH.exists() else ""
INI = INI_PATH.read_text(encoding="utf-8") if INI_PATH.exists() else ""


def setUpModule():
    missing = [str(p) for p in (DIRECT_PATH, HEADER_PATH, MAIN_PATH, INI_PATH) if not p.exists()]
    if missing:
        raise unittest.SkipTest("standalone firmware source missing: " + ", ".join(missing))


def _function(text: str, signature: str) -> str:
    start = text.index(signature)
    brace = text.index("{", start)
    depth = 0
    for i in range(brace, len(text)):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                return text[start:i + 1]
    raise AssertionError(f"unterminated function {signature}")


def _numeric_array(name: str) -> bytes:
    match = re.search(
        rf"(?:const\s+)?uint8_t\s+{re.escape(name)}\s*\[[^]]*\]\s*=\s*\{{(.*?)\}};",
        DIRECT, re.S)
    if not match:
        raise AssertionError(f"array {name} not found")
    values = [int(token, 0) for token in re.findall(r"0x[0-9A-Fa-f]+|\b\d+\b",
                                                    match.group(1))]
    return bytes(values)


class TestCanonicalConnectionSpine(unittest.TestCase):
    def test_ble_identity_and_service_match_the_wire_contract(self):
        constants = dict(re.findall(r'constexpr char (\w+)\[\] = "([^"]*)";', DIRECT))
        self.assertEqual(constants["SERVICE_UUID"], WIRE['service_uuid'])
        self.assertEqual(constants["NOTIFY_UUID"], WIRE['notify_uuid'])
        self.assertEqual(constants["WRITE_UUID"], WIRE['write_uuid'])
        self.assertEqual(constants["APP_IDENTIFIER"], WIRE['app_identifier'])
        self.assertEqual(constants["PAIRING_PIN"], WIRE['pairing_pin'])

    def test_network_endpoint_matches_the_wire_contract(self):
        self.assertIn(f'CAMERA_HOST[] = "{WIRE["camera_host"]}"', DIRECT)
        self.assertRegex(DIRECT, rf"CAMERA_TCP_PORT\s*=\s*{WIRE['tcp_port']}\b")
        self.assertRegex(DIRECT, rf"CAMERA_UDP_PORT\s*=\s*{WIRE['udp_port']}\b")

    def test_handshake_template_is_byte_identical(self):
        self.assertEqual(_numeric_array("handshake"),
                         bytes.fromhex(WIRE['handshake']))
        self.assertIn("putLe16(handshake, baseSeq_);", DIRECT)

    def test_subscription_names_and_first_id_match(self):
        block = DIRECT[DIRECT.index("SUBSCRIPTION_KEYS"):]
        block = block[:block.index("};")]
        names = re.findall(r'"([^"]+)"', block)
        self.assertEqual(names, WIRE['subscription_keys'])
        self.assertRegex(DIRECT, rf"FIRST_SUB_ID\s*=\s*0x{WIRE['first_sub_id']:X}\b")

    def test_ble_finishes_before_wifi_and_credentials_are_ram_only(self):
        run = _function(DIRECT, "bool run()")
        provision = _function(DIRECT, "bool provisionOverBle(")
        self.assertLess(run.index("provisionOverBle"), run.index("joinWifi"))
        self.assertLess(run.index("joinWifi"), run.index("openDatalink"))
        self.assertLess(provision.rindex("closeBle()"), provision.rindex("return true"))
        self.assertIn("WiFi.persistent(false)", DIRECT)
        self.assertIn("WIFI_STORAGE_RAM", DIRECT)
        self.assertEqual(run.count("secureZero(password"), 2)
        first_scrub = run.index("secureZero(password")
        self.assertLess(first_scrub, run.index("return false", first_scrub))
        joined = run.index("const bool joined")
        second_scrub = run.index("secureZero(password", joined)
        self.assertLess(second_scrub, run.index("if (!joined)", second_scrub))
        self.assertNotIn("Preferences", DIRECT)

    def test_ble_discovery_matches_name_company_model_or_exact_service(self):
        provision = _function(DIRECT, "bool provisionOverBle(")
        names = _function(DIRECT, "bool containsCameraName(")
        manufacturer = _function(DIRECT, "bool isPocketManufacturerData(")
        self.assertIn("std::tolower", names)
        for value in ("0x08AA", "0xF7AA", "0x20", "0x21", "0x22"):
            self.assertIn(value, manufacturer)
        self.assertIn("hasPocketManufacturerData(device)", provision)
        self.assertIn("device.isAdvertisingService(serviceUuid)", provision)

    def test_each_ble_candidate_proves_gatt_and_capabilities_before_selection(self):
        provision = _function(DIRECT, "bool provisionOverBle(")
        selected = provision.index("client_ = candidate")
        for required in (
                "candidate->getService(SERVICE_UUID)",
                "service->getCharacteristic(NOTIFY_UUID)",
                "service->getCharacteristic(WRITE_UUID)",
                "candidate->getMTU() < BLE_REQUIRED_MTU",
                "notify->canNotify()", "notify->canWrite()",
                "writer->canWriteNoResponse()"):
            self.assertLess(provision.index(required), selected)
        self.assertGreaterEqual(provision.count("deleteClient(candidate)"), 4)

    def test_transient_gatt_drop_reprobes_only_the_selected_camera(self):
        provision = _function(DIRECT, "bool provisionOverBle(")
        self.assertRegex(DIRECT, r"BLE_LINK_ATTEMPTS\s*=\s*4\b")
        self.assertRegex(DIRECT, r"BLE_LINK_RETRY_BASE_MS\s*=\s*750\b")

        address_gate = provision.index("address.getType() != selected.requestedAddressType")
        retry_loop = provision.index("attempt < BLE_LINK_ATTEMPTS", address_gate)
        selected_client = provision.index("client_ = candidate", retry_loop)
        self.assertLess(address_gate, retry_loop)
        self.assertLess(retry_loop, selected_client)
        self.assertIn("candidate->connect(&device)", provision[retry_loop:selected_client])
        self.assertIn("candidate->connect(address)", provision[retry_loop:selected_client])
        self.assertIn("camera BLE busy; retrying", provision[retry_loop:selected_client])
        self.assertIn("candidate->getLastError()", provision[retry_loop:selected_client])
        self.assertIn("!candidate->isConnected()", provision[retry_loop:selected_client])
        for transient in ("BLE_HS_ENOTCONN", "BLE_HS_EAGAIN", "BLE_HS_ETIMEOUT"):
            self.assertIn(transient, provision[retry_loop:selected_client])
        self.assertIn("NimBLEDevice::deleteClient(candidate)",
                      provision[retry_loop:selected_client])
        self.assertIn("waitWhileWanted(backoff)",
                      provision[retry_loop:selected_client])
        self.assertIn("the selected identity is singular; never try another peer",
                      provision)
        self.assertIn("camera BLE dropped during GATT", provision)
        self.assertNotIn("deleteBond", provision)

    def test_gatt_probe_is_presented_as_pair_not_scan(self):
        provision = _function(DIRECT, "bool provisionOverBle(")
        retry_loop = provision.index("attempt < BLE_LINK_ATTEMPTS")
        service = provision.index("candidate->getService(SERVICE_UUID)", retry_loop)
        pair_stage = provision.index("setPhase(DirectPhase::BlePair, detail)",
                                     retry_loop)
        self.assertLess(pair_stage, service)

    def test_ble_mtu_gate_matches_the_conservative_probe(self):
        provision = _function(DIRECT, "bool provisionOverBle(")
        self.assertIn("NimBLEDevice::setMTU(255)", provision)
        self.assertRegex(DIRECT, r"BLE_REQUIRED_MTU\s*=\s*54\b")
        self.assertIn("camera BLE MTU below 54", provision)

    def test_operator_failures_name_ownership_and_core2_radio_constraint(self):
        provision = _function(DIRECT, "bool provisionOverBle(")
        join = _function(DIRECT, "bool joinWifi(")
        self.assertIn("close Mimo or other camera controller", provision)
        self.assertIn("camera AP not visible; use 2.4GHz", join)

    def test_wifi_provisions_once_then_retries_observed_ap_identity(self):
        run = _function(DIRECT, "bool run()")
        join = _function(DIRECT, "bool joinWifi(")
        self.assertEqual(run.count("provisionOverBle("), 1)
        self.assertLess(run.index("provisionOverBle"), run.index("joinWifi"))
        self.assertNotIn("WIFI_JOIN_ATTEMPTS", run)
        self.assertNotIn('"re-waking AP"', run)
        self.assertIn("WIFI_JOIN_ATTEMPTS", join)
        self.assertIn("WiFi.scanNetworks(\n                true, true, false, 120, 0, nullptr, nullptr)", join)
        self.assertIn("WiFi.scanComplete()", join)
        self.assertIn("WIFI_SCAN_RUNNING", join)
        self.assertIn("WiFi.SSID(i) != ssid", join)
        self.assertIn("WiFi.BSSID(i)", join)
        self.assertIn("WiFi.channel(i)", join)
        self.assertIn("WiFi.begin(ssid, password, channel, bssid, true)", join)

    def test_sleepy_camera_ap_gets_the_host_visibility_window(self):
        match = re.search(r"WIFI_SCAN_WINDOW_MS\s*=\s*(\d+)", DIRECT)
        self.assertIsNotNone(match)
        self.assertGreaterEqual(int(match.group(1)), 25_000)
        join = _function(DIRECT, "bool joinWifi(")
        self.assertIn("while (wanted() && millis() - scanStarted < WIFI_SCAN_WINDOW_MS)", join)
        self.assertIn('"waiting for AP"', join)

    def test_wifi_join_is_bounded_observable_and_never_reports_the_key(self):
        join = _function(DIRECT, "bool joinWifi(")
        status = HEADER[HEADER.index("struct DirectStatus"):HEADER.index("class DirectCamera")]
        for field in ("wifiAttempt", "wifiStatus", "wifiChannel", "wifiBssid",
                      "wifiReason", "blockedAt", "retryAtMs"):
            self.assertIn(field, status)
        self.assertIn("WIFI_SCAN_WINDOW_MS", join)
        self.assertIn("WIFI_ASSOCIATE_WINDOW_MS", join)
        self.assertIn("WL_CONNECT_FAILED", join)
        self.assertIn("WL_NO_SSID_AVAIL", join)
        self.assertIn("setWifiEvidence", join)
        self.assertIn("const wl_status_t beginStatus", join)
        self.assertIn('"join request failed"', join)
        self.assertNotIn('"authentication failed"', join)
        self.assertNotIn("password, sizeof(status_", join)
        self.assertNotIn("password, sizeof(lastError_", join)

    def test_tcp_socket_options_are_applied_only_after_connect(self):
        opened = _function(DIRECT, "bool openDatalink()")
        connect = opened.index("tcp_.connect(")
        nodelay = opened.index("tcp_.setNoDelay(true)")
        self.assertLess(connect, nodelay)

    def test_wifi_key_copies_are_scrubbed_when_the_runtime_ends(self):
        provision = _function(DIRECT, "bool provisionOverBle(")
        close_ble = _function(DIRECT, "void closeBle()")
        cleanup = _function(DIRECT, "void cleanup()")
        self.assertIn("secureZero(reply.payload, sizeof(reply.payload))", provision)
        self.assertIn("secureZero(bleRx_, sizeof(bleRx_))", close_ble)
        self.assertIn("WiFi.disconnect(true, true)", cleanup)

    def test_operator_failure_copy_fits_the_status_buffer(self):
        # DirectStatus.detail is char[48], including the terminator. A useful
        # suffix silently disappearing on the device is a UI failure.
        for message in re.findall(r'fail(?:IfWanted)?\("([^"]+)"', DIRECT):
            self.assertLessEqual(len(message), 47, message)

    def test_ble_cancellation_is_checked_after_blocking_work_and_before_writes(self):
        provision = _function(DIRECT, "bool provisionOverBle(")
        connect = provision.index("candidate->connect(&device)")
        service = provision.index("candidate->getService(SERVICE_UUID)")
        characteristics = provision.index("service->getCharacteristic(WRITE_UUID)")
        selected = provision.index("client_ = candidate")
        subscribe = provision.index("notify_->subscribe(true, notifyCallback, true)")
        arm = provision.index("notify_->writeValue(arm, sizeof(arm), true)")
        self.assertIn("if (!wanted())", provision[connect:service])
        self.assertIn("if (!wanted())", provision[service:characteristics])
        self.assertIn("if (!wanted())", provision[characteristics:selected])
        self.assertIn("if (!wanted())", provision[selected:subscribe])
        self.assertIn("if (!wanted())", provision[subscribe:arm])

    def test_ble_cccd_write_requires_an_att_response(self):
        provision = _function(DIRECT, "bool provisionOverBle(")
        self.assertIn("notify_->subscribe(true, notifyCallback, true)", provision)

        send = _function(DIRECT, "bool bleSend(")
        self.assertLess(send.index("if (!wanted())"), send.index("writer_"))
        second_gate = send.index("if (!wanted())", send.index("encodeFrame"))
        self.assertLess(second_gate, send.index("writer_->writeValue"))

    def test_ordinary_cancellation_does_not_publish_a_failure(self):
        helper = _function(DIRECT, "bool failIfWanted(")
        self.assertIn("wanted() ? fail(detail, controlFault) : false", helper)
        provision = _function(DIRECT, "bool provisionOverBle(")
        self.assertIn("return failIfWanted", provision)
        self.assertNotIn("return fail(\"camera BLE", provision)


class TestTransportWindowSafety(unittest.TestCase):
    def test_commands_wait_for_the_camera_sequence_window(self):
        opened = _function(DIRECT, "bool openDatalink()")
        missing = opened.index('failIfWanted("camera sequence window missing")')
        adopt = opened.index("udpSeq_ = static_cast<uint16_t>(cameraChannel_ + 8)")
        register = opened.index("registerController()")
        self.assertLess(missing, adopt)
        self.assertLess(adopt, register)

    def test_ack_never_consumes_the_command_window(self):
        ack = _function(DIRECT, "bool sendAck()")
        self.assertIn("putLe16(header + 4, 0)", ack)
        self.assertNotIn("udpSeq_ =", ack)
        self.assertNotIn("++udpSeq_", ack)

    def test_inbound_transport_is_length_xor_and_flag_checked(self):
        receive = _function(DIRECT, "void handleDatagram(")
        for check in ("data[7] != x", "declared > len", "data[1] & 0x80"):
            self.assertIn(check, receive)

    def test_receive_census_and_scanner_match_the_host_parity_contract(self):
        status = HEADER[HEADER.index("struct DirectStatus"):HEADER.index("class DirectCamera")]
        for field in (
                "rxPeerPackets", "rxForeignPackets", "rxHeaderRejects",
                "rxWindowPackets", "rxAckedDataPackets", "rxVideoPackets",
                "rxFrames", "rxAttitudeFrames",
                "rxAcceptedAttitudes", "lastRxLen", "lastRxType",
                "lastFrameCmdSet", "lastFrameCmdId", "lastFramePayloadLen",
                "lastAttitudeRxLen", "lastAttitudeSeq", "lastAttitudeType",
                "ackVideoCursor", "ackDataCursor", "ackExtraCursor"):
            self.assertIn(field, status)

        pump = _function(DIRECT, "void pumpIncoming()")
        receive = _function(DIRECT, "void handleDatagram(")
        frame_handler = _function(DIRECT, "void handleFrame(")
        for counter in ("rxPeerPackets++", "rxForeignPackets++"):
            self.assertIn(counter, pump)
        for counter in ("rxHeaderRejects++", "rxWindowPackets++", "rxFrames++",
                        "rxAttitudeFrames++"):
            self.assertIn(counter, receive)
        self.assertIn("rxAcceptedAttitudes++", frame_handler)
        self.assertIn("search += offset + 1", receive)
        self.assertNotIn("search += offset + frame.totalLen", receive)

    def test_attitude_window_trace_is_recorded_only_after_validation(self):
        receive = _function(DIRECT, "void handleDatagram(")
        accepted = receive.index("if (frame.cmdSet == 0x04 && frame.cmdId == 0x05)")
        payload_gate = receive.index("if (frame.payloadLen >= 40)", accepted)
        for field in ("lastAttitudeRxLen", "lastAttitudeSeq", "lastAttitudeType"):
            self.assertIn(field, receive)
            self.assertGreater(receive.index(field, accepted), payload_gate)

    def test_primary_source_three_windows_drive_the_receive_ack(self):
        receive = _function(DIRECT, "void handleDatagram(")
        ack = _function(DIRECT, "bool sendAck()")
        self.assertIn("PKT_ACKED_DATA = 0x03", DIRECT)
        telemetry = receive[receive.index("if (packetType == PKT_TELEMETRY && len >= 34)"):]
        telemetry = telemetry[:telemetry.index("} else if (packetType == PKT_ACKED_DATA)")]
        self.assertIn("if (!ackedDataCursor_)", telemetry)
        self.assertIn("ackedDataCursor_ = data[18] | (data[19] << 8)", telemetry)
        self.assertIn("extraCursor_ = data[26] | (data[27] << 8)", telemetry)
        self.assertIn("status_.rxAckedDataPackets++", receive)
        self.assertIn("ackedDataCursor_ = data[4] | (data[5] << 8)", receive)
        self.assertIn("status_.rxVideoPackets++", receive)
        self.assertIn("videoCursor_ = data[4] | (data[5] << 8)", receive)
        for group in ("group(0, videoCursor_)",
                      "group(8, ackedDataCursor_ ? ackedDataCursor_ : baseSeq_)",
                      "group(16, extraCursor_ ? extraCursor_ : baseSeq_)"):
            self.assertIn(group, ack)
        accepted = receive.index("if (frame.cmdSet == 0x04 && frame.cmdId == 0x05)")
        self.assertNotIn("peerCursor_", receive[accepted:receive.index("handleFrame(frame)", accepted)])

    def test_udp_only_accepts_the_camera_peer(self):
        pump = _function(DIRECT, "void pumpIncoming()")
        self.assertIn("remoteIp == cameraIp_", pump)
        self.assertIn("remotePort == CAMERA_UDP_PORT", pump)

    def test_receive_bursts_cannot_starve_the_ack_pump(self):
        pump = _function(DIRECT, "void pumpIncoming()")
        ready = _function(DIRECT, "bool readyLoop()")
        self.assertIn("MAX_RX_PACKETS_PER_PUMP", pump)
        self.assertIn("processed < MAX_RX_PACKETS_PER_PUMP", pump)
        first_ack = ready.index("if (now - lastAck >= 25)")
        receive = ready.index("pumpIncoming()")
        second_ack = ready.index("if (now - lastAck >= 25)", first_ack + 1)
        self.assertLess(first_ack, receive)
        self.assertLess(receive, second_ack)

    def test_direct_wifi_disables_modem_sleep_after_association(self):
        join = _function(DIRECT, "bool joinWifi(")
        connected = join.index("if (current == WL_CONNECTED)")
        disable_sleep = join.index("WiFi.setSleep(false)", connected)
        publish = join.index('setWifiEvidence(attempt, current, channel, bssid,', connected)
        self.assertLess(disable_sleep, publish)


class TestStandaloneMotionSafety(unittest.TestCase):
    def test_networking_cannot_block_lvgl(self):
        self.assertIn('xTaskCreatePinnedToCore(', DIRECT)
        self.assertIn('"osmo-direct"', DIRECT)
        self.assertIn("lv_timer_handler();", MAIN)

    def test_network_task_allocation_failure_is_visible_and_retryable(self):
        begin = _function(DIRECT, "void DirectCamera::begin()")
        self.assertIn("result == pdPASS", begin)
        self.assertIn("direct task allocation failed", begin)
        self.assertIn("failed.controlFault = true", begin)

    def test_abort_is_processed_before_a_reconnect_exit(self):
        ready = _function(DIRECT, "bool readyLoop()")
        self.assertLess(ready.index("in.abortEpoch != seenAbort"),
                        ready.index("in.reconnectEpoch != reconnectAtStart"))
        abort_block = ready[ready.index("in.abortEpoch != seenAbort"):]
        abort_block = abort_block[:abort_block.index("if (in.reconnectEpoch")]
        self.assertIn("sendCentre()", abort_block)
        self.assertIn("noteStopUnconfirmed()", abort_block)
        self.assertIn("requireClutchRelease = true", abort_block)
        self.assertIn("requireJogRelease = abortedJog.held", abort_block)

    def test_clutch_and_jog_require_fresh_telemetry_and_imu(self):
        ready = _function(DIRECT, "bool readyLoop()")
        self.assertIn("telemetryFresh", ready)
        self.assertIn("imuFresh", ready)
        self.assertIn("!requireClutchRelease", ready)
        self.assertIn("!requireJogRelease", ready)
        self.assertIn("control stopped: stale data", ready)
        self.assertIn("camera telemetry stopped", ready)

    def test_telemetry_timeout_reports_the_received_evidence_frontier(self):
        ready = _function(DIRECT, "bool readyLoop()")
        for reason in ("no camera UDP downlink", "no valid camera DUML frames",
                       "camera sent no attitude telemetry",
                       "camera attitude payload unsupported",
                       "camera telemetry stopped"):
            self.assertIn(reason, ready)
        self.assertLess(ready.index("status_.rxPeerPackets == 0"),
                        ready.index("status_.rxFrames == 0"))
        self.assertLess(ready.index("status_.rxFrames == 0"),
                        ready.index("status_.rxAttitudeFrames == 0"))

    def test_one_hz_presence_is_immediately_acknowledged(self):
        ready = _function(DIRECT, "bool readyLoop()")
        keepalive = ready[ready.index("if (now - lastKeepalive >= 1000)"):]
        keepalive = keepalive[:keepalive.index("const bool telemetryFresh")]
        sent = keepalive.index("sendDuml(RX_DM368_1, FLAG_REQUEST, 0x00, 0x88,")
        self.assertLess(sent, keepalive.index("sendAck()"))
        self.assertIn('failIfWanted("camera keepalive failed", motionActive_)',
                      keepalive)
        self.assertRegex(
            keepalive,
            r'failIfWanted\("camera keepalive ACK failed",\s*motionActive_\)')

    def test_explicit_reconnect_cancels_every_connection_phase_and_backoff(self):
        wanted = _function(DIRECT, "bool wanted() const")
        task = _function(DIRECT, "void DirectCamera::taskLoop()")
        self.assertIn("in.reconnectEpoch == reconnectAtStart_", wanted)
        immediate = "inputs().reconnectEpoch != attemptReconnectEpoch) continue"
        self.assertIn(immediate, task)

    def test_motion_send_failures_are_fatal_and_keep_motion_evidence(self):
        ready = _function(DIRECT, "bool readyLoop()")
        send = _function(DIRECT, "bool sendRates(")
        self.assertIn('failIfWanted("gimbal command send failed", true)', ready)
        self.assertLess(send.index("if (!ok) return false"),
                        send.index("motionActive_ ="))

    def test_jog_latch_tracks_raw_hold_and_release_across_abort(self):
        ready = _function(DIRECT, "bool readyLoop()")
        set_jog = _function(DIRECT, "void DirectCamera::setJog(")
        self.assertIn("const RawJogSnapshot initialJog = snapshotRawJog()", ready)
        self.assertIn("requireJogRelease = initialJog.held", ready)
        self.assertIn("seenJogReleaseEpoch = initialJog.releaseEpoch", ready)
        abort = ready[ready.index("in.abortEpoch != seenAbort"):]
        abort = abort[:abort.index("if (in.reconnectEpoch")]
        self.assertIn("const RawJogSnapshot abortedJog = snapshotRawJog()", abort)
        self.assertIn("requireJogRelease = abortedJog.held", abort)
        self.assertIn("seenJogReleaseEpoch = abortedJog.releaseEpoch", abort)
        self.assertIn("releaseEpoch != seenJogReleaseEpoch", ready)
        self.assertNotIn("if (!in.jogActive) requireJogRelease = false", ready)

        # main.cpp owns raw-contact debouncing and never calls setJog(false)
        # for its offline-held synthetic neutral. The transport therefore keeps
        # the raw edge alive across reconnect instead of gating it on READY.
        self.assertNotIn("status_.ready", set_jog)
        self.assertNotIn("status_.telemetry", set_jog)
        self.assertIn("rawJogHeld.store(true", set_jog)
        self.assertIn("rawJogHeld.exchange(false", set_jog)
        self.assertIn("rawJogReleaseEpoch.fetch_add", set_jog)

        # STOP/reconnect/offline neutral may clear the logical command, but
        # none is allowed to fabricate the physical edge that clears the latch.
        for signature in ("void DirectCamera::abort()",
                          "void DirectCamera::requestReconnect()",
                          "void DirectCamera::disable()"):
            body = _function(DIRECT, signature)
            self.assertNotIn("rawJogHeld", body)
            self.assertNotIn("rawJogReleaseEpoch", body)

    def test_abort_stop_failure_survives_until_cleanup_retry(self):
        ready = _function(DIRECT, "bool readyLoop()")
        start = ready.index("if (in.abortEpoch != seenAbort)")
        end = ready.index("if (in.reconnectEpoch", start)
        abort = ready[start:end]
        failed = abort.index("if (!sendCentre())")
        self.assertLess(failed, abort.index("noteStopUnconfirmed()"))
        self.assertLess(failed,
                        abort.index('return fail("STOP delivery unconfirmed", true)'))
        self.assertLess(abort.index('return fail("STOP delivery unconfirmed", true)'),
                        abort.index("resetMotion()"))

    def test_cancel_exit_stop_failure_preserves_motion_for_cleanup(self):
        ready = _function(DIRECT, "bool readyLoop()")
        tail = ready[ready.rindex("if (!sendCentre())"):]
        self.assertIn("noteStopUnconfirmed()", tail)
        self.assertLess(tail.index("return false"), tail.index("resetMotion()"))
        cleanup = _function(DIRECT, "void cleanup()")
        self.assertLess(cleanup.index("if (motionActive_) sendCentre()"),
                        cleanup.index("resetMotion()"))

    def test_stale_motion_only_resets_after_a_confirmed_centre(self):
        ready = _function(DIRECT, "bool readyLoop()")
        start = ready.index("if (staleManual)")
        end = ready.index("if (now - lastStick", start)
        stale = ready[start:end]
        stop_check = stale.index("motionActive_ && !sendCentre()")
        self.assertIn("noteStopUnconfirmed()", stale)
        self.assertIn('return fail("STOP delivery unconfirmed", true)', stale)
        self.assertLess(stop_check, stale.index("resetMotion()"))

    def test_live_axis_lock_changes_rebase_the_relative_hand_frame(self):
        ready = _function(DIRECT, "bool readyLoop()")
        self.assertIn("in.lockTilt != priorLockTilt", ready)
        self.assertIn("cameraPitchRef_ = cameraPitch_", ready)
        self.assertIn("in.lockPan != priorLockPan", ready)
        self.assertIn("clutchQ0_ = {in.qw, in.qx, in.qy, in.qz}", ready)
        self.assertIn("yawShaper_.reset(lastYawDps_)", ready)

    def test_retry_wait_happens_after_runtime_socket_cleanup(self):
        task = _function(DIRECT, "void DirectCamera::taskLoop()")
        cleanup = task.index("}  // close sockets/radios")
        self.assertLess(cleanup,
                        task.index("DirectStatus retry = runtimeStatus", cleanup))
        self.assertLess(cleanup,
                        task.index("DirectStatus retry = status()", cleanup))

    def test_missing_24ghz_ap_waits_for_explicit_retry_without_repairing(self):
        task = _function(DIRECT, "void DirectCamera::taskLoop()")
        manual = _function(DIRECT, "if (runtimeNeedsManualWifiRetry)")
        self.assertIn("runtime.needsManualWifiRetry()", task)
        self.assertIn("DirectStatus retry = runtimeStatus", manual)
        self.assertIn("inputs_.deviceMode = DeviceMode::Idle", manual)
        self.assertIn("retry.blockedAt = DirectPhase::WifiJoin", manual)
        self.assertIn("retry.retryAtMs = 0", manual)
        self.assertNotIn("RETRY_MS", manual)
        self.assertIn("awaitingExplicitRetry", task)

    def test_usb_and_direct_ownership_are_mutually_exclusive(self):
        line = _function(MAIN, "static void handleLine(")
        loop = _function(MAIN, "void loop()")
        self.assertIn("usbHostSeen = true", line)
        self.assertIn("directCamera.disable()", line)
        self.assertIn("if (!usbHostSeen) osmo::directCamera.begin()", loop)
        self.assertIn("rig.linked && !rig.direct", loop)

    def test_jog_and_release_tail_have_a_truthful_owner(self):
        state = _function(MAIN, "static void pumpDirectState()")
        self.assertIn("st.clutch || st.moving", state)

    def test_direct_pitch_headroom_drives_the_existing_limit_warnings(self):
        state = _function(MAIN, "static void pumpDirectState()")
        self.assertIn("st.pitchWarnLow", state)
        self.assertIn("st.pitchWarnHigh", state)
        self.assertIn("limitNear", state)
        self.assertIn("yaw envelope is still contradictory", state)

    def test_raw_imu_stream_starts_before_the_host_claim_window(self):
        setup = _function(MAIN, "void setup()")
        loop = _function(MAIN, "void loop()")
        self.assertIn("hostClaimDeadlineMs = millis() + 750", setup)
        self.assertIn("startImuTask();", setup)
        self.assertLess(setup.index("startImuTask();"),
                        setup.index("hostClaimDeadlineMs = millis() + 750"))
        self.assertIn("if (!imuTaskHandle) pumpImu(now)", loop)
        self.assertIn("now - hostClaimDeadlineMs", loop)

    def test_imu_sampling_has_a_deterministic_ui_independent_task(self):
        task = _function(MAIN, "static void imuTaskEntry(")
        start = _function(MAIN, "static bool startImuTask()")
        self.assertIn("sampleImu();", task)
        self.assertIn("const TickType_t period = pdMS_TO_TICKS(10)", task)
        self.assertIn("if (now - previousWake >= period) previousWake = now", task)
        self.assertIn("vTaskDelayUntil(&previousWake, period)", task)
        for forbidden in ("lv_", "M5.Touch", "refreshUi", "delay(",
                          "pumpDirectState"):
            self.assertNotIn(forbidden, task)
        self.assertIn('imuTaskEntry, "core2-imu", 4096, nullptr, 2, &created, 1',
                      start)
        self.assertIn("result != pdPASS || !created", start)
        sample = _function(MAIN, "static void sampleImu()")
        self.assertIn("M5.Imu.update()", sample)
        self.assertIn("sensor_mask_accel", sample)
        self.assertIn("sensor_mask_gyro", sample)
        self.assertIn("M5.Imu.getImuData(&data)", sample)
        self.assertLess(sample.index("M5.Imu.getImuData(&data)"),
                        sample.index("const uint32_t sampleAt = millis()"))
        self.assertIn("Serial.availableForWrite() >= len", sample)
        self.assertIn("++imuSerialDrops", sample)

    def test_host_control_settings_reconcile_into_the_same_ui_state(self):
        apply = _function(MAIN, "static void applyState(")
        for key in ('"gain"', '"trsp"', '"prsp"', '"tstab"', '"pstab"',
                    '"speed"', '"invt"', '"invp"'):
            self.assertIn(key, apply)
        self.assertIn("responseIndexFromWire", apply)
        self.assertIn("stabilityIndexFromWire", apply)


class TestStandaloneBuildConfiguration(unittest.TestCase):
    def test_ble_dependency_and_large_app_partition_are_explicit(self):
        self.assertIn("h2zero/NimBLE-Arduino@1.4.2", INI)
        self.assertIn("board_build.partitions = huge_app.csv", INI)

    def test_lvgl_draw_buffers_are_in_psram(self):
        self.assertIn("MALLOC_CAP_SPIRAM", MAIN)
        self.assertNotRegex(MAIN, r"lv_color_t\s+buf[12]\s*\[")


class TestHandgripMotionContracts(unittest.TestCase):
    """Pin the operator-facing Core2 handgrip semantics at source level."""

    def test_raw_6dof_estimator_has_real_dt_all_axis_bias_and_no_ui_mapping(self):
        header = _function(HEADER, "void setImu(")
        imu = _function(DIRECT, "void DirectCamera::setImu(")
        self.assertIn("ax, float ay, float az, float gx, float gy, float gz", header)
        self.assertIn("imu_.lastAtMs", imu)
        self.assertIn("std::max(0.004f, std::min(0.03f, dt))", imu)
        for axis in ("imu_.bx", "imu_.by", "imu_.bz"):
            self.assertIn(axis, imu)
        self.assertIn("sampleAtMs - imu_.stationarySinceMs >= 900", imu)
        self.assertIn("sampleAtMs < imu_.freezeUntilMs", imu)
        self.assertIn("Quat omega", imu)
        self.assertIn("gravityBody(q)", imu)
        self.assertIn("cross(measured, estimated)", imu)
        self.assertNotIn("cross(estimated, measured)", imu)
        self.assertNotIn("invertTilt", imu)
        self.assertNotIn("invertPan", imu)

    def test_arbitrary_pose_clutch_neutralises_then_waits_for_post_press_pose(self):
        ready = _function(DIRECT, "bool readyLoop()")
        acquire = ready[ready.index("if (in.clutch && !priorClutch"):]
        acquire = acquire[:acquire.index("if (clutchAcquirePending_)")]
        self.assertIn("sendCentre()", acquire)
        self.assertIn("clutchAcquirePending_ = true", acquire)
        self.assertIn("clutchAcquireAfterMs_ = now", acquire)
        pending = ready[ready.index("if (clutchAcquirePending_)"):]
        pending = pending[:pending.index("// A lock toggle")]
        self.assertIn("lastTelemetryMs_ > clutchAcquireAfterMs_", pending)
        self.assertIn("clutchPressQ_ : Quat{in.qw, in.qx, in.qy, in.qz}", pending)
        self.assertIn("rotateVector(clutchQ0_", pending)
        self.assertIn("cross(worldDown, screenForward)", pending)
        self.assertIn("clutch pose not confirmed", pending)

    def test_three_templates_are_relative_and_never_use_yaw_position_feedback(self):
        clutch = _function(DIRECT, "bool commandClutch(")
        self.assertIn("if (in.templateIndex == HAND_TEMPLATE_FOLLOW)", clutch)
        self.assertIn("HAND FOLLOW", clutch)
        self.assertIn("GYRO RATE", clutch)
        self.assertIn("AIR MOUSE", clutch)
        self.assertIn("airMouseFilter_.step(", clutch)
        self.assertIn("cameraPitchRef_", clutch)
        self.assertIn("wantedYaw = panRate * panResponse", clutch)
        self.assertNotIn("cameraYaw_", clutch)
        self.assertNotIn("targetYaw", clutch)

    def test_air_mouse_captures_the_press_edge_without_weakening_pose_gate(self):
        ready = _function(DIRECT, "bool readyLoop()")
        press = ready.index("clutchPressQ_ = {in.qw, in.qx, in.qy, in.qz};")
        gate = ready.index("lastTelemetryMs_ > clutchAcquireAfterMs_")
        acquire = ready.index("clutchQ0_ = in.templateIndex == HAND_TEMPLATE_AIR_MOUSE")
        self.assertLess(press, gate)
        self.assertLess(gate, acquire)
        self.assertIn("sendCentre()", ready[:press])
        self.assertIn("airMouseFilter_.prime();", ready[press:gate])
        self.assertIn("airMouseFilter_.reset();", ready)

    def test_air_mouse_filter_state_is_continuous_across_axis_constraints(self):
        ready = _function(DIRECT, "bool readyLoop()")
        locks = ready[ready.index("// A lock toggle"):
                      ready.index("priorLockTilt = in.lockTilt")]
        self.assertIn("airMouseFilter_.rebaseAngles();", locks)
        pan_lock = locks[locks.index("if (clutchActive_ && in.lockPan"):]
        self.assertNotIn("airMouseFilter_.reset();", pan_lock)
        release = ready[ready.index("(clutchActive_ && !in.clutch)"):
                        ready.index("} else if (releasing_)")]
        self.assertIn("airMouseFilter_.reset();", release)

    def test_mpu6886_bandwidth_is_anti_aliased_and_read_back_before_sampling(self):
        configure = _function(MAIN, "static bool configureImuControlBandwidth()")
        self.assertIn("M5.Imu.getType() != m5::imu_mpu6886", configure)
        self.assertIn("gyroFilterRegister = 0x1A", configure)
        self.assertIn("accelFilterRegister = 0x1D", configure)
        self.assertGreaterEqual(configure.count("readRegister("), 4)
        self.assertEqual(configure.count("writeRegister8("), 2)
        setup = _function(MAIN, "void setup()")
        self.assertLess(setup.index("configureImuControlBandwidth();"),
                        setup.index("startImuTask();"))

    def test_speed_sensitivity_and_axis_tune_have_bounded_independent_shapers(self):
        self.assertIn("RESPONSE_GAINS[] = {0.25f, 0.5f, 1.0f}", DIRECT)
        self.assertIn("SPEED_CAPS[] = {6.0f, 18.0f, 42.0f}", DIRECT)
        self.assertIn("STABILITY_FACTORS[] = {0.55f, 1.0f, 1.6f}", DIRECT)
        shaper = _function(DIRECT, "float step(float target, float dt, uint8_t smooth,")
        self.assertIn("maxAccel[]", shaper)
        self.assertIn("maxJerk[]", shaper)
        self.assertIn("wantedAccel", shaper)
        self.assertIn("stability", shaper)
        clutch = _function(DIRECT, "bool commandClutch(")
        jog = _function(DIRECT, "bool commandJog(")
        for body in (clutch, jog):
            self.assertIn("capVector(", body)
        self.assertIn("in.smoothIndex", clutch)
        self.assertIn("in.jogSmoothIndex", jog)
        self.assertIn("in.jogSpeedIndex", jog)
        self.assertIn("pitchShaper_.step", clutch)
        self.assertIn("yawShaper_.step", clutch)
        self.assertIn("in.tiltStabilityIndex", clutch)
        self.assertIn("in.panStabilityIndex", clutch)

    def test_axis_tune_crosses_the_atomic_direct_config_contract(self):
        header = HEADER[HEADER.index("void setControlConfig("):
                        HEADER.index("void setJogConfig(")]
        for field in ("tiltResponseIndex", "panResponseIndex",
                      "tiltStabilityIndex", "panStabilityIndex"):
            self.assertIn(field, header)
            self.assertIn(field, HEADER)
        setter = _function(DIRECT, "void DirectCamera::setControlConfig(")
        for field in ("tiltResponseIndex", "panResponseIndex",
                      "tiltStabilityIndex", "panStabilityIndex"):
            self.assertIn(f"inputs_.{field}", setter)
            self.assertIn(f"std::min<uint8_t>({field}, 2)", setter)

    def test_hand_response_is_axis_specific_but_final_vector_is_always_recapped(self):
        clutch = _function(DIRECT, "bool commandClutch(")
        self.assertIn("tiltResponse", clutch)
        self.assertIn("panResponse", clutch)
        self.assertIn("in.tiltResponseIndex", clutch)
        self.assertIn("in.panResponseIndex", clutch)
        first_cap = clutch.index("capVector(wantedPitch, wantedYaw, cap)")
        tilt_response = clutch.index("tiltResponse")
        pan_response = clutch.index("panResponse")
        self.assertLess(tilt_response, first_cap)
        self.assertLess(pan_response, first_cap)
        shaper = clutch.index("pitchShaper_.step")
        final_cap = clutch.index("capVector(pitch, yaw, cap);", shaper)
        self.assertLess(shaper, final_cap)
        self.assertNotIn("HAND_TEMPLATE_AIR_MOUSE", clutch[shaper:final_cap])

    def test_jog_does_not_inherit_hand_axis_tune(self):
        jog = _function(DIRECT, "bool commandJog(")
        for hand_only in ("tiltResponseIndex", "panResponseIndex",
                          "tiltStabilityIndex", "panStabilityIndex"):
            self.assertNotIn(hand_only, jog)

    def test_jog_is_radial_deadzone_expo_then_vector_limited(self):
        jog = _function(DIRECT, "bool commandJog(")
        response = _function(DIRECT, "float DirectCamera::jogResponse(")
        self.assertIn("DirectCamera::jogResponse(magnitude)", jog)
        self.assertIn("constexpr float deadzone = 0.12f", response)
        self.assertIn("const float magnitude = std::sqrt(x*x + y*y)", jog)
        self.assertIn("std::pow((magnitude - deadzone) / (1.0f - deadzone), 1.6f)",
                      response)
        self.assertLess(jog.index("capVector(wantedPitch, wantedYaw, cap)"),
                        jog.index("pitchShaper_.step"))

    def test_pitch_braking_has_coast_math_and_endpoint_anti_windup(self):
        brake = _function(DIRECT, "float brakePitch(")
        clutch = _function(DIRECT, "bool commandClutch(")
        self.assertIn("CAMERA_COAST_S", brake)
        self.assertIn("v^2/(2a)", brake)
        self.assertIn("vSafe", brake)
        self.assertIn("clampPitch(rawTarget)", clutch)
        self.assertIn("cameraPitchRef_ += wrap180(targetPitch - rawTarget)", clutch)
        self.assertLess(
            clutch.index("clampPitch(rawTarget)"),
            clutch.index("wantedPitch = (targetPitch - cameraPitch_)"))

    def test_direct_record_request_is_explicit_but_never_claimed_as_recording(self):
        status = HEADER[HEADER.index("struct DirectStatus"):HEADER.index("class DirectCamera")]
        for field in ("recordIntent", "recordPending", "recordUnconfirmed",
                      "recordCommandFault"):
            self.assertIn(field, status)
        request = _function(DIRECT, "void DirectCamera::requestRecord(")
        ready = _function(DIRECT, "bool readyLoop()")
        send = _function(DIRECT, "bool sendRecordRequest(")
        self.assertIn("++inputs_.recordEpoch", request)
        self.assertIn("status_.recordPending = true", request)
        self.assertIn("telemetryFresh && sendRecordRequest(in.recordStart)", ready)
        self.assertIn("status_.recordUnconfirmed = true", ready)
        self.assertIn("status_.recordCommandFault = true", ready)
        self.assertIn("RX_CAMERA, FLAG_REQUEST, 0x02, 0x02", send)
        self.assertNotIn("recording", status)

    def test_inversion_is_applied_after_estimation_at_the_command_boundary(self):
        estimator = _function(DIRECT, "void DirectCamera::setImu(")
        clutch = _function(DIRECT, "bool commandClutch(")
        jog = _function(DIRECT, "bool commandJog(")
        send = _function(DIRECT, "bool sendRates(")
        self.assertNotIn("invertTilt", estimator)
        self.assertNotIn("invertPan", estimator)
        self.assertIn("HAND_TILT_BASE_SIGN", DIRECT)
        self.assertIn("HAND_PAN_BASE_SIGN", DIRECT)
        self.assertIn("HAND_TILT_BASE_SIGN *", clutch)
        self.assertIn("HAND_PAN_BASE_SIGN *", clutch)
        self.assertIn("in.invertTilt ? -1.0f : 1.0f", clutch)
        self.assertIn("in.invertPan ? -1.0f : 1.0f", clutch)
        self.assertIn("in.invertTilt ? -1.0f : 1.0f", jog)
        self.assertIn("in.invertPan ? -1.0f : 1.0f", jog)
        self.assertNotIn("invertTilt", send)
        self.assertNotIn("invertPan", send)


if __name__ == "__main__":
    unittest.main()
