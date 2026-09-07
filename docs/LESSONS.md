# Core2 lessons carried into OsmoPalm

- One camera connection owner. BLE provisions credentials; Wi-Fi carries gimbal control.
  A connection is not ready until camera telemetry is fresh.
- Clutch engagement must be relative to the current Core2 pose, not an absolute startup pose.
  Re-engagement must not jump the camera. Keep inversion independent per axis.
- Release easing is not emergency braking. STOP, stale telemetry and lost authority bypass it.
  Fine-mode changes need native shaping tests plus real gimbal observation.
- Thumb ergonomics are a hardware constraint: 320x240, 40px header, 168px page, 32px rail.
  The 168x168 HAND/JOG squares must not intercept the separate axis gear targets.
- Create every LVGL widget before refresh touches it. Missing widgets have caused boot loops.
  Color contrast applies to drawn icons too: the return arrow is dark on an amber button.
- AXP192 haptic voltage has a minimum effective level. Softness comes from bounded short
  pulses, cooldown and cancellation, not an unsupported low voltage setting.
- Preferences writes can fail. Validate stored blobs and restore in-memory values on failure.
- Telemetry-backed recording differs from a record request; focus is still unconfirmed.
- The historical USB-unresponsive halts had no established root cause. A successful flash
  or a short panic-free boot must not be reported as fixing those halts.

These are inherited engineering observations, not fresh hardware acceptance for this split.
Read the UI skill for exact geometry, color and input contracts. Consult OsmoDesk's transport
and tests for protocol fixes; port only with embedded timing and safety verification.
