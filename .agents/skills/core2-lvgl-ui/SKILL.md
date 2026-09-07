---
name: core2-lvgl-ui
description: Building or changing the M5Stack Core2 panel UI (firmware/core2_panel, LVGL 8.4). Use before touching main.cpp — it carries the screen geometry contract, the failure modes that boot-loop the box, the colour grammar shared with the web panel, and the flash/verify loop. Every rule here came from a bug that actually happened on this hardware.
---

# The Core2 panel

A 320x240 LVGL 8.4 panel in one ~4,300-line `main.cpp`, plus a ~3,300-line
standalone camera runtime in `direct_camera.cpp`, driving a gimbal rig.
The operator is a camera operator: they are looking at the subject, not at this
screen, and they are holding the box in one hand.

Everything below is a fact about *this* hardware or a mistake that has already
been made on it. None of it is general LVGL advice — read the LVGL docs for
that.

## The geometry contract

The screen is 240 px tall and the soft-key nav row is nailed to the bottom of
it. Break this and content hides under the nav bar, which is invisible until
someone photographs the box.

```
  y=0                     header   40 px on every page, including STOP
  y=40                    page     168 px on every page
  y=208                   nav row  32 px, three soft keys
  y=240                    bottom
```

**Invariant: `HEADER_H + PAGE_H == 208`.** Asserted in
`tests/test_firmware_source.py::TestPageGeometry`. The header is a reserved
safety region, not page content: the record cell, context/transport/owner, and
the persistent STOP occupy its three cells. STOP owns `x=262..320, y=0..40`,
is parented to `screenMain` after page content, has a 2 px white left rule,
and fires on `LV_EVENT_PRESSED`; it is never dimmed. Every page begins at
y=40, so no later control can sit under STOP. HAND and JOG use a 168x168
square at local x=76, y=0. Axis selectors are 64x104 at x=6 and x=250;
their separate gear targets are 64x56 at y=112, with an 8 px vertical gap.
JOG retains floating-origin pickup. Never expand pad hit-testing into the gears.

**Lists must scroll.** Settings content can exceed the 168 px page as soon as
the persisted sub-pages are exposed; fixed-height layouts have to be re-tuned
every time an item is added, so the settings menu sets
`lv_obj_set_scroll_dir(..., LV_DIR_VER)`. Overflowing Feel, point editing,
Camera and easing panes also scroll; live movement surfaces never scroll.
Opening an easing gear requests neutral and blocks underlying control input.
Normal release easing is independent per axis, bounded to 750 ms, and never
used for STOP, stale input/telemetry, or connection failure.

## Two ways to brick the box

**A null widget is a boot loop.** `profBtn[]` was declared and driven by
`refreshUi()` but never created. LVGL dereferences without checking, so the
first refresh panicked and the box boot-looped — which reads as dead hardware,
not a missing widget. `setKey()` now guards `if (!btn || !lbl) return;`. When
you add an array of widgets, create them in the same commit as the code that
paints them, and add a `test_firmware_source` check that both exist.

**Declaration order.** One translation unit, callbacks above their
definitions. Three separate build failures this session were nothing but
this. If a function is defined next to what it belongs with but called
earlier, forward-declare it *next to the state it operates on*, not just
anywhere above — a declaration that lands at line 650 does not help a caller
at line 519.

## Colour grammar

One colour means one thing, in the firmware **and** the web panel. An operator
who learns it on one surface must not have to relearn it on the other.

| Colour | Meaning | Never used for |
|---|---|---|
| `C_CYAN` (`0x3DD6F5`) | live control, commanded motion | anything passive |
| `C_AMBER` (`0xFFB020`) | armed, constrained, near a limit, unconfirmed record state | success |
| `C_GREEN` (`0x4CE05B`) | cue release or affirmative system evidence such as a proven link or charging state | generic ready, a local request, or an unproven tally |
| `C_REC` (`0xFF453A`) | authoritative record tally | any other alarm or standalone record request |
| `C_MAGENTA` (`0xE04CFF`) | fault, lost authority | ordinary warnings |

Travel warnings run amber to red with proximity. Green is deliberately narrow:
it confirms only cue release or affirmative evidence, never a blanket "ready"
claim. Nothing blinks except the authoritative record tally — a second blinking
element makes the first one stop meaning "rolling".

## The side LED bars

Ten SK6812 on pin 25: two vertical bars of five, `LED_LEFT_BASE 0` and
`LED_RIGHT_BASE 5`. Use the geometry — top of both bars is tilt-up, bottom is
tilt-down, middle-left is pan-left, middle-right is pan-right.

**Overlay, never replace.** `renderBeacon()` paints the state colour first and
`overlayLimitLeds()` takes only the LEDs a live warning needs. Arm, record and
fault must never be masked by a travel warning. `LIMIT_LED_FLOOR` is 30 because
the host caps a merely-parked axis at 35 — a lower floor leaves the bars
permanently lit and the operator learns to ignore them.

`renderFlash()` deliberately does *not* overlay: it is a brief post-take
verdict, not a steady state.

## Touch and the nav bar

The nav bar is global and its meaning must not change page to page. The one
allowed exception is a sub-view (SETTINGS panes, DEVICE sub-views, the MOVES
run pane while it can still be left), where the left key becomes BACK — and it
is tinted amber so the change of duty is visible. BACK never cancels a running
scan or connect as a side effect of leaving a page; on the DEVICE home it is a
page key. A BACK that cannot act (run pane armed or running) falls through to
page navigation rather than going dead.

The middle rail key and BtnB cycle HAND -> JOG -> CAMERA (the existing
CAM_TOOLS view) -> HAND. From adjustment sheets and other workspaces they
return to HAND. The middle key is navigation, never a clutch. The narrow
40 px arrows remain separate. Camera REC stays in the persistent header;
camera actions retain their evidence gates and unconfirmed-focus labeling.

Holds come in three lengths and no other: `HOLD_ACT_MS` for anything that
moves or arms the head, `HOLD_DESTROY_MS` for anything that discards setup,
`HOLD_POWER_MS` for the one hold that tears the radio down. REC keeps its own
shorter `HOLD_RECORD_MS`. A new hold uses one of these, never a literal.

Minimum comfortable target on this screen is about 40 px; full-width rows beat
tiles for a menu, because a row is easier to hit one-handed.

## The box does not invent state

The Core2 has two mutually exclusive transport modes. In USB mode it displays
only the camera and telemetry evidence pushed down the `S` line. In DIRECT mode
it displays the phase reported by `DirectCamera`: START, SCAN, PAIR, APPROVE,
WIFI, LINK, TELEMETRY, RETRY, then READY. In either mode, READY requires a
camera session and fresh telemetry; a serial line, BLE connection, Wi-Fi
association, or UDP socket alone is not enough. The Wi-Fi page shows the full
connection detail and says "not reported" when no SSID is known.

Clutch and jog use the same evidence gate as the display. An offline press must
not move the visual control or provide grab feedback, and a clutch held while
the link becomes ready must be released before it can acquire the head.

Same rule for the firmware version string: it comes from one `FW_VERSION`
define used by both the hello line and the system page, so they cannot
disagree.

## Testing C++ from Python

`tests/test_firmware_source.py`, `tests/test_core2_product_spine.py` and the
runtime/store contract files read the sources as text and assert structural
facts. Every file is unittest-style; `.venv/Scripts/python.exe -m unittest
discover -s tests -t .` runs all of them (a pytest-style file is invisible to
discovery and reports OK without running — that happened once). They cannot
run the firmware, so they
tests the things that are cheap to get wrong and expensive to discover:
geometry arithmetic, every pane being both created and reachable, widgets that
`refreshUi` touches actually existing, and the LED overlay running after the
beacon rather than before.

Write the assertion so it *fails* if the bug returns, then prove it by
introducing the bug and watching it fail. A structural test that only ever
passes is decoration.

## Flash and verify

Identify the exact Core2 serial device before any authorized upload. If another
process owns its port, inspect that process and coordinate before stopping it.
Do not kill a listener merely because it uses the expected panel port.

```sh
pio run --project-dir firmware/core2_panel
pio run --project-dir firmware/core2_panel -t upload --upload-port YOUR_CORE2_PORT
```

After upload, verify the H version line, IMU traffic and absence of observed panic/reset.
A source-only repository split does not change firmware and does not require flashing.

Always bump `FW_VERSION` so the hello line proves which image is running.
Then restart the panel; the Core2 link supervises its own port and reattaches
on its own.

## Before you finish

- Does every page still fit above y=208?
- Is STOP wholly inside its reserved header rectangle and clear of every page control?
- Does every widget `refreshUi` touches get created?
- Can READY appear only with camera plus fresh telemetry?
- Do clutch and jog refuse input under the same evidence gate?
- Did the box actually boot, with IMU lines and no panic?
- Does the colour you used already mean something else?
- If you added a list, does it scroll?
