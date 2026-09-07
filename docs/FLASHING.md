# OsmoPalm v0.1.0-preview.1 — flashing guide

Target: **M5Stack Core2, ESP32**, not CoreS3, CoreInk, M5Paper or another ESP32 product.
The original development hardware uses an MPU6886 IMU and AXP192 power management.
Other Core2 hardware revisions have not completed this project's hardware acceptance.
Firmware hello: `H 7.9.1-return-icon ...`.
Preview release: use on a clear, stationary test setup, not an unattended paid shoot.

## Before writing

1. Verify the download hashes against SHA256SUMS.txt.
2. Identify the exact Core2 serial port. Close its serial monitor or coordinate with any
   application holding that port. Never kill an unrelated process just to free a port.
3. Use a reliable USB data cable and stable power. Keep the camera disconnected/off
   during firmware installation; the firmware can reconnect to a remembered camera.
4. Preserve your existing firmware/settings before changing applications or partition layouts.
   A private full-flash backup contains saved devices/settings: never upload it to GitHub.
   The normal Core2 has 16 MB flash, but verify the actual device before choosing backup size.
5. Do not use erase_flash or a merged full-flash image as a routine update.

These commands are for **esptool 4.9.0**. Install it into a local Python environment if needed:

```sh
python -m pip install esptool==4.9.0
python -m esptool --chip esp32 --port YOUR_CORE2_PORT flash_id
```

For a confirmed 16 MB Core2, an optional private backup command is:

```sh
python -m esptool --chip esp32 --port YOUR_CORE2_PORT read_flash 0x0 0x1000000 core2-private-backup.bin
```

## Update an existing OsmoPalm with the same huge_app partition layout

From the directory containing the application download:

```sh
python -m esptool --chip esp32 --port YOUR_CORE2_PORT --baud 460800 write_flash 0x10000 OsmoPalm-v0.1.0-preview.1-firmware.bin
```

**Never write this application-only binary at offset 0x0.**
This does not intentionally overwrite NVS or the filesystem; do not assume it is compatible
with an unknown prior partition table. Use the first-install path after backing up instead.

## First installation or a different application/partition layout

Back up first. Unzip the core2-install package and change into its directory. The supplied
partition map is Arduino-ESP32 huge_app (3 MB application beginning at 0x10000).
Changing partition tables can make previous application data inaccessible. These files are
a matched set; do not mix them with files from another build.

```sh
python -m esptool --chip esp32 --port YOUR_CORE2_PORT --baud 460800 write_flash 0x1000 bootloader.bin 0x8000 partitions.bin 0xe000 boot_app0.bin 0x10000 firmware.bin
```

There is no automatic erase, camera pairing or motion step in this package. The command
does overwrite the bootloader, partition table, boot-app data and application sectors.
If you prefer building from source, check out this release tag and use its PlatformIO
configuration, which supplies these files and offsets automatically.

## Verify after reboot

Observe the serial port at 115200 baud with monitor control lines configured not to hold
the board in reset. A safe `?\n` diagnostic request returns the firmware hello; do not send
an `S` state line while checking boot because it can claim USB camera authority.
Check the correct version, continuing IMU lines and absence of observed panic/reboot.
Close the monitor before a later upload. A short boot check does not certify motion safety.

Then, only in a deliberately authorized camera test with a clear gimbal area, check
connection/telemetry, small HAND/JOG movements, release, STOP and reconnection. Pocket 4
and Pocket 3 have **not been tested by us**. Stop testing on unexpected movement or halts.
