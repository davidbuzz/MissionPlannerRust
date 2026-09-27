---
name: bench-cubeorange
description: The bench CubeOrange (serial 373532393330510F002E0019) - its bootloader history, how to reach it, and what the first real flash attempts taught (2026-09-25)
metadata:
  type: project
---

**The board.** A CubeOrange on the bench. Its by-id name follows the firmware's USB strings: with
ArduCopter 4.7.1 stable (flashed 2026-09-25 13:55) it is
`/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange_19002E000F51303339323537-if00` (MAVLink; `-if02`
is the second CDC port); the build it carried before called itself
`usb-ArduPilot_CubeOrange_373532393330510F002E0019-if-mavlink`. The `/dev/ttyACMn` number moves:
a Black Magic Probe v1.9.1 on the same hub takes ACM0/ACM1 when Buzz attaches it. In the bootloader
it is `usb-Hex_ProfiCNC_CubeOrange-BL_...`.
Its bootloader was an experimental Zephyr one that presented **no USB device at all** once told to
reboot into it (`PREFLIGHT_REBOOT_SHUTDOWN` param1 3) - the board simply vanished from USB until a
power cycle - and its firmware answered `MAV_RESULT_UNSUPPORTED` to `MAV_CMD_FLASH_BOOTLOADER`.
On 2026-09-25 13:50 Buzz ruled it replaced and I wrote ArduPilot's stock `CubeOrange_bl.bin`
(firmware.ardupilot.org/Tools/Bootloaders, 40,204 bytes) through the probe with `gdb-multiarch`:
`target extended-remote /dev/ttyACM0`, `monitor swdp_scan`, `attach 1`, `load CubeOrange_bl.hex`
(an Intel HEX made with `arm-none-eabi-objcopy -I binary -O ihex --change-addresses 0x08000000`;
GDB refuses `restore ... binary` into flash), read back byte-identical, `kill` to reset. The old
first sector is saved in the session scratchpad as `before_sector0.bin`.

**How to flash it from the application.** The Install Firmware page shows only while nothing is
connected (InitialSetup.cs:169-173), and the application connects to the settings' `comport` at
start-up, so the bench script `tests/gui/setup-firmware-flash-bench.gui` starts with no link, an
empty `config.xml` of its own, and `MP_FIRMWARE_PORT` (test scaffolding until MainV2's port box is
ported, PLAN §13.6 row 80) naming the board's by-id device. It runs only on Buzz's explicit go.

**Lessons.** `tools/gui-test.sh` deletes `$WORK` when the run ends, so the run's `.tlog` is gone
with it; to see a vehicle's `COMMAND_ACK` use `headless-planner command <url> <MAV_CMD> [p1..p7]` (development
scaffolding in the CLI). A GUI click on the SETUP list can land before the list is laid out
(run 4 of the flash: the page never activated); give the list a settle before clicking.

**The flash itself, 2026-09-25 13:52-13:55:** the application's Install Firmware page wrote
ArduCopter 4.7.1 stable for CubeOrange to the board over the stock bootloader - reboot into the
bootloader, scan, same-firmware check, erase, program, CRC verify, reboot: "Upload Done",
`config.firmware.flashed 140`. The one fix on the way: the identify's 50 ms read timeout has to
be raised before the CRC check (1 s in the C#, `Uploader.cs:812`) and the erase (20 s, `:535`),
or the board's CRC over 2 MB of flash reads as "lost communication".

**From Windows, 2026-09-27 (Buzz's go):** passed through to tiny10 by a VirtualBox USB filter on
vendor 2dae (both the application and `CubeOrange-BL` match); COM4 is MAVLink (MI_00), COM3 the
second CDC port. Two things the first attempts taught:
- It already runs the current stable (4.7.1, dbe79216), so after the scan the flow asks "The board
  already has the same firmware version. Upload anyway?" - the bench script answers Yes. The
  first attempt had no answer and sat on the question (the board held in its bootloader) until
  the runner closed the planner.
- It sends **two heartbeats**: the autopilot as 1:1 and an ADS-B receiver as 1:0 (type 27,
  autopilot invalid). MP's `doReboot(true,false)` takes whichever comes next and sends nothing to
  component 0, so half the flashes found no bootloader ("No Response from board"). Buzz ruled
  (2026-09-27): the flash's reboot skips component-0 heartbeats, as MP's own Connect does.
`python3 C:\setup\board-version.py COM4` in the VM reads its banner (pyserial installed there).

**Not this board: the MR-VMU-RT1176 on the host's /dev/ttyACM0** (ArduPilot, 27b1:0004) is Buzz's
arduzeph project's; another session flashes and reboots it all day (106 bootloader entries
2026-09-26/27). Its reboots are not ours - nothing here opens ttyACM0 - and nothing here should.

**Ruling (Buzz, 2026-09-25):** the "lost communication with the board." / "comms timeout" box is
never shown; the words go on the status line only. The link's state is always at the top right.

**Why this matters:** PLAN §13.6 row 79 (a real board flashed) is the owner's own test of the
firmware path; every attempt and its finding is recorded in that row.
