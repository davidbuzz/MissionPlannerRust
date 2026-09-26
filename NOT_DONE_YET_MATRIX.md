| Area or filename | Functional item | Priority | Progress |
|---|---|---|---|
| crates/mp-transport/src/win32.rs | Windows board detection: the port list takes SetupAPI's hardware id and bus-reported name as Win32DeviceMgmt reads them - Install Firmware found no board on Windows (owner's bug report 2026-09-26); committed 9a51796, the bench CubeOrange detected in the VM as board id 140 | high | 100 |
| dist/ | Linux release app rebuilt at today's main (owner's ask): built 2026-09-26 14:25 at 962cd28, thin LTO and no debuginfo to fit beside the VM (fat LTO needs the VM off) | high | 100 |
| crates/mp-gui/src/plotline.rs | Logs > PLOT drew a string of dots (owner's bug report 2026-09-26): every ZedGraph curve - the log browser's, the tuning graph's, the FFT screen's - as a line through its points, clipped, the FFT's diamonds; log-browse.gui passed headless (ATT.Roll one line of 182 points) | high | 100 |
| crates/mp-gui/src/glyph_text.rs | Plan > Text said "Error" on Windows: the font looked up with fontconfig, which Windows has not got; now the installed font's file or GDI+'s Microsoft Sans Serif; plan-text.gui passed in the VM 2026-09-26 | high | 100 |


| crates/mp-script | Scripts handed MAV, MainV2, screens, Ports, Joystick: merged 2026-09-26 with the 19 review findings fixed (getWP and setWPTotal as the C#'s link requests, a dropped link and Abort end a waiting script, the named vehicle throughout) and a second review's four; fly-scripts-clr.gui passed headless, the SITL test on a settled SITL. Owed: MAV.wps and the rally list a script's setWP would fill (not kept outside the mission transfer) | med | 90 |
| crates/mp-script | Fourteen shipped scripts that import clr run: 10 reach their end or loop; merged with the row above | med | 70 |
| crates/mp-gui/src/plan.rs | GeoFence upload and download menu items: merged with the review's fixes (MISSION_FENCE bit 16384, Geo-Fence and Rally hidden over such a vehicle, return marker, link drop); its five scripts passed headless 2026-09-26 against tests/gui/legacy-vehicle.py and the SITL | med | 100 |
| crates/mp-gui/src/config/firmware.rs | Install Firmware connected page: Bootloader Update flow merged 2026-09-26; the bench bootloader rewrite from the application on the owner's go | med | 85 |
| crates/mp-gui/src/config/firmware.rs | Install Firmware manifest page: Ctrl+Q, FirmwareSelection's pickers, the bootloader probe on a port's arrival, Force Bootloader, Bootloader Update, the stale "flashing is not enabled" texts gone (the owner's Windows report) - merged 2026-09-26 with the review's fixes (no serial port opened while MP_FIRMWARE_DEVICE names the device; the found board forgotten with the SETUP screen; a port back as another device an arrival; no probe around a flash left running; a lost link said). config-firmware, config-firmware-legacy and config-firmware-blupdate passed in the VM too. Owed: the legacy page's own "flashing is not enabled" text | med | 95 |
| crates/mp-gui/src/joystick.rs | Joystick Setup: merged 2026-09-26 with the review's fixes (no frame before the device is first read - Enable sent mid throttle; Import's names as ZipArchiveEntry.Name; stale button presses dropped; nonsense RC ranges wrap); config-joystick.gui passed headless. Owed: the Windows device reader (JoystickWindows.cs), so Windows lists no joystick; a dropped reader's thread waits for its device's next event to end (a blocking read has no safe interrupt here) | med | 85 |
| Windows bench | The bench CubeOrange passed through to the VM (done 2026-09-26) and detected there (board id 140); a flash from Windows on the owner's go | med | 50 |
| crates/mp-video | Video capture on Windows: the C#'s DirectShow capture (Video Device, Video Format, Start/Stop on the Planner page, the HUD's camera) is not ported - V4L2 only, so Windows lists no camera; config-video.gui skips there. COM and `unsafe` (the owner's call); a camera passed through to the VM to test it | med | 1 |
| crates/mp-video/src/gstreamer.rs | On Windows the HUD's and the gimbal's GStreamer pipelines gave no frames in the suite (fly-gstreamer, fly-gimbal-video): not the planner - GStreamer's first-run plugin scan outlasts a script's wait in the VM and a launcher stopped mid-scan writes no registry. The Windows runner builds the registry once, before the scripts; both passed from an empty registry 2026-09-27 | med | 100 |
| crates/mp-gui/src/setup.rs | On Windows a click on the SETUP list in the first frames after the Serial page opens was lost one run in three (setup-list.gui probed 2026-09-26: 1 of 3 without a pause, 0 of 3 with one); the script now waits for the page's rows. Why the debug build drops it is not known | low | 10 |
| crates/mp-gui/src/config/advanced.rs | Advanced page: twelve buttons open windows not ported | med | 30 |
| crates/mp-gui/src/config/friendly_params.rs | Standard Params page remainder | med | 80 |
| crates/mp-gui/src/config/friendly_params.rs | Advanced Params page remainder | med | 80 |
| crates/mp-gui/src/config/mavftp.rs | MAVFtp page: remaining wirings of 16 | med | 60 |
| crates/mp-gui/src/config | Sik Radio page (Radio/Sikradio.cs, 17 wirings) | med | 1 |
| crates/mp-gui/src/config | DroneCAN/UAVCAN page (15 wirings) | med | 1 |
| crates/mp-gui/src/config | Onboard OSD page body (ConfigOSD) | med | 5 |
| crates/mp-gui/src/config/compass_mot.rs | Owner's two open questions on firmware-refusal boxes | med | 90 |
| crates/mp-log | Log analysis (loganalysis) reachable from the DataFlash Logs page | med | 60 |
| crates/mp-gui/src/sitl | SITL screen: a real launch on macOS (Linux and Windows done) | med | 95 |
| DELIVERABLES D20 | Installers, updates, crash reporting | med | 5 |
| DELIVERABLES D19 | Per-message decoder fuzzing; macOS graphics smoke; the Windows GUI suite against a Linux headless run at one commit - done 2026-09-26 at 42c3ea8 (143 pass on both, no Windows-only failure a planner defect), then the known issues fixed on the owner's word: the VM given Python 3 with pymavlink, the GStreamer 1.28.7 runtime, the wasm32 target and a 1640x1320 screen, Windows Defender's prompt off by policy; per-platform `linux:`/`windows:` lines in both runners (the SITL launcher, CRLF file sizes); the Python stand-ins into the background on Windows (socket.share); sitl-launch given port 5760; PATH, cargo, rustup and one GStreamer registry kept for the scripts. Of the Windows run's 17 failures and 14 skips, all pass now but the two GStreamer scripts (the row below), config-video (skips: capture not ported), config-compass-livecal and storm (skip by design). Owed: a whole Windows run to confirm, fuzzing, macOS | med | 90 |

| crates/mp-gui/src/fly.rs | Record HUD to AVI, and stop recording | low | 1 |
| crates/mp-gui/src/fly.rs | RAW_Sensor window from the Actions grid | low | 1 |
| crates/mp-gui/src/fly.rs | Camera overlap toggle: CAMERA_FEEDBACK photo markers on map | low | 1 |
| crates/mp-gui/src/hud | HUD latency figure owed a measured run | low | 90 |
| crates/mp-gui/src/plan.rs | Rotate map menu item | low | 1 |
| crates/mp-gui/src/plan.rs | GDAL opacity menu item and GDAL overlays | low | 1 |
| crates/mp-gui/src/plan.rs | Inject custom map button | low | 1 |
| crates/mp-gui/src/plan.rs | KML link (lnk_kml) | low | 1 |
| crates/mp-gui/src/config/firmware_legacy.rs | Install Firmware Legacy: remaining wirings of 20 | low | 70 |
| crates/mp-gui/src/config/adsb.rs | ADSB page: remaining wirings of 5 | low | 70 |
| crates/mp-gui/src/config/optical_flow.rs | Optical Flow: remaining wirings | low | 70 |
| crates/mp-gui/src/config/user_params.rs | User Params page remainder | low | 80 |
| crates/mp-gui/src/config | Antenna Tracker page (Antenna/TrackerUI.cs) | low | 1 |
| crates/mp-gui/src/config | Mandatory and Optional Hardware heading panels' text | low | 50 |
| crates/mp-log | ulog reading | low | 1 |
| tools/sitl/wasm | Node bridge from the WASM SITL's SERIAL0 to tcp:5760 | low | 40 |
| CI, macOS | First macOS run of the application | low | 5 |
| DELIVERABLES D18 | Translation factory beyond the ledger | low | 30 |
| DELIVERABLES D1 | Crate splits the plan names (mp-log, mp-geo, mp-map, mp-ui...) | low | 55 |
| crates/mp-gui (row 73) | mp-ui, the widget facade crate | low | 30 |
| DELIVERABLES D15 | Swarm, HIL and speech | low | 5 |

| crates/mp-gui/src/fly.rs | Set aspect ratio 4:3 (owner's call) | not | 1 |
| DELIVERABLES D21 | Native in-process plugin host | not | 1 |
| DELIVERABLES D17, assets/i18n | i18n: cultures loaded at run time (muted by ruling) | not | 50 |
| DELIVERABLES D3 | BLE transport (not in the C#) | not | 1 |
