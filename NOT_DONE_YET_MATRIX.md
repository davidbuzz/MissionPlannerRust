| Area or filename | Functional item | Priority | Progress |
|---|---|---|---|
| crates/mp-transport/src/win32.rs | Windows board detection: the port list takes SetupAPI's hardware id and bus-reported name as Win32DeviceMgmt reads them - Install Firmware found no board on Windows (owner's bug report 2026-09-26); code written, Windows test owed in the VM | high | 60 |
| dist/ | Linux release app rebuilt at today's main (owner's ask): built 2026-09-26 14:25 at 962cd28, thin LTO and no debuginfo to fit beside the VM (fat LTO needs the VM off) | high | 100 |
| crates/mp-gui/src/plotline.rs | Logs > PLOT drew a string of dots (owner's bug report 2026-09-26): every ZedGraph curve - the log browser's, the tuning graph's, the FFT screen's - as a line through its points, clipped, the FFT's diamonds; log-browse.gui passed headless (ATT.Roll one line of 182 points) | high | 100 |
| crates/mp-gui/src/glyph_text.rs | Plan > Text said "Error" on Windows: the font looked up with fontconfig, which Windows has not got; now the installed font's file or GDI+'s Microsoft Sans Serif; plan-text.gui owed a Windows run | high | 80 |


| crates/mp-script | Scripts handed MAV, MainV2, screens, Ports, Joystick: agent's patch in its worktree, 19 review findings to fix before merge (getWP a whole-mission download, setWPTotal without resends, a hang when the link drops) | med | 50 |
| crates/mp-script | Fourteen shipped scripts that import clr run: 10 reach their end or loop with the patch; merged with the row above | med | 50 |
| crates/mp-gui/src/plan.rs | GeoFence upload and download menu items: merged with the review's fixes (MISSION_FENCE bit 16384, Geo-Fence and Rally hidden over such a vehicle, return marker, link drop); its scripts against tests/gui/legacy-vehicle.py and the SITL owed a run | med | 90 |
| crates/mp-gui/src/config/firmware.rs | Install Firmware connected page: Bootloader Update flow; the bench bootloader rewrite on the owner's go (agent working, continuing a stopped agent's worktree) | med | 60 |
| crates/mp-gui/src/config/firmware.rs | Install Firmware manifest page: Ctrl+Q, FirmwareSelection's pickers, the bootloader probe, Force Bootloader, Bootloader Update, the stale "flashing is not enabled" texts - reported on Windows by the owner (agent working, continuing a stopped agent's worktree) | med | 70 |
| crates/mp-gui/src/joystick.rs | Joystick Setup: merged 2026-09-26 with the review's fixes (no frame before the device is first read - Enable sent mid throttle; Import's names as ZipArchiveEntry.Name; stale button presses dropped; nonsense RC ranges wrap); config-joystick.gui passed headless. Owed: the Windows device reader (JoystickWindows.cs), so Windows lists no joystick; a dropped reader's thread waits for its device's next event to end (a blocking read has no safe interrupt here) | med | 85 |
| Windows bench | The bench CubeOrange passed through to the VM (done 2026-09-26): detection checked once win32.rs lands, then a flash from Windows on the owner's go | med | 30 |
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
| DELIVERABLES D19 | Per-message decoder fuzzing; macOS graphics smoke; the Windows GUI suite against a Linux headless run at one commit - first whole run 2026-09-26 at the VM's eb472ed: 121 pass, 5 over budget, 34 fail, 10 skip of 170; the runner's SITL now one for the suite behind tools/win10/sitl-relay.ps1 (the Cygwin build exits when its client leaves - measured), settled 30 s, 13 of the 33 then passed; Windows budgets measured into tools/win10/budgets.txt by the suite (owner: "successful test run time + 3 sec"); owed: the VM at main and rebuilt, a whole run, the Linux headless run at the same commit, the VM's screen taller than 1200 for 1600x1200 windows (owner's call) | med | 75 |

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
