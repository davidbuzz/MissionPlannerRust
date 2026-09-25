# Configuration panel coverage

Generated from `crates/mp-gui/src/config_coverage.rs` by `cargo test -p mp-gui
        // config_coverage::tests::update_report -- --ignored`; a test fails when this file is
        // stale. One
        // row per panel in `GCSViews/ConfigurationView/` - every `Config*.cs` - in the order
        // `GCSViews/InitialSetup.cs` (the SETUP button) and then `GCSViews/SoftwareConfig.cs` (the
        // CONFIG button) first add it to their left-hand lists; panels neither list adds come
        // last. Wirings are the events the panel's Designer wires, a measure of its size that
        // undercounts a panel which builds its controls in code.

| panels | done | partial | missing | plumbing | dropped | wirings |
|---:|---:|---:|---:|---:|---:|---:|
| 61 | 32 | 13 | 2 | 2 | 12 | 569 |

| group | panels | done | partial | missing | plumbing | dropped | wirings | wirings
        // in missing panels |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| SETUP, `InitialSetup.HardwareConfig_Load` | 44 | 26 | 8 | 1 | 2 | 7 | 258 |
        //     15 |
| CONFIG, `SoftwareConfig.SoftwareConfig_Load` | 13 | 6 | 5 | 1 | 0 | 1 | 277 |
        //     0 |
| neither list | 4 | 0 | 0 | 0 | 0 | 4 | 34 |
        //     0 |

The lists also add 4 pages that are not in `ConfigurationView/` (`Sikradio`, `JoystickSetup`, `TrackerUI`, `MavFTPUI`): 0 done,
        // 2 partial, 2 missing, 0 plumbing, 0 dropped. They are
        // in the lists below and not in the counts above.

The largest missing panels, by wirings:

| panel | title | wirings |
|---|---|---:|
| `ConfigDroneCAN` | DroneCAN/UAVCAN | 15 |
| `ConfigOSD` | Onboard OSD | 0 |

Vehicles: **any** is a connected vehicle whose parameter list is whole
        // (`isConnected && gotAllParams`); **always** is connected or not; **connected** and
        // **disconnected** are the link alone; a named vehicle, parameter or view is what the
        // call, or the `if` around it, checks. **Advanced view** is `DisplayView.isAdvancedMode`.
        // A page with a `DisplayView` switch also needs it on, which it is by default unless the
        // vehicles say otherwise. The list shows a heading as `>> title` and indents what is
        // under it (`ExtLibs/Controls/BackstageView/BackstageView.cs:227`, `:232`).

## SETUP - `GCSViews/InitialSetup.cs` `HardwareConfig_Load`

| line | page | title | under | vehicles | wirings | ours |
|---:|---|---|---|---|---:|---|
| 162 | `ConfigParamLoading` | Loading |  | connected, parameters still arriving | 2 | done: `crates/mp-gui/src/setup.rs` `fn param_loading_page` |
| 169 | `ConfigFirmwareDisabled` | Install Firmware |  | connected | 1 | partial: `crates/mp-gui/src/config/firmware.rs` `fn page` - the connected page's text; Bootloader Update asks its two questions and stops
        //     before MAV_CMD_FLASH_BOOTLOADER, which rewrites the board's bootloader - nothing
        //     flashes in this build |
| 171 | `ConfigFirmwareManifest` | Install Firmware |  | disconnected | 16 | partial: `crates/mp-gui/src/config/firmware.rs` `fn page` - the catalogue fetched as APFirmware.GetList fetches it, each vehicle labelled with
        //     the newest firmware of the release, Beta; a vehicle's click asks, runs LookForPort,
        //     opens FirmwareSelection on the board's platform and downloads the file chosen with
        //     the progress bar and status line, and UploadFlash reads it; All Options over the
        //     whole catalogue; Load custom firmware by extension; each stops where it would write
        //     to a board - the upload, DFU, Force Bootloader and Bootloader Update are disabled, as
        //     nothing flashes in this build; not Ctrl+Q, the bootloader probe on a device's
        //     arrival, nor FirmwareSelection's filter pickers; the vehicle pictures are the C#'s
        //     images (crate::pictures) |
| 173 | `ConfigFirmware` | Install Firmware Legacy |  | disconnected | 20 | partial: `crates/mp-gui/src/config/firmware_legacy.rs` `fn page` - every control at its .resx place; the firmware2.xml list loaded behind its progress
        //     dialog with each entry's git-version.txt, and each picture labelled and tagged as
        //     updateDisplayName does; a vehicle's click asks, detects the board from the device
        //     list, chooses the entry's URL for it (CubeBlack, ChibiOS), downloads firmware.hex
        //     and reads it; Pick previous firmware from FirmwareHistory.txt, Beta firmwares, Load
        //     custom firmware, and the three links; each stops where it would write to a board -
        //     the upload and Force Bootloader are disabled, as nothing flashes in this build; not
        //     Ctrl+Q or Ctrl+P, nor the bootloader probe on a device's arrival; the pictures are
        //     the C#'s images (crate::pictures) |
| 178 | `ConfigSecureAP` | Secure |  | disconnected | 4 | done: `crates/mp-gui/src/config/secure.rs` `fn page` |
| 182 | `ConfigMandatory` | Mandatory Hardware |  | any | 0 | plumbing: the Mandatory Hardware heading of the list: one sentence, no controls |
| 187 | `ConfigTradHeli4` | Heli Setup | Mandatory Hardware | heli | 0 | done: `crates/mp-gui/src/config/trad_heli.rs` `fn page` |
| 188 | `ConfigFrameType` | Frame Type | Mandatory Hardware | copter before 3.5 | 12 | done: `crates/mp-gui/src/config/frame_type_legacy.rs` `fn page` |
| 189 | `ConfigFrameClassType` | Frame Type | Mandatory Hardware | any with FRAME_CLASS; copter 3.5 and later | 19 | done: `crates/mp-gui/src/config/frame_type.rs` `fn page` |
| 196 | `ConfigAccelerometerCalibration` | Accel Calibration | Mandatory Hardware | any | 3 | done: `crates/mp-gui/src/config/accel_calibration.rs` `fn page` |
| 203 | `ConfigHWCompass2` | Compass | Mandatory Hardware | any with COMPASS_PRIO1_ID | 11 | done: `crates/mp-gui/src/config/compass.rs` `fn page` |
| 206 | `ConfigHWCompass` | Compass | Mandatory Hardware | any without COMPASS_PRIO1_ID | 21 | partial: `crates/mp-gui/src/config/compass.rs` `fn page` - has the declination and its automatic box, learn, the primary compass, each
        //     compass's use, external, orientation, offsets and MOT, the three quick-configure
        //     buttons, the onboard calibration with its timer and fitness, Large Vehicle MagCal
        //     and both links - 20 of the 21 wirings; missing Live Calibration, drawn and
        //     disabled: its handler is MagCalib.DoGUIMagCalib (MagCalib.cs), Mission Planner's
        //     own calibration from RAW_IMU and SCALED_IMU2/3 samples - the ProgressReporterSphere
        //     window with three OpenGL spheres, the sphere and ellipsoid fits (ported, in
        //     mp_calibration::magcalib), and the offsets saved through
        //     PREFLIGHT_SET_SENSOR_OFFSETS - a feature of its own, its window not ported; its
        //     group shows only for ArduPlane 3.7.1 to 4.0 or a vehicle without onboard
        //     calibration. No Log Calibration button: BUT_MagCalibrationLog_Click has no caller
        //     (PLAN §12 D16) |
| 211 | `ConfigRadioInput` | Radio Calibration | Mandatory Hardware | any | 8 | done: `crates/mp-gui/src/config/radio.rs` `fn page` |
| 215 | `ConfigRadioOutput` | Servo Output | Mandatory Hardware | any | 1 | done: `crates/mp-gui/src/config/servo_output.rs` `fn page` |
| 220 | `ConfigSerial` | Serial Ports | Mandatory Hardware | any | 0 | done: `crates/mp-gui/src/config/serial_ports.rs` `fn page` |
| 224 | `ConfigESCCalibration` | ESC Calibration | Mandatory Hardware | any | 1 | done: `crates/mp-gui/src/config/esc_calibration.rs` `fn page` |
| 228 | `ConfigFlightModes` | Flight Modes | Mandatory Hardware | any | 8 | done: `crates/mp-gui/src/config/flight_modes.rs` `fn page` |
| 232 | `ConfigFailSafe` | FailSafe | Mandatory Hardware | any | 4 | done: `crates/mp-gui/src/config/failsafe.rs` `fn page` |
| 237 | `ConfigInitialParams` | Initial Tune Parameter | Mandatory Hardware | copter, quadplane | 3 | done: `crates/mp-gui/src/config/initial_params.rs` `fn page` |
| 241 | `ConfigHWIDs` | HW ID | Mandatory Hardware | any | 0 | done: `crates/mp-gui/src/config/hw_ids.rs` `fn page` |
| 243 | `ConfigOptional` | Optional Hardware |  | always | 0 | plumbing: the Optional Hardware heading of the list: one sentence, no controls |
| 251 | `ConfigSerialInjectGPS` | RTK/GPS Inject | Optional Hardware | always | 24 | partial: `crates/mp-gui/src/config/rtk_inject.rs` `fn page` - every control and handler, the read loop, the RTCM/SBP/UBX/NMEA parsers,
        //     GPS_RTCM_DATA and GPS_INJECT_DATA injection, cs.Base, the .gpsbase log, the u-blox,
        //     Septentrio and Unicore set-up, the base positions; not DroneCAN over SLCAN
        //     (ExtLibs/DroneCAN is not ported) nor the Windows named-pipe fallback
        //     (CommsSerialPipe) |
| 254 | `ConfigCubeID` | CubeID Update | Optional Hardware | connected | 2 | dropped: ruled out of the port by the owner, 2026-09-25 (PLAN §12 D13) |
| 259 | `Sikradio` (`Radio/Sikradio.cs`, not a panel) | Sik Radio | Optional Hardware | always | 17 | **missing** |
| 263 | `ConfigADSB` | ADSB | Mandatory Hardware | any | 5 | partial: `crates/mp-gui/src/config/adsb.rs` `fn page` - a RangeControl, bitmask or ValuesControl per documented ADSB_/AVD_ parameter,
        //     favourites first, recording changes; Write Params writing them ENABLE-first, each
        //     in its own try, then "Parameters successfully saved."; Refresh Params with
        //     MessageShowAgain, its unticked box kept as SHOWAGAIN_Refresh_Params; Find
        //     filtering as typed, its OK's word kept as InputBox keeps it; a bitmask updated on Activate writing as the C#'s does; missing Ctrl+S,
        //     dragging the track bar (a click pages it) and typing into a ValuesControl |
| 266 | `ConfigGPSOrder` | CAN GPS Order | Optional Hardware | any | 1 | done: `crates/mp-gui/src/config/gps_order.rs` `fn page` |
| 270 | `ConfigBatteryMonitoring` | Battery Monitor | Optional Hardware | any | 13 | done: `crates/mp-gui/src/config/battery_monitor.rs` `fn page` |
| 271 | `ConfigBatteryMonitoring2` | Battery Monitor 2 | Optional Hardware | any | 10 | done: `crates/mp-gui/src/config/battery_monitor2.rs` `fn page` |
| 276 | `ConfigDroneCAN` | DroneCAN/UAVCAN | Optional Hardware | always | 15 | **missing** |
| 280 | `JoystickSetup` (`Joystick/JoystickSetup.cs`, not a panel) | Joystick | Optional Hardware | always | 11 | partial: `crates/mp-gui/src/joystick.rs` `fn panel_for` - has the device list and Enable; missing the per-channel axis grid, the button
        //     functions, Elevons, Save, Manual Control, Import and Export |
| 285 | `ConfigCompassMot` | Compass/Motor Calib | Optional Hardware | any | 2 | done: `crates/mp-gui/src/config/compass_mot.rs` `fn page` |
| 289 | `ConfigHWRangeFinder` | Range Finder | Optional Hardware | any | 2 | done: `crates/mp-gui/src/config/rangefinder.rs` `fn page` |
| 293 | `ConfigHWAirspeed` | Airspeed | Optional Hardware | any | 1 | done: `crates/mp-gui/src/config/airspeed.rs` `fn page` |
| 297 | `ConfigHWPX4Flow` | PX4Flow | Optional Hardware | always | 1 | dropped: ruled out of the port by the owner, 2026-09-25 (PLAN §12 D13) |
| 301 | `ConfigHWOptFlow` | Optical Flow | Optional Hardware | any | 2 | partial: `crates/mp-gui/src/config/optical_flow.rs` `fn page` - the legacy FLOW_ENABLE page or the new-style one: FLOW_TYPE, the yaw in degrees,
        //     the scalers and positions writing 300 ms after a change, the rover's height
        //     override shown by the type's handler, the sensor picture (crate::pictures); a yaw
        //     below -179 degrees is kept rather than written back as the C#'s Minimum does |
| 305 | `ConfigHWOSD` | OSD | Optional Hardware | any | 1 | done: `crates/mp-gui/src/config/osd.rs` `fn page` |
| 309 | `ConfigMount` | Camera Gimbal | Optional Hardware | any | 5 | done: `crates/mp-gui/src/config/mount.rs` `fn page` |
| 313 | `ConfigAntennaTracker` | Antenna tracker | Optional Hardware | tracker | 3 | dropped: ruled out of the port by the owner, 2026-09-25 (PLAN §12 D13) |
| 317 | `ConfigMotorTest` | Motor Test | Optional Hardware | any | 3 | done: `crates/mp-gui/src/config/motor_test.rs` `fn page` |
| 321 | `ConfigHWBT` | Bluetooth Setup | Optional Hardware | always | 1 | dropped: ruled out of the port by the owner, 2026-09-25 (PLAN §12 D13) |
| 325 | `ConfigHWParachute` | Parachute | Optional Hardware | any | 1 | done: `crates/mp-gui/src/config/parachute.rs` `fn page` |
| 329 | `ConfigHWESP8266` (`ConfigHWesp8266.cs`) | ESP8266 Setup | Optional Hardware | any | 3 | dropped: ruled out of the port by the owner, 2026-09-25 (PLAN §12 D13) |
| 333 | `TrackerUI` (`Antenna/TrackerUI.cs`, not a panel) | Antenna Tracker | Optional Hardware | always | 0 | **missing** |
| 337 | `ConfigFFT` | FFT Setup | Optional Hardware | any | no Designer | done: `crates/mp-gui/src/config/fft.rs` `fn page` |
| 342 | `ConfigAdvanced` | Advanced |  | always, Advanced view | 13 | partial: `crates/mp-gui/src/config/advanced.rs` `fn page` - the text and the thirteen buttons with their labels at the table's places; FFT
        //     opens the FFT window (config/fftui.rs), 1 of the 13 wirings; the other twelve
        //     dimmed - the Warnings Manager, MAVLink Inspector, proximity, signing keys, MAVLink
        //     mirror, NMEA output, Follow Me, parameter regeneration, moving base, log
        //     anonymiser, spectrogram and support proxy windows they open are not ported |
| 346 | `ConfigTerminal` | Terminal | Advanced | always, Advanced view | 12 | dropped: ruled out of the port by the owner, 2026-09-25 (PLAN §12 D13) |
| 351 | `ConfigREPL` | Script REPL | Advanced | connected, Advanced view | 4 | dropped: ruled out of the port by the owner, 2026-09-25 (PLAN §12 D13) |

## CONFIG - `GCSViews/SoftwareConfig.cs` `SoftwareConfig_Load`

| line | page | title | under | vehicles | wirings | ours |
|---:|---|---|---|---|---:|---|
| 156 | `ConfigAC_Fence` | GeoFence |  | copter | 0 | done: `crates/mp-gui/src/config/geofence.rs` `fn page` |
| 164 | `ConfigSimplePids` | Basic Tuning |  | copter | 1 | done: `crates/mp-gui/src/config/simple_pids.rs` `fn page` |
| 169 | `ConfigArducopter` | Extended Tuning |  | copter | 128 | done: `crates/mp-gui/src/config/extended_tuning.rs` `fn page` |
| 177 | `ConfigArduplane` | Basic Tuning |  | plane | 47 | done: `crates/mp-gui/src/config/basic_tuning.rs` `fn page` |
| 182 | `ConfigArducopter` | QP Extended Tuning |  | plane (enabled for a quadplane) | 128 | as at `GCSViews/SoftwareConfig.cs:169` |
| 188 | `ConfigArdurover` | Basic Tuning |  | rover | 3 | done: `crates/mp-gui/src/config/rover_tuning.rs` `fn page` |
| 193 | `ConfigAntennaTracker` | Extended Tuning |  | tracker | 3 | as at `GCSViews/InitialSetup.cs:313` |
| 198 | `ConfigFriendlyParams` | Standard Params |  | any, Custom view with Standard Params | 1 | partial: `crates/mp-gui/src/config/friendly_params.rs` `fn page` - a RangeControl, bitmask or ValuesControl per parameter documented @User: Standard
        //     with a display name, fav_params first; changes recorded; Write Params ENABLE-first,
        //     each in its own try, then "Parameters successfully saved."; Refresh Params with
        //     MessageShowAgain, its unticked box kept as SHOWAGAIN_Refresh_Params; Find
        //     filtering as typed, its OK's word kept as InputBox keeps it; the flow panel's
        //     layout; a failed write or fetch on the status line (the owner's ruling); missing
        //     Ctrl+S, dragging the track bar (a click pages it) and typing into a ValuesControl |
| 203 | `ConfigFriendlyParamsAdv` | Advanced Params |  | any, Custom view with Advanced Params and Advanced mode | no Designer | partial: `crates/mp-gui/src/config/friendly_params.rs` `fn page` - Standard Params' page over the parameters documented @User: Advanced or not at all;
        //     missing what it misses |
| 208 | `ConfigOSD` | Onboard OSD |  | any with OSD parameters, not on Mono | 0 | **missing** |
| 215 | `MavFTPUI` (`Controls/MavFTPUI.cs`, not a panel) | MAVFtp |  | any reporting MAVLink FTP | 16 | partial: `crates/mp-gui/src/config/mavftp.rs` `fn page` - the tree of / and @SYS/ with @SYS selected and listed, a node's listing replacing its
        //     children, the list's double click and column sort; Download Burst, Download,
        //     Upload with its CRC check, Delete, Rename in place, New Folder and GetCRC32 behind
        //     the progress window with its Cancel and the sessions' reset; dropped files uploaded;
        //     the status strip's reports every 100 ms; Mount as Drive failing as the C# does
        //     without Dokan, on the status line; the link's failures there too (the owner's
        //     rulings); missing
        //     the tree's and list's keys, several files in one Upload (the path is typed: there
        //     is no platform dialog) and dragging the column headers |
| 221 | `ConfigUserDefined` | User Params |  | any | 0 | partial: `crates/mp-gui/src/config/user_params.rs` `fn page` - the UserParams list (or the C#'s 22 RC option names), a row for each name the
        //     vehicle has with a combo of its documented values writing it, Modify's multiline
        //     InputBox saving the list and building the page again - Cancel included, as the
        //     C#'s handler ignores the answer; a name without values is its label alone, as the
        //     C# never adds its number; the InputBox's OK answer kept as
        //     InputBoxParamsEnterParamNames |
| 229 | `ConfigRawParams` | Full Parameter List |  | any, or disconnected | 22 | partial: `crates/mp-gui/src/params.rs` `fn list_panel` - the parameter screen has Refresh Params (MAVFTP first), Search, the group tree and
        //     its collapse (kept in rawparam_panel1collapsed), the Default column and the None
        //     Default filter when the vehicle's param.pck gave defaults, the Modified filter
        //     over _changes (the writes not yet heard back, and those that timed out - a value
        //     is written as it is edited), editing a value, Save to file, Compare Params, Load
        //     from file as compare then apply, Load Presaved with its GitHub Frame_params list
        //     and ParamCompare, Reset to Default, Commit Params (under
        //     displayParamCommitButton) and Refresh Table (under SlowMachine) - raw_params.rs;
        //     the grid's Name, Value, Default, Units, Options, Desc and Fav columns at the .resx
        //     widths, the Fav column kept in fav_params and the grid sorted favourites first in
        //     NaturalStringComparer's order, the Options cell's text and the control Params_RowEnter
        //     puts over it (Set Bitmask's window, the values' drop-down, the range's
        //     NumericUpDown), the Desc cell's link, the typed Value cell (double click, F2 or a
        //     key) with Params_CellValueChanged's REV fix, red cell, ReadOnly box and out-of-range
        //     question, Ctrl+S as Write Params over _changes with its question and its ends, the
        //     columns' widths and the splitter kept under rawparam_* on Deactivate and read on
        //     Activate (both draggable), and RawParamWarning's MessageShowAgain box -
        //     raw_params_grid.rs; missing the cells' tooltips (AddNewLinesForTooltip), the
        //     headers' click sorting (a text column's automatic SortMode through
        //     OnParamsOnSortCompare), mXparser's expressions in a typed value (read as a number
        //     here), typing into the NumericUpDown's own box, the questions on a file's or a
        //     presaved file's values (the C#'s ParamCompare puts them through the grid's
        //     CellValueChanged), ReadOnly from anything but the fetched apm.pdef.xml (the C#'s
        //     SITL, AP_Periph and ParameterMetaData.xml fallbacks), and the Options cell's range
        //     as the file writes it (here from its two numbers) |
| 235 | `ConfigFlightModes` | Flight Modes |  | Ateryx | 8 | as at `GCSViews/InitialSetup.cs:228` |
| 236 | `ConfigAteryxSensors` | Ateryx Zero Sensors |  | Ateryx | 3 | dropped: ruled out of the port by the owner, 2026-09-25 (PLAN §12 D13) |
| 237 | `ConfigAteryx` | Ateryx Pids |  | Ateryx | 8 | done: `crates/mp-gui/src/config/ateryx.rs` `fn page` |
| 243 | `ConfigParamLoading` | Loading |  | connected, parameters still arriving | 2 | as at `GCSViews/InitialSetup.cs:162` |
| 245 | `ConfigParamLoading` | Loading |  | connected, parameters still arriving | 2 | as at `GCSViews/InitialSetup.cs:162` |
| 250 | `ConfigPlanner` | Planner |  | connected | 64 | partial: `crates/mp-gui/src/config/planner.rs` `fn planner_page` - every control at its place, each bound to the Settings key its handler writes; the
        //     units (ChangeUnits), the telemetry rates and their stream requests, the speech boxes
        //     and their InputBox templates (each OK's answer kept as InputBox keeps it), Load
        //     Waypoints on connect, the map access mode, Joystick Setup, Browse and Open Map
        //     Cache act at once; Layout picks the display
        //     view the lists read (Basic, Advanced, or the Custom file), saved as displayview;
        //     Video Device and Video Format list the V4L2 capture devices and their MJPEG and
        //     YUYV formats, Start captures (mp-video) and the HUD draws the frame under
        //     everything, Stop ends it - Linux only, DirectShow on Windows not yet; Enable HUD
        //     Overlay sets hudon, the picture alone when unticked; dimmed for
        //     want of what they drive: GDI+, language, theme, OSD colour, Vario,
        //     password, the ADSB server, analytics, beta updates, MAVLink debug and the testing
        //     screen; the flight screen does not yet read the units, the track length, the map's
        //     rotation or the icon settings, nor the link the GCS id or the rates on connecting |
| 257 | `ConfigPlanner` | Planner |  | disconnected | 64 | as at `GCSViews/SoftwareConfig.cs:250` |

## Neither list

| page | wirings | ours |
|---|---:|---|
| `ConfigHWCAN` | 6 | dropped: Mission Planner never shows it: its entry is commented out at InitialSetup.cs:275,
        //     beside ConfigDroneCAN's |
| `ConfigPlannerAdv` | 0 | dropped: Mission Planner never shows it: nothing lists or opens it |
| `ConfigSecure` | 6 | dropped: Mission Planner never shows it: nothing lists or opens it; the Secure page is
        //     ConfigSecureAP |
| `ConfigTradHeli` | 22 | dropped: Mission Planner never shows it: its entry is commented out at InitialSetup.cs:186;
        //     Heli Setup is ConfigTradHeli4 |
