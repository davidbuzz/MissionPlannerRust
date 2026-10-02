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
| 61 | 39 | 8 | 0 | 2 | 12 | 569 |

| group | panels | done | partial | missing | plumbing | dropped | wirings | wirings
        // in missing panels |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| SETUP, `InitialSetup.HardwareConfig_Load` | 44 | 29 | 6 | 0 | 2 | 7 | 258 |
        //     0 |
| CONFIG, `SoftwareConfig.SoftwareConfig_Load` | 13 | 10 | 2 | 0 | 0 | 1 | 277 |
        //     0 |
| neither list | 4 | 0 | 0 | 0 | 0 | 4 | 34 |
        //     0 |

The lists also add 4 pages that are not in `ConfigurationView/` (`Sikradio`, `JoystickSetup`, `TrackerUI`, `MavFTPUI`): 2 done,
        // 1 partial, 1 missing, 0 plumbing, 0 dropped. They are
        // in the lists below and not in the counts above.

The largest missing panels, by wirings:

| panel | title | wirings |
|---|---|---:|

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
| 169 | `ConfigFirmwareDisabled` | Install Firmware |  | connected | 1 | done: `crates/mp-gui/src/config/firmware.rs` `fn disabled_page` |
| 171 | `ConfigFirmwareManifest` | Install Firmware |  | disconnected | 16 | partial: `crates/mp-gui/src/config/firmware.rs` `fn page` - the catalogue fetched as APFirmware.GetList fetches it, each vehicle labelled with
        //     the newest firmware of the release, Beta, and DEV after Ctrl+Q's warning; a device
        //     arriving probed for its bootloader on every port, its board id taken by LookForPort;
        //     a vehicle's click asks, runs LookForPort, opens FirmwareSelection - its pickers
        //     narrowing the list from the board's platform, Ignore undoing each - downloads the
        //     file chosen with the progress bar and status line, and UploadFlash reboots the board
        //     into its bootloader and writes it; All Options over the whole catalogue; Load custom
        //     firmware by extension; Force Bootloader over the window's link; Bootloader Update
        //     over a link of its own; the vehicle pictures are the C#'s images (crate::pictures);
        //     not DFU, the STK500 upload and probes, VRBRAIN, Parrot or Solo, which stop where
        //     they would write to a board and say that step is not ported, nor Tracking's
        //     AddFW and AddTiming |
| 173 | `ConfigFirmware` | Install Firmware Legacy |  | disconnected | 20 | partial: `crates/mp-gui/src/config/firmware_legacy.rs` `fn page` - every control at its .resx place; the firmware2.xml list loaded behind its progress
        //     dialog with each entry's git-version.txt, and each picture labelled and tagged as
        //     updateDisplayName does; a vehicle's click asks, detects the board from the device
        //     list, chooses the entry's URL for it (CubeBlack, ChibiOS), downloads firmware.hex
        //     and reads it; Pick previous firmware from FirmwareHistory.txt, Beta firmwares, Load
        //     custom firmware, and the three links; Force Bootloader over the window's link, the
        //     manifest page's handler shared (config/force_bootloader.rs); each flow stops where
        //     it would write to a board - this page's upload is not ported; not Ctrl+Q or Ctrl+P,
        //     nor the bootloader probe on a device's arrival; the pictures are the C#'s images
        //     (crate::pictures) |
| 178 | `ConfigSecureAP` | Secure |  | disconnected | 4 | done: `crates/mp-gui/src/config/secure.rs` `fn page` |
| 182 | `ConfigMandatory` | Mandatory Hardware |  | any | 0 | plumbing: the Mandatory Hardware heading of the list: one sentence, no controls |
| 187 | `ConfigTradHeli4` | Heli Setup | Mandatory Hardware | heli | 0 | done: `crates/mp-gui/src/config/trad_heli.rs` `fn page` |
| 188 | `ConfigFrameType` | Frame Type | Mandatory Hardware | copter before 3.5 | 12 | done: `crates/mp-gui/src/config/frame_type_legacy.rs` `fn page` |
| 189 | `ConfigFrameClassType` | Frame Type | Mandatory Hardware | any with FRAME_CLASS; copter 3.5 and later | 19 | done: `crates/mp-gui/src/config/frame_type.rs` `fn page` |
| 196 | `ConfigAccelerometerCalibration` | Accel Calibration | Mandatory Hardware | any | 3 | done: `crates/mp-gui/src/config/accel_calibration.rs` `fn page` |
| 203 | `ConfigHWCompass2` | Compass | Mandatory Hardware | any with COMPASS_PRIO1_ID | 11 | done: `crates/mp-gui/src/config/compass.rs` `fn page` |
| 206 | `ConfigHWCompass` | Compass | Mandatory Hardware | any without COMPASS_PRIO1_ID | 21 | done: `crates/mp-gui/src/config/compass.rs` `fn page` |
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
        //     Septentrio and Unicore set-up, the base positions; config-rtk.gui passes against
        //     its own caster; not DroneCAN over SLCAN (ExtLibs/DroneCAN is not ported - the
        //     DroneCAN page's row); CommsSerialPipe, the C#'s Win32 CreateFile fallback for a
        //     port name .NET's SerialPort refuses, is not needed: SerialTransport opens those
        //     names first time (rtk_inject.rs's module note) |
| 254 | `ConfigCubeID` | CubeID Update | Optional Hardware | connected | 2 | dropped: ruled out of the port by the owner, 2026-09-25 (PLAN §12 D13) |
| 259 | `Sikradio` (`Radio/Sikradio.cs`, not a panel) | Sik Radio | Optional Hardware | always | 17 | done: `crates/mp-gui/src/config/sikradio.rs` `fn page` |
| 263 | `ConfigADSB` | ADSB | Mandatory Hardware | any | 5 | done: `crates/mp-gui/src/config/adsb.rs` `fn page` |
| 266 | `ConfigGPSOrder` | CAN GPS Order | Optional Hardware | any | 1 | done: `crates/mp-gui/src/config/gps_order.rs` `fn page` |
| 270 | `ConfigBatteryMonitoring` | Battery Monitor | Optional Hardware | any | 13 | done: `crates/mp-gui/src/config/battery_monitor.rs` `fn page` |
| 271 | `ConfigBatteryMonitoring2` | Battery Monitor 2 | Optional Hardware | any | 10 | done: `crates/mp-gui/src/config/battery_monitor2.rs` `fn page` |
| 276 | `ConfigDroneCAN` | DroneCAN/UAVCAN | Optional Hardware | always | 15 | partial: `crates/mp-gui/src/config/dronecan.rs` `fn page` - the interface list, Connect over MAVLinkCAN1/2 (CAN_FORWARD each second, CAN_FRAME
        //     both ways), SLCAN (the CPORT/TIMOUT/DRIVER writes, the link's port taken, the
        //     adapter opened) and MCastCan1/2; node 127 heard by itself; the node grid, details and
        //     debug grid; the node menu's Parameters (the parameter window), Restart, Update and
        //     Update Beta (CubePilot's server, the manifest, a .bin or .apj), both passthroughs;
        //     Filter, Stats, the Inspector (every message heard field by field, Graph It, the
        //     Subscriber), Check for Updates, Log, Exit SLCAN; not the parameter window's Compare
        //     Params and Reset to Default, which act on the autopilot from the node's window
        //     (for the owner's ruling) |
| 280 | `JoystickSetup` (`Joystick/JoystickSetup.cs`, not a panel) | Joystick | Optional Hardware | always | 11 | partial: `crates/mp-gui/src/joystick.rs` `fn load` - all of the page - the device list, the sixteen channel rows with axis, Auto Detect,
        //     bar, expo and reverse, the button rows with number, Detect, bar, function and the
        //     seven Joy_* settings forms, Enable, Save, Elevons, Manual Control, Export and Import,
        //     each button function sending what ProcessButtonEvent sends - flying a Linux js
        //     device; missing: a device on Windows, where mp-input has no reader for DirectInput
        //     (the C#'s JoystickWindows.cs), so the list is empty there |
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
| 342 | `ConfigAdvanced` | Advanced |  | always, Advanced view | 13 | partial: `crates/mp-gui/src/config/advanced.rs` `fn page` - the text and the thirteen buttons with their labels at the table's places; Warning
        //     Manager opens the manager (config/warnings_manager.rs: a row per rule and per
        //     child over the engine's rules, warnings.rs, which checks them every 250 ms and
        //     raises the HUD's message or a quick view's colour), Proximity the proximity window
        //     (config/proximity.rs: the readings from DISTANCE_SENSOR and OBSTACLE_DISTANCE
        //     drawn as Temp_Paint draws them, its keys), Mavlink Signing the keys window
        //     (config/auth_keys.rs: the store authkeys.xml as Crypto.cs encrypts it, Add with
        //     the strength score, Use and Disable Signing driving the link's setupSigning), FFT
        //     opens the FFT window (config/fftui.rs) and MAVLink Inspector the inspector
        //     (config/mavlink_inspector.rs: the tree of each system, component, message with its
        //     rate and bytes a second, and field with its value and .NET type, every 333 ms; Show
        //     GCS Traffic; Graph It's history question and live graph of a field), Spectrogram the
        //     spectrogram window (config/spectrogram.rs: a log's IMU or ISBH samples drawn as
        //     Spectrogram.cs's GenerateImage draws them), Support Proxy the proxy
        //     (config/support_proxy.rs: the link mirrored to a support engineer's server over
        //     TCP or UDP, mp-link's mirror.rs), Mavlink Mirror the grid of outputs
        //     (config/mavlink_mirror.rs: each row a mirror of the link started by Go, kept in the
        //     serialpasslist setting), NMEA the NMEA output (config/nmea_output.rs: GGA, GLL,
        //     HDG, VTG, RMC and RPY to a port at the rate, the geoid from mp-terrain) and Param
        //     gen the run behind Downloading updated data (config/param_gen.rs:
        //     ParameterMetaDataParser over every location's Parameters.cpp and its groups'
        //     files, ParameterMetaData.xml written in the user data directory), 10 of the 13
        //     wirings; the other three dimmed - the Follow Me, moving base and log anonymiser
        //     windows they open are ruled out of scope |
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
| 198 | `ConfigFriendlyParams` | Standard Params |  | any, Custom view with Standard Params | 1 | done: `crates/mp-gui/src/config/friendly_params.rs` `fn page` |
| 203 | `ConfigFriendlyParamsAdv` | Advanced Params |  | any, Custom view with Advanced Params and Advanced mode | no Designer | done: `crates/mp-gui/src/config/friendly_params.rs` `fn page` |
| 208 | `ConfigOSD` | Onboard OSD |  | any with OSD parameters, not on Mono | 0 | done: `crates/mp-gui/src/config/onboard_osd_ui.rs` `fn page` |
| 215 | `MavFTPUI` (`Controls/MavFTPUI.cs`, not a panel) | MAVFtp |  | any reporting MAVLink FTP | 16 | done: `crates/mp-gui/src/config/mavftp.rs` `fn page` |
| 221 | `ConfigUserDefined` | User Params |  | any | 0 | partial: `crates/mp-gui/src/config/user_params.rs` `fn page` - the UserParams list (or the C#'s 22 RC option names), a row for each name the
        //     vehicle has with a combo of its documented values writing it, Modify's multiline
        //     InputBox saving the list and building the page again - Cancel included, as the
        //     C#'s handler ignores the answer; a name without values is its label alone, as the
        //     C# never adds its number; the InputBox's OK answer kept as
        //     InputBoxParamsEnterParamNames |
| 229 | `ConfigRawParams` | Full Parameter List |  | any, or disconnected | 22 | done: `crates/mp-gui/src/params.rs` `fn list_panel` |
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
