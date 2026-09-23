# Configuration panel coverage

Generated from `crates/mp-gui/src/config_coverage.rs` by `cargo test -p mp-gui config_coverage::tests::update_report -- --ignored`; a test fails when this file is stale. One row per panel in `GCSViews/ConfigurationView/` - every `Config*.cs` - in the order `GCSViews/InitialSetup.cs` (the SETUP button) and then `GCSViews/SoftwareConfig.cs` (the CONFIG button) first add it to their left-hand lists; panels neither list adds come last. Wirings are the events the panel's Designer wires, a measure of its size that undercounts a panel which builds its controls in code.

| panels | done | partial | missing | plumbing | dropped | wirings |
|---:|---:|---:|---:|---:|---:|---:|
| 61 | 0 | 9 | 46 | 2 | 4 | 569 |

| group | panels | done | partial | missing | plumbing | dropped | wirings | wirings in missing panels |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| SETUP, `InitialSetup.HardwareConfig_Load` | 44 | 0 | 8 | 34 | 2 | 0 | 258 | 198 |
| CONFIG, `SoftwareConfig.SoftwareConfig_Load` | 13 | 0 | 1 | 12 | 0 | 0 | 277 | 255 |
| neither list | 4 | 0 | 0 | 0 | 0 | 4 | 34 | 0 |

The lists also add 4 pages that are not in `ConfigurationView/` (`Sikradio`, `JoystickSetup`, `TrackerUI`, `MavFTPUI`): 0 done, 1 partial, 3 missing, 0 plumbing, 0 dropped. They are in the lists below and not in the counts above.

The largest missing panels, by wirings:

| panel | title | wirings |
|---|---|---:|
| `ConfigArducopter` | Extended Tuning | 128 |
| `ConfigPlanner` | Planner | 64 |
| `ConfigArduplane` | Basic Tuning | 47 |
| `ConfigSerialInjectGPS` | RTK/GPS Inject | 24 |
| `ConfigFirmware` | Install Firmware Legacy | 20 |
| `ConfigFrameClassType` | Frame Type | 19 |
| `ConfigFirmwareManifest` | Install Firmware | 16 |
| `ConfigDroneCAN` | DroneCAN/UAVCAN | 15 |
| `ConfigAdvanced` | Advanced | 13 |
| `ConfigBatteryMonitoring` | Battery Monitor | 13 |
| `ConfigFrameType` | Frame Type | 12 |
| `ConfigTerminal` | Terminal | 12 |

Vehicles: **any** is a connected vehicle whose parameter list is whole (`isConnected && gotAllParams`); **always** is connected or not; **connected** and **disconnected** are the link alone; a named vehicle, parameter or view is what the call, or the `if` around it, checks. **Advanced view** is `DisplayView.isAdvancedMode`. A page with a `DisplayView` switch also needs it on, which it is by default unless the vehicles say otherwise. The list shows a heading as `>> title` and indents what is under it (`ExtLibs/Controls/BackstageView/BackstageView.cs:227`, `:232`).

## SETUP - `GCSViews/InitialSetup.cs` `HardwareConfig_Load`

| line | page | title | under | vehicles | wirings | ours |
|---:|---|---|---|---|---:|---|
| 162 | `ConfigParamLoading` | Loading |  | connected, parameters still arriving | 2 | partial: `crates/mp-gui/src/params.rs` `fn browser_panel` - the parameter screen shows the download's progress and re-requests it, as Force Refresh does; nothing stands a Loading page in for either list while parameters arrive |
| 169 | `ConfigFirmwareDisabled` | Install Firmware |  | connected | 1 | **missing** |
| 171 | `ConfigFirmwareManifest` | Install Firmware |  | disconnected | 16 | **missing** |
| 173 | `ConfigFirmware` | Install Firmware Legacy |  | disconnected | 20 | **missing** |
| 178 | `ConfigSecureAP` | Secure |  | disconnected | 4 | **missing** |
| 182 | `ConfigMandatory` | Mandatory Hardware |  | any | 0 | plumbing: the Mandatory Hardware heading of the list: one sentence, no controls |
| 187 | `ConfigTradHeli4` | Heli Setup | Mandatory Hardware | heli | 0 | **missing** |
| 188 | `ConfigFrameType` | Frame Type | Mandatory Hardware | copter before 3.5 | 12 | **missing** |
| 189 | `ConfigFrameClassType` | Frame Type | Mandatory Hardware | any with FRAME_CLASS; copter 3.5 and later | 19 | **missing** |
| 196 | `ConfigAccelerometerCalibration` | Accel Calibration | Mandatory Hardware | any | 3 | partial: `crates/mp-gui/src/setup.rs` `fn accelerometer_panel` - has Calibrate Accel's six positions, and Calibrate Level as `cal-level` in the calibration panel; missing Simple Accel Cal |
| 203 | `ConfigHWCompass2` | Compass | Mandatory Hardware | any with COMPASS_PRIO1_ID | 11 | partial: `crates/mp-gui/src/setup.rs` `fn compass_panel` - has the onboard calibration's start and cancel with each compass's progress; missing Accept, the priority grid (order, use, external, remove missing), Large Vehicle MagCal and Reboot |
| 206 | `ConfigHWCompass` | Compass | Mandatory Hardware | any without COMPASS_PRIO1_ID | 21 | partial: `crates/mp-gui/src/setup.rs` `fn compass_panel` - has the onboard calibration's start and cancel; missing declination, each compass's use and external boxes, the board presets, live calibration, compass learn and Large Vehicle MagCal |
| 211 | `ConfigRadioInput` | Radio Calibration | Mandatory Hardware | any | 8 | partial: `crates/mp-gui/src/setup.rs` `fn radio_panel` - has the live channel bars and Calibrate Radio's capture of each channel's range, written as RCn_MIN and RCn_MAX; missing RCn_TRIM, the four Reverse boxes, the Spektrum bind buttons and the plane's elevon boxes |
| 215 | `ConfigRadioOutput` | Servo Output | Mandatory Hardware | any | 1 | **missing** |
| 220 | `ConfigSerial` | Serial Ports | Mandatory Hardware | any | 0 | **missing** |
| 224 | `ConfigESCCalibration` | ESC Calibration | Mandatory Hardware | any | 1 | **missing** |
| 228 | `ConfigFlightModes` | Flight Modes | Mandatory Hardware | any | 8 | partial: `crates/mp-gui/src/config/flight_modes.rs` `fn page` - the six combos from the firmware's mode list, the lit PWM band, Simple and Super Simple, Save through the retrying set; not Ctrl+S, standardFlightModesOnly beyond its default, nor the message box |
| 232 | `ConfigFailSafe` | FailSafe | Mandatory Hardware | any | 4 | partial: `crates/mp-gui/src/config/failsafe.rs` `fn page` - the channel bars, the mode/armed/GPS readouts, the throttle, battery and GCS controls writing their parameters on change through the retrying set; numbers by step arrows only, no typing |
| 237 | `ConfigInitialParams` | Initial Tune Parameter | Mandatory Hardware | copter, quadplane | 3 | **missing** |
| 241 | `ConfigHWIDs` | HW ID | Mandatory Hardware | any | 0 | **missing** |
| 243 | `ConfigOptional` | Optional Hardware |  | always | 0 | plumbing: the Optional Hardware heading of the list: one sentence, no controls |
| 251 | `ConfigSerialInjectGPS` | RTK/GPS Inject | Optional Hardware | always | 24 | **missing** |
| 254 | `ConfigCubeID` | CubeID Update | Optional Hardware | connected | 2 | **missing** |
| 259 | `Sikradio` (`Radio/Sikradio.cs`, not a panel) | Sik Radio | Optional Hardware | always | 17 | **missing** |
| 263 | `ConfigADSB` | ADSB | Mandatory Hardware | any | 5 | **missing** |
| 266 | `ConfigGPSOrder` | CAN GPS Order | Optional Hardware | any | 1 | **missing** |
| 270 | `ConfigBatteryMonitoring` | Battery Monitor | Optional Hardware | any | 13 | **missing** |
| 271 | `ConfigBatteryMonitoring2` | Battery Monitor 2 | Optional Hardware | any | 10 | **missing** |
| 276 | `ConfigDroneCAN` | DroneCAN/UAVCAN | Optional Hardware | always | 15 | **missing** |
| 280 | `JoystickSetup` (`Joystick/JoystickSetup.cs`, not a panel) | Joystick | Optional Hardware | always | 11 | partial: `crates/mp-gui/src/joystick.rs` `fn panel_for` - has the device list and Enable; missing the per-channel axis grid, the button functions, Elevons, Save, Manual Control, Import and Export |
| 285 | `ConfigCompassMot` | Compass/Motor Calib | Optional Hardware | any | 2 | **missing** |
| 289 | `ConfigHWRangeFinder` | Range Finder | Optional Hardware | any | 2 | **missing** |
| 293 | `ConfigHWAirspeed` | Airspeed | Optional Hardware | any | 1 | **missing** |
| 297 | `ConfigHWPX4Flow` | PX4Flow | Optional Hardware | always | 1 | **missing** |
| 301 | `ConfigHWOptFlow` | Optical Flow | Optional Hardware | any | 2 | **missing** |
| 305 | `ConfigHWOSD` | OSD | Optional Hardware | any | 1 | **missing** |
| 309 | `ConfigMount` | Camera Gimbal | Optional Hardware | any | 5 | **missing** |
| 313 | `ConfigAntennaTracker` | Antenna tracker | Optional Hardware | tracker | 3 | **missing** |
| 317 | `ConfigMotorTest` | Motor Test | Optional Hardware | any | 3 | partial: `crates/mp-gui/src/setup.rs` `fn motor_panel` - has a button per motor, the throttle and Stop all motors, over MAV_CMD_DO_MOTOR_TEST; missing Test all motors, Test all in Sequence, the duration box, the motor count, letters and rotation from the frame layout, the MOT_SPIN_ARM and MOT_SPIN_MIN setters and the motor-order link |
| 321 | `ConfigHWBT` | Bluetooth Setup | Optional Hardware | always | 1 | **missing** |
| 325 | `ConfigHWParachute` | Parachute | Optional Hardware | any | 1 | **missing** |
| 329 | `ConfigHWESP8266` (`ConfigHWesp8266.cs`) | ESP8266 Setup | Optional Hardware | any | 3 | **missing** |
| 333 | `TrackerUI` (`Antenna/TrackerUI.cs`, not a panel) | Antenna Tracker | Optional Hardware | always | 0 | **missing** |
| 337 | `ConfigFFT` | FFT Setup | Optional Hardware | any | no Designer | **missing** |
| 342 | `ConfigAdvanced` | Advanced |  | always, Advanced view | 13 | **missing** |
| 346 | `ConfigTerminal` | Terminal | Advanced | always, Advanced view | 12 | **missing** |
| 351 | `ConfigREPL` | Script REPL | Advanced | connected, Advanced view | 4 | **missing** |

## CONFIG - `GCSViews/SoftwareConfig.cs` `SoftwareConfig_Load`

| line | page | title | under | vehicles | wirings | ours |
|---:|---|---|---|---|---:|---|
| 156 | `ConfigAC_Fence` | GeoFence |  | copter | 0 | **missing** |
| 164 | `ConfigSimplePids` | Basic Tuning |  | copter | 1 | **missing** |
| 169 | `ConfigArducopter` | Extended Tuning |  | copter | 128 | **missing** |
| 177 | `ConfigArduplane` | Basic Tuning |  | plane | 47 | **missing** |
| 182 | `ConfigArducopter` | QP Extended Tuning |  | plane (enabled for a quadplane) | 128 | as at `GCSViews/SoftwareConfig.cs:169` |
| 188 | `ConfigArdurover` | Basic Tuning |  | rover | 3 | **missing** |
| 193 | `ConfigAntennaTracker` | Extended Tuning |  | tracker | 3 | as at `GCSViews/InitialSetup.cs:313` |
| 198 | `ConfigFriendlyParams` | Standard Params |  | any, Custom view with Standard Params | 1 | **missing** |
| 203 | `ConfigFriendlyParamsAdv` | Advanced Params |  | any, Custom view with Advanced Params and Advanced mode | no Designer | **missing** |
| 208 | `ConfigOSD` | Onboard OSD |  | any with OSD parameters, not on Mono | 0 | **missing** |
| 215 | `MavFTPUI` (`Controls/MavFTPUI.cs`, not a panel) | MAVFtp |  | any reporting MAVLink FTP | 16 | **missing** |
| 221 | `ConfigUserDefined` | User Params |  | any | 0 | **missing** |
| 229 | `ConfigRawParams` | Full Parameter List |  | any, or disconnected | 22 | partial: `crates/mp-gui/src/params.rs` `fn list_panel` - the parameter screen has Refresh Params, Search, the group tree, editing a value, Save to file, Compare Params, and Load from file as compare then apply; missing Reset to Default, Load Presaved and its file list, Commit Params, the Modified and None Default filters, Refresh Table and the tree's collapse |
| 235 | `ConfigFlightModes` | Flight Modes |  | Ateryx | 8 | as at `GCSViews/InitialSetup.cs:228` |
| 236 | `ConfigAteryxSensors` | Ateryx Zero Sensors |  | Ateryx | 3 | **missing** |
| 237 | `ConfigAteryx` | Ateryx Pids |  | Ateryx | 8 | **missing** |
| 243 | `ConfigParamLoading` | Loading |  | connected, parameters still arriving | 2 | as at `GCSViews/InitialSetup.cs:162` |
| 245 | `ConfigParamLoading` | Loading |  | connected, parameters still arriving | 2 | as at `GCSViews/InitialSetup.cs:162` |
| 250 | `ConfigPlanner` | Planner |  | connected | 64 | **missing** |
| 257 | `ConfigPlanner` | Planner |  | disconnected | 64 | as at `GCSViews/SoftwareConfig.cs:250` |

## Neither list

| page | wirings | ours |
|---|---:|---|
| `ConfigHWCAN` | 6 | dropped: Mission Planner never shows it: its entry is commented out at InitialSetup.cs:275, beside ConfigDroneCAN's |
| `ConfigPlannerAdv` | 0 | dropped: Mission Planner never shows it: nothing lists or opens it |
| `ConfigSecure` | 6 | dropped: Mission Planner never shows it: nothing lists or opens it; the Secure page is ConfigSecureAP |
| `ConfigTradHeli` | 22 | dropped: Mission Planner never shows it: its entry is commented out at InitialSetup.cs:186; Heli Setup is ConfigTradHeli4 |
