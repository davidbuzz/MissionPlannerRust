pub mod accel_calibration;
pub mod battery_monitor;
pub mod compass;
// Live Calibration, the older Compass page's `MagCalib.DoGUIMagCalib`.
pub mod live_magcal;
pub mod default_settings;
pub mod esc_calibration;
pub mod failsafe;
pub mod firmware;
// Force Bootloader, which both Install Firmware pages share.
pub mod force_bootloader;
pub mod flight_modes;
pub mod frame_type;
pub mod frame_type_legacy;
pub mod motor_test;
pub mod planner;
pub mod radio;
pub mod secure;
pub mod serial_ports;
pub mod servo_output;
// The Optional Hardware pages (and ADSB), ported together: `optional.rs` holds what they share.
pub mod adsb;
pub mod airspeed;
pub mod battery_monitor2;
pub mod mount;
pub mod optical_flow;
pub mod optional;
pub mod rangefinder;
// ---- Basic Tuning / Advanced ----
pub mod advanced;
pub mod basic_tuning;
pub mod mavlink_inspector;
pub mod warnings_manager;
// ---- end Basic Tuning / Advanced ----
// ---- Extended Tuning ----
pub mod extended_tuning;
// ---- RTK/GPS Inject ----
pub mod rtk_inject;
// ---- end RTK/GPS Inject ----
// ---- Firmware Legacy / Ateryx ----
pub mod ateryx;
pub mod firmware_legacy;
// ---- end Firmware Legacy / Ateryx ----
// ---- GeoFence / rover Basic Tuning / User Params ----
pub mod geofence;
pub mod rover_tuning;
pub mod simple_pids;
pub mod software_pages;
pub mod user_params;
// ---- end GeoFence / rover Basic Tuning / User Params ----
// ---- SETUP's small pages (PLAN §13.6 row 70) ----
pub mod compass_mot;
pub mod extra_setup;
pub mod gps_order;
pub mod hw_ids;
pub mod initial_params;
pub mod osd;
pub mod parachute;
pub mod param_compare;
// ---- end SETUP's small pages ----
// ---- Standard / Advanced Params, MAVFtp, Heli Setup (row 71) ----
pub mod friendly_params;
pub mod mavftp;
pub mod software_pages2;
pub mod trad_heli;
// ---- end Standard / Advanced Params, MAVFtp, Heli Setup ----
// ---- FFT Setup ----
pub mod fft;
pub mod fftui;
// ---- end FFT Setup ----
