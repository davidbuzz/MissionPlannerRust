pub mod accel_calibration;
pub mod battery_monitor;
pub mod compass;
pub mod esc_calibration;
pub mod failsafe;
pub mod firmware;
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
