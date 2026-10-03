// Copyright (C) 2026 David "Buzz" Bussenschutt
//
// This file is part of MissionPlannerRust, a Rust implementation derived from
// Mission Planner (Copyright (C) 2010-2024 Michael Oborne and contributors,
// https://github.com/ArduPilot/MissionPlanner); NOTICE records the changes.
//
// MissionPlannerRust is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by the
// Free Software Foundation, version 3 of the License.
//
// MissionPlannerRust is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License along with
// MissionPlannerRust. If not, see <https://www.gnu.org/licenses/>.
//
// SPDX-License-Identifier: GPL-3.0-only

//! HW ID: `GCSViews/ConfigurationView/ConfigHWIDs.cs`, a Mandatory Hardware page of Initial Setup
//! (`GCSViews/InitialSetup.cs:240-241`), listed once every parameter is in.
//!
//! What it shows: one read-only grid filling the page, a row per parameter whose name contains
//! `_ID` or `_DEVID` but neither `_IDX` nor `FRSKY`, ordered by name, each device id taken apart
//! as `DeviceInfo` takes it: ParamName, DevID, BusType, Bus, Address and DevType
//! (`ConfigHWIDs.cs:16-31`, `DeviceInfo.cs:6-51`, `ExtLibs/Utilities/Device.cs:16-225`). The
//! test is on the name alone, so `MOT_IDLE_SEC` and `SIM_MAG_SAVE_IDS` are rows too, as they are
//! in Mission Planner. `Activate` builds the list each time the page is shown; the page is always
//! enabled (it disables itself without a link and enables itself at once).
//!
//! DevType is the name of the device type in the enum the parameter's name picks - `SENSOR_ID#`
//! and the number on a DroneCAN bus; else the compass's for a name with `COMP`, the barometer's for
//! `BARO`, the airspeed sensor's for `ASP`, the IMU's for anything else - less `DEVTYPE_`, or the
//! number for a value the enum does not name, as `Enum.ToString` writes it.
//!
//! The order is `OrderBy(ParamName)`: `Comparer<string>.Default`, a culture comparison, which puts
//! `INS_ACC_ID` before `INS_ACC2_ID` ([`mp_log::netfmt::culture_compare`]).
//!
//! The layout is the Designer's: the grid docked to fill the 856 x 496 page, its row headers at
//! their default 41 pixels, ParamName 150 wide and the rest at the default 100, scrolling when the
//! rows do not fit.
//!
//! What is not ported, and why: `AllowUserToOrderColumns`, dragging a column header to another
//! place - a way of looking at the table that changes nothing, and gpui has no header drag of its
//! own. The C#'s rows cannot be sorted either: a `BindingSource` over a `List<T>` does not support
//! sorting, so a header click does nothing, here as there.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use gpui::{AnyElement, Context, div, prelude::*, px, rgb};

use super::compass::{address, bus, bus_type_name, compass_dev_type, devtype};
use crate::MissionPlanner;
use crate::setup::Key;
use crate::telemetry::TelemetryView;
use crate::ui::{panel, theme};

/// The page's title in Initial Setup's list, the literal `InitialSetup` passes.
/// `// C#: GCSViews/InitialSetup.cs:241`
pub const TITLE: &str = "HW ID";

/// The column headers, in the Designer's order, and their widths.
/// `// C#: GCSViews/ConfigurationView/ConfigHWIDs.Designer.cs:66-114`
pub const COLUMNS: [(&str, f32); 6] = [
    ("ParamName", 150.0),
    ("DevID", 100.0),
    ("BusType", 100.0),
    ("Bus", 100.0),
    ("Address", 100.0),
    ("DevType", 100.0),
];

/// `$this.Size`, which the grid fills.
/// `// C#: GCSViews/ConfigurationView/ConfigHWIDs.Designer.cs:120`
const PAGE_SIZE: (f32, f32) = (856.0, 496.0);
/// `DataGridView.RowHeadersWidth`'s default.
const ROW_HEADER: f32 = 41.0;
/// The header row's height, `AutoSize` for one line.
const HEADER_HEIGHT: f32 = 23.0;
/// A row's, `DataGridView`'s default template.
const ROW_HEIGHT: f32 = 22.0;

/// One row: `DeviceInfo`.
/// `// C#: GCSViews/ConfigurationView/DeviceInfo.cs:6-51`
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DeviceRow {
    /// `ParamName`.
    pub param: String,
    /// `DevID`: `(int)devid`.
    pub dev_id: i32,
    /// `BusType`.
    pub bus_type: String,
    /// `Bus`.
    pub bus: u32,
    /// `Address`.
    pub address: u32,
    /// `DevType`.
    pub dev_type: String,
}

/// `imu_types`, by value.
/// `// C#: ExtLibs/Utilities/Device.cs:130-171`
const IMU_TYPES: [(u32, &str); 38] = [
    (0x09, "BMI160"),
    (0x10, "L3G4200D"),
    (0x11, "ACC_LSM303D"),
    (0x12, "ACC_BMA180"),
    (0x13, "ACC_MPU6000"),
    (0x16, "ACC_MPU9250"),
    (0x17, "ACC_IIS328DQ"),
    (0x18, "ACC_LSM9DS1"),
    (0x21, "GYR_MPU6000"),
    (0x22, "GYR_L3GD20"),
    (0x24, "GYR_MPU9250"),
    (0x25, "GYR_I3G4250D"),
    (0x26, "GYR_LSM9DS1"),
    (0x27, "INS_ICM20789"),
    (0x28, "INS_ICM20689"),
    (0x29, "INS_BMI055"),
    (0x2A, "SITL"),
    (0x2B, "INS_BMI088"),
    (0x2C, "INS_ICM20948"),
    (0x2D, "INS_ICM20648"),
    (0x2E, "INS_ICM20649"),
    (0x2F, "INS_ICM20602"),
    (0x30, "INS_ICM20601"),
    (0x31, "INS_ADIS1647X"),
    (0x32, "SERIAL"),
    (0x33, "INS_ICM40609"),
    (0x34, "INS_ICM42688"),
    (0x35, "INS_ICM42605"),
    (0x36, "INS_ICM40605"),
    (0x37, "INS_IIM42652"),
    (0x38, "BMI270"),
    (0x39, "INS_BMI085"),
    (0x3A, "INS_ICM42670"),
    (0x3B, "INS_ICM45686"),
    (0x3C, "INS_SCHA63T"),
    (0x3D, "INS_IIM42653"),
    (0x3E, "INS_LSM6DSV"),
    (0x3F, "INS_ASM330"),
];

/// `baro_types`, by value.
/// `// C#: ExtLibs/Utilities/Device.cs:174-200`
const BARO_TYPES: [(u32, &str); 24] = [
    (0x01, "BARO_SITL"),
    (0x02, "BARO_BMP085"),
    (0x03, "BARO_BMP280"),
    (0x04, "BARO_BMP388"),
    (0x05, "BARO_DPS280"),
    (0x06, "BARO_DPS310"),
    (0x07, "BARO_FBM320"),
    (0x08, "BARO_ICM20789"),
    (0x09, "BARO_KELLERLD"),
    (0x0A, "BARO_LPS2XH"),
    (0x0B, "BARO_MS5611"),
    (0x0C, "BARO_SPL06"),
    (0x0D, "BARO_DRONECAN"),
    (0x0E, "BARO_MSP"),
    (0x0F, "BARO_ICP101XX"),
    (0x10, "BARO_ICP201XX"),
    (0x11, "BARO_MS5607"),
    (0x12, "BARO_MS5837_30BA"),
    (0x13, "BARO_MS5637"),
    (0x14, "BARO_BMP390"),
    (0x15, "BARO_BMP581"),
    (0x16, "BARO_SPA06"),
    (0x17, "BARO_AUAV"),
    (0x18, "BARO_MS5837_02BA"),
];

/// `airspeed_types`, by value.
/// `// C#: ExtLibs/Utilities/Device.cs:203-215`
const AIRSPEED_TYPES: [(u32, &str); 10] = [
    (0x01, "AIRSPEED_SITL"),
    (0x02, "AIRSPEED_MS4525"),
    (0x03, "AIRSPEED_MS5525"),
    (0x04, "AIRSPEED_DLVR"),
    (0x05, "AIRSPEED_MSP"),
    (0x06, "AIRSPEED_SDP3X"),
    (0x07, "AIRSPEED_DRONECAN"),
    (0x08, "AIRSPEED_ANALOG"),
    (0x09, "AIRSPEED_NMEA"),
    (0x0A, "AIRSPEED_ASP5033"),
];

/// `Enum.ToString()` less `DEVTYPE_`: the name, or the number for a value the enum lacks.
fn enum_name(table: &[(u32, &str)], value: u32) -> String {
    table
        .iter()
        .find(|(held, _)| *held == value)
        .map_or_else(|| value.to_string(), |(_, name)| (*name).to_owned())
}

/// `DeviceInfo.DevType`: by the bus, then the name.
/// `// C#: GCSViews/ConfigurationView/DeviceInfo.cs:27-45`
#[must_use]
pub fn dev_type(param: &str, devid: u32) -> String {
    if devid & 0x7 == 3 || param.contains("COMP") {
        // `compass_dev_type` takes the DroneCAN branch first, as `DevType` does.
        return compass_dev_type(devid);
    }
    let kind = devtype(devid);
    if param.contains("BARO") {
        enum_name(&BARO_TYPES, kind)
    } else if param.contains("ASP") {
        enum_name(&AIRSPEED_TYPES, kind)
    } else {
        enum_name(&IMU_TYPES, kind)
    }
}

/// `(uint)a.Value`: the double to a 64-bit integer, towards zero, and its low 32 bits - what the
/// x86-64 JIT's conversion leaves for a value outside `uint`, a negative one among them.
fn to_uint(value: f64) -> u32 {
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let low = value as i64 as u32;
    low
}

/// `Activate`'s list: the names, filtered and ordered as the C# filters and orders them.
/// `// C#: GCSViews/ConfigurationView/ConfigHWIDs.cs:24-26`
#[must_use]
pub fn rows(parameters: &[(String, f64)]) -> Vec<DeviceRow> {
    let mut rows: Vec<DeviceRow> = parameters
        .iter()
        .filter(|(name, _)| {
            (name.contains("_ID") || name.contains("_DEVID"))
                && !name.contains("_IDX")
                && !name.contains("FRSKY")
        })
        .map(|(name, value)| {
            let devid = to_uint(*value);
            DeviceRow {
                param: name.clone(),
                // `(int)_devid.devid`: unchecked, so an id past `int` wraps.
                #[allow(clippy::cast_possible_wrap)]
                dev_id: devid as i32,
                bus_type: bus_type_name(devid),
                bus: bus(devid),
                address: address(devid),
                dev_type: dev_type(name, devid),
            }
        })
        .collect();
    // `OrderBy` is stable; the names are unique anyway.
    rows.sort_by(|a, b| mp_log::netfmt::culture_compare(&a.param, &b.param));
    rows
}

/// The page object.
#[derive(Debug, Default)]
pub struct HwIds {
    made_for: Option<Key>,
    active: bool,
    /// The grid's rows.
    rows: Vec<DeviceRow>,
}

impl HwIds {
    /// Whether the page is showing.
    #[must_use]
    pub const fn is_active(&self) -> bool {
        self.active
    }

    /// The grid's rows.
    #[must_use]
    pub fn rows(&self) -> &[DeviceRow] {
        &self.rows
    }

    /// Shows the page: a new page object for a new screen, then `Activate`, which builds the
    /// list anew.
    /// `// C#: GCSViews/ConfigurationView/ConfigHWIDs.cs:16-31`
    pub fn activate(&mut self, parameters: &[(String, f64)], key: Key) {
        if self.made_for != Some(key) {
            *self = Self {
                made_for: Some(key),
                ..Self::default()
            };
        }
        self.active = true;
        self.rows = rows(parameters);
    }

    /// The page hidden. `ConfigHWIDs` is `IActivate` only.
    pub fn hide(&mut self) {
        self.active = false;
    }

    /// Once a frame: a page object whose screen has gone is let go.
    pub fn tick(&mut self, view: &TelemetryView, on_setup: bool) {
        if !self.active
            && self.made_for.is_some()
            && (!on_setup || self.made_for != Some(Key::of(view)))
        {
            *self = Self::default();
        }
    }
}

/// Facts a UI test asserts on.
pub fn record_facts(page: &HwIds) {
    use crate::facts::record;
    record("config.hwids.active", page.is_active());
    record(
        "config.hwids.columns",
        COLUMNS
            .iter()
            .map(|(name, _)| *name)
            .collect::<Vec<_>>()
            .join(","),
    );
    record("config.hwids.rows", page.rows().len());
    record(
        "config.hwids.order",
        page.rows()
            .iter()
            .map(|row| row.param.as_str())
            .collect::<Vec<_>>()
            .join(","),
    );
    for row in page.rows() {
        record(
            format!("config.hwids.row.{}", row.param),
            format!(
                "{},{},{},{},{}",
                row.dev_id, row.bus_type, row.bus, row.address, row.dev_type
            ),
        );
    }
}

/// One cell of the grid.
fn cell(width: f32, height: f32, text: String, header: bool) -> gpui::Div {
    div()
        .flex_shrink_0()
        .w(px(width))
        .h(px(height))
        .flex()
        .items_center()
        .px_1()
        .border_r_1()
        .border_b_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(if header { theme::ACTION } else { theme::BG }))
        .text_xs()
        .overflow_hidden()
        .whitespace_nowrap()
        .text_color(rgb(theme::TEXT))
        .child(text)
}

/// The page: the grid, filling the Designer's page.
pub fn page(ids: &HwIds, _cx: &mut Context<MissionPlanner>) -> AnyElement {
    if !ids.is_active() {
        return div().into_any_element();
    }
    let mut header =
        div()
            .flex()
            .flex_shrink_0()
            .child(cell(ROW_HEADER, HEADER_HEIGHT, String::new(), true));
    for (name, width) in COLUMNS {
        header = header.child(cell(width, HEADER_HEIGHT, name.to_owned(), true));
    }
    let mut grid = crate::probe::measured("hwids-grid", div())
        .id("hwids-grid")
        .flex()
        .flex_col()
        .w(px(PAGE_SIZE.0))
        .h(px(PAGE_SIZE.1))
        .overflow_y_scroll()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::PANEL))
        .child(header);
    for row in ids.rows() {
        let texts = [
            row.param.clone(),
            row.dev_id.to_string(),
            row.bus_type.clone(),
            row.bus.to_string(),
            row.address.to_string(),
            row.dev_type.clone(),
        ];
        let mut line =
            div()
                .flex()
                .flex_shrink_0()
                .child(cell(ROW_HEADER, ROW_HEIGHT, String::new(), true));
        for ((_, width), text) in COLUMNS.iter().zip(texts) {
            line = line.child(cell(*width, ROW_HEIGHT, text, false));
        }
        grid = grid.child(line);
    }
    panel(TITLE, grid).into_any_element()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn key() -> Key {
        Key::of(&TelemetryView::disconnected("test"))
    }

    /// SITL's copter, every line of its parameter dump.
    fn sitl() -> Vec<(String, f64)> {
        include_str!("../../../../testdata/params/sitl-copter.param")
            .lines()
            .filter_map(|line| {
                let (name, value) = line.split_once(',')?;
                Some((name.to_owned(), value.trim().parse().ok()?))
            })
            .collect()
    }

    /// The Designer's columns, read from the tree when it is here.
    #[test]
    fn the_columns_are_the_designers() {
        let Some(designer) = crate::config_coverage::source::csharp(
            "GCSViews/ConfigurationView/ConfigHWIDs.Designer.cs",
        ) else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        for (name, _) in COLUMNS {
            assert!(
                designer.contains(&format!(".HeaderText = \"{name}\";")),
                "{name}"
            );
        }
        assert!(designer.contains("this.paramNameDataGridViewTextBoxColumn.Width = 150;"));
        assert!(designer.contains("this.Size = new System.Drawing.Size(856, 496);"));
        assert!(designer.contains("AllowUserToOrderColumns = true"));
    }

    /// The device-type tables are the C#'s enums, value for value.
    #[test]
    fn the_device_type_names_are_the_enums() {
        let Some(device) = crate::config_coverage::source::csharp("ExtLibs/Utilities/Device.cs")
        else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        let body = |name: &str| -> Vec<(u32, String)> {
            let start = device.find(&format!("public enum {name}")).expect(name);
            let rest = &device[start..];
            let end = rest.find('}').expect("its end");
            rest[..end]
                .lines()
                .filter_map(|line| {
                    let line = line.split("//").next()?.trim().trim_end_matches(',');
                    let (name, value) = line.split_once('=')?;
                    let value = value.trim().trim_start_matches("0x");
                    Some((
                        u32::from_str_radix(value, 16).ok()?,
                        name.trim().trim_start_matches("DEVTYPE_").to_owned(),
                    ))
                })
                .collect()
        };
        let ours = |table: &[(u32, &str)]| -> Vec<(u32, String)> {
            table
                .iter()
                .map(|(value, name)| (*value, (*name).to_owned()))
                .collect()
        };
        assert_eq!(body("imu_types"), ours(&IMU_TYPES));
        assert_eq!(body("baro_types"), ours(&BARO_TYPES));
        assert_eq!(body("airspeed_types"), ours(&AIRSPEED_TYPES));
    }

    /// SITL's 30: every name with `_ID` or `_DEVID` - `MOT_IDLE_SEC` and `SIM_MAG_SAVE_IDS`
    /// among them - in the culture's order.
    #[test]
    fn sitls_rows_are_the_names_in_culture_order() {
        let rows = rows(&sitl());
        let names: Vec<&str> = rows.iter().map(|row| row.param.as_str()).collect();
        assert_eq!(names.len(), 30);
        assert_eq!(
            names.get(..3),
            Some(&["BARO1_DEVID", "BARO2_DEVID", "BARO3_DEVID"][..])
        );
        let position = |name: &str| names.iter().position(|held| *held == name);
        assert!(position("INS_ACC_ID") < position("INS_ACC2_ID"));
        assert!(position("INS_ACC3_ID") < position("INS_GYR_ID"));
        assert!(position("SIM_MAG_SAVE_IDS") < position("SIM_MAG1_DEVID"));
        assert!(position("COMPASS_DEV_ID") < position("COMPASS_DEV_ID2"));
        assert!(names.contains(&"MOT_IDLE_SEC"));
        assert!(!names.iter().any(|name| name.contains("_IDX")));
    }

    /// Each id taken apart as `Device.DeviceStructure` does, the type from the name.
    #[test]
    fn each_id_is_taken_apart_by_its_name() {
        let rows = rows(&sitl());
        let row = |name: &str| {
            rows.iter()
                .find(|row| row.param == name)
                .map(|row| {
                    format!(
                        "{},{},{},{},{}",
                        row.dev_id, row.bus_type, row.bus, row.address, row.dev_type
                    )
                })
                .unwrap_or_default()
        };
        assert_eq!(row("INS_ACC_ID"), "2753028,SITL,0,2,SITL");
        assert_eq!(row("BARO1_DEVID"), "65540,SITL,0,0,BARO_SITL");
        assert_eq!(row("COMPASS_DEV_ID"), "97539,UAVCAN,0,125,SENSOR_ID#1");
        assert_eq!(row("COMPASS_DEV_ID2"), "131874,SPI,4,3,LSM303D");
        // No COMP in the name: the IMU's enum, which has no 2.
        assert_eq!(row("SIM_MAG2_DEVID"), "131874,SPI,4,3,2");
        assert_eq!(row("MOT_IDLE_SEC"), "0,UNKNOWN,0,0,0");
        assert_eq!(row("BARO3_DEVID"), "0,UNKNOWN,0,0,0");
    }

    #[test]
    fn a_negative_value_wraps_as_the_jit_converts_it() {
        assert_eq!(to_uint(-1.0), u32::MAX);
        assert_eq!(to_uint(0.5), 0);
        // `DevType` looks for "ASP" in the name, and ArduPilot's airspeed id is `ARSPD_DEVID`,
        // which has not got it: the C# sends every airspeed sensor down the IMU branch, where 2
        // names nothing - so "2", as Mission Planner shows it (bug-for-bug, PLAN §12 D8).
        assert_eq!(dev_type("ARSPD_DEVID", 0x0002_0001), "2");
        assert_eq!(dev_type("ASP_DEVID", 0x0002_0001), "AIRSPEED_MS4525");
    }

    /// `Activate` builds the list anew each time.
    #[test]
    fn activate_rebuilds_the_list() {
        let mut page = HwIds::default();
        page.activate(&sitl(), key());
        assert!(page.is_active());
        assert_eq!(page.rows().len(), 30);
        page.hide();
        page.activate(&[("INS_ACC_ID".to_owned(), 2_753_028.0)], key());
        assert_eq!(page.rows().len(), 1);
    }

    /// Every fact the GUI script asserts on is one this page records.
    #[test]
    fn the_gui_script_names_facts_this_page_has() {
        let script = include_str!("../../../../tests/gui/config-hwids.gui");
        let source = include_str!("hw_ids.rs");
        let mut facts = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            if let (Some("expect"), Some(key)) = (words.next(), words.next())
                && key.starts_with("config.hwids.")
            {
                let generic = if key.starts_with("config.hwids.row.") {
                    "config.hwids.row.{}"
                } else {
                    key
                };
                assert!(
                    source.contains(&format!("\"{generic}\"")),
                    "{key} is not recorded"
                );
                facts += 1;
            }
        }
        assert!(facts > 5, "{facts} facts");
    }
}
