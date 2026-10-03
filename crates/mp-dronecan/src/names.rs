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

//! `canard_dsdlc/messages.cs`'s `MSG_INFO`: every data type `ExtLibs/DroneCAN` knows, with its
//! id and its signature, in the file's order - the order `ProcessFrame`'s `First` searches.
//!
//! A type's C# name ends in `_req` or `_res` for a service's request or response; the two share
//! the service's id and signature. The table is copied from the file as it stands (155 rows);
//! [`tests::the_table_is_messages_cs`] reads the file back when the C# tree is there.
//! `// C#: ExtLibs/DroneCAN/canard_dsdlc/messages.cs:4-158; ExtLibs/DroneCAN/DroneCAN.cs:1910-1953`

use crate::frame::{Frame, TransferType};

/// One row: the C# type's name, its data type id, and its signature (`crcseed`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MsgInfo {
    /// `type.Name`, e.g. `uavcan_protocol_NodeStatus`.
    pub name: &'static str,
    /// `msgid`.
    pub id: u16,
    /// `crcseed`: the data type signature.
    pub signature: u64,
}

impl MsgInfo {
    /// A service's request.
    #[must_use]
    pub fn is_request(&self) -> bool {
        self.name.ends_with("_req")
    }

    /// A service's response.
    #[must_use]
    pub fn is_response(&self) -> bool {
        self.name.ends_with("_res")
    }

    /// A broadcast message: neither.
    #[must_use]
    pub fn is_message(&self) -> bool {
        !self.is_request() && !self.is_response()
    }
}

/// The type a received frame carries, as `ProcessFrame` finds it: `None` where it says "No
/// Message ID" - no row has the id for a frame of that kind - and where its `First` finds no row
/// that fits (the C#'s throws, and the line is dropped).
#[must_use]
pub fn lookup(frame: &Frame) -> Option<&'static MsgInfo> {
    let kind = frame.transfer_type();
    let message_id = frame.msg_type_id();
    let service_id = u16::from(frame.svc_type_id());
    let known = match kind {
        TransferType::Anonymous => MSG_INFO
            .iter()
            .any(|row| row.id == message_id && row.is_message()),
        TransferType::Service => MSG_INFO.iter().any(|row| row.id == service_id),
        TransferType::Message => MSG_INFO.iter().any(|row| row.id == message_id),
    };
    if !known {
        return None;
    }
    MSG_INFO.iter().find(|row| match kind {
        TransferType::Message | TransferType::Anonymous => row.id == message_id && row.is_message(),
        TransferType::Service => {
            row.id == service_id
                && if frame.svc_is_request() {
                    row.is_request()
                } else {
                    row.is_response()
                }
        }
    })
}

/// The row of a type, by its C# name.
#[must_use]
pub fn by_name(name: &str) -> Option<&'static MsgInfo> {
    MSG_INFO.iter().find(|row| row.name == name)
}

/// Each row, as `(type, msgid, crcseed)`.
const fn row(name: &'static str, id: u16, signature: u64) -> MsgInfo {
    MsgInfo {
        name,
        id,
        signature,
    }
}

/// `MSG_INFO`.
pub static MSG_INFO: &[MsgInfo] = &[
    row("dronecan_protocol_FlexDebug", 16371, 0xECA60382FF038F39),
    row("dronecan_protocol_Stats", 342, 0x763AE3B8A986F8D1),
    row("dronecan_protocol_CanStats", 343, 0xCE080CAE3CA33C75),
    row("dronecan_protocol_GlobalTime", 344, 0xA55177448A490F33),
    row("dronecan_remoteid_BasicID", 20030, 0x5B1C624A8E4FC533),
    row("dronecan_remoteid_Location", 20031, 0xEAA3A2C5BCB14CAA),
    row("dronecan_remoteid_SelfID", 20032, 0x59BE81DC4C06A185),
    row("dronecan_remoteid_System", 20033, 0x9AC872F49BF32437),
    row("dronecan_remoteid_OperatorID", 20034, 0x581E7FC7F03AF935),
    row("dronecan_remoteid_ArmStatus", 20035, 0xFEDF72CCF06F3BDD),
    row(
        "dronecan_remoteid_SecureCommand_req",
        64,
        0x126A47C9C17A8BD7,
    ),
    row(
        "dronecan_remoteid_SecureCommand_res",
        64,
        0x126A47C9C17A8BD7,
    ),
    row(
        "dronecan_sensors_hygrometer_Hygrometer",
        1032,
        0xCEB308892BF163E8,
    ),
    row(
        "dronecan_sensors_magnetometer_MagneticFieldStrengthHiRes",
        1043,
        0x3053EBE3D750286F,
    ),
    row("dronecan_sensors_rc_RCInput", 1140, 0x771555E596AAB4CF),
    row("dronecan_sensors_rpm_RPM", 1045, 0x140707C09274F6E7),
    row(
        "com_hex_equipment_flow_Measurement",
        20200,
        0x6A908866BCB49C18,
    ),
    row("com_himark_servo_ServoCmd", 2018, 0x5D09E48551CE9194),
    row("com_himark_servo_ServoInfo", 2019, 0xCA8F4B8F97D23B57),
    row("com_hobbywing_esc_GetEscID", 20013, 0x00004E2D),
    row("com_hobbywing_esc_StatusMsg1", 20050, 0x813B3E2C4AD670E),
    row("com_hobbywing_esc_StatusMsg2", 20051, 0x1675DA01C3B91297),
    row("com_hobbywing_esc_StatusMsg3", 20052, 0x24919CD1EB34ECE9),
    row("com_hobbywing_esc_RawCommand", 20100, 0xBDF086C79F6640AD),
    row("com_hobbywing_esc_SetID_req", 210, 0xC323CB5E9EC2B6F7),
    row("com_hobbywing_esc_SetID_res", 210, 0xC323CB5E9EC2B6F7),
    row("com_hobbywing_esc_SetBaud_req", 211, 0xADA98653B52DE435),
    row("com_hobbywing_esc_SetBaud_res", 211, 0xADA98653B52DE435),
    row("com_hobbywing_esc_SetLED_req", 212, 0xB493BD48C0853EE5),
    row("com_hobbywing_esc_SetLED_res", 212, 0xB493BD48C0853EE5),
    row(
        "com_hobbywing_esc_SetDirection_req",
        213,
        0x9D793111D262BA68,
    ),
    row(
        "com_hobbywing_esc_SetDirection_res",
        213,
        0x9D793111D262BA68,
    ),
    row(
        "com_hobbywing_esc_SetReportingFrequency_req",
        214,
        0x1FD0404420983DEB,
    ),
    row(
        "com_hobbywing_esc_SetReportingFrequency_res",
        214,
        0x1FD0404420983DEB,
    ),
    row(
        "com_hobbywing_esc_SetThrottleSource_req",
        215,
        0xC248FAAEFE5E29A,
    ),
    row(
        "com_hobbywing_esc_SetThrottleSource_res",
        215,
        0xC248FAAEFE5E29A,
    ),
    row("com_hobbywing_esc_SelfTest_req", 216, 0xC48D4DE61C5295DF),
    row("com_hobbywing_esc_SelfTest_res", 216, 0xC48D4DE61C5295DF),
    row("com_hobbywing_esc_SetAngle_req", 217, 0x81D9B10761C28E0A),
    row("com_hobbywing_esc_SetAngle_res", 217, 0x81D9B10761C28E0A),
    row(
        "com_hobbywing_esc_GetMaintenanceInformation_req",
        241,
        0xB81DBD4EC9A5977D,
    ),
    row(
        "com_hobbywing_esc_GetMaintenanceInformation_res",
        241,
        0xB81DBD4EC9A5977D,
    ),
    row(
        "com_hobbywing_esc_GetMajorConfig_req",
        242,
        0x1506774DA3930BFD,
    ),
    row(
        "com_hobbywing_esc_GetMajorConfig_res",
        242,
        0x1506774DA3930BFD,
    ),
    row("com_tmotor_esc_ParamCfg", 1033, 0x948F5E0B33E0EDEE),
    row("com_tmotor_esc_FocCtrl", 1035, 0x598143612FBC000B),
    row("com_tmotor_esc_PUSHSCI", 1038, 0xCE2B6D6B6BDC0AE8),
    row("com_tmotor_esc_PUSHCAN", 1039, 0xAACF9B4B2577BC6E),
    row("com_tmotor_esc_ParamGet", 1332, 0x462875A0ED874302),
    row("com_volz_servo_ActuatorStatus", 20020, 0x29BF0D53B4060263),
    row("com_xacti_GnssStatus", 20305, 0x3413AC5D3E1DCBE3),
    row("com_xacti_GnssStatusReq", 20306, 0x60F5464E1CA03449),
    row("com_xacti_GimbalAttitudeStatus", 20402, 0xEB428B6C25832692),
    row("com_xacti_CopterAttStatus", 20407, 0x6C1F30F1893763B1),
    row("com_xacti_GimbalControlData", 20554, 0x3B058FA5B150C5BE),
    row(
        "ardupilot_equipment_power_BatteryInfoAux",
        20004,
        0x7D7F49FC75484882,
    ),
    row(
        "ardupilot_equipment_power_BatteryContinuous",
        20010,
        0x756B561340D5E4AE,
    ),
    row(
        "ardupilot_equipment_power_BatteryPeriodic",
        20011,
        0xF012494E97358D2,
    ),
    row(
        "ardupilot_equipment_power_BatteryCells",
        20012,
        0x5C8B1ABD15890EA4,
    ),
    row(
        "ardupilot_equipment_power_BatteryTag",
        20500,
        0x4A5A9B42099F73E1,
    ),
    row(
        "ardupilot_equipment_proximity_sensor_Proximity",
        21910,
        0x99DD3985FB3222CE,
    ),
    row(
        "ardupilot_equipment_trafficmonitor_TrafficReport",
        20790,
        0x68E45DB60B6981F8,
    ),
    row("ardupilot_gnss_Heading", 20002, 0x315CAE39ECED3412),
    row("ardupilot_gnss_Status", 20003, 0xBA3CB4ABBB007F69),
    row(
        "ardupilot_gnss_MovingBaselineData",
        20005,
        0x9F323748C32133A,
    ),
    row("ardupilot_gnss_RelPosHeading", 20006, 0xA1727AF295F94478),
    row(
        "ardupilot_indication_SafetyState",
        20000,
        0xE965701A95A1A6A1,
    ),
    row("ardupilot_indication_Button", 20001, 0x645A46EFBA7466E),
    row(
        "ardupilot_indication_NotifyState",
        20007,
        0x631F2A9C1651FDEC,
    ),
    row("cuav_equipment_power_CBAT", 20300, 0xB4DACE3A38E09A74),
    row(
        "uavcan_equipment_actuator_ArrayCommand",
        1010,
        0xD8A7486238EC3AF3,
    ),
    row("uavcan_equipment_actuator_Status", 1011, 0x5E9BBA44FAF1EA04),
    row("uavcan_equipment_ahrs_Solution", 1000, 0x72A63A3C6F41FA9B),
    row(
        "uavcan_equipment_ahrs_MagneticFieldStrength",
        1001,
        0xE2A7D4A9460BC2F2,
    ),
    row(
        "uavcan_equipment_ahrs_MagneticFieldStrength2",
        1002,
        0xB6AC0C442430297E,
    ),
    row("uavcan_equipment_ahrs_RawIMU", 1003, 0x8280632C40E574B5),
    row(
        "uavcan_equipment_air_data_TrueAirspeed",
        1020,
        0x306F69E0A591AFAA,
    ),
    row(
        "uavcan_equipment_air_data_IndicatedAirspeed",
        1021,
        0xA1892D72AB8945F,
    ),
    row(
        "uavcan_equipment_air_data_AngleOfAttack",
        1025,
        0xD5513C3F7AFAC74E,
    ),
    row(
        "uavcan_equipment_air_data_Sideslip",
        1026,
        0x7B48E55FCFF42A57,
    ),
    row(
        "uavcan_equipment_air_data_RawAirData",
        1027,
        0xC77DF38BA122F5DA,
    ),
    row(
        "uavcan_equipment_air_data_StaticPressure",
        1028,
        0xCDC7C43412BDC89A,
    ),
    row(
        "uavcan_equipment_air_data_StaticTemperature",
        1029,
        0x49272A6477D96271,
    ),
    row(
        "uavcan_equipment_camera_gimbal_AngularCommand",
        1040,
        0x4AF6E57B2B2BE29C,
    ),
    row(
        "uavcan_equipment_camera_gimbal_GEOPOICommand",
        1041,
        0x9371428A92F01FD6,
    ),
    row(
        "uavcan_equipment_camera_gimbal_Status",
        1044,
        0xB9F127865BE0D61E,
    ),
    row(
        "uavcan_equipment_device_Temperature",
        1110,
        0x70261C28A94144C6,
    ),
    row("uavcan_equipment_esc_RawCommand", 1030, 0x217F5C87D7EC951D),
    row("uavcan_equipment_esc_RPMCommand", 1031, 0xCE0F9F621CF7E70B),
    row("uavcan_equipment_esc_Status", 1034, 0xA9AF28AEA2FBB254),
    row(
        "uavcan_equipment_esc_StatusExtended",
        1036,
        0x2DC203C50960EDC,
    ),
    row("uavcan_equipment_gnss_Fix", 1060, 0x54C1572B9E07F297),
    row("uavcan_equipment_gnss_Auxiliary", 1061, 0x9BE8BDC4C3DBBFD2),
    row("uavcan_equipment_gnss_RTCMStream", 1062, 0x1F56030ECB171501),
    row("uavcan_equipment_gnss_Fix2", 1063, 0xCA41E7000F37435F),
    row(
        "uavcan_equipment_hardpoint_Command",
        1070,
        0xA1A036268B0C3455,
    ),
    row(
        "uavcan_equipment_hardpoint_Status",
        1071,
        0x624A519D42553D82,
    ),
    row(
        "uavcan_equipment_ice_FuelTankStatus",
        1129,
        0x286B4A387BA84BC4,
    ),
    row(
        "uavcan_equipment_ice_reciprocating_Status",
        1120,
        0xD38AA3EE75537EC6,
    ),
    row(
        "uavcan_equipment_indication_BeepCommand",
        1080,
        0xBE9EA9FEC2B15D52,
    ),
    row(
        "uavcan_equipment_indication_LightsCommand",
        1081,
        0x2031D93C8BDD1EC4,
    ),
    row(
        "uavcan_equipment_power_PrimaryPowerSupplyStatus",
        1090,
        0xBBA05074AD757480,
    ),
    row(
        "uavcan_equipment_power_CircuitStatus",
        1091,
        0x8313D33D0DDDA115,
    ),
    row(
        "uavcan_equipment_power_BatteryInfo",
        1092,
        0x249C26548A711966,
    ),
    row(
        "uavcan_equipment_range_sensor_Measurement",
        1050,
        0x68FFFE70FC771952,
    ),
    row(
        "uavcan_equipment_safety_ArmingStatus",
        1100,
        0x8700F375556A8003,
    ),
    row(
        "uavcan_navigation_GlobalNavigationSolution",
        2000,
        0x463B10CCCBE51C3D,
    ),
    row("uavcan_protocol_GetNodeInfo_req", 1, 0xEE468A8121C46A9E),
    row("uavcan_protocol_GetNodeInfo_res", 1, 0xEE468A8121C46A9E),
    row("uavcan_protocol_GetDataTypeInfo_req", 2, 0x1B283338A7BED2D8),
    row("uavcan_protocol_GetDataTypeInfo_res", 2, 0x1B283338A7BED2D8),
    row("uavcan_protocol_NodeStatus", 341, 0xF0868D0C1A7C6F1),
    row(
        "uavcan_protocol_GetTransportStats_req",
        4,
        0xBE6F76A7EC312B04,
    ),
    row(
        "uavcan_protocol_GetTransportStats_res",
        4,
        0xBE6F76A7EC312B04,
    ),
    row("uavcan_protocol_GlobalTimeSync", 4, 0x20271116A793C2DB),
    row("uavcan_protocol_Panic", 5, 0x8B79B4101811C1D7),
    row("uavcan_protocol_RestartNode_req", 5, 0x569E05394A3017F0),
    row("uavcan_protocol_RestartNode_res", 5, 0x569E05394A3017F0),
    row(
        "uavcan_protocol_AccessCommandShell_req",
        6,
        0x59276B5921C9246E,
    ),
    row(
        "uavcan_protocol_AccessCommandShell_res",
        6,
        0x59276B5921C9246E,
    ),
    row("uavcan_protocol_debug_KeyValue", 16370, 0xE02F25D6E0C98AE0),
    row(
        "uavcan_protocol_debug_LogMessage",
        16383,
        0xD654A48E0C049D75,
    ),
    row(
        "uavcan_protocol_dynamic_node_id_Allocation",
        1,
        0xB2A812620A11D40,
    ),
    row(
        "uavcan_protocol_dynamic_node_id_server_AppendEntries_req",
        30,
        0x8032C7097B48A3CC,
    ),
    row(
        "uavcan_protocol_dynamic_node_id_server_AppendEntries_res",
        30,
        0x8032C7097B48A3CC,
    ),
    row(
        "uavcan_protocol_dynamic_node_id_server_RequestVote_req",
        31,
        0xCDDE07BB89A56356,
    ),
    row(
        "uavcan_protocol_dynamic_node_id_server_RequestVote_res",
        31,
        0xCDDE07BB89A56356,
    ),
    row(
        "uavcan_protocol_dynamic_node_id_server_Discovery",
        390,
        0x821AE2F525F69F21,
    ),
    row(
        "uavcan_protocol_enumeration_Begin_req",
        15,
        0x196AE06426A3B5D8,
    ),
    row(
        "uavcan_protocol_enumeration_Begin_res",
        15,
        0x196AE06426A3B5D8,
    ),
    row(
        "uavcan_protocol_enumeration_Indication",
        380,
        0x884CB63050A84F35,
    ),
    row(
        "uavcan_protocol_file_BeginFirmwareUpdate_req",
        40,
        0xB7D725DF72724126,
    ),
    row(
        "uavcan_protocol_file_BeginFirmwareUpdate_res",
        40,
        0xB7D725DF72724126,
    ),
    row("uavcan_protocol_file_GetInfo_req", 45, 0x5004891EE8A27531),
    row("uavcan_protocol_file_GetInfo_res", 45, 0x5004891EE8A27531),
    row(
        "uavcan_protocol_file_GetDirectoryEntryInfo_req",
        46,
        0x8C46E8AB568BDA79,
    ),
    row(
        "uavcan_protocol_file_GetDirectoryEntryInfo_res",
        46,
        0x8C46E8AB568BDA79,
    ),
    row("uavcan_protocol_file_Delete_req", 47, 0x78648C99170B47AA),
    row("uavcan_protocol_file_Delete_res", 47, 0x78648C99170B47AA),
    row("uavcan_protocol_file_Read_req", 48, 0x8DCDCA939F33F678),
    row("uavcan_protocol_file_Read_res", 48, 0x8DCDCA939F33F678),
    row("uavcan_protocol_file_Write_req", 49, 0x515AA1DC77E58429),
    row("uavcan_protocol_file_Write_res", 49, 0x515AA1DC77E58429),
    row(
        "uavcan_protocol_param_ExecuteOpcode_req",
        10,
        0x3B131AC5EB69D2CD,
    ),
    row(
        "uavcan_protocol_param_ExecuteOpcode_res",
        10,
        0x3B131AC5EB69D2CD,
    ),
    row("uavcan_protocol_param_GetSet_req", 11, 0xA7B622F939D1A4D5),
    row("uavcan_protocol_param_GetSet_res", 11, 0xA7B622F939D1A4D5),
    row("uavcan_tunnel_Broadcast", 2010, 0x5AA2D4D9CF4B1E85),
    row("uavcan_tunnel_SerialConfig", 2011, 0x4237AACEE87E82AD),
    row("uavcan_tunnel_Targetted", 3001, 0xB138E7EA72A2A2E9),
    row("uavcan_tunnel_Call_req", 63, 0xDB11EDC510502658),
    row("uavcan_tunnel_Call_res", 63, 0xDB11EDC510502658),
    row("mppt_Stream", 20009, 0xDD7096B255FB6358),
    row("mppt_OutputEnable_req", 240, 0xEA251F2A6DD1D8A5),
    row("mppt_OutputEnable_res", 240, 0xEA251F2A6DD1D8A5),
];

#[cfg(test)]
mod tests {
    use super::*;

    /// The rows are the file's, in its order, when the C# tree is checked out.
    #[test]
    fn the_table_is_messages_cs() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../references/missionplanner/ExtLibs/DroneCAN/canard_dsdlc/messages.cs");
        let Ok(text) = std::fs::read_to_string(&path) else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        let rows: Vec<String> = text
            .lines()
            .filter(|line| line.trim_start().starts_with("(typeof("))
            .map(str::to_owned)
            .collect();
        assert_eq!(rows.len(), MSG_INFO.len());
        for (line, info) in rows.iter().zip(MSG_INFO) {
            let expect = format!("(typeof({}), {}, 0x", info.name, info.id);
            assert!(line.contains(&expect), "{line} vs {expect}");
            // The signature as a number: one row writes its leading zeros (`0x00004E2D`).
            let hex = line
                .split(", 0x")
                .nth(1)
                .and_then(|rest| rest.split(',').next())
                .unwrap_or("");
            assert_eq!(
                u64::from_str_radix(hex, 16).ok(),
                Some(info.signature),
                "{line}"
            );
        }
    }

    /// The types the page uses, at their ids.
    #[test]
    fn the_page_types_are_where_the_spec_puts_them() {
        let id = |name: &str| by_name(name).map(|row| row.id);
        assert_eq!(id("uavcan_protocol_NodeStatus"), Some(341));
        assert_eq!(id("uavcan_protocol_GetNodeInfo_req"), Some(1));
        assert_eq!(id("uavcan_protocol_param_GetSet_res"), Some(11));
        assert_eq!(id("uavcan_protocol_dynamic_node_id_Allocation"), Some(1));
        assert_eq!(id("uavcan_protocol_debug_LogMessage"), Some(16383));
        assert_eq!(MSG_INFO.len(), 155);
    }

    /// A frame finds its row by kind: a GetNodeInfo request and response share id 1 with the
    /// allocation an anonymous frame carries.
    #[test]
    fn frames_find_their_rows() {
        let mut request = Frame::from_id(0, true, false);
        request.set_source_node(127);
        request.set_service(true);
        request.set_svc_is_request(true);
        request.set_svc_destination_node(10);
        request.set_svc_type_id(1);
        assert_eq!(
            lookup(&request).map(|row| row.name),
            Some("uavcan_protocol_GetNodeInfo_req")
        );
        request.set_svc_is_request(false);
        assert_eq!(
            lookup(&request).map(|row| row.name),
            Some("uavcan_protocol_GetNodeInfo_res")
        );
        let mut anonymous = Frame::from_id(0, true, false);
        anonymous.set_msg_type_id(1 | (0x155 << 2));
        assert_eq!(
            lookup(&anonymous).map(|row| row.name),
            Some("uavcan_protocol_dynamic_node_id_Allocation")
        );
        let mut unknown = Frame::from_id(0, true, false);
        unknown.set_source_node(5);
        unknown.set_msg_type_id(12345);
        assert_eq!(lookup(&unknown), None);
    }
}
