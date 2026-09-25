//! The mission file the planner sends over MAVFTP: `missionpck.pack` and `unpack`. Ported from
//! `ExtLibs/ArduPilot/missionpck.cs` @ efb0801 (GPL-3.0-or-later).
//!
//! `@MISSION/mission.dat` is a ten-byte header - the magic `0x763d`, the mission type, options,
//! the first item's number and the item count, each a little-endian `ushort` - followed by the
//! items as `mavlink_mission_item_int_t` structures, 38 bytes each, laid out as the MAVLink
//! payload lays them: the four parameters, x, y and z, the sequence, the command, the target
//! system and component, the frame, current, autocontinue and the mission type.
//! ArduPilot's `AP_Mission` reads the same file (`mission_upload` in `AP_Filesystem_Mission`).

use crate::item::MissionItem;
use crate::wire::WireItem;

/// `mission_magic`: the header's first two bytes.
/// `// C#: ExtLibs/ArduPilot/missionpck.cs:21`
pub const MISSION_MAGIC: u16 = 0x763d;

/// The header's five `ushort`s.
/// `// C#: ExtLibs/ArduPilot/missionpck.cs:23-30`
pub const HEADER_LEN: usize = 10;

/// `msginfo.length` for `MISSION_ITEM_INT`: the size of one packed item.
/// `// C#: ExtLibs/ArduPilot/missionpck.cs:46-49`
pub const ITEM_LEN: usize = 38;

/// What `unpack` returns: the items, the mission type and the first item's number.
#[derive(Debug, Clone, PartialEq)]
pub struct Unpacked {
    /// The items, in the order the file holds them.
    pub items: Vec<MissionItem>,
    /// `header.data_type`, a `MAV_MISSION_TYPE`.
    pub mission_type: u16,
    /// `header.start`, 0 for a full upload.
    pub start: u16,
}

/// Where `unpack` throws.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum PckError {
    /// Fewer bytes than a header.
    #[error("Index was outside the bounds of the array.")]
    Short,
    /// `"Invalid Magic " + header.magic`.
    #[error("Invalid Magic {0}")]
    InvalidMagic(u16),
    /// `"Bad Item count " + header.num_items + " vs " + msgcount`.
    #[error("Bad Item count {header} vs {found}")]
    BadItemCount {
        /// What the header said.
        header: u16,
        /// How many whole items followed it.
        found: u16,
    },
}

/// `missionpck.pack`: the header, then each item as a `mavlink_mission_item_int_t`.
///
/// The items go through [`MissionItem::to_wire`], as `(mavlink_mission_item_int_t)Locationwp`
/// scales a navigation command's latitude and longitude by 1e7 and carries the rest raw; the
/// target system, component and mission type of each item are 0, as the conversion leaves them.
/// `// C#: ExtLibs/ArduPilot/missionpck.cs:70-84; ExtLibs/ArduPilot/Locationwp.cs`
#[must_use]
pub fn pack(items: &[MissionItem], mission_type: u16, start: u16) -> Vec<u8> {
    let mut out = Vec::with_capacity(HEADER_LEN + items.len() * ITEM_LEN);
    out.extend_from_slice(&MISSION_MAGIC.to_le_bytes());
    out.extend_from_slice(&mission_type.to_le_bytes());
    out.extend_from_slice(&0u16.to_le_bytes());
    out.extend_from_slice(&start.to_le_bytes());
    out.extend_from_slice(&u16::try_from(items.len()).unwrap_or(u16::MAX).to_le_bytes());
    for item in items {
        put_item(&mut out, &item.to_wire());
    }
    out
}

/// One `mavlink_mission_item_int_t`, as `MavlinkUtil.StructureToByteArray` lays it out.
fn put_item(out: &mut Vec<u8>, wire: &WireItem) {
    out.extend_from_slice(&wire.param1.to_le_bytes());
    out.extend_from_slice(&wire.param2.to_le_bytes());
    out.extend_from_slice(&wire.param3.to_le_bytes());
    out.extend_from_slice(&wire.param4.to_le_bytes());
    out.extend_from_slice(&wire.x.to_le_bytes());
    out.extend_from_slice(&wire.y.to_le_bytes());
    out.extend_from_slice(&wire.z.to_le_bytes());
    out.extend_from_slice(&wire.seq.to_le_bytes());
    out.extend_from_slice(&wire.command.to_le_bytes());
    // target_system, target_component
    out.extend_from_slice(&[0, 0]);
    out.extend_from_slice(&[wire.frame, wire.current, wire.autocontinue]);
    // mission_type
    out.push(0);
}

fn u16_at(data: &[u8], at: usize) -> Option<u16> {
    Some(u16::from_le_bytes([*data.get(at)?, *data.get(at + 1)?]))
}

fn i32_at(data: &[u8], at: usize) -> Option<i32> {
    Some(i32::from_le_bytes(data.get(at..at + 4)?.try_into().ok()?))
}

fn f32_at(data: &[u8], at: usize) -> Option<f32> {
    Some(f32::from_le_bytes(data.get(at..at + 4)?.try_into().ok()?))
}

/// One item read back, `MavlinkUtil.ByteArrayToStructure<mavlink_mission_item_int_t>`.
fn get_item(bytes: &[u8]) -> Option<WireItem> {
    Some(WireItem {
        param1: f32_at(bytes, 0)?,
        param2: f32_at(bytes, 4)?,
        param3: f32_at(bytes, 8)?,
        param4: f32_at(bytes, 12)?,
        x: i32_at(bytes, 16)?,
        y: i32_at(bytes, 20)?,
        z: f32_at(bytes, 24)?,
        seq: u16_at(bytes, 28)?,
        command: u16_at(bytes, 30)?,
        frame: *bytes.get(34)?,
        current: *bytes.get(35)?,
        autocontinue: *bytes.get(36)?,
    })
}

/// `missionpck.unpack`: the header checked for its magic, the item count checked against the
/// bytes that follow (whole items only; a trailing part of one is ignored, as the C#'s integer
/// division ignores it), and each item read back.
///
/// # Errors
///
/// Fewer bytes than a header, a header without the magic, or a count that does not match.
/// `// C#: ExtLibs/ArduPilot/missionpck.cs:32-68`
pub fn unpack(data: &[u8]) -> Result<Unpacked, PckError> {
    let magic = u16_at(data, 0).ok_or(PckError::Short)?;
    let mission_type = u16_at(data, 2).ok_or(PckError::Short)?;
    let _options = u16_at(data, 4).ok_or(PckError::Short)?;
    let start = u16_at(data, 6).ok_or(PckError::Short)?;
    let num_items = u16_at(data, 8).ok_or(PckError::Short)?;
    if magic != MISSION_MAGIC {
        return Err(PckError::InvalidMagic(magic));
    }
    let body = data.get(HEADER_LEN..).unwrap_or_default();
    let found = u16::try_from(body.len() / ITEM_LEN).unwrap_or(u16::MAX);
    if found != num_items {
        return Err(PckError::BadItemCount {
            header: num_items,
            found,
        });
    }
    let items = body
        .chunks_exact(ITEM_LEN)
        .filter_map(get_item)
        .map(|wire| MissionItem::from_wire(&wire))
        .collect();
    Ok(Unpacked {
        items,
        mission_type,
        start,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn waypoint(seq: u16) -> MissionItem {
        MissionItem {
            seq,
            frame: 3,
            command: 16,
            param1: 2.0,
            x: -35.363_262_1,
            y: 149.165_237_4,
            z: 50.0,
            autocontinue: 1,
            ..MissionItem::default()
        }
    }

    #[test]
    fn the_header_is_ten_little_endian_bytes_and_each_item_thirty_eight() {
        let bytes = pack(&[waypoint(0), waypoint(1)], 0, 0);
        assert_eq!(bytes.len(), HEADER_LEN + 2 * ITEM_LEN);
        assert_eq!(&bytes[..10], &[0x3d, 0x76, 0, 0, 0, 0, 0, 0, 2, 0]);
        // Item 1's seq sits at byte 28 of its structure; its command, 16, at 30; its frame at 34.
        let item1 = &bytes[HEADER_LEN + ITEM_LEN..];
        assert_eq!(&item1[28..30], &[1, 0]);
        assert_eq!(&item1[30..32], &[16, 0]);
        assert_eq!(item1[34], 3);
        assert_eq!(item1[36], 1);
        // Latitude scaled by 1e7 as an int, as `(mavlink_mission_item_int_t)Locationwp` does.
        assert_eq!(i32_at(item1, 16), Some(-353_632_621));
    }

    #[test]
    fn what_is_packed_unpacks_to_the_same_items() {
        let items = vec![
            waypoint(0),
            waypoint(1),
            MissionItem {
                seq: 2,
                command: 203, // DO_DIGICAM_CONTROL: x and y raw
                x: 1.0,
                y: 0.0,
                ..MissionItem::default()
            },
        ];
        let unpacked = unpack(&pack(&items, 1, 0)).expect("unpacks");
        assert_eq!(unpacked.mission_type, 1);
        assert_eq!(unpacked.start, 0);
        assert_eq!(unpacked.items.len(), 3);
        assert_eq!(unpacked.items[2].command, 203);
        assert!((unpacked.items[2].x - 1.0).abs() < f64::EPSILON);
        assert!((unpacked.items[1].x - waypoint(1).x).abs() < 1e-7);
        assert!((unpacked.items[1].y - waypoint(1).y).abs() < 1e-7);
        assert!((unpacked.items[1].z - 50.0).abs() < f64::EPSILON);
        assert_eq!(unpacked.items[1].frame, 3);
    }

    #[test]
    fn the_wrong_magic_and_a_wrong_count_are_refused_in_the_csharps_words() {
        let mut bytes = pack(&[waypoint(0)], 0, 0);
        bytes[0] = 0x3e;
        assert_eq!(
            unpack(&bytes).unwrap_err().to_string(),
            format!("Invalid Magic {}", 0x763e)
        );
        let mut bytes = pack(&[waypoint(0), waypoint(1)], 0, 0);
        bytes.truncate(HEADER_LEN + ITEM_LEN + 5);
        assert_eq!(
            unpack(&bytes).unwrap_err().to_string(),
            "Bad Item count 2 vs 1"
        );
        assert_eq!(unpack(&[0x3d, 0x76, 0]), Err(PckError::Short));
    }

    #[test]
    fn an_empty_mission_packs_to_a_header_alone() {
        let bytes = pack(&[], 0, 0);
        assert_eq!(bytes.len(), HEADER_LEN);
        let unpacked = unpack(&bytes).expect("unpacks");
        assert!(unpacked.items.is_empty());
    }
}
