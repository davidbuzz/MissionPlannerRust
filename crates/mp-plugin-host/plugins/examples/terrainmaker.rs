//! `Plugins/TerrainMakerPlugin/`, "TerrainMakerPlugin", one of the C#'s four real plugins: "Make
//! Terrain DAT" on the planning screen's map menu writes ArduPilot terrain files (`N35E149.DAT`)
//! for every whole degree the map shows, from the planner's elevation data, into the user data
//! folder's `TerrainData`, for an SD card.
//!
//! Ported whole - `TerrainMakerPlugin.cs`, `TerrainDataFile.cs` and `Location.cs` - with two
//! differences the world makes: the C#'s progress dialog is the status line ("Making block n of
//! m"), and it has no Cancel; and the file is written in one piece when it is complete, rather
//! than block by block, so a failed tile leaves no partial file (the C# deletes its partial file
//! on Cancel).

use std::sync::{Mutex, PoisonError};

use mp_plugins::Guest;
use mp_plugins::host::{self, Area, DialogResult, MapMenu, MessageButtons};

/// The menu entry's id.
static ENTRY: Mutex<Option<u32>> = Mutex::new(None);

// `TerrainDataFile`'s constants. `// C#: Plugins/TerrainMakerPlugin/TerrainDataFile.cs:17-43`
const TERRAIN_GRID_MAVLINK_SIZE: i32 = 4;
const TERRAIN_GRID_BLOCK_MUL_X: i32 = 7;
const TERRAIN_GRID_BLOCK_MUL_Y: i32 = 8;
const TERRAIN_GRID_BLOCK_SPACING_X: i32 =
    (TERRAIN_GRID_BLOCK_MUL_X - 1) * TERRAIN_GRID_MAVLINK_SIZE;
const TERRAIN_GRID_BLOCK_SPACING_Y: i32 =
    (TERRAIN_GRID_BLOCK_MUL_Y - 1) * TERRAIN_GRID_MAVLINK_SIZE;
const TERRAIN_GRID_BLOCK_SIZE_X: i32 = TERRAIN_GRID_MAVLINK_SIZE * TERRAIN_GRID_BLOCK_MUL_X;
const TERRAIN_GRID_BLOCK_SIZE_Y: i32 = TERRAIN_GRID_MAVLINK_SIZE * TERRAIN_GRID_BLOCK_MUL_Y;
const TERRAIN_GRID_FORMAT_VERSION: u16 = 1;
const IO_BLOCK_SIZE: i32 = 2048;
const IO_BLOCK_DATA_SIZE: usize = 1821;
const IO_BLOCK_TRAILER_SIZE: usize = 2048 - IO_BLOCK_DATA_SIZE;

// `Location`'s scaling. `// C#: Plugins/TerrainMakerPlugin/Location.cs:14-15`
const LOCATION_SCALING_FACTOR: f64 = 0.011_131_884_502_145_034;
const LOCATION_SCALING_FACTOR_INV: f64 = 89.832_049_533_689_22;

/// `Location`: a position in degrees times 1e7, with the C#'s wrapping `Int32` arithmetic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Location {
    lat: i32,
    lng: i32,
}

/// `(Int32)x`: the C# cast, truncating toward zero.
#[allow(clippy::cast_possible_truncation)]
const fn int(x: f64) -> i32 {
    x as i32
}

impl Location {
    /// `longitude_scale`. `// C#: Plugins/TerrainMakerPlugin/Location.cs:46-51`
    fn longitude_scale(lat: i32) -> f64 {
        (f64::from(lat) * (1.0e-7 * (std::f64::consts::PI / 180.0)))
            .cos()
            .max(0.01)
    }

    /// `diff_longitude`: lon1 - lon2, wrapping at +-180e7. `// C#: Plugins/TerrainMakerPlugin/Location.cs:56-72`
    #[allow(clippy::cast_possible_truncation)]
    fn diff_longitude(lon1: i32, lon2: i32) -> i32 {
        if (lon1 < 0) == (lon2 < 0) {
            return lon1.wrapping_sub(lon2);
        }
        let mut dlon = i64::from(lon1) - i64::from(lon2);
        if dlon > 1_800_000_000 {
            dlon -= 3_600_000_000;
        } else if dlon < -1_800_000_000 {
            dlon += 3_600_000_000;
        }
        dlon as i32
    }

    /// `get_distance_NE`: metres north and east to `other`. `// C#: Plugins/TerrainMakerPlugin/Location.cs:80-87`
    fn distance_ne(self, other: Self) -> (f64, f64) {
        (
            f64::from(other.lat.wrapping_sub(self.lat)) * LOCATION_SCALING_FACTOR,
            f64::from(Self::diff_longitude(other.lng, self.lng))
                * LOCATION_SCALING_FACTOR
                * Self::longitude_scale(other.lat.wrapping_add(self.lat) / 2),
        )
    }

    /// `add_offset_meters`. `// C#: Plugins/TerrainMakerPlugin/Location.cs:91-100`
    fn add_offset_meters(self, north: f64, east: f64) -> Self {
        let dlat = north * LOCATION_SCALING_FACTOR_INV;
        let dlng = (east * LOCATION_SCALING_FACTOR_INV)
            / Self::longitude_scale(int(f64::from(self.lat) + dlat / 2.0));
        Self {
            lat: self.lat.wrapping_add(int(dlat)),
            lng: int(dlng + f64::from(self.lng)),
        }
    }
}

/// The degree a file covers, as a `Location`.
const fn degree(lat: i32, lng: i32) -> Location {
    Location {
        lat: lat.wrapping_mul(10_000_000),
        lng: lng.wrapping_mul(10_000_000),
    }
}

/// `east_blocks`: how many blocks a row of the file holds. `// C#: Plugins/TerrainMakerPlugin/TerrainDataFile.cs:47-57`
fn east_blocks(loc: Location, spacing: i32) -> i32 {
    let mut loc2 = loc.add_offset_meters(0.0, f64::from(2 * spacing * TERRAIN_GRID_BLOCK_SIZE_Y));
    loc2.lng = loc2.lng.wrapping_add(10_000_000);
    let (_, east) = loc.distance_ne(loc2);
    int(east.round_ties_even() / f64::from(spacing * TERRAIN_GRID_BLOCK_SPACING_Y))
}

/// `pos_from_file_offset`: where the block at `file_offset` starts.
/// `// C#: Plugins/TerrainMakerPlugin/TerrainDataFile.cs:59-85`
fn pos_from_file_offset(lat: i32, lng: i32, file_offset: i32, spacing: i32) -> Location {
    let reference = degree(lat, lng);
    let stride = east_blocks(reference, spacing);
    let blocks = file_offset / IO_BLOCK_SIZE;
    let grid_idx_x = blocks / stride;
    let grid_idx_y = blocks % stride;
    let idx_x = grid_idx_x * TERRAIN_GRID_BLOCK_SPACING_X;
    let idx_y = grid_idx_y * TERRAIN_GRID_BLOCK_SPACING_Y;
    let grid_idx_x = idx_x / TERRAIN_GRID_BLOCK_SPACING_X;
    let grid_idx_y = idx_y / TERRAIN_GRID_BLOCK_SPACING_Y;
    reference.add_offset_meters(
        f64::from(grid_idx_x * TERRAIN_GRID_BLOCK_SPACING_X * spacing),
        f64::from(grid_idx_y * TERRAIN_GRID_BLOCK_SPACING_Y * spacing),
    )
}

/// `GridBlock`: one 2 KB block of the file. `// C#: Plugins/TerrainMakerPlugin/TerrainDataFile.cs:87-271`
struct GridBlock {
    bitmap: u64,
    lat: i32,
    lon: i32,
    top_left: Location,
    crc: u16,
    spacing: u16,
    height: Vec<i16>,
    grid_idx_x: u16,
    grid_idx_y: u16,
    lon_degrees: i16,
    lat_degrees: i8,
}

impl GridBlock {
    /// The block at `location` of the file for (`lat`, `lng`). `// C#: Plugins/TerrainMakerPlugin/TerrainDataFile.cs:122-145`
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    fn new(lat: i8, lng: i16, location: Location, spacing: u16) -> Self {
        let space = i32::from(spacing);
        let reference = degree(i32::from(lat), i32::from(lng));
        let (north, east) = reference.distance_ne(location);
        let idx_x = (north.round_ties_even() / f64::from(space)) as i64;
        let idx_y = (east.round_ties_even() / f64::from(space)) as i64;
        let grid_idx_x = (idx_x as f64 / f64::from(TERRAIN_GRID_BLOCK_SPACING_X)).floor() as u16;
        let grid_idx_y = (idx_y as f64 / f64::from(TERRAIN_GRID_BLOCK_SPACING_Y)).floor() as u16;
        let corner = reference.add_offset_meters(
            f64::from(i32::from(grid_idx_x) * TERRAIN_GRID_BLOCK_SPACING_X * space),
            f64::from(i32::from(grid_idx_y) * TERRAIN_GRID_BLOCK_SPACING_Y * space),
        );
        Self {
            bitmap: 0,
            lat: corner.lat,
            lon: corner.lng,
            top_left: corner,
            crc: 0,
            spacing,
            height: vec![
                0;
                usize::try_from(TERRAIN_GRID_BLOCK_SIZE_X * TERRAIN_GRID_BLOCK_SIZE_Y)
                    .unwrap_or(0)
            ],
            grid_idx_x,
            grid_idx_y,
            lon_degrees: lng,
            lat_degrees: lat,
        }
    }

    fn index(x: i32, y: i32) -> usize {
        usize::try_from(y * TERRAIN_GRID_BLOCK_SIZE_X + x).unwrap_or(usize::MAX)
    }

    fn get(&self, x: i32, y: i32) -> i16 {
        self.height.get(Self::index(x, y)).copied().unwrap_or(0)
    }

    fn set(&mut self, x: i32, y: i32, value: i16) {
        if let Some(slot) = self.height.get_mut(Self::index(x, y)) {
            *slot = value;
        }
    }

    /// `isvalid`: none of the 4 x 4 grid's heights is zero. `// C#: Plugins/TerrainMakerPlugin/TerrainDataFile.cs:158-180`
    fn valid(&self, x: i32, y: i32) -> bool {
        (0..4).all(|dy| (0..4).all(|dx| self.get(x * 4 + dx, y * 4 + dy) != 0))
    }

    /// `Pack`: the block as the file holds it. The C# writes 226 trailer bytes, not 227, so a
    /// block is 2047 bytes; kept. `// C#: Plugins/TerrainMakerPlugin/TerrainDataFile.cs:219-261`
    fn pack(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(2048);
        out.extend_from_slice(&self.bitmap.to_le_bytes());
        out.extend_from_slice(&self.lat.to_le_bytes());
        out.extend_from_slice(&self.lon.to_le_bytes());
        out.extend_from_slice(&self.crc.to_le_bytes());
        out.extend_from_slice(&TERRAIN_GRID_FORMAT_VERSION.to_le_bytes());
        out.extend_from_slice(&self.spacing.to_le_bytes());
        for gx in 0..TERRAIN_GRID_BLOCK_SIZE_X {
            for gy in 0..TERRAIN_GRID_BLOCK_SIZE_Y {
                out.extend_from_slice(&self.get(gx, gy).to_le_bytes());
            }
        }
        out.extend_from_slice(&self.grid_idx_x.to_le_bytes());
        out.extend_from_slice(&self.grid_idx_y.to_le_bytes());
        out.extend_from_slice(&self.lon_degrees.to_le_bytes());
        out.extend_from_slice(&self.lat_degrees.to_le_bytes());
        out.resize(out.len() + IO_BLOCK_TRAILER_SIZE - 1, 0);
        out
    }

    /// `getPackedBytes`: packed with the CRC of its data, taken with the CRC at zero.
    /// `// C#: Plugins/TerrainMakerPlugin/TerrainDataFile.cs:263-270`
    fn packed(&mut self) -> Vec<u8> {
        self.crc = 0;
        let first = self.pack();
        self.crc = crc16(first.get(..IO_BLOCK_DATA_SIZE).unwrap_or(&first));
        self.pack()
    }
}

/// `crc16tab`: CRC-16/CCITT's table, polynomial 0x1021.
// A `const` block: indexing a fixed array by a counter below its length.
#[allow(clippy::indexing_slicing)]
const CRC16_TABLE: [u16; 256] = {
    let mut table = [0u16; 256];
    let mut i = 0;
    while i < 256 {
        #[allow(clippy::cast_possible_truncation)]
        let mut crc = (i as u16) << 8;
        let mut bit = 0;
        while bit < 8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
            bit += 1;
        }
        table[i] = crc;
        i += 1;
    }
    table
};

/// `crc16`. `// C#: Plugins/TerrainMakerPlugin/TerrainDataFile.cs:276-321`
fn crc16(data: &[u8]) -> u16 {
    data.iter().fold(0u16, |crc, &b| {
        (crc << 8)
            ^ CRC16_TABLE
                .get(usize::from(((crc >> 8) ^ u16::from(b)) & 0xff))
                .copied()
                .unwrap_or(0)
    })
}

/// `createTerrainDataFile` for one degree: the file's name and its bytes.
/// `// C#: Plugins/TerrainMakerPlugin/TerrainMakerPlugin.cs:161-285`
#[allow(clippy::cast_possible_truncation)]
fn terrain_file(lat: i32, lng: i32, spacing: u16) -> (String, Vec<u8>) {
    let space = i32::from(spacing);
    let name = format!(
        "{}{:02}{}{:03}.DAT",
        if lat < 0 { "S" } else { "N" },
        lat.abs(),
        if lng < 0 { "W" } else { "E" },
        lng.abs()
    );
    // How many blocks: until a block starts a whole degree north.
    let mut n = 0;
    loop {
        let block = pos_from_file_offset(lat, lng, n * IO_BLOCK_SIZE, space);
        if f64::from(block.lat) * 1.0e-7 - f64::from(lat) >= 1.0 {
            break;
        }
        n += 1;
    }
    let mut data = Vec::new();
    for number in 0..n {
        host::status(&format!("Making block {number} of {n}"));
        let location = pos_from_file_offset(lat, lng, number * IO_BLOCK_SIZE, space);
        let mut grid = GridBlock::new(lat as i8, lng as i16, location, spacing);
        for gx in 0..TERRAIN_GRID_BLOCK_SIZE_X {
            for gy in 0..TERRAIN_GRID_BLOCK_SIZE_Y {
                let point = grid
                    .top_left
                    .add_offset_meters(f64::from(gx * space), f64::from(gy * space));
                let (plat, plng) = (f64::from(point.lat) * 1.0e-7, f64::from(point.lng) * 1.0e-7);
                let height = match host::terrain_altitude(plat, plng) {
                    Some(alt) if alt != 0.0 => alt.round_ties_even() as i16,
                    _ => 0,
                };
                grid.set(gx, gy, height);
            }
        }
        grid.bitmap = 0;
        for x in 0..TERRAIN_GRID_BLOCK_MUL_X {
            for y in 0..TERRAIN_GRID_BLOCK_MUL_Y {
                if grid.valid(x, y) {
                    grid.bitmap |= 1u64 << (y + TERRAIN_GRID_BLOCK_MUL_Y * x);
                }
            }
        }
        data.extend(grid.packed());
    }
    (name, data)
}

/// `(int)Math.Floor` / `(int)Math.Ceiling` of a degree.
#[allow(clippy::cast_possible_truncation)]
fn whole(degrees: f64) -> i32 {
    degrees as i32
}

struct TerrainMaker;

impl Guest for TerrainMaker {
    fn name() -> String {
        "TerrainMakerPlugin".to_owned()
    }

    fn version() -> String {
        "2.0".to_owned()
    }

    fn author() -> String {
        "Andras \"EosBandi\" Schaffer".to_owned()
    }

    fn loop_rate_hz() -> f32 {
        0.0
    }

    fn init() -> bool {
        true
    }

    // `// C#: Plugins/TerrainMakerPlugin/TerrainMakerPlugin.cs:60-65`
    fn loaded() -> bool {
        let id = host::menu_add(MapMenu::FlightPlanner, None, "Make Terrain DAT");
        *ENTRY.lock().unwrap_or_else(PoisonError::into_inner) = Some(id);
        true
    }

    fn run_loop() -> bool {
        true
    }

    fn exit() -> bool {
        true
    }

    // `MakeTerrainDAT`. `// C#: Plugins/TerrainMakerPlugin/TerrainMakerPlugin.cs:75-158`
    fn menu_click(id: u32, _lat: f64, _lng: f64) {
        if *ENTRY.lock().unwrap_or_else(PoisonError::into_inner) != Some(id) {
            return;
        }
        let mut area: Option<Area> = host::fp_selected_area();
        if area.is_none()
            && host::message_box(
                "No area defined, use area displayed on screen?",
                "Terrain DAT",
                MessageButtons::YesNo,
            ) == DialogResult::Yes
        {
            area = host::fp_view_area();
        }
        let Some(area) = area else { return };
        let Some(spacing) =
            host::input_box("SPACING", "Enter the grid spacing in meters (5-100).", "30")
        else {
            return;
        };
        let Ok(spacing) = spacing.trim().parse::<i32>() else {
            let _ = host::message_box("Invalid Number", "ERROR", MessageButtons::Ok);
            return;
        };
        if !(5..=100).contains(&spacing) {
            let _ = host::message_box(
                "Spacing must be between 5 and 100 meters",
                "ERROR",
                MessageButtons::Ok,
            );
            return;
        }
        let spacing = u16::try_from(spacing).unwrap_or(30);
        for lat in whole(area.bottom.floor())..whole(area.top.ceil()) {
            for lng in whole(area.left.floor())..whole(area.right.ceil()) {
                host::log(&format!("Make Terrain DAT {lat} {lng}"));
                let (name, data) = terrain_file(lat, lng, spacing);
                if let Err(err) = host::write_user_data(&format!("TerrainData/{name}"), &data) {
                    let _ = host::message_box(
                        &format!("Can't create directory: {err}"),
                        "",
                        MessageButtons::Ok,
                    );
                    return;
                }
                let _ = host::message_box(
                    "Terrain DAT created in Documents/Mission Planner/TerrainDat folder",
                    "Terrain DAT",
                    MessageButtons::Ok,
                );
            }
        }
    }

    fn form_event(_id: String, _value: String) {}
}

mp_plugins::export_plugin!(TerrainMaker);
