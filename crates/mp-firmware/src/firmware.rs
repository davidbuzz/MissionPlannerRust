//! The `.apj` / `.px4` firmware container.
//!
//! Ported from `ExtLibs/px4uploader/Firmware.cs` @ efb0801 (GPL-3.0-or-later).
//!
//! JSON holding a base64-encoded zlib stream, plus the board id the bootloader must agree with.
//! Two details are transliterated rather than corrected, and both would be easy to "fix" into a
//! board that will not boot - see `crc` and `image_buffer_length`.

use std::io::Read;

/// Something wrong with a firmware file.
#[derive(Debug, thiserror::Error)]
pub enum FirmwareError {
    /// The file could not be read.
    #[error("could not read the firmware file: {0}")]
    Io(#[from] std::io::Error),
    /// The file is not JSON, or not the JSON this expects.
    #[error("not a firmware file: {0}")]
    Format(String),
    /// The image is not valid base64.
    #[error("the firmware image is not valid base64: {0}")]
    Base64(String),
    /// The image is not a valid zlib stream.
    #[error("the firmware image could not be decompressed: {0}")]
    Decompress(String),
}

/// A firmware image and what it says about itself.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Firmware {
    /// The board type this firmware is for. Must equal the bootloader's `BOARD_ID`.
    pub board_id: u32,
    /// The board revision, where the file states one.
    pub board_revision: i64,
    /// What the file calls itself.
    pub description: String,
    /// The ArduPilot commit this was built from, where present.
    pub git_hash: String,
    /// Human-readable version string, where present.
    pub version: String,
    /// The decompressed image, padded to a multiple of four bytes.
    pub image: Vec<u8>,
    /// The decompressed external-flash image, if the file carries one.
    pub external_image: Vec<u8>,
    /// The image size the file declares, before padding.
    pub declared_image_size: usize,
    /// The external image size the file declares, before padding.
    pub declared_external_size: usize,
}

/// The buffer length the C# allocates for a declared image size.
///
/// `image_size + (image_size % 4)` — **not** a round-up to the next multiple of four, which is
/// what it looks like and what a careless reading turns it into. For 1,001 bytes it gives 1,002;
/// the round-to-four happens afterwards, as a separate step. Reproduced because the result feeds
/// the CRC the bootloader checks, and a buffer one word longer or shorter changes it.
///
/// `// C#: ExtLibs/px4uploader/Firmware.cs:112, 133`
#[must_use]
pub const fn image_buffer_length(declared: usize) -> usize {
    declared + (declared % 4)
}

/// Rounds a length up to a multiple of four.
const fn to_multiple_of_four(length: usize) -> usize {
    length.div_ceil(4) * 4
}

impl Firmware {
    /// Reads a firmware file from bytes.
    ///
    /// # Errors
    /// If the file is not the JSON container, the image is not base64, or it is not a zlib stream.
    pub fn parse(bytes: &[u8]) -> Result<Self, FirmwareError> {
        let json: serde_json::Value =
            serde_json::from_slice(bytes).map_err(|err| FirmwareError::Format(err.to_string()))?;

        let number = |key: &str| -> i64 {
            json.get(key)
                .and_then(serde_json::Value::as_i64)
                .unwrap_or(0)
        };
        let text = |key: &str| -> String {
            json.get(key)
                .and_then(serde_json::Value::as_str)
                .unwrap_or_default()
                .to_owned()
        };

        let declared_image_size = usize::try_from(number("image_size")).unwrap_or(0);
        let declared_external_size = usize::try_from(number("extf_image_size")).unwrap_or(0);

        let image = if declared_image_size > 0 {
            decompress(&text("image"), declared_image_size, 0x00)?
        } else {
            Vec::new()
        };
        // The external image buffer is pre-filled with 0xff, not zero - it is flash, and an erased
        // flash byte is 0xff. The tail past the declared size therefore reads as erased rather
        // than as programmed zeros, which is what the CRC over it depends on.
        // `// C#: ExtLibs/px4uploader/Firmware.cs:135-138`
        let external_image = if declared_external_size > 0 {
            decompress(&text("extf_image"), declared_external_size, 0xff)?
        } else {
            Vec::new()
        };

        Ok(Self {
            board_id: u32::try_from(number("board_id")).unwrap_or(0),
            board_revision: number("board_revision"),
            description: text("description"),
            git_hash: text("ardupilot_git_hash"),
            version: text("version"),
            image,
            external_image,
            declared_image_size,
            declared_external_size,
        })
    }

    /// Reads a firmware file from disk.
    ///
    /// # Errors
    /// As [`Firmware::parse`], plus any error opening the file.
    pub fn load(path: &std::path::Path) -> Result<Self, FirmwareError> {
        Self::parse(&std::fs::read(path)?)
    }

    /// The CRC the bootloader will report for this image, padded to `pad_length`.
    ///
    /// **Not a standard CRC-32.** It uses the standard CRC-32 table, but it starts at zero rather
    /// than `0xFFFF_FFFF` and does not invert the result — so `flate2`'s `crc32`, or any other
    /// off-the-shelf implementation, gives a different answer for the same bytes. Swapping one in
    /// would produce a verify failure after a flash that actually succeeded, and the natural
    /// response to that is to flash again.
    ///
    /// The padding loop is also transliterated: it runs `while i < pad_length - 1`, stepping four
    /// bytes at a time from the end of the image. That reads like an off-by-one and is what the
    /// bootloader computes, so it stays.
    ///
    /// `// C#: ExtLibs/px4uploader/Firmware.cs:157-176`
    #[must_use]
    pub fn crc(&self, pad_length: usize) -> u32 {
        let mut state = crc32(&self.image, 0);
        // An erased flash word.
        const PAD: [u8; 4] = [0xff, 0xff, 0xff, 0xff];
        let mut index = self.image.len();
        while index + 1 < pad_length {
            state = crc32(&PAD, state);
            index += 4;
        }
        state
    }

    /// The CRC over the first `length` bytes of the external image.
    ///
    /// No padding: the C# takes a prefix and stops. `// C#: ExtLibs/px4uploader/Firmware.cs:178-183`
    #[must_use]
    pub fn external_crc(&self, length: usize) -> u32 {
        let end = length.min(self.external_image.len());
        crc32(self.external_image.get(..end).unwrap_or_default(), 0)
    }
}

/// Decodes and inflates one image field.
fn decompress(encoded: &str, declared: usize, fill: u8) -> Result<Vec<u8>, FirmwareError> {
    use base64::Engine as _;
    let compressed = base64::engine::general_purpose::STANDARD
        .decode(encoded.trim())
        .map_err(|err| FirmwareError::Base64(err.to_string()))?;

    let mut image = vec![fill; image_buffer_length(declared)];
    let mut decoder = flate2::read::ZlibDecoder::new(compressed.as_slice());
    // Exactly `declared` bytes, into a buffer that may be longer - the tail keeps its fill. A
    // short read is not an error here: the C# catches the exception and carries on with the note
    // "Possible bad file - usually safe to ignore", because some released files are genuinely
    // truncated by a byte or two and flash correctly anyway.
    // `// C#: ExtLibs/px4uploader/Firmware.cs:116-121`
    let mut written = 0usize;
    while written < declared {
        let Some(slot) = image.get_mut(written..declared) else {
            break;
        };
        match decoder.read(slot) {
            Ok(0) => break,
            Ok(n) => written += n,
            Err(err) if err.kind() == std::io::ErrorKind::Interrupted => {}
            Err(_) => break,
        }
    }

    image.resize(to_multiple_of_four(image.len()), fill);
    Ok(image)
}

/// The CRC-32 table the bootloader uses. Standard polynomial, generated rather than pasted.
fn crc_table() -> &'static [u32; 256] {
    static TABLE: std::sync::OnceLock<[u32; 256]> = std::sync::OnceLock::new();
    TABLE.get_or_init(|| {
        let mut table = [0u32; 256];
        for (index, slot) in table.iter_mut().enumerate() {
            let mut value = u32::try_from(index).unwrap_or(0);
            for _ in 0..8 {
                value = if value & 1 == 1 {
                    0xedb8_8320 ^ (value >> 1)
                } else {
                    value >> 1
                };
            }
            *slot = value;
        }
        table
    })
}

/// The bootloader's CRC step: standard table, caller-supplied state, no final inversion.
#[must_use]
pub fn crc32(bytes: &[u8], mut state: u32) -> u32 {
    let table = crc_table();
    for byte in bytes {
        let index = usize::from(u8::try_from(state & 0xff).unwrap_or(0) ^ byte);
        state = table.get(index).copied().unwrap_or(0) ^ (state >> 8);
    }
    state
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The generated table has to be the one pasted into the C#.
    #[test]
    fn the_crc_table_matches_the_one_in_the_c_sharp() {
        // Spot values from ExtLibs/px4uploader/Firmware.cs:44-75.
        let table = crc_table();
        assert_eq!(table[0], 0x0000_0000);
        assert_eq!(table[1], 0x7707_3096);
        assert_eq!(table[2], 0xee0e_612c);
        assert_eq!(table[7], 0x9e64_95a3);
        assert_eq!(table[128], 0xedb8_8320);
        assert_eq!(table[255], 0x2d02_ef8d);
    }

    /// Zero-initialised and not inverted, which is what makes it not a standard CRC-32.
    ///
    /// If somebody swaps in an off-the-shelf crc32 this fails, which is the point: the failure
    /// mode otherwise is a verify error after a flash that worked, and the natural response to
    /// that is to flash again.
    #[test]
    fn the_crc_is_not_a_standard_crc32() {
        // Standard CRC-32 of "123456789" is 0xCBF43926. This one starts at 0 and does not invert,
        // so it must NOT produce that.
        let ours = crc32(b"123456789", 0);
        assert_ne!(
            ours, 0xCBF4_3926,
            "this is the zero-seeded uninverted variant, not standard CRC-32"
        );
        // And it is self-consistent: the standard value is recoverable the usual way.
        let standard = !crc32(b"123456789", 0xFFFF_FFFF);
        assert_eq!(standard, 0xCBF4_3926);
    }

    /// The state carries between calls, which is how the padding loop works.
    #[test]
    fn the_crc_is_resumable() {
        let whole = crc32(b"hello world", 0);
        let part = crc32(b"hello ", 0);
        assert_eq!(crc32(b"world", part), whole);
    }

    /// `image_size + (image_size % 4)` is not a round-up, and turning it into one changes the CRC.
    #[test]
    fn the_image_buffer_length_is_the_c_sharps_arithmetic_not_a_round_up() {
        assert_eq!(image_buffer_length(1000), 1000);
        assert_eq!(image_buffer_length(1001), 1002);
        assert_eq!(image_buffer_length(1002), 1004);
        assert_eq!(image_buffer_length(1003), 1006);
        // A round-up to four would give 1004 for every one of 1001..1004.
        assert_ne!(image_buffer_length(1001), 1004);
    }

    /// The padding loop stops at `pad_length - 1`, which is one word short of the obvious reading.
    #[test]
    fn the_crc_padding_stops_where_the_bootloader_stops() {
        let firmware = Firmware {
            board_id: 9,
            board_revision: 0,
            description: String::new(),
            git_hash: String::new(),
            version: String::new(),
            image: vec![0xAA; 8],
            external_image: Vec::new(),
            declared_image_size: 8,
            declared_external_size: 0,
        };
        let bare = crc32(&[0xAA; 8], 0);
        let one = crc32(&[0xff; 4], bare);
        let two = crc32(&[0xff; 4], one);

        // The loop is `for (i = len; i < padlen - 1; i += 4)`, so it stops a word earlier than the
        // obvious reading of "pad up to padlen". Walked through, for an 8-byte image:
        //   padlen 8:  i=8, 8 < 7  -> no pads
        //   padlen 12: i=8, 8 < 11 -> one pad; i=12, 12 < 11 -> stop
        //   padlen 13: i=8, 8 < 12 -> one pad; i=12, 12 < 12 -> stop
        //   padlen 16: i=8 and i=12 both pad; i=16 -> stop
        // The 13 case is the one that catches a `<=`: it looks like it should take two.
        assert_eq!(firmware.crc(8), bare);
        assert_eq!(firmware.crc(12), one);
        assert_eq!(firmware.crc(13), one, "13 takes one pad word, not two");
        assert_eq!(firmware.crc(16), two);
    }
}
