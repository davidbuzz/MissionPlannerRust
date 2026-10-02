//! An Intel HEX firmware image as the SiK bootloader takes it: `Radio/IHex.cs`.
//!
//! The data records become blocks by address, a record that starts where another ends merged
//! into it; a type-4 record (the upper 16 address bits) marks the image as needing a banking
//! bootloader and moves the records after it. The checksums are not read, as the C# reads none.

use std::collections::BTreeMap;
use std::fmt;

/// Why an image could not be read: the C#'s exceptions out of `IHex.load`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum HexError {
    /// "invalid IntelHex file": a line that does not start with `:`.
    Invalid,
    /// "no data in IntelHex file".
    NoData,
    /// A record too short or not hex, where the C#'s `Substring` or `Convert.ToByte` throws.
    BadRecord(usize),
    /// Two blocks at one address, where the C#'s `SortedList.Add` throws.
    Overlap(u32),
}

impl fmt::Display for HexError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Invalid => f.write_str("invalid IntelHex file"),
            Self::NoData => f.write_str("no data in IntelHex file"),
            Self::BadRecord(line) => write!(f, "bad record on line {line}"),
            Self::Overlap(address) => write!(
                f,
                "An item with the same key has already been added. Key: {address}"
            ),
        }
    }
}

impl std::error::Error for HexError {}

/// `IHex`: the blocks by address, and whether a banking record was seen.
/// `// C#: Radio/IHex.cs:7-166`
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct IHex {
    /// The blocks, by their first address (`SortedList<uint, byte[]>`).
    pub blocks: BTreeMap<u32, Vec<u8>>,
    /// `bankingDetected`.
    pub banking_detected: bool,
    /// `merge_index`: each block's end, to its start.
    merge_index: BTreeMap<u32, u32>,
    /// `upperaddress`.
    upper_address: u32,
}

impl IHex {
    /// `load`, over the file's text.
    /// `// C#: Radio/IHex.cs:29-90`
    ///
    /// # Errors
    /// What the C# throws.
    pub fn parse(text: &str) -> Result<Self, HexError> {
        let mut hex = Self::default();
        for (number, line) in text.lines().enumerate() {
            // Every line must start with a `:`.
            if !line.starts_with(':') {
                return Err(HexError::Invalid);
            }
            let bad = || HexError::BadRecord(number + 1);
            let field = |from: usize, len: usize| -> Result<u32, HexError> {
                let digits = line.get(from..from + len).ok_or_else(bad)?;
                u32::from_str_radix(digits, 16).map_err(|_| bad())
            };
            // The record type and data length, assuming ihex8; the checksum ignored.
            let length = usize::try_from(field(1, 2)?).map_err(|_| bad())?;
            let mut address = field(3, 4)?;
            let record = field(7, 2)?;
            if record == 0 {
                let mut data = Vec::with_capacity(length);
                for i in 0..length {
                    let byte = field(9 + i * 2, 2)?;
                    data.push(u8::try_from(byte).map_err(|_| bad())?);
                }
                // Add for the banking address.
                address = address.wrapping_add(hex.upper_address << 16);
                hex.insert(address, data)?;
            } else if record == 4 && length == 2 && address == 0 {
                hex.banking_detected = true;
                hex.upper_address = field(9, 4)?;
            }
        }
        if hex.blocks.is_empty() {
            return Err(HexError::NoData);
        }
        Ok(hex)
    }

    /// `insert`: the block merged with one that starts at its end and one that ends at its start.
    /// `// C#: Radio/IHex.cs:116-165`
    ///
    /// # Errors
    /// [`HexError::Overlap`] where the C#'s `Add` throws.
    pub fn insert(&mut self, key: u32, data: Vec<u8>) -> Result<(), HexError> {
        let mut key = key;
        let mut data = data;
        let len = u32::try_from(data.len()).unwrap_or(u32::MAX);
        // Can it merge with the next block?
        let other = key.wrapping_add(len);
        if let Some(next) = self.blocks.remove(&other) {
            let next_len = u32::try_from(next.len()).unwrap_or(u32::MAX);
            self.merge_index.remove(&other.wrapping_add(next_len));
            data.extend_from_slice(&next);
        }
        // A preceding block that ends here.
        if let Some(&start) = self.merge_index.get(&key)
            && let Some(mut previous) = self.blocks.remove(&start)
        {
            self.merge_index.remove(&key);
            previous.extend_from_slice(&data);
            key = start;
            data = previous;
        }
        if self.blocks.contains_key(&key) {
            return Err(HexError::Overlap(key));
        }
        let end = key.wrapping_add(u32::try_from(data.len()).unwrap_or(u32::MAX));
        if self.merge_index.contains_key(&end) {
            return Err(HexError::Overlap(end));
        }
        self.blocks.insert(key, data);
        self.merge_index.insert(end, key);
        Ok(())
    }

    /// The bytes of every block, in address order: what `SearchHex` looks through.
    pub fn bytes(&self) -> impl Iterator<Item = u8> + '_ {
        self.blocks.values().flatten().copied()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Records that run on are one block; a gap makes two; a type-4 record banks what follows.
    #[test]
    fn records_merge_into_blocks() {
        let text = ":0400000001020304F2\n:020004000506EF\n:020010000708DF\n:020020000909CC\n";
        let hex = IHex::parse(text).unwrap();
        assert_eq!(hex.blocks.len(), 3);
        assert_eq!(hex.blocks[&0], [1, 2, 3, 4, 5, 6]);
        assert_eq!(hex.blocks[&0x10], [7, 8]);
        assert_eq!(hex.blocks[&0x20], [9, 9]);
        assert!(!hex.banking_detected);

        let banked = IHex::parse(":020000040001F9\n:0200000011220B\n").unwrap();
        assert!(banked.banking_detected);
        assert_eq!(banked.blocks[&0x1_0000], [0x11, 0x22]);
    }

    /// A block written before the one it follows is merged on the other side.
    #[test]
    fn a_block_before_merges_too() {
        let hex = IHex::parse(":020002000304F5\n:0200000001020B\n").unwrap();
        assert_eq!(hex.blocks.len(), 1);
        assert_eq!(hex.blocks[&0], [1, 2, 3, 4]);
    }

    /// The C#'s failures.
    #[test]
    fn bad_files_are_refused() {
        assert_eq!(IHex::parse("garbage\n"), Err(HexError::Invalid));
        assert_eq!(IHex::parse(":00000001FF\n"), Err(HexError::NoData));
        assert_eq!(IHex::parse(":04000000"), Err(HexError::BadRecord(1)));
        assert_eq!(
            IHex::parse(":0100000001FE\n:0100000002FD\n"),
            Err(HexError::Overlap(0))
        );
        assert_eq!(HexError::Invalid.to_string(), "invalid IntelHex file");
    }
}
