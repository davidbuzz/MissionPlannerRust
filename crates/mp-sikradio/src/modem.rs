//! The modem models: `RFD900` and its subclasses (`SikRadio/RFD900.cs:1728-2700`) - which board
//! each is, the firmware files each takes and how each checks one, the country lock of the
//! RFD900x family - and `ATIResponseToFWVersion`.
//!
//! The programming itself is the session's ([`crate::session::Session::program_firmware`]), as it
//! moves the radio between modes. Not ported, having no callers in the planner: `GetBand`,
//! `GetValidCountryLockOptions`, `GetValidCountryLockOptionsForBand`, `GetCanBeLockedToCountry`
//! and `WriteCounty` (`SikRadio/RFD900.cs:1759-1793, 2198-2235, 2422-2425, 2657-2661`), and the
//! `HB1060` and `HM_TRP` classes, which nothing constructs (`:2533-2572`).

use crate::ihex::IHex;
use crate::uploader::Board;

/// Which `RFD900` subclass `GetModemObject` made.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Model {
    /// `RFD900a`.
    Rfd900a,
    /// `RFD900p`.
    Rfd900p,
    /// `RFD900u`.
    Rfd900u,
    /// `RFD900ux`.
    Rfd900ux,
    /// `RFD900x`.
    Rfd900x,
    /// `RFD900X2`.
    Rfd900x2,
    /// `RFD900UX2`.
    Rfd900ux2,
}

/// Which branch of the class tree a model is on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Family {
    /// `RFD900APU`: the SiK bootloader and an Intel HEX file.
    Apu,
    /// `RFD900xux`: XModem, and a firmware certified for a country-locked modem.
    Xux,
    /// `RFD900xuxRev2`: XModem at a higher baud rate, any file.
    Rev2,
}

/// The modem object: its model, and whether its firmware is DINIO.
/// `// C#: SikRadio/RFD900.cs:1728-1968`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Modem {
    /// The model.
    pub model: Model,
    /// `IsDINIO`: the `ATI` answer said "DINIO".
    pub dinio: bool,
}

impl Modem {
    /// A model's object, not DINIO.
    #[must_use]
    pub const fn new(model: Model) -> Self {
        Self {
            model,
            dinio: false,
        }
    }

    /// `GetModemObject`'s choice from `ATI2`'s board: the RFD models only.
    /// `// C#: SikRadio/RFD900.cs:293-316`
    #[must_use]
    pub fn for_board(board: Board) -> Option<Self> {
        let model = match board {
            Board::RFD900A => Model::Rfd900a,
            Board::RFD900P => Model::Rfd900p,
            Board::RFD900U => Model::Rfd900u,
            Board::RFD900UX => Model::Rfd900ux,
            Board::RFD900X => Model::Rfd900x,
            Board::RFD900X2 => Model::Rfd900x2,
            Board::RFD900UX2 => Model::Rfd900ux2,
            _ => return None,
        };
        Some(Self::new(model))
    }

    /// The model's family.
    #[must_use]
    pub const fn family(self) -> Family {
        match self.model {
            Model::Rfd900a | Model::Rfd900p | Model::Rfd900u => Family::Apu,
            Model::Rfd900ux | Model::Rfd900x => Family::Xux,
            Model::Rfd900x2 | Model::Rfd900ux2 => Family::Rev2,
        }
    }

    /// `Board`.
    /// `// C#: SikRadio/RFD900.cs:2079-2085, 2101-2107, 2123-2129, 2524-2530, 2587-2593, 2679-2685, 2695-2701`
    #[must_use]
    pub const fn board(self) -> Board {
        match self.model {
            Model::Rfd900a => Board::RFD900A,
            Model::Rfd900p => Board::RFD900P,
            Model::Rfd900u => Board::RFD900U,
            Model::Rfd900ux => Board::RFD900UX,
            Model::Rfd900x => Board::RFD900X,
            Model::Rfd900x2 => Board::RFD900X2,
            Model::Rfd900ux2 => Board::RFD900UX2,
        }
    }

    /// `FirmwareFileNameExtensions`: what the custom firmware's dialog shows.
    /// `// C#: SikRadio/RFD900.cs:2031-2037, 2414-2420, 2663-2669`
    #[must_use]
    pub const fn extensions(self) -> &'static [&'static str] {
        match self.family() {
            Family::Apu => &["hex", "ihx"],
            Family::Xux => &["bin"],
            Family::Rev2 => &["gbl"],
        }
    }

    /// `GetFirmwareSearchTokens`: what a firmware for this model holds.
    /// `// C#: SikRadio/RFD900.cs:2074-2077, 2096-2099, 2118-2121, 2519-2522, 2595-2598, 2631-2634`
    #[must_use]
    pub const fn search_tokens(self) -> &'static [&'static str] {
        match self.model {
            Model::Rfd900a => &["RFD900A"],
            Model::Rfd900p => &["RFD900P"],
            Model::Rfd900u => &["RFD900U"],
            Model::Rfd900x => &["RFD900x", "RFD900X"],
            Model::Rfd900ux => &["RFD900ux", "RFD900UX"],
            Model::Rfd900x2 | Model::Rfd900ux2 => &[],
        }
    }

    /// `GetCountryCodeRegPosition`: the register of the country lock, for the RFD900x family.
    /// `// C#: SikRadio/RFD900.cs:2379-2382, 2621-2624`
    #[must_use]
    pub const fn country_register(self) -> Option<u8> {
        match self.family() {
            Family::Apu => None,
            Family::Xux => Some(32),
            Family::Rev2 => Some(51),
        }
    }

    /// The `OpenFileDialog.Filter` `getFirmwareLocal` builds: "Firmware|*.hex;*.ihx".
    /// `// C#: Radio/Sikradio.cs:394-411`
    #[must_use]
    pub fn filter(self) -> String {
        let patterns: Vec<String> = self
            .extensions()
            .iter()
            .map(|ext| format!("*.{ext}"))
            .collect();
        format!("Firmware|{}", patterns.join(";"))
    }

    /// `ShowWrongFirmwareMessageBox`'s text.
    /// `// C#: SikRadio/RFD900.cs:1904-1929`
    #[must_use]
    pub fn wrong_firmware_text(self) -> String {
        let tokens = self.search_tokens();
        let mut text =
            "File doesn't appear to be valid for this radio.  Could not find ".to_owned();
        for (n, token) in tokens.iter().enumerate() {
            if n != 0 {
                if n + 1 >= tokens.len() {
                    text.push_str(" or ");
                } else {
                    text.push_str(", ");
                }
            }
            text.push('"');
            text.push_str(token);
            text.push('"');
        }
        text.push_str(" in File.");
        text
    }
}

/// `SearchTokenUpdate`: one byte of a search for `token`. A byte that does not match starts the
/// search over without looking at itself again, so a token whose first byte repeats inside a
/// near-miss is not found - as the C# does not find it.
/// `// C#: SikRadio/RFD900.cs:1829-1844`
fn search_update(token: &[u8], index: &mut usize, next: u8) -> bool {
    if token.get(*index) == Some(&next) {
        *index += 1;
        if *index >= token.len() {
            return true;
        }
    } else {
        *index = 0;
    }
    false
}

/// `SearchBinary` over a file's bytes: whether any of the tokens is in them.
/// `// C#: SikRadio/RFD900.cs:1852-1875`
#[must_use]
pub fn search_binary(bytes: &[u8], tokens: &[&str]) -> bool {
    tokens.iter().any(|token| {
        let token = token.as_bytes();
        let mut index = 0;
        bytes.iter().any(|b| search_update(token, &mut index, *b))
    })
}

/// `SearchHex`: whether any of the tokens is in the image's blocks, read one after another.
/// `// C#: SikRadio/RFD900.cs:1881-1900`
#[must_use]
pub fn search_hex(image: &IHex, tokens: &[&str]) -> bool {
    tokens.iter().any(|token| {
        let token = token.as_bytes();
        let mut index = 0;
        image.bytes().any(|b| search_update(token, &mut index, b))
    })
}

/// `IsFirmwareCertified`: the file holds "HastaLaVistaBaby".
/// `// C#: SikRadio/RFD900.cs:2374-2377`
#[must_use]
pub fn is_certified(bytes: &[u8]) -> bool {
    search_binary(bytes, &["HastaLaVistaBaby"])
}

/// `RFD900xuxRevN.TCountry`: the country a modem is locked to; `ToString` its name or number.
/// `// C#: SikRadio/RFD900.cs:2347-2358`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Country(pub i32);

impl Country {
    /// `NONE`.
    pub const NONE: Self = Self(0);
    /// `Undefined`.
    pub const UNDEFINED: Self = Self(255);

    /// `GetIsCountryLocked`: anything but `NONE` and `Undefined`.
    /// `// C#: SikRadio/RFD900.cs:2242-2251`
    #[must_use]
    pub const fn is_locked(self) -> bool {
        !matches!(self.0, 0 | 255)
    }
}

impl std::fmt::Display for Country {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let name = match self.0 {
            0 => "NONE",
            1 => "AU",
            2 => "NZ",
            3 => "US",
            4 => "EU",
            5 => "PRC",
            6 => "Ins",
            8 => "India",
            255 => "Undefined",
            other => return write!(f, "{other}"),
        };
        f.write_str(name)
    }
}

/// `ATIResponseToFWVersion`: what is between "RFD" and the first "on" after it, trimmed.
/// `// C#: SikRadio/RFD900.cs:1953-1967`
#[must_use]
pub fn firmware_version(ati: &str) -> Option<String> {
    let rfd = ati.find("RFD")?;
    let on = ati.find("on")?;
    if on > rfd {
        return ati.get(rfd + 3..on).map(|v| v.trim().to_owned());
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The models of `ATI2`'s boards, their files and their checks.
    #[test]
    fn each_model_takes_its_own_firmware() {
        assert_eq!(Modem::for_board(Board::HM_TRP), None, "not an RFD");
        assert_eq!(
            Modem::for_board(Board::RFD900),
            None,
            "the first RFD900 has no class"
        );
        let p = Modem::for_board(Board::RFD900P).unwrap();
        assert_eq!(p.family(), Family::Apu);
        assert_eq!(p.filter(), "Firmware|*.hex;*.ihx");
        assert_eq!(p.country_register(), None);
        let x = Modem::for_board(Board::RFD900X).unwrap();
        assert_eq!(x.filter(), "Firmware|*.bin");
        assert_eq!(x.country_register(), Some(32));
        assert_eq!(
            x.wrong_firmware_text(),
            "File doesn't appear to be valid for this radio.  Could not find \"RFD900x\" or \"RFD900X\" in File."
        );
        assert_eq!(
            p.wrong_firmware_text(),
            "File doesn't appear to be valid for this radio.  Could not find \"RFD900P\" in File."
        );
        let x2 = Modem::for_board(Board::RFD900X2).unwrap();
        assert_eq!(x2.family(), Family::Rev2);
        assert_eq!(x2.country_register(), Some(51));
    }

    /// The search restarts without looking at the byte that broke it, as the C#'s does.
    #[test]
    fn the_search_is_the_csharps() {
        assert!(search_binary(b"xxRFD900Pyy", &["RFD900P"]));
        assert!(
            !search_binary(b"RRFD900P", &["RFD900P"]),
            "the C# misses it"
        );
        assert!(search_binary(b"..RFD900X..", &["RFD900x", "RFD900X"]));
        assert!(is_certified(b"..HastaLaVistaBaby.."));
        // The blocks are read as one run, the search going on across a gap, as the C#'s does.
        let image = IHex::parse(":03000000524644F6\n:0400100039303050DB\n").unwrap();
        assert!(search_hex(&image, &["RFD900P"]));
        let other = IHex::parse(":03000000524644F6\n:0400030039303041DB\n").unwrap();
        assert!(!search_hex(&other, &["RFD900P"]), "an RFD900A's");
    }

    /// The firmware version from `ATI`, and the countries.
    #[test]
    fn versions_and_countries() {
        assert_eq!(
            firmware_version("RFD SiK 2.65 on RFD900P").as_deref(),
            Some("SiK 2.65")
        );
        assert_eq!(firmware_version("SiK 1.9 on HM-TRP"), None);
        assert_eq!(Country(3).to_string(), "US");
        assert_eq!(Country(7).to_string(), "7");
        assert!(Country(4).is_locked());
        assert!(!Country::NONE.is_locked());
        assert!(!Country::UNDEFINED.is_locked());
    }
}
