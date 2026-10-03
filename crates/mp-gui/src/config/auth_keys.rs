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

//! MAVLink signing: `Controls/AuthKeys.cs`, the window SETUP's Advanced page's "Mavlink Signing"
//! opens (`ConfigAdvanced.but_signkey_Click`, `GCSViews/ConfigurationView/ConfigAdvanced.cs:37-40`:
//! `new AuthKeys().Show()`), over the key store `ExtLibs/ArduPilot/Mavlink/MAVAuthKeys.cs` and
//! the link's signing (`mp_link::signing`).
//!
//! What it shows, as `AuthKeys.resx` lays it out in a 622 x 227 form: a grid at (12, 12),
//! 598 x 167, of the stored keys - "Friendly Name", a "Use" button, and the key in base64 filling
//! the rest - and under it Add, Disable Signing, Save, and a label that every 190 ms says
//! "Using Key: <name>, Signed Packets: <n>": the stored key the shown vehicle's packets last passed
//! with ("None/Unknown" for none) and the signed packets read since the start of the second
//! (`AuthKeys.cs:135-152`; "...." until its first tick).
//!
//! What it does (`AuthKeys.cs:11-133`):
//!
//! * Add: an empty row, then "Please enter a friendly name" - Cancel leaves the empty row - then
//!   "Please enter your pass phrase/sentence ..."; OK scores the phrase and says how strong it is
//!   ("Password Strength: 130 Strong"), then `MAVAuthKeys.AddKey` stores the SHA-256 of it under
//!   the name; either way the store is saved and the grid filled from it again;
//! * Use, on a row: `setupSigning` with that row's key - `SETUP_SIGNING` to the shown vehicle,
//!   and signing on, the key adopted from the vehicle's first signed packet;
//! * Disable Signing: `setupSigning` with nothing - zeros sent, signing off;
//! * Save: the store written;
//! * a row selected by its header and the Delete key: the key removed from the store, saved only
//!   by Save.
//!
//! The store, `MAVAuthKeys`: `authkeys.xml` in the user data directory, the `DataContractSerializer`
//! XML of a `Dictionary<string, AuthKey>` encrypted with `Crypto` - AES-256 (Rijndael's 128-bit
//! block) in CBC with PKCS7 padding, its key and IV the C#'s constants with the first network
//! interface's MAC address written over their first bytes (`ExtLibs/Utilities/Crypto.cs:13-61`).
//! Read once, the first time anything asks; every link checks signatures against it.
//!
//! Where this is not the C#, each written at its site:
//!
//! * the form is drawn over SETUP, modal, where the C#'s is a free form; a second click replaces
//!   it with a fresh one, as the MAVLink Inspector's is (`config/mavlink_inspector.rs`);
//! * the store is read at the application's first frame, where the C#'s static constructor reads
//!   it the first time the window opens or a signed packet arrives - so a link has the keys
//!   before its first signed packet either way;
//! * the first interface's MAC is read from `/sys/class/net` on Linux - the interface with the
//!   lowest index, the loopback's no address, as Mono lists them first; other systems have no
//!   way here without `unsafe`, and use the constants as they are, as the C# does when the MAC
//!   cannot be read. A store written by the C# on Windows, keyed by an adapter's MAC, does not
//!   read here, and the C#'s `log.Error` is the console's line;
//! * what the C# throws - Use on the row Add left empty, Delete on it, a Save that cannot write -
//!   goes on the status line (the owner's ruling of 2026-09-25), where the C# shows its unhandled
//!   exception's box; the strength of the phrase keeps its box, a report and not an error;
//! * with no vehicle shown, Use and Disable Signing do nothing, where the C# signs to its
//!   placeholder `MAVState`;
//! * the grid's columns, rows and header are WinForms' defaults (`RowHeadersWidth` 41, a
//!   column 100, a row 22), which the `.resx` does not hold; a stored key is a 32-byte key to
//!   the link, cut or padded as `setupSigning` does, where the C#'s receive path hashes a key
//!   of any length;
//! * a key removed and another added go to the end of the store, where .NET's `Dictionary`
//!   reuses the freed slot, so the C#'s order after a deletion can differ.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use aes::Aes256;
use base64::Engine as _;
use cbc::cipher::{
    BlockDecryptMut as _, BlockEncryptMut as _, KeyIvInit as _, block_padding::Pkcs7,
};
use gpui::{
    AnyElement, Context, FocusHandle, KeyDownEvent, SharedString, Window, div, prelude::*, px, rgb,
};
use mp_link::LinkSender;
use mp_vehicle::VehicleId;

use super::optional::{InputBox, button};
use crate::MissionPlanner;
use crate::textfield::KeyOutcome;
use crate::ui::theme;

// ---------------------------------------------------------------------------------------------
// The words and places of the Designer and the handlers.
// ---------------------------------------------------------------------------------------------

/// The form's caption, `$this.Text`. `// C#: Controls/AuthKeys.resx ($this.Text)`
pub const FORM_TEXT: &str = "AuthKeys";
/// `$this.ClientSize`. `// C#: Controls/AuthKeys.resx ($this.ClientSize)`
pub const CLIENT: (f32, f32) = (622.0, 227.0);
/// `dataGridView1`'s `Location` and `Size`. `// C#: Controls/AuthKeys.resx (dataGridView1.*)`
pub const GRID_AT: (f32, f32, f32, f32) = (12.0, 12.0, 598.0, 167.0);
/// `but_add`. `// C#: Controls/AuthKeys.resx (but_add.*)`
pub const ADD_AT: (f32, f32, f32, f32) = (12.0, 185.0, 75.0, 23.0);
/// `but_disablesigning`. `// C#: Controls/AuthKeys.resx (but_disablesigning.*)`
pub const DISABLE_AT: (f32, f32, f32, f32) = (93.0, 185.0, 75.0, 23.0);
/// `but_save`. `// C#: Controls/AuthKeys.resx (but_save.*)`
pub const SAVE_AT: (f32, f32, f32, f32) = (535.0, 185.0, 75.0, 23.0);
/// `lbl_sgnpkts.Location`. `// C#: Controls/AuthKeys.resx (lbl_sgnpkts.Location)`
pub const LABEL_AT: (f32, f32) = (205.0, 195.0);
/// `but_add.Text`.
pub const ADD: &str = "Add";
/// `but_disablesigning.Text`.
pub const DISABLE: &str = "Disable Signing";
/// `but_save.Text`.
pub const SAVE: &str = "Save";
/// `lbl_sgnpkts.Text` until the timer's first tick.
pub const LABEL: &str = "....";
/// The columns' `HeaderText`: `FName`, `Use`, `Key`.
pub const HEADERS: [&str; 3] = ["Friendly Name", "Use", "Key"];
/// `Use.Width`. `// C#: Controls/AuthKeys.resx (Use.Width)`
pub const USE_WIDTH: f32 = 50.0;
/// The `Use` button's text in every row, `dataGridView1_RowsAdded`.
/// `// C#: Controls/AuthKeys.cs:125-128`
pub const USE: &str = "Use";
/// `DataGridView.RowHeadersWidth`'s default.
const ROW_HEADER_WIDTH: f32 = 41.0;
/// `DataGridViewColumn.Width`'s default, `FName`'s.
const NAME_WIDTH: f32 = 100.0;
/// A row's height, `DataGridViewRow.Height`'s default.
const ROW_HEIGHT: f32 = 22.0;
/// The header's height at the form's font.
const HEADER_HEIGHT: f32 = 23.0;

/// `timer1.Interval`, as the constructor sets it over the Designer's 150.
/// `// C#: Controls/AuthKeys.cs:19`
pub const TICK: Duration = Duration::from_millis(190);

/// `InputBox.Show("Name", "Please enter a friendly name", ref name)`.
/// `// C#: Controls/AuthKeys.cs:59`
pub const NAME_TITLE: &str = "Name";
/// Its question.
pub const NAME_PROMPT: &str = "Please enter a friendly name";
/// `InputBox.Show("Input Seed", ...)`. `// C#: Controls/AuthKeys.cs:65`
pub const SEED_TITLE: &str = "Input Seed";
/// Its question.
pub const SEED_PROMPT: &str = "Please enter your pass phrase/sentence\nNumbers, Lower Case, Upper Case, Symbols, and 12+ chars long using atleast 2 of each";

/// `"None/Unknown"`: the label's name for a key not in the store. `// C#: Controls/AuthKeys.cs:137`
pub const UNKNOWN: &str = "None/Unknown";

/// `NullReferenceException.Message`: Use or Delete on the row Add left without a name or key.
pub const NULL_REFERENCE: &str = "Object reference not set to an instance of an object.";
/// `FormatException.Message` of `Convert.FromBase64String`.
pub const NOT_BASE64: &str = "The input is not a valid Base-64 string as it contains a non-base 64 character, more than two padding characters, or an illegal character among the padding characters.";

/// The store's file under the user data directory. `// C#: ExtLibs/ArduPilot/Mavlink/MAVAuthKeys.cs:18`
pub const KEY_FILE: &str = "authkeys.xml";

// ---------------------------------------------------------------------------------------------
// `Crypto`: the store's encryption.
// ---------------------------------------------------------------------------------------------

/// `Crypto.Key`, before the MAC is written over it. `// C#: ExtLibs/Utilities/Crypto.cs:13-19`
pub const CRYPTO_KEY: [u8; 32] = [
    0xd1, 0x3c, 0x35, 0x6f, 0xb5, 0x0d, 0x87, 0xf0, 0x92, 0x07, 0x6d, 0xab, 0x76, 0x82, 0x36, 0x0a,
    0x13, 0x5a, 0x77, 0xfe, 0x77, 0xf3, 0x7f, 0xa8, 0xa4, 0x04, 0x11, 0x46, 0x68, 0x2d, 0x48, 0xa1,
];

/// `Crypto.IV`, likewise. `// C#: ExtLibs/Utilities/Crypto.cs:21-25`
pub const CRYPTO_IV: [u8; 16] = [
    0x6d, 0x2d, 0xf5, 0x34, 0xc7, 0x60, 0xc5, 0x33, 0xe2, 0xa3, 0xd7, 0xc3, 0xf3, 0x39, 0xf2, 0x16,
];

/// `new Crypto()`'s key and IV: the MAC's bytes copied over the start of each - `Array.Copy`,
/// which throws for a MAC longer than the IV before it copies anything, and the `catch` leaves
/// both as they were.
/// `// C#: ExtLibs/Utilities/Crypto.cs:35-60`
#[must_use]
pub fn crypto_key(mac: &[u8]) -> ([u8; 32], [u8; 16]) {
    let mut key = CRYPTO_KEY;
    let mut iv = CRYPTO_IV;
    if mac.len() <= iv.len() {
        for (slot, byte) in iv.iter_mut().zip(mac) {
            *slot = *byte;
        }
        for (slot, byte) in key.iter_mut().zip(mac) {
            *slot = *byte;
        }
    }
    (key, iv)
}

/// `CryptoStream` over `CreateEncryptor()`: AES-256-CBC, PKCS7.
#[must_use]
pub fn encrypt(plain: &[u8], mac: &[u8]) -> Vec<u8> {
    let (key, iv) = crypto_key(mac);
    let Ok(cipher) = cbc::Encryptor::<Aes256>::new_from_slices(&key, &iv) else {
        return Vec::new();
    };
    let mut buffer = vec![0u8; plain.len() + 16];
    if let Some(start) = buffer.get_mut(..plain.len()) {
        start.copy_from_slice(plain);
    }
    cipher
        .encrypt_padded_mut::<Pkcs7>(&mut buffer, plain.len())
        .map(<[u8]>::to_vec)
        .unwrap_or_default()
}

/// `CryptoStream` over `CreateDecryptor()`; `None` where the C#'s read throws.
#[must_use]
pub fn decrypt(cipher_text: &[u8], mac: &[u8]) -> Option<Vec<u8>> {
    let (key, iv) = crypto_key(mac);
    let cipher = cbc::Decryptor::<Aes256>::new_from_slices(&key, &iv).ok()?;
    let mut buffer = cipher_text.to_vec();
    cipher
        .decrypt_padded_mut::<Pkcs7>(&mut buffer)
        .ok()
        .map(<[u8]>::to_vec)
}

/// `NetworkInterface.GetAllNetworkInterfaces().FirstOrDefault().GetPhysicalAddress()
/// .GetAddressBytes()` as Mono answers on Linux: the interface with the lowest index - Mono
/// lists them as `getifaddrs` returns them, links in index order - and no bytes for the
/// loopback, whose hardware address Mono leaves null.
#[cfg(target_os = "linux")]
#[must_use]
pub fn first_mac() -> Vec<u8> {
    /// `ARPHRD_LOOPBACK`.
    const LOOPBACK: &str = "772";
    let read = |path: PathBuf| std::fs::read_to_string(path).unwrap_or_default();
    let Ok(entries) = std::fs::read_dir("/sys/class/net") else {
        return Vec::new();
    };
    let first = entries
        .filter_map(Result::ok)
        .filter_map(|entry| {
            let index = read(entry.path().join("ifindex"))
                .trim()
                .parse::<u32>()
                .ok()?;
            Some((index, entry.path()))
        })
        .min_by_key(|(index, _)| *index);
    let Some((_, path)) = first else {
        return Vec::new();
    };
    if read(path.join("type")).trim() == LOOPBACK {
        return Vec::new();
    }
    read(path.join("address"))
        .trim()
        .split(':')
        .filter_map(|byte| u8::from_str_radix(byte, 16).ok())
        .collect()
}

/// Elsewhere there is no reading it without `unsafe`: the constants as they are, as the C#
/// leaves them when the MAC cannot be read.
#[cfg(not(target_os = "linux"))]
#[must_use]
pub fn first_mac() -> Vec<u8> {
    Vec::new()
}

// ---------------------------------------------------------------------------------------------
// `MAVAuthKeys`: the store.
// ---------------------------------------------------------------------------------------------

/// `MAVAuthKeys.AuthKey`. `// C#: ExtLibs/ArduPilot/Mavlink/MAVAuthKeys.cs:31-38`
#[derive(Clone, PartialEq, Eq)]
pub struct AuthKey {
    /// `Name`.
    pub name: String,
    /// `Key`.
    pub key: Vec<u8>,
}

impl core::fmt::Debug for AuthKey {
    /// Never prints the key.
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("AuthKey")
            .field("name", &self.name)
            .finish_non_exhaustive()
    }
}

impl AuthKey {
    /// The key as the link holds one: 32 bytes, cut or padded.
    #[must_use]
    pub fn link_key(&self) -> [u8; 32] {
        let mut key = [0u8; 32];
        for (slot, byte) in key.iter_mut().zip(&self.key) {
            *slot = *byte;
        }
        key
    }
}

/// `MAVAuthKeys.Keys`, a `Dictionary<string, AuthKey>` in its order, and where it is kept.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct KeyStore {
    /// The dictionary: each name with its key.
    pub keys: Vec<(String, AuthKey)>,
    /// `keyfile`, or none where there is no user data directory.
    pub file: Option<PathBuf>,
    /// The MAC the file is encrypted under.
    pub mac: Vec<u8>,
}

/// The XML's root: `MAVAuthKeys.AuthKeys`, a nested class's data contract name.
const ROOT: &str = "MAVAuthKeys.AuthKeys";
/// `xmlns:i`, which `DataContractSerializer` declares on the root.
const INSTANCE_NS: &str = "http://www.w3.org/2001/XMLSchema-instance";

/// The first child element of `node` named `name`.
fn child<'a, 'input>(
    node: roxmltree::Node<'a, 'input>,
    name: &str,
) -> Option<roxmltree::Node<'a, 'input>> {
    node.children()
        .find(|child| child.is_element() && child.tag_name().name() == name)
}

/// An element's text, its pieces joined.
fn text_of(node: roxmltree::Node<'_, '_>) -> String {
    node.children()
        .filter_map(|child| child.text())
        .collect::<String>()
}

/// Text as `XmlDictionaryWriter` escapes it: `&`, `<` and `>`.
fn escape(text: &str) -> String {
    text.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
}

impl KeyStore {
    /// `Load()`: the file read if it is there; a file that does not decrypt or parse leaves the
    /// store empty, and the C#'s `log.Error` is a line on the console.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVAuthKeys.cs:40-43, 71-93`
    #[must_use]
    pub fn load(file: Option<PathBuf>, mac: Vec<u8>) -> Self {
        let mut store = Self {
            keys: Vec::new(),
            file,
            mac,
        };
        let Some(path) = store.file.as_deref() else {
            return store;
        };
        let Ok(bytes) = std::fs::read(path) else {
            return store;
        };
        match decrypt(&bytes, &store.mac)
            .and_then(|plain| String::from_utf8(plain).ok())
            .and_then(|text| Self::from_xml(&text))
        {
            Some(keys) => store.keys = keys,
            None => eprintln!("{}: not a key store this machine wrote", path.display()),
        }
        store
    }

    /// The store at its place: the user data directory's `authkeys.xml`, under this machine's
    /// first MAC.
    #[must_use]
    pub fn load_default() -> Self {
        Self::load(
            mp_settings::user_data_directory().map(|dir| dir.join(KEY_FILE)),
            first_mac(),
        )
    }

    /// `AddKey(name, seed)`: the SHA-256 of the seed's UTF-8 stored under the name, replacing a
    /// key of that name where it stands.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVAuthKeys.cs:45-55`
    pub fn add_key(&mut self, name: &str, seed: &str) {
        let key = mp_mavlink::SigningKey::from_passphrase(seed)
            .as_bytes()
            .to_vec();
        let entry = AuthKey {
            name: name.to_owned(),
            key,
        };
        if let Some((_, held)) = self.keys.iter_mut().find(|(held, _)| held == name) {
            *held = entry;
        } else {
            self.keys.push((name.to_owned(), entry));
        }
    }

    /// `Keys.Remove(name)`: whether it was there.
    pub fn remove(&mut self, name: &str) -> bool {
        let before = self.keys.len();
        self.keys.retain(|(held, _)| held != name);
        self.keys.len() != before
    }

    /// The name of the stored key equal to `key` - the first, as the C#'s loop breaks.
    /// `// C#: Controls/AuthKeys.cs:139-149`
    #[must_use]
    pub fn name_of(&self, key: &[u8; 32]) -> Option<&str> {
        self.keys
            .iter()
            .find(|(_, held)| held.key.as_slice() == key)
            .map(|(name, _)| name.as_str())
    }

    /// The keys as every link checks a signature against them.
    #[must_use]
    pub fn link_keys(&self) -> Vec<[u8; 32]> {
        self.keys.iter().map(|(_, key)| key.link_key()).collect()
    }

    /// The store handed to every link: `MAVAuthKeys.Keys` is static, read by the receive path.
    pub fn publish(&self) {
        mp_link::signing::set_auth_keys(self.link_keys());
    }

    /// `DataContractSerializer.WriteObject` of the dictionary: no declaration, the root declaring
    /// `xmlns:i`, each pair an `AuthKeys` element of `Key` and `Value`, the value's members in
    /// order - `Key` in base64, then `Name`.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVAuthKeys.cs:26-38, 60-68`
    #[must_use]
    pub fn to_xml(&self) -> String {
        if self.keys.is_empty() {
            return format!("<{ROOT} xmlns:i=\"{INSTANCE_NS}\"/>");
        }
        let mut out = format!("<{ROOT} xmlns:i=\"{INSTANCE_NS}\">");
        for (name, key) in &self.keys {
            out.push_str(&format!(
                "<AuthKeys><Key>{}</Key><Value><Key>{}</Key><Name>{}</Name></Value></AuthKeys>",
                escape(name),
                base64::engine::general_purpose::STANDARD.encode(&key.key),
                escape(&key.name)
            ));
        }
        out.push_str(&format!("</{ROOT}>"));
        out
    }

    /// `ReadObject`: `None` where the C# throws - not the root it wrote, a pair without its key,
    /// a key that is not base64, a name twice (the dictionary's `Add`).
    #[must_use]
    pub fn from_xml(text: &str) -> Option<Vec<(String, AuthKey)>> {
        let document = roxmltree::Document::parse(text).ok()?;
        let root = document.root_element();
        if root.tag_name().name() != ROOT {
            return None;
        }
        let mut keys: Vec<(String, AuthKey)> = Vec::new();
        for pair in root
            .children()
            .filter(|node| node.is_element() && node.tag_name().name() == "AuthKeys")
        {
            let name = text_of(child(pair, "Key")?);
            let value = child(pair, "Value")?;
            let key = base64::engine::general_purpose::STANDARD
                .decode(text_of(child(value, "Key")?).trim())
                .ok()?;
            let inner = child(value, "Name").map(text_of).unwrap_or_default();
            if keys.iter().any(|(held, _)| *held == name) {
                return None;
            }
            keys.push((name, AuthKey { name: inner, key }));
        }
        Some(keys)
    }

    /// `Save()`: the XML, encrypted, written over the file. The error the C# throws, where the
    /// file cannot be written.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVAuthKeys.cs:57-69`
    pub fn save(&self) -> Result<(), String> {
        let Some(path) = self.file.as_deref() else {
            return Err("There is no user data directory to save the keys in.".to_owned());
        };
        write(path, &encrypt(self.to_xml().as_bytes(), &self.mac))
    }
}

/// `new FileStream(keyfile, FileMode.Create)` and the write.
fn write(path: &Path, bytes: &[u8]) -> Result<(), String> {
    std::fs::write(path, bytes).map_err(|error| format!("{}: {error}", path.display()))
}

// ---------------------------------------------------------------------------------------------
// The pass phrase's strength.
// ---------------------------------------------------------------------------------------------

/// `char.IsUpper`, `IsLower`, `IsNumber`, `IsLetter` and `IsSymbol || IsPunctuation` of one
/// UTF-16 unit: .NET's categories in ASCII; outside it Rust's Unicode properties stand in, and a
/// symbol or punctuation mark is anything visible that is neither letter nor number. A
/// surrogate is none of them, as .NET's `char` overloads have it.
fn classes(unit: u16) -> (bool, bool, bool, bool, bool) {
    let Some(c) = char::from_u32(u32::from(unit)) else {
        return (false, false, false, false, false);
    };
    if c.is_ascii() {
        (
            c.is_ascii_uppercase(),
            c.is_ascii_lowercase(),
            c.is_ascii_digit(),
            c.is_ascii_alphabetic(),
            c.is_ascii_punctuation(),
        )
    } else {
        let symbol = !c.is_alphanumeric() && !c.is_whitespace() && !c.is_control();
        (
            c.is_uppercase(),
            c.is_lowercase(),
            c.is_numeric(),
            c.is_alphabetic(),
            symbol,
        )
    }
}

/// The score `but_add_Click` gives a pass phrase, counted over its UTF-16 units as `string`'s
/// `Count` and `Length` are.
/// `// C#: Controls/AuthKeys.cs:69-100`
#[must_use]
pub fn strength(input: &str) -> i64 {
    let units: Vec<u16> = input.encode_utf16().collect();
    let len = i64::try_from(units.len()).unwrap_or(i64::MAX);
    let count = |pick: &dyn Fn((bool, bool, bool, bool, bool)) -> bool, units: &[u16]| {
        i64::try_from(units.iter().filter(|unit| pick(classes(**unit))).count()).unwrap_or(0)
    };
    let mut score = 0;
    // chars
    score += len * 4;
    // upper
    let n = count(&|c| c.0, &units);
    if n > 0 {
        score += (len - n) * 2;
    }
    // lower
    let n = count(&|c| c.1, &units);
    if n > 0 {
        score += (len - n) * 2;
    }
    // number
    let n = count(&|c| c.2, &units);
    if n > 0 {
        score += n * 4;
    }
    // symbols
    let n = count(&|c| c.4, &units);
    if n > 0 {
        score += n * 6;
    }
    // middle number or symbol: `Skip(1).Take(len - 2)`, nothing when that is not positive.
    let middle = units
        .get(1..units.len().saturating_sub(1).max(1))
        .unwrap_or(&[]);
    let n = count(&|c| c.4 || c.2, middle);
    if n > 0 {
        score += n * 2;
    }
    // letters only
    let n = count(&|c| c.3, &units);
    score += if len == n { -len } else { 0 };
    // numbers only
    let n = count(&|c| c.2, &units);
    score += if len == n { -len } else { 0 };
    score
}

/// The box after the phrase: "WEAK - it will be added, but please pick a better password" to 40,
/// "Good" to 60, "Strong" above.
/// `// C#: Controls/AuthKeys.cs:102-107`
#[must_use]
pub fn strength_text(score: i64) -> String {
    if score <= 40 {
        format!(
            "Password Strength: {score} WEAK - it will be added, but please pick a better password"
        )
    } else if score <= 60 {
        format!("Password Strength: {score} Good")
    } else {
        format!("Password Strength: {score} Strong")
    }
}

// ---------------------------------------------------------------------------------------------
// The form.
// ---------------------------------------------------------------------------------------------

/// A grid row: `FName`'s and `Key`'s values, `None` in the row Add makes until it is named.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Row {
    /// `FName`.
    pub name: Option<String>,
    /// `Key`, base64.
    pub key: Option<String>,
}

/// The question Add is asking, and the row it is for.
#[derive(Debug)]
pub enum Asking {
    /// "Please enter a friendly name".
    Name(InputBox),
    /// The pass phrase, for the row named `name`.
    Seed {
        /// The row's name.
        name: String,
        /// The box.
        input: InputBox,
    },
}

/// The strength box, and the key it holds back until its OK.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Strength {
    /// The box's text.
    pub text: String,
    /// The name the key goes under.
    pub name: String,
    /// The phrase.
    pub seed: String,
}

/// `AuthKeys`: the grid, what Add is asking, and the label.
#[derive(Debug)]
pub struct AuthKeysForm {
    /// The grid's rows.
    pub rows: Vec<Row>,
    /// The row selected by its header, for Delete.
    pub selected: Option<usize>,
    /// Add's question.
    pub asking: Option<Asking>,
    /// Add's strength box.
    pub strength: Option<Strength>,
    /// `lbl_sgnpkts.Text`.
    pub label: String,
    /// The count the label shows, `Mavlink2Signed`, for a test to compare.
    pub signed: u32,
    /// Whether the shown vehicle is being signed to (`MAV.signing`), `None` with no vehicle,
    /// as of the last tick: not shown by the C#'s window, a fact for a test.
    pub signing: Option<bool>,
    /// When the timer last ticked.
    pub last_tick: Option<Instant>,
}

impl AuthKeysForm {
    /// The constructor: `LoadKeys`, the timer started, the label "....".
    /// `// C#: Controls/AuthKeys.cs:11-21`
    #[must_use]
    pub fn new(store: &KeyStore, now: Instant) -> Self {
        let mut form = Self {
            rows: Vec::new(),
            selected: None,
            asking: None,
            strength: None,
            label: LABEL.to_owned(),
            signed: 0,
            signing: None,
            last_tick: Some(now),
        };
        form.load_keys(store);
        form
    }

    /// `LoadKeys`: a row per stored key, its name and its key in base64.
    /// `// C#: Controls/AuthKeys.cs:28-37`
    pub fn load_keys(&mut self, store: &KeyStore) {
        self.rows = store
            .keys
            .iter()
            .map(|(name, key)| Row {
                name: Some(name.clone()),
                key: Some(base64::engine::general_purpose::STANDARD.encode(&key.key)),
            })
            .collect();
        self.selected = None;
    }
}

/// The window, the store behind it, and how often it has been opened, held with the Advanced
/// page.
#[derive(Debug, Default)]
pub struct AuthKeysWindow {
    /// `MAVAuthKeys.Keys`, once read.
    pub store: Option<KeyStore>,
    /// The form, while it is open.
    pub window: Option<AuthKeysForm>,
    /// How many times it has been opened.
    pub opened: usize,
}

impl AuthKeysWindow {
    /// The store, read the first time anything asks and handed to every link.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVAuthKeys.cs:40-43`
    pub fn store(&mut self) -> &mut KeyStore {
        self.store.get_or_insert_with(|| {
            let store = KeyStore::load_default();
            store.publish();
            store
        })
    }

    /// `new AuthKeys().Show()`: a fresh form over the store.
    /// `// C#: GCSViews/ConfigurationView/ConfigAdvanced.cs:37-40; Controls/AuthKeys.cs:11-21`
    pub fn show(&mut self, now: Instant) {
        self.opened += 1;
        let form = AuthKeysForm::new(self.store(), now);
        self.window = Some(form);
    }

    /// The close box: the form and its timer gone.
    pub fn close(&mut self) {
        self.window = None;
    }

    /// Once a frame: the store read at the first, and while the form is open its timer -
    /// `timer1_Tick`'s label from the shown vehicle's signing.
    /// `// C#: Controls/AuthKeys.cs:135-152`
    pub fn tick(&mut self, link: Option<&(LinkSender, VehicleId)>, now: Instant) {
        self.store();
        let Some(store) = self.store.as_ref() else {
            return;
        };
        let Some(form) = self.window.as_mut() else {
            return;
        };
        if form
            .last_tick
            .is_some_and(|last| now.duration_since(last) < TICK)
        {
            return;
        }
        form.last_tick = Some(now);
        let key = link.and_then(|(sender, id)| sender.signing(*id)?.key);
        let name = key
            .as_ref()
            .and_then(|key| store.name_of(key))
            .unwrap_or(UNKNOWN);
        let signed = link.map_or(0, |(sender, _)| sender.signed_packets());
        form.label = format!("Using Key: {name}, Signed Packets: {signed}");
        form.signed = signed;
        form.signing =
            link.map(|(sender, id)| sender.signing(*id).is_some_and(|state| state.signing));
    }

    /// `but_add_Click`'s start: an empty row (`Rows.Add()`, its Use button by `RowsAdded`), and
    /// the name asked.
    /// `// C#: Controls/AuthKeys.cs:53-59, 125-128`
    pub fn add(&mut self) {
        let Some(form) = self.window.as_mut() else {
            return;
        };
        form.rows.push(Row {
            name: None,
            key: None,
        });
        form.asking = Some(Asking::Name(InputBox::new(NAME_TITLE, NAME_PROMPT, "")));
    }

    /// A key typed into Add's question: Enter is OK and Escape Cancel.
    pub fn prompt_key(&mut self, event: &KeyDownEvent) -> Option<bool> {
        let form = self.window.as_mut()?;
        let input = match form.asking.as_mut()? {
            Asking::Name(input) | Asking::Seed { input, .. } => input,
        };
        match input.field.key(event) {
            KeyOutcome::Submitted => Some(true),
            KeyOutcome::Cancelled => Some(false),
            KeyOutcome::Changed | KeyOutcome::Ignored => None,
        }
    }

    /// Add's question answered: the name OK'd goes in the empty row and the phrase is asked, a
    /// name cancelled leaves the row; the phrase OK'd is scored and its box shown, the key
    /// stored at the box's OK; a phrase cancelled saves the store and fills the grid again.
    /// The answer OK'd, for `InputBox` to keep; the C#'s error, if the save threw.
    /// `// C#: Controls/AuthKeys.cs:59-117`
    pub fn prompt_done(&mut self, ok: bool) -> (Option<InputBox>, Result<(), String>) {
        let Some(asking) = self.window.as_mut().and_then(|form| form.asking.take()) else {
            return (None, Ok(()));
        };
        match asking {
            Asking::Name(input) => {
                if !ok {
                    return (None, Ok(()));
                }
                let name = input.field.value().to_owned();
                if let Some(form) = self.window.as_mut() {
                    if let Some(row) = form.rows.last_mut() {
                        row.name = Some(name.clone());
                    }
                    form.asking = Some(Asking::Seed {
                        name,
                        input: InputBox::new(SEED_TITLE, SEED_PROMPT, ""),
                    });
                }
                (Some(input), Ok(()))
            }
            Asking::Seed { name, input } => {
                if !ok {
                    return (None, self.end_add());
                }
                let seed = input.field.value().to_owned();
                if let Some(form) = self.window.as_mut() {
                    form.strength = Some(Strength {
                        text: strength_text(strength(&seed)),
                        name,
                        seed,
                    });
                }
                (Some(input), Ok(()))
            }
        }
    }

    /// The strength box's OK: `MAVAuthKeys.AddKey(name, input)`, then the end of Add.
    /// `// C#: Controls/AuthKeys.cs:109-117`
    pub fn strength_ok(&mut self) -> Result<(), String> {
        let Some(strength) = self.window.as_mut().and_then(|form| form.strength.take()) else {
            return Ok(());
        };
        let store = self.store();
        store.add_key(&strength.name, &strength.seed);
        store.publish();
        self.end_add()
    }

    /// `EndEdit()`, `Save()`, `LoadKeys()`.
    /// `// C#: Controls/AuthKeys.cs:112-116`
    fn end_add(&mut self) -> Result<(), String> {
        let saved = self.store().save();
        let store = self.store().clone();
        if let Some(form) = self.window.as_mut() {
            form.load_keys(&store);
        }
        saved
    }

    /// Save: `MAVAuthKeys.Save()`. `// C#: Controls/AuthKeys.cs:23-26, 39-42`
    pub fn save(&mut self) -> Result<(), String> {
        self.store().save()
    }

    /// Use, on row `index`: `setupSigning(MAV.sysid, MAV.compid, "", Convert.FromBase64String(
    /// key))` on the shown vehicle. The C#'s exception for a row with no key or a key that is
    /// not base64.
    /// `// C#: Controls/AuthKeys.cs:44-51`
    pub fn use_row(
        &self,
        index: usize,
        link: Option<&(LinkSender, VehicleId)>,
    ) -> Result<(), String> {
        let Some(row) = self.window.as_ref().and_then(|form| form.rows.get(index)) else {
            return Ok(());
        };
        let key = row
            .key
            .as_deref()
            .ok_or_else(|| NULL_REFERENCE.to_owned())?;
        let key = base64::engine::general_purpose::STANDARD
            .decode(key)
            .map_err(|_| NOT_BASE64.to_owned())?;
        if let Some((sender, id)) = link {
            sender.setup_signing(*id, "", Some(&key));
        }
        Ok(())
    }

    /// Disable Signing: `setupSigning(MAV.sysid, MAV.compid, "")`.
    /// `// C#: Controls/AuthKeys.cs:130-133`
    pub fn disable(link: Option<&(LinkSender, VehicleId)>) {
        if let Some((sender, id)) = link {
            sender.setup_signing(*id, "", None);
        }
    }

    /// A row's header clicked: the row selected.
    pub fn select(&mut self, index: usize) {
        if let Some(form) = self.window.as_mut()
            && index < form.rows.len()
        {
            form.selected = Some(index);
        }
    }

    /// Delete with a row selected: the row gone, then `UserDeletedRow` -
    /// `MAVAuthKeys.Keys.Remove(name)`, unsaved; the C#'s exception for the row Add left
    /// unnamed, which is gone from the grid all the same.
    /// `// C#: Controls/AuthKeys.cs:120-123`
    pub fn delete_selected(&mut self) -> Result<(), String> {
        let Some(form) = self.window.as_mut() else {
            return Ok(());
        };
        let Some(index) = form.selected.take() else {
            return Ok(());
        };
        if index >= form.rows.len() {
            return Ok(());
        }
        let row = form.rows.remove(index);
        let name = row.name.ok_or_else(|| NULL_REFERENCE.to_owned())?;
        let store = self.store();
        store.remove(&name);
        store.publish();
        Ok(())
    }
}

/// Facts a UI test asserts on.
pub fn record_facts(holder: &AuthKeysWindow) {
    use crate::facts::record;
    record("config.authkeys.window", holder.window.is_some());
    record("config.authkeys.opened", holder.opened);
    if let Some(store) = holder.store.as_ref() {
        record("config.authkeys.store", store.keys.len());
        record(
            "config.authkeys.store.names",
            store
                .keys
                .iter()
                .map(|(name, _)| name.as_str())
                .collect::<Vec<_>>()
                .join(","),
        );
        record(
            "config.authkeys.file",
            store.file.as_deref().is_some_and(Path::exists),
        );
    }
    let Some(form) = holder.window.as_ref() else {
        return;
    };
    record("config.authkeys.rows", form.rows.len());
    for (index, row) in form.rows.iter().enumerate() {
        record(
            format!("config.authkeys.row.{index}"),
            format!(
                "{}|{}",
                row.name.as_deref().unwrap_or(""),
                row.key.as_deref().unwrap_or("")
            ),
        );
    }
    record("config.authkeys.label", &form.label);
    record("config.authkeys.signed", form.signed);
    record(
        "config.authkeys.signing",
        match form.signing {
            Some(true) => "on",
            Some(false) => "off",
            None => "none",
        },
    );
    record(
        "config.authkeys.selected",
        form.selected
            .map_or_else(|| "none".to_owned(), |index| index.to_string()),
    );
    record(
        "config.authkeys.prompt",
        match form.asking.as_ref() {
            Some(Asking::Name(input) | Asking::Seed { input, .. }) => input.title,
            None => "none",
        },
    );
    record(
        "config.authkeys.box",
        form.strength
            .as_ref()
            .map_or("none", |strength| strength.text.as_str()),
    );
}

// ---------------------------------------------------------------------------------------------
// Drawing.
// ---------------------------------------------------------------------------------------------

/// What a handler threw, on the status line (the owner's ruling).
fn status(this: &mut MissionPlanner, result: Result<(), String>) {
    if let Err(error) = result {
        this.file_status = Some(error);
    }
}

/// A grid cell's text.
fn cell(x: f32, width: f32, text: impl Into<SharedString>) -> AnyElement {
    div()
        .absolute()
        .left(px(x))
        .top_0()
        .w(px(width))
        .h(px(ROW_HEIGHT))
        .px_1()
        .flex()
        .items_center()
        .overflow_hidden()
        .whitespace_nowrap()
        .border_r_1()
        .border_b_1()
        .border_color(rgb(theme::BORDER))
        .text_xs()
        .child(text.into())
        .into_any_element()
}

/// The grid: the header, then each row - its header (which selects it), its name, its Use
/// button and its key.
fn grid(form: &AuthKeysForm, focus: &FocusHandle, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let (gx, gy, gw, gh) = GRID_AT;
    let key_width = gw - 2.0 - ROW_HEADER_WIDTH - NAME_WIDTH - USE_WIDTH;
    let columns = [
        (ROW_HEADER_WIDTH, NAME_WIDTH, HEADERS[0]),
        (ROW_HEADER_WIDTH + NAME_WIDTH, USE_WIDTH, HEADERS[1]),
        (
            ROW_HEADER_WIDTH + NAME_WIDTH + USE_WIDTH,
            key_width,
            HEADERS[2],
        ),
    ];
    let mut header = div()
        .relative()
        .h(px(HEADER_HEIGHT))
        .bg(rgb(theme::PANEL))
        .child(cell(0.0, ROW_HEADER_WIDTH, ""));
    for (x, width, text) in columns {
        header = header.child(cell(x, width, text));
    }
    let focus_on_click = focus.clone();
    let mut body = crate::probe::measured("authkeys-grid", div())
        .id("authkeys-grid")
        .absolute()
        .left(px(gx))
        .top(px(gy))
        .w(px(gw))
        .h(px(gh))
        .overflow_hidden()
        .bg(rgb(theme::BG))
        .border_1()
        .border_color(rgb(theme::BORDER))
        .text_color(rgb(theme::TEXT))
        .track_focus(focus)
        .on_click(cx.listener(move |_this, _event, window, cx| {
            focus_on_click.focus(window, cx);
        }))
        .on_key_down(cx.listener(|this, event: &KeyDownEvent, _window, cx| {
            if event.keystroke.key == "delete" {
                let result = this.extra.auth_keys.delete_selected();
                status(this, result);
                cx.notify();
            }
        }))
        .child(header);
    for (index, row) in form.rows.iter().enumerate() {
        let selected = form.selected == Some(index);
        let row_header = crate::probe::measured(format!("authkeys-row-{index}"), div())
            .id(SharedString::from(format!("authkeys-row-{index}")))
            .absolute()
            .left_0()
            .top_0()
            .w(px(ROW_HEADER_WIDTH))
            .h(px(ROW_HEIGHT))
            .border_r_1()
            .border_b_1()
            .border_color(rgb(theme::BORDER))
            .bg(rgb(if selected {
                theme::ACCENT
            } else {
                theme::PANEL
            }))
            .cursor_pointer()
            .on_click(cx.listener(move |this, _event, _window, cx| {
                this.extra.auth_keys.select(index);
                cx.notify();
            }));
        let use_button = crate::probe::measured(format!("authkeys-use-{index}"), div())
            .id(SharedString::from(format!("authkeys-use-{index}")))
            .absolute()
            .left(px(ROW_HEADER_WIDTH + NAME_WIDTH + 2.0))
            .top(px(2.0))
            .w(px(USE_WIDTH - 4.0))
            .h(px(ROW_HEIGHT - 4.0))
            .flex()
            .items_center()
            .justify_center()
            .rounded_sm()
            .border_1()
            .border_color(rgb(theme::BORDER))
            .bg(rgb(theme::ACTION))
            .text_xs()
            .cursor_pointer()
            .hover(|style| style.border_color(rgb(theme::ACCENT)))
            .child(USE)
            .on_click(cx.listener(move |this, _event, _window, cx| {
                let link = this.telemetry.send_handle();
                let result = this.extra.auth_keys.use_row(index, link.as_ref());
                status(this, result);
                cx.notify();
            }));
        let line = div()
            .relative()
            .h(px(ROW_HEIGHT))
            .bg(rgb(if selected { theme::ACTION } else { theme::BG }))
            .child(row_header)
            .child(cell(
                ROW_HEADER_WIDTH,
                NAME_WIDTH,
                row.name.clone().unwrap_or_default(),
            ))
            .child(cell(ROW_HEADER_WIDTH + NAME_WIDTH, USE_WIDTH, ""))
            .child(use_button)
            .child(cell(
                ROW_HEADER_WIDTH + NAME_WIDTH + USE_WIDTH,
                key_width,
                row.key.clone().unwrap_or_default(),
            ));
        body = body.child(line);
    }
    body.into_any_element()
}

/// The form: its caption and close box, the grid, the three buttons and the label.
fn form(
    form: &AuthKeysForm,
    focus: &FocusHandle,
    prompt: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let viewport = window.viewport_size();
    let prompt = prompt.clone();
    let client = div()
        .relative()
        .w(px(CLIENT.0))
        .h(px(CLIENT.1))
        .child(grid(form, focus, cx))
        .child(button(
            "authkeys-but_add",
            ADD,
            ADD_AT,
            true,
            move |this, window, cx| {
                this.extra.auth_keys.add();
                prompt.focus(window, cx);
            },
            cx,
        ))
        .child(button(
            "authkeys-but_disablesigning",
            DISABLE,
            DISABLE_AT,
            true,
            |this, _window, _cx| {
                let link = this.telemetry.send_handle();
                AuthKeysWindow::disable(link.as_ref());
            },
            cx,
        ))
        .child(button(
            "authkeys-but_save",
            SAVE,
            SAVE_AT,
            true,
            |this, _window, _cx| {
                let result = this.extra.auth_keys.save();
                status(this, result);
            },
            cx,
        ))
        .child(
            crate::probe::measured("authkeys-label", div())
                .absolute()
                .left(px(LABEL_AT.0))
                .top(px(LABEL_AT.1))
                .text_xs()
                .whitespace_nowrap()
                .text_color(rgb(theme::TEXT))
                .child(form.label.clone()),
        );
    let caption = div()
        .flex()
        .items_center()
        .justify_between()
        .px_2()
        .py_1()
        .border_b_1()
        .border_color(rgb(theme::BORDER))
        .child(div().text_xs().text_color(rgb(theme::DIM)).child(FORM_TEXT))
        .child(crate::ui::action(
            "authkeys-close",
            "X",
            theme::TEXT,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                this.extra.auth_keys.close();
                cx.notify();
            }),
        ));
    let body = crate::probe::measured("authkeys", div())
        .flex()
        .flex_col()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .rounded_md()
        .child(caption)
        .child(client);
    gpui::deferred(
        gpui::anchored()
            .position(gpui::point(px(0.0), px(0.0)))
            .child(
                div()
                    .id("authkeys-backdrop")
                    .w(viewport.width)
                    .h(viewport.height)
                    .flex()
                    .items_center()
                    .justify_center()
                    .occlude()
                    .child(body),
            ),
    )
    .with_priority(1)
    .into_any_element()
}

/// Add's question answered by a button or a key: the answer kept as `InputBox` keeps it, and what
/// the C# throws on the status line.
/// `// C#: ExtLibs/Controls/InputBox.cs:178-184`
fn finish_prompt(this: &mut MissionPlanner, ok: bool, window: &mut Window, cx: &mut gpui::App) {
    let (answered, result) = this.extra.auth_keys.prompt_done(ok);
    if let Some(input) = answered {
        input.remember(&mut this.persisted);
    }
    status(this, result);
    // The phrase asked next takes the keys as the name did.
    if this
        .extra
        .auth_keys
        .window
        .as_ref()
        .is_some_and(|form| form.asking.is_some())
    {
        this.extra_focus.auth_keys_prompt.focus(window, cx);
    }
}

/// Add's question: `InputBox`, its caption, its question - a line each - its box, OK and Cancel.
/// `// C#: ExtLibs/Controls/InputBox.cs:62-190`
fn question(
    input: &InputBox,
    focus: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let viewport = window.viewport_size();
    let buttons = vec![
        crate::ui::action(
            "authkeys-prompt-ok",
            "OK",
            theme::ACCENT,
            true,
            cx.listener(|this, _event: &(), window, cx| {
                finish_prompt(this, true, window, cx);
                cx.notify();
            }),
        ),
        crate::ui::action(
            "authkeys-prompt-cancel",
            "Cancel",
            theme::DIM,
            true,
            cx.listener(|this, _event: &(), window, cx| {
                finish_prompt(this, false, window, cx);
                cx.notify();
            }),
        ),
    ];
    let dialog = crate::probe::measured("authkeys-prompt-box", div())
        .flex()
        .flex_col()
        .gap_2()
        .w(px(340.0))
        .p_3()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .rounded_md()
        .child(
            div()
                .text_xs()
                .text_color(rgb(theme::DIM))
                .child(input.title),
        )
        .children(input.prompt.lines().map(|line| {
            div()
                .text_sm()
                .text_color(rgb(theme::TEXT))
                .child(line.to_owned())
        }))
        .child(crate::textfield::text_field(
            "authkeys-prompt-value",
            &input.field,
            focus,
            focus.is_focused(window),
            px(310.0),
            cx.listener(|this, event: &KeyDownEvent, window, cx| {
                if let Some(ok) = this.extra.auth_keys.prompt_key(event) {
                    finish_prompt(this, ok, window, cx);
                }
                cx.notify();
            }),
        ))
        .child(div().flex().justify_end().gap_2().children(buttons));
    gpui::deferred(
        gpui::anchored()
            .position(gpui::point(px(0.0), px(0.0)))
            .child(
                div()
                    .id("authkeys-prompt-backdrop")
                    .w(viewport.width)
                    .h(viewport.height)
                    .flex()
                    .items_center()
                    .justify_center()
                    .occlude()
                    .child(dialog),
            ),
    )
    .with_priority(2)
    .into_any_element()
}

/// The form over the window, and over it Add's question or its strength box.
pub fn overlay(
    holder: &AuthKeysWindow,
    focus: &FocusHandle,
    prompt: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    let open = holder.window.as_ref()?;
    let mut layers = div().child(form(open, focus, prompt, window, cx));
    if let Some(Asking::Name(input) | Asking::Seed { input, .. }) = open.asking.as_ref() {
        layers = layers.child(question(input, prompt, window, cx));
    }
    if let Some(strength) = open.strength.as_ref() {
        let ok = crate::ui::action(
            "authkeys-box-ok",
            "OK",
            theme::ACCENT,
            true,
            cx.listener(|this, _event: &(), _window, cx| {
                let result = this.extra.auth_keys.strength_ok();
                status(this, result);
                cx.notify();
            }),
        );
        // `CustomMessageBox.Show(text)`: no caption.
        layers = layers.child(crate::config::servo_output::modal(
            "authkeys-box",
            "",
            &strength.text,
            false,
            vec![ok],
            window,
        ));
    }
    Some(layers.into_any_element())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config_coverage::source::{csharp, resx};

    /// `authkeys.xml` as the C#'s own `MAVAuthKeys.Save` wrote it under Mono on this Linux
    /// machine - `AddKey("bench", "Correct Horse 42!")` and `AddKey("Second key",
    /// "pass<&>\"phrase")` - the loopback first, so encrypted under `Crypto`'s constants as they
    /// are (`MAVAuthKeys.cs` and `Crypto.cs` compiled with `mcs` and run, 2026-10-02).
    const CSHARP_TWO_KEYS: &str = "7fI6tgJai3SXz8qbMfbbHFM6bBGfE/KNtrkRXPr/EJylrcq5VoJOUMXob40qNwTYN7zROXZQiZWuoYPBCMSqKDlzFabaDT4NSjvRS2w9i7R1znnBB0KzGHxFrH4V2qeWkCGF4hrm7bftrpk0J422hygZqKrx4cPUQ5ronupuF1Dyc3DK1aWaNbTsPYewwz5dxiBV0DJeMbj/ugfl3Q9eLoxfpjeC64zDrWbuAUuekk2sF9vkucwpvkDtsC0fj+Z+VA3wIqKdsQQJAESWMBCrw4ktBc+xAEfmxBk7HfM8SQfoV1rCt61bDg2TxoO5jr3OCvaolF9w8Rj/T1OJmk5pYB+gzGc0plonlQNsWaX6RzB1Y27HCXlqpj4/rJlbRhRJmGCbMS0Y77QCCjI4fF1hv9/tKHjvY2A5LyB8NKsM+JCuVVL4Tz/hZVdVdTcaGkxxF2avp5xv01phlugl8QuQRgJbpEHFvV4Xsm/0xUTT0RQ=";

    /// The same with a third, `AddKey("a<b&c>\"d'é", "x")`: a name that needs escaping.
    const CSHARP_ESCAPED: &str = "7fI6tgJai3SXz8qbMfbbHFM6bBGfE/KNtrkRXPr/EJylrcq5VoJOUMXob40qNwTYN7zROXZQiZWuoYPBCMSqKDlzFabaDT4NSjvRS2w9i7R1znnBB0KzGHxFrH4V2qeWkCGF4hrm7bftrpk0J422hygZqKrx4cPUQ5ronupuF1Dyc3DK1aWaNbTsPYewwz5dxiBV0DJeMbj/ugfl3Q9eLoxfpjeC64zDrWbuAUuekk2sF9vkucwpvkDtsC0fj+Z+VA3wIqKdsQQJAESWMBCrw4ktBc+xAEfmxBk7HfM8SQfoV1rCt61bDg2TxoO5jr3OCvaolF9w8Rj/T1OJmk5pYB+gzGc0plonlQNsWaX6RzB1Y27HCXlqpj4/rJlbRhRJmGCbMS0Y77QCCjI4fF1hv9/tKHjvY2A5LyB8NKsM+JBN1EsKno9BJfJbdx/BkwFhxcpS4BXH9AhV1TCB8uarbXm/ngJd59G1jJ4nbeWhQYBa3e2SrNQRSMgEFRQXK5FrIzisxfke5rZLOP+liETv0k0cRt6uQcP1IA9jYygNHTsI/cTUdUhe7VPDvNCU3JzPjAeGT+g1CqKMtBS2U0fmLq7nGxt9hH2N4vLK01EQOtTT5DrT9F+MHyrqAQw/fIHakTDntQu09qGDvFW/AgtPJCVIlCm26lHHHgmymaVhfONXNE4TCbffABxDXMPiDTuU";

    /// The plain XML the C#'s `DataContractSerializer` wrote for the two keys.
    const CSHARP_XML: &str = "<MAVAuthKeys.AuthKeys xmlns:i=\"http://www.w3.org/2001/XMLSchema-instance\"><AuthKeys><Key>bench</Key><Value><Key>oBPRD6pBgzO8Tbf6nQxJ5aCQaorJmv5BfT35Mr9oyDM=</Key><Name>bench</Name></Value></AuthKeys><AuthKeys><Key>Second key</Key><Value><Key>hSSgSygFHL8rO3MfVyEqu8fE+j806s2srG7zXaK0XP8=</Key><Name>Second key</Name></Value></AuthKeys></MAVAuthKeys.AuthKeys>";

    /// The tests that hand the store to every link, one at a time: the store is the process's.
    static STORE: std::sync::Mutex<()> = std::sync::Mutex::new(());

    fn bytes(text: &str) -> Vec<u8> {
        base64::engine::general_purpose::STANDARD
            .decode(text)
            .unwrap()
    }

    fn scratch(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("authkeys-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    /// The store the C# wrote reads here, and the same keys written here are its bytes exactly:
    /// the XML, the escaping, the encryption.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVAuthKeys.cs:45-93; ExtLibs/Utilities/Crypto.cs`
    #[test]
    fn the_store_reads_and_writes_the_csharps_file() {
        let dir = scratch("csharp");
        let file = dir.join(KEY_FILE);
        std::fs::write(&file, bytes(CSHARP_TWO_KEYS)).unwrap();
        let store = KeyStore::load(Some(file.clone()), Vec::new());
        let names: Vec<&str> = store.keys.iter().map(|(name, _)| name.as_str()).collect();
        assert_eq!(names, ["bench", "Second key"]);
        assert_eq!(store.keys[0].1.name, "bench");
        assert_eq!(
            store.keys[0].1.key,
            mp_mavlink::SigningKey::from_passphrase("Correct Horse 42!").as_bytes()
        );
        assert_eq!(store.to_xml(), CSHARP_XML);

        let mut ours = KeyStore {
            file: Some(file.clone()),
            ..KeyStore::default()
        };
        ours.add_key("bench", "Correct Horse 42!");
        ours.add_key("Second key", "pass<&>\"phrase");
        ours.save().unwrap();
        assert_eq!(std::fs::read(&file).unwrap(), bytes(CSHARP_TWO_KEYS));
        ours.add_key("a<b&c>\"d'é", "x");
        ours.save().unwrap();
        assert_eq!(std::fs::read(&file).unwrap(), bytes(CSHARP_ESCAPED));
        assert_eq!(KeyStore::load(Some(file.clone()), Vec::new()), ours);

        // Under another MAC it does not read, and the store is empty.
        let other = KeyStore::load(Some(file), vec![0, 0x42, 0x38, 0xc6, 0x31, 0x5d]);
        assert!(other.keys.is_empty());
        let _ = std::fs::remove_dir_all(dir);
    }

    /// `Crypto`'s key and IV: the MAC over their first bytes; one longer than the IV leaves both.
    #[test]
    fn the_mac_is_written_over_the_key_and_iv() {
        let mac = [1, 2, 3, 4, 5, 6];
        let (key, iv) = crypto_key(&mac);
        assert_eq!(&key[..6], &mac);
        assert_eq!(&key[6..], &CRYPTO_KEY[6..]);
        assert_eq!(&iv[..6], &mac);
        assert_eq!(&iv[6..], &CRYPTO_IV[6..]);
        assert_eq!(crypto_key(&[9; 20]), (CRYPTO_KEY, CRYPTO_IV));
        assert_eq!(crypto_key(&[]), (CRYPTO_KEY, CRYPTO_IV));
        let sealed = encrypt(b"plain", &mac);
        assert_eq!(decrypt(&sealed, &mac).as_deref(), Some(&b"plain"[..]));
        assert_ne!(decrypt(&sealed, &[]).as_deref(), Some(&b"plain"[..]));
    }

    /// An empty store is the serializer's empty element; a key added again replaces itself where
    /// it stands; one removed is gone; the name of a key is the first stored with it.
    #[test]
    fn the_store_adds_replaces_and_removes() {
        let mut store = KeyStore::default();
        assert_eq!(
            store.to_xml(),
            "<MAVAuthKeys.AuthKeys xmlns:i=\"http://www.w3.org/2001/XMLSchema-instance\"/>"
        );
        assert_eq!(KeyStore::from_xml(&store.to_xml()), Some(Vec::new()));
        store.add_key("a", "one");
        store.add_key("b", "two");
        store.add_key("a", "three");
        let names: Vec<&str> = store.keys.iter().map(|(name, _)| name.as_str()).collect();
        assert_eq!(names, ["a", "b"]);
        let three = *mp_mavlink::SigningKey::from_passphrase("three").as_bytes();
        assert_eq!(store.name_of(&three), Some("a"));
        assert_eq!(store.link_keys()[0], three);
        assert!(store.remove("a"));
        assert!(!store.remove("a"));
        assert_eq!(store.name_of(&three), None);
        assert_eq!(KeyStore::from_xml("<Other/>"), None);
        // A name twice: the dictionary's `Add` throws, and nothing is read.
        let pair = "<AuthKeys><Key>a</Key><Value><Key>AA==</Key><Name>a</Name></Value></AuthKeys>";
        let twice = format!("<{ROOT} xmlns:i=\"{INSTANCE_NS}\">{pair}{pair}</{ROOT}>");
        assert_eq!(KeyStore::from_xml(&twice), None);
        let once = format!("<{ROOT} xmlns:i=\"{INSTANCE_NS}\">{pair}</{ROOT}>");
        assert_eq!(KeyStore::from_xml(&once).map(|keys| keys.len()), Some(1));
        assert!(KeyStore::default().save().is_err(), "nowhere to save");
    }

    /// The pass phrase's score, the C#'s arithmetic, and its three boxes.
    /// `// C#: Controls/AuthKeys.cs:69-107`
    #[test]
    fn the_phrase_is_scored_as_the_csharp_scores_it() {
        // 17 chars: 68; 2 upper: +30; 10 lower: +14; 2 digits: +8; "!": +6; the digits in
        // the middle: +4.
        assert_eq!(strength("Correct Horse 42!"), 130);
        assert_eq!(strength_text(130), "Password Strength: 130 Strong");
        // Letters only: 4 * 4, + (4 - 4) * 2 for the lower, - 4.
        assert_eq!(strength("abcd"), 12);
        // Digits only: 16 + 16, + 4 for the two in the middle, - 4.
        assert_eq!(strength("1234"), 32);
        assert_eq!(strength(""), 0);
        // One letter: 4, - 1.
        assert_eq!(strength("a"), 3);
        assert_eq!(
            strength_text(40),
            "Password Strength: 40 WEAK - it will be added, but please pick a better password"
        );
        assert_eq!(strength_text(60), "Password Strength: 60 Good");
    }

    fn holder(dir: &Path) -> AuthKeysWindow {
        AuthKeysWindow {
            store: Some(KeyStore::load(Some(dir.join(KEY_FILE)), Vec::new())),
            ..AuthKeysWindow::default()
        }
    }

    fn answer(holder: &mut AuthKeysWindow, text: &str, ok: bool) {
        if let Some(Asking::Name(input) | Asking::Seed { input, .. }) =
            holder.window.as_mut().unwrap().asking.as_mut()
        {
            input.field.set(text);
        }
        let (answered, result) = holder.prompt_done(ok);
        assert_eq!(answered.is_some(), ok);
        result.unwrap();
    }

    /// Add: an empty row and the name asked; the phrase asked; its strength shown; the key stored
    /// and saved at the box's OK, the grid filled again. Cancel on the name leaves the empty row,
    /// whose Use and Delete throw.
    /// `// C#: Controls/AuthKeys.cs:53-128`
    #[test]
    fn add_asks_scores_stores_and_saves() {
        let _store = STORE
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let dir = scratch("add");
        let mut window = holder(&dir);
        window.show(Instant::now());
        assert_eq!(window.opened, 1);
        assert_eq!(window.window.as_ref().unwrap().label, LABEL);
        window.add();
        let form = window.window.as_ref().unwrap();
        assert_eq!(
            form.rows,
            [Row {
                name: None,
                key: None
            }]
        );
        assert!(
            matches!(form.asking, Some(Asking::Name(ref input)) if input.prompt == NAME_PROMPT)
        );
        answer(&mut window, "bench", true);
        let form = window.window.as_ref().unwrap();
        assert_eq!(form.rows[0].name.as_deref(), Some("bench"));
        assert!(
            matches!(form.asking, Some(Asking::Seed { ref input, .. }) if input.title == SEED_TITLE)
        );
        answer(&mut window, "Correct Horse 42!", true);
        let form = window.window.as_ref().unwrap();
        assert_eq!(
            form.strength.as_ref().unwrap().text,
            "Password Strength: 130 Strong"
        );
        assert!(
            window.store.as_ref().unwrap().keys.is_empty(),
            "not before the box's OK"
        );
        window.strength_ok().unwrap();
        let form = window.window.as_ref().unwrap();
        assert_eq!(
            form.rows,
            [Row {
                name: Some("bench".to_owned()),
                key: Some("oBPRD6pBgzO8Tbf6nQxJ5aCQaorJmv5BfT35Mr9oyDM=".to_owned()),
            }]
        );
        assert!(dir.join(KEY_FILE).exists());
        assert_eq!(
            KeyStore::load(Some(dir.join(KEY_FILE)), Vec::new())
                .keys
                .len(),
            1
        );

        // The phrase cancelled: saved, the grid filled again - the named row gone.
        window.add();
        answer(&mut window, "half", true);
        answer(&mut window, "", false);
        assert_eq!(window.window.as_ref().unwrap().rows.len(), 1);

        // The name cancelled: the empty row stays, and Use and Delete on it throw.
        window.add();
        answer(&mut window, "", false);
        let form = window.window.as_ref().unwrap();
        assert_eq!(form.rows.len(), 2);
        assert!(form.asking.is_none());
        assert_eq!(window.use_row(1, None), Err(NULL_REFERENCE.to_owned()));
        window.select(1);
        assert_eq!(window.delete_selected(), Err(NULL_REFERENCE.to_owned()));
        assert_eq!(
            window.window.as_ref().unwrap().rows.len(),
            1,
            "gone all the same"
        );

        // Delete on the named row: out of the store, the file untouched until Save.
        window.select(0);
        window.delete_selected().unwrap();
        assert!(window.store.as_ref().unwrap().keys.is_empty());
        assert_eq!(
            KeyStore::load(Some(dir.join(KEY_FILE)), Vec::new())
                .keys
                .len(),
            1
        );
        window.save().unwrap();
        assert!(
            KeyStore::load(Some(dir.join(KEY_FILE)), Vec::new())
                .keys
                .is_empty()
        );
        let _ = std::fs::remove_dir_all(dir);
    }

    /// Use and Disable Signing over a real link, and the label: Use sends `SETUP_SIGNING` with
    /// the row's key, twice, and turns signing on; the vehicle signing with it is adopted and the
    /// label names the key and counts its packets; Disable Signing sends zeros and turns it off.
    /// `// C#: Controls/AuthKeys.cs:44-51, 130-152`
    #[test]
    fn use_and_disable_drive_the_links_signing() {
        use crate::telemetry::Telemetry;
        use crate::telemetry::scripted::until;
        use mp_mavlink_dialects::all::{DIALECT, MavMessage, SetupSigning};
        use mp_transport::Transport as _;
        use mp_transport::testing::Loopback;

        let _store = STORE
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let dir = scratch("use");
        let mut window = holder(&dir);
        if let Some(store) = window.store.as_mut() {
            store.add_key("bench", "use_and_disable_drive_the_links_signing");
            store.publish();
        }
        let key = window.store.as_ref().unwrap().link_keys()[0];
        let now = Instant::now();
        window.show(now);
        window.tick(None, now + TICK);
        assert_eq!(
            window.window.as_ref().unwrap().label,
            "Using Key: None/Unknown, Signed Packets: 0"
        );

        let (mut vehicle, gcs) = Loopback::pair();
        let link = mp_link::Link::from_transport(
            Box::new(gcs),
            mp_link::LinkConfig {
                send_heartbeat: false,
                stream_rate_hz: 0,
                ..mp_link::LinkConfig::default()
            },
        );
        let telemetry = Telemetry::over(link, "loopback");
        vehicle.write_all(&mp_link::testing::heartbeat(0)).unwrap();
        let id = VehicleId::new(1, 1);
        until("the vehicle", || {
            telemetry
                .send_handle()
                .is_some_and(|(_, shown)| shown == id)
        });
        let link = telemetry.send_handle().unwrap();

        window.use_row(0, Some(&link)).unwrap();
        until("signing on", || {
            link.0.signing(id).is_some_and(|state| state.signing)
        });
        let mut setups = Vec::new();
        let mut bytes = Vec::new();
        until("two SETUP_SIGNING", || {
            let mut buf = [0u8; 4096];
            let n = vehicle.read(&mut buf).unwrap_or(0);
            bytes.extend_from_slice(&buf[..n]);
            setups.clear();
            let mut decoder = mp_mavlink::FrameDecoder::new();
            decoder.push_and_drain(&bytes, &DIALECT, |frame| {
                if frame.msgid == <SetupSigning as mp_mavlink::Message>::ID
                    && let Some(MavMessage::SetupSigning(m)) =
                        MavMessage::decode(frame.msgid, frame.payload)
                {
                    setups.push(m);
                }
            });
            setups.len() >= 2
        });
        assert_eq!(setups.len(), 2);
        assert_eq!(setups[0].secret_key, key);
        assert!(setups[0].initial_timestamp > 0);
        assert_eq!(
            (setups[0].target_system, setups[0].target_component),
            (1, 1)
        );

        // The vehicle signs with it: adopted from the store, and the label says so.
        let signed = mp_link::signing::sign_frame(
            &mp_link::testing::heartbeat(1),
            &key,
            0,
            mp_link::signing::timestamp_now(),
        )
        .unwrap();
        vehicle.write_all(&signed).unwrap();
        until("the key adopted", || {
            link.0
                .signing(id)
                .is_some_and(|state| state.key == Some(key))
        });
        window.tick(Some(&link), now + TICK * 2);
        let label = window.window.as_ref().unwrap().label.clone();
        assert!(
            label.starts_with("Using Key: bench, Signed Packets: "),
            "{label}"
        );

        AuthKeysWindow::disable(Some(&link));
        until("signing off", || {
            link.0.signing(id).is_some_and(|state| !state.signing)
        });
        window.tick(Some(&link), now + TICK * 3);
        let label = window.window.as_ref().unwrap().label.clone();
        assert!(
            label.starts_with("Using Key: None/Unknown, Signed Packets: "),
            "{label}"
        );
        let _ = std::fs::remove_dir_all(dir);
    }

    /// The places and words are the Designer's and the `.resx`'s.
    #[test]
    fn the_places_and_words_are_the_resx() {
        let (Some(text), Some(designer), Some(source)) = (
            csharp("Controls/AuthKeys.resx"),
            csharp("Controls/AuthKeys.Designer.cs"),
            csharp("Controls/AuthKeys.cs"),
        ) else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        let values = resx(&text);
        let get = |key: &str| values.get(key).cloned().unwrap_or_default();
        let pair = |(x, y): (f32, f32)| format!("{x}, {y}");
        assert_eq!(get("$this.ClientSize"), pair(CLIENT));
        assert_eq!(get("$this.Text"), FORM_TEXT);
        assert_eq!(get("dataGridView1.Location"), pair((GRID_AT.0, GRID_AT.1)));
        assert_eq!(get("dataGridView1.Size"), pair((GRID_AT.2, GRID_AT.3)));
        for (name, place, words) in [
            ("but_add", ADD_AT, ADD),
            ("but_disablesigning", DISABLE_AT, DISABLE),
            ("but_save", SAVE_AT, SAVE),
        ] {
            assert_eq!(get(&format!("{name}.Location")), pair((place.0, place.1)));
            assert_eq!(get(&format!("{name}.Size")), pair((place.2, place.3)));
            assert_eq!(get(&format!("{name}.Text")), words);
        }
        assert_eq!(get("lbl_sgnpkts.Location"), pair(LABEL_AT));
        assert_eq!(get("lbl_sgnpkts.Text"), LABEL);
        assert_eq!(get("FName.HeaderText"), HEADERS[0]);
        assert_eq!(get("Use.HeaderText"), HEADERS[1]);
        assert_eq!(get("Key.HeaderText"), HEADERS[2]);
        assert_eq!(get("Use.Width"), "50");
        assert!(designer.contains("this.dataGridView1.AllowUserToAddRows = false;"));
        assert!(source.contains("timer1.Interval = 190;"));
        assert!(source.contains(&format!(
            "InputBox.Show(\"{NAME_TITLE}\", \"{NAME_PROMPT}\""
        )));
        let seed = SEED_PROMPT.replace('\n', "\\n");
        assert!(source.contains(&format!("InputBox.Show(\"{SEED_TITLE}\", \"{seed}\"")));
        assert!(source.contains(&format!("var name = \"{UNKNOWN}\";")));
        let crypto = csharp("ExtLibs/Utilities/Crypto.cs").unwrap_or_default();
        assert!(crypto.contains("0xd1, 0x3c, 0x35, 0x6f, 0xb5, 0xd, 0x87, 0xf0"));
        assert!(crypto.contains("0x6d, 0x2d, 0xf5, 0x34, 0xc7, 0x60, 0xc5, 0x33"));
        assert!(crypto.contains("CipherMode.CBC") && crypto.contains("PaddingMode.PKCS7"));
    }

    /// Every fact the GUI script asserts on is recorded here, and every control of this window
    /// it clicks is drawn here.
    #[test]
    fn the_gui_script_names_facts_and_controls_this_window_has() {
        let script = include_str!("../../../../tests/gui/config-signing.gui");
        let source = include_str!("auth_keys.rs");
        let mut facts = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("config.authkeys.") => {
                    let recorded = source.contains(&format!("\"{key}\""))
                        || key.starts_with("config.authkeys.row.");
                    assert!(recorded, "{key} is not recorded");
                    facts += 1;
                }
                (Some("click"), Some(id)) if id.starts_with("authkeys-") => {
                    let drawn = source.contains(&format!("\"{id}\""))
                        || id.starts_with("authkeys-use-")
                        || id.starts_with("authkeys-row-");
                    assert!(drawn, "{id} is not drawn");
                }
                (Some("click"), Some(id)) if id.starts_with("advanced-") => {
                    assert_eq!(id, "advanced-but_signkey");
                }
                _ => {}
            }
        }
        assert!(facts >= 10, "{facts} facts");
    }
}
