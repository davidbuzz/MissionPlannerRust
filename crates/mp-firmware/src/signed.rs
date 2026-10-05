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

//! `ExtLibs/Utilities/SignedFW.cs`: ArduPilot's secure boot, as the Secure page
//! (`GCSViews/ConfigurationView/ConfigSecureAP.cs`) drives it - an Ed25519 key pair, a
//! bootloader given the public keys it will accept, and a firmware image signed with the private
//! key.
//!
//! The C# does its Ed25519 with BouncyCastle; here it is `ring`'s, the implementation this
//! application's TLS already carries (`ureq` -> `rustls` -> `ring`), so the page adds no
//! cryptography of its own. The key is the 32-byte seed BouncyCastle's `Ed25519PrivateKeyParameters`
//! holds; the files the page writes and reads are BouncyCastle's too: its `PemWriter`'s PKCS#8
//! `PRIVATE KEY` (the version-2 form, carrying the public key), and the `PRIVATE_KEYV1:` and
//! `PUBLIC_KEYV1:` lines with the key in base64.

use std::io::{Read as _, Write as _};

use base64::Engine as _;
use ring::signature::KeyPair as _;

/// `"PRIVATE_KEYV1:"`, the start of a `_private_key.dat`.
/// `// C#: GCSViews/ConfigurationView/ConfigSecureAP.cs:37-40, 105`
pub const PRIVATE_KEY_HEADER: &str = "PRIVATE_KEYV1:";
/// `"PUBLIC_KEYV1:"`, the start of a `_public_key.dat` and of ArduPilot's own keys.
/// `// C#: ExtLibs/Utilities/SignedFW.cs:64-68`
pub const PUBLIC_KEY_HEADER: &str = "PUBLIC_KEYV1:";

/// ArduPilot's three public keys, which `CreateSignedBL` puts in every bootloader before the
/// user's own.
/// `// C#: ExtLibs/Utilities/SignedFW.cs:63-66`
pub const ARDUPILOT_KEYS: [&str; 3] = [
    "PUBLIC_KEYV1:WJbbpbjOz/yMB3JxnvqyTUInCQdZcStkA0qhn2ldhPI=",
    "PUBLIC_KEYV1:X8jdVqxIIUmCuMSi8IhTZ40VkXW0gbRczzMtdSghqCI=",
    "PUBLIC_KEYV1:snNHkX96F9A+/ISppHZrc1jjPo3jMNN+g2PToKhWSgA=",
];

/// The bootloader's key table's descriptor.
/// `// C#: ExtLibs/Utilities/SignedFW.cs:49`
pub const BL_DESCRIPTOR: [u8; 8] = [0x4e, 0xcf, 0x4e, 0xa5, 0xa6, 0xb6, 0xf7, 0x29];
/// The firmware's app descriptor's.
/// `// C#: ExtLibs/Utilities/SignedFW.cs:90`
pub const APJ_DESCRIPTOR: [u8; 8] = [0x41, 0xa3, 0xe5, 0xf2, 0x65, 0x69, 0x92, 0x07];

/// `sig_len`, `sig_version` and `desc_len`.
/// `// C#: ExtLibs/Utilities/SignedFW.cs:88-89, 105`
const SIG_LEN: u32 = 64;
/// See [`SIG_LEN`].
const SIG_VERSION: u64 = 30437;
/// See [`SIG_LEN`].
const DESC_LEN: usize = 92;

/// PKCS#8's `PrivateKeyInfo` for an Ed25519 key as BouncyCastle encodes one - version 1, the
/// algorithm `1.3.101.112`, the seed as an `OCTET STRING` inside an `OCTET STRING` - up to the
/// seed; the public key follows it as `[1] IMPLICIT BIT STRING`.
const PKCS8_PREFIX: [u8; 16] = [
    0x30, 0x51, 0x02, 0x01, 0x01, 0x30, 0x05, 0x06, 0x03, 0x2b, 0x65, 0x70, 0x04, 0x22, 0x04, 0x20,
];
/// What comes between the seed and the public key.
const PKCS8_PUBLIC: [u8; 3] = [0x81, 0x21, 0x00];

/// The line ending .NET writes: `Environment.NewLine`, which `PemWriter` and Newtonsoft's
/// indented writer use.
pub const NEWLINE: &str = if cfg!(windows) { "\r\n" } else { "\n" };

/// An Ed25519 key pair: `AsymmetricCipherKeyPair` of BouncyCastle's Ed25519 parameters.
#[derive(Clone, PartialEq, Eq)]
pub struct KeyPair {
    /// `Ed25519PrivateKeyParameters.GetEncoded()`: the 32-byte seed.
    seed: [u8; 32],
    /// `Ed25519PublicKeyParameters.GetEncoded()`.
    public: [u8; 32],
}

impl std::fmt::Debug for KeyPair {
    /// The public key only: a private key does not go into logs.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("KeyPair")
            .field("public", &self.public_base64())
            .finish_non_exhaustive()
    }
}

impl KeyPair {
    /// `SignedFW.GenerateKey()`: a new pair from the system's random numbers.
    ///
    /// # Errors
    /// The system has no random numbers to give.
    /// `// C#: ExtLibs/Utilities/SignedFW.cs:19-31`
    pub fn generate() -> Result<Self, String> {
        let mut seed = [0u8; 32];
        ring::rand::SecureRandom::fill(&ring::rand::SystemRandom::new(), &mut seed)
            .map_err(|_| "no random numbers".to_owned())?;
        Self::from_seed(&seed)
    }

    /// `SignedFW.GenerateKey(knownseed)`: the pair whose seed is the first 32 bytes given -
    /// `preseedrandom` copies them into the generator's buffer, and a shorter array throws.
    ///
    /// # Errors
    /// Fewer than 32 bytes, as `Array.Copy` refuses them.
    /// `// C#: ExtLibs/Utilities/SignedFW.cs:33-45, 162-165`
    pub fn from_seed(knownseed: &[u8]) -> Result<Self, String> {
        let seed: [u8; 32] = knownseed
            .get(..32)
            .and_then(|seed| seed.try_into().ok())
            .ok_or("Source array was not long enough. Check srcIndex and length, and the array's lower bounds.")?;
        let pair = ring::signature::Ed25519KeyPair::from_seed_unchecked(&seed)
            .map_err(|err| err.to_string())?;
        let public: [u8; 32] = pair
            .public_key()
            .as_ref()
            .try_into()
            .map_err(|_| "a public key that is not 32 bytes".to_owned())?;
        Ok(Self { seed, public })
    }

    /// The public key, 32 bytes.
    #[must_use]
    pub const fn public(&self) -> &[u8; 32] {
        &self.public
    }

    /// The public key in base64: what `txt_pubkey` shows.
    /// `// C#: GCSViews/ConfigurationView/ConfigSecureAP.cs:49, 108`
    #[must_use]
    pub fn public_base64(&self) -> String {
        base64::engine::general_purpose::STANDARD.encode(self.public)
    }

    /// A `_private_key.dat`'s text: `PRIVATE_KEYV1:` and the seed in base64.
    /// `// C#: GCSViews/ConfigurationView/ConfigSecureAP.cs:105`
    #[must_use]
    pub fn private_dat(&self) -> String {
        format!(
            "{PRIVATE_KEY_HEADER}{}",
            base64::engine::general_purpose::STANDARD.encode(self.seed)
        )
    }

    /// A `_public_key.dat`'s text.
    /// `// C#: GCSViews/ConfigurationView/ConfigSecureAP.cs:106`
    #[must_use]
    pub fn public_dat(&self) -> String {
        format!("{PUBLIC_KEY_HEADER}{}", self.public_base64())
    }

    /// `PemWriter.WriteObject(keyPair)`: the private key as PKCS#8 `PRIVATE KEY`, base64 in lines
    /// of 64, each line ended with [`NEWLINE`].
    /// `// C#: GCSViews/ConfigurationView/ConfigSecureAP.cs:92-96`
    #[must_use]
    pub fn to_pem(&self) -> String {
        let mut der = Vec::with_capacity(83);
        der.extend_from_slice(&PKCS8_PREFIX);
        der.extend_from_slice(&self.seed);
        der.extend_from_slice(&PKCS8_PUBLIC);
        der.extend_from_slice(&self.public);
        let body = base64::engine::general_purpose::STANDARD.encode(der);
        let mut pem = format!("-----BEGIN PRIVATE KEY-----{NEWLINE}");
        for line in body.as_bytes().chunks(64) {
            pem.push_str(&String::from_utf8_lossy(line));
            pem.push_str(NEWLINE);
        }
        pem.push_str("-----END PRIVATE KEY-----");
        pem.push_str(NEWLINE);
        pem
    }

    /// `PemReader.ReadObject()` cast to `Ed25519PrivateKeyParameters`, and the pair made from it:
    /// the first PEM object, which must be a PKCS#8 `PRIVATE KEY` holding an Ed25519 seed.
    ///
    /// # Errors
    /// No PEM object, or not an Ed25519 private key - the C#'s cast throws.
    /// `// C#: GCSViews/ConfigurationView/ConfigSecureAP.cs:44-47`
    pub fn from_pem(text: &str) -> Result<Self, String> {
        let begin = "-----BEGIN PRIVATE KEY-----";
        let start = text
            .find(begin)
            .ok_or("Unable to cast object: no PRIVATE KEY in the file")?;
        let rest = text.get(start + begin.len()..).unwrap_or_default();
        let end = rest
            .find("-----END PRIVATE KEY-----")
            .ok_or("malformed PEM: no END line")?;
        let body: String = rest
            .get(..end)
            .unwrap_or_default()
            .chars()
            .filter(|c| !c.is_whitespace())
            .collect();
        let der = base64::engine::general_purpose::STANDARD
            .decode(body)
            .map_err(|err| err.to_string())?;
        Self::from_seed(&pkcs8_seed(&der)?)
    }

    /// The private key file the Private Key button reads: a `.dat` holding `PRIVATE_KEYV1:` and a
    /// base64 seed - `Replace` takes the header out wherever it is, and the rest must be base64 -
    /// or else PEM.
    ///
    /// # Errors
    /// What the C#'s `Convert.FromBase64String`, `GenerateKey` or `PemReader` throws.
    /// `// C#: GCSViews/ConfigurationView/ConfigSecureAP.cs:36-48`
    pub fn from_key_file(text: &str) -> Result<Self, String> {
        if text.contains("PRIVATE_KEYV1") {
            let compact: String = text
                .replace(PRIVATE_KEY_HEADER, "")
                .chars()
                .filter(|c| !c.is_whitespace())
                .collect();
            let seed = base64::engine::general_purpose::STANDARD
                .decode(compact)
                .map_err(|err| err.to_string())?;
            Self::from_seed(&seed)
        } else {
            Self::from_pem(text)
        }
    }

    /// Ed25519 over `message`, as BouncyCastle's `Ed25519Signer` makes it.
    #[must_use]
    pub fn sign(&self, message: &[u8]) -> [u8; 64] {
        let mut signature = [0u8; 64];
        if let Ok(pair) = ring::signature::Ed25519KeyPair::from_seed_unchecked(&self.seed) {
            signature.copy_from_slice(pair.sign(message).as_ref());
        }
        signature
    }
}

/// The seed in a PKCS#8 Ed25519 `PrivateKeyInfo`, version 1 or 2: `SEQUENCE { INTEGER,
/// SEQUENCE { OID 1.3.101.112 }, OCTET STRING { OCTET STRING seed }, ... }`.
fn pkcs8_seed(der: &[u8]) -> Result<Vec<u8>, String> {
    let bad = || "Unable to cast object to Ed25519PrivateKeyParameters".to_owned();
    // One TLV with a short or one-byte long length: the lengths here are all under 256.
    let tlv = |bytes: &[u8]| -> Option<(u8, usize, usize)> {
        let tag = *bytes.first()?;
        let first = usize::from(*bytes.get(1)?);
        if first < 0x80 {
            Some((tag, 2, first))
        } else if first == 0x81 {
            Some((tag, 3, usize::from(*bytes.get(2)?)))
        } else {
            None
        }
    };
    let (tag, head, len) = tlv(der).ok_or_else(bad)?;
    if tag != 0x30 {
        return Err(bad());
    }
    let mut body = der.get(head..head + len).ok_or_else(bad)?;
    let mut next = |expected: u8| -> Result<&[u8], String> {
        let (tag, head, len) = tlv(body).ok_or_else(bad)?;
        if tag != expected {
            return Err(bad());
        }
        let content = body.get(head..head + len).ok_or_else(bad)?;
        body = body.get(head + len..).unwrap_or_default();
        Ok(content)
    };
    next(0x02)?;
    let algorithm = next(0x30)?;
    if algorithm != [0x06, 0x03, 0x2b, 0x65, 0x70] {
        return Err(bad());
    }
    let key = next(0x04)?;
    let (tag, head, len) = tlv(key).ok_or_else(bad)?;
    if tag != 0x04 || len != 32 {
        return Err(bad());
    }
    key.get(head..head + len)
        .map(<[u8]>::to_vec)
        .ok_or_else(bad)
}

/// Where `pattern` first starts in `src`, as `Extensions.Search` finds it.
/// `// C#: ExtLibs/Utilities/Extensions.cs:964-977`
#[must_use]
pub fn search(src: &[u8], pattern: &[u8]) -> Option<usize> {
    if pattern.is_empty() {
        return None;
    }
    src.windows(pattern.len())
        .position(|window| window == pattern)
}

/// `SignedFW.CreateSignedBL`: the bootloader with ArduPilot's three public keys and the pair's
/// written into its key table, just after the table's descriptor. The file keeps its length; a
/// table too near its end to take the keys throws, as a `MemoryStream` over the bytes does.
///
/// # Errors
/// "Invalid bin, descriptor not found", or the keys running past the end.
/// `// C#: ExtLibs/Utilities/SignedFW.cs:47-84`
pub fn create_signed_bl(key: &KeyPair, bl: &[u8]) -> Result<Vec<u8>, String> {
    let offset = search(bl, &BL_DESCRIPTOR).ok_or("Invalid bin, descriptor not found")? + 8;
    let mut keys = Vec::with_capacity(128);
    for text in ARDUPILOT_KEYS {
        let encoded = text.get(PUBLIC_KEY_HEADER.len()..).unwrap_or_default();
        keys.extend(
            base64::engine::general_purpose::STANDARD
                .decode(encoded)
                .map_err(|err| err.to_string())?,
        );
    }
    keys.extend_from_slice(key.public());
    let mut out = bl.to_vec();
    out.get_mut(offset..offset + keys.len())
        .ok_or("Memory stream is not expandable.")?
        .copy_from_slice(&keys);
    Ok(out)
}

/// `SignedFW.CreateSignedAPJ`: the `.apj`'s image inflated to `image_size`, signed - Ed25519
/// over the image without its 92-byte app descriptor - the signature written into the
/// descriptor with its length and version, the image deflated back, and `image_size`,
/// `flash_free` (when there is a `flash_total`), `signed_firmware` and `sha` (the SHA-512 of what
/// was signed, in base64) set; the whole written out indented, in ASCII.
///
/// # Errors
/// Not JSON, no `image` or `image_size`, not base64 or not zlib, "Invalid APJ, descriptor not
/// found", or a descriptor too near the image's end.
/// `// C#: ExtLibs/Utilities/SignedFW.cs:85-141`
pub fn create_signed_apj(key: &KeyPair, apj: &str) -> Result<Vec<u8>, String> {
    let mut d: serde_json::Value = serde_json::from_str(apj).map_err(|err| err.to_string())?;
    let size = d
        .get("image_size")
        .and_then(serde_json::Value::as_u64)
        .and_then(|size| usize::try_from(size).ok())
        .ok_or("no image_size")?;
    let image = d
        .get("image")
        .and_then(serde_json::Value::as_str)
        .ok_or("no image")?;
    let compact: String = image.chars().filter(|c| !c.is_whitespace()).collect();
    let packed = base64::engine::general_purpose::STANDARD
        .decode(compact)
        .map_err(|err| err.to_string())?;
    // `new byte[image_size]` filled by the inflater, as far as it goes.
    let mut img = vec![0u8; size];
    let mut inflater = flate2::read::ZlibDecoder::new(packed.as_slice());
    let mut filled = 0;
    while filled < size {
        let read = inflater
            .read(img.get_mut(filled..).unwrap_or_default())
            .map_err(|err| err.to_string())?;
        if read == 0 {
            break;
        }
        filled += read;
    }

    let offset = search(&img, &APJ_DESCRIPTOR).ok_or("Invalid APJ, descriptor not found")? + 8;
    let (head, tail) = (
        img.get(..offset).ok_or("descriptor past the image")?,
        img.get(offset + DESC_LEN..)
            .ok_or("Offset and length were out of bounds for the array")?,
    );
    let mut signed = Vec::with_capacity(head.len() + tail.len());
    signed.extend_from_slice(head);
    signed.extend_from_slice(tail);
    let sig = key.sign(&signed);
    let sha = ring::digest::digest(&ring::digest::SHA512, &signed);

    let mut descriptor = Vec::with_capacity(76);
    descriptor.extend_from_slice(&(SIG_LEN + 8).to_le_bytes());
    descriptor.extend_from_slice(&SIG_VERSION.to_le_bytes());
    descriptor.extend_from_slice(&sig);
    img.get_mut(offset + 16..offset + 16 + descriptor.len())
        .ok_or("descriptor past the image")?
        .copy_from_slice(&descriptor);

    let mut deflater = flate2::write::ZlibEncoder::new(Vec::new(), flate2::Compression::default());
    deflater.write_all(&img).map_err(|err| err.to_string())?;
    let packed = deflater.finish().map_err(|err| err.to_string())?;

    let object = d.as_object_mut().ok_or("not a JSON object")?;
    object.insert(
        "image".into(),
        base64::engine::general_purpose::STANDARD
            .encode(packed)
            .into(),
    );
    object.insert("image_size".into(), img.len().into());
    // `d["flash_total"] - d["image_size"]`, inside a `try`: nothing when there is no total.
    let free = match object.get("flash_total") {
        Some(serde_json::Value::Number(total)) => total.as_i64().map_or_else(
            || {
                total.as_f64().and_then(|total| {
                    #[allow(clippy::cast_precision_loss)]
                    let size = img.len() as f64;
                    serde_json::Number::from_f64(total - size).map(serde_json::Value::Number)
                })
            },
            |total| {
                i64::try_from(img.len())
                    .ok()
                    .map(|size| serde_json::Value::from(total - size))
            },
        ),
        _ => None,
    };
    if let Some(free) = free {
        object.insert("flash_free".into(), free);
    }
    object.insert("signed_firmware".into(), true.into());
    object.insert(
        "sha".into(),
        base64::engine::general_purpose::STANDARD
            .encode(sha.as_ref())
            .into(),
    );

    // `JsonConvert.SerializeObject(d, Formatting.Indented)` - two spaces, `"key": value` - with
    // `Environment.NewLine`, then `ASCIIEncoding.GetBytes`, which makes anything else a '?'.
    let text = serde_json::to_string_pretty(&d).map_err(|err| err.to_string())?;
    let text = text.replace('\n', NEWLINE);
    Ok(text
        .chars()
        .map(|c| if c.is_ascii() { c } else { '?' })
        .collect::<String>()
        .into_bytes())
}

/// `<dir>/<name>-signed<.ext>`: where the page saves a signed file, beside the one it read.
/// `// C#: GCSViews/ConfigurationView/ConfigSecureAP.cs:64-65, 83-84`
#[must_use]
pub fn signed_path(path: &std::path::Path, extension: &str) -> std::path::PathBuf {
    let stem = path
        .file_stem()
        .map(|stem| stem.to_string_lossy().into_owned())
        .unwrap_or_default();
    path.with_file_name(format!("{stem}-signed{extension}"))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The seed the reference run used: `i * 7 + 1`.
    fn seed() -> [u8; 32] {
        std::array::from_fn(|i| u8::try_from(i * 7 + 1).unwrap_or(0))
    }

    /// What BouncyCastle 2.4.0 - the version Mission Planner builds with - made of that seed,
    /// run under mono over `SignedFW.GenerateKey(knownseed)`, `PemWriter` and `Ed25519Signer`.
    #[test]
    fn the_key_its_pem_and_its_signature_are_bouncycastles() {
        let pair = KeyPair::from_seed(&seed()).expect("a pair");
        assert_eq!(
            pair.private_dat(),
            "PRIVATE_KEYV1:AQgPFh0kKzI5QEdOVVxjanF4f4aNlJuiqbC3vsXM09o="
        );
        assert_eq!(
            pair.public_base64(),
            "5AMJmM/VrRcjwWn5VqoLnrhhm1mSvWEsKvQo68efjfA="
        );
        assert_eq!(
            pair.to_pem().replace("\r\n", "\n"),
            "-----BEGIN PRIVATE KEY-----\n\
             MFECAQEwBQYDK2VwBCIEIAEIDxYdJCsyOUBHTlVcY2pxeH+GjZSboqmwt77FzNPa\n\
             gSEA5AMJmM/VrRcjwWn5VqoLnrhhm1mSvWEsKvQo68efjfA=\n\
             -----END PRIVATE KEY-----\n"
        );
        let message: Vec<u8> = (0..300_u32)
            .map(|i| u8::try_from((i * 13) % 256).unwrap_or(0))
            .collect();
        assert_eq!(
            base64::engine::general_purpose::STANDARD.encode(pair.sign(&message)),
            "yAdAWpoTPOaPsqWR5CI5yyLYGsFODI84L2yLxsHjFUiz8TWosGReulPxQwQc8/Iq2XtA46SJRUR43j8003E8Dg=="
        );
        assert_eq!(
            base64::engine::general_purpose::STANDARD
                .encode(ring::digest::digest(&ring::digest::SHA512, &message)),
            "ifH3MNcuWwlaa44bfrM86soLL5//BCH4vyoOK7YK7yG9iM6PzKxgplUc+F9VLZ0aVO7/GMzYbdoHTjfgJJGIiw=="
        );
    }

    #[test]
    fn both_key_files_read_back_to_the_pair() {
        let pair = KeyPair::from_seed(&seed()).expect("a pair");
        assert_eq!(KeyPair::from_key_file(&pair.to_pem()), Ok(pair.clone()));
        assert_eq!(
            KeyPair::from_key_file(&pair.private_dat()),
            Ok(pair.clone())
        );
        // A trailing line break is white space to `FromBase64String`.
        assert_eq!(
            KeyPair::from_key_file(&format!("{}\n", pair.private_dat())),
            Ok(pair.clone())
        );
        assert!(
            KeyPair::from_key_file(&pair.public_dat()).is_err(),
            "a public key"
        );
        assert!(
            KeyPair::from_key_file("PRIVATE_KEYV1:AQID").is_err(),
            "too short a seed"
        );
        assert!(KeyPair::from_key_file("nothing").is_err());
    }

    #[test]
    fn a_generated_pair_is_new_and_whole() {
        let one = KeyPair::generate().expect("a pair");
        let two = KeyPair::generate().expect("a pair");
        assert_ne!(one, two);
        assert_eq!(KeyPair::from_key_file(&one.private_dat()), Ok(one.clone()));
        assert!(
            !format!("{one:?}").contains(&one.private_dat()),
            "no private key in Debug"
        );
    }

    #[test]
    fn a_bootloader_gets_the_four_keys_after_its_descriptor() {
        let pair = KeyPair::from_seed(&seed()).expect("a pair");
        let mut bl = vec![0xffu8; 400];
        bl.splice(100..108, BL_DESCRIPTOR);
        let signed = create_signed_bl(&pair, &bl).expect("signed");
        assert_eq!(signed.len(), bl.len());
        assert_eq!(signed.get(..108), bl.get(..108));
        let first = base64::engine::general_purpose::STANDARD
            .decode("WJbbpbjOz/yMB3JxnvqyTUInCQdZcStkA0qhn2ldhPI=")
            .expect("base64");
        assert_eq!(signed.get(108..140), Some(first.as_slice()));
        assert_eq!(signed.get(204..236), Some(pair.public().as_slice()));
        assert_eq!(signed.get(236..), bl.get(236..));
        assert_eq!(
            create_signed_bl(&pair, &[0u8; 50]),
            Err("Invalid bin, descriptor not found".into())
        );
        let mut short = vec![0u8; 120];
        short.splice(100..108, BL_DESCRIPTOR);
        assert!(create_signed_bl(&pair, &short).is_err());
    }

    #[test]
    fn signed_files_sit_beside_the_originals() {
        assert_eq!(
            signed_path(std::path::Path::new("/fw/arducopter.apj"), ".apj"),
            std::path::PathBuf::from("/fw/arducopter-signed.apj")
        );
        assert_eq!(
            signed_path(std::path::Path::new("/fw/bl.bin"), ".bin"),
            std::path::PathBuf::from("/fw/bl-signed.bin")
        );
    }

    fn fixture(name: &str) -> Vec<u8> {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/secure")
            .join(name);
        mp_os::fs::read(&path).unwrap_or_else(|err| panic!("{}: {err}", path.display()))
    }

    /// `testdata/secure/bl-signed-by-mp.bin` and `signed-by-mp.apj` are what Mission Planner's own
    /// `SignedFW.CreateSignedBL` and `CreateSignedAPJ` - run verbatim under mono with
    /// BouncyCastle 2.4.0, Newtonsoft.Json 13.0.3 and DotNetZip, the packages it builds with -
    /// made of `bl.bin` and `unsigned.apj` with the seed above. The bootloader is byte for byte
    /// theirs; the `.apj` is too but for the image's deflated bytes, which are another zlib's: the
    /// image they inflate to is the same.
    #[test]
    fn the_signed_files_are_mission_planners() {
        let pair = KeyPair::from_seed(&seed()).expect("a pair");
        assert_eq!(
            create_signed_bl(&pair, &fixture("bl.bin")),
            Ok(fixture("bl-signed-by-mp.bin"))
        );

        let apj = String::from_utf8(fixture("unsigned.apj")).expect("text");
        let ours = create_signed_apj(&pair, &apj).expect("signed");
        let ours = String::from_utf8(ours).expect("ASCII");
        let theirs = String::from_utf8(fixture("signed-by-mp.apj")).expect("ASCII");
        let image_line = |text: &str| {
            text.lines()
                .map(|line| {
                    if line.trim_start().starts_with("\"image\":") {
                        "  \"image\": ...".to_owned()
                    } else {
                        line.to_owned()
                    }
                })
                .collect::<Vec<_>>()
                .join("\n")
        };
        assert_eq!(
            image_line(&ours.replace("\r\n", "\n")),
            image_line(&theirs),
            "every line but the image's"
        );
        let inflate = |text: &str| {
            let d: serde_json::Value = serde_json::from_str(text).expect("JSON");
            let packed = base64::engine::general_purpose::STANDARD
                .decode(d["image"].as_str().expect("an image"))
                .expect("base64");
            let mut image = Vec::new();
            flate2::read::ZlibDecoder::new(packed.as_slice())
                .read_to_end(&mut image)
                .expect("zlib");
            image
        };
        assert_eq!(inflate(&ours), inflate(&theirs));
    }

    #[test]
    fn an_apj_without_its_descriptor_is_refused() {
        let pair = KeyPair::from_seed(&seed()).expect("a pair");
        let mut deflater =
            flate2::write::ZlibEncoder::new(Vec::new(), flate2::Compression::default());
        deflater.write_all(&[0u8; 64]).expect("deflated");
        let image =
            base64::engine::general_purpose::STANDARD.encode(deflater.finish().expect("deflated"));
        let apj = format!("{{\"image\": \"{image}\", \"image_size\": 64}}");
        assert_eq!(
            create_signed_apj(&pair, &apj),
            Err("Invalid APJ, descriptor not found".into())
        );
        assert!(create_signed_apj(&pair, "{}").is_err());
        assert!(create_signed_apj(&pair, "not json").is_err());
    }
}
