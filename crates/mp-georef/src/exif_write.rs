//! `WriteCoordinatesToImage`: a copy of the photo with its GPS position in its EXIF, written the
//! way ExifLibNet 2.1.3 (`ExifLibrary.dll`, the NuGet package `MissionPlanner.Utilities.csproj:41`
//! references) writes it - which is not "the original with five tags added". ExifLibrary reads
//! every tag into a typed property and writes the whole APP1 segment again from those, so what
//! comes out is ExifLibrary's layout:
//!
//! - the TIFF header in the file's own byte order, IFD0 at 8, then each directory - IFD0, Exif,
//!   GPS, Interop, IFD1 - followed by its values, no padding between them, and every directory
//!   written even when empty (six zero bytes), IFD0's next-IFD link always pointing at IFD1;
//! - entries in the order the tags were read, a tag added later at the end of its directory;
//! - rationals reduced by their greatest common divisor, dates rewritten as
//!   `yyyy:MM:dd HH:mm:ss`, a date that is not a date as `0001:01:01 00:00:00`, ASCII re-encoded
//!   from UTF-8 with the count its UTF-16 length plus one, one- to three-byte `BYTE` and
//!   `UNDEFINED` values widened to four, enumerations with their fixed type, and
//!   `GPSVersionID` as four copies of its first digit;
//! - `SSHORT` values of a big-endian file written little-endian, `FLOAT` and `DOUBLE` tags
//!   dropped, the thumbnail moved to the end of IFD1, a JFIF APP0 rebuilt from its fields.
//!
//! Then `WriteCoordinatesToImage` removes the old GPS latitude, longitude, altitude and their
//! references, the maker note, both thumbnail resolutions and `FlashpixVersion`, and adds the new
//! position: `GPSLongitude` and `GPSLatitude` as degrees, minutes and seconds (`toDMS`, each
//! through `UFraction32(float)`'s continued fraction), `GPSAltitude` as `UFraction32(double)` -
//! which throws for a negative altitude, so a photo below the datum is not geotagged - and the
//! two references as ASCII. `GPSAltitudeRef` is not written.
//!
//! Anything that throws on the way - an unreadable file, a date whose fields are not numbers,
//! a malformed GPS value `ToString` indexes past, a segment grown past 64 KiB, a negative
//! altitude - is "There was a problem with image".
//!
//! Only JPEG is ported: ExifLibrary's `TIFFFile`, which writes a `.tif` photo's copy, is not, and
//! a TIFF is reported as a problem.
//! `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:1167-1231; ExifLibrary.dll (IL: JPEGFile, ImageFile,
//! ExifPropertyFactory, ExifPropertyCollection, ExifProperty and its subclasses, ExifBitConverter,
//! MathEx.UFraction32/Fraction32, BitConverterEx)`

use std::collections::HashMap;

use crate::georef::Output;
use crate::photos::{self, SEPARATOR};
use crate::time::{DateTime, Kind};

const ZEROTH: u32 = 100_000;
const EXIF: u32 = 200_000;
const GPS: u32 = 300_000;
const INTEROP: u32 = 400_000;
const FIRST: u32 = 500_000;
const JFIF: u32 = 700_000;
const JFXX: u32 = 800_000;

const THUMB_OFFSET_TAG: u32 = FIRST + 0x201;
const THUMB_LENGTH_TAG: u32 = FIRST + 0x202;
const EXIF_POINTER: u32 = ZEROTH + 0x8769;
const GPS_POINTER: u32 = ZEROTH + 0x8825;
const INTEROP_POINTER: u32 = EXIF + 0xA005;
const MAKER_NOTE: u32 = EXIF + 0x927C;

/// Why ExifLibrary threw.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error("{0}")]
pub struct ExifError(pub String);

fn err<T>(what: &str) -> Result<T, ExifError> {
    Err(ExifError(what.to_owned()))
}

/// `BitConverterEx.ByteOrder`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Order {
    Little,
    Big,
}

/// `BitConverterEx.CheckData` then the conversion: `count` bytes at `at`, in `order`.
fn bytes_at(data: &[u8], at: i64, count: usize, order: Order) -> Result<Vec<u8>, ExifError> {
    let Ok(at) = usize::try_from(at) else {
        return err("Index out of range");
    };
    let Some(slice) = data.get(at..at.saturating_add(count)) else {
        return err("Destination array was not long enough");
    };
    let mut b = slice.to_vec();
    if order == Order::Big {
        b.reverse();
    }
    Ok(b)
}

fn u16_at(data: &[u8], at: i64, order: Order) -> Result<u16, ExifError> {
    let b: [u8; 2] = bytes_at(data, at, 2, order)?
        .try_into()
        .map_err(|_| ExifError("Destination array was not long enough".to_owned()))?;
    Ok(u16::from_le_bytes(b))
}

fn u32_at(data: &[u8], at: i64, order: Order) -> Result<u32, ExifError> {
    let b: [u8; 4] = bytes_at(data, at, 4, order)?
        .try_into()
        .map_err(|_| ExifError("Destination array was not long enough".to_owned()))?;
    Ok(u32::from_le_bytes(b))
}

/// `MathEx.GCD(uint, uint)`.
const fn gcd(mut a: u32, mut b: u32) -> u32 {
    while b != 0 {
        let t = a % b;
        a = b;
        b = t;
    }
    a
}

/// `MathEx.UFraction32`, reduced whenever its denominator is not zero.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UFraction32 {
    /// Numerator.
    pub num: u32,
    /// Denominator.
    pub den: u32,
}

impl UFraction32 {
    /// `new UFraction32(num, den)` and `Set(num, den)`.
    #[must_use]
    pub const fn new(num: u32, den: u32) -> Self {
        if den == 0 {
            return Self { num, den };
        }
        let g = gcd(num, den);
        Self {
            num: num / g,
            den: den / g,
        }
    }

    /// `FromDouble`: the continued fraction of `value`, stopped at the first convergent equal to
    /// it, with the C#'s `uint` arithmetic wrapping and its error test that never fires after the
    /// first step (its first error is NaN). Negative values throw.
    /// `// C#: ExifLibrary MathEx.UFraction32.FromDouble (IL)`
    ///
    /// # Errors
    ///
    /// `ArgumentException` for a negative value.
    pub fn from_double(value: f64) -> Result<Self, ExifError> {
        if value < 0.0 {
            return err("value cannot be negative.");
        }
        if value.is_nan() {
            return Ok(Self::new(0, 0));
        }
        if value.is_infinite() {
            return Ok(Self::new(1, 0));
        }
        const EPSILON: f64 = 4.940_656_458_412_47e-324;
        let mut v0 = value;
        let v1 = value;
        let (mut h0, mut k0, mut h1, mut k1) = (0u32, 1u32, 1u32, 0u32);
        let mut err_prev = 1.0f64;
        let mut a = 0u32;
        let mut i: i64 = 0;
        loop {
            i += 1;
            if i > 10_000_000 {
                break;
            }
            a = conv_u4(v0.floor());
            v0 -= f64::from(a);
            if v0.abs() < EPSILON {
                break;
            }
            v0 = 1.0 / v0;
            if v0.is_infinite() {
                break;
            }
            let h2 = h1.wrapping_mul(a).wrapping_add(h0);
            let k2 = k1.wrapping_mul(a).wrapping_add(k0);
            if (f64::from(h2) / f64::from(k2) - v1).abs() < EPSILON {
                break;
            }
            let ratio = f64::from(h1) / f64::from(k1);
            let err_now = (f64::from(h2) / f64::from(k2) - ratio) / ratio;
            // `bge.s`: ordered, so a NaN on either side never stops the loop.
            if err_now >= err_prev {
                break;
            }
            err_prev = err_now;
            h0 = h1;
            k0 = k1;
            h1 = h2;
            k1 = k2;
        }
        let num = h1.wrapping_mul(a).wrapping_add(h0);
        let den = k1.wrapping_mul(a).wrapping_add(k0);
        Ok(Self::new(num, den))
    }
}

/// `conv.u4` of a double on x86-64: a 64-bit truncating conversion, its low 32 bits; out of the
/// 64-bit range the CPU's "integer indefinite", whose low 32 bits are 0.
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn conv_u4(value: f64) -> u32 {
    const TWO_TO_63: f64 = 9_223_372_036_854_775_808.0;
    if value.is_nan() || !(-TWO_TO_63..TWO_TO_63).contains(&value) {
        0
    } else {
        (value as i64) as u32
    }
}

/// `MathEx.Fraction32`: a sign, and a reduced magnitude.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Fraction32 {
    negative: bool,
    num: i32,
    den: i32,
}

impl Fraction32 {
    /// `new Fraction32(num, den)` and `Set(num, den)`.
    #[must_use]
    pub const fn new(num: i32, den: i32) -> Self {
        let mut negative = false;
        let mut n = num;
        let mut d = den;
        if n < 0 {
            negative = !negative;
            n = n.wrapping_neg();
        }
        if d < 0 {
            negative = !negative;
            d = d.wrapping_neg();
        }
        if d != 0 {
            #[allow(clippy::cast_sign_loss, clippy::cast_possible_wrap)]
            let g = gcd(n as u32, d as u32);
            #[allow(clippy::cast_possible_wrap)]
            let g = if g == 0 { 1 } else { g as i32 };
            n = n.wrapping_div(g);
            d = d.wrapping_div(g);
        }
        Self {
            negative,
            num: n,
            den: d,
        }
    }

    /// `Numerator`: the magnitude with the sign.
    const fn numerator(self) -> i32 {
        if self.negative {
            self.num.wrapping_neg()
        } else {
            self.num
        }
    }
}

/// The text encodings a property can carry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TextEncoding {
    Utf8,
    Ascii,
    Unicode,
}

impl TextEncoding {
    fn decode(self, bytes: &[u8]) -> String {
        match self {
            Self::Utf8 => String::from_utf8_lossy(bytes).into_owned(),
            Self::Ascii => bytes
                .iter()
                .map(|&b| if b < 0x80 { char::from(b) } else { '?' })
                .collect(),
            Self::Unicode => {
                let units: Vec<u16> = bytes
                    .chunks(2)
                    .map(|c| {
                        u16::from_le_bytes([
                            c.first().copied().unwrap_or(0),
                            c.get(1).copied().unwrap_or(0),
                        ])
                    })
                    .collect();
                // An odd last byte decodes to U+FFFD.
                let mut s = String::from_utf16_lossy(&units);
                if bytes.len() % 2 == 1 {
                    s.pop();
                    s.push('\u{FFFD}');
                }
                s
            }
        }
    }

    fn encode(self, text: &str) -> Vec<u8> {
        match self {
            Self::Utf8 => text.as_bytes().to_vec(),
            Self::Ascii => text
                .chars()
                .map(|c| u8::try_from(c).ok().filter(u8::is_ascii).unwrap_or(b'?'))
                .collect(),
            Self::Unicode => text.encode_utf16().flat_map(u16::to_le_bytes).collect(),
        }
    }
}

/// How an `ExifEnumProperty<T>` writes itself, by `T`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum EnumKind {
    /// `FileSource`, `SceneType`: one `UNDEFINED` byte.
    Undefined,
    /// The GPS references: two `ASCII` bytes, the value and a NUL.
    Ascii,
    /// An enumeration over `byte`.
    Byte,
    /// An enumeration over `ushort`.
    UShort,
}

/// Which `ToString` could index past its array (and so throw in `WriteCoordinatesToImage`'s
/// listing of every property).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Shape {
    Plain,
    /// `ExifPointSubjectArea`: two values read.
    Point,
    /// `GPSLatitudeLongitude`, `GPSTimeStamp`: three.
    Three,
    /// `LensSpecification`: four.
    Four,
}

/// A property's value, in the typed form ExifLibrary holds it in.
#[derive(Debug, Clone, PartialEq)]
enum Value {
    Byte(u8),
    ByteArray(Vec<u8>),
    SByte(u8),
    SByteArray(Vec<u8>),
    Ascii(String, TextEncoding),
    UShort(u16),
    UShortArray(Vec<u16>, Shape),
    UInt(u32),
    UIntArray(Vec<u32>),
    URational(UFraction32),
    URationalArray(Vec<UFraction32>, Shape),
    Undefined(Vec<u8>),
    SInt(i32),
    SIntArray(Vec<i32>),
    SRational(Fraction32),
    SRationalArray(Vec<Fraction32>),
    SShort(i16),
    SShortArray(Vec<i16>),
    Enum(u16, EnumKind),
    EncodedString(String, Option<TextEncoding>),
    DateTime(DateTime),
    Date(DateTime),
    Version(String),
    WindowsByteString(String),
    /// `JFIFThumbnailProperty`: the palette (empty unless a palette thumbnail) and the pixels.
    Thumbnail(Vec<u8>, Vec<u8>),
}

/// `ExifProperty`: a tag (`IFD + id`) and its value.
#[derive(Debug, Clone, PartialEq)]
struct Property {
    tag: u32,
    value: Value,
}

/// `ExifInterOperability`: what a property writes - its id, type, count and bytes, the bytes in
/// the machine's (little-endian) order.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Interop {
    id: u16,
    typ: u16,
    count: u32,
    data: Vec<u8>,
}

#[allow(clippy::cast_possible_truncation)]
const fn tag_id(tag: u32) -> u16 {
    (tag - (tag / 100_000) * 100_000) as u16
}

const fn tag_ifd(tag: u32) -> u32 {
    (tag / 100_000) * 100_000
}

fn count(n: usize) -> u32 {
    u32::try_from(n).unwrap_or(u32::MAX)
}

impl Property {
    /// `Interoperability`. `// C#: ExifLibrary ExifProperty subclasses' get_Interoperability (IL)`
    fn interop(&self) -> Result<Interop, ExifError> {
        let id = tag_id(self.tag);
        let make = |typ: u16, n: usize, data: Vec<u8>| Interop {
            id,
            typ,
            count: count(n),
            data,
        };
        Ok(match &self.value {
            Value::Byte(v) => make(1, 1, vec![*v]),
            Value::ByteArray(v) => make(1, v.len(), v.clone()),
            Value::SByte(v) => make(6, 1, vec![*v]),
            Value::SByteArray(v) => make(6, v.len(), v.clone()),
            Value::Ascii(text, encoding) => {
                let mut data = encoding.encode(text);
                data.extend(encoding.encode("\0"));
                make(2, text.encode_utf16().count() + 1, data)
            }
            Value::UShort(v) => make(3, 1, v.to_le_bytes().to_vec()),
            Value::UShortArray(v, _) => {
                make(3, v.len(), v.iter().flat_map(|x| x.to_le_bytes()).collect())
            }
            Value::UInt(v) => make(4, 1, v.to_le_bytes().to_vec()),
            Value::UIntArray(v) => {
                make(4, v.len(), v.iter().flat_map(|x| x.to_le_bytes()).collect())
            }
            Value::URational(f) => make(5, 1, rational_bytes(*f)),
            Value::URationalArray(v, _) => make(
                5,
                v.len(),
                v.iter().flat_map(|f| rational_bytes(*f)).collect(),
            ),
            Value::Undefined(v) => make(7, v.len(), v.clone()),
            Value::SInt(v) => make(9, 1, v.to_le_bytes().to_vec()),
            Value::SIntArray(v) => {
                make(9, v.len(), v.iter().flat_map(|x| x.to_le_bytes()).collect())
            }
            Value::SRational(f) => make(10, 1, srational_bytes(*f)),
            Value::SRationalArray(v) => make(
                10,
                v.len(),
                v.iter().flat_map(|f| srational_bytes(*f)).collect(),
            ),
            Value::SShort(v) => make(8, 1, v.to_le_bytes().to_vec()),
            Value::SShortArray(v) => {
                make(8, v.len(), v.iter().flat_map(|x| x.to_le_bytes()).collect())
            }
            #[allow(clippy::cast_possible_truncation)]
            Value::Enum(v, kind) => match kind {
                EnumKind::Undefined => make(7, 1, vec![*v as u8]),
                EnumKind::Ascii => make(2, 2, vec![*v as u8, 0]),
                EnumKind::Byte => make(1, 1, vec![*v as u8]),
                EnumKind::UShort => make(3, 1, v.to_le_bytes().to_vec()),
            },
            Value::EncodedString(text, encoding) => {
                let header: &[u8; 8] = match encoding {
                    Some(TextEncoding::Ascii) => b"ASCII\0\0\0",
                    Some(TextEncoding::Unicode) => b"Unicode\0",
                    _ => &[0; 8],
                };
                let mut data = header.to_vec();
                data.extend(encoding.unwrap_or(TextEncoding::Ascii).encode(text));
                make(7, data.len(), data)
            }
            Value::DateTime(t) => {
                let mut data = t.format_exif().into_bytes();
                data.push(0);
                make(2, 20, data)
            }
            Value::Date(t) => {
                let text = t.format_exif();
                let mut data = text.get(..10).unwrap_or_default().as_bytes().to_vec();
                data.push(0);
                make(2, 11, data)
            }
            Value::Version(text) => {
                if matches!(self.tag, 0x39D40 | 0x3AD40 | 0x61A82) {
                    make(7, 4, TextEncoding::Ascii.encode(text))
                } else {
                    // Byte.Parse of the first character, four times.
                    let Some(first) = text.chars().next() else {
                        return err("Index was outside the bounds of the array.");
                    };
                    let Some(digit) = first.to_digit(10) else {
                        return err("Input string was not in a correct format.");
                    };
                    #[allow(clippy::cast_possible_truncation)]
                    let digit = digit as u8;
                    make(7, 4, vec![digit; 4])
                }
            }
            Value::WindowsByteString(text) => {
                let data = TextEncoding::Unicode.encode(text);
                make(1, data.len(), data)
            }
            Value::Thumbnail(palette, pixels) => {
                let mut data = palette.clone();
                data.extend_from_slice(pixels);
                make(1, data.len(), data)
            }
        })
    }

    /// Whether `ToString()` throws: the fixed-size types index their array.
    fn to_string_throws(&self) -> bool {
        let (shape, len) = match &self.value {
            Value::UShortArray(v, shape) => (*shape, v.len()),
            Value::URationalArray(v, shape) => (*shape, v.len()),
            _ => return false,
        };
        match shape {
            Shape::Plain => false,
            Shape::Point => len < 2,
            Shape::Three => len < 3,
            Shape::Four => len < 4,
        }
    }
}

fn rational_bytes(f: UFraction32) -> Vec<u8> {
    let mut b = f.num.to_le_bytes().to_vec();
    b.extend(f.den.to_le_bytes());
    b
}

fn srational_bytes(f: Fraction32) -> Vec<u8> {
    let mut b = f.numerator().to_le_bytes().to_vec();
    b.extend(f.den.to_le_bytes());
    b
}

/// `ExifBitConverter.ToAscii(data, encoding)`: up to the first NUL.
fn to_ascii(data: &[u8], encoding: TextEncoding) -> String {
    let end = data.iter().position(|&b| b == 0).unwrap_or(data.len());
    encoding.decode(data.get(..end).unwrap_or_default())
}

/// `ExifBitConverter.ToDateTime(data, hastime)`: the text split on `:` and space, six (or
/// three) fields through `int.Parse` into a `DateTime`; `DateTime.MinValue` for the wrong number
/// of fields or an impossible date. A field that is not a number throws.
fn to_date_time(data: &[u8], hastime: bool) -> Result<DateTime, ExifError> {
    let text = to_ascii(data, TextEncoding::Ascii);
    let fields: Vec<&str> = text.split([':', ' ']).collect();
    let parse = |s: &str| -> Result<i32, ExifError> {
        mp_log::netfmt::parse_i32(s)
            .ok_or_else(|| ExifError("Input string was not in a correct format.".to_owned()))
    };
    if hastime && fields.len() == 6 {
        let v: Result<Vec<i32>, ExifError> = fields.iter().map(|s| parse(s)).collect();
        let v = v?;
        return Ok(match v.as_slice() {
            [y, mo, d, h, mi, s] => {
                DateTime::from_parts(*y, *mo, *d, *h, *mi, *s, Kind::Unspecified)
            }
            _ => None,
        }
        .unwrap_or(DateTime::MIN));
    }
    if !hastime && fields.len() == 3 {
        let v: Result<Vec<i32>, ExifError> = fields.iter().map(|s| parse(s)).collect();
        let v = v?;
        return Ok(match v.as_slice() {
            [y, mo, d] => DateTime::from_parts(*y, *mo, *d, 0, 0, 0, Kind::Unspecified),
            _ => None,
        }
        .unwrap_or(DateTime::MIN));
    }
    Ok(DateTime::MIN)
}

fn u16_array(value: &[u8], n: usize, order: Order) -> Result<Vec<u16>, ExifError> {
    (0..n)
        .map(|i| u16_at(value, (i * 2) as i64, order))
        .collect()
}

fn u32_array(value: &[u8], n: usize, order: Order) -> Result<Vec<u32>, ExifError> {
    (0..n)
        .map(|i| u32_at(value, (i * 4) as i64, order))
        .collect()
}

fn urational_array(value: &[u8], n: usize, order: Order) -> Result<Vec<UFraction32>, ExifError> {
    (0..n)
        .map(|i| {
            Ok(UFraction32::new(
                u32_at(value, (i * 8) as i64, order)?,
                u32_at(value, (i * 8 + 4) as i64, order)?,
            ))
        })
        .collect()
}

#[allow(clippy::cast_possible_wrap)]
fn srational_array(value: &[u8], n: usize, order: Order) -> Result<Vec<Fraction32>, ExifError> {
    (0..n)
        .map(|i| {
            Ok(Fraction32::new(
                u32_at(value, (i * 8) as i64, order)? as i32,
                u32_at(value, (i * 8 + 4) as i64, order)? as i32,
            ))
        })
        .collect()
}

/// `ExifPropertyFactory.Get`: a directory entry as the property ExifLibrary makes of it.
/// `// C#: ExifLibrary ExifPropertyFactory.Get (IL)`
#[allow(clippy::too_many_lines, clippy::cast_possible_wrap)]
fn factory(
    tag_id: u16,
    typ: u16,
    n: u32,
    value: &[u8],
    order: Order,
    ifd: u32,
) -> Result<Property, ExifError> {
    let tag = ifd + u32::from(tag_id);
    let n_usize = usize::try_from(n).unwrap_or(usize::MAX);
    let enum_u16 = |kind: EnumKind| -> Result<Value, ExifError> {
        Ok(Value::Enum(u16_at(value, 0, order)?, kind))
    };
    let byte0 = || -> Result<u8, ExifError> {
        value
            .first()
            .copied()
            .ok_or_else(|| ExifError("Index was outside the bounds of the array.".to_owned()))
    };
    let special: Option<Value> = match (ifd, tag_id) {
        (ZEROTH | FIRST, 0x103 | 0x106 | 0x112 | 0x11C | 0x213 | 0x128) => {
            Some(enum_u16(EnumKind::UShort)?)
        }
        (ZEROTH | FIRST, 0x132) => Some(Value::DateTime(to_date_time(value, true)?)),
        (ZEROTH, 0x9C9B..=0x9C9F) => {
            let text = TextEncoding::Unicode.decode(value);
            Some(Value::WindowsByteString(
                text.trim_end_matches('\0').to_owned(),
            ))
        }
        (EXIF, 0x9000 | 0xA000) => Some(Value::Version(to_ascii(value, TextEncoding::Ascii))),
        (
            EXIF,
            0xA001 | 0x8822 | 0x9207 | 0x9208 | 0x9209 | 0xA210 | 0xA217 | 0xA401 | 0xA402 | 0xA403
            | 0xA406 | 0xA407 | 0xA408 | 0xA409 | 0xA40A | 0xA40C,
        ) => Some(enum_u16(EnumKind::UShort)?),
        (EXIF, 0x9286) => {
            let (encoding, text) = if value.len() < 8 {
                (None, TextEncoding::Ascii.decode(value))
            } else {
                let head = TextEncoding::Ascii.decode(value.get(..8).unwrap_or_default());
                let rest = value.get(8..).unwrap_or_default();
                if head.eq_ignore_ascii_case("ASCII\0\0\0") {
                    (Some(TextEncoding::Ascii), TextEncoding::Ascii.decode(rest))
                } else if head.eq_ignore_ascii_case("JIS\0\0\0\0\0") {
                    // Encoding.GetEncoding("Japanese (JIS 0208-1990 and 0212-1990)") throws.
                    return err(
                        "'Japanese (JIS 0208-1990 and 0212-1990)' is not a supported encoding name.",
                    );
                } else if head.eq_ignore_ascii_case("Unicode\0") {
                    (
                        Some(TextEncoding::Unicode),
                        TextEncoding::Unicode.decode(rest),
                    )
                } else {
                    (None, TextEncoding::Ascii.decode(value))
                }
            };
            // An unrecognised header keeps the ASCII encoding it started with.
            Some(Value::EncodedString(
                text.trim_matches('\0').to_owned(),
                Some(encoding.unwrap_or(TextEncoding::Ascii)),
            ))
        }
        (EXIF, 0x9003 | 0x9004) => Some(Value::DateTime(to_date_time(value, true)?)),
        (EXIF, 0x9214) => {
            let v = u16_array(value, n_usize, order)?;
            Some(Value::UShortArray(
                v,
                if n == 3 || n == 4 {
                    Shape::Plain
                } else {
                    Shape::Point
                },
            ))
        }
        (EXIF, 0xA214) => Some(Value::UShortArray(
            u16_array(value, n_usize, order)?,
            Shape::Point,
        )),
        #[allow(clippy::cast_possible_truncation)]
        (EXIF, 0xA300 | 0xA301) => Some(Value::Enum(
            u16::from(u16_at(value, 0, order)? as u8),
            EnumKind::Undefined,
        )),
        (EXIF, 0xA432) => Some(Value::URationalArray(
            urational_array(value, n_usize, order)?,
            Shape::Four,
        )),
        (GPS, 0) => Some(Value::Version(value.iter().map(u8::to_string).collect())),
        (GPS, 1 | 3 | 9 | 10 | 12 | 14 | 16 | 19 | 21 | 23 | 25) => {
            Some(Value::Enum(u16::from(byte0()?), EnumKind::Ascii))
        }
        (GPS, 5) => Some(Value::Enum(u16::from(byte0()?), EnumKind::Byte)),
        (GPS, 2 | 4 | 7 | 20 | 22) => Some(Value::URationalArray(
            urational_array(value, n_usize, order)?,
            Shape::Three,
        )),
        (GPS, 29) => Some(Value::Date(to_date_time(value, false)?)),
        (GPS, 30) => Some(enum_u16(EnumKind::UShort)?),
        (INTEROP, 1) => Some(Value::Ascii(
            to_ascii(value, TextEncoding::Ascii),
            TextEncoding::Ascii,
        )),
        (INTEROP, 2) => Some(Value::Version(to_ascii(value, TextEncoding::Ascii))),
        _ => None,
    };
    let value = match special {
        Some(v) => v,
        None => match typ {
            1 if n == 1 => Value::Byte(byte0()?),
            1 => Value::ByteArray(value.to_vec()),
            2 => Value::Ascii(to_ascii(value, TextEncoding::Utf8), TextEncoding::Utf8),
            3 if n == 1 => Value::UShort(u16_at(value, 0, order)?),
            3 => Value::UShortArray(u16_array(value, n_usize, order)?, Shape::Plain),
            4 if n == 1 => Value::UInt(u32_at(value, 0, order)?),
            4 => Value::UIntArray(u32_array(value, n_usize, order)?),
            5 if n == 1 => Value::URational(UFraction32::new(
                u32_at(value, 0, order)?,
                u32_at(value, 4, order)?,
            )),
            5 => Value::URationalArray(urational_array(value, n_usize, order)?, Shape::Plain),
            6 if n == 1 => Value::SByte(byte0()?),
            6 => Value::SByteArray(value.get(..n_usize).unwrap_or(value).to_vec()),
            7 => Value::Undefined(value.to_vec()),
            #[allow(clippy::cast_possible_wrap)]
            8 if n == 1 => Value::SShort(u16_at(value, 0, order)? as i16),
            #[allow(clippy::cast_possible_wrap)]
            8 => Value::SShortArray(
                u16_array(value, n_usize, order)?
                    .into_iter()
                    .map(|x| x as i16)
                    .collect(),
            ),
            9 if n == 1 => Value::SInt(u32_at(value, 0, order)? as i32),
            9 => Value::SIntArray(
                u32_array(value, n_usize, order)?
                    .into_iter()
                    .map(|x| x as i32)
                    .collect(),
            ),
            10 if n == 1 => Value::SRational(Fraction32::new(
                u32_at(value, 0, order)? as i32,
                u32_at(value, 4, order)? as i32,
            )),
            10 => Value::SRationalArray(srational_array(value, n_usize, order)?),
            _ => return err("Unknown property type."),
        },
    };
    Ok(Property { tag, value })
}

/// A JPEG section: its marker, the bytes after the length, and for a scan the entropy-coded data
/// after it. `// C#: ExifLibrary JPEGSection (IL)`
#[derive(Debug, Clone, PartialEq, Eq)]
struct Section {
    marker: u8,
    header: Vec<u8>,
    entropy: Vec<u8>,
}

/// `JPEGFile`, read.
#[derive(Debug, Clone)]
struct JpegFile {
    sections: Vec<Section>,
    properties: Vec<Property>,
    order: Order,
    thumbnail: Option<Vec<u8>>,
    exif_app1: usize,
    jfif_app0: Option<usize>,
    jfxx_app0: Option<usize>,
    maker_note_offset: u32,
}

/// `Utility.GetStreamBytes(stream, length)`: below 32 KiB exactly `length` bytes, zero-filled
/// past the end; above it, what there is up to `length`.
fn stream_bytes(data: &[u8], pos: &mut usize, length: i64) -> Result<Vec<u8>, ExifError> {
    if length < 0 {
        return err("Arithmetic operation resulted in an overflow.");
    }
    let length = usize::try_from(length).unwrap_or(usize::MAX);
    let available = data.len().saturating_sub(*pos).min(length);
    let mut out = data
        .get(*pos..*pos + available)
        .unwrap_or_default()
        .to_vec();
    *pos += available;
    if length < 0x8000 {
        out.resize(length, 0);
    }
    Ok(out)
}

impl JpegFile {
    /// `new JPEGFile(stream, encoding, readTrailingData: false)`: the sections, then the JFIF,
    /// JFXX and EXIF segments read into properties.
    /// `// C#: ExifLibrary JPEGFile..ctor (IL)`
    fn read(data: &[u8]) -> Result<Self, ExifError> {
        if data.len() < 2 || data.first() != Some(&0xFF) || data.get(1) != Some(&0xD8) {
            return err("Not a valid JPEG file.");
        }
        let mut pos = 0usize;
        let mut sections = Vec::new();
        while pos != data.len() {
            let (Some(&a), Some(&b)) = (data.get(pos), data.get(pos + 1)) else {
                return err("Not a valid JPEG file.");
            };
            pos += 2;
            if a != 0xFF || b == 0 || b == 0xFF {
                return err("Not a valid JPEG file.");
            }
            let marker = b;
            let mut header = Vec::new();
            if !(marker == 0xD8 || marker == 0xD9 || (0xD0..=0xD7).contains(&marker)) {
                let (Some(&h), Some(&l)) = (data.get(pos), data.get(pos + 1)) else {
                    return err("Not a valid JPEG file.");
                };
                pos += 2;
                let length = i64::from(u16::from_be_bytes([h, l])) - 2;
                header = stream_bytes(data, &mut pos, length)?;
            }
            let mut entropy = Vec::new();
            if marker == 0xDA || (0xD0..=0xD7).contains(&marker) {
                let start = pos;
                let mut byte: i32;
                loop {
                    loop {
                        byte = data.get(pos).map_or(-1, |&b| i32::from(b));
                        if byte != -1 {
                            pos += 1;
                        }
                        if byte == -1 || byte == 0xFF {
                            break;
                        }
                    }
                    loop {
                        byte = data.get(pos).map_or(-1, |&b| i32::from(b));
                        if byte != -1 {
                            pos += 1;
                        }
                        if byte != 0xFF {
                            break;
                        }
                    }
                    if byte != 0 {
                        break;
                    }
                }
                if byte != -1 {
                    pos -= 2;
                }
                entropy = data.get(start..pos).unwrap_or_default().to_vec();
            }
            sections.push(Section {
                marker,
                header,
                entropy,
            });
            if marker == 0xD9 {
                // Trailing data is not read (readTrailingData is false), and is lost.
                break;
            }
        }
        let mut file = Self {
            sections,
            properties: Vec::new(),
            order: Order::Little,
            thumbnail: None,
            exif_app1: 0,
            jfif_app0: None,
            jfxx_app0: None,
            maker_note_offset: 0,
        };
        file.read_jfif()?;
        file.read_jfxx()?;
        file.read_exif()?;
        Ok(file)
    }

    fn find_app(&self, marker: u8, magic: &[u8]) -> Option<usize> {
        self.sections.iter().position(|s| {
            s.marker == marker
                && s.header.len() >= magic.len()
                && s.header.get(..magic.len()) == Some(magic)
        })
    }

    /// `ReadJFIFAPP0`. `// C#: ExifLibrary JPEGFile.ReadJFIFAPP0 (IL)`
    fn read_jfif(&mut self) -> Result<(), ExifError> {
        self.jfif_app0 = self.find_app(0xE0, b"JFIF\0");
        let Some(at) = self.jfif_app0 else {
            return Ok(());
        };
        let h = self.header(at);
        let version = u16_at(&h, 5, Order::Big)?;
        let get = |i: usize| {
            h.get(i)
                .copied()
                .ok_or_else(|| ExifError("Index was outside the bounds of the array.".to_owned()))
        };
        self.properties.push(Property {
            tag: JFIF + 1,
            value: Value::UShort(version),
        });
        self.properties.push(Property {
            tag: JFIF + 101,
            value: Value::Enum(u16::from(get(7)?), EnumKind::Byte),
        });
        self.properties.push(Property {
            tag: JFIF + 102,
            value: Value::UShort(u16_at(&h, 8, Order::Big)?),
        });
        self.properties.push(Property {
            tag: JFIF + 103,
            value: Value::UShort(u16_at(&h, 10, Order::Big)?),
        });
        let x = get(12)?;
        let y = get(13)?;
        self.properties.push(Property {
            tag: JFIF + 201,
            value: Value::Byte(x),
        });
        self.properties.push(Property {
            tag: JFIF + 202,
            value: Value::Byte(y),
        });
        let n = usize::from(x) * usize::from(y);
        let Some(pixels) = h.get(14..14 + n) else {
            return err("Source array was not long enough.");
        };
        self.properties.push(Property {
            tag: JFIF + 203,
            value: Value::Thumbnail(Vec::new(), pixels.to_vec()),
        });
        Ok(())
    }

    /// `ReadJFXXAPP0`. `// C#: ExifLibrary JPEGFile.ReadJFXXAPP0 (IL)`
    fn read_jfxx(&mut self) -> Result<(), ExifError> {
        self.jfxx_app0 = self.find_app(0xE0, b"JFXX\0");
        let Some(at) = self.jfxx_app0 else {
            return Ok(());
        };
        let h = self.header(at);
        let get = |i: usize| {
            h.get(i)
                .copied()
                .ok_or_else(|| ExifError("Index was outside the bounds of the array.".to_owned()))
        };
        let ext = get(5)?;
        self.properties.push(Property {
            tag: JFXX + 1,
            value: Value::Enum(u16::from(ext), EnumKind::Byte),
        });
        match ext {
            0x10 => {
                let pixels = h.get(6..).unwrap_or_default().to_vec();
                self.properties.push(Property {
                    tag: JFXX + 202,
                    value: Value::Thumbnail(Vec::new(), pixels),
                });
            }
            0x13 => {
                let (x, y) = (get(6)?, get(7)?);
                self.properties.push(Property {
                    tag: JFXX + 101,
                    value: Value::Byte(x),
                });
                self.properties.push(Property {
                    tag: JFXX + 102,
                    value: Value::Byte(y),
                });
                let n = 3 * usize::from(x) * usize::from(y);
                let Some(pixels) = h.get(8..8 + n) else {
                    return err("Source array was not long enough.");
                };
                self.properties.push(Property {
                    tag: JFXX + 202,
                    value: Value::Thumbnail(Vec::new(), pixels.to_vec()),
                });
            }
            0x11 => {
                let (x, y) = (get(6)?, get(7)?);
                self.properties.push(Property {
                    tag: JFXX + 101,
                    value: Value::Byte(x),
                });
                self.properties.push(Property {
                    tag: JFXX + 102,
                    value: Value::Byte(y),
                });
                let Some(palette) = h.get(8..8 + 0x300) else {
                    return err("Source array was not long enough.");
                };
                let n = usize::from(x) * usize::from(y);
                let Some(pixels) = h.get(0x308..0x308 + n) else {
                    return err("Source array was not long enough.");
                };
                self.properties.push(Property {
                    tag: JFXX + 202,
                    value: Value::Thumbnail(palette.to_vec(), pixels.to_vec()),
                });
            }
            _ => {}
        }
        Ok(())
    }

    /// `ReadExifAPP1`: every directory entry of the first `Exif` APP1 as a property, IFDs visited
    /// by offset (a `SortedList`, whose `Add` throws on an offset seen twice).
    /// `// C#: ExifLibrary JPEGFile.ReadExifAPP1 (IL)`
    #[allow(clippy::too_many_lines, clippy::cast_possible_wrap)]
    fn read_exif(&mut self) -> Result<(), ExifError> {
        let Some(at) = self.find_app(0xE1, b"Exif\0\0") else {
            // A new, empty APP1 after the last APP0, or after SOI.
            let last_app0 = self.sections.iter().rposition(|s| s.marker == 0xE0);
            let index = last_app0.map_or(1, |i| i + 1).min(self.sections.len());
            self.sections.insert(
                index,
                Section {
                    marker: 0xE1,
                    header: Vec::new(),
                    entropy: Vec::new(),
                },
            );
            self.shift_after_insert(index);
            self.exif_app1 = index;
            self.order = Order::Little;
            return Ok(());
        };
        self.exif_app1 = at;
        let header = self.header(at);
        let len = i64::try_from(header.len()).unwrap_or(i64::MAX);
        self.maker_note_offset = 0;
        let tiff: i64 = 6;
        self.order = match header.get(6..8) {
            Some(b"II") => Order::Little,
            Some(b"MM") => Order::Big,
            _ => return err("Not a valid Exif file."),
        };
        let header_order = if u16_at(&header, 8, Order::Little)? == 42 {
            Order::Little
        } else if u16_at(&header, 8, Order::Big)? == 42 {
            Order::Big
        } else {
            return err("Not a valid Exif file.");
        };
        let mut queue: Vec<(i64, u32)> = Vec::new();
        let add = |queue: &mut Vec<(i64, u32)>, offset: i64, ifd: u32| -> Result<(), ExifError> {
            if queue.iter().any(|(o, _)| *o == offset) {
                return err("An entry with the same key already exists.");
            }
            queue.push((offset, ifd));
            queue.sort_by_key(|(o, _)| *o);
            Ok(())
        };
        if len - (tiff + 4) >= 4 {
            let first = i64::from(u32_at(&header, tiff + 4, header_order)? as i32);
            add(&mut queue, first, ZEROTH)?;
        }
        let order = self.order;
        // `int` locals, so a pointer past 2^31 is negative.
        let (mut thumb_offset, mut thumb_length, mut thumb_type): (i32, i32, i32) = (-1, 0, -1);
        while !queue.is_empty() {
            let (key, ifd) = queue.remove(0);
            let ifd_offset = tiff + key;
            let field_count = u16_at(&header, ifd_offset, order)?;
            if ifd_offset > len - 1 || ifd_offset + 2 > len {
                continue;
            }
            #[allow(clippy::cast_possible_wrap)]
            let field_count = field_count as i16;
            let mut j: i16 = 0;
            while j < field_count {
                let field_offset = ifd_offset + 2 + 12 * i64::from(j);
                j = j.wrapping_add(1);
                let tag = u16_at(&header, field_offset, order)?;
                let typ = u16_at(&header, field_offset + 2, order)?;
                let n = u32_at(&header, field_offset + 4, order)?;
                if field_offset + 8 + 4 > len {
                    continue;
                }
                let mut value = bytes_at(&header, field_offset + 8, 4, Order::Little)?;
                let pointed = i64::from(u32_at(&value, 0, order)? as i32);
                if ifd == ZEROTH && tag == 0x8769 {
                    add(&mut queue, pointed, EXIF)?;
                } else if ifd == ZEROTH && tag == 0x8825 {
                    add(&mut queue, pointed, GPS)?;
                } else if ifd == EXIF && tag == 0xA005 {
                    add(&mut queue, pointed, INTEROP)?;
                }
                if ifd == EXIF && tag == 0x927C {
                    self.maker_note_offset = u32_at(&value, 0, order)?;
                }
                let base: i32 = match typ {
                    1 | 2 | 6 | 7 => 1,
                    3 | 8 => 2,
                    4 | 9 => 4,
                    5 | 10 => 8,
                    _ => continue,
                };
                let total = (n as i32).wrapping_mul(base);
                if total < 0 {
                    continue;
                }
                if total > 4 {
                    let data_offset = 6i32.wrapping_add(u32_at(&value, 0, order)? as i32);
                    if data_offset < 0
                        || i64::from(data_offset) > len - 1
                        || i64::from(data_offset) + i64::from(total) > len
                    {
                        continue;
                    }
                    let start = usize::try_from(data_offset).unwrap_or(0);
                    let end = start + usize::try_from(total).unwrap_or(0);
                    value = header.get(start..end).unwrap_or_default().to_vec();
                }
                if ifd == FIRST && tag == 0x201 {
                    thumb_type = 0;
                    thumb_offset = u32_at(&value, 0, order)? as i32;
                } else if ifd == FIRST && tag == 0x202 {
                    thumb_length = u32_at(&value, 0, order)? as i32;
                }
                if ifd == FIRST && tag == 0x111 {
                    thumb_type = 1;
                    thumb_offset = if typ == 3 {
                        i32::from(u16_at(&value, 0, order)?)
                    } else {
                        u32_at(&value, 0, order)? as i32
                    };
                } else if ifd == FIRST && tag == 0x117 {
                    // Each strip's length read from the first slot, `count` times.
                    thumb_length = 0;
                    let mut i: i64 = 0;
                    while i < i64::from(n) {
                        let add = if typ == 3 {
                            i32::from(u16_at(&value, 0, order)?)
                        } else {
                            u32_at(&value, 0, order)? as i32
                        };
                        thumb_length = thumb_length.wrapping_add(add);
                        i += 1;
                    }
                }
                let property = factory(tag, typ, n, &value, order, ifd)?;
                self.properties.push(property);
            }
            if ifd == ZEROTH {
                let next_at = ifd_offset + 2 + 12 * i64::from(field_count as u16);
                if next_at + 4 <= len {
                    let next = u32_at(&header, next_at, order)?;
                    if next != 0 && i64::from(next) + 2 <= len {
                        add(&mut queue, i64::from(next), FIRST)?;
                    }
                }
            }
            if thumb_offset != -1
                && thumb_length != 0
                && self.thumbnail.is_none()
                && thumb_type == 0
            {
                let len32 = i32::try_from(header.len()).unwrap_or(i32::MAX);
                if thumb_length > len32.wrapping_sub(6).wrapping_sub(thumb_offset) {
                    self.thumbnail = None;
                } else {
                    // `new byte[length]` and `Array.Copy`, which throw for a negative length or
                    // a source range outside the segment.
                    let start = 6i64 + i64::from(thumb_offset);
                    let end = start + i64::from(thumb_length);
                    let (Ok(start), Ok(end)) = (usize::try_from(start), usize::try_from(end))
                    else {
                        return err("Index was out of range.");
                    };
                    let Some(bytes) = header.get(start..end) else {
                        return err("Source array was not long enough.");
                    };
                    self.thumbnail = Some(bytes.to_vec());
                }
            }
        }
        Ok(())
    }

    /// A section's bytes after its length.
    fn header(&self, at: usize) -> Vec<u8> {
        self.sections
            .get(at)
            .map(|s| s.header.clone())
            .unwrap_or_default()
    }

    fn set_header(&mut self, at: usize, header: Vec<u8>) {
        if let Some(section) = self.sections.get_mut(at) {
            section.header = header;
        }
    }

    fn shift_after_insert(&mut self, index: usize) {
        for slot in [&mut self.jfif_app0, &mut self.jfxx_app0]
            .into_iter()
            .flatten()
        {
            if *slot >= index {
                *slot += 1;
            }
        }
    }

    fn shift_after_remove(&mut self, index: usize) {
        if self.exif_app1 > index {
            self.exif_app1 -= 1;
        }
        for slot in [&mut self.jfif_app0, &mut self.jfxx_app0]
            .into_iter()
            .flatten()
        {
            if *slot > index {
                *slot -= 1;
            }
        }
    }

    /// `WriteJFIFApp0`: the JFIF properties back into their APP0 - version, units, densities,
    /// thumbnail size and pixels, the `SHORT`s big-endian - or the segment removed when there
    /// are none. `// C#: ExifLibrary JPEGFile.WriteJFIFApp0 (IL)`
    fn write_jfif(&mut self) -> Result<(), ExifError> {
        let props: Vec<Property> = self
            .properties
            .iter()
            .filter(|p| tag_ifd(p.tag) == JFIF)
            .cloned()
            .collect();
        if props.is_empty() {
            if let Some(at) = self.jfif_app0.take() {
                self.sections.remove(at);
                self.shift_after_remove(at);
            }
            return Ok(());
        }
        let find = |tag: u32, default: Value| {
            props
                .iter()
                .find(|p| p.tag == tag)
                .cloned()
                .unwrap_or(Property {
                    tag,
                    value: default,
                })
        };
        let ordered = [
            find(JFIF + 1, Value::UShort(0x0102)),
            find(JFIF + 101, Value::Enum(0, EnumKind::Byte)),
            find(JFIF + 102, Value::UShort(1)),
            find(JFIF + 103, Value::UShort(1)),
            find(JFIF + 201, Value::Byte(0)),
            find(JFIF + 202, Value::Byte(0)),
            find(JFIF + 203, Value::Thumbnail(Vec::new(), Vec::new())),
        ];
        let mut header = b"JFIF\0".to_vec();
        for p in &ordered {
            let io = p.interop()?;
            let mut data = io.data;
            if io.typ == 3 {
                data.reverse();
            }
            header.extend(data);
        }
        match self.jfif_app0 {
            Some(at) => self.set_header(at, header),
            None => return err("Object reference not set to an instance of an object"),
        }
        Ok(())
    }

    /// `WriteJFXXApp0`. `// C#: ExifLibrary JPEGFile.WriteJFXXApp0 (IL)`
    fn write_jfxx(&mut self) -> Result<(), ExifError> {
        let props: Vec<Property> = self
            .properties
            .iter()
            .filter(|p| tag_ifd(p.tag) == JFXX)
            .cloned()
            .collect();
        if props.is_empty() {
            if let Some(at) = self.jfxx_app0.take() {
                self.sections.remove(at);
                self.shift_after_remove(at);
            }
            return Ok(());
        }
        let mut header = b"JFXX\0".to_vec();
        for p in &props {
            let io = p.interop()?;
            let mut data = io.data;
            if io.typ == 3 {
                data.reverse();
            }
            header.extend(data);
        }
        match self.jfxx_app0 {
            Some(at) => self.set_header(at, header),
            None => return err("Object reference not set to an instance of an object"),
        }
        Ok(())
    }

    /// `WriteExifApp1(preserveMakerNote: true)`. `// C#: ExifLibrary JPEGFile.WriteExifApp1 (IL)`
    #[allow(clippy::too_many_lines)]
    fn write_exif(&mut self) -> Result<(), ExifError> {
        // The thumbnail's two pointers, found by a walk that keeps the latest of each and stops
        // once it has both: dropped with no thumbnail, added as 0 with one.
        let (mut offset_at, mut length_at) = (None, None);
        for (i, p) in self.properties.iter().enumerate() {
            if p.tag == THUMB_OFFSET_TAG {
                offset_at = Some(i);
            }
            if p.tag == THUMB_LENGTH_TAG {
                length_at = Some(i);
            }
            if offset_at.is_some() && length_at.is_some() {
                break;
            }
        }
        if self.thumbnail.is_none() {
            let mut gone: Vec<usize> = offset_at.into_iter().chain(length_at).collect();
            gone.sort_unstable();
            for i in gone.into_iter().rev() {
                self.properties.remove(i);
            }
        } else {
            if offset_at.is_none() {
                self.properties.push(Property {
                    tag: THUMB_OFFSET_TAG,
                    value: Value::UInt(0),
                });
            }
            if length_at.is_none() {
                self.properties.push(Property {
                    tag: THUMB_LENGTH_TAG,
                    value: Value::UInt(0),
                });
            }
        }
        // One dictionary per IFD: a tag seen again replaces the earlier in its place.
        let mut ifds: [Vec<Property>; 5] = Default::default();
        let slot = |ifd: u32| match ifd {
            ZEROTH => Some(0),
            EXIF => Some(1),
            GPS => Some(2),
            INTEROP => Some(3),
            FIRST => Some(4),
            _ => None,
        };
        for p in &self.properties {
            if let Some(s) = slot(tag_ifd(p.tag))
                && let Some(dict) = ifds.get_mut(s)
            {
                match dict.iter_mut().find(|q| q.tag == p.tag) {
                    Some(q) => *q = p.clone(),
                    None => dict.push(p.clone()),
                }
            }
        }
        let set_if_missing = |dict: &mut Vec<Property>, tag: u32| {
            if !dict.iter().any(|p| p.tag == tag) {
                dict.push(Property {
                    tag,
                    value: Value::UInt(0),
                });
            }
        };
        let remove = |dict: &mut Vec<Property>, tag: u32| dict.retain(|p| p.tag != tag);
        if !ifds[1].is_empty() {
            let [zeroth, ..] = &mut ifds;
            set_if_missing(zeroth, EXIF_POINTER);
        }
        if !ifds[2].is_empty() {
            set_if_missing(&mut ifds[0], GPS_POINTER);
        }
        if !ifds[3].is_empty() {
            set_if_missing(&mut ifds[1], INTEROP_POINTER);
        }
        if ifds[1].is_empty() {
            remove(&mut ifds[0], EXIF_POINTER);
        }
        if ifds[2].is_empty() {
            remove(&mut ifds[0], GPS_POINTER);
        }
        if ifds[3].is_empty() {
            remove(&mut ifds[1], INTEROP_POINTER);
        }
        if ifds.iter().all(Vec::is_empty) && self.thumbnail.is_none() {
            self.set_header(self.exif_app1, Vec::new());
            return Ok(());
        }

        let mut w = Writer {
            out: b"Exif\0\0".to_vec(),
            pos: 6,
            order: self.order,
            fields: HashMap::new(),
            thumb: (0, 0),
        };
        w.write_bytes(if self.order == Order::Little {
            b"II"
        } else {
            b"MM"
        });
        w.write_u16(42);
        w.write_u32(8);
        let tiff = 6usize;
        let [zeroth, exif, gps, interop, first] = ifds;
        w.write_ifd(&zeroth, ZEROTH, tiff, self.maker_note_offset, None)?;
        let exif_at = w.pos - tiff;
        w.write_ifd(&exif, EXIF, tiff, self.maker_note_offset, None)?;
        let gps_at = w.pos - tiff;
        w.write_ifd(&gps, GPS, tiff, self.maker_note_offset, None)?;
        let interop_at = w.pos - tiff;
        w.write_ifd(&interop, INTEROP, tiff, self.maker_note_offset, None)?;
        let first_at = w.pos - tiff;
        w.write_ifd(
            &first,
            FIRST,
            tiff,
            self.maker_note_offset,
            self.thumbnail.as_deref(),
        )?;
        for (field, value) in [
            ("exif", exif_at),
            ("gps", gps_at),
            ("interop", interop_at),
            ("first", first_at),
            ("thumboffset", w.thumb.0),
            ("thumbsize", w.thumb.1),
        ] {
            if let Some(&at) = w.fields.get(field) {
                let bytes = w.u32_bytes(u32::try_from(value).unwrap_or(0));
                w.patch(at, &bytes);
            }
        }
        self.set_header(self.exif_app1, w.out);
        Ok(())
    }

    /// `SaveInternal`: the APP segments rebuilt, then every section - an APP segment left empty
    /// is dropped - each at most 64 KiB.
    /// `// C#: ExifLibrary JPEGFile.SaveInternal (IL)`
    fn save(mut self) -> Result<Vec<u8>, ExifError> {
        self.write_jfif()?;
        self.write_jfxx()?;
        self.write_exif()?;
        let mut out = Vec::new();
        for s in &self.sections {
            if s.header.len() + 2 > 0x10000 {
                return err("Section exceeds 64 KB.");
            }
            if (0xE0..=0xEF).contains(&s.marker) && s.header.is_empty() {
                continue;
            }
            out.extend([0xFF, s.marker]);
            if !(s.marker == 0xD8 || s.marker == 0xD9 || (0xD0..=0xD7).contains(&s.marker)) {
                let length = u16::try_from(s.header.len() + 2).unwrap_or(0);
                out.extend(length.to_be_bytes());
                out.extend_from_slice(&s.header);
            }
            out.extend_from_slice(&s.entropy);
        }
        Ok(out)
    }
}

/// The APP1 being written: a memory stream, with where each pointer field went.
struct Writer {
    out: Vec<u8>,
    pos: usize,
    order: Order,
    fields: HashMap<&'static str, usize>,
    thumb: (usize, usize),
}

impl Writer {
    fn write_bytes(&mut self, bytes: &[u8]) {
        let end = self.pos + bytes.len();
        if self.out.len() < end {
            self.out.resize(end, 0);
        }
        if let Some(slot) = self.out.get_mut(self.pos..end) {
            slot.copy_from_slice(bytes);
        }
        self.pos = end;
    }

    fn patch(&mut self, at: usize, bytes: &[u8]) {
        let saved = self.pos;
        self.pos = at;
        self.write_bytes(bytes);
        self.pos = saved;
    }

    fn u16_bytes(&self, v: u16) -> [u8; 2] {
        match self.order {
            Order::Little => v.to_le_bytes(),
            Order::Big => v.to_be_bytes(),
        }
    }

    fn u32_bytes(&self, v: u32) -> [u8; 4] {
        match self.order {
            Order::Little => v.to_le_bytes(),
            Order::Big => v.to_be_bytes(),
        }
    }

    fn write_u16(&mut self, v: u16) {
        let b = self.u16_bytes(v);
        self.write_bytes(&b);
    }

    fn write_u32(&mut self, v: u32) {
        let b = self.u32_bytes(v);
        self.write_bytes(&b);
    }

    /// `WriteIFD`: the entries in dictionary order (a maker note last, and before it any value
    /// that would run over the maker note's old offset), each value of more than four bytes in
    /// the data area after the directory - the maker note padded with `0xFF` to where it was -
    /// then a zero next-IFD link, and for IFD1 the thumbnail.
    /// `// C#: ExifLibrary JPEGFile.WriteIFD (IL)`
    fn write_ifd(
        &mut self,
        dict: &[Property],
        ifd: u32,
        tiff: usize,
        maker_note_offset: u32,
        thumbnail: Option<&[u8]>,
    ) -> Result<(), ExifError> {
        let mut queue: std::collections::VecDeque<Property> = dict
            .iter()
            .filter(|p| p.tag != MAKER_NOTE)
            .cloned()
            .collect();
        let has_maker_note = dict.iter().any(|p| p.tag == MAKER_NOTE);
        if let Some(note) = dict.iter().find(|p| p.tag == MAKER_NOTE) {
            queue.push_back(note.clone());
        }
        let n = dict.len();
        let mut data_offset = u32::try_from(2 + n * 12 + 4 + self.pos - tiff).unwrap_or(0);
        let mut data_pos = self.pos + 2 + n * 12 + 4;
        let mut maker_note_written = false;
        #[allow(clippy::cast_possible_truncation)]
        self.write_u16(n as u16);
        while let Some(p) = queue.pop_front() {
            let io = p.interop()?;
            let mut padding: u32 = 0;
            if !maker_note_written
                && maker_note_offset != 0
                && ifd == EXIF
                && p.tag != MAKER_NOTE
                && io.data.len() > 4
                && u64::from(data_offset) + io.data.len() as u64 > u64::from(maker_note_offset)
                && has_maker_note
            {
                queue.push_back(p);
                continue;
            }
            if p.tag == MAKER_NOTE {
                maker_note_written = true;
                padding = maker_note_offset.saturating_sub(data_offset);
            }
            self.write_u16(io.id);
            self.write_u16(io.typ);
            self.write_u32(io.count);
            let mut data = io.data;
            if self.order == Order::Big {
                let size = match io.typ {
                    3 => Some(2),
                    4 | 9 | 5 | 10 => Some(4),
                    _ => None,
                };
                if let Some(size) = size {
                    for chunk in data.chunks_mut(size) {
                        if chunk.len() == size {
                            chunk.reverse();
                        }
                    }
                }
            }
            match (ifd, io.id) {
                (ZEROTH, 0x8769) => {
                    self.fields.insert("exif", self.pos);
                }
                (ZEROTH, 0x8825) => {
                    self.fields.insert("gps", self.pos);
                }
                (EXIF, 0xA005) => {
                    self.fields.insert("interop", self.pos);
                }
                (FIRST, 0x201) => {
                    self.fields.insert("thumboffset", self.pos);
                }
                (FIRST, 0x202) => {
                    self.fields.insert("thumbsize", self.pos);
                }
                _ => {}
            }
            if data.len() <= 4 {
                let mut padded = data.clone();
                padded.resize(4, 0);
                self.write_bytes(&padded);
            } else {
                let value = self.u32_bytes(data_offset.wrapping_add(padding));
                self.write_bytes(&value);
                let back = self.pos;
                self.pos = data_pos;
                self.write_bytes(&vec![0xFF; usize::try_from(padding).unwrap_or(0)]);
                self.write_bytes(&data);
                self.pos = back;
                let grown = padding.wrapping_add(u32::try_from(data.len()).unwrap_or(0));
                data_offset = data_offset.wrapping_add(grown);
                data_pos += usize::try_from(grown).unwrap_or(0);
            }
        }
        if ifd == ZEROTH {
            self.fields.insert("first", self.pos);
        }
        self.write_bytes(&[0; 4]);
        self.pos = data_pos;
        if self.out.len() < data_pos {
            self.out.resize(data_pos, 0);
        }
        if ifd == FIRST {
            match thumbnail {
                Some(thumb) => {
                    self.thumb = (self.pos - tiff, thumb.len());
                    self.write_bytes(thumb);
                }
                None => self.thumb = (0, 0),
            }
        }
        Ok(())
    }
}

/// `double.toDMS()`: whole degrees, whole minutes and the seconds as a `float`, each truncated
/// toward zero. `// C#: ExtLibs/Utilities/Extensions.cs:1024-1031`
#[allow(clippy::cast_possible_truncation)]
#[must_use]
pub fn to_dms(angle: f64) -> (i32, i32, f32) {
    let degrees = angle;
    let minutes = (degrees - f64::from(degrees as i32)) * 60.0;
    let seconds = (minutes - f64::from(minutes as i32)) * 60.0;
    (degrees as i32, minutes as i32, seconds as f32)
}

/// The geotagged copy of one photo's bytes, or why ExifLibrary threw.
/// `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:1172-1223`
///
/// # Errors
///
/// Where the C# lands in `catch { AppendText("There was a problem with image " ...) }`.
#[allow(clippy::cast_precision_loss)]
pub fn geotag_bytes(photo: &[u8], d_lat: f64, d_long: f64, alt: f64) -> Result<Vec<u8>, ExifError> {
    // `ImageFile.FromStream`: eight bytes at least, then the format by its magic number.
    if photo.len() < 8 {
        return err("Not a valid image file.");
    }
    if photo.first() != Some(&0xFF) || photo.get(1) != Some(&0xD8) {
        // TIFF (TIFFFile), PNG and GIF are ExifLibrary's too; only JPEG is ported.
        return err("Not a valid image file.");
    }
    let mut data = JpegFile::read(photo)?;
    // The listing loop: each property's ToString and Interoperability, then the removals.
    let listed = data.properties.clone();
    for item in &listed {
        if item.to_string_throws() {
            return err("Index was outside the bounds of the array.");
        }
        item.interop()?;
        let removed = matches!(
            item.tag,
            t if t == GPS + 4 || t == GPS + 2 || t == GPS + 6 || t == GPS + 1 || t == GPS + 3
                || t == MAKER_NOTE || t == FIRST + 0x11B || t == FIRST + 0x11A || t == EXIF + 0xA000
        );
        if removed && let Some(i) = data.properties.iter().position(|p| p == item) {
            data.properties.remove(i);
        }
    }
    let (lon_d, lon_m, lon_s) = to_dms(d_long);
    let fractions = |d: i32, m: i32, s: f32| -> Result<Vec<UFraction32>, ExifError> {
        Ok(vec![
            UFraction32::from_double(f64::from(d.unsigned_abs() as f32))?,
            UFraction32::from_double(f64::from(m.unsigned_abs() as f32))?,
            UFraction32::from_double(f64::from(s.abs()))?,
        ])
    };
    data.properties.push(Property {
        tag: GPS + 4,
        value: Value::URationalArray(fractions(lon_d, lon_m, lon_s)?, Shape::Plain),
    });
    let (lat_d, lat_m, lat_s) = to_dms(d_lat);
    data.properties.push(Property {
        tag: GPS + 2,
        value: Value::URationalArray(fractions(lat_d, lat_m, lat_s)?, Shape::Plain),
    });
    data.properties.push(Property {
        tag: GPS + 6,
        value: Value::URational(UFraction32::from_double(alt)?),
    });
    data.properties.push(Property {
        tag: GPS + 1,
        value: Value::Ascii(
            if d_lat < 0.0 { "S" } else { "N" }.to_owned(),
            TextEncoding::Utf8,
        ),
    });
    data.properties.push(Property {
        tag: GPS + 3,
        value: Value::Ascii(
            if d_long < 0.0 { "W" } else { "E" }.to_owned(),
            TextEncoding::Utf8,
        ),
    });
    data.save()
}

/// `WriteCoordinatesToImage`: `<root>/geotagged/<name>_geotag<ext>`, replacing any such file,
/// or "There was a problem with image `<path>`" (no line end) when ExifLibrary throws.
/// `// C#: ExtLibs/Utilities/GeoRefImageBase.cs:1167-1231`
pub fn write_coordinates_to_image(
    filename: &str,
    d_lat: f64,
    d_long: f64,
    alt: f64,
    root_folder: &str,
    append_text: Output<'_>,
) {
    // File.ReadAllBytes is outside the try: an unreadable photo throws out of the loop.
    let Ok(bytes) = std::fs::read(filename) else {
        append_text(&format!("There was a problem with image {filename}"));
        return;
    };
    append_text(&format!("GeoTagging {filename}\n"));
    let result = geotag_bytes(&bytes, d_lat, d_long, alt).and_then(|out| {
        let folder = format!("{root_folder}{SEPARATOR}geotagged");
        let output = format!(
            "{folder}{SEPARATOR}{}_geotag{}",
            photos::file_stem(filename),
            photos::extension(filename)
        );
        std::fs::create_dir_all(&folder).map_err(|e| ExifError(e.to_string()))?;
        std::fs::write(&output, out).map_err(|e| ExifError(e.to_string()))
    });
    if result.is_err() {
        append_text(&format!("There was a problem with image {filename}"));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fractions_reduce_and_continued_fractions_stop_at_equality() {
        assert_eq!(UFraction32::new(300, 100), UFraction32 { num: 3, den: 1 });
        assert_eq!(UFraction32::new(0, 5), UFraction32 { num: 0, den: 1 });
        assert_eq!(
            UFraction32::from_double(20.5).unwrap(),
            UFraction32 { num: 41, den: 2 }
        );
        assert_eq!(
            UFraction32::from_double(153.0).unwrap(),
            UFraction32 { num: 153, den: 1 }
        );
        assert!(UFraction32::from_double(-1.0).is_err());
        let f = Fraction32::new(-10, 4);
        assert_eq!((f.numerator(), f.den), (-5, 2));
    }

    #[test]
    fn dms_truncates_toward_zero() {
        let (d, m, s) = to_dms(-27.469_799_8);
        assert_eq!((d, m), (-27, -28));
        assert!((s + 11.279).abs() < 0.001);
    }
}
