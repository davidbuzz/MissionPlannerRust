//! Synthetic photos: tiny valid JPEGs carrying the EXIF a camera writes, built here byte by byte
//! so every layout the readers and the geotag writer must handle is present and known.
//!
//! The 25 photos of `testdata/georef/photos` are the 25 pictures `camera.bin` records: photo *n*
//! is dated by `CAM` message *n* as a camera would - its clock on Brisbane time (UTC+10) and 3.3 s
//! fast, the fraction dropped - so `CAM` 12 and 13, 0.4 s apart, give two photos of the same
//! second. Their EXIF differs on purpose: both byte orders, a JFIF segment, an existing GPS
//! directory, a user comment and a maker note, a photo dated only by `DateTimeDigitized`, one
//! with no thumbnail directory, rationals a camera did not reduce, a Windows comment and a subject
//! area.

#![allow(dead_code)]

/// Byte order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Order {
    /// `II`.
    Intel,
    /// `MM`.
    Motorola,
}

impl Order {
    fn u16(self, v: u16) -> [u8; 2] {
        match self {
            Self::Intel => v.to_le_bytes(),
            Self::Motorola => v.to_be_bytes(),
        }
    }

    fn u32(self, v: u32) -> [u8; 4] {
        match self {
            Self::Intel => v.to_le_bytes(),
            Self::Motorola => v.to_be_bytes(),
        }
    }
}

/// One IFD entry, its value already in the file's byte order.
#[derive(Debug, Clone)]
pub(crate) struct Entry {
    pub(crate) tag: u16,
    pub(crate) typ: u16,
    pub(crate) count: u32,
    pub(crate) data: Vec<u8>,
}

pub(crate) fn ascii(tag: u16, text: &str) -> Entry {
    let mut data = text.as_bytes().to_vec();
    data.push(0);
    Entry {
        tag,
        typ: 2,
        count: u32::try_from(data.len()).unwrap(),
        data,
    }
}

pub(crate) fn short(order: Order, tag: u16, values: &[u16]) -> Entry {
    Entry {
        tag,
        typ: 3,
        count: u32::try_from(values.len()).unwrap(),
        data: values.iter().flat_map(|&v| order.u16(v)).collect(),
    }
}

pub(crate) fn long(order: Order, tag: u16, values: &[u32]) -> Entry {
    Entry {
        tag,
        typ: 4,
        count: u32::try_from(values.len()).unwrap(),
        data: values.iter().flat_map(|&v| order.u32(v)).collect(),
    }
}

pub(crate) fn rational(order: Order, tag: u16, values: &[(u32, u32)]) -> Entry {
    Entry {
        tag,
        typ: 5,
        count: u32::try_from(values.len()).unwrap(),
        data: values
            .iter()
            .flat_map(|&(n, d)| {
                let mut b = order.u32(n).to_vec();
                b.extend(order.u32(d));
                b
            })
            .collect(),
    }
}

pub(crate) fn srational(order: Order, tag: u16, values: &[(i32, i32)]) -> Entry {
    Entry {
        tag,
        typ: 10,
        count: u32::try_from(values.len()).unwrap(),
        data: values
            .iter()
            .flat_map(|&(n, d)| {
                let mut b = order.u32(n as u32).to_vec();
                b.extend(order.u32(d as u32));
                b
            })
            .collect(),
    }
}

pub(crate) fn undefined(tag: u16, bytes: &[u8]) -> Entry {
    Entry {
        tag,
        typ: 7,
        count: u32::try_from(bytes.len()).unwrap(),
        data: bytes.to_vec(),
    }
}

pub(crate) fn byte(tag: u16, bytes: &[u8]) -> Entry {
    Entry {
        tag,
        typ: 1,
        count: u32::try_from(bytes.len()).unwrap(),
        data: bytes.to_vec(),
    }
}

/// A TIFF structure: IFD0, the Exif, Interop and GPS directories and IFD1, with the pointers
/// between them made by [`Tiff::build`].
#[derive(Debug, Clone)]
pub(crate) struct Tiff {
    pub(crate) order: Order,
    pub(crate) ifd0: Vec<Entry>,
    pub(crate) exif: Vec<Entry>,
    pub(crate) interop: Vec<Entry>,
    pub(crate) gps: Vec<Entry>,
    pub(crate) ifd1: Vec<Entry>,
    pub(crate) thumbnail: Option<Vec<u8>>,
    /// Entries kept in the order given rather than sorted by tag, as a careless writer leaves them.
    pub(crate) unsorted: bool,
}

fn dir_size(entries: &[Entry]) -> usize {
    2 + 12 * entries.len() + 4
}

fn data_size(entries: &[Entry]) -> usize {
    entries
        .iter()
        .map(|e| {
            if e.data.len() > 4 {
                (e.data.len() + 1) & !1
            } else {
                0
            }
        })
        .sum()
}

impl Tiff {
    /// The TIFF bytes, laid out as cameras lay them out: the header, then each directory followed
    /// by its values (each on an even offset), in the order IFD0, Exif, Interop, GPS, IFD1, and the
    /// thumbnail last.
    pub(crate) fn build(&self) -> Vec<u8> {
        let o = self.order;
        let mut ifd0 = self.ifd0.clone();
        let mut exif = self.exif.clone();
        let interop = self.interop.clone();
        let gps = self.gps.clone();
        let mut ifd1 = self.ifd1.clone();
        // Placeholders; the values are set once the offsets are known.
        if !exif.is_empty() {
            ifd0.push(long(o, 0x8769, &[0]));
        }
        if !gps.is_empty() {
            ifd0.push(long(o, 0x8825, &[0]));
        }
        if !interop.is_empty() {
            exif.push(long(o, 0xA005, &[0]));
        }
        if let Some(thumb) = &self.thumbnail {
            ifd1.push(long(o, 0x0201, &[0]));
            ifd1.push(long(o, 0x0202, &[u32::try_from(thumb.len()).unwrap()]));
        }
        let mut interop = interop;
        let mut gps = gps;
        if !self.unsorted {
            for list in [&mut ifd0, &mut exif, &mut ifd1, &mut interop, &mut gps] {
                list.sort_by_key(|e| e.tag);
            }
        }

        let mut at = 8usize;
        let mut place = |entries: &Vec<Entry>| -> usize {
            if entries.is_empty() {
                return 0;
            }
            let here = at;
            at += dir_size(entries) + data_size(entries);
            here
        };
        let ifd0_at = place(&ifd0);
        let exif_at = place(&exif);
        let interop_at = place(&interop);
        let gps_at = place(&gps);
        let ifd1_at = place(&ifd1);
        let thumb_at = at;

        let set = |entries: &mut Vec<Entry>, tag: u16, value: usize| {
            for e in entries.iter_mut().filter(|e| e.tag == tag) {
                e.data = o.u32(u32::try_from(value).unwrap()).to_vec();
            }
        };
        set(&mut ifd0, 0x8769, exif_at);
        set(&mut ifd0, 0x8825, gps_at);
        set(&mut exif, 0xA005, interop_at);
        set(&mut ifd1, 0x0201, thumb_at);

        let mut out = match o {
            Order::Intel => b"II".to_vec(),
            Order::Motorola => b"MM".to_vec(),
        };
        out.extend(o.u16(42));
        out.extend(o.u32(8));
        let write = |out: &mut Vec<u8>, entries: &Vec<Entry>, start: usize, next: usize| {
            if entries.is_empty() {
                return;
            }
            assert_eq!(out.len(), start);
            out.extend(o.u16(u16::try_from(entries.len()).unwrap()));
            let mut data_at = start + dir_size(entries);
            let mut data = Vec::new();
            for e in entries {
                out.extend(o.u16(e.tag));
                out.extend(o.u16(e.typ));
                out.extend(o.u32(e.count));
                if e.data.len() > 4 {
                    out.extend(o.u32(u32::try_from(data_at).unwrap()));
                    data.extend_from_slice(&e.data);
                    if e.data.len() % 2 == 1 {
                        data.push(0);
                    }
                    data_at = start + dir_size(entries) + data.len();
                } else {
                    let mut v = e.data.clone();
                    v.resize(4, 0);
                    out.extend(v);
                }
            }
            out.extend(o.u32(u32::try_from(next).unwrap()));
            out.extend(data);
        };
        write(&mut out, &ifd0, ifd0_at, ifd1_at);
        write(&mut out, &exif, exif_at, 0);
        write(&mut out, &interop, interop_at, 0);
        write(&mut out, &gps, gps_at, 0);
        write(&mut out, &ifd1, ifd1_at, 0);
        if let Some(thumb) = &self.thumbnail {
            assert_eq!(out.len(), thumb_at);
            out.extend_from_slice(thumb);
        }
        out
    }
}

fn segment(marker: u8, body: &[u8]) -> Vec<u8> {
    let mut out = vec![0xFF, marker];
    out.extend(u16::try_from(body.len() + 2).unwrap().to_be_bytes());
    out.extend_from_slice(body);
    out
}

/// A complete baseline JPEG of one flat grey 8x8 block, and nothing else: a quantisation table of
/// ones, one component, a DC and an AC Huffman table each holding the one code `0` (DC difference
/// 0, end of block), and the two bits of that scan padded with ones.
pub(crate) fn jpeg_body() -> Vec<u8> {
    let mut out = segment(0xDB, &{
        let mut b = vec![0x00];
        b.extend([1u8; 64]);
        b
    });
    out.extend(segment(0xC0, &[8, 0, 8, 0, 8, 1, 1, 0x11, 0]));
    let mut table = vec![0x00, 1];
    table.extend([0u8; 15]);
    table.push(0x00);
    out.extend(segment(0xC4, &table));
    if let Some(class) = table.first_mut() {
        *class = 0x10;
    }
    out.extend(segment(0xC4, &table));
    out.extend(segment(0xDA, &[1, 1, 0x00, 0, 63, 0]));
    out.extend([0x3F, 0xFF, 0xD9]);
    out
}

/// A thumbnail: the same image as its own file.
pub(crate) fn thumbnail() -> Vec<u8> {
    let mut out = vec![0xFF, 0xD8];
    out.extend(jpeg_body());
    out
}

/// A JFIF APP0: version 1.01, 72 dots per inch, no thumbnail.
pub(crate) fn jfif() -> Vec<u8> {
    segment(
        0xE0,
        &[b'J', b'F', b'I', b'F', 0, 1, 1, 1, 0, 72, 0, 72, 0, 0],
    )
}

/// A JPEG file: SOI, the JFIF segment if asked for, the EXIF in APP1, the image.
pub(crate) fn jpeg(tiff: &[u8], with_jfif: bool) -> Vec<u8> {
    let mut out = vec![0xFF, 0xD8];
    if with_jfif {
        out.extend(jfif());
    }
    let mut app1 = b"Exif\0\0".to_vec();
    app1.extend_from_slice(tiff);
    out.extend(segment(0xE1, &app1));
    out.extend(jpeg_body());
    out
}

/// Seconds after 01:30:00 UTC-and-a-second (the C#'s `CAM` time, 17 leap seconds) of the 25
/// `CAM` messages in `camera.bin`.
pub(crate) const CAM_SECONDS: [f64; 25] = [
    5.6, 7.6, 9.6, 11.4, 13.8, 15.8, 17.8, 19.8, 21.6, 23.8, 26.0, 28.0, 28.4, 30.2, 32.2, 34.2,
    36.0, 38.2, 40.2, 42.2, 44.2, 46.2, 48.4, 50.4, 52.4,
];

/// Photo `n`'s EXIF time: its `CAM` time on a camera clock of UTC+10, 3.3 s fast, the fraction
/// dropped.
pub(crate) fn photo_time(n: usize) -> String {
    let s = CAM_SECONDS[n - 1] + 3.3;
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let whole = s.floor() as u32;
    format!("2026:09:24 11:{:02}:{:02}", 30 + whole / 60, whole % 60)
}

/// How photo `n` differs from the plain one.
#[derive(Debug, Clone, Copy, Default)]
pub(crate) struct Variant {
    pub(crate) motorola: bool,
    pub(crate) jfif: bool,
    pub(crate) gps: bool,
    pub(crate) comment: bool,
    pub(crate) digitized_only: bool,
    pub(crate) no_thumbnail: bool,
    pub(crate) odd_values: bool,
}

pub(crate) fn variant(n: usize) -> Variant {
    let mut v = Variant::default();
    match n {
        2 => v.motorola = true,
        3 => v.jfif = true,
        4 => v.gps = true,
        5 => v.comment = true,
        6 => v.digitized_only = true,
        7 => v.no_thumbnail = true,
        8 => v.odd_values = true,
        9 => {
            v.motorola = true;
            v.gps = true;
            v.jfif = true;
        }
        10 => {
            v.motorola = true;
            v.comment = true;
            v.no_thumbnail = true;
        }
        _ => v.motorola = n.is_multiple_of(2),
    }
    v
}

/// Photo `n` (1 to 25) of `testdata/georef/photos`.
pub(crate) fn photo(n: usize) -> Vec<u8> {
    let v = variant(n);
    let o = if v.motorola {
        Order::Motorola
    } else {
        Order::Intel
    };
    let time = photo_time(n);
    let mut ifd0 = vec![
        ascii(0x010F, "MPR"),
        ascii(0x0110, "GeorefCam"),
        short(o, 0x0112, &[1]),
        rational(o, 0x011A, &[(72, 1)]),
        rational(o, 0x011B, &[(72, 1)]),
        short(o, 0x0128, &[2]),
        ascii(0x0131, "mp-georef test"),
        ascii(0x0132, &time),
        short(o, 0x0213, &[1]),
    ];
    let mut exif = vec![
        rational(o, 0x829A, &[(1, 500)]),
        rational(o, 0x829D, &[(28, 10)]),
        short(o, 0x8822, &[2]),
        short(o, 0x8827, &[100]),
        undefined(0x9000, b"0230"),
        undefined(0x9101, &[1, 2, 3, 0]),
        srational(o, 0x9201, &[(8966, 1000)]),
        srational(o, 0x9204, &[(0, 10)]),
        short(o, 0x9207, &[5]),
        short(o, 0x9209, &[16]),
        rational(o, 0x920A, &[(36, 10)]),
        undefined(0xA000, b"0100"),
        short(o, 0xA001, &[1]),
        long(o, 0xA002, &[8]),
        long(o, 0xA003, &[8]),
    ];
    if v.digitized_only {
        exif.push(ascii(0x9004, &time));
    } else {
        exif.push(ascii(0x9003, &time));
        // A different digitized time, so only the original can be the one read.
        exif.push(ascii(0x9004, "2026:09:24 12:00:00"));
    }
    if v.comment {
        let mut comment = b"ASCII\0\0\0".to_vec();
        comment.extend_from_slice(format!("photo {n}   ").as_bytes());
        exif.push(undefined(0x9286, &comment));
        exif.push(undefined(
            0x927C,
            b"MPRNOTE\0\x01\x02\x03\x04\x05\x06\x07\x08",
        ));
    }
    if v.odd_values {
        // Rationals a camera did not reduce, a Windows comment, a subject area.
        ifd0.retain(|e| e.tag != 0x011A && e.tag != 0x011B);
        ifd0.push(rational(o, 0x011A, &[(720_000, 10_000)]));
        ifd0.push(rational(o, 0x011B, &[(300, 100)]));
        let xp: Vec<u8> = "Hi\0".encode_utf16().flat_map(u16::to_le_bytes).collect();
        ifd0.push(byte(0x9C9C, &xp));
        exif.push(short(o, 0x9214, &[4, 4, 2, 2]));
    }
    let interop = vec![ascii(0x0001, "R98"), undefined(0x0002, b"0100")];
    let gps = if v.gps {
        vec![
            byte(0x0000, &[2, 3, 0, 0]),
            ascii(0x0001, "S"),
            rational(o, 0x0002, &[(27, 1), (28, 1), (1100, 100)]),
            ascii(0x0003, "E"),
            rational(o, 0x0004, &[(153, 1), (1, 1), (3036, 100)]),
            byte(0x0005, &[0]),
            rational(o, 0x0006, &[(6500, 100)]),
            rational(o, 0x0007, &[(1, 1), (30, 1), (5, 1)]),
            ascii(0x001D, "2026:09:24"),
        ]
    } else {
        Vec::new()
    };
    let (ifd1, thumb) = if v.no_thumbnail {
        (Vec::new(), None)
    } else {
        (
            vec![
                short(o, 0x0103, &[6]),
                rational(o, 0x011A, &[(72, 1)]),
                rational(o, 0x011B, &[(72, 1)]),
                short(o, 0x0128, &[2]),
            ],
            Some(thumbnail()),
        )
    };
    let tiff = Tiff {
        order: o,
        ifd0,
        exif,
        interop,
        gps,
        ifd1,
        thumbnail: thumb,
        unsorted: false,
    };
    jpeg(&tiff.build(), v.jfif)
}

/// The file name of photo `n`.
pub(crate) fn photo_name(n: usize) -> String {
    format!("IMG_{n:04}.jpg")
}

/// A plain EXIF structure dated `date` in `DateTimeOriginal`, for the edge photos to change.
fn plain(order: Order, date: &str) -> Tiff {
    let o = order;
    Tiff {
        order: o,
        ifd0: vec![
            ascii(0x010F, "MPR"),
            ascii(0x0110, "EdgeCam"),
            short(o, 0x0112, &[1]),
            rational(o, 0x011A, &[(72, 1)]),
            rational(o, 0x011B, &[(72, 1)]),
            short(o, 0x0128, &[2]),
        ],
        exif: vec![undefined(0x9000, b"0230"), ascii(0x9003, date)],
        interop: Vec::new(),
        gps: Vec::new(),
        ifd1: Vec::new(),
        thumbnail: None,
        unsorted: false,
    }
}

/// Photos at the edges of what the readers and the geotag writer handle, in
/// `testdata/georef/edge`: each file's name says what it holds.
pub(crate) fn edge_photos() -> Vec<(&'static str, Vec<u8>)> {
    let date = "2026:09:24 11:30:08";
    let mut out = Vec::new();

    // No APP1 at all: no date, and ExifLibrary makes one.
    let mut bare = vec![0xFF, 0xD8];
    bare.extend(jpeg_body());
    out.push(("e01_no_exif.jpg", bare));

    // An XMP APP1 before the Exif one: the date reader reads only the first.
    let mut xmp = vec![0xFF, 0xD8];
    xmp.extend(segment(0xE1, b"http://ns.adobe.com/xap/1.0/\0<x:xmpmeta/>"));
    let mut app1 = b"Exif\0\0".to_vec();
    app1.extend(plain(Order::Intel, date).build());
    xmp.extend(segment(0xE1, &app1));
    xmp.extend(jpeg_body());
    out.push(("e02_xmp_first.jpg", xmp));

    // Dates no format reads, one TryParse reads, and one of zeros.
    out.push((
        "e03_blank_date.jpg",
        jpeg(&plain(Order::Intel, "    :  :     :  :  ").build(), false),
    ));
    out.push((
        "e04_zero_date.jpg",
        jpeg(&plain(Order::Intel, "0000:00:00 00:00:00").build(), false),
    ));
    out.push((
        "e05_iso_date.jpg",
        jpeg(&plain(Order::Intel, "2026-09-24 11:30:08").build(), false),
    ));

    // Big-endian signed shorts, which ExifLibrary writes back little-endian; a FLOAT it drops;
    // signed rationals with their signs on either side.
    let mut t = plain(Order::Motorola, date);
    t.exif.push(Entry {
        tag: 0xC000,
        typ: 8,
        count: 2,
        data: vec![0xFF, 0xFE, 0x00, 0x02],
    });
    t.exif.push(Entry {
        tag: 0xC001,
        typ: 11,
        count: 1,
        data: 1.5f32.to_be_bytes().to_vec(),
    });
    t.ifd0
        .push(srational(Order::Motorola, 0xC002, &[(-10, 4), (7, -21)]));
    out.push(("e06_mm_sshort_float.jpg", jpeg(&t.build(), false)));

    // Short BYTE, SBYTE and UNDEFINED values: the unsigned ones widened to four on the way back.
    let mut t = plain(Order::Intel, date);
    t.exif.push(undefined(0xC003, &[1, 2]));
    t.exif.push(byte(0xC004, &[9, 8, 7]));
    t.exif.push(Entry {
        tag: 0xC005,
        typ: 6,
        count: 3,
        data: vec![0xFF, 0x01, 0x80],
    });
    out.push(("e07_short_values.jpg", jpeg(&t.build(), false)));

    // A Unicode user comment, and a comment in no encoding it knows.
    let mut t = plain(Order::Intel, date);
    let mut comment = b"Unicode\0".to_vec();
    comment.extend("h\u{e9}llo\0".encode_utf16().flat_map(u16::to_le_bytes));
    t.exif.push(undefined(0x9286, &comment));
    out.push(("e08_unicode_comment.jpg", jpeg(&t.build(), false)));
    let mut t = plain(Order::Intel, date);
    t.exif
        .push(undefined(0x9286, b"\0\0\0\0\0\0\0\0  plain text  "));
    out.push(("e09_unknown_comment.jpg", jpeg(&t.build(), false)));

    // A GPS latitude of two values, which GPSLatitudeLongitude.ToString indexes past.
    let mut t = plain(Order::Intel, date);
    t.gps = vec![
        ascii(0x0001, "S"),
        rational(Order::Intel, 0x0002, &[(27, 1), (28, 1)]),
    ];
    out.push(("e10_gps_short_latitude.jpg", jpeg(&t.build(), false)));

    // JFIF and a JFXX thumbnail extension.
    let mut jfxx = vec![0xFF, 0xD8];
    jfxx.extend(jfif());
    let mut ext = b"JFXX\0\x10".to_vec();
    ext.extend(thumbnail());
    jfxx.extend(segment(0xE0, &ext));
    let mut app1 = b"Exif\0\0".to_vec();
    app1.extend(plain(Order::Intel, date).build());
    jfxx.extend(segment(0xE1, &app1));
    jfxx.extend(jpeg_body());
    out.push(("e11_jfxx.jpg", jfxx));

    // Bytes after the end of the image, which ExifLibrary does not keep.
    let mut trailing = jpeg(&plain(Order::Intel, date).build(), false);
    trailing.extend(b"trailing data after EOI");
    out.push(("e12_trailing.jpg", trailing));

    // Cut off inside the scan: no EOI.
    let mut cut = jpeg(&plain(Order::Intel, date).build(), false);
    cut.truncate(cut.len() - 2);
    out.push(("e13_no_eoi.jpg", cut));

    // The GPS pointer at the Exif directory's offset: ExifLibrary's SortedList refuses it.
    let mut t = plain(Order::Intel, date);
    t.gps = vec![ascii(0x0001, "N")];
    let mut built = t.build();
    // IFD0's entries are sorted, so the Exif pointer and then the GPS pointer are the last two.
    let count = usize::from(u16::from_le_bytes([built[8], built[9]]));
    let gps_entry = 10 + 12 * (count - 1);
    let exif_entry = 10 + 12 * (count - 2);
    let exif_value: [u8; 4] = built[exif_entry + 8..exif_entry + 12].try_into().unwrap();
    built[gps_entry + 8..gps_entry + 12].copy_from_slice(&exif_value);
    out.push(("e14_shared_ifd_offset.jpg", jpeg(&built, false)));

    // A strip thumbnail, which ExifLibrary does not keep.
    let mut t = plain(Order::Intel, date);
    t.ifd1 = vec![
        short(Order::Intel, 0x0103, &[1]),
        long(Order::Intel, 0x0111, &[0]),
        long(Order::Intel, 0x0117, &[16]),
    ];
    out.push(("e15_strip_thumbnail.jpg", jpeg(&t.build(), false)));

    // Text that is not ASCII: UTF-8 e-acute, and a byte UTF-8 cannot hold.
    let mut t = plain(Order::Intel, date);
    t.ifd0.push(Entry {
        tag: 0x013B,
        typ: 2,
        count: 6,
        data: b"Caf\xc3\xa9\0".to_vec(),
    });
    t.ifd0.push(Entry {
        tag: 0x8298,
        typ: 2,
        count: 4,
        data: b"a\xffb\0".to_vec(),
    });
    out.push(("e16_non_ascii.jpg", jpeg(&t.build(), false)));

    // No Make, and a maker note before the date in an unsorted directory: the date reader's
    // maker-note dispatch throws and never reaches the date.
    let mut t = plain(Order::Intel, date);
    t.ifd0.retain(|e| e.tag != 0x010F);
    t.exif = vec![
        undefined(0x927C, b"UNKNOWNNOTE\0\0\0\0\0"),
        ascii(0x9003, date),
        undefined(0x9000, b"0230"),
    ];
    t.unsorted = true;
    out.push(("e17_makernote_first.jpg", jpeg(&t.build(), false)));

    // The same with a Make: the dispatch finds nothing to do and the date is read.
    let mut t = plain(Order::Intel, date);
    t.exif = vec![
        undefined(0x927C, b"UNKNOWNNOTE\0\0\0\0\0"),
        ascii(0x9003, date),
    ];
    t.unsorted = true;
    out.push(("e18_makernote_with_make.jpg", jpeg(&t.build(), false)));

    // A TIFF photo: dated by the TIFF reader, geotagged by nothing ported.
    out.push(("e19_tiff.tif", plain(Order::Intel, date).build()));

    // GPS fields: the version, an unreduced time stamp, the date stamp, a differential, a
    // speed reference.
    let mut t = plain(Order::Motorola, date);
    t.gps = vec![
        byte(0x0000, &[2, 3, 0, 0]),
        rational(Order::Motorola, 0x0007, &[(10, 1), (3000, 100), (500, 100)]),
        ascii(0x001D, "2026:09:24"),
        short(Order::Motorola, 0x001E, &[1]),
        ascii(0x000C, "K"),
    ];
    out.push(("e20_gps_fields.jpg", jpeg(&t.build(), false)));

    out
}
