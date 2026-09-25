//! `LogMetaData`: what each log message and field is, from ArduPilot's `LogMessages.xml`.
//!
//! At start-up Mission Planner queues `BGLogMessagesMetaData` on the thread pool: `GetMetaData`
//! downloads `LogMessages.xml.xz` for Copter, Plane, Rover and Tracker from autotest into the
//! data directory - unless the copy there is under seven days old - and unpacks each to
//! `LogMessages<Vehicle>.xml`; then `ParseMetaData` reads the four into one static dictionary,
//! message name to field name to description, with `"description"` for the message's own. The
//! log browser's field tree reads it when the pointer rests on a node
//! (`treeView1_TreeNodeMouseHover`), putting the description in `txt_info`.
//!
//! The download is skipped under `MP_OFFLINE`, as the map's and the terrain's are; what is on
//! disk is unpacked and read all the same.
//! `// C#: ExtLibs/ArduPilot/LogMetaData.cs:19-166; MainV2.cs:3299, 3868-3872`

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::OnceLock;
use std::time::Duration;

/// `vehicles`, in the C#'s order: a message in more than one file takes the last one's text.
pub const VEHICLES: [&str; 4] = ["Copter", "Plane", "Rover", "Tracker"];

/// `url`, with the vehicle for `{0}`.
pub fn url(vehicle: &str) -> String {
    format!("https://autotest.ardupilot.org/LogMessages/{vehicle}/LogMessages.xml.xz")
}

/// A download younger than this is kept: `LastWriteTime.AddDays(7) > DateTime.Now`.
pub const FRESH: Duration = Duration::from_secs(7 * 24 * 60 * 60);

/// The key a message's own description is kept under, beside its fields.
pub const DESCRIPTION: &str = "description";

/// `LogItemFeildBitmask`: one bit of a bitmask field.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Bit {
    /// `name`: the `bit` element's `name` attribute, null without one.
    pub name: Option<String>,
    /// `mask`: its `value`.
    pub mask: u32,
    /// `description`: null without one.
    pub description: Option<String>,
}

/// `LogItemFeild`: a field's description, and its bits when it is a bitmask.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Field {
    /// `description`.
    pub description: String,
    /// `bitmask`: null unless the field has a `bitmask` element.
    pub bitmask: Option<Vec<Bit>>,
}

/// `LogMetaData.MetaData`: message name, then field name - or [`DESCRIPTION`] - to its text.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct MetaData {
    messages: BTreeMap<String, BTreeMap<String, Field>>,
}

/// `XElement.Value`: the text of every text node under the element, in order.
fn value(node: roxmltree::Node<'_, '_>) -> String {
    node.descendants()
        .filter(roxmltree::Node::is_text)
        .filter_map(|text| text.text())
        .collect()
}

/// `Descendants(name).FirstOrDefault()`: the first element of that name under `node`.
fn first<'a, 'input>(
    node: roxmltree::Node<'a, 'input>,
    name: &str,
) -> Option<roxmltree::Node<'a, 'input>> {
    node.descendants()
        .skip(1)
        .find(|child| child.is_element() && child.has_tag_name(name))
}

/// `Descendants(name)`: every element of that name under `node`, in document order.
fn all<'a, 'input>(
    node: roxmltree::Node<'a, 'input>,
    name: &'a str,
) -> impl Iterator<Item = roxmltree::Node<'a, 'input>> {
    node.descendants()
        .skip(1)
        .filter(move |child| child.is_element() && child.has_tag_name(name))
}

/// `(uint)element`: `XmlConvert.ToUInt32`, digits with white space either side.
fn to_u32(text: &str) -> Option<u32> {
    let digits = text.trim_matches([' ', '\t', '\n', '\r']);
    if digits.is_empty() || !digits.bytes().all(|byte| byte.is_ascii_digit()) {
        return None;
    }
    digits.parse().ok()
}

impl MetaData {
    /// `ParseMetaData`: each vehicle's `LogMessages<Vehicle>.xml` in `data_directory` that
    /// exists, read in turn into one dictionary.
    /// `// C#: ExtLibs/ArduPilot/LogMetaData.cs:100-163`
    #[must_use]
    pub fn load(data_directory: &Path) -> Self {
        let mut meta = Self::default();
        for vehicle in VEHICLES {
            if let Ok(bytes) = std::fs::read(xml_file(data_directory, vehicle)) {
                meta.parse(&String::from_utf8_lossy(&bytes));
            }
        }
        meta
    }

    /// One file's `try`: every `logformat` and its fields into the dictionary, a later file's
    /// text over an earlier's. What the C# dereferences without checking - a `logformat`
    /// without a `name` or a `description`, a field without either, a bit without a `value`
    /// `(uint)` can read - throws, and its `catch` leaves the rest of the file unread, keeping
    /// what came before; a file that is not XML is not read at all.
    /// `// C#: ExtLibs/ArduPilot/LogMetaData.cs:111-161`
    pub fn parse(&mut self, xml: &str) {
        let Ok(document) = roxmltree::Document::parse(xml) else {
            return;
        };
        let _ = self.read(document.root_element());
    }

    /// The body of the `try`; `None` where the C# throws.
    fn read(&mut self, root: roxmltree::Node<'_, '_>) -> Option<()> {
        for format in all(root, "logformat") {
            let kind = format.attribute("name")?;
            let description = first(format, "description");
            let fields = self.messages.entry(kind.to_owned()).or_default();
            fields.insert(
                DESCRIPTION.to_owned(),
                Field {
                    description: value(description?),
                    bitmask: None,
                },
            );
            for list in all(format, "fields") {
                for field in all(list, "field") {
                    let name = field.attribute("name");
                    let description = value(first(field, "description")?);
                    let bitmask = match first(field, "bitmask") {
                        None => None,
                        Some(bits) => Some(
                            all(bits, "bit")
                                .map(|bit| {
                                    Some(Bit {
                                        name: bit.attribute("name").map(ToOwned::to_owned),
                                        mask: to_u32(&value(first(bit, "value")?))?,
                                        description: first(bit, "description").map(value),
                                    })
                                })
                                .collect::<Option<Vec<Bit>>>()?,
                        ),
                    };
                    fields.insert(
                        name?.to_owned(),
                        Field {
                            description,
                            bitmask,
                        },
                    );
                }
            }
        }
        Some(())
    }

    /// `MetaData[message][field]`, when both are there.
    #[must_use]
    pub fn field(&self, message: &str, field: &str) -> Option<&Field> {
        self.messages.get(message)?.get(field)
    }
}

/// `LogMessages<Vehicle>.xml.xz`, where the download goes.
#[must_use]
pub fn xz_file(data_directory: &Path, vehicle: &str) -> PathBuf {
    data_directory.join(format!("LogMessages{vehicle}.xml.xz"))
}

/// `LogMessages<Vehicle>.xml`, what it unpacks to.
#[must_use]
pub fn xml_file(data_directory: &Path, vehicle: &str) -> PathBuf {
    data_directory.join(format!("LogMessages{vehicle}.xml"))
}

/// Whether a download is young enough to keep: its last write within [`FRESH`] of `now`.
fn fresh(path: &Path, now: std::time::SystemTime) -> bool {
    std::fs::metadata(path)
        .and_then(|meta| meta.modified())
        .is_ok_and(|written| now.duration_since(written).map_or(true, |age| age < FRESH))
}

/// `XZStream.IsXZStream`: the six bytes an xz stream starts with.
const XZ_MAGIC: [u8; 6] = [0xFD, b'7', b'z', b'X', b'Z', 0x00];

/// `GetMetaData`: each vehicle's file downloaded unless the copy on disk is fresh (all four at
/// once in the C#, one after another here), then each `.xml.xz` that is an xz stream unpacked
/// over its `.xml`. A download that fails leaves what was there; an unpacking that fails part
/// way leaves what it wrote, as the C#'s `CopyTo` into the truncated file does.
/// `// C#: ExtLibs/ArduPilot/LogMetaData.cs:41-98`
pub fn get_meta_data(data_directory: &Path, fetch: Option<&dyn mp_firmware::manifest::Fetch>) {
    if let Some(fetch) = fetch {
        let now = std::time::SystemTime::now();
        for vehicle in VEHICLES {
            let file = xz_file(data_directory, vehicle);
            if fresh(&file, now) {
                continue;
            }
            let _ =
                mp_firmware::flow::get_file_from_net(fetch, &url(vehicle), &file, &mut |_, _| {});
        }
    }
    for vehicle in VEHICLES {
        let _ = unpack(
            &xz_file(data_directory, vehicle),
            &xml_file(data_directory, vehicle),
        );
    }
}

/// One `.xml.xz` to its `.xml`, if it is an xz stream.
fn unpack(file: &Path, fileout: &Path) -> std::io::Result<()> {
    let packed = std::fs::read(file)?;
    if !packed.starts_with(&XZ_MAGIC) {
        return Ok(());
    }
    let mut out = std::io::BufWriter::new(std::fs::File::create(fileout)?);
    let unpacked = lzma_rs::xz_decompress(&mut packed.as_slice(), &mut out);
    std::io::Write::flush(&mut out)?;
    unpacked.map_err(|err| std::io::Error::other(err.to_string()))
}

/// The dictionary once `BGLogMessagesMetaData` has filled it; empty until then.
static SHARED: OnceLock<MetaData> = OnceLock::new();

/// `ThreadPool.QueueUserWorkItem(BGLogMessagesMetaData)`: the download, the unpacking and the
/// reading on a thread of their own, the dictionary shared once read.
/// `// C#: MainV2.cs:3299, 3868-3872`
pub fn start() {
    let Some(data_directory) = mp_settings::data_directory() else {
        return;
    };
    let offline = std::env::var_os("MP_OFFLINE").is_some();
    let _ = std::thread::Builder::new()
        .name("log-metadata".to_owned())
        .spawn(move || {
            let http = mp_firmware::manifest::Http;
            let fetch: Option<&dyn mp_firmware::manifest::Fetch> =
                (!offline).then_some(&http as &dyn mp_firmware::manifest::Fetch);
            get_meta_data(&data_directory, fetch);
            let _ = SHARED.set(MetaData::load(&data_directory));
        });
}

/// `LogMetaData.MetaData` as the screens read it.
#[must_use]
pub fn shared() -> Option<&'static MetaData> {
    SHARED.get()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The shape of autotest's `LogMessages.xml`, cut down.
    const SAMPLE: &str = r#"<?xml version="1.0" encoding="utf-8"?>
<!-- Dynamically generated list of documented logfile messages (generated by parse.py) -->
<loggermessagefile>
  <logformat name="ACC">
    <description>IMU accelerometer data</description>
    <fields>
      <field name="TimeUS" units="μs" type="uint64_t">
        <description>Time since system startup</description>
      </field>
      <field name="AccX" units="m/s/s" type="float">
        <description>acceleration along X axis</description>
      </field>
    </fields>
  </logformat>
  <logformat name="ARM">
    <description>Arming status changes</description>
    <fields>
      <field name="ArmChecks" type="uint32_t">
        <description>arming bitmask at time of arming</description>
        <bitmask name="AP_Arming::Check">
          <bit name="ARMING_CHECK_ALL">
            <description>all checks</description>
            <value>1</value>
          </bit>
          <bit name="ARMING_CHECK_BARO">
            <value> 2 </value>
          </bit>
        </bitmask>
      </field>
    </fields>
  </logformat>
</loggermessagefile>"#;

    fn scratch(test: &str) -> PathBuf {
        let dir =
            std::env::temp_dir().join(format!("mp-gui-logmeta-{test}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("scratch");
        dir
    }

    /// Each message's own description under "description", each field's under its name, and a
    /// bitmask field's bits.
    /// `// C#: ExtLibs/ArduPilot/LogMetaData.cs:119-156`
    #[test]
    fn the_file_is_read_as_parsemetadata_reads_it() {
        let mut meta = MetaData::default();
        meta.parse(SAMPLE);
        assert_eq!(
            meta.field("ACC", DESCRIPTION)
                .map(|f| f.description.as_str()),
            Some("IMU accelerometer data")
        );
        assert_eq!(
            meta.field("ACC", "AccX").map(|f| f.description.as_str()),
            Some("acceleration along X axis")
        );
        assert_eq!(
            meta.field("ACC", "AccX").and_then(|f| f.bitmask.as_ref()),
            None
        );
        let checks = meta.field("ARM", "ArmChecks").expect("the field");
        assert_eq!(
            checks.bitmask,
            Some(vec![
                Bit {
                    name: Some("ARMING_CHECK_ALL".to_owned()),
                    mask: 1,
                    description: Some("all checks".to_owned()),
                },
                Bit {
                    name: Some("ARMING_CHECK_BARO".to_owned()),
                    mask: 2,
                    description: None,
                },
            ])
        );
    }

    /// A field with no description throws: that file's reading stops there, and what came
    /// before it stays - the message's entry made, its own description, the fields before.
    #[test]
    fn a_missing_description_stops_the_file_there() {
        let mut meta = MetaData::default();
        meta.parse(
            "<f><logformat name=\"A\"><description>a</description><fields>\
             <field name=\"x\"><description>x</description></field>\
             <field name=\"y\"></field>\
             <field name=\"z\"><description>z</description></field>\
             </fields></logformat>\
             <logformat name=\"B\"><description>b</description></logformat></f>",
        );
        assert!(meta.field("A", "x").is_some());
        assert!(meta.field("A", "y").is_none() && meta.field("A", "z").is_none());
        assert!(meta.field("B", DESCRIPTION).is_none());
        // Not XML: nothing.
        let mut meta = MetaData::default();
        meta.parse("<f><logformat name=\"A\">");
        assert_eq!(meta, MetaData::default());
    }

    /// The four files in the C#'s order, a later one's text over an earlier's.
    #[test]
    fn a_later_vehicle_wins() {
        let dir = scratch("order");
        let one = |text: &str| {
            format!("<f><logformat name=\"GPS\"><description>{text}</description></logformat></f>")
        };
        std::fs::write(xml_file(&dir, "Copter"), one("copter")).unwrap();
        std::fs::write(xml_file(&dir, "Rover"), one("rover")).unwrap();
        let meta = MetaData::load(&dir);
        assert_eq!(
            meta.field("GPS", DESCRIPTION)
                .map(|f| f.description.as_str()),
            Some("rover")
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// Offline, a `.xml.xz` already downloaded is unpacked to its `.xml` and read; a file that
    /// is not an xz stream is left alone.
    #[test]
    fn a_download_on_disk_is_unpacked_and_read() {
        let dir = scratch("unpack");
        let mut packed = Vec::new();
        lzma_rs::xz_compress(&mut SAMPLE.as_bytes(), &mut packed).unwrap();
        assert!(packed.starts_with(&XZ_MAGIC));
        std::fs::write(xz_file(&dir, "Plane"), &packed).unwrap();
        std::fs::write(xz_file(&dir, "Rover"), b"not xz").unwrap();
        get_meta_data(&dir, None);
        assert_eq!(
            std::fs::read_to_string(xml_file(&dir, "Plane")).unwrap(),
            SAMPLE
        );
        assert!(!xml_file(&dir, "Rover").exists());
        let meta = MetaData::load(&dir);
        assert_eq!(
            meta.field("ACC", "TimeUS").map(|f| f.description.as_str()),
            Some("Time since system startup")
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// A download under seven days old is kept; an older one, or none, is fetched again.
    #[test]
    fn a_fresh_download_is_kept() {
        let dir = scratch("fresh");
        let file = xz_file(&dir, "Copter");
        let now = std::time::SystemTime::now();
        assert!(!fresh(&file, now));
        std::fs::write(&file, b"x").unwrap();
        assert!(fresh(&file, now));
        assert!(!fresh(&file, now + FRESH + Duration::from_secs(1)));
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// `(uint)` on a bit's value: digits, white space either side, nothing else.
    #[test]
    fn a_bit_value_is_xmlconvert_touint32() {
        assert_eq!(to_u32(" 4096\n"), Some(4096));
        assert_eq!(to_u32("0x10"), None);
        assert_eq!(to_u32("-1"), None);
        assert_eq!(to_u32(""), None);
    }
}
