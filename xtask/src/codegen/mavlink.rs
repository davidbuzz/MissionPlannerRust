//! Generates Rust from the MAVLink XML message definitions.
//!
//! Source of truth: `references/missionplanner/ExtLibs/Mavlink/message_definitions/*.xml`, the
//! same files the C# build generates from. We reimplement mavgen's layout rules rather than
//! shelling out to Python so the build has no external toolchain dependency.
//!
//! The rules that matter, and that are easy to get subtly wrong:
//!
//! * **Wire order** is fields sorted by type size, descending, stably. Declaration order is not
//!   wire order.
//! * **Extension fields** (those after `<extensions/>`) are *not* sorted, do not contribute to
//!   `min_len`, and are excluded from the CRC seed.
//! * **`CRC_EXTRA`** is a CRC over the message name and each non-extension field's type and name
//!   in wire order, folded to a single byte.
//!
//! Every one of those rules is verified against the shipping C# table in
//! `testdata/mavlink/binary_message_infos.csv`, so a mistake here fails a test rather than
//! silently producing a dialect that drops packets.

use std::collections::{BTreeMap, HashSet};
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};

/// One field of a message.
#[derive(Debug, Clone)]
pub struct Field {
    /// Field name as written in the XML.
    pub name: String,
    /// Base type name without array suffix, e.g. `uint8_t`.
    pub base_type: String,
    /// Type string as it appears in the CRC computation.
    pub crc_type: String,
    /// Array length, or `None` for scalars.
    pub array_len: Option<usize>,
    /// Size of one element in bytes.
    pub elem_size: usize,
    /// Whether the field appeared after `<extensions/>`.
    pub extension: bool,
    /// Enum this field refers to, if any.
    pub enum_name: Option<String>,
    /// The `units` attribute, empty where there is none.
    pub units: String,
    /// Documentation text.
    pub description: String,
}

impl Field {
    /// Total wire size of this field.
    #[must_use]
    pub fn wire_size(&self) -> usize {
        self.elem_size * self.array_len.unwrap_or(1)
    }
}

/// One message definition.
#[derive(Debug, Clone)]
pub struct Message {
    /// Message id.
    pub id: u32,
    /// Message name, e.g. `HEARTBEAT`.
    pub name: String,
    /// Fields in declaration order.
    pub fields: Vec<Field>,
    /// Documentation text.
    pub description: String,
    /// Dialect file this message came from.
    pub origin: String,
}

impl Message {
    /// Fields in wire order: non-extension fields sorted by size descending (stable), then
    /// extension fields in declaration order.
    #[must_use]
    pub fn ordered_fields(&self) -> Vec<&Field> {
        let mut base: Vec<&Field> = self.fields.iter().filter(|f| !f.extension).collect();
        // Stable sort by descending element size - mavgen's layout rule.
        base.sort_by_key(|f| std::cmp::Reverse(f.elem_size));
        base.extend(self.fields.iter().filter(|f| f.extension));
        base
    }

    /// Payload length excluding extension fields.
    #[must_use]
    pub fn min_len(&self) -> usize {
        self.fields
            .iter()
            .filter(|f| !f.extension)
            .map(Field::wire_size)
            .sum()
    }

    /// Full payload length including extension fields.
    #[must_use]
    pub fn len(&self) -> usize {
        self.fields.iter().map(Field::wire_size).sum()
    }

    /// Whether the message carries no fields at all.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.fields.is_empty()
    }

    /// Computes `CRC_EXTRA` the way mavgen does.
    #[must_use]
    pub fn crc_extra(&self) -> u8 {
        let mut crc = crc_accumulate_str(&format!("{} ", self.name), 0xFFFF);
        for field in self.ordered_fields() {
            if field.extension {
                continue;
            }
            crc = crc_accumulate_str(&format!("{} ", field.crc_type), crc);
            crc = crc_accumulate_str(&format!("{} ", field.name), crc);
            if let Some(len) = field.array_len {
                crc = crc_accumulate(u8::try_from(len).unwrap_or(0), crc);
            }
        }
        // Fold the 16-bit CRC into the single seed byte.
        u8::try_from((crc & 0xFF) ^ (crc >> 8)).unwrap_or(0)
    }
}

/// One enum entry.
#[derive(Debug, Clone)]
pub struct EnumEntry {
    /// Entry name.
    pub name: String,
    /// Entry value, when the XML specifies one.
    pub value: Option<i64>,
    /// Documentation text.
    pub description: String,
    /// For `MAV_CMD` entries, what each of the seven parameters means.
    ///
    /// The definitions label and give units for every command parameter, which is the difference
    /// between a mission editor that shows "param1" and one that shows "hold time, seconds". This
    /// is the only place that knowledge exists, and it changes with the definitions, so it is
    /// generated rather than typed out.
    pub params: Vec<CommandParam>,
}

/// One parameter of a `MAV_CMD`.
#[derive(Debug, Clone)]
pub struct CommandParam {
    /// Which parameter, 1 to 7.
    pub index: u8,
    /// Short label, where the definitions give one.
    pub label: Option<String>,
    /// Units, where the definitions give them.
    pub units: Option<String>,
}

/// One enum definition.
#[derive(Debug, Clone)]
pub struct Enum {
    /// Enum name.
    pub name: String,
    /// Entries in declaration order.
    pub entries: Vec<EnumEntry>,
    /// Documentation text.
    pub description: String,
    /// Whether the enum is a bitmask.
    pub bitmask: bool,
}

/// A parsed dialect, with includes resolved.
#[derive(Debug, Default)]
pub struct Dialect {
    /// Dialect name, e.g. `ardupilotmega`.
    pub name: String,
    /// Messages by id.
    pub messages: BTreeMap<u32, Message>,
    /// Enums by name.
    pub enums: BTreeMap<String, Enum>,
}

#[allow(clippy::cast_possible_truncation)] // the u8 truncation is the algorithm
fn crc_accumulate(byte: u8, crc: u16) -> u16 {
    let ch = byte ^ (crc as u8);
    let ch = ch ^ (ch << 4);
    let ch = u16::from(ch);
    (crc >> 8) ^ (ch << 8) ^ (ch << 3) ^ (ch >> 4)
}

fn crc_accumulate_str(s: &str, mut crc: u16) -> u16 {
    for b in s.as_bytes() {
        crc = crc_accumulate(*b, crc);
    }
    crc
}

/// Element size for a MAVLink base type.
fn elem_size(base: &str) -> Result<usize> {
    Ok(match base {
        "char" | "uint8_t" | "int8_t" | "uint8_t_mavlink_version" => 1,
        "uint16_t" | "int16_t" => 2,
        "uint32_t" | "int32_t" | "float" => 4,
        "uint64_t" | "int64_t" | "double" => 8,
        other => bail!("unknown MAVLink field type: {other}"),
    })
}

/// Splits `uint8_t[4]` into (`uint8_t`, Some(4)).
fn split_array(ty: &str) -> (String, Option<usize>) {
    if let Some((base, rest)) = ty.split_once('[') {
        let len = rest.trim_end_matches(']').parse().ok();
        (base.to_owned(), len)
    } else {
        (ty.to_owned(), None)
    }
}

/// Parses a dialect and everything it includes.
pub fn parse_dialect(path: &Path) -> Result<Dialect> {
    let name = path
        .file_stem()
        .and_then(|s| s.to_str())
        .context("dialect file has no name")?
        .to_owned();
    let mut dialect = Dialect {
        name,
        ..Dialect::default()
    };
    let mut seen = HashSet::new();
    parse_into(path, &mut dialect, &mut seen)?;
    Ok(dialect)
}

fn parse_into(path: &Path, out: &mut Dialect, seen: &mut HashSet<PathBuf>) -> Result<()> {
    let canonical = path
        .canonicalize()
        .with_context(|| format!("resolving {}", path.display()))?;
    if !seen.insert(canonical.clone()) {
        return Ok(()); // already included
    }
    let text = std::fs::read_to_string(&canonical)
        .with_context(|| format!("reading {}", canonical.display()))?;
    let doc = roxmltree::Document::parse(&text)
        .with_context(|| format!("parsing {}", canonical.display()))?;
    let root = doc.root_element();
    let dir = canonical
        .parent()
        .context("dialect has no parent directory")?
        .to_owned();
    let origin = canonical
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("unknown")
        .to_owned();

    // Includes first, so that a dialect's own definitions override what it includes.
    for node in root.children().filter(|n| n.has_tag_name("include")) {
        if let Some(rel) = node.text().map(str::trim) {
            let included = dir.join(rel);
            if included.exists() {
                parse_into(&included, out, seen)?;
            }
        }
    }

    for node in root.children().filter(|n| n.has_tag_name("enums")) {
        for e in node.children().filter(|n| n.has_tag_name("enum")) {
            let Some(name) = e.attribute("name") else {
                continue;
            };
            let mut entries = Vec::new();
            for entry in e.children().filter(|n| n.has_tag_name("entry")) {
                let Some(entry_name) = entry.attribute("name") else {
                    continue;
                };
                let mut params = Vec::new();
                for param in entry.children().filter(|n| n.has_tag_name("param")) {
                    let Some(index) = param.attribute("index").and_then(|i| i.parse().ok()) else {
                        continue;
                    };
                    // A parameter with no label is one the command does not use - the definitions
                    // write "Empty" in the body and leave the attribute off. Recording it as
                    // unlabelled lets the editor hide it rather than offer a control that does
                    // nothing.
                    params.push(CommandParam {
                        index,
                        label: param.attribute("label").map(ToOwned::to_owned),
                        units: param.attribute("units").map(ToOwned::to_owned),
                    });
                }
                entries.push(EnumEntry {
                    name: entry_name.to_owned(),
                    value: entry.attribute("value").and_then(parse_enum_value),
                    description: child_text(entry, "description"),
                    params,
                });
            }
            let enum_def = Enum {
                name: name.to_owned(),
                entries,
                description: child_text(e, "description"),
                bitmask: e.attribute("bitmask") == Some("true"),
            };
            // Later dialects extend earlier enums rather than replacing them.
            out.enums
                .entry(name.to_owned())
                .and_modify(|existing| existing.entries.extend(enum_def.entries.clone()))
                .or_insert(enum_def);
        }
    }

    for node in root.children().filter(|n| n.has_tag_name("messages")) {
        for m in node.children().filter(|n| n.has_tag_name("message")) {
            let (Some(id), Some(name)) = (m.attribute("id"), m.attribute("name")) else {
                continue;
            };
            let id: u32 = id.parse().with_context(|| format!("message id {id}"))?;

            let mut fields = Vec::new();
            let mut in_extensions = false;
            for child in m.children().filter(roxmltree::Node::is_element) {
                if child.has_tag_name("extensions") {
                    in_extensions = true;
                    continue;
                }
                if !child.has_tag_name("field") {
                    continue;
                }
                let (Some(ty), Some(fname)) = (child.attribute("type"), child.attribute("name"))
                else {
                    continue;
                };
                let (base_type, array_len) = split_array(ty);
                // mavgen normalises the pseudo-type `uint8_t_mavlink_version` to `uint8_t`
                // before seeding the CRC. HEARTBEAT is the only message that exercises this,
                // and getting it wrong means every heartbeat on the link is discarded.
                let crc_type = if base_type == "uint8_t_mavlink_version" {
                    "uint8_t".to_owned()
                } else {
                    base_type.clone()
                };
                fields.push(Field {
                    name: fname.to_owned(),
                    crc_type,
                    elem_size: elem_size(&base_type).with_context(|| format!("{name}.{fname}"))?,
                    base_type,
                    array_len,
                    extension: in_extensions,
                    enum_name: child.attribute("enum").map(ToOwned::to_owned),
                    units: child.attribute("units").unwrap_or("").to_owned(),
                    description: child.text().unwrap_or("").trim().to_owned(),
                });
            }

            out.messages.insert(
                id,
                Message {
                    id,
                    name: name.to_owned(),
                    fields,
                    description: child_text(m, "description"),
                    origin: origin.clone(),
                },
            );
        }
    }

    Ok(())
}

fn child_text(node: roxmltree::Node<'_, '_>, tag: &str) -> String {
    node.children()
        .find(|n| n.has_tag_name(tag))
        .and_then(|n| n.text())
        .unwrap_or("")
        .split_whitespace()
        .collect::<Vec<_>>()
        .join(" ")
}

fn parse_enum_value(raw: &str) -> Option<i64> {
    let raw = raw.trim();
    if let Some(hex) = raw.strip_prefix("0x").or_else(|| raw.strip_prefix("0X")) {
        i64::from_str_radix(hex, 16).ok()
    } else if let Some(shift) = raw.strip_prefix("2**") {
        shift.parse::<u32>().ok().map(|s| 1i64 << s)
    } else {
        raw.parse().ok()
    }
}
