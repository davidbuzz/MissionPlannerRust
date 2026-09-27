//! Emits Rust source from a parsed MAVLink dialect.
//!
//! Output is deterministic: identical input produces byte-identical output, so `--check` in CI is
//! a meaningful gate and diffs stay reviewable.

use std::fmt::Write as _;

use crate::codegen::mavlink::{Dialect, Field};

/// Rust keywords that cannot be used as raw identifiers.
const NON_RAW_KEYWORDS: &[&str] = &["crate", "self", "super", "Self"];

const KEYWORDS: &[&str] = &[
    "as", "break", "const", "continue", "crate", "dyn", "else", "enum", "extern", "false", "fn",
    "for", "if", "impl", "in", "let", "loop", "match", "mod", "move", "mut", "pub", "ref",
    "return", "self", "static", "struct", "super", "trait", "true", "type", "unsafe", "use",
    "where", "while", "async", "await", "box", "final", "macro", "override", "priv", "try",
    "typeof", "unsized", "virtual", "yield", "abstract", "become", "do",
];

/// SCREAMING_SNAKE or snake_case to PascalCase.
fn pascal_case(s: &str) -> String {
    s.split('_')
        .filter(|p| !p.is_empty())
        .map(|p| {
            let lower = p.to_lowercase();
            let mut c = lower.chars();
            match c.next() {
                Some(first) => first.to_uppercase().collect::<String>() + c.as_str(),
                None => String::new(),
            }
        })
        .collect()
}

/// Makes an identifier safe to use in Rust source.
fn ident(name: &str) -> String {
    let lower = name.to_lowercase();
    if NON_RAW_KEYWORDS.contains(&lower.as_str()) {
        format!("{lower}_")
    } else if KEYWORDS.contains(&lower.as_str()) {
        format!("r#{lower}")
    } else if lower.chars().next().is_some_and(|c| c.is_ascii_digit()) {
        format!("_{lower}")
    } else {
        lower
    }
}

/// Maps a MAVLink base type to its Rust scalar.
fn scalar_type(base: &str) -> &'static str {
    match base {
        "char" | "uint8_t" | "uint8_t_mavlink_version" => "u8",
        "int8_t" => "i8",
        "uint16_t" => "u16",
        "int16_t" => "i16",
        "uint32_t" => "u32",
        "int32_t" => "i32",
        "uint64_t" => "u64",
        "int64_t" => "i64",
        "float" => "f32",
        "double" => "f64",
        _ => "u8",
    }
}

fn getter_fn(base: &str) -> String {
    format!("get_{}", scalar_type(base))
}

fn rust_type(field: &Field) -> String {
    let scalar = scalar_type(&field.base_type);
    match field.array_len {
        Some(n) => format!("[{scalar}; {n}]"),
        None => scalar.to_owned(),
    }
}

/// Builds an index expression without no-op arithmetic (`0 + i * 1` is just `i`).
fn offset_expr(offset: usize, elem_size: usize) -> String {
    match (offset, elem_size) {
        (0, 1) => "i".to_owned(),
        (0, e) => format!("i * {e}"),
        (o, 1) => format!("{o} + i"),
        (o, e) => format!("{o} + i * {e}"),
    }
}

fn doc_lines(text: &str, indent: &str) -> String {
    if text.trim().is_empty() {
        return String::new();
    }
    // Keep docs to one line: the XML descriptions are long prose and the value here is
    // searchability, not typography.
    let cleaned = text
        .replace(['\r', '\n'], " ")
        .replace('[', "\\[")
        .replace(']', "\\]");
    let cleaned = cleaned.split_whitespace().collect::<Vec<_>>().join(" ");
    format!("{indent}/// {cleaned}\n")
}

/// Emits the whole dialect module.
pub fn emit_dialect(dialect: &Dialect) -> String {
    let mut out = String::with_capacity(2 * 1024 * 1024);

    out.push_str(&format!(
        "//! Generated from the MAVLink XML definitions for the `{}` dialect.\n\
         //!\n\
         //! DO NOT EDIT. Regenerate with `cargo xtask codegen mavlink`.\n\
         //!\n\
         //! Source: `references/missionplanner/ExtLibs/Mavlink/message_definitions/{}.xml`\n\
         //! Metadata (CRC_EXTRA, min_len, len) is verified against the shipping C# table by\n\
         //! `cargo xtask verify-mavlink`.\n\n\
         #![allow(clippy::unreadable_literal)]\n\
         #![allow(clippy::too_many_lines)]\n\
         #![allow(clippy::doc_markdown)]\n\
         #![allow(clippy::struct_excessive_bools)]\n\n\
         use mp_mavlink::dialect::{{MessageInfo, StaticDialect}};\n\
         use mp_mavlink::field::{{FieldInfo, FieldValue}};\n\
         use mp_mavlink::message::Message;\n\
         use mp_mavlink::payload::{{get_f32, get_f64, get_i16, get_i32, get_i64, get_i8, get_u16, get_u32, get_u64, get_u8, put_bytes}};\n\n",
        dialect.name, dialect.name
    ));

    emit_table(dialect, &mut out);
    emit_enums(dialect, &mut out);
    emit_messages(dialect, &mut out);
    emit_dispatch(dialect, &mut out);
    out
}

fn emit_table(dialect: &Dialect, out: &mut String) {
    let _ = writeln!(
        out,
        "/// Message metadata table, sorted by id so lookup is a binary search."
    );
    let _ = writeln!(out, "pub static MESSAGES: &[MessageInfo] = &[");
    for msg in dialect.messages.values() {
        let _ = writeln!(
            out,
            "    MessageInfo {{ id: {}, name: \"{}\", crc_extra: {}, min_len: {}, len: {} }},",
            msg.id,
            msg.name,
            msg.crc_extra(),
            msg.min_len(),
            msg.len()
        );
    }
    let _ = writeln!(out, "];\n");
    let _ = writeln!(out, "/// The `{}` dialect.", dialect.name);
    let _ = writeln!(
        out,
        "pub static DIALECT: StaticDialect = StaticDialect::new(\"{}\", MESSAGES);\n",
        dialect.name
    );
}

fn emit_enums(dialect: &Dialect, out: &mut String) {
    for e in dialect.enums.values() {
        let type_name = pascal_case(&e.name);
        let docs = doc_lines(&e.description, "");
        if docs.is_empty() {
            let _ = writeln!(out, "/// MAVLink enum `{}`.", e.name);
        } else {
            out.push_str(&docs);
        }
        let _ = writeln!(out, "///");
        let _ = writeln!(
            out,
            "/// MAVLink enum `{}`{}. Values are open: an unknown value from a newer autopilot",
            e.name,
            if e.bitmask { " (bitmask)" } else { "" }
        );
        let _ = writeln!(out, "/// is preserved rather than rejected.");
        let _ = writeln!(
            out,
            "#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]"
        );
        let _ = writeln!(out, "pub struct {type_name}(pub u32);\n");
        let _ = writeln!(out, "impl {type_name} {{");
        let mut emitted = std::collections::HashSet::new();
        let mut named: Vec<(u32, String)> = Vec::new();
        for entry in &e.entries {
            let Some(value) = entry.value else { continue };
            let Ok(value) = u32::try_from(value) else {
                continue;
            };
            let const_name = entry.name.to_uppercase();
            if !emitted.insert(const_name.clone()) {
                continue;
            }
            let const_name = if const_name
                .chars()
                .next()
                .is_some_and(|c| c.is_ascii_digit())
            {
                format!("_{const_name}")
            } else {
                const_name
            };
            let docs = doc_lines(&entry.description, "    ");
            if docs.is_empty() {
                let _ = writeln!(out, "    /// `{}` = {value}.", entry.name);
            } else {
                out.push_str(&docs);
            }
            let _ = writeln!(out, "    pub const {const_name}: Self = Self({value});");
            named.push((value, entry.name.clone()));
        }

        // A reverse lookup, which the UI needs far more often than the constants do. A COMMAND_ACK
        // carrying `22` is useless on screen; "MAV_CMD_NAV_TAKEOFF: denied" is the whole message.
        // Generating it keeps the names tied to the same XML the wire format comes from, so a
        // firmware that adds a command does not leave the UI printing bare numbers for it.
        //
        // `None` for an unknown value, never a guess: a newer autopilot's command shown under an
        // older name would be worse than showing the number.
        named.sort_unstable_by_key(|(value, _)| *value);
        named.dedup_by_key(|(value, _)| *value);
        let _ = writeln!(
            out,
            "\n    /// The name of a value, or `None` if this dialect does not define it."
        );
        let _ = writeln!(out, "    #[must_use]");
        let _ = writeln!(
            out,
            "    pub const fn name(self) -> Option<&'static str> {{"
        );
        if named.is_empty() {
            let _ = writeln!(out, "        None");
        } else {
            let _ = writeln!(out, "        Some(match self.0 {{");
            for (value, name) in &named {
                let _ = writeln!(out, "            {value} => \"{name}\",");
            }
            let _ = writeln!(out, "            _ => return None,");
            let _ = writeln!(out, "        }})");
        }
        let _ = writeln!(out, "    }}");

        // For MAV_CMD only: what each command's parameters mean. The definitions label and give
        // units for every one, which is the difference between a mission editor that shows
        // "param1" and one that shows "hold time, seconds". A parameter the command does not use
        // has no label in the XML and is emitted as None, so the editor can hide it rather than
        // offer a control that does nothing.
        if e.name == "MAV_CMD" {
            emit_command_params(&e.entries, out);
        }

        let _ = writeln!(out, "}}\n");
    }
}

/// A label and units for each of a command's first four parameters.
///
/// Four, not seven: 5, 6 and 7 are latitude, longitude and altitude on every command that has a
/// location, and the editor shows those as a position rather than as numbered parameters.
fn emit_command_params(entries: &[crate::codegen::mavlink::EnumEntry], out: &mut String) {
    let _ = writeln!(
        out,
        "\n    /// What this command's parameters 1 to 4 mean, as (label, units)."
    );
    let _ = writeln!(out, "    ///");
    let _ = writeln!(
        out,
        "    /// `None` for a parameter the command does not use."
    );
    let _ = writeln!(out, "    #[must_use]");
    let _ = writeln!(
        out,
        "    pub const fn parameters(self) -> [Option<(&'static str, &'static str)>; 4] {{"
    );
    let _ = writeln!(out, "        match self.0 {{");

    let mut emitted = std::collections::HashSet::new();
    for entry in entries {
        let Some(value) = entry.value else { continue };
        let Ok(value) = u32::try_from(value) else {
            continue;
        };
        if !emitted.insert(value) {
            continue;
        }
        let described: Vec<String> = (1..=4)
            .map(|index| {
                entry
                    .params
                    .iter()
                    .find(|param| param.index == index)
                    .and_then(|param| param.label.as_ref().map(|label| (param, label)))
                    .map_or_else(
                        || "None".to_owned(),
                        |(param, label)| {
                            format!(
                                "Some((\"{}\", \"{}\"))",
                                escape(label),
                                escape(param.units.as_deref().unwrap_or(""))
                            )
                        },
                    )
            })
            .collect();
        if described.iter().all(|value| value == "None") {
            continue;
        }
        let _ = writeln!(out, "            {value} => [{}],", described.join(", "));
    }

    let _ = writeln!(out, "            _ => [None, None, None, None],");
    let _ = writeln!(out, "        }}");
    let _ = writeln!(out, "    }}");
}

/// Escapes a string for a Rust literal.
fn escape(text: &str) -> String {
    text.replace('\\', "\\\\").replace('"', "\\\"")
}

fn emit_messages(dialect: &Dialect, out: &mut String) {
    for msg in dialect.messages.values() {
        let struct_name = pascal_case(&msg.name);
        let ordered = msg.ordered_fields();

        let docs = doc_lines(&msg.description, "");
        if docs.is_empty() {
            let _ = writeln!(out, "/// MAVLink message `{}`.", msg.name);
        } else {
            out.push_str(&docs);
        }
        let _ = writeln!(out, "///");
        let _ = writeln!(
            out,
            "/// MAVLink message {} (`{}`), from `{}.xml`.",
            msg.id, msg.name, msg.origin
        );
        let _ = writeln!(out, "#[derive(Debug, Clone, Copy, PartialEq)]");
        let _ = writeln!(out, "pub struct {struct_name} {{");
        // Fields are declared in wire order so the struct mirrors the packet layout.
        for field in &ordered {
            let docs = doc_lines(&field.description, "    ");
            if docs.is_empty() {
                let _ = writeln!(out, "    /// Field `{}`.", field.name);
            } else {
                out.push_str(&docs);
            }
            if let Some(en) = &field.enum_name {
                let _ = writeln!(out, "    /// Values from [`{}`].", pascal_case(en));
            }
            if field.extension {
                let _ = writeln!(out, "    /// MAVLink2 extension field: zero when absent.");
            }
            let _ = writeln!(out, "    pub {}: {},", ident(&field.name), rust_type(field));
        }
        let _ = writeln!(out, "}}\n");

        let _ = writeln!(out, "impl Message for {struct_name} {{");
        let _ = writeln!(out, "    const ID: u32 = {};", msg.id);
        let _ = writeln!(out, "    const NAME: &'static str = \"{}\";", msg.name);
        let _ = writeln!(out, "    const CRC_EXTRA: u8 = {};", msg.crc_extra());
        let _ = writeln!(out, "    const MIN_LEN: usize = {};", msg.min_len());
        let _ = writeln!(out, "    const LEN: usize = {};\n", msg.len());

        // decode
        let _ = writeln!(out, "    fn decode(payload: &[u8]) -> Self {{");
        let _ = writeln!(out, "        Self {{");
        let mut offset = 0usize;
        for field in &ordered {
            let name = ident(&field.name);
            let getter = getter_fn(&field.base_type);
            match field.array_len {
                Some(n) => {
                    let idx = offset_expr(offset, field.elem_size);
                    let _ = writeln!(
                        out,
                        "            {name}: core::array::from_fn(|i| {getter}(payload, {idx})),"
                    );
                    offset += field.elem_size * n;
                }
                None => {
                    let _ = writeln!(out, "            {name}: {getter}(payload, {offset}),");
                    offset += field.elem_size;
                }
            }
        }
        let _ = writeln!(out, "        }}");
        let _ = writeln!(out, "    }}\n");

        // encode
        let _ = writeln!(out, "    fn encode(&self, out: &mut [u8]) -> usize {{");
        let mut offset = 0usize;
        for field in &ordered {
            let name = ident(&field.name);
            match field.array_len {
                Some(n) => {
                    let idx = offset_expr(offset, field.elem_size);
                    let _ = writeln!(
                        out,
                        "        for (i, v) in self.{name}.iter().enumerate() {{ put_bytes(out, {idx}, &v.to_le_bytes()); }}"
                    );
                    offset += field.elem_size * n;
                }
                None => {
                    let _ = writeln!(
                        out,
                        "        put_bytes(out, {offset}, &self.{name}.to_le_bytes());"
                    );
                    offset += field.elem_size;
                }
            }
        }
        let _ = writeln!(out, "        Self::LEN");
        let _ = writeln!(out, "    }}");
        let _ = writeln!(out, "}}\n");

        // Named field access, used by the differential harness to compare against the C#
        // implementation field by field.
        let _ = writeln!(out, "impl {struct_name} {{");
        let _ = writeln!(out, "    /// Every field, by name, in wire order.");
        let _ = writeln!(out, "    #[must_use]");
        let _ = writeln!(
            out,
            "    pub fn fields(&self) -> Vec<(&'static str, FieldValue)> {{"
        );
        let _ = writeln!(out, "        vec![");
        for field in &ordered {
            let name = ident(&field.name);
            let scalar = scalar_type(&field.base_type);
            let variant = match scalar {
                "f32" | "f64" => "Float",
                "i8" | "i16" | "i32" | "i64" => "Signed",
                _ => "Unsigned",
            };
            // Only widen when the scalar is actually narrower than the FieldValue payload;
            // an `into()` to the same type is a lint warning in generated code.
            let already_wide = matches!(scalar, "u64" | "i64" | "f64");
            if field.array_len.is_some() {
                let mapper = if already_wide {
                    "to_vec()"
                } else {
                    "iter().map(|v| (*v).into()).collect()"
                };
                let _ = writeln!(
                    out,
                    "            (\"{}\", FieldValue::{variant}Array(self.{name}.{mapper})),",
                    field.name
                );
            } else {
                let conv = if already_wide { "" } else { ".into()" };
                let _ = writeln!(
                    out,
                    "            (\"{}\", FieldValue::{variant}(self.{name}{conv})),",
                    field.name
                );
            }
        }
        let _ = writeln!(out, "        ]");
        let _ = writeln!(out, "    }}\n");
        // Each field's XML type and units beside its value, for a screen that shows them: the
        // MAVLink Inspector's type column and Graph It's axis title.
        let _ = writeln!(
            out,
            "    /// Every field's XML type and units, in wire order, as `fields` names them."
        );
        let _ = writeln!(out, "    pub const FIELD_INFO: &'static [FieldInfo] = &[");
        for field in &ordered {
            let _ = writeln!(
                out,
                "        FieldInfo::new(\"{}\", \"{}\", {}, \"{}\"),",
                field.name,
                field.base_type,
                field.array_len.unwrap_or(0),
                escape(&field.units)
            );
        }
        let _ = writeln!(out, "    ];");
        let _ = writeln!(out, "}}\n");
    }
}

/// Emits the dynamic dispatch enum, the bridge between a decoded frame and typed messages.
fn emit_dispatch(dialect: &Dialect, out: &mut String) {
    let _ = writeln!(out, "/// Any message in this dialect.");
    let _ = writeln!(out, "#[derive(Debug, Clone, Copy, PartialEq)]");
    let _ = writeln!(out, "#[non_exhaustive]");
    let _ = writeln!(out, "pub enum MavMessage {{");
    for msg in dialect.messages.values() {
        let variant = pascal_case(&msg.name);
        let _ = writeln!(out, "    /// [`{variant}`]");
        let _ = writeln!(out, "    {variant}({variant}),");
    }
    let _ = writeln!(out, "}}\n");

    let _ = writeln!(out, "impl MavMessage {{");
    let _ = writeln!(
        out,
        "    /// Decodes a payload into a typed message, or `None` if the id is unknown here."
    );
    let _ = writeln!(out, "    #[must_use]");
    let _ = writeln!(
        out,
        "    pub fn decode(msgid: u32, payload: &[u8]) -> Option<Self> {{"
    );
    let _ = writeln!(out, "        match msgid {{");
    for msg in dialect.messages.values() {
        let variant = pascal_case(&msg.name);
        let _ = writeln!(
            out,
            "            {} => Some(Self::{variant}({variant}::decode(payload))),",
            msg.id
        );
    }
    let _ = writeln!(out, "            _ => None,");
    let _ = writeln!(out, "        }}");
    let _ = writeln!(out, "    }}\n");

    let _ = writeln!(out, "    /// The message id.");
    let _ = writeln!(out, "    #[must_use]");
    let _ = writeln!(out, "    pub const fn id(&self) -> u32 {{");
    let _ = writeln!(out, "        match self {{");
    for msg in dialect.messages.values() {
        let variant = pascal_case(&msg.name);
        let _ = writeln!(out, "            Self::{variant}(_) => {},", msg.id);
    }
    let _ = writeln!(out, "        }}");
    let _ = writeln!(out, "    }}\n");

    let _ = writeln!(
        out,
        "    /// Encodes the payload into `out`, returning bytes written."
    );
    let _ = writeln!(out, "    pub fn encode(&self, out: &mut [u8]) -> usize {{");
    let _ = writeln!(out, "        match self {{");
    for msg in dialect.messages.values() {
        let variant = pascal_case(&msg.name);
        let _ = writeln!(out, "            Self::{variant}(m) => m.encode(out),");
    }
    let _ = writeln!(out, "        }}");
    let _ = writeln!(out, "    }}\n");

    let _ = writeln!(
        out,
        "    /// Full payload length including extension fields."
    );
    let _ = writeln!(out, "    #[must_use]");
    let _ = writeln!(out, "    pub const fn len(&self) -> usize {{");
    let _ = writeln!(out, "        match self {{");
    for msg in dialect.messages.values() {
        let variant = pascal_case(&msg.name);
        let _ = writeln!(out, "            Self::{variant}(_) => {},", msg.len());
    }
    let _ = writeln!(out, "        }}");
    let _ = writeln!(out, "    }}\n");

    let _ = writeln!(out, "    /// Whether the message has an empty payload.");
    let _ = writeln!(out, "    #[must_use]");
    let _ = writeln!(
        out,
        "    pub const fn is_empty(&self) -> bool {{ self.len() == 0 }}\n"
    );

    let _ = writeln!(out, "    /// The CRC_EXTRA seed for this message.");
    let _ = writeln!(out, "    #[must_use]");
    let _ = writeln!(out, "    pub const fn crc_extra(&self) -> u8 {{");
    let _ = writeln!(out, "        match self {{");
    for msg in dialect.messages.values() {
        let variant = pascal_case(&msg.name);
        let _ = writeln!(
            out,
            "            Self::{variant}(_) => {},",
            msg.crc_extra()
        );
    }
    let _ = writeln!(out, "        }}");
    let _ = writeln!(out, "    }}\n");

    let _ = writeln!(
        out,
        "    /// Every field of the contained message, by name."
    );
    let _ = writeln!(out, "    #[must_use]");
    let _ = writeln!(
        out,
        "    pub fn fields(&self) -> Vec<(&'static str, FieldValue)> {{"
    );
    let _ = writeln!(out, "        match self {{");
    for msg in dialect.messages.values() {
        let variant = pascal_case(&msg.name);
        let _ = writeln!(out, "            Self::{variant}(m) => m.fields(),");
    }
    let _ = writeln!(out, "        }}");
    let _ = writeln!(out, "    }}\n");

    let _ = writeln!(
        out,
        "    /// Every field's XML type and units, in the order `fields` lists them."
    );
    let _ = writeln!(out, "    #[must_use]");
    let _ = writeln!(
        out,
        "    pub const fn field_info(&self) -> &'static [FieldInfo] {{"
    );
    let _ = writeln!(out, "        match self {{");
    for msg in dialect.messages.values() {
        let variant = pascal_case(&msg.name);
        let _ = writeln!(
            out,
            "            Self::{variant}(_) => {variant}::FIELD_INFO,"
        );
    }
    let _ = writeln!(out, "        }}");
    let _ = writeln!(out, "    }}\n");

    let _ = writeln!(out, "    /// The message name.");
    let _ = writeln!(out, "    #[must_use]");
    let _ = writeln!(out, "    pub const fn name(&self) -> &'static str {{");
    let _ = writeln!(out, "        match self {{");
    for msg in dialect.messages.values() {
        let variant = pascal_case(&msg.name);
        let _ = writeln!(out, "            Self::{variant}(_) => \"{}\",", msg.name);
    }
    let _ = writeln!(out, "        }}");
    let _ = writeln!(out, "    }}");
    let _ = writeln!(out, "}}");
}
