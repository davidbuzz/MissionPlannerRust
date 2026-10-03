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

//! The two .NET XML writers the output files come from, reduced to what those files use.
//!
//! [`TextWriter`] is `System.Xml.XmlTextWriter`, which writes the `.jxl`, the `.gpx` and (through
//! `XmlSerializer`) the positions beside the log. [`IndentWriter`] is what `XmlWriter.Create`
//! with `Indent = true` gives, which SharpKml serializes the `.kml` with. Their indentation differs
//! in one way that shows in the files: both stop indenting inside an element once text (or, for
//! `XmlTextWriter`, raw text) has been written into it - "mixed content" - and the children of
//! such an element inherit that, which is why everything after the `.jxl`'s `WriteRaw` block runs
//! on one line.

use crate::numfmt::NEWLINE;

/// Escapes text content: `&`, `<`, `>`.
fn escape_text(text: &str, out: &mut String) {
    for c in text.chars() {
        match c {
            '&' => out.push_str("&amp;"),
            '<' => out.push_str("&lt;"),
            '>' => out.push_str("&gt;"),
            _ => out.push(c),
        }
    }
}

/// Escapes an attribute value: `&`, `<`, `>`, `"`.
fn escape_attribute(text: &str, out: &mut String) {
    for c in text.chars() {
        match c {
            '"' => out.push_str("&quot;"),
            '&' => out.push_str("&amp;"),
            '<' => out.push_str("&lt;"),
            '>' => out.push_str("&gt;"),
            _ => out.push(c),
        }
    }
}

#[derive(Debug, Clone)]
struct Frame {
    name: String,
    mixed: bool,
}

/// `XmlTextWriter`, with `Formatting.Indented` and two spaces, or with no formatting.
#[derive(Debug, Clone)]
pub struct TextWriter {
    out: String,
    stack: Vec<Frame>,
    indented: bool,
    /// Nothing has been written yet (`State.Start`).
    start: bool,
    /// A start tag is still open for attributes.
    open: bool,
}

impl TextWriter {
    /// A writer; `indented` is `Formatting.Indented`.
    #[must_use]
    pub const fn new(indented: bool) -> Self {
        Self {
            out: String::new(),
            stack: Vec::new(),
            indented,
            start: true,
            open: false,
        }
    }

    fn close_start_tag(&mut self) {
        if self.open {
            self.out.push('>');
            self.open = false;
        }
    }

    /// `Indent(beforeEndElement)`: a new line, and two spaces a level - none at all inside mixed
    /// content. `// C#: referencesource System.Xml/Core/XmlTextWriter.cs, Indent`
    fn indent(&mut self, before_end: bool) {
        match self.stack.last() {
            None => self.out.push_str(NEWLINE),
            Some(top) if !top.mixed => {
                self.out.push_str(NEWLINE);
                let levels = if before_end {
                    self.stack.len() - 1
                } else {
                    self.stack.len()
                };
                for _ in 0..levels * 2 {
                    self.out.push(' ');
                }
            }
            Some(_) => {}
        }
    }

    /// `WriteStartDocument(standalone)` with the encoding the writer was made with, or the bare
    /// declaration `XmlSerializer` writes when it was made with none.
    pub fn declaration(&mut self, text: &str) {
        self.out.push_str(text);
        self.start = false;
    }

    /// `WriteStartElement(name)`.
    pub fn start_element(&mut self, name: &str) {
        self.close_start_tag();
        if self.indented && !self.start {
            self.indent(false);
        }
        self.start = false;
        let mixed = self.stack.last().is_some_and(|f| f.mixed);
        self.stack.push(Frame {
            name: name.to_owned(),
            mixed,
        });
        self.out.push('<');
        self.out.push_str(name);
        self.open = true;
    }

    /// `WriteAttributeString(name, value)`.
    pub fn attribute(&mut self, name: &str, value: &str) {
        self.out.push(' ');
        self.out.push_str(name);
        self.out.push_str("=\"");
        escape_attribute(value, &mut self.out);
        self.out.push('"');
    }

    /// `WriteString(text)`: nothing at all for an empty string.
    pub fn string(&mut self, text: &str) {
        if text.is_empty() {
            return;
        }
        self.close_start_tag();
        if let Some(top) = self.stack.last_mut() {
            top.mixed = true;
        }
        escape_text(text, &mut self.out);
    }

    /// `WriteRaw(text)`: as it is, and the element's content is mixed from then on.
    pub fn raw(&mut self, text: &str) {
        self.close_start_tag();
        if let Some(top) = self.stack.last_mut() {
            top.mixed = true;
        }
        self.out.push_str(text);
    }

    /// `WriteEndElement()`: ` />` for an element with no content.
    pub fn end_element(&mut self) {
        if self.open {
            self.out.push_str(" />");
            self.open = false;
        } else {
            if self.indented {
                self.indent(true);
            }
            if let Some(top) = self.stack.last() {
                self.out.push_str("</");
                self.out.push_str(&top.name);
                self.out.push('>');
            }
        }
        self.stack.pop();
    }

    /// `WriteElementString(name, value)`.
    pub fn element_string(&mut self, name: &str, value: &str) {
        self.start_element(name);
        self.string(value);
        self.end_element();
    }

    /// `WriteEndDocument()`: every open element closed.
    pub fn end_document(&mut self) {
        while !self.stack.is_empty() {
            self.end_element();
        }
    }

    /// What has been written.
    #[must_use]
    pub fn finish(mut self) -> String {
        self.end_document();
        self.out
    }
}

/// `XmlWriter.Create(StringBuilder, new XmlWriterSettings { Indent = true })`: an
/// `XmlEncodedRawTextWriterIndent` behind an `XmlWellFormedWriter`.
#[derive(Debug, Clone)]
pub struct IndentWriter {
    out: String,
    names: Vec<String>,
    /// `mixedContentStack`, and `mixedContent` on top.
    mixed_stack: Vec<bool>,
    mixed: bool,
    level: usize,
    open: bool,
    /// Whether the element on top has content yet (`contentPos != bufPos`).
    content: Vec<bool>,
    /// Whether text was the last thing written (`textPos == bufPos`).
    after_text: bool,
}

impl Default for IndentWriter {
    fn default() -> Self {
        Self::new()
    }
}

impl IndentWriter {
    /// A writer into a `StringBuilder`, whose declaration therefore says UTF-16.
    #[must_use]
    pub fn new() -> Self {
        Self {
            out: "<?xml version=\"1.0\" encoding=\"utf-16\"?>".to_owned(),
            names: Vec::new(),
            mixed_stack: Vec::new(),
            mixed: false,
            level: 0,
            open: false,
            content: Vec::new(),
            after_text: false,
        }
    }

    fn close_start_tag(&mut self) {
        if self.open {
            self.out.push('>');
            self.open = false;
        }
    }

    fn write_indent(&mut self) {
        self.out.push_str(NEWLINE);
        for _ in 0..self.level {
            self.out.push_str("  ");
        }
    }

    /// `WriteStartElement(name)`, with `xmlns` when the namespace is new.
    pub fn start_element(&mut self, name: &str, xmlns: Option<&str>) {
        self.close_start_tag();
        if let Some(c) = self.content.last_mut() {
            *c = true;
        }
        if !self.mixed && !self.after_text {
            self.write_indent();
        }
        self.level += 1;
        self.mixed_stack.push(self.mixed);
        self.names.push(name.to_owned());
        self.content.push(false);
        self.out.push('<');
        self.out.push_str(name);
        if let Some(ns) = xmlns {
            self.out.push_str(" xmlns=\"");
            escape_attribute(ns, &mut self.out);
            self.out.push('"');
        }
        self.open = true;
        self.after_text = false;
    }

    /// `WriteString(text)`; text's new lines become [`NEWLINE`] (`NewLineHandling.Replace`).
    pub fn string(&mut self, text: &str) {
        self.mixed = true;
        if text.is_empty() {
            return;
        }
        self.close_start_tag();
        if let Some(c) = self.content.last_mut() {
            *c = true;
        }
        let normalised = text.replace("\r\n", "\n").replace('\r', "\n");
        let mut escaped = String::new();
        escape_text(&normalised, &mut escaped);
        self.out.push_str(&escaped.replace('\n', NEWLINE));
        self.after_text = true;
    }

    /// `WriteCData(text)`.
    pub fn cdata(&mut self, text: &str) {
        self.mixed = true;
        self.close_start_tag();
        if let Some(c) = self.content.last_mut() {
            *c = true;
        }
        self.out.push_str("<![CDATA[");
        let normalised = text.replace("\r\n", "\n").replace('\r', "\n");
        self.out.push_str(
            &normalised
                .replace("]]>", "]]]]><![CDATA[>")
                .replace('\n', NEWLINE),
        );
        self.out.push_str("]]>");
        self.after_text = true;
    }

    /// `WriteEndElement()`.
    pub fn end_element(&mut self) {
        self.level = self.level.saturating_sub(1);
        let has_content = self.content.pop().unwrap_or(false);
        if self.open {
            self.out.push_str(" />");
            self.open = false;
        } else {
            if !self.mixed && has_content && !self.after_text {
                self.write_indent();
            }
            if let Some(name) = self.names.last() {
                self.out.push_str("</");
                self.out.push_str(name);
                self.out.push('>');
            }
        }
        self.names.pop();
        self.mixed = self.mixed_stack.pop().unwrap_or(false);
        self.after_text = false;
    }

    /// What has been written, every element closed.
    #[must_use]
    pub fn finish(mut self) -> String {
        while !self.names.is_empty() {
            self.end_element();
        }
        self.out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn raw_text_ends_indentation_below_it() {
        let mut w = TextWriter::new(true);
        w.declaration("<?xml version=\"1.0\"?>");
        w.start_element("A");
        w.start_element("B");
        w.element_string("C", "");
        w.end_element();
        w.start_element("D");
        w.raw(" raw ");
        w.start_element("E");
        w.element_string("F", "x&y");
        w.end_element();
        w.end_element();
        let text = w.finish();
        assert_eq!(
            text,
            format!(
                "<?xml version=\"1.0\"?>{n}<A>{n}  <B>{n}    <C />{n}  </B>{n}  <D> raw <E><F>x&amp;y</F></E></D>{n}</A>",
                n = NEWLINE
            )
        );
    }

    #[test]
    fn text_leaves_stay_on_their_line() {
        let mut w = IndentWriter::new();
        w.start_element("Document", Some("ns"));
        w.start_element("name", None);
        w.string("a\nb");
        w.end_element();
        w.start_element("empty", None);
        w.end_element();
        let text = w.finish();
        assert_eq!(
            text,
            format!(
                "<?xml version=\"1.0\" encoding=\"utf-16\"?>{n}<Document xmlns=\"ns\">{n}  <name>a{n}b</name>{n}  <empty />{n}</Document>",
                n = NEWLINE
            )
        );
    }
}
