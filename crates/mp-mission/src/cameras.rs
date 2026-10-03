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

//! The Survey (Grid) dialog's camera list: `Grid/camerainfo.cs` and `GridUI.xmlcamera`.
//!
//! `GridUI`'s constructor reads two files into one dictionary, name to camera: the list Mission
//! Planner ships beside its executable, `camerasBuiltin.xml`, and then the user's own
//! `cameras.xml` in the user data directory, which can add cameras and replace built-in ones by
//! name (`GridUI.cs:122-124`). `CMB_camera` lists the names in the dictionary's order after each
//! file (`GridUI.cs:568-573`). The shipped file is copied into this crate as
//! `assets/camerasBuiltin.xml`; a test holds the copy to the reference tree's.
//!
//! The reader is `xmlcamera`'s `XmlTextReader` loop, node by node, because its quirks decide what
//! a damaged file yields: a `Camera` is added when its end tag is reached, a value `float.Parse`
//! refuses abandons the camera, and the reader then picks up mid-element rather than at the next
//! camera.
//!
//! The writer is `xmlcamera`'s other half, `XmlTextWriter` in ASCII, indented: when `cameras.xml`
//! does not exist the constructor's `xmlcamera(false, ...)` writes it - every camera the
//! dictionary holds, the shipped ones - instead of reading it, and `BUT_save_Click` writes it
//! again with the camera being edited (`GridUI.cs:458-498, 1567-1602`). The user data directory
//! is this application's own, `mp_settings::user_data_directory()`.

use std::path::Path;

use crate::dotnet::{general_f32, parse_f32};

/// `Environment.NewLine`, which the indented `XmlTextWriter` writes between elements: CR LF
/// under .NET on Windows, LF under mono elsewhere - the file is what Mission Planner writes on
/// the platform this runs on.
pub const NEW_LINE: &str = if cfg!(windows) { "\r\n" } else { "\n" };

/// The user's file, in the user data directory.
pub const USER_FILE: &str = "cameras.xml";

/// The camera list Mission Planner ships, `camerasBuiltin.xml`.
pub const BUILTIN_XML: &str = include_str!("../assets/camerasBuiltin.xml");

/// `camerainfo`, `Grid/camerainfo.cs`: a sensor and lens, as `float`s.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct CameraInfo {
    /// `name`.
    pub name: String,
    /// `focallen`, millimetres.
    pub focallen: f32,
    /// `sensorwidth`, millimetres.
    pub sensorwidth: f32,
    /// `sensorheight`, millimetres.
    pub sensorheight: f32,
    /// `imagewidth`, pixels.
    pub imagewidth: f32,
    /// `imageheight`, pixels.
    pub imageheight: f32,
}

/// `GridUI.cameras` and `CMB_camera.Items`.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Cameras {
    /// The dictionary, in insertion order: a name added again keeps its place.
    cameras: Vec<CameraInfo>,
    /// `CMB_camera.Items`.
    items: Vec<String>,
}

impl Cameras {
    /// The shipped list: `xmlcamera(false, GetRunningDirectory() + "camerasBuiltin.xml")`.
    #[must_use]
    pub fn builtin() -> Self {
        let mut cameras = Self::default();
        cameras.read(BUILTIN_XML);
        cameras
    }

    /// The constructor's two `xmlcamera(false, ...)` calls: the shipped list, then the user's
    /// `cameras.xml` from the user data directory - read when it exists, and when it does not,
    /// written with every camera held so far (`if (write || !exists)`). The error is the write's,
    /// whose `catch` is a message box of the exception; the cameras are loaded either way.
    /// `// C#: Grid/GridUI.cs:122-124, 460-498`
    pub fn load(user_data_directory: Option<&Path>) -> (Self, Option<std::io::Error>) {
        let mut cameras = Self::builtin();
        let Some(path) = user_data_directory.map(|directory| directory.join(USER_FILE)) else {
            return (cameras, None);
        };
        if !path.exists() {
            let written = cameras.write(&path).err();
            return (cameras, written);
        }
        // The reader's outer `catch` ("Bad Camera File") swallows a file it cannot open.
        if let Ok(bytes) = std::fs::read(&path) {
            cameras.read(&String::from_utf8_lossy(&bytes));
        }
        (cameras, None)
    }

    /// `xmlcamera(true, filename)`: [`Self::to_xml`] to `path`, replacing what is there. The
    /// directory is made first: the C#'s always exists, Mission Planner making it at start-up.
    /// `// C#: Grid/GridUI.cs:462-497`
    ///
    /// # Errors
    /// The directory or the file cannot be written: the `catch`'s exception.
    pub fn write(&self, path: &Path) -> std::io::Result<()> {
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        std::fs::write(path, self.to_xml())
    }

    /// What `xmlcamera(true, ...)`'s `XmlTextWriter` puts in the file, byte for byte: ASCII,
    /// indented two spaces, [`NEW_LINE`] between elements and none at the end; every camera but
    /// one named "" as `name`, `flen`, `imgh`, `imgw`, `senh`, `senw`, the numbers
    /// `float.ToString` in en-US - seven significant digits. Held to `XmlTextWriter` run under
    /// mono.
    /// `// C#: Grid/GridUI.cs:466-494`
    #[must_use]
    pub fn to_xml(&self) -> Vec<u8> {
        const NL: &str = NEW_LINE;
        let mut out = String::from("<?xml version=\"1.0\" encoding=\"us-ascii\"?>");
        out.push_str(NL);
        let cameras: Vec<&CameraInfo> = self
            .cameras
            .iter()
            .filter(|camera| !camera.name.is_empty())
            .collect();
        if cameras.is_empty() {
            // WriteEndElement on an element with nothing in it closes it short.
            out.push_str("<Cameras />");
            return out.into_bytes();
        }
        out.push_str("<Cameras>");
        for camera in cameras {
            out.push_str(NL);
            out.push_str("  <Camera>");
            let fields = [
                ("name", xml_text(&camera.name)),
                ("flen", general_f32(camera.focallen)),
                ("imgh", general_f32(camera.imageheight)),
                ("imgw", general_f32(camera.imagewidth)),
                ("senh", general_f32(camera.sensorheight)),
                ("senw", general_f32(camera.sensorwidth)),
            ];
            for (tag, value) in fields {
                out.push_str(NL);
                out.push_str("    ");
                out.push_str(&format!("<{tag}>{value}</{tag}>"));
            }
            out.push_str(NL);
            out.push_str("  </Camera>");
        }
        out.push_str(NL);
        out.push_str("</Cameras>");
        out.into_bytes()
    }

    /// `cameras[name]` for `BUT_save_Click`: the camera of that name, added with nothing set
    /// when there is none (`cameras.Add(CMB_camera.Text, camera)`), at the end of the
    /// dictionary. `CMB_camera`'s list is not changed: the C# adds to it only when it reads a
    /// file.
    /// Always `Some`.
    /// `// C#: Grid/GridUI.cs:1578-1586`
    pub fn entry(&mut self, name: &str) -> Option<&mut CameraInfo> {
        let at = if let Some(at) = self.cameras.iter().position(|held| held.name == name) {
            at
        } else {
            self.cameras.push(CameraInfo::default());
            self.cameras.len() - 1
        };
        self.cameras.get_mut(at)
    }

    /// The names `CMB_camera` lists, in order.
    #[must_use]
    pub fn items(&self) -> &[String] {
        &self.items
    }

    /// The camera a name picks, as `cameras.ContainsKey(CMB_camera.Text)` finds it.
    #[must_use]
    pub fn get(&self, name: &str) -> Option<&CameraInfo> {
        self.cameras.iter().find(|camera| camera.name == name)
    }

    /// `xmlcamera(false, filename)`: the file's cameras into the dictionary, then any new name onto
    /// `CMB_camera`. A file that is not well formed is read up to where it breaks, as the C#'s
    /// outer `catch` ("Bad Camera File") keeps what was read before.
    /// `// C#: Grid/GridUI.cs:512-583`
    pub fn read(&mut self, xml: &str) {
        let nodes = tokenize(xml);
        let mut reader = Reader {
            nodes: &nodes,
            at: None,
        };
        // while (xmlreader.Read())
        while reader.read() {
            // try { switch (xmlreader.Name) ... } catch { } - silent fail on bad entry
            if reader.name() != "Camera" {
                continue;
            }
            let mut camera = CameraInfo::default();
            let mut named: Option<String> = None;
            let mut failed = false;
            while reader.read() {
                let field: Option<&mut f32> = match reader.name() {
                    "name" => {
                        named = Some(reader.read_string());
                        None
                    }
                    "imgw" => Some(&mut camera.imagewidth),
                    "imgh" => Some(&mut camera.imageheight),
                    "senw" => Some(&mut camera.sensorwidth),
                    "senh" => Some(&mut camera.sensorheight),
                    "flen" => Some(&mut camera.focallen),
                    "Camera" => {
                        // cameras[camera.name] = camera: a null name throws.
                        match named.take() {
                            Some(name) => {
                                camera.name = name;
                                self.insert(camera.clone());
                            }
                            None => failed = true,
                        }
                        break;
                    }
                    _ => None,
                };
                if let Some(field) = field {
                    // float.Parse(xmlreader.ReadString(), en-US)
                    match parse_f32(&reader.read_string()) {
                        Some(value) => *field = value,
                        None => {
                            failed = true;
                            break;
                        }
                    }
                }
            }
            if !failed {
                // string temp = xmlreader.ReadString(): on the end tag, which it leaves alone.
                let _ = reader.read_string();
            }
        }
        // populate list
        for camera in &self.cameras {
            if !self.items.contains(&camera.name) {
                self.items.push(camera.name.clone());
            }
        }
    }

    /// `cameras[name] = camera`.
    fn insert(&mut self, camera: CameraInfo) {
        match self
            .cameras
            .iter_mut()
            .find(|held| held.name == camera.name)
        {
            Some(held) => *held = camera,
            None => self.cameras.push(camera),
        }
    }
}

/// One node as `XmlTextReader` reports it, with the `Name` the loop switches on.
#[derive(Debug, Clone, PartialEq)]
enum Node {
    /// An element's start tag; `empty` for `<name/>`, which has no end tag.
    Element { name: String, empty: bool },
    /// An end tag.
    End { name: String },
    /// Text, white space or CDATA: `Name` is empty.
    Text(String),
    /// The XML declaration (`Name` "xml"), a processing instruction (its target), a comment or a
    /// document type (empty).
    Other(String),
}

impl Node {
    fn name(&self) -> &str {
        match self {
            Self::Element { name, .. } | Self::End { name } | Self::Other(name) => name,
            Self::Text(_) => "",
        }
    }
}

/// The document's nodes in order, up to the first thing that is not well formed.
fn tokenize(xml: &str) -> Vec<Node> {
    let mut nodes = Vec::new();
    let mut rest = xml.trim_start_matches('\u{feff}');
    while !rest.is_empty() {
        if let Some(after) = rest.strip_prefix("<!--") {
            let Some(end) = after.find("-->") else { break };
            nodes.push(Node::Other(String::new()));
            rest = &after[end + 3..];
        } else if let Some(after) = rest.strip_prefix("<![CDATA[") {
            let Some(end) = after.find("]]>") else { break };
            nodes.push(Node::Text(after[..end].to_owned()));
            rest = &after[end + 3..];
        } else if let Some(after) = rest.strip_prefix("<?") {
            let Some(end) = after.find("?>") else { break };
            let target = after[..end].split_whitespace().next().unwrap_or("");
            nodes.push(Node::Other(target.to_owned()));
            rest = &after[end + 2..];
        } else if let Some(after) = rest.strip_prefix("<!") {
            let Some(end) = after.find('>') else { break };
            nodes.push(Node::Other(String::new()));
            rest = &after[end + 1..];
        } else if let Some(after) = rest.strip_prefix("</") {
            let Some(end) = after.find('>') else { break };
            nodes.push(Node::End {
                name: after[..end].trim().to_owned(),
            });
            rest = &after[end + 1..];
        } else if let Some(after) = rest.strip_prefix('<') {
            let Some(end) = after.find('>') else { break };
            let tag = &after[..end];
            let (tag, empty) = tag
                .strip_suffix('/')
                .map_or((tag, false), |tag| (tag, true));
            let name = tag.split_whitespace().next().unwrap_or("").to_owned();
            if name.is_empty() {
                break;
            }
            nodes.push(Node::Element { name, empty });
            rest = &after[end + 1..];
        } else {
            let end = rest.find('<').unwrap_or(rest.len());
            nodes.push(Node::Text(unescape(&rest[..end])));
            rest = &rest[end..];
        }
    }
    nodes
}

/// The five predefined entities and character references.
fn unescape(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    let mut rest = text;
    while let Some(at) = rest.find('&') {
        out.push_str(&rest[..at]);
        let after = &rest[at + 1..];
        let Some(end) = after.find(';') else {
            out.push_str(&rest[at..]);
            return out;
        };
        let entity = &after[..end];
        let decoded = match entity {
            "amp" => Some('&'),
            "lt" => Some('<'),
            "gt" => Some('>'),
            "quot" => Some('"'),
            "apos" => Some('\''),
            _ => entity
                .strip_prefix("#x")
                .and_then(|hex| u32::from_str_radix(hex, 16).ok())
                .or_else(|| entity.strip_prefix('#').and_then(|dec| dec.parse().ok()))
                .and_then(char::from_u32),
        };
        match decoded {
            Some(c) => out.push(c),
            None => out.push_str(&rest[at..=at + end + 1]),
        }
        rest = &after[end + 1..];
    }
    out.push_str(rest);
    out
}

/// `XmlTextWriter.WriteString` in an element, as the ASCII file ends up holding it: `<`, `>` and
/// `&` as entities, a control character other than tab, line feed and return - and U+FFFE and
/// U+FFFF - as a character reference (`XmlTextEncoder.Write`), and any other character the
/// ASCII encoding cannot hold as its replacement, `?` for each UTF-16 unit: two for a character
/// past U+FFFF. As `XmlTextWriter` writes under mono.
fn xml_text(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    for c in text.chars() {
        match c {
            '<' => out.push_str("&lt;"),
            '>' => out.push_str("&gt;"),
            '&' => out.push_str("&amp;"),
            '\t' | '\n' | '\r' => out.push(c),
            c if c < ' ' || matches!(c, '\u{FFFE}' | '\u{FFFF}') => {
                out.push_str(&format!("&#x{:X};", u32::from(c)));
            }
            c if c.is_ascii() => out.push(c),
            c => {
                for _ in 0..c.len_utf16() {
                    out.push('?');
                }
            }
        }
    }
    out
}

/// An `XmlTextReader` over the nodes: `Read`, `Name` and `ReadString`.
struct Reader<'a> {
    nodes: &'a [Node],
    at: Option<usize>,
}

impl<'a> Reader<'a> {
    /// `Read()`: the next node, or false at the end.
    fn read(&mut self) -> bool {
        let next = self.at.map_or(0, |at| at + 1);
        if next < self.nodes.len() {
            self.at = Some(next);
            true
        } else {
            self.at = Some(self.nodes.len());
            false
        }
    }

    fn node(&self) -> Option<&'a Node> {
        self.at.and_then(|at| self.nodes.get(at))
    }

    /// `Name`.
    fn name(&self) -> &'a str {
        self.node().map_or("", Node::name)
    }

    /// `ReadString()`: on a start tag, the text up to the next markup, leaving the reader on that
    /// markup - the end tag, normally; on anything else, the text nodes from here.
    fn read_string(&mut self) -> String {
        if let Some(Node::Element { empty, .. }) = self.node() {
            if *empty || !self.read() {
                return String::new();
            }
            if matches!(self.node(), Some(Node::End { .. })) {
                return String::new();
            }
        }
        let mut result = String::new();
        while let Some(Node::Text(text)) = self.node() {
            result.push_str(text);
            if !self.read() {
                break;
            }
        }
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_shipped_list_reads_whole() {
        let cameras = Cameras::builtin();
        assert_eq!(cameras.items().len(), 31);
        assert_eq!(
            cameras.items().first().map(String::as_str),
            Some("Nikon aw100")
        );
        let sx230 = cameras.get("Canon SX230 HS").expect("a shipped camera");
        assert_eq!(sx230.focallen, 5.0);
        assert_eq!(sx230.imagewidth, 4000.0);
        assert_eq!(sx230.imageheight, 3000.0);
        assert_eq!(sx230.sensorwidth, 6.16);
        assert_eq!(sx230.sensorheight, 4.62);
    }

    /// The copy in this crate is the file Mission Planner ships, when the reference tree is here.
    #[test]
    fn the_asset_is_mission_planners_file() {
                // `MP_SRC` names a clone of https://github.com/ArduPilot/MissionPlanner.
        let Some(tree) = std::env::var_os("MP_SRC") else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        let path = std::path::PathBuf::from(tree).join("camerasBuiltin.xml");
        let Ok(shipped) = std::fs::read_to_string(&path) else {
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
            return;
        };
        assert_eq!(shipped, BUILTIN_XML);
    }

    #[test]
    fn a_user_file_adds_and_replaces_by_name() {
        let mut cameras = Cameras::builtin();
        cameras.read(
            "<?xml version=\"1.0\"?><Cameras>\n<Camera><name>Canon SX230 HS</name><flen>6</flen>\
             <imgh>3000</imgh><imgw>4000</imgw><senh>4.62</senh><senw>6.16</senw></Camera>\n\
             <Camera><name>Mine &amp; yours</name><flen>8.5</flen><imgw>1000</imgw></Camera>\n\
             </Cameras>",
        );
        assert_eq!(cameras.items().len(), 32);
        assert_eq!(
            cameras.items().get(1).map(String::as_str),
            Some("Canon SX230 HS")
        );
        assert_eq!(cameras.get("Canon SX230 HS").map(|c| c.focallen), Some(6.0));
        assert_eq!(
            cameras.items().last().map(String::as_str),
            Some("Mine & yours")
        );
        let mine = cameras.get("Mine & yours").expect("added");
        assert_eq!(
            (mine.focallen, mine.imagewidth, mine.imageheight),
            (8.5, 1000.0, 0.0)
        );
    }

    /// A value `float.Parse` refuses throws inside the camera; the reader carries on from inside it
    /// and falls out of step, so the cameras after it are lost too.
    #[test]
    fn a_bad_number_loses_that_camera_and_the_rest() {
        let mut cameras = Cameras::default();
        cameras.read(
            "<Cameras>\n<Camera><name>A</name><flen>5</flen></Camera>\n\
             <Camera><name>B</name><flen>five</flen></Camera>\n\
             <Camera><name>C</name><flen>7</flen></Camera>\n\
             <Camera><name>D</name><flen>8</flen></Camera>\n</Cameras>",
        );
        assert_eq!(cameras.items(), ["A"]);
    }

    #[test]
    fn a_camera_without_a_name_is_skipped() {
        let mut cameras = Cameras::default();
        cameras.read(
            "<Cameras><Camera><flen>5</flen></Camera><Camera><name>E</name></Camera></Cameras>",
        );
        // The nameless camera throws at its end tag; the reader is then between cameras, so the
        // next one reads normally.
        assert_eq!(cameras.items(), ["E"]);
    }

    /// A directory of the test's own under the system's temporary one, empty.
    fn scratch(test: &str) -> std::path::PathBuf {
        let dir =
            std::env::temp_dir().join(format!("mp-mission-cameras-{test}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        dir
    }

    fn camera(name: &str) -> CameraInfo {
        CameraInfo {
            name: name.to_owned(),
            focallen: 4.5,
            sensorwidth: 6.17,
            sensorheight: 4.55,
            imagewidth: 4000.0,
            imageheight: 3000.0,
        }
    }

    /// `XmlTextWriter(filename, Encoding.ASCII)` with `Formatting.Indented`: the declaration
    /// naming `us-ascii`, two-space indents, the platform's line end, none after the root, each number
    /// `float.ToString` - seven significant digits, so 1/3 is 0.3333333.
    /// `// C#: Grid/GridUI.cs:466-494`
    #[test]
    fn the_file_is_what_xmltextwriter_writes() {
        let mut cameras = Cameras::default();
        *cameras.entry("Mine & <yours>").unwrap() = CameraInfo {
            focallen: 1.0 / 3.0,
            ..camera("Mine & <yours>")
        };
        *cameras.entry("B").unwrap() = camera("B");
        let expected = "<?xml version=\"1.0\" encoding=\"us-ascii\"?>\n\
            <Cameras>\n\
            \x20 <Camera>\n\
            \x20   <name>Mine &amp; &lt;yours&gt;</name>\n\
            \x20   <flen>0.3333333</flen>\n\
            \x20   <imgh>3000</imgh>\n\
            \x20   <imgw>4000</imgw>\n\
            \x20   <senh>4.55</senh>\n\
            \x20   <senw>6.17</senw>\n\
            \x20 </Camera>\n\
            \x20 <Camera>\n\
            \x20   <name>B</name>\n\
            \x20   <flen>4.5</flen>\n\
            \x20   <imgh>3000</imgh>\n\
            \x20   <imgw>4000</imgw>\n\
            \x20   <senh>4.55</senh>\n\
            \x20   <senw>6.17</senw>\n\
            \x20 </Camera>\n\
            </Cameras>"
            .replace('\n', NEW_LINE);
        assert_eq!(String::from_utf8(cameras.to_xml()).unwrap(), expected);
    }

    /// Nothing to write, or only a camera named "" (`if (key == "") continue;`): the root closed
    /// short.
    #[test]
    fn an_empty_list_is_an_empty_root() {
        let mut cameras = Cameras::default();
        cameras.entry("");
        assert_eq!(
            cameras.to_xml(),
            format!("<?xml version=\"1.0\" encoding=\"us-ascii\"?>{NEW_LINE}<Cameras />")
                .into_bytes()
        );
    }

    /// What the ASCII writer makes of a name: markup characters as entities, a control
    /// character as a character reference, anything else outside ASCII as the encoding's `?`
    /// per UTF-16 unit - `XmlTextWriter`'s own output under mono for this name.
    #[test]
    fn a_name_is_escaped_as_the_ascii_writer_escapes_it() {
        assert_eq!(xml_text("a<b>&c"), "a&lt;b&gt;&amp;c");
        assert_eq!(xml_text("tab\there\"'"), "tab\there\"'");
        assert_eq!(xml_text("\u{1}"), "&#x1;");
        assert_eq!(xml_text("Sony \u{3b1}7"), "Sony ?7");
        assert_eq!(xml_text("\u{1F600}"), "??");
    }

    /// A camera saved and written is the camera the next dialog reads back, in its place in the
    /// list and with its values, from the file the C# would read.
    #[test]
    fn a_saved_camera_round_trips_through_the_file() {
        let dir = scratch("round-trip");
        let (mut cameras, error) = Cameras::load(Some(&dir));
        assert!(error.is_none(), "{error:?}");
        *cameras.entry("Mine & yours").unwrap() = CameraInfo {
            focallen: 8.8,
            ..camera("Mine & yours")
        };
        // An edit of a shipped camera replaces it in its place.
        cameras.entry("Canon SX230 HS").unwrap().focallen = 6.5;
        cameras.write(&dir.join(USER_FILE)).expect("written");

        let (read, error) = Cameras::load(Some(&dir));
        assert!(error.is_none(), "{error:?}");
        assert_eq!(read.items().len(), 32);
        assert_eq!(
            read.items().get(1).map(String::as_str),
            Some("Canon SX230 HS")
        );
        assert_eq!(
            read.items().last().map(String::as_str),
            Some("Mine & yours")
        );
        assert_eq!(read.get("Canon SX230 HS").map(|c| c.focallen), Some(6.5));
        assert_eq!(
            read.get("Mine & yours"),
            Some(&CameraInfo {
                focallen: 8.8,
                ..camera("Mine & yours")
            })
        );
        // Read back and written again, the file is the same bytes.
        assert_eq!(read.to_xml(), std::fs::read(dir.join(USER_FILE)).unwrap());
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// No `cameras.xml`: the constructor's `xmlcamera(false, ...)` writes it, every shipped
    /// camera in the shipped order, and the next load reads that file.
    /// `// C#: Grid/GridUI.cs:124, 460-462`
    #[test]
    fn a_missing_file_is_written_with_the_shipped_list() {
        let dir = scratch("missing");
        let (cameras, error) = Cameras::load(Some(&dir));
        assert!(error.is_none(), "{error:?}");
        assert_eq!(cameras, Cameras::builtin());
        let written = std::fs::read(dir.join(USER_FILE)).expect("written");
        assert_eq!(written, Cameras::builtin().to_xml());
        let mut reread = Cameras::default();
        reread.read(&String::from_utf8(written).unwrap());
        assert_eq!(reread.items(), Cameras::builtin().items());
        for name in reread.items() {
            assert_eq!(reread.get(name), Cameras::builtin().get(name), "{name}");
        }
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// A directory that cannot be made: the cameras load all the same, and the error is the
    /// write's, for the `catch`'s box.
    #[test]
    fn an_unwritable_directory_is_the_writes_error() {
        let dir = scratch("unwritable");
        std::fs::create_dir_all(dir.parent().unwrap()).unwrap();
        std::fs::write(&dir, b"a file where the directory would be").unwrap();
        let (cameras, error) = Cameras::load(Some(&dir.join("inner")));
        assert!(error.is_some());
        assert_eq!(cameras, Cameras::builtin());
        let _ = std::fs::remove_file(&dir);
    }

    #[test]
    fn a_broken_file_keeps_what_came_before() {
        let mut cameras = Cameras::default();
        cameras.read("<Cameras><Camera><name>A</name></Camera><Camera><name>B</name");
        assert_eq!(cameras.items(), ["A"]);
    }
}
