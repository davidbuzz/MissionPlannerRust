//! The Survey (Grid) dialog's camera list: `Grid/camerainfo.cs` and `GridUI.xmlcamera`.
//!
//! `GridUI`'s constructor reads two files into one dictionary, name to camera: the list Mission
//! Planner ships beside its executable, `camerasBuiltin.xml`, and then the user's own
//! `cameras.xml` in the user data directory, which can add cameras and replace built-in ones by
//! name (`GridUI.cs:125-127`). `CMB_camera` lists the names in the dictionary's order after each
//! file (`GridUI.cs:577-582`). The shipped file is copied into this crate as
//! `assets/camerasBuiltin.xml`; a test holds the copy to the reference tree's.
//!
//! The reader is `xmlcamera`'s `XmlTextReader` loop, node by node, because its quirks decide what
//! a damaged file yields: a `Camera` is added when its end tag is reached, a value `float.Parse`
//! refuses abandons the camera, and the reader then picks up mid-element rather than at the next
//! camera.
//!
//! Not ported: when `cameras.xml` does not exist `xmlcamera` writes it, every camera the dictionary
//! holds (`GridUI.cs:516-547`), and `BUT_save_Click` writes it again with the camera being edited.
//! This application does not write Mission Planner's data directory.

use crate::dotnet::parse_f32;

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

    /// The shipped list, then the user's `cameras.xml` from Mission Planner's user data directory
    /// if there is one, as the constructor reads them.
    /// `// C#: Grid/GridUI.cs:125-127`
    #[must_use]
    pub fn load(user_data_directory: Option<&std::path::Path>) -> Self {
        let mut cameras = Self::builtin();
        if let Some(text) = user_data_directory
            .map(|directory| directory.join("cameras.xml"))
            .and_then(|path| std::fs::read_to_string(path).ok())
        {
            cameras.read(&text);
        }
        cameras
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
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../referneces/missionplanner/camerasBuiltin.xml");
        let Ok(shipped) = std::fs::read_to_string(&path) else {
            eprintln!("skipped: the C# tree is not checked out here");
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

    #[test]
    fn a_broken_file_keeps_what_came_before() {
        let mut cameras = Cameras::default();
        cameras.read("<Cameras><Camera><name>A</name></Camera><Camera><name>B</name");
        assert_eq!(cameras.items(), ["A"]);
    }
}
