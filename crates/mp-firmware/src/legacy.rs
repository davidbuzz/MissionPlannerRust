//! The legacy firmware catalogue: `firmware2.xml`, the history of it, and which vehicle picture
//! each of its entries labels.
//!
//! Ported from `Utilities/Firmware.cs` @ efb0801 (GPL-3.0-or-later) - `software`, `getFWList`,
//! `getAPMVersion`, `GetAPMVERSIONFile`, the static constructor's `niceNames` and `getUrl` - with
//! `updateDisplayName` from `GCSViews/ConfigurationView/ConfigFirmware.cs`, the Install Firmware
//! Legacy page that reads them. What happens once an entry is chosen is `crate::flow`.
//!
//! `firmware2.xml` is the catalogue Mission Planner used before the manifest: one `<Firmware>` per
//! vehicle and frame, each with a URL per board family (`url2560`, `urlfmuv3`, ...). ArduPilot
//! still publishes it. `getFWList` reads it from GitHub, then from ardupilot.org, and for each
//! entry reads `git-version.txt` beside its `fmuv3` (else `px4v2`) build to put the version it
//! names - `ArduCopter V4.7.1` - in place of the entry's name, the name moving to its `desc`.

use std::collections::HashMap;

use crate::manifest::{Fetch, is_absolute_url};

/// `Firmware.firmwareurl`: the list's two sources, `;`-separated, tried in turn.
/// `// C#: Utilities/Firmware.cs:32`
pub const FIRMWARE_URLS: &str = "https://github.com/ArduPilot/binary/raw/master/Firmware/firmware2.xml;https://firmware.ardupilot.org/Tools/MissionPlanner/Firmware/firmware2.xml";

/// "Beta firmwares": the `dev` list.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmware.cs:600`
pub const BETA_URLS: &str = "https://github.com/ArduPilot/binary/raw/master/dev/firmware2.xml;https://firmware.ardupilot.org/Tools/MissionPlanner/dev/firmware2.xml";

/// `Ctrl+Q`'s trunk list.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmware.cs:117`
pub const TRUNK_URLS: &str = "https://github.com/ArduPilot/binary/raw/master/dev/firmwarelatest.xml;https://firmware.ardupilot.org/Tools/MissionPlanner/dev/firmwarelatest.xml";

/// `gholdurl`: an old list by commit.
/// `// C#: Utilities/Firmware.cs:34`
const GHOLD_URL: &str = "https://github.com/diydrones/binary/raw/!Hash!/Firmware/firmware2.xml";

/// `gholdfirmwareurl`: an old firmware by commit.
/// `// C#: Utilities/Firmware.cs:35`
const GHOLD_FIRMWARE_URL: &str =
    "https://github.com/diydrones/binary/raw/!Hash!/Firmware/!Firmware!";

/// `FirmwareHistory.txt`, which Mission Planner reads from beside its executable. Built in here:
/// there is no install directory to read it from.
/// `// C#: FirmwareHistory.txt; Utilities/Firmware.cs:124-125`
pub const HISTORY: &str = include_str!("../data/FirmwareHistory.txt");

/// `Strings.GettingFWList`. `// C#: ExtLibs/Strings/Strings.resx:369-371`
pub const GETTING_FW_LIST: &str = "Getting FW List";
/// `Strings.ReceivedList`. `// C#: ExtLibs/Strings/Strings.resx:384-386`
pub const RECEIVED_LIST: &str = "Received List";
/// `Strings.GettingFWVersion`. `// C#: ExtLibs/Strings/Strings.resx:372-374`
pub const GETTING_FW_VERSION: &str = "Getting FW Version";

/// `Firmware.software`: one entry of the list. Every string starts empty, as its initialisers
/// make it; an element the XML lacks leaves it so.
/// `// C#: Utilities/Firmware.cs:51-94`
#[derive(Debug, Clone, Default, PartialEq, Eq)]
#[allow(missing_docs)] // the C#'s field names, one per board family
pub struct Software {
    pub url: String,
    pub url2560: String,
    /// `<url2560-2>`.
    pub url2560_2: String,
    pub urlpx4v1: String,
    pub urlpx4rl: String,
    pub urlpx4v2: String,
    pub urlpx4v3: String,
    pub urlpx4v4: String,
    pub urlpx4v4pro: String,
    pub urlvrbrainv40: String,
    pub urlvrbrainv45: String,
    pub urlvrbrainv50: String,
    pub urlvrbrainv51: String,
    pub urlvrbrainv52: String,
    pub urlvrbrainv54: String,
    pub urlvrcorev10: String,
    pub urlvrubrainv51: String,
    pub urlvrubrainv52: String,
    pub urlbebop2: String,
    pub urldisco: String,
    pub urlnxpfmuk66: String,
    pub urlfmuv2: String,
    pub urlfmuv3: String,
    pub urlfmuv4: String,
    pub urlfmuv5: String,
    pub urlrevomini: String,
    pub urlmindpxv2: String,
    /// What the entry is called; `getAPMVersion` replaces it with the version it reads.
    pub name: String,
    /// What it was called, once `getAPMVersion` has replaced the name.
    pub desc: String,
    /// `k_format_version`, an `int`. No list ArduPilot publishes has the element: theirs is
    /// `<format_version>`, which the serializer ignores.
    pub k_format_version: i32,
}

/// The number of URL fields.
pub const URL_FIELDS: usize = 27;

impl Software {
    /// Every URL field with its XML element name, in declaration order: what the C#'s reflection
    /// over the fields whose names contain "url" walks (`ConfigFirmware.cs:413-427`).
    pub fn urls_mut(&mut self) -> [(&'static str, &mut String); URL_FIELDS] {
        [
            ("url", &mut self.url),
            ("url2560", &mut self.url2560),
            ("url2560-2", &mut self.url2560_2),
            ("urlpx4v1", &mut self.urlpx4v1),
            ("urlpx4rl", &mut self.urlpx4rl),
            ("urlpx4v2", &mut self.urlpx4v2),
            ("urlpx4v3", &mut self.urlpx4v3),
            ("urlpx4v4", &mut self.urlpx4v4),
            ("urlpx4v4pro", &mut self.urlpx4v4pro),
            ("urlvrbrainv40", &mut self.urlvrbrainv40),
            ("urlvrbrainv45", &mut self.urlvrbrainv45),
            ("urlvrbrainv50", &mut self.urlvrbrainv50),
            ("urlvrbrainv51", &mut self.urlvrbrainv51),
            ("urlvrbrainv52", &mut self.urlvrbrainv52),
            ("urlvrbrainv54", &mut self.urlvrbrainv54),
            ("urlvrcorev10", &mut self.urlvrcorev10),
            ("urlvrubrainv51", &mut self.urlvrubrainv51),
            ("urlvrubrainv52", &mut self.urlvrubrainv52),
            ("urlbebop2", &mut self.urlbebop2),
            ("urldisco", &mut self.urldisco),
            ("urlnxpfmuk66", &mut self.urlnxpfmuk66),
            ("urlfmuv2", &mut self.urlfmuv2),
            ("urlfmuv3", &mut self.urlfmuv3),
            ("urlfmuv4", &mut self.urlfmuv4),
            ("urlfmuv5", &mut self.urlfmuv5),
            ("urlrevomini", &mut self.urlrevomini),
            ("urlmindpxv2", &mut self.urlmindpxv2),
        ]
    }

    /// The entry with each URL rewritten for a history entry chosen in "Pick previous firmware":
    /// `getUrl(history, url)` for every URL that is not empty. The C# swallows an error in any
    /// one (`catch { }`), leaving that URL as it was.
    /// `// C#: GCSViews/ConfigurationView/ConfigFirmware.cs:411-430`
    #[must_use]
    pub fn for_history(&self, history: &str) -> Self {
        let mut rewritten = self.clone();
        for (_, url) in rewritten.urls_mut() {
            if url.is_empty() {
                continue;
            }
            if let Ok(new) = get_url(history, url) {
                *url = new;
            }
        }
        rewritten
    }
}

/// `XmlSerializer.Deserialize` of `optionsObject`: the root must be `<options>`; each `<Firmware>`
/// under it is an entry, each child element the field of its name, the text its value; unknown
/// elements are ignored - `<urlpx4>` and `<format_version>` among them - and a repeated one is
/// the last.
/// `// C#: Utilities/Firmware.cs:43-49, 202-213`
///
/// # Errors
/// Text that is not XML, a root that is not `<options>`, or a `k_format_version` that is not an
/// `int`: the serializer throws, and `getFWList` goes on to its next source.
pub fn parse_list(text: &str) -> Result<Vec<Software>, String> {
    let document = roxmltree::Document::parse(text).map_err(|err| err.to_string())?;
    let root = document.root_element();
    if root.tag_name().name() != "options" {
        return Err(format!(
            "<{} xmlns=''> was not expected.",
            root.tag_name().name()
        ));
    }
    let mut list = Vec::new();
    for firmware in root
        .children()
        .filter(|node| node.is_element() && node.tag_name().name() == "Firmware")
    {
        let mut software = Software::default();
        for field in firmware.children().filter(roxmltree::Node::is_element) {
            let name = field.tag_name().name();
            let value: String = field
                .children()
                .filter_map(|child| child.text())
                .collect::<String>();
            match name {
                "name" => software.name = value,
                "desc" => software.desc = value,
                "k_format_version" => {
                    software.k_format_version = value
                        .trim()
                        .parse()
                        .map_err(|_| format!("k_format_version '{value}' is not an int"))?;
                }
                _ => {
                    if let Some((_, slot)) = software
                        .urls_mut()
                        .into_iter()
                        .find(|(element, _)| *element == name)
                    {
                        *slot = value;
                    }
                }
            }
        }
        list.push(software);
    }
    Ok(list)
}

/// `new Uri(new Uri(base), relative).AbsoluteUri` for the URLs these lists hold: an absolute
/// `relative` is itself; one starting `//` takes the base's scheme; one starting `/` the base's
/// scheme and host; anything else replaces the last segment of the base's path.
///
/// # Errors
/// A base that is not an absolute URL: `new Uri` throws `UriFormatException`.
pub fn resolve(base: &str, relative: &str) -> Result<String, String> {
    if !is_absolute_url(base) {
        return Err(format!("Invalid URI: {base}"));
    }
    if is_absolute_url(relative) {
        return Ok(relative.to_owned());
    }
    let (scheme, rest) = base.split_once(':').unwrap_or((base, ""));
    if let Some(net) = relative.strip_prefix("//") {
        return Ok(format!("{scheme}://{net}"));
    }
    let rest = rest.strip_prefix("//").unwrap_or(rest);
    let rest = rest.split(['?', '#']).next().unwrap_or_default();
    let (authority, path) = rest.split_at(rest.find('/').unwrap_or(rest.len()));
    if relative.starts_with('/') {
        return Ok(format!("{scheme}://{authority}{relative}"));
    }
    let directory = path
        .rfind('/')
        .map_or("/", |at| path.get(..=at).unwrap_or("/"));
    Ok(format!("{scheme}://{authority}{directory}{relative}"))
}

/// `niceNames`: each line of `FirmwareHistory.txt` longer than forty characters whose first space
/// is at forty or beyond, as (its first word, the rest). The key is a list's URL; the value what
/// "Pick previous firmware" shows: `AC 4.0.3 AP 4.0.5 AS 4.0.0`.
/// `// C#: Utilities/Firmware.cs:122-162`
#[must_use]
pub fn nice_names() -> Vec<(String, String)> {
    let text = HISTORY.strip_prefix('\u{feff}').unwrap_or(HISTORY);
    let mut names = Vec::new();
    for line in text.lines() {
        // `gh.Length > 40`, in UTF-16 units; the file is ASCII.
        if line.chars().count() <= 40 {
            continue;
        }
        let Some(index) = line.find(' ') else {
            continue;
        };
        if index < 40 {
            continue;
        }
        // `gh.Trim().Substring(0, index)` and `gh.Substring(index + 1).Trim()`.
        let key: String = line.trim().chars().take(index).collect();
        let value = line.get(index + 1..).unwrap_or_default().trim().to_owned();
        names.push((key, value));
    }
    names
}

/// `getUrl(hash, filename)`: for a history entry that is a URL, the URL itself when no file is
/// asked for, else the file resolved against it; for a commit hash in the history, the list or
/// the firmware at that commit; otherwise nothing.
/// `// C#: Utilities/Firmware.cs:96-120`
///
/// # Errors
/// A history URL that `new Uri` refuses.
pub fn get_url(hash: &str, filename: &str) -> Result<String, String> {
    if hash.to_lowercase().starts_with("http") {
        if filename.is_empty() {
            return Ok(hash.to_owned());
        }
        return resolve(hash, filename);
    }
    // `gholdurls`: the history's first words.
    if nice_names().iter().any(|(key, _)| key == hash) {
        if filename.is_empty() {
            return Ok(GHOLD_URL.replace("!Hash!", hash));
        }
        let file = filename.rsplit(['/', '\\']).next().unwrap_or(filename);
        return Ok(GHOLD_FIRMWARE_URL
            .replace("!Hash!", hash)
            .replace("!Firmware!", file));
    }
    Ok(String::new())
}

/// `getFWList`: each source in turn until one gives a list, then `getAPMVersion` for each entry.
/// `firmwareurl` empty is [`FIRMWARE_URLS`]. `progress` is `updateProgress`, which the page's
/// progress dialog shows.
/// `// C#: Utilities/Firmware.cs:175-246`
///
/// The C# resolves each source's host first (`Dns.GetHostAddresses`), which the fetch does here;
/// it reads the versions with `Parallel.ForEach`, which here is one after another - each writes
/// only its own entry, so the list comes out the same.
///
/// # Errors
/// No source gave a list: the last source's error, which the C# rethrows.
pub fn get_fw_list(
    firmwareurl: &str,
    fetch: &dyn Fetch,
    progress: &mut dyn FnMut(i32, &str),
) -> Result<Vec<Software>, String> {
    let firmwareurl = if firmwareurl.is_empty() {
        FIRMWARE_URLS
    } else {
        firmwareurl
    };
    progress(-1, GETTING_FW_LIST);
    let mut failure = None;
    let mut list = None;
    for url in firmwareurl.split(';').filter(|url| !url.is_empty()) {
        let read = fetch.get(url).and_then(|bytes| {
            parse_list(String::from_utf8_lossy(&bytes).trim_start_matches('\u{feff}'))
        });
        match read {
            Ok(found) => {
                list = Some(found);
                break;
            }
            Err(err) => failure = Some(format!("{url}: {err}")),
        }
    }
    let Some(mut list) = list else {
        // `throw invalidex`, which is null - a `NullReferenceException` - when there was no
        // source to try.
        return Err(failure.unwrap_or_else(|| "no firmware list URL".to_owned()));
    };
    let mut cache = HashMap::new();
    for software in &mut list {
        get_apm_version(software, fetch, &mut cache, progress);
    }
    progress(-1, RECEIVED_LIST);
    Ok(list)
}

/// `getAPMVersion`: `git-version.txt` beside the entry's `fmuv3` build - or its `px4v2` build
/// when the `fmuv3` one is not there - and its `APMVERSION:` line, which becomes the entry's name
/// and its name its description. Anything going wrong leaves the entry as it was.
/// `// C#: Utilities/Firmware.cs:265-304`
fn get_apm_version(
    temp: &mut Software,
    fetch: &dyn Fetch,
    cache: &mut HashMap<String, String>,
    progress: &mut dyn FnMut(i32, &str),
) {
    let mut baseurl = temp.urlfmuv3.clone();
    // `CheckHTTPFileExists` throwing is the `catch` around the whole: nothing changes.
    match fetch.exists(&baseurl) {
        Ok(true) => {}
        Ok(false) => baseurl.clone_from(&temp.urlpx4v2),
        Err(_) => return,
    }
    if baseurl.is_empty() || !baseurl.to_lowercase().starts_with("http") {
        return;
    }
    let Ok(url) = resolve(&baseurl, "git-version.txt") else {
        return;
    };
    progress(-1, GETTING_FW_VERSION);
    let Some(line) = apm_version_file(&url, fetch, cache) else {
        return;
    };
    // `line.Substring(line.IndexOf(':') + 2)`.
    let Some(name) = line
        .find(':')
        .and_then(|colon| line.get(colon + 2..))
        .map(str::to_owned)
    else {
        return;
    };
    temp.desc = std::mem::replace(&mut temp.name, name);
}

/// `GetAPMVERSIONFile`: the first line of the file containing `APMVERSION:`, cached by URL for
/// the process in the C# and for one list here. `None` for a file without one - the C# throws
/// `TimeoutException` - or one that cannot be read.
/// `// C#: Utilities/Firmware.cs:306-352`
fn apm_version_file(
    url: &str,
    fetch: &dyn Fetch,
    cache: &mut HashMap<String, String>,
) -> Option<String> {
    if let Some(line) = cache.get(url) {
        return Some(line.clone());
    }
    let bytes = fetch.get(url).ok()?;
    let text = String::from_utf8_lossy(&bytes);
    let line = text.lines().find(|line| line.contains("APMVERSION:"))?;
    cache.insert(url.to_owned(), line.to_owned());
    Some(line.to_owned())
}

/// The vehicle pictures of the Install Firmware Legacy page, which `updateDisplayName` labels.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Picture {
    /// `pictureBoxRover`.
    Rover,
    /// `pictureBoxAPM`: the plane.
    Apm,
    /// `pictureBoxQuad`.
    Quad,
    /// `pictureBoxTri`.
    Tri,
    /// `pictureBoxHexa`.
    Hexa,
    /// `pictureBoxY6`.
    Y6,
    /// `pictureBoxHeli`.
    Heli,
    /// `pictureBoxOctaQuad`.
    OctaQuad,
    /// `pictureBoxOcta`.
    Octa,
    /// `pictureAntennaTracker`.
    AntennaTracker,
    /// `pictureBoxSub`.
    Sub,
}

impl Picture {
    /// The Designer's control name.
    #[must_use]
    pub const fn control(self) -> &'static str {
        match self {
            Self::Rover => "pictureBoxRover",
            Self::Apm => "pictureBoxAPM",
            Self::Quad => "pictureBoxQuad",
            Self::Tri => "pictureBoxTri",
            Self::Hexa => "pictureBoxHexa",
            Self::Y6 => "pictureBoxY6",
            Self::Heli => "pictureBoxHeli",
            Self::OctaQuad => "pictureBoxOctaQuad",
            Self::Octa => "pictureBoxOcta",
            Self::AntennaTracker => "pictureAntennaTracker",
            Self::Sub => "pictureBoxSub",
        }
    }
}

/// `updateDisplayName`: which pictures an entry labels, and with what text. Each labelled
/// picture's `Tag` becomes the entry. Where the C# writes `temp.name += " Quad"` the entry's own
/// name changes too, and the confirmation that clicking the picture asks says the new name; the
/// catch-all copter branch labels seven pictures without changing it. An entry that fits none is
/// logged ("No Home") and labels nothing.
/// `// C#: GCSViews/ConfigurationView/ConfigFirmware.cs:270-389`
pub fn display_name(temp: &mut Software) -> Vec<(Picture, String)> {
    let url2560 = temp.url2560.to_lowercase();
    let url2560_2 = temp.url2560_2.to_lowercase();
    let urlpx4v2 = temp.urlpx4v2.to_lowercase();
    let urlfmuv2 = temp.urlfmuv2.to_lowercase();
    let urlfmuv3 = temp.urlfmuv3.to_lowercase();
    let name = temp.name.to_lowercase();
    let desc = temp.desc.to_lowercase();
    // `url2560` with either marker, or the name or description naming the frame.
    let frame = |url_a: &str, url_b: &str, words: &str| {
        url2560.contains(url_a)
            || url2560.contains(url_b)
            || name.contains(words)
            || desc.contains(words)
    };
    // `pictureBoxQuad.Text = temp.name += " Quad"`: the entry renamed, and the picture with it.
    let renamed = |temp: &mut Software, picture: Picture, suffix: &str| {
        temp.name.push_str(suffix);
        vec![(picture, temp.name.clone())]
    };
    if url2560.contains("ar2") || url2560.contains("apm1/apmrover") {
        vec![(Picture::Rover, temp.name.clone())]
    } else if url2560.contains("ap-") || url2560.contains("apm1/arduplane") {
        vec![(Picture::Apm, temp.name.clone())]
    } else if frame("ac2-quad-", "1-quad/arducopter", "arducopter quad") {
        renamed(temp, Picture::Quad, " Quad")
    } else if frame("ac2-tri", "-tri/arducopter", "arducopter tri") {
        renamed(temp, Picture::Tri, " Tri")
    } else if frame("ac2-hexa", "-hexa/arducopter", "arducopter hexa") {
        renamed(temp, Picture::Hexa, " Hexa")
    } else if frame("ac2-y6", "-y6/arducopter", "arducopter y6") {
        renamed(temp, Picture::Y6, " Y6")
    } else if frame("ac2-heli-", "-heli/arducopter", "arducopter heli")
        || urlfmuv2.contains("-heli")
    {
        renamed(temp, Picture::Heli, " heli")
    } else if frame(
        "ac2-octaquad-",
        "-octa-quad/arducopter",
        "arducopter octa quad",
    ) {
        renamed(temp, Picture::OctaQuad, " Octa Quad")
    } else if frame("ac2-octa-", "-octa/arducopter", "arducopter octa") {
        renamed(temp, Picture::Octa, " Octa")
    } else if url2560_2.contains("antennatracker")
        || urlpx4v2.contains("antennatracker")
        || urlfmuv3.contains("antennatracker")
    {
        vec![(Picture::AntennaTracker, temp.name.clone())]
    } else if urlpx4v2.contains("ardusub") || urlfmuv2.contains("ardusub") {
        vec![(Picture::Sub, temp.name.clone())]
    } else if urlpx4v2.contains("copter") && !urlpx4v2.contains("heli")
        || urlfmuv3.contains("copter") && !urlfmuv3.contains("heli")
    {
        // Every copter frame from the one entry, in the C#'s order; the name is left alone.
        [
            (Picture::Octa, " Octa"),
            (Picture::OctaQuad, " Octa Quad"),
            (Picture::Heli, " heli"),
            (Picture::Y6, " Y6"),
            (Picture::Hexa, " Hexa"),
            (Picture::Tri, " Tri"),
            (Picture::Quad, " Quad"),
        ]
        .into_iter()
        .map(|(picture, suffix)| (picture, format!("{}{suffix}", temp.name)))
        .collect()
    } else if urlpx4v2.contains("rover") || urlfmuv2.contains("rover") {
        vec![(Picture::Rover, temp.name.clone())]
    } else if urlpx4v2.contains("plane") || urlfmuv2.contains("plane") {
        vec![(Picture::Apm, temp.name.clone())]
    } else {
        Vec::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn urls_resolve_as_system_uri_resolves_them() {
        let base = "http://firmware.ardupilot.org/Copter/stable/fmuv3/arducopter.apj";
        assert_eq!(
            resolve(base, "git-version.txt").as_deref(),
            Ok("http://firmware.ardupilot.org/Copter/stable/fmuv3/git-version.txt")
        );
        assert_eq!(
            resolve(base, "https://example.org/a.apj").as_deref(),
            Ok("https://example.org/a.apj")
        );
        assert_eq!(
            resolve(base, "/x/y").as_deref(),
            Ok("http://firmware.ardupilot.org/x/y")
        );
        assert_eq!(
            resolve("https://host", "a").as_deref(),
            Ok("https://host/a")
        );
        assert_eq!(
            resolve("https://host/a/b?q=1", "c").as_deref(),
            Ok("https://host/a/c")
        );
        assert!(resolve("not a url", "a").is_err());
    }

    #[test]
    fn the_history_is_the_files_long_lines() {
        let names = nice_names();
        assert_eq!(names.len(), 70);
        assert_eq!(
            names[0],
            (
                "https://github.com/diydrones/binary/raw/e94e833a5628f19137d297822343f6282af6d9cc/History/firmware2.xml"
                    .to_owned(),
                "AC 4.0.3 AP 4.0.5 AS 4.0.0".to_owned()
            )
        );
        assert_eq!(names[69].1, "AR2.20b AP 2.68 AC 2.9");
        // A history URL is its own list; a file under it resolves against it.
        let (key, _) = &names[3];
        assert_eq!(get_url(key, "").as_deref(), Ok(key.as_str()));
        assert_eq!(
            get_url(
                key,
                "http://firmware.ardupilot.org/Copter/stable/fmuv3/arducopter.apj"
            )
            .as_deref(),
            Ok("http://firmware.ardupilot.org/Copter/stable/fmuv3/arducopter.apj")
        );
        assert_eq!(get_url("unknown", "x").as_deref(), Ok(""));
    }

    #[test]
    fn an_unknown_root_or_a_bad_int_fails_the_list() {
        assert!(parse_list("<Firmwares/>").is_err());
        assert!(
            parse_list(
                "<options><Firmware><k_format_version>x</k_format_version></Firmware></options>"
            )
            .is_err()
        );
        let list = parse_list(
            "<options><Firmware><urlpx4>a</urlpx4><url2560-2>b</url2560-2><name>n</name><desc/><k_format_version>3</k_format_version></Firmware></options>",
        )
        .expect("parses");
        assert_eq!(list.len(), 1);
        assert_eq!(list[0].url2560_2, "b");
        assert_eq!(list[0].urlpx4v1, "", "<urlpx4> is not a field");
        assert_eq!(list[0].k_format_version, 3);
        assert_eq!(list[0].desc, "");
    }
}
