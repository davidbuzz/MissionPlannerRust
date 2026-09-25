//! `ExtLibs/Utilities/GitHubContent.cs`: a directory listing and a file from GitHub's contents
//! API (`GET /repos/:owner/:repo/contents/:path`), as the Frame Type page's Default Settings
//! (`Controls/DefaultSettings.cs`) reads ArduPilot's `Tools/Frame_params`.
//!
//! The listing is a JSON array of entries, each with its `name` and `path`; the file is one such
//! entry with the file in `content`, base64 with a line break every 60 characters. Both go
//! through [`Fetch`], the network or the directory standing in for it, blocking, on a thread of
//! the caller's choosing - as the C#'s `HttpClient.GetStringAsync(...).GetAwaiter().GetResult()`
//! blocks.

use base64::Engine as _;

use crate::manifest::Fetch;

/// `githubapiurl`.
/// `// C#: ExtLibs/Utilities/GitHubContent.cs:21`
pub const API_URL: &str = "https://api.github.com/repos";

/// One entry of a listing: `GitHubContent.FileInfo`, the fields the pages read.
/// `// C#: ExtLibs/Utilities/GitHubContent.cs:26-48`
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FileInfo {
    /// `name`: the file's name, which a combo box shows (`DisplayMember = "name"`).
    pub name: String,
    /// `path`: from the repository's root, what `GetFileContent` is given.
    pub path: String,
    /// `size`, in bytes.
    pub size: u64,
}

/// The listing's URL: `path` given `/contents` in front and its trailing slashes taken off.
/// `// C#: ExtLibs/Utilities/GitHubContent.cs:50-61`
#[must_use]
pub fn dir_url(owner: &str, repo: &str, path: &str) -> String {
    let path = if path.is_empty() {
        String::new()
    } else {
        format!("/contents{path}")
    };
    let path = path.trim_end_matches(['/', '\\']);
    format!("{API_URL}/{owner}/{repo}{path}")
}

/// A file's URL: `contents/` and the path.
/// `// C#: ExtLibs/Utilities/GitHubContent.cs:85-92`
#[must_use]
pub fn file_url(owner: &str, repo: &str, path: &str) -> String {
    let path = if path.is_empty() {
        String::new()
    } else {
        format!("contents/{path}")
    };
    format!("{API_URL}/{owner}/{repo}/{path}")
}

/// A listing's entries whose names contain `filter`, ignoring case, in the listing's order.
///
/// # Errors
/// The text is not a JSON array of entries with names: `JsonConvert.DeserializeObject` throws.
/// `// C#: ExtLibs/Utilities/GitHubContent.cs:71-82`
pub fn parse_dir(text: &str, filter: &str) -> Result<Vec<FileInfo>, String> {
    let entries: Vec<serde_json::Value> =
        serde_json::from_str(text).map_err(|err| err.to_string())?;
    let filter = filter.to_lowercase();
    let mut answer = Vec::new();
    for entry in entries {
        let text = |key: &str| entry.get(key).and_then(serde_json::Value::as_str);
        // `fi.name.ToLower()` on a missing name is a `NullReferenceException`.
        let name = text("name").ok_or("an entry without a name")?;
        if name.to_lowercase().contains(&filter) {
            answer.push(FileInfo {
                name: name.to_owned(),
                path: text("path").unwrap_or_default().to_owned(),
                size: entry
                    .get("size")
                    .and_then(serde_json::Value::as_u64)
                    .unwrap_or_default(),
            });
        }
    }
    Ok(answer)
}

/// A file's bytes from its entry: `content`, base64, the line breaks GitHub puts in it skipped as
/// `Convert.FromBase64String` skips white space. `None` for a body that is JSON `null`.
///
/// # Errors
/// The text is not a JSON object, it has no `content`, or that is not base64.
/// `// C#: ExtLibs/Utilities/GitHubContent.cs:100-109`
pub fn parse_file(text: &str) -> Result<Option<Vec<u8>>, String> {
    let output: serde_json::Value = serde_json::from_str(text).map_err(|err| err.to_string())?;
    if output.is_null() {
        return Ok(None);
    }
    // `output["content"]` on a missing key is a `KeyNotFoundException`.
    let content = output
        .get("content")
        .ok_or("The given key was not present in the dictionary.")?;
    let content = content
        .as_str()
        .map_or_else(|| content.to_string(), ToOwned::to_owned);
    let compact: String = content.chars().filter(|c| !c.is_whitespace()).collect();
    base64::engine::general_purpose::STANDARD
        .decode(compact)
        .map(Some)
        .map_err(|err| err.to_string())
}

/// `GetDirContent(owner, repo, path, filter)`.
///
/// # Errors
/// The fetch failed, or its answer is not a listing.
/// `// C#: ExtLibs/Utilities/GitHubContent.cs:50-83`
pub fn dir_content(
    fetch: &dyn Fetch,
    owner: &str,
    repo: &str,
    path: &str,
    filter: &str,
) -> Result<Vec<FileInfo>, String> {
    let body = fetch.get(&dir_url(owner, repo, path))?;
    parse_dir(&String::from_utf8_lossy(&body), filter)
}

/// `GetFileContent(owner, repo, path)`.
///
/// # Errors
/// The fetch failed, or its answer is not a file.
/// `// C#: ExtLibs/Utilities/GitHubContent.cs:85-110`
pub fn file_content(
    fetch: &dyn Fetch,
    owner: &str,
    repo: &str,
    path: &str,
) -> Result<Option<Vec<u8>>, String> {
    let body = fetch.get(&file_url(owner, repo, path))?;
    parse_file(&String::from_utf8_lossy(&body))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_urls_are_the_csharps() {
        assert_eq!(
            dir_url("ArduPilot", "ardupilot", "/Tools/Frame_params/"),
            "https://api.github.com/repos/ArduPilot/ardupilot/contents/Tools/Frame_params"
        );
        assert_eq!(
            dir_url("ArduPilot", "ardupilot", ""),
            "https://api.github.com/repos/ArduPilot/ardupilot"
        );
        assert_eq!(
            file_url("ArduPilot", "ardupilot", "Tools/Frame_params/Solo.param"),
            "https://api.github.com/repos/ArduPilot/ardupilot/contents/Tools/Frame_params/Solo.param"
        );
    }

    #[test]
    fn a_listing_keeps_the_names_the_filter_matches() {
        let text = r#"[
            {"name": "3DR_Iris+.param", "path": "Tools/Frame_params/3DR_Iris+.param", "size": 120, "type": "file"},
            {"name": "README.md", "path": "Tools/Frame_params/README.md", "size": 9, "type": "file"},
            {"name": "Sub", "path": "Tools/Frame_params/Sub", "size": 0, "type": "dir"},
            {"name": "SOLO.PARAM", "path": "Tools/Frame_params/SOLO.PARAM", "size": 7, "type": "file"}
        ]"#;
        let files = parse_dir(text, ".param").expect("a listing");
        let names: Vec<&str> = files.iter().map(|file| file.name.as_str()).collect();
        assert_eq!(names, ["3DR_Iris+.param", "SOLO.PARAM"]);
        assert_eq!(files[0].path, "Tools/Frame_params/3DR_Iris+.param");
        assert_eq!(files[0].size, 120);
        assert!(parse_dir("{\"message\": \"rate limited\"}", ".param").is_err());
        assert!(parse_dir("[{\"path\": \"x\"}]", "").is_err());
    }

    #[test]
    fn a_file_is_its_content_decoded_line_breaks_and_all() {
        let text = r#"{"name": "a.param", "encoding": "base64", "content": "RlJB\nTUUg\nMQo=\n"}"#;
        assert_eq!(parse_file(text), Ok(Some(b"FRAME 1\n".to_vec())));
        assert_eq!(parse_file("null"), Ok(None));
        assert!(parse_file("{\"name\": \"a\"}").is_err());
        assert!(parse_file("{\"content\": \"!!!\"}").is_err());
    }

    /// Through a fetch: the mirror standing in for the web serves the listing at its URL.
    #[test]
    fn the_listing_and_the_file_come_through_the_fetch() {
        struct Served;
        impl Fetch for Served {
            fn get(&self, url: &str) -> Result<Vec<u8>, String> {
                match url {
                    "https://api.github.com/repos/ArduPilot/ardupilot/contents/Tools/Frame_params" => {
                        Ok(
                            br#"[{"name": "a.param", "path": "Tools/Frame_params/a.param"}]"#
                                .to_vec(),
                        )
                    }
                    "https://api.github.com/repos/ArduPilot/ardupilot/contents/Tools/Frame_params/a.param" => {
                        Ok(br#"{"content": "RlJBTUUgMQo="}"#.to_vec())
                    }
                    _ => Err(format!("404 {url}")),
                }
            }
        }
        let files = dir_content(
            &Served,
            "ArduPilot",
            "ardupilot",
            "/Tools/Frame_params/",
            ".param",
        )
        .expect("listed");
        assert_eq!(files.len(), 1);
        let bytes = file_content(&Served, "ArduPilot", "ardupilot", &files[0].path).expect("read");
        assert_eq!(bytes, Some(b"FRAME 1\n".to_vec()));
        assert!(file_content(&Served, "ArduPilot", "ardupilot", "nope").is_err());
    }
}
