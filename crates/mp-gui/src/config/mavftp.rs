//! The MAVFtp page: `Controls/MavFTPUI.cs`, which CONFIG's list adds as "MAVFtp" once every
//! parameter is in, for a vehicle that reports MAVLink FTP (`GCSViews/SoftwareConfig.cs:211-216`).
//!
//! What it shows: a split container - on the left a tree of the vehicle's directories, on the
//! right a details list of the chosen directory (Name, Type, Size, Date modified) - over a status
//! strip of a label and a progress bar, and "Mount as Drive" at the top right
//! (`MavFTPUI.Designer.cs:29-235`). When the page is loaded it lists `/` and `@SYS/`, each a root
//! of the tree with its subdirectories under it, selects the last one made - `@SYS` - and lists it
//! (`PopulateTreeView`, `MavFTPUI.cs:73-140`). Clicking a directory in the tree lists it again,
//! replacing its children, directories first and then files (`TreeView1_NodeMouseClick`,
//! `:151-203`); a double click on a list row opens the directory of that name (`:587-605`); a
//! column header sorts the list by it, a column of digits as numbers (`:300-322`).
//!
//! The list's right-click menu: Download Burst and Download (a burst read and a plain read, into
//! a folder asked for, under the file's name, numbered when taken, `:324-379, 607-663`), Upload (a
//! file asked for, written into the directory, then the vehicle's CRC of it checked against the
//! file's, `:381-448`), Delete (`:450-480`), Rename (the row's name edited in place, `:482-513`),
//! New Folder (an `InputBox`, `:515-546`) and GetCRC32 (a box with the vehicle's CRC, `:548-572`).
//! Files dropped on the list are uploaded (`:279-298`). Each runs behind a
//! `ProgressReporterDialogue` with a Cancel, which asks the command to stop and resets the
//! vehicle's sessions. The page's `MAVFtp` reports its progress onto the status strip, at most
//! every 100 ms (`:34-65`), and a transfer's window shows it too.
//!
//! The link has one MAVFTP client per vehicle (`mp_link::ftp`), which runs one request at a
//! time: the page's commands run one after another on it, as `lock (_mavftp)` has them in the
//! C#'s listings, and a command another page is running on the vehicle is waited for.
//!
//! Where this differs from the C#, and why:
//!
//! * the progress window's error state (`ShowDoneWithError`: "There was an unexpected error (...)",
//!   "User Cancel") and the failure boxes "Failed to delete file", "Failed to create directory"
//!   and "Failed to mount" go on the status line: the owner's rulings of 2026-09-25 - a failure of
//!   the link's work is never a box, and neither is the mount's. The CRC's report and the
//!   questions keep their boxes;
//! * `FolderBrowserDialog` and `OpenFileDialog` are boxes with a path typed into them, as the
//!   application's other pages have them: there is no platform dialog here. The folder starts in
//!   `Settings.GetUserDataDirectory()`, as `SelectedPath` does; the upload takes one file, where
//!   the C#'s `Multiselect` dialog takes several;
//! * "Mount as Drive" mounts through Dokan, a Windows file-system driver that this platform does
//!   not have: `Mount` fails, and the C#'s "Failed to mount" text, with the .NET message for the
//!   missing driver, goes on the status line;
//! * a listing another command is waiting behind starts when that command ends, not beside it;
//! * the tree and the list take no keys: WinForms' arrow keys, and F2 for a rename, are not
//!   wired; neither is `AllowColumnReorder`'s dragging of the headers;
//! * a crash of the C#'s - `SelectedItems[0]` with nothing selected, `SelectedNode.FullPath` with
//!   no node - is the .NET exception's text on the status line, where Mission Planner's
//!   unhandled-exception box shows it.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::cmp::Ordering;
use std::collections::{BTreeSet, VecDeque};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use gpui::{
    AnyElement, Context, ExternalPaths, FocusHandle, KeyDownEvent, MouseButton, SharedString,
    Window, div, prelude::*, px, rgb,
};
use mp_link::FtpError;
use mp_link::mavftp::{FtpFileInfo, FtpOutcome, FtpRequest, Progress, RW_SIZE, crc_crc32};
use mp_vehicle::VehicleId;

use super::adsb::culture_cmp;
use super::optional::{InputBox, at, input_box, message_box, plain};
use super::serial_ports::{Bar, CANCELLING, ProgressIds, progress_dialog};
use crate::MissionPlanner;
use crate::config::servo_output::Message;
use crate::setup::Key;
use crate::telemetry::{Telemetry, TelemetryView};
use crate::textfield::{KeyOutcome, TextField};
use crate::ui::{panel, theme};

/// The class, as the list names it.
pub const CLASS: &str = "MavFTPUI";

/// `Strings.MAVFtp`, the page's title in CONFIG's list.
/// `// C#: GCSViews/SoftwareConfig.cs:215; ExtLibs/Strings/Strings.resx:664-666`
pub const TITLE: &str = "MAVFtp";

/// `toolStripStatusLabel1.Text` as the Designer leaves it.
/// `// C#: Controls/MavFTPUI.Designer.cs (toolStripStatusLabel1.Text)`
pub const STATUS_START: &str = "...";

/// What `PopulateTreeView` says while it lists.
/// `// C#: Controls/MavFTPUI.cs:75`
pub const UPDATING_FOLDERS: &str = "Updating Folders";

/// What the page says when a command is done.
/// `// C#: Controls/MavFTPUI.cs:114, 378, 447`
pub const READY: &str = "Ready";

/// How often the page's `Progress` handler lets a report onto the status strip.
/// `// C#: Controls/MavFTPUI.cs:44-49`
pub const REPORT_EVERY: Duration = Duration::from_millis(100);

/// `ProgressReporterDialogue`'s cancel message, `doWorkArgs.ErrorMessage`.
/// `// C#: Controls/MavFTPUI.cs:340`
pub const USER_CANCEL: &str = "User Cancel";

/// `DeleteToolStripMenuItem_Click`'s box, captioned with the file's name.
/// `// C#: Controls/MavFTPUI.cs:470-471`
pub const FAILED_DELETE: &str = "Failed to delete file";

/// `NewFolderToolStripMenuItem_Click`'s box.
/// `// C#: Controls/MavFTPUI.cs:536`
pub const FAILED_DIRECTORY: &str = "Failed to create directory";

/// `NullReferenceException.Message`: `GetFile`'s null stream written, a node that is not there.
pub const NULL_REFERENCE: &str = "Object reference not set to an instance of an object.";

/// `ArgumentOutOfRangeException.Message` for `SelectedItems[0]` of an empty selection, as the
/// .NET Framework words it.
pub const NO_SELECTION: &str =
    "InvalidArgument=Value of '0' is not valid for 'index'.\r\nParameter name: index";

/// `Ionic.Zip.BadCrcException`'s message: it is made with no text, so it is `Exception`'s own.
/// `// C#: Controls/MavFTPUI.cs:440-443`
pub const BAD_CRC: &str = "Exception of type 'Ionic.Zip.BadCrcException' was thrown.";

/// "Mount as Drive"'s `InputBox`, and the drive it offers.
/// `// C#: Controls/MavFTPUI.cs:671, 693-694`
pub const MOUNT_TITLE: &str = "Mount Point";
/// Its question.
pub const MOUNT_PROMPT: &str = "Enter drive letter or path (e.g. M:\\)";
/// `DefaultMountPoint`.
pub const DEFAULT_MOUNT_POINT: &str = "M:\\";

/// What `new Dokan(...)` throws where the driver is not installed - `DllNotFoundException` for
/// `dokan2.dll`, as the .NET Framework words it - which is every machine this runs on.
pub const DOKAN_MISSING: &str =
    "Unable to load DLL 'dokan2.dll': The specified module could not be found.";

/// The status strip's line after a mount fails.
/// `// C#: Controls/MavFTPUI.cs:704-708`
#[must_use]
pub fn mount_failed(message: &str) -> String {
    format!("Failed to mount: {message}\n\nMake sure Dokan driver is installed.")
}

/// New Folder's `InputBox`.
/// `// C#: Controls/MavFTPUI.cs:518`
pub const FOLDER_TITLE: &str = "Folder Name";
/// Its question.
pub const FOLDER_PROMPT: &str = "Enter folder name";

/// The caption of the box standing in for `FolderBrowserDialog`, whose `Description` the C# does
/// not set.
pub const BROWSE_TITLE: &str = "Browse For Folder";

/// The caption of the box standing in for `OpenFileDialog`, whose `Title` the C# does not set.
pub const OPEN_TITLE: &str = "Open";

/// `ProgressReporterDialogue`'s "There was an unexpected error (...)", for a command that threw.
/// `// C#: ExtLibs/Controls/ProgressReporterDialogue.cs:224-226`
#[must_use]
pub fn unexpected(message: &str) -> String {
    format!("There was an unexpected error ({message})")
}

/// `long.ToSizeUnits()`: bytes under a kilobyte, whole kilobytes under a megabyte, else whole
/// megabytes.
/// `// C#: ExtLibs/Utilities/Extensions.cs:1116-1153`
#[must_use]
pub fn size_units(size: u64) -> String {
    if size < 1024 {
        format!("{size}B")
    } else if size < 1024 * 1024 {
        format!("{}KB", size / 1024)
    } else {
        format!("{}MB", size / 1024 / 1024)
    }
}

/// `ModifiedString`: a listing's time in local time, "yyyy-MM-dd HH:mm:ss"; blank when the
/// vehicle did not give one.
/// `// C#: Controls/MavFTPUI.cs:144-149`
#[must_use]
pub fn modified_string(modified_utc: Option<u32>) -> String {
    use chrono::TimeZone;
    modified_utc
        .and_then(|secs| chrono::Local.timestamp_opt(i64::from(secs), 0).single())
        .map(|time| time.format("%Y-%m-%d %H:%M:%S").to_string())
        .unwrap_or_default()
}

/// `Path.GetFileName`: what follows the last separator.
#[must_use]
pub fn file_name(path: &str) -> &str {
    path.rsplit(['/', '\\']).next().unwrap_or(path)
}

// ---------------------------------------------------------------------------------------------
// The link.
// ---------------------------------------------------------------------------------------------

/// What the page needs of the link's MAVFTP client: the application's `Telemetry`, or a test's
/// vehicle.
pub trait FtpPort {
    /// Starts a request on the vehicle's client; false if it would not take it.
    fn start(&self, vehicle: VehicleId, request: FtpRequest) -> bool;
    /// Whether a request runs on the client, and its last report.
    fn progress(&self, vehicle: VehicleId) -> Option<(bool, Progress)>;
    /// The finished request's outcome.
    fn take(&self, vehicle: VehicleId) -> Option<Result<FtpOutcome, FtpError>>;
    /// `cancel.Cancel()`.
    fn cancel(&self, vehicle: VehicleId);
}

impl FtpPort for Telemetry {
    fn start(&self, vehicle: VehicleId, request: FtpRequest) -> bool {
        self.ftp_on(vehicle, request)
    }

    fn progress(&self, vehicle: VehicleId) -> Option<(bool, Progress)> {
        self.ftp_progress(vehicle)
    }

    fn take(&self, vehicle: VehicleId) -> Option<Result<FtpOutcome, FtpError>> {
        self.take_ftp_outcome(vehicle)
    }

    fn cancel(&self, vehicle: VehicleId) {
        self.cancel_ftp(vehicle);
    }
}

// ---------------------------------------------------------------------------------------------
// The tree and the list.
// ---------------------------------------------------------------------------------------------

/// A `TreeNode` and the `DirectoryInfo` in its `Tag`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Node {
    /// `Text`.
    pub text: String,
    /// The tag's `FullPath`: the directory as the vehicle names it.
    pub path: String,
    /// The tag's `ModifiedUtc`.
    pub modified: Option<u32>,
    /// The tag's `cache`: its last listing, which `GetFiles` reads.
    pub cache: Option<Vec<FtpFileInfo>>,
    /// `Nodes`.
    pub children: Vec<Node>,
    /// Whether it is expanded.
    pub expanded: bool,
}

impl Node {
    /// `new TreeNode(text) { Tag = new DirectoryInfo(path, _mavftp, modified) }`.
    #[must_use]
    pub fn new(text: &str, path: &str, modified: Option<u32>) -> Self {
        Self {
            text: text.to_owned(),
            path: path.to_owned(),
            modified,
            cache: None,
            children: Vec::new(),
            expanded: false,
        }
    }
}

/// The nodes as the tree draws them, depth first: each with its depth.
fn visible(nodes: &[Node], depth: usize, out: &mut Vec<(usize, Node)>) {
    for node in nodes {
        out.push((depth, node.clone()));
        if node.expanded {
            visible(&node.children, depth + 1, out);
        }
    }
}

/// A node by its tag's path, anywhere in the tree.
fn find_mut<'a>(nodes: &'a mut [Node], path: &str) -> Option<&'a mut Node> {
    for node in nodes {
        if node.path == path {
            return Some(node);
        }
        if let Some(found) = find_mut(&mut node.children, path) {
            return Some(found);
        }
    }
    None
}

/// A node by its tag's path.
fn find<'a>(nodes: &'a [Node], path: &str) -> Option<&'a Node> {
    for node in nodes {
        if node.path == path {
            return Some(node);
        }
        if let Some(found) = find(&node.children, path) {
            return Some(found);
        }
    }
    None
}

/// `TreeNode.FullPath` with `PathSeparator` "/": the texts from the root down, joined.
fn full_path(nodes: &[Node], path: &str) -> Option<String> {
    for node in nodes {
        if node.path == path {
            return Some(node.text.clone());
        }
        if let Some(below) = full_path(&node.children, path) {
            return Some(format!("{}/{below}", node.text));
        }
    }
    None
}

/// A `ListViewItem`: the name, its three sub-items, and the directory it was listed from - the
/// `Tag`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Item {
    /// `Text`.
    pub name: String,
    /// Whether it is a directory: "Directory" in Type, else "File".
    pub directory: bool,
    /// Size, `ToSizeUnits`, blank for a directory.
    pub size: String,
    /// Date modified.
    pub modified: String,
    /// The tag's `FullName`: the directory listed.
    pub dir: String,
}

impl Item {
    /// A column's text: `SubItems[column].Text`.
    #[must_use]
    pub fn column(&self, column: usize) -> &str {
        match column {
            0 => &self.name,
            1 => {
                if self.directory {
                    "Directory"
                } else {
                    "File"
                }
            }
            2 => &self.size,
            _ => &self.modified,
        }
    }
}

/// The list's columns: `Text` and `Width`.
/// `// C#: Controls/MavFTPUI.Designer.cs (columnHeaderName..columnHeaderModified)`
pub const COLUMNS: [(&str, f32); 4] = [
    ("Name", 83.0),
    ("Type", 60.0),
    ("Size", 60.0),
    ("Date modified", 120.0),
];

/// `ListView1_ColumnClick`'s comparer: two texts of digits alone - an empty one counts - as
/// numbers, else as strings; reversed when descending.
/// `// C#: Controls/MavFTPUI.cs:307-321`
#[must_use]
pub fn compare(a: &str, b: &str, descending: bool) -> Ordering {
    let digits = |text: &str| text.chars().all(|c| c.is_ascii_digit());
    let order = if digits(a) && digits(b) {
        let number = |text: &str| format!("0{text}").parse::<f64>().unwrap_or(0.0);
        number(a).partial_cmp(&number(b)).unwrap_or(Ordering::Equal)
    } else {
        culture_cmp(a, b)
    };
    if descending { order.reverse() } else { order }
}

/// The context menu's items, in `contextMenuStrip1`'s order.
/// `// C#: Controls/MavFTPUI.Designer.cs (contextMenuStrip1.Items.AddRange)`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Menu {
    /// "Download Burst".
    DownloadBurst,
    /// "Download".
    Download,
    /// "Upload".
    Upload,
    /// "&Delete".
    Delete,
    /// "&Rename".
    Rename,
    /// "New Folder".
    NewFolder,
    /// "GetCRC32".
    Crc,
}

impl Menu {
    /// The seven.
    pub const ALL: [Self; 7] = [
        Self::DownloadBurst,
        Self::Download,
        Self::Upload,
        Self::Delete,
        Self::Rename,
        Self::NewFolder,
        Self::Crc,
    ];

    /// Its `Text`, the mnemonic's `&` dropped.
    /// `// C#: Controls/MavFTPUI.Designer.cs (*ToolStripMenuItem.Text)`
    #[must_use]
    pub const fn text(self) -> &'static str {
        match self {
            Self::DownloadBurst => "Download Burst",
            Self::Download => "Download",
            Self::Upload => "Upload",
            Self::Delete => "Delete",
            Self::Rename => "Rename",
            Self::NewFolder => "New Folder",
            Self::Crc => "GetCRC32",
        }
    }

    /// Its id.
    #[must_use]
    pub const fn id(self) -> &'static str {
        match self {
            Self::DownloadBurst => "mavftp-menu-downloadburst",
            Self::Download => "mavftp-menu-download",
            Self::Upload => "mavftp-menu-upload",
            Self::Delete => "mavftp-menu-delete",
            Self::Rename => "mavftp-menu-rename",
            Self::NewFolder => "mavftp-menu-newfolder",
            Self::Crc => "mavftp-menu-crc",
        }
    }
}

// ---------------------------------------------------------------------------------------------
// What the handlers do, in order.
// ---------------------------------------------------------------------------------------------

/// What happens with a request's outcome.
#[derive(Debug, Clone, PartialEq, Eq)]
enum Then {
    /// `PopulateTreeView`'s root: `new TreeNode(text)` over the listing's directories.
    Root { text: String, path: String },
    /// `NodeMouseClick`'s `GetDirectories`, for the node with this tag.
    Listed { node: String },
    /// `GetFiles`' own listing, when the node's first failed.
    Files { node: String },
    /// A download: the file written into the folder under the name.
    Downloaded { folder: PathBuf, name: String },
    /// `UploadFile`, then "Calc CRC".
    Uploaded,
    /// `kCmdCalcFileCRC32` of an upload, against the file's.
    CrcCheck { local: u32 },
    /// `kCmdRemoveFile`.
    Removed { name: String },
    /// `kCmdCreateDirectory`.
    Created,
    /// `kCmdRename`, `kCmdResetSessions`: nothing is done with them.
    Nothing,
    /// GetCRC32's `kCmdCalcFileCRC32`.
    Crc,
}

/// A request and what its outcome does.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Call {
    request: FtpRequest,
    then: Then,
}

/// What a handler's `DoWork` or `await` runs on the page's `MAVFtp`.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Work {
    /// Behind a `ProgressReporterDialogue`.
    dialog: bool,
    /// `_mavftp.Progress += progress`: the window shows the reports.
    subscribed: bool,
    /// Whether what comes after waits for it: `ShowDialog` and `await` do; a call of an
    /// `async void` - `TreeView1_NodeMouseClick` from a handler - does not.
    awaited: bool,
    /// The requests, in order; an exception stops them.
    calls: VecDeque<Call>,
}

impl Work {
    /// A command behind the progress window.
    fn dialog(subscribed: bool, calls: impl IntoIterator<Item = Call>) -> Self {
        Self {
            dialog: true,
            subscribed,
            awaited: true,
            calls: calls.into_iter().collect(),
        }
    }

    /// A listing, awaited or not.
    fn listing(awaited: bool, call: Call) -> Self {
        Self {
            dialog: false,
            subscribed: false,
            awaited,
            calls: VecDeque::from([call]),
        }
    }
}

/// One thing a handler does, in its order.
#[derive(Debug, Clone, PartialEq, Eq)]
enum Step {
    /// `toolStripStatusLabel1.Text = ...`.
    Status(String),
    /// `toolStripProgressBar1.ProgressBar.Style`: marquee when true.
    Marquee(bool),
    /// `treeView1.Enabled`.
    TreeEnabled(bool),
    /// `treeView1.Nodes.Clear()`.
    ClearTree,
    /// `treeView1.SelectedNode = rootNode`: the last root made.
    SelectLastRoot,
    /// `TreeView1_NodeMouseClick(SelectedNode)`: the list cleared and the node listed.
    Click,
    /// Requests on the link.
    Work(Work),
    /// GetCRC32's box, once its window has closed.
    CrcBox { name: String },
}

/// The progress window.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Dialog {
    /// When it opened, for the marquee.
    pub started: Instant,
    /// `lblProgressMessage.Text`.
    pub text: String,
    /// `progressBar1`.
    pub bar: Bar,
    /// Cancel was pressed: "Cancelling...", the Cancel gone.
    pub cancelling: bool,
}

/// A request running on the link.
#[derive(Debug)]
struct Running {
    work: Work,
    then: Then,
    dialog: Option<Dialog>,
}

/// A box with a typed answer over the page.
#[derive(Debug)]
pub enum Prompt {
    /// New Folder's `InputBox`.
    NewFolder(InputBox),
    /// "Mount as Drive"'s.
    Mount(InputBox),
    /// Download's `FolderBrowserDialog`: the folder, and whether it is Download Burst.
    Folder(InputBox, bool),
    /// Upload's `OpenFileDialog`.
    Open(InputBox),
}

impl Prompt {
    /// Its `InputBox`.
    #[must_use]
    pub const fn input(&self) -> &InputBox {
        match self {
            Self::NewFolder(input)
            | Self::Mount(input)
            | Self::Folder(input, _)
            | Self::Open(input) => input,
        }
    }

    const fn input_mut(&mut self) -> &mut InputBox {
        match self {
            Self::NewFolder(input)
            | Self::Mount(input)
            | Self::Folder(input, _)
            | Self::Open(input) => input,
        }
    }
}

// ---------------------------------------------------------------------------------------------
// The page object.
// ---------------------------------------------------------------------------------------------

/// The page object.
#[derive(Debug)]
pub struct MavFtp {
    made_for: Option<Key>,
    active: bool,
    /// The vehicle `new MAVFtp(_mav, sysidcurrent, compidcurrent)` talks to.
    vehicle: Option<VehicleId>,
    /// `treeView1.Nodes`.
    tree: Vec<Node>,
    /// `treeView1.Enabled`.
    tree_enabled: bool,
    /// `SelectedNode`, by its tag's path.
    selected_node: Option<String>,
    /// The last root `PopulateTreeView` made, which it selects.
    last_root: Option<String>,
    /// `listView1.Items`.
    items: Vec<Item>,
    /// `SelectedItems`, by index.
    selected: BTreeSet<usize>,
    /// `Sorting` and the column the sorter reads, once a header is clicked.
    sort: Option<(usize, bool)>,
    /// `toolStripStatusLabel1.Text`.
    status: String,
    /// `toolStripProgressBar1`: its `Value`, and its marquee style.
    bar_value: i32,
    /// Its style.
    bar_marquee: bool,
    /// When the next report may reach the strip.
    next_update: Option<Instant>,
    /// The report last seen, so each is taken once.
    last_report: Option<Progress>,
    /// The context menu, while it is open: where.
    menu: Option<(f32, f32)>,
    /// A row's name being edited: its index and the text.
    renaming: Option<(usize, TextField)>,
    /// The box asking for a name, a folder or a file.
    prompt: Option<Prompt>,
    /// What the handlers have yet to do.
    steps: VecDeque<Step>,
    /// The request on the link.
    running: Option<Running>,
    /// The last `kCmdCalcFileCRC32`'s `crc32`.
    crc: u32,
    /// Message boxes, the first showing.
    messages: VecDeque<Message>,
    /// A line for the status line, from what the C# boxes and the ruling does not.
    status_line: VecDeque<String>,
}

impl Default for MavFtp {
    fn default() -> Self {
        Self {
            made_for: None,
            active: false,
            vehicle: None,
            tree: Vec::new(),
            tree_enabled: true,
            selected_node: None,
            last_root: None,
            items: Vec::new(),
            selected: BTreeSet::new(),
            sort: None,
            status: STATUS_START.to_owned(),
            bar_value: 0,
            bar_marquee: false,
            next_update: None,
            last_report: None,
            menu: None,
            renaming: None,
            prompt: None,
            steps: VecDeque::new(),
            running: None,
            crc: 0,
            messages: VecDeque::new(),
            status_line: VecDeque::new(),
        }
    }
}

impl MavFtp {
    /// Whether the page is showing.
    #[must_use]
    pub const fn is_active(&self) -> bool {
        self.active
    }

    /// The tree's roots.
    #[must_use]
    pub fn tree(&self) -> &[Node] {
        &self.tree
    }

    /// Whether the tree takes clicks.
    #[must_use]
    pub const fn tree_enabled(&self) -> bool {
        self.tree_enabled
    }

    /// The selected node's tag path.
    #[must_use]
    pub fn selected_node(&self) -> Option<&str> {
        self.selected_node.as_deref()
    }

    /// `SelectedNode.FullPath`.
    #[must_use]
    pub fn selected_full_path(&self) -> Option<String> {
        full_path(&self.tree, self.selected_node.as_deref()?)
    }

    /// The list's rows, in their order.
    #[must_use]
    pub fn items(&self) -> &[Item] {
        &self.items
    }

    /// The selected rows' indices.
    #[must_use]
    pub const fn selection(&self) -> &BTreeSet<usize> {
        &self.selected
    }

    /// The status strip's text.
    #[must_use]
    pub fn status(&self) -> &str {
        &self.status
    }

    /// The status strip's bar: its value, and whether it is a marquee.
    #[must_use]
    pub const fn bar(&self) -> (i32, bool) {
        (self.bar_value, self.bar_marquee)
    }

    /// Where the context menu is open.
    #[must_use]
    pub const fn menu(&self) -> Option<(f32, f32)> {
        self.menu
    }

    /// The row being renamed, and the text.
    #[must_use]
    pub fn renaming(&self) -> Option<(usize, &TextField)> {
        self.renaming.as_ref().map(|(index, field)| (*index, field))
    }

    /// The box asking, while it is open.
    #[must_use]
    pub const fn prompt(&self) -> Option<&Prompt> {
        self.prompt.as_ref()
    }

    /// The progress window, while one is open.
    #[must_use]
    pub fn dialog(&self) -> Option<&Dialog> {
        self.running
            .as_ref()
            .and_then(|running| running.dialog.as_ref())
    }

    /// Whether a command runs, or waits to.
    #[must_use]
    pub fn busy(&self) -> bool {
        self.running.is_some()
            || self
                .steps
                .iter()
                .any(|step| matches!(step, Step::Work(_) | Step::Click))
    }

    /// The message box showing.
    #[must_use]
    pub fn message(&self) -> Option<&Message> {
        self.messages.front()
    }

    /// Dismisses it.
    pub fn dismiss_message(&mut self) {
        self.messages.pop_front();
    }

    /// A line for the application's status line, once.
    pub fn take_status_line(&mut self) -> Option<String> {
        self.status_line.pop_front()
    }

    /// Shows the page: a new page object for a new screen, whose `Load` lists the tree.
    /// `MavFTPUI` is not `IActivate`: showing it again does nothing more.
    /// `// C#: Controls/MavFTPUI.cs:29-71, 665-668`
    pub fn activate(&mut self, vehicle: Option<VehicleId>, key: Key) {
        if self.made_for != Some(key) {
            let messages = std::mem::take(&mut self.messages);
            let status_line = std::mem::take(&mut self.status_line);
            *self = Self {
                made_for: Some(key),
                vehicle,
                messages,
                status_line,
                ..Self::default()
            };
            self.populate_tree_view();
        }
        self.active = true;
    }

    /// The page hidden: a rename being typed is taken, as the edit box losing the focus takes it,
    /// and the menu closes.
    pub fn hide(&mut self) {
        self.commit_rename();
        self.active = false;
        self.menu = None;
    }

    /// `PopulateTreeView`: `/` and `@SYS/` listed, each a root; the last selected and listed.
    /// `// C#: Controls/MavFTPUI.cs:73-125`
    fn populate_tree_view(&mut self) {
        self.steps.extend([
            Step::Status(UPDATING_FOLDERS.to_owned()),
            Step::Marquee(true),
            Step::TreeEnabled(false),
            Step::ClearTree,
            // `new DirectoryInfo(@"/", _mavftp)`, named `info.Name`: `GetFileName("/")`, "".
            Step::Work(Work::listing(
                true,
                Call {
                    request: FtpRequest::List {
                        path: "/".to_owned(),
                    },
                    then: Then::Root {
                        text: file_name("/").to_owned(),
                        path: "/".to_owned(),
                    },
                },
            )),
            Step::Work(Work::listing(
                true,
                Call {
                    request: FtpRequest::List {
                        path: "@SYS/".to_owned(),
                    },
                    then: Then::Root {
                        text: "@SYS".to_owned(),
                        path: "@SYS/".to_owned(),
                    },
                },
            )),
            Step::Status(READY.to_owned()),
            Step::TreeEnabled(true),
            Step::Marquee(false),
            Step::SelectLastRoot,
            Step::Click,
        ]);
    }

    /// A node clicked: selected, and `TreeView1_NodeMouseClick` for it.
    /// `// C#: Controls/MavFTPUI.cs:151-203`
    pub fn click_node(&mut self, path: &str) {
        self.close_menu_and_rename();
        if !self.tree_enabled || find(&self.tree, path).is_none() {
            return;
        }
        self.selected_node = Some(path.to_owned());
        self.steps.push_back(Step::Click);
    }

    /// A node's plus or minus: expanded or collapsed.
    pub fn toggle_node(&mut self, path: &str) {
        self.close_menu_and_rename();
        if !self.tree_enabled {
            return;
        }
        if let Some(node) = find_mut(&mut self.tree, path) {
            node.expanded = !node.expanded;
        }
    }

    /// A row clicked: it alone selected, or with Control, toggled in the selection.
    pub fn click_item(&mut self, index: usize, control: bool) {
        self.close_menu_and_rename();
        if index >= self.items.len() {
            return;
        }
        if control {
            if !self.selected.remove(&index) {
                self.selected.insert(index);
            }
        } else {
            self.selected.clear();
            self.selected.insert(index);
        }
    }

    /// `ListView1_MouseDoubleClick`: the selected node expanded, and the child named as the first
    /// selected row selected and listed.
    /// `// C#: Controls/MavFTPUI.cs:587-605`
    pub fn double_click(&mut self) {
        let Some(first) = self
            .selected
            .first()
            .and_then(|index| self.items.get(*index))
        else {
            return;
        };
        let name = first.name.clone();
        let Some(selected) = self.selected_node.clone() else {
            return;
        };
        let Some(node) = find_mut(&mut self.tree, &selected) else {
            return;
        };
        node.expanded = true;
        let child = node
            .children
            .iter()
            .find(|child| child.text == name)
            .map(|child| child.path.clone());
        if let Some(child) = child {
            self.selected_node = Some(child);
            self.steps.push_back(Step::Click);
        }
    }

    /// A column header clicked: the order flipped - from `None`, which is not `Descending`, to
    /// `Descending` first - and the rows sorted by it.
    /// `// C#: Controls/MavFTPUI.cs:300-322`
    pub fn click_column(&mut self, column: usize) {
        self.close_menu_and_rename();
        let descending = !matches!(self.sort, Some((_, true)));
        self.sort = Some((column, descending));
        self.sort_items();
    }

    /// The rows in the sorter's order; the selection follows its rows.
    fn sort_items(&mut self) {
        let Some((column, descending)) = self.sort else {
            return;
        };
        let mut rows: Vec<(Item, bool)> = self
            .items
            .drain(..)
            .enumerate()
            .map(|(index, item)| (item, self.selected.contains(&index)))
            .collect();
        rows.sort_by(|a, b| compare(a.0.column(column), b.0.column(column), descending));
        self.selected = rows
            .iter()
            .enumerate()
            .filter(|(_, (_, selected))| *selected)
            .map(|(index, _)| index)
            .collect();
        self.items = rows.into_iter().map(|(item, _)| item).collect();
    }

    /// A right-click on the list: the row under it selected if it is not, and the menu opened.
    pub fn open_menu(&mut self, row: Option<usize>, at: (f32, f32)) {
        self.commit_rename();
        if let Some(index) = row
            && !self.selected.contains(&index)
            && index < self.items.len()
        {
            self.selected.clear();
            self.selected.insert(index);
        }
        self.menu = Some(at);
    }

    /// The menu closed without a choice.
    pub fn close_menu(&mut self) {
        self.menu = None;
    }

    fn close_menu_and_rename(&mut self) {
        self.menu = None;
        self.commit_rename();
    }

    /// A menu item chosen.
    pub fn choose(&mut self, item: Menu) {
        self.menu = None;
        match item {
            Menu::DownloadBurst | Menu::Download => {
                // `toolStripStatusLabel1.Text = "Download "`, then the folder asked for.
                // `// C#: Controls/MavFTPUI.cs:326-329, 609-612`
                self.status = "Download ".to_owned();
                let folder = mp_settings::user_data_directory()
                    .map(|dir| dir.display().to_string())
                    .unwrap_or_default();
                self.prompt = Some(Prompt::Folder(
                    InputBox::new(BROWSE_TITLE, "", &folder),
                    item == Menu::DownloadBurst,
                ));
            }
            Menu::Upload => {
                self.prompt = Some(Prompt::Open(InputBox::new(OPEN_TITLE, "", "")));
            }
            Menu::Delete => self.delete(),
            Menu::Rename => {
                // `listView1.SelectedItems[0].BeginEdit()`. `// C#: Controls/MavFTPUI.cs:482-485`
                match self.selected.first().copied() {
                    Some(index) => {
                        let name = self.items.get(index).map_or("", |item| item.name.as_str());
                        let mut field = TextField::new("");
                        field.set(name);
                        self.renaming = Some((index, field));
                    }
                    None => self.status_line.push_back(NO_SELECTION.to_owned()),
                }
            }
            Menu::NewFolder => {
                self.prompt = Some(Prompt::NewFolder(InputBox::new(
                    FOLDER_TITLE,
                    FOLDER_PROMPT,
                    "",
                )));
            }
            Menu::Crc => self.get_crc(),
        }
    }

    /// "Mount as Drive": unmounted, so the mount point asked for.
    /// `// C#: Controls/MavFTPUI.cs:673-697`
    pub fn press_mount(&mut self) {
        self.close_menu_and_rename();
        self.prompt = Some(Prompt::Mount(InputBox::new(
            MOUNT_TITLE,
            MOUNT_PROMPT,
            DEFAULT_MOUNT_POINT,
        )));
    }

    /// A key in the box asking.
    pub fn prompt_key(&mut self, event: &KeyDownEvent) -> bool {
        let Some(prompt) = self.prompt.as_mut() else {
            return false;
        };
        match prompt.input_mut().field.key(event) {
            KeyOutcome::Changed => {}
            KeyOutcome::Submitted => self.close_prompt(true),
            KeyOutcome::Cancelled => self.close_prompt(false),
            KeyOutcome::Ignored => return false,
        }
        true
    }

    /// Types into the box asking, for a test.
    #[cfg(test)]
    pub fn type_prompt(&mut self, text: &str) {
        if let Some(prompt) = self.prompt.as_mut() {
            prompt.input_mut().field.set(text);
        }
    }

    /// The box asking closed with OK or Cancel.
    pub fn close_prompt(&mut self, ok: bool) {
        let Some(prompt) = self.prompt.take() else {
            return;
        };
        let answer = prompt.input().field.value().to_owned();
        match prompt {
            Prompt::NewFolder(_) => self.new_folder(ok.then_some(answer)),
            Prompt::Mount(_) => {
                if ok {
                    // `MavFtpDokan.Mount` throws: no Dokan here. The C#'s box is the status line's,
                    // by the owner's ruling. `// C#: Controls/MavFTPUI.cs:698-708`
                    self.status_line.push_back(mount_failed(DOKAN_MISSING));
                }
            }
            Prompt::Folder(_, burst) => self.download(ok.then(|| PathBuf::from(answer)), burst),
            Prompt::Open(_) => {
                if ok {
                    self.upload(&[PathBuf::from(answer)]);
                }
            }
        }
    }

    /// Download and Download Burst, the folder chosen or not: each selected row read - plainly,
    /// or in bursts - behind the window, into the folder; Cancel stops at once, with the status
    /// as it was.
    /// `// C#: Controls/MavFTPUI.cs:324-379, 607-663`
    fn download(&mut self, folder: Option<PathBuf>, burst: bool) {
        let rows: Vec<Item> = self
            .selected
            .iter()
            .filter_map(|index| self.items.get(*index).cloned())
            .collect();
        let Some(folder) = folder else {
            // Download names the first row before it looks at the answer; Download Burst after.
            // `// C#: Controls/MavFTPUI.cs:330-375, 613-659`
            match rows.first() {
                None => self.steps.push_back(Step::Status(READY.to_owned())),
                Some(first) if !burst => self
                    .steps
                    .push_back(Step::Status(format!("Download {}", first.name))),
                Some(_) => {}
            }
            return;
        };
        for row in rows {
            let Some(node) = self.selected_full_path() else {
                self.status_line.push_back(NULL_REFERENCE.to_owned());
                return;
            };
            self.steps
                .push_back(Step::Status(format!("Download {}", row.name)));
            let path = format!("{node}/{}", row.name);
            self.steps.push_back(Step::Work(Work::dialog(
                true,
                [Call {
                    request: FtpRequest::Get {
                        path,
                        burst,
                        readsize: RW_SIZE,
                    },
                    then: Then::Downloaded {
                        folder: folder.clone(),
                        name: row.name.clone(),
                    },
                }],
            )));
        }
        self.steps.push_back(Step::Status(READY.to_owned()));
    }

    /// Upload, and files dropped on the list: each written into the selected directory behind
    /// the window, its CRC checked, then the directory listed again.
    /// `// C#: Controls/MavFTPUI.cs:279-298, 381-448`
    pub fn upload(&mut self, files: &[PathBuf]) {
        for file in files {
            let name = file_name(&file.to_string_lossy()).to_owned();
            self.steps.push_back(Step::Status(format!("Upload {name}")));
            let Some(node) = self.selected_full_path() else {
                self.status_line.push_back(NULL_REFERENCE.to_owned());
                continue;
            };
            let remote = format!("{node}/{name}");
            match std::fs::read(file) {
                Ok(data) => {
                    let local = crc_crc32(0, &data);
                    self.steps.push_back(Step::Work(Work::dialog(
                        true,
                        [
                            Call {
                                request: FtpRequest::Put {
                                    path: remote.clone(),
                                    data,
                                },
                                then: Then::Uploaded,
                            },
                            Call {
                                request: FtpRequest::Crc32 { path: remote },
                                then: Then::CrcCheck { local },
                            },
                        ],
                    )));
                }
                // `UploadFile` reads the file inside `DoWork`: the window's error.
                Err(error) => self.status_line.push_back(unexpected(&error.to_string())),
            }
            self.steps.push_back(Step::Status(READY.to_owned()));
        }
        self.steps.push_back(Step::Click);
    }

    /// Delete: each selected row removed behind the window, the directory listed again.
    /// `// C#: Controls/MavFTPUI.cs:450-480`
    fn delete(&mut self) {
        let rows: Vec<Item> = self
            .selected
            .iter()
            .filter_map(|index| self.items.get(*index).cloned())
            .collect();
        for row in rows {
            self.steps
                .push_back(Step::Status(format!("Delete {}", row.name)));
            // `((DirectoryInfo)Tag).FullName + "/" + Text`: from `/` that is "//name".
            self.steps.push_back(Step::Work(Work::dialog(
                false,
                [Call {
                    request: FtpRequest::RemoveFile {
                        path: format!("{}/{}", row.dir, row.name),
                    },
                    then: Then::Removed {
                        name: row.name.clone(),
                    },
                }],
            )));
        }
        self.steps.push_back(Step::Click);
        self.steps.push_back(Step::Status(READY.to_owned()));
    }

    /// A key in the rename box.
    pub fn rename_key(&mut self, event: &KeyDownEvent) -> bool {
        let Some((_, field)) = self.renaming.as_mut() else {
            return false;
        };
        match field.key(event) {
            KeyOutcome::Changed => {}
            KeyOutcome::Submitted => self.commit_rename(),
            KeyOutcome::Cancelled => self.renaming = None,
            KeyOutcome::Ignored => return false,
        }
        true
    }

    /// Types into the rename box, for a test.
    #[cfg(test)]
    pub fn type_rename(&mut self, text: &str) {
        if let Some((_, field)) = self.renaming.as_mut() {
            field.set(text);
        }
    }

    /// The edit ended with a label: `ListView1_AfterLabelEdit`, the row renamed behind the window
    /// and the directory listed again.
    /// `// C#: Controls/MavFTPUI.cs:487-513`
    pub fn commit_rename(&mut self) {
        let Some((index, field)) = self.renaming.take() else {
            return;
        };
        let label = field.value().to_owned();
        let Some(text) = self.items.get(index).map(|item| item.name.clone()) else {
            return;
        };
        let Some(node) = self.selected_full_path() else {
            self.status_line.push_back(NULL_REFERENCE.to_owned());
            return;
        };
        self.steps.push_back(Step::Work(Work::dialog(
            false,
            [Call {
                request: FtpRequest::Rename {
                    from: format!("{node}/{text}"),
                    to: format!("{node}/{label}"),
                },
                then: Then::Nothing,
            }],
        )));
        self.steps.push_back(Step::Click);
        self.steps.push_back(Step::Status(READY.to_owned()));
    }

    /// New Folder answered: with OK the directory made behind the window; either way the
    /// directory listed again.
    /// `// C#: Controls/MavFTPUI.cs:515-546`
    fn new_folder(&mut self, folder: Option<String>) {
        if let Some(folder) = folder {
            let Some(node) = self.selected_full_path() else {
                self.status_line.push_back(NULL_REFERENCE.to_owned());
                return;
            };
            self.steps.push_back(Step::Work(Work::dialog(
                false,
                [Call {
                    request: FtpRequest::CreateDirectory {
                        path: format!("{node}/{folder}"),
                    },
                    then: Then::Created,
                }],
            )));
        }
        self.steps.push_back(Step::Click);
        self.steps.push_back(Step::Status(READY.to_owned()));
    }

    /// GetCRC32: the first selected row's CRC behind the window, then the box.
    /// `// C#: Controls/MavFTPUI.cs:548-572`
    fn get_crc(&mut self) {
        let Some(name) = self
            .selected
            .first()
            .and_then(|index| self.items.get(*index))
            .map(|item| item.name.clone())
        else {
            self.status_line.push_back(NO_SELECTION.to_owned());
            return;
        };
        let Some(node) = self.selected_full_path() else {
            self.status_line.push_back(NULL_REFERENCE.to_owned());
            return;
        };
        self.steps.push_back(Step::Work(Work::dialog(
            false,
            [Call {
                request: FtpRequest::Crc32 {
                    path: format!("{node}/{name}"),
                },
                then: Then::Crc,
            }],
        )));
        self.steps.push_back(Step::CrcBox { name });
    }

    /// The window's Cancel: "Cancelling...", a marquee, the Cancel gone, and the command asked to
    /// stop.
    /// `// C#: ExtLibs/Controls/ProgressReporterDialogue.cs:254-268; Controls/MavFTPUI.cs:338-343`
    pub fn cancel<P: FtpPort>(&mut self, port: &P) {
        let Some(vehicle) = self.vehicle else {
            return;
        };
        let Some(dialog) = self
            .running
            .as_mut()
            .and_then(|running| running.dialog.as_mut())
        else {
            return;
        };
        if dialog.cancelling {
            return;
        }
        dialog.cancelling = true;
        CANCELLING.clone_into(&mut dialog.text);
        dialog.bar.marquee = true;
        port.cancel(vehicle);
    }

    /// Once a frame: a page object whose screen has gone is let go; the request on the link
    /// followed; and what the handlers queued done, as far as the link lets it.
    pub fn tick<P: FtpPort>(
        &mut self,
        port: &P,
        view: &TelemetryView,
        on_config: bool,
        now: Instant,
    ) {
        if !self.active
            && self.made_for.is_some()
            && (!on_config || self.made_for != Some(Key::of(view)))
        {
            let messages = std::mem::take(&mut self.messages);
            let status_line = std::mem::take(&mut self.status_line);
            *self = Self {
                messages,
                status_line,
                ..Self::default()
            };
            return;
        }
        self.follow(port, now);
        self.run_steps(port, now);
    }

    /// The request on the link: its reports onto the strip and the window, and when it has
    /// ended, what its outcome does.
    fn follow<P: FtpPort>(&mut self, port: &P, now: Instant) {
        let (Some(vehicle), Some(_)) = (self.vehicle, self.running.as_ref()) else {
            return;
        };
        let state = port.progress(vehicle);
        if let Some((_, progress)) = &state
            && self.last_report.as_ref() != Some(progress)
        {
            self.last_report = Some(progress.clone());
            self.report(progress, now);
        }
        if matches!(state, Some((true, _))) {
            return;
        }
        let outcome = port.take(vehicle);
        self.finish(port, outcome);
    }

    /// A `Progress` report: onto the strip at most every 100 ms - the value when it has one, the
    /// text always - and into a transfer's window unless Cancel was pressed.
    /// `// C#: Controls/MavFTPUI.cs:34-65, 345-349; ExtLibs/Controls/ProgressReporterDialogue.cs:282-296`
    fn report(&mut self, progress: &Progress, now: Instant) {
        if self.next_update.is_none_or(|next| now >= next) {
            self.next_update = Some(now + REPORT_EVERY);
            if progress.percent >= 0 {
                self.bar_value = progress.percent;
            }
            self.status.clone_from(&progress.message);
        }
        let status = self.status.clone();
        if let Some(running) = self.running.as_mut()
            && running.work.subscribed
            && let Some(dialog) = running.dialog.as_mut()
            && !dialog.cancelling
        {
            dialog.text = status;
            dialog.bar.report(progress.percent);
        }
    }

    /// The request ended: what its outcome does, then the next request of its command, or the
    /// command's end.
    fn finish<P: FtpPort>(&mut self, port: &P, outcome: Option<Result<FtpOutcome, FtpError>>) {
        let Some(mut running) = self.running.take() else {
            return;
        };
        let cancelled = running
            .dialog
            .as_ref()
            .is_some_and(|dialog| dialog.cancelling);
        let then = running.then.clone();
        // An outcome that never came is the link gone from under the command.
        let outcome = outcome.ok_or_else(|| "the link has closed".to_owned());
        let result = self.land(then, outcome, cancelled, &mut running);
        match result {
            Landed::Next => {
                if let Some(call) = running.work.calls.pop_front() {
                    running.then = call.then.clone();
                    if self.start_call(port, call.request) {
                        self.running = Some(running);
                        return;
                    }
                    self.status_line
                        .push_back(unexpected("the link would not take the request"));
                } else if cancelled {
                    // `ShowDoneWithError(null, "User Cancel")`: nothing acknowledged it.
                    self.status_line.push_back(USER_CANCEL.to_owned());
                }
            }
            Landed::Threw(message) => {
                self.status_line.push_back(if cancelled {
                    USER_CANCEL.to_owned()
                } else {
                    unexpected(&message)
                });
            }
            Landed::Acknowledged => {}
        }
        if cancelled {
            // `_mavftp.kCmdResetSessions()`, once the command has stopped.
            self.steps.push_front(Step::Work(Work::listing(
                false,
                Call {
                    request: FtpRequest::ResetSessions,
                    then: Then::Nothing,
                },
            )));
        }
    }

    /// What an outcome does.
    fn land(
        &mut self,
        then: Then,
        outcome: Result<Result<FtpOutcome, FtpError>, String>,
        cancelled: bool,
        running: &mut Running,
    ) -> Landed {
        let outcome = match outcome {
            Ok(Ok(outcome)) => Ok(outcome),
            Ok(Err(error)) => Err(error.to_string()),
            Err(gone) => Err(gone),
        };
        match then {
            Then::Root { text, path } => {
                let mut node = Node::new(&text, &path, None);
                if let Ok(FtpOutcome::Listing { entries, .. }) = &outcome {
                    node.children = directories(entries);
                    node.cache = Some(entries.clone());
                }
                self.last_root = Some(path);
                self.tree.push(node);
                Landed::Next
            }
            Then::Listed { node } => {
                let entries = match outcome {
                    Ok(FtpOutcome::Listing { entries, .. }) => Some(entries),
                    _ => None,
                };
                self.listed(&node, entries);
                Landed::Next
            }
            Then::Files { node } => {
                let entries = match outcome {
                    Ok(FtpOutcome::Listing { entries, .. }) => Some(entries),
                    _ => None,
                };
                if let Some(entries) = &entries
                    && let Some(tag) = find_mut(&mut self.tree, &node)
                {
                    tag.cache = Some(entries.clone());
                }
                self.add_files(&node, entries.as_deref().unwrap_or(&[]));
                Landed::Next
            }
            Then::Downloaded { folder, name } => {
                if cancelled {
                    return Landed::Acknowledged;
                }
                match outcome {
                    Ok(FtpOutcome::File {
                        data: Some(data), ..
                    }) => match write_unused(&folder, &name, &data) {
                        Ok(()) => Landed::Next,
                        Err(error) => Landed::Threw(error.to_string()),
                    },
                    // `ms.ToArray()` on the null `GetFile` returned.
                    Ok(_) => Landed::Threw(NULL_REFERENCE.to_owned()),
                    Err(error) => Landed::Threw(error),
                }
            }
            Then::Uploaded => {
                if cancelled {
                    return Landed::Acknowledged;
                }
                match outcome {
                    Ok(_) => {
                        // `prd.UpdateProgressAndStatus(-1, "Calc CRC")`.
                        if let Some(dialog) = running.dialog.as_mut() {
                            "Calc CRC".clone_into(&mut dialog.text);
                            dialog.bar.report(-1);
                        }
                        Landed::Next
                    }
                    Err(error) => Landed::Threw(error),
                }
            }
            Then::CrcCheck { local } => match outcome {
                Ok(FtpOutcome::Crc32 { crc, .. }) if crc == local => Landed::Next,
                Ok(_) => Landed::Threw(BAD_CRC.to_owned()),
                Err(error) => Landed::Threw(error),
            },
            Then::Removed { name } => match outcome {
                Ok(FtpOutcome::Done(true)) => Landed::Next,
                Ok(_) => {
                    // `CustomMessageBox.Show("Failed to delete file", text)`: the status line.
                    self.status_line
                        .push_back(format!("{FAILED_DELETE}: {name}"));
                    Landed::Next
                }
                Err(error) => Landed::Threw(error),
            },
            Then::Created => match outcome {
                Ok(FtpOutcome::Done(true)) => Landed::Next,
                Ok(_) => {
                    self.status_line.push_back(FAILED_DIRECTORY.to_owned());
                    Landed::Next
                }
                Err(error) => Landed::Threw(error),
            },
            Then::Crc => match outcome {
                Ok(FtpOutcome::Crc32 { crc, .. }) => {
                    self.crc = crc;
                    Landed::Next
                }
                Ok(_) => Landed::Next,
                Err(error) => {
                    // `crc32 = UInt32.MaxValue` before anything can throw.
                    self.crc = u32::MAX;
                    Landed::Threw(error)
                }
            },
            Then::Nothing => match outcome {
                Ok(_) => Landed::Next,
                Err(error) => Landed::Threw(error),
            },
        }
    }

    /// `NodeMouseClick` after `GetDirectories`: the node's children made again from the listing's
    /// directories, the rows the directories, then the files - from the node's listing, or a
    /// listing of its own when the node has none.
    /// `// C#: Controls/MavFTPUI.cs:157-203, 235-276`
    fn listed(&mut self, path: &str, entries: Option<Vec<FtpFileInfo>>) {
        let Some(node) = find_mut(&mut self.tree, path) else {
            return;
        };
        let dirs = entries.as_deref().map(directories).unwrap_or_default();
        if let Some(entries) = entries {
            node.cache = Some(entries);
        }
        node.children.clone_from(&dirs);
        let has_cache = node.cache.is_some();
        for dir in &dirs {
            self.items.push(Item {
                name: dir.text.clone(),
                directory: true,
                size: String::new(),
                modified: modified_string(dir.modified),
                dir: path.to_owned(),
            });
        }
        if has_cache {
            let files = find(&self.tree, path)
                .and_then(|node| node.cache.clone())
                .unwrap_or_default();
            self.add_files(path, &files);
        } else {
            // `if (cache == null) await GetDirectories()`.
            self.steps.push_front(Step::Work(Work::listing(
                false,
                Call {
                    request: FtpRequest::List {
                        path: path.to_owned(),
                    },
                    then: Then::Files {
                        node: path.to_owned(),
                    },
                },
            )));
        }
    }

    /// `GetFiles`' rows: every entry that is not a directory.
    fn add_files(&mut self, path: &str, entries: &[FtpFileInfo]) {
        for file in entries.iter().filter(|entry| !entry.is_directory) {
            self.items.push(Item {
                name: file.name.clone(),
                directory: false,
                size: size_units(file.size),
                modified: modified_string(file.modified_utc),
                dir: path.to_owned(),
            });
        }
        self.sort_items();
    }

    /// Starts a request on the vehicle.
    fn start_call<P: FtpPort>(&mut self, port: &P, request: FtpRequest) -> bool {
        let Some(vehicle) = self.vehicle else {
            return false;
        };
        port.start(vehicle, request)
    }

    /// What the handlers queued, in order, up to a command that must end first.
    fn run_steps<P: FtpPort>(&mut self, port: &P, now: Instant) {
        loop {
            if self
                .running
                .as_ref()
                .is_some_and(|running| running.work.awaited)
            {
                return;
            }
            let Some(step) = self.steps.front() else {
                return;
            };
            if matches!(step, Step::Work(_) | Step::Click) && self.running.is_some() {
                return;
            }
            // A command another page runs on the vehicle is waited for.
            if matches!(step, Step::Work(_) | Step::Click)
                && let Some(vehicle) = self.vehicle
                && matches!(port.progress(vehicle), Some((true, _)))
            {
                return;
            }
            let Some(step) = self.steps.pop_front() else {
                return;
            };
            match step {
                Step::Status(text) => self.status = text,
                Step::Marquee(on) => self.bar_marquee = on,
                Step::TreeEnabled(on) => self.tree_enabled = on,
                Step::ClearTree => self.tree.clear(),
                Step::SelectLastRoot => self.selected_node.clone_from(&self.last_root),
                Step::Click => {
                    // `listView1.Items.Clear()`, then the node's `GetDirectories`.
                    self.items.clear();
                    self.selected.clear();
                    self.renaming = None;
                    let Some(node) = self.selected_node.clone() else {
                        continue;
                    };
                    self.start_work(
                        port,
                        Work::listing(
                            false,
                            Call {
                                request: FtpRequest::List { path: node.clone() },
                                then: Then::Listed { node },
                            },
                        ),
                        now,
                    );
                }
                Step::Work(work) => self.start_work(port, work, now),
                Step::CrcBox { name } => {
                    self.messages
                        .push_back(plain(format!("{name}: 0x{:X}", self.crc)));
                }
            }
        }
    }

    /// A command's first request started, behind its window if it has one.
    fn start_work<P: FtpPort>(&mut self, port: &P, mut work: Work, now: Instant) {
        let Some(call) = work.calls.pop_front() else {
            return;
        };
        if !self.start_call(port, call.request) {
            if work.dialog {
                self.status_line
                    .push_back(unexpected("the link would not take the request"));
            }
            return;
        }
        let dialog = work.dialog.then(|| Dialog {
            started: now,
            text: String::new(),
            bar: Bar::default(),
            cancelling: false,
        });
        self.running = Some(Running {
            work,
            then: call.then,
            dialog,
        });
    }
}

/// How an outcome left its command.
enum Landed {
    /// On to its next request.
    Next,
    /// It threw: the command stops, with the window's error.
    Threw(String),
    /// A cancel it acknowledged: the command stops quietly.
    Acknowledged,
}

/// `GetDirectories`: a listing's directories, not `.`, `..` or the nameless placeholders, each a
/// node named `Path.GetFileName` of its full name.
/// `// C#: Controls/MavFTPUI.cs:235-258, 127-140`
fn directories(entries: &[FtpFileInfo]) -> Vec<Node> {
    entries
        .iter()
        .filter(|entry| entry.is_directory && !matches!(entry.name.as_str(), "." | ".." | ""))
        .map(|entry| {
            Node::new(
                file_name(&entry.full_path),
                &entry.full_path,
                entry.modified_utc,
            )
        })
        .collect()
}

/// `Path.Combine(folder, name)`, numbered until it is not taken - `file + a++` - and the bytes
/// written there.
/// `// C#: Controls/MavFTPUI.cs:361-367`
fn write_unused(folder: &Path, name: &str, data: &[u8]) -> std::io::Result<()> {
    let base = folder.join(name);
    let mut file = base.clone();
    let mut a = 0_u32;
    while file.exists() {
        file = PathBuf::from(format!("{}{a}", base.display()));
        a += 1;
    }
    std::fs::write(file, data)
}

// ---------------------------------------------------------------------------------------------
// Facts.
// ---------------------------------------------------------------------------------------------

/// Facts a UI test asserts on.
pub fn record_facts(page: &MavFtp) {
    use crate::facts::record;
    record("config.mavftp.active", page.is_active());
    record("config.mavftp.status", page.status());
    let (value, marquee) = page.bar();
    record(
        "config.mavftp.bar",
        if marquee {
            "marquee".to_owned()
        } else {
            value.to_string()
        },
    );
    let mut nodes = Vec::new();
    visible(page.tree(), 0, &mut nodes);
    record(
        "config.mavftp.tree",
        nodes
            .iter()
            .map(|(_, node)| node.path.as_str())
            .collect::<Vec<_>>()
            .join(","),
    );
    record("config.mavftp.tree.enabled", page.tree_enabled());
    record(
        "config.mavftp.selected",
        page.selected_node().unwrap_or("none"),
    );
    record(
        "config.mavftp.list",
        page.items()
            .iter()
            .map(|item| item.name.as_str())
            .collect::<Vec<_>>()
            .join(","),
    );
    record("config.mavftp.list.count", page.items().len());
    for item in page.items() {
        let key = format!("config.mavftp.item.{}", item.name);
        record(format!("{key}.type"), item.column(1));
        record(format!("{key}.size"), &item.size);
    }
    record(
        "config.mavftp.selection",
        page.selection()
            .iter()
            .filter_map(|index| page.items().get(*index))
            .map(|item| item.name.as_str())
            .collect::<Vec<_>>()
            .join(","),
    );
    record("config.mavftp.menu", page.menu().is_some());
    record(
        "config.mavftp.prompt",
        page.prompt().map_or("none", |prompt| prompt.input().title),
    );
    record(
        "config.mavftp.renaming",
        page.renaming()
            .map_or_else(|| "none".to_owned(), |(_, field)| field.value().to_owned()),
    );
    record(
        "config.mavftp.dialog",
        page.dialog().map_or("none", |dialog| {
            if dialog.text.is_empty() {
                "open"
            } else {
                dialog.text.as_str()
            }
        }),
    );
    record("config.mavftp.busy", page.busy());
    record(
        "config.mavftp.message",
        page.message()
            .map_or("none", |message| message.text.as_str()),
    );
}

// ---------------------------------------------------------------------------------------------
// Drawing.
// ---------------------------------------------------------------------------------------------

/// `$this.Size`.
/// `// C#: Controls/MavFTPUI.Designer.cs (this.Size)`
const PAGE_SIZE: (f32, f32) = (888.0, 541.0);
/// `splitContainer1.SplitterDistance`, and WinForms' four-pixel splitter.
const SPLITTER: (f32, f32) = (208.0, 4.0);
/// `statusStrip1`'s height.
const STATUS_HEIGHT: f32 = 22.0;
/// `btnMountFuse`: anchored to the top right, 125 by 23.
/// `// C#: Controls/MavFTPUI.Designer.cs (btnMountFuse.Location, .Size)`
const MOUNT: (f32, f32, f32, f32) = (763.0, 0.0, 125.0, 23.0);
/// `toolStripProgressBar1.Size`.
const STRIP_BAR: (f32, f32) = (100.0, 16.0);
/// A tree row, and its indent.
const TREE_ROW: f32 = 16.0;
/// `TreeView.Indent`'s default.
const TREE_INDENT: f32 = 19.0;
/// A list row, and the header.
const LIST_ROW: f32 = 17.0;
/// A text's width at six pixels a character, for the columns' `AutoResizeColumns`.
const CHAR: f32 = 6.0;
/// A menu row, and the menu's width.
/// `// C#: Controls/MavFTPUI.Designer.cs (contextMenuStrip1.Size, *ToolStripMenuItem.Size)`
const MENU_ROW: (f32, f32) = (133.0, 22.0);

/// `AutoResizeColumns(HeaderSize)`: each column as wide as its header or its widest text, the
/// last filling the rest.
fn column_widths(items: &[Item], width: f32) -> [f32; 4] {
    let mut widths = [0.0_f32; 4];
    for (column, (header, _)) in COLUMNS.iter().enumerate() {
        let longest = items
            .iter()
            .map(|item| item.column(column).chars().count())
            .max()
            .unwrap_or(0)
            .max(header.chars().count());
        #[allow(clippy::cast_precision_loss)]
        let text = longest as f32 * CHAR + 12.0;
        if let Some(slot) = widths.get_mut(column) {
            *slot = text;
        }
    }
    let used: f32 = widths.iter().take(3).sum();
    if let Some(last) = widths.get_mut(3) {
        *last = last.max(width - used);
    }
    widths
}

/// A folder's image: `imageList1`'s "folder", drawn.
fn folder_icon() -> AnyElement {
    div()
        .w(px(12.0))
        .h(px(9.0))
        .flex_shrink_0()
        .rounded_sm()
        .bg(rgb(theme::WARN))
        .into_any_element()
}

/// The tree, left of the splitter.
fn tree_panel(page: &MavFtp, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let mut nodes = Vec::new();
    visible(page.tree(), 0, &mut nodes);
    let enabled = page.tree_enabled();
    let mut tree = crate::probe::measured("mavftp-tree", div())
        .id("mavftp-tree")
        .absolute()
        .left(px(0.0))
        .top(px(0.0))
        .w(px(SPLITTER.0))
        .h(px(PAGE_SIZE.1 - STATUS_HEIGHT))
        .overflow_y_scroll()
        .flex()
        .flex_col()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::BG));
    for (depth, node) in nodes {
        let id = format!("mavftp-node-{}", node.path);
        let selected = page.selected_node() == Some(node.path.as_str());
        #[allow(clippy::cast_precision_loss)]
        let indent = 4.0 + TREE_INDENT * depth as f32;
        let expander = if node.children.is_empty() {
            div().w(px(12.0)).into_any_element()
        } else {
            let path = node.path.clone();
            let name = format!("{id}-expand");
            crate::probe::measured(name.clone(), div())
                .id(SharedString::from(name))
                .w(px(12.0))
                .text_size(px(9.0))
                .text_color(rgb(theme::DIM))
                .cursor_pointer()
                .child(if node.expanded { "−" } else { "+" })
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    this.software_pages2.mavftp.toggle_node(&path);
                    cx.notify();
                }))
                .into_any_element()
        };
        let path = node.path.clone();
        let label = crate::probe::measured(id.clone(), div())
            .id(SharedString::from(id))
            .flex()
            .items_center()
            .gap_1()
            .px_1()
            .h(px(TREE_ROW))
            .text_xs()
            .whitespace_nowrap()
            .text_color(rgb(if enabled { theme::TEXT } else { theme::DIM }))
            .when(selected, |label| label.bg(rgb(theme::ACCENT)))
            .child(folder_icon())
            .child(node.text.clone())
            .when(enabled, |label| {
                label
                    .cursor_pointer()
                    .on_click(cx.listener(move |this, _event, _window, cx| {
                        this.software_pages2.mavftp.click_node(&path);
                        cx.notify();
                    }))
            });
        tree = tree.child(
            div()
                .flex()
                .items_center()
                .pl(px(indent))
                .h(px(TREE_ROW))
                .child(expander)
                .child(label),
        );
    }
    tree.into_any_element()
}

/// The list, right of the splitter: the headers, the rows, the rename box.
fn list_panel(
    page: &MavFtp,
    rename: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let left = SPLITTER.0 + SPLITTER.1;
    let width = PAGE_SIZE.0 - left;
    let widths = column_widths(page.items(), width);
    let mut header = div()
        .flex()
        .h(px(LIST_ROW))
        .border_b_1()
        .border_color(rgb(theme::BORDER));
    for (column, ((text, _), w)) in COLUMNS.iter().zip(widths).enumerate() {
        let id = format!("mavftp-col-{column}");
        header = header.child(
            crate::probe::measured(id.clone(), div())
                .id(SharedString::from(id))
                .w(px(w))
                .px_1()
                .text_xs()
                .whitespace_nowrap()
                .overflow_hidden()
                .text_color(rgb(theme::DIM))
                .border_r_1()
                .border_color(rgb(theme::BORDER))
                .cursor_pointer()
                .child(*text)
                .on_click(cx.listener(move |this, _event, _window, cx| {
                    this.software_pages2.mavftp.click_column(column);
                    cx.notify();
                })),
        );
    }
    let mut rows = div().flex().flex_col();
    let renaming = page.renaming();
    for (index, item) in page.items().iter().enumerate() {
        let id = format!("mavftp-item-{}", item.name);
        let selected = page.selection().contains(&index);
        let mut row = crate::probe::measured(id.clone(), div())
            .id(SharedString::from(id))
            .flex()
            .items_center()
            .h(px(LIST_ROW))
            .text_xs()
            .whitespace_nowrap()
            .text_color(rgb(theme::TEXT))
            .when(selected, |row| row.bg(rgb(theme::ACCENT)))
            .on_click(
                cx.listener(move |this, event: &gpui::ClickEvent, _window, cx| {
                    let page = &mut this.software_pages2.mavftp;
                    if event.click_count() == 2 {
                        page.double_click();
                    } else {
                        page.click_item(index, event.modifiers().control);
                    }
                    cx.notify();
                }),
            )
            .on_mouse_down(
                MouseButton::Right,
                cx.listener(move |this, event: &gpui::MouseDownEvent, _window, cx| {
                    let at = (f32::from(event.position.x), f32::from(event.position.y));
                    this.software_pages2.mavftp.open_menu(Some(index), at);
                    cx.stop_propagation();
                    cx.notify();
                }),
            );
        for (column, w) in widths.iter().enumerate() {
            let cell = div()
                .w(px(*w))
                .px_1()
                .overflow_hidden()
                .flex()
                .items_center();
            let cell = if column == 0 {
                match renaming {
                    Some((editing, field)) if editing == index => {
                        cell.child(crate::textfield::text_field(
                            "mavftp-rename-value",
                            field,
                            rename,
                            rename.is_focused(window),
                            px(w - 8.0),
                            cx.listener(|this, event: &KeyDownEvent, _window, cx| {
                                if this.software_pages2.mavftp.rename_key(event) {
                                    cx.notify();
                                }
                            }),
                        ))
                    }
                    _ => cell
                        .gap_1()
                        .children(item.directory.then(folder_icon))
                        .child(item.name.clone()),
                }
            } else {
                cell.child(item.column(column).to_owned())
            };
            row = row.child(cell);
        }
        rows = rows.child(row);
    }
    crate::probe::measured("mavftp-list", div())
        .id("mavftp-list")
        .absolute()
        .left(px(left))
        .top(px(0.0))
        .w(px(width))
        .h(px(PAGE_SIZE.1 - STATUS_HEIGHT))
        .overflow_y_scroll()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::BG))
        .child(header)
        .child(rows)
        .on_mouse_down(
            MouseButton::Right,
            cx.listener(|this, event: &gpui::MouseDownEvent, _window, cx| {
                let at = (f32::from(event.position.x), f32::from(event.position.y));
                this.software_pages2.mavftp.open_menu(None, at);
                cx.notify();
            }),
        )
        .on_drop(cx.listener(|this, paths: &ExternalPaths, _window, cx| {
            this.software_pages2.mavftp.upload(paths.paths());
            cx.notify();
        }))
        .into_any_element()
}

/// The status strip: the label, then the bar.
fn status_strip(page: &MavFtp) -> AnyElement {
    let (value, marquee) = page.bar();
    #[allow(clippy::cast_precision_loss)]
    let fill = if marquee {
        STRIP_BAR.0 / 3.0
    } else {
        STRIP_BAR.0 * value.clamp(0, 100) as f32 / 100.0
    };
    at(0.0, PAGE_SIZE.1 - STATUS_HEIGHT, PAGE_SIZE.0, STATUS_HEIGHT)
        .flex()
        .items_center()
        .gap_2()
        .px_1()
        .border_t_1()
        .border_color(rgb(theme::BORDER))
        .child(
            crate::probe::measured("mavftp-status", div())
                .text_xs()
                .whitespace_nowrap()
                .text_color(rgb(theme::TEXT))
                .child(page.status().to_owned()),
        )
        .child(
            crate::probe::measured("mavftp-status-bar", div())
                .relative()
                .w(px(STRIP_BAR.0))
                .h(px(STRIP_BAR.1))
                .border_1()
                .border_color(rgb(theme::BORDER))
                .child(
                    div()
                        .absolute()
                        .left(px(0.0))
                        .top(px(0.0))
                        .w(px(fill))
                        .h_full()
                        .bg(rgb(theme::OK)),
                ),
        )
        .into_any_element()
}

/// The keyboard to the box a handler opened: an `InputBox`'s text, or the name being edited.
fn take_focus(this: &MissionPlanner, window: &mut Window, cx: &mut Context<MissionPlanner>) {
    let page = &this.software_pages2.mavftp;
    if page.prompt().is_some() {
        this.software2_focus.prompt.focus(window, cx);
    } else if page.renaming().is_some() {
        this.software2_focus.rename.focus(window, cx);
    }
}

/// The context menu, where it was opened: over the page, as a `ContextMenuStrip` is its own
/// window.
fn context_menu((x, y): (f32, f32), cx: &mut Context<MissionPlanner>) -> AnyElement {
    let mut menu = crate::probe::measured("mavftp-menu", div())
        .id("mavftp-menu")
        .w(px(MENU_ROW.0))
        .flex()
        .flex_col()
        .bg(rgb(theme::PANEL))
        .border_1()
        .border_color(rgb(theme::ACCENT))
        .occlude()
        // A click anywhere else closes it, as a `ContextMenuStrip` closes.
        .on_mouse_down_out(cx.listener(|this, _event, _window, cx| {
            this.software_pages2.mavftp.close_menu();
            cx.notify();
        }));
    for item in Menu::ALL {
        menu = menu.child(
            crate::probe::measured(item.id(), div())
                .id(item.id())
                .h(px(MENU_ROW.1))
                .px_2()
                .flex()
                .items_center()
                .text_xs()
                .text_color(rgb(theme::TEXT))
                .cursor_pointer()
                .hover(|style| style.bg(rgb(theme::ACTION)))
                .child(item.text())
                .on_click(cx.listener(move |this, _event, window, cx| {
                    this.software_pages2.mavftp.choose(item);
                    take_focus(this, window, cx);
                    cx.notify();
                })),
        );
    }
    gpui::deferred(
        gpui::anchored()
            .position(gpui::point(px(x), px(y)))
            .snap_to_window()
            .child(menu),
    )
    .with_priority(1)
    .into_any_element()
}

/// The page, laid out as `MavFTPUI.Designer.cs` lays it out.
/// `// C#: Controls/MavFTPUI.Designer.cs:29-235`
pub fn page(
    page: &MavFtp,
    rename: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    if !page.is_active() {
        return div().into_any_element();
    }
    let (mx, my, mw, mh) = MOUNT;
    let body = div()
        .relative()
        .w(px(PAGE_SIZE.0))
        .h(px(PAGE_SIZE.1))
        .child(tree_panel(page, cx))
        .child(list_panel(page, rename, window, cx))
        .child(status_strip(page))
        .child(super::optional::button(
            "mavftp-mount",
            "Mount as Drive",
            (mx, my, mw, mh),
            true,
            |this, window, cx| {
                this.software_pages2.mavftp.press_mount();
                take_focus(this, window, cx);
            },
            cx,
        ))
        .children(page.menu().map(|at| context_menu(at, cx)));
    panel(TITLE, body).into_any_element()
}

/// The progress window, the box asking, or a message box, over the whole window.
pub fn overlay(
    page: &MavFtp,
    prompt: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> Option<AnyElement> {
    if let Some(dialog) = page.dialog() {
        let on_cancel = (!dialog.cancelling).then_some(|this: &mut MissionPlanner| {
            let telemetry = &this.telemetry;
            this.software_pages2.mavftp.cancel(telemetry);
        });
        return Some(progress_dialog(
            ProgressIds {
                frame: "mavftp-progress",
                bar: "mavftp-progress-bar",
                cancel: "mavftp-progress-cancel",
                backdrop: "mavftp-progress-backdrop",
            },
            &dialog.text,
            dialog.bar,
            dialog.started,
            on_cancel,
            window,
            cx,
        ));
    }
    if let Some(message) = page.message() {
        return Some(message_box(
            "mavftp-message",
            "mavftp-message-ok",
            message,
            window,
            |this| this.software_pages2.mavftp.dismiss_message(),
            cx,
        ));
    }
    let asking = page.prompt()?;
    Some(input_box(
        "mavftp-prompt-box",
        asking.input(),
        prompt,
        window,
        |this, event| this.software_pages2.mavftp.prompt_key(event),
        |this| this.software_pages2.mavftp.close_prompt(true),
        |this| this.software_pages2.mavftp.close_prompt(false),
        cx,
    ))
}

#[cfg(test)]
mod tests {
    use std::cell::RefCell;

    use mp_link::mavftp::MavFtp as Client;
    use mp_link::mavftp::testing::FakeVehicle;
    use mp_link::mavftp::wire::Header;

    use super::*;

    const VEHICLE: VehicleId = VehicleId::new(1, 1);

    /// The link's client against a vehicle in memory, each request answered to its end as it
    /// starts: the page's path through `FtpPort`, with nothing between.
    struct Bench {
        client: RefCell<Client>,
        vehicle: RefCell<FakeVehicle>,
        /// Requests left running, answered only when asked: for Cancel.
        hold: RefCell<bool>,
        started: RefCell<Vec<FtpRequest>>,
    }

    impl Bench {
        fn new(vehicle: FakeVehicle) -> Self {
            Self {
                client: RefCell::new(Client::new(
                    VEHICLE,
                    mp_link::mavftp::retry::FtpTimeouts::default(),
                )),
                vehicle: RefCell::new(vehicle),
                hold: RefCell::new(false),
                started: RefCell::new(Vec::new()),
            }
        }

        /// Passes messages both ways until nothing is left to send.
        fn pump(&self, mut sends: Vec<Header>) {
            let now = Instant::now();
            while !sends.is_empty() {
                let mut next = Vec::new();
                for request in sends {
                    let replies = self.vehicle.borrow_mut().answer(&request);
                    for reply in replies {
                        self.client
                            .borrow_mut()
                            .on_message(&reply.encode(), now, &mut next);
                    }
                }
                sends = next;
            }
        }

        /// Lets a held request's replies through again.
        fn release(&self) {
            *self.hold.borrow_mut() = false;
        }
    }

    impl FtpPort for Bench {
        fn start(&self, vehicle: VehicleId, request: FtpRequest) -> bool {
            assert_eq!(vehicle, VEHICLE);
            self.started.borrow_mut().push(request.clone());
            let mut sends = Vec::new();
            if !self
                .client
                .borrow_mut()
                .start(request, Instant::now(), &mut sends)
            {
                return false;
            }
            if !*self.hold.borrow() {
                self.pump(sends);
            }
            true
        }

        fn progress(&self, _vehicle: VehicleId) -> Option<(bool, Progress)> {
            if !*self.hold.borrow() {
                // Time passes between frames: the client's waits run out - a refusal is
                // reported then, as `kCmdRemoveFile`'s is - and what it sends again is answered.
                let mut sends = Vec::new();
                self.client
                    .borrow_mut()
                    .on_tick(Instant::now() + Duration::from_secs(120), &mut sends);
                self.pump(sends);
            }
            let client = self.client.borrow();
            Some((client.is_busy(), client.progress().clone()))
        }

        fn take(&self, _vehicle: VehicleId) -> Option<Result<FtpOutcome, FtpError>> {
            self.client.borrow_mut().take_outcome()
        }

        fn cancel(&self, _vehicle: VehicleId) {
            self.client.borrow_mut().cancel();
        }
    }

    fn key() -> Key {
        Key::of(&TelemetryView::disconnected("test"))
    }

    fn view() -> TelemetryView {
        TelemetryView::disconnected("test")
    }

    /// Ticks until nothing is left to do.
    fn settle(page: &mut MavFtp, bench: &Bench) {
        let view = view();
        let mut now = Instant::now();
        for _ in 0..200 {
            page.tick(bench, &view, true, now);
            now += REPORT_EVERY;
            if !page.busy() && page.steps.is_empty() {
                break;
            }
        }
    }

    fn vehicle() -> FakeVehicle {
        FakeVehicle::new()
            .with_file("/APM/LOGS/00000001.BIN", &[7; 3000])
            .with_file("/APM/param.pck", b"params")
            .with_dir("/APM/scripts")
            .with_file("@SYS/uarts.txt", b"SERIAL0 OTG1")
            .with_dir("@SYS/threads")
    }

    fn loaded(bench: &Bench) -> MavFtp {
        let mut page = MavFtp::default();
        page.activate(Some(VEHICLE), key());
        settle(&mut page, bench);
        page
    }

    fn names(page: &MavFtp) -> Vec<&str> {
        page.items().iter().map(|item| item.name.as_str()).collect()
    }

    fn select(page: &mut MavFtp, name: &str) {
        let index = page
            .items()
            .iter()
            .position(|item| item.name == name)
            .expect("the row");
        page.click_item(index, false);
    }

    fn scratch(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("headless-planner-mavftp-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("a scratch folder");
        dir
    }

    /// The texts the page shows are the Designer's.
    #[test]
    fn the_texts_are_the_designers() {
        let Some(designer) =
            crate::config_coverage::source::csharp("Controls/MavFTPUI.Designer.cs")
        else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        for (text, width) in COLUMNS {
            assert!(designer.contains(&format!(".Text = \"{text}\";")), "{text}");
            #[allow(clippy::cast_possible_truncation)]
            let width = width as i32;
            if width != 60 {
                assert!(designer.contains(&format!(".Width = {width};")), "{text}");
            }
        }
        for item in Menu::ALL {
            let text = item.text();
            assert!(
                designer.contains(&format!(".Text = \"{text}\";"))
                    || designer.contains(&format!(".Text = \"&{text}\";")),
                "{text}"
            );
        }
        assert!(designer.contains("this.btnMountFuse.Text = \"Mount as Drive\";"));
        assert!(
            designer.contains("this.btnMountFuse.Location = new System.Drawing.Point(763, 0);")
        );
        assert!(designer.contains("this.toolStripStatusLabel1.Text = \"...\";"));
        assert!(designer.contains("this.splitContainer1.SplitterDistance = 208;"));
        assert!(designer.contains("this.Size = new System.Drawing.Size(888, 541);"));
    }

    /// Loading lists `/` and `@SYS/`, each a root - `/` named "" - selects `@SYS` and lists it.
    /// (The test vehicle lists `@SYS` in `/` too, after `APM`, where ArduPilot does not.)
    #[test]
    fn loading_lists_the_two_roots_and_selects_sys() {
        let bench = Bench::new(vehicle());
        let page = loaded(&bench);
        let roots: Vec<(&str, &str)> = page
            .tree()
            .iter()
            .map(|node| (node.text.as_str(), node.path.as_str()))
            .collect();
        assert_eq!(roots, [("", "/"), ("@SYS", "@SYS/")]);
        let slash = page.tree().first().expect("/");
        assert_eq!(
            slash
                .children
                .iter()
                .map(|n| n.text.as_str())
                .collect::<Vec<_>>(),
            ["APM", "@SYS"]
        );
        assert_eq!(page.selected_node(), Some("@SYS/"));
        assert_eq!(page.selected_full_path().as_deref(), Some("@SYS"));
        assert_eq!(names(&page), ["threads", "uarts.txt"]);
        let uarts = page.items().get(1).expect("uarts.txt");
        assert_eq!(uarts.column(1), "File");
        assert_eq!(uarts.size, "12B");
        assert!(page.tree_enabled());
        assert!(!page.bar().1, "blocks again");
        let started = bench.started.borrow();
        assert_eq!(
            started.first(),
            Some(&FtpRequest::List {
                path: "/".to_owned()
            })
        );
        assert_eq!(
            started.get(1),
            Some(&FtpRequest::List {
                path: "@SYS/".to_owned()
            })
        );
    }

    /// A node clicked is listed again, its children made again; a double click on a row opens
    /// the directory of that name.
    #[test]
    fn clicking_lists_and_a_double_click_opens() {
        let bench = Bench::new(vehicle());
        let mut page = loaded(&bench);
        page.click_node("/");
        settle(&mut page, &bench);
        assert_eq!(page.selected_full_path().as_deref(), Some(""));
        assert_eq!(names(&page), ["APM", "@SYS"]);
        select(&mut page, "APM");
        page.double_click();
        settle(&mut page, &bench);
        assert_eq!(page.selected_node(), Some("/APM"));
        assert_eq!(page.selected_full_path().as_deref(), Some("/APM"));
        assert_eq!(names(&page), ["LOGS", "scripts", "param.pck"]);
        assert!(page.tree().first().is_some_and(|node| node.expanded));
    }

    /// A header sorts: descending first, then ascending; digits alone as numbers.
    #[test]
    fn a_header_sorts_the_list() {
        assert_eq!(compare("9", "10", false), Ordering::Less);
        assert_eq!(compare("", "1", false), Ordering::Less);
        assert_eq!(compare("b", "a", true), Ordering::Less);
        let bench = Bench::new(vehicle());
        let mut page = loaded(&bench);
        page.click_node("/APM");
        settle(&mut page, &bench);
        page.click_column(0);
        assert_eq!(names(&page), ["scripts", "param.pck", "LOGS"]);
        page.click_column(0);
        assert_eq!(names(&page), ["LOGS", "param.pck", "scripts"]);
        // Listed again, the rows keep the order.
        page.click_column(0);
        page.click_node("/APM");
        settle(&mut page, &bench);
        assert_eq!(names(&page), ["scripts", "param.pck", "LOGS"]);
    }

    /// Download: the folder asked for, the file read into it under its name, then numbered.
    #[test]
    fn download_writes_the_file_and_numbers_a_second() {
        let bench = Bench::new(vehicle());
        let mut page = loaded(&bench);
        page.click_node("/APM");
        settle(&mut page, &bench);
        select(&mut page, "param.pck");
        page.open_menu(None, (0.0, 0.0));
        page.choose(Menu::Download);
        assert_eq!(page.status(), "Download ");
        assert_eq!(page.prompt().map(|p| p.input().title), Some(BROWSE_TITLE));
        let folder = scratch("download");
        page.type_prompt(&folder.display().to_string());
        page.close_prompt(true);
        settle(&mut page, &bench);
        assert_eq!(
            std::fs::read(folder.join("param.pck")).ok().as_deref(),
            Some(&b"params"[..])
        );
        assert_eq!(page.status(), READY);
        assert!(bench.started.borrow().contains(&FtpRequest::Get {
            path: "/APM/param.pck".to_owned(),
            burst: false,
            readsize: RW_SIZE,
        }));
        page.choose(Menu::DownloadBurst);
        page.type_prompt(&folder.display().to_string());
        page.close_prompt(true);
        settle(&mut page, &bench);
        assert!(folder.join("param.pck0").exists());
        assert!(bench.started.borrow().contains(&FtpRequest::Get {
            path: "/APM/param.pck".to_owned(),
            burst: true,
            readsize: RW_SIZE,
        }));
        // Cancelled: Download has named the first row already, Download Burst has not.
        page.choose(Menu::Download);
        page.close_prompt(false);
        settle(&mut page, &bench);
        assert_eq!(page.status(), "Download param.pck");
        page.choose(Menu::DownloadBurst);
        page.close_prompt(false);
        settle(&mut page, &bench);
        assert_eq!(page.status(), "Download ");
        let _ = std::fs::remove_dir_all(&folder);
    }

    /// Upload: the file written into the selected directory, its CRC checked, the list again.
    #[test]
    fn upload_writes_checks_and_lists_again() {
        let bench = Bench::new(vehicle());
        let mut page = loaded(&bench);
        page.click_node("/APM");
        settle(&mut page, &bench);
        let folder = scratch("upload");
        let file = folder.join("hello.lua");
        std::fs::write(&file, b"print('hi')").expect("a file");
        page.choose(Menu::Upload);
        assert_eq!(page.prompt().map(|p| p.input().title), Some(OPEN_TITLE));
        page.type_prompt(&file.display().to_string());
        page.close_prompt(true);
        settle(&mut page, &bench);
        assert_eq!(
            bench
                .vehicle
                .borrow()
                .files
                .get("/APM/hello.lua")
                .map(Vec::as_slice),
            Some(&b"print('hi')"[..])
        );
        assert!(names(&page).contains(&"hello.lua"));
        assert!(page.take_status_line().is_none(), "the CRCs agree");
        let _ = std::fs::remove_dir_all(&folder);
    }

    /// Delete removes each selected row - by the listed directory's path and the name, "//name"
    /// from the root - and a refusal is the status line's, not a box.
    #[test]
    fn delete_removes_and_a_refusal_is_a_status_line() {
        let bench = Bench::new(vehicle());
        let mut page = loaded(&bench);
        page.click_node("/APM");
        settle(&mut page, &bench);
        select(&mut page, "param.pck");
        page.choose(Menu::Delete);
        settle(&mut page, &bench);
        assert!(!bench.vehicle.borrow().files.contains_key("/APM/param.pck"));
        assert!(!names(&page).contains(&"param.pck"));
        assert!(bench.started.borrow().contains(&FtpRequest::RemoveFile {
            path: "/APM/param.pck".to_owned()
        }));
        assert_eq!(page.status(), READY);
        // A file that has gone: the vehicle refuses, and the command's error is a status line.
        page.items.push(Item {
            name: "gone.txt".to_owned(),
            directory: false,
            size: "1B".to_owned(),
            modified: String::new(),
            dir: "/APM".to_owned(),
        });
        let last = page.items().len() - 1;
        page.click_item(last, false);
        page.choose(Menu::Delete);
        settle(&mut page, &bench);
        let line = page.take_status_line().expect("a status line");
        assert!(
            line.starts_with("There was an unexpected error ("),
            "{line}"
        );
        assert!(page.message().is_none(), "no box");
    }

    /// New Folder asks for a name and makes it; the list shows it.
    #[test]
    fn new_folder_makes_a_directory() {
        let bench = Bench::new(vehicle());
        let mut page = loaded(&bench);
        page.click_node("/APM");
        settle(&mut page, &bench);
        page.choose(Menu::NewFolder);
        assert_eq!(page.prompt().map(|p| p.input().prompt), Some(FOLDER_PROMPT));
        page.type_prompt("fresh");
        page.close_prompt(true);
        settle(&mut page, &bench);
        assert!(bench.vehicle.borrow().dirs.contains("/APM/fresh"));
        assert!(names(&page).contains(&"fresh"));
        // Made again, the vehicle says it exists, which `kCmdCreateDirectory` counts as made
        // (MAVFtp.cs:1094-1097): no "Failed to create directory".
        page.choose(Menu::NewFolder);
        page.type_prompt("fresh");
        page.close_prompt(true);
        settle(&mut page, &bench);
        assert!(page.take_status_line().is_none());
        // Refused, it is the status line's.
        bench.vehicle.borrow_mut().refuse = Some((
            mp_link::mavftp::wire::Opcode::CREATE_DIRECTORY,
            mp_link::mavftp::wire::ErrorCode::FAIL,
            mp_link::mavftp::wire::Errno(0),
        ));
        page.choose(Menu::NewFolder);
        page.type_prompt("refused");
        page.close_prompt(true);
        settle(&mut page, &bench);
        let line = page.take_status_line().expect("a status line");
        assert!(
            line.starts_with("There was an unexpected error ("),
            "{line}"
        );
        assert!(page.message().is_none(), "no box");
    }

    /// Rename edits the row's name in place; Enter renames it on the vehicle.
    #[test]
    fn rename_renames_on_enter() {
        let bench = Bench::new(vehicle());
        let mut page = loaded(&bench);
        page.click_node("/APM");
        settle(&mut page, &bench);
        select(&mut page, "param.pck");
        page.choose(Menu::Rename);
        assert_eq!(
            page.renaming()
                .map(|(_, f)| f.value().to_owned())
                .as_deref(),
            Some("param.pck")
        );
        page.type_rename("param.old");
        page.commit_rename();
        settle(&mut page, &bench);
        assert!(bench.vehicle.borrow().files.contains_key("/APM/param.old"));
        assert!(names(&page).contains(&"param.old"));
    }

    /// GetCRC32: the box with the vehicle's CRC in hexadecimal; nothing selected is the C#'s
    /// exception, on the status line.
    #[test]
    fn get_crc_shows_the_crc() {
        let bench = Bench::new(vehicle());
        let mut page = loaded(&bench);
        page.click_node("/APM");
        settle(&mut page, &bench);
        page.choose(Menu::Crc);
        assert_eq!(page.take_status_line().as_deref(), Some(NO_SELECTION));
        select(&mut page, "param.pck");
        page.choose(Menu::Crc);
        settle(&mut page, &bench);
        let expected = format!("param.pck: 0x{:X}", crc_crc32(0, b"params"));
        assert_eq!(
            page.message().map(|m| m.text.as_str()),
            Some(expected.as_str())
        );
        assert_eq!(page.message().map(|m| m.title), Some(""));
    }

    /// Mount as Drive asks for the point and fails, there being no Dokan: the C#'s text, on the
    /// status line and in no box.
    #[test]
    fn mount_fails_with_the_csharps_box() {
        let bench = Bench::new(vehicle());
        let mut page = loaded(&bench);
        page.press_mount();
        assert_eq!(
            page.prompt()
                .map(|p| p.input().field.value().to_owned())
                .as_deref(),
            Some(DEFAULT_MOUNT_POINT)
        );
        page.close_prompt(true);
        assert!(page.message().is_none(), "no box");
        assert_eq!(page.take_status_line(), Some(mount_failed(DOKAN_MISSING)));
    }

    /// Cancel on a transfer: "Cancelling...", the command stopped quietly, the sessions reset.
    #[test]
    fn cancel_stops_a_download_and_resets() {
        let bench = Bench::new(vehicle());
        let mut page = loaded(&bench);
        page.click_node("/APM");
        settle(&mut page, &bench);
        select(&mut page, "param.pck");
        page.choose(Menu::Download);
        let folder = scratch("cancel");
        page.type_prompt(&folder.display().to_string());
        *bench.hold.borrow_mut() = true;
        page.close_prompt(true);
        let view = view();
        page.tick(&bench, &view, true, Instant::now());
        assert!(page.dialog().is_some(), "the window is open");
        page.cancel(&bench);
        assert_eq!(page.dialog().map(|d| d.text.as_str()), Some(CANCELLING));
        bench.release();
        settle(&mut page, &bench);
        assert!(page.dialog().is_none());
        assert!(!folder.join("param.pck").exists());
        assert!(page.take_status_line().is_none(), "acknowledged: no error");
        assert_eq!(
            bench.started.borrow().last(),
            Some(&FtpRequest::ResetSessions)
        );
        let _ = std::fs::remove_dir_all(&folder);
    }

    #[test]
    fn sizes_and_names_are_the_csharps() {
        assert_eq!(size_units(12), "12B");
        assert_eq!(size_units(3000), "2KB");
        assert_eq!(size_units(5 * 1024 * 1024 + 1), "5MB");
        assert_eq!(file_name("/"), "");
        assert_eq!(file_name("@SYS/uarts.txt"), "uarts.txt");
        assert_eq!(modified_string(None), "");
    }

    #[test]
    fn the_gui_script_names_facts_and_controls_this_page_has() {
        let script = include_str!("../../../../tests/gui/config-mavftp.gui");
        let source = include_str!("mavftp.rs");
        let mut facts = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("config.mavftp.") => {
                    let recorded = source.contains(&format!("\"{key}\""))
                        || key.starts_with("config.mavftp.item.");
                    assert!(recorded, "{key} is not recorded");
                    facts += 1;
                }
                (Some("click" | "doubleclick"), Some(id)) if id.starts_with("mavftp-") => {
                    let id = id.trim_end_matches(":right");
                    let drawn = source.contains(&format!("\"{id}\""))
                        || id.starts_with("mavftp-node-")
                        || id.starts_with("mavftp-item-")
                        || id.starts_with("mavftp-col-")
                        || id.starts_with("mavftp-prompt-");
                    assert!(drawn, "{id} is not drawn");
                }
                _ => {}
            }
        }
        assert!(facts > 10, "{facts} facts");
    }
}
