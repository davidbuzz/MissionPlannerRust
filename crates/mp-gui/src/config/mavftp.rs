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

//! The MAVFtp page: `Controls/MavFTPUI.cs`, which CONFIG's list adds as "MAVFtp" once every
//! parameter is in, for a vehicle that reports MAVLink FTP (`GCSViews/SoftwareConfig.cs:211-216`).
//!
//! What it shows: a split container - on the left a tree of the vehicle's directories, on the
//! right a details list of the chosen directory (Name, Type, Size, Date modified) - over a status
//! strip of a label and a progress bar, and "Mount as Drive" at the top right
//! (`MavFTPUI.Designer.cs:29-235`). When the page is loaded it lists `/` and `@SYS/`, each a root
//! of the tree with its subdirectories under it, selects the last one made - `@SYS` - and lists it
//! (`PopulateTreeView`, `MavFTPUI.cs:73-140`). A click on a node's text selects it and
//! `TreeView1_NodeMouseClick` lists it again, replacing its children, directories first and then
//! files (`:151-203`); WinForms raises that event for a click on any part of a node - its plus or
//! minus, the rest of its row, the right button - so those list the node too, the selection
//! staying where it was, and a node above the selection that is closed or listed takes it, as
//! comctl32 moves it. A double click on a node's text opens or closes it. The tree's keys move
//! its selection - Up, Down, Home, End, Page Up and Page Down through the nodes drawn, Left and
//! Right closing and opening or going to the parent and the first child, Backspace to the parent,
//! the keypad's +, - and * - and list nothing: the C# wires no `AfterSelect`.
//!
//! The list is `MultiSelect`: a click selects a row alone, Control toggles one, Shift takes in
//! the rows from the last one clicked, a press where there is no row selects nothing; its keys
//! move and extend the selection likewise (Up, Down, Home, End, Page Up, Page Down, Control+Space).
//! A double click on a row opens the directory of that name (`:587-605`). A column header sorts
//! the list by its column, a column of digits as numbers (`:300-322`), and a header dragged to
//! another place moves its column there (`AllowColumnReorder`); a header's divider dragged sizes
//! its column, double-clicked fits it to its texts, until the next listing sizes them all again
//! (`AutoResizeColumns`, `:196-202`); the splitter between the tree and the list drags, never
//! nearer an edge than a panel's `MinSize`. `ListView1_MouseDown` is wired to an empty handler
//! (`:574-577`): there is nothing of it to port.
//!
//! The list's right-click menu - or the menu key, or Shift+F10, which open it in the list's
//! middle: Download Burst and Download (a burst read and a plain read, into a folder asked for,
//! under the file's name, numbered when taken, `:324-379, 607-663`), Upload (the files asked for,
//! each written into the directory, then the vehicle's CRC of it checked against the file's,
//! `:381-448`), Delete (`:450-480`), Rename (the row's name edited in place, `:482-513`), New
//! Folder (an `InputBox`, `:515-546`) and GetCRC32 (a box with the vehicle's CRC, `:548-572`). A
//! second click on a row already selected edits its name too, once the double-click time has
//! passed (`LabelEdit = true`), ending in the same `ListView1_AfterLabelEdit`; a name left as it
//! was renames nothing (`e.Label == null`). Files dropped on the list are uploaded
//! (`ListView1_DragEnter` lets only files in, `ListView1_DragDrop` uploads them, `:279-298,
//! 579-585`). Each runs behind a `ProgressReporterDialogue` with a Cancel, which asks the command
//! to stop and resets the vehicle's sessions. The page's `MAVFtp` reports its progress onto the
//! status strip, at most every 100 ms (`:34-65`), and a transfer's window shows it too.
//!
//! What the two controls do of themselves - selection, keys, the delayed edit, the header's drag -
//! is comctl32's, which WinForms' `TreeView` and `ListView` wrap: it is ported from Wine's
//! reimplementation of it (`dlls/comctl32/treeview.c`, `listview.c`, `header.c`, named at each
//! site) and from WinForms' own source (`TreeView.WndProc`, `Control.WmContextMenu`).
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
//!   `Settings.GetUserDataDirectory()`, as `SelectedPath` does; the upload's box takes several
//!   names, each in double quotes, as the Windows dialog's File name box does with `Multiselect`,
//!   and a name of no file is taken as Cancel, where the dialog would not close on it;
//! * "Mount as Drive" mounts through Dokan, a Windows file-system driver that this platform does
//!   not have: `Mount` fails, and the C#'s "Failed to mount" text, with the .NET message for the
//!   missing driver, goes on the status line;
//! * a listing another command is waiting behind starts when that command ends, not beside it;
//! * the controls' type-ahead - letters typed moving the selection to a name starting with them -
//!   is not ported: Windows does not document its rules, and Wine's comctl32, the reference here,
//!   gives the tree and the list two different sets; the keys above reach every node and row.
//!   F2 does nothing, as in Mission Planner: neither comctl32's list nor WinForms begins an edit
//!   on it, and the C# wires no key;
//! * the context menu takes no keys but Escape - its arrows, Enter and the `&Delete` and
//!   `&Rename` mnemonics are not wired: no menu in this application takes the keyboard;
//! * a crash of the C#'s - `SelectedItems[0]` with nothing selected, `SelectedNode.FullPath` with
//!   no node - is the .NET exception's text on the status line, where Mission Planner's
//!   unhandled-exception box shows it.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::cell::{Cell, OnceCell};
use std::cmp::Ordering;
use std::collections::{BTreeSet, VecDeque};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use gpui::{
    AnyElement, Context, ExternalPaths, FocusHandle, KeyDownEvent, MouseButton, ScrollHandle,
    SharedString, Window, div, prelude::*, px, relative, rgb,
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

/// `SystemInformation.DoubleClickTime`'s default: a click on a row already selected edits its
/// name when no second click follows within it (`LabelEdit = true`; comctl32's delayed edit,
/// `SetTimer(..., GetDoubleClickTime(), LISTVIEW_DelayedEditItem)`).
pub const DOUBLE_CLICK_TIME: Duration = Duration::from_millis(500);

/// The rows the list shows at once under its header - comctl32's count per column in details
/// view, the client height over a row's, which Page Up and Page Down move by.
pub const LIST_PAGE: usize = 29;

/// The nodes the tree shows at once - `TVM_GETVISIBLECOUNT`, the client height over a node's -
/// which Page Up and Page Down move by.
pub const TREE_PAGE: usize = 32;

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

/// `OpenFileDialog.FileNames` from its File name box with `Multiselect = true`: several names,
/// each in double quotes, as the Windows dialog takes them; a line with no quotes is one name.
/// `// C#: Controls/MavFTPUI.cs:383-392`
#[must_use]
pub fn file_names(text: &str) -> Vec<PathBuf> {
    if text.contains('"') {
        text.split('"')
            .skip(1)
            .step_by(2)
            .filter(|name| !name.trim().is_empty())
            .map(PathBuf::from)
            .collect()
    } else {
        let name = text.trim();
        if name.is_empty() {
            Vec::new()
        } else {
            vec![PathBuf::from(name)]
        }
    }
}

/// The keys held through a click or a key: `Control.ModifierKeys`, as the list and the tree read
/// them.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Mods {
    /// Control.
    pub control: bool,
    /// Shift.
    pub shift: bool,
}

impl Mods {
    /// The keys of a gpui event.
    #[must_use]
    pub const fn of(modifiers: gpui::Modifiers) -> Self {
        Self {
            control: modifiers.control,
            shift: modifiers.shift,
        }
    }
}

/// The page's control that has the keyboard: the one clicked last.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Control {
    /// `treeView1`.
    Tree,
    /// `listView1`.
    List,
}

/// Where `contextMenuStrip1` opens.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MenuAt {
    /// At the pointer, in the window.
    Pointer(f32, f32),
    /// From the keyboard - the menu key, or Shift+F10 - in the list's middle, as
    /// `Control.WmContextMenu` places a menu the keyboard opened (`Width / 2, Height / 2`).
    Middle,
}

/// What a press on the page's edges drags.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Dragged {
    /// `splitContainer1`'s splitter, between the tree and the list.
    Splitter,
    /// The divider at a column's right edge in the header: the column, by its index in
    /// `Columns`.
    Divider(usize),
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

/// The tags' paths of the nodes the tree draws, in order: comctl32's list items, which the keys
/// move through.
fn visible_paths(nodes: &[Node]) -> Vec<String> {
    let mut drawn = Vec::new();
    visible(nodes, 0, &mut drawn);
    drawn.into_iter().map(|(_, node)| node.path).collect()
}

/// `TreeNode.Parent`'s tag path; none for a root.
fn parent_path(nodes: &[Node], path: &str) -> Option<String> {
    for node in nodes {
        if node.children.iter().any(|child| child.path == path) {
            return Some(node.path.clone());
        }
        if let Some(parent) = parent_path(&node.children, path) {
            return Some(parent);
        }
    }
    None
}

/// Whether a node with this tag path is anywhere under `node`: comctl32's `TREEVIEW_IsChildOf`.
fn is_below(node: &Node, path: &str) -> bool {
    node.children
        .iter()
        .any(|child| child.path == path || is_below(child, path))
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
    /// `TreeView1_NodeMouseClick` for the node a click was on - `e.Node` - selected or not.
    ClickNode(String),
    /// Requests on the link.
    Work(Work),
    /// GetCRC32's box, once its window has closed.
    CrcBox { name: String },
}

impl Step {
    /// Whether it puts a request on the link, and so waits for the one there.
    const fn uses_link(&self) -> bool {
        matches!(self, Self::Work(_) | Self::Click | Self::ClickNode(_))
    }
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
    /// comctl32's `nFocusedItem`: the row the keys move from.
    focused: Option<usize>,
    /// comctl32's `nSelectionMark`: where a Shift range starts.
    mark: Option<usize>,
    /// A click on a row already selected: the row, and when its name is edited if no second
    /// click has come.
    edit_due: Option<(usize, Instant)>,
    /// The edit that timer opened wants the keyboard, once the page is drawn.
    edit_focus: Cell<bool>,
    /// Which control has the keyboard.
    keys_to: Option<Control>,
    /// The tree's and the list's keyboard focus, made when the page is first drawn.
    keys: OnceCell<FocusHandle>,
    /// The rows' scroll position: the top row, for Page Up and Page Down, and a row the keys
    /// reach brought into view.
    scroll: ScrollHandle,
    /// `ColumnHeader.DisplayIndex`: the columns in the order the header shows them.
    column_order: [usize; 4],
    /// `splitContainer1.SplitterDistance`: the tree's width, where the splitter was dragged.
    splitter: f32,
    /// The columns' widths as a divider's drag or double click left them, until the next
    /// listing sizes them again (`AutoResizeColumns` ends `NodeMouseClick`); none, sized so.
    widths: Option<[f32; 4]>,
    /// The splitter or a header's divider being dragged: what, where it was pressed, and the
    /// distance or the width then.
    drag: Option<(Dragged, f32, f32)>,
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
    menu: Option<MenuAt>,
    /// A row's name being edited: its index and the text.
    renaming: Option<(usize, TextField)>,
    /// The box asking for a name, a folder or a file.
    prompt: Option<Prompt>,
    /// New Folder's or "Mount as Drive"'s box as its OK closed it, until the holder keeps the
    /// answer in `Settings.Instance`.
    answered: Option<InputBox>,
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
            focused: None,
            mark: None,
            edit_due: None,
            edit_focus: Cell::new(false),
            keys_to: None,
            keys: OnceCell::new(),
            scroll: ScrollHandle::new(),
            column_order: [0, 1, 2, 3],
            splitter: SPLITTER.0,
            widths: None,
            drag: None,
            sort: None,
            status: STATUS_START.to_owned(),
            bar_value: 0,
            bar_marquee: false,
            next_update: None,
            last_report: None,
            menu: None,
            renaming: None,
            prompt: None,
            answered: None,
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
    pub const fn menu(&self) -> Option<MenuAt> {
        self.menu
    }

    /// The row the keys move from.
    #[must_use]
    pub const fn focused(&self) -> Option<usize> {
        self.focused
    }

    /// The control with the keyboard.
    #[must_use]
    pub const fn keys_to(&self) -> Option<Control> {
        self.keys_to
    }

    /// The columns in the order the header shows them.
    #[must_use]
    pub const fn column_order(&self) -> [usize; 4] {
        self.column_order
    }

    /// The tree's width: `SplitterDistance`.
    #[must_use]
    pub const fn splitter(&self) -> f32 {
        self.splitter
    }

    /// The list's left edge and width, right of the splitter.
    #[must_use]
    pub fn list_bounds(&self) -> (f32, f32) {
        let left = self.splitter + SPLITTER.1;
        (left, PAGE_SIZE.0 - left)
    }

    /// The columns' widths, by their index in `Columns`: a drag's, or `AutoResizeColumns`' in the
    /// list's client area, inside its border.
    #[must_use]
    pub fn column_widths(&self) -> [f32; 4] {
        self.widths
            .unwrap_or_else(|| column_widths(&self.items, self.list_bounds().1 - 2.0))
    }

    /// The splitter or a divider pressed, at `at` across the window.
    pub fn begin_drag(&mut self, what: Dragged, at: f32) {
        self.close_menu_and_rename();
        let start = match what {
            Dragged::Splitter => self.splitter,
            Dragged::Divider(column) => self.column_widths().get(column).copied().unwrap_or(0.0),
        };
        self.drag = Some((what, at, start));
    }

    /// The pointer moved with the button down: the splitter follows it, never nearer an edge than
    /// a panel's `MinSize`; a divider likewise, a column never under nothing (comctl32's
    /// `HEADER_MouseMove`, `iNewWidth < 0`). Whether something is being dragged.
    pub fn drag_to(&mut self, at: f32) -> bool {
        let Some((what, from, start)) = self.drag else {
            return false;
        };
        let moved = at - from;
        match what {
            Dragged::Splitter => {
                let most = PAGE_SIZE.0 - PANEL_MIN - SPLITTER.1;
                self.splitter = (start + moved).round().clamp(PANEL_MIN, most);
            }
            Dragged::Divider(column) => {
                let mut widths = self.column_widths();
                if let Some(width) = widths.get_mut(column) {
                    *width = (start + moved).round().max(0.0);
                }
                self.widths = Some(widths);
            }
        }
        true
    }

    /// The button let go.
    pub fn end_drag(&mut self) {
        self.drag = None;
    }

    /// A divider double-clicked: the column as wide as its widest text, the header's left out -
    /// comctl32's list answers `HDN_DIVIDERDBLCLICK` with `LVSCW_AUTOSIZE`.
    pub fn fit_column(&mut self, column: usize) {
        let mut widths = self.column_widths();
        let longest = self
            .items
            .iter()
            .map(|item| item.column(column).chars().count())
            .max()
            .unwrap_or(0);
        if let Some(width) = widths.get_mut(column) {
            *width = text_width(longest);
        }
        self.widths = Some(widths);
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
        self.running.is_some() || self.steps.iter().any(Step::uses_link)
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
        self.drag = None;
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

    /// Whether a click on the tree reaches a node: the tree enabled - `treeView1.Enabled` is false
    /// while `PopulateTreeView` lists - and the node there. The tree takes the keyboard.
    fn tree_takes(&mut self, path: &str) -> bool {
        self.close_menu_and_rename();
        if !self.tree_enabled || find(&self.tree, path).is_none() {
            return false;
        }
        self.keys_to = Some(Control::Tree);
        true
    }

    /// A node's text clicked: comctl32 selects it, and `TreeView1_NodeMouseClick` lists it.
    /// `// C#: Controls/MavFTPUI.cs:151-203`
    pub fn click_node(&mut self, path: &str) {
        if !self.tree_takes(path) {
            return;
        }
        self.selected_node = Some(path.to_owned());
        self.steps.push_back(Step::ClickNode(path.to_owned()));
    }

    /// A click on the rest of a node's row - its indent, or right of its text - or with the right
    /// button anywhere on it: WinForms raises `NodeMouseClick` for a click on any part of a node
    /// (`TreeView.WndProc`'s `WM_LBUTTONUP` and `NM_RCLICK`, which hit-test the row), and the C#
    /// lists that node whatever the button. comctl32 selects only on the node's text, so
    /// `SelectedNode` stays where it was: the list shows one directory while Upload, Download,
    /// Rename, New Folder and GetCRC32 name `SelectedNode`'s, as in Mission Planner.
    /// `// C#: Controls/MavFTPUI.cs:151-203`
    pub fn click_node_row(&mut self, path: &str) {
        if self.tree_takes(path) {
            self.steps.push_back(Step::ClickNode(path.to_owned()));
        }
    }

    /// A node's plus or minus: comctl32 expands or collapses it on the press, then
    /// `NodeMouseClick` lists it, the selection staying as it was. A second click within the
    /// double-click time only toggles it again: WinForms raises no `NodeMouseClick` for the
    /// second click of a double click (`doubleclickFired`).
    /// `// C#: Controls/MavFTPUI.cs:151-203`
    pub fn toggle_node(&mut self, path: &str, clicks: usize) {
        if !self.tree_takes(path) {
            return;
        }
        self.toggle(path);
        if clicks < 2 {
            self.steps.push_back(Step::ClickNode(path.to_owned()));
        }
    }

    /// A node's text double-clicked: the first click selected and listed it; the second toggles
    /// it, comctl32's `TREEVIEW_LButtonDoubleClick`, and lists nothing.
    pub fn double_click_node(&mut self, path: &str) {
        if self.tree_takes(path) {
            self.toggle(path);
        }
    }

    /// comctl32's `TREEVIEW_Toggle`: an open node closed, a closed one opened.
    fn toggle(&mut self, path: &str) {
        if find(&self.tree, path).is_some_and(|node| node.expanded) {
            self.collapse(path);
        } else {
            self.expand(path);
        }
    }

    /// comctl32's `TREEVIEW_Expand`: a node with children opened.
    fn expand(&mut self, path: &str) {
        if let Some(node) = find_mut(&mut self.tree, path)
            && !node.children.is_empty()
        {
            node.expanded = true;
        }
    }

    /// comctl32's `TREEVIEW_ExpandAll`: a node and every node under it opened.
    fn expand_all(&mut self, path: &str) {
        fn open(node: &mut Node) {
            if !node.children.is_empty() {
                node.expanded = true;
            }
            for child in &mut node.children {
                open(child);
            }
        }
        if let Some(node) = find_mut(&mut self.tree, path) {
            open(node);
        }
    }

    /// comctl32's `TREEVIEW_Collapse`: an open node closed; a node selected under it gives the
    /// selection to it (with no `AfterSelect`, so the list stays).
    fn collapse(&mut self, path: &str) {
        let Some(node) = find_mut(&mut self.tree, path) else {
            return;
        };
        if node.children.is_empty() || !node.expanded {
            return;
        }
        node.expanded = false;
        let hidden = self
            .selected_node
            .as_deref()
            .is_some_and(|selected| is_below(node, selected));
        if hidden {
            self.selected_node = Some(path.to_owned());
        }
    }

    /// A key with the tree holding the keyboard: comctl32's `TREEVIEW_KeyDown` - Up, Down, Home,
    /// End, Page Up and Page Down move the selection through the nodes drawn, Left closes a node
    /// or goes to its parent, Right opens one or goes to its first child, Backspace goes to the
    /// parent, the keypad's +, - and * open, close and open everything under. With Control the
    /// view scrolls and the selection stays. `SelectedNode` changes with no `NodeMouseClick` -
    /// the C# wires no `AfterSelect` - so the list stays as it was, as in Mission Planner.
    /// Whether the key was the tree's.
    pub fn tree_key(&mut self, key: &str, mods: Mods) -> bool {
        if !matches!(
            key,
            "up" | "down"
                | "home"
                | "end"
                | "pageup"
                | "pagedown"
                | "left"
                | "right"
                | "backspace"
                | "add"
                | "subtract"
                | "multiply"
        ) {
            return false;
        }
        let Some(current) = self.selected_node.clone() else {
            return true;
        };
        if !self.tree_enabled || mods.control {
            return true;
        }
        let order = visible_paths(&self.tree);
        let at = order.iter().position(|path| *path == current);
        let first = self.tree.first().map(|node| node.path.clone());
        let (expanded, has_children, first_child) =
            find(&self.tree, &current).map_or((false, false, None), |node| {
                (
                    node.expanded,
                    !node.children.is_empty(),
                    node.children.first().map(|child| child.path.clone()),
                )
            });
        let next = match key {
            // `TREEVIEW_GetPrevListItem`, or the first root when there is none.
            "up" => at
                .and_then(|i| i.checked_sub(1))
                .and_then(|i| order.get(i).cloned())
                .or(first),
            "down" => at.and_then(|i| order.get(i + 1).cloned()),
            "home" => first,
            "end" => order.last().cloned(),
            // `TREEVIEW_GetListItem(item, -/+TREEVIEW_GetVisibleCount)`, stopping at the ends.
            "pageup" => at.and_then(|i| order.get(i.saturating_sub(TREE_PAGE)).cloned()),
            "pagedown" => at.and_then(|i| {
                order
                    .get((i + TREE_PAGE).min(order.len().saturating_sub(1)))
                    .cloned()
            }),
            "left" if expanded => {
                self.collapse(&current);
                None
            }
            "left" | "backspace" => parent_path(&self.tree, &current),
            "right" if has_children && !expanded => {
                self.expand(&current);
                None
            }
            "right" => first_child,
            "add" => {
                self.expand(&current);
                None
            }
            "subtract" => {
                self.collapse(&current);
                None
            }
            _ => {
                self.expand_all(&current);
                None
            }
        };
        if let Some(next) = next {
            self.selected_node = Some(next);
        }
        true
    }

    /// A row clicked with the left button: comctl32's `LISTVIEW_LButtonDown` and
    /// `LISTVIEW_LButtonUp` for a `MultiSelect` list (the default) - Control toggles the row,
    /// Shift selects from the mark to it, both add that range; a plain click selects it alone,
    /// and when it was selected already its name is edited once the double-click time has passed
    /// with no second click (`LabelEdit = true`, with `FullRowSelect` anywhere on the row), the
    /// edit ending in `ListView1_AfterLabelEdit`. The list takes the keyboard.
    /// `// C#: Controls/MavFTPUI.Designer.cs (listView1.LabelEdit, FullRowSelect)`
    pub fn click_item(&mut self, index: usize, mods: Mods, now: Instant) {
        self.close_menu_and_rename();
        self.keys_to = Some(Control::List);
        if index >= self.items.len() {
            return;
        }
        match (mods.control, mods.shift) {
            (true, true) => {
                self.add_range(index);
                self.focused = Some(index);
            }
            (true, false) => {
                if !self.selected.remove(&index) {
                    self.selected.insert(index);
                }
                self.focused = Some(index);
                self.mark = Some(index);
            }
            (false, true) => self.group_select(index),
            (false, false) => {
                let was_selected = self.selected.contains(&index);
                self.select_only(index);
                if was_selected {
                    self.edit_due = Some((index, now + DOUBLE_CLICK_TIME));
                }
            }
        }
    }

    /// The left button on the list where there is no row: comctl32 deselects everything, unless
    /// Control or Shift is held (`LISTVIEW_LButtonDown`). The list takes the keyboard.
    pub fn click_list(&mut self, mods: Mods) {
        self.close_menu_and_rename();
        self.keys_to = Some(Control::List);
        if !mods.control && !mods.shift {
            self.selected.clear();
        }
    }

    /// comctl32's `LISTVIEW_SetSelection`: the row alone selected, focused, and the mark.
    fn select_only(&mut self, index: usize) {
        self.selected.clear();
        self.selected.insert(index);
        self.focused = Some(index);
        self.mark = Some(index);
    }

    /// comctl32's `LISTVIEW_SetGroupSelection`: the mark to the row selected and nothing else - the
    /// row alone, and the mark, when there is none - and the row focused.
    fn group_select(&mut self, index: usize) {
        match self.mark {
            Some(mark) => self.selected = (mark.min(index)..=mark.max(index)).collect(),
            None => {
                self.mark = Some(index);
                self.selected = BTreeSet::from([index]);
            }
        }
        self.focused = Some(index);
    }

    /// comctl32's `LISTVIEW_AddGroupSelection`: the mark to the row added to the selection.
    fn add_range(&mut self, index: usize) {
        let mark = self.mark.unwrap_or(index);
        self.selected.extend(mark.min(index)..=mark.max(index));
    }

    /// A key with the list holding the keyboard: comctl32's `LISTVIEW_KeyDown` in details view -
    /// Up, Down, Home, End, Page Up and Page Down move to a row and select it alone; with Shift
    /// from the mark to it; with Control they move the focus only, and Control+Space toggles the
    /// focused row (`LISTVIEW_KeySelection`). The row is brought into view. Left and Right move
    /// nothing in details view. Whether the key was the list's.
    pub fn list_key(&mut self, key: &str, mods: Mods) -> bool {
        let count = self.items.len();
        let top = self.top_row();
        let last_shown = top + LIST_PAGE - 1;
        let target = match key {
            "space" => self.focused,
            "home" => (count > 0).then_some(0),
            "end" => count.checked_sub(1),
            "up" => self.focused.and_then(|row| row.checked_sub(1)),
            "down" => match self.focused {
                None => (count > 0).then_some(0),
                Some(row) => (row + 1 < count).then_some(row + 1),
            },
            "pageup" => Some(if self.focused == Some(top) {
                top.saturating_sub(LIST_PAGE - 1)
            } else {
                top
            }),
            "pagedown" => {
                let row = if self.focused == Some(last_shown) {
                    last_shown + LIST_PAGE - 1
                } else {
                    last_shown
                };
                count.checked_sub(1).map(|end| row.min(end))
            }
            "left" | "right" => None,
            _ => return false,
        };
        let space = key == "space";
        if let Some(row) = target
            && row < count
            && (Some(row) != self.focused || space)
        {
            if !mods.shift && !mods.control {
                self.select_only(row);
            } else if mods.shift {
                self.group_select(row);
            } else {
                if space && !self.selected.remove(&row) {
                    self.selected.insert(row);
                    self.mark = Some(row);
                }
                self.focused = Some(row);
            }
            // `LISTVIEW_EnsureVisible`.
            self.scroll.scroll_to_item(row);
        }
        true
    }

    /// The top row shown: the rows' scroll position over a row's height.
    fn top_row(&self) -> usize {
        let offset = -f32::from(self.scroll.offset().y);
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let row = (offset.max(0.0) / LIST_ROW).floor() as usize;
        row
    }

    /// A key with the page holding the keyboard: to the tree or the list, whichever was clicked
    /// last. While the menu is open it has the keys: Escape closes it. The menu key or
    /// Shift+F10 on the list opens `contextMenuStrip1` in the list's middle (`WM_CONTEXTMENU`
    /// from the keyboard, `Control.WmContextMenu`). Nothing while a box or a window is over the
    /// page, or a name is being edited. Whether the key was taken.
    pub fn key(&mut self, key: &str, mods: Mods) -> bool {
        if self.renaming.is_some()
            || self.prompt.is_some()
            || !self.messages.is_empty()
            || self.dialog().is_some()
        {
            return false;
        }
        if self.menu.is_some() {
            if key == "escape" {
                self.menu = None;
            }
            return true;
        }
        match self.keys_to {
            Some(Control::Tree) => self.tree_key(key, mods),
            Some(Control::List) if key == "menu" || (key == "f10" && mods.shift) => {
                self.menu = Some(MenuAt::Middle);
                true
            }
            Some(Control::List) => self.list_key(key, mods),
            None => false,
        }
    }

    /// `ListView1_MouseDoubleClick`: the selected node expanded, and the child named as the first
    /// selected row selected and listed. A double click cancels the edit its first click armed
    /// (comctl32's `LISTVIEW_LButtonDblClk`).
    /// `// C#: Controls/MavFTPUI.cs:587-605`
    pub fn double_click(&mut self) {
        self.edit_due = None;
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
    /// `Descending` first - and the rows sorted by it. The column is the header's own, wherever
    /// a drag has put it (`ColumnClickEventArgs.Column` is the index in `Columns`).
    /// `// C#: Controls/MavFTPUI.cs:300-322`
    pub fn click_column(&mut self, column: usize) {
        self.close_menu_and_rename();
        let descending = !matches!(self.sort, Some((_, true)));
        self.sort = Some((column, descending));
        self.sort_items();
    }

    /// `AllowColumnReorder`: a header dragged and let go on a divider - `divider` counts the
    /// headers left of it, as shown: before the header under the pointer, or after it on its
    /// right half - moves there, and the rows show their cells in the new order. comctl32's
    /// `HEADER_SetHotDivider` and `HEADER_LButtonUp`: the new place is the divider's, one less
    /// when that is right of where the header was. A drag sorts nothing.
    /// `// C#: Controls/MavFTPUI.Designer.cs (listView1.AllowColumnReorder = true)`
    pub fn drop_column(&mut self, column: usize, divider: usize) {
        let mut order = self.column_order.to_vec();
        let Some(from) = order.iter().position(|shown| *shown == column) else {
            return;
        };
        let last = order.len() - 1;
        let to = if divider > last {
            last
        } else if divider > from {
            divider - 1
        } else {
            divider
        };
        order.remove(from);
        order.insert(to, column);
        for (slot, shown) in self.column_order.iter_mut().zip(order) {
            *slot = shown;
        }
    }

    /// The rows in the sorter's order; the selection, the focus and the mark follow their rows,
    /// as comctl32's `LISTVIEW_SortItems` keeps them.
    fn sort_items(&mut self) {
        let Some((column, descending)) = self.sort else {
            return;
        };
        let mut rows: Vec<(usize, Item)> = self.items.drain(..).enumerate().collect();
        rows.sort_by(|a, b| compare(a.1.column(column), b.1.column(column), descending));
        let moved = |old: usize| rows.iter().position(|(was, _)| *was == old);
        self.selected = self.selected.iter().filter_map(|old| moved(*old)).collect();
        self.focused = self.focused.and_then(moved);
        self.mark = self.mark.and_then(moved);
        self.edit_due = self
            .edit_due
            .and_then(|(row, due)| moved(row).map(|row| (row, due)));
        self.items = rows.into_iter().map(|(_, item)| item).collect();
    }

    /// The right button on the list: comctl32's `LISTVIEW_RButtonDown` - a row focused, and
    /// selected alone unless it is selected already or Control or Shift is held; where there is
    /// no row, nothing selected - then `contextMenuStrip1` at the pointer. The list takes the
    /// keyboard.
    pub fn open_menu(&mut self, row: Option<usize>, mods: Mods, at: (f32, f32)) {
        self.commit_rename();
        self.keys_to = Some(Control::List);
        match row.filter(|index| *index < self.items.len()) {
            Some(index) => {
                self.focused = Some(index);
                if !mods.control && !mods.shift && !self.selected.contains(&index) {
                    self.select_only(index);
                }
            }
            None => self.selected.clear(),
        }
        self.menu = Some(MenuAt::Pointer(at.0, at.1));
    }

    /// The right button on the header: the header is the list's child window, which takes the
    /// press itself, so the selection stays; its `WM_CONTEXTMENU` reaches the list's
    /// `ContextMenuStrip`, at the pointer.
    pub fn open_menu_on_header(&mut self, at: (f32, f32)) {
        self.commit_rename();
        self.menu = Some(MenuAt::Pointer(at.0, at.1));
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
                    Some(index) => self.begin_edit(index),
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

    /// `ListViewItem.BeginEdit`: the row's name in an edit box over it.
    fn begin_edit(&mut self, index: usize) {
        self.edit_due = None;
        let name = self.items.get(index).map_or("", |item| item.name.as_str());
        let mut field = TextField::new("");
        field.set(name);
        self.renaming = Some((index, field));
    }

    /// "Mount as Drive": unmounted, so the mount point asked for. The button takes the keyboard.
    /// `// C#: Controls/MavFTPUI.cs:673-697`
    pub fn press_mount(&mut self) {
        self.close_menu_and_rename();
        self.keys_to = None;
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

    /// New Folder's or "Mount as Drive"'s box as its OK closed it, once: the answer `InputBox`
    /// keeps in `Settings.Instance`, which [`keep_answer`] writes.
    /// `// C#: Controls/MavFTPUI.cs:518, 693; ExtLibs/Controls/InputBox.cs:73-84, 178-184`
    pub fn take_answered(&mut self) -> Option<InputBox> {
        self.answered.take()
    }

    /// The box asking closed with OK or Cancel.
    pub fn close_prompt(&mut self, ok: bool) {
        let Some(prompt) = self.prompt.take() else {
            return;
        };
        let answer = prompt.input().field.value().to_owned();
        match prompt {
            // `InputBox.Show`'s OK keeps the answer; the two dialogs' stand-ins keep nothing.
            Prompt::NewFolder(input) | Prompt::Mount(input) if ok => {
                let mount = input.title == MOUNT_TITLE;
                self.answered = Some(input);
                if mount {
                    // `MavFtpDokan.Mount` throws: no Dokan here. The C#'s box is the status
                    // line's, by the owner's ruling. `// C#: Controls/MavFTPUI.cs:698-708`
                    self.status_line.push_back(mount_failed(DOKAN_MISSING));
                } else {
                    self.new_folder(Some(answer));
                }
            }
            Prompt::NewFolder(_) => self.new_folder(None),
            Prompt::Mount(_) => {}
            Prompt::Folder(_, burst) => self.download(ok.then(|| PathBuf::from(answer)), burst),
            Prompt::Open(_) => {
                // `ofd.FileNames`, each uploaded; the directory is listed again whatever the
                // answer (`MavFTPUI.cs:402-403` is outside the `if`). A name that is not a file
                // is as Cancel: the dialog does not close on it (`CheckFileExists`).
                // `// C#: Controls/MavFTPUI.cs:381-404`
                let files = if ok { file_names(&answer) } else { Vec::new() };
                if files.iter().all(|file| file.is_file()) {
                    self.upload(&files);
                } else {
                    self.upload(&[]);
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
    /// and the directory listed again. A name left as it was is `e.Label == null` - comctl32
    /// passes no text for it (`LISTVIEW_EndEditLabelT`) - and the C# returns at once.
    /// `// C#: Controls/MavFTPUI.cs:487-513`
    pub fn commit_rename(&mut self) {
        let Some((index, field)) = self.renaming.take() else {
            return;
        };
        let label = field.value().to_owned();
        let Some(text) = self.items.get(index).map(|item| item.name.clone()) else {
            return;
        };
        if label == text {
            return;
        }
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
        self.edit_when_due(now);
    }

    /// comctl32's `LISTVIEW_DelayedEditItem`: the double-click time has passed since a click on a
    /// row already selected, with no second click - the row's name is edited if it is still
    /// selected and the list still has the keyboard (and nothing is over the page).
    fn edit_when_due(&mut self, now: Instant) {
        let Some((row, due)) = self.edit_due else {
            return;
        };
        if now < due {
            return;
        }
        self.edit_due = None;
        if self.keys_to == Some(Control::List)
            && self.selected.contains(&row)
            && self.renaming.is_none()
            && self.menu.is_none()
            && self.prompt.is_none()
            && self.dialog().is_none()
        {
            self.begin_edit(row);
            self.edit_focus.set(true);
        }
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
    /// listing of its own when the node has none. `Nodes.Clear()` deleting the selected node, or
    /// one it is under, gives the selection to the node listed: comctl32's `TREEVIEW_DeleteItem`
    /// selects a deleted item's next sibling or else its parent, and the children go last first.
    /// `// C#: Controls/MavFTPUI.cs:157-203, 235-276`
    fn listed(&mut self, path: &str, entries: Option<Vec<FtpFileInfo>>) {
        let Some(node) = find_mut(&mut self.tree, path) else {
            return;
        };
        if self
            .selected_node
            .as_deref()
            .is_some_and(|selected| is_below(node, selected))
        {
            self.selected_node = Some(path.to_owned());
        }
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

    /// `GetFiles`' rows: every entry that is not a directory. Then
    /// `AutoResizeColumns(HeaderSize)`: widths a drag left go.
    /// `// C#: Controls/MavFTPUI.cs:182-202`
    fn add_files(&mut self, path: &str, entries: &[FtpFileInfo]) {
        self.widths = None;
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
            if step.uses_link() && self.running.is_some() {
                return;
            }
            // A command another page runs on the vehicle is waited for.
            if step.uses_link()
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
                    if let Some(node) = self.selected_node.clone() {
                        self.start_listing(port, node, now);
                    }
                }
                Step::ClickNode(node) => self.start_listing(port, node, now),
                Step::Work(work) => self.start_work(port, work, now),
                Step::CrcBox { name } => {
                    self.messages
                        .push_back(plain(format!("{name}: 0x{:X}", self.crc)));
                }
            }
        }
    }

    /// `TreeView1_NodeMouseClick` for a node: `e.Node == null` returns at once (a node no longer
    /// in the tree likewise); else `listView1.Items.Clear()` - its selection, focus and mark
    /// with it - and the node's `GetDirectories`.
    /// `// C#: Controls/MavFTPUI.cs:151-166`
    fn start_listing<P: FtpPort>(&mut self, port: &P, node: String, now: Instant) {
        if find(&self.tree, &node).is_none() {
            return;
        }
        self.items.clear();
        self.selected.clear();
        self.focused = None;
        self.mark = None;
        self.edit_due = None;
        self.renaming = None;
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
    record(
        "config.mavftp.focused",
        page.focused()
            .and_then(|index| page.items().get(index))
            .map_or("none", |item| item.name.as_str()),
    );
    record(
        "config.mavftp.keys",
        match page.keys_to() {
            Some(Control::Tree) => "tree",
            Some(Control::List) => "list",
            None => "none",
        },
    );
    record(
        "config.mavftp.columns",
        page.column_order()
            .iter()
            .filter_map(|column| COLUMNS.get(*column))
            .map(|(text, _)| *text)
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

/// `SplitContainer.Panel1MinSize` and `Panel2MinSize`'s default: how near an edge the splitter
/// goes.
const PANEL_MIN: f32 = 25.0;

/// A column wide enough for a text of so many characters, and its margins.
fn text_width(chars: usize) -> f32 {
    #[allow(clippy::cast_precision_loss)]
    let text = chars as f32 * CHAR;
    text + 12.0
}

/// `AutoResizeColumns(HeaderSize)`: each column as wide as its header or its widest text, the
/// last - `Columns`' last, wherever the header shows it - filling the rest.
fn column_widths(items: &[Item], width: f32) -> [f32; 4] {
    let mut widths = [0.0_f32; 4];
    for (column, (header, _)) in COLUMNS.iter().enumerate() {
        let longest = items
            .iter()
            .map(|item| item.column(column).chars().count())
            .max()
            .unwrap_or(0)
            .max(header.chars().count());
        if let Some(slot) = widths.get_mut(column) {
            *slot = text_width(longest);
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

/// The keyboard to the page's tree and list: the handle the page is drawn with.
fn focus_keys(this: &MissionPlanner, window: &mut Window, cx: &mut Context<MissionPlanner>) {
    if let Some(keys) = this.software_pages2.mavftp.keys.get() {
        keys.focus(window, cx);
    }
}

/// The tree, left of the splitter. A node's row: its plus or minus, its text, and the rest of the
/// row, each clicked as `treeView1` takes it (`MavFtp::toggle_node`, `click_node`,
/// `click_node_row`); the right button anywhere on the row is `NodeMouseClick` too.
fn tree_panel(page: &MavFtp, cx: &mut Context<MissionPlanner>) -> AnyElement {
    let mut nodes = Vec::new();
    visible(page.tree(), 0, &mut nodes);
    let enabled = page.tree_enabled();
    let mut tree = crate::probe::measured("mavftp-tree", div())
        .id("mavftp-tree")
        .absolute()
        .left(px(0.0))
        .top(px(0.0))
        .w(px(page.splitter()))
        .h(px(PAGE_SIZE.1 - STATUS_HEIGHT))
        .overflow_y_scroll()
        .flex()
        .flex_col()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::BG))
        // A press where there is no node: the tree takes the keyboard, nothing more.
        .on_mouse_down(
            MouseButton::Left,
            cx.listener(|this, _event, window, cx| {
                let page = &mut this.software_pages2.mavftp;
                if page.tree_enabled() {
                    page.keys_to = Some(Control::Tree);
                    focus_keys(this, window, cx);
                }
            }),
        );
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
                .on_click(
                    cx.listener(move |this, event: &gpui::ClickEvent, window, cx| {
                        let clicks = event.click_count();
                        this.software_pages2.mavftp.toggle_node(&path, clicks);
                        focus_keys(this, window, cx);
                        cx.stop_propagation();
                        cx.notify();
                    }),
                )
                .into_any_element()
        };
        let path = node.path.clone();
        let label = crate::probe::measured(id.clone(), div())
            .id(SharedString::from(id.clone()))
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
                label.cursor_pointer().on_click(cx.listener(
                    move |this, event: &gpui::ClickEvent, window, cx| {
                        let page = &mut this.software_pages2.mavftp;
                        if event.click_count() == 2 {
                            page.double_click_node(&path);
                        } else {
                            page.click_node(&path);
                        }
                        focus_keys(this, window, cx);
                        cx.stop_propagation();
                        cx.notify();
                    },
                ))
            });
        let row_path = node.path.clone();
        let right_path = node.path.clone();
        tree = tree.child(
            div()
                .id(SharedString::from(format!("{id}-row")))
                .flex()
                .items_center()
                .flex_shrink_0()
                .pl(px(indent))
                .h(px(TREE_ROW))
                .child(expander)
                .child(label)
                .on_click(
                    cx.listener(move |this, event: &gpui::ClickEvent, window, cx| {
                        if event.click_count() < 2 {
                            this.software_pages2.mavftp.click_node_row(&row_path);
                        }
                        focus_keys(this, window, cx);
                        cx.notify();
                    }),
                )
                .on_mouse_down(
                    MouseButton::Right,
                    cx.listener(move |this, _event, window, cx| {
                        this.software_pages2.mavftp.click_node_row(&right_path);
                        focus_keys(this, window, cx);
                        cx.stop_propagation();
                        cx.notify();
                    }),
                ),
        );
    }
    tree.into_any_element()
}

/// What a header carries while it is dragged: its column, and its text for the ghost.
#[derive(Debug, Clone)]
struct ColumnDrag {
    column: usize,
    text: SharedString,
}

/// The header drawn under the pointer while it is dragged, as comctl32's header drags an image
/// of itself (`ImageList_BeginDrag`).
struct ColumnGhost(SharedString);

impl Render for ColumnGhost {
    fn render(&mut self, _window: &mut Window, _cx: &mut Context<Self>) -> impl IntoElement {
        div()
            .px_1()
            .h(px(LIST_ROW))
            .text_xs()
            .text_color(rgb(theme::TEXT))
            .bg(rgb(theme::PANEL))
            .border_1()
            .border_color(rgb(theme::ACCENT))
            .opacity(0.8)
            .child(self.0.clone())
    }
}

/// One half of a header, where a dragged header can be let go: the divider before the header,
/// or after it on its right half (comctl32's `HEADER_SetHotDivider`), lit while a header is
/// over it.
fn drop_half(divider: usize, right: bool, cx: &mut Context<MissionPlanner>) -> impl IntoElement {
    div()
        .absolute()
        .top(px(0.0))
        .h_full()
        .w(relative(0.5))
        .left(relative(if right { 0.5 } else { 0.0 }))
        .drag_over::<ColumnDrag>(move |style, _drag, _window, _cx| {
            if right {
                style.border_r_2().border_color(rgb(theme::ACCENT))
            } else {
                style.border_l_2().border_color(rgb(theme::ACCENT))
            }
        })
        .on_drop(cx.listener(move |this, drag: &ColumnDrag, _window, cx| {
            this.software_pages2
                .mavftp
                .drop_column(drag.column, divider);
            cx.notify();
        }))
}

/// A column's divider, at its right edge: pressed and dragged it sizes the column (comctl32's
/// `HEADER_LButtonDown` on `HHT_ONDIVIDER`, then `HEADER_MouseMove`), double-clicked it fits it
/// to its texts (`HDN_DIVIDERDBLCLICK`).
fn divider(column: usize, cx: &mut Context<MissionPlanner>) -> impl IntoElement {
    let id = format!("mavftp-divider-{column}");
    crate::probe::measured(id.clone(), div())
        .id(SharedString::from(id))
        .absolute()
        .top(px(0.0))
        .right(px(0.0))
        .h_full()
        .w(px(5.0))
        .cursor(gpui::CursorStyle::ResizeLeftRight)
        .on_mouse_down(
            MouseButton::Left,
            cx.listener(move |this, event: &gpui::MouseDownEvent, _window, cx| {
                let page = &mut this.software_pages2.mavftp;
                if event.click_count == 2 {
                    page.fit_column(column);
                } else {
                    page.begin_drag(Dragged::Divider(column), f32::from(event.position.x));
                }
                cx.stop_propagation();
                cx.notify();
            }),
        )
}

/// The header, its columns in the order shown: a click sorts by the column
/// (`ListView1_ColumnClick`), a drag moves it (`AllowColumnReorder`), its divider sizes it, the
/// right button opens the list's menu.
fn header_strip(page: &MavFtp, widths: [f32; 4], cx: &mut Context<MissionPlanner>) -> AnyElement {
    let mut header = div()
        .flex()
        .flex_shrink_0()
        .h(px(LIST_ROW))
        .border_b_1()
        .border_color(rgb(theme::BORDER))
        .on_mouse_down(
            MouseButton::Right,
            cx.listener(|this, event: &gpui::MouseDownEvent, _window, cx| {
                let at = (f32::from(event.position.x), f32::from(event.position.y));
                this.software_pages2.mavftp.open_menu_on_header(at);
                cx.stop_propagation();
                cx.notify();
            }),
        );
    for (shown, column) in page.column_order().into_iter().enumerate() {
        let Some((text, _)) = COLUMNS.get(column) else {
            continue;
        };
        let w = widths.get(column).copied().unwrap_or(0.0);
        let id = format!("mavftp-col-{column}");
        let drag = ColumnDrag {
            column,
            text: SharedString::from(*text),
        };
        header = header.child(
            crate::probe::measured(id.clone(), div())
                .id(SharedString::from(id))
                .relative()
                .flex_shrink_0()
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
                .child(drop_half(shown, false, cx))
                .child(drop_half(shown + 1, true, cx))
                .child(divider(column, cx))
                .on_drag(drag, |drag, _offset, _window, cx| {
                    let text = drag.text.clone();
                    cx.new(|_| ColumnGhost(text))
                })
                .on_click(cx.listener(move |this, _event, window, cx| {
                    this.software_pages2.mavftp.click_column(column);
                    focus_keys(this, window, cx);
                    cx.stop_propagation();
                    cx.notify();
                })),
        );
    }
    header.into_any_element()
}

/// The list, right of the splitter: the header, then the rows under it - which scroll, the
/// header staying - with the name being edited in place.
fn list_panel(
    page: &MavFtp,
    rename: &FocusHandle,
    window: &Window,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let (left, width) = page.list_bounds();
    let height = PAGE_SIZE.1 - STATUS_HEIGHT;
    let widths = page.column_widths();
    let order = page.column_order();
    let mut rows = div()
        .id("mavftp-rows")
        .flex()
        .flex_col()
        .h(px(height - LIST_ROW))
        .overflow_y_scroll()
        .track_scroll(&page.scroll);
    let renaming = page.renaming();
    for (index, item) in page.items().iter().enumerate() {
        let id = format!("mavftp-item-{}", item.name);
        let selected = page.selection().contains(&index);
        let mut row = crate::probe::measured(id.clone(), div())
            .id(SharedString::from(id))
            .flex()
            .flex_shrink_0()
            .items_center()
            .h(px(LIST_ROW))
            .text_xs()
            .whitespace_nowrap()
            .text_color(rgb(theme::TEXT))
            .when(selected, |row| row.bg(rgb(theme::ACCENT)))
            .on_click(
                cx.listener(move |this, event: &gpui::ClickEvent, window, cx| {
                    let page = &mut this.software_pages2.mavftp;
                    if event.click_count() == 2 {
                        page.double_click();
                    } else {
                        page.click_item(index, Mods::of(event.modifiers()), Instant::now());
                    }
                    focus_keys(this, window, cx);
                    cx.stop_propagation();
                    cx.notify();
                }),
            )
            .on_mouse_down(
                MouseButton::Right,
                cx.listener(move |this, event: &gpui::MouseDownEvent, window, cx| {
                    let at = (f32::from(event.position.x), f32::from(event.position.y));
                    let mods = Mods::of(event.modifiers);
                    this.software_pages2.mavftp.open_menu(Some(index), mods, at);
                    focus_keys(this, window, cx);
                    cx.stop_propagation();
                    cx.notify();
                }),
            );
        for column in order {
            let w = widths.get(column).copied().unwrap_or(0.0);
            let cell = div()
                .w(px(w))
                .flex_shrink_0()
                .px_1()
                .overflow_hidden()
                .flex()
                .items_center();
            let cell = if column == 0 {
                match renaming {
                    // The edit box takes its own clicks: one in it places the caret and does
                    // not end the edit, as a click on the row would.
                    Some((editing, field)) if editing == index => cell.child(
                        div()
                            .id("mavftp-rename-box")
                            .on_click(|_event, _window, cx| cx.stop_propagation())
                            .child(crate::textfield::text_field(
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
                            )),
                    ),
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
    // The columns as wide as they are: wider than the list, it scrolls across, header and rows.
    let across = widths.iter().sum::<f32>().max(width - 2.0);
    crate::probe::measured("mavftp-list", div())
        .id("mavftp-list")
        .absolute()
        .left(px(left))
        .top(px(0.0))
        .w(px(width))
        .h(px(height))
        .overflow_hidden()
        .border_1()
        .border_color(rgb(theme::BORDER))
        .bg(rgb(theme::BG))
        .child(
            div()
                .id("mavftp-across")
                .size_full()
                .overflow_x_scroll()
                .child(
                    div()
                        .flex()
                        .flex_col()
                        .w(px(across))
                        .h_full()
                        .child(header_strip(page, widths, cx))
                        .child(rows),
                ),
        )
        // Where there is no row: the selection cleared, and a double click is the C#'s, which
        // reads the selection.
        .on_click(cx.listener(|this, event: &gpui::ClickEvent, window, cx| {
            let page = &mut this.software_pages2.mavftp;
            if event.click_count() == 2 {
                page.double_click();
            } else {
                page.click_list(Mods::of(event.modifiers()));
            }
            focus_keys(this, window, cx);
            cx.notify();
        }))
        .on_mouse_down(
            MouseButton::Right,
            cx.listener(|this, event: &gpui::MouseDownEvent, window, cx| {
                let at = (f32::from(event.position.x), f32::from(event.position.y));
                let mods = Mods::of(event.modifiers);
                this.software_pages2.mavftp.open_menu(None, mods, at);
                focus_keys(this, window, cx);
                cx.notify();
            }),
        )
        // `ListView1_DragEnter` lets only files in (`DataFormats.FileDrop`, `DragDropEffects.Copy`,
        // else `None`): gpui offers the list a drop of paths alone. `ListView1_DragDrop` uploads
        // each. `// C#: Controls/MavFTPUI.cs:279-298, 579-585`
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

/// The keyboard to the box a handler opened: an `InputBox`'s text, or the name being edited;
/// else back to the list, as a `ContextMenuStrip` gives it back when it closes.
fn take_focus(this: &MissionPlanner, window: &mut Window, cx: &mut Context<MissionPlanner>) {
    let page = &this.software_pages2.mavftp;
    if page.prompt().is_some() {
        this.software2_focus.prompt.focus(window, cx);
    } else if page.renaming().is_some() {
        this.software2_focus.rename.focus(window, cx);
    } else if page.keys_to().is_some() {
        focus_keys(this, window, cx);
    }
}

/// The context menu, where it was opened - at the pointer, or in the list's middle from the
/// keyboard - over the page, as a `ContextMenuStrip` is its own window.
fn context_menu(page: &MavFtp, at: MenuAt, cx: &mut Context<MissionPlanner>) -> AnyElement {
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
    match at {
        MenuAt::Pointer(x, y) => gpui::deferred(
            gpui::anchored()
                .position(gpui::point(px(x), px(y)))
                .snap_to_window()
                .child(menu),
        )
        .with_priority(1)
        .into_any_element(),
        // `new Point(Width / 2, Height / 2)` of the list: anchored where a box placed there sits.
        MenuAt::Middle => {
            let (left, width) = page.list_bounds();
            div()
                .absolute()
                .left(px(left + width / 2.0))
                .top(px((PAGE_SIZE.1 - STATUS_HEIGHT) / 2.0))
                .child(
                    gpui::deferred(gpui::anchored().snap_to_window().child(menu)).with_priority(1),
                )
                .into_any_element()
        }
    }
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
    // The name the edit timer opened takes the keyboard, once the box is drawn.
    if page.edit_focus.take() {
        cx.defer_in(window, |this, window, cx| {
            this.software2_focus.rename.focus(window, cx);
        });
    }
    let keys = page.keys.get_or_init(|| cx.focus_handle());
    let (mx, my, mw, mh) = MOUNT;
    let body = div()
        .relative()
        .w(px(PAGE_SIZE.0))
        .h(px(PAGE_SIZE.1))
        // The tree's and the list's keys: whichever was clicked last takes them.
        .track_focus(keys)
        .on_key_down(cx.listener(|this, event: &KeyDownEvent, _window, cx| {
            let stroke = &event.keystroke;
            let mods = Mods::of(stroke.modifiers);
            if this.software_pages2.mavftp.key(&stroke.key, mods) {
                cx.stop_propagation();
                cx.notify();
            }
        }))
        // The splitter or a divider follows the pointer while the button is down.
        .on_mouse_move(
            cx.listener(|this, event: &gpui::MouseMoveEvent, _window, cx| {
                let page = &mut this.software_pages2.mavftp;
                if event.pressed_button == Some(MouseButton::Left) {
                    if page.drag_to(f32::from(event.position.x)) {
                        cx.notify();
                    }
                } else {
                    page.end_drag();
                }
            }),
        )
        .on_mouse_up(
            MouseButton::Left,
            cx.listener(|this, _event: &gpui::MouseUpEvent, _window, _cx| {
                this.software_pages2.mavftp.end_drag();
            }),
        )
        .child(tree_panel(page, cx))
        .child(splitter(page, cx))
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
        .children(page.menu().map(|at| context_menu(page, at, cx)));
    panel(TITLE, body).into_any_element()
}

/// `splitContainer1`'s splitter, between the tree and the list: pressed, it follows the pointer.
fn splitter(page: &MavFtp, cx: &mut Context<MissionPlanner>) -> AnyElement {
    crate::probe::measured("mavftp-splitter", div())
        .id("mavftp-splitter")
        .absolute()
        .left(px(page.splitter()))
        .top(px(0.0))
        .w(px(SPLITTER.1))
        .h(px(PAGE_SIZE.1 - STATUS_HEIGHT))
        .cursor(gpui::CursorStyle::ResizeLeftRight)
        .on_mouse_down(
            MouseButton::Left,
            cx.listener(|this, event: &gpui::MouseDownEvent, _window, cx| {
                this.software_pages2
                    .mavftp
                    .begin_drag(Dragged::Splitter, f32::from(event.position.x));
                cx.stop_propagation();
            }),
        )
        .into_any_element()
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
        |this, event| {
            let used = this.software_pages2.mavftp.prompt_key(event);
            keep_answer(this);
            used
        },
        |this| {
            this.software_pages2.mavftp.close_prompt(true);
            keep_answer(this);
        },
        |this| this.software_pages2.mavftp.close_prompt(false),
        cx,
    ))
}

/// New Folder's or "Mount as Drive"'s answer kept as `InputBox` keeps it, after a key or a button
/// that may have closed the box with OK: the page object holds no settings, the window does.
/// `// C#: Controls/MavFTPUI.cs:518, 693; ExtLibs/Controls/InputBox.cs:178-184`
fn keep_answer(this: &mut MissionPlanner) {
    if let Some(input) = this.software_pages2.mavftp.take_answered() {
        input.remember(&mut this.persisted);
    }
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

    fn row(page: &MavFtp, name: &str) -> usize {
        page.items()
            .iter()
            .position(|item| item.name == name)
            .expect("the row")
    }

    fn select(page: &mut MavFtp, name: &str) {
        let index = row(page, name);
        page.click_item(index, Mods::default(), Instant::now());
    }

    const SHIFT: Mods = Mods {
        control: false,
        shift: true,
    };

    const CONTROL: Mods = Mods {
        control: true,
        shift: false,
    };

    fn selected_names(page: &MavFtp) -> Vec<&str> {
        page.selection()
            .iter()
            .filter_map(|index| page.items().get(*index))
            .map(|item| item.name.as_str())
            .collect()
    }

    fn scratch(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!(
            "headless-planner-mavftp-{name}-{}",
            std::process::id()
        ));
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
            eprintln!("skipped: MP_SRC does not name a clone of https://github.com/ArduPilot/MissionPlanner");
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
        let param = row(&page, "param.pck");
        page.open_menu(Some(param), Mods::default(), (0.0, 0.0));
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
        page.click_item(last, Mods::default(), Instant::now());
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

    /// New Folder's and "Mount as Drive"'s OK keep the answer as `InputBox` keeps every titled
    /// answer; Cancel keeps nothing, and neither do the two dialogs' stand-ins.
    /// `// C#: Controls/MavFTPUI.cs:518, 693; ExtLibs/Controls/InputBox.cs:73-84, 178-184`
    #[test]
    fn the_input_boxes_ok_keeps_the_answer_under_the_input_box_key() {
        let folder_key = crate::config::optional::answers_key(FOLDER_TITLE, FOLDER_PROMPT);
        let mount_key = crate::config::optional::answers_key(MOUNT_TITLE, MOUNT_PROMPT);
        assert_eq!(folder_key, "InputBoxFolderNameEnterfoldername");
        assert_eq!(mount_key, "InputBoxMountPointEnterdriveletterorpathegM");
        for key in [&folder_key, &mount_key] {
            assert!(crate::settings::PUBLISHED.contains(&key.as_str()), "{key}");
        }
        let bench = Bench::new(vehicle());
        let mut page = loaded(&bench);
        page.click_node("/APM");
        settle(&mut page, &bench);
        page.choose(Menu::NewFolder);
        page.type_prompt("gone");
        page.close_prompt(false);
        assert!(page.take_answered().is_none(), "Cancel keeps nothing");
        page.choose(Menu::NewFolder);
        page.type_prompt("fresh");
        page.close_prompt(true);
        settle(&mut page, &bench);
        let mut settings = crate::settings::Persisted::at(None);
        page.take_answered()
            .expect("OK's box")
            .remember(&mut settings);
        assert!(page.take_answered().is_none(), "kept once");
        assert_eq!(settings.get(&folder_key), Some("fresh"));
        page.press_mount();
        page.close_prompt(true);
        page.take_answered()
            .expect("OK's box")
            .remember(&mut settings);
        assert_eq!(settings.get(&mount_key), Some("M%3A%5C"));
        let param = row(&page, "param.pck");
        page.open_menu(Some(param), Mods::default(), (0.0, 0.0));
        page.choose(Menu::Download);
        page.type_prompt(&scratch("answered").display().to_string());
        page.close_prompt(true);
        settle(&mut page, &bench);
        assert!(page.take_answered().is_none(), "a dialog keeps nothing");
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

    /// The pages the keys move by are the page's geometry: the list's client height under the
    /// header over a row's, the tree's over a node's.
    #[test]
    fn the_keys_pages_are_the_geometry() {
        let rows = (PAGE_SIZE.1 - STATUS_HEIGHT - LIST_ROW) / LIST_ROW;
        let nodes = (PAGE_SIZE.1 - STATUS_HEIGHT) / TREE_ROW;
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let (rows, nodes) = (rows.floor() as usize, nodes.floor() as usize);
        assert_eq!(LIST_PAGE, rows);
        assert_eq!(TREE_PAGE, nodes);
    }

    /// The File name box takes several names in double quotes, as `OpenFileDialog` does with
    /// `Multiselect`; a line without quotes is one name, spaces and all.
    #[test]
    fn file_names_are_the_dialogs() {
        assert_eq!(
            file_names("\"/a/one.txt\" \"/b/two words.txt\""),
            [
                PathBuf::from("/a/one.txt"),
                PathBuf::from("/b/two words.txt")
            ]
        );
        assert_eq!(
            file_names("  /c/a file.lua "),
            [PathBuf::from("/c/a file.lua")]
        );
        assert!(file_names("   ").is_empty());
        assert!(file_names("\"\" \" \"").is_empty());
    }

    /// Upload takes every file named; Cancel, or a name of no file, uploads nothing and the
    /// directory is listed again all the same (`MavFTPUI.cs:402-403` is outside the `if`).
    #[test]
    fn upload_takes_several_files_and_lists_again_on_cancel() {
        let bench = Bench::new(vehicle());
        let mut page = loaded(&bench);
        page.click_node("/APM");
        settle(&mut page, &bench);
        let folder = scratch("several");
        let one = folder.join("one.lua");
        let two = folder.join("two words.lua");
        std::fs::write(&one, b"print(1)").expect("a file");
        std::fs::write(&two, b"print(2)").expect("a file");
        page.choose(Menu::Upload);
        page.type_prompt(&format!("\"{}\" \"{}\"", one.display(), two.display()));
        page.close_prompt(true);
        settle(&mut page, &bench);
        {
            let vehicle = bench.vehicle.borrow();
            assert_eq!(
                vehicle.files.get("/APM/one.lua").map(Vec::as_slice),
                Some(&b"print(1)"[..])
            );
            assert_eq!(
                vehicle.files.get("/APM/two words.lua").map(Vec::as_slice),
                Some(&b"print(2)"[..])
            );
        }
        assert!(names(&page).contains(&"one.lua"));
        assert!(names(&page).contains(&"two words.lua"));
        assert!(page.take_status_line().is_none(), "both CRCs agree");
        for (ok, text) in [
            (false, String::new()),
            (true, "/no/such/file.lua".to_owned()),
        ] {
            let before = bench.started.borrow().len();
            page.choose(Menu::Upload);
            page.type_prompt(&text);
            page.close_prompt(ok);
            settle(&mut page, &bench);
            let started = bench.started.borrow();
            let since: Vec<&FtpRequest> = started.iter().skip(before).collect();
            assert_eq!(
                since,
                [&FtpRequest::List {
                    path: "/APM".to_owned()
                }],
                "listed again, nothing written"
            );
        }
        let _ = std::fs::remove_dir_all(&folder);
    }

    /// A node's plus or minus toggles it and lists it, and so does the right button anywhere on
    /// a node's row: `NodeMouseClick` with the node clicked, `SelectedNode` left where it was. The
    /// second click of a double click on the plus toggles it back and lists nothing.
    #[test]
    fn the_plus_and_the_right_button_list_a_node_and_leave_the_selection() {
        let bench = Bench::new(vehicle());
        let mut page = loaded(&bench);
        assert_eq!(page.selected_node(), Some("@SYS/"));
        page.toggle_node("/", 1);
        assert!(page.tree().first().is_some_and(|node| node.expanded));
        settle(&mut page, &bench);
        assert_eq!(names(&page), ["APM", "@SYS"]);
        assert_eq!(page.selected_node(), Some("@SYS/"), "the selection stays");
        assert_eq!(page.keys_to(), Some(Control::Tree));
        let before = bench.started.borrow().len();
        page.toggle_node("/", 2);
        settle(&mut page, &bench);
        assert!(page.tree().first().is_some_and(|node| !node.expanded));
        assert_eq!(bench.started.borrow().len(), before, "no listing");
        page.toggle_node("/", 1);
        settle(&mut page, &bench);
        page.click_node_row("/APM");
        settle(&mut page, &bench);
        assert_eq!(names(&page), ["LOGS", "scripts", "param.pck"]);
        assert_eq!(page.selected_node(), Some("@SYS/"));
        // Upload, Download and the rest go where `SelectedNode` is, as the C#'s do.
        assert_eq!(page.selected_full_path().as_deref(), Some("@SYS"));
        // Delete reads the row's tag: the directory listed.
        assert!(page.items().iter().all(|item| item.dir == "/APM"));
    }

    /// The text of a node double-clicked: the first click selects and lists it, the second only
    /// toggles it.
    #[test]
    fn a_double_click_on_a_node_toggles_it() {
        let bench = Bench::new(vehicle());
        let mut page = loaded(&bench);
        page.click_node("/");
        settle(&mut page, &bench);
        let before = bench.started.borrow().len();
        page.double_click_node("/");
        settle(&mut page, &bench);
        assert!(page.tree().first().is_some_and(|node| node.expanded));
        assert_eq!(bench.started.borrow().len(), before, "no listing");
        page.double_click_node("/");
        assert!(page.tree().first().is_some_and(|node| !node.expanded));
    }

    /// A node closed or listed while the selection is under it takes the selection: comctl32's
    /// `TREEVIEW_Collapse`, and `Nodes.Clear()` deleting the selected node.
    #[test]
    fn closing_or_listing_a_node_above_the_selection_selects_it() {
        let bench = Bench::new(vehicle());
        let mut page = loaded(&bench);
        page.click_node("/");
        settle(&mut page, &bench);
        select(&mut page, "APM");
        page.double_click();
        settle(&mut page, &bench);
        assert_eq!(page.selected_node(), Some("/APM"));
        page.toggle_node("/", 1);
        assert_eq!(page.selected_node(), Some("/"), "closed: the node above");
        settle(&mut page, &bench);
        // Open again, the selection under it, and the node listed by the right button.
        page.toggle_node("/", 1);
        settle(&mut page, &bench);
        page.click_node("/APM");
        settle(&mut page, &bench);
        assert_eq!(page.selected_node(), Some("/APM"));
        page.click_node_row("/");
        settle(&mut page, &bench);
        assert_eq!(page.selected_node(), Some("/"), "listed: the node above");
    }

    /// The tree's keys move the selection through the nodes drawn and open and close them, and
    /// list nothing: the C# wires no `AfterSelect`.
    #[test]
    fn the_trees_keys_move_the_selection_and_list_nothing() {
        let bench = Bench::new(vehicle());
        let mut page = loaded(&bench);
        page.click_node("@SYS/");
        settle(&mut page, &bench);
        let listed = names(&page).join(",");
        let before = bench.started.borrow().len();
        let none = Mods::default();
        let press = |page: &mut MavFtp, key: &str| {
            assert!(page.key(key, none), "{key} is the tree's");
            page.selected_node().map(str::to_owned)
        };
        assert_eq!(press(&mut page, "up").as_deref(), Some("/"));
        assert_eq!(
            press(&mut page, "up").as_deref(),
            Some("/"),
            "the first stays"
        );
        assert_eq!(press(&mut page, "right").as_deref(), Some("/"), "opened");
        assert!(page.tree().first().is_some_and(|node| node.expanded));
        assert_eq!(press(&mut page, "right").as_deref(), Some("/APM"));
        assert_eq!(press(&mut page, "down").as_deref(), Some("/@SYS"));
        assert_eq!(press(&mut page, "end").as_deref(), Some("@SYS/"));
        assert_eq!(press(&mut page, "pageup").as_deref(), Some("/"));
        assert_eq!(press(&mut page, "pagedown").as_deref(), Some("@SYS/"));
        assert_eq!(press(&mut page, "home").as_deref(), Some("/"));
        assert_eq!(press(&mut page, "down").as_deref(), Some("/APM"));
        assert_eq!(
            press(&mut page, "left").as_deref(),
            Some("/"),
            "to the parent"
        );
        assert_eq!(press(&mut page, "left").as_deref(), Some("/"), "closed");
        assert!(page.tree().first().is_some_and(|node| !node.expanded));
        assert_eq!(press(&mut page, "add").as_deref(), Some("/"));
        assert!(page.tree().first().is_some_and(|node| node.expanded));
        assert_eq!(press(&mut page, "subtract").as_deref(), Some("/"));
        assert!(page.tree().first().is_some_and(|node| !node.expanded));
        assert_eq!(press(&mut page, "multiply").as_deref(), Some("/"));
        assert!(page.tree().first().is_some_and(|node| node.expanded));
        assert_eq!(press(&mut page, "down").as_deref(), Some("/APM"));
        assert_eq!(press(&mut page, "backspace").as_deref(), Some("/"));
        assert_eq!(
            press(&mut page, "backspace").as_deref(),
            Some("/"),
            "a root"
        );
        // With Control the view scrolls; the selection stays.
        let control = Mods {
            control: true,
            shift: false,
        };
        assert!(page.key("down", control));
        assert_eq!(page.selected_node(), Some("/"));
        assert!(!page.key("x", none), "not the tree's");
        settle(&mut page, &bench);
        assert_eq!(bench.started.borrow().len(), before, "nothing listed");
        assert_eq!(names(&page).join(","), listed);
    }

    /// Clicks select as a `MultiSelect` list does: alone, with Shift from the mark, with Control
    /// toggled, with both added; a press where there is no row clears the selection; the right
    /// button selects a row alone unless it is selected already.
    #[test]
    fn clicks_select_as_the_list_does() {
        let bench = Bench::new(vehicle());
        let mut page = loaded(&bench);
        page.click_node("/APM");
        settle(&mut page, &bench);
        assert_eq!(names(&page), ["LOGS", "scripts", "param.pck"]);
        let now = Instant::now();
        page.click_item(0, Mods::default(), now);
        page.click_item(2, SHIFT, now);
        assert_eq!(selected_names(&page), ["LOGS", "scripts", "param.pck"]);
        page.click_item(1, CONTROL, now);
        assert_eq!(selected_names(&page), ["LOGS", "param.pck"]);
        let both = Mods {
            control: true,
            shift: true,
        };
        page.click_item(2, both, now);
        assert_eq!(selected_names(&page), ["LOGS", "scripts", "param.pck"]);
        page.click_item(2, SHIFT, now);
        assert_eq!(
            selected_names(&page),
            ["scripts", "param.pck"],
            "from the mark"
        );
        page.click_list(CONTROL);
        assert_eq!(
            selected_names(&page),
            ["scripts", "param.pck"],
            "Control keeps"
        );
        page.click_list(Mods::default());
        assert!(page.selection().is_empty());
        page.click_item(0, Mods::default(), now);
        page.click_item(1, CONTROL, now);
        page.open_menu(Some(1), Mods::default(), (0.0, 0.0));
        assert_eq!(
            selected_names(&page),
            ["LOGS", "scripts"],
            "selected already"
        );
        page.close_menu();
        page.open_menu(Some(2), Mods::default(), (0.0, 0.0));
        assert_eq!(selected_names(&page), ["param.pck"]);
        assert_eq!(page.focused(), Some(2));
        page.close_menu();
        page.open_menu(None, Mods::default(), (0.0, 0.0));
        assert!(page.selection().is_empty());
        assert_eq!(page.menu(), Some(MenuAt::Pointer(0.0, 0.0)));
        page.close_menu();
        // The header's right button leaves the selection.
        page.click_item(1, Mods::default(), now);
        page.open_menu_on_header((5.0, 5.0));
        assert_eq!(selected_names(&page), ["scripts"]);
        assert!(page.menu().is_some());
    }

    /// The list's keys: a row selected alone, with Shift from the mark, with Control the focus
    /// alone moved and Control+Space toggling; Left and Right move nothing.
    #[test]
    fn the_lists_keys_select_as_the_list_does() {
        let bench = Bench::new(vehicle());
        let mut page = loaded(&bench);
        page.click_node("/APM");
        settle(&mut page, &bench);
        page.click_item(0, Mods::default(), Instant::now());
        let none = Mods::default();
        assert!(page.key("down", none));
        assert_eq!(selected_names(&page), ["scripts"]);
        assert!(page.key("down", SHIFT));
        assert_eq!(selected_names(&page), ["scripts", "param.pck"]);
        assert_eq!(page.focused(), Some(2));
        assert!(page.key("down", none), "the last row: nothing moves");
        assert_eq!(selected_names(&page), ["scripts", "param.pck"]);
        assert!(page.key("home", none));
        assert_eq!(selected_names(&page), ["LOGS"]);
        assert!(page.key("down", CONTROL));
        assert_eq!(page.focused(), Some(1));
        assert_eq!(selected_names(&page), ["LOGS"], "the focus alone");
        assert!(page.key("space", CONTROL));
        assert_eq!(selected_names(&page), ["LOGS", "scripts"]);
        assert!(page.key("space", CONTROL));
        assert_eq!(selected_names(&page), ["LOGS"]);
        assert!(page.key("end", none));
        assert_eq!(selected_names(&page), ["param.pck"]);
        assert!(page.key("pageup", none));
        assert_eq!(selected_names(&page), ["LOGS"], "to the top row");
        assert!(page.key("pagedown", none));
        assert_eq!(
            selected_names(&page),
            ["param.pck"],
            "to the last row there is"
        );
        assert!(page.key("up", none));
        assert_eq!(selected_names(&page), ["scripts"]);
        assert!(page.key("left", none) && page.key("right", none));
        assert_eq!(selected_names(&page), ["scripts"]);
        assert!(!page.key("x", none), "not the list's");
        // Sorted, the focus and the mark stay with their rows.
        page.click_column(0);
        assert_eq!(names(&page), ["scripts", "param.pck", "LOGS"]);
        assert_eq!(page.focused(), Some(0));
        assert!(page.key("down", SHIFT));
        assert_eq!(selected_names(&page), ["scripts", "param.pck"]);
    }

    /// The menu key and Shift+F10 open the list's menu in its middle; while it is open it has
    /// the keys, and Escape closes it. Nothing takes keys while a box is over the page.
    #[test]
    fn the_menu_key_opens_the_menu_and_boxes_hold_the_keys() {
        let bench = Bench::new(vehicle());
        let mut page = loaded(&bench);
        page.click_node("/APM");
        settle(&mut page, &bench);
        page.click_item(0, Mods::default(), Instant::now());
        assert!(page.key("menu", Mods::default()));
        assert_eq!(page.menu(), Some(MenuAt::Middle));
        assert!(page.key("down", Mods::default()));
        assert_eq!(selected_names(&page), ["LOGS"], "the menu has the keys");
        assert!(page.key("escape", Mods::default()));
        assert_eq!(page.menu(), None);
        assert!(page.key("f10", SHIFT));
        assert_eq!(page.menu(), Some(MenuAt::Middle));
        page.choose(Menu::NewFolder);
        assert!(!page.key("down", Mods::default()), "the box has the keys");
        page.close_prompt(false);
        settle(&mut page, &bench);
        // The tree has no menu.
        page.click_node("/APM");
        settle(&mut page, &bench);
        assert!(!page.key("menu", Mods::default()));
        assert_eq!(page.menu(), None);
    }

    /// A click on a row already selected edits its name once the double-click time has passed;
    /// a second click within it - a double click - does not, nor does a first click, nor a row
    /// no longer selected.
    #[test]
    fn a_second_click_on_a_selected_row_edits_its_name() {
        let bench = Bench::new(vehicle());
        let mut page = loaded(&bench);
        page.click_node("/APM");
        settle(&mut page, &bench);
        let view = view();
        let t0 = Instant::now();
        let param = row(&page, "param.pck");
        page.click_item(param, Mods::default(), t0);
        page.tick(&bench, &view, true, t0 + DOUBLE_CLICK_TIME * 2);
        assert!(page.renaming().is_none(), "a first click");
        page.click_item(param, Mods::default(), t0);
        page.tick(&bench, &view, true, t0 + DOUBLE_CLICK_TIME / 2);
        assert!(page.renaming().is_none(), "not yet");
        page.tick(&bench, &view, true, t0 + DOUBLE_CLICK_TIME);
        assert_eq!(
            page.renaming()
                .map(|(_, field)| field.value().to_owned())
                .as_deref(),
            Some("param.pck")
        );
        assert!(page.edit_focus.take(), "the edit box takes the keyboard");
        // Escape: the edit ends with no label.
        page.renaming = None;
        page.click_item(param, Mods::default(), t0);
        page.double_click();
        page.tick(&bench, &view, true, t0 + DOUBLE_CLICK_TIME * 2);
        assert!(page.renaming().is_none(), "a double click");
        page.click_item(param, Mods::default(), t0);
        page.click_item(0, Mods::default(), t0);
        page.tick(&bench, &view, true, t0 + DOUBLE_CLICK_TIME * 2);
        assert!(page.renaming().is_none(), "no longer selected");
    }

    /// A name left as it was renames nothing: `e.Label` is null and the C# returns.
    #[test]
    fn an_unchanged_name_renames_nothing() {
        let bench = Bench::new(vehicle());
        let mut page = loaded(&bench);
        page.click_node("/APM");
        settle(&mut page, &bench);
        select(&mut page, "param.pck");
        page.choose(Menu::Rename);
        let before = bench.started.borrow().len();
        page.commit_rename();
        settle(&mut page, &bench);
        assert_eq!(bench.started.borrow().len(), before);
        assert!(page.renaming().is_none());
    }

    /// A header dragged to a divider moves there, as comctl32's header moves it; the rows follow,
    /// and a click still sorts by the header's own column.
    #[test]
    fn a_header_dragged_moves_its_column() {
        let mut page = MavFtp::default();
        page.drop_column(3, 0);
        assert_eq!(page.column_order(), [3, 0, 1, 2]);
        page.drop_column(3, 4);
        assert_eq!(page.column_order(), [0, 1, 2, 3], "to the end");
        page.drop_column(0, 2);
        assert_eq!(
            page.column_order(),
            [1, 0, 2, 3],
            "one less right of where it was"
        );
        page.drop_column(0, 1);
        assert_eq!(
            page.column_order(),
            [1, 0, 2, 3],
            "its own divider: no move"
        );
        page.drop_column(2, 0);
        assert_eq!(page.column_order(), [2, 1, 0, 3]);
        let bench = Bench::new(vehicle());
        let mut page = loaded(&bench);
        page.click_node("/APM");
        settle(&mut page, &bench);
        page.drop_column(0, 4);
        page.click_column(0);
        assert_eq!(names(&page), ["scripts", "param.pck", "LOGS"], "by Name");
    }

    /// The splitter follows the pointer, never nearer an edge than a panel's `MinSize`; the list
    /// sits right of it.
    #[test]
    fn the_splitter_drags_within_the_panels_minimum() {
        let mut page = MavFtp::default();
        assert_eq!(page.splitter(), 208.0, "the Designer's SplitterDistance");
        assert_eq!(page.list_bounds(), (212.0, 676.0));
        page.begin_drag(Dragged::Splitter, 500.0);
        assert!(page.drag_to(560.0));
        assert_eq!(page.splitter(), 268.0);
        assert_eq!(page.list_bounds(), (272.0, 616.0));
        page.drag_to(-1000.0);
        assert_eq!(page.splitter(), PANEL_MIN);
        page.drag_to(5000.0);
        assert_eq!(page.splitter(), PAGE_SIZE.0 - PANEL_MIN - SPLITTER.1);
        page.end_drag();
        assert!(!page.drag_to(0.0), "let go");
        assert_eq!(page.splitter(), PAGE_SIZE.0 - PANEL_MIN - SPLITTER.1);
    }

    /// A divider dragged sizes its column, never under nothing; double-clicked it fits the column
    /// to its texts; the next listing sizes every column again.
    #[test]
    fn a_divider_sizes_its_column_until_the_next_listing() {
        let bench = Bench::new(vehicle());
        let mut page = loaded(&bench);
        page.click_node("/APM");
        settle(&mut page, &bench);
        let auto = page.column_widths();
        let name = auto.first().copied().expect("Name");
        let first = |page: &MavFtp| page.column_widths().first().copied();
        page.begin_drag(Dragged::Divider(0), 300.0);
        page.drag_to(340.0);
        assert_eq!(first(&page), Some(name + 40.0));
        assert_eq!(
            page.column_widths().get(1..),
            auto.get(1..),
            "the others stay"
        );
        page.drag_to(-1000.0);
        assert_eq!(first(&page), Some(0.0));
        page.end_drag();
        // "param.pck", the longest name: nine characters.
        page.fit_column(0);
        assert_eq!(first(&page), Some(text_width(9)));
        page.click_node("/APM");
        settle(&mut page, &bench);
        assert_eq!(page.column_widths(), auto, "AutoResizeColumns");
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
