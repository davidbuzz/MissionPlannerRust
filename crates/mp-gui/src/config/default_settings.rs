//! `Controls/DefaultSettings.cs`: the "Default Settings" control the older Frame Type page
//! ([`super::frame_type_legacy`]) holds in its `groupBox1` - a note, a combo box of ArduPilot's
//! standard frame parameter files and Load Params.
//!
//! What it does:
//!
//! * `Load`, the first time the control shows, runs its `Activate`: the combo box and the button
//!   disabled, and on a worker thread the listing of `Tools/Frame_params` in ArduPilot's
//!   repository from GitHub's contents API, the `.param` files kept (`GitHubContent.GetDirContent`,
//!   [`mp_firmware::github`]). When it comes the combo box lists their names, the first chosen,
//!   and both controls are enabled. A listing that fails is logged and nothing more: the combo box
//!   goes on reading "Loading", disabled (`DefaultSettings.cs:25-60, 103-106`).
//! * Load Params with nothing chosen says "Please select an option first". Otherwise it fetches
//!   the file chosen (`GetFileContent`), saves it in the user data directory under the combo
//!   box's text, reads it as a parameter file and opens `ParamCompare` ([`super::param_compare`])
//!   over the vehicle's parameters and the file's. Its Continue writes the rows ticked; closed
//!   with `OK` the control says "Loaded parameters!". Either way it then raises `OnChange`, on
//!   which the Frame Type page runs its `Activate` again. Anything that throws on the way says
//!   "Failed to load file." and the exception (`:62-101`).
//!
//! What the C# does on the UI thread, blocking - the file's fetch - runs on a worker thread here,
//! the button disabled until it ends; what `Task.Run` does runs on one too. Neither is started by
//! this module: it asks for a fetch ([`Want`]), which the application's tick starts
//! ([`DefaultSettings::dispatch`]), so the unit tests answer the asks themselves.
//!
//! Where the C# shows a box for a link error - "Failed to load file." with the exception, and
//! `ParamCompare`'s "Error setting parameter" - this application says it on the status line, by
//! the owner's ruling of 2026-09-25; "Please select an option first" and "Loaded parameters!"
//! keep their boxes.
//!
//! One divergence: `CMB_paramfiles` is a `DropDown` combo box, so its text can be typed over, and
//! a typed name becomes the saved file's name while the chosen entry is still what is fetched.
//! Here the box is a list only; the saved file is named for the entry chosen, which is what the
//! C# saves when nothing is typed.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::collections::VecDeque;
use std::path::PathBuf;
use std::sync::mpsc::{Receiver, TryRecvError, channel};

use mp_firmware::github::{self, FileInfo};

use super::flight_modes::ParamWriter;
use super::optional::{Event, SetQueue, plain};
use super::param_compare::ParamCompare;
use super::servo_output::{Combo, Message};

/// `GetDirContent("ArduPilot", "ardupilot", "/Tools/Frame_params/", ".param")`.
/// `// C#: Controls/DefaultSettings.cs:41`
pub const OWNER: &str = "ArduPilot";
/// The repository.
pub const REPO: &str = "ardupilot";
/// The directory listed.
pub const DIRECTORY: &str = "/Tools/Frame_params/";
/// What a listed name must contain.
pub const FILTER: &str = ".param";

/// Load Params with nothing chosen: a plain box.
/// `// C#: Controls/DefaultSettings.cs:66-70`
pub const SELECT_FIRST: &str = "Please select an option first";
/// `ParamCompare` closed with `OK`.
/// `// C#: Controls/DefaultSettings.cs:84-87`
pub const LOADED: &str = "Loaded parameters!";
/// That box's caption.
pub const LOADED_TITLE: &str = "Loaded";
/// The start of the box for anything that throws: `"Failed to load file.\n" + ex`.
/// `// C#: Controls/DefaultSettings.cs:97-100`
pub const FAILED: &str = "Failed to load file.\n";

/// The job `ParamCompare`'s Continue makes.
const SAVE_TAG: &str = "defaultsettings-compare";

/// A fetch the control asks for, which [`DefaultSettings::dispatch`] starts on a worker thread.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Want {
    /// The listing of `Tools/Frame_params`.
    Listing,
    /// A file: its path in the repository, and where it is saved.
    File {
        /// `FileInfo.path`.
        path: String,
        /// `Settings.GetUserDataDirectory() + CMB_paramfiles.Text`.
        save_as: PathBuf,
    },
}

/// What a worker thread sends back.
#[derive(Debug)]
pub enum Arrived {
    /// The listing, or why there is none.
    Listing(Result<Vec<FileInfo>, String>),
    /// A file's bytes (`None` for a JSON `null`), or why there are none, and where to save them.
    File {
        /// Where it is saved.
        save_as: PathBuf,
        /// The bytes.
        bytes: Result<Option<Vec<u8>>, String>,
    },
}

/// The control.
#[derive(Debug)]
pub struct DefaultSettings<H = mp_link::RequestId> {
    /// Whether `Load` has run.
    loaded: bool,
    /// `paramfiles`: the listing, once it has come.
    paramfiles: Option<Vec<FileInfo>>,
    /// `CMB_paramfiles`: the listed names by index, and its `Enabled`.
    combo: Combo,
    /// Whether its list is dropped down.
    dropdown: bool,
    /// `BUT_paramfileload.Enabled`.
    button: bool,
    /// A fetch asked for and not yet started.
    want: Option<Want>,
    /// The fetch running.
    fetching: Option<Receiver<Arrived>>,
    /// Why the listing failed, as the C# logs it.
    listing_error: Option<String>,
    /// `ParamCompare`, while it is open.
    compare: Option<ParamCompare>,
    /// Its writes.
    queue: SetQueue<H>,
    /// Boxes, the first showing.
    messages: VecDeque<Message>,
    /// `OnChange` raised and not yet taken by the page.
    changed: bool,
}

impl<H> Default for DefaultSettings<H> {
    /// `InitializeComponent`: the combo box reading "Loading" and the button disabled
    /// (`DefaultSettings.resx` `BUT_paramfileload.Enabled`); the combo box is enabled until `Load`
    /// disables it.
    fn default() -> Self {
        Self {
            loaded: false,
            paramfiles: None,
            combo: Combo {
                enabled: true,
                ..Combo::default()
            },
            dropdown: false,
            button: false,
            want: None,
            fetching: None,
            listing_error: None,
            compare: None,
            queue: SetQueue::default(),
            messages: VecDeque::new(),
            changed: false,
        }
    }
}

impl<H: Copy> DefaultSettings<H> {
    /// Whether `Load` has run.
    #[must_use]
    pub const fn loaded(&self) -> bool {
        self.loaded
    }

    /// The combo box.
    #[must_use]
    pub const fn combo(&self) -> &Combo {
        &self.combo
    }

    /// Whether the listing has come: the combo box shows its names rather than "Loading".
    #[must_use]
    pub const fn listed(&self) -> bool {
        self.paramfiles.is_some()
    }

    /// Whether the combo box's list is down.
    #[must_use]
    pub const fn dropdown(&self) -> bool {
        self.dropdown
    }

    /// `BUT_paramfileload.Enabled`.
    #[must_use]
    pub const fn button_enabled(&self) -> bool {
        self.button
    }

    /// The fetch asked for and not started.
    #[cfg(test)]
    #[must_use]
    pub const fn want(&self) -> Option<&Want> {
        self.want.as_ref()
    }

    /// Whether a fetch is running or asked for.
    #[must_use]
    pub const fn busy(&self) -> bool {
        self.want.is_some() || self.fetching.is_some()
    }

    /// Why the listing failed.
    #[must_use]
    pub fn listing_error(&self) -> Option<&str> {
        self.listing_error.as_deref()
    }

    /// `ParamCompare`, while it is open.
    #[must_use]
    pub const fn compare(&self) -> Option<&ParamCompare> {
        self.compare.as_ref()
    }

    /// Whether `ParamCompare`'s writes are running.
    #[must_use]
    pub fn writing(&self) -> bool {
        self.queue.pending() > 0
    }

    /// How the last write ended.
    #[must_use]
    pub fn last_write(&self) -> Option<&str> {
        self.queue.last()
    }

    /// The box showing.
    #[must_use]
    pub fn message(&self) -> Option<&Message> {
        self.messages.front()
    }

    /// Dismisses it.
    pub fn dismiss_message(&mut self) {
        self.messages.pop_front();
    }

    /// The boxes for link errors taken out of the queue: the status line's words for the last.
    /// "Failed to load file." with its exception, and `ParamCompare`'s error box.
    pub fn take_link_errors(&mut self) -> Option<String> {
        let mut status = None;
        self.messages.retain(|message| {
            if message.text.starts_with(FAILED) || super::extra_setup::link_error(message) {
                status = Some(super::extra_setup::status_words(message));
                false
            } else {
                true
            }
        });
        status
    }

    /// `OnChange`, raised since the last call: the page runs its `Activate`.
    pub const fn take_changed(&mut self) -> bool {
        let changed = self.changed;
        self.changed = false;
        changed
    }

    /// `Load`: the control's `Activate`, once. Both controls disabled, and the listing asked for -
    /// fetched only when this control has none, which a new control never has.
    /// `// C#: Controls/DefaultSettings.cs:25-60, 103-106`
    pub fn load(&mut self) {
        if self.loaded {
            return;
        }
        self.loaded = true;
        self.combo.enabled = false;
        self.button = false;
        match self.paramfiles.clone() {
            Some(files) => self.arrive(Arrived::Listing(Ok(files)), &[]),
            None => self.want = Some(Want::Listing),
        }
    }

    /// Drops the combo box's list down, or back up.
    pub fn toggle_dropdown(&mut self) {
        if self.combo.enabled && !self.combo.options.is_empty() {
            self.dropdown = !self.dropdown;
            if self.dropdown {
                self.combo.open_list();
            }
        } else {
            self.dropdown = false;
        }
    }

    /// The wheel over the dropped-down list.
    pub fn scroll_list(&mut self, lines: i32) {
        self.combo.scroll_list(lines);
    }

    /// A name chosen from the list.
    pub fn choose(&mut self, index: i64) {
        self.dropdown = false;
        self.combo.select(index);
    }

    /// Load Params: "Please select an option first" with nothing chosen, else the chosen file
    /// asked for, to be saved in `user_data` under its name.
    /// `// C#: Controls/DefaultSettings.cs:62-75`
    pub fn click_load(&mut self, user_data: &std::path::Path) {
        if !self.button || self.busy() || self.compare.is_some() {
            return;
        }
        self.dropdown = false;
        let chosen = self
            .combo
            .selected
            .and_then(|index| usize::try_from(index).ok())
            .and_then(|index| self.paramfiles.as_ref()?.get(index));
        let Some(file) = chosen else {
            self.messages.push_back(plain(SELECT_FIRST));
            return;
        };
        // `Settings.GetUserDataDirectory() + CMB_paramfiles.Text`: the directory ends in its
        // separator.
        self.want = Some(Want::File {
            path: file.path.clone(),
            save_as: user_data.join(&file.name),
        });
    }

    /// A fetch's answer. The listing fills the combo box - `DataSource` set, which chooses its
    /// first entry - and enables both controls; a file is saved, read as a parameter file and
    /// compared with the vehicle's `parameters`.
    /// `// C#: Controls/DefaultSettings.cs:47-58, 72-82, 97-100`
    pub fn arrive(&mut self, arrived: Arrived, parameters: &[(String, f64)]) {
        match arrived {
            Arrived::Listing(Ok(files)) => {
                self.combo.options = files
                    .iter()
                    .zip(0_i64..)
                    .map(|(file, index)| (index, file.name.clone()))
                    .collect();
                self.combo.selected = self.combo.options.first().map(|(index, _)| *index);
                self.combo.top_index = 0;
                self.combo.enabled = true;
                self.button = true;
                self.paramfiles = Some(files);
            }
            // `log.Error(ex)`: nothing on the screen.
            Arrived::Listing(Err(why)) => self.listing_error = Some(why),
            Arrived::File { save_as, bytes } => match Self::read(&save_as, bytes) {
                Ok(param2) => self.compare = Some(ParamCompare::new(parameters, &param2)),
                Err(why) => self.messages.push_back(plain(format!("{FAILED}{why}"))),
            },
        }
    }

    /// `File.WriteAllBytes(filepath, data)` and `ParamFile.loadParamFile(filepath)`.
    fn read(
        save_as: &std::path::Path,
        bytes: Result<Option<Vec<u8>>, String>,
    ) -> Result<Vec<(String, f64)>, String> {
        let bytes = bytes?.ok_or("System.ArgumentNullException: Value cannot be null.")?;
        std::fs::write(save_as, &bytes).map_err(|err| err.to_string())?;
        let file =
            mp_params::param_file::ParamFile::load(save_as).map_err(|err| err.to_string())?;
        Ok(file
            .iter()
            .map(|(name, value)| (name.to_owned(), value))
            .collect())
    }

    /// `ParamCompare`'s Continue: its writes, while none are running.
    pub fn click_save(&mut self) {
        if self.writing() {
            return;
        }
        if let Some(form) = &self.compare {
            self.queue.push([form.save(SAVE_TAG)]);
        }
    }

    /// A row's Use box.
    pub fn toggle_row(&mut self, index: usize) {
        if let Some(form) = &mut self.compare {
            form.toggle_row(index);
        }
    }

    /// "Check/Uncheck All".
    pub fn toggle_all(&mut self) {
        if let Some(form) = &mut self.compare {
            form.click_toggle_all();
        }
    }

    /// The form closed by its close box: `DialogResult.Cancel`, and then `OnChange`.
    /// `// C#: Controls/DefaultSettings.cs:84-93`
    pub fn close_compare(&mut self) {
        if !self.writing() && self.compare.take().is_some() {
            self.changed = true;
        }
    }

    /// Once a frame: the fetch's answer taken, and the writes moved on. Continue's writes ending
    /// without a throw close the form with `OK` - "Loaded parameters!" - and raise `OnChange`.
    /// `// C#: Controls/DefaultSettings.cs:81-93; Controls/paramcompare.cs:84-109`
    pub fn tick<W: ParamWriter<Handle = H>>(&mut self, writer: &W, parameters: &[(String, f64)]) {
        if let Some(receiver) = &self.fetching {
            match receiver.try_recv() {
                Ok(arrived) => {
                    self.fetching = None;
                    self.arrive(arrived, parameters);
                }
                Err(TryRecvError::Empty) => {}
                Err(TryRecvError::Disconnected) => {
                    self.fetching = None;
                }
            }
        }
        let events = self.queue.advance(writer, &mut self.messages);
        for event in events {
            if let Event::Done {
                tag: SAVE_TAG,
                threw: false,
            } = event
            {
                self.compare = None;
                self.messages.push_back(Message {
                    title: LOADED_TITLE,
                    text: LOADED.to_owned(),
                });
                self.changed = true;
            }
        }
    }

    /// Starts the fetch asked for on a worker thread, over the network - or the directory
    /// standing in for it, `MP_FIRMWARE_MIRROR` ([`mp_firmware::manifest::fetcher`]).
    pub fn dispatch(&mut self) {
        let Some(want) = self.want.take() else {
            return;
        };
        let (sender, receiver) = channel();
        self.fetching = Some(receiver);
        std::thread::spawn(move || {
            let fetch = mp_firmware::manifest::fetcher();
            let arrived = match want {
                Want::Listing => Arrived::Listing(github::dir_content(
                    fetch.as_ref(),
                    OWNER,
                    REPO,
                    DIRECTORY,
                    FILTER,
                )),
                Want::File { path, save_as } => Arrived::File {
                    bytes: github::file_content(fetch.as_ref(), OWNER, REPO, &path),
                    save_as,
                },
            };
            let _ = sender.send(arrived);
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::optional::tests::Answering;

    fn files() -> Vec<FileInfo> {
        ["3DR_Iris+.param", "Solo.param"]
            .into_iter()
            .map(|name| FileInfo {
                name: name.to_owned(),
                path: format!("Tools/Frame_params/{name}"),
                size: 10,
            })
            .collect()
    }

    /// Loaded, the listing asked for taken as `dispatch` takes it, and answered.
    fn answered(listing: Vec<FileInfo>) -> DefaultSettings<usize> {
        let mut control = DefaultSettings::default();
        control.load();
        assert_eq!(control.want.take(), Some(Want::Listing));
        control.arrive(Arrived::Listing(Ok(listing)), &[]);
        control
    }

    fn listed() -> DefaultSettings<usize> {
        answered(files())
    }

    fn scratch(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!(
            "mp-gui-defaultsettings-{}-{name}",
            std::process::id()
        ));
        std::fs::create_dir_all(&dir).expect("a scratch directory");
        dir
    }

    #[test]
    fn load_disables_both_and_asks_for_the_listing_once() {
        let mut control = DefaultSettings::<usize>::default();
        assert!(
            control.combo().enabled,
            "the Designer's combo box is enabled"
        );
        assert!(!control.button_enabled(), "the .resx disables the button");
        control.load();
        assert!(!control.combo().enabled);
        assert!(!control.button_enabled());
        assert_eq!(control.want(), Some(&Want::Listing));
        control.want = None;
        control.load();
        assert_eq!(control.want(), None, "Load runs once");
    }

    #[test]
    fn the_listing_fills_the_combo_box_and_chooses_the_first() {
        let control = listed();
        assert!(control.listed());
        assert!(control.combo().enabled && control.button_enabled());
        assert_eq!(control.combo().text(), "3DR_Iris+.param");
        assert_eq!(control.combo().options.len(), 2);
    }

    #[test]
    fn a_failed_listing_is_logged_and_leaves_both_disabled() {
        let mut control = DefaultSettings::<usize>::default();
        control.load();
        control.arrive(Arrived::Listing(Err("403 rate limited".into())), &[]);
        assert!(!control.listed());
        assert!(!control.combo().enabled && !control.button_enabled());
        assert_eq!(control.listing_error(), Some("403 rate limited"));
        assert!(control.message().is_none(), "nothing on the screen");
    }

    #[test]
    fn load_params_asks_for_the_chosen_file_saved_under_its_name() {
        let mut control = listed();
        control.toggle_dropdown();
        assert!(control.dropdown());
        control.choose(1);
        assert!(!control.dropdown());
        control.click_load(std::path::Path::new("/data"));
        assert_eq!(
            control.want(),
            Some(&Want::File {
                path: "Tools/Frame_params/Solo.param".into(),
                save_as: PathBuf::from("/data/Solo.param"),
            })
        );
        assert!(control.busy());
        control.click_load(std::path::Path::new("/data"));
        assert_eq!(control.want().map(|_| ()), Some(()), "one fetch at a time");
    }

    #[test]
    fn load_params_with_nothing_chosen_says_so() {
        let mut control = answered(Vec::new());
        control.click_load(std::path::Path::new("/data"));
        assert_eq!(
            control.message().map(|m| m.text.as_str()),
            Some(SELECT_FIRST)
        );
        assert_eq!(control.take_link_errors(), None, "a box, not a status line");
        assert!(control.want().is_none());
    }

    /// The file arrives: saved, read, and compared with the vehicle's `FRAME` - the one value
    /// that differs is the row; Continue writes it, the form closes with "Loaded parameters!" and
    /// `OnChange` is raised.
    #[test]
    fn a_file_is_saved_compared_and_written() {
        let dir = scratch("written");
        let save_as = dir.join("Solo.param");
        let mut control = listed();
        let vehicle = vec![
            ("FRAME".to_owned(), 0.0),
            ("ATC_RAT_RLL_P".to_owned(), 0.15),
        ];
        control.arrive(
            Arrived::File {
                save_as: save_as.clone(),
                bytes: Ok(Some(
                    b"FRAME,1\nATC_RAT_RLL_P,0.15\nNOT_ON_VEHICLE,3\n".to_vec(),
                )),
            },
            &vehicle,
        );
        assert!(save_as.is_file(), "saved in the user data directory");
        let form = control.compare().expect("ParamCompare");
        let names: Vec<&str> = form.rows().iter().map(|row| row.name.as_str()).collect();
        assert_eq!(names, ["FRAME"]);

        control.click_save();
        let link = Answering::new(&[]);
        for _ in 0..10 {
            control.tick(&link, &vehicle);
        }
        assert_eq!(link.taken(), [("FRAME".to_owned(), 1.0)]);
        assert!(control.compare().is_none());
        assert_eq!(control.message().map(|m| m.title), Some(LOADED_TITLE));
        assert_eq!(control.message().map(|m| m.text.as_str()), Some(LOADED));
        assert!(control.take_changed());
        assert!(!control.take_changed(), "taken once");
        let _ = std::fs::remove_dir_all(dir);
    }

    #[test]
    fn closing_the_form_raises_on_change_and_says_nothing() {
        let dir = scratch("closed");
        let mut control = listed();
        control.arrive(
            Arrived::File {
                save_as: dir.join("a.param"),
                bytes: Ok(Some(b"FRAME 1\n".to_vec())),
            },
            &[("FRAME".to_owned(), 0.0)],
        );
        assert!(control.compare().is_some());
        control.close_compare();
        assert!(control.compare().is_none());
        assert!(control.message().is_none());
        assert!(control.take_changed());
        let _ = std::fs::remove_dir_all(dir);
    }

    /// A fetch that failed, or a body that is JSON `null`, is "Failed to load file." with the
    /// reason - on the status line, not in a box.
    #[test]
    fn a_failed_fetch_is_a_status_line() {
        let mut control = listed();
        control.arrive(
            Arrived::File {
                save_as: PathBuf::from("/nonexistent/a.param"),
                bytes: Err("404 Not Found".into()),
            },
            &[],
        );
        assert!(control.compare().is_none());
        assert_eq!(
            control.take_link_errors().as_deref(),
            Some("Failed to load file. 404 Not Found")
        );
        assert!(control.message().is_none());
        control.arrive(
            Arrived::File {
                save_as: PathBuf::from("/nonexistent/a.param"),
                bytes: Ok(None),
            },
            &[],
        );
        assert!(
            control
                .take_link_errors()
                .is_some_and(|words| words.contains("ArgumentNullException"))
        );
    }
}
