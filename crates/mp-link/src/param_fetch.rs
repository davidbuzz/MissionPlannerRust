//! The parameter fetch as Mission Planner does it: `getParamListMavftp`, the whole table read
//! as one file over MAVFTP, with the classic `PARAM_REQUEST_LIST` stream as the fallback.
//! `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1781-1936`
//!
//! `getParamList` (`:1781-1799`) - the button on the parameter screen, and the connect - runs
//! `getParamListMavftp`: when the vehicle's `AUTOPILOT_VERSION` capabilities include FTP and the
//! `UseMavFtpParams` setting is on (its default), `MAVFtp.GetFile("@PARAM/param.pck?withdefaults=1")`
//! in 110-byte reads (`:1859-1878`), `parampck.unpack` on what comes back, and the vehicle's table
//! replaced whole - `param.Clear()`, `TotalReported = count`, `AddRange` (`:1892-1897`). Anything
//! else - no FTP capability, a file that will not open, a pack that will not unpack, an exception
//! on the way - falls through to `getParamListAsync`, the stream with its gap recovery
//! (`:1930`; here [`crate::param_download`]).
//!
//! The owner's rule (2026-09-25): MAVFTP is always tried first. So a vehicle whose capabilities
//! have not been heard yet is asked over MAVFTP too; one that has said it has no FTP is not made
//! to wait for the request to time out, as the C# does not ask it either. The C# also sends
//! `DO_SEND_BANNER` first and collects the banner lines (`:1815-1858`); the link asks for the
//! banner on its own when a vehicle appears, so that is not repeated here.
//!
//! The link thread runs the fetch ([`tick`]): it watches the FTP client for the file, unpacks it
//! and fills the table, or starts the classic download and watches that. The owner reads where
//! it is with [`crate::Link::param_fetch`].

use std::collections::BTreeMap;
use std::sync::Arc;
use std::time::Instant;

use mp_ftp::mavftp::{FtpOutcome, FtpRequest};
use mp_params::{ParamTable, parampck};
use mp_vehicle::VehicleId;

use crate::param_download::{ParamAction, ParamDownload};
use crate::timeouts::ProtocolTimeouts;
use crate::{Link, Shared};

/// The file `getParamListMavftp` reads.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1877`
pub const PARAM_FILE: &str = "@PARAM/param.pck?withdefaults=1";

/// `GetFile(..., true, 110)`: a burst read, 110 bytes at a time.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1877`
pub const READ_SIZE: u8 = 110;

/// `MAV_PROTOCOL_CAPABILITY_FTP`.
pub const CAPABILITY_FTP: u32 = 32;

/// How the parameters came.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FetchVia {
    /// The file over MAVFTP.
    MavFtp,
    /// The `PARAM_REQUEST_LIST` stream.
    Stream,
}

/// Where a fetch is.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ParamFetchState {
    /// Reading `@PARAM/param.pck` over MAVFTP.
    Ftp,
    /// The classic stream, started because MAVFTP did not do (`why`) or was not offered.
    Stream {
        /// What sent it here: the C#'s logged exception, or "no FTP capability".
        why: String,
    },
    /// The table is full.
    Complete {
        /// Which way it came.
        via: FetchVia,
        /// How many parameters.
        count: usize,
    },
    /// The stream was cancelled with holes outstanding.
    Cancelled,
}

/// One vehicle's fetch.
#[derive(Debug, Clone)]
pub struct ParamFetch {
    /// Which vehicle.
    pub target: VehicleId,
    /// Where it is.
    pub state: ParamFetchState,
    /// When it began.
    pub started: Instant,
    /// The defaults the file carried, by name, for the screens that show them. Empty over the
    /// stream, which has no defaults to give.
    pub defaults: Arc<BTreeMap<String, f64>>,
}

impl ParamFetch {
    /// Whether it has stopped, either way.
    #[must_use]
    pub const fn is_finished(&self) -> bool {
        matches!(
            self.state,
            ParamFetchState::Complete { .. } | ParamFetchState::Cancelled
        )
    }
}

impl Link {
    /// `getParamList`: the table fetched MAVFTP first, the stream as the fallback, and returns at
    /// once; the outcome is read with [`Link::param_fetch`]. A fetch already running for this
    /// vehicle starts again from nothing, as a second call to the C# does. False if nothing could
    /// be sent.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1781-1799, 1813-1936`
    pub fn fetch_params(&self, target: VehicleId) -> bool {
        let now = Instant::now();
        let no_ftp = self.vehicle(target).is_some_and(|handle| {
            let state = handle.load();
            let info = &state.autopilot_info;
            // Capabilities are known once AUTOPILOT_VERSION has been heard; before that the
            // word is 0 and FTP is tried.
            info.version != [0; 4] && info.capabilities & CAPABILITY_FTP == 0
        });
        let mut fetch = ParamFetch {
            target,
            state: ParamFetchState::Ftp,
            started: now,
            defaults: Arc::default(),
        };
        let sent = if no_ftp {
            fetch.state = ParamFetchState::Stream {
                why: "no FTP capability".to_owned(),
            };
            self.download_params(target)
        } else {
            // A request already on the client (the MAVFtp page's, say) is not interrupted: the
            // fetch falls back to the stream, as the C# does when GetFile throws.
            let started = self.ftp(
                target,
                FtpRequest::Get {
                    path: PARAM_FILE.to_owned(),
                    burst: true,
                    readsize: READ_SIZE,
                },
            );
            if started {
                true
            } else {
                fetch.state = ParamFetchState::Stream {
                    why: "the MAVFTP client is busy".to_owned(),
                };
                self.download_params(target)
            }
        };
        if let Ok(mut fetches) = self.shared.param_fetches.lock() {
            fetches.insert(target, fetch);
        }
        sent
    }

    /// Where a vehicle's fetch is, if one has been started.
    #[must_use]
    pub fn param_fetch(&self, target: VehicleId) -> Option<ParamFetch> {
        self.shared.param_fetches.lock().ok()?.get(&target).cloned()
    }

    /// Stops a fetch: the file read cancelled, or the stream, as the progress dialog's Cancel
    /// does. What arrived stays in the table.
    pub fn cancel_param_fetch(&self, target: VehicleId) {
        let running = self
            .param_fetch(target)
            .is_some_and(|fetch| !fetch.is_finished());
        if !running {
            return;
        }
        self.cancel_ftp(target);
        self.cancel_param_download(target);
        if let Ok(mut fetches) = self.shared.param_fetches.lock()
            && let Some(fetch) = fetches.get_mut(&target)
        {
            fetch.state = ParamFetchState::Cancelled;
        }
    }
}

/// One pass of the link loop: each fetch moved on. A fetch reading the file takes the client's
/// outcome once it has one - the table replaced from the pack, or the stream started; a fetch on
/// the stream ends when the download does. What the stream wants sent is added to `actions`.
pub(crate) fn tick(
    shared: &Arc<Shared>,
    timeouts: ProtocolTimeouts,
    now: Instant,
    actions: &mut Vec<(VehicleId, ParamAction)>,
) {
    let Ok(mut fetches) = shared.param_fetches.lock() else {
        return;
    };
    for (id, fetch) in fetches.iter_mut() {
        match &fetch.state {
            ParamFetchState::Ftp => {
                let outcome = {
                    let Ok(mut clients) = shared.ftp.lock() else {
                        continue;
                    };
                    let Some(client) = clients.get_mut(id) else {
                        continue;
                    };
                    if client.is_busy() {
                        continue;
                    }
                    client.take_outcome()
                };
                let Some(outcome) = outcome else {
                    continue;
                };
                match unpack_outcome(outcome) {
                    Ok(list) => {
                        // `param.Clear(); TotalReported = count; AddRange(mavlist)`: the table
                        // replaced whole, each entry at its place in the file.
                        // C#: MAVLinkInterface.cs:1892-1897
                        let count = u16::try_from(list.len()).unwrap_or(u16::MAX);
                        let mut table = ParamTable::new();
                        let mut defaults = BTreeMap::new();
                        for (index, entry) in list.into_iter().enumerate() {
                            if let Some(default) = entry.default {
                                defaults.insert(entry.name.clone(), default);
                            }
                            table.insert(
                                entry.name,
                                entry.value,
                                u16::try_from(index).unwrap_or(u16::MAX),
                                count,
                            );
                        }
                        let held = table.len();
                        if let Ok(mut tables) = shared.params.lock() {
                            tables.insert(*id, table);
                        }
                        fetch.defaults = Arc::new(defaults);
                        fetch.state = ParamFetchState::Complete {
                            via: FetchVia::MavFtp,
                            count: held,
                        };
                    }
                    Err(why) => {
                        // `log.Error(e)` and `return await getParamListAsync(...)`.
                        // C#: MAVLinkInterface.cs:1922-1930
                        if let Ok(mut downloads) = shared.param_downloads.lock() {
                            let download = ParamDownload::new(*id, timeouts, now);
                            actions.push((*id, download.begin()));
                            downloads.insert(*id, download);
                        }
                        fetch.state = ParamFetchState::Stream { why };
                    }
                }
            }
            ParamFetchState::Stream { .. } => {
                let Ok(downloads) = shared.param_downloads.lock() else {
                    continue;
                };
                let Some(download) = downloads.get(id) else {
                    continue;
                };
                if !download.is_finished() {
                    continue;
                }
                let cancelled = matches!(
                    download.state(),
                    crate::param_download::ParamDownloadState::Cancelled
                );
                drop(downloads);
                fetch.state = if cancelled {
                    ParamFetchState::Cancelled
                } else {
                    let count = shared
                        .params
                        .lock()
                        .ok()
                        .and_then(|tables| tables.get(id).map(ParamTable::len))
                        .unwrap_or(0);
                    ParamFetchState::Complete {
                        via: FetchVia::Stream,
                        count,
                    }
                };
            }
            ParamFetchState::Complete { .. } | ParamFetchState::Cancelled => {}
        }
    }
}

/// The file's parameters, or why the C# would have logged and fallen back: `GetFile` returned
/// null or nothing (`paramfile != null && paramfile.Length > 0`), the read failed, or `unpack`
/// returned null.
fn unpack_outcome(
    outcome: Result<FtpOutcome, mp_ftp::FtpError>,
) -> Result<Vec<parampck::PackedParam>, String> {
    match outcome {
        Ok(FtpOutcome::File {
            data: Some(data),
            ended: true,
        }) if !data.is_empty() => parampck::unpack(&data).map_err(|error| error.to_string()),
        Ok(FtpOutcome::File { ended: false, .. }) => {
            Err("the MAVFTP read did not finish".to_owned())
        }
        Ok(FtpOutcome::File { .. }) => Err("param.pck was empty".to_owned()),
        Ok(other) => Err(format!("unexpected MAVFTP outcome: {other:?}")),
        Err(error) => Err(error.to_string()),
    }
}
