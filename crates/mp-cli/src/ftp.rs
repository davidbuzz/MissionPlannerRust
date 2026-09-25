//! `headless-planner ftp`: the vehicle's file system over MAVFTP, as Mission Planner's MAVFtp page
//! (Controls/MavFTPUI.cs) drives it.
//!
//! * `ls` - a listing, directories then files, as the page's list shows them (:151-194).
//! * `get` - `GetFile` with its defaults, a burst read of 80-byte chunks (the page's "Download
//!   Burst", :637), written where asked or, as the page does, under the file's own name without
//!   overwriting (:361-365).
//! * `put` - `UploadFile`, then the vehicle's CRC of what arrived against the local file's, as the
//!   page's upload does (:405-443).
//! * `rm` - `kCmdRemoveFile` (:468-471).
//! * `crc` - `kCmdCalcFileCRC32`, printed as the page prints it (:548-571).

#![allow(clippy::print_stdout, clippy::print_stderr)]

use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::time::{Duration, Instant};

use mp_link::mavftp::{FtpFileInfo, FtpOutcome, FtpRequest, RW_SIZE, crc_crc32};
use mp_link::{FtpError, Link, LinkConfig};
use mp_vehicle::VehicleId;

const USAGE: &str = "usage:\n  \
    headless-planner ftp ls  <url> [path]            list a directory (default /)\n  \
    headless-planner ftp get <url> <remote> [local]  download a file (burst read)\n  \
    headless-planner ftp put <url> <local> <remote>  upload a file and check its CRC\n  \
    headless-planner ftp rm  <url> <path>            remove a file\n  \
    headless-planner ftp crc <url> <path>            the vehicle's CRC32 of a file";

/// How long any one request may run before `headless-planner` gives up on it. Longer than any of the C#'s own
/// ladders (thirty one-second retries, or three thirty-second CRC waits), so the client's own
/// timeouts always end a request first; this only guards against a link that died under it.
const REQUEST_LIMIT: Duration = Duration::from_secs(600);

/// `headless-planner ftp <verb> ...`, with `args` the words after `ftp`.
pub(crate) fn run(args: &[String]) -> ExitCode {
    let word = |i: usize| args.get(i).map(String::as_str);
    let result = match (word(0), word(1), word(2), word(3)) {
        (Some("ls"), Some(url), path, None) => {
            with_link(url, |link, id| ls(link, id, path.unwrap_or("/")))
        }
        (Some("get"), Some(url), Some(remote), local) => {
            with_link(url, |link, id| get(link, id, remote, local.map(Path::new)))
        }
        (Some("put"), Some(url), Some(local), Some(remote)) => match std::fs::read(local) {
            Ok(data) => with_link(url, |link, id| put(link, id, remote, data)),
            Err(err) => Err(format!("could not read {local}: {err}")),
        },
        (Some("rm"), Some(url), Some(path), None) => with_link(url, |link, id| rm(link, id, path)),
        (Some("crc"), Some(url), Some(path), None) => {
            with_link(url, |link, id| crc(link, id, path))
        }
        _ => {
            eprintln!("{USAGE}");
            return ExitCode::from(2);
        }
    };
    match result {
        Ok(report) => {
            print!("{report}");
            ExitCode::SUCCESS
        }
        Err(error) => {
            eprintln!("{error}");
            ExitCode::FAILURE
        }
    }
}

/// Connects, waits for the vehicle, runs `verb`, and closes the link.
fn with_link(
    url: &str,
    verb: impl FnOnce(&Link, VehicleId) -> Result<String, String>,
) -> Result<String, String> {
    let config = LinkConfig {
        stream_rate_hz: 0,
        ..LinkConfig::default()
    };
    let mut link =
        Link::connect(url, config).map_err(|err| format!("could not open {url}: {err}"))?;
    let deadline = Instant::now() + Duration::from_secs(20);
    while link.primary_vehicle().is_none() && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(50));
    }
    let Some((id, _)) = link.primary_vehicle() else {
        return Err(format!("no vehicle appeared on {url}"));
    };
    let result = verb(&link, id);
    link.close();
    result
}

/// Runs one request on the link to its outcome.
fn request(link: &Link, id: VehicleId, request: FtpRequest) -> Result<FtpOutcome, String> {
    if !link.ftp(id, request) {
        return Err("the link would not take the request".to_owned());
    }
    let deadline = Instant::now() + REQUEST_LIMIT;
    loop {
        if let Some(outcome) = link.take_ftp_outcome(id) {
            return outcome.map_err(|error: FtpError| error.to_string());
        }
        if Instant::now() > deadline || !link.is_running() {
            link.cancel_ftp(id);
            return Err("the link stopped answering".to_owned());
        }
        std::thread::sleep(Duration::from_millis(5));
    }
}

/// `headless-planner ftp ls`: directories, then files, as MavFTPUI.cs:151-194 lists them. The nameless
/// placeholders for skipped entries and `.`/`..` are left out, as `GetDirectories` leaves them
/// out (:248); a file's size is in bytes, and its time, when the vehicle gives one, in seconds
/// since the epoch.
fn ls(link: &Link, id: VehicleId, path: &str) -> Result<String, String> {
    let FtpOutcome::Listing { entries, ended } = request(
        link,
        id,
        FtpRequest::List {
            path: path.to_owned(),
        },
    )?
    else {
        return Err("the vehicle answered something other than a listing".to_owned());
    };
    let mut out = String::new();
    let shown = |entry: &&FtpFileInfo| !matches!(entry.name.as_str(), "" | "." | "..");
    for entry in entries.iter().filter(|e| e.is_directory).filter(shown) {
        out.push_str(&line("Directory", "", entry));
    }
    for entry in entries.iter().filter(|e| !e.is_directory) {
        out.push_str(&line("File", &entry.size.to_string(), entry));
    }
    if !ended {
        // The C# shows what it has; saying so is this tool's.
        return Err(format!(
            "{out}the listing of {path} stopped part way: the vehicle stopped answering"
        ));
    }
    Ok(out)
}

fn line(kind: &str, size: &str, entry: &FtpFileInfo) -> String {
    let when = entry
        .modified_utc
        .map(|secs| format!("  utc {secs}"))
        .unwrap_or_default();
    format!("{kind:<9}  {size:>10}  {}{when}\n", entry.name)
}

/// `headless-planner ftp get`: `GetFile(remote, cancel)` with its defaults, burst and 80 bytes a chunk
/// (MAVFtp.cs:560), then the bytes to `local`.
fn get(link: &Link, id: VehicleId, remote: &str, local: Option<&Path>) -> Result<String, String> {
    let outcome = request(
        link,
        id,
        FtpRequest::Get {
            path: remote.to_owned(),
            burst: true,
            readsize: RW_SIZE,
        },
    )?;
    let FtpOutcome::File { data, ended } = outcome else {
        return Err("the vehicle answered something other than a file".to_owned());
    };
    let Some(data) = data else {
        return Err(format!("the vehicle did not open {remote}"));
    };
    if !ended {
        // The C# hands back what arrived, holes and all (MAVFtp.cs:862-869); writing that as the
        // file would be a file that looks whole and is not.
        return Err(format!(
            "the read of {remote} gave up after its retries with {} bytes; nothing written",
            data.len()
        ));
    }
    let path = match local {
        Some(path) => path.to_path_buf(),
        None => unused_name(Path::new(""), remote),
    };
    std::fs::write(&path, &data)
        .map_err(|err| format!("could not write {}: {err}", path.display()))?;
    Ok(format!(
        "{remote}: {} bytes to {}\n",
        data.len(),
        path.display()
    ))
}

/// The remote file's own name in `dir`, numbered until it is not taken, as the page names a
/// download (MavFTPUI.cs:361-365: `file = name + a++` while it exists).
fn unused_name(dir: &Path, remote: &str) -> PathBuf {
    let name = remote.rsplit(['/', '\\']).next().unwrap_or(remote);
    let name = if name.is_empty() { "download" } else { name };
    let mut path = dir.join(name);
    let mut a = 0u32;
    while path.exists() {
        path = dir.join(format!("{name}{a}"));
        a += 1;
    }
    path
}

/// `headless-planner ftp put`: `UploadFile(remote, local)`, then the page's check: the vehicle's CRC of the
/// file against `crc_crc32(0, local bytes)`, a mismatch being `BadCrcException`
/// (MavFTPUI.cs:428-442).
fn put(link: &Link, id: VehicleId, remote: &str, data: Vec<u8>) -> Result<String, String> {
    let local_crc = crc_crc32(0, &data);
    let len = data.len();
    request(
        link,
        id,
        FtpRequest::Put {
            path: remote.to_owned(),
            data,
        },
    )?;
    let FtpOutcome::Crc32 { crc, .. } = request(
        link,
        id,
        FtpRequest::Crc32 {
            path: remote.to_owned(),
        },
    )?
    else {
        return Err("the vehicle answered something other than a CRC".to_owned());
    };
    if crc != local_crc {
        return Err(format!(
            "{remote}: bad CRC after upload, vehicle 0x{crc:X}, file 0x{local_crc:X}"
        ));
    }
    Ok(format!("{remote}: {len} bytes, CRC 0x{crc:X}\n"))
}

/// `headless-planner ftp rm`: `kCmdRemoveFile`; false is the page's "Failed to delete file" (MavFTPUI.cs:470).
fn rm(link: &Link, id: VehicleId, path: &str) -> Result<String, String> {
    match request(
        link,
        id,
        FtpRequest::RemoveFile {
            path: path.to_owned(),
        },
    )? {
        FtpOutcome::Done(true) => Ok(format!("removed {path}\n")),
        _ => Err(format!("Failed to delete file {path}")),
    }
}

/// `headless-planner ftp crc`: `kCmdCalcFileCRC32`, as the page shows it: the name, ": 0x", the CRC in
/// hexadecimal (MavFTPUI.cs:571). Unanswered, the C#'s CRC is `UInt32.MaxValue` (MAVFtp.cs:929),
/// which is printed too, and the command fails.
fn crc(link: &Link, id: VehicleId, path: &str) -> Result<String, String> {
    let FtpOutcome::Crc32 { crc, answered } = request(
        link,
        id,
        FtpRequest::Crc32 {
            path: path.to_owned(),
        },
    )?
    else {
        return Err("the vehicle answered something other than a CRC".to_owned());
    };
    let name = path.rsplit('/').next().unwrap_or(path);
    let report = format!("{name}: 0x{crc:X}\n");
    if answered {
        Ok(report)
    } else {
        Err(format!("{report}the vehicle did not answer"))
    }
}

#[cfg(test)]
mod tests {
    //! Each verb through the real link against `FakeVehicle`, and against SITL (ignored).

    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

    use std::sync::Arc;
    use std::sync::atomic::{AtomicBool, Ordering};

    use mp_link::ProtocolTimeouts;
    use mp_link::ftp::ftp_message;
    use mp_link::mavftp::testing::FakeVehicle;
    use mp_link::mavftp::wire::Header;
    use mp_mavlink::{FrameDecoder, encode_v2};
    use mp_mavlink_dialects::all::{DIALECT, Heartbeat, MavMessage};
    use mp_transport::Transport;
    use mp_transport::testing::{Loopback, LoopbackEnd};

    use super::*;

    const VEHICLE: VehicleId = VehicleId::new(1, 1);
    const GCS: VehicleId = VehicleId::new(255, 190);

    fn frame(seq: u8, message: &MavMessage) -> Vec<u8> {
        let mut payload = [0u8; 255];
        let len = message.encode(&mut payload);
        let mut out = [0u8; mp_mavlink::MAX_FRAME_LEN];
        let n = encode_v2(
            &mut out,
            seq,
            VEHICLE.sysid,
            VEHICLE.compid,
            message.id(),
            &payload[..len],
            message.crc_extra(),
            0,
        )
        .unwrap();
        out[..n].to_vec()
    }

    /// The fake vehicle on a thread of its own, answering MAVFTP until stopped.
    fn vehicle(
        mut end: LoopbackEnd,
        stop: Arc<AtomicBool>,
        mut fake: FakeVehicle,
    ) -> std::thread::JoinHandle<FakeVehicle> {
        std::thread::spawn(move || {
            let mut seq = 0u8;
            let heartbeat = MavMessage::Heartbeat(Heartbeat {
                custom_mode: 0,
                r#type: 2,
                autopilot: 3,
                base_mode: 81,
                system_status: 3,
                mavlink_version: 3,
            });
            end.write_all(&frame(seq, &heartbeat)).unwrap();
            let mut decoder = FrameDecoder::new();
            let mut buf = [0u8; 4096];
            while !stop.load(Ordering::Acquire) {
                let n = end.read(&mut buf).unwrap_or(0);
                if n == 0 {
                    std::thread::sleep(Duration::from_millis(1));
                    continue;
                }
                let mut requests = Vec::new();
                decoder.push_and_drain(&buf[..n], &DIALECT, |frame| {
                    if let Some(MavMessage::FileTransferProtocol(ftp)) =
                        MavMessage::decode(frame.msgid, frame.payload)
                    {
                        requests.push(Header::decode(&ftp.payload));
                    }
                });
                for request in requests {
                    for reply in fake.answer(&request) {
                        seq = seq.wrapping_add(1);
                        end.write_all(&frame(seq, &ftp_message(GCS, &reply)))
                            .unwrap();
                    }
                }
            }
            fake
        })
    }

    fn link(end: LoopbackEnd) -> Link {
        let config = LinkConfig {
            send_heartbeat: false,
            stream_rate_hz: 0,
            timeouts: ProtocolTimeouts::default().faster(20),
            ..LinkConfig::default()
        };
        let link = Link::from_transport(Box::new(end), config);
        let deadline = Instant::now() + Duration::from_secs(5);
        while link.vehicle(VEHICLE).is_none() {
            assert!(Instant::now() < deadline, "the vehicle was never heard");
            std::thread::sleep(Duration::from_millis(1));
        }
        link
    }

    fn scratch(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!(
            "headless-planner-ftp-{}-{name}",
            std::process::id()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    /// `ls`, `get`, `put`, `rm` and `crc`, each through the link.
    #[test]
    fn every_verb_against_a_scripted_vehicle() {
        let param = (0..1500)
            .map(|i| u8::try_from(i % 256).unwrap())
            .collect::<Vec<u8>>();
        let fake = FakeVehicle::new()
            .with_file("/APM/param.pck", &param)
            .with_file("/APM/old.txt", b"old")
            .with_dir("/APM/LOGS");
        let (vehicle_side, gcs_side) = Loopback::pair();
        let stop = Arc::new(AtomicBool::new(false));
        let script = vehicle(vehicle_side, Arc::clone(&stop), fake);
        let link = link(gcs_side);

        let listing = ls(&link, VEHICLE, "/APM").unwrap();
        assert_eq!(
            listing,
            "Directory              LOGS\n\
             File                3  old.txt\n\
             File             1500  param.pck\n"
        );

        let dir = scratch("verbs");
        let out = dir.join("param.pck");
        let report = get(&link, VEHICLE, "/APM/param.pck", Some(&out)).unwrap();
        assert!(report.starts_with("/APM/param.pck: 1500 bytes"), "{report}");
        assert_eq!(std::fs::read(&out).unwrap(), param);

        let report = put(&link, VEHICLE, "/APM/up.bin", b"123456789".to_vec()).unwrap();
        assert_eq!(report, "/APM/up.bin: 9 bytes, CRC 0x2DFD2D88\n");

        assert_eq!(
            crc(&link, VEHICLE, "/APM/up.bin").unwrap(),
            "up.bin: 0x2DFD2D88\n"
        );
        assert_eq!(
            rm(&link, VEHICLE, "/APM/old.txt").unwrap(),
            "removed /APM/old.txt\n"
        );
        assert_eq!(
            get(&link, VEHICLE, "/APM/old.txt", Some(&dir.join("gone"))),
            Err("File Not Found".to_owned())
        );

        stop.store(true, Ordering::Release);
        let fake = script.join().unwrap();
        assert!(!fake.files.contains_key("/APM/old.txt"));
        assert_eq!(fake.files["/APM/up.bin"], b"123456789");
        let _ = std::fs::remove_dir_all(dir);
    }

    #[test]
    fn a_download_does_not_overwrite_what_is_there() {
        let dir = scratch("names");
        let taken = dir.join("threads.txt");
        std::fs::write(&taken, b"x").unwrap();
        assert_eq!(
            unused_name(&dir, "@SYS/threads.txt"),
            dir.join("threads.txt0")
        );
        std::fs::write(dir.join("threads.txt0"), b"x").unwrap();
        assert_eq!(
            unused_name(&dir, "@SYS/threads.txt"),
            dir.join("threads.txt1")
        );
        assert_eq!(unused_name(&dir, "@SYS/uarts.txt"), dir.join("uarts.txt"));
        let _ = std::fs::remove_dir_all(dir);
    }

    /// Against ArduPilot SITL on `tcp:127.0.0.1:5763`, the port kept for this test: the root and
    /// `@SYS` listings, `@SYS/uarts.txt` read both ways (plain, as the serial ports page reads it,
    /// and burst), `@SYS/threads.txt`, and `@PARAM/param.pck`, whose bytes must have the CRC the
    /// vehicle computes for it.
    #[test]
    #[ignore = "requires ArduPilot SITL listening on tcp:127.0.0.1:5763"]
    fn against_sitl() {
        let url = "tcp:127.0.0.1:5763";
        let config = LinkConfig {
            stream_rate_hz: 0,
            ..LinkConfig::default()
        };
        let mut link = Link::connect(url, config).expect("SITL on 5763");
        let deadline = Instant::now() + Duration::from_secs(20);
        while link.primary_vehicle().is_none() && Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(50));
        }
        let (id, _) = link.primary_vehicle().expect("a vehicle");

        let root = ls(&link, id, "/").unwrap();
        println!("/:\n{root}");
        assert!(!root.is_empty());
        let sys = ls(&link, id, "@SYS").unwrap();
        println!("@SYS:\n{sys}");
        assert!(
            sys.contains("uarts.txt") && sys.contains("threads.txt"),
            "{sys}"
        );

        let plain = request(
            &link,
            id,
            FtpRequest::Get {
                path: "@SYS/uarts.txt".to_owned(),
                burst: false,
                readsize: RW_SIZE,
            },
        )
        .unwrap();
        let FtpOutcome::File {
            data: Some(uarts),
            ended: true,
        } = plain
        else {
            panic!("{plain:?}");
        };
        let text = String::from_utf8_lossy(&uarts);
        println!("uarts.txt ({} bytes):\n{text}", uarts.len());
        assert!(text.contains("SERIAL0"), "{text}");

        let dir = scratch("sitl");
        get(&link, id, "@SYS/uarts.txt", Some(&dir.join("uarts.txt"))).unwrap();
        let burst = std::fs::read(dir.join("uarts.txt")).unwrap();
        assert!(String::from_utf8_lossy(&burst).contains("SERIAL0"));

        // SITL lists threads.txt but has nothing to put in it: its HAL keeps the empty default
        // `thread_info` (ArduPilot libraries/AP_HAL/Util.h:162), and an empty @SYS file fails to
        // open with ENOENT (AP_Filesystem_Sys.cpp:185-186), which the vehicle sends as
        // kErrFileNotFound. That is the C#'s FileNotFoundException, reported when the open's
        // two-second wait runs out (MAVFtp.cs:608, 646-652); Mission Planner under mono says the
        // same of this SITL (tools/csharp-reference/MpFtp.cs, `sitl`). A board with threads gives bytes.
        let started = Instant::now();
        match get(
            &link,
            id,
            "@SYS/threads.txt",
            Some(&dir.join("threads.txt")),
        ) {
            Ok(report) => {
                println!("{report}");
                assert!(!std::fs::read(dir.join("threads.txt")).unwrap().is_empty());
            }
            Err(error) => {
                println!("threads.txt: {error}, after {:?}", started.elapsed());
                assert_eq!(error, "File Not Found");
                assert!(started.elapsed() >= Duration::from_secs(2));
            }
        }

        // The parameter file as Mission Planner reads it: a burst of 110-byte reads
        // (MAVLinkInterface.cs:1877, without the "?withdefaults=1" it adds). It starts with the
        // pack's magic, 0x671B little-endian.
        let param = read_all(&link, id, "@PARAM/param.pck", 110);
        println!("param.pck at 110 bytes a read: {} bytes", param.len());
        assert!(param.len() > 1000);
        assert_eq!(param[..2], [0x1B, 0x67]);

        // Its CRC. ArduPilot writes param.pck on the fly and pads each read with zeros rather
        // than split a parameter across two, so the file's bytes depend on the size it is read
        // in; kCmdCalcFileCRC32 reads it 64 bytes at a time (AP_Filesystem::crc32,
        // libraries/AP_Filesystem/AP_Filesystem.cpp:403). So the CRC is checked against a read
        // of 64 - bracketed by a second read, because SITL changes parameters as it runs, and a
        // CRC is only comparable with bytes that were the file while it was computed.
        let mut matched = false;
        for attempt in 0..3 {
            let before = read_all(&link, id, "@PARAM/param.pck", 64);
            let FtpOutcome::Crc32 { crc, answered } = request(
                &link,
                id,
                FtpRequest::Crc32 {
                    path: "@PARAM/param.pck".to_owned(),
                },
            )
            .unwrap() else {
                panic!();
            };
            assert!(answered);
            let after = read_all(&link, id, "@PARAM/param.pck", 64);
            println!(
                "attempt {attempt}: {} bytes at 64 a read, ours 0x{:X}, vehicle 0x{crc:X}, \
                 the file {} while the CRC was computed",
                before.len(),
                crc_crc32(0, &before),
                if before == after {
                    "held still"
                } else {
                    "changed"
                }
            );
            if before == after {
                assert_eq!(crc_crc32(0, &before), crc);
                matched = true;
                break;
            }
        }
        assert!(
            matched,
            "the parameters never held still for a read, a CRC and a read"
        );
        link.close();
        let _ = std::fs::remove_dir_all(dir);
    }

    /// A whole file by burst read at `readsize`, as `GetFile` makes it.
    fn read_all(link: &Link, id: VehicleId, path: &str, readsize: u8) -> Vec<u8> {
        let outcome = request(
            link,
            id,
            FtpRequest::Get {
                path: path.to_owned(),
                burst: true,
                readsize,
            },
        )
        .unwrap();
        let FtpOutcome::File {
            data: Some(data),
            ended: true,
        } = outcome
        else {
            panic!("{path}: {outcome:?}");
        };
        data
    }
}
