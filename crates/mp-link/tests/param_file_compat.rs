//! `.param` files written by other ground stations are read, and what we write is read back.
//!
//! Named in D12's definition of done. A parameter file is the one artefact an operator moves
//! between tools - saved in Mission Planner, posted on a forum, pulled out of MAVProxy - so
//! failing to read somebody else's file is failing at the format's whole purpose.
//!
//! On the fixtures: `mavproxy.parm` is a genuine capture, the first forty lines of a file pulled
//! off a real vehicle. `mission-planner.param` is written in the C# application's output format -
//! its `#NOTE:` header, comma separator, six decimal places and sorted order - rather than
//! captured from a run of it, because that application does not run on this machine. It is a
//! fixture for the format, not proof that a particular build of Mission Planner produced those
//! bytes, and it is worth saying so rather than letting a future reader assume otherwise.

use mp_link::param_file::{Change, ParamFile};

fn fixture(name: &str) -> ParamFile {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures")
        .join(name);
    let file = ParamFile::load(&path).unwrap_or_else(|err| panic!("reading {name}: {err}"));
    assert!(
        file.rejected().is_empty(),
        "{name} had unreadable lines: {:?}",
        file.rejected()
    );
    file
}

/// The C# application's own format, header and all.
#[test]
fn a_mission_planner_file_is_read() {
    let file = fixture("mission-planner.param");
    assert_eq!(
        file.len(),
        30,
        "thirty-one lines, one of which is the header"
    );
    assert_eq!(file.get("ATC_ANG_PIT_P"), Some(4.5));
    assert_eq!(file.get("BATT_CAPACITY"), Some(3300.0));
    // Negative values, which a naive split on '-' would lose.
    assert_eq!(file.get("COMPASS_OFS_X"), Some(-31.25));
    // A name that fills the 16-byte MAVLink field exactly, with no room for a terminator.
    assert_eq!(file.get("SERVO16_FUNCTION"), Some(0.0));
    // The `#NOTE:` header is a comment, not a parameter called `#NOTE:`.
    assert_eq!(file.get("#NOTE:"), None);
}

/// A real file off a real vehicle, whitespace separated as MAVProxy writes them.
#[test]
fn a_mavproxy_file_is_read() {
    let file = fixture("mavproxy.parm");
    assert_eq!(file.len(), 40);
    assert_eq!(file.get("ACRO_BAL_PITCH"), Some(1.0));
    // Written as a bare integer rather than with six decimal places.
    assert_eq!(file.get("ACRO_OPTIONS"), Some(0.0));
    assert_eq!(file.get("ADSB_LIST_RADIUS"), Some(2000.0));
}

/// The same parameters are the same parameters however they were written down.
#[test]
fn the_two_formats_agree_where_they_overlap() {
    let planner = fixture("mission-planner.param");
    let proxy = fixture("mavproxy.parm");
    let differences = planner.compare(&proxy);
    for difference in &differences {
        // Anything present in both must agree; the two files cover different parameter sets, so
        // names in only one of them are expected.
        assert!(
            !matches!(difference.kind, Change::Changed { .. }),
            "{difference}"
        );
    }
    // And they do overlap, so the test above is testing something.
    let shared = planner
        .iter()
        .filter(|(name, _)| proxy.get(name).is_some())
        .count();
    assert!(shared >= 8, "only {shared} parameters in common");
}

/// What we write is read back as what we wrote. This is the promise a backup makes.
#[test]
fn our_own_output_round_trips_through_the_disk() {
    let original = fixture("mission-planner.param");
    let directory = std::env::temp_dir().join(format!("mpr-param-compat-{}", std::process::id()));
    std::fs::create_dir_all(&directory).expect("a writable temp directory");
    let path = directory.join("written.param");

    original.save(&path).expect("writing the file");
    let reread = ParamFile::load(&path).expect("reading it back");

    assert!(reread.rejected().is_empty(), "{:?}", reread.rejected());
    assert_eq!(original.len(), reread.len());
    assert!(
        original.compare(&reread).is_empty(),
        "{:?}",
        original.compare(&reread)
    );

    let _ = std::fs::remove_dir_all(&directory);
}

/// A file we wrote is still in the shape the C# application reads: `NAME,VALUE`, sorted.
///
/// Asserted on the text rather than by re-parsing, because re-parsing only proves we can read our
/// own output, which is not the question.
#[test]
fn what_we_write_is_shaped_the_way_the_other_tools_read() {
    let text = fixture("mission-planner.param").render();
    let mut previous = String::new();
    for line in text.lines() {
        let (name, value) = line
            .split_once(',')
            .unwrap_or_else(|| panic!("a comma in {line:?}"));
        assert!(name > previous.as_str(), "{name} came after {previous}");
        assert!(
            value.contains('.') && value.split('.').nth(1).is_some_and(|part| part.len() == 6),
            "six decimal places in {line:?}"
        );
        previous = name.to_owned();
    }
}
