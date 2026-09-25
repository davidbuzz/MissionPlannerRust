//! Repository automation entry point: `cargo xtask <command>`.

use xtask::{codegen, ledger};

use std::collections::HashMap;
use std::path::PathBuf;

use anyhow::{Context, Result, bail};

fn main() {
    if let Err(err) = run() {
        eprintln!("error: {err:#}");
        std::process::exit(1);
    }
}

fn run() -> Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.first().map(String::as_str) {
        Some("verify-mavlink") => verify_mavlink(args.get(1).map(String::as_str)),
        Some("dump-tlog") => dump_tlog(args.get(1).map(String::as_str)),
        Some("codegen-modes") => codegen_modes(),
        Some("codegen-param-meta") => codegen_param_meta(),
        Some("codegen-resx") => codegen_resx(args.iter().any(|a| a == "--check")),
        Some("ledger") => ledger::run(args.get(1..).unwrap_or_default(), &repo_root()),
        Some("codegen") => {
            let check = args.iter().any(|a| a == "--check");
            let dialect = args
                .get(1)
                .filter(|a| !a.starts_with("--"))
                .map(String::as_str);
            codegen_mavlink(dialect, check)
        }
        Some("") | Some("help") | None => {
            usage();
            Ok(())
        }
        Some(other) => {
            eprintln!("unknown command: {other}\n");
            usage();
            std::process::exit(2);
        }
    }
}

fn usage() {
    println!(
        "cargo xtask <command>\n\n\
         commands:\n  \
         codegen [dialect]         regenerate the MAVLink dialect crate from the XML definitions\n  \
         dump-tlog <file>          decode a tlog and print the same CSV the C# reference emits\n  \
         codegen-modes             regenerate flight mode tables from the parameter metadata\n  \
         codegen-param-meta        regenerate parameter descriptions, ranges and enumerations\n  \
         codegen-resx [--check]    regenerate assets/i18n from the .resx files: the .ftl per culture, the key map, the zero-loss report\n  \
         ledger <init|check|status> the porting ledger: one row per C# file (PLAN.md §6.2)\n  \
         verify-mavlink [dialect]  check generated MAVLink metadata against the C# reference\n  \
         help                      show this message"
    );
}

fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..")
}

/// Parses the XML definitions and compares the derived metadata against the table dumped from
/// the shipping C# assembly. This is the gate that proves our mavgen reimplementation is right.
fn verify_mavlink(dialect: Option<&str>) -> Result<()> {
    let root = repo_root();
    let dialect = dialect.unwrap_or("all");
    let xml = root
        .join("references/missionplanner/ExtLibs/Mavlink/message_definitions")
        .join(format!("{dialect}.xml"));
    if !xml.exists() {
        bail!("dialect not found: {}", xml.display());
    }

    let parsed = codegen::mavlink::parse_dialect(&xml)?;
    println!(
        "parsed {}: {} messages, {} enums",
        parsed.name,
        parsed.messages.len(),
        parsed.enums.len()
    );

    let csv_path = root.join("testdata/mavlink/binary_message_infos.csv");
    let csv = std::fs::read_to_string(&csv_path)
        .with_context(|| format!("reading {}", csv_path.display()))?;

    let mut reference: HashMap<u32, (u8, u32, u32)> = HashMap::new();
    for line in csv.lines().skip(1) {
        let f: Vec<&str> = line.split(',').collect();
        let [id, _name, crc, min_len, len] = f[..] else {
            continue;
        };
        reference.insert(id.parse()?, (crc.parse()?, min_len.parse()?, len.parse()?));
    }

    let (mut checked, mut crc_bad, mut min_bad, mut len_bad, mut missing) = (0, 0, 0, 0, 0);
    let mut first_failures = Vec::new();

    for msg in parsed.messages.values() {
        let Some(&(ref_crc, ref_min, ref_len)) = reference.get(&msg.id) else {
            missing += 1;
            continue;
        };
        checked += 1;
        let (crc, min, len) = (
            msg.crc_extra(),
            u32::try_from(msg.min_len()).unwrap_or(u32::MAX),
            u32::try_from(msg.len()).unwrap_or(u32::MAX),
        );
        if crc != ref_crc || min != ref_min || len != ref_len {
            if first_failures.len() < 12 {
                first_failures.push(format!(
                    "  {:>6} {:<36} crc {:>3} vs {:>3}   min {:>3} vs {:>3}   len {:>3} vs {:>3}",
                    msg.id, msg.name, crc, ref_crc, min, ref_min, len, ref_len
                ));
            }
            if crc != ref_crc {
                crc_bad += 1;
            }
            if min != ref_min {
                min_bad += 1;
            }
            if len != ref_len {
                len_bad += 1;
            }
        }
    }

    println!(
        "checked {checked} against the C# table: {crc_bad} crc mismatches, \
         {min_bad} min_len mismatches, {len_bad} len mismatches, {missing} not in reference"
    );
    if !first_failures.is_empty() {
        println!("first mismatches (ours vs C#):");
        for line in &first_failures {
            println!("{line}");
        }
    }

    if crc_bad + min_bad + len_bad > 0 {
        bail!("generated metadata does not match the C# reference");
    }
    println!("all generated metadata matches the C# reference");
    Ok(())
}

/// Regenerates the dialect crate from the XML definitions.
fn codegen_mavlink(dialect: Option<&str>, check_only: bool) -> Result<()> {
    let root = repo_root();
    let dialect = dialect.unwrap_or("all");
    let xml = root
        .join("references/missionplanner/ExtLibs/Mavlink/message_definitions")
        .join(format!("{dialect}.xml"));
    if !xml.exists() {
        bail!(
            "dialect not found: {}\nThe reference tree is git-excluded; clone Mission Planner \
             into references/missionplanner to regenerate.",
            xml.display()
        );
    }

    let parsed = codegen::mavlink::parse_dialect(&xml)?;
    let source = codegen::emit::emit_dialect(&parsed);

    let out_dir = root.join("crates/mp-mavlink-dialects/src/generated");
    std::fs::create_dir_all(&out_dir)?;
    let out_file = out_dir.join(format!("{dialect}.rs"));

    // --check regenerates into a scratch file and diffs, so CI can prove the checked-in code
    // still corresponds to the definitions it claims to come from.
    let target = if check_only {
        out_dir.join(format!(".{dialect}.check.rs"))
    } else {
        out_file.clone()
    };
    std::fs::write(&target, &source).with_context(|| format!("writing {}", target.display()))?;
    let out_file = target;

    // Format the output so `cargo fmt --check` stays green and diffs stay readable.
    let status = std::process::Command::new("rustfmt")
        .args(["--edition", "2024"])
        .arg(&out_file)
        .status();
    match status {
        Ok(s) if s.success() => {}
        Ok(s) => bail!("rustfmt failed on generated output: {s}"),
        Err(e) => eprintln!("warning: rustfmt not available ({e}); output left unformatted"),
    }

    if check_only {
        let fresh = std::fs::read_to_string(&out_file)?;
        let committed_path = out_dir.join(format!("{dialect}.rs"));
        let committed = std::fs::read_to_string(&committed_path).unwrap_or_default();
        let _ = std::fs::remove_file(&out_file);
        if fresh != committed {
            bail!(
                "{} is out of date; run `cargo xtask codegen {dialect}`",
                committed_path.display()
            );
        }
        println!("generated code is up to date with the XML definitions");
        return Ok(());
    }

    println!(
        "generated {} ({} messages, {} enums, {} KiB)",
        out_file.display(),
        parsed.messages.len(),
        parsed.enums.len(),
        source.len() / 1024
    );
    Ok(())
}

/// Decodes a tlog and prints the same CSV shape `MpRefDump tlog` produces, so the two can be
/// diffed line by line when they disagree.
fn dump_tlog(path: Option<&str>) -> Result<()> {
    use mp_mavlink::parse;

    let path = path.context("usage: cargo xtask dump-tlog <file.tlog>")?;
    let log = std::fs::read(path).with_context(|| format!("reading {path}"))?;
    let dialect = &mp_mavlink_dialects::all::DIALECT;

    println!("index,msgid,seq,sysid,compid,payload_len,crc16,frame_hex");

    const MAX_SCAN: usize = 280;
    let mut pos = 0usize;
    let mut index = 0u64;
    while pos + 8 <= log.len() {
        pos += 8;
        let mut scan = pos;
        let limit = (pos + MAX_SCAN).min(log.len());
        while scan < limit
            && log.get(scan) != Some(&mp_mavlink::STX_V2)
            && log.get(scan) != Some(&mp_mavlink::STX_V1)
        {
            scan += 1;
        }
        if scan >= limit {
            if limit <= pos {
                break;
            }
            pos = limit;
            continue;
        }
        let Some(window) = log.get(scan..) else { break };
        match parse(window, dialect) {
            Ok((frame, used)) => {
                let hex: String = frame
                    .raw
                    .iter()
                    .map(|b| format!("{b:02x}"))
                    .collect::<Vec<_>>()
                    .join("");
                println!(
                    "{index},{},{},{},{},{},{},{hex}",
                    frame.msgid,
                    frame.seq,
                    frame.sysid,
                    frame.compid,
                    frame.payload.len(),
                    frame.checksum
                );
                pos = scan + used;
                index += 1;
            }
            Err(_) => pos = scan + 1,
        }
    }
    eprintln!("# frames={index}");
    Ok(())
}

/// Regenerates the flight mode tables.
fn codegen_modes() -> Result<()> {
    let root = repo_root();
    let metadata = root.join("references/missionplanner/ParameterMetaDataBackup.xml");
    if !metadata.exists() {
        bail!(
            "parameter metadata not found at {}\nThe reference tree is git-excluded; clone \
             Mission Planner into references/missionplanner to regenerate.",
            metadata.display()
        );
    }

    let source = codegen::modes::generate(&metadata)?;
    let out_dir = root.join("crates/mp-vehicle/src/generated");
    std::fs::create_dir_all(&out_dir)?;
    let out_file = out_dir.join("modes.rs");
    std::fs::write(&out_file, &source)?;

    let status = std::process::Command::new("rustfmt")
        .args(["--edition", "2024"])
        .arg(&out_file)
        .status();
    match status {
        Ok(s) if s.success() => {}
        Ok(s) => bail!("rustfmt failed on generated output: {s}"),
        Err(e) => eprintln!("warning: rustfmt not available ({e})"),
    }

    println!("generated {} ({} bytes)", out_file.display(), source.len());
    Ok(())
}

/// Regenerates parameter metadata.
fn codegen_param_meta() -> Result<()> {
    let root = repo_root();
    let metadata = root.join("references/missionplanner/ParameterMetaDataBackup.xml");
    if !metadata.exists() {
        bail!("parameter metadata not found at {}", metadata.display());
    }

    let source = codegen::param_meta::generate(&metadata, "ArduCopter2")?;
    let out_dir = root.join("crates/mp-params/src/generated");
    std::fs::create_dir_all(&out_dir)?;
    let out_file = out_dir.join("param_meta_copter.rs");
    std::fs::write(&out_file, &source)?;

    let status = std::process::Command::new("rustfmt")
        .args(["--edition", "2024"])
        .arg(&out_file)
        .status();
    match status {
        Ok(s) if s.success() => {}
        Ok(s) => bail!("rustfmt failed: {s}"),
        Err(e) => eprintln!("warning: rustfmt not available ({e})"),
    }

    println!(
        "generated {} ({} KiB)",
        out_file.display(),
        source.len() / 1024
    );
    Ok(())
}

/// `.resx` → `.ftl` under `assets/i18n/`, with `keymap.toml` and `report.md` (D17).
///
/// `--check` regenerates in memory and fails if any file differs from what is committed, or is
/// missing, or is there and would not be generated.
fn codegen_resx(check_only: bool) -> Result<()> {
    let root = repo_root();
    let tree = root.join("references/missionplanner");
    if !tree.is_dir() {
        bail!(
            "{} is absent (it is gitignored): clone Mission Planner into references/missionplanner \
             to regenerate.",
            tree.display()
        );
    }
    let out_dir = root.join("assets/i18n");
    let keymap = match std::fs::read_to_string(out_dir.join("keymap.toml")) {
        Ok(text) => codegen::resx::Keymap::parse(&text)?,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => codegen::resx::Keymap::default(),
        Err(e) => return Err(e).context("reading assets/i18n/keymap.toml"),
    };
    let screens = codegen::resx::screen_keys(&root)?;
    let output = codegen::resx::convert(&tree, keymap, &screens)?;

    if check_only {
        let mut stale = Vec::new();
        for (relative, text) in &output.files {
            if std::fs::read_to_string(out_dir.join(relative))
                .ok()
                .as_deref()
                != Some(text)
            {
                stale.push(relative.display().to_string());
            }
        }
        let mut walk = vec![out_dir.clone()];
        while let Some(dir) = walk.pop() {
            for entry in std::fs::read_dir(&dir).into_iter().flatten().flatten() {
                let path = entry.path();
                if path.is_dir() {
                    walk.push(path);
                } else if let Ok(relative) = path.strip_prefix(&out_dir)
                    && !output.files.contains_key(relative)
                {
                    stale.push(format!("{} (would not be generated)", relative.display()));
                }
            }
        }
        if !stale.is_empty() {
            bail!(
                "assets/i18n is stale; run `cargo xtask codegen-resx`. Differs: {}",
                stale.join(", ")
            );
        }
        println!("assets/i18n is up to date ({} files)", output.files.len());
        return Ok(());
    }
    for (relative, text) in &output.files {
        let path = out_dir.join(relative);
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        std::fs::write(&path, text).with_context(|| format!("writing {}", path.display()))?;
    }
    for base in &output.bases {
        println!(
            "{}: {} keys, {} cultures",
            base.source,
            base.keys,
            base.cultures.len() - 1
        );
    }
    println!("wrote {} files under assets/i18n", output.files.len());
    Ok(())
}
