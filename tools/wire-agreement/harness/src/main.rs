// Copyright (C) 2026 David "Buzz" Bussenschutt
//
// This file is part of MissionPlannerRust; see LICENSE (GPL-3.0-only).
//
// SPDX-License-Identifier: GPL-3.0-only

//! This repository's MAVLink binding - `mp-mavlink`'s framing, `mp-mavlink-dialects`' messages,
//! `mp-link`'s signing and target reading - against tridge's generated Rust binding (pymavlink
//! pull request 1303, `mavgen --lang=Rust`) as the reference, both ways, byte for byte.
//!
//! For every case `cases.py` writes (a payload per message of Mission Planner's `all.xml` and
//! pattern), and every MAVLink 1 and 2 framing, source id (narrow and 32-bit), target (narrow and
//! 32-bit, for a message that names one) and signing either way, it checks:
//!
//! * both bindings' tables agree with the XML: CRC_EXTRA, lengths, the target's offset;
//! * each binding's typed message, decoded from the payload, encodes back to the same bytes;
//! * the frame each builds is the same bytes as the other's;
//! * each parses the other's frame - source, target, signature, typed message - and the
//!   reference writes ours back unchanged;
//! * this repository's streaming decoder takes all the reference's frames, back to back, without
//!   a byte skipped.
//!
//! Any difference is printed and the exit status is 1. Last, one byte of our frames is changed
//! for one message, and the harness must see it, so that one which compared nothing would fail.

use std::panic::{AssertUnwindSafe, catch_unwind};
use std::process::ExitCode;
use std::sync::atomic::{AtomicBool, Ordering};

use mavlink_generated::DefaultDialect as Reference;
use mavlink_generated::spec::{Dialect as _, IntoPayload as _, MavLinkVersion, Payload};
use mavlink_generated::wire::{Frame as RefFrame, MAX_FRAME_SIZE, WireDialect as _};
use mp_mavlink::{Dialect as _, FrameDecoder, MAX_FRAME_LEN, SigningKey};
use mp_mavlink_dialects::all::{DIALECT, MESSAGES, MavMessage};

/// The pull request's own test's header values (`tests/rust/oracle.c`).
const SEQ: u8 = 239;
const COMPID: u8 = 11;
const KEY: [u8; 32] = [42; 32];
const LINK_ID: u8 = 3;
const TIMESTAMP: u64 = 1000;
const SOURCES: [u32; 5] = [42, 255, 256, 0x8000_0001, u32::MAX];
const TARGETS: [u32; 5] = [0, 7, 255, 256, u32::MAX];
/// Differences printed in full; the rest are counted.
const SHOWN: usize = 40;

/// The self-check's changed byte in every frame this repository builds.
static TAMPER: AtomicBool = AtomicBool::new(false);

struct Case {
    id: u32,
    name: String,
    pattern: u32,
    min_len: usize,
    len: usize,
    crc_extra: u8,
    target_offset: Option<usize>,
    payload: Vec<u8>,
}

/// One framing of a case.
#[derive(Clone, Copy)]
struct Shape {
    v1: bool,
    source: u32,
    target: u32,
    signed: bool,
}

#[derive(Default)]
struct Report {
    differences: Vec<String>,
    frames: usize,
    /// Of `frames`: MAVLink 1, signed, from a 32-bit source, to a 32-bit target.
    v1: usize,
    signed: usize,
    wide_source: usize,
    wide_target: usize,
    /// The reference's frames, back to back, and each one's length, for the streaming check.
    stream: Vec<u8>,
    lengths: Vec<usize>,
}

impl Report {
    fn differ(&mut self, case: &Case, shape: Option<Shape>, what: impl AsRef<str>) {
        let at = shape.map_or(String::new(), |s| {
            format!(
                " v{} source {} target {} {}",
                if s.v1 { 1 } else { 2 },
                s.source,
                s.target,
                if s.signed { "signed" } else { "unsigned" }
            )
        });
        self.differences.push(format!(
            "{} ({}) pattern {}{at}: {}",
            case.name,
            case.id,
            case.pattern,
            what.as_ref()
        ));
    }
}

fn hex(bytes: &[u8]) -> String {
    use std::fmt::Write as _;
    bytes
        .iter()
        .fold(String::with_capacity(bytes.len() * 2), |mut out, b| {
            let _ = write!(out, "{b:02x}");
            out
        })
}

fn unhex(text: &str) -> Vec<u8> {
    (0..text.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&text[i..i + 2], 16).expect("hex"))
        .collect()
}

fn read_cases(path: &str) -> Vec<Case> {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("{path}: {e}"));
    text.lines()
        .map(|line| {
            let parts: Vec<&str> = line.split_whitespace().collect();
            let number = |i: usize| parts[i].parse::<i64>().expect("a number");
            Case {
                id: u32::try_from(number(0)).expect("an id"),
                name: parts[1].to_owned(),
                pattern: u32::try_from(number(2)).expect("a pattern"),
                min_len: usize::try_from(number(3)).expect("a length"),
                len: usize::try_from(number(4)).expect("a length"),
                crc_extra: u8::try_from(number(5)).expect("a CRC_EXTRA"),
                target_offset: usize::try_from(number(6)).ok(),
                payload: unhex(parts[7]),
            }
        })
        .collect()
}

/// MAVLink 2's trailing-zero truncation, at least one byte kept.
fn trimmed(payload: &[u8]) -> &[u8] {
    mp_mavlink::trim_payload(payload)
}

/// Both bindings' tables against the XML's numbers.
fn check_tables(case: &Case, report: &mut Report) {
    match DIALECT.info(case.id) {
        None => report.differ(case, None, "not in this repository's dialect"),
        Some(info) => {
            let ours = (
                info.crc_extra,
                usize::from(info.min_len),
                usize::from(info.len),
            );
            if ours != (case.crc_extra, case.min_len, case.len) {
                report.differ(
                    case,
                    None,
                    format!(
                        "this repository's table {ours:?}, the XML's {:?}",
                        (case.crc_extra, case.min_len, case.len)
                    ),
                );
            }
        }
    }
    match Reference::wire_info(case.id) {
        None => report.differ(case, None, "not in the reference's dialect"),
        Some(info) => {
            let theirs = (
                info.crc_extra,
                usize::from(info.min_length),
                usize::from(info.length),
                info.target_offset,
            );
            let xml = (case.crc_extra, case.min_len, case.len, case.target_offset);
            if theirs != xml {
                report.differ(
                    case,
                    None,
                    format!("the reference's table {theirs:?}, the XML's {xml:?}"),
                );
            }
        }
    }
}

/// This repository's typed message from a payload, encoded again: the full payload.
fn ours_typed(id: u32, payload: &[u8]) -> Option<Vec<u8>> {
    let message = MavMessage::decode(id, payload)?;
    let mut out = [0u8; 255];
    let n = message.encode(&mut out);
    Some(out[..n].to_vec())
}

/// The reference's typed message from a payload, encoded again in the same framing.
fn theirs_typed(id: u32, payload: &[u8], version: MavLinkVersion) -> Result<Vec<u8>, String> {
    let message =
        Reference::decode(&Payload::new(id, payload, version)).map_err(|e| format!("{e:?}"))?;
    let encoded = message.encode(version).map_err(|e| format!("{e:?}"))?;
    Ok(encoded.bytes().to_vec())
}

fn check_typed(case: &Case, report: &mut Report) {
    match ours_typed(case.id, &case.payload) {
        None => report.differ(case, None, "this repository's dialect does not decode it"),
        Some(bytes) if bytes != case.payload => report.differ(
            case,
            None,
            format!(
                "this repository's typed round trip\n    gave {}\n    want {}",
                hex(&bytes),
                hex(&case.payload)
            ),
        ),
        Some(_) => {}
    }
    match theirs_typed(case.id, &case.payload, MavLinkVersion::V2) {
        Err(e) => report.differ(case, None, format!("the reference does not decode it: {e}")),
        Ok(bytes) if bytes != trimmed(&case.payload) => report.differ(
            case,
            None,
            format!(
                "the reference's typed round trip\n    gave {}\n    want {}",
                hex(&bytes),
                hex(trimmed(&case.payload))
            ),
        ),
        Ok(_) => {}
    }
    if case.id <= 255 {
        let v1 = &case.payload[..case.min_len];
        match theirs_typed(case.id, v1, MavLinkVersion::V1) {
            Err(e) => report.differ(
                case,
                None,
                format!("the reference does not decode its MAVLink 1 payload: {e}"),
            ),
            Ok(bytes) if bytes != v1 => report.differ(
                case,
                None,
                format!(
                    "the reference's MAVLink 1 typed round trip\n    gave {}\n    want {}",
                    hex(&bytes),
                    hex(v1)
                ),
            ),
            Ok(_) => {}
        }
    }
}

/// The payload with the target in it as a sender puts it: the id, or 255 for one over a byte
/// (`SetPayloadTarget`).
fn with_target(case: &Case, target: u32) -> Vec<u8> {
    let mut payload = case.payload.clone();
    if let Some(offset) = case.target_offset {
        payload[offset] = u8::try_from(target).unwrap_or(255);
    }
    payload
}

/// The frame as this repository sends it: the typed message encoded, framed by `mp-mavlink`
/// (a wide target in the header, as `mp-link`'s `with_wide_target` frames it) and signed by
/// `mp-link`.
fn ours_frame(case: &Case, shape: Shape) -> Result<Vec<u8>, String> {
    let payload =
        ours_typed(case.id, &with_target(case, shape.target)).ok_or("no typed message")?;
    let mut out = [0u8; MAX_FRAME_LEN];
    let n = if shape.v1 {
        mp_mavlink::encode_v1(
            &mut out,
            SEQ,
            u8::try_from(shape.source).map_err(|e| e.to_string())?,
            COMPID,
            case.id,
            &payload[..case.min_len],
            case.crc_extra,
        )
    } else {
        mp_mavlink::encode_v2_targeted(
            &mut out,
            SEQ,
            shape.source,
            COMPID,
            case.id,
            &payload,
            case.crc_extra,
            0,
            case.target_offset.map(|_| shape.target),
        )
    }
    .map_err(|e| e.to_string())?;
    let mut frame = out[..n].to_vec();
    if TAMPER.load(Ordering::Relaxed) {
        frame[n - 1] ^= 0x01;
    }
    if shape.signed {
        mp_link::signing::sign_frame(&frame, &KEY, LINK_ID, TIMESTAMP)
            .ok_or_else(|| "not signed".to_owned())
    } else {
        Ok(frame)
    }
}

/// The frame as the reference builds it: its typed message encoded, framed, retargeted, signed.
fn theirs_frame(case: &Case, shape: Shape) -> Result<Vec<u8>, String> {
    let version = if shape.v1 {
        MavLinkVersion::V1
    } else {
        MavLinkVersion::V2
    };
    let payload = if shape.v1 {
        &case.payload[..case.min_len]
    } else {
        &case.payload[..]
    };
    let message = Reference::decode(&Payload::new(case.id, payload, version))
        .map_err(|e| format!("{e:?}"))?;
    let encoded = message.encode(version).map_err(|e| format!("{e:?}"))?;
    let info = Reference::wire_info(case.id).ok_or("no wire info")?;
    let mut frame = RefFrame::from_payload(SEQ, shape.source, COMPID, &encoded, info)
        .map_err(|e| format!("{e:?}"))?;
    if case
        .target_offset
        .is_some_and(|offset| !shape.v1 || offset < case.min_len)
    {
        frame.retarget(shape.target).map_err(|e| format!("{e:?}"))?;
    }
    if shape.signed {
        frame
            .sign(&KEY, LINK_ID, TIMESTAMP)
            .map_err(|e| format!("{e:?}"))?;
    }
    let mut out = [0u8; MAX_FRAME_SIZE];
    let n = frame.write(&mut out);
    Ok(out[..n].to_vec())
}

/// The target a receiver should read: none where MAVLink 1 leaves an extension's target out.
fn expected_target(case: &Case, shape: Shape) -> Option<u32> {
    case.target_offset
        .filter(|&offset| !shape.v1 || offset < case.min_len)
        .map(|_| shape.target)
}

/// The reference reads this repository's frame.
fn reference_reads(case: &Case, shape: Shape, ours: &[u8]) -> Result<(), String> {
    let frame = RefFrame::parse::<Reference>(ours).map_err(|e| format!("{e:?}"))?;
    let heard = (
        frame.system_id(),
        frame.component_id(),
        frame.message_id(),
        frame.sequence(),
    );
    if heard != (shape.source, COMPID, case.id, SEQ) {
        return Err(format!("source/component/id/sequence {heard:?}"));
    }
    if frame.target_system() != expected_target(case, shape) {
        return Err(format!("target {:?}", frame.target_system()));
    }
    if frame.is_signed() != shape.signed {
        return Err(format!("signed {}", frame.is_signed()));
    }
    if shape.signed {
        frame
            .verify_signature(&KEY)
            .map_err(|e| format!("signature {e:?}"))?;
    }
    let message = frame
        .decode::<Reference>()
        .map_err(|e| format!("typed {e:?}"))?;
    let version = frame.version();
    let again = message
        .encode(version)
        .map_err(|e| format!("typed {e:?}"))?;
    if again.bytes() != frame.payload() {
        return Err(format!(
            "typed round trip {} of {}",
            hex(again.bytes()),
            hex(frame.payload())
        ));
    }
    let mut out = [0u8; MAX_FRAME_SIZE];
    let n = frame.write(&mut out);
    if out[..n] != *ours {
        return Err(format!("written back as {}", hex(&out[..n])));
    }
    Ok(())
}

/// This repository reads the reference's frame, as the product does: `parse`, the signature,
/// the typed message, `mp-link`'s target.
fn we_read(case: &Case, shape: Shape, theirs: &[u8]) -> Result<(), String> {
    let (frame, used) = mp_mavlink::parse(theirs, &DIALECT).map_err(|e| e.to_string())?;
    if used != theirs.len() {
        return Err(format!("{used} of {} bytes read", theirs.len()));
    }
    let heard = (frame.sysid, frame.compid, frame.msgid, frame.seq);
    if heard != (shape.source, COMPID, case.id, SEQ) {
        return Err(format!("source/component/id/sequence {heard:?}"));
    }
    if frame.is_signed() != shape.signed {
        return Err(format!("signed {}", frame.is_signed()));
    }
    if shape.signed && !mp_mavlink::verify(&SigningKey::new(KEY), &frame) {
        return Err("signature does not verify".to_owned());
    }
    let message = MavMessage::decode(frame.msgid, frame.payload).ok_or("no typed message")?;
    let (target, _) = mp_link::target_of(&frame, &message);
    // Without a target field, `target_of` has none; MAVLink 1 without the extension reads 0.
    let want = expected_target(case, shape).or(case.target_offset.map(|_| 0));
    if target != want {
        return Err(format!("target {target:?}, want {want:?}"));
    }
    let mut full = vec![0u8; case.len];
    frame.payload_into(&mut full);
    let typed = ours_typed(case.id, &full).ok_or("no typed message")?;
    if typed != full {
        return Err(format!(
            "typed round trip {} of {}",
            hex(&typed),
            hex(&full)
        ));
    }
    Ok(())
}

fn shapes(case: &Case) -> Vec<Shape> {
    let targets: &[u32] = if case.target_offset.is_some() {
        &TARGETS
    } else {
        &[0]
    };
    let mut out = Vec::new();
    for v1 in [true, false] {
        for &source in &SOURCES {
            for &target in targets {
                for signed in [false, true] {
                    if v1 && (case.id > 255 || source > 255 || target > 255 || signed) {
                        continue;
                    }
                    // MAVLink 1 carries no extension, so no target that is one.
                    if v1 && target != 0 && case.target_offset.is_some_and(|o| o >= case.min_len) {
                        continue;
                    }
                    out.push(Shape {
                        v1,
                        source,
                        target,
                        signed,
                    });
                }
            }
        }
    }
    out
}

fn check_frames(case: &Case, report: &mut Report) {
    for shape in shapes(case) {
        let ours = match ours_frame(case, shape) {
            Ok(frame) => frame,
            Err(e) => {
                report.differ(
                    case,
                    Some(shape),
                    format!("this repository builds no frame: {e}"),
                );
                continue;
            }
        };
        let theirs = match theirs_frame(case, shape) {
            Ok(frame) => frame,
            Err(e) => {
                report.differ(
                    case,
                    Some(shape),
                    format!("the reference builds no frame: {e}"),
                );
                continue;
            }
        };
        report.frames += 1;
        report.v1 += usize::from(shape.v1);
        report.signed += usize::from(shape.signed);
        report.wide_source += usize::from(shape.source > 255);
        report.wide_target += usize::from(case.target_offset.is_some() && shape.target > 255);
        if ours != theirs {
            report.differ(
                case,
                Some(shape),
                format!(
                    "frames differ\n    ours      {}\n    reference {}",
                    hex(&ours),
                    hex(&theirs)
                ),
            );
        }
        if let Err(e) = reference_reads(case, shape, &ours) {
            report.differ(
                case,
                Some(shape),
                format!("the reference reading ours: {e}"),
            );
        }
        if let Err(e) = we_read(case, shape, &theirs) {
            report.differ(
                case,
                Some(shape),
                format!("this repository reading the reference's: {e}"),
            );
        }
        report.lengths.push(theirs.len());
        report.stream.extend_from_slice(&theirs);
    }
}

/// The reference's frames back to back through the product's streaming decoder, in reads of an
/// awkward size: every frame, in order, nothing skipped.
fn check_stream(report: &mut Report) {
    let mut decoder = FrameDecoder::new();
    let mut heard = Vec::new();
    let stream = std::mem::take(&mut report.stream);
    for chunk in stream.chunks(61) {
        decoder.push_and_drain(chunk, &DIALECT, |frame| heard.push(frame.raw.len()));
    }
    let stats = decoder.stats();
    if heard != report.lengths || stats.resync_bytes != 0 || stats.crc_errors != 0 {
        report.differences.push(format!(
            "streaming decoder: {} frames of {}, {} bytes skipped, {} checksum errors",
            heard.len(),
            report.lengths.len(),
            stats.resync_bytes,
            stats.crc_errors
        ));
    }
}

fn main() -> ExitCode {
    let path = std::env::args()
        .nth(1)
        .expect("usage: wire-agreement <cases.txt>");
    let cases = read_cases(&path);
    let mut report = Report::default();
    let mut ids = std::collections::BTreeSet::new();
    for case in &cases {
        ids.insert(case.id);
        let outcome = catch_unwind(AssertUnwindSafe(|| {
            let mut local = Report::default();
            check_tables(case, &mut local);
            check_typed(case, &mut local);
            check_frames(case, &mut local);
            local
        }));
        match outcome {
            Ok(local) => {
                report.differences.extend(local.differences);
                report.frames += local.frames;
                report.v1 += local.v1;
                report.signed += local.signed;
                report.wide_source += local.wide_source;
                report.wide_target += local.wide_target;
                report.stream.extend(local.stream);
                report.lengths.extend(local.lengths);
            }
            Err(_) => report.differ(case, None, "panicked"),
        }
    }
    let extra: Vec<u32> = MESSAGES
        .iter()
        .map(|m| m.id)
        .filter(|id| !ids.contains(id))
        .collect();
    if !extra.is_empty() {
        report.differences.push(format!(
            "this repository's dialect has ids the XML does not: {extra:?}"
        ));
    }
    check_stream(&mut report);

    TAMPER.store(true, Ordering::Relaxed);
    let mut tampered = Report::default();
    if let Some(case) = cases.first() {
        check_frames(case, &mut tampered);
    }
    if tampered.differences.is_empty() {
        report
            .differences
            .push("self-check: a changed byte in our frames was not seen".to_owned());
    }

    for difference in report.differences.iter().take(SHOWN) {
        println!("DIFFERENCE {difference}");
    }
    if report.differences.len() > SHOWN {
        println!("... and {} more", report.differences.len() - SHOWN);
    }
    println!(
        "wire agreement: {} messages, {} cases, {} frames built by both and read by each other \
         ({} MAVLink 1, {} signed, {} from a 32-bit source, {} to a 32-bit target), {} differences",
        ids.len(),
        cases.len(),
        report.frames,
        report.v1,
        report.signed,
        report.wide_source,
        report.wide_target,
        report.differences.len()
    );
    if report.differences.is_empty() {
        ExitCode::SUCCESS
    } else {
        ExitCode::FAILURE
    }
}
