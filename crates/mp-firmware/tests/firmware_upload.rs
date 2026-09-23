//! The byte protocol, against an in-process mock bootloader.
//!
//! D13's definition of done names this file and it is the gate before any real board is flashed.
//! PLAN.md R9 rates firmware flashing the only risk in the programme with *fatal* impact and no
//! SITL equivalent: a wrong byte here does not produce a wrong reading on a screen, it produces a
//! vehicle that will not boot and an operator who cannot recover it in the field.
//!
//! So the mock is deliberately strict. It asserts the exact bytes of every command rather than
//! looking for a prefix, and it refuses anything it did not expect. A mock that accepts whatever
//! arrives proves that the code runs, which is not the question.

// The mock (now in `common/`) indexes into buffers it has just length-checked, and the tests index
// into what it recorded. `expect` in a test: the panic message is the failure report.
#![allow(clippy::indexing_slicing, clippy::expect_used)]

use mp_firmware::protocol::{Code, Info};
use mp_firmware::{Firmware, Uploader, UploaderError};

mod common;

use common::MockBootloader;

/// Builds a firmware file the way the tools do: JSON, base64, zlib.
fn firmware_file(board_id: u32, image: &[u8]) -> Vec<u8> {
    use base64::Engine as _;
    use std::io::Write as _;
    let mut encoder = flate2::write::ZlibEncoder::new(Vec::new(), flate2::Compression::default());
    encoder.write_all(image).expect("compressing");
    let compressed = encoder.finish().expect("finishing");
    let encoded = base64::engine::general_purpose::STANDARD.encode(&compressed);
    format!(
        r#"{{"board_id":{board_id},"image_size":{},"image":"{encoded}","description":"test","board_revision":0}}"#,
        image.len()
    )
    .into_bytes()
}

#[test]
fn a_firmware_file_round_trips_through_the_container() {
    let image: Vec<u8> = (0..1000u32)
        .map(|i| u8::try_from(i % 251).unwrap_or(0))
        .collect();
    let firmware = Firmware::parse(&firmware_file(9, &image)).expect("a valid firmware file");

    assert_eq!(firmware.board_id, 9);
    assert_eq!(firmware.declared_image_size, 1000);
    assert_eq!(firmware.image.len(), 1000, "already a multiple of four");
    assert_eq!(firmware.image, image);
}

/// The padding is the part that changes the CRC, so it is asserted on a size that needs it.
#[test]
fn an_image_that_is_not_a_multiple_of_four_is_padded() {
    let image = vec![0xAB; 1001];
    let firmware = Firmware::parse(&firmware_file(9, &image)).expect("valid");
    assert_eq!(firmware.declared_image_size, 1001);
    // 1001 + (1001 % 4) = 1002, then rounded up to 1004.
    assert_eq!(firmware.image.len(), 1004);
    assert_eq!(&firmware.image[..1001], &image[..]);
}

/// The whole sequence, with every byte checked by the mock.
#[test]
fn a_clean_upload_sends_the_protocol_the_bootloader_expects() {
    let image: Vec<u8> = (0..600u32)
        .map(|i| u8::try_from(i % 251).unwrap_or(0))
        .collect();
    let firmware = Firmware::parse(&firmware_file(9, &image)).expect("valid");

    let mut uploader = Uploader::new(MockBootloader::new(9, 2048));
    let mut seen = Vec::new();
    let board = uploader
        .upload(&firmware, |fraction| seen.push(fraction))
        .expect("the upload should succeed against a well-behaved bootloader");

    assert_eq!(board.board_id, 9);
    assert_eq!(board.bootloader_revision, 5);
    assert_eq!(board.flash_size, 2048);

    let mock = uploader.into_inner();
    assert!(mock.erased, "the flash must be erased before it is written");
    assert_eq!(mock.flash, firmware.image, "the flash must hold the image");
    assert!(
        mock.received.is_empty(),
        "every byte sent must be a complete command: {:?} left over",
        mock.received
    );

    // 600 bytes in 252-byte blocks is three blocks, and progress must reach 1.0 exactly.
    assert_eq!(seen.len(), 3);
    assert!((seen.last().copied().unwrap_or(0.0) - 1.0).abs() < f32::EPSILON);
}

/// Wrong firmware for the board must be refused **before** the erase.
///
/// The ordering is the whole point: erasing first and finding out afterwards leaves a vehicle with
/// no firmware and an operator who has to find the right file before it flies again.
#[test]
fn a_firmware_for_another_board_is_refused_before_anything_is_erased() {
    let firmware = Firmware::parse(&firmware_file(9, &[0u8; 64])).expect("valid");
    let mut uploader = Uploader::new(MockBootloader::new(50, 2048));

    let outcome = uploader.upload(&firmware, |_| {});
    assert!(
        matches!(
            outcome,
            Err(UploaderError::WrongBoard {
                firmware: 9,
                board: 50
            })
        ),
        "expected a board mismatch, got {outcome:?}"
    );
    assert!(
        !uploader.into_inner().erased,
        "the flash was erased for a firmware that was never going to be written"
    );
}

/// An image too large for the board is also refused before the erase.
#[test]
fn an_oversized_firmware_is_refused_before_anything_is_erased() {
    let firmware = Firmware::parse(&firmware_file(9, &[0u8; 4096])).expect("valid");
    let mut uploader = Uploader::new(MockBootloader::new(9, 1024));

    let outcome = uploader.upload(&firmware, |_| {});
    assert!(
        matches!(outcome, Err(UploaderError::TooLarge { .. })),
        "expected a size refusal, got {outcome:?}"
    );
    assert!(!uploader.into_inner().erased);
}

/// A board whose CRC disagrees must fail loudly rather than reboot into a bad image.
#[test]
fn a_crc_mismatch_fails_rather_than_rebooting() {
    let firmware = Firmware::parse(&firmware_file(9, &[0x5A; 256])).expect("valid");
    let mut mock = MockBootloader::new(9, 2048);
    mock.crc_override = Some(0xDEAD_BEEF);
    let mut uploader = Uploader::new(mock);

    let outcome = uploader.upload(&firmware, |_| {});
    assert!(
        matches!(
            outcome,
            Err(UploaderError::CrcMismatch {
                reported: 0xDEAD_BEEF,
                ..
            })
        ),
        "expected a CRC mismatch, got {outcome:?}"
    );
}

/// A bootloader too old or too new is refused rather than guessed at.
#[test]
fn an_unsupported_bootloader_revision_is_refused() {
    for revision in [1u32, 21, 99] {
        let mut mock = MockBootloader::new(9, 2048);
        mock.info.insert(Info::BootloaderRevision.byte(), revision);
        let mut uploader = Uploader::new(mock);
        let outcome = uploader.identify();
        assert!(
            matches!(outcome, Err(UploaderError::UnsupportedRevision(r)) if r == revision),
            "revision {revision} should be refused, got {outcome:?}"
        );
    }
    // And the ends of the supported range are accepted.
    for revision in [2u32, 20] {
        let mut mock = MockBootloader::new(9, 2048);
        mock.info.insert(Info::BootloaderRevision.byte(), revision);
        let mut uploader = Uploader::new(mock);
        assert!(
            uploader.identify().is_ok(),
            "revision {revision} is supported"
        );
    }
}

/// `INVALID` and `FAILED` are different answers and must not be collapsed.
#[test]
fn the_bootloaders_two_refusals_are_told_apart() {
    for (byte, name) in [
        (Code::Invalid.byte(), "invalid"),
        (Code::Failed.byte(), "failed"),
    ] {
        let mut mock = MockBootloader::new(9, 2048);
        mock.outgoing.push_back(Code::InSync.byte());
        mock.outgoing.push_back(byte);
        let mut uploader = Uploader::new(mock);
        let outcome = uploader.sync();
        match (name, &outcome) {
            ("invalid", Err(UploaderError::Invalid)) | ("failed", Err(UploaderError::Failed)) => {}
            _ => panic!("{name} was not reported as itself: {outcome:?}"),
        }
    }
}

/// A reply that does not start with INSYNC is a desynchronised link, not a failure.
#[test]
fn a_reply_without_insync_is_reported_as_such() {
    let mut mock = MockBootloader::new(9, 2048);
    mock.outgoing.push_back(0x99);
    mock.outgoing.push_back(Code::Ok.byte());
    let mut uploader = Uploader::new(mock);
    assert!(matches!(
        uploader.sync(),
        Err(UploaderError::NotInSync(0x99))
    ));
}
