//! What the integration tests share: the synthetic photos and the fixtures' paths.

#![allow(
    dead_code,
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing
)]

pub(crate) mod photos;

use std::path::PathBuf;

/// `testdata/georef`.
pub(crate) fn data() -> PathBuf {
    PathBuf::from(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../testdata/georef"
    ))
}

/// A scratch directory for one test, emptied first.
pub(crate) fn scratch(name: &str) -> PathBuf {
    let dir = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join(name);
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// Copies the 25 photos into `dir`.
pub(crate) fn copy_photos(dir: &std::path::Path) {
    for n in 1..=25 {
        let name = photos::photo_name(n);
        std::fs::copy(data().join("photos").join(&name), dir.join(&name)).unwrap();
    }
}
