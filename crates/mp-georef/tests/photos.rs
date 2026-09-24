//! The synthetic photos of `testdata/georef/photos` are what `common::photos` builds, and each
//! reads back as the time it was built with.
//!
//! `MP_GEOREF_REGEN=1 cargo test -p mp-georef --test photos` writes them afresh; the oracle
//! (`tools/csharp-reference/regen-georef.sh`) reads the committed copies.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

mod common;

use common::photos;
use mp_georef::exif_read;
use mp_georef::photos::PhotoTimes;
use mp_georef::time::{DateTime, Kind};

#[test]
fn the_committed_photos_are_the_generated_ones() {
    let dir = common::data().join("photos");
    let edge = common::data().join("edge");
    if std::env::var_os("MP_GEOREF_REGEN").is_some() {
        std::fs::create_dir_all(&dir).unwrap();
        for n in 1..=25 {
            std::fs::write(dir.join(photos::photo_name(n)), photos::photo(n)).unwrap();
        }
        std::fs::create_dir_all(&edge).unwrap();
        for (name, bytes) in photos::edge_photos() {
            std::fs::write(edge.join(name), bytes).unwrap();
        }
    }
    for n in 1..=25 {
        let committed = std::fs::read(dir.join(photos::photo_name(n))).unwrap();
        assert_eq!(committed, photos::photo(n), "photo {n}");
    }
    for (name, bytes) in photos::edge_photos() {
        assert_eq!(std::fs::read(edge.join(name)).unwrap(), bytes, "{name}");
    }
}

#[test]
fn every_photo_reads_back_its_time() {
    let today = DateTime::from_parts(2030, 1, 1, 0, 0, 0, Kind::Local).unwrap();
    let mut times = PhotoTimes::new(today);
    for n in 1..=25 {
        let path = common::data().join("photos").join(photos::photo_name(n));
        let time = times.get(path.to_str().unwrap());
        assert_eq!(time.format_exif(), photos::photo_time(n), "photo {n}");
        // The date comes from the Exif directory, the first created.
        let metadata = exif_read::read_jpeg(&std::fs::read(&path).unwrap()).unwrap();
        assert!(matches!(
            metadata.directories.first(),
            Some((exif_read::Directory::Exif, _))
        ));
    }
}
