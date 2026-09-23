//! GMap.NET's URL cache, including the part of its behaviour that depends on the whole process.
//!
//! One test, in a process of its own, because what it checks is a sequence: in the C#, nothing is
//! read from the URL cache until something has been written to it in the same run
//! (`Cache.cs:119, 188-189, 220-221`), so the first version check of a run always goes to the
//! network. A test that ran after another had written would see a different answer.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use std::path::PathBuf;
use std::time::{Duration, SystemTime};

use mp_tiles::urlcache;

const PAGE: &str = "http://www.bing.com/maps";
const OTHER: &str = "http://maps.google.com/maps/api/js?v=3.2&sensor=false";

#[test]
fn nothing_is_read_before_something_is_written_and_old_pages_are_deleted() {
    let root = std::env::temp_dir().join(format!("mp-tiles-urlcache-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&root);
    let cleanup = Cleanup(root.clone());

    // A fresh page the C# left behind, exactly where it leaves it.
    let page = urlcache::path(&root, PAGE);
    std::fs::create_dir_all(page.parent().unwrap()).unwrap();
    std::fs::write(&page, "\u{feff}tilegeneration:15512").unwrap();

    // Not read: this process has not written to the cache yet, and neither would the C# have.
    assert_eq!(urlcache::get(&root, PAGE, urlcache::STAY_IN_CACHE), None);
    assert!(page.is_file(), "a miss for that reason deletes nothing");

    // The first write - of another page - is what makes the cache readable.
    urlcache::save(&root, OTHER, "loader");
    let written = std::fs::read(urlcache::path(&root, OTHER)).unwrap();
    assert_eq!(
        written,
        "\u{feff}loader".as_bytes(),
        "UTF-8 with a byte-order mark"
    );

    // Now the fresh page is read, without the byte-order mark.
    assert_eq!(
        urlcache::get(&root, PAGE, urlcache::STAY_IN_CACHE).as_deref(),
        Some("tilegeneration:15512")
    );

    // An old page is deleted rather than read.
    let nine_hours_ago = SystemTime::now() - Duration::from_secs(9 * 60 * 60);
    std::fs::File::options()
        .write(true)
        .open(&page)
        .unwrap()
        .set_modified(nine_hours_ago)
        .unwrap();
    assert_eq!(urlcache::get(&root, PAGE, urlcache::STAY_IN_CACHE), None);
    assert!(!page.exists(), "a page past its eight hours is deleted");

    // And a page never saved is simply absent.
    assert_eq!(
        urlcache::get(&root, "http://example.invalid/", urlcache::STAY_IN_CACHE),
        None
    );
    drop(cleanup);
}

/// Removes a directory when dropped, so a failed assertion does not leave it behind.
struct Cleanup(PathBuf);

impl Drop for Cleanup {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}
