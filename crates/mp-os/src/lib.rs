// Copyright (C) 2026 David "Buzz" Bussenschutt
//
// This file is part of MissionPlannerRust, a Rust implementation derived from
// Mission Planner (Copyright (C) 2010-2024 Michael Oborne and contributors,
// https://github.com/ArduPilot/MissionPlanner); NOTICE records the changes.
//
// MissionPlannerRust is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by the
// Free Software Foundation, version 3 of the License.
//
// MissionPlannerRust is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License along with
// MissionPlannerRust. If not, see <https://www.gnu.org/licenses/>.
//
// SPDX-License-Identifier: GPL-3.0-only

//! The std calls that panic in a web page rather than fail, each std's own on the desktop.
//!
//! On `wasm32-unknown-unknown` most of std's operating system calls return an error - a file
//! does not open, a socket does not connect - and the planner already handles that. Three panic
//! instead: `std::env::temp_dir` ("no filesystem on this platform"), `std::process::id` ("no
//! pids on this platform") and `std::env::split_paths`. Here a web page gets an answer that leads
//! to the ordinary error further on: a temporary folder no file can be made in, an id of 0, and
//! no folders on the path. (web/tools/port_os.py puts these in place.)
//!
//! And one the desktop has another way, [`http`]: a web page has no sockets, so the crates that
//! fetch over ureq on the desktop fetch through the browser in a page. And one only a page has,
//! [`page_query`]: what its address asks for. And [`fs`]: std's files on the desktop, the page's
//! own in a page, kept between visits.

pub mod fs;

/// Whether this thread may wait - for a thread's end, a sleep, a lock held elsewhere: always on
/// the desktop; in a web page not on its main thread, where the browser forbids it and std's wait
/// (`memory.atomic.wait`) traps. A trap there ends the planner mid-update and leaves gpui's state
/// borrowed for good - the next update panics "RefCell already borrowed" (the owner's DISCONNECT
/// in two browsers, 2026-10-05: Link::close joined the link's thread). What would wait there is
/// told to stop and left to end on its own.
#[must_use]
pub fn may_block() -> bool {
    #[cfg(target_family = "wasm")]
    {
        !on_main_thread()
    }
    #[cfg(not(target_family = "wasm"))]
    {
        true
    }
}

/// A thread's handle let go of: joined where this thread may wait ([`may_block`]), its result
/// given; on a web page's main thread, which may not, left to end on its own - for a thread told
/// to stop, or done. Not even a thread `is_finished` calls done may be joined there:
/// wasm_thread's thread lets go of its share of the result, which `is_finished` counts, before
/// it signals the end that `join` waits for, so a join in between waits, and a wait there traps
/// ("Atomics.wait cannot be called in this context" - a map type chosen in a page ended the
/// planner so, its tile store's threads joined as it went, 2026-10-06).
pub fn join_or_leave<T>(handle: wasm_thread::JoinHandle<T>) -> Option<std::thread::Result<T>> {
    if may_block() {
        Some(handle.join())
    } else {
        drop(handle);
        None
    }
}

/// [`wake`]s so far.
static WAKES: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// Something a window shows has changed off the window's thread - a plugin's request sent, a
/// result handed over: the window's repaint loop, which looks at [`wakes`], draws at its next look
/// rather than at its floor (crates/mp-gui/src/repaint.rs). One atomic add, from any thread.
pub fn wake() {
    WAKES.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
}

/// How many [`wake`]s there have been.
#[must_use]
pub fn wakes() -> u64 {
    WAKES.load(std::sync::atomic::Ordering::Relaxed)
}

use std::ffi::OsStr;
use std::path::PathBuf;
#[cfg(target_family = "wasm")]
use std::sync::TryLockError;
#[cfg(target_family = "wasm")]
use std::sync::mpsc::TryRecvError;
use std::sync::mpsc::{Receiver, RecvTimeoutError};
use std::sync::{LockResult, Mutex, MutexGuard, RwLock, RwLockReadGuard, RwLockWriteGuard};
use std::time::Duration;

/// `std::env::temp_dir`; in a web page `/tmp`, where nothing can be written.
#[must_use]
pub fn temp_dir() -> PathBuf {
    #[cfg(not(target_family = "wasm"))]
    {
        std::env::temp_dir()
    }
    #[cfg(target_family = "wasm")]
    {
        PathBuf::from("/tmp")
    }
}

/// `std::process::id`; 0 in a web page, which has no processes.
#[must_use]
pub fn process_id() -> u32 {
    #[cfg(not(target_family = "wasm"))]
    {
        std::process::id()
    }
    #[cfg(target_family = "wasm")]
    {
        0
    }
}

/// `std::env::split_paths`, collected; nothing in a web page, which has no programs to find.
#[must_use]
pub fn split_paths(paths: &(impl AsRef<OsStr> + ?Sized)) -> std::vec::IntoIter<PathBuf> {
    #[cfg(not(target_family = "wasm"))]
    {
        std::env::split_paths(paths).collect::<Vec<_>>().into_iter()
    }
    #[cfg(target_family = "wasm")]
    {
        let _ = paths;
        Vec::new().into_iter()
    }
}

/// `Mutex::lock`, except on a web page's main thread, which a browser never lets wait
/// (`Atomics.wait` throws there, and the throw leaves whatever gpui had borrowed borrowed): there
/// it spins until the lock is free, as Emscripten's main thread does. The holder is a Web Worker
/// on a core of its own, so the spin lasts the holder's critical section.
///
/// For the locks the screen's thread shares with the planner's threads: on the desktop, and on a
/// worker, it is `lock` itself.
///
/// # Errors
/// A poisoned lock, as `lock`'s.
pub fn lock<T: ?Sized>(mutex: &Mutex<T>) -> LockResult<MutexGuard<'_, T>> {
    #[cfg(target_family = "wasm")]
    if on_main_thread() {
        loop {
            match mutex.try_lock() {
                Ok(guard) => return Ok(guard),
                Err(TryLockError::Poisoned(poisoned)) => return Err(poisoned),
                Err(TryLockError::WouldBlock) => std::hint::spin_loop(),
            }
        }
    }
    mutex.lock()
}

/// [`lock`] as a method, so a call reads as `mutex.os_lock()` where it read `mutex.lock()`
/// (web/tools/port_locks.py puts it in place).
pub trait Lock<T: ?Sized> {
    /// [`lock`].
    ///
    /// # Errors
    /// A poisoned lock, as `lock`'s.
    fn os_lock(&self) -> LockResult<MutexGuard<'_, T>>;
}

impl<T: ?Sized> Lock<T> for Mutex<T> {
    fn os_lock(&self) -> LockResult<MutexGuard<'_, T>> {
        lock(self)
    }
}

/// `RwLock::read`, spinning on a web page's main thread as [`lock`] does there, where a wait for a
/// writer would trap; `read` itself on the desktop and on a worker.
///
/// # Errors
/// A poisoned lock, as `read`'s.
pub fn read<T: ?Sized>(rwlock: &RwLock<T>) -> LockResult<RwLockReadGuard<'_, T>> {
    #[cfg(target_family = "wasm")]
    if on_main_thread() {
        loop {
            match rwlock.try_read() {
                Ok(guard) => return Ok(guard),
                Err(TryLockError::Poisoned(poisoned)) => return Err(poisoned),
                Err(TryLockError::WouldBlock) => std::hint::spin_loop(),
            }
        }
    }
    rwlock.read()
}

/// `RwLock::write`, spinning on a web page's main thread as [`lock`] does there; `write` itself
/// on the desktop and on a worker.
///
/// # Errors
/// A poisoned lock, as `write`'s.
pub fn write<T: ?Sized>(rwlock: &RwLock<T>) -> LockResult<RwLockWriteGuard<'_, T>> {
    #[cfg(target_family = "wasm")]
    if on_main_thread() {
        loop {
            match rwlock.try_write() {
                Ok(guard) => return Ok(guard),
                Err(TryLockError::Poisoned(poisoned)) => return Err(poisoned),
                Err(TryLockError::WouldBlock) => std::hint::spin_loop(),
            }
        }
    }
    rwlock.write()
}

/// [`read`] and [`write`] as an `RwLock`'s own methods, as [`Lock`] gives a `Mutex` [`lock`]: for
/// the `RwLock`s the screen's thread shares with the planner's threads.
pub trait ReadWrite<T: ?Sized> {
    /// [`read`].
    ///
    /// # Errors
    /// A poisoned lock, as `read`'s.
    fn os_read(&self) -> LockResult<RwLockReadGuard<'_, T>>;

    /// [`write`].
    ///
    /// # Errors
    /// A poisoned lock, as `write`'s.
    fn os_write(&self) -> LockResult<RwLockWriteGuard<'_, T>>;
}

impl<T: ?Sized> ReadWrite<T> for RwLock<T> {
    fn os_read(&self) -> LockResult<RwLockReadGuard<'_, T>> {
        read(self)
    }

    fn os_write(&self) -> LockResult<RwLockWriteGuard<'_, T>> {
        write(self)
    }
}

/// `Receiver::recv_timeout`, which reads std's clock for its deadline and so panics in a web
/// page: there the channel is polled against the page's clock, a millisecond's sleep apart on a
/// Web Worker and spinning on the page's main thread, which may not sleep. On the desktop it is
/// `recv_timeout` itself (web/tools/port_locks.py puts it in place).
pub trait RecvTimeout<T> {
    /// `recv_timeout`.
    ///
    /// # Errors
    /// As `recv_timeout`: [`RecvTimeoutError::Timeout`] when nothing came in time,
    /// [`RecvTimeoutError::Disconnected`] when nothing can come.
    fn os_recv_timeout(&self, timeout: Duration) -> Result<T, RecvTimeoutError>;
}

impl<T> RecvTimeout<T> for Receiver<T> {
    fn os_recv_timeout(&self, timeout: Duration) -> Result<T, RecvTimeoutError> {
        #[cfg(not(target_family = "wasm"))]
        {
            self.recv_timeout(timeout)
        }
        #[cfg(target_family = "wasm")]
        {
            let deadline = web_time::Instant::now() + timeout;
            loop {
                match self.try_recv() {
                    Ok(value) => return Ok(value),
                    Err(TryRecvError::Disconnected) => return Err(RecvTimeoutError::Disconnected),
                    Err(TryRecvError::Empty) => {}
                }
                if web_time::Instant::now() >= deadline {
                    return Err(RecvTimeoutError::Timeout);
                }
                if on_main_thread() {
                    std::hint::spin_loop();
                } else {
                    wasm_thread::sleep(Duration::from_millis(1));
                }
            }
        }
    }
}

/// Whether this is the page's main thread rather than a Web Worker, asked of the browser once a
/// thread.
#[cfg(target_family = "wasm")]
fn on_main_thread() -> bool {
    thread_local! {
        static MAIN: bool = !wasm_thread::is_web_worker_thread();
    }
    MAIN.with(|main| *main)
}

/// The script beside the page that the planner's threads are made from (web/www/thread-worker.js).
pub const THREAD_WORKER: &str = "thread-worker.js";

/// The page's threads made from [`THREAD_WORKER`], a script of the site's own, rather than from
/// the `blob:` address wasm_thread makes one up for (the owner's bug of 2026-10-06: OpenStreetMap's
/// tiles refused in the page, "403 Access blocked"). A worker's requests carry its script's address
/// as their `Referer`, and the Fetch standard strips a `blob:` address to none ("strip url for use
/// as a referrer": a local scheme gives no referrer); OpenStreetMap's tile servers refuse a request
/// with no `Referer` (https://osm.wiki/Blocked), and the tiles are fetched on these threads. Made
/// from the site's script, they send the page's origin, as the browser's referrer policy cuts it.
/// Every thread here - gpui's, the tile store's, the planner's own - is made by
/// `wasm_thread::Builder::new()`, which starts from this default; so first thing, before any is.
#[cfg(target_family = "wasm")]
pub fn threads_from_the_site() {
    let Some(page) = js_sys::Reflect::get(&js_sys::global(), &"location".into())
        .and_then(|location| js_sys::Reflect::get(&location, &"href".into()))
        .ok()
        .and_then(|href| href.as_string())
    else {
        return;
    };
    wasm_thread::Builder::empty()
        .worker_script_url(beside_page(&page, THREAD_WORKER))
        .set_default();
}

/// The address of `file` in the page's folder: the page's address without its query, fragment or
/// last segment, then `file`.
#[must_use]
pub fn beside_page(page: &str, file: &str) -> String {
    let page = page.split(['?', '#']).next().unwrap_or(page);
    let folder = page
        .rfind('/')
        .and_then(|slash| page.get(..=slash))
        .unwrap_or(page);
    format!("{folder}{file}")
}

/// An HTTP request in a web page, blocking: a synchronous `XMLHttpRequest`, which a browser
/// allows in a Web Worker - the threads the planner fetches tiles, terrain and catalogues on (on
/// the page's main thread it refuses a binary answer). The status and the body; a status that is
/// not a success is the caller's to judge, as ureq's `http_status_as_error(false)` leaves it.
///
/// The browser's rules apply: the server must let the page's origin read the answer (CORS), and
/// `User-Agent` and `Referer` are the browser's own, so a caller's are not sent.
///
/// # Errors
/// The browser's refusal - a network failure, a server that does not allow the origin - in its
/// own words.
#[cfg(target_family = "wasm")]
pub fn http(method: &str, url: &str, body: Option<&str>) -> Result<(u16, Vec<u8>), String> {
    // A page that came over https may not ask over plain http (the browser refuses it as mixed
    // content), so there the https address is the only one.
    let protocol = js_sys::Reflect::get(&js_sys::global(), &"location".into())
        .and_then(|location| js_sys::Reflect::get(&location, &"protocol".into()))
        .ok()
        .and_then(|protocol| protocol.as_string())
        .unwrap_or_default();
    let addresses = https_first(url);
    if protocol == "https:"
        && let Some(secure) = addresses.first()
    {
        return http_once(method, secure, body);
    }
    // A status is an answer (`Ok`); an `Err` is none.
    ask_https_first(url, |address| http_once(method, address, body), |_| false)
}

/// One request of [`http`]'s, to one address.
#[cfg(target_family = "wasm")]
fn http_once(method: &str, url: &str, body: Option<&str>) -> Result<(u16, Vec<u8>), String> {
    let text = |err: wasm_bindgen::JsValue| format!("{url}: {err:?}");
    let request = web_sys::XmlHttpRequest::new().map_err(text)?;
    request.open_with_async(method, url, false).map_err(text)?;
    request.set_response_type(web_sys::XmlHttpRequestResponseType::Arraybuffer);
    match body {
        Some(body) => request.send_with_opt_str(Some(body)).map_err(text)?,
        None => request.send().map_err(text)?,
    }
    let status = request.status().map_err(text)?;
    if status == 0 {
        // What a browser reports for a request it would not let the page see (CORS) or that
        // never reached the server.
        return Err(format!("{url}: no answer the page may read"));
    }
    let answer = request.response().map_err(text)?;
    Ok((status, js_sys::Uint8Array::new(&answer).to_vec()))
}

/// The addresses to ask for `url`, in turn, until one answers: over https first, and for an
/// address written `http://` the plain one after it (the owner, 2026-10-05: "it should all start
/// with trying https, and only then if it doesnt work try http"). Many of the planner's addresses
/// are Mission Planner's own, written `http://` - Google's version page among them, which a page
/// on GitHub Pages could not ask at all, so its satellite tiles kept a version Google no longer
/// serves and no map drew. "Answers" is any answer, an error status too: a server that answers
/// over https has answered, and only no answer at all - no connection, a TLS failure, a time out
/// - goes on to plain http. An https address is asked as written, and only so.
#[must_use]
pub fn https_first(url: &str) -> Vec<std::borrow::Cow<'_, str>> {
    match url.strip_prefix("http://") {
        Some(rest) => vec![format!("https://{rest}").into(), url.into()],
        None => vec![url.into()],
    }
}

/// Asks `ask` each of [`https_first`]'s addresses for `url` in turn, until one answers: the first
/// answer, else the plain address's failure, as the address written would have failed.
/// `answered` tells an error that is an answer - an error status - from no answer at all.
///
/// # Errors
/// The last address's error, when none answered.
pub fn ask_https_first<T, E>(
    url: &str,
    mut ask: impl FnMut(&str) -> Result<T, E>,
    answered: impl Fn(&E) -> bool,
) -> Result<T, E> {
    let addresses = https_first(url);
    let Some((last, first)) = addresses.split_last() else {
        return ask(url);
    };
    for address in first {
        match ask(address) {
            Err(error) if !answered(&error) => {}
            result => return result,
        }
    }
    ask(last)
}

/// The value the web page's address gives `name` after its `?` (`?demo=0`), from
/// the page's main thread; none on the desktop, which has no page, and none for a name the
/// address does not carry. The value as written, not percent-decoded.
#[must_use]
pub fn page_query(name: &str) -> Option<String> {
    #[cfg(target_family = "wasm")]
    {
        let location = js_sys::Reflect::get(&js_sys::global(), &"location".into()).ok()?;
        let search = js_sys::Reflect::get(&location, &"search".into())
            .ok()?
            .as_string()?;
        query_value(&search, name)
    }
    #[cfg(not(target_family = "wasm"))]
    {
        let _ = name;
        None
    }
}

/// `name`'s value in an address's query (`?a=1&b`), a name without `=` giving an empty one.
#[cfg_attr(not(target_family = "wasm"), allow(dead_code))]
fn query_value(search: &str, name: &str) -> Option<String> {
    search.trim_start_matches('?').split('&').find_map(|pair| {
        let (key, value) = pair.split_once('=').unwrap_or((pair, ""));
        (key == name).then(|| value.to_owned())
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The threads' script is beside the page wherever the page is: at a server's root, under
    /// GitHub Pages' project path, with a query, a fragment, or a file name of its own.
    #[test]
    fn the_thread_script_is_beside_the_page() {
        let at = |page| beside_page(page, THREAD_WORKER);
        assert_eq!(
            at("http://127.0.0.1:8080/?facts=1&demo=0"),
            "http://127.0.0.1:8080/thread-worker.js"
        );
        assert_eq!(
            at("https://davidbuzz.github.io/MissionPlannerRust/?vehicle=copter#map"),
            "https://davidbuzz.github.io/MissionPlannerRust/thread-worker.js"
        );
        assert_eq!(
            at("https://example.org/planner/index.html?a=/b/c"),
            "https://example.org/planner/thread-worker.js"
        );
        assert_eq!(
            at("https://example.org/planner/index.html#/x/y"),
            "https://example.org/planner/thread-worker.js"
        );
    }

    #[test]
    fn https_is_asked_first_and_plain_http_after_it() {
        let google = "http://maps.google.com/maps/api/js?v=3.2&sensor=false";
        assert_eq!(
            https_first(google),
            [
                "https://maps.google.com/maps/api/js?v=3.2&sensor=false",
                google
            ]
        );
        // An https address is asked as written, and only so.
        let tile = "https://khms1.google.com/kh/v=1015";
        assert_eq!(https_first(tile), [tile]);
    }

    #[test]
    fn plain_http_is_asked_only_when_https_has_no_answer() {
        let url = "http://firmware.ardupilot.org/manifest.json.gz";
        let asked = |https: Result<u16, &'static str>| {
            let mut asked = Vec::new();
            let result = ask_https_first(
                url,
                |address| {
                    asked.push(address.to_owned());
                    if address.starts_with("https://") {
                        https
                    } else {
                        Ok(200)
                    }
                },
                |error| *error == "status 404",
            );
            (result, asked)
        };
        // https answers: asked once.
        let (result, asked_at) = asked(Ok(200));
        assert_eq!(result, Ok(200));
        assert_eq!(asked_at, ["https://firmware.ardupilot.org/manifest.json.gz"]);
        // https answers with an error status: that is its answer.
        let (result, asked_at) = asked(Err("status 404"));
        assert_eq!(result, Err("status 404"));
        assert_eq!(asked_at.len(), 1);
        // https does not answer: then plain http, as written.
        let (result, asked_at) = asked(Err("connection refused"));
        assert_eq!(result, Ok(200));
        assert_eq!(asked_at, [
            "https://firmware.ardupilot.org/manifest.json.gz",
            url
        ]);
    }

    #[test]
    fn on_the_desktop_os_recv_timeout_is_recv_timeout() {
        let (send, receive) = std::sync::mpsc::channel();
        send.send(7).expect("open");
        assert_eq!(receive.os_recv_timeout(Duration::from_millis(10)), Ok(7));
        assert_eq!(
            receive.os_recv_timeout(Duration::from_millis(10)),
            Err(RecvTimeoutError::Timeout)
        );
        drop(send);
        assert_eq!(
            receive.os_recv_timeout(Duration::from_millis(10)),
            Err(RecvTimeoutError::Disconnected)
        );
    }

    #[test]
    fn on_the_desktop_lock_is_mutex_lock() {
        let mutex = Mutex::new(1);
        *lock(&mutex).expect("not poisoned") += 1;
        assert_eq!(*mutex.lock().expect("not poisoned"), 2);
        let poisoned = std::sync::Arc::new(Mutex::new(0));
        let held = std::sync::Arc::clone(&poisoned);
        let _ = wasm_thread::spawn(move || {
            let _guard = held.lock();
            panic!("poison it");
        })
        .join();
        assert!(lock(&poisoned).is_err());
    }

    #[test]
    fn on_the_desktop_a_thread_let_go_of_is_joined() {
        let handle = wasm_thread::spawn(|| 7);
        assert_eq!(join_or_leave(handle).map(Result::ok), Some(Some(7)));
        let panicked = wasm_thread::spawn(|| panic!("its result is the panic"));
        assert!(join_or_leave(panicked).is_some_and(|result| result.is_err()));
    }

    #[test]
    fn on_the_desktop_read_and_write_are_rwlocks_own() {
        let rwlock = RwLock::new(1);
        *rwlock.os_write().expect("not poisoned") += 1;
        assert_eq!(*rwlock.os_read().expect("not poisoned"), 2);
        // Readers together, as RwLock's own.
        let first = rwlock.os_read().expect("not poisoned");
        let second = rwlock.os_read().expect("not poisoned");
        assert_eq!(*first + *second, 4);
        drop((first, second));
        let poisoned = std::sync::Arc::new(RwLock::new(0));
        let held = std::sync::Arc::clone(&poisoned);
        let _ = wasm_thread::spawn(move || {
            let _guard = held.write();
            panic!("poison it");
        })
        .join();
        assert!(poisoned.os_read().is_err());
        assert!(poisoned.os_write().is_err());
    }

    #[test]
    fn a_query_gives_each_name_its_value() {
        assert_eq!(query_value("?facts=1&demo=0", "demo").as_deref(), Some("0"));
        assert_eq!(
            query_value("?facts=1&demo=0", "facts").as_deref(),
            Some("1")
        );
        assert_eq!(query_value("?facts&demo=0", "facts").as_deref(), Some(""));
        assert_eq!(query_value("?facts=1", "demo"), None);
        assert_eq!(query_value("", "demo"), None);
        // On the desktop there is no page.
        assert_eq!(page_query("demo"), None);
    }

    #[test]
    fn on_the_desktop_each_is_std_s_own() {
        assert_eq!(temp_dir(), std::env::temp_dir());
        assert_eq!(process_id(), std::process::id());
        let joined = std::env::join_paths(["/a", "/b/c"]).expect("joinable");
        assert_eq!(
            split_paths(&joined).collect::<Vec<_>>(),
            std::env::split_paths(&joined).collect::<Vec<_>>()
        );
    }
}
