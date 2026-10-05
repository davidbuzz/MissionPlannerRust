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

//! The files of a web page: `std::fs`'s calls over a store held in memory, which the page keeps
//! in the browser's own storage between visits ([`Store::preload`], [`Store::take_changes`]).
//!
//! `std::fs` on `wasm32-unknown-unknown` answers every path with "operation not supported on this
//! platform": a page has no file system. The planner's settings, missions and logs are files, so
//! in a page they were lost - config.xml was never written, and everything a visit chose was gone
//! at the next (the owner's request, 2026-10-05: settings, missions and logs to survive a reload).
//! This is the subset of `std::fs` the planner calls, with std's names and std's errors, over
//! paths in memory. Compiled everywhere so its tests run on the desktop; only a page uses it
//! ([`super`]).
//!
//! Paths are kept absolute and without `.` or `..`; a relative one is taken from `/`, as a page
//! has no working directory. A file's folders exist once it does: writing `a/b/c` makes `a` and
//! `a/b`, which `std::fs::write` would refuse - a page starts with no folders at all, and the
//! planner makes the ones it needs at start-up on the desktop only where they are missing.

use std::collections::{BTreeMap, BTreeSet};
use std::ffi::OsString;
use std::io::{self, Read, Seek, SeekFrom, Write};
use std::path::{Component, Path, PathBuf};
use std::sync::Mutex;

use web_time::SystemTime;

/// One file's bytes and when they last changed.
#[derive(Debug)]
struct Node {
    data: Vec<u8>,
    modified: SystemTime,
}

/// What has changed in a file since the browser's copy was last brought up to date.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Dirty {
    /// Its bytes from this offset to its end; those before it are as the browser has them.
    From(u64),
    /// It, or the folder, is gone.
    Removed,
}

#[derive(Debug, Default)]
struct Inner {
    files: BTreeMap<PathBuf, Node>,
    dirs: BTreeSet<PathBuf>,
    /// Each file's length as the browser last had it.
    saved: BTreeMap<PathBuf, u64>,
    dirty: BTreeMap<PathBuf, Dirty>,
}

/// A change for the browser's copy, as [`Store::take_changes`] hands them over, in order.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Change {
    /// `data` written at `offset` of the file at `path`, which is then `len` bytes long.
    Write {
        /// The file.
        path: PathBuf,
        /// Where `data` goes.
        offset: u64,
        /// The bytes from `offset` to the file's end.
        data: Vec<u8>,
        /// The file's length.
        len: u64,
    },
    /// The file or folder at `path`, and all under it, removed.
    Remove {
        /// What went.
        path: PathBuf,
    },
}

/// A store of files in memory.
#[derive(Debug, Default)]
pub struct Store(Mutex<Inner>);

/// The page's files.
pub static STORE: Store = Store::new();

/// `path` as the store keys it: absolute, without `.` or `..`.
#[must_use]
pub fn normalize(path: &Path) -> PathBuf {
    let mut out = PathBuf::from("/");
    for component in path.components() {
        match component {
            Component::Normal(part) => out.push(part),
            Component::ParentDir => {
                out.pop();
            }
            Component::RootDir | Component::CurDir | Component::Prefix(_) => {}
        }
    }
    out
}

fn not_found(path: &Path) -> io::Error {
    io::Error::new(
        io::ErrorKind::NotFound,
        format!("{}: no such file or directory", path.display()),
    )
}

fn is_a_directory(path: &Path) -> io::Error {
    io::Error::new(
        io::ErrorKind::IsADirectory,
        format!("{}: is a directory", path.display()),
    )
}

fn now() -> SystemTime {
    SystemTime::now()
}

/// Folders the browser does not keep: the map's tile cache (`gmapcache`), which grows with every
/// place looked at, would be loaded whole at every visit - the browser's own cache keeps the
/// tiles' downloads. What the owner asked kept is settings, missions and logs (2026-10-05).
pub const NOT_KEPT: &[&str] = &["gmapcache"];

/// Whether the browser keeps the file or folder at `path`.
#[must_use]
pub fn kept(path: &Path) -> bool {
    !path
        .components()
        .any(|part| NOT_KEPT.iter().any(|name| part.as_os_str() == *name))
}

impl Inner {
    fn is_dir(&self, path: &Path) -> bool {
        path == Path::new("/") || self.dirs.contains(path)
    }

    /// `path`'s folders, made.
    fn make_parents(&mut self, path: &Path) {
        let mut parent = path.parent();
        while let Some(dir) = parent {
            if dir == Path::new("/") || !self.dirs.insert(dir.to_path_buf()) {
                break;
            }
            parent = dir.parent();
        }
    }

    /// The file at `path` changed from `offset` on.
    fn touched(&mut self, path: &Path, offset: u64) {
        if !kept(path) {
            return;
        }
        let saved = self.saved.get(path).copied().unwrap_or(0);
        let from = offset.min(saved);
        let from = match self.dirty.get(path) {
            Some(Dirty::From(earlier)) => from.min(*earlier),
            // Gone and back: the browser's copy is gone too, so all of it.
            Some(Dirty::Removed) => 0,
            None => from,
        };
        self.dirty.insert(path.to_path_buf(), Dirty::From(from));
    }

    /// The file or folder at `path` gone, and everything under it.
    fn removed(&mut self, path: &Path) {
        if !kept(path) {
            return;
        }
        self.saved.retain(|held, _| !held.starts_with(path));
        self.dirty.retain(|held, _| !held.starts_with(path));
        self.dirty.insert(path.to_path_buf(), Dirty::Removed);
    }

    fn write_at(&mut self, path: &Path, offset: u64, bytes: &[u8]) -> io::Result<()> {
        let node = self.files.get_mut(path).ok_or_else(|| not_found(path))?;
        let start = usize::try_from(offset).map_err(io::Error::other)?;
        let end = start + bytes.len();
        if node.data.len() < end {
            node.data.resize(end, 0);
        }
        if let Some(target) = node.data.get_mut(start..end) {
            target.copy_from_slice(bytes);
        }
        node.modified = now();
        self.touched(path, offset);
        Ok(())
    }

    fn set_len(&mut self, path: &Path, len: u64) -> io::Result<()> {
        let node = self.files.get_mut(path).ok_or_else(|| not_found(path))?;
        node.data
            .resize(usize::try_from(len).map_err(io::Error::other)?, 0);
        node.modified = now();
        self.touched(path, len);
        Ok(())
    }
}

impl Store {
    /// An empty store.
    #[must_use]
    pub const fn new() -> Self {
        Self(Mutex::new(Inner {
            files: BTreeMap::new(),
            dirs: BTreeSet::new(),
            saved: BTreeMap::new(),
            dirty: BTreeMap::new(),
        }))
    }

    fn with<T>(&self, f: impl FnOnce(&mut Inner) -> T) -> T {
        match crate::lock(&self.0) {
            Ok(mut inner) => f(&mut inner),
            Err(poisoned) => f(&mut poisoned.into_inner()),
        }
    }

    /// The browser's copy, as a visit starts: files the store holds as the browser has them, so
    /// nothing of them is handed back by [`Store::take_changes`].
    pub fn preload(&self, files: impl IntoIterator<Item = (PathBuf, Vec<u8>)>) {
        self.with(|inner| {
            for (path, data) in files {
                let path = normalize(&path);
                inner.make_parents(&path);
                inner.saved.insert(path.clone(), data.len() as u64);
                inner.files.insert(
                    path,
                    Node {
                        data,
                        modified: now(),
                    },
                );
            }
        });
    }

    /// What has changed since the last call, for the browser's copy: each file written from
    /// where it first differs, each removal; the store then counts the browser's copy as
    /// matching it.
    pub fn take_changes(&self) -> Vec<Change> {
        self.with(|inner| {
            let dirty = std::mem::take(&mut inner.dirty);
            let mut changes = Vec::with_capacity(dirty.len());
            // Removals first, so a folder removed and a file written in it again since land in
            // that order.
            for (path, _) in dirty.iter().filter(|(_, dirty)| **dirty == Dirty::Removed) {
                changes.push(Change::Remove { path: path.clone() });
            }
            for (path, dirty) in dirty {
                let Dirty::From(offset) = dirty else {
                    continue;
                };
                let Some(node) = inner.files.get(&path) else {
                    continue;
                };
                let len = node.data.len() as u64;
                let start = usize::try_from(offset.min(len)).unwrap_or(node.data.len());
                let data = node.data.get(start..).unwrap_or_default().to_vec();
                inner.saved.insert(path.clone(), len);
                changes.push(Change::Write {
                    path,
                    offset: start as u64,
                    data,
                    len,
                });
            }
            changes
        })
    }

    /// `std::fs::read`.
    ///
    /// # Errors
    /// No file there, or a folder.
    pub fn read(&self, path: &Path) -> io::Result<Vec<u8>> {
        let path = normalize(path);
        self.with(|inner| match inner.files.get(&path) {
            Some(node) => Ok(node.data.clone()),
            None if inner.is_dir(&path) => Err(is_a_directory(&path)),
            None => Err(not_found(&path)),
        })
    }

    /// `std::fs::write`: the file replaced, its folders made.
    ///
    /// # Errors
    /// A folder there.
    pub fn write(&self, path: &Path, contents: &[u8]) -> io::Result<()> {
        let path = normalize(path);
        self.with(|inner| {
            if inner.is_dir(&path) {
                return Err(is_a_directory(&path));
            }
            inner.make_parents(&path);
            inner.files.insert(
                path.clone(),
                Node {
                    data: contents.to_vec(),
                    modified: now(),
                },
            );
            inner.touched(&path, 0);
            Ok(())
        })
    }

    /// `std::fs::create_dir_all`.
    ///
    /// # Errors
    /// A file where a folder would be.
    pub fn create_dir_all(&self, path: &Path) -> io::Result<()> {
        let path = normalize(path);
        self.with(|inner| {
            let mut dir = Some(path.as_path());
            while let Some(at) = dir {
                if inner.files.contains_key(at) {
                    return Err(io::Error::new(
                        io::ErrorKind::AlreadyExists,
                        format!("{}: a file, not a directory", at.display()),
                    ));
                }
                dir = at.parent();
            }
            inner.make_parents(&path.join("_"));
            Ok(())
        })
    }

    /// `std::fs::create_dir`.
    ///
    /// # Errors
    /// Something there already.
    pub fn create_dir(&self, path: &Path) -> io::Result<()> {
        let normal = normalize(path);
        let exists = self.with(|inner| inner.is_dir(&normal) || inner.files.contains_key(&normal));
        if exists {
            return Err(io::Error::new(
                io::ErrorKind::AlreadyExists,
                format!("{}: already exists", normal.display()),
            ));
        }
        self.create_dir_all(&normal)
    }

    /// `std::fs::remove_file`.
    ///
    /// # Errors
    /// No file there.
    pub fn remove_file(&self, path: &Path) -> io::Result<()> {
        let path = normalize(path);
        self.with(|inner| {
            if inner.files.remove(&path).is_none() {
                return Err(if inner.is_dir(&path) {
                    is_a_directory(&path)
                } else {
                    not_found(&path)
                });
            }
            inner.removed(&path);
            Ok(())
        })
    }

    /// `std::fs::remove_dir_all`.
    ///
    /// # Errors
    /// No folder there.
    pub fn remove_dir_all(&self, path: &Path) -> io::Result<()> {
        let path = normalize(path);
        self.with(|inner| {
            if !inner.is_dir(&path) {
                return Err(if inner.files.contains_key(&path) {
                    io::Error::new(
                        io::ErrorKind::NotADirectory,
                        format!("{}: not a directory", path.display()),
                    )
                } else {
                    not_found(&path)
                });
            }
            inner.files.retain(|held, _| !held.starts_with(&path));
            inner.dirs.retain(|held| !held.starts_with(&path));
            inner.removed(&path);
            Ok(())
        })
    }

    /// `std::fs::remove_dir`: an empty folder.
    ///
    /// # Errors
    /// No folder there, or one with something in it.
    pub fn remove_dir(&self, path: &Path) -> io::Result<()> {
        let normal = normalize(path);
        if !self.read_dir(&normal)?.is_empty() {
            return Err(io::Error::new(
                io::ErrorKind::DirectoryNotEmpty,
                format!("{}: directory not empty", normal.display()),
            ));
        }
        self.remove_dir_all(&normal)
    }

    /// `std::fs::read_dir`'s entries: the folder's files and folders, by name.
    ///
    /// # Errors
    /// No folder there.
    pub fn read_dir(&self, path: &Path) -> io::Result<Vec<DirEntry>> {
        let path = normalize(path);
        self.with(|inner| {
            if !inner.is_dir(&path) {
                return Err(not_found(&path));
            }
            let child = |held: &Path| held.parent() == Some(path.as_path());
            let dirs = inner
                .dirs
                .iter()
                .filter(|held| child(held))
                .map(|held| DirEntry {
                    path: held.clone(),
                    metadata: Metadata::dir(),
                });
            let files = inner
                .files
                .iter()
                .filter(|(held, _)| child(held))
                .map(|(held, node)| DirEntry {
                    path: held.clone(),
                    metadata: Metadata::file(node),
                });
            let mut entries: Vec<DirEntry> = dirs.chain(files).collect();
            entries.sort_by(|a, b| a.path.cmp(&b.path));
            Ok(entries)
        })
    }

    /// `std::fs::metadata`.
    ///
    /// # Errors
    /// Nothing there.
    pub fn metadata(&self, path: &Path) -> io::Result<Metadata> {
        let path = normalize(path);
        self.with(|inner| match inner.files.get(&path) {
            Some(node) => Ok(Metadata::file(node)),
            None if inner.is_dir(&path) => Ok(Metadata::dir()),
            None => Err(not_found(&path)),
        })
    }

    /// `std::fs::rename`: a file, or a folder and all in it.
    ///
    /// # Errors
    /// Nothing at `from`.
    pub fn rename(&self, from: &Path, to: &Path) -> io::Result<()> {
        let (from, to) = (normalize(from), normalize(to));
        self.with(|inner| {
            if let Some(node) = inner.files.remove(&from) {
                inner.make_parents(&to);
                inner.files.insert(to.clone(), node);
                inner.removed(&from);
                inner.touched(&to, 0);
                return Ok(());
            }
            if !inner.is_dir(&from) {
                return Err(not_found(&from));
            }
            let moved = |held: &Path| {
                held.strip_prefix(&from)
                    .map(|rest| to.join(rest))
                    .unwrap_or_else(|_| held.to_path_buf())
            };
            let files: Vec<PathBuf> = inner
                .files
                .keys()
                .filter(|held| held.starts_with(&from))
                .cloned()
                .collect();
            let dirs: Vec<PathBuf> = inner
                .dirs
                .iter()
                .filter(|held| held.starts_with(&from))
                .cloned()
                .collect();
            inner.removed(&from);
            for dir in dirs {
                inner.dirs.remove(&dir);
                inner.dirs.insert(moved(&dir));
            }
            inner.make_parents(&to);
            for file in files {
                if let Some(node) = inner.files.remove(&file) {
                    let target = moved(&file);
                    inner.files.insert(target.clone(), node);
                    inner.touched(&target, 0);
                }
            }
            Ok(())
        })
    }

    /// `std::fs::copy`: the bytes copied.
    ///
    /// # Errors
    /// No file at `from`.
    pub fn copy(&self, from: &Path, to: &Path) -> io::Result<u64> {
        let data = self.read(from)?;
        self.write(to, &data)?;
        Ok(data.len() as u64)
    }

    /// `std::fs::canonicalize`: the path as the store keys it, if something is there.
    ///
    /// # Errors
    /// Nothing there.
    pub fn canonicalize(&self, path: &Path) -> io::Result<PathBuf> {
        self.metadata(path).map(|_| normalize(path))
    }

    /// Opens `path` as `options` say.
    fn open(&'static self, path: &Path, options: &OpenOptions) -> io::Result<File> {
        let path = normalize(path);
        self.with(|inner| {
            let exists = inner.files.contains_key(&path);
            if inner.is_dir(&path) {
                return Err(is_a_directory(&path));
            }
            if exists && options.create_new {
                return Err(io::Error::new(
                    io::ErrorKind::AlreadyExists,
                    format!("{}: already exists", path.display()),
                ));
            }
            if !exists {
                if !(options.create || options.create_new) {
                    return Err(not_found(&path));
                }
                inner.make_parents(&path);
                inner.files.insert(
                    path.clone(),
                    Node {
                        data: Vec::new(),
                        modified: now(),
                    },
                );
                inner.touched(&path, 0);
            } else if options.truncate {
                inner.set_len(&path, 0)?;
            }
            Ok(())
        })?;
        Ok(File {
            store: self,
            path,
            position: 0,
            append: options.append,
        })
    }
}

/// `changes` packed for the page's storage.js: per change a kind byte (1 a write, 2 a removal),
/// the path's length and UTF-8 bytes, and for a write the offset, the file's length and the
/// data's length and bytes - each number a little-endian u32, which a page's files stay under.
#[must_use]
pub fn encode(changes: &[Change]) -> Vec<u8> {
    let mut out = Vec::new();
    let number = |out: &mut Vec<u8>, value: u64| {
        out.extend_from_slice(&u32::try_from(value).unwrap_or(u32::MAX).to_le_bytes());
    };
    for change in changes {
        let (kind, path) = match change {
            Change::Write { path, .. } => (1_u8, path),
            Change::Remove { path } => (2_u8, path),
        };
        let path = path.to_string_lossy();
        out.push(kind);
        number(&mut out, path.len() as u64);
        out.extend_from_slice(path.as_bytes());
        if let Change::Write {
            offset, data, len, ..
        } = change
        {
            number(&mut out, *offset);
            number(&mut out, *len);
            number(&mut out, data.len() as u64);
            out.extend_from_slice(data);
        }
    }
    out
}

// ---- std's free functions, over the page's store ----

/// `std::fs::read`.
///
/// # Errors
/// As [`Store::read`].
pub fn read(path: impl AsRef<Path>) -> io::Result<Vec<u8>> {
    STORE.read(path.as_ref())
}

/// `std::fs::read_to_string`.
///
/// # Errors
/// As [`Store::read`], or the bytes not UTF-8.
pub fn read_to_string(path: impl AsRef<Path>) -> io::Result<String> {
    String::from_utf8(read(path)?).map_err(|err| io::Error::new(io::ErrorKind::InvalidData, err))
}

/// `std::fs::write`.
///
/// # Errors
/// As [`Store::write`].
pub fn write(path: impl AsRef<Path>, contents: impl AsRef<[u8]>) -> io::Result<()> {
    STORE.write(path.as_ref(), contents.as_ref())
}

/// `std::fs::create_dir`.
///
/// # Errors
/// As [`Store::create_dir`].
pub fn create_dir(path: impl AsRef<Path>) -> io::Result<()> {
    STORE.create_dir(path.as_ref())
}

/// `std::fs::create_dir_all`.
///
/// # Errors
/// As [`Store::create_dir_all`].
pub fn create_dir_all(path: impl AsRef<Path>) -> io::Result<()> {
    STORE.create_dir_all(path.as_ref())
}

/// `std::fs::remove_file`.
///
/// # Errors
/// As [`Store::remove_file`].
pub fn remove_file(path: impl AsRef<Path>) -> io::Result<()> {
    STORE.remove_file(path.as_ref())
}

/// `std::fs::remove_dir`.
///
/// # Errors
/// As [`Store::remove_dir`].
pub fn remove_dir(path: impl AsRef<Path>) -> io::Result<()> {
    STORE.remove_dir(path.as_ref())
}

/// `std::fs::remove_dir_all`.
///
/// # Errors
/// As [`Store::remove_dir_all`].
pub fn remove_dir_all(path: impl AsRef<Path>) -> io::Result<()> {
    STORE.remove_dir_all(path.as_ref())
}

/// `std::fs::read_dir`.
///
/// # Errors
/// As [`Store::read_dir`].
pub fn read_dir(path: impl AsRef<Path>) -> io::Result<ReadDir> {
    STORE
        .read_dir(path.as_ref())
        .map(|entries| ReadDir(entries.into_iter()))
}

/// `std::fs::metadata`.
///
/// # Errors
/// As [`Store::metadata`].
pub fn metadata(path: impl AsRef<Path>) -> io::Result<Metadata> {
    STORE.metadata(path.as_ref())
}

/// `std::fs::symlink_metadata`: [`metadata`], a page's files having no links.
///
/// # Errors
/// As [`Store::metadata`].
pub fn symlink_metadata(path: impl AsRef<Path>) -> io::Result<Metadata> {
    metadata(path)
}

/// `std::fs::exists`.
///
/// # Errors
/// Never; std's can.
pub fn exists(path: impl AsRef<Path>) -> io::Result<bool> {
    Ok(metadata(path).is_ok())
}

/// `std::fs::copy`.
///
/// # Errors
/// As [`Store::copy`].
pub fn copy(from: impl AsRef<Path>, to: impl AsRef<Path>) -> io::Result<u64> {
    STORE.copy(from.as_ref(), to.as_ref())
}

/// `std::fs::rename`.
///
/// # Errors
/// As [`Store::rename`].
pub fn rename(from: impl AsRef<Path>, to: impl AsRef<Path>) -> io::Result<()> {
    STORE.rename(from.as_ref(), to.as_ref())
}

/// `std::fs::canonicalize`.
///
/// # Errors
/// As [`Store::canonicalize`].
pub fn canonicalize(path: impl AsRef<Path>) -> io::Result<PathBuf> {
    STORE.canonicalize(path.as_ref())
}

/// `std::fs::set_permissions`: of no effect, if something is there.
///
/// # Errors
/// Nothing there.
pub fn set_permissions(path: impl AsRef<Path>, _permissions: Permissions) -> io::Result<()> {
    metadata(path).map(|_| ())
}

/// `std::fs::ReadDir`: a folder's entries.
#[derive(Debug)]
pub struct ReadDir(std::vec::IntoIter<DirEntry>);

impl Iterator for ReadDir {
    type Item = io::Result<DirEntry>;

    fn next(&mut self) -> Option<Self::Item> {
        self.0.next().map(Ok)
    }
}

/// `std::fs::Metadata`.
#[derive(Debug, Clone, Copy)]
pub struct Metadata {
    dir: bool,
    len: u64,
    modified: SystemTime,
}

impl Metadata {
    fn file(node: &Node) -> Self {
        Self {
            dir: false,
            len: node.data.len() as u64,
            modified: node.modified,
        }
    }

    fn dir() -> Self {
        Self {
            dir: true,
            len: 0,
            modified: now(),
        }
    }

    /// Whether it is a file.
    #[must_use]
    pub const fn is_file(&self) -> bool {
        !self.dir
    }

    /// Whether it is a folder.
    #[must_use]
    pub const fn is_dir(&self) -> bool {
        self.dir
    }

    /// Never: a page's files have no links.
    #[must_use]
    pub const fn is_symlink(&self) -> bool {
        false
    }

    /// Its length in bytes.
    #[must_use]
    #[allow(clippy::len_without_is_empty)] // std's Metadata has none either
    pub const fn len(&self) -> u64 {
        self.len
    }

    /// When it last changed.
    ///
    /// # Errors
    /// Never; std's can.
    pub const fn modified(&self) -> io::Result<SystemTime> {
        Ok(self.modified)
    }

    /// When it was made: as kept here, when it last changed.
    ///
    /// # Errors
    /// Never; std's can.
    pub const fn created(&self) -> io::Result<SystemTime> {
        Ok(self.modified)
    }

    /// When it was last read: as kept here, when it last changed.
    ///
    /// # Errors
    /// Never; std's can.
    pub const fn accessed(&self) -> io::Result<SystemTime> {
        Ok(self.modified)
    }

    /// Its kind.
    #[must_use]
    pub const fn file_type(&self) -> FileType {
        FileType { dir: self.dir }
    }

    /// Its permissions: a page's files are all writable.
    #[must_use]
    pub const fn permissions(&self) -> Permissions {
        Permissions { readonly: false }
    }
}

/// `std::fs::FileType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FileType {
    dir: bool,
}

impl FileType {
    /// Whether it is a file.
    #[must_use]
    pub const fn is_file(&self) -> bool {
        !self.dir
    }

    /// Whether it is a folder.
    #[must_use]
    pub const fn is_dir(&self) -> bool {
        self.dir
    }

    /// Never: a page's files have no links.
    #[must_use]
    pub const fn is_symlink(&self) -> bool {
        false
    }
}

/// `std::fs::Permissions`: only whether read-only, which a page's files never are.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Permissions {
    readonly: bool,
}

impl Permissions {
    /// Whether read-only.
    #[must_use]
    pub const fn readonly(&self) -> bool {
        self.readonly
    }

    /// Kept, and of no effect.
    pub const fn set_readonly(&mut self, readonly: bool) {
        self.readonly = readonly;
    }
}

/// `std::fs::DirEntry`.
#[derive(Debug, Clone)]
pub struct DirEntry {
    path: PathBuf,
    metadata: Metadata,
}

impl DirEntry {
    /// Its path.
    #[must_use]
    pub fn path(&self) -> PathBuf {
        self.path.clone()
    }

    /// Its name.
    #[must_use]
    pub fn file_name(&self) -> OsString {
        self.path
            .file_name()
            .map(ToOwned::to_owned)
            .unwrap_or_default()
    }

    /// Its metadata.
    ///
    /// # Errors
    /// Never; std's can.
    pub const fn metadata(&self) -> io::Result<Metadata> {
        Ok(self.metadata)
    }

    /// Its kind.
    ///
    /// # Errors
    /// Never; std's can.
    pub const fn file_type(&self) -> io::Result<FileType> {
        Ok(self.metadata.file_type())
    }
}

/// `std::fs::OpenOptions`.
#[derive(Debug, Clone, Default)]
#[allow(clippy::struct_excessive_bools)] // std's options, one flag each
pub struct OpenOptions {
    append: bool,
    truncate: bool,
    create: bool,
    create_new: bool,
}

impl OpenOptions {
    /// No options.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Kept for std's sake: every file here can be read.
    pub const fn read(&mut self, _read: bool) -> &mut Self {
        self
    }

    /// Kept for std's sake: every file here can be written.
    pub const fn write(&mut self, _write: bool) -> &mut Self {
        self
    }

    /// Each write at the end.
    pub const fn append(&mut self, append: bool) -> &mut Self {
        self.append = append;
        self
    }

    /// Emptied as it opens.
    pub const fn truncate(&mut self, truncate: bool) -> &mut Self {
        self.truncate = truncate;
        self
    }

    /// Made if not there.
    pub const fn create(&mut self, create: bool) -> &mut Self {
        self.create = create;
        self
    }

    /// Made, and an error if there already.
    pub const fn create_new(&mut self, create_new: bool) -> &mut Self {
        self.create_new = create_new;
        self
    }

    /// Opens the page's file at `path`.
    ///
    /// # Errors
    /// As `std::fs::OpenOptions::open`.
    pub fn open(&self, path: impl AsRef<Path>) -> io::Result<File> {
        STORE.open(path.as_ref(), self)
    }

    /// The same in `store`, for a test.
    ///
    /// # Errors
    /// As `std::fs::OpenOptions::open`.
    pub fn open_in(&self, store: &'static Store, path: impl AsRef<Path>) -> io::Result<File> {
        store.open(path.as_ref(), self)
    }
}

/// `std::fs::File`: a file in a store, read and written where its cursor is.
#[derive(Debug)]
pub struct File {
    store: &'static Store,
    path: PathBuf,
    position: u64,
    append: bool,
}

impl File {
    /// `std::fs::File::open`: to read.
    ///
    /// # Errors
    /// No file there.
    pub fn open(path: impl AsRef<Path>) -> io::Result<Self> {
        OpenOptions::new().read(true).open(path)
    }

    /// `std::fs::File::create`: made, or emptied.
    ///
    /// # Errors
    /// A folder there.
    pub fn create(path: impl AsRef<Path>) -> io::Result<Self> {
        OpenOptions::new()
            .write(true)
            .create(true)
            .truncate(true)
            .open(path)
    }

    /// `std::fs::File::create_new`: made, and an error if something is there.
    ///
    /// # Errors
    /// Something there already.
    pub fn create_new(path: impl AsRef<Path>) -> io::Result<Self> {
        OpenOptions::new().write(true).create_new(true).open(path)
    }

    /// `std::fs::File::options`.
    #[must_use]
    pub fn options() -> OpenOptions {
        OpenOptions::new()
    }

    /// Its metadata.
    ///
    /// # Errors
    /// The file removed meanwhile.
    pub fn metadata(&self) -> io::Result<Metadata> {
        self.store.metadata(&self.path)
    }

    /// Made `len` bytes long.
    ///
    /// # Errors
    /// The file removed meanwhile.
    pub fn set_len(&self, len: u64) -> io::Result<()> {
        self.store.with(|inner| inner.set_len(&self.path, len))
    }

    /// Nothing to do: the store is the file.
    ///
    /// # Errors
    /// Never; std's can.
    pub const fn sync_all(&self) -> io::Result<()> {
        Ok(())
    }

    /// Nothing to do: the store is the file.
    ///
    /// # Errors
    /// Never; std's can.
    pub const fn sync_data(&self) -> io::Result<()> {
        Ok(())
    }

    /// Another handle on the same file, at the same place.
    ///
    /// # Errors
    /// Never; std's can.
    pub fn try_clone(&self) -> io::Result<Self> {
        Ok(Self {
            store: self.store,
            path: self.path.clone(),
            position: self.position,
            append: self.append,
        })
    }
}

impl Read for File {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let position = usize::try_from(self.position).map_err(io::Error::other)?;
        let read = self.store.with(|inner| {
            let node = inner
                .files
                .get(&self.path)
                .ok_or_else(|| not_found(&self.path))?;
            let available = node.data.get(position..).unwrap_or_default();
            let n = available.len().min(buf.len());
            if let (Some(to), Some(from)) = (buf.get_mut(..n), available.get(..n)) {
                to.copy_from_slice(from);
            }
            Ok::<usize, io::Error>(n)
        })?;
        self.position += read as u64;
        Ok(read)
    }
}

impl Write for File {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        let position = self.position;
        let append = self.append;
        let written_at = self.store.with(|inner| {
            let at = if append {
                inner
                    .files
                    .get(&self.path)
                    .map_or(0, |node| node.data.len() as u64)
            } else {
                position
            };
            inner.write_at(&self.path, at, buf).map(|()| at)
        })?;
        self.position = written_at + buf.len() as u64;
        Ok(buf.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

impl Seek for File {
    fn seek(&mut self, to: SeekFrom) -> io::Result<u64> {
        let len = self.metadata()?.len();
        let target = match to {
            SeekFrom::Start(at) => Some(at),
            SeekFrom::End(by) => len.checked_add_signed(by),
            SeekFrom::Current(by) => self.position.checked_add_signed(by),
        };
        let target = target
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, "seek before the start"))?;
        self.position = target;
        Ok(target)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn store() -> &'static Store {
        Box::leak(Box::new(Store::new()))
    }

    fn p(path: &str) -> PathBuf {
        PathBuf::from(path)
    }

    /// What storage.js decodes: kind, path, and for a write offset, length and data.
    #[test]
    fn changes_are_packed_as_storage_js_reads_them() {
        let packed = encode(&[
            Change::Remove { path: p("/a") },
            Change::Write {
                path: p("/b/c"),
                offset: 2,
                data: b"xy".to_vec(),
                len: 4,
            },
        ]);
        let mut want = vec![2_u8, 2, 0, 0, 0];
        want.extend_from_slice(b"/a");
        want.extend_from_slice(&[1, 4, 0, 0, 0]);
        want.extend_from_slice(b"/b/c");
        want.extend_from_slice(&[2, 0, 0, 0, 4, 0, 0, 0, 2, 0, 0, 0]);
        want.extend_from_slice(b"xy");
        assert_eq!(packed, want);
        assert!(encode(&[]).is_empty());
    }

    /// The map's tile cache is the session's only: its files are held, and never handed to the
    /// browser to keep.
    #[test]
    fn the_map_cache_is_not_kept() {
        let store = store();
        let tile = p("/home/web/.local/share/MissionPlannerRust/gmapcache/TileDBv3/a/3/4/2.jpg");
        store.write(&tile, b"jpeg").unwrap();
        store
            .write(&p("/home/web/config.xml"), b"<Config/>")
            .unwrap();
        assert_eq!(store.read(&tile).unwrap(), b"jpeg");
        let changes = store.take_changes();
        assert_eq!(changes.len(), 1);
        assert!(
            matches!(&changes[0], Change::Write { path, .. } if path == &p("/home/web/config.xml"))
        );
        store.remove_file(&tile).unwrap();
        assert_eq!(store.take_changes(), []);
        assert!(kept(Path::new("/home/web/missions/a.waypoints")));
        assert!(!kept(Path::new("/x/gmapcache/y")));
    }

    #[test]
    fn paths_are_absolute_and_plain() {
        assert_eq!(normalize(Path::new("a/./b/../c")), p("/a/c"));
        assert_eq!(normalize(Path::new("/home/web/x")), p("/home/web/x"));
        assert_eq!(normalize(Path::new("../..")), p("/"));
    }

    #[test]
    fn a_file_written_is_read_back_and_its_folders_are_made() {
        let store = store();
        store
            .write(&p("/home/web/a/config.xml"), b"<Config/>")
            .unwrap();
        assert_eq!(
            store.read(&p("/home/web/a/config.xml")).unwrap(),
            b"<Config/>"
        );
        assert!(store.metadata(&p("/home/web/a")).unwrap().is_dir());
        assert!(
            store
                .metadata(&p("/home/web/a/config.xml"))
                .unwrap()
                .is_file()
        );
        assert_eq!(
            store.read(&p("/home/web/b")).unwrap_err().kind(),
            io::ErrorKind::NotFound
        );
        let names: Vec<_> = store
            .read_dir(&p("/home/web"))
            .unwrap()
            .iter()
            .map(DirEntry::file_name)
            .collect();
        assert_eq!(names, [OsString::from("a")]);
    }

    #[test]
    fn a_folder_lists_its_own_children_only() {
        let store = store();
        store.write(&p("/d/one.txt"), b"1").unwrap();
        store.write(&p("/d/sub/two.txt"), b"2").unwrap();
        store.create_dir_all(&p("/d/empty")).unwrap();
        let entries = store.read_dir(&p("/d")).unwrap();
        let listed: Vec<(String, bool)> = entries
            .iter()
            .map(|entry| {
                (
                    entry.file_name().to_string_lossy().into_owned(),
                    entry.file_type().unwrap().is_dir(),
                )
            })
            .collect();
        assert_eq!(
            listed,
            [
                ("empty".to_owned(), true),
                ("one.txt".to_owned(), false),
                ("sub".to_owned(), true)
            ]
        );
        store.remove_dir_all(&p("/d/sub")).unwrap();
        assert!(store.read(&p("/d/sub/two.txt")).is_err());
        assert_eq!(
            store.remove_dir(&p("/d")).unwrap_err().kind(),
            io::ErrorKind::DirectoryNotEmpty
        );
    }

    #[test]
    fn a_file_is_read_written_appended_and_sought_as_std_does() {
        let store = store();
        let mut file = OpenOptions::new()
            .write(true)
            .create(true)
            .truncate(true)
            .open_in(store, "/f.bin")
            .unwrap();
        file.write_all(b"hello world").unwrap();
        file.seek(SeekFrom::Start(6)).unwrap();
        file.write_all(b"WORLD").unwrap();
        let mut text = String::new();
        file.seek(SeekFrom::Start(0)).unwrap();
        file.read_to_string(&mut text).unwrap();
        assert_eq!(text, "hello WORLD");
        let mut log = OpenOptions::new()
            .append(true)
            .create(true)
            .open_in(store, "/f.bin")
            .unwrap();
        log.write_all(b"!").unwrap();
        assert_eq!(store.read(&p("/f.bin")).unwrap(), b"hello WORLD!");
        file.set_len(5).unwrap();
        assert_eq!(store.read(&p("/f.bin")).unwrap(), b"hello");
        assert_eq!(
            OpenOptions::new()
                .create_new(true)
                .open_in(store, "/f.bin")
                .unwrap_err()
                .kind(),
            io::ErrorKind::AlreadyExists
        );
        assert_eq!(
            OpenOptions::new()
                .open_in(store, "/none")
                .unwrap_err()
                .kind(),
            io::ErrorKind::NotFound
        );
        // `File::create_new`'s options, as a recording opens its file: made once, refused after.
        let mut fresh = OpenOptions::new()
            .write(true)
            .create_new(true)
            .open_in(store, "/new.tlog")
            .unwrap();
        fresh.write_all(b"t").unwrap();
        assert_eq!(store.read(&p("/new.tlog")).unwrap(), b"t");
        assert!(
            OpenOptions::new()
                .create_new(true)
                .open_in(store, "/new.tlog")
                .is_err()
        );
    }

    #[test]
    fn rename_and_copy_move_files_and_folders() {
        let store = store();
        store.write(&p("/old/a.txt"), b"a").unwrap();
        store.write(&p("/old/in/b.txt"), b"b").unwrap();
        store.rename(&p("/old"), &p("/new")).unwrap();
        assert_eq!(store.read(&p("/new/in/b.txt")).unwrap(), b"b");
        assert!(store.metadata(&p("/old")).is_err());
        assert_eq!(store.copy(&p("/new/a.txt"), &p("/copy.txt")).unwrap(), 1);
        store.rename(&p("/copy.txt"), &p("/moved.txt")).unwrap();
        assert_eq!(store.read(&p("/moved.txt")).unwrap(), b"a");
        assert!(store.read(&p("/copy.txt")).is_err());
    }

    /// What the browser is handed: nothing for what it already has, a new file whole, a log only
    /// the bytes it has grown by, a removal - and nothing again until the next change.
    #[test]
    fn the_browser_is_handed_what_changed_and_no_more() {
        let store = store();
        store.preload([
            (p("/home/web/config.xml"), b"<Config/>".to_vec()),
            (p("/home/web/old.log"), b"0123".to_vec()),
        ]);
        assert_eq!(store.take_changes(), []);
        assert_eq!(
            store.read(&p("/home/web/config.xml")).unwrap(),
            b"<Config/>"
        );

        store.write(&p("/home/web/new.txt"), b"new").unwrap();
        let mut log = OpenOptions::new()
            .append(true)
            .open_in(store, "/home/web/old.log")
            .unwrap();
        log.write_all(b"45").unwrap();
        log.write_all(b"67").unwrap();
        store.remove_file(&p("/home/web/config.xml")).unwrap();
        assert_eq!(
            store.take_changes(),
            [
                Change::Remove {
                    path: p("/home/web/config.xml")
                },
                Change::Write {
                    path: p("/home/web/new.txt"),
                    offset: 0,
                    data: b"new".to_vec(),
                    len: 3
                },
                Change::Write {
                    path: p("/home/web/old.log"),
                    offset: 4,
                    data: b"4567".to_vec(),
                    len: 8
                },
            ]
        );
        assert_eq!(store.take_changes(), []);
        // A write inside what the browser has: from there on.
        let mut file = OpenOptions::new()
            .write(true)
            .open_in(store, "/home/web/old.log")
            .unwrap();
        file.seek(SeekFrom::Start(2)).unwrap();
        file.write_all(b"X").unwrap();
        assert_eq!(
            store.take_changes(),
            [Change::Write {
                path: p("/home/web/old.log"),
                offset: 2,
                data: b"X34567".to_vec(),
                len: 8
            }]
        );
        // Removed and written again: the whole file.
        store.remove_file(&p("/home/web/new.txt")).unwrap();
        store.write(&p("/home/web/new.txt"), b"again").unwrap();
        assert_eq!(
            store.take_changes(),
            [Change::Write {
                path: p("/home/web/new.txt"),
                offset: 0,
                data: b"again".to_vec(),
                len: 5
            }]
        );
    }
}
