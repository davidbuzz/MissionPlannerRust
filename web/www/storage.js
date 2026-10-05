// The planner's files in the browser's own storage, so a visit keeps what the last one saved -
// config.xml, missions, logs (the owner's request of 2026-10-05). In the page the planner's files
// are held in memory (crates/mp-os/src/fs/mem.rs); this keeps them in the origin private file
// system (OPFS), under one folder, the planner's paths as folders within it:
//
// - loadFiles(), before the planner starts: every file kept, as [path, bytes] pairs, which the
//   planner takes in at the top of its main() (mp_os::fs::preload_from_page);
// - keepFiles(take), once it runs: every half second, what the planner's files changed since -
//   planner_storage_take(), mp_os::fs::mem::encode's bytes - written to OPFS in order, a file from
//   where it first differs (a growing log by what it grew), a removal as a removal.
const ROOT = "planner";
const EVERY_MS = 500;

async function root(create) {
    const top = await navigator.storage.getDirectory();
    return top.getDirectoryHandle(ROOT, { create });
}

/// Every file kept, as [path, Uint8Array] pairs; none when the browser keeps none for the page.
export async function loadFiles() {
    const files = [];
    let dir;
    try {
        dir = await root(false);
    } catch (_) {
        return files;
    }
    async function walk(handle, path) {
        for await (const [name, entry] of handle.entries()) {
            const at = `${path}/${name}`;
            if (entry.kind === "directory") {
                await walk(entry, at);
            } else {
                const file = await entry.getFile();
                files.push([at, new Uint8Array(await file.arrayBuffer())]);
            }
        }
    }
    await walk(dir, "");
    return files;
}

async function folder(parts, create) {
    let handle = await root(true);
    for (const part of parts) {
        handle = await handle.getDirectoryHandle(part, { create });
    }
    return handle;
}

/// One file of the planner's as the browser keeps it, by the planner's own path ("/home/web/...");
/// null when none is kept. For a file the page writes itself - the in-page SITL's eeprom.bin.
export async function readKept(path) {
    const parts = path.split("/").filter(Boolean);
    const name = parts.pop();
    try {
        const dir = await folder(parts, false);
        const file = await (await dir.getFileHandle(name)).getFile();
        return new Uint8Array(await file.arrayBuffer());
    } catch (_) {
        return null;
    }
}

/// Keeps one file of the planner's, written whole, by the planner's own path.
export async function writeKept(path, bytes) {
    const parts = path.split("/").filter(Boolean);
    const name = parts.pop();
    const dir = await folder(parts, true);
    const writable = await (await dir.getFileHandle(name, { create: true })).createWritable();
    await writable.write(bytes);
    await writable.close();
}

/// The changes `encode` packed (crates/mp-os/src/fs/mem.rs): per change a kind byte (1 a write,
/// 2 a removal), the path's length and UTF-8 bytes, and for a write the offset, the file's length
/// and the data's length and bytes - each number a little-endian u32.
function decode(bytes) {
    const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    const text = new TextDecoder();
    const changes = [];
    let at = 0;
    const u32 = () => { const value = view.getUint32(at, true); at += 4; return value; };
    while (at < bytes.length) {
        const kind = view.getUint8(at); at += 1;
        const pathLength = u32();
        const path = text.decode(bytes.subarray(at, at + pathLength)); at += pathLength;
        if (kind === 1) {
            const offset = u32(), length = u32(), dataLength = u32();
            const data = bytes.slice(at, at + dataLength); at += dataLength;
            changes.push({ kind: "write", path, offset, length, data });
        } else {
            changes.push({ kind: "remove", path });
        }
    }
    return changes;
}

async function apply(change) {
    const parts = change.path.split("/").filter(Boolean);
    const name = parts.pop();
    if (name === undefined) return;
    if (change.kind === "write") {
        const dir = await folder(parts, true);
        const file = await dir.getFileHandle(name, { create: true });
        const writable = await file.createWritable({ keepExistingData: true });
        await writable.write({ type: "write", position: change.offset, data: change.data });
        await writable.truncate(change.length);
        await writable.close();
    } else {
        try {
            const dir = await folder(parts, false);
            await dir.removeEntry(name, { recursive: true });
        } catch (_) {
            // Never kept, or gone already.
        }
    }
}

/// Keeps the planner's files: `take` is the planner's planner_storage_take.
export function keepFiles(take) {
    // Asked once: a persistent origin's storage is not cleared under pressure.
    if (navigator.storage.persist) navigator.storage.persist().catch(() => {});
    let chain = Promise.resolve();
    const flush = () => {
        const bytes = take();
        if (!bytes || bytes.length === 0) return;
        for (const change of decode(bytes)) {
            chain = chain.then(() => apply(change)).catch((err) => console.warn(`storage: ${change.path}: ${err}`));
        }
    };
    setInterval(flush, EVERY_MS);
    // Leaving the page: what changed since the last flush, as far as the browser lets it finish.
    addEventListener("pagehide", flush);
    globalThis.mpStorageFlushed = () => chain;
}

/// The page's own faults as crash reports. A WebAssembly fault in the planner - out of memory, an
/// unreachable - ends it without passing through Rust's panic hook, so no report says why; what
/// the planner reports next ("RefCell already borrowed", gpui's state left mid-update) is only
/// its consequence (the owner's browsers, 2026-10-05). The browser's own error, with its stack,
/// is written where the planner keeps its reports (crates/mp-gui/src/crash.rs: the data
/// folder's crash-reports/, named by the local time to the second), and the next start asks
/// about it first.
const REPORTS = "home/web/.local/share/MissionPlannerRust/crash-reports";

function stamp(date) {
    const two = (n) => String(n).padStart(2, "0");
    return `${date.getFullYear()}-${two(date.getMonth() + 1)}-${two(date.getDate())}T` +
        `${two(date.getHours())}-${two(date.getMinutes())}-${two(date.getSeconds())}`;
}

async function writeReport(text) {
    const dir = await folder(REPORTS.split("/"), true);
    const base = stamp(new Date());
    let name = `${base}.txt`;
    for (let counter = 1; ; counter++) {
        try {
            await dir.getFileHandle(name);
            name = `${base}-${counter}.txt`;
        } catch (_) {
            break;
        }
    }
    const writable = await (await dir.getFileHandle(name, { create: true })).createWritable();
    await writable.write(text);
    await writable.close();
}

export function keepFaults() {
    let written = 0;
    const report = (kind, error) => {
        // One fault's echoes are not reports of their own.
        if (written >= 3) return;
        written += 1;
        const name = error?.name ?? kind;
        const message = error?.message ?? String(error);
        const stack = (error?.stack ?? "").replace(/\n/g, " | ");
        const text = `message=${navigator.userAgent}\nException ${name}: ${message}\nStack: ${stack}\n` +
            `TargetSite the page (${kind})\ndata \n`;
        console.error(`planner fault (${kind}): ${name}: ${message}`);
        writeReport(text).catch((err) => console.warn(`storage: the fault's report: ${err}`));
    };
    addEventListener("error", (event) => report("error", event.error ?? event.message));
    addEventListener("unhandledrejection", (event) => report("unhandled rejection", event.reason));
}
