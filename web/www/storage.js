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
