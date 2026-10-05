// The computer's files for the planner in a page (crates/mp-gui/src/page_files.rs): a page's files
// are the planner's own, kept in the browser's storage (storage.js), and the computer's are out of
// its reach. So a file box that opens a file offers the browser's picker, and a file a box saves is
// handed to the browser as a download too. The planner calls the two functions this puts on the
// page:
//
// - mprPickFile(accept): the browser's file picker, filtered to `accept` (".waypoints,.txt"; empty
//   for any file). The file chosen goes back to the planner, its name and bytes, through
//   planner_file_picked, which puts it in the box's folder and types its path into the box;
// - mprDownload(name, bytes): `bytes` saved by the browser as the file `name`, where it saves
//   downloads.

/// Installs the two; `picked` is the planner's planner_file_picked.
export function serveFiles(picked) {
    globalThis.mprPickFile = (accept) => {
        const input = document.createElement("input");
        input.type = "file";
        if (accept) input.accept = accept;
        input.style.display = "none";
        input.addEventListener("change", async () => {
            const file = input.files?.[0];
            input.remove();
            if (!file) return;
            try {
                picked(file.name, new Uint8Array(await file.arrayBuffer()));
            } catch (err) {
                console.warn(`the file chosen, ${file.name}: ${err}`);
            }
        }, { once: true });
        input.addEventListener("cancel", () => input.remove(), { once: true });
        document.body.append(input);
        input.click();
    };
    globalThis.mprDownload = (name, bytes) => {
        const url = URL.createObjectURL(new Blob([bytes], { type: "application/octet-stream" }));
        const link = document.createElement("a");
        link.href = url;
        link.download = name;
        link.style.display = "none";
        document.body.append(link);
        link.click();
        link.remove();
        setTimeout(() => URL.revokeObjectURL(url), 60000);
    };
}
