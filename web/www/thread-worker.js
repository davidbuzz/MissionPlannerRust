// What every one of the planner's threads runs in a page: wasm_thread's worker script
// (src/wasm32/js/web_worker_module.js, zed-industries/wasm_thread rev 0cf96c77, MIT or
// Apache-2.0), as a file of the site's own. wasm_thread makes the same script up as a blob: address
// unless told otherwise, and a worker made from a blob: address sends no Referer with its
// requests - so OpenStreetMap's tile servers refused the map's tiles, "403 Access blocked" (the
// owner's bug of 2026-10-06). mp_os::threads_from_the_site points wasm_thread here, beside the page.
import init, { wasm_thread_entry_point } from "./pkg-planner/planner.js";

// Wait for the main thread to send the shared module and memory and the work, then run the work.
self.onmessage = event => {
    const [module_or_path, memory, work] = event.data;

    init({ module_or_path, memory }).catch(err => {
        console.log(err);

        // Propagate to main `onerror`:
        setTimeout(() => {
            throw err;
        });
        // Rethrow to keep promise rejected and prevent execution of further commands:
        throw err;
    }).then(() => {
        // Enter rust code by calling entry point defined in `lib.rs`.
        // This executes closure defined by work context.
        wasm_thread_entry_point(work);

        // Once done, terminate web worker
        close();
    });
};
