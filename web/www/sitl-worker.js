// ArduPilot's WebAssembly SITL in a Web Worker of its own, so that starting another (the planner's
// SIMULATION screen, a click on a vehicle) ends this one: the page terminates the worker.
// SERIAL0 is pumped as tools/sitl/wasm/bridge.mjs pumps it, with messages to the page in place of
// its TCP socket.
//
// The vehicle's parameters outlive the page, as tools/sitl/wasm/bridge.mjs keeps them on the
// desktop: ArduPilot's SITL keeps them in eeprom.bin in its working directory (libraries/
// AP_HAL_SITL/Storage.cpp), here Emscripten's in-memory filesystem, gone with the worker. The page
// hands over what it kept, put into the module before its main() opens it, and every half second
// is given the file back when it has changed - the page ends a worker with no last word.
//
//   page -> worker: { start: { module, args, eeprom } }, { bytes: Uint8Array } for the vehicle
//   worker -> page: { started: module }, { bytes: Uint8Array } from the vehicle,
//                   { eeprom: Uint8Array }, { print: text }, { failed: text }
const SERIAL0 = 0;
const BUFFER = 4096;
const EEPROM = "eeprom.bin";
const SAVE_EVERY_MS = 500;
const waiting = [];

onmessage = (event) => {
    const message = event.data;
    if (message.start) {
        const { module, args, eeprom } = message.start;
        start(module, args, eeprom ?? null).catch((err) => postMessage({ failed: String(err) }));
    } else if (message.bytes) {
        waiting.push(message.bytes);
    }
};

function same(a, b) {
    if (a === null || a.length !== b.length) return false;
    for (let i = 0; i < a.length; i++) if (a[i] !== b[i]) return false;
    return true;
}

async function start(module, args, eeprom) {
    const { default: createModule } = await import(`./sitl/${module}`);
    let kept = eeprom;
    const options = {
        arguments: args,
        print: (text) => postMessage({ print: text }),
        printErr: (text) => postMessage({ print: text }),
        // Emscripten calls these with the module, after its filesystem is up and before main().
        preRun: [(sitlModule) => {
            if (eeprom !== null) {
                (sitlModule ?? options).FS.writeFile(EEPROM, eeprom);
                postMessage({ print: `${EEPROM} restored (${eeprom.length} bytes)` });
            }
        }],
    };
    const sitl = await createModule(options);
    setInterval(() => {
        let now;
        try {
            now = sitl.FS.readFile(EEPROM);
        } catch (_) {
            return; // not opened yet
        }
        if (same(kept, now)) return;
        kept = now.slice();
        postMessage({ eeprom: now });
    }, SAVE_EVERY_MS);
    const malloc = sitl.cwrap("ardupilot_malloc", "number", ["number"]);
    const read = sitl.cwrap("ardupilot_serial_read", "number", ["number", "number", "number"]);
    const write = sitl.cwrap("ardupilot_serial_write", "number", ["number", "number", "number"]);
    const fromVehicle = malloc(BUFFER);
    const toVehicle = malloc(BUFFER);
    postMessage({ started: module });
    setInterval(() => {
        for (;;) {
            const length = read(SERIAL0, fromVehicle, BUFFER);
            if (length <= 0) break;
            const chunk = sitl.HEAPU8.slice(fromVehicle, fromVehicle + length);
            postMessage({ bytes: chunk }, [chunk.buffer]);
            if (length < BUFFER) break;
        }
        while (waiting.length > 0) {
            const chunk = waiting[0];
            const take = Math.min(chunk.length, BUFFER);
            sitl.HEAPU8.set(chunk.subarray(0, take), toVehicle);
            const written = write(SERIAL0, toVehicle, take);
            if (written <= 0) break;
            if (written < chunk.length) waiting[0] = chunk.subarray(written);
            else waiting.shift();
        }
    }, 5);
}
