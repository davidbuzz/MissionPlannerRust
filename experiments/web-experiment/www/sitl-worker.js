// ArduPilot's WebAssembly SITL in a Web Worker of its own, so that starting another (the planner's
// SIMULATION screen, a click on a vehicle) ends this one: the page terminates the worker.
// SERIAL0 is pumped as tools/sitl/wasm/bridge.mjs pumps it, with messages to the page in place of
// its TCP socket.
//
//   page -> worker: { start: { module, args } }, { bytes: Uint8Array } for the vehicle
//   worker -> page: { started: module }, { bytes: Uint8Array } from the vehicle,
//                   { print: text }, { failed: text }
const SERIAL0 = 0;
const BUFFER = 4096;
const waiting = [];

onmessage = (event) => {
    const message = event.data;
    if (message.start) {
        start(message.start.module, message.start.args).catch((err) => postMessage({ failed: String(err) }));
    } else if (message.bytes) {
        waiting.push(message.bytes);
    }
};

async function start(module, args) {
    const { default: createModule } = await import(`./sitl/${module}`);
    const sitl = await createModule({
        arguments: args,
        print: (text) => postMessage({ print: text }),
        printErr: (text) => postMessage({ print: text }),
    });
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
