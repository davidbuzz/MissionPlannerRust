// The page's link to a vehicle, as globalThis.mpLink: what mp-transport's Transport is on the
// desktop. The planner (src/lib.rs) drains `take()` on a timer and writes with `send(bytes)`.
//
//   ?link=sitl[&vehicle=copter|plane|rover|heli]  ArduPilot's WebAssembly SITL in this page:
//                     tools/sitl/wasm (served at sitl/), SERIAL0 pumped as tools/sitl/wasm/bridge.mjs
//                     pumps it, with no TCP in between. The default.
//   ?link=ws://host:port/path                     a vehicle behind a WebSocket: Mission Planner's
//                     "WS" link (ExtLibs/Comms/CommsWebSocket.cs), binary frames both ways.
//
// The whole planner (planner.html) asks for its link itself, through crates/mp-transport/src/page.rs:
// `servePlanner` answers a `tcp:` link with the SITL in this page, started on the first one, and a
// `ws://` link with a WebSocket.

const inbox = [];
let status = "no link";
let sendTo = () => {};

export const mpLink = {
    take() {
        if (inbox.length === 0) {
            return new Uint8Array(0);
        }
        const total = inbox.reduce((sum, chunk) => sum + chunk.length, 0);
        const out = new Uint8Array(total);
        let at = 0;
        for (const chunk of inbox) {
            out.set(chunk, at);
            at += chunk.length;
        }
        inbox.length = 0;
        return out;
    },
    // `bytes` is a view of the planner's shared memory: copied before it is kept.
    send(bytes) {
        sendTo(bytes.slice());
    },
    status() {
        return status;
    },
};
globalThis.mpLink = mpLink;

// The vehicles tools/sitl/wasm carries, and the model each starts with (README.md there).
const VEHICLES = {
    copter: ["arducopter.js", "quad"],
    plane: ["arduplane.js", "plane"],
    rover: ["ardurover.js", "rover"],
    heli: ["arducopter-heli.js", "heli"],
};

let sitlStarting = null;

async function startSitl(vehicle) {
    const [file, model] = VEHICLES[vehicle] ?? VEHICLES.copter;
    status = `starting SITL ${vehicle}`;
    const { default: createModule } = await import(`./sitl/${file}`);
    // The bridge's command line from tools/sitl/wasm/README.md, less the port.
    const sitl = await createModule({
        arguments: [`-M${model}`, "-O-35.36,149.16,584,353", "-s1", "--serial0", "wasm", "--serial1", "none", "--serial2", "none"],
        print: (text) => console.log(`sitl: ${text}`),
        printErr: (text) => console.warn(`sitl: ${text}`),
    });
    const SERIAL0 = 0;
    const BUFFER = 4096;
    const malloc = sitl.cwrap("ardupilot_malloc", "number", ["number"]);
    const read = sitl.cwrap("ardupilot_serial_read", "number", ["number", "number", "number"]);
    const write = sitl.cwrap("ardupilot_serial_write", "number", ["number", "number", "number"]);
    const fromVehicle = malloc(BUFFER);
    const toVehicle = malloc(BUFFER);
    const waiting = [];
    sendTo = (bytes) => waiting.push(bytes);
    status = `SITL ${vehicle} in this page`;
    setInterval(() => {
        for (;;) {
            const length = read(SERIAL0, fromVehicle, BUFFER);
            if (length <= 0) {
                break;
            }
            inbox.push(sitl.HEAPU8.slice(fromVehicle, fromVehicle + length));
            if (length < BUFFER) {
                break;
            }
        }
        while (waiting.length > 0) {
            const chunk = waiting[0];
            const take = Math.min(chunk.length, BUFFER);
            sitl.HEAPU8.set(chunk.subarray(0, take), toVehicle);
            const written = write(SERIAL0, toVehicle, take);
            if (written <= 0) {
                break;
            }
            if (written < chunk.length) {
                waiting[0] = chunk.subarray(written);
            } else {
                waiting.shift();
            }
        }
    }, 5);
}

function openWebSocket(url) {
    status = `connecting ${url}`;
    const socket = new WebSocket(url);
    socket.binaryType = "arraybuffer";
    socket.onopen = () => (status = `WS ${url}`);
    socket.onmessage = (event) => {
        if (event.data instanceof ArrayBuffer) {
            inbox.push(new Uint8Array(event.data));
        }
    };
    socket.onclose = () => (status = `WS closed ${url}`);
    socket.onerror = () => (status = `WS error ${url}`);
    sendTo = (bytes) => {
        if (socket.readyState === WebSocket.OPEN) {
            socket.send(bytes);
        }
    };
}

export function startLink() {
    const query = new URLSearchParams(location.search);
    const link = query.get("link") ?? "sitl";
    if (link.startsWith("ws://") || link.startsWith("wss://")) {
        openWebSocket(link);
    } else {
        startSitl(query.get("vehicle") ?? "copter").catch((err) => {
            status = `SITL failed: ${err}`;
            console.error(err);
        });
    }
}

// The planner's side (planner.html): what it asks for, and the bytes both ways, every 5 ms. The
// planner's calls never wait (page.rs), so a refused hand-over is kept for the next turn.
export function servePlanner(planner) {
    const query = new URLSearchParams(location.search);
    let forwarding = false;
    // What crossed, for check/planner_check.js: bytes to the planner, and bytes it sent.
    const crossed = { asked: [], toPlanner: 0, fromPlanner: 0 };
    globalThis.mpLinkCrossed = () => ({ ...crossed });
    setInterval(() => {
        const asked = planner.requested();
        if (asked !== undefined && asked !== null) {
            console.log(`link: the planner asks for ${asked}`);
            crossed.asked.push(asked);
            if (asked === "close") {
                forwarding = false;
                status = "closed";
            } else if (asked.startsWith("ws://") || asked.startsWith("wss://")) {
                inbox.length = 0;
                openWebSocket(asked);
                forwarding = true;
            } else {
                // A tcp: (or udpcl:) link: the SITL in this page, as a SITL already running is
                // reached on the desktop. Started once; a second connect finds it running.
                inbox.length = 0;
                forwarding = true;
                if (sitlStarting === null) {
                    sitlStarting = startSitl(query.get("vehicle") ?? "copter").catch((err) => {
                        status = `SITL failed: ${err}`;
                        console.error(err);
                    });
                }
            }
        }
        while (forwarding && inbox.length > 0) {
            if (!planner.push(inbox[0])) break;
            crossed.toPlanner += inbox[0].length;
            inbox.shift();
        }
        if (!forwarding) inbox.length = 0;
        const out = planner.take();
        if (forwarding && out.length > 0) {
            crossed.fromPlanner += out.length;
            sendTo(out);
        }
    }, 5);
}
