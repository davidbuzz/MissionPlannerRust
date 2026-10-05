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

// The SITL running, in a Web Worker of its own (sitl-worker.js), so another start ends it.
let sitlWorker = null;

// Starts `module` (a file of sitl/) with `args`, ending any SITL before it; its SERIAL0 becomes
// the link's.
function startSitlModule(module, args) {
    stopSitl();
    status = `starting SITL ${module}`;
    const worker = new Worker(new URL("./sitl-worker.js", import.meta.url), { type: "module" });
    worker.onmessage = (event) => {
        const message = event.data;
        if (message.bytes) {
            inbox.push(message.bytes);
        } else if (message.print !== undefined) {
            console.log(`sitl: ${message.print}`);
        } else if (message.started) {
            status = `SITL ${message.started} in this page`;
        } else if (message.failed) {
            status = `SITL failed: ${message.failed}`;
            console.error(`sitl: ${message.failed}`);
        }
    };
    worker.postMessage({ start: { module, args } });
    sendTo = (bytes) => worker.postMessage({ bytes }, [bytes.buffer]);
    sitlWorker = worker;
}

function stopSitl() {
    if (sitlWorker !== null) {
        sitlWorker.terminate();
        sitlWorker = null;
        status = "SITL stopped";
    }
}

// A vehicle by name, with the bridge's command line from tools/sitl/wasm/README.md, less the port.
function startSitl(vehicle) {
    const [file, model] = VEHICLES[vehicle] ?? VEHICLES.copter;
    startSitlModule(file, [`-M${model}`, "-O-35.36,149.16,584,353", "-s1", "--serial0", "wasm", "--serial1", "none", "--serial2", "none"]);
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
        startSitl(query.get("vehicle") ?? "copter");
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
            } else if (asked.startsWith("sitl\n")) {
                // The SIMULATION screen's "try local wasm": a vehicle clicked, the module and its
                // command line as the desktop's bridge is given them (crates/mp-transport/src/page.rs).
                const [, module, ...args] = asked.split("\n");
                inbox.length = 0;
                startSitlModule(module, args);
            } else if (asked === "sitl-stop") {
                stopSitl();
            } else if (asked.startsWith("ws://") || asked.startsWith("wss://")) {
                inbox.length = 0;
                openWebSocket(asked);
                forwarding = true;
            } else {
                // A tcp: (or udpcl:) link: the SITL in this page, as a SITL already running is
                // reached on the desktop - started here, with ?vehicle=, when none runs yet.
                inbox.length = 0;
                forwarding = true;
                if (sitlWorker === null) {
                    startSitl(query.get("vehicle") ?? "copter");
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
