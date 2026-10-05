// The page's link to a vehicle, for the planner (index.html), which asks for its link itself
// through crates/mp-transport/src/page.rs: `servePlanner` answers a `tcp:` link to this machine
// with ArduPilot's WebAssembly SITL in this page (tools/sitl/wasm, served at sitl/, SERIAL0 pumped
// as tools/sitl/wasm/bridge.mjs pumps it, with no TCP in between), started on the first one; a
// `ws://` link with a WebSocket (Mission Planner's "WS" link, ExtLibs/Comms/CommsWebSocket.cs,
// binary frames both ways); a `serial:` link with a port the browser has let the page use
// (WebSerial, serial.js); any other address over the tailnet (tailscale.js); and a `udp:` link,
// which waits for a vehicle to send first, on the page's own tailnet address.

import { dialTailscale, listenTailscale } from "./tailscale.js";
import { readKept, writeKept } from "./storage.js";
import { openSerial } from "./serial.js";

const inbox = [];
let status = "no link";
let sendTo = () => {};
// Which source's bytes are the link's: "sitl", "ws" or "tailscale". A SITL left running while the
// planner talks to the tailnet keeps its bytes to itself.
let active = null;
// The bytes for the SITL in the page, while it runs.
let sitlSend = () => {};
// A serial port the page has open (serial.js), as the promise of its link: what is asked of it -
// bytes, a rate, DTR - waits for it to open, each in the order asked. Closed before any other link:
// a port open is a port no other link, nor the next visit, may open.
let serialLink = null;
// Its closing, which the next open waits for: a port still closing will not open again.
let serialClosing = Promise.resolve();

function closeSerial() {
    if (serialLink) serialClosing = serialLink.then((link) => link?.close());
    serialLink = null;
}

// "serial:<name>:<baud>": the port the planner named, at the baud box's rate.
function openSerialLink(asked) {
    const rest = asked.slice("serial:".length);
    const colon = rest.lastIndexOf(":");
    const name = colon > 0 ? rest.slice(0, colon) : rest;
    const baud = (colon > 0 && Number(rest.slice(colon + 1))) || 115200;
    active = "serial";
    status = `opening serial ${name}`;
    const link = serialClosing.then(() => openSerial(name, baud, (bytes) => {
        if (active === "serial") inbox.push(bytes);
    }, (text) => {
        status = text;
        console.log(`link: ${text}`);
    }));
    serialLink = link;
    sendTo = (bytes) => { link.then((open) => open?.send(bytes)); };
}


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
// the link's. `folder`, the vehicle's SITL folder, holds its eeprom.bin - its parameters - in the
// browser's storage (storage.js): put into the module before it starts, and kept whenever the
// worker says it changed, as the desktop's bridge keeps it on disk (tools/sitl/wasm/bridge.mjs).
// Without one - a page opened with ?vehicle= - nothing is kept.
function startSitlModule(module, args, folder) {
    stopSitl();
    status = `starting SITL ${module}`;
    const eeprom = folder ? `${folder}/eeprom.bin` : null;
    const worker = new Worker(new URL("./sitl-worker.js", import.meta.url), { type: "module" });
    worker.onmessage = (event) => {
        const message = event.data;
        if (message.bytes) {
            if (active === "sitl") inbox.push(message.bytes);
        } else if (message.eeprom) {
            if (eeprom) writeKept(eeprom, message.eeprom).catch((err) => console.warn(`sitl: ${eeprom}: ${err}`));
        } else if (message.print !== undefined) {
            console.log(`sitl: ${message.print}`);
        } else if (message.started) {
            status = `SITL ${message.started} in this page`;
        } else if (message.failed) {
            status = `SITL failed: ${message.failed}`;
            console.error(`sitl: ${message.failed}`);
        }
    };
    (eeprom ? readKept(eeprom) : Promise.resolve(null))
        .then((saved) => worker.postMessage({ start: { module, args, eeprom: saved } }));
    sitlSend = (bytes) => worker.postMessage({ bytes }, [bytes.buffer]);
    sitlWorker = worker;
}

// The SITL in the page becomes the link.
function useSitl() {
    active = "sitl";
    sendTo = (bytes) => sitlSend(bytes);
}

// A tailnet address becomes the link: `network` "tcp" or "udp", `address` "host:port".
let tailnet = null;
// "udp:0.0.0.0:<port>": Mission Planner's UDP link, which waits for a vehicle to send first - in a
// page, on the page's own tailnet address, answering whoever sent last.
function listenTailnet(port) {
    tailnet?.close();
    active = "tailscale";
    status = `listening on udp ${port} over the tailnet`;
    const listener = listenTailscale("udp", Number(port), {
        onOpen: (address) => {
            status = `tailnet udp ${address}`;
            console.log(`tailscale: listening on udp ${address}`);
        },
        onData: (bytes) => {
            if (active === "tailscale" && tailnet === listener) inbox.push(bytes);
        },
        onClose: (reason) => {
            if (tailnet === listener) status = `tailnet closed: ${reason}`;
            console.log(`tailscale: udp ${port} closed: ${reason}`);
        },
    });
    tailnet = listener;
    sendTo = (bytes) => listener.write(bytes);
}

function openTailnet(network, address) {
    tailnet?.close();
    active = "tailscale";
    status = `connecting ${network} ${address} over the tailnet`;
    const stream = dialTailscale(network, address, {
        onOpen: () => (status = `tailnet ${network} ${address}`),
        onData: (bytes) => {
            if (active === "tailscale" && tailnet === stream) inbox.push(bytes);
        },
        onClose: (reason) => {
            if (tailnet === stream) status = `tailnet closed: ${reason}`;
            console.log(`tailscale: ${network} ${address} closed: ${reason}`);
        },
    });
    tailnet = stream;
    sendTo = (bytes) => stream.write(bytes);
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
    useSitl();
    const [file, model] = VEHICLES[vehicle] ?? VEHICLES.copter;
    startSitlModule(file, [`-M${model}`, "-O-35.36,149.16,584,353", "-s1", "--serial0", "wasm", "--serial1", "none", "--serial2", "none"]);
}

function openWebSocket(url) {
    status = `connecting ${url}`;
    const socket = new WebSocket(url);
    socket.binaryType = "arraybuffer";
    socket.onopen = () => (status = `WS ${url}`);
    active = "ws";
    socket.onmessage = (event) => {
        if (event.data instanceof ArrayBuffer && active === "ws") {
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

// "tcp:host:port" or "udpcl:host:port" as its scheme, host and port; an IPv6 host in brackets.
function splitLink(link) {
    const scheme = link.slice(0, link.indexOf(":"));
    const rest = link.slice(scheme.length + 1);
    const colon = rest.lastIndexOf(":");
    const host = rest.slice(0, colon).replace(/^\[(.*)\]$/, "$1");
    return [scheme, host, rest.slice(colon + 1)];
}

// The planner's side (index.html): what it asks for, and the bytes both ways, every 5 ms. The
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
            // A serial port's rate or DTR, in order with its bytes (page.rs's controls).
            const control = asked.startsWith("serial-baud\n") || asked.startsWith("serial-dtr\n");
            // Any link asked for, or none, ends the serial port the page had open.
            if (!control && !asked.startsWith("sitl")) closeSerial();
            if (control) {
                const [kind, value] = asked.split("\n");
                serialLink?.then((link) => {
                    if (kind === "serial-baud") link?.setBaud(Number(value));
                    else link?.setDtr(value === "1");
                });
            } else if (asked === "close") {
                forwarding = false;
                status = "closed";
            } else if (asked.startsWith("serial:")) {
                inbox.length = 0;
                forwarding = true;
                openSerialLink(asked);
            } else if (asked.startsWith("sitl\n")) {
                // The SIMULATION screen's "try local wasm": a vehicle clicked, the module and its
                // command line as the desktop's bridge is given them (crates/mp-transport/src/page.rs).
                const [, module, folder, ...args] = asked.split("\n");
                inbox.length = 0;
                startSitlModule(module, args, folder);
            } else if (asked === "sitl-stop") {
                stopSitl();
            } else if (asked.startsWith("ws://") || asked.startsWith("wss://")) {
                inbox.length = 0;
                openWebSocket(asked);
                forwarding = true;
            } else {
                inbox.length = 0;
                forwarding = true;
                const [scheme, host, port] = splitLink(asked);
                if (scheme === "tcp" && (host === "127.0.0.1" || host === "localhost")) {
                    // This machine's 5760, as a SITL already running is reached on the desktop:
                    // the SITL in this page - started here, with ?vehicle=, when none runs yet.
                    useSitl();
                    if (sitlWorker === null) {
                        startSitl(query.get("vehicle") ?? "copter");
                    }
                } else if (scheme === "tcp" || scheme === "udpcl") {
                    // Anywhere else: over the tailnet.
                    openTailnet(scheme === "tcp" ? "tcp" : "udp", `${host}:${port}`);
                } else if (scheme === "udp") {
                    // A vehicle that sends first: on the page's tailnet address.
                    listenTailnet(port);
                } else {
                    status = `no way to ${asked} from a web page`;
                    console.warn(`link: ${status}`);
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
