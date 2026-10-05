// A SiK radio on a serial port in a page (the browser build's serial row: "Install Firmware and SiK
// radios need WebSerial"). The SiK Radio page opens its own port (crates/mp-gui/src/config/
// sikradio.rs's LinkPort, mp-transport's SerialTransport), which in a page is carried by the page
// as its links are, a rate change and DTR going to the port in order with the bytes around them.
//
// A headless browser has no USB, so the page is given a stand-in for WebSerial whose one port is
// backed by tests/gui/sik-radio.py - the RFD900+ and its remote that the desktop's
// config-sikradio.gui loads - through web/tools/ws_relay.py, a WebSocket in front of its TCP port.
// Here: the port chosen from a click; SETUP's Sik Radio page's Load Settings reads both radios
// through it ("Done", the local radio's ATI), and leaving SETUP closes the port. Fails otherwise,
// or on any error in the page.
//
//   NODE_PATH=<a node_modules holding playwright> node check/serial_sik_check.js [out-dir] [url]
const fs = require("fs");
const os = require("os");
const path = require("path");
const { spawn } = require("child_process");
const { chromium } = require("playwright");
const { clickAt } = require("./clicks");
const out = process.argv[2] || ".";
const url = process.argv[3] || "http://127.0.0.1:8080/?facts=1&demo=0";
const fail = (why) => { console.log(`FAIL: ${why}`); process.exitCode = 1; };
const PORT_NAME = "WebSerial 1 (1209:5741)";
const CHOOSE = "Choose a serial port...";
const RADIO_PORT = 5795, RELAY_PORT = 5801;
const REPO = path.join(__dirname, "../..");

// The stand-in, run in the page before its own scripts: one USB port, granted only from a click,
// whose bytes go to and from the WebSocket at `relay` while it is open.
function standIn(relay) {
  const seen = { requested: 0, opens: [], closed: 0, dtr: [], written: 0, read: 0 };
  globalThis.standInSerial = seen;
  const port = {
    getInfo: () => ({ usbVendorId: 0x1209, usbProductId: 0x5741 }),
    async open(options) {
      if (this.socket) throw new DOMException("The port is already open.", "InvalidStateError");
      seen.opens.push(options.baudRate);
      const socket = new WebSocket(relay);
      socket.binaryType = "arraybuffer";
      this.socket = socket;
      await new Promise((resolve, reject) => { socket.onopen = resolve; socket.onerror = reject; });
      this.readable = new ReadableStream({
        start(controller) {
          socket.onmessage = (event) => {
            const bytes = new Uint8Array(event.data);
            seen.read += bytes.length;
            controller.enqueue(bytes);
          };
          socket.onclose = () => { try { controller.close(); } catch (_) {} };
        },
      });
      this.writable = new WritableStream({
        write(chunk) {
          seen.written += chunk.length;
          if (socket.readyState === WebSocket.OPEN) socket.send(chunk);
        },
      });
    },
    async setSignals(signals) { seen.dtr.push(signals.dataTerminalReady); },
    async close() {
      this.socket?.close();
      this.socket = null;
      seen.closed += 1;
    },
  };
  let granted = [];
  Object.defineProperty(navigator, "serial", {
    value: {
      getPorts: async () => granted,
      requestPort: async () => {
        seen.requested += 1;
        if (!navigator.userActivation.isActive) {
          throw new DOMException("Must be handling a user gesture to show a permission request.", "SecurityError");
        }
        granted = [port];
        return port;
      },
      addEventListener: () => {},
    },
  });
}

(async () => {
  // The radio, and the WebSocket in front of it.
  const work = fs.mkdtempSync(path.join(os.tmpdir(), "serial-sik-"));
  const children = [
    spawn("python3", [path.join(REPO, "tests/gui/sik-radio.py"), "--work", work, "--port", String(RADIO_PORT)], { stdio: "ignore" }),
    spawn("python3", [path.join(REPO, "web/tools/ws_relay.py"), String(RELAY_PORT), "127.0.0.1", String(RADIO_PORT)], { stdio: "ignore" }),
  ];
  const stop = () => children.forEach((child) => child.kill());
  process.on("exit", stop);
  await new Promise((resolve) => setTimeout(resolve, 2000));

  const browser = await chromium.launch({
    headless: true,
    args: ["--enable-unsafe-swiftshader", "--use-angle=swiftshader", "--ignore-gpu-blocklist"],
  });
  const context = await browser.newContext({ viewport: { width: 1400, height: 900 } });
  await context.addInitScript(standIn, `ws://127.0.0.1:${RELAY_PORT}`);
  const page = await context.newPage();
  const errors = [];
  page.on("console", (m) => { const t = m.text(); if (t.startsWith("panicked") || t.startsWith("planner fault")) errors.push(t.split("\n")[0]); });
  page.on("pageerror", (e) => errors.push(e.message));
  const facts = () => page.evaluate(() => (globalThis.mpFacts ? globalThis.mpFacts() : {})).catch(() => ({}));
  const probe = () => page.evaluate(() => (globalThis.mpProbe ? globalThis.mpProbe() : {})).catch(() => ({}));
  const seen = () => page.evaluate(() => JSON.parse(JSON.stringify(globalThis.standInSerial)));
  const until = async (what, test, ms = 15000) => {
    let f = {};
    for (let waited = 0; waited < ms; waited += 250) {
      await page.waitForTimeout(250);
      f = await facts();
      if (test(f)) return f;
    }
    fail(`${what}: screen "${f.screen}", setup.page "${f["setup.page"]}", sikradio port "${f["config.sikradio.port"]}", busy "${f["config.sikradio.busy"]}", status "${f["config.sikradio.status"]}"`);
    return f;
  };
  const at = async (name, ms = 10000) => {
    let last = null;
    for (let waited = 0; waited < ms; waited += 250) {
      const place = (await probe())[name];
      if (place && last && place.x === last.x && place.y === last.y) return [place.x, place.y];
      last = place ?? null;
      await page.waitForTimeout(250);
    }
    fail(`${name} never showed, or never stayed put`);
    return null;
  };
  const click = async (name) => { const xy = await at(name); if (xy) await clickAt(page, xy); return xy; };

  await page.goto(url, { waitUntil: "load" });
  await until("the planner up", (f) => f.screen === "fly", 60000);

  // The radio's port chosen from a click, at the SiK radios' 57600.
  await click("main-port");
  await click(`main-port-${CHOOSE}`);
  await until("the port chosen", (f) => f["link.port"] === PORT_NAME);
  await click("main-baud");
  await click("main-baud-57600");
  await until("57600", (f) => f["link.baud"] === "57600");

  // SETUP's Sik Radio page, and Load Settings over the port.
  await click("tab-setup");
  await until("SETUP", (f) => f.screen === "setup");
  await click("setup-page-ConfigOptional");
  await click("setup-page-Sikradio");
  await until("the Sik Radio page", (f) => f["setup.page"] === "Sikradio" && f["config.sikradio.active"] === "true");
  await click("sikradio-BUT_getcurrent");
  let f = await until("the radios read", (f) => f["config.sikradio.status"] === "Done" && f["config.sikradio.busy"] === "none", 60000);
  let s = await seen();
  console.log(`port "${f["config.sikradio.port"]}", status "${f["config.sikradio.status"]}", mode ${f["config.sikradio.mode"]}, ATI "${f["config.sikradio.ATI.text"]}"; opened at ${JSON.stringify(s.opens)}, ${s.written} bytes written, ${s.read} read`);
  if (!(f["config.sikradio.port"] ?? "").startsWith(`serial:${PORT_NAME}`)) fail(`the page opened "${f["config.sikradio.port"]}", not the serial port`);
  if (f["config.sikradio.ATI.text"] !== "RFD SiK 2.65 on RFD900P") fail(`the local radio's ATI is "${f["config.sikradio.ATI.text"]}"`);
  if (s.opens[0] !== 57600) fail(`the port was opened at ${s.opens[0]}, not 57600`);

  // Leaving SETUP puts the radio back and closes the port.
  await click("tab-fly");
  await until("FLIGHT DATA", (f) => f.screen === "fly");
  for (let waited = 0; waited < 10000 && (await seen()).closed === 0; waited += 250) await page.waitForTimeout(250);
  s = await seen();
  console.log(`closed ${s.closed}`);
  if (s.closed < 1) fail("leaving SETUP left the port open");

  for (const error of [...new Set(errors)].slice(0, 5)) console.log(`error: ${error.slice(0, 300)}`);
  if (errors.length) fail(`${errors.length} fault(s) in the page`);
  await page.screenshot({ path: `${out}/serial-sik.png` });
  await browser.close();
  stop();
  fs.rmSync(work, { recursive: true, force: true });
  if (!process.exitCode) console.log("PASS");
})();
