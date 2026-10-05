// A serial link in a page (the browser build's serial row: "a USB autopilot, Install Firmware and
// SiK radios need WebSerial"; www/serial.js, crates/mp-gui/src/page_serial.rs). A headless browser
// has no USB, so the page is given a stand-in for WebSerial before it starts: navigator.serial with
// one USB device (1209:5741), which the browser's chooser grants only from a click, as Chromium's
// does, and whose port, once open, sends a vehicle's MAVLink heartbeats and counts what it is sent.
//
// Here: the port box lists "Choose a serial port..." and nothing granted; choosing it grants the
// port from the click, and the port box selects it by its name; CONNECT opens it at the baud box's
// rate; the planner hears the vehicle and writes to it; DISCONNECT closes the port. Fails
// otherwise, or on any error in the page.
//
//   NODE_PATH=<a node_modules holding playwright> node check/serial_check.js [out-dir] [url]
const { chromium } = require("playwright");
const { clickAt } = require("./clicks");
const out = process.argv[2] || ".";
const url = process.argv[3] || "http://127.0.0.1:8080/?facts=1&demo=0";
const fail = (why) => { console.log(`FAIL: ${why}`); process.exitCode = 1; };
const PORT_NAME = "WebSerial 1 (1209:5741)";
const CHOOSE = "Choose a serial port...";

// The stand-in, run in the page before its own scripts.
function standIn() {
  // A MAVLink 1 HEARTBEAT from system 1: a quadrotor (2), ArduPilot (3), active (4).
  const x25 = (bytes) => {
    let crc = 0xffff;
    for (const b of bytes) {
      let tmp = (b ^ (crc & 0xff)) & 0xff;
      tmp = (tmp ^ (tmp << 4)) & 0xff;
      crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xffff;
    }
    return crc;
  };
  let seq = 0;
  const heartbeat = () => {
    const frame = [0xfe, 9, seq++ & 0xff, 1, 1, 0, 0, 0, 0, 0, 2, 3, 81, 4, 3];
    const crc = x25([...frame.slice(1), 50]);
    return new Uint8Array([...frame, crc & 0xff, crc >> 8]);
  };
  const seen = { requested: 0, grantedFromClick: null, opened: null, closed: 0, written: 0 };
  globalThis.standInSerial = seen;
  const port = {
    getInfo: () => ({ usbVendorId: 0x1209, usbProductId: 0x5741 }),
    async open(options) {
      if (seen.opened) throw new DOMException("The port is already open.", "InvalidStateError");
      seen.opened = options;
      let timer = null;
      this.readable = new ReadableStream({
        start(controller) { timer = setInterval(() => controller.enqueue(heartbeat()), 100); },
        cancel() { clearInterval(timer); },
      });
      this.writable = new WritableStream({ write(chunk) { seen.written += chunk.length; } });
    },
    async close() { seen.opened = null; seen.closed += 1; },
  };
  let granted = [];
  Object.defineProperty(navigator, "serial", {
    value: {
      getPorts: async () => granted,
      requestPort: async () => {
        seen.requested += 1;
        seen.grantedFromClick = navigator.userActivation.isActive;
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
  const browser = await chromium.launch({
    headless: true,
    args: ["--enable-unsafe-swiftshader", "--use-angle=swiftshader", "--ignore-gpu-blocklist"],
  });
  const context = await browser.newContext({ viewport: { width: 1400, height: 900 } });
  await context.addInitScript(standIn);
  const page = await context.newPage();
  const errors = [];
  page.on("console", (m) => { const t = m.text(); if (t.startsWith("panicked") || t.startsWith("planner fault")) errors.push(t.split("\n")[0]); });
  page.on("pageerror", (e) => errors.push(e.message));
  const facts = () => page.evaluate(() => (globalThis.mpFacts ? globalThis.mpFacts() : {})).catch(() => ({}));
  const probe = () => page.evaluate(() => (globalThis.mpProbe ? globalThis.mpProbe() : {})).catch(() => ({}));
  const seen = () => page.evaluate(() => ({ ...globalThis.standInSerial }));
  const until = async (what, test, ms = 15000) => {
    let f = {};
    for (let waited = 0; waited < ms; waited += 250) {
      await page.waitForTimeout(250);
      f = await facts();
      if (test(f)) return f;
    }
    fail(`${what}: link.port "${f["link.port"]}", link.target "${f["link.target"]}", vehicle.count "${f["vehicle.count"]}", status "${f.status}"`);
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

  // The port box: the chooser listed, nothing granted yet.
  await click("main-port");
  await at(`main-port-${CHOOSE}`);
  const listed = Object.keys(await probe()).filter((name) => name.startsWith("main-port-"));
  console.log(`listed: ${JSON.stringify(listed)}`);
  if (listed.some((name) => name.includes("WebSerial"))) fail("a port is listed before any was granted");

  // Chosen from the click; the port box takes it by its name.
  await click(`main-port-${CHOOSE}`);
  let f = await until("the port chosen", (f) => f["link.port"] === PORT_NAME);
  let s = await seen();
  console.log(`requested ${s.requested}, from a click ${s.grantedFromClick}; link.port "${f["link.port"]}", baud ${f["link.baud"]}`);
  if (s.requested !== 1 || s.grantedFromClick !== true) fail(`the chooser was asked ${s.requested} times, from a click: ${s.grantedFromClick}`);

  // Listed from now on; a second click on the box closes its list.
  await click("main-port");
  await at(`main-port-${PORT_NAME}`);
  await click("main-port");

  // CONNECT: opened at the baud box's rate, the vehicle heard, and written to.
  await click("main-connect");
  f = await until("the vehicle heard", (f) => Number(f["vehicle.count"]) >= 1 && (f["link.target"] ?? "").startsWith("serial:"), 30000);
  await page.waitForTimeout(2000);
  s = await seen();
  console.log(`link.target "${f["link.target"]}", vehicle.count ${f["vehicle.count"]}, frames ${f["link.frames"]}; opened at ${s.opened?.baudRate}, ${s.written} bytes written`);
  if (s.opened?.baudRate !== Number(f["link.baud"])) fail(`opened at ${s.opened?.baudRate}, not the baud box's ${f["link.baud"]}`);
  if (!(s.written > 0)) fail("the planner wrote nothing to the port");

  // DISCONNECT closes the port.
  await click("main-connect");
  await until("disconnected", (f) => f["link.button"] === "CONNECT", 15000);
  for (let waited = 0; waited < 5000 && (await seen()).closed === 0; waited += 250) await page.waitForTimeout(250);
  s = await seen();
  console.log(`closed ${s.closed}, open ${JSON.stringify(s.opened)}`);
  if (s.closed !== 1 || s.opened !== null) fail("DISCONNECT left the port open");

  for (const error of [...new Set(errors)].slice(0, 5)) console.log(`error: ${error.slice(0, 300)}`);
  if (errors.length) fail(`${errors.length} fault(s) in the page`);
  await page.screenshot({ path: `${out}/serial.png` });
  await browser.close();
  if (!process.exitCode) console.log("PASS");
})();
