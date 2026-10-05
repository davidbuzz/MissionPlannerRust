// The experiment's test: loads the page in headless Chromium (Playwright) with ArduPilot's
// WebAssembly SITL in the page, and fails unless the planner's HUD draws that vehicle - read from
// the facts the page keeps of the scene it painted (src/lib.rs `facts`), as the desktop's GUI
// scripts read MP_FACTS - and the status line stays above the HUD rather than under its sky.
//
//   NODE_PATH=<a node_modules holding playwright> node check/check.js [out-dir] [url] [mode]
//
// The default is the copter in the page, which starts in Stabilize. Another vehicle or link names
// the mode it starts in: a plane behind tools/ws_relay.py is
//   check/check.js . "http://127.0.0.1:8080/hud.html?link=ws://127.0.0.1:5800" Manual
const { chromium } = require("playwright");
const out = process.argv[2] || ".";
const url = process.argv[3] || "http://127.0.0.1:8080/hud.html?link=sitl&vehicle=copter";
const mode = process.argv[4] || "Stabilize";
// The colour of a 1x1 PNG: its one row inflated, past the filter byte (every filter leaves a lone
// pixel as it is).
const onePixel = (png) => {
  const idat = [];
  for (let at = 8; at < png.length; ) {
    const length = png.readUInt32BE(at);
    if (png.toString("ascii", at + 4, at + 8) === "IDAT") idat.push(png.subarray(at + 8, at + 8 + length));
    at += 12 + length;
  }
  const row = require("zlib").inflateSync(Buffer.concat(idat));
  return [row[1], row[2], row[3]];
};
const fail = (why) => { console.log(`FAIL: ${why}`); process.exitCode = 1; };
(async () => {
  const browser = await chromium.launch({
    headless: true,
    args: ["--enable-unsafe-swiftshader", "--use-angle=swiftshader", "--ignore-gpu-blocklist"],
  });
  const page = await browser.newPage({ viewport: { width: 1280, height: 800 } });
  const errors = [];
  page.on("console", (m) => { if (m.type() === "error") console.log(`console.error: ${m.text().slice(0, 300)}`); });
  page.on("pageerror", (e) => { errors.push(e.message); console.log(`pageerror: ${e.message}`); });
  await page.goto(url, { waitUntil: "load" });
  if (!(await page.evaluate(() => self.crossOriginIsolated))) fail("the page is not cross-origin isolated");
  // The SITL boots in a few seconds; the HUD is due once it sends ATTITUDE.
  let facts = {};
  for (let waited = 0; waited < 45000; waited += 1000) {
    await page.waitForTimeout(1000);
    facts = await page.evaluate(() => (globalThis.mpFacts ? globalThis.mpFacts() : {}));
    if (facts["vehicle.mode"] && (facts["hud.gps"] ?? "").includes("Fix") && Number(facts["link.frames"]) > 50) break;
  }
  await page.screenshot({ path: `${out}/web-hud.png` });
  console.log(JSON.stringify(facts));
  if (!(Number(facts["link.frames"]) > 50)) fail(`too few frames from the SITL: ${facts["link.frames"]}`);
  if (facts["vehicle.mode"] !== mode) fail(`the vehicle's mode is ${facts["vehicle.mode"]}, not ${mode}`);
  if (facts["vehicle.armed"] !== "false") fail(`armed is ${facts["vehicle.armed"]}`);
  if (!/^Bat1 12\.\d+v/.test(facts["hud.battery.lower"] ?? "")) fail(`the battery line is ${facts["hud.battery.lower"]}`);
  if (!(facts["hud.gps"] ?? "").includes("Fix")) fail(`the GPS line is ${facts["hud.gps"]}`);
  // The status line's colour at its middle, not the HUD's sky (src/lib.rs clips the HUD).
  const strip = await page.screenshot({ clip: { x: 900, y: 10, width: 1, height: 1 } });
  const pixel = onePixel(strip);
  if (Math.abs(pixel[0] - 0x26) > 8 || Math.abs(pixel[1] - 0x2a) > 8 || Math.abs(pixel[2] - 0x30) > 8)
    fail(`the status line is painted over: ${pixel}`);
  if (errors.length) fail(`${errors.length} page error(s)`);
  await browser.close();
  if (!process.exitCode) console.log("PASS");
})();
