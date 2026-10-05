// The SIMULATION screen in a browser, as a pilot uses it: "try local wasm" ticked by default,
// a click on Multirotor, and the copter starts in the page and the planner connects to it - no
// typing, no Cygwin note, and no fetch of ArduPilot's manifest. Fails otherwise, or on any error.
//
//   NODE_PATH=<a node_modules holding playwright> node check/sim_check.js [out-dir] [url]
const { chromium } = require("playwright");
const out = process.argv[2] || ".";
const url = process.argv[3] || "http://127.0.0.1:8080/?demo=0";
const fail = (why) => { console.log(`FAIL: ${why}`); process.exitCode = 1; };
// At 1400x900: the SIMULATION tab, and the Multirotor picture.
const SIMULATION = [487, 43], MULTIROTOR = [736, 805];
(async () => {
  const browser = await chromium.launch({
    headless: true,
    args: ["--enable-unsafe-swiftshader", "--use-angle=swiftshader", "--ignore-gpu-blocklist", "--js-flags=--stack-trace-limit=200"],
  });
  const page = await browser.newPage({ viewport: { width: 1400, height: 900 } });
  const errors = [];
  const manifest = [];
  page.on("console", (m) => { const t = m.text(); if (t.startsWith("panicked")) errors.push(t.split("\n").slice(0, 2).join(" ")); });
  page.on("pageerror", (e) => errors.push(e.message));
  page.on("request", (r) => { if (r.url().includes("manifest.json")) manifest.push(r.url()); });
  await page.goto(url, { waitUntil: "load" });
  await page.waitForTimeout(8000);
  // The built-in Drone ID plugin asks its question at every start in a page, which keeps no
  // settings yet: its OK, over the flight screen's map at 1400x900 (a click on the map otherwise).
  await page.mouse.click(854, 484);
  await page.waitForTimeout(800);
  await page.mouse.click(...SIMULATION);
  await page.waitForTimeout(3000);
  await page.screenshot({ path: `${out}/sim-page.png` });
  await page.mouse.click(...MULTIROTOR);
  let crossed = {};
  for (let waited = 0; waited < 45000; waited += 1000) {
    await page.waitForTimeout(1000);
    crossed = await page.evaluate(() => (globalThis.mpLinkCrossed ? globalThis.mpLinkCrossed() : {}));
    if (crossed.toPlanner > 50000 && crossed.fromPlanner > 1000) break;
  }
  await page.screenshot({ path: `${out}/sim-connected.png` });
  console.log(JSON.stringify(crossed).slice(0, 400));
  const asked = crossed.asked ?? [];
  const start = asked.find((a) => a.startsWith("sitl\n")) ?? "";
  if (!start.startsWith("sitl\narducopter.js")) fail(`no copter start asked: ${JSON.stringify(asked)}`);
  if (!start.includes("--serial0\nwasm")) fail(`the start's command line: ${JSON.stringify(start)}`);
  if (!asked.includes("tcp:127.0.0.1:5760")) fail(`no connect after the start: ${JSON.stringify(asked)}`);
  if (!(crossed.toPlanner > 50000)) fail(`only ${crossed.toPlanner} bytes reached the planner`);
  if (!(crossed.fromPlanner > 1000)) fail(`only ${crossed.fromPlanner} bytes came from the planner`);
  if (manifest.length) fail(`the manifest was fetched: ${manifest.join(", ")}`);
  for (const error of [...new Set(errors)].slice(0, 5)) console.log(`error: ${error.slice(0, 300)}`);
  if (errors.length) fail(`${errors.length} error(s) in the page`);
  await browser.close();
  if (!process.exitCode) console.log("PASS");
})();
