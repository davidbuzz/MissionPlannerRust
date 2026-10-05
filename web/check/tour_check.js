// The whole planner's tour in a browser: connects as check/planner_check.js does, then opens every
// top screen with the vehicle connected, and fails on any panic or error in the page - the blocks
// on the page's main thread (`Atomics.wait`) and std calls a web page cannot make that only a
// screen in use reaches.
//
//   NODE_PATH=<a node_modules holding playwright> node check/tour_check.js [out-dir] [url]
const { chromium } = require("playwright");
const out = process.argv[2] || ".";
const url = process.argv[3] || "http://127.0.0.1:8080/?vehicle=copter&demo=0";
// The top tabs' x at 1400x900 (y 43), and the port box, its TCP entry and CONNECT.
const TABS = [["plan", 281], ["setup", 340], ["config", 406], ["params", 577], ["logs", 636], ["simulation", 487], ["experimental", 719], ["help", 877], ["fly", 230]];
const PORT_BOX = [996, 36], TCP = [964, 97], CONNECT = [1187, 36];
(async () => {
  const browser = await chromium.launch({
    headless: true,
    args: ["--enable-unsafe-swiftshader", "--use-angle=swiftshader", "--ignore-gpu-blocklist", "--js-flags=--stack-trace-limit=200"],
  });
  const page = await browser.newPage({ viewport: { width: 1400, height: 900 } });
  const events = [];
  const frames = (stack) => (stack.match(/planner\.wasm\.[^ ]+/g) || [])
    .map((f) => f.replace("planner.wasm.", "").replace(/\[[0-9a-f]{16}\]/g, ""))
    .filter((f) => /^(<)?(planner|mp_)/.test(f) && !f.includes("crash::install"))
    .slice(0, 4).join(" < ");
  page.on("console", (m) => { const t = m.text(); if (t.startsWith("panicked")) events.push(`panic: ${t.split("\n")[1]} @ ${frames(t)}`); });
  page.on("pageerror", (e) => events.push(`error: ${e.message.slice(0, 120)} @ ${frames(e.stack || "")}`));
  await page.goto(url, { waitUntil: "load" });
  await page.waitForTimeout(8000);
  // The built-in Drone ID plugin asks its question at every start in a page, which keeps no
  // settings yet: its OK, over the flight screen's map at 1400x900 (a click on the map otherwise).
  await page.mouse.click(854, 484);
  await page.waitForTimeout(800);
  await page.mouse.click(...PORT_BOX); await page.waitForTimeout(800);
  await page.mouse.click(...TCP); await page.waitForTimeout(800);
  await page.mouse.click(...CONNECT); await page.waitForTimeout(1500);
  await page.keyboard.press("Enter"); await page.waitForTimeout(1500);
  await page.keyboard.press("Enter");
  await page.waitForTimeout(25000);
  const crossed = await page.evaluate(() => (globalThis.mpLinkCrossed ? globalThis.mpLinkCrossed() : {}));
  if (!(crossed.toPlanner > 50000)) { console.log(`FAIL: not connected (${JSON.stringify(crossed)})`); process.exitCode = 1; }
  for (const [name, x] of TABS) {
    const before = events.length;
    await page.mouse.click(x, 43);
    await page.waitForTimeout(4000);
    await page.screenshot({ path: `${out}/tour-${name}.png` });
    console.log(`${name}: ${events.length - before} event(s)`);
  }
  const unique = [...new Set(events)];
  for (const e of unique.slice(0, 10)) console.log(e.slice(0, 400));
  if (events.length) { console.log(`FAIL: ${events.length} event(s), ${unique.length} distinct`); process.exitCode = 1; }
  await browser.close();
  if (!process.exitCode) console.log("PASS");
})();
