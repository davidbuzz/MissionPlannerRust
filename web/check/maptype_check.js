// Choosing a map type in a page (found 2026-10-06 sweeping the page's main thread for waits): the
// map lets go of its tile store when its imagery changes, and the store's threads were joined as it
// went - on the page's main thread a wait traps ("Atomics.wait cannot be called in this context")
// and the planner ended. mp_os::join_or_leave now leaves them to end on their own there.
//
// Here: FLIGHT PLAN's map types chosen one after another - each taken (the map.source fact), the
// planner still drawing after each, and no fault in the page. Fails otherwise.
//
//   NODE_PATH=<a node_modules holding playwright> node check/maptype_check.js [out-dir] [url]
const { chromium } = require("playwright");
const { clickNamed } = require("./clicks");
const out = process.argv[2] || ".";
const url = process.argv[3] || "http://127.0.0.1:8080/?facts=1&demo=0";
const fail = (why) => { console.log(`FAIL: ${why}`); process.exitCode = 1; };
const TYPES = ["osm", "bing-map", "google-satellite", "opentopo"];
(async () => {
  const browser = await chromium.launch({
    headless: true,
    args: ["--enable-unsafe-swiftshader", "--use-angle=swiftshader", "--ignore-gpu-blocklist"],
  });
  const page = await browser.newPage({ viewport: { width: 1400, height: 900 } });
  const errors = [];
  page.on("console", (m) => { const t = m.text(); if (t.startsWith("panicked") || t.startsWith("planner fault")) errors.push(t.split("\n")[0]); });
  page.on("pageerror", (e) => errors.push(e.message));
  const facts = () => page.evaluate(() => (globalThis.mpFacts ? globalThis.mpFacts() : {})).catch(() => ({}));
  const until = async (what, test, ms = 15000) => {
    let f = {};
    for (let waited = 0; waited < ms; waited += 250) {
      await page.waitForTimeout(250);
      f = await facts();
      if (test(f)) return f;
    }
    fail(`${what}: screen "${f.screen}", map.source "${f["map.source"]}", ui.frame ${f["ui.frame"]}`);
    return f;
  };

  await page.goto(url, { waitUntil: "load" });
  await until("the planner up", (f) => f.screen === "fly", 60000);
  await clickNamed(page, "tab-plan");
  await until("FLIGHT PLAN", (f) => f.screen === "plan");
  for (const type of TYPES) {
    if (!(await clickNamed(page, `plan-map-${type}`))) continue;
    const taken = await until(`${type} taken`, (f) => f["map.source"] === type);
    // Still drawing: frames counted after it, as a click on the screen asks for one.
    const before = Number(taken["ui.frame"]);
    await page.mouse.move(700, 450);
    await page.mouse.move(720, 470);
    const after = await until(`frames after ${type}`, (f) => Number(f["ui.frame"]) > before);
    console.log(`${type}: map.source ${after["map.source"]}, frame ${before} -> ${after["ui.frame"]}, faults ${errors.length}`);
    if (errors.length) break;
  }
  for (const error of [...new Set(errors)].slice(0, 5)) console.log(`error: ${error.slice(0, 300)}`);
  if (errors.length) fail(`${errors.length} fault(s) in the page`);
  await page.screenshot({ path: `${out}/maptype.png` });
  await browser.close();
  if (!process.exitCode) console.log("PASS");
})();
