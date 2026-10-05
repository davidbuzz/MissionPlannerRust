// The bound on what a visit loads (the browser build's storage row: "a bound on what every visit
// loads, logs accumulating"). Every file the browser keeps is read into memory before the planner
// starts; logs are read newest first only up to a budget (www/storage.js's LOG_BUDGET, here 2 MB
// by ?logbudget=2), and the older ones stay in the browser's storage unread.
//
// Here: three copies of a 1.5 MB tlog kept in the log folder, oldest to newest; the next visit says
// on its status line that two (3.0 MB) were not loaded; the newest plays through Load Log on LOGS'
// Telemetry Logs; the two older ones are still in the browser's storage. Fails otherwise, or on any
// error in the page.
//
//   NODE_PATH=<a node_modules holding playwright> node check/storage_budget_check.js [out-dir] [url]
const fs = require("fs");
const path = require("path");
const { chromium } = require("playwright");
const out = process.argv[2] || ".";
const url = process.argv[3] || "http://127.0.0.1:8080/?facts=1&demo=0";
const fail = (why) => { console.log(`FAIL: ${why}`); process.exitCode = 1; };
const LOG_DIR = "home/web/.local/share/MissionPlannerRust/logs";
const TLOG = fs.readFileSync(path.join(__dirname, "../../testdata/mavlink/autotest.tlog"));
(async () => {
  const browser = await chromium.launch({
    headless: true,
    args: ["--enable-unsafe-swiftshader", "--use-angle=swiftshader", "--ignore-gpu-blocklist"],
  });
  const context = await browser.newContext({ viewport: { width: 1400, height: 900 } });
  const page = await context.newPage();
  const errors = [];
  page.on("console", (m) => { const t = m.text(); if (t.startsWith("panicked") || t.startsWith("planner fault")) errors.push(t.split("\n")[0]); });
  page.on("pageerror", (e) => errors.push(e.message));
  const facts = () => page.evaluate(() => (globalThis.mpFacts ? globalThis.mpFacts() : {})).catch(() => ({}));
  const probe = () => page.evaluate(() => (globalThis.mpProbe ? globalThis.mpProbe() : {})).catch(() => ({}));
  const until = async (what, test, ms = 15000) => {
    let f = {};
    for (let waited = 0; waited < ms; waited += 250) {
      await page.waitForTimeout(250);
      f = await facts();
      if (test(f)) return f;
    }
    fail(`${what}: screen "${f.screen}", status "${f.status}", fly.prompt "${f["fly.prompt"]}", fly.playback.file "${f["fly.playback.file"]}"`);
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
  const click = async (name) => { const xy = await at(name); if (xy) await page.mouse.click(...xy); };
  // The logs the browser keeps, by name.
  const kept = () => page.evaluate(async (dir) => {
    try {
      let handle = await (await navigator.storage.getDirectory()).getDirectoryHandle("planner");
      for (const part of dir.split("/")) handle = await handle.getDirectoryHandle(part);
      const names = [];
      for await (const [name] of handle.entries()) names.push(name);
      return names.sort();
    } catch (_) {
      return [];
    }
  }, LOG_DIR);

  // A first visit, and three logs kept as the planner would keep them, oldest first.
  await page.goto(url, { waitUntil: "load" });
  await until("the planner up", (f) => f.screen === "fly", 60000);
  for (const name of ["a.tlog", "b.tlog", "c.tlog"]) {
    await page.evaluate(async ([dir, name, bytes]) => {
      let handle = await (await navigator.storage.getDirectory()).getDirectoryHandle("planner", { create: true });
      for (const part of dir.split("/")) handle = await handle.getDirectoryHandle(part, { create: true });
      const writable = await (await handle.getFileHandle(name, { create: true })).createWritable();
      await writable.write(new Uint8Array(bytes));
      await writable.close();
    }, [LOG_DIR, name, [...TLOG]]);
    await page.waitForTimeout(100);
  }
  console.log(`kept before: ${JSON.stringify(await kept())}`);

  // The next visit loads 2 MB of logs: the newest alone.
  const budgeted = `${url}${url.includes("?") ? "&" : "?"}logbudget=2`;
  await page.goto(budgeted, { waitUntil: "load" });
  let f = await until("the note", (f) => (f.status ?? "").includes("older logs"), 60000);
  console.log(`status "${f.status}"`);
  const want = "2 older logs (3.0 MB) are kept in the browser but not loaded: a visit loads the newest 2 MB of logs";
  if (f.status !== want) fail(`the status line says "${f.status}", not "${want}"`);

  // The newest plays. The tab clicked by name once it has settled: the note is said in the
  // visit's first frames, before the planner takes a click.
  await until("the planner up", (f) => f.screen === "fly", 60000);
  await click("tab-logs");
  await until("LOGS", (f) => f.screen === "logs");
  await click("logs-tab-tlogs");
  await until("Telemetry Logs", (f) => f["logs.page"] === "tlogs");
  await click("fly-loadtelem");
  f = await until("Load Log's box", (f) => (f["fly.prompt"] ?? "").includes("Telemetry log") && (f["fly.prompt.value"] ?? "").includes("/logs/"));
  await page.keyboard.type("c.tlog");
  await page.keyboard.press("Enter");
  f = await until("the newest log playing", (f) => f["fly.playback.file"] === "c.tlog");
  console.log(`fly.playback.file ${f["fly.playback.file"]}`);

  // The older two are still kept, unread.
  const after = await kept();
  console.log(`kept after: ${JSON.stringify(after)}`);
  for (const name of ["a.tlog", "b.tlog", "c.tlog"]) if (!after.includes(name)) fail(`${name} is no longer kept`);

  for (const error of [...new Set(errors)].slice(0, 5)) console.log(`error: ${error.slice(0, 300)}`);
  if (errors.length) fail(`${errors.length} fault(s) in the page`);
  await page.screenshot({ path: `${out}/storage-budget.png` });
  await browser.close();
  if (!process.exitCode) console.log("PASS");
})();
