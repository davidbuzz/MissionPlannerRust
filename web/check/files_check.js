// The computer's files in a page (the browser build's storage row: "the browser's file picker and
// downloads for Load and Save"; crates/mp-gui/src/page_files.rs, www/files.js). On FLIGHT PLAN:
// Load File's box offers "From this computer...", the browser's picker filtered to the box's
// types; QGroundControl's plan chosen there is put in the box's folder - kept in the browser's
// storage - and loaded as a double click loads a listed file. Save File then saves the mission as
// .waypoints and as .mission, and each is handed to the browser as a download of that name, with
// what was saved in it. Fails otherwise, or on any error in the page.
//
//   NODE_PATH=<a node_modules holding playwright> node check/files_check.js [out-dir] [url]
const fs = require("fs");
const path = require("path");
const { chromium } = require("playwright");
const out = process.argv[2] || ".";
const url = process.argv[3] || "http://127.0.0.1:8080/?facts=1&demo=0";
const fail = (why) => { console.log(`FAIL: ${why}`); process.exitCode = 1; };
const PLAN_TAB = [281, 43];
const PLAN_FILE = path.join(__dirname, "../../testdata/missions/qgc_survey.plan");
(async () => {
  const browser = await chromium.launch({
    headless: true,
    args: ["--enable-unsafe-swiftshader", "--use-angle=swiftshader", "--ignore-gpu-blocklist"],
  });
  const context = await browser.newContext({ viewport: { width: 1400, height: 900 }, acceptDownloads: true });
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
    fail(`${what}: screen "${f.screen}", plan.prompt "${f["plan.prompt"]}", status "${f.status}", mission.items "${f["mission.items"]}"`);
    return f;
  };
  // A control's centre, once it is on screen and has stayed put: the frame a dialog opens in
  // may measure it before the dialog is laid out where it stays.
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
  const click = async (name) => { const xy = await at(name); if (xy) await page.mouse.click(...xy); return xy; };

  await page.goto(url, { waitUntil: "load" });
  await until("the planner up", (f) => f.screen === "fly", 60000);
  await page.mouse.click(...PLAN_TAB);
  await until("FLIGHT PLAN", (f) => f.screen === "plan");

  // Load File, and the browser's picker from its box.
  await click("plan-load");
  await until("Load File's box", (f) => f["plan.prompt"] === "Open");
  const browse = await at("plan-file-browse");
  if (browse) {
    const [chooser] = await Promise.all([page.waitForEvent("filechooser", { timeout: 10000 }), page.mouse.click(...browse)]);
    const accept = await chooser.element().evaluate((input) => input.accept);
    console.log(`the picker: accept "${accept}", multiple ${chooser.isMultiple()}`);
    if (!accept.split(",").includes(".plan")) fail(`the picker does not take a .plan: accept "${accept}"`);
    await chooser.setFiles({ name: "survey.plan", mimeType: "application/json", buffer: fs.readFileSync(PLAN_FILE) });
    let f = await until("the plan loaded", (f) => (f.status ?? "").includes("loaded 4 items"));
    // Its home is not the boxes', so the planner asks whether to take it, as the C# does: No.
    if (f["plan.prompt"] === "Reset Home Coords") {
      await page.keyboard.press("Escape");
      f = await until("the home question answered", (f) => f["plan.prompt"] === "none");
    }
    console.log(`status "${f.status}", mission.items ${f["mission.items"]}, commands ${f["mission.commands"]}`);
    if (f["mission.commands"] !== "22,16,16,16") fail(`the rows are ${f["mission.commands"]}, not 22,16,16,16`);
    // Kept in the browser's storage, in the box's folder, for the next visit.
    await page.waitForTimeout(1500);
    const kept = await page.evaluate(async () => {
      const found = [];
      async function walk(handle, at) {
        for await (const [name, entry] of handle.entries()) {
          if (entry.kind === "directory") await walk(entry, `${at}/${name}`);
          else if (name === "survey.plan") found.push(`${at}/${name}`);
        }
      }
      try { await walk(await (await navigator.storage.getDirectory()).getDirectoryHandle("planner"), ""); } catch (_) {}
      return found;
    });
    console.log(`kept: ${JSON.stringify(kept)}`);
    if (kept.length !== 1) fail(`survey.plan kept ${kept.length} times in the browser's storage`);
    if (!(f.status ?? "").includes(kept[0] ?? "\u0000")) fail(`loaded from "${f.status}", not the kept ${kept[0]}`);
  }

  // Save File, as each of its two kinds: a download of each.
  for (const [name, test] of [
    ["trip.waypoints", (text) => text.startsWith("QGC WPL 110") && text.trim().split("\n").length === 6],
    ["trip.mission", (text) => JSON.parse(text).mission.items.length === 4],
  ]) {
    await click("plan-save");
    await until("Save File's box", (f) => f["plan.prompt"] === "Save As");
    await page.keyboard.press("Control+A");
    await page.keyboard.type(name);
    const [download] = await Promise.all([page.waitForEvent("download", { timeout: 10000 }), page.keyboard.press("Enter")]);
    const file = path.join(out, download.suggestedFilename());
    await download.saveAs(file);
    const text = fs.readFileSync(file, "utf8");
    console.log(`downloaded ${download.suggestedFilename()}: ${text.length} bytes`);
    if (download.suggestedFilename() !== name) fail(`the download is named ${download.suggestedFilename()}, not ${name}`);
    let good = false;
    try { good = test(text); } catch (_) {}
    if (!good) fail(`${name} downloaded is not the mission saved: ${JSON.stringify(text.slice(0, 120))}`);
    await until(`${name} saved`, (f) => (f.status ?? "").includes(`saved 4 items`) && (f.status ?? "").includes(name));
  }

  for (const error of [...new Set(errors)].slice(0, 5)) console.log(`error: ${error.slice(0, 300)}`);
  if (errors.length) fail(`${errors.length} fault(s) in the page`);
  await page.screenshot({ path: `${out}/files.png` });
  await browser.close();
  if (!process.exitCode) console.log("PASS");
})();
