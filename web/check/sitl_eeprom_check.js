// The in-page SITL's parameters kept between visits (web/README.md's storage, the browser build's
// row of NOT_DONE_YET_MATRIX.md: "the in-page SITL's eeprom.bin"). ArduPilot's SITL keeps its
// parameters in eeprom.bin in its working directory; in a page that is the worker's in-memory
// filesystem, gone with it. The page now keeps the file in the browser's storage under the
// vehicle's SITL folder - the folder the planner names in its start, where the desktop's bridge
// keeps the same file on disk - and puts it back before the next start.
//
// Here: SIMULATION and Multirotor as a pilot clicks them; once connected, eeprom.bin is in the
// browser's storage under the folder the start named; the page reloaded and the copter started
// again, the worker says it restored it. Fails otherwise, or on any error.
//
//   NODE_PATH=<a node_modules holding playwright> node check/sitl_eeprom_check.js [out-dir] [url]
const { chromium } = require("playwright");
const out = process.argv[2] || ".";
const url = process.argv[3] || "http://127.0.0.1:8080/?facts=1&demo=0";
const fail = (why) => { console.log(`FAIL: ${why}`); process.exitCode = 1; };
// At 1400x900: the SIMULATION tab, and the Multirotor picture.
const SIMULATION = [487, 43], MULTIROTOR = [736, 805];
(async () => {
  const browser = await chromium.launch({
    headless: true,
    args: ["--enable-unsafe-swiftshader", "--use-angle=swiftshader", "--ignore-gpu-blocklist"],
  });
  const context = await browser.newContext({ viewport: { width: 1400, height: 900 } });
  const page = await context.newPage();
  const errors = [];
  const restored = [];
  page.on("console", (m) => {
    const t = m.text();
    if (t.startsWith("panicked") || t.startsWith("planner fault")) errors.push(t.split("\n")[0]);
    if (t.includes("eeprom.bin restored")) restored.push(t);
  });
  page.on("pageerror", (e) => errors.push(e.message));
  const facts = () => page.evaluate(() => (globalThis.mpFacts ? globalThis.mpFacts() : {})).catch(() => ({}));
  const startCopter = async () => {
    for (let waited = 0; waited < 60000; waited += 1000) {
      await page.waitForTimeout(1000);
      if ((await facts()).screen === "fly") break;
    }
    await page.mouse.click(...SIMULATION);
    await page.waitForTimeout(3000);
    await page.mouse.click(...MULTIROTOR);
    let crossed = {};
    for (let waited = 0; waited < 60000; waited += 1000) {
      await page.waitForTimeout(1000);
      crossed = await page.evaluate(() => (globalThis.mpLinkCrossed ? globalThis.mpLinkCrossed() : {}));
      if (crossed.toPlanner > 50000 && crossed.fromPlanner > 1000) break;
    }
    return crossed;
  };
  // Every eeprom.bin the browser keeps for the page, by its path.
  const kept = () => page.evaluate(async () => {
    const found = [];
    async function walk(handle, path) {
      for await (const [name, entry] of handle.entries()) {
        const at = `${path}/${name}`;
        if (entry.kind === "directory") await walk(entry, at);
        else if (name === "eeprom.bin") found.push([at, (await entry.getFile()).size]);
      }
    }
    try {
      await walk(await (await navigator.storage.getDirectory()).getDirectoryHandle("planner"), "");
    } catch (_) {}
    return found;
  });

  await page.goto(url, { waitUntil: "load" });
  const crossed = await startCopter();
  const start = (crossed.asked ?? []).find((a) => a.startsWith("sitl\n")) ?? "";
  const folder = start.split("\n")[2] ?? "";
  console.log(`started: module ${start.split("\n")[1]}, folder "${folder}"; ${crossed.toPlanner} bytes to the planner`);
  if (!folder.startsWith("/")) fail(`the start named no folder: ${JSON.stringify(start.slice(0, 200))}`);
  if (!(crossed.toPlanner > 50000)) fail(`only ${crossed.toPlanner} bytes reached the planner`);
  // A save every half second: a moment for the first.
  await page.waitForTimeout(3000);
  const files = await kept();
  console.log(`kept: ${JSON.stringify(files)}`);
  const file = files.find(([path]) => path === `${folder}/eeprom.bin`);
  if (!file) fail(`no eeprom.bin kept under ${folder}`);
  else if (!(file[1] > 0)) fail(`the kept eeprom.bin is empty`);

  // The next visit: the copter started again starts from it.
  await page.reload({ waitUntil: "load" });
  await startCopter();
  for (let waited = 0; waited < 10000 && restored.length === 0; waited += 500) await page.waitForTimeout(500);
  console.log(`restored: ${restored.join(" | ").slice(0, 200) || "none"}`);
  if (restored.length === 0) fail("the second start restored no eeprom.bin");
  for (const error of [...new Set(errors)].slice(0, 5)) console.log(`error: ${error.slice(0, 300)}`);
  if (errors.length) fail(`${errors.length} fault(s) in the page`);
  await page.screenshot({ path: `${out}/sitl-eeprom.png` });
  await browser.close();
  if (!process.exitCode) console.log("PASS");
})();
