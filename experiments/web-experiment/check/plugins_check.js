// The planner's built-in plugins in a web page: wasmtime's Pulley interpreter running the ten
// plugins mp-gui's build script compiled to Pulley bytecode (crates/mp-plugin-host/src/web.rs).
// Fails unless the planner's own facts (planner.html's ?facts=1, MP_FACTS's in a page) count the
// ten loaded, and the page has no error.
//
//   NODE_PATH=<a node_modules holding playwright> node check/plugins_check.js [out-dir] [url]
const { chromium } = require("playwright");
const out = process.argv[2] || ".";
const url = process.argv[3] || "http://127.0.0.1:8080/planner.html?facts=1&demo=0";
const fail = (why) => { console.log(`FAIL: ${why}`); process.exitCode = 1; };
// The plugins Mission Planner ships, built into the planner (crates/mp-gui/build.rs, SHIPPED).
const SHIPPED = 10;
(async () => {
  const browser = await chromium.launch({
    headless: true,
    args: ["--enable-unsafe-swiftshader", "--use-angle=swiftshader", "--ignore-gpu-blocklist", "--js-flags=--stack-trace-limit=200"],
  });
  const page = await browser.newPage({ viewport: { width: 1400, height: 900 } });
  const errors = [];
  page.on("console", (m) => { const t = m.text(); if (t.startsWith("panicked")) errors.push(t.split("\n").slice(0, 2).join(" ")); });
  page.on("pageerror", (e) => errors.push(e.message));
  await page.goto(url, { waitUntil: "load" });
  // What tests/gui/plugins-shipped.gui expects on the desktop, as the C# loads them: seven run,
  // three refuse in their Init.
  const RUNNING = ["FenceDist", "Small_stuff", "MapIconDesc", "Open_Drone_ID", "Dowding", "TerrainMakerPlugin", "Anonymize_Binlog"];
  const REFUSED = ["example.wasm", "modechange.wasm", "persistentsimple.wasm"];
  const settled = (facts) =>
    RUNNING.every((name) => facts[`plugins.${name}.state`] === "running") &&
    REFUSED.every((file) => facts[`plugins.${file}.state`] === "not-loaded");
  let facts = {};
  const t0 = Date.now();
  for (let waited = 0; waited < 90000 && !settled(facts); waited += 1000) {
    await page.waitForTimeout(1000);
    facts = await page.evaluate(() => (globalThis.mpFacts ? globalThis.mpFacts() : {}));
  }
  console.log(`settled after ${((Date.now() - t0) / 1000).toFixed(0)} s`);
  await page.screenshot({ path: `${out}/plugins.png` });
  if (Number(facts["plugins.count"]) !== SHIPPED) fail(`plugins.count is ${facts["plugins.count"]}, not ${SHIPPED}`);
  for (const name of RUNNING) {
    const state = facts[`plugins.${name}.state`];
    if (state !== "running") fail(`${name} is ${state ?? "not reported"}, not running`);
  }
  for (const file of REFUSED) {
    const state = facts[`plugins.${file}.state`];
    if (state !== "not-loaded") fail(`${file} is ${state ?? "not reported"}, not not-loaded`);
  }
  for (const error of [...new Set(errors)].slice(0, 5)) console.log(`error: ${error.slice(0, 300)}`);
  if (errors.length) fail(`${errors.length} error(s) in the page`);
  await browser.close();
  if (!process.exitCode) console.log("PASS");
})();
