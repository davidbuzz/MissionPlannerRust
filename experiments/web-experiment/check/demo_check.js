// The owner's Welcome-Demo-Sitl plugin (2026-10-05) in a web page, as a first visit sees it: no
// click from here at all. The demo's pointer clicks SIMULATION > Multirotor (the copter starts in
// the page), PLAN, Zoom To Vehicle, Set Home Here on the map's menu at the copter, four
// waypoints, Write, FLY > Actions, force arm, TakeOff and its OK, Auto, then unticks itself on
// PLUGINS, saves, and goes back to FLY. Fails unless the planner's own facts (?facts=1) show it
// all done: the demo's last click FLY and its pointer gone, home and four waypoints on FLIGHT
// PLAN, the vehicle armed in Auto with the mission written, and the demo on the disabled list
// Save && Close wrote. A screenshot of the
// end goes into out-dir, and with DEMO_SHOTS=1 one at each click too - each stalls the page under
// SwiftShader for seconds, long enough for ArduCopter to disarm a copter waiting for its TakeOff.
//
//   NODE_PATH=<a node_modules holding playwright> node check/demo_check.js [out-dir] [url]
const { chromium } = require("playwright");
const out = process.argv[2] || ".";
const url = process.argv[3] || "http://127.0.0.1:8080/planner.html?facts=1";
const fail = (why) => { console.log(`FAIL: ${why}`); process.exitCode = 1; };
// The demo's script (welcomedemositl.rs, SCRIPT): twenty-two clicks of its own, the last FLY - two
// more when the copter disarmed before its climb and was armed again.
const CLICKS = 22;
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
  // A screenshot waits on the page's main thread, which the simulator's start keeps busy for a
  // while: a late one is noted, not a failure.
  const shot = async (name) => {
    try {
      await page.screenshot({ path: `${out}/${name}.png`, timeout: 60000 });
    } catch (e) {
      console.log(`no screenshot ${name}: ${e.message.split("\n")[0]}`);
    }
  };
  let facts = {};
  let seen = -1;
  let lastPlan = "";
  const t0 = Date.now();
  const done = () => facts["demo.last"] === "tab-fly" && Number(facts["demo.clicks"]) >= CLICKS;
  for (let waited = 0; waited < 420000 && !done(); waited += 1000) {
    await page.waitForTimeout(1000);
    facts = await page.evaluate(() => (globalThis.mpFacts ? globalThis.mpFacts() : {}));
    const clicks = Number(facts["demo.clicks"] ?? 0);
    if (clicks !== seen) {
      seen = clicks;
      const at = ((Date.now() - t0) / 1000).toFixed(0);
      console.log(`${at} s: click ${clicks} ${facts["demo.last"] ?? "none"} (screen ${facts.screen}, mode ${facts["vehicle.mode"]}, armed ${facts["vehicle.armed"]})`);
      if (process.env.DEMO_SHOTS === "1") await shot(`demo-${String(clicks).padStart(2, "0")}`);
    }
    // What PLAN's Write did, whenever it changes: the mission, the transfer, any question.
    const plan = ["mission.items", "mission.written", "plan.transfer", "plan.prompt", "plan.prompt.text", "status"]
      .map((key) => `${key}=${facts[key] ?? "-"}`).join(" ");
    if (plan !== lastPlan) {
      lastPlan = plan;
      console.log(`  ${((Date.now() - t0) / 1000).toFixed(0)} s: ${plan}`);
    }
    const status = facts["status"] ?? "";
    if (status.startsWith("Welcome demo stopped")) break;
  }
  // A few seconds more to watch it fly.
  await page.waitForTimeout(5000);
  facts = await page.evaluate(() => (globalThis.mpFacts ? globalThis.mpFacts() : {}));
  await shot("demo-flying");
  console.log(`after ${((Date.now() - t0) / 1000).toFixed(0)} s: clicks ${facts["demo.clicks"]}, last ${facts["demo.last"]}, screen ${facts.screen}, mode ${facts["vehicle.mode"]}, armed ${facts["vehicle.armed"]}, wps ${facts["vehicle.wps"]}, mission_current ${facts["vehicle.mission_current"]}, disabled ${facts["plugins.disabled"]}, status "${facts.status}"`);
  if (!(Number(facts["demo.clicks"]) >= CLICKS)) fail(`the demo clicked ${facts["demo.clicks"]} times, not ${CLICKS} or more`);
  if (facts["demo.last"] !== "tab-fly") fail(`the demo's last click was ${facts["demo.last"]}, not tab-fly`);
  if (facts["demo.pointer"] !== "hidden") fail(`the demo's pointer is ${facts["demo.pointer"]} at its end, not hidden`);
  if (facts.screen !== "fly") fail(`the screen is ${facts.screen}, not fly`);
  if (facts["vehicle.armed"] !== "true") fail("the vehicle is not armed");
  if (facts["vehicle.mode"] !== "Auto") fail(`the vehicle is in ${facts["vehicle.mode"]}, not Auto`);
  // On FLIGHT PLAN: home set by the Home Location link, and four waypoints; on the vehicle, home
  // and the four (the owner's run of 2026-10-05 had one waypoint and no home).
  if (facts["mission.home"] !== "true") fail("FLIGHT PLAN has no home");
  if (Number(facts["mission.items"]) !== 4) fail(`FLIGHT PLAN holds ${facts["mission.items"]} waypoints, not four`);
  if (Number(facts["vehicle.wps"]) < 5) fail(`the vehicle holds ${facts["vehicle.wps"]} mission items, not home and four`);
  if (!(facts["plugins.disabled"] ?? "").includes("welcomedemositl.wasm")) fail(`the disabled list is ${facts["plugins.disabled"]}, without the demo`);
  if (facts["plugins.Welcome-Demo-Sitl.state"] !== "running") fail(`the demo is ${facts["plugins.Welcome-Demo-Sitl.state"]}, not loaded this run`);
  for (const error of [...new Set(errors)].slice(0, 5)) console.log(`error: ${error.slice(0, 300)}`);
  if (errors.length) fail(`${errors.length} error(s) in the page`);
  await browser.close();
  if (!process.exitCode) console.log("PASS");
})();
