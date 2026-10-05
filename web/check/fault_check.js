// A fault of the page's own - what a WebAssembly fault in the planner is, ending it without
// Rust's panic hook - is kept as a crash report (www/storage.js, keepFaults), and the next start
// asks about it as about a panic's (crates/mp-gui/src/crash.rs). Here a test error thrown in the
// page, then a reload: the planner's crash question must carry the error's own words.
//
//   NODE_PATH=<a node_modules holding playwright> node check/fault_check.js [out-dir] [url]
const { chromium } = require("playwright");
const out = process.argv[2] || ".";
const url = process.argv[3] || "http://127.0.0.1:8080/?facts=1&demo=0";
const fail = (why) => { console.log(`FAIL: ${why}`); process.exitCode = 1; };
const facts = (page) => page.evaluate(() => (globalThis.mpFacts ? globalThis.mpFacts() : {})).catch(() => ({}));
(async () => {
  const browser = await chromium.launch({
    headless: true,
    args: ["--enable-unsafe-swiftshader", "--use-angle=swiftshader", "--ignore-gpu-blocklist"],
  });
  const context = await browser.newContext({ viewport: { width: 1400, height: 900 } });
  const page = await context.newPage();
  await page.goto(url, { waitUntil: "load" });
  let f = {};
  for (let waited = 0; waited < 60000 && f.screen !== "fly"; waited += 1000) {
    await page.waitForTimeout(1000);
    f = await facts(page);
  }
  if (f["crash.pending"] !== "0") fail(`a fresh profile has ${f["crash.pending"]} crash reports`);
  // A fault outside anything that catches it, as a trap in the planner's callback is.
  await page.evaluate(() => setTimeout(() => { throw new Error("fault_check's test fault"); }, 0));
  await page.waitForTimeout(3000);
  await page.reload({ waitUntil: "load" });
  f = {};
  for (let waited = 0; waited < 60000 && f.screen !== "fly"; waited += 1000) {
    await page.waitForTimeout(1000);
    f = await facts(page);
  }
  console.log(`after the reload: crash.flow ${f["crash.flow"]}, pending ${f["crash.pending"]}, text "${(f["crash.text"] ?? "").slice(0, 160)}"`);
  if (f["crash.flow"] !== "question") fail(`the planner asks nothing about the fault (crash.flow ${f["crash.flow"]})`);
  if (!(f["crash.text"] ?? "").includes("fault_check's test fault")) fail("the question does not carry the fault's words");
  await page.screenshot({ path: `${out}/fault.png` });
  await browser.close();
  if (!process.exitCode) console.log("PASS");
})();
