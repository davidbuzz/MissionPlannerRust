// DISCONNECT in a page (the owner's bug of 2026-10-05: after DISCONNECT the page took no more
// input, and the next start reported "RefCell already borrowed"). Closing the link joined its
// thread, and a page's main thread may not wait - the wait trapped mid-click, leaving gpui's
// state borrowed for good. Here: connect to the SITL in the page as a pilot does, DISCONNECT,
// CONNECT again; the planner must follow each, the page fault-free, and the next start ask about
// no crash.
//
//   NODE_PATH=<a node_modules holding playwright> node check/disconnect_check.js [out-dir] [url]
const { chromium } = require("playwright");
const { clickNamed } = require("./clicks");
const out = process.argv[2] || ".";
const url = process.argv[3] || "http://127.0.0.1:8080/?facts=1&demo=0";
const fail = (why) => { console.log(`FAIL: ${why}`); process.exitCode = 1; };
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
  const until = async (what, test, ms) => {
    let f = {};
    for (let waited = 0; waited < ms; waited += 1000) {
      await page.waitForTimeout(1000);
      f = await facts();
      if (test(f)) return f;
    }
    fail(`${what}: link.button ${f["link.button"]}, status "${f.status}"`);
    return f;
  };
  await page.goto(url, { waitUntil: "load" });
  await until("the planner up", (f) => f.screen === "fly", 60000);
  const connect = async () => {
    await clickNamed(page, "main-port"); await page.waitForTimeout(800);
    await clickNamed(page, "main-port-TCP"); await page.waitForTimeout(800);
    await clickNamed(page, "main-connect"); await page.waitForTimeout(1500);
    await page.keyboard.press("Enter"); await page.waitForTimeout(1500);
    await page.keyboard.press("Enter");
  };
  await connect();
  await until("connected", (f) => f["link.button"] === "DISCONNECT" && Number(f.frames ?? f["link.frames"] ?? 0) > 0 || f["link.button"] === "DISCONNECT", 60000);
  await page.waitForTimeout(5000);
  console.log("connected; DISCONNECT");
  await clickNamed(page, "main-connect");
  let f = await until("disconnected", (f) => f["link.button"] === "CONNECT", 15000);
  console.log(`after DISCONNECT: link.button ${f["link.button"]}, status "${f.status}"`);
  // The page still takes input: CONNECT again, and connected again.
  await connect();
  f = await until("connected again", (f) => f["link.button"] === "DISCONNECT", 60000);
  console.log(`after CONNECT again: link.button ${f["link.button"]}, status "${f.status}"`);
  // And the next start has nothing to report.
  await page.waitForTimeout(1500);
  await page.reload({ waitUntil: "load" });
  f = await until("the planner up again", (f) => f.screen === "fly", 60000);
  if (f["crash.flow"] !== "idle") fail(`the next start asks about a crash: ${(f["crash.text"] ?? "").slice(0, 200)}`);
  for (const error of [...new Set(errors)].slice(0, 5)) console.log(`error: ${error.slice(0, 300)}`);
  if (errors.length) fail(`${errors.length} fault(s) in the page`);
  await page.screenshot({ path: `${out}/disconnect.png` });
  await browser.close();
  if (!process.exitCode) console.log("PASS");
})();
