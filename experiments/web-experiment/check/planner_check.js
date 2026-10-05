// The whole planner's test in a browser: loads www/planner.html (crates/mp-gui built by
// tools/planner-wasm.sh build and wasm-bindgen into www/pkg-planner) in headless Chromium, connects
// the way a pilot does - the port box's TCP, CONNECT, the host and the port Mission Planner asks
// for, both left as they are - and fails unless the link the planner opens reaches the SITL in the
// page both ways, with no error in the page: no panic, and no wait on the page's main thread.
//
//   NODE_PATH=<a node_modules holding playwright> node check/planner_check.js [out-dir] [url]
const { chromium } = require("playwright");
const out = process.argv[2] || ".";
const url = process.argv[3] || "http://127.0.0.1:8080/planner.html?vehicle=copter&demo=0";
const fail = (why) => { console.log(`FAIL: ${why}`); process.exitCode = 1; };
// Where the window puts the port box, its TCP entry and CONNECT at 1400x900.
const PORT_BOX = [996, 36], TCP = [964, 97], CONNECT = [1187, 36];
(async () => {
  const browser = await chromium.launch({
    headless: true,
    args: ["--enable-unsafe-swiftshader", "--use-angle=swiftshader", "--ignore-gpu-blocklist", "--js-flags=--stack-trace-limit=200"],
  });
  const page = await browser.newPage({ viewport: { width: 1400, height: 900 } });
  const errors = [];
  page.on("console", (m) => {
    const text = m.text();
    if (text.startsWith("panicked")) errors.push(text.split("\n").slice(0, 2).join(" "));
  });
  page.on("pageerror", (e) => errors.push(e.message));
  await page.goto(url, { waitUntil: "load" });
  await page.waitForTimeout(8000);
  // The built-in Drone ID plugin asks its question at every start in a page, which keeps no
  // settings yet: its OK, over the flight screen's map at 1400x900 (a click on the map otherwise).
  await page.mouse.click(854, 484);
  await page.waitForTimeout(800);
  await page.mouse.click(...PORT_BOX);
  await page.waitForTimeout(800);
  await page.mouse.click(...TCP);
  await page.waitForTimeout(800);
  await page.mouse.click(...CONNECT);
  await page.waitForTimeout(1500);
  await page.keyboard.press("Enter"); // the host: 127.0.0.1
  await page.waitForTimeout(1500);
  await page.keyboard.press("Enter"); // the port: 5760
  let crossed = {};
  for (let waited = 0; waited < 45000; waited += 1000) {
    await page.waitForTimeout(1000);
    crossed = await page.evaluate(() => (globalThis.mpLinkCrossed ? globalThis.mpLinkCrossed() : {}));
    if (crossed.toPlanner > 50000 && crossed.fromPlanner > 1000) break;
  }
  await page.screenshot({ path: `${out}/planner-connected.png` });
  console.log(JSON.stringify(crossed));
  if (!(crossed.asked ?? []).includes("tcp:127.0.0.1:5760")) fail(`the planner asked for ${JSON.stringify(crossed.asked)}`);
  if (!(crossed.toPlanner > 50000)) fail(`only ${crossed.toPlanner} bytes reached the planner`);
  if (!(crossed.fromPlanner > 1000)) fail(`only ${crossed.fromPlanner} bytes came from the planner`);
  for (const error of [...new Set(errors)].slice(0, 5)) console.log(`error: ${error.slice(0, 300)}`);
  if (errors.length) fail(`${errors.length} error(s) in the page`);
  await browser.close();
  if (!process.exitCode) console.log("PASS");
})();
