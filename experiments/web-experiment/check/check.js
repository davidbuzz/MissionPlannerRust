// The experiment's test: loads the page in headless Chromium (Playwright), and fails unless the page
// is cross-origin isolated, raises no error, draws into a canvas, and redraws the button a click hits.
//
//   NODE_PATH=<a node_modules holding playwright> node check/check.js [out-dir] [url]
const { chromium } = require("playwright");
const out = process.argv[2] || ".";
const url = process.argv[3] || "http://127.0.0.1:8080/";
const fail = (why) => { console.log(`FAIL: ${why}`); process.exitCode = 1; };
(async () => {
  const browser = await chromium.launch({
    headless: true,
    args: ["--enable-unsafe-swiftshader", "--use-angle=swiftshader", "--ignore-gpu-blocklist"],
  });
  const page = await browser.newPage({ viewport: { width: 1280, height: 800 } });
  const errors = [];
  page.on("console", (m) => console.log(`console.${m.type()}: ${m.text().slice(0, 300)}`));
  page.on("pageerror", (e) => { errors.push(e.message); console.log(`pageerror: ${e.message}`); });
  await page.goto(url, { waitUntil: "load" });
  if (!(await page.evaluate(() => self.crossOriginIsolated))) fail("the page is not cross-origin isolated");
  await page.waitForTimeout(8000);
  const canvases = await page.evaluate(() => document.querySelectorAll("canvas").length);
  if (canvases === 0) fail("no canvas");
  // The button sits just below the middle of the window (src/lib.rs centres the column).
  const button = { x: 560, y: 405, width: 160, height: 55 };
  const before = await page.screenshot({ path: `${out}/web-1.png`, clip: button });
  await page.mouse.click(640, 432);
  await page.waitForTimeout(1500);
  const after = await page.screenshot({ path: `${out}/web-2.png`, clip: button });
  if (before.equals(after)) fail("the click did not redraw the button");
  if (errors.length) fail(`${errors.length} page error(s)`);
  await browser.close();
  if (!process.exitCode) console.log("PASS");
})();
