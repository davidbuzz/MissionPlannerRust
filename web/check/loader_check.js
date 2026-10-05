// The loading screen (the owner's request, 2026-10-06: "a first-page-is-still-loading spinner
// widget that gets shown while the wasm is still being downloaded, esp on slow links like mobile
// phones"). Here the page is loaded over a link held to a phone's speed: the screen must show
// from the start, count the planner's download up as it comes ("Downloading the planner - N%"),
// and be gone once the planner is up, its flight screen showing.
//
//   NODE_PATH=<a node_modules holding playwright> node check/loader_check.js [out-dir] [url]
const { chromium } = require("playwright");
const out = process.argv[2] || ".";
const url = process.argv[3] || "http://127.0.0.1:8080/?facts=1&demo=0";
const fail = (why) => { console.log(`FAIL: ${why}`); process.exitCode = 1; };
// 6 MB a second: the planner's 87 MB in some fifteen seconds, long enough to watch it count.
const THROUGHPUT = 6 * 1048576;
(async () => {
  const browser = await chromium.launch({
    headless: true,
    args: ["--enable-unsafe-swiftshader", "--use-angle=swiftshader", "--ignore-gpu-blocklist"],
  });
  const context = await browser.newContext({ viewport: { width: 412, height: 915 }, isMobile: true, hasTouch: true });
  const page = await context.newPage();
  const cdp = await context.newCDPSession(page);
  await cdp.send("Network.enable");
  await cdp.send("Network.emulateNetworkConditions", {
    offline: false, latency: 80, downloadThroughput: THROUGHPUT, uploadThroughput: THROUGHPUT / 4,
  });
  const loader = () => page.evaluate(() => {
    const element = document.getElementById("mpr-loader");
    return element ? { shown: !element.classList.contains("done"), phase: element.querySelector(".phase").textContent } : null;
  }).catch(() => null);
  await page.goto(url, { waitUntil: "commit" });
  const percents = [];
  let shot = false;
  let state = null;
  for (let waited = 0; waited < 180000; waited += 500) {
    await page.waitForTimeout(500);
    state = await loader();
    const percent = state?.phase.match(/Downloading the planner — (\d+)%/);
    if (percent) {
      percents.push(Number(percent[1]));
      if (!shot && Number(percent[1]) >= 30) {
        await page.screenshot({ path: `${out}/loader.png` });
        console.log(`mid-download: "${state.phase}"`);
        shot = true;
      }
    }
    const facts = await page.evaluate(() => (globalThis.mpFacts ? globalThis.mpFacts() : {})).catch(() => ({}));
    if (facts.screen === "fly" && (state === null || !state.shown)) break;
  }
  console.log(`percentages seen: ${[...new Set(percents)].join(" ")}`);
  if (percents.length < 3) fail("the download was not counted up on the loading screen");
  if (percents.some((value, index) => index > 0 && value < percents[index - 1])) fail("the percentage went down");
  if (state && state.shown) fail(`the loading screen is still up: "${state.phase}"`);
  await page.waitForTimeout(1500);
  await page.screenshot({ path: `${out}/planner-up.png` });
  if (await page.$("#mpr-loader")) fail("the loading screen was not taken away");
  await browser.close();
  if (!process.exitCode) console.log("PASS");
})();
