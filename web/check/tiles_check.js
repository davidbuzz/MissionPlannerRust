// The map's tiles in a page over https, as GitHub Pages serves it (the owner's bug of 2026-10-05:
// on https://davidbuzz.github.io/MissionPlannerRust/ no map tile drew). The planner reads Google's
// current tile versions from `http://maps.google.com/...`, as the C# does; a page over https may
// not ask over plain http, so the check failed as mixed content, the satellite tiles kept a
// version Google no longer serves, and every tile was a 404. Here: the page up, the map drawing
// tiles, and no request of the page's refused as mixed content.
//
//   NODE_PATH=<a node_modules holding playwright> node check/tiles_check.js [out-dir] [url]
//
// The url defaults to the page over https from this machine: www/serve.py 8443 cert.pem key.pem.
const { chromium } = require("playwright");
const out = process.argv[2] || ".";
const url = process.argv[3] || "https://127.0.0.1:8443/?facts=1&demo=0";
const fail = (why) => { console.log(`FAIL: ${why}`); process.exitCode = 1; };
(async () => {
  const browser = await chromium.launch({
    headless: true,
    args: ["--enable-unsafe-swiftshader", "--use-angle=swiftshader", "--ignore-gpu-blocklist"],
  });
  const context = await browser.newContext({ viewport: { width: 1400, height: 900 }, ignoreHTTPSErrors: true });
  const page = await context.newPage();
  const refused = [];
  const versions = [];
  context.on("requestfailed", (r) => { if (/mixed/i.test(r.failure()?.errorText ?? "")) refused.push(r.url()); });
  context.on("request", (r) => { if (r.url().includes("maps/api/js")) versions.push(r.url()); });
  const facts = () => page.evaluate(() => (globalThis.mpFacts ? globalThis.mpFacts() : {})).catch(() => ({}));
  await page.goto(url, { waitUntil: "load" });
  let f = {};
  for (let waited = 0; waited < 90000; waited += 1000) {
    await page.waitForTimeout(1000);
    f = await facts();
    if (Number(f["map.tiles.drawn"] ?? 0) > 0) break;
  }
  console.log(`map.source ${f["map.source"]}, tiles drawn ${f["map.tiles.drawn"]}, missing ${f["map.tiles.missing"]}, fetched ${f["map.tiles.fetched"]}`);
  console.log(`the version page asked at: ${versions.join(", ") || "none"}`);
  if (!(Number(f["map.tiles.drawn"] ?? 0) > 0)) fail("no tile drawn");
  for (const u of refused) fail(`refused as mixed content: ${u}`);
  await page.screenshot({ path: `${out}/tiles.png` });
  await browser.close();
  if (!process.exitCode) console.log("PASS");
})();
