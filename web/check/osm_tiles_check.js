// OpenStreetMap's tiles in a page (the owner's bug of 2026-10-06: "OpenStreetMap map tiles are
// getting 403 Access blocked in the browser"). OSM's tile servers refuse a request that carries no
// Referer (https://osm.wiki/Blocked). The planner fetches tiles on its threads, which are web
// workers; a worker made from a blob: URL sends no Referer at all - the Fetch standard strips a
// referrer of a local scheme to none - so every tile was refused. Here: FLIGHT PLAN's map set to
// OpenStreetMap, and every request to tile.openstreetmap.org carrying the page's origin as its
// Referer. The tile server is answered here, with a tile of one colour, so the check needs no
// network and asks nothing of OpenStreetMap's; MP_OSM_LIVE=1 asks the real one and wants its 200.
//
//   NODE_PATH=<a node_modules holding playwright> node check/osm_tiles_check.js [out-dir] [url]
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
  const tiles = [];
  const live = process.env.MP_OSM_LIVE === "1";
  if (!live) {
    // A 1x1 PNG, readable cross-origin as the real server's tiles are.
    const png = Buffer.from("iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mNk+M9QDwADhgGAWjR9awAAAABJRU5ErkJggg==", "base64");
    await context.route("https://tile.openstreetmap.org/**", (route) =>
      route.fulfill({ status: 200, contentType: "image/png", headers: { "access-control-allow-origin": "*", "cross-origin-resource-policy": "cross-origin" }, body: png }));
  }
  context.on("requestfinished", async (r) => {
    if (!r.url().includes("tile.openstreetmap.org")) return;
    const headers = await r.allHeaders().catch(() => ({}));
    const response = await r.response().catch(() => null);
    tiles.push({ url: r.url(), referer: headers["referer"] ?? "", status: response ? response.status() : 0 });
  });
  context.on("requestfailed", (r) => {
    if (r.url().includes("tile.openstreetmap.org")) tiles.push({ url: r.url(), referer: "?", status: 0 });
  });
  const facts = () => page.evaluate(() => (globalThis.mpFacts ? globalThis.mpFacts() : {})).catch(() => ({}));
  const until = async (what, test, ms = 30000) => {
    let f = {};
    for (let waited = 0; waited < ms; waited += 500) {
      await page.waitForTimeout(500);
      f = await facts();
      if (test(f)) return f;
    }
    fail(`${what}: screen "${f.screen}", map.source "${f["map.source"]}"`);
    return f;
  };

  await page.goto(url, { waitUntil: "load" });
  await until("the planner up", (f) => f.screen === "fly", 90000);
  await clickNamed(page, "tab-plan");
  await until("FLIGHT PLAN", (f) => f.screen === "plan");
  await clickNamed(page, "plan-map-osm");
  await until("OpenStreetMap taken", (f) => f["map.source"] === "osm");
  const f = await until("OpenStreetMap's tiles drawn", () => tiles.some((t) => t.status === 200), 45000);
  const origin = new URL(url).origin + "/";
  for (const t of tiles.slice(0, 5)) console.log(`${t.status} referer "${t.referer}" ${t.url}`);
  console.log(`tile requests ${tiles.length}, map.tiles.drawn ${f["map.tiles.drawn"]}`);
  if (tiles.length === 0) fail("no request to tile.openstreetmap.org");
  for (const t of tiles.filter((t) => t.referer !== origin).slice(0, 3)) fail(`Referer "${t.referer}", not "${origin}": ${t.url}`);
  for (const t of tiles.filter((t) => t.status !== 200).slice(0, 3)) fail(`answered ${t.status}: ${t.url}`);
  await page.screenshot({ path: `${out}/osm_tiles.png` });
  await browser.close();
  if (!process.exitCode) console.log("PASS");
})();
