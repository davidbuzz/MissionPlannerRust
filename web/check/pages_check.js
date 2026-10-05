// The site tools/pages-site.sh assembles, served as GitHub Pages serves it: under the project's
// path, /MissionPlannerRust/, and with no header of the site's choosing - no COOP, no COEP, which
// the page's threads need. Fails unless coi-serviceworker makes the page cross-origin isolated
// (it reloads the page once it is in charge) and the planner then starts in it: its facts there,
// the flight screen up, no panic.
//
//   tools/pages-site.sh <dir>/MissionPlannerRust
//   NODE_PATH=<a node_modules holding playwright> node check/pages_check.js <dir> [out-dir]
const http = require("http");
const fs = require("fs");
const path = require("path");
const { chromium } = require("playwright");
const root = process.argv[2];
const out = process.argv[3] || ".";
const fail = (why) => { console.log(`FAIL: ${why}`); process.exitCode = 1; };
if (!root) { console.log("usage: pages_check.js <dir holding MissionPlannerRust/>"); process.exit(2); }
const TYPES = { ".html": "text/html", ".js": "text/javascript", ".wasm": "application/wasm", ".gz": "application/gzip", ".json": "application/json", ".txt": "text/plain" };
// A static server and nothing more: the type by extension, as GitHub Pages sends it.
const served = [];
const server = http.createServer((req, res) => {
  let file = path.join(root, decodeURIComponent(new URL(req.url, "http://x").pathname));
  if (file.endsWith("/")) file += "index.html";
  served.push(req.url);
  fs.readFile(file, (err, data) => {
    if (err) { res.writeHead(404); res.end(); return; }
    res.writeHead(200, { "Content-Type": TYPES[path.extname(file)] || "application/octet-stream" });
    res.end(data);
  });
});
server.listen(0, "127.0.0.1", async () => {
  const url = `http://127.0.0.1:${server.address().port}/MissionPlannerRust/?facts=1&demo=0`;
  const browser = await chromium.launch({
    headless: true,
    args: ["--enable-unsafe-swiftshader", "--use-angle=swiftshader", "--ignore-gpu-blocklist"],
  });
  const page = await browser.newPage({ viewport: { width: 1400, height: 900 } });
  const errors = [];
  page.on("console", (m) => { const t = m.text(); if (t.startsWith("panicked") || t.startsWith("planner:")) errors.push(t.split("\n")[0]); });
  page.on("pageerror", (e) => errors.push(e.message));
  await page.goto(url, { waitUntil: "load" });
  let state = {};
  for (let waited = 0; waited < 90000; waited += 1000) {
    await page.waitForTimeout(1000);
    try {
      state = await page.evaluate(() => ({
        isolated: globalThis.crossOriginIsolated === true,
        worker: !!(navigator.serviceWorker && navigator.serviceWorker.controller),
        screen: globalThis.mpFacts ? globalThis.mpFacts().screen : undefined,
      }));
    } catch (_) {
      // The service worker's reload, mid-evaluate.
    }
    if (state.isolated && state.screen === "fly") break;
  }
  await page.screenshot({ path: `${out}/pages.png` });
  console.log(`after load: ${JSON.stringify(state)}; ${served.length} requests`);
  if (!state.worker) fail("no service worker in charge of the page");
  if (!state.isolated) fail("the page is not cross-origin isolated");
  if (state.screen !== "fly") fail(`the planner's screen is ${state.screen}, not fly`);
  for (const error of [...new Set(errors)].slice(0, 5)) console.log(`error: ${error.slice(0, 300)}`);
  if (errors.length) fail(`${errors.length} error(s) in the page`);
  await browser.close();
  server.close();
  if (!process.exitCode) console.log("PASS");
});
