// The planner's files kept across a reload (the owner's request of 2026-10-05: settings,
// missions and logs to survive one). A fresh browser profile visits the page: the planner's
// start-up save of config.xml ("to test we have write access", MainV2's) must reach the browser's
// storage (OPFS, through www/storage.js). The stored config.xml is then given a default altitude
// of 123 and the page reloaded: the planner must read it back (its fact config.TXT_DefaultAlt),
// with no error saving, and its own save must keep it.
//
//   NODE_PATH=<a node_modules holding playwright> node check/storage_check.js [out-dir] [url]
const { chromium } = require("playwright");
const out = process.argv[2] || ".";
const url = process.argv[3] || "http://127.0.0.1:8080/planner.html?facts=1&demo=0";
const fail = (why) => { console.log(`FAIL: ${why}`); process.exitCode = 1; };
const CONFIG = "home/web/.local/share/MissionPlannerRust/config.xml";

// In the page: the stored files' paths, and config.xml's text if kept.
const stored = () => (async (config) => {
  const paths = [];
  let text = null;
  try {
    const top = await navigator.storage.getDirectory();
    const root = await top.getDirectoryHandle("planner");
    const walk = async (dir, at) => {
      for await (const [name, entry] of dir.entries()) {
        const path = at ? `${at}/${name}` : name;
        if (entry.kind === "directory") await walk(entry, path);
        else {
          paths.push(path);
          if (path === config) text = await (await entry.getFile()).text();
        }
      }
    };
    await walk(root, "");
  } catch (_) {}
  return { paths, text };
});

async function planner(page, until) {
  let facts = {};
  for (let waited = 0; waited < 90000; waited += 1000) {
    await page.waitForTimeout(1000);
    facts = await page.evaluate(() => (globalThis.mpFacts ? globalThis.mpFacts() : {})).catch(() => ({}));
    if (facts.screen === "fly" && until(facts)) break;
  }
  return facts;
}

(async () => {
  const browser = await chromium.launch({
    headless: true,
    args: ["--enable-unsafe-swiftshader", "--use-angle=swiftshader", "--ignore-gpu-blocklist"],
  });
  // A context of its own: an empty profile, as a first visit has.
  const context = await browser.newContext({ viewport: { width: 1400, height: 900 } });
  const page = await context.newPage();
  const errors = [];
  page.on("console", (m) => { const t = m.text(); if (t.startsWith("panicked") || t.startsWith("storage:") || t.startsWith("planner:")) errors.push(t.split("\n")[0]); });
  page.on("pageerror", (e) => errors.push(e.message));

  await page.goto(url, { waitUntil: "load" });
  let facts = await planner(page, (f) => Number(f["config.saves"]) >= 1);
  console.log(`first visit: screen ${facts.screen}, saves ${facts["config.saves"]} (${facts["config.saved"]}), error ${facts["config.error"]}`);
  if (Number(facts["config.saves"]) < 1) fail(`the planner saved config.xml ${facts["config.saves"]} times`);
  if (facts["config.error"] !== "none") fail(`saving config.xml: ${facts["config.error"]}`);
  // A flush or two for it to reach the browser's storage.
  await page.waitForTimeout(1500);
  await page.evaluate(() => globalThis.mpStorageFlushed && globalThis.mpStorageFlushed());
  let kept = await page.evaluate(stored(), CONFIG);
  console.log(`stored: ${kept.paths.join(", ")}`);
  if (!kept.text || !kept.text.includes("<Config")) fail("config.xml is not in the browser's storage");

  // A default altitude the planner never set, put in the stored file, for the next visit to read.
  if (kept.text) {
    await page.evaluate(async ({ config, text }) => {
      const parts = config.split("/");
      const name = parts.pop();
      let dir = await (await navigator.storage.getDirectory()).getDirectoryHandle("planner");
      for (const part of parts) dir = await dir.getDirectoryHandle(part);
      const writable = await (await dir.getFileHandle(name)).createWritable();
      await writable.write(text.replace("</Config>", "<TXT_DefaultAlt>123</TXT_DefaultAlt></Config>"));
      await writable.close();
    }, { config: CONFIG, text: kept.text });
  }

  await page.reload({ waitUntil: "load" });
  facts = await planner(page, (f) => Number(f["config.saves"]) >= 1);
  console.log(`second visit: default altitude ${facts["config.TXT_DefaultAlt"]}, saves ${facts["config.saves"]}, error ${facts["config.error"]}`);
  if (facts["config.TXT_DefaultAlt"] !== "123") fail(`the planner read a default altitude of ${facts["config.TXT_DefaultAlt"]}, not the stored 123`);
  if (facts["config.error"] !== "none") fail(`saving config.xml: ${facts["config.error"]}`);
  await page.waitForTimeout(1500);
  await page.evaluate(() => globalThis.mpStorageFlushed && globalThis.mpStorageFlushed());
  kept = await page.evaluate(stored(), CONFIG);
  if (!kept.text || !kept.text.includes("123")) fail("the planner's own save lost the stored default altitude");

  for (const error of [...new Set(errors)].slice(0, 5)) console.log(`error: ${error.slice(0, 300)}`);
  if (errors.length) fail(`${errors.length} error(s) in the page`);
  await page.screenshot({ path: `${out}/storage.png` });
  await browser.close();
  if (!process.exitCode) console.log("PASS");
})();
