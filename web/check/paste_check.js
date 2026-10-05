// Paste into a box in a page (the owner's bug of 2026-10-05: text copied on the desktop would not
// paste into a text box in the browser). A page cannot read the clipboard when it likes; the
// browser hands the text over in its own paste event, which only follows a Ctrl+V the page lets
// through. Here: CONNECT on TCP as a pilot does, the host question's text replaced by a paste of
// the clipboard's, the port typed - so typing stays single - and the link must be opened on what
// was pasted, the page fault-free.
//
//   NODE_PATH=<a node_modules holding playwright> node check/paste_check.js [out-dir] [url]
const { chromium } = require("playwright");
const { clickNamed } = require("./clicks");
const out = process.argv[2] || ".";
const url = process.argv[3] || "http://127.0.0.1:8080/?facts=1&demo=0";
const fail = (why) => { console.log(`FAIL: ${why}`); process.exitCode = 1; };
const HOST = "localhost";
(async () => {
  const browser = await chromium.launch({
    headless: true,
    args: ["--enable-unsafe-swiftshader", "--use-angle=swiftshader", "--ignore-gpu-blocklist"],
  });
  const context = await browser.newContext({ viewport: { width: 1400, height: 900 } });
  await context.grantPermissions(["clipboard-read", "clipboard-write"], { origin: new URL(url).origin });
  const page = await context.newPage();
  const errors = [];
  page.on("console", (m) => { const t = m.text(); if (t.startsWith("panicked") || t.startsWith("planner fault")) errors.push(t.split("\n")[0]); });
  page.on("pageerror", (e) => errors.push(e.message));
  const facts = () => page.evaluate(() => (globalThis.mpFacts ? globalThis.mpFacts() : {})).catch(() => ({}));
  const until = async (what, test, ms) => {
    let f = {};
    for (let waited = 0; waited < ms; waited += 500) {
      await page.waitForTimeout(500);
      f = await facts();
      if (test(f)) return f;
    }
    fail(`${what}: link.prompt "${f["link.prompt"]}", link.target "${f["link.target"]}", status "${f.status}"`);
    return f;
  };
  await page.goto(url, { waitUntil: "load" });
  await until("the planner up", (f) => f.screen === "fly", 60000);
  // What the browser made of the Ctrl+V and whether it pasted, seen after the planner's handlers.
  await page.evaluate(() => {
    globalThis.pasteSeen = { prevented: null, pastes: 0 };
    addEventListener("keydown", (e) => {
      if (e.key.toLowerCase() === "v" && (e.ctrlKey || e.metaKey)) globalThis.pasteSeen.prevented = e.defaultPrevented;
    });
    addEventListener("paste", () => { globalThis.pasteSeen.pastes += 1; }, true);
  });
  await clickNamed(page, "main-port"); await page.waitForTimeout(800);
  await clickNamed(page, "main-port-TCP"); await page.waitForTimeout(800);
  await clickNamed(page, "main-connect");
  await until("the host question", (f) => f["link.prompt"] && f["link.prompt"] !== "none", 15000);
  await page.evaluate((text) => navigator.clipboard.writeText(text), HOST);
  await page.keyboard.press("Control+A");
  await page.keyboard.press("Control+V");
  await page.waitForTimeout(500);
  let seen = await page.evaluate(() => globalThis.pasteSeen);
  if (seen.prevented !== false) fail(`the page kept Ctrl+V from the browser (defaultPrevented ${seen.prevented})`);
  if (seen.pastes === 0) {
    // A headless browser may not paste on the keys; the paste it would have sent, then.
    console.log("the browser sent no paste on Ctrl+V; sending its paste event");
    await page.evaluate((text) => {
      const data = new DataTransfer();
      data.setData("text/plain", text);
      document.activeElement.dispatchEvent(new ClipboardEvent("paste", { clipboardData: data, bubbles: true, cancelable: true }));
    }, HOST);
    await page.waitForTimeout(500);
  }
  seen = await page.evaluate(() => globalThis.pasteSeen);
  console.log(`Ctrl+V: defaultPrevented ${seen.prevented}, paste events ${seen.pastes}`);
  await page.screenshot({ path: `${out}/paste-host.png` });
  await page.keyboard.press("Enter"); await page.waitForTimeout(1000);
  await page.keyboard.press("Control+A");
  await page.keyboard.type("5760");
  await page.keyboard.press("Enter");
  const f = await until("the link opened on the pasted host", (f) => (f["link.target"] ?? "").includes(HOST), 20000);
  console.log(`link.target "${f["link.target"]}", status "${f.status}"`);
  if (!(f["link.target"] ?? "").includes(`${HOST}:5760`)) fail(`the link was not opened on ${HOST}:5760: "${f["link.target"]}"`);
  for (const error of [...new Set(errors)].slice(0, 5)) console.log(`error: ${error.slice(0, 300)}`);
  if (errors.length) fail(`${errors.length} fault(s) in the page`);
  await page.screenshot({ path: `${out}/paste.png` });
  await browser.close();
  if (!process.exitCode) console.log("PASS");
})();
