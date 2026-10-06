// A phone's screen (the owner's report of 2026-10-06: the published page on an Android phone was
// laid out at the phone's width, controls cut off, and showed the red CUT OFF bar). An Android
// phone as Chromium emulates one - Playwright's Pixel 7: 412x915 CSS pixels, mobile, touch,
// Android's user agent; no Android itself is needed for the layout. The page lays the
// planner out at no less than 1280x800 (index.html), the size crates/mp-gui/tests/layout.rs
// proves nothing is cut off at, so the phone scrolls to it. What is checked:
//
// * the planner is laid out at 1280x800 or more, wider than the phone, so the page scrolls;
// * nothing is cut off at that size (layout.hidden 0), and no red strip (layout.banner none);
// * each control the flight screen must never lose, scrolled to, is wholly on the phone's screen;
// * a finger's pan scrolls the page (gpui's canvas says touch-action: none; the page lets pans
//   through), and a tap on a tab, scrolled to, still opens it;
// * the page as it was - the canvas the phone's own size - has controls cut off, and still no
//   strip: a release build draws it nowhere, whatever turned its probe on (here ?facts=1).
//
//   NODE_PATH=<a node_modules holding playwright> node check/phone_check.js [out-dir] [url]
const { chromium, devices } = require("playwright");
const { placeOf } = require("./clicks");
const out = process.argv[2] || ".";
const url = process.argv[3] || "http://127.0.0.1:8080/?facts=1&demo=0";
const fail = (why) => { console.log(`FAIL: ${why}`); process.exitCode = 1; };
// The flight screen's controls an operator must never lose (crates/mp-gui/src/layout_guard.rs).
const IMPORTANT = ["main-port", "main-connect", "fly-tabs", "fly-column", "map"];
(async () => {
  const browser = await chromium.launch({
    headless: true,
    args: ["--enable-unsafe-swiftshader", "--use-angle=swiftshader", "--ignore-gpu-blocklist"],
  });
  // Pixel 7, but at a pixel ratio of 1: headless Chromium does not scale a canvas's device-pixel
  // box by an emulated ratio (a 1280x839 canvas reports 1280x839 device pixels at 2.625, and at 2
  // on a desktop's window too), and gpui, which divides that box by devicePixelRatio, would lay
  // the planner out at 1/2.625 of its canvas. A phone's own browser reports its real pixels.
  const phone = { ...devices["Pixel 7"], deviceScaleFactor: 1 };
  const context = await browser.newContext(phone);
  const page = await context.newPage();
  page.on("pageerror", (error) => fail(`page error: ${error.message}`));
  const facts = () => page.evaluate(() => (globalThis.mpFacts ? globalThis.mpFacts() : {})).catch(() => ({}));
  const until = async (what, test, ms = 30000) => {
    let f = {};
    for (let waited = 0; waited < ms; waited += 500) {
      await page.waitForTimeout(500);
      f = await facts();
      if (test(f)) return f;
    }
    fail(`${what}: screen "${f.screen}", layout.hidden "${f["layout.hidden"]}" (${f["layout.hidden.names"]}), layout.banner "${f["layout.banner"]}"`);
    return f;
  };
  const geometry = () => page.evaluate(() => {
    const canvas = document.querySelector("body > canvas");
    const rect = canvas ? canvas.getBoundingClientRect() : { left: 0, top: 0, width: 0, height: 0 };
    const doc = document.scrollingElement;
    return {
      canvas: { left: rect.left, top: rect.top, width: rect.width, height: rect.height },
      view: { width: innerWidth, height: innerHeight },
      scroll: { x: scrollX, y: scrollY, width: doc.scrollWidth, height: doc.scrollHeight },
    };
  });
  // A control's rectangle on the phone's screen, from its place on the canvas.
  const onScreen = async (name) => {
    const place = await page.evaluate((name) => (globalThis.mpProbe ? globalThis.mpProbe()[name] : undefined), name);
    if (!place) return null;
    const g = await geometry();
    const left = g.canvas.left + place.x - place.width / 2;
    const top = g.canvas.top + place.y - place.height / 2;
    return { left, top, right: left + place.width, bottom: top + place.height, view: g.view, place };
  };
  const scrollToPlace = (place) =>
    page.evaluate(([x, y]) => window.scrollTo(x - innerWidth / 2, y - innerHeight / 2), [place.x, place.y]);
  const inside = (r) => r.left >= -0.5 && r.top >= -0.5 && r.right <= r.view.width + 0.5 && r.bottom <= r.view.height + 0.5;

  await page.goto(url, { waitUntil: "load" });
  await until("the planner up", (f) => f.screen === "fly", 90000);
  // gpui follows the canvas's size a frame or two after it gets it.
  await page.waitForTimeout(1000);

  const g = await geometry();
  console.log(`phone ${g.view.width}x${g.view.height}, canvas ${g.canvas.width}x${g.canvas.height}, page ${g.scroll.width}x${g.scroll.height}`);
  if (g.canvas.width < 1280 || g.canvas.height < 800) fail(`the canvas is ${g.canvas.width}x${g.canvas.height}, less than 1280x800`);
  if (g.scroll.width <= g.view.width) fail(`the page is ${g.scroll.width} wide on a ${g.view.width}-wide phone: nothing to scroll to`);

  const f = await until("nothing cut off", (f) => f["layout.hidden"] === "0");
  if (f["layout.banner"] !== "none") fail(`the red strip shows: ${f["layout.banner"]}`);

  // Each control, scrolled to, wholly on the phone's screen; one of them was off it to start.
  let offAtFirst = 0;
  for (const name of IMPORTANT) {
    const before = await onScreen(name);
    if (!before) { fail(`${name} never measured`); continue; }
    if (!inside(before)) offAtFirst++;
    // As a browser's scrollIntoView would: the control's middle to the screen's, as far as the
    // page goes. The canvas starts the page, so its places are the page's.
    await scrollToPlace(before.place);
    await page.waitForTimeout(300);
    const after = await onScreen(name);
    // A control bigger than the phone's screen (the map) shows its middle; one that fits shows
    // whole.
    const fits = before.right - before.left <= g.view.width && before.bottom - before.top <= g.view.height;
    if (fits && !inside(after)) fail(`${name} scrolled to, still not wholly on screen: ${JSON.stringify(after)}`);
    if (!fits && (after.right < 0 || after.left > g.view.width || after.bottom < 0 || after.top > g.view.height)) fail(`${name} scrolled to, nothing of it on screen`);
    console.log(`${name}: ${inside(before) ? "on screen" : "off screen"} at first, ${fits ? (inside(after) ? "wholly on screen" : "NOT on screen") : "its middle on screen"} scrolled to`);
  }
  if (offAtFirst === 0) fail("every control was on the phone's screen without scrolling: the check proves nothing");

  // A finger's pan to the right scrolls the page.
  await page.evaluate(() => window.scrollTo(0, 0));
  await page.waitForTimeout(300);
  // A finger as the browser has it: a touch put down, moved left in steps, lifted. (Chromium's
  // Input.synthesizeScrollGesture scrolls nothing in a headless browser, even over a plain
  // element; touches dispatched one by one pan as a finger does.)
  const cdp = await context.newCDPSession(page);
  const finger = (x) => [{ x, y: 500, id: 1 }];
  await cdp.send("Input.dispatchTouchEvent", { type: "touchStart", touchPoints: finger(350) });
  for (let x = 340; x >= 50; x -= 10) {
    await cdp.send("Input.dispatchTouchEvent", { type: "touchMove", touchPoints: finger(x) });
    await page.waitForTimeout(16);
  }
  await cdp.send("Input.dispatchTouchEvent", { type: "touchEnd", touchPoints: [] });
  await page.waitForTimeout(800);
  const panned = await geometry();
  console.log(`after a pan: scrolled to ${panned.scroll.x},${panned.scroll.y}`);
  if (panned.scroll.x <= 0) fail("a finger's pan did not scroll the page");

  // A tap on FLIGHT PLAN's tab, scrolled to, opens it.
  const tab = await onScreen("tab-plan");
  if (!tab) fail("tab-plan never measured");
  else {
    await scrollToPlace(tab.place);
    await page.waitForTimeout(300);
    const at = await onScreen("tab-plan");
    await page.touchscreen.tap((at.left + at.right) / 2, (at.top + at.bottom) / 2);
    await until("FLIGHT PLAN tapped open", (f) => f.screen === "plan");
  }
  await page.screenshot({ path: `${out}/phone.png`, fullPage: true });

  // The page as it was before 2026-10-06: the canvas the phone's own size. Controls cut off -
  // and still no red strip in this, a release build.
  await page.addStyleTag({ content: "body > canvas { min-width: 0 !important; min-height: 0 !important; }" });
  await until("controls cut off on the phone-sized canvas", (f) => Number(f["layout.hidden"]) > 0);
  // Past the strip's half second (layout_guard.rs, BANNER_AFTER).
  await page.waitForTimeout(1500);
  const small = await facts();
  console.log(`phone-sized canvas: layout.hidden ${small["layout.hidden"]}, layout.banner "${small["layout.banner"]}"`);
  if (small["layout.banner"] !== "none") fail(`the red strip shows in a release build: ${small["layout.banner"]}`);

  await browser.close();
  if (!process.exitCode) console.log("PASS");
})();

