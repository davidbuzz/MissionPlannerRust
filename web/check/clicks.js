// Clicking the planner's controls by name in a page: with ?facts=1 the page gives each control's
// place (index.html's mpProbe(), the probe the desktop's GUI scripts click by), so a check is not
// tied to where a control happened to be drawn - the port box's rows moved when the page's serial
// chooser joined them (2026-10-06), and checks clicking by coordinates clicked the wrong row.

/// Where `name` is once it is on screen and has stayed put a moment - a dialog's first frame may
/// place it before it is laid out where it stays - or null after `ms`.
async function placeOf(page, name, ms = 10000) {
  let last = null;
  for (let waited = 0; waited < ms; waited += 250) {
    const place = await page.evaluate((name) => (globalThis.mpProbe ? globalThis.mpProbe()[name] : undefined), name).catch(() => undefined);
    if (place && last && place.x === last.x && place.y === last.y) return [place.x, place.y];
    last = place ?? null;
    await page.waitForTimeout(250);
  }
  return null;
}

/// A click as a hand makes one: the pointer over the control first, the press a moment later.
/// Playwright's own click moves and presses at once; files_check's click on Review a Log's picker
/// button made no click in two runs of four that way, and none failed with a moment between
/// (2026-10-06). Why is not known: gpui hit-tests every mouse event against the frame drawn.
async function clickAt(page, place) {
  await page.mouse.move(...place);
  await page.waitForTimeout(100);
  await page.mouse.click(...place);
}

/// Clicks `name`; false when it never showed.
async function clickNamed(page, name, ms) {
  const place = await placeOf(page, name, ms);
  if (!place) {
    console.log(`FAIL: ${name} never showed, or never stayed put`);
    process.exitCode = 1;
    return false;
  }
  await clickAt(page, place);
  return true;
}

module.exports = { placeOf, clickAt, clickNamed };
