// The page's Tailscale sign-in, as a pilot without an auth key meets it: a link to a tailnet
// address starts the page's node, the coordination server answers NeedsLogin with its sign-in URL,
// and the page shows "sign in to Tailscale" linking there. Fails unless it does.
//
//   NODE_PATH=<playwright> node check/tailscale_login_check.js <out-dir> <control-url> <host>
const { chromium } = require("playwright");
const [out = ".", control, host] = process.argv.slice(2);
const fail = (why) => { console.log(`FAIL: ${why}`); process.exitCode = 1; };
const PORT_BOX = [996, 36], TCP = [964, 97], CONNECT = [1187, 36];
(async () => {
  const browser = await chromium.launch({
    headless: true,
    args: ["--enable-unsafe-swiftshader", "--use-angle=swiftshader", "--ignore-gpu-blocklist"],
  });
  const page = await browser.newPage({ viewport: { width: 1400, height: 900 } });
  const query = new URLSearchParams({ tscontrol: control, tsderphttp: "1" });
  await page.goto(`http://127.0.0.1:8080/planner.html?${query}`, { waitUntil: "load" });
  await page.waitForTimeout(8000);
  await page.mouse.click(...PORT_BOX); await page.waitForTimeout(800);
  await page.mouse.click(...TCP); await page.waitForTimeout(800);
  await page.mouse.click(...CONNECT); await page.waitForTimeout(1500);
  await page.keyboard.press("End");
  for (let i = 0; i < 40; i++) await page.keyboard.press("Backspace");
  await page.keyboard.type(host);
  await page.keyboard.press("Enter"); await page.waitForTimeout(1500);
  await page.keyboard.press("Enter");
  let link = null;
  for (let waited = 0; waited < 60000 && !link; waited += 1000) {
    await page.waitForTimeout(1000);
    link = await page.evaluate(() => document.querySelector("#mpr-tailscale a")?.href ?? null);
  }
  await page.screenshot({ path: `${out}/tailscale-sign-in.png` });
  const status = await page.evaluate(() => (globalThis.mpTailscale ? globalThis.mpTailscale() : {}));
  console.log(JSON.stringify({ link, state: status.state }));
  if (!link) fail("no sign-in link shown");
  else if (!link.startsWith(control.replace(/\/$/, "").replace(":8091", ":8090")) && !link.startsWith(control)) fail(`the link is not the coordination server's: ${link}`);
  if (status.state !== "NeedsLogin") fail(`the page's node is ${status.state}, not NeedsLogin`);
  await browser.close();
  if (!process.exitCode) console.log("PASS");
})();
