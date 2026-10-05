// The whole planner reaching a vehicle over a tailnet from a web page, as a pilot connects: the
// port box's TCP, CONNECT, a tailnet address and a port. The page's Tailscale node (www/tailscale.js)
// joins the tailnet with an auth key, and the link crosses it to the vehicle; fails unless the
// node reaches Running, the planner asks for that address, bytes cross both ways, and the page has
// no error.
//
//   NODE_PATH=<playwright> node check/tailscale_check.js <out-dir> <control-url> <auth-key> <host> <port>
//
// Against a tailnet of one's own: Headscale with its embedded DERP on plain HTTP (the page asks for
// DERP over HTTP, tsderphttp), reached through check/cors_proxy.py, which adds the CORS header
// Tailscale's own control server sends (access-control-allow-origin: *).
const { chromium } = require("playwright");
const [out = ".", control, authKey, host, port] = process.argv.slice(2);
const fail = (why) => { console.log(`FAIL: ${why}`); process.exitCode = 1; };
const PORT_BOX = [996, 36], TCP = [964, 97], CONNECT = [1187, 36];
(async () => {
  const browser = await chromium.launch({
    headless: true,
    args: ["--enable-unsafe-swiftshader", "--use-angle=swiftshader", "--ignore-gpu-blocklist"],
  });
  const page = await browser.newPage({ viewport: { width: 1400, height: 900 } });
  const errors = [];
  page.on("console", (m) => {
    const t = m.text();
    if (t.startsWith("panicked")) errors.push(t.split("\n").slice(0, 2).join(" "));
    if (t.startsWith("tailscale:") || t.startsWith("link:")) console.log(t.slice(0, 200));
  });
  page.on("pageerror", (e) => errors.push(e.message));
  const query = new URLSearchParams({ tscontrol: control, tsauthkey: authKey, tshostname: "mpr-browser", tsderphttp: "1" });
  await page.goto(`http://127.0.0.1:8080/planner.html?${query}`, { waitUntil: "load" });
  await page.waitForTimeout(8000);
  await page.mouse.click(...PORT_BOX); await page.waitForTimeout(800);
  await page.mouse.click(...TCP); await page.waitForTimeout(800);
  await page.mouse.click(...CONNECT); await page.waitForTimeout(1500);
  // Mission Planner's boxes offer 127.0.0.1 and 5760: emptied, then the tailnet's address typed.
  const replace = async (text) => {
    await page.keyboard.press("End");
    for (let i = 0; i < 40; i++) await page.keyboard.press("Backspace");
    await page.keyboard.type(text);
    await page.keyboard.press("Enter");
  };
  await replace(host); await page.waitForTimeout(1500);
  // The port box: kept when it is already the port asked for (Mission Planner's 5760).
  if (port === "5760") await page.keyboard.press("Enter");
  else await replace(port);
  let crossed = {}, status = {};
  for (let waited = 0; waited < 90000; waited += 1000) {
    await page.waitForTimeout(1000);
    crossed = await page.evaluate(() => (globalThis.mpLinkCrossed ? globalThis.mpLinkCrossed() : {}));
    status = await page.evaluate(() => (globalThis.mpTailscale ? globalThis.mpTailscale() : {}));
    if (crossed.toPlanner > 20000 && crossed.fromPlanner > 1000) break;
  }
  await page.screenshot({ path: `${out}/tailscale-connected.png` });
  console.log(JSON.stringify({ crossed, status }).slice(0, 500));
  if (status.state !== "Running") fail(`the page's node is ${status.state}`);
  if (!(crossed.asked ?? []).includes(`tcp:${host}:${port}`)) fail(`the planner asked for ${JSON.stringify(crossed.asked)}`);
  if (!(crossed.toPlanner > 20000)) fail(`only ${crossed.toPlanner} bytes reached the planner over the tailnet`);
  if (!(crossed.fromPlanner > 1000)) fail(`only ${crossed.fromPlanner} bytes came from the planner`);
  for (const error of [...new Set(errors)].slice(0, 5)) console.log(`error: ${error.slice(0, 300)}`);
  if (errors.length) fail(`${errors.length} error(s) in the page`);
  await browser.close();
  if (!process.exitCode) console.log("PASS");
})();
