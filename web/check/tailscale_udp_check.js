// A vehicle that sends first, over UDP, to the planner in a web page (the browser build's
// odds-and-ends row: "UDP listening (udp:0.0.0.0:14550) over the tailnet"). The port box's UDP and
// its Listen Port, as a pilot connects: in a page the planner listens on its own tailnet address
// (www/tailscale.js's listenTailscale, tailscale/main.go's listen) and answers whoever sent last,
// as Mission Planner's UdpSerial does. Then a vehicle on the tailnet - tailscale/udpvehicle, a
// SITL's stream relayed as datagrams, as a companion computer's router sends one - sends to that
// address. Fails unless the page's node reaches Running and listens, the planner hears a vehicle
// and its answers reach the vehicle, and the page has no fault.
//
//   NODE_PATH=<playwright> node check/tailscale_udp_check.js <out-dir> <page-control-url> <auth-key> \
//       <udp-vehicle> <vehicle-control-url> [sitl 127.0.0.1:5760]
//
// Run by check/tailnet_e2e.sh, against its own tailnet.
const { spawn } = require("child_process");
const { chromium } = require("playwright");
const { clickNamed } = require("./clicks");
const [out = ".", control, authKey, vehicleBin, vehicleControl, sitl = "127.0.0.1:5760"] = process.argv.slice(2);
const fail = (why) => { console.log(`FAIL: ${why}`); process.exitCode = 1; };
(async () => {
  const browser = await chromium.launch({
    headless: true,
    args: ["--enable-unsafe-swiftshader", "--use-angle=swiftshader", "--ignore-gpu-blocklist"],
  });
  const page = await browser.newPage({ viewport: { width: 1400, height: 900 } });
  const errors = [];
  page.on("console", (m) => {
    const t = m.text();
    if (t.startsWith("panicked") || t.startsWith("planner fault")) errors.push(t.split("\n").slice(0, 2).join(" "));
    if (t.startsWith("tailscale:") || t.startsWith("link:")) console.log(t.slice(0, 200));
  });
  page.on("pageerror", (e) => errors.push(e.message));
  const facts = () => page.evaluate(() => (globalThis.mpFacts ? globalThis.mpFacts() : {})).catch(() => ({}));
  const node = () => page.evaluate(() => (globalThis.mpTailscale ? globalThis.mpTailscale() : {})).catch(() => ({}));
  const query = new URLSearchParams({ facts: "1", demo: "0", tscontrol: control, tsauthkey: authKey, tshostname: "mpr-browser-udp", tsderphttp: "1" });
  await page.goto(`http://127.0.0.1:8080/?${query}`, { waitUntil: "load" });
  for (let waited = 0; waited < 60000 && (await facts()).screen !== "fly"; waited += 500) await page.waitForTimeout(500);

  // UDP, CONNECT, and Mission Planner's Listen Port, 14550, as offered.
  await clickNamed(page, "main-port");
  await clickNamed(page, "main-port-UDP");
  await clickNamed(page, "main-connect");
  await page.waitForTimeout(1500);
  await page.keyboard.press("Enter");

  // The page's node on the tailnet, and listening there.
  let status = {};
  for (let waited = 0; waited < 90000; waited += 1000) {
    await page.waitForTimeout(1000);
    status = await node();
    if (status.state === "Running" && status.self?.addresses?.some((a) => a.includes("."))) break;
  }
  const address = status.self?.addresses?.find((a) => a.includes("."));
  console.log(`the page's node: ${status.state}, ${address}`);
  if (!address) {
    fail(`the page's node is ${status.state}, with no IPv4 address`);
  } else {
    // The vehicle on the tailnet, sending first.
    const vehicle = spawn(vehicleBin, ["-control", vehicleControl, "-authkey", authKey, "-to", `${address}:14550`, "-sitl", sitl], {
      env: { ...process.env, TS_DEBUG_USE_DERP_HTTP: "1" },
    });
    let said = "";
    vehicle.stderr.on("data", (data) => { said += data; });
    let f = {}, crossed = {};
    for (let waited = 0; waited < 90000; waited += 1000) {
      await page.waitForTimeout(1000);
      f = await facts();
      crossed = await page.evaluate(() => (globalThis.mpLinkCrossed ? globalThis.mpLinkCrossed() : {}));
      if (Number(f["vehicle.count"]) >= 1 && crossed.fromPlanner > 1000 && said.includes("answered")) break;
    }
    vehicle.kill();
    console.log(`vehicle.count ${f["vehicle.count"]}, link.target "${f["link.target"]}"; ${crossed.toPlanner} bytes to the planner, ${crossed.fromPlanner} from it`);
    console.log(`the vehicle said: ${said.trim().split("\n").slice(-3).join(" | ")}`);
    if (!(crossed.asked ?? []).includes("udp:0.0.0.0:14550")) fail(`the planner asked for ${JSON.stringify(crossed.asked)}`);
    if (!(Number(f["vehicle.count"]) >= 1)) fail("the planner heard no vehicle");
    if (!said.includes("answered")) fail("the planner's answers never reached the vehicle");
  }
  for (const error of [...new Set(errors)].slice(0, 5)) console.log(`error: ${error.slice(0, 300)}`);
  if (errors.length) fail(`${errors.length} fault(s) in the page`);
  await page.screenshot({ path: `${out}/tailscale-udp.png` });
  await browser.close();
  if (!process.exitCode) console.log("PASS");
})();
