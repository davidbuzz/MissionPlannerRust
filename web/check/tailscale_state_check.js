// The page's Tailscale node as the planner's Tailscale indicator reads it (the owner's ruling of
// 2026-10-14: "In the browser: the page node's state"): www/tailscale.js's mprTailscaleState(),
// which crates/mp-transport/src/page.rs's `tailnet` calls, and mprTailscaleStart(), the
// indicator's "sign in" (page.rs's `start_tailnet`). With the planner up and nothing yet asking for
// the node, it is NotStarted and none of it has been fetched; started without a link it is Loading
// at once, then what Tailscale says it is, its 8 MB fetched once however often it is started.
// Fails unless so, on any fault in the page, or on any request to Tailscale's own servers.
//
//   NODE_PATH=<playwright> node check/tailscale_state_check.js [out-dir] [url] [control-url [auth-key]]
//
// Without a control URL - as check/ci.sh runs it - the node's coordination server is the page's own
// server, which is none: the node loads, comes up and asks to be signed in, and is given no sign-in
// link; nothing leaves the machine. With a test tailnet's (check/tailnet_e2e.sh's Headscale behind
// check/cors_proxy.py, its DERP on plain HTTP) and its auth key, the node must reach Running under
// the name it was given, at its 100.x and fd7a: addresses; then - or with that control URL alone -
// in a browser profile of its own, without the key: NeedsLogin, at the sign-in link the page shows.
const { chromium } = require("playwright");
const out = process.argv[2] || ".";
const url = process.argv[3] || "http://127.0.0.1:8080/?facts=1&demo=0";
const [control, authKey] = process.argv.slice(4);
const fail = (why) => { console.log(`FAIL: ${why}`); process.exitCode = 1; };
// The name the node is given on a test tailnet.
const HOSTNAME = "mpr-browser-state";
// What www/tailscale.js shows for each of Tailscale's own states.
const SHOWN = {
  NoState: "Loading", Starting: "Loading", NeedsLogin: "NeedsLogin", NeedsMachineAuth: "NeedsApproval",
  Running: "Running", Stopped: "Stopped", InUseOtherUser: "Stopped",
};
// Tailscale's own servers - coordination, DERP relays, logs - which no check asks: a test
// tailnet's are on this machine.
const tailscalesOwn = (address) => /(^|\.)tailscale\.(com|io)$/.test(new URL(address).hostname);
// The node's files, fetched only once something asks for it, and its 8 MB.
const nodeFile = (address) => /\/tailscale\/(tailscale\.wasm\.gz|wasm_exec\.js)$/.test(new URL(address).pathname);
const theNode = (address) => new URL(address).pathname.endsWith("/tailscale/tailscale.wasm.gz");
const nodeState = (page) =>
  page.evaluate(() => (globalThis.mprTailscaleState ? globalThis.mprTailscaleState() : null)).catch(() => null);

// A browser profile of its own - a node never signed in - with the planner up, and the node's
// options in the page's address. What the page says and asks for is kept.
async function open(browser, label, options) {
  const context = await browser.newContext({ viewport: { width: 1400, height: 900 } });
  const page = await context.newPage();
  const seen = { faults: [], requests: [], said: [] };
  page.on("console", (m) => {
    const text = m.text();
    if (text.startsWith("panicked") || text.startsWith("planner fault") || text.startsWith("planner:")) {
      seen.faults.push(text.split("\n").slice(0, 2).join(" "));
    }
    if (text.startsWith("tailscale:")) {
      seen.said.push(text);
      console.log(`${label}: ${text.slice(0, 200)}`);
    }
  });
  page.on("pageerror", (e) => seen.faults.push(e.message));
  page.on("request", (request) => seen.requests.push(request.url()));
  page.on("websocket", (socket) => seen.requests.push(socket.url()));
  const address = new URL(url);
  for (const [key, value] of Object.entries({ facts: "1", demo: "0", ...options })) {
    address.searchParams.set(key, value);
  }
  await page.goto(address.href, { waitUntil: "load" });
  let facts = {};
  for (let waited = 0; waited < 90000 && facts.screen !== "fly"; waited += 1000) {
    await page.waitForTimeout(1000);
    facts = await page.evaluate(() => (globalThis.mpFacts ? globalThis.mpFacts() : {})).catch(() => ({}));
  }
  if (facts.screen !== "fly") fail(`${label}: the planner never came up`);
  return { context, page, seen };
}

// Before anything asks for it: NotStarted, with nothing more to say, and none of it fetched - the
// planner up and reading it has not started it.
async function notStarted(label, page, seen) {
  const before = await nodeState(page);
  console.log(`${label}: before, ${JSON.stringify(before)}`);
  if (!before) return fail(`${label}: the page gives no mprTailscaleState()`);
  if (before.state !== "NotStarted") fail(`${label}: before anything asked for it, the node is ${before.state}`);
  const said = [before.backend, before.name, before.dnsName, before.loginURL, before.reason];
  if (said.some((value) => value !== null) || before.addresses.length) {
    fail(`${label}: a node not started says ${JSON.stringify(before)}`);
  }
  if (seen.requests.some(nodeFile)) fail(`${label}: the node was fetched before anything asked for it`);
}

// The indicator's "sign in": Loading at once. Asked again, it is the same node.
async function startNode(label, page) {
  const started = await page.evaluate(() => globalThis.mprTailscaleStart());
  console.log(`${label}: started, ${JSON.stringify(started)}`);
  if (started?.state !== "Loading") fail(`${label}: just started, the node is ${started?.state}, not Loading`);
  await page.evaluate(() => globalThis.mprTailscaleStart());
}

// The node's state every half second until `done` or `ms` have passed, each change said.
async function watch(label, page, done, ms) {
  let now = null;
  let last = "";
  for (let waited = 0; waited <= ms; waited += 500) {
    now = await nodeState(page);
    const word = now ? `${now.state} (Tailscale's ${now.backend})` : "no answer";
    if (word !== last) console.log(`${label}: ${word}`);
    last = word;
    if (now && done(now)) break;
    await page.waitForTimeout(500);
  }
  return now;
}

// A node that has said its state shows it as tailscale.js maps it - but a node gone, Stopped
// whatever it said last.
function shownAsSaid(label, now) {
  if (now.backend !== null && now.state !== "Stopped" && now.state !== SHOWN[now.backend]) {
    fail(`${label}: Tailscale's ${now.backend} is shown as ${now.state}, not ${SHOWN[now.backend]}`);
  }
}

// The node's 8 MB fetched once, nothing asked of Tailscale's own servers, and no fault.
async function finish(label, context, page, seen, shot) {
  await page.screenshot({ path: `${out}/${shot}` });
  const fetched = seen.requests.filter(theNode).length;
  if (fetched !== 1) fail(`${label}: the node was fetched ${fetched} times, not once`);
  const theirs = [...new Set(seen.requests.filter(tailscalesOwn))];
  if (theirs.length) fail(`${label}: Tailscale's own servers were asked: ${theirs.slice(0, 3).join(" ")}`);
  for (const fault of [...new Set(seen.faults)].slice(0, 5)) console.log(`error: ${fault.slice(0, 300)}`);
  if (seen.faults.length) fail(`${label}: ${seen.faults.length} fault(s) in the page`);
  await context.close();
}

// No coordination server: the page's own server stands in for one and answers none of its
// questions. The node comes up and asks to be signed in, and no server gives it a link.
async function noServer(browser) {
  const label = "no server";
  const { context, page, seen } = await open(browser, label, { tscontrol: new URL(url).origin });
  await notStarted(label, page, seen);
  await startNode(label, page);
  await watch(label, page, (now) => now.backend !== null && seen.said.includes("tailscale: NeedsLogin"), 90000);
  // What follows its asking: a sign-in begun with a server that answers nothing.
  await page.waitForTimeout(3000);
  const last = await nodeState(page);
  console.log(`${label}: ${JSON.stringify(last)}`);
  if (!seen.said.includes("tailscale: NeedsLogin")) fail(`${label}: the node never asked to be signed in`);
  if (!last || last.backend === null) {
    fail(`${label}: the node never came up`);
  } else {
    shownAsSaid(label, last);
    if (last.state !== "NeedsLogin" && last.state !== "Loading") fail(`${label}: the node is ${last.state}`);
    if (last.loginURL !== null) fail(`${label}: a sign-in link no server gave: ${last.loginURL}`);
    if (last.name !== null || last.dnsName !== null || last.addresses.length || last.reason !== null) {
      fail(`${label}: a node off the tailnet says ${JSON.stringify(last)}`);
    }
  }
  await finish(label, context, page, seen, "tailscale-state.png");
}

// A test tailnet and its auth key: on the tailnet, under the name it was given, at its addresses.
async function onTailnet(browser) {
  const label = "on the tailnet";
  const options = { tscontrol: control, tsauthkey: authKey, tshostname: HOSTNAME, tsderphttp: "1" };
  const { context, page, seen } = await open(browser, label, options);
  await notStarted(label, page, seen);
  await startNode(label, page);
  const last = await watch(label, page, (now) => now.state === "Running" || now.state === "Stopped", 90000);
  console.log(`${label}: ${JSON.stringify(last)}`);
  if (last?.state !== "Running") {
    fail(`${label}: the node is ${last?.state}, not Running`);
  } else {
    shownAsSaid(label, last);
    if (!last.name?.startsWith(HOSTNAME)) fail(`${label}: its name is ${last.name}, not ${HOSTNAME}`);
    if (!last.dnsName?.startsWith(`${last.name}.`)) fail(`${label}: its MagicDNS name, ${last.dnsName}, is not ${last.name}'s`);
    if (!/^100\.\d+\.\d+\.\d+$/.test(last.addresses[0] ?? "")) fail(`${label}: its first address is ${last.addresses[0]}, not a 100.x`);
    if (!last.addresses.some((address) => address.startsWith("fd7a:"))) {
      fail(`${label}: no fd7a: address among ${last.addresses.join(" ")}`);
    }
    if (last.loginURL !== null || last.reason !== null) fail(`${label}: on the tailnet it says ${JSON.stringify(last)}`);
  }
  await finish(label, context, page, seen, "tailscale-state-running.png");
}

// A test tailnet without a key: waiting to be signed in, at the link the page's banner shows.
async function signingIn(browser) {
  const label = "signing in";
  const { context, page, seen } = await open(browser, label, { tscontrol: control, tsderphttp: "1" });
  await notStarted(label, page, seen);
  await startNode(label, page);
  const signedIn = (now) => (now.state === "NeedsLogin" && now.loginURL !== null) || now.state === "Stopped";
  const last = await watch(label, page, signedIn, 60000);
  const shown = await page.evaluate(() => document.querySelector("#mpr-tailscale a")?.href ?? null);
  console.log(`${label}: ${JSON.stringify(last)}; the page shows ${shown}`);
  if (last?.state !== "NeedsLogin") {
    fail(`${label}: the node is ${last?.state}, not NeedsLogin`);
  } else {
    shownAsSaid(label, last);
    if (!last.loginURL) fail(`${label}: no sign-in link`);
    else if (shown === null || new URL(last.loginURL).href !== shown) {
      fail(`${label}: the planner's sign-in link, ${last.loginURL}, is not the page's, ${shown}`);
    }
  }
  await finish(label, context, page, seen, "tailscale-state-sign-in.png");
}

(async () => {
  const browser = await chromium.launch({
    headless: true,
    args: ["--enable-unsafe-swiftshader", "--use-angle=swiftshader", "--ignore-gpu-blocklist"],
  });
  if (!control) {
    await noServer(browser);
  } else {
    if (authKey) await onTailnet(browser);
    await signingIn(browser);
  }
  await browser.close();
  if (!process.exitCode) console.log("PASS");
})();
