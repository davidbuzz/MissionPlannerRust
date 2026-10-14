// The page's Tailscale node: Tailscale's own Go client built for the browser
// (web/tailscale, kept here as tailscale/tailscale.wasm.gz), started the
// first time a link needs it. It joins the tailnet through Tailscale's coordination server and
// carries its traffic over DERP relays on WebSockets, so the planner's tcp: and udpcl: links reach
// a vehicle, a companion computer or a SITL anywhere on the tailnet.
//
// The node's keys are kept in the browser's localStorage, so a signed-in page stays signed in.
// Until it is, the page shows Tailscale's sign-in link (`notifyBrowseToURL`).
//
// The planner's Tailscale indicator (the owner's ruling of 2026-10-14: "In the browser: the page
// node's state") reads the node's state through globalThis.mprTailscaleState (`tailscaleState`;
// crates/mp-transport/src/page.rs's `tailnet`), and its "sign in" starts the node without a link
// through globalThis.mprTailscaleStart (page.rs's `start_tailnet`).
//
// Page query options, for a tailnet of one's own and for tests:
//   ?tscontrol=<url>    a coordination server other than Tailscale's (Headscale, say)
//   ?tsauthkey=<key>    join with an auth key instead of signing in
//   ?tshostname=<name>  the node's name on the tailnet (default: mpr-<words>)
//   ?tsderphttp=1       DERP over plain HTTP, for a test tailnet whose relay has no TLS

const STORAGE_PREFIX = "mpr-tailscale:";
let starting = null;
let ipn = null;
let state = "NoState";
let netMap = null;
const running = [];
// For the planner's indicator (`tailscaleState`): the sign-in link the coordination server gave
// the node last, until it is on the tailnet, and why the node stopped, once it has.
let loginURL = null;
let failure = null;

/** The node's state and addresses, for the page's status and for tests. */
export function tailscaleStatus() {
    return { state, self: netMap?.self ?? null, peers: (netMap?.peers ?? []).length };
}
globalThis.mpTailscale = tailscaleStatus;

/**
 * The node as the planner's Tailscale indicator shows it (crates/mp-transport/src/page.rs's
 * `tailnet`), answered at once from what is held here, so the planner may ask at every frame:
 * - state: "NotStarted" (nothing has asked for it, and none of its 8 MB is fetched), "Loading"
 *   (being fetched, or on its way to the coordination server and the tailnet), "NeedsLogin"
 *   (waiting to be signed in, at `loginURL` once the server has given it), "NeedsApproval" (signed
 *   in, waiting for the tailnet's admin to approve the device), "Running" (on the tailnet) or
 *   "Stopped" (off it and not getting there, until the page is loaded again);
 * - backend: Tailscale's own word for it ("NoState", "Starting", "Running", ...), null until the
 *   node has said one;
 * - name, dnsName and addresses: its name on the tailnet (mpr-jerboa-stonecat), its MagicDNS name
 *   and its tailnet addresses, the 100.x first - while it is Running or NeedsApproval, when they
 *   are its own and not a former sign-in's;
 * - loginURL: the sign-in link the page shows, while it is NeedsLogin;
 * - reason: why it is Stopped, when the page knows.
 */
export function tailscaleState() {
    let shown = "Loading";
    if (failure !== null) shown = "Stopped";
    else if (starting === null) shown = "NotStarted";
    else if (state === "NeedsLogin") shown = "NeedsLogin";
    else if (state === "NeedsMachineAuth") shown = "NeedsApproval";
    else if (state === "Running") shown = "Running";
    else if (state === "Stopped" || state === "InUseOtherUser") shown = "Stopped";
    const node = shown === "Running" || shown === "NeedsApproval" ? netMap?.self : null;
    // The netmap's name is the node's MagicDNS name, with DNS's trailing dot.
    const dnsName = node?.name ? node.name.replace(/\.$/, "") : null;
    return {
        state: shown,
        backend: ipn === null ? null : state,
        name: dnsName ? dnsName.split(".")[0] : null,
        dnsName,
        // The IPv4 first, each family in the order the coordination server gave it.
        addresses: [...(node?.addresses ?? [])].sort((a, b) => a.includes(":") - b.includes(":")),
        loginURL: shown === "NeedsLogin" ? loginURL : null,
        reason: shown === "Stopped" ? failure : null,
    };
}
globalThis.mprTailscaleState = tailscaleState;

/**
 * Starts the node for the planner without a link - its Tailscale indicator's "sign in" - as a
 * link's first use does: the sign-in link is shown once the coordination server gives one. A node
 * started already is left as it is. Answers its state after, as `tailscaleState` does.
 */
globalThis.mprTailscaleStart = () => {
    startTailscale();
    return tailscaleState();
};

/** Starts the node once; resolves when it is on the tailnet. */
export function startTailscale() {
    if (starting === null) {
        starting = start();
        // A node that could not start - its 8 MB not fetched, say - is stopped, and says why.
        starting.catch((err) => stopped(`could not start: ${err?.message ?? err}`));
    }
    return starting;
}

// The node gone, until the page is loaded again: the first reason is the planner's to show.
function stopped(reason) {
    failure ??= reason;
    console.error(`tailscale: ${reason}`);
}

async function start() {
    const query = new URLSearchParams(location.search);
    await import("./tailscale/wasm_exec.js"); // defines globalThis.Go
    const go = new Go();
    // A tailnet of one's own whose DERP relay serves plain HTTP (Headscale's embedded one, in a
    // test): Tailscale's TS_DEBUG_USE_DERP_HTTP, through the Go runtime's environment.
    if (query.get("tsderphttp")) {
        go.env = { ...go.env, TS_DEBUG_USE_DERP_HTTP: "1" };
    }
    // Kept gzipped in the tree; the browser unpacks it as it arrives.
    const packed = await fetch(new URL("./tailscale/tailscale.wasm.gz", import.meta.url));
    const unpacked = new Response(packed.body.pipeThrough(new DecompressionStream("gzip")), {
        headers: { "Content-Type": "application/wasm" },
    });
    const { instance } = await WebAssembly.instantiateStreaming(unpacked, go.importObject);
    // Its Go program ends only on a fatal error of the node's own (main.go's log.Fatal, whose
    // words are in the console): the node is gone then.
    go.run(instance).then(
        () => stopped("the node ended (the browser's console says why)"),
        (err) => stopped(`the node failed: ${err?.message ?? err}`),
    );

    const config = {
        stateStorage: {
            getState: (key) => {
                try {
                    return localStorage.getItem(STORAGE_PREFIX + key) ?? "";
                } catch {
                    return "";
                }
            },
            setState: (key, value) => {
                try {
                    localStorage.setItem(STORAGE_PREFIX + key, value);
                } catch {
                    // Private browsing: the node lives as long as the page.
                }
            },
        },
    };
    if (query.get("tscontrol")) config.controlURL = query.get("tscontrol");
    if (query.get("tsauthkey")) config.authKey = query.get("tsauthkey");
    if (query.get("tshostname")) config.hostname = query.get("tshostname");
    ipn = globalThis.newIPN(config);

    return new Promise((resolve) => {
        ipn.run({
            notifyState: (next) => {
                state = next;
                console.log(`tailscale: ${next}`);
                if (next === "NeedsLogin") {
                    ipn.login();
                }
                if (next === "Running") {
                    loginURL = null;
                    hideSignIn();
                    resolve();
                    for (const go of running.splice(0)) go();
                }
                showStatus();
            },
            notifyNetMap: (json) => {
                netMap = JSON.parse(json);
                showStatus();
            },
            notifyBrowseToURL: (url) => {
                loginURL = url;
                showSignIn(url);
            },
            notifyPanicRecover: (err) => console.error(`tailscale: ${err}`),
        });
    });
}

/**
 * Opens `network` ("tcp" or "udp") to `address` ("host:port") over the tailnet, once the node is
 * on it. Returns { write(Uint8Array), close() }; `onData(Uint8Array)` gets what arrives, and
 * `onClose(reason)` is called once.
 */
export function dialTailscale(network, address, { onOpen, onData, onClose }) {
    let stream = null;
    let closed = false;
    const queued = [];
    const open = () => {
        if (closed) return;
        stream = ipn.dial(network, address, {
            onOpen: () => onOpen?.(),
            onData: (bytes) => onData(bytes),
            onClose: (reason) => {
                closed = true;
                onClose(reason);
            },
        });
        for (const bytes of queued.splice(0)) stream.write(bytes);
    };
    startTailscale();
    if (state === "Running") open();
    else running.push(open);
    return {
        write(bytes) {
            if (stream) stream.write(bytes);
            else queued.push(bytes);
        },
        close() {
            closed = true;
            stream?.close();
        },
    };
}

/**
 * Listens for datagrams to `port` on this node's tailnet address ("udp": Mission Planner's UDP
 * link, which waits for a vehicle to send first), once the node is on the tailnet. Returns
 * { write(Uint8Array), close() }: a write goes to whoever sent last, and is dropped before anyone
 * has. `onOpen(address)` once listening, `onData(Uint8Array)` for each datagram, `onClose(reason)`
 * once.
 */
export function listenTailscale(network, port, { onOpen, onData, onClose }) {
    let listener = null;
    let closed = false;
    const open = () => {
        if (closed) return;
        listener = ipn.listen(network, port, {
            onOpen: (address) => onOpen?.(address),
            onData: (bytes) => onData(bytes),
            onClose: (reason) => {
                closed = true;
                onClose(reason);
            },
        });
    };
    startTailscale();
    if (state === "Running") open();
    else running.push(open);
    return {
        write(bytes) {
            listener?.write(bytes);
        },
        close() {
            closed = true;
            listener?.close();
        },
    };
}

// --- what the pilot sees: the sign-in link, and the node's state -------------------------------

function banner() {
    let element = document.getElementById("mpr-tailscale");
    if (!element) {
        element = document.createElement("div");
        element.id = "mpr-tailscale";
        element.style.cssText =
            "position:fixed;right:12px;bottom:12px;z-index:10;max-width:420px;padding:10px 12px;" +
            "border-radius:6px;font:13px system-ui,sans-serif;color:#e6edf3;background:#262a30ee;" +
            "border:1px solid #3a4048;box-shadow:0 2px 8px #0008";
        document.body.appendChild(element);
    }
    return element;
}

function showSignIn(url) {
    const element = banner();
    element.replaceChildren();
    element.append("The link needs this page on your tailnet: ");
    const link = document.createElement("a");
    link.href = url;
    link.target = "_blank";
    link.rel = "noopener";
    link.textContent = "sign in to Tailscale";
    link.style.color = "#58a6ff";
    element.append(link);
    element.dataset.state = "sign-in";
}

function hideSignIn() {
    const element = document.getElementById("mpr-tailscale");
    if (element && element.dataset.state === "sign-in") element.remove();
}

function showStatus() {
    if (state !== "Running") return;
    const element = banner();
    const name = netMap?.self?.name ?? "";
    const address = netMap?.self?.addresses?.[0] ?? "";
    element.textContent = `Tailscale: on the tailnet as ${name.split(".")[0]} ${address}`;
    element.dataset.state = "running";
    clearTimeout(showStatus.fade);
    showStatus.fade = setTimeout(() => element.remove(), 8000);
}
