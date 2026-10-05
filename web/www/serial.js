// The computer's serial ports for the planner in a page: WebSerial, which Chromium-based browsers
// have and Firefox and Safari have not. A page sees only the ports the browser has let it use - each
// chosen once in the browser's own chooser, which only a click may open - and the planner lists
// those as its serial ports (crates/mp-transport/src/serial.rs, page.rs's serial_ports); its port
// box's "Choose a serial port..." opens the chooser (crates/mp-gui/src/page_serial.rs). A serial
// link the planner asks for ("serial:<name>:<baud>", page.rs) is opened here and carried as
// link.js carries the others.
//
// - serveSerial(chosen): publishes the ports as globalThis.mprSerialPorts, [name, vid, pid] each
//   (-1 for an id not known), kept up as ports are plugged and unplugged, and gives the page
//   mprSerialChoose(), the chooser; the port chosen is handed to `chosen` by its name, the
//   planner's planner_serial_chosen;
// - openSerial(name, baud, received, said): the port opened at `baud`, its bytes to `received`,
//   what happens to it to `said`; resolves to { send, setBaud, setDtr, close }, or null - the
//   rate and DTR asked by the planner's SiK radio and firmware tools (page.rs's controls).

let ports = [];

const hex = (id) => id.toString(16).padStart(4, "0");

/// A port's name in the planner's list: its place among the page's ports, and its USB ids when the
/// browser gives them, as a desktop lists a USB autopilot by what it is.
function nameOf(port, index) {
    const { usbVendorId: vid, usbProductId: pid } = port.getInfo();
    const ids = vid !== undefined && pid !== undefined ? ` (${hex(vid)}:${hex(pid)})` : "";
    return `WebSerial ${index + 1}${ids}`;
}

function publish() {
    globalThis.mprSerialPorts = ports.map((port, index) => {
        const { usbVendorId: vid, usbProductId: pid } = port.getInfo();
        return [nameOf(port, index), vid ?? -1, pid ?? -1];
    });
}

/// Installs the ports and the chooser; nothing in a browser without WebSerial.
export function serveSerial(chosen) {
    if (!navigator.serial) return;
    const refresh = () => navigator.serial.getPorts().then((granted) => {
        ports = granted;
        publish();
    });
    refresh();
    navigator.serial.addEventListener("connect", refresh);
    navigator.serial.addEventListener("disconnect", refresh);
    globalThis.mprSerialChoose = () => {
        navigator.serial.requestPort().then((port) => {
            if (!ports.includes(port)) ports.push(port);
            publish();
            chosen(nameOf(port, ports.indexOf(port)));
        }).catch((err) => console.info(`serial: no port chosen (${err.message ?? err})`));
    };
}

/// The port the planner named, opened at `baud` and read until closed: { send, setBaud, setDtr,
/// close }, each done in the order asked - WebSerial changes a port's rate only by closing and
/// opening it again, which waits for the bytes written before it.
export async function openSerial(name, baud, received, said) {
    const port = ports.find((port, index) => nameOf(port, index) === name);
    if (!port) {
        said(`no serial port ${name} in this page`);
        return null;
    }
    let reader = null;
    let writer = null;
    let written = Promise.resolve();
    const start = async (rate) => {
        await port.open({ baudRate: rate, bufferSize: 65536 });
        writer = port.writable.getWriter();
        const mine = port.readable.getReader();
        reader = mine;
        (async () => {
            try {
                while (true) {
                    const { value, done } = await mine.read();
                    if (done) break;
                    if (value && value.length) received(value);
                }
            } catch (err) {
                if (reader === mine) said(`serial ${name}: ${err.message ?? err}`);
            }
        })();
    };
    const stop = async () => {
        await written.catch(() => {});
        const [mine, out] = [reader, writer];
        reader = null;
        writer = null;
        try { await mine?.cancel(); } catch (_) {}
        mine?.releaseLock();
        out?.releaseLock();
        await port.close().catch(() => {});
    };
    try {
        await start(baud);
    } catch (err) {
        said(`serial ${name}: ${err.message ?? err}`);
        return null;
    }
    said(`serial ${name} at ${baud}`);
    // What is asked, done in turn; a write is handed to the port at once, in order.
    let turn = Promise.resolve();
    const inTurn = (job) => {
        turn = turn.then(job).catch((err) => said(`serial ${name}: ${err.message ?? err}`));
        return turn;
    };
    return {
        send: (bytes) => inTurn(() => {
            if (writer) written = writer.write(bytes);
        }),
        setBaud: (rate) => inTurn(async () => {
            await stop();
            await start(rate);
            said(`serial ${name} at ${rate}`);
        }),
        setDtr: (level) => inTurn(() => port.setSignals({ dataTerminalReady: level })),
        close: () => inTurn(async () => {
            await stop();
            said(`serial ${name} closed`);
        }),
    };
}
