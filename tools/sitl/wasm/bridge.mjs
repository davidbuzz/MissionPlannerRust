// Copyright (C) 2026 David "Buzz" Bussenschutt
//
// This file is part of MissionPlannerRust, a Rust implementation derived from
// Mission Planner (Copyright (C) 2010-2024 Michael Oborne and contributors,
// https://github.com/ArduPilot/MissionPlanner); NOTICE records the changes.
//
// MissionPlannerRust is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by the
// Free Software Foundation, version 3 of the License.
//
// SPDX-License-Identifier: GPL-3.0-only
//
// The bridge the SIMULATION screen's "try local wasm" box starts (the owner's word, 2026-10-04;
// crates/mp-gui/src/sitl/launcher.rs, LocalWasm): it loads one of this folder's ArduPilot
// WebAssembly SITL modules (arducopter.js, arduplane.js - Emscripten ES6 modules, README.md),
// starts the vehicle with the arguments it is given, and carries SERIAL0 - the module's
// ardupilot_serial_read/write exports over its heap - to and from tcp:127.0.0.1:<port>, one client
// at a time: what the planner connects to, as it connects to a native SITL on 5760.
//
// The server listens before the module has loaded, so the planner's connect finds it; what the
// planner sends before the vehicle is up waits, and goes in once it is.
//
// The vehicle's parameters outlive it, as a native SITL's do (the owner's bug, 2026-10-04: a
// parameter written to the WebAssembly copter was gone after a stop and a start). ArduPilot's SITL
// keeps them in eeprom.bin in its working directory (libraries/AP_HAL_SITL/Storage.cpp,
// HAL_STORAGE_FILE), which in this build is Emscripten's in-memory filesystem, gone with the
// process. The planner starts the bridge in the vehicle's SITL folder, where a native SITL writes
// its own eeprom.bin: the bridge puts that file into the module before the vehicle starts, and
// writes it back to the folder whenever it changes - every half second, since the planner stops
// the bridge with a kill that leaves it no last word. --wipe empties it, as it empties a native one.
//
// usage: node bridge.mjs <module.js> <port> [SITL arguments...]

import fs from 'node:fs';
import net from 'node:net';
import path from 'node:path';
import { pathToFileURL } from 'node:url';

const [modulePath, portText, ...sitlArguments] = process.argv.slice(2);
if (modulePath === undefined || portText === undefined) {
    console.error('usage: node bridge.mjs <module.js> <port> [SITL arguments...]');
    process.exit(2);
}
const port = Number(portText);
const SERIAL0 = 0;
const BUFFER = 4096;
const EEPROM = 'eeprom.bin';
const EEPROM_ON_DISK = path.join(process.cwd(), EEPROM);
const SAVE_EVERY_MS = 500;

// What the folder holds, and so what the module starts with; then what was last written there.
let saved = fs.existsSync(EEPROM_ON_DISK) ? fs.readFileSync(EEPROM_ON_DISK) : null;

/** The folder's eeprom.bin into the module, before its main() opens it. */
function restore(module) {
    if (saved !== null) {
        module.FS.writeFile(EEPROM, saved);
        console.log(`bridge: ${EEPROM_ON_DISK} restored (${saved.length} bytes)`);
    }
}

/** The module's eeprom.bin to the folder, when it differs from what is there. */
function save(module) {
    let now;
    try {
        now = Buffer.from(module.FS.readFile(EEPROM));
    } catch {
        return; // not opened yet
    }
    if (saved !== null && saved.equals(now)) {
        return;
    }
    const temporary = `${EEPROM_ON_DISK}.tmp`;
    fs.writeFileSync(temporary, now);
    fs.renameSync(temporary, EEPROM_ON_DISK);
    saved = now;
}

let client = null;
// What the client has sent and the vehicle has not yet taken.
const waiting = [];

const server = net.createServer((socket) => {
    if (client !== null) {
        client.destroy();
    }
    client = socket;
    waiting.length = 0;
    socket.setNoDelay(true);
    socket.on('data', (data) => waiting.push(data));
    socket.on('close', () => {
        if (client === socket) {
            client = null;
        }
    });
    socket.on('error', () => {});
});
server.on('error', (err) => {
    console.error(`bridge: cannot serve tcp:127.0.0.1:${port}: ${err.message}`);
    process.exit(1);
});
server.listen(port, '127.0.0.1', () => console.log(`bridge: serving tcp:127.0.0.1:${port}`));

const { default: createModule } = await import(pathToFileURL(modulePath));
const options = {
    arguments: sitlArguments,
    print: console.log,
    printErr: console.error,
    // Emscripten calls these with the module, after its filesystem is up and before main().
    preRun: [(module) => restore(module ?? options)],
};
const sitl = await createModule(options);
setInterval(() => save(sitl), SAVE_EVERY_MS);
const malloc = sitl.cwrap('ardupilot_malloc', 'number', ['number']);
const read = sitl.cwrap('ardupilot_serial_read', 'number', ['number', 'number', 'number']);
const write = sitl.cwrap('ardupilot_serial_write', 'number', ['number', 'number', 'number']);
const fromVehicle = malloc(BUFFER);
const toVehicle = malloc(BUFFER);
console.log(`bridge: ${modulePath} started with ${sitlArguments.join(' ')}`);

// Every 5 ms: what the vehicle wrote to the client, and what the client sent to the vehicle. The
// heap is read afresh each time, as it grows.
setInterval(() => {
    for (;;) {
        const length = read(SERIAL0, fromVehicle, BUFFER);
        if (length <= 0) {
            break;
        }
        if (client !== null) {
            client.write(Buffer.from(sitl.HEAPU8.slice(fromVehicle, fromVehicle + length)));
        }
        if (length < BUFFER) {
            break;
        }
    }
    while (waiting.length > 0) {
        const chunk = waiting[0];
        const take = Math.min(chunk.length, BUFFER);
        sitl.HEAPU8.set(chunk.subarray(0, take), toVehicle);
        const written = write(SERIAL0, toVehicle, take);
        if (written <= 0) {
            break;
        }
        if (written < chunk.length) {
            waiting[0] = chunk.subarray(written);
        } else {
            waiting.shift();
        }
    }
}, 5);

process.on('SIGTERM', () => {
    save(sitl);
    process.exit(0);
});
