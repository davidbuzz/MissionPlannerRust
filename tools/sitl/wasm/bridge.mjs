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
// usage: node bridge.mjs <module.js> <port> [SITL arguments...]

import net from 'node:net';
import { pathToFileURL } from 'node:url';

const [modulePath, portText, ...sitlArguments] = process.argv.slice(2);
if (modulePath === undefined || portText === undefined) {
    console.error('usage: node bridge.mjs <module.js> <port> [SITL arguments...]');
    process.exit(2);
}
const port = Number(portText);
const SERIAL0 = 0;
const BUFFER = 4096;

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
const sitl = await createModule({
    arguments: sitlArguments,
    print: console.log,
    printErr: console.error,
});
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

process.on('SIGTERM', () => process.exit(0));
