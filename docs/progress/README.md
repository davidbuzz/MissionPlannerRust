# Progress screenshots

Each screenshot here is a progress report for a deliverable that changes what the user sees.
They are captured by `tools/screenshot.sh`, which launches the GUI, waits for the window to map,
grabs it, and then leaves it on screen so a human watching the desktop sees the same thing.

```
cargo build -p mp-gui
tools/screenshot.sh d06-first-window 12
tools/screenshot.sh d10-flight-data 12 -- tcp:127.0.0.1:5760
```

Naming: `d<NN>-<what-changed>.png`, so the directory reads as a timeline of the port; the later
ones are named for what they show. All were taken on 2026-09-23; the two pictures at the top of
`README.md` (2026-10-03) are in `docs/readme/`.

| Screenshot | Deliverable | What it shows |
|---|---|---|
| `d05-live-telemetry.png` | Deliverable 5 | Live telemetry in the GUI, from the bundled SITL |
| `d05-real-hardware.png` | Deliverable 5 | A real autopilot over USB, by its device path: the flight screen and its pre-arm messages |
| `d06-first-window.png` | Deliverable 6 | The first gpui window: shell, theme, panel layout |
| `d06-gpui-wgpu.png` | Deliverable 6 | The window on the git-pinned gpui |
| `d06-fly-screen.png` | Deliverable 6 | The flight screen's layout |
| `d06-fly-messages.png` | Deliverable 6 | The flight screen's messages: an arm and its acknowledgement |
| `d08-map-tiles.png` | Deliverable 8 | The first real map: tiles |
| `d08-live-map.png` | Deliverable 8 | The real flight path drawn on the map |
| `d09-hud.png` | Deliverable 9 | The primary flight display |
| `d09-prearm.png` | Deliverable 9 | Why the aircraft will not arm |
| `d09-force-arm.png` | Deliverable 9 | The force arm button |
| `d09-scroll-indicator.png` | Deliverable 9 | The actions panel's scroll indicator, with force arm |
| `d10-plan-editor.png` | Deliverable 10 | Mission items edited, the GUI clicked by name |
| `d10-item-params.png` | Deliverable 10 | A mission item's parameters, labelled from the definitions |
| `d10-waypoint-drag.png` | Deliverable 10 | A waypoint dragged on the map |
| `d11-mission-map.png` | Deliverable 11 | The planned mission on the map |
| `d11-survey-mission.png` | Deliverable 11 | A survey generated from the command line, on the map |
| `d11-survey.png` | Deliverable 11 | The survey tool |
| `d11-geofence.png` | Deliverable 11 | A geofence drawn and sent from the plan screen |
| `d11-plan-tools.png` | Deliverable 11 | Rally points, and the four drawing tools in one panel |
| `d12-accel-cal.png` | Deliverable 12 | Accelerometer calibration |
| `d12-compass-cal.png` | Deliverable 12 | Compass calibration, with each compass's progress |
| `d12-radio-cal.png` | Deliverable 12 | Radio calibration, all sixteen channels |
| `d12-setup.png` | Deliverable 12 | The motor test, one motor at a time |
| `d12-params.png` | Deliverable 12 | The parameter screen |
| `d12-param-search.png` | Deliverable 12 | Parameter search in a text field |
| `d14-logs.png` | Deliverable 14 | Logs downloaded from the setup screen |
| `estimator.png`, `estimator-hardware.png` | - | The EKF and vibration monitors, on SITL and on hardware |
| `flight-recording.png` | - | Every flight recorded to a `.tlog`, and the screen saying so |
| `joystick.png` | - | Flying from a joystick |
| `log-browse.png` | - | A dataflash log reviewed on screen |
| `param-files.png` | - | `.param` files saved, loaded and compared |
| `satellite.png` | - | Satellite imagery, and the choice of it |
| `tuning-graph.png` | - | The tuning graph, shown when asked for, where the C# puts it |
| `waypoint-click.png` | - | A waypoint added by a click on the map |

One more, ADR 0001's picture of the map viewport measured inside gpui, is named in that ADR.

Why screenshots rather than a written status line: a GUI port can be "90% done" in a tracker while
showing a blank window. An image is the one claim about the interface that cannot be overstated.
