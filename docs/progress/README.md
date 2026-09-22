# Progress screenshots

Each screenshot here is a progress report for a deliverable that changes what the user sees.
They are captured by `tools/screenshot.sh`, which launches the GUI, waits for the window to map,
grabs it, and then leaves it on screen so a human watching the desktop sees the same thing.

```
cargo build -p mp-gui
tools/screenshot.sh d06-first-window 12
tools/screenshot.sh d10-flight-data 12 -- tcp:127.0.0.1:5760
```

Naming: `d<NN>-<what-changed>.png`, so the directory reads as a timeline of the port.

| Screenshot | Deliverable | What it shows |
|---|---|---|
| `d06-first-window.png` | D6 | The first gpui window: shell, theme, panel layout |

Why screenshots rather than a written status line: a GUI port can be "90% done" in a tracker while
showing a blank window. An image is the one claim about the interface that cannot be overstated.
