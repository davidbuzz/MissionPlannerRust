#!/usr/bin/env python3
# The browser build's clock: every `std::time::Instant`, `SystemTime` and `UNIX_EPOCH` in the
# planner's crates becomes web-time's, and each crate that now names web_time gets the dependency.
#
# On the desktop web_time's types *are* std's (it re-exports them), so nothing changes there; in a
# web page std's clock panics ("time not implemented on this platform") and web_time reads
# `performance.now()` and `Date.now()`. Every crate converts at once, so an Instant handed from one
# crate to another is the same type on wasm too.
#
# Kept as a script so it can be run again on a newer tree:   python3 tools/port_clock.py <repo>
import pathlib, re, sys

repo = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()
crates = repo / "crates"
CLOCK = ("Instant", "SystemTime", "UNIX_EPOCH")
group = re.compile(r"use std::time::\{([^}]*)\};")
single = re.compile(r"use std::time::(Instant|SystemTime|UNIX_EPOCH);")
path = re.compile(r"\bstd::time::(Instant|SystemTime|UNIX_EPOCH)\b")

changed_crates = set()
files = 0
for source in sorted(crates.glob("*/src/**/*.rs")) + sorted(crates.glob("*/tests/**/*.rs")) + sorted(crates.glob("*/benches/**/*.rs")):
    if source.is_symlink():
        continue
    text = source.read_text()
    lines = []
    for line in text.splitlines(keepends=True):
        # A file's time (`Metadata::modified`) stays std's, and is compared with std's clock.
        if "port_clock: keep" not in line:
            line = group.sub(lambda m: f"use web_time::{{{m.group(1)}}};" if any(c in m.group(1) for c in CLOCK) else m.group(0), line)
            line = single.sub(r"use web_time::\1;", line)
            line = path.sub(r"web_time::\1", line)
        lines.append(line)
    new = "".join(lines)
    if new != text:
        source.write_text(new)
        files += 1
        changed_crates.add(source.relative_to(crates).parts[0])

dependency = 'web-time = "1"\n'
for crate in sorted(changed_crates):
    manifest = crates / crate / "Cargo.toml"
    text = manifest.read_text()
    if re.search(r"^web-time\s*=", text, re.M):
        # A target-only entry becomes an every-target one: the code now names it everywhere.
        text = re.sub(r"\n(# [^\n]*\n)*\[target\.'cfg\(target_family = \"wasm\"\)'\.dependencies\]\nweb-time = \"1\"\n", "\n[target.'cfg(target_family = \"wasm\")'.dependencies]\n", text)
        if re.search(r"^\[dependencies\][^\[]*^web-time\s*=", text, re.M | re.S):
            manifest.write_text(text)
            continue
    text = text.replace("[dependencies]\n", "[dependencies]\n# The clock, std's on the desktop and the browser's in a web page (experiments/web-experiment/tools/port_clock.py).\n" + dependency, 1)
    manifest.write_text(text)

print(f"{files} files in {len(changed_crates)} crates: {' '.join(sorted(changed_crates))}")
