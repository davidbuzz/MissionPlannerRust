#!/usr/bin/env python3
# The browser build's threads: every `std::thread` path in the planner's crates becomes
# `wasm_thread`'s (Zed's fork, the revision gpui_web uses), and each crate that now names it gets
# the dependency.
#
# On the desktop wasm_thread *is* std::thread (`pub use std::thread::*`), so nothing changes there;
# in a web page std cannot spawn a thread, and wasm_thread's `spawn`, `Builder` and `scope` start
# a Web Worker on the same shared memory instead.
#
# Kept as a script so it can be run again on a newer tree:   python3 tools/port_threads.py <repo>
import pathlib, re, sys

repo = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()
crates = repo / "crates"
DEPENDENCY = ('# Threads, std\'s on the desktop and Web Workers in a web page '
              '(experiments/web-experiment/tools/port_threads.py).\n'
              'wasm_thread = { git = "https://github.com/zed-industries/wasm_thread", '
              'rev = "0cf96c7708dfb97ccf3da50347e25edcf75d6937" }\n')

changed_crates = set()
files = 0
for source in sorted(crates.glob("*/src/**/*.rs")) + sorted(crates.glob("*/tests/**/*.rs")) + sorted(crates.glob("*/benches/**/*.rs")):
    if source.is_symlink():
        continue
    text = source.read_text()
    new = re.sub(r"use std::thread::\{self, ", "use wasm_thread::{self as thread, ", text)
    new = re.sub(r"use std::thread;", "use wasm_thread as thread;", new)
    new = re.sub(r"\bstd::thread::", "wasm_thread::", new)
    if new != text:
        source.write_text(new)
        files += 1
        changed_crates.add(source.relative_to(crates).parts[0])

for crate in sorted(changed_crates):
    manifest = crates / crate / "Cargo.toml"
    text = manifest.read_text()
    if re.search(r"^wasm_thread\s*=", text, re.M):
        continue
    manifest.write_text(text.replace("[dependencies]\n", "[dependencies]\n" + DEPENDENCY, 1))

print(f"{files} files in {len(changed_crates)} crates: {' '.join(sorted(changed_crates))}")
