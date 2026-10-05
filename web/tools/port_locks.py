#!/usr/bin/env python3
# The browser build's waits: every `.lock()` in the planner's crates becomes `.os_lock()`, from
# crates/mp-os's `Lock`, which is `Mutex::lock` on the desktop and on a Web Worker, and spins on the
# page's main thread, where a browser never lets a thread wait (`Atomics.wait` throws there, and
# leaves gpui's borrows held); every `.recv_timeout(` becomes `.os_recv_timeout(`, which polls
# against the page's clock where std's would read its own and panic. Each scope that calls one gets
# its trait (`use mp_os::Lock as _;`, `use mp_os::RecvTimeout as _;`), each crate the dependency.
#
# Kept as a script so it can be run again on a newer tree:   python3 tools/port_locks.py <repo>
import pathlib, re, sys

repo = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()
crates = repo / "crates"
DEPENDENCY = ('# The std calls that panic in a web page, std\'s on the desktop '
              '(web/tools/port_os.py).\nmp-os.workspace = true\n')

def place_imports(text, method, IMPORT):
    """The trait imported into each scope that calls `os_lock` itself: the file, or an inline
    module (whose `use super::*;` brings the file's in, when the file has one)."""
    lines = text.splitlines(keepends=True)
    # Each inline module's span, by the brace at its own indentation.
    spans = []
    for i, line in enumerate(lines):
        opening = re.match(r"^(\s*)(?:pub(?:\([a-z]+\))? )?mod \w+ \{\s*$", line)
        if opening:
            indent = opening.group(1)
            end = next((j for j in range(i + 1, len(lines)) if lines[j].rstrip("\n") == indent + "}"), len(lines))
            spans.append((i, end, indent))
    def owner(n):
        inner = [span for span in spans if span[0] < n < span[1]]
        return max(inner, key=lambda span: span[0]) if inner else None
    users = {owner(n) for n, line in enumerate(lines) if method in line}
    file_scope = None in users
    needed = set(users)
    for span in users:
        if span is not None and file_scope and owner(span[0]) is None and "use super::*;" in "".join(lines[span[0] + 1:span[1]]):
            needed.discard(span)
    # An import already in a scope that needs it stays where it is (rustfmt sorts it among the
    # others); one in a scope that does not is removed.
    have = {}
    for n, line in enumerate(lines):
        if line.strip() == IMPORT.strip():
            have.setdefault(owner(n), []).append(n)
    drop = {n for scope, ns in have.items() for n in (ns if scope not in needed else ns[1:])}
    inserts = []
    for span in needed:
        if span is None or span in have:
            continue
        start, end, indent = span
        at = start + 1
        while at < end and (lines[at].strip().startswith("//!") or lines[at].strip().startswith("#![")):
            at += 1
        inserts.append((at, indent + "    " + IMPORT))
    if file_scope and None not in have:
        at = next((i for i, line in enumerate(lines) if line.startswith("use ")), None)
        if at is None:
            at = 0
            while at < len(lines) and (lines[at].startswith("//") or lines[at].startswith("#![") or not lines[at].strip()):
                at += 1
            inserts.append((at, IMPORT + "\n"))
        else:
            inserts.append((at, IMPORT))
    lines = [line if n not in drop else None for n, line in enumerate(lines)]
    for at, line in sorted(inserts, reverse=True):
        lines.insert(at, line)
    return "".join(line for line in lines if line is not None)

changed_crates = set()
files = 0
# Each std method that waits, its replacement from mp-os, and the trait that brings it in.
SWAPS = [
    (".lock()", ".os_lock()", "use mp_os::Lock as _;\n"),
    (".recv_timeout(", ".os_recv_timeout(", "use mp_os::RecvTimeout as _;\n"),
]
for source in sorted(crates.glob("*/src/**/*.rs")) + sorted(crates.glob("*/tests/**/*.rs")) + sorted(crates.glob("*/benches/**/*.rs")):
    crate = source.relative_to(crates).parts[0]
    if source.is_symlink() or crate in ("mp-os", "mp-cli"):
        continue
    text = source.read_text()
    new = text
    for old, method, import_line in SWAPS:
        if old == ".lock()" and re.search(r"\bfn lock\(&self\)", new):
            # A type of this file has a `lock()` of its own, which its callers mean: only the
            # Mutex lock that method makes, on the line under it, is swapped.
            lines = new.splitlines(keepends=True)
            for i in range(1, len(lines)):
                if re.search(r"\bfn lock\(&self\)", lines[i - 1]):
                    lines[i] = lines[i].replace(old, method)
            new = "".join(lines)
        else:
            new = new.replace(old, method)
        if method in new:
            new = place_imports(new, method, import_line)
    if new == text:
        continue
    source.write_text(new)
    files += 1
    changed_crates.add(crate)
for crate in sorted(changed_crates):
    manifest = crates / crate / "Cargo.toml"
    text = manifest.read_text()
    if not re.search(r"^mp-os\s*[.=]", text, re.M):
        manifest.write_text(text.replace("[dependencies]\n", "[dependencies]\n" + DEPENDENCY, 1))
print(f"{files} files in {len(changed_crates)} crates: {' '.join(sorted(changed_crates))}")
