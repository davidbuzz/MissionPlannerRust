#!/usr/bin/env python3
# The browser build's three panicking std calls: `std::env::temp_dir`, `std::process::id` and
# `std::env::split_paths` become crates/mp-os's, which are std's on the desktop and answer
# harmlessly in a web page; each crate that now names mp_os gets the dependency.
#
# Kept as a script so it can be run again on a newer tree:   python3 tools/port_os.py <repo>
import pathlib, re, sys

repo = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()
crates = repo / "crates"
DEPENDENCY = ('# The std calls that panic in a web page, std\'s on the desktop '
              '(experiments/web-experiment/tools/port_os.py).\nmp-os.workspace = true\n')
changed_crates = set()
files = 0
for source in sorted(crates.glob("*/src/**/*.rs")) + sorted(crates.glob("*/tests/**/*.rs")) + sorted(crates.glob("*/benches/**/*.rs")):
    crate = source.relative_to(crates).parts[0]
    if source.is_symlink() or crate == "mp-os":
        continue
    text = source.read_text()
    new = re.sub(r"\bstd::env::temp_dir\b", "mp_os::temp_dir", text)
    new = re.sub(r"\bstd::process::id\(\)", "mp_os::process_id()", new)
    new = re.sub(r"\bstd::env::split_paths\(", "mp_os::split_paths(", new)
    if new != text:
        source.write_text(new)
        files += 1
        changed_crates.add(crate)
for crate in sorted(changed_crates):
    manifest = crates / crate / "Cargo.toml"
    text = manifest.read_text()
    if not re.search(r"^mp-os\s*[.=]", text, re.M):
        manifest.write_text(text.replace("[dependencies]\n", "[dependencies]\n" + DEPENDENCY, 1))
print(f"{files} files in {len(changed_crates)} crates: {' '.join(sorted(changed_crates))}")
