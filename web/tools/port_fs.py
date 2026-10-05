#!/usr/bin/env python3
# The browser build's files (the owner's request of 2026-10-05: settings, missions and logs to
# survive a reload). Every `std::fs` in the crates' sources becomes crates/mp-os's `mp_os::fs` -
# std's own on the desktop, the page's files in a web page (crates/mp-os/src/fs.rs) - and every
# `.exists()`, `.is_file()` and `.is_dir()` in code becomes `.os_exists()`, `.os_is_file()` and
# `.os_is_dir()`, with mp_os::fs::FsExt in scope in each module that calls one: std's ask the
# operating system, which a page has not got, and a type's own method cannot be taken over by a
# trait. Each crate that now names mp_os gets the dependency.
#
# The crates' sources only (src/): their tests and benches run on the desktop, where the two are
# the same. Calls on the planner's own types, which have methods of those names, are kept by name
# in KEEP.
#
# Kept as a script so it can be run again on a newer tree:   python3 web/tools/port_fs.py <repo>
import pathlib, re, sys

repo = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()
crates = repo / "crates"
DEPENDENCY = ('# The files, std\'s on the desktop and the page\'s own in a web page '
              '(web/tools/port_fs.py).\nmp-os.workspace = true\n')
USE = "use mp_os::fs::FsExt as _;"
# (file relative to crates/, receiver) whose `.is_file()`, `.is_dir()` or `.exists()` is a type's
# own, not std's.
KEEP = set()
METHOD = re.compile(r"(\b[A-Za-z_][A-Za-z0-9_]*(?:\([^()]*\))?)\.(exists|is_file|is_dir)\(\)")
MODULE = re.compile(r"\bmod\s+[A-Za-z_][A-Za-z0-9_]*\s*\{")


def mask(text):
    """`text` with its comments, strings and character literals blanked, length kept, so that what
    is found in it is code."""
    out = list(text)
    i, n = 0, len(text)

    def blank(start, end):
        for k in range(start, min(end, n)):
            if out[k] != "\n":
                out[k] = " "

    while i < n:
        c = text[i]
        if text.startswith("//", i):
            end = text.find("\n", i)
            end = n if end < 0 else end
            blank(i, end)
            i = end
        elif text.startswith("/*", i):
            depth, j = 1, i + 2
            while j < n and depth:
                if text.startswith("/*", j):
                    depth, j = depth + 1, j + 2
                elif text.startswith("*/", j):
                    depth, j = depth - 1, j + 2
                else:
                    j += 1
            blank(i, j)
            i = j
        elif (raw := re.match(r'b?r(#*)"', text[i:i + 300])) and (i == 0 or not (text[i - 1].isalnum() or text[i - 1] == "_")):
            close = '"' + raw.group(1)
            end = text.find(close, i + raw.end())
            end = n if end < 0 else end + len(close)
            blank(i, end)
            i = end
        elif c == '"':
            j = i + 1
            while j < n and text[j] != '"':
                j += 2 if text[j] == "\\" else 1
            blank(i, j + 1)
            i = j + 1
        elif c == "'":
            literal = re.match(r"'(\\u\{[0-9a-fA-F]+\}|\\.|[^\\'\n])'", text[i:i + 12])
            if literal:
                blank(i, i + literal.end())
                i += literal.end()
            else:
                i += 1  # a lifetime or a label
        else:
            i += 1
    return "".join(out)


def closing(masked, open_at):
    depth = 0
    for k in range(open_at, len(masked)):
        if masked[k] == "{":
            depth += 1
        elif masked[k] == "}":
            depth -= 1
            if depth == 0:
                return k
    return len(masked)


def header_end(text):
    """Where the file's first item starts: after its licence, inner docs and inner attributes."""
    pos = 0
    in_attribute = False
    for line in text.splitlines(keepends=True):
        stripped = line.strip()
        if in_attribute:
            in_attribute = not stripped.endswith("]")
        elif stripped.startswith("#!["):
            in_attribute = not stripped.endswith("]")
        elif not (stripped == "" or (stripped.startswith("//") and not stripped.startswith("///"))):
            break
        pos += len(line)
    return pos


def port(name, text):
    text = re.sub(r"\bstd::fs\b", "mp_os::fs", text)
    masked = mask(text)
    modules = []
    for found in MODULE.finditer(masked):
        brace = found.end() - 1
        modules.append((brace, closing(masked, brace)))
    edits = []  # (position, characters replaced, replacement)
    scopes = set()
    for found in METHOD.finditer(masked):
        if (name, found.group(1)) in KEEP:
            continue
        at = found.start(2)
        edits.append((at, 0, "os_"))
        inside = [module for module in modules if module[0] < at < module[1]]
        scopes.add(max(inside)[0] if inside else None)
    for scope in scopes:
        if scope is None:
            if not re.search(r"^" + re.escape(USE) + r"$", text, re.M):
                edits.append((header_end(text), 0, USE + "\n"))
        else:
            # After the module's own inner docs and attributes, which must come first.
            at = text.find("\n", scope) + 1
            in_attribute = False
            while at < len(text):
                line_end = text.find("\n", at)
                line_end = len(text) if line_end < 0 else line_end + 1
                stripped = text[at:line_end].strip()
                if in_attribute:
                    in_attribute = not stripped.endswith("]")
                elif stripped.startswith("#!["):
                    in_attribute = not stripped.endswith("]")
                elif not (stripped == "" or stripped.startswith("//!")):
                    break
                at = line_end
            body = text[at:]
            indent = re.match(r"[ \t]*", body).group(0) if body.strip() else "    "
            if not body.lstrip().startswith(USE):
                edits.append((at, 0, indent + USE + "\n"))
    for at, length, replacement in sorted(edits, key=lambda edit: edit[0], reverse=True):
        text = text[:at] + replacement + text[at + length:]
    return text


changed_crates = set()
files = 0
for source in sorted(crates.glob("*/src/**/*.rs")):
    crate = source.relative_to(crates).parts[0]
    if source.is_symlink() or crate == "mp-os":
        continue
    name = str(source.relative_to(crates))
    text = source.read_text()
    new = port(name, text)
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
