#!/bin/bash
# What macOS does when a program's file is replaced and the new one started: the facts
# updates.md's "the planner replaces itself" rests on for a Mac (the owner's question of
# 2026-11-18, "do we need to test macos ?"). It opens no window and asks Gatekeeper nothing that
# could put a dialog on the screen: no quarantined file is ever run.
#
#   tools/macos/self-replace-test.sh [missionplanner-rust_<v>_macos-universal.tar.gz]
#
# Parts A to C use two small C programs built here, signed by the linker as rustc's output is
# (ad hoc, linker-signed): "old" stands for the installed planner and "new" for the downloaded
# one. Part A, the new program written into the old one's file against a fresh file moved over
# it. Part B, the same while the old program runs. Part C, the running program moving itself
# aside, moving the new one in and starting it. Part D, what a download and an unpack leave on a
# file, and - with a release archive named - the real planner unpacked, checked and swapped.
# A line says what happened; "ok" or "DIFF" where an outcome was expected, "seen" where it was
# only to be learned. The exit code is the number of DIFF lines.

set -u
archive=${1:-}
work=$(mktemp -d "${TMPDIR:-/tmp}/mp-selfreplace.XXXXXX")
trap 'rm -rf "$work"' EXIT
unexpected=0

check() { # what, expected, got
    if [ "$2" = "$3" ]; then
        printf '  ok   %s: %s\n' "$1" "$3"
    else
        unexpected=$((unexpected + 1))
        printf '  DIFF %s: %s (expected %s)\n' "$1" "$3" "$2"
    fi
}
seen() { printf '  seen %s: %s\n' "$1" "$2"; }

# What running a program came to: what it said, its exit code, or the signal that killed it.
run() {
    local out rc
    out=$("$@" 2>&1)
    rc=$?
    if [ $rc -gt 128 ]; then
        echo "killed by signal $((rc - 128))"
    elif [ $rc -ne 0 ]; then
        echo "exit $rc: $out"
    else
        echo "said '$out'"
    fi
}
# Whether an operation was let through.
attempt() { if "$@" > /dev/null 2>"$work/why"; then echo allowed; else echo "refused ($(tr '\n' ' ' < "$work/why"))"; fi; }
inode() { stat -f %i "$1"; }
quarantine() { if xattr -p com.apple.quarantine "$1" > /dev/null 2>&1; then echo marked; else echo none; fi; }

echo "macOS $(sw_vers -productVersion) ($(sw_vers -buildVersion)) on $(uname -m); $(diskutil info / 2>/dev/null | awk -F': *' '/File System Personality/ {print $2}')"

cat > "$work/old.c" <<'C'
#include <stdio.h>
#include <string.h>
#include <unistd.h>
int main(int argc, char **argv) {
    if (argc > 1 && !strcmp(argv[1], "wait")) { sleep(120); return 0; }
    if (argc > 3 && !strcmp(argv[1], "swap")) {
        /* The planner's three steps: itself aside, the new file in, the new file started. */
        char aside[4096];
        snprintf(aside, sizeof aside, "%s.old", argv[2]);
        if (rename(argv[2], aside)) { perror("rename self"); return 2; }
        if (rename(argv[3], argv[2])) { perror("rename staged"); return 3; }
        execl(argv[2], argv[2], (char *)0);
        perror("exec");
        return 4;
    }
    puts("old");
    return 0;
}
C
cat > "$work/new.c" <<'C'
#include <stdio.h>
int main(void) { puts("new"); return 0; }
C
cc -o "$work/old" "$work/old.c" && cc -o "$work/new" "$work/new.c" || { echo "cc failed"; exit 99; }
seen "how the stand-ins are signed" "$(codesign -dv "$work/old" 2>&1 | grep -E '^(CodeDirectory|Signature)' | sed 's/^CodeDirectory.*flags=/flags=/; s/ hashes=.*//' | tr '\n' ' ')"

# ---- Part A: the new program written into the old file, against a fresh file moved over it ----
echo
echo "Part A: after the old program has run - written into its file, or moved over it"
d="$work/a"; mkdir "$d"
cp "$work/old" "$d/app"
check "the old program runs" "said 'old'" "$(run "$d/app")"
before=$(inode "$d/app")
cat "$work/new" > "$d/app"
check "written into the same file, it is the same file" "$before" "$(inode "$d/app")"
# With the old program ended this ran on macOS 26.3 (2026-11-18); written into while the old
# program runs, Part B, the name is killed at its next start - the signature kept for the file.
seen "the new program, written into the old one's file" "$(run "$d/app")"
cp "$work/old" "$d/app2"
run "$d/app2" > /dev/null
before=$(inode "$d/app2")
cp "$work/new" "$d/app2.staged"
check "a fresh file moved over the old one" "allowed" "$(attempt mv "$d/app2.staged" "$d/app2")"
if [ "$before" != "$(inode "$d/app2")" ]; then moved=different; else moved=same; fi
check "moved over, it is a different file" "different" "$moved"
check "the new program, moved over the old one, runs" "said 'new'" "$(run "$d/app2")"

# ---- Part B: the same while the old program runs ----------------------------------------------
echo
echo "Part B: while the old program runs"
d="$work/b"; mkdir "$d"
cp "$work/old" "$d/app"
"$d/app" wait &
pid=$!
sleep 1
check "the old program is running" "yes" "$(kill -0 $pid 2>/dev/null && echo yes || echo no)"
cp "$work/new" "$d/app.staged"
check "a fresh file moved over the running program's" "allowed" "$(attempt mv "$d/app.staged" "$d/app")"
check "the name now runs the new program" "said 'new'" "$(run "$d/app")"
check "the old program is still running" "yes" "$(kill -0 $pid 2>/dev/null && echo yes || echo no)"
kill $pid 2>/dev/null; wait $pid 2>/dev/null
cp "$work/old" "$d/busy"
"$d/busy" wait &
pid=$!
sleep 1
seen "writing into a running program's file" "$(attempt sh -c "cat '$work/new' > '$d/busy'")"
seen "the old program after its file was written into" "$(kill -0 $pid 2>/dev/null && echo 'still running' || echo ended)"
check "the name after it was written into" "killed by signal 9" "$(run "$d/busy")"
check "deleting a running program's file" "allowed" "$(attempt rm "$d/busy")"
kill $pid 2>/dev/null; wait $pid 2>/dev/null

# ---- Part C: the running program does it to itself --------------------------------------------
echo
echo "Part C: the running program moves itself aside, moves the new one in, starts it"
d="$work/c"; mkdir "$d"
cp "$work/old" "$d/app"
cp "$work/new" "$d/app.staged"
check "the old program's last act started the new one" "said 'new'" "$(run "$d/app" swap "$d/app" "$d/app.staged")"
check "the old program is beside it as app.old" "said 'old'" "$(run "$d/app.old")"
check "app.old deleted at the next start" "allowed" "$(attempt rm "$d/app.old")"

# ---- Part D: what a download and an unpack leave on a file ------------------------------------
echo
echo "Part D: downloads, the quarantine mark, and the real planner"
d="$work/d"; mkdir "$d"
if curl -sL -m 20 -o "$d/fetched" https://example.com/; then
    check "a file this session downloaded carries the quarantine mark" "none" "$(quarantine "$d/fetched")"
else
    seen "a download" "not possible from here"
fi
if [ -n "$archive" ] && [ -f "$archive" ]; then
    seen "the archive's own quarantine mark" "$(quarantine "$archive")"
    mkdir "$d/installed" "$d/staged"
    tar -xzf "$archive" -C "$d/installed" --strip-components 1
    tar -xzf "$archive" -C "$d/staged" --strip-components 1
    planner="$d/installed/planner"
    check "the unpacked planner's quarantine mark" "none" "$(quarantine "$planner")"
    seen "its architectures" "$(lipo -archs "$planner")"
    # Apple Silicon starts nothing unsigned; Intel asks for no signature and its linker adds none.
    check "the Apple Silicon half's signature after the unpack" "valid" "$(codesign -v --arch arm64 "$planner" > /dev/null 2>&1 && echo valid || echo invalid)"
    seen "the Intel half's signature" "$(codesign -v --arch x86_64 "$planner" 2>&1 | sed 's/.*: //' | head -1)"
    seen "how it is signed" "$(codesign -dv "$planner" 2>&1 | grep -E '^(CodeDirectory|Signature)' | sed 's/^CodeDirectory.*flags=/flags=/; s/ hashes=.*//' | tr '\n' ' ')"
    seen "Gatekeeper's assessment, asked without running it" "$(spctl --assess --type execute "$planner" 2>&1 | tr '\n' ' ')"
    version=$(run "$planner" --version)
    seen "the installed planner's --version" "$version"
    check "the staged planner moved over the installed one" "allowed" "$(attempt mv "$d/staged/planner" "$planner")"
    check "the planner runs after the swap as before it" "$version" "$(run "$planner" --version)"
    seen "the same, as Intel under Rosetta" "$(run arch -x86_64 "$planner" --version)"
else
    seen "the real planner" "no archive named, not tried"
fi

echo
echo "self-replace-test: $unexpected line(s) other than expected"
exit $unexpected
