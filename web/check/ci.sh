#!/usr/bin/env bash
# The browser checks CI runs on the page it has just built, before the site is published
# (.github/workflows/pages.yml; the matrix's browser odds-and-ends row, "the browser checks in CI").
# Each opens the planner in headless Chromium and fails on what it was written for or on any fault
# in the page. The ones that need nothing beyond the page: no Go (the tailnet's), no simulated
# radio, no network but the page's own server.
#
#   web/check/ci.sh <site-dir> <out-dir> [port]
#
# <site-dir> is what web/tools/pages-site.sh assembled, served as GitHub Pages serves it for
# check/pages_check.js; www/ itself is served on [port] (8080) for the rest. Playwright is looked
# for beside the checks (npm install --prefix web/check playwright) or on NODE_PATH. Exits 0 when
# every check passed; each one's log and screenshots are left in <out-dir>.
set -u
here=$(cd "$(dirname "$0")" && pwd)
site=${1:?the site pages-site.sh assembled}
out=${2:?where to leave the logs}
port=${3:-8080}
mkdir -p "$out"

python3 "$here/../www/serve.py" "$port" > "$out/serve.log" 2>&1 &
server=$!
trap 'kill $server 2>/dev/null' EXIT
for _ in $(seq 1 50); do
    curl -s -o /dev/null "http://127.0.0.1:$port/" && break
    sleep 0.2
done

failed=0
run() {
    local check=$1 query=$2
    mkdir -p "$out/$check"
    if timeout 400 node "$here/$check.js" "$out/$check" "http://127.0.0.1:$port/$query" > "$out/$check.log" 2>&1 \
        && grep -q '^PASS' "$out/$check.log"; then
        echo "$check: PASS"
    else
        echo "$check: FAIL"
        grep -E '^(FAIL|error)' "$out/$check.log" | head -5
        failed=$((failed + 1))
    fi
}
run planner_check "?vehicle=copter&demo=0&facts=1"
run sim_check "?facts=1&demo=0"
run tour_check "?vehicle=copter&demo=0&facts=1"
run files_check "?facts=1&demo=0"
run storage_check "?facts=1&demo=0"
run serial_check "?facts=1&demo=0"
run maptype_check "?facts=1&demo=0"
run plugins_check "?facts=1&demo=0"

# The site as Pages serves it: under the project's path, with no header of its own.
mkdir -p "$out/pages-root"
rm -rf "$out/pages-root/MissionPlannerRust"
cp -r "$site" "$out/pages-root/MissionPlannerRust"
mkdir -p "$out/pages_check"
if timeout 400 node "$here/pages_check.js" "$out/pages-root" "$out/pages_check" > "$out/pages_check.log" 2>&1 \
    && grep -q '^PASS' "$out/pages_check.log"; then
    echo "pages_check: PASS"
else
    echo "pages_check: FAIL"
    grep -E '^(FAIL|error)' "$out/pages_check.log" | head -5
    failed=$((failed + 1))
fi
rm -rf "$out/pages-root"

echo "$failed of 9 failed"
[ "$failed" -eq 0 ]
