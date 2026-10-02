#!/bin/bash
# The goldens for mp_log::analysis: ArduPilot's LogAnalyzer - the Python 2 source Mission Planner
# ships as runner.exe, in references/missionplanner/LogAnalyzer/py2exe - run by Python 2.7 in a
# container over the text the port's own "Convert .Bin to .Log" makes of each checked-in .bin, and
# over the text logs as they are. The runner lists its tests folder by name (NTFS), so the copy run
# here sorts the glob. The runner reads a log in Windows text mode, where "\r\n" is one byte, so
# a CRLF log is fed to the Linux interpreter with its "\r" dropped: the sizes then agree.
#
# The container's numpy (1.16.6, the last for Python 2) differs from the runner's (1.11.2) only in
# polyfit's covariance scaling, which no golden reaches (none has optical flow data); the port
# follows 1.11.2 and tests that case itself.
#
#   tools/loganalyzer-golden.sh            regenerate testdata/dataflash/golden/loganalysis/*.xml
#   tools/loganalyzer-golden.sh <dir> a.bin b.log ...   write <dir>/<name>.log and <name>.xml for
#                                          each, a corpus for `MP_LOGANALYZER_CORPUS`
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd)
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

mkdir -p "$WORK/la" "$WORK/logs" "$WORK/out"
cp "$ROOT"/references/missionplanner/LogAnalyzer/py2exe/*.py "$WORK/la/"
cp -r "$ROOT"/references/missionplanner/LogAnalyzer/py2exe/tests "$WORK/la/"
sed -i "s|testScripts = glob.glob(dirName + '/tests/\*.py')|testScripts = sorted(glob.glob(dirName + '/tests/*.py'))|" "$WORK/la/LogAnalyzer.py"
grep -q "sorted(glob.glob" "$WORK/la/LogAnalyzer.py"

printf 'FROM python:2.7.18-slim\nRUN pip install --no-cache-dir numpy==1.16.6\n' > "$WORK/Dockerfile"
docker build -q -t py27-loganalyzer "$WORK" >/dev/null

PLANNER="$ROOT/target/debug/headless-planner"
[ -x "$PLANNER" ] || cargo build -p mp-cli --bin headless-planner

# to_text <log> <name>: <name>.log in $WORK/logs, LF, converted first if a .bin.
to_text() {
  case "${1,,}" in
    *.bin) "$PLANNER" log bintolog "$1" "$WORK/logs/$2.crlf.log" >/dev/null
           tr -d '\r' < "$WORK/logs/$2.crlf.log" > "$WORK/logs/$2.log" ;;
    *)     tr -d '\r' < "$1" > "$WORK/logs/$2.log" ;;
  esac
}

analyse() {
  docker run --rm -v "$WORK:/w" -w /w/la py27-loganalyzer \
    python LogAnalyzer.py -q -x "/w/out/$1.xml" -s "/w/logs/$1.log"
}

if [ $# -eq 0 ]; then
  OUT="$ROOT/testdata/dataflash/golden/loganalysis"
  to_text "$ROOT/testdata/dataflash.bin" dataflash
  to_text "$ROOT/testdata/dataflash/edge.bin" edge
  to_text "$ROOT/testdata/dataflash/synthetic.log" synthetic
  to_text "$ROOT/testdata/dataflash/loganalyzer/robert_lefebvre_octo_PM.log" robert_lefebvre_octo_PM
  for n in dataflash edge synthetic robert_lefebvre_octo_PM; do
    analyse "$n"
    cp "$WORK/out/$n.xml" "$OUT/$n.xml"
    echo "$OUT/$n.xml"
  done
else
  OUT=$1; shift
  mkdir -p "$OUT"
  for log in "$@"; do
    n=$(basename "$log"); n=${n%.*}
    to_text "$log" "$n"
    analyse "$n"
    cp "$WORK/logs/$n.log" "$OUT/$n.log"
    cp "$WORK/out/$n.xml" "$OUT/$n.xml"
    echo "$OUT/$n.xml"
  done
fi
