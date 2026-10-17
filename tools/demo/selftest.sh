#!/usr/bin/env bash
# The demo tooling's guards, without a window: run.sh parses; tour.py parses the crawl's recipe
# and lets through no click the deny lists name - no connection, flight-mode, arming, mission or
# parameter write, no simulation or plugins tab - and plays nothing but blocks the crawl saw happen.
set -uo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
bash -n "$HERE/run.sh" || exit 1
python3 - "$ROOT" <<'PY'
import sys, os
root = sys.argv[1]
sys.path.insert(0, os.path.join(root, "tools", "demo")); sys.path.insert(0, os.path.join(root, "tools", "gui-crawl"))
import tour
recipe = os.path.join(root, "tools", "gui-crawl", "out", "recipe.txt")
if not os.path.exists(recipe):
    print("no recipe at", recipe, "- run tools/gui-crawl/run.sh first"); sys.exit(1)
blocks = tour.read_recipe(recipe)
tour.OPENERS.update(tour.load_openers(os.path.dirname(recipe)))
assert tour.OPENERS, "the crawl's results name no opener"
played = tour.collapse_families(tour.collapse_window_sequences(tour.collapse_dialog_sequences([b for b in blocks if tour.allowed(b)])))
# PLAN's draw area plays once, its first block alone.
draws = [b for b in played if b["screen"] == "plan" and b["clicks"] and b["clicks"][0].startswith("draw-")]
assert len(draws) == 1, [b["clicks"] for b in draws]
# A window opened from a page's button is entered once: the joystick setup from CONFIG > Planner.
joystick = [b for b in played if b["page"] == "config-page-ConfigPlanner" and "planner-BUT_Joystick" in b["clicks"]]
assert len(joystick) == 1, [b["clicks"] for b in joystick]
# Nothing the crawl saw open a browser, an editor or a file is played, nor HELP's Beta Updates.
opened_names = {a.removeprefix("key:") for b in played for a in b["clicks"]}
assert not (opened_names & tour.OPENERS), opened_names & tour.OPENERS
for forbidden in ["help-changelog", "help-tailscale", "help-betaupdate", "help-updatecheck", "plan-kml", "georef-BUT_networklinkgeoref"]:
    assert forbidden not in opened_names, forbidden
# The review page's chooser is opened once: one sequence, and no block that only opens it.
review = [b for b in played if b["page"] == "logs-tab-review"]
dialogs = [b for b in review if any(tour.DIALOG_STEP.match(a) for a in b["clicks"])]
assert len(dialogs) == 1, [b["clicks"] for b in dialogs]
assert not any(b["clicks"] == ["logs-tab-review", "log-open"] for b in review), "the opener alone is played"
assert played, "nothing to play"
names = {a.removeprefix("key:") for b in played for a in b["clicks"]}
for forbidden in ["tab-simulation", "tab-plugins", "main-connect", "plan-write", "disarm", "arm", "fly-action-arm",
                  "fly-flytohere", "fly-setwp", "ctrl+t", "plugin-manager-save", "fence-write", "menu-clearMission",
                  "menu-prefetch", "Acro", "Loiter", "RTL", "Land", "sikradio-BUT_connect", "BUT_connect"]:
    assert forbidden not in names, f"{forbidden} would be played"
assert all(b["verdict"] in tour.PLAYED for b in played)
assert all(b["screen"] in tour.CONNECTED_SCREENS or b["label"].startswith("disc") for b in played)
screens = {b["screen"] for b in played}
for s in ("fly", "plan", "setup", "config"):
    assert s in screens, f"no blocks on {s}"
# What a block left open is closed by name: the NMEA window's prompt cancelled first, then the
# window closed, nothing of the tour's start touched, nothing the deny lists name.
ground = {"tab-fly", "map", "fly-ekf-close", "status-row"}
probe = ground | {"nmea-close", "nmea-prompt-box", "nmea-prompt-ok", "nmea-prompt-cancel", "nmea-CMB_serialport",
                  "plugin-manager-close", "main-connect"}
assert tour.dialogs_to_close(ground, probe) == ["nmea-prompt-cancel", "nmea-close"], tour.dialogs_to_close(ground, probe)
assert tour.dialogs_to_close(ground, ground) == []
# A page's tab already in effect is not clicked again (the owner, 2026-10-17).
assert tour.clicks_to_play(("fly-tab-actions",), ["fly-tab-actions", "fly-changealt"]) == ["fly-changealt"]
assert tour.clicks_to_play((), ["fly-tab-actions", "fly-alt-value"]) == ["fly-tab-actions", "fly-alt-value"]
assert tour.clicks_to_play(("fly-tab-quick",), ["fly-tab-actions", "fly-alt-value"]) == ["fly-tab-actions", "fly-alt-value"]
assert tour.clicks_to_play(("fly-tab-actions",), ["fly-tab-actions"]) == ["fly-tab-actions"]
# The opening: CONFIG > Planner > Video Device Start, the owner's, named exactly as the planner names it.
assert tour.OPENING[0]["clicks"] == ["config-page-ConfigPlanner", "planner-BUT_videostart"]
assert any(b["control"] == "planner-BUT_videostart" for b in blocks), "the recipe knows the button"
# The owner's own sequence: owner.gui's lines as `planner --record` prints them, a block a screen.
import tempfile
with tempfile.NamedTemporaryFile("w", suffix=".gui", delete=False) as f:
    f.write("# mine\nscreen plan\nclick plan-zoomicon\nclick map@0.40x0.40:right\nkey Escape\nscreen fly\nclick tab-fly\n")
owner = tour.read_owner(f.name)
assert [b["screen"] for b in owner] == ["plan", "fly"], owner
assert owner[0]["clicks"] == ["plan-zoomicon", "map@0.40x0.40:right", "key:Escape"], owner[0]
assert tour.read_owner("/nonexistent/owner.gui") == []
# SETUP's Advanced page is kept for the end (the owner, 2026-10-17); its blocks are many.
tail = [b for b in played if b["page"] in tour.LAST_PAGES]
assert tail and all(b["page"] == "setup-page-ConfigAdvanced" for b in tail), len(tail)
print(f"tour: {len(played)} of {len(blocks)} blocks playable, {len(tail)} kept for the end; the deny lists hold; leftovers close by name")
PY
