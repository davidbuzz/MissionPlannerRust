# The ledger audit of 2026-10-16

The owner asked (2026-10-16) that the 283 rows of `ledger/ledger.csv` still `ready` while the
Rust cites their C# (`// C#: File.cs:lines` or a `Ported from` header) be read and set as they
stand. Four readers took 70 or 71 rows each, read the C# file and the Rust that cites it, and
proposed each row's state, classes, evidence, omissions and notes under the ledger's rules
(`xtask/src/ledger/mod.rs`); proposals of low confidence were left `ready` for the owner. Every
evidence path exists; every `ported`/`tested` row of size M and up has its omissions;
`cargo xtask ledger check` is ok at 3,684 rows. Commits 8176daa, 70cd0dd.

**Outcome:** 249 rows to `tested`, 3 to `ported` (ConfigParamLoading.Designer, OpenDroneID_Backend,
GStreamerUI: code with no test of its own), 31 left `ready` with notes saying what is cited and why
they stay. Of the 252 moved, 209 are `behaviour-equivalent`, 32 `bit-exact` (the DroneCAN wire
codecs, round-tripped), 9 `redesigned`, 2 `tolerance-bounded`; 231 `spec+review`, 18 `oracle` and
3 `golden` (only where a test holds the Rust to the C#'s own output under mono: the KML and georef
goldens, MAVLink.dll's dumps, the GMap provider oracle, the grid goldens, a cache file name the C#
wrote).

## For the owner: the rows left `ready`, with a recommendation each

1. **ZedGraph, 12 rows here, 107 in all** (Axis, DateScale, GraphPane, Line, LineBase, MasterPane,
   Scale, Symbol, XDate, ZedGraphControl and its ContextMenu and Events). The library is replaced
   (mp-chart and the screens' own drawing), not ported; its zoom, pan and context menu are
   reproduced in the log browser alone, while every ZedGraph control in the C# has them.
   **Recommend:** the missing zoom, pan and menu on the other graphs go to the matrix (below), and
   the 107 rows are `dropped` with the owner's reason - "replaced by mp-chart; its interactions
   ported per graph" - once that row is done.
2. **ThemeManager.cs** (one constant cited). **Recommend:** `dropped` - the owner's ruling of
   2026-09-23 keeps the dark palette.
3. **UserAlert.cs** (a file name cited): its start-up download writes `UserAlerts.json` and sets
   `Instance`, and nothing in the C# reads either. **Recommend:** `dropped`, "no reader in the C#".
4. **MavFtpDokan.cs** (an error's words cited): the Windows-only Dokan driver binding; Mount as
   Drive fails on the status line here as the C# does without Dokan. **Recommend:** `dropped`,
   "Dokan is Windows-only; the status line says it is not installed".
5. **ArcGIS_Imagery_World_2D_MapProvider.cs** (its URL order cited): the C#'s endpoint is retired;
   the Esri source here uses another endpoint and cache directory, a choice `source.rs` leaves to
   the owner. **Recommend:** `dropped`, "retired endpoint; the Esri source replaces it".
6. **MissionPlanner.Drawing/Image.cs** (the JPEG quality cited): the SkiaSharp stand-in for
   System.Drawing off Windows; the `image` crate replaces it. **Recommend:** `dropped`.
7. **Extensions.cs**: about ninety helpers with no port of their own; eight are ported and tested
   where used. **Recommend:** stays `ready` until the port's end, when the helpers nothing calls
   are its omissions ("no caller") and the row goes `ported`.
8. **Sysid32Tests.cs** - and every `MissionPlannerTests` row, all still `ready`: 6 of its 18 tests
   are ported. **Recommend:** a test file counts `tested` when the Rust's tests cover its cases;
   the twelve remaining are ported as one batch (they test our MAVLink crate) and the row set.
9. **GMaps.cs, GMarkerGoogle.cs, ObjectListView.cs** (proposed `tested`, low confidence) and
   **GMapRoute.cs** (proposed `ported`: its default pen alone): in each the Rust redesigns the file
   or ports only the part the application uses, and that part is tested. **Recommend:** accept
   them as `tested`, `redesigned`; GMapRoute `ready` until the route's drawing (below) is ported.
10. **MavlinkLogBase.cs**: `ready` with only `writeKML`'s per-mode path ported, yet
    `crates/mp-kml/src/lib.rs` carries a `Ported from Log/MavlinkLogBase.cs` header. The header
    asserts a port the row denies. **Recommend:** `ported` with the omissions written (the form's
    other outputs), the header kept. **MavlinkLog.cs** stays `ready`: its "Tlog > Kml or Graph"
    form is not ported (below).
11. **ConnectionControl.cs, ConnectionStats.cs, CheckListControl.cs, CustomMessageBox.cs,
    Propagation.cs, Speech.cs**: a line, a constant or a helper cited while the behaviour is not
    ported (the vehicle selector and Link Stats; the pre-flight checklist; the message box's link
    markup, wrapping and icons; the propagation overlay; speech - Deliverable 15). They stay
    `ready` and their gaps are in the matrix rows below. No decision needed.
12. **`MAVLinkParam.SetValue`**: the Rust caps an out-of-range value where the C# wraps it (300
    typed into a UINT8 becomes 255 here, 44 there). **Recommend:** keep the cap and write the
    divergence at the site - the wrap is a C# defect a pilot would not thank us for.

## Gaps with callers, found by the readers (to the matrix as one row)

- FLIGHT PLAN's map: the mission route as `WPOverlay.RegenerateWPRoute` draws it (4 px yellow,
  dashed legs to home, spline curves - a thin blue line here); fence circles; the rally points'
  "Rally Point / Alt" tooltip; the selected waypoint's red disc and the Ctrl box-select that sets
  it; the `SelectedArea` rubber band (Prefetch takes the whole view without it); the Dist to Home
  and the survey dialog's "Line:" tooltips; marker pins beyond green and red; map rotation and the
  scale bar.
- FLIGHT DATA: the position read-out under the map (`Coords.cs`, `coords1`); the vehicle marker's
  tooltip (`mapicondesc`); the pre-flight checklist (`CheckListControl.cs`).
- The message box's `[link;url;text]` markup (the zero-altitude plane warning shows it raw).
- SETUP and CONFIG: a page popped out into its own window on a double-click
  (`BackstageView.cs`).
- The Live Calibration sphere's arrow keys (`Sphere.cs`).
- The parameter cache `param.json`, written after every change and read with `UseCachedParams`
  (`MAVState.cs`).
- The connection control's vehicle (sysid) selector and Link Stats label.
- LOGS: MavlinkLog's "Tlog > Kml or Graph" form (Create KML is `headless-planner kml` alone).
- Every graph's zoom, pan and context menu (ZedGraph's, reproduced in the log browser alone).
- The OpenDroneID backend never sends: it waits for `OPEN_DRONE_ID_ARM_STATUS`, and the plugin
  interface (`wit/plugin.wit`) has no way to subscribe to a packet, so its code and the arm-status
  half of `OpenDroneID_UI.cs` cannot run.
- CONFIG > Planner's Beta Updates box is dimmed "no updater" while mp-update ports
  `Utilities/Update.cs` and `help.rs` reads `beta_updates`.
- ConfigRawParams' Load, Save and Compare are `params.rs`'s own typed-file panel, not the chooser
  every other file dialog here uses (the owner's word of 2026-10-16 on Browse).

## Behaviour that differs from the C#, found by the readers (to the matrix as one row)

- The UDP transport sends to the last peer heard; the C# sends every write to each endpoint it has
  heard from (`CommsUdpSerial.EndPointList`) - several sources on one port.
- `RoundToSignificantDigits` rounds an exact half away from zero; the C# rounds to even.
- The OpenStreetMap geocoder's results are not cached: every Zoom To search asks the server.
- The OpenStreetMap provider stops at zoom 19; the C# sets no limit.
- `getFilefromNet` always downloads; the C# skips a file on disk matching the server's date and
  length.
- `ParameterMetaDataRepositoryAPM`: the bundled fallback table holds the copter section alone and
  answers for every vehicle; the C# reads the vehicle's own section.
- `pdef.rs`'s `from_banner`: for a banner with one letter, "V4.5a", the C# reads an empty group,
  throws and gives no version; the Rust's test pins 4.5.49.
- Shared controls (`MavlinkCheckBox`, `MavlinkComboBox`, `MavlinkNumericUpDown`, `InputBox`, the
  two `HorizontalProgressBar`s) are re-implemented per screen, not as one module - `tested` at
  medium confidence.

## Set right on the way

- The seven DroneCAN rows marked `tested` earlier pointed `target_crate` at mp-gui while their
  notes named mp-dronecan: they point at mp-dronecan.
- `gimbal_video.rs` cited `ClickBindingButton.cs:415-430` and `:361-376` in a file of 208 lines:
  `GetClickBindingString` is 168-183 and `OnKeyDown` 114-129.
