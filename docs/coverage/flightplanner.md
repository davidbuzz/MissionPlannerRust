# FlightPlanner action coverage

Generated from `crates/mp-gui/src/planner_coverage.rs` by `cargo test -p mp-gui planner_coverage -- --ignored update_report`; a test fails when this file is stale. One row per event wiring in `GCSViews/FlightPlanner.Designer.cs`.

| total | done | elsewhere | missing | plumbing | dropped |
|---:|---:|---:|---:|---:|---:|
| 121 | 40 | 0 | 69 | 12 | 0 |

Missing, by where the control sits:

| group | missing |
|---|---:|
| planning panel | 20 |
| Map Tool | 7 |
| polygon icon menu | 7 |
| Geo-Fence | 6 |
| Rally Points | 6 |
| Auto WP | 5 |
| Polygon | 5 |
| map menu | 4 |
| File Load/Save | 3 |
| POI | 3 |
| zoom icon menu | 3 |

| control | event | handler | text | ours |
|---|---|---|---|---|
| `TXT_WPRad` | KeyPress | `TXT_WPRad_KeyPress` | WP Radius | **missing** |
| `TXT_WPRad` | Leave | `TXT_WPRad_Leave` | WP Radius | **missing** |
| `TXT_DefaultAlt` | KeyPress | `TXT_DefaultAlt_KeyPress` | Default Alt | **missing** |
| `TXT_DefaultAlt` | Leave | `TXT_DefaultAlt_Leave` | Default Alt | **missing** |
| `TXT_loiterrad` | KeyPress | `TXT_loiterrad_KeyPress` | Loiter Radius | **missing** |
| `TXT_loiterrad` | Leave | `TXT_loiterrad_Leave` | Loiter Radius | **missing** |
| `but_writewpfast` | Click | `but_writewpfast_Click` | Write Fast | **missing** |
| `BUT_write` | Click | `BUT_write_Click` | Write | done: `plan-write` |
| `BUT_read` | Click | `BUT_read_Click` | Read | done: `plan-read` |
| `label4` | LinkClicked | `label4_LinkClicked` | Home Location | **missing** |
| `TXT_homealt` | TextChanged | `TXT_homealt_TextChanged` | Home Location: ASL | **missing** |
| `TXT_homelng` | TextChanged | `TXT_homelng_TextChanged` | Home Location: Long | **missing** |
| `TXT_homelat` | TextChanged | `TXT_homelat_TextChanged` | Home Location: Lat | **missing** |
| `TXT_homelat` | Enter | `TXT_homelat_Enter` | Home Location: Lat | **missing** |
| `coords1` | SystemChanged | `coords1_SystemChanged` | the pointer coordinates: system | **missing** |
| `chk_usemavftp` | CheckedChanged | `chk_usemavftp_CheckedChanged` | MAVFTP | **missing** |
| `but_mincommands` | Click | `but_mincommands_Click` | ˅ | **missing** |
| `CMB_altmode` | SelectedIndexChanged | `CMB_altmode_SelectedIndexChanged` | the altitude frame | done: `fn set_altitude_frame` |
| `CHK_splinedefault` | CheckedChanged | `CHK_splinedefault_CheckedChanged` | Spline | **missing** |
| `Commands` | CellContentClick | `Commands_CellContentClick` | the waypoint grid: Delete, Up, Down | done: `fn row_controls` |
| `Commands` | CellEndEdit | `Commands_CellEndEdit` | the waypoint grid: a cell edited | done: `fn editor_panel` |
| `Commands` | DataError | `Commands_DataError` | the waypoint grid | plumbing |
| `Commands` | DefaultValuesNeeded | `Commands_DefaultValuesNeeded` | the waypoint grid: a new row's frame | done: `fn add_waypoint_in` |
| `Commands` | EditingControlShowing | `Commands_EditingControlShowing` | the waypoint grid | plumbing |
| `Commands` | RowEnter | `Commands_RowEnter` | the waypoint grid: the row's parameter names | done: `fn editor_panel` |
| `Commands` | RowsAdded | `Commands_RowsAdded` | the waypoint grid | plumbing |
| `Commands` | RowsRemoved | `Commands_RowsRemoved` | the waypoint grid | plumbing |
| `Commands` | RowValidating | `Commands_RowValidating` | the waypoint grid | plumbing |
| `BUT_Add` | Click | `BUT_Add_Click` | Add Below | **missing** |
| `BUT_InjectCustomMap` | Click | `BUT_InjectCustomMap_Click` | Inject Custom Map | **missing** |
| `chk_grid` | CheckedChanged | `chk_grid_CheckedChanged` | Grid | **missing** |
| `lnk_kml` | LinkClicked | `lnk_kml_LinkClicked` | View KML | **missing** |
| `BUT_loadwpfile` | Click | `BUT_loadwpfile_Click` | Load File | done: `plan-load` |
| `BUT_saveWPFile` | Click | `BUT_saveWPFile_Click` | Save File | done: `plan-save` |
| `panelMap` | Resize | `panelMap_Resize` | the map panel | plumbing |
| `Zoomlevel` | ValueChanged | `Zoomlevel_ValueChanged` | Zoom | done: `map` |
| `TRK_zoom` | Scroll | `TRK_zoom_Scroll` | Zoom | done: `map` |
| `cmb_missiontype` | SelectedIndexChanged | `Cmb_missiontype_SelectedIndexChanged` | Mission / Fence / Rally | done: `draw-fence` |
| `MainMap` | Paint | `MainMap_Paint` | the map | plumbing |
| `contextMenuStrip1` | Closed | `contextMenuStrip1_Closed` | the map's right-click menu | plumbing |
| `contextMenuStrip1` | Opening | `contextMenuStrip1_Opening` | the map's right-click menu | done: `fn open_map_menu` |
| `deleteWPToolStripMenuItem` | Click | `deleteWPToolStripMenuItem_Click` | Delete WP | done: `menu-deleteWP` |
| `insertWpToolStripMenuItem` | Click | `insertWpToolStripMenuItem_Click` | Insert Wp | done: `menu-insertWp` |
| `currentPositionToolStripMenuItem` | Click | `currentPositionToolStripMenuItem_Click` | At Current Position | done: `menu-currentPosition` |
| `insertSplineWPToolStripMenuItem` | Click | `insertSplineWPToolStripMenuItem_Click` | Insert Spline WP | done: `menu-insertSplineWP` |
| `loiterForeverToolStripMenuItem` | Click | `loiterForeverToolStripMenuItem_Click` | Forever | done: `menu-loiterForever` |
| `loitertimeToolStripMenuItem` | Click | `loitertimeToolStripMenuItem_Click` | Time | done: `menu-loitertime` |
| `loitercirclesToolStripMenuItem` | Click | `loitercirclesToolStripMenuItem_Click` | Circles | done: `menu-loitercircles` |
| `jumpstartToolStripMenuItem` | Click | `jumpstartToolStripMenuItem_Click` | Start | done: `menu-jumpstart` |
| `jumpwPToolStripMenuItem` | Click | `jumpwPToolStripMenuItem_Click` | WP # | done: `menu-jumpwP` |
| `rTLToolStripMenuItem` | Click | `rTLToolStripMenuItem_Click` | RTL | done: `menu-rTL` |
| `landToolStripMenuItem` | Click | `landToolStripMenuItem_Click` | Land | done: `menu-land` |
| `takeoffToolStripMenuItem` | Click | `takeoffToolStripMenuItem_Click` | Takeoff | done: `menu-takeoff` |
| `setROIToolStripMenuItem` | Click | `setROIToolStripMenuItem_Click` | DO_SET_ROI | done: `menu-setROI` |
| `clearMissionToolStripMenuItem` | Click | `clearMissionToolStripMenuItem_Click` | Clear Mission | done: `menu-clearMission` |
| `addPolygonPointToolStripMenuItem2` | Click | `addPolygonPointToolStripMenuItem_Click` | Draw a Polygon | done: `menu-addPolygonPoint2` |
| `clearPolygonToolStripMenuItem2` | Click | `clearPolygonToolStripMenuItem_Click` | Clear Polygon | done: `menu-clearPolygon2` |
| `savePolygonToolStripMenuItem2` | Click | `savePolygonToolStripMenuItem_Click` | Save Polygon | **missing** |
| `loadPolygonToolStripMenuItem2` | Click | `loadPolygonToolStripMenuItem_Click` | Load Polygon | **missing** |
| `fromSHPToolStripMenuItem2` | Click | `fromSHPToolStripMenuItem_Click` | From SHP | **missing** |
| `fromCurrentWaypointsToolStripMenuItem` | Click | `fromCurrentWaypointsMenuItem_Click` | From Current Waypoints | done: `menu-fromCurrentWaypoints` |
| `offsetPolygonToolStripMenuItem2` | Click | `offsetPolygonToolStripMenuItem_Click` | Offset Polygon | **missing** |
| `areaToolStripMenuItem2` | Click | `areaToolStripMenuItem_Click` | Area | **missing** |
| `GeoFenceuploadToolStripMenuItem` | Click | `GeoFenceuploadToolStripMenuItem_Click` | Upload | **missing** |
| `GeoFencedownloadToolStripMenuItem` | Click | `GeoFencedownloadToolStripMenuItem_Click` | Download | **missing** |
| `setReturnLocationToolStripMenuItem` | Click | `setReturnLocationToolStripMenuItem_Click` | Set Return Location | **missing** |
| `loadFromFileToolStripMenuItem` | Click | `loadFromFileToolStripMenuItem_Click` | Load from File | **missing** |
| `saveToFileToolStripMenuItem` | Click | `saveToFileToolStripMenuItem_Click` | Save to File | **missing** |
| `clearToolStripMenuItem` | Click | `clearToolStripMenuItem_Click` | Clear | **missing** |
| `setRallyPointToolStripMenuItem` | Click | `setRallyPointToolStripMenuItem_Click` | Set Rally Point | **missing** |
| `getRallyPointsToolStripMenuItem` | Click | `getRallyPointsToolStripMenuItem_Click` | Download | **missing** |
| `saveRallyPointsToolStripMenuItem` | Click | `saveRallyPointsToolStripMenuItem_Click` | Upload | **missing** |
| `clearRallyPointsToolStripMenuItem` | Click | `clearRallyPointsToolStripMenuItem_Click` | Clear Rally Points | **missing** |
| `saveToFileToolStripMenuItem1` | Click | `saveToFileToolStripMenuItem1_Click` | Save Rally to File | **missing** |
| `loadFromFileToolStripMenuItem1` | Click | `loadFromFileToolStripMenuItem1_Click` | Load Rally from File | **missing** |
| `createWpCircleToolStripMenuItem` | Click | `createWpCircleToolStripMenuItem_Click` | Create Wp Circle | **missing** |
| `createSplineCircleToolStripMenuItem` | Click | `createSplineCircleToolStripMenuItem_Click` | Create Spline Circle | **missing** |
| `areaToolStripMenuItem1` | Click | `areaToolStripMenuItem_Click` | Area | **missing** |
| `textToolStripMenuItem` | Click | `textToolStripMenuItem_Click` | Text | **missing** |
| `createCircleSurveyToolStripMenuItem` | Click | `createCircleSurveyToolStripMenuItem_Click` | Create Circle Survey | **missing** |
| `surveyGridToolStripMenuItem` | Click | `surveyGridToolStripMenuItem_Click` | Survey (Grid) | done: `survey-generate` |
| `ContextMeasure` | Click | `ContextMeasure_Click` | Measure Distance | done: `menu-ContextMeasure` |
| `rotateMapToolStripMenuItem` | Click | `rotateMapToolStripMenuItem_Click` | Rotate Map | **missing** |
| `zoomToToolStripMenuItem` | Click | `zoomToToolStripMenuItem_Click` | Zoom To | **missing** |
| `prefetchToolStripMenuItem` | Click | `prefetchToolStripMenuItem_Click` | Prefetch | **missing** |
| `prefetchWPPathToolStripMenuItem` | Click | `prefetchWPPathToolStripMenuItem_Click` | Prefetch WP Path | **missing** |
| `kMLOverlayToolStripMenuItem` | Click | `kMLOverlayToolStripMenuItem_Click` | KML Overlay | **missing** |
| `elevationGraphToolStripMenuItem` | Click | `elevationGraphToolStripMenuItem_Click` | Elevation Graph | **missing** |
| `reverseWPsToolStripMenuItem` | Click | `reverseWPsToolStripMenuItem_Click` | Reverse WPs | done: `menu-reverseWPs` |
| `loadWPFileToolStripMenuItem` | Click | `loadWPFileToolStripMenuItem_Click` | Load WP File | done: `menu-loadWPFile` |
| `loadAndAppendToolStripMenuItem` | Click | `loadAndAppendToolStripMenuItem_Click` | Load and Append | **missing** |
| `saveWPFileToolStripMenuItem` | Click | `saveWPFileToolStripMenuItem_Click` | Save WP File | done: `menu-saveWPFile` |
| `loadKMLFileToolStripMenuItem` | Click | `loadKMLFileToolStripMenuItem_Click` | Load KML File | **missing** |
| `loadSHPFileToolStripMenuItem` | Click | `loadSHPFileToolStripMenuItem_Click` | Load SHP File | **missing** |
| `poiaddToolStripMenuItem` | Click | `poiaddToolStripMenuItem_Click` | Add | **missing** |
| `poideleteToolStripMenuItem` | Click | `poideleteToolStripMenuItem_Click` | Delete | **missing** |
| `poieditToolStripMenuItem` | Click | `poieditToolStripMenuItem_Click` | Edit | **missing** |
| `trackerHomeToolStripMenuItem` | Click | `trackerHomeToolStripMenuItem_Click` | Tracker Home | **missing** |
| `modifyAltToolStripMenuItem` | Click | `modifyAltToolStripMenuItem_Click` | Modify Alt | done: `menu-modifyAlt` |
| `enterUTMCoordToolStripMenuItem` | Click | `enterUTMCoordToolStripMenuItem_Click` | Enter UTM Coord | **missing** |
| `switchDockingToolStripMenuItem` | Click | `switchDockingToolStripMenuItem_Click` | Switch Docking | **missing** |
| `setHomeHereToolStripMenuItem` | Click | `setHomeHereToolStripMenuItem_Click` | Set Home Here | **missing** |
| `addPolygonPointToolStripMenuItem` | Click | `addPolygonPointToolStripMenuItem_Click` | Draw a Polygon | done: `menu-addPolygonPoint2` |
| `clearPolygonToolStripMenuItem` | Click | `clearPolygonToolStripMenuItem_Click` | Clear Polygon | done: `menu-clearPolygon2` |
| `savePolygonToolStripMenuItem` | Click | `savePolygonToolStripMenuItem_Click` | Save Polygon | **missing** |
| `loadPolygonToolStripMenuItem` | Click | `loadPolygonToolStripMenuItem_Click` | Load Polygon | **missing** |
| `fromSHPToolStripMenuItem` | Click | `fromSHPToolStripMenuItem_Click` | From SHP | **missing** |
| `areaToolStripMenuItem` | Click | `areaToolStripMenuItem_Click` | Area | **missing** |
| `fenceInclusionToolStripMenuItem` | Click | `FenceInclusionToolStripMenuItem_Click` | Fence Inclusion | done: `draw-fence` |
| `fenceExclusionToolStripMenuItem` | Click | `FenceExclusionToolStripMenuItem_Click` | Fence Exclusion | **missing** |
| `timer1` | Tick | `timer1_Tick` | the map refresh timer | plumbing |
| `contextMenuStripPoly` | Opening | `ContextMenuStripPoly_Opening` | the polygon icon's menu | **missing** |
| `convertWPToPolygonToolStripMenuItem` | Click | `fromCurrentWaypointsMenuItem_Click` | From Current Waypoints | done: `menu-fromCurrentWaypoints` |
| `offsetPolygonToolStripMenuItem` | Click | `offsetPolygonToolStripMenuItem_Click` | Offset Polygon | **missing** |
| `zoomToVehicleToolStripMenuItem` | Click | `zoomToVehicleToolStripMenuItem_Click` | Zoom to Vehicle | **missing** |
| `zoomToMissionToolStripMenuItem` | Click | `zoomToMissionToolStripMenuItem_Click` | Zoom to Mission | **missing** |
| `zoomToHomeToolStripMenuItem` | Click | `zoomToHomeToolStripMenuItem_Click` | Zoom to Home | **missing** |
| `gDALOpacityToolStripMenuItem` | Click | `gDALOpacityToolStripMenuItem_Click` | GDAL Opacity | **missing** |
| `FlightPlanner` | FormClosing | `FlightPlanner_FormClosing` | the screen | plumbing |
| `FlightPlanner` | Load | `FlightPlanner_Load` | the screen | plumbing |
| `FlightPlanner` | Resize | `Planner_Resize` | the screen | plumbing |
