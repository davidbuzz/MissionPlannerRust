# FlightPlanner action coverage

Generated from `crates/mp-gui/src/planner_coverage.rs` by `cargo test -p mp-gui planner_coverage -- --ignored update_report`; a test fails when this file is stale. One row per event wiring in `GCSViews/FlightPlanner.Designer.cs`.

| total | done | elsewhere | missing | plumbing | dropped |
|---:|---:|---:|---:|---:|---:|
| 121 | 107 | 0 | 2 | 12 | 0 |

Missing, by where the control sits:

| group | missing |
|---|---:|
| Map Tool | 2 |

| control | event | handler | text | ours |
|---|---|---|---|---|
| `TXT_WPRad` | KeyPress | `TXT_WPRad_KeyPress` | WP Radius | done: `plan-wprad` |
| `TXT_WPRad` | Leave | `TXT_WPRad_Leave` | WP Radius | done: `fn panel_leave` |
| `TXT_DefaultAlt` | KeyPress | `TXT_DefaultAlt_KeyPress` | Default Alt | done: `plan-defaultalt` |
| `TXT_DefaultAlt` | Leave | `TXT_DefaultAlt_Leave` | Default Alt | done: `fn panel_leave` |
| `TXT_loiterrad` | KeyPress | `TXT_loiterrad_KeyPress` | Loiter Radius | done: `plan-loiterrad` |
| `TXT_loiterrad` | Leave | `TXT_loiterrad_Leave` | Loiter Radius | done: `fn panel_leave` |
| `but_writewpfast` | Click | `but_writewpfast_Click` | Write Fast | done: `plan-writefast` |
| `BUT_write` | Click | `BUT_write_Click` | Write | done: `plan-write` |
| `BUT_read` | Click | `BUT_read_Click` | Read | done: `plan-read` |
| `label4` | LinkClicked | `label4_LinkClicked` | Home Location | done: `plan-home-link` |
| `TXT_homealt` | TextChanged | `TXT_homealt_TextChanged` | Home Location: ASL | done: `plan-home-alt` |
| `TXT_homelng` | TextChanged | `TXT_homelng_TextChanged` | Home Location: Long | done: `plan-home-lng` |
| `TXT_homelat` | TextChanged | `TXT_homelat_TextChanged` | Home Location: Lat | done: `plan-home-lat` |
| `TXT_homelat` | Enter | `TXT_homelat_Enter` | Home Location: Lat | done: `fn track_home_focus` |
| `coords1` | SystemChanged | `coords1_SystemChanged` | the pointer coordinates: system | done: `plan-coords-geo` |
| `chk_usemavftp` | CheckedChanged | `chk_usemavftp_CheckedChanged` | MAVFTP | done: `plan-mavftp` |
| `but_mincommands` | Click | `but_mincommands_Click` | ˅ | done: `plan-mincommands` |
| `CMB_altmode` | SelectedIndexChanged | `CMB_altmode_SelectedIndexChanged` | the altitude frame | done: `fn set_altitude_frame` |
| `CHK_splinedefault` | CheckedChanged | `CHK_splinedefault_CheckedChanged` | Spline | done: `plan-spline` |
| `Commands` | CellContentClick | `Commands_CellContentClick` | the waypoint grid: Delete, Up, Down | done: `fn row_controls` |
| `Commands` | CellEndEdit | `Commands_CellEndEdit` | the waypoint grid: a cell edited | done: `fn editor_panel` |
| `Commands` | DataError | `Commands_DataError` | the waypoint grid | plumbing |
| `Commands` | DefaultValuesNeeded | `Commands_DefaultValuesNeeded` | the waypoint grid: a new row's frame | done: `fn add_waypoint_in` |
| `Commands` | EditingControlShowing | `Commands_EditingControlShowing` | the waypoint grid | plumbing |
| `Commands` | RowEnter | `Commands_RowEnter` | the waypoint grid: the row's parameter names | done: `fn editor_panel` |
| `Commands` | RowsAdded | `Commands_RowsAdded` | the waypoint grid | plumbing |
| `Commands` | RowsRemoved | `Commands_RowsRemoved` | the waypoint grid | plumbing |
| `Commands` | RowValidating | `Commands_RowValidating` | the waypoint grid | plumbing |
| `BUT_Add` | Click | `BUT_Add_Click` | Add Below | done: `plan-add-below` |
| `BUT_InjectCustomMap` | Click | `BUT_InjectCustomMap_Click` | Inject Custom Map | done: `plan-inject` |
| `chk_grid` | CheckedChanged | `chk_grid_CheckedChanged` | Grid | done: `plan-grid` |
| `lnk_kml` | LinkClicked | `lnk_kml_LinkClicked` | View KML | done: `plan-kml` |
| `BUT_loadwpfile` | Click | `BUT_loadwpfile_Click` | Load File | done: `plan-load` |
| `BUT_saveWPFile` | Click | `BUT_saveWPFile_Click` | Save File | done: `plan-save` |
| `panelMap` | Resize | `panelMap_Resize` | the map panel | plumbing |
| `Zoomlevel` | ValueChanged | `Zoomlevel_ValueChanged` | Zoom | done: `plan-zoomlevel` |
| `TRK_zoom` | Scroll | `TRK_zoom_Scroll` | Zoom | done: `plan-trk-zoom` |
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
| `savePolygonToolStripMenuItem2` | Click | `savePolygonToolStripMenuItem_Click` | Save Polygon | done: `menu-savePolygon2` |
| `loadPolygonToolStripMenuItem2` | Click | `loadPolygonToolStripMenuItem_Click` | Load Polygon | done: `menu-loadPolygon2` |
| `fromSHPToolStripMenuItem2` | Click | `fromSHPToolStripMenuItem_Click` | From SHP | done: `menu-fromSHP2` |
| `fromCurrentWaypointsToolStripMenuItem` | Click | `fromCurrentWaypointsMenuItem_Click` | From Current Waypoints | done: `menu-fromCurrentWaypoints` |
| `offsetPolygonToolStripMenuItem2` | Click | `offsetPolygonToolStripMenuItem_Click` | Offset Polygon | done: `menu-offsetPolygon2` |
| `areaToolStripMenuItem2` | Click | `areaToolStripMenuItem_Click` | Area | done: `menu-area2` |
| `GeoFenceuploadToolStripMenuItem` | Click | `GeoFenceuploadToolStripMenuItem_Click` | Upload | done: `menu-GeoFenceupload` |
| `GeoFencedownloadToolStripMenuItem` | Click | `GeoFencedownloadToolStripMenuItem_Click` | Download | done: `menu-GeoFencedownload` |
| `setReturnLocationToolStripMenuItem` | Click | `setReturnLocationToolStripMenuItem_Click` | Set Return Location | done: `menu-setReturnLocation` |
| `loadFromFileToolStripMenuItem` | Click | `loadFromFileToolStripMenuItem_Click` | Load from File | done: `menu-loadFromFile` |
| `saveToFileToolStripMenuItem` | Click | `saveToFileToolStripMenuItem_Click` | Save to File | done: `menu-saveToFile` |
| `clearToolStripMenuItem` | Click | `clearToolStripMenuItem_Click` | Clear | done: `menu-clear` |
| `setRallyPointToolStripMenuItem` | Click | `setRallyPointToolStripMenuItem_Click` | Set Rally Point | done: `menu-setRallyPoint` |
| `getRallyPointsToolStripMenuItem` | Click | `getRallyPointsToolStripMenuItem_Click` | Download | done: `menu-getRallyPoints` |
| `saveRallyPointsToolStripMenuItem` | Click | `saveRallyPointsToolStripMenuItem_Click` | Upload | done: `menu-saveRallyPoints` |
| `clearRallyPointsToolStripMenuItem` | Click | `clearRallyPointsToolStripMenuItem_Click` | Clear Rally Points | done: `menu-clearRallyPoints` |
| `saveToFileToolStripMenuItem1` | Click | `saveToFileToolStripMenuItem1_Click` | Save Rally to File | done: `menu-saveToFile1` |
| `loadFromFileToolStripMenuItem1` | Click | `loadFromFileToolStripMenuItem1_Click` | Load Rally from File | done: `menu-loadFromFile1` |
| `createWpCircleToolStripMenuItem` | Click | `createWpCircleToolStripMenuItem_Click` | Create Wp Circle | done: `menu-createWpCircle` |
| `createSplineCircleToolStripMenuItem` | Click | `createSplineCircleToolStripMenuItem_Click` | Create Spline Circle | done: `menu-createSplineCircle` |
| `areaToolStripMenuItem1` | Click | `areaToolStripMenuItem_Click` | Area | done: `menu-area1` |
| `textToolStripMenuItem` | Click | `textToolStripMenuItem_Click` | Text | done: `menu-text` |
| `createCircleSurveyToolStripMenuItem` | Click | `createCircleSurveyToolStripMenuItem_Click` | Create Circle Survey | done: `menu-createCircleSurvey` |
| `surveyGridToolStripMenuItem` | Click | `surveyGridToolStripMenuItem_Click` | Survey (Grid) | done: `menu-surveyGrid` |
| `ContextMeasure` | Click | `ContextMeasure_Click` | Measure Distance | done: `menu-ContextMeasure` |
| `rotateMapToolStripMenuItem` | Click | `rotateMapToolStripMenuItem_Click` | Rotate Map | **missing** |
| `zoomToToolStripMenuItem` | Click | `zoomToToolStripMenuItem_Click` | Zoom To | done: `menu-zoomTo` |
| `prefetchToolStripMenuItem` | Click | `prefetchToolStripMenuItem_Click` | Prefetch | done: `menu-prefetch` |
| `prefetchWPPathToolStripMenuItem` | Click | `prefetchWPPathToolStripMenuItem_Click` | Prefetch WP Path | done: `menu-prefetchWPPath` |
| `kMLOverlayToolStripMenuItem` | Click | `kMLOverlayToolStripMenuItem_Click` | KML Overlay | done: `menu-kMLOverlay` |
| `elevationGraphToolStripMenuItem` | Click | `elevationGraphToolStripMenuItem_Click` | Elevation Graph | done: `menu-elevationGraph` |
| `reverseWPsToolStripMenuItem` | Click | `reverseWPsToolStripMenuItem_Click` | Reverse WPs | done: `menu-reverseWPs` |
| `loadWPFileToolStripMenuItem` | Click | `loadWPFileToolStripMenuItem_Click` | Load WP File | done: `menu-loadWPFile` |
| `loadAndAppendToolStripMenuItem` | Click | `loadAndAppendToolStripMenuItem_Click` | Load and Append | done: `menu-loadAndAppend` |
| `saveWPFileToolStripMenuItem` | Click | `saveWPFileToolStripMenuItem_Click` | Save WP File | done: `menu-saveWPFile` |
| `loadKMLFileToolStripMenuItem` | Click | `loadKMLFileToolStripMenuItem_Click` | Load KML File | done: `menu-loadKMLFile` |
| `loadSHPFileToolStripMenuItem` | Click | `loadSHPFileToolStripMenuItem_Click` | Load SHP File | done: `menu-loadSHPFile` |
| `poiaddToolStripMenuItem` | Click | `poiaddToolStripMenuItem_Click` | Add | done: `menu-poiadd` |
| `poideleteToolStripMenuItem` | Click | `poideleteToolStripMenuItem_Click` | Delete | done: `menu-poidelete` |
| `poieditToolStripMenuItem` | Click | `poieditToolStripMenuItem_Click` | Edit | done: `menu-poiedit` |
| `trackerHomeToolStripMenuItem` | Click | `trackerHomeToolStripMenuItem_Click` | Tracker Home | done: `menu-trackerHome` |
| `modifyAltToolStripMenuItem` | Click | `modifyAltToolStripMenuItem_Click` | Modify Alt | done: `menu-modifyAlt` |
| `enterUTMCoordToolStripMenuItem` | Click | `enterUTMCoordToolStripMenuItem_Click` | Enter UTM Coord | done: `menu-enterUTMCoord` |
| `switchDockingToolStripMenuItem` | Click | `switchDockingToolStripMenuItem_Click` | Switch Docking | done: `menu-switchDocking` |
| `setHomeHereToolStripMenuItem` | Click | `setHomeHereToolStripMenuItem_Click` | Set Home Here | done: `menu-setHomeHere` |
| `addPolygonPointToolStripMenuItem` | Click | `addPolygonPointToolStripMenuItem_Click` | Draw a Polygon | done: `menu-poly-addPolygonPoint` |
| `clearPolygonToolStripMenuItem` | Click | `clearPolygonToolStripMenuItem_Click` | Clear Polygon | done: `menu-poly-clearPolygon` |
| `savePolygonToolStripMenuItem` | Click | `savePolygonToolStripMenuItem_Click` | Save Polygon | done: `menu-poly-savePolygon` |
| `loadPolygonToolStripMenuItem` | Click | `loadPolygonToolStripMenuItem_Click` | Load Polygon | done: `menu-poly-loadPolygon` |
| `fromSHPToolStripMenuItem` | Click | `fromSHPToolStripMenuItem_Click` | From SHP | done: `menu-poly-fromSHP` |
| `areaToolStripMenuItem` | Click | `areaToolStripMenuItem_Click` | Area | done: `menu-poly-area` |
| `fenceInclusionToolStripMenuItem` | Click | `FenceInclusionToolStripMenuItem_Click` | Fence Inclusion | done: `menu-fenceInclusion` |
| `fenceExclusionToolStripMenuItem` | Click | `FenceExclusionToolStripMenuItem_Click` | Fence Exclusion | done: `menu-fenceExclusion` |
| `timer1` | Tick | `timer1_Tick` | the map refresh timer | plumbing |
| `contextMenuStripPoly` | Opening | `ContextMenuStripPoly_Opening` | the polygon icon's menu | done: `plan-polyicon` |
| `convertWPToPolygonToolStripMenuItem` | Click | `fromCurrentWaypointsMenuItem_Click` | From Current Waypoints | done: `menu-poly-convertWPToPolygon` |
| `offsetPolygonToolStripMenuItem` | Click | `offsetPolygonToolStripMenuItem_Click` | Offset Polygon | done: `menu-poly-offsetPolygon` |
| `zoomToVehicleToolStripMenuItem` | Click | `zoomToVehicleToolStripMenuItem_Click` | Zoom to Vehicle | done: `menu-zoomToVehicle` |
| `zoomToMissionToolStripMenuItem` | Click | `zoomToMissionToolStripMenuItem_Click` | Zoom to Mission | done: `menu-zoomToMission` |
| `zoomToHomeToolStripMenuItem` | Click | `zoomToHomeToolStripMenuItem_Click` | Zoom to Home | done: `menu-zoomToHome` |
| `gDALOpacityToolStripMenuItem` | Click | `gDALOpacityToolStripMenuItem_Click` | GDAL Opacity | **missing** |
| `FlightPlanner` | FormClosing | `FlightPlanner_FormClosing` | the screen | plumbing |
| `FlightPlanner` | Load | `FlightPlanner_Load` | the screen | plumbing |
| `FlightPlanner` | Resize | `Planner_Resize` | the screen | plumbing |
