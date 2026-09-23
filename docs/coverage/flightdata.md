# FlightData action coverage

Generated from `crates/mp-gui/src/coverage.rs` by `cargo test -p mp-gui coverage -- --ignored update_report`; a test fails when this file is stale. One row per event wiring in `GCSViews/FlightData.Designer.cs`.

| total | done | elsewhere | missing | plumbing | dropped |
|---:|---:|---:|---:|---:|---:|
| 136 | 68 | 1 | 47 | 18 | 2 |

| control | event | handler | text | ours |
|---|---|---|---|---|
| `addPoiToolStripMenuItem` | Click | `addPoiToolStripMenuItem_Click` | Add Poi | done: `fly-poi-add` |
| `ALT_btn` | Click | `ALT_btn_Click` | ALT (transponder) | **missing** |
| `BUT_abortland` | Click | `BUT_abortland_Click` | Abort Landing | done: `fly-abortland` |
| `BUT_abort_script` | Click | `BUT_abort_script_Click` | Abort Running Script | **missing** |
| `BUTactiondo` | Click | `BUTactiondo_Click` | Do Action | done: `fly-doaction` |
| `BUT_ARM` | Click | `BUT_ARM_Click` | Arm/ Disarm | done: `arm` |
| `but_bintolog` | Click | `but_bintolog_Click` | Convert .Bin to .Log | done: `fly-bintolog` |
| `BUT_clear_track` | Click | `BUT_clear_track_Click` | Clear Track | **missing** |
| `but_dflogtokml` | Click | `but_dflogtokml_Click` | Create KML + gpx | done: `fly-dflogtokml` |
| `BUT_DFMavlink` | Click | `BUT_DFMavlink_Click` | Download DataFlash Log Via Mavlink | done: `fly-dfmavlink` |
| `but_disablejoystick` | Click | `but_disablejoystick_Click` | Disable Joystick | done: `joystick-enable` |
| `BUT_edit_selected` | Click | `BUT_edit_selected_Click` | Edit Selected Script | **missing** |
| `BUT_georefimage` | Click | `BUT_georefimage_Click` | Geo Reference Images | **missing** |
| `BUT_GimbalVideo` | Click | `gimbalVideoPopOutToolStripMenuItem_Click` | Video Control | **missing** |
| `BUT_Homealt` | Click | `BUT_Homealt_Click` | Set Home Alt | done: `fly-homealt` |
| `BUT_joystick` | Click | `BUT_joystick_Click` | Joystick | done: `joystick-refresh` |
| `BUT_loadtelem` | Click | `BUT_loadtelem_Click` | Load Log | done: `fly-loadtelem` |
| `BUT_log2kml` | Click | `BUT_log2kml_Click` | Tlog > Kml or Graph | elsewhere: mpr kml |
| `BUT_loganalysis` | Click | `BUT_loganalysis_Click` | Auto Analysis | done: `fly-loganalysis` |
| `BUT_logbrowse` | Click | `BUT_logbrowse_Click` | Review a Log | done: `fly-logbrowse` |
| `BUT_matlab` | Click | `BUT_matlab_Click` | Create Matlab File | done: `fly-matlab` |
| `BUT_mountmode` | Click | `BUT_mountmode_Click` | Set Mount | **missing** |
| `BUT_playlog` | Click | `BUT_playlog_Click` | Play/Pause | done: `fly-playlog` |
| `BUT_quickauto` | Click | `BUT_quickauto_Click` | Auto | done: `mode` |
| `BUT_quickmanual` | Click | `BUT_quickmanual_Click` | Loiter | done: `mode` |
| `BUT_quickrtl` | Click | `BUT_quickrtl_Click` | RTL | done: `mode` |
| `BUT_RAWSensor` | Click | `BUT_RAWSensor_Click` | Raw Sensor View | **missing** |
| `BUT_resetGimbalPos` | Click | `BUT_resetGimbalPos_Click` | Reset Position | **missing** |
| `BUTrestartmission` | Click | `BUTrestartmission_Click` | Restart Mission | done: `fly-restartmission` |
| `BUT_resumemis` | Click | `BUT_resumemis_Click` | Resume Mission | done: `fly-resumemis` |
| `BUT_run_script` | Click | `BUT_run_script_Click` | Run Script | **missing** |
| `BUT_select_script` | Click | `BUT_select_script_Click` | Select Script | **missing** |
| `BUT_SendMSG` | Click | `BUT_SendMSG_Click` | Message | **missing** |
| `BUT_setmode` | Click | `BUT_setmode_Click` | Set Mode | done: `mode` |
| `BUT_setwp` | Click | `BUT_setwp_Click` | Set WP | done: `fly-setwp` |
| `BUT_speed10` | Click | `BUT_speed1_Click` | 10x | done: `fly-speed10` |
| `BUT_speed1_10` | Click | `BUT_speed1_Click` | 0.1 | done: `fly-speed1_10` |
| `BUT_speed1_2` | Click | `BUT_speed1_Click` | 0.5 | done: `fly-speed1_2` |
| `BUT_speed1_4` | Click | `BUT_speed1_Click` | 0.25 | done: `fly-speed1_4` |
| `BUT_speed1` | Click | `BUT_speed1_Click` | 1x | done: `fly-speed1` |
| `BUT_speed2` | Click | `BUT_speed1_Click` | 2x | done: `fly-speed2` |
| `BUT_speed5` | Click | `BUT_speed1_Click` | 5x | done: `fly-speed5` |
| `CB_tuning` | CheckedChanged | `CB_tuning_CheckedChanged` | Tuning | done: `tuning-show` |
| `CHK_autopan` | CheckedChanged | `CHK_autopan_CheckedChanged` | Auto Pan | done: `map-follow` |
| `CMB_modes` | Click | `CMB_modes_Click` | the mode list | done: `mode` |
| `CMB_setwp` | Click | `CMB_setwp_Click` | the waypoint list | done: `fly-setwp-list` |
| `customizeToolStripMenuItem` | Click | `customizeToolStripMenuItem_Click` | Customize | **missing** |
| `deleteToolStripMenuItem` | Click | `deleteToolStripMenuItem_Click` | Delete (POI) | done: `fly-poi-delete` |
| `FlightID_tb` | TextChanged | `FlightID_tb_TextChanged` | FlightID | **missing** |
| `flightPlannerToolStripMenuItem` | Click | `flightPlannerToolStripMenuItem_Click` | Flight Planner | done: `tab-plan` |
| `flyToCoordsToolStripMenuItem` | Click | `flyToCoordsToolStripMenuItem_Click` | Fly To Coords | done: `fly-flytocoords` |
| `flyToHereAltToolStripMenuItem` | Click | `flyToHereAltToolStripMenuItem_Click` | Fly To Here Alt | done: `fly-flytohere-alt` |
| `gimbalVideoFullSizedToolStripMenuItem` | Click | `gimbalVideoFullSizedToolStripMenuItem_Click` | Full Sized | **missing** |
| `gimbalVideoMiniToolStripMenuItem` | Click | `gimbalVideoMiniToolStripMenuItem_Click` | Mini | **missing** |
| `gimbalVideoPopOutToolStripMenuItem` | Click | `gimbalVideoPopOutToolStripMenuItem_Click` | Pop Out | **missing** |
| `gMapControl1` | Click | `gMapControl1_Click` | the map | plumbing |
| `gMapControl1` | MouseDown | `gMapControl1_MouseDown` | the map: start a drag | done: `map` |
| `gMapControl1` | MouseLeave | `gMapControl1_MouseLeave` | the map | plumbing |
| `gMapControl1` | MouseMove | `gMapControl1_MouseMove` | the map: drag | done: `map` |
| `gMapControl1` | MouseUp | `gMapControl1_MouseUp` | the map: end a drag | done: `map` |
| `gMapControl1` | OnPositionChanged | `gMapControl1_OnPositionChanged` | the map | plumbing |
| `goHereToolStripMenuItem` | Click | `goHereToolStripMenuItem_Click` | Fly To Here | done: `fn fly_here` |
| `groundColorToolStripMenuItem` | Click | `groundColorToolStripMenuItem_Click` | Ground Color | done: `fly-hud-groundcolor` |
| `Gspeed` | DoubleClick | `Gspeed_DoubleClick` | the speed gauge | **missing** |
| `gStreamerStopToolStripMenuItem` | Click | `GStreamerStopToolStripMenuItem_Click` | GStreamer Stop | **missing** |
| `hereLinkVideoToolStripMenuItem` | Click | `HereLinkVideoToolStripMenuItem_Click` | HereLink Video | **missing** |
| `hud1` | DoubleClick | `hud1_DoubleClick` | the HUD: HUD Dropout, its own window | dropped: one window: the HUD has nowhere to drop out to |
| `hud1` | ekfclick | `hud1_ekfclick` | the HUD's EKF indicator | done: `hud-ekf` |
| `hud1` | Load | `hud1_Load` | the HUD | plumbing |
| `hud1` | prearmclick | `hud1_prearmclick` | the HUD's pre-arm indicator | done: `pre-arm` |
| `hud1` | Resize | `hud1_Resize` | the HUD | plumbing |
| `hud1` | vibeclick | `hud1_vibeclick` | the HUD's vibration indicator | done: `hud-vibe` |
| `IDENT_btn` | Click | `IDENT_btn_Click` | IDENT (transponder) | **missing** |
| `jumpToTagToolStripMenuItem` | Click | `jumpToTagToolStripMenuItem_Click` | Jump To Tag | done: `fly-jumptotag` |
| `loadFileToolStripMenuItem` | Click | `loadFileToolStripMenuItem_Click` | Load File | **missing** |
| `Messagetabtimer` | Tick | `Messagetabtimer_Tick` | the messages tab timer | plumbing |
| `modifyandSetAlt` | Click | `modifyandSetAlt_Click` | Change Alt | done: `fly-changealt` |
| `modifyandSetLoiterRad` | Click | `modifyandSetLoiterRad_Click` | Set Loiter Rad | done: `fly-setloiterrad` |
| `modifyandSetSpeed` | Click | `modifyandSetSpeed_Click` | Change Speed | done: `fly-changespeed` |
| `modifyandSetSpeed` | ParentChanged | `modifyandSetSpeed_ParentChanged` | Change Speed | plumbing |
| `multiLineToolStripMenuItem` | Click | `multiLineToolStripMenuItem_Click` | MultiLine | **missing** |
| `myButton1` | Click | `BUT_quickmanual_Click` | Loiter | done: `mode` |
| `myButton2` | Click | `BUT_quickrtl_Click` | RTL | done: `mode` |
| `myButton3` | Click | `BUT_quickauto_Click` | Auto | done: `mode` |
| `ON_btn` | Click | `ON_btn_Click` | ON (transponder) | **missing** |
| `onOffCameraOverlapToolStripMenuItem` | Click | `onOffCameraOverlapToolStripMenuItem_Click` | Camera Overlap | **missing** |
| `poiatcoordsToolStripMenuItem` | Click | `poiatcoordsToolStripMenuItem_Click` | Coords (POI) | done: `fly-poi-coords` |
| `PointCameraCoordsToolStripMenuItem1` | Click | `PointCameraCoordsToolStripMenuItem1_Click` | Point Camera Coords | **missing** |
| `pointCameraHereToolStripMenuItem` | Click | `pointCameraHereToolStripMenuItem_Click` | Point Camera Here | **missing** |
| `quickView1` | DoubleClick | `quickView_DoubleClick` | quick view 1: choose its field | done: `fly-quick-1` |
| `quickView2` | DoubleClick | `quickView_DoubleClick` | quick view 2: choose its field | done: `fly-quick-2` |
| `quickView3` | DoubleClick | `quickView_DoubleClick` | quick view 3: choose its field | done: `fly-quick-3` |
| `quickView4` | DoubleClick | `quickView_DoubleClick` | quick view 4: choose its field | done: `fly-quick-4` |
| `quickView5` | DoubleClick | `quickView_DoubleClick` | quick view 5: choose its field | done: `fly-quick-5` |
| `quickView6` | DoubleClick | `quickView_DoubleClick` | quick view 6: choose its field | done: `fly-quick-6` |
| `recordHudToAVIToolStripMenuItem` | Click | `recordHudToAVIToolStripMenuItem_Click` | Record Hud to AVI | **missing** |
| `russianHudToolStripMenuItem` | Click | `russianHudToolStripMenuItem_Click` | Russian Hud | done: `fly-hud-russian` |
| `saveFileToolStripMenuItem` | Click | `saveFileToolStripMenuItem_Click` | Save File | **missing** |
| `scriptChecker` | Tick | `scriptChecker_Tick` | the script status timer | plumbing |
| `setAspectRatioToolStripMenuItem` | Click | `setAspectRatioToolStripMenuItem_Click` | Set Aspect Ratio | **missing** |
| `setBatteryCellCountToolStripMenuItem` | Click | `setBatteryCellCountToolStripMenuItem_Click` | Battery Cell Voltage | **missing** |
| `setEKFHomeHereToolStripMenuItem` | Click | `setEKFHomeHereToolStripMenuItem_Click` | Set EKF Origin Here | **missing** |
| `setGStreamerSourceToolStripMenuItem` | Click | `setGStreamerSourceToolStripMenuItem_Click` | Set GStreamer Source | **missing** |
| `setHomeHereToolStripMenuItem1` | Click | `setHomeHereToolStripMenuItem_Click` | Set Home Here | **missing** |
| `setMJPEGSourceToolStripMenuItem` | Click | `setMJPEGSourceToolStripMenuItem_Click` | Set MJPEG source | **missing** |
| `setViewCountToolStripMenuItem` | Click | `setViewCountToolStripMenuItem_Click` | Set View Count | **missing** |
| `showIconsToolStripMenuItem` | Click | `showIconsToolStripMenuItem_Click` | Show icons | **missing** |
| `Squawk_nud` | MouseWheel | `Squawk_nud_MouseWheel` | Squawk (transponder) | **missing** |
| `Squawk_nud` | ValueChanged | `Squawk_nud_ValueChanged` | Squawk (transponder) | **missing** |
| `startCameraToolStripMenuItem` | Click | `startCameraToolStripMenuItem_Click` | Start Camera | **missing** |
| `STBY_btn` | Click | `STBY_btn_Click` | STBY (transponder) | **missing** |
| `stopRecordToolStripMenuItem` | Click | `stopRecordToolStripMenuItem_Click` | Stop Record | **missing** |
| `swapWithMapToolStripMenuItem` | Click | `swapWithMapToolStripMenuItem_Click` | Swap With Map | done: `fly-hud-swap` |
| `tabControlactions` | DrawItem | `tabControl1_DrawItem` | the actions tabs | plumbing |
| `tabControlactions` | SelectedIndexChanged | `tabControl1_SelectedIndexChanged` | the actions tabs | done: `fly-tabs` |
| `tabGauges` | Resize | `tabPage1_Resize` | Gauges | plumbing |
| `tabQuick` | Resize | `tabQuick_Resize` | Quick | plumbing |
| `tabStatus` | Paint | `tabStatus_Paint` | Status | plumbing |
| `takeOffToolStripMenuItem` | Click | `takeOffToolStripMenuItem_Click` | TakeOff | done: `takeoff` |
| `trackBarPitch` | Scroll | `gimbalTrackbar_Scroll` | Tilt (gimbal) | **missing** |
| `trackBarRoll` | Scroll | `gimbalTrackbar_Scroll` | Roll (gimbal) | **missing** |
| `trackBarYaw` | Scroll | `gimbalTrackbar_Scroll` | Pan (gimbal) | **missing** |
| `tracklog` | Scroll | `tracklog_Scroll` | the playback position | done: `fly-tracklog` |
| `triggerCameraToolStripMenuItem` | Click | `triggerCameraToolStripMenuItem_Click` | Trigger Camera NOW | **missing** |
| `TRK_zoom` | Scroll | `TRK_zoom_Scroll` | Zoom | done: `map` |
| `undockToolStripMenuItem` | Click | `undockDockToolStripMenuItem_Click` | Undock | dropped: one window: nothing to undock from |
| `userItemsToolStripMenuItem` | Click | `hud_UserItem` | User Items | done: `fly-hud-useritems` |
| `XPDRConnect_btn` | Click | `XPDRConnect_btn_Click` | Connect (transponder) | **missing** |
| `ZedGraphTimer` | Tick | `ZedGraphTimer_Tick` | the tuning graph timer | plumbing |
| `zg1` | DoubleClick | `zg1_DoubleClick` | the tuning graph: choose fields | done: `tuning-show` |
| `Zoomlevel` | ValueChanged | `Zoomlevel_ValueChanged` | Zoom | done: `map` |
| `FlightData` | Load | `FlightData_Load` | the screen | plumbing |
| `FlightData` | FormClosing | `FlightData_FormClosing` | the screen | plumbing |
| `FlightData` | ParentChanged | `FlightData_ParentChanged` | the screen | plumbing |
| `FlightData` | Resize | `FlightData_Resize` | the screen | plumbing |
| `splitContainer1` | Panel2.Resize | `splitContainer1_Panel2_Resize` | the map/HUD splitter | plumbing |
