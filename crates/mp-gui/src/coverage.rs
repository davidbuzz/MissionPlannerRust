//! What Mission Planner's flight screen can do, and what this one can (DELIVERABLES.md D10).
//!
//! `GCSViews/FlightData.Designer.cs` wires 136 events to handlers - every button, menu item,
//! double-click and timer the screen has. D10's definition of done is "every tab, button and
//! action of the C# `FlightData` present", and the first step toward a number like that is a
//! list nobody can argue with: each wiring, what the C# calls it, and what stands in for it here.
//! [`FLIGHTDATA`] is that list. The report it renders lives at `docs/coverage/flightdata.md` and
//! a test keeps it current, so the count of what is missing is in the repository rather than in
//! somebody's head.
//!
//! Three tests hold the list to the truth. When the C# tree is present the Designer file is
//! parsed and every wiring must have exactly one row, so an upstream change or a typo fails.
//! Every row that claims an implementation names an id or a function, and that name must appear
//! in this crate's source, so a row cannot claim what was removed. And the committed report must
//! match what the table renders.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

/// What stands in for a C# action here.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Ours {
    /// On the flight screen, under this control id (a `probe::measured` or `action` id) or
    /// `fn name` for a handler.
    Done(&'static str),
    /// Implemented, but somewhere other than the flight screen - a tab, the CLI, a link URL.
    Elsewhere(&'static str),
    /// Not implemented.
    Missing,
    /// WinForms mechanics with no user action behind them: timers, paint and resize handlers,
    /// mouse-move tracking. Listed so the count is honest, not owed.
    Plumbing,
    /// Deliberately not carried over, with the reason.
    Dropped(&'static str),
}

/// One wiring in `FlightData.Designer.cs`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Action {
    /// The control's name in the Designer.
    pub control: &'static str,
    /// The event.
    pub event: &'static str,
    /// The handler the event is wired to.
    pub handler: &'static str,
    /// The control's text in `FlightData.resx`, or a description where it has none.
    pub text: &'static str,
    /// What this application has for it.
    pub ours: Ours,
}

const fn row(
    control: &'static str,
    event: &'static str,
    handler: &'static str,
    text: &'static str,
    ours: Ours,
) -> Action {
    Action {
        control,
        event,
        handler,
        text,
        ours,
    }
}

use Ours::{Done, Dropped, Elsewhere, Missing, Plumbing};

/// Every event wiring in `GCSViews/FlightData.Designer.cs`, in the Designer's order.
pub const FLIGHTDATA: &[Action] = &[
    row(
        "addPoiToolStripMenuItem",
        "Click",
        "addPoiToolStripMenuItem_Click",
        "Add Poi",
        Done("fly-poi-add"),
    ),
    row(
        "ALT_btn",
        "Click",
        "ALT_btn_Click",
        "ALT (transponder)",
        Done("fly-xpdr-alt"),
    ),
    row(
        "BUT_abortland",
        "Click",
        "BUT_abortland_Click",
        "Abort Landing",
        Done("fly-abortland"),
    ),
    row(
        "BUT_abort_script",
        "Click",
        "BUT_abort_script_Click",
        "Abort Running Script",
        Missing,
    ),
    row(
        "BUTactiondo",
        "Click",
        "BUTactiondo_Click",
        "Do Action",
        Done("fly-doaction"),
    ),
    row(
        "BUT_ARM",
        "Click",
        "BUT_ARM_Click",
        "Arm/ Disarm",
        Done("arm"),
    ),
    row(
        "but_bintolog",
        "Click",
        "but_bintolog_Click",
        "Convert .Bin to .Log",
        Done("fly-bintolog"),
    ),
    row(
        "BUT_clear_track",
        "Click",
        "BUT_clear_track_Click",
        "Clear Track",
        Done("fly-cleartrack"),
    ),
    row(
        "but_dflogtokml",
        "Click",
        "but_dflogtokml_Click",
        "Create KML + gpx",
        Done("fly-dflogtokml"),
    ),
    row(
        "BUT_DFMavlink",
        "Click",
        "BUT_DFMavlink_Click",
        "Download DataFlash Log Via Mavlink",
        Done("fly-dfmavlink"),
    ),
    row(
        "but_disablejoystick",
        "Click",
        "but_disablejoystick_Click",
        "Disable Joystick",
        Done("joystick-enable"),
    ),
    row(
        "BUT_edit_selected",
        "Click",
        "BUT_edit_selected_Click",
        "Edit Selected Script",
        Missing,
    ),
    row(
        "BUT_georefimage",
        "Click",
        "BUT_georefimage_Click",
        "Geo Reference Images",
        Done("fly-georefimage"),
    ),
    row(
        "BUT_GimbalVideo",
        "Click",
        "gimbalVideoPopOutToolStripMenuItem_Click",
        "Video Control",
        Missing,
    ),
    row(
        "BUT_Homealt",
        "Click",
        "BUT_Homealt_Click",
        "Set Home Alt",
        Done("fly-homealt"),
    ),
    row(
        "BUT_joystick",
        "Click",
        "BUT_joystick_Click",
        "Joystick",
        Done("joystick-refresh"),
    ),
    row(
        "BUT_loadtelem",
        "Click",
        "BUT_loadtelem_Click",
        "Load Log",
        Done("fly-loadtelem"),
    ),
    row(
        "BUT_log2kml",
        "Click",
        "BUT_log2kml_Click",
        "Tlog > Kml or Graph",
        Elsewhere("mpr kml"),
    ),
    row(
        "BUT_loganalysis",
        "Click",
        "BUT_loganalysis_Click",
        "Auto Analysis",
        Done("fly-loganalysis"),
    ),
    row(
        "BUT_logbrowse",
        "Click",
        "BUT_logbrowse_Click",
        "Review a Log",
        Done("fly-logbrowse"),
    ),
    row(
        "BUT_matlab",
        "Click",
        "BUT_matlab_Click",
        "Create Matlab File",
        Done("fly-matlab"),
    ),
    row(
        "BUT_mountmode",
        "Click",
        "BUT_mountmode_Click",
        "Set Mount",
        Done("fly-mountmode"),
    ),
    row(
        "BUT_playlog",
        "Click",
        "BUT_playlog_Click",
        "Play/Pause",
        Done("fly-playlog"),
    ),
    row(
        "BUT_quickauto",
        "Click",
        "BUT_quickauto_Click",
        "Auto",
        Done("mode"),
    ),
    row(
        "BUT_quickmanual",
        "Click",
        "BUT_quickmanual_Click",
        "Loiter",
        Done("mode"),
    ),
    row(
        "BUT_quickrtl",
        "Click",
        "BUT_quickrtl_Click",
        "RTL",
        Done("mode"),
    ),
    row(
        "BUT_RAWSensor",
        "Click",
        "BUT_RAWSensor_Click",
        "Raw Sensor View",
        Missing,
    ),
    row(
        "BUT_resetGimbalPos",
        "Click",
        "BUT_resetGimbalPos_Click",
        "Reset Position",
        Done("fly-gimbal-reset"),
    ),
    row(
        "BUTrestartmission",
        "Click",
        "BUTrestartmission_Click",
        "Restart Mission",
        Done("fly-restartmission"),
    ),
    row(
        "BUT_resumemis",
        "Click",
        "BUT_resumemis_Click",
        "Resume Mission",
        Done("fly-resumemis"),
    ),
    row(
        "BUT_run_script",
        "Click",
        "BUT_run_script_Click",
        "Run Script",
        Missing,
    ),
    row(
        "BUT_select_script",
        "Click",
        "BUT_select_script_Click",
        "Select Script",
        Missing,
    ),
    row(
        "BUT_SendMSG",
        "Click",
        "BUT_SendMSG_Click",
        "Message",
        Done("fly-sendmsg"),
    ),
    row(
        "BUT_setmode",
        "Click",
        "BUT_setmode_Click",
        "Set Mode",
        Done("mode"),
    ),
    row(
        "BUT_setwp",
        "Click",
        "BUT_setwp_Click",
        "Set WP",
        Done("fly-setwp"),
    ),
    row(
        "BUT_speed10",
        "Click",
        "BUT_speed1_Click",
        "10x",
        Done("fly-speed10"),
    ),
    row(
        "BUT_speed1_10",
        "Click",
        "BUT_speed1_Click",
        "0.1",
        Done("fly-speed1_10"),
    ),
    row(
        "BUT_speed1_2",
        "Click",
        "BUT_speed1_Click",
        "0.5",
        Done("fly-speed1_2"),
    ),
    row(
        "BUT_speed1_4",
        "Click",
        "BUT_speed1_Click",
        "0.25",
        Done("fly-speed1_4"),
    ),
    row(
        "BUT_speed1",
        "Click",
        "BUT_speed1_Click",
        "1x",
        Done("fly-speed1"),
    ),
    row(
        "BUT_speed2",
        "Click",
        "BUT_speed1_Click",
        "2x",
        Done("fly-speed2"),
    ),
    row(
        "BUT_speed5",
        "Click",
        "BUT_speed1_Click",
        "5x",
        Done("fly-speed5"),
    ),
    row(
        "CB_tuning",
        "CheckedChanged",
        "CB_tuning_CheckedChanged",
        "Tuning",
        Done("tuning-show"),
    ),
    row(
        "CHK_autopan",
        "CheckedChanged",
        "CHK_autopan_CheckedChanged",
        "Auto Pan",
        Done("map-follow"),
    ),
    row(
        "CMB_modes",
        "Click",
        "CMB_modes_Click",
        "the mode list",
        Done("mode"),
    ),
    row(
        "CMB_setwp",
        "Click",
        "CMB_setwp_Click",
        "the waypoint list",
        Done("fly-setwp-list"),
    ),
    row(
        "customizeToolStripMenuItem",
        "Click",
        "customizeToolStripMenuItem_Click",
        "Customize",
        Done("fly-tabs-customize"),
    ),
    row(
        "deleteToolStripMenuItem",
        "Click",
        "deleteToolStripMenuItem_Click",
        "Delete (POI)",
        Done("fly-poi-delete"),
    ),
    row(
        "FlightID_tb",
        "TextChanged",
        "FlightID_tb_TextChanged",
        "FlightID",
        Done("fly-xpdr-flightid"),
    ),
    row(
        "flightPlannerToolStripMenuItem",
        "Click",
        "flightPlannerToolStripMenuItem_Click",
        "Flight Planner",
        Done("tab-plan"),
    ),
    row(
        "flyToCoordsToolStripMenuItem",
        "Click",
        "flyToCoordsToolStripMenuItem_Click",
        "Fly To Coords",
        Done("fly-flytocoords"),
    ),
    row(
        "flyToHereAltToolStripMenuItem",
        "Click",
        "flyToHereAltToolStripMenuItem_Click",
        "Fly To Here Alt",
        Done("fly-flytohere-alt"),
    ),
    row(
        "gimbalVideoFullSizedToolStripMenuItem",
        "Click",
        "gimbalVideoFullSizedToolStripMenuItem_Click",
        "Full Sized",
        Missing,
    ),
    row(
        "gimbalVideoMiniToolStripMenuItem",
        "Click",
        "gimbalVideoMiniToolStripMenuItem_Click",
        "Mini",
        Missing,
    ),
    row(
        "gimbalVideoPopOutToolStripMenuItem",
        "Click",
        "gimbalVideoPopOutToolStripMenuItem_Click",
        "Pop Out",
        Missing,
    ),
    row(
        "gMapControl1",
        "Click",
        "gMapControl1_Click",
        "the map",
        Plumbing,
    ),
    row(
        "gMapControl1",
        "MouseDown",
        "gMapControl1_MouseDown",
        "the map: start a drag",
        Done("map"),
    ),
    row(
        "gMapControl1",
        "MouseLeave",
        "gMapControl1_MouseLeave",
        "the map",
        Plumbing,
    ),
    row(
        "gMapControl1",
        "MouseMove",
        "gMapControl1_MouseMove",
        "the map: drag",
        Done("map"),
    ),
    row(
        "gMapControl1",
        "MouseUp",
        "gMapControl1_MouseUp",
        "the map: end a drag",
        Done("map"),
    ),
    row(
        "gMapControl1",
        "OnPositionChanged",
        "gMapControl1_OnPositionChanged",
        "the map",
        Plumbing,
    ),
    row(
        "goHereToolStripMenuItem",
        "Click",
        "goHereToolStripMenuItem_Click",
        "Fly To Here",
        Done("fn fly_here"),
    ),
    row(
        "groundColorToolStripMenuItem",
        "Click",
        "groundColorToolStripMenuItem_Click",
        "Ground Color",
        Done("fly-hud-groundcolor"),
    ),
    row(
        "Gspeed",
        "DoubleClick",
        "Gspeed_DoubleClick",
        "the speed gauge",
        Done("fly-gauge-speed"),
    ),
    row(
        "gStreamerStopToolStripMenuItem",
        "Click",
        "GStreamerStopToolStripMenuItem_Click",
        "GStreamer Stop",
        Missing,
    ),
    row(
        "hereLinkVideoToolStripMenuItem",
        "Click",
        "HereLinkVideoToolStripMenuItem_Click",
        "HereLink Video",
        Missing,
    ),
    row(
        "hud1",
        "DoubleClick",
        "hud1_DoubleClick",
        "the HUD: HUD Dropout, its own window",
        Dropped("one window: the HUD has nowhere to drop out to"),
    ),
    row(
        "hud1",
        "ekfclick",
        "hud1_ekfclick",
        "the HUD's EKF indicator",
        Done("hud-ekf"),
    ),
    row("hud1", "Load", "hud1_Load", "the HUD", Plumbing),
    row(
        "hud1",
        "prearmclick",
        "hud1_prearmclick",
        "the HUD's pre-arm indicator",
        Done("pre-arm"),
    ),
    row("hud1", "Resize", "hud1_Resize", "the HUD", Plumbing),
    row(
        "hud1",
        "vibeclick",
        "hud1_vibeclick",
        "the HUD's vibration indicator",
        Done("hud-vibe"),
    ),
    row(
        "IDENT_btn",
        "Click",
        "IDENT_btn_Click",
        "IDENT (transponder)",
        Done("fly-xpdr-ident"),
    ),
    row(
        "jumpToTagToolStripMenuItem",
        "Click",
        "jumpToTagToolStripMenuItem_Click",
        "Jump To Tag",
        Done("fly-jumptotag"),
    ),
    // The POI menu's Load File, `POI.POILoad`: the map menu's POI drop-down, not the tuning graph's.
    row(
        "loadFileToolStripMenuItem",
        "Click",
        "loadFileToolStripMenuItem_Click",
        "Load File",
        Done("fly-poi-load"),
    ),
    row(
        "Messagetabtimer",
        "Tick",
        "Messagetabtimer_Tick",
        "the messages tab timer",
        Plumbing,
    ),
    row(
        "modifyandSetAlt",
        "Click",
        "modifyandSetAlt_Click",
        "Change Alt",
        Done("fly-changealt"),
    ),
    row(
        "modifyandSetLoiterRad",
        "Click",
        "modifyandSetLoiterRad_Click",
        "Set Loiter Rad",
        Done("fly-setloiterrad"),
    ),
    row(
        "modifyandSetSpeed",
        "Click",
        "modifyandSetSpeed_Click",
        "Change Speed",
        Done("fly-changespeed"),
    ),
    row(
        "modifyandSetSpeed",
        "ParentChanged",
        "modifyandSetSpeed_ParentChanged",
        "Change Speed",
        Plumbing,
    ),
    row(
        "multiLineToolStripMenuItem",
        "Click",
        "multiLineToolStripMenuItem_Click",
        "MultiLine",
        Done("fly-tabs-multiline"),
    ),
    row(
        "myButton1",
        "Click",
        "BUT_quickmanual_Click",
        "Loiter",
        Done("mode"),
    ),
    row(
        "myButton2",
        "Click",
        "BUT_quickrtl_Click",
        "RTL",
        Done("mode"),
    ),
    row(
        "myButton3",
        "Click",
        "BUT_quickauto_Click",
        "Auto",
        Done("mode"),
    ),
    row(
        "ON_btn",
        "Click",
        "ON_btn_Click",
        "ON (transponder)",
        Done("fly-xpdr-on"),
    ),
    row(
        "onOffCameraOverlapToolStripMenuItem",
        "Click",
        "onOffCameraOverlapToolStripMenuItem_Click",
        "Camera Overlap",
        Missing,
    ),
    row(
        "poiatcoordsToolStripMenuItem",
        "Click",
        "poiatcoordsToolStripMenuItem_Click",
        "Coords (POI)",
        Done("fly-poi-coords"),
    ),
    row(
        "PointCameraCoordsToolStripMenuItem1",
        "Click",
        "PointCameraCoordsToolStripMenuItem1_Click",
        "Point Camera Coords",
        Done("fly-pointcameracoords"),
    ),
    row(
        "pointCameraHereToolStripMenuItem",
        "Click",
        "pointCameraHereToolStripMenuItem_Click",
        "Point Camera Here",
        Done("fly-pointcamerahere"),
    ),
    row(
        "quickView1",
        "DoubleClick",
        "quickView_DoubleClick",
        "quick view 1: choose its field",
        Done("fly-quick-1"),
    ),
    row(
        "quickView2",
        "DoubleClick",
        "quickView_DoubleClick",
        "quick view 2: choose its field",
        Done("fly-quick-2"),
    ),
    row(
        "quickView3",
        "DoubleClick",
        "quickView_DoubleClick",
        "quick view 3: choose its field",
        Done("fly-quick-3"),
    ),
    row(
        "quickView4",
        "DoubleClick",
        "quickView_DoubleClick",
        "quick view 4: choose its field",
        Done("fly-quick-4"),
    ),
    row(
        "quickView5",
        "DoubleClick",
        "quickView_DoubleClick",
        "quick view 5: choose its field",
        Done("fly-quick-5"),
    ),
    row(
        "quickView6",
        "DoubleClick",
        "quickView_DoubleClick",
        "quick view 6: choose its field",
        Done("fly-quick-6"),
    ),
    row(
        "recordHudToAVIToolStripMenuItem",
        "Click",
        "recordHudToAVIToolStripMenuItem_Click",
        "Record Hud to AVI",
        Missing,
    ),
    row(
        "russianHudToolStripMenuItem",
        "Click",
        "russianHudToolStripMenuItem_Click",
        "Russian Hud",
        Done("fly-hud-russian"),
    ),
    // The POI menu's Save File, `POI.POISave`.
    row(
        "saveFileToolStripMenuItem",
        "Click",
        "saveFileToolStripMenuItem_Click",
        "Save File",
        Done("fly-poi-save"),
    ),
    row(
        "scriptChecker",
        "Tick",
        "scriptChecker_Tick",
        "the script status timer",
        Plumbing,
    ),
    row(
        "setAspectRatioToolStripMenuItem",
        "Click",
        "setAspectRatioToolStripMenuItem_Click",
        "Set Aspect Ratio",
        Missing,
    ),
    row(
        "setBatteryCellCountToolStripMenuItem",
        "Click",
        "setBatteryCellCountToolStripMenuItem_Click",
        "Battery Cell Voltage",
        Done("fly-hud-batterycells"),
    ),
    row(
        "setEKFHomeHereToolStripMenuItem",
        "Click",
        "setEKFHomeHereToolStripMenuItem_Click",
        "Set EKF Origin Here",
        Done("fly-setekforigin"),
    ),
    row(
        "setGStreamerSourceToolStripMenuItem",
        "Click",
        "setGStreamerSourceToolStripMenuItem_Click",
        "Set GStreamer Source",
        Missing,
    ),
    row(
        "setHomeHereToolStripMenuItem1",
        "Click",
        "setHomeHereToolStripMenuItem_Click",
        "Set Home Here",
        Done("fly-sethome"),
    ),
    row(
        "setMJPEGSourceToolStripMenuItem",
        "Click",
        "setMJPEGSourceToolStripMenuItem_Click",
        "Set MJPEG source",
        Missing,
    ),
    row(
        "setViewCountToolStripMenuItem",
        "Click",
        "setViewCountToolStripMenuItem_Click",
        "Set View Count",
        Missing,
    ),
    row(
        "showIconsToolStripMenuItem",
        "Click",
        "showIconsToolStripMenuItem_Click",
        "Show icons",
        Missing,
    ),
    row(
        "Squawk_nud",
        "MouseWheel",
        "Squawk_nud_MouseWheel",
        "Squawk (transponder)",
        Done("fly-xpdr-squawk-box"),
    ),
    row(
        "Squawk_nud",
        "ValueChanged",
        "Squawk_nud_ValueChanged",
        "Squawk (transponder)",
        Done("fly-xpdr-squawk"),
    ),
    row(
        "startCameraToolStripMenuItem",
        "Click",
        "startCameraToolStripMenuItem_Click",
        "Start Camera",
        Missing,
    ),
    row(
        "STBY_btn",
        "Click",
        "STBY_btn_Click",
        "STBY (transponder)",
        Done("fly-xpdr-stby"),
    ),
    row(
        "stopRecordToolStripMenuItem",
        "Click",
        "stopRecordToolStripMenuItem_Click",
        "Stop Record",
        Missing,
    ),
    row(
        "swapWithMapToolStripMenuItem",
        "Click",
        "swapWithMapToolStripMenuItem_Click",
        "Swap With Map",
        Done("fly-hud-swap"),
    ),
    row(
        "tabControlactions",
        "DrawItem",
        "tabControl1_DrawItem",
        "the actions tabs",
        Plumbing,
    ),
    // Choosing a page of the strip under the HUD. The handler's body only refreshes the Status
    // and Messages pages the C# fills on a timer; the page change is the strip's own.
    row(
        "tabControlactions",
        "SelectedIndexChanged",
        "tabControl1_SelectedIndexChanged",
        "the actions tabs",
        Done("fly-tabs"),
    ),
    row("tabGauges", "Resize", "tabPage1_Resize", "Gauges", Plumbing),
    row("tabQuick", "Resize", "tabQuick_Resize", "Quick", Plumbing),
    row("tabStatus", "Paint", "tabStatus_Paint", "Status", Plumbing),
    row(
        "takeOffToolStripMenuItem",
        "Click",
        "takeOffToolStripMenuItem_Click",
        "TakeOff",
        Done("takeoff"),
    ),
    row(
        "trackBarPitch",
        "Scroll",
        "gimbalTrackbar_Scroll",
        "Tilt (gimbal)",
        Done("fly-gimbal-pitch"),
    ),
    row(
        "trackBarRoll",
        "Scroll",
        "gimbalTrackbar_Scroll",
        "Roll (gimbal)",
        Done("fly-gimbal-roll"),
    ),
    row(
        "trackBarYaw",
        "Scroll",
        "gimbalTrackbar_Scroll",
        "Pan (gimbal)",
        Done("fly-gimbal-yaw"),
    ),
    row(
        "tracklog",
        "Scroll",
        "tracklog_Scroll",
        "the playback position",
        Done("fly-tracklog"),
    ),
    row(
        "triggerCameraToolStripMenuItem",
        "Click",
        "triggerCameraToolStripMenuItem_Click",
        "Trigger Camera NOW",
        Done("fly-triggercamera"),
    ),
    row("TRK_zoom", "Scroll", "TRK_zoom_Scroll", "Zoom", Done("map")),
    row(
        "undockToolStripMenuItem",
        "Click",
        "undockDockToolStripMenuItem_Click",
        "Undock",
        Dropped("one window: nothing to undock from"),
    ),
    row(
        "userItemsToolStripMenuItem",
        "Click",
        "hud_UserItem",
        "User Items",
        Done("fly-hud-useritems"),
    ),
    row(
        "XPDRConnect_btn",
        "Click",
        "XPDRConnect_btn_Click",
        "Connect (transponder)",
        Done("fly-xpdr-connect"),
    ),
    row(
        "ZedGraphTimer",
        "Tick",
        "ZedGraphTimer_Tick",
        "the tuning graph timer",
        Plumbing,
    ),
    row(
        "zg1",
        "DoubleClick",
        "zg1_DoubleClick",
        "the tuning graph: choose fields",
        Done("tuning-show"),
    ),
    row(
        "Zoomlevel",
        "ValueChanged",
        "Zoomlevel_ValueChanged",
        "Zoom",
        Done("map"),
    ),
    // The form's own events, wired as `this.Load` and so on, and one nested panel.
    row(
        "FlightData",
        "Load",
        "FlightData_Load",
        "the screen",
        Plumbing,
    ),
    row(
        "FlightData",
        "FormClosing",
        "FlightData_FormClosing",
        "the screen",
        Plumbing,
    ),
    row(
        "FlightData",
        "ParentChanged",
        "FlightData_ParentChanged",
        "the screen",
        Plumbing,
    ),
    row(
        "FlightData",
        "Resize",
        "FlightData_Resize",
        "the screen",
        Plumbing,
    ),
    row(
        "splitContainer1",
        "Panel2.Resize",
        "splitContainer1_Panel2_Resize",
        "the map/HUD splitter",
        Plumbing,
    ),
];

/// Why each row still missing is missing, rendered beside it in the report: left for later, a
/// window of its own, or the owner's call.
#[cfg(test)]
pub const WHY_MISSING: &[(&str, &str, &str)] = &[
    ("BUT_abort_script", "Click", SCRIPTS),
    ("BUT_edit_selected", "Click", SCRIPTS),
    ("BUT_run_script", "Click", SCRIPTS),
    ("BUT_select_script", "Click", SCRIPTS),
    (
        "BUT_RAWSensor",
        "Click",
        "later: RAW_Sensor is a window of its own; drawn dimmed in the Actions grid",
    ),
    ("BUT_GimbalVideo", "Click", VIDEO),
    ("gimbalVideoFullSizedToolStripMenuItem", "Click", VIDEO),
    ("gimbalVideoMiniToolStripMenuItem", "Click", VIDEO),
    ("gimbalVideoPopOutToolStripMenuItem", "Click", VIDEO),
    ("gStreamerStopToolStripMenuItem", "Click", VIDEO),
    ("hereLinkVideoToolStripMenuItem", "Click", VIDEO),
    ("recordHudToAVIToolStripMenuItem", "Click", VIDEO),
    ("setGStreamerSourceToolStripMenuItem", "Click", VIDEO),
    ("setMJPEGSourceToolStripMenuItem", "Click", VIDEO),
    ("startCameraToolStripMenuItem", "Click", VIDEO),
    ("stopRecordToolStripMenuItem", "Click", VIDEO),
    (
        "onOffCameraOverlapToolStripMenuItem",
        "Click",
        "drawn dimmed: it acts on the CAMERA_FEEDBACK photo markers, which the map does not draw",
    ),
    (
        "setAspectRatioToolStripMenuItem",
        "Click",
        "the owner's call: the C#'s 4:3 would reshape the column",
    ),
    (
        "showIconsToolStripMenuItem",
        "Click",
        "the owner's call: the HUD's icons are not ported",
    ),
    (
        "setViewCountToolStripMenuItem",
        "Click",
        "the quick views' menu and both questions are there (`fly-quick-setviewcount`) and the \
         answer is kept; the grid is `quick.rs`'s six views, which have no resize yet",
    ),
];

/// Left for later: scripts.
#[cfg(test)]
const SCRIPTS: &str = "later: scripts - `mp-script` has no interpreter yet";

/// Left for later: video.
#[cfg(test)]
const VIDEO: &str = "later: video (GStreamer, HereLink, MJPEG, the camera, AVI)";

/// How many rows are in each state: (done, elsewhere, missing, plumbing, dropped).
#[must_use]
pub fn counts() -> (usize, usize, usize, usize, usize) {
    let mut counts = (0, 0, 0, 0, 0);
    for action in FLIGHTDATA {
        match action.ours {
            Done(_) => counts.0 += 1,
            Elsewhere(_) => counts.1 += 1,
            Missing => counts.2 += 1,
            Plumbing => counts.3 += 1,
            Dropped(_) => counts.4 += 1,
        }
    }
    counts
}

/// The report, as Markdown: the counts, then every row.
#[cfg(test)]
#[must_use]
pub fn report() -> String {
    let (done, elsewhere, missing, plumbing, dropped) = counts();
    let mut out = String::new();
    out.push_str("# FlightData action coverage\n\n");
    out.push_str(
        "Generated from `crates/mp-gui/src/coverage.rs` by `cargo test -p mp-gui coverage -- \
         --ignored update_report`; a test fails when this file is stale. One row per event \
         wiring in `GCSViews/FlightData.Designer.cs`.\n\n",
    );
    out.push_str(&format!(
        "| total | done | elsewhere | missing | plumbing | dropped |\n|---:|---:|---:|---:|---:|---:|\n| {} | {done} | {elsewhere} | {missing} | {plumbing} | {dropped} |\n\n",
        FLIGHTDATA.len()
    ));
    out.push_str("| control | event | handler | text | ours |\n|---|---|---|---|---|\n");
    for action in FLIGHTDATA {
        let ours = match action.ours {
            Done(id) => format!("done: `{id}`"),
            Elsewhere(what) => format!("elsewhere: {what}"),
            Missing => WHY_MISSING
                .iter()
                .find(|(control, event, _)| *control == action.control && *event == action.event)
                .map_or_else(
                    || "**missing**".to_owned(),
                    |(_, _, why)| format!("**missing** - {why}"),
                ),
            Plumbing => "plumbing".to_owned(),
            Dropped(reason) => format!("dropped: {reason}"),
        };
        out.push_str(&format!(
            "| `{}` | {} | `{}` | {} | {} |\n",
            action.control, action.event, action.handler, action.text, ours
        ));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Where the committed report lives, relative to this crate.
    const REPORT: &str = "../../docs/coverage/flightdata.md";

    /// This crate's source, for checking that a claimed id or function exists.
    const SOURCES: &[&str] = &[
        include_str!("fly.rs"),
        include_str!("main.rs"),
        include_str!("telemetry.rs"),
        include_str!("tuning.rs"),
        include_str!("joystick.rs"),
        include_str!("logbrowse.rs"),
        include_str!("plan.rs"),
        include_str!("mapview.rs"),
        include_str!("setup.rs"),
        include_str!("params.rs"),
        include_str!("quick.rs"),
        include_str!("poi.rs"),
        include_str!("logdownload.rs"),
        include_str!("transponder.rs"),
        include_str!("payload.rs"),
        include_str!("gauge.rs"),
    ];

    fn designer() -> Option<String> {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../referneces/missionplanner/GCSViews/FlightData.Designer.cs");
        std::fs::read_to_string(path).ok()
    }

    /// `this.X.Y += new Z(this.H);` → (X, Y, H). `this.Y += ...` is the form itself, named
    /// `FlightData` here; a nested `this.X.Panel2.Y` keeps `Panel2.Y` as its event.
    fn wirings(designer: &str) -> Vec<(String, String, String)> {
        designer
            .lines()
            .filter_map(|line| {
                let line = line.trim();
                let (left, right) = line.split_once(" += new ")?;
                let left = left.strip_prefix("this.")?;
                let (control, event) = left.split_once('.').unwrap_or(("FlightData", left));
                let handler = right.rsplit_once("(this.")?.1.strip_suffix(");")?;
                Some((control.to_owned(), event.to_owned(), handler.to_owned()))
            })
            .collect()
    }

    /// Every wiring in the Designer has one row, with the same handler, and nothing else does.
    #[test]
    fn every_designer_wiring_has_exactly_one_row() {
        let Some(designer) = designer() else {
            eprintln!("skipped: the C# tree is not checked out here");
            return;
        };
        let mut wired = wirings(&designer);
        wired.sort();
        wired.dedup();
        assert_eq!(wired.len(), 136, "the Designer wires 136 events");
        let mut ours: Vec<(String, String, String)> = FLIGHTDATA
            .iter()
            .map(|a| {
                (
                    a.control.to_owned(),
                    a.event.to_owned(),
                    a.handler.to_owned(),
                )
            })
            .collect();
        ours.sort();
        for (control, event, handler) in &wired {
            assert!(
                ours.iter()
                    .any(|(c, e, h)| c == control && e == event && h == handler),
                "no row for this.{control}.{event} -> {handler}"
            );
        }
        for (control, event, handler) in &ours {
            assert!(
                wired
                    .iter()
                    .any(|(c, e, h)| c == control && e == event && h == handler),
                "row {control}.{event} -> {handler} is not in the Designer"
            );
        }
        assert_eq!(ours.len(), wired.len());
    }

    /// Nothing is claimed that the source does not have.
    #[test]
    fn every_claimed_id_exists_in_this_crate() {
        for action in FLIGHTDATA {
            let Done(id) = action.ours else { continue };
            let found = SOURCES.iter().any(|source| {
                id.strip_prefix("fn ").map_or_else(
                    || source.contains(&format!("\"{id}\"")),
                    |name| source.contains(&format!("fn {name}(")),
                )
            });
            assert!(
                found,
                "{}.{} claims `{id}`, which is not in the source",
                action.control, action.event
            );
        }
    }

    /// The committed report matches the table.
    #[test]
    fn the_committed_report_is_current() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(REPORT);
        let committed = std::fs::read_to_string(&path).unwrap_or_default();
        assert!(
            committed == report(),
            "docs/coverage/flightdata.md is stale; run `cargo test -p mp-gui coverage -- --ignored update_report`"
        );
    }

    /// Rewrites the report. Run on purpose, not on every test.
    #[test]
    #[ignore = "writes docs/coverage/flightdata.md"]
    fn update_report() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(REPORT);
        std::fs::write(&path, report()).expect("write the report");
    }

    /// Every reason given for a missing row is for a row that is there and missing, once.
    #[test]
    fn each_reason_is_for_a_missing_row() {
        for (index, (control, event, _)) in WHY_MISSING.iter().enumerate() {
            let row = FLIGHTDATA
                .iter()
                .find(|action| action.control == *control && action.event == *event);
            assert!(
                matches!(row, Some(Action { ours: Missing, .. })),
                "{control}.{event} is not a missing row"
            );
            assert!(
                !WHY_MISSING[..index]
                    .iter()
                    .any(|(c, e, _)| c == control && e == event),
                "{control}.{event} twice"
            );
        }
    }

    /// The count is what the plan says, so a change in either direction is a deliberate edit.
    #[test]
    fn the_counts_are_the_ones_the_plan_records() {
        let (done, elsewhere, missing, plumbing, dropped) = counts();
        assert_eq!(
            done + elsewhere + missing + plumbing + dropped,
            FLIGHTDATA.len()
        );
        eprintln!(
            "FlightData: {done} done, {elsewhere} elsewhere, {missing} missing, {plumbing} plumbing, {dropped} dropped"
        );
        assert_eq!(
            (done, elsewhere, missing, plumbing, dropped),
            (95, 1, 20, 18, 2)
        );
    }
}
