# LogBrowse action coverage

Generated from `crates/mp-gui/src/logbrowse/coverage.rs` by `cargo test -p mp-gui logbrowse::coverage -- --ignored update_report`; a test fails when this file is stale. One row per event wiring in `Log/LogBrowse.designer.cs`, then what the window does beyond them: `LogBrowse.cs`'s own handlers and the ZedGraph chart's default behaviour.

| table | total | done | elsewhere | missing | plumbing | dropped |
|---|---:|---:|---:|---:|---:|---:|
| designer | 37 | 29 | 0 | 0 | 6 | 2 |
| beyond the designer | 15 | 10 | 0 | 0 | 3 | 2 |

## `Log/LogBrowse.designer.cs`

| control | event | handler | text | ours |
|---|---|---|---|---|
| `exportVisibleToolStripMenuItem` | Click | `exportVisibleToolStripMenuItem_Click` | Export Visible (the grid's menu) | done: `loggrid-menu-visible` |
| `exportFilesToolStripMenuItem` | Click | `exportFilesToolStripMenuItem_Click` | Export Files (the grid's menu) | done: `loggrid-menu-files` |
| `BUT_Graphit` | Click | `Graphit_Click` | Graph Left | done: `log-graph-left` |
| `BUT_cleargraph` | Click | `BUT_cleargraph_Click` | Clear Graph | done: `log-clear` |
| `BUT_loadlog` | Click | `BUT_loadlog_Click` | Load A Log | done: `log-open` |
| `splitContainerZgGrid` | Resize | `splitContainer1_Resize` | the chart and grid's splitter, repainted | plumbing |
| `splitContainerZgMap` | Resize | `splitContainer2_Resize` | the chart and map's splitter, repainted | plumbing |
| `zg1` | ZoomEvent | `zg1_ZoomEvent` | a zoom or pan: the labels and the map redrawn | done: `fn zoom_event` |
| `zg1` | MouseMoveEvent | `zg1_MouseMoveEvent` | the pointer over the chart: a debounce | plumbing |
| `zg1` | MouseDoubleClick | `zg1_MouseDoubleClick` | the chart double-clicked: the cursor | done: `fn double_click` |
| `myGMAP1` | OnRouteClick | `myGMAP1_OnRouteClick` | a route clicked | dropped: dead in the C#: `DrawMap` makes every route `IsHitTestVisible = false`, so GMap never raises it |
| `myGMAP1` | MouseDown | `myGMAP1_MouseDown` | the map dragged | done: `log-map` |
| `myGMAP1` | MouseMove | `myGMAP1_MouseMove` | the map dragged | done: `log-map` |
| `myGMAP1` | MouseUp | `myGMAP1_MouseUp` | the map dragged | done: `log-map` |
| `chk_params` | CheckedChanged | `chk_params_CheckedChanged` | Show Params | done: `log-chk-params` |
| `chk_events` | CheckedChanged | `chk_events_CheckedChanged` | Events | done: `log-chk-events` |
| `chk_datagrid` | CheckedChanged | `chk_datagrid_CheckedChanged` | Data Table | done: `log-chk-datagrid` |
| `chk_msg` | CheckedChanged | `chk_msg_CheckedChanged` | MSG | done: `log-chk-msg` |
| `chk_errors` | CheckedChanged | `chk_errors_CheckedChanged` | Errors | done: `log-chk-errors` |
| `chk_mode` | CheckedChanged | `chk_mode_CheckedChanged` | Mode | done: `log-chk-mode` |
| `BUT_Graphit_R` | Click | `BUT_Graphit_R_Click` | Graph Right | done: `log-graph-right` |
| `chk_time` | CheckedChanged | `chk_time_CheckedChanged` | Time | done: `log-chk-time` |
| `CHK_map` | CheckedChanged | `CHK_map_CheckedChanged` | Map | done: `log-chk-map` |
| `CMB_preselect` | SelectedIndexChanged | `CMB_preselect_SelectedIndexChanged` | the preselected graphs | done: `log-preselect` |
| `BUT_removeitem` | Click | `BUT_removeitem_Click` | Remove Item | dropped: hidden in the C#: `BUT_removeitem.Visible = False` in the resx |
| `dataGridView1` | CellDoubleClick | `dataGridView1_CellDoubleClick` | a row double-clicked: the map and the chart go to it | done: `fn grid_double_click` |
| `dataGridView1` | CellValueNeeded | `dataGridView1_CellValueNeeded` | a cell's text | done: `fn build_row` |
| `dataGridView1` | ColumnHeaderMouseClick | `dataGridView1_ColumnHeaderMouseClick` | a header clicked: the grid filtered to a type | done: `fn toggle_grid_chooser` |
| `dataGridView1` | RowEnter | `dataGridView1_RowEnter` | the headers named for the current row | done: `fn headers` |
| `treeView1` | AfterCheck | `treeView1_AfterCheck` | a field ticked: graphed, or unticked: removed | done: `fn graph` |
| `treeView1` | DrawNode | `treeView1_DrawNode` | the tree's text, owner-drawn | plumbing |
| `treeView1` | NodeMouseHover | `treeView1_TreeNodeMouseHover` | a field's description in `txt_info` | done: `fn hover_field` |
| `treeView1` | DoubleClick | `treeView1_DoubleClick` | a field's scaler and offset | done: `fn ask_modifier` |
| `treeView1` | MouseDown | `treeView1_MouseDown` | a right click: the right axis | done: `fn graph` |
| `LogBrowse` | FormClosed | `LogBrowse_FormClosed` | the log let go | plumbing |
| `LogBrowse` | Load | `LogBrowse_Load` | a log opened | done: `fn open` |
| `LogBrowse` | Resize | `LogBrowse_Resize` | the splitters placed | plumbing |

## Beyond the designer

| control | event | handler | text | ours |
|---|---|---|---|---|
| `LogBrowse` | ProcessCmdKey | `Ctrl+G` | Line no: the grid to a line | done: `fn go_to_line` |
| `chk_datagrid, chk_time, CHK_map, chk_errors, chk_mode, chk_msg` | CheckedChanged | `(lambdas in LoadLog2)` | the boxes remembered as LB_Grid, LB_Time, LB_Map, LB_Error, LB_Mode, LB_MSG | done: `fn apply_remembered` |
| `dataGridView1` | RowUnshared | `dataGridView1_RowUnshared` | an empty handler | plumbing |
| `zg1` | ContextMenuBuilder | `Zg1_ContextMenuBuilder` | adds nothing: its three items are commented out | plumbing |
| `zg1` | MouseDown/MouseUp | `(ZedGraph) HandleZoomFinish` | a left drag: every axis zoomed to the rectangle | done: `fn chart_drag_zoom` |
| `zg1` | MouseWheel | `(ZedGraph) ZedGraphControl_MouseWheel` | the wheel: every axis zoomed a tenth | done: `fn chart_wheel` |
| `zg1` | MouseDown/MouseMove | `(ZedGraph) HandlePanDrag` | Ctrl and a left drag, or a middle drag: every axis panned | done: `fn chart_pan_to` |
| `zg1` | MouseMove | `(ZedGraph) HandlePointValues` | the point under the pointer, when Show Point Values is on | done: `fn point_tooltip` |
| `zg1 menu` | Click | `(ZedGraph) MenuClick_Copy` | Copy | dropped: an image of the chart to the clipboard: gpui renders no image of a view to copy |
| `zg1 menu` | Click | `(ZedGraph) MenuClick_SaveAs` | Save Image As... | dropped: an image of the chart to a file: gpui renders no image of a view to save |
| `zg1 menu` | Click | `(ZedGraph) MenuClick_ShowValues` | Show Point Values | done: `log-chart-menu-show_val` |
| `zg1 menu` | Click | `(ZedGraph) MenuClick_ZoomOut` | Un-Zoom / Un-Pan | done: `log-chart-menu-unzoom` |
| `zg1 menu` | Click | `(ZedGraph) MenuClick_ZoomOutAll` | Undo All Zoom/Pan | done: `log-chart-menu-undo_all` |
| `zg1 menu` | Click | `(ZedGraph) MenuClick_RestoreScale` | Set Scale to Default | done: `log-chart-menu-set_default` |
| `zg1` | Scroll | `(ZedGraph) hScrollBar1` | the scroll bars: hidden, `IsShowHScrollBar` is false | plumbing |
