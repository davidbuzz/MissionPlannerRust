//! Tests of what this port adds to the log browser: Show Params, the preselected graphs, the
//! routes `DrawMap` draws, ZedGraph's zoom, pan, menu and point values, the grid's double click
//! and menu, Ctrl+G, a field's scaler and offset, and the remembered boxes. Each drives the
//! model the screen drives, with the fixtures the scripts use.

use std::collections::BTreeMap;

use super::*;

fn testdata(name: &str) -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata")
        .join(name)
}

fn healthy() -> std::path::PathBuf {
    testdata("dataflash.bin")
}

fn damaged() -> std::path::PathBuf {
    testdata("dataflash_damaged.bin")
}

fn opened(path: &std::path::Path) -> LogBrowse {
    let mut browse = LogBrowse::new();
    browse.open(path);
    assert!(browse.is_open(), "{}", path.display());
    browse
}

fn field(browse: &LogBrowse, message: &str, name: &str) -> PlottableField {
    browse
        .fields()
        .iter()
        .find(|field| field.message == message && field.field == name)
        .unwrap_or_else(|| panic!("{message}.{name}"))
        .clone()
}

fn facts(browse: &LogBrowse) -> BTreeMap<String, String> {
    browse.facts().into_iter().collect()
}

/// A scratch copy of a fixture, for the tests that write beside the log.
fn scratch_copy(path: &std::path::Path, tag: &str) -> std::path::PathBuf {
    let directory = std::env::temp_dir().join(format!("mp-logbrowse-{tag}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&directory);
    std::fs::create_dir_all(&directory).expect("scratch directory");
    let copy = directory.join(path.file_name().expect("a file"));
    std::fs::copy(path, &copy).expect("copy the fixture");
    copy
}

fn index_of(browse: &LogBrowse, name: &str) -> usize {
    browse
        .graphs()
        .iter()
        .position(|graph| graph.name == name)
        .unwrap_or_else(|| panic!("no graph {name}"))
}

/// Show Params lists every parameter the log carries, once each, sorted naturally, and the box
/// never stays ticked.
#[test]
fn show_params_lists_the_logs_parameters() {
    let mut browse = opened(&healthy());
    assert_eq!(facts(&browse)["log.params.shown"], "false");
    browse.show_params();
    let view = browse.params_view().expect("the list is shown");
    assert_eq!(
        view.rows.len(),
        982,
        "one per parameter the .param file saved"
    );
    for pair in view.rows.windows(2) {
        assert_ne!(
            mp_log::logparams::natural_compare(&pair[0].name, &pair[1].name),
            std::cmp::Ordering::Greater
        );
    }
    let facts = facts(&browse);
    assert_eq!(facts["log.params.shown"], "true");
    assert_eq!(facts["log.params.count"], "982");
    assert_eq!(facts["log.params.first"], "ACRO_BAL_PITCH=1");
    assert_eq!(facts["log.check.params"], "false");
    browse.scroll_params(10_000);
    assert_eq!(
        browse.params_view().map(|view| view.first),
        Some(982 - PARAM_ROWS)
    );
    browse.close_params();
    assert!(browse.params_view().is_none());
}

/// The drop-down is filled when a log opens, starts on "a/None", and a set graphs each of its
/// pieces the log has, labelled as expressions and on the unit-less axis.
#[test]
fn a_preselected_graph_graphs_its_pieces() {
    let mut browse = opened(&damaged());
    assert_eq!(browse.preselected(), Some("a/None"));
    assert!(browse.graphs().len() > 300);
    let roll_and_pitch = index_of(&browse, "Attitude/Roll and Pitch mavgraphs");
    browse.choose_preselect(roll_and_pitch);
    let labels: Vec<&str> = browse
        .plotted()
        .iter()
        .map(|shown| shown.label.as_str())
        .collect();
    // The MAVLink pieces, `degrees(ATTITUDE.roll)`, find no records in a dataflash log.
    assert_eq!(labels, vec!["ATT.Roll.", "ATT.Pitch."]);
    assert!(browse.plotted().iter().all(|shown| shown.unit.is_empty()));
    let facts = facts(&browse);
    assert_eq!(facts["log.preselect.graphed"], "2");
    assert_eq!(facts["log.preselect.skipped"], "0");
    assert_eq!(
        facts["log.preselect.selected"],
        "Attitude/Roll and Pitch mavgraphs"
    );
    // The same values the field list plots, unscaled.
    let roll = field(&browse, "ATT", "Roll");
    let data = std::fs::read(damaged()).expect("fixture");
    let points = mp_log::plot::extract_instance(&data, "ATT", None, "Roll");
    assert_eq!(browse.plotted()[0].series.len(), points.len());
    assert!(roll.samples > 0);

    // A right-axis set, and one that needs Python: its pieces are named, the rest graphed.
    let failure = index_of(&browse, "Builtin/Mechanical Failure");
    browse.choose_preselect(failure);
    assert!(browse.right_count() >= 1, "CTUN.Alt:2 is on the right");
    // "a/None" does nothing, not even clear the chart.
    let plotted = browse.plotted().len();
    browse.choose_preselect(0);
    assert_eq!(browse.plotted().len(), plotted);
    let circular = index_of(&browse, "Attitude/Circular Angle mavgraphs");
    browse.choose_preselect(circular);
    assert_eq!(
        browse
            .plotted()
            .iter()
            .map(|shown| shown.label.as_str())
            .collect::<Vec<_>>(),
        vec!["sqrt(ATT.Roll*ATT.Roll+ATT.Pitch*ATT.Pitch)."]
    );
}

/// A piece Python would run and this evaluator cannot is left out, by name.
#[test]
fn a_piece_needing_python_is_named_in_the_status() {
    let mut browse = opened(&damaged());
    let index = browse
        .graphs()
        .iter()
        .position(|graph| {
            graph
                .items
                .iter()
                .flatten()
                .any(|item| item.graphed().starts_with("expected_mag_yaw"))
        })
        .expect("a graph using expected_mag_yaw");
    browse.choose_preselect(index);
    let facts = facts(&browse);
    assert_ne!(facts["log.preselect.skipped"], "0");
    assert!(
        browse
            .status()
            .is_some_and(|status| status.contains("expected_mag_yaw")),
        "{:?}",
        browse.status()
    );
}

/// With the map shown, `DrawMap` draws every route of the log with nothing plotted, and only
/// the stretch the chart shows once something is; the healthy log's mission is drawn with its
/// markers.
#[test]
fn the_map_draws_every_route_of_the_stretch_charted() {
    let mut browse = opened(&damaged());
    assert_eq!(
        facts(&browse)["log.map.drawn.gps"],
        "0",
        "the map is hidden"
    );
    browse.toggle_check(Check::Map);
    let facts_now = facts(&browse);
    assert_eq!(facts_now["log.map.drawn.gps"], "63");
    assert_eq!(facts_now["log.map.drawn.pos"], "119");
    assert!(
        browse.fit_routes.get(),
        "the map fits the routes when next painted"
    );
    // Zoomed on a line axis to the first tenth of the log: fewer points.
    browse.toggle_check(Check::Time);
    let roll = field(&browse, "ATT", "Roll");
    browse.toggle(&roll);
    let all = browse.drawn().pos.len();
    browse.chart_drag_zoom((0.0, 0.0), (0.1, 1.0), (400.0, 260.0));
    let (low, high) = browse.x_range().expect("a range");
    assert!(browse.drawn().pos.len() < all);
    assert!(
        browse
            .drawn()
            .pos
            .iter()
            .all(|point| (low.trunc()..=high).contains(&(point.line as f64)))
    );

    let mut healthy = opened(&healthy());
    healthy.toggle_check(Check::Map);
    let facts_healthy = facts(&healthy);
    assert_eq!(facts_healthy["log.map.drawn.gps"], "0", "no fix");
    assert_eq!(facts_healthy["log.map.drawn.cmd"], "6");
    assert!(
        facts_healthy["log.map.drawn.markers"]
            .parse::<usize>()
            .unwrap_or(0)
            >= 6
    );
}

/// ZedGraph's wheel, rectangle and pan, each on the stack, and the menu walking it back.
#[test]
fn the_chart_zooms_pans_and_undoes() {
    let mut browse = opened(&damaged());
    // Laid out as the screen lays it out, so a drag has a size in pixels.
    browse.chart_bounds.set(Some(Bounds {
        origin: gpui::point(px(0.0), px(0.0)),
        size: gpui::size(px(400.0), px(PLOT_HEIGHT)),
    }));
    let roll = field(&browse, "ATT", "Roll");
    browse.toggle(&roll);
    let full = browse.x_range().expect("a range");
    assert_eq!(facts(&browse)["log.zoom.x"], "auto");

    browse.chart_wheel(false);
    let zoomed = browse.x_range().expect("a range");
    assert!(zoomed.1 - zoomed.0 < full.1 - full.0);
    assert_eq!(facts(&browse)["log.zoom.depth"], "1");

    browse.chart_press((0.2, 0.2), false);
    browse.chart_move((0.6, 0.8));
    assert_eq!(facts(&browse)["log.zoom.dragging"], "true");
    browse.chart_release();
    assert_eq!(facts(&browse)["log.zoom.depth"], "2");
    assert_eq!(facts(&browse)["log.zoom.undo"], "Un-Zoom");

    browse.chart_press((0.5, 0.5), true);
    browse.chart_move((0.4, 0.5));
    browse.chart_release();
    assert_eq!(facts(&browse)["log.zoom.undo"], "Un-Pan");

    browse.open_chart_menu((0.5, 0.5));
    assert_eq!(facts(&browse)["log.chart.menu"], "true");
    browse.chart_menu_item("unzoom");
    assert_eq!(facts(&browse)["log.zoom.depth"], "2");
    browse.chart_menu_item("undo_all");
    assert_eq!(facts(&browse)["log.zoom.depth"], "0");
    assert_eq!(browse.x_range(), Some(full));
    browse.chart_wheel(true);
    browse.chart_menu_item("set_default");
    assert_eq!(browse.x_range(), Some(full));
    assert_eq!(facts(&browse)["log.zoom.depth"], "2");

    // A new curve zooms out all the way.
    let pitch = field(&browse, "ATT", "Pitch");
    browse.toggle(&pitch);
    assert_eq!(facts(&browse)["log.zoom.depth"], "0");
}

/// A notch is one wheel event and one step; a trackpad's pixels are carried until they make one.
#[test]
fn each_wheel_notch_is_one_step() {
    let mut browse = opened(&damaged());
    let roll = field(&browse, "ATT", "Roll");
    browse.toggle(&roll);
    for _ in 0..3 {
        assert!(browse.chart_wheel_pixels(WHEEL_NOTCH));
    }
    assert_eq!(browse.zoom().depth(), 3);
    assert!(!browse.chart_wheel_pixels(7.0));
    assert!(!browse.chart_wheel_pixels(7.0));
    assert!(browse.chart_wheel_pixels(7.0));
    assert_eq!(browse.zoom().depth(), 4);
    assert_eq!(browse.zoom().top(), Some(view::Kind::WheelZoom));
    let narrow = browse.x_range().expect("a range");
    // A twentieth of a notch is still carried; two notches back make one whole one past it.
    assert!(browse.chart_wheel_pixels(-2.0 * WHEEL_NOTCH));
    assert_eq!(browse.zoom().depth(), 5);
    let wider = browse.x_range().expect("a range");
    assert!(
        wider.1 - wider.0 > narrow.1 - narrow.0,
        "towards the user zooms out"
    );
}

/// The zoom redraws the labels, which takes the cursor off, as `zg1_ZoomEvent` does.
#[test]
fn a_zoom_relabels_the_chart() {
    let mut browse = opened(&damaged());
    let roll = field(&browse, "ATT", "Roll");
    browse.toggle(&roll);
    browse.double_click(0.5);
    assert!(browse.cursor().is_some());
    browse.chart_wheel(false);
    assert_eq!(browse.cursor(), None);
}

/// Show Point Values is off as ZedGraph starts; on, the point nearest the pointer is shown as
/// `( time, value )`, the time as `HH:mm:ss.fff`.
#[test]
fn point_values_show_the_nearest_point() {
    let mut browse = opened(&damaged());
    let roll = field(&browse, "ATT", "Roll");
    browse.toggle(&roll);
    let size = (400.0, 260.0);
    let scales = browse.scales().expect("scales");
    let sample = *browse.plotted()[0]
        .series
        .samples()
        .nth(20)
        .expect("a sample");
    let range = *scales.left.values().next().expect("a left axis");
    let at = (
        (sample.at - scales.x.0) / (scales.x.1 - scales.x.0),
        1.0 - range.fraction(sample.value),
    );
    browse.chart_move(at);
    assert_eq!(browse.point_tooltip(size), None, "off, as ZedGraph starts");
    browse.chart_menu_item("show_val");
    assert!(browse.point_values());
    let tooltip = browse
        .point_tooltip(size)
        .expect("a point under the pointer");
    assert!(
        tooltip.starts_with("( ") && tooltip.ends_with(" )"),
        "{tooltip}"
    );
    let (time, value) = tooltip
        .trim_start_matches("( ")
        .trim_end_matches(" )")
        .split_once(", ")
        .expect("x, y");
    assert_eq!(time.len(), "12:34:56.789".len(), "{time}");
    assert_eq!(time.matches(':').count(), 2, "{time}");
    let shown: f64 = value.parse().expect("a number");
    assert!(
        (shown - sample.value).abs() < 1e-9,
        "{shown} {}",
        sample.value
    );

    // On a line axis the x is the line number.
    browse.toggle_check(Check::Time);
    browse.toggle(&roll);
    let scales = browse.scales().expect("scales");
    let sample = *browse.plotted()[0]
        .series
        .samples()
        .nth(20)
        .expect("a sample");
    let range = *scales.left.values().next().expect("a left axis");
    browse.chart_move((
        (sample.at - scales.x.0) / (scales.x.1 - scales.x.0),
        1.0 - range.fraction(sample.value),
    ));
    let tooltip = browse.point_tooltip(size).expect("a point");
    assert!(
        tooltip.starts_with(&format!("( {}, ", sample.at)),
        "{tooltip}"
    );
}

/// A double click on a row moves the map and the chart to its record - its record, filtered or
/// not - and leaves the grid where it is.
#[test]
fn a_double_clicked_row_goes_to_its_record() {
    let mut browse = opened(&damaged());
    browse.toggle_check(Check::DataTable);
    browse.toggle_check(Check::Time);
    let roll = field(&browse, "ATT", "Roll");
    browse.toggle(&roll);
    browse.filter_grid(Some("GPS"));
    let line = browse
        .grid()
        .and_then(|grid| grid.line_of_row(30))
        .expect("a GPS row");
    browse.select_cell(0, 1);
    browse.grid_double_click(3);
    let third = browse
        .grid()
        .and_then(|grid| grid.line_of_row(3))
        .expect("a GPS row");
    assert_eq!(browse.cursor(), Some(third));
    let (low, high) = browse.x_range().expect("a range");
    #[allow(clippy::cast_precision_loss)]
    let centre = third as f64;
    assert!(((low + high) / 2.0 - centre).abs() < 1e-6, "centred on it");
    assert_eq!(browse.grid_current(), Some((0, 1)), "the grid stays");
    assert_ne!(line, third);
}

/// Ctrl+G's line is 1-based; a line past the end, or no number, does not exist.
#[test]
fn ctrl_g_goes_to_a_line() {
    let mut browse = opened(&healthy());
    browse.ask_go_to_line();
    assert_eq!(facts(&browse)["log.prompt"], "Line no");
    browse.prompt_ok();
    assert_eq!(
        browse.refused(),
        Some("Line Doesn't Exist"),
        "the grid has no rows"
    );
    browse.toggle_check(Check::DataTable);
    browse.go_to_line("100");
    assert_eq!(browse.grid_current(), Some((99, 1)));
    assert_eq!(browse.refused(), None);
    browse.go_to_line("0");
    assert_eq!(browse.refused(), Some("Line Doesn't Exist"));
    browse.go_to_line("11440");
    assert_eq!(browse.refused(), Some("Line Doesn't Exist"));
    browse.go_to_line("five");
    assert_eq!(browse.refused(), Some("Line Doesn't Exist"));
    browse.go_to_line("11439");
    assert_eq!(browse.grid_current(), Some((11438, 1)));
}

/// A field's scaler applies the next time it is graphed, and its text goes on the label.
#[test]
fn a_fields_scaler_applies_when_it_is_next_graphed() {
    let mut browse = opened(&damaged());
    let roll = field(&browse, "ATT", "Roll");
    browse.toggle(&roll);
    let plain: Vec<f64> = browse.plotted()[0]
        .series
        .samples()
        .map(|sample| sample.value)
        .collect();
    browse.ask_modifier(&roll);
    let prompt = browse.prompt().expect("the input box");
    assert_eq!(prompt.title, "Apply scaler and offset to ATT.Roll");
    browse.set_modifier("ATT.Roll", "x2 +1");
    browse.prompt_cancel();
    assert_eq!(facts(&browse)["log.modifiers"], "1");
    assert_eq!(
        browse.plotted()[0].series.len(),
        plain.len(),
        "not until graphed"
    );
    browse.toggle(&roll);
    browse.toggle(&roll);
    let shown = &browse.plotted()[0];
    assert!(shown.label.contains("x2 +1"), "{}", shown.label);
    for (scaled, value) in shown.series.samples().zip(&plain) {
        assert!((scaled.value - value.mul_add(2.0, 1.0)).abs() < 1e-9);
    }
    // An empty answer removes it.
    browse.set_modifier("ATT.Roll", "");
    assert_eq!(facts(&browse)["log.modifiers"], "0");
}

/// Export Visible writes every row the grid holds, each cell and a comma, as wide as the C#'s
/// grid; Export Files writes the log's files, and this log carries none.
#[test]
fn the_grid_menu_exports_rows_and_files() {
    let log = scratch_copy(&healthy(), "export");
    let mut browse = opened(&log);
    browse.toggle_check(Check::DataTable);
    browse.filter_grid(Some("ATT"));
    browse.toggle_grid_menu();
    assert!(browse.grid_menu());
    browse.ask_export_visible();
    assert!(!browse.grid_menu());
    assert_eq!(facts(&browse)["log.prompt"], "Save As");
    browse.prompt_ok();
    let written = std::fs::read_to_string(log.with_file_name("output.csv")).expect("the export");
    let lines: Vec<&str> = written.lines().collect();
    assert_eq!(lines.len(), 182, "the ATT rows");
    let columns = browse.grid().map_or(0, Grid::csv_columns);
    assert!(columns > 20);
    for line in &lines {
        assert!(line.starts_with(|c: char| c.is_ascii_digit()));
        assert!(line.contains(",ATT,"));
        assert_eq!(line.matches(',').count(), columns, "{line}");
    }
    assert_eq!(facts(&browse)["log.exported"], "182 rows");

    browse.ask_export_files();
    browse.prompt_ok();
    assert!(
        browse.refused().is_some(),
        "an empty folder name is no folder"
    );
    // The healthy log carries eight: a Lua script and the board's reports.
    browse.export_files("files");
    assert_eq!(facts(&browse)["log.exported"], "8 files");
    let script = std::fs::read_to_string(log.with_file_name("files").join("drop_test_1.lua"))
        .expect("the script");
    assert!(!script.is_empty());
    let _ = std::fs::remove_dir_all(log.parent().expect("scratch"));

    // The damaged log carries five files in 134 FILE records: the board's hwdef and four of its
    // @SYS reports, each rebuilt from its 64-byte pieces.
    let log = scratch_copy(&damaged(), "files");
    let mut browse = opened(&log);
    browse.export_files("files");
    assert_eq!(facts(&browse)["log.exported"], "5 files");
    let folder = log.with_file_name("files");
    let uarts = std::fs::read_to_string(folder.join("@SYS/uarts.txt")).expect("uarts.txt");
    assert!(uarts.starts_with("UARTV1"), "{uarts:?}");
    assert!(uarts.contains("SERIAL0 OTG1"), "{uarts:?}");
    assert!(folder.join("@ROMFS/hwdef.dat").is_file());
    let _ = std::fs::remove_dir_all(log.parent().expect("scratch"));
}

/// `LoadLog2` sets six boxes from config.xml; Events is not remembered.
#[test]
fn the_remembered_boxes_are_read_as_the_log_opens() {
    let mut browse = opened(&damaged());
    let settings: BTreeMap<&str, &str> = BTreeMap::from([
        ("LB_Map", "True"),
        ("LB_Time", "false"),
        ("LB_Grid", " TRUE "),
        ("LB_MSG", "no"),
    ]);
    browse.apply_remembered(|key| settings.get(key).map(|value| (*value).to_owned()));
    let facts = facts(&browse);
    assert_eq!(facts["log.check.map"], "true");
    assert_eq!(facts["log.axis"], "line");
    assert_eq!(facts["log.check.datagrid"], "true");
    assert_eq!(facts["log.check.msg"], "true", "not a bool: the default");
    assert_eq!(facts["log.check.events"], "true");
    assert_eq!(Check::Events.setting(), None);
    assert_eq!(setting_text(true), "True");
    assert_eq!(setting_text(false), "False");
    assert!(
        !browse.drawn().pos.is_empty(),
        "the map drew when it was ticked"
    );
}

/// The ledger's claims hold in the source it names: every id a row claims is drawn by this
/// screen. (The ledger's own tests hold it to the designer.)
#[test]
fn the_ledgers_counts_are_reported() {
    let (done, _, missing, plumbing, dropped) = coverage::counts(coverage::LOGBROWSE);
    assert_eq!(
        done + missing + plumbing + dropped,
        coverage::LOGBROWSE.len()
    );
}

/// `tests/gui/log-params.gui`, step for step, against the model.
#[test]
fn the_log_params_script_holds_without_a_window() {
    let mut browse = opened(&healthy());
    browse.show_params();
    let shown = facts(&browse);
    assert_eq!(shown["log.params.shown"], "true");
    assert_eq!(shown["log.params.count"], "982");
    assert_eq!(shown["log.params.first"], "ACRO_BAL_PITCH=1");
    assert_eq!(shown["log.params.defaults"], "false");
    assert_eq!(shown["log.params.top"], "0");
    browse.scroll_params(3);
    let shown = facts(&browse);
    assert_eq!(shown["log.params.top"], "3");
    assert_eq!(shown["log.params.first"], "ACRO_BAL_PITCH=1");
    browse.close_params();
    assert_eq!(facts(&browse)["log.params.shown"], "false");
}

/// `tests/gui/log-preselect.gui`, step for step, against the model.
#[test]
fn the_log_preselect_script_holds_without_a_window() {
    let mut browse = opened(&damaged());
    let facts_now = facts(&browse);
    assert_eq!(facts_now["log.preselect.selected"], "a/None");
    assert_eq!(facts_now["log.preselect.open"], "false");
    browse.toggle_preselect_list();
    assert_eq!(facts(&browse)["log.preselect.open"], "true");
    // The script picks the item on the list's first page.
    let index = index_of(&browse, "Attitude/Roll and Pitch mavgraphs");
    assert!(index < PRESELECT_ROWS, "on the first page: {index}");
    browse.choose_preselect(index);
    assert_eq!(index, 9, "log-preselect-item-9");
    let facts_now = facts(&browse);
    assert_eq!(facts_now["log.preselect.items"], "378");
    assert_eq!(facts_now["log.preselect.open"], "false");
    assert_eq!(facts_now["log.preselect.graphed"], "2");
    assert_eq!(facts_now["log.preselect.skipped"], "0");
    assert_eq!(browse.plotted().len(), 2);
    assert_eq!(browse.right_count(), 0);
    // Reopened, the list shows item 0 again, and choosing it changes nothing on the chart.
    browse.toggle_preselect_list();
    assert_eq!(browse.preselect_list, Some(0));
    browse.choose_preselect(0);
    assert_eq!(facts(&browse)["log.preselect.selected"], "a/None");
    assert_eq!(browse.plotted().len(), 2);
}

/// `tests/gui/log-routes.gui`, step for step, against the model.
#[test]
fn the_log_routes_script_holds_without_a_window() {
    let mut browse = opened(&damaged());
    browse.toggle_check(Check::Map);
    let facts_now = facts(&browse);
    assert_eq!(facts_now["log.map.drawn.gps"], "63");
    assert_eq!(facts_now["log.map.drawn.gps2"], "0");
    assert_eq!(facts_now["log.map.drawn.gpsb"], "0");
    assert_eq!(facts_now["log.map.drawn.pos"], "119");
    browse.toggle_check(Check::Time);
    let roll = field(&browse, "ATT", "Roll");
    browse.toggle(&roll);
    for _ in 0..3 {
        browse.chart_wheel(false);
    }
    let facts_now = facts(&browse);
    assert_eq!(facts_now["log.zoom.depth"], "3");
    assert!(
        facts_now["log.map.drawn.pos"]
            .parse::<usize>()
            .unwrap_or(119)
            < 119
    );
    // The second log reads the ticked Map back from config.xml, as LB_Map.
    let mut healthy = opened(&healthy());
    healthy.apply_remembered(|key| (key == "LB_Map").then(|| "True".to_owned()));
    let facts_now = facts(&healthy);
    assert_eq!(facts_now["log.check.map"], "true");
    assert_eq!(facts_now["log.map.drawn.gps"], "0");
    assert_eq!(facts_now["log.map.drawn.pos"], "0");
    assert_eq!(facts_now["log.map.drawn.cmd"], "6");
    assert!(
        facts_now["log.map.drawn.markers"]
            .parse::<usize>()
            .unwrap_or(0)
            >= 6
    );
}
