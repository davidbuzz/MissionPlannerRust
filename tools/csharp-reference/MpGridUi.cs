// The `accept` verb of MpGrid.exe: Mission Planner's Survey (Grid) dialog, Grid/GridUI.cs, run
// without its form, for crates/mp-mission/tests/gridui_vectors.rs and D11.
//
// GridUI is a WinForms Form: its constructor needs InitializeComponent, a GMap control, MainV2 and
// the flight planner behind plugin.Host, none of which run headless. What the dialog computes does
// not need any of them, so this file is GridUI.cs's own code for it - the constructor's defaults,
// xmlcamera's reader, CMB_camera_SelectedIndexChanged, CMB_startfrom_SelectedIndexChanged,
// domainUpDown1_ValueChanged with doCalc, getFOV, calcpolygonarea, secondsToNice and
// CalcHeadingHold, and BUT_Accept_Click with AddWP and savesettings - statement for statement,
// with each control replaced by a field holding what the control holds: a NumericUpDown's decimal
// Value with its Designer range, a TextBox's Text, a CheckBox's Checked. The map drawing is left
// out; everything it computed is kept. What reaches the mission is what BUT_Accept_Click hands
// plugin.Host.AddWPtoList and InsertWP, recorded call by call.
//
// The arithmetic is therefore the C#'s own: decimal, float and double as the dialog mixes them,
// .NET's formatting of the Stats labels, Grid.CreateGrid/CreateCorridor/CreateRotary from the
// Utilities build, ProjNet's UTM for the area, GMap's route distance. Three things are fixed rather
// than read from a running Mission Planner, and each golden records them:
//   - units are metric: CurrentState.multiplierdist and multiplierspeed are 1 and `distunits` is
//     unset (CurrentState.cs:27, :32; GridUI.cs:94), as they are on a fresh install;
//   - srtm.getAltitude answers altresponce.Invalid, altitude 0: there is no terrain data here, as
//     there is none in the Rust application (ExtLibs/Utilities/srtm.cs:116);
//   - PlannedHomeLocation, the vehicle's firmware, the planner's existing row count and the
//     WPNAV_SPEED / WP_SPD parameters come from the case (`HomeLocation`, `firmware`, `rows`,
//     `WPNAV_SPEED`, `WP_SPD`).
//
// A case, in testdata/grid/cases.txt:
//   accept <name> <polygon> [<control>=<value> ...]
// Each control assignment is what the operator does to that control, in order, with the events
// the control raises: a NumericUpDown typed into (ParseEditText: decimal.Parse, clamped to the
// range, ValueChanged if the value moved), a CheckBox clicked when it is not already as asked
// (CheckedChanged, then Click), a RadioButton clicked, a TextBox's text replaced (TextChanged), an
// item picked in CMB_camera or CMB_startfrom (SelectedIndexChanged). Spaces in a value are written
// %20. `point=<n>` is the number typed into CMB_startfrom's "Enter point #" box.
//
// Output, <outdir>/<name>.csv, `key,value...` lines: the case, its vertices and context, every
// control assignment as given, then every control's state after it (decimals as Value.ToString(),
// so the scale is kept), the calculated text boxes, the Stats labels, how many times the grid was
// generated, the grid's points, Accept's calls - `call,add,-,...` or `call,insert,<index>,...`,
// then the command and its seven numbers G17 - the rows they leave, and the settings Accept saves.
// <outdir>/cameras.csv is the camera list xmlcamera reads from camerasBuiltin.xml.

using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Xml;
using GMap.NET;
using MissionPlanner.Utilities;
using ProjNet.CoordinateSystems;
using ProjNet.CoordinateSystems.Transformations;
using GeoAPI.CoordinateSystems;
using GeoAPI.CoordinateSystems.Transformations;

public static class MpGridUi
{
    static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    // Grid/camerainfo.cs
    public struct camerainfo
    {
        public string name;
        public float focallen;
        public float sensorwidth;
        public float sensorheight;
        public float imagewidth;
        public float imageheight;
    }

    // The MAV_CMD values BUT_Accept_Click and AddWP use (MAVLink's MAV_CMD enum).
    const int WAYPOINT = 16, RETURN_TO_LAUNCH = 20, LAND = 21, TAKEOFF = 22, SPLINE_WAYPOINT = 82,
        CONDITION_YAW = 115, DO_JUMP = 177, DO_CHANGE_SPEED = 178, DO_SET_SERVO = 183,
        DO_REPEAT_SERVO = 184, DO_DIGICAM_CONTROL = 203, DO_SET_CAM_TRIGG_DIST = 206;

    // A NumericUpDown: the Value property as System.Windows.Forms.NumericUpDown has it - range
    // checked, ValueChanged only when the value moves - and ParseEditText's Constrain.
    public class Num
    {
        public string Name;
        decimal currentValue;
        public decimal Minimum;
        public decimal Maximum;
        public event Action ValueChanged;

        public Num(string name, decimal value, decimal minimum, decimal maximum)
        {
            Name = name;
            currentValue = value;
            Minimum = minimum;
            Maximum = maximum;
        }

        public decimal Value
        {
            get { return currentValue; }
            set
            {
                if (value != currentValue)
                {
                    if (value < Minimum || value > Maximum)
                        throw new ArgumentOutOfRangeException(Name);
                    currentValue = value;
                    if (ValueChanged != null)
                        ValueChanged();
                }
            }
        }

        // What typing and leaving the box does: ParseEditText, Value = Constrain(decimal.Parse(Text)).
        public void Type(string text)
        {
            decimal parsed = decimal.Parse(text, NumberStyles.Number, Inv);
            if (parsed < Minimum) parsed = Minimum;
            if (parsed > Maximum) parsed = Maximum;
            Value = parsed;
        }
    }

    // A TextBox: TextChanged when the text changes.
    public class Txt
    {
        string text;
        public event Action TextChanged;
        public Txt(string initial) { text = initial; }
        public string Text
        {
            get { return text; }
            set
            {
                if (value != text)
                {
                    text = value;
                    if (TextChanged != null)
                        TextChanged();
                }
            }
        }
    }

    // A CheckBox or RadioButton: CheckedChanged when it changes, Click when the operator clicks.
    public class Chk
    {
        bool isChecked;
        public event Action CheckedChanged;
        public event Action Click;
        public Chk(bool initial) { isChecked = initial; }
        public bool Checked
        {
            get { return isChecked; }
            set
            {
                if (value != isChecked)
                {
                    isChecked = value;
                    if (CheckedChanged != null)
                        CheckedChanged();
                }
            }
        }
        public void OnClick()
        {
            if (Click != null)
                Click();
        }
    }

    // One case: the dialog's state and the planner's around it.
    class Dialog
    {
        // Variables, GridUI.cs:36-37
        const double rad2deg = (180 / Math.PI);
        const double deg2rad = (1.0 / rad2deg);

        public List<PointLatLngAlt> list = new List<PointLatLngAlt>();
        public List<PointLatLngAlt> grid;
        bool loading = false;
        public int recomputes = 0;

        public Dictionary<string, camerainfo> cameras = new Dictionary<string, camerainfo>();
        public List<string> CMB_camera_Items = new List<string>();
        public string CMB_camera_Text = "";

        public string DistUnits = "";
        public string inchpixel = "";
        public string feet_fovH = "";
        public string feet_fovV = "";

        // The planner around the dialog.
        public PointLatLngAlt PlannedHomeLocation = new PointLatLngAlt();
        public string firmware = "ArduCopter2";
        public int rowsBefore = 0;
        public double? WPNAV_SPEED;
        public double? WP_SPD;
        public int point = 1;

        // Controls, with the Designer's values (GridUI.Designer.cs) and the .resx's text.
        public Num NUM_altitude = new Num("NUM_altitude", 100, 1, 99999);                    // :1316-1333
        public Num NUM_angle = new Num("NUM_angle", 0, 0, 360);                                // :1290
        public Num NUM_UpDownFlySpeed = new Num("NUM_UpDownFlySpeed", 5, 0, 360);             // :1264-1270
        public Num NUM_split = new Num("NUM_split", 1, 1, 300);                                // :1215-1226
        public Num NUM_Distance = new Num("NUM_Distance", 50, 0.3m, 9999);                     // :1138-1149
        public Num NUM_spacing = new Num("NUM_spacing", 0, 0, 5000);                           // :1164
        public Num NUM_overshoot = new Num("NUM_overshoot", 0, -999, 9999);                   // :1116-1121
        public Num NUM_overshoot2 = new Num("NUM_overshoot2", 0, -999, 9999);                 // :1049-1054
        public Num NUM_leadin = new Num("NUM_leadin", 0, -999, 9999);                         // :1033-1038
        public Num NUM_leadin2 = new Num("NUM_leadin2", 0, -999, 9999);                       // :964-969
        public Num num_overlap = new Num("num_overlap", 50, 0, 100);                           // :1089
        public Num num_sidelap = new Num("num_sidelap", 60, 0, 100);                           // :1101
        public Num num_corridorwidth = new Num("num_corridorwidth", 100, 1, 5000);             // :1000-1011
        public Num NUM_copter_delay = new Num("NUM_copter_delay", 0, 0, 9999);                 // :929
        public Num NUM_Lane_Dist = new Num("NUM_Lane_Dist", 0, 0, 9999);                       // :848
        public Num NUM_laps = new Num("NUM_laps", 200, 1, 9999);                               // :761-772
        public Num NUM_clockwise_laps = new Num("NUM_clockwise_laps", 0, -1, 9999);            // :799-804
        public Num NUM_focallength = new Num("NUM_focallength", 5, 1, 500);                    // :678-689
        public Num NUM_reptservo = new Num("NUM_reptservo", 5, 5, 12);                         // :538-549
        public Num num_reptpwm = new Num("num_reptpwm", 1100, 0, 5000);                        // :523-529
        public Num NUM_repttime = new Num("NUM_repttime", 2, 1, 5000);                         // :503-514
        public Num num_setservono = new Num("num_setservono", 5, 5, 12);                       // :413-424
        public Num num_setservolow = new Num("num_setservolow", 1100, 0, 5000);                // :443-449
        public Num num_setservohigh = new Num("num_setservohigh", 1900, 0, 5000);              // :458-464

        public Chk CHK_camdirection = new Chk(true);
        public Chk CHK_usespeed = new Chk(false);
        public Chk CHK_toandland = new Chk(true);
        public Chk CHK_toandland_RTL = new Chk(true);
        public Chk CHK_internals = new Chk(false);
        public Chk CHK_footprints = new Chk(false);
        public Chk CHK_advanced = new Chk(false);
        public Chk CHK_boundary = new Chk(true);
        public Chk CHK_markers = new Chk(true);
        public Chk CHK_grid = new Chk(true);
        public Chk chk_crossgrid = new Chk(false);
        public Chk chk_Corridor = new Chk(false);
        public Chk chk_spiral = new Chk(false);
        public Chk CHK_copter_headinghold = new Chk(false);
        public Chk CHK_copter_headingholdlock = new Chk(false);
        public Chk chk_spline = new Chk(false);
        public Chk chk_optimize_for_distance = new Chk(false);
        public Chk CHK_match_spiral_perimeter = new Chk(false);
        public Chk chk_stopstart = new Chk(true);
        public Chk rad_trigdist = new Chk(true);
        public Chk rad_digicam = new Chk(false);
        public Chk rad_repeatservo = new Chk(false);
        public Chk rad_do_set_servo = new Chk(false);

        public string CMB_startfrom_Text = "Home";
        public Txt TXT_headinghold = new Txt("0");
        public Txt TXT_imgwidth = new Txt("4608");
        public Txt TXT_imgheight = new Txt("3456");
        public Txt TXT_senswidth = new Txt("6.16");
        public Txt TXT_sensheight = new Txt("4.62");
        public Txt TXT_cmpixel = new Txt("");
        public Txt TXT_fovH = new Txt("");
        public Txt TXT_fovV = new Txt("");

        public string lbl_area = "0.00", lbl_distance = "0.00", lbl_spacing = "0.00", lbl_grndres = "0.00",
            lbl_distbetweenlines = "0.00", lbl_footprint = "0.00", lbl_turnrad = "0.00", lbl_gndelev = "0.00",
            lbl_minshutter = "0.00", lbl_pictures = "0.00", lbl_strips = "0.00", lbl_flighttime = "0.00",
            lbl_photoevery = "0.00";

        public List<string> calls = new List<string>();
        public List<int> rows = new List<int>();
        public string acceptMessage;
        public Dictionary<string, string> config = new Dictionary<string, string>();

        public IEnumerable<Num> Nums()
        {
            return new[] { NUM_altitude, NUM_angle, NUM_UpDownFlySpeed, NUM_split, NUM_Distance, NUM_spacing,
                NUM_overshoot, NUM_overshoot2, NUM_leadin, NUM_leadin2, num_overlap, num_sidelap,
                num_corridorwidth, NUM_copter_delay, NUM_Lane_Dist, NUM_laps, NUM_clockwise_laps,
                NUM_focallength, NUM_reptservo, num_reptpwm, NUM_repttime, num_setservono,
                num_setservolow, num_setservohigh };
        }

        public Dictionary<string, Chk> Chks()
        {
            return new Dictionary<string, Chk> {
                {"CHK_camdirection", CHK_camdirection}, {"CHK_usespeed", CHK_usespeed},
                {"CHK_toandland", CHK_toandland}, {"CHK_toandland_RTL", CHK_toandland_RTL},
                {"CHK_internals", CHK_internals}, {"CHK_footprints", CHK_footprints},
                {"CHK_advanced", CHK_advanced}, {"CHK_boundary", CHK_boundary},
                {"CHK_markers", CHK_markers}, {"CHK_grid", CHK_grid},
                {"chk_crossgrid", chk_crossgrid}, {"chk_Corridor", chk_Corridor},
                {"chk_spiral", chk_spiral}, {"CHK_copter_headinghold", CHK_copter_headinghold},
                {"CHK_copter_headingholdlock", CHK_copter_headingholdlock}, {"chk_spline", chk_spline},
                {"chk_optimize_for_distance", chk_optimize_for_distance},
                {"CHK_match_spiral_perimeter", CHK_match_spiral_perimeter},
                {"chk_stopstart", chk_stopstart},
                {"rad_trigdist", rad_trigdist}, {"rad_digicam", rad_digicam},
                {"rad_repeatservo", rad_repeatservo}, {"rad_do_set_servo", rad_do_set_servo},
            };
        }

        public Dictionary<string, Txt> Txts()
        {
            return new Dictionary<string, Txt> {
                {"TXT_headinghold", TXT_headinghold}, {"TXT_imgwidth", TXT_imgwidth},
                {"TXT_imgheight", TXT_imgheight}, {"TXT_senswidth", TXT_senswidth},
                {"TXT_sensheight", TXT_sensheight}, {"TXT_cmpixel", TXT_cmpixel},
                {"TXT_fovH", TXT_fovH}, {"TXT_fovV", TXT_fovV},
            };
        }

        // The Designer's event wiring, GridUI.Designer.cs: which handler each control raises.
        public void Wire()
        {
            Action changed = () => domainUpDown1_ValueChanged();
            foreach (var n in new[] { NUM_altitude, NUM_angle, NUM_UpDownFlySpeed, NUM_split,
                         NUM_overshoot, NUM_overshoot2, NUM_leadin, NUM_leadin2, num_overlap,
                         num_sidelap, num_corridorwidth, NUM_laps, NUM_clockwise_laps })
                n.ValueChanged += changed;
            NUM_spacing.ValueChanged += () => { if (spacingHandlersOn) domainUpDown1_ValueChanged(); };
            NUM_Distance.ValueChanged += () => { if (spacingHandlersOn) domainUpDown1_ValueChanged(); };
            NUM_Lane_Dist.ValueChanged += () => domainUpDown1_ValueChanged();       // NUM_Lane_Dist_ValueChanged
            NUM_focallength.ValueChanged += () => domainUpDown1_ValueChanged();     // NUM_ValueChanged
            CHK_match_spiral_perimeter.CheckedChanged += changed;
            chk_optimize_for_distance.CheckedChanged += changed;
            chk_crossgrid.Click += changed;
            chk_Corridor.Click += changed;
            chk_spiral.Click += changed;
            CHK_footprints.CheckedChanged += changed;
            CHK_internals.CheckedChanged += changed;
            CHK_grid.CheckedChanged += changed;
            CHK_markers.CheckedChanged += changed;
            CHK_boundary.CheckedChanged += changed;
            CHK_camdirection.CheckedChanged += changed;                              // CHK_camdirection_CheckedChanged
            CHK_copter_headinghold.CheckedChanged += CHK_copter_headinghold_CheckedChanged;
            CHK_copter_headingholdlock.CheckedChanged += CHK_copter_headingholdlock_CheckedChanged;
            TXT_imgwidth.TextChanged += changed;                                     // TXT_TextChanged
            TXT_imgheight.TextChanged += changed;
            TXT_senswidth.TextChanged += changed;
            TXT_sensheight.TextChanged += changed;
        }

        // GridUI(GridPlugin plugin), GridUI.cs:66-129, the parts that are not the map.
        public void Construct(List<PointLatLngAlt> polygon, string builtinCameras)
        {
            loading = true;

            polygon.ForEach(x => { list.Add(x); });

            // CMB_startfrom.DataSource = Enum.GetNames(...); SelectedIndex = 0
            CMB_startfrom_Text = Enum.GetNames(typeof(Grid.StartPosition))[0];

            // set and angle that is good
            NUM_angle.Value = (decimal)((getAngleOfLongestSide(list) + 360) % 360);
            TXT_headinghold.Text = (Math.Round(NUM_angle.Value)).ToString();

            if (firmware == "ArduPlane")
                NUM_UpDownFlySpeed.Value = (decimal)(12 * 1f); // CurrentState.multiplierspeed

            xmlcamera(builtinCameras);

            loading = false;
        }

        // GridUI_Load, GridUI.cs:131-149, with no saved settings.
        public void Load()
        {
            loading = true;
            loading = false;
            domainUpDown1_ValueChanged();
        }

        // xmlcamera(false, filename), the reading half, GridUI.cs:512-583.
        void xmlcamera(string filename)
        {
            try
            {
                using (XmlTextReader xmlreader = new XmlTextReader(filename))
                {
                    while (xmlreader.Read())
                    {
                        xmlreader.MoveToElement();
                        try
                        {
                            switch (xmlreader.Name)
                            {
                                case "Camera":
                                    {
                                        camerainfo camera = new camerainfo();

                                        while (xmlreader.Read())
                                        {
                                            bool dobreak = false;
                                            xmlreader.MoveToElement();
                                            switch (xmlreader.Name)
                                            {
                                                case "name":
                                                    camera.name = xmlreader.ReadString();
                                                    break;
                                                case "imgw":
                                                    camera.imagewidth = float.Parse(xmlreader.ReadString(), new System.Globalization.CultureInfo("en-US"));
                                                    break;
                                                case "imgh":
                                                    camera.imageheight = float.Parse(xmlreader.ReadString(), new System.Globalization.CultureInfo("en-US"));
                                                    break;
                                                case "senw":
                                                    camera.sensorwidth = float.Parse(xmlreader.ReadString(), new System.Globalization.CultureInfo("en-US"));
                                                    break;
                                                case "senh":
                                                    camera.sensorheight = float.Parse(xmlreader.ReadString(), new System.Globalization.CultureInfo("en-US"));
                                                    break;
                                                case "flen":
                                                    camera.focallen = float.Parse(xmlreader.ReadString(), new System.Globalization.CultureInfo("en-US"));
                                                    break;
                                                case "Camera":
                                                    cameras[camera.name] = camera;
                                                    dobreak = true;
                                                    break;
                                            }
                                            if (dobreak)
                                                break;
                                        }
                                        string temp = xmlreader.ReadString();
                                    }
                                    break;
                                case "Config":
                                    break;
                                case "xml":
                                    break;
                                default:
                                    if (xmlreader.Name == "") // line feeds
                                        break;
                                    break;
                            }
                        }
                        catch (Exception ee) { Console.Error.WriteLine(ee.Message); } // silent fail on bad entry
                    }
                }
            }
            catch (Exception ex) { Console.Error.WriteLine("Bad Camera File: " + ex.ToString()); } // bad config file

            // populate list
            foreach (var camera in cameras.Values)
            {
                if (!CMB_camera_Items.Contains(camera.name))
                    CMB_camera_Items.Add(camera.name);
            }
        }

        // domainUpDown1_ValueChanged, GridUI.cs:586-872, with the map left out, the awaited
        // generators called directly, and srtm's answer 0.
        public void domainUpDown1_ValueChanged()
        {
            if (loading)
                return;

            if (CMB_camera_Text != "")
            {
                doCalc();
            }

            recomputes++;

            if (chk_Corridor.Checked)
            {
                grid = Grid.CreateCorridor(list, fromDistDisplayUnit((double)NUM_altitude.Value),
                    (double)NUM_Distance.Value, (double)NUM_spacing.Value, (double)NUM_angle.Value,
                    (double)NUM_overshoot.Value, (double)NUM_overshoot2.Value,
                    (Grid.StartPosition)Enum.Parse(typeof(Grid.StartPosition), CMB_startfrom_Text), false,
                    (float)NUM_Lane_Dist.Value, (float)num_corridorwidth.Value, (float)NUM_leadin.Value);
            }
            else if (chk_spiral.Checked)
            {
                grid = Grid.CreateRotary(list, fromDistDisplayUnit((double)NUM_altitude.Value),
                    (double)NUM_Distance.Value, (double)NUM_spacing.Value, (double)NUM_angle.Value,
                    (double)NUM_overshoot.Value, (double)NUM_overshoot2.Value,
                    (Grid.StartPosition)Enum.Parse(typeof(Grid.StartPosition), CMB_startfrom_Text), false,
                    (float)NUM_Lane_Dist.Value, (float)NUM_leadin.Value, PlannedHomeLocation,
                    (int)NUM_clockwise_laps.Value, CHK_match_spiral_perimeter.Checked, (int)NUM_laps.Value);
            }
            else
            {
                grid = Grid.CreateGrid(list,
                    fromDistDisplayUnit((double) NUM_altitude.Value),
                    (double) NUM_Distance.Value, (double) NUM_spacing.Value, (double) NUM_angle.Value,
                    (double) NUM_overshoot.Value, (double) NUM_overshoot2.Value,
                    (Grid.StartPosition) Enum.Parse(typeof(Grid.StartPosition), CMB_startfrom_Text),
                    false, (float) NUM_Lane_Dist.Value, (float) NUM_leadin.Value, (float) NUM_leadin2.Value,
                    PlannedHomeLocation, chk_optimize_for_distance.Checked);
            }

            if (grid.Count == 0)
            {
                return;
            }

            if (chk_crossgrid.Checked)
            {
                // add crossover
                Grid.StartPointLatLngAlt = grid[grid.Count - 1];

                grid.AddRange(Grid.CreateGrid(list,
                    fromDistDisplayUnit((double) NUM_altitude.Value),
                    (double) NUM_Distance.Value, (double) NUM_spacing.Value, (double) NUM_angle.Value + 90.0,
                    (double) NUM_overshoot.Value, (double) NUM_overshoot2.Value,
                    Grid.StartPosition.Point, false,
                    (float) NUM_Lane_Dist.Value, (float) NUM_leadin.Value, (float) NUM_leadin2.Value,
                    PlannedHomeLocation, chk_optimize_for_distance.Checked));
            }

            if (grid.Count == 0)
            {
                return;
            }

            int strips = 0;
            int images = 0;
            int a = 1;
            PointLatLngAlt prevprevpoint = grid[0];
            PointLatLngAlt prevpoint = grid[0];
            // distance to/from home
            double routetotal = grid.First().GetDistance(PlannedHomeLocation) / 1000.0 +
                               grid.Last().GetDistance(PlannedHomeLocation) / 1000.0;
            List<PointLatLng> segment = new List<PointLatLng>();
            double maxgroundelevation = double.MinValue;
            double mingroundelevation = double.MaxValue;
            double startalt = PlannedHomeLocation.Alt;

            foreach (var item in grid)
            {
                double currentalt = 0; // srtm.getAltitude(item.Lat, item.Lng).alt, altresponce.Invalid
                mingroundelevation = Math.Min(mingroundelevation, currentalt);
                maxgroundelevation = Math.Max(maxgroundelevation, currentalt);

                prevprevpoint = prevpoint;

                if (item.Tag == "M")
                {
                    images++;

                    if (CHK_internals.Checked)
                    {
                        a++;

                        segment.Add(prevpoint);
                        segment.Add(item);
                        prevpoint = item;
                    }
                }
                else
                {
                    if (item.Tag != "SM" && item.Tag != "ME")
                        strips++;

                    segment.Add(prevpoint);
                    segment.Add(item);
                    prevpoint = item;
                    a++;
                }
                MapRoute seg = new MapRoute(segment, "segment" + a.ToString());
                routetotal = routetotal + (float)seg.Distance;

                segment.Clear();
            }

            // turn radrad = tas^2 / (tan(angle) * G)
            float v_sq = (float)(((float)NUM_UpDownFlySpeed.Value / 1f) * ((float)NUM_UpDownFlySpeed.Value / 1f));
            float turnrad = (float)(v_sq / (float)(9.808f * Math.Tan(45 * deg2rad)));

            // Update Stats
            if (DistUnits == "Feet")
            {
                throw new NotSupportedException("the harness is metric");
            }
            else
            {
                // Meters
                lbl_area = calcpolygonarea(list).ToString("#") + " m^2";
                lbl_distance = routetotal.ToString("0.##") + " km";
                lbl_spacing = NUM_spacing.Value.ToString("0.#") + " m";
                lbl_grndres = TXT_cmpixel.Text;
                lbl_distbetweenlines = NUM_Distance.Value.ToString("0.##") + " m";
                lbl_footprint = TXT_fovH.Text + " x " + TXT_fovV.Text + " m";
                lbl_turnrad = (turnrad * 2).ToString("0") + " m";
                lbl_gndelev = mingroundelevation.ToString("0") + "-" + maxgroundelevation.ToString("0") + " m";
            }

            try
            {
                if (TXT_cmpixel.Text != "")
                {
                    // speed m/s
                    var speed = ((float) NUM_UpDownFlySpeed.Value / 1f);
                    // cmpix cm/pixel
                    var cmpix = float.Parse(TXT_cmpixel.Text.TrimEnd(new[] {'c', 'm', ' '}));
                    // m pix = m/pixel
                    var mpix = cmpix * 0.01;
                    // gsd / 2.0
                    var minmpix = mpix / 2.0;
                    // min sutter speed
                    var minshutter = speed / minmpix;
                    lbl_minshutter = "1/" + (minshutter - minshutter % 1).ToString();
                }
            }
            catch { }

            double flyspeedms = fromSpeedDisplayUnit((double)NUM_UpDownFlySpeed.Value);

            lbl_pictures = images.ToString();
            lbl_strips = ((int)(strips / 2)).ToString();
            double seconds = ((routetotal * 1000.0) / ((flyspeedms) * 0.8));
            // reduce flying speed by 20 %
            lbl_flighttime = secondsToNice(seconds);
            seconds = ((routetotal * 1000.0) / (flyspeedms));
            lbl_photoevery = secondsToNice(((double)NUM_spacing.Value / flyspeedms));

            CalcHeadingHold();
        }

        static double fromDistDisplayUnit(double input) { return input / 1f; } // CurrentState.cs:4375
        static double fromSpeedDisplayUnit(double input) { return input / 1f; } // CurrentState.cs:4380

        // AddWP, GridUI.cs:874-897
        void AddWP(double Lng, double Lat, double Alt, string tag, object gridobject = null)
        {
            if (CHK_copter_headinghold.Checked)
            {
                AddWPtoList(CONDITION_YAW, Convert.ToInt32(TXT_headinghold.Text), 0, 0, 0, 0, 0, 0, gridobject);
            }

            if (NUM_copter_delay.Value > 0)
            {
                AddWPtoList(WAYPOINT, (double)NUM_copter_delay.Value, 0, 0, 0, Lng, Lat, Alt * 1f, gridobject);
            }
            else
            {
                if ((tag == "S" || tag == "SM") && chk_spline.Checked)
                {
                    AddWPtoList(SPLINE_WAYPOINT, 0, 0, 0, 0, Lng, Lat, (int)(Alt * 1f), gridobject);
                }
                else
                {
                    AddWPtoList(WAYPOINT, 0, 0, 0, 0, Lng, Lat, (int)(Alt * 1f), gridobject);
                }
            }
        }

        // plugin.Host.AddWPtoList: FlightPlanner.AddCommand, `selectedrow = Commands.Rows.Add()`,
        // recorded. `rows` is the whole Commands grid, the rows there before the dialog as -1.
        int AddWPtoList(int cmd, double p1, double p2, double p3, double p4, double x, double y, double z, object tag = null)
        {
            calls.Add("add,-," + cmd + "," + D(p1) + "," + D(p2) + "," + D(p3) + "," + D(p4) + "," + D(x) + "," + D(y) + "," + D(z));
            rows.Add(cmd);
            return rows.Count - 1;
        }

        // plugin.Host.InsertWP: FlightPlanner.InsertCommand, which adds at the end when the index is
        // past the last row (FlightPlanner.cs:971-989), recorded.
        void InsertWP(int idx, int cmd, double p1, double p2, double p3, double p4, double x, double y, double z, object tag = null)
        {
            calls.Add("insert," + idx + "," + cmd + "," + D(p1) + "," + D(p2) + "," + D(p3) + "," + D(p4) + "," + D(x) + "," + D(y) + "," + D(z));
            if (rows.Count <= idx)
                rows.Add(cmd);
            else
                rows.Insert(idx, cmd);
        }

        // secondsToNice, GridUI.cs:899-920
        string secondsToNice(double seconds)
        {
            if (seconds < 0)
                return "Infinity Seconds";

            double secs = seconds % 60;
            int mins = (int)(seconds / 60) % 60;
            int hours = (int)(seconds / 3600);// % 24;

            if (hours > 0)
            {
                return hours + ":" + mins.ToString("00") + ":" + secs.ToString("00") + " Hours";
            }
            else if (mins > 0)
            {
                return mins + ":" + secs.ToString("00") + " Minutes";
            }
            else
            {
                return secs.ToString("0.00") + " Seconds";
            }
        }

        // calcpolygonarea, GridUI.cs:957-1011
        double calcpolygonarea(List<PointLatLngAlt> polygon)
        {
            if (polygon.Count == 0)
            {
                return 0;
            }

            // close the polygon
            if (polygon[0] != polygon[polygon.Count - 1])
                polygon.Add(polygon[0]); // make a full loop

            CoordinateTransformationFactory ctfac = new CoordinateTransformationFactory();

            IGeographicCoordinateSystem wgs84 = GeographicCoordinateSystem.WGS84;

            int utmzone = (int)((polygon[0].Lng - -186.0) / 6.0);

            IProjectedCoordinateSystem utm = ProjectedCoordinateSystem.WGS84_UTM(utmzone, polygon[0].Lat < 0 ? false : true);

            ICoordinateTransformation trans = ctfac.CreateFromCoordinateSystems(wgs84, utm);

            double prod1 = 0;
            double prod2 = 0;

            for (int a = 0; a < (polygon.Count - 1); a++)
            {
                double[] pll1 = { polygon[a].Lng, polygon[a].Lat };
                double[] pll2 = { polygon[a + 1].Lng, polygon[a + 1].Lat };

                double[] p1 = trans.MathTransform.Transform(pll1);
                double[] p2 = trans.MathTransform.Transform(pll2);

                prod1 += p1[0] * p2[1];
                prod2 += p1[1] * p2[0];
            }

            double answer = (prod1 - prod2) / 2;

            if (polygon[0] == polygon[polygon.Count - 1])
                polygon.RemoveAt(polygon.Count - 1); // unmake a full loop

            return Math.Abs(answer);
        }

        // getAngleOfLongestSide, GridUI.cs:1013-1033
        double getAngleOfLongestSide(List<PointLatLngAlt> list)
        {
            if (list.Count == 0)
                return 0;
            double angle = 0;
            double maxdist = 0;
            PointLatLngAlt last = list[list.Count - 1];
            foreach (var item in list)
            {
                if (item.GetDistance(last) > maxdist)
                {
                    angle = item.GetBearing(last);
                    maxdist = item.GetDistance(last);
                }
                last = item;
            }

            return (angle + 360) % 360;
        }

        // getFOV, GridUI.cs:1035-1054
        void getFOV(double flyalt, ref double fovh, ref double fovv)
        {
            double focallen = (double)NUM_focallength.Value;
            double sensorwidth = double.Parse(TXT_senswidth.Text);
            double sensorheight = double.Parse(TXT_sensheight.Text);

            // scale      mm / mm
            double flscale = (1000 * flyalt) / focallen;

            //   mm * mm / 1000
            double viewwidth = (sensorwidth * flscale / 1000);
            double viewheight = (sensorheight * flscale / 1000);

            float fovh1 = (float)(Math.Atan(sensorwidth / (2 * focallen)) * rad2deg * 2);
            float fovv1 = (float)(Math.Atan(sensorheight / (2 * focallen)) * rad2deg * 2);

            fovh = viewwidth;
            fovv = viewheight;
        }

        // getFOVangle, GridUI.cs:1056-1064
        void getFOVangle(ref double fovh, ref double fovv)
        {
            double focallen = (double)NUM_focallength.Value;
            double sensorwidth = double.Parse(TXT_senswidth.Text);
            double sensorheight = double.Parse(TXT_sensheight.Text);

            fovh = (float)(Math.Atan(sensorwidth / (2 * focallen)) * rad2deg * 2);
            fovv = (float)(Math.Atan(sensorheight / (2 * focallen)) * rad2deg * 2);
        }

        // doCalc, GridUI.cs:1066-1117. The two handlers it takes off and puts back are this
        // harness's own ValueChanged subscriptions.
        void doCalc()
        {
            try
            {
                // entered values
                float flyalt = (float)fromDistDisplayUnit((float)NUM_altitude.Value);
                int imagewidth = int.Parse(TXT_imgwidth.Text);
                int imageheight = int.Parse(TXT_imgheight.Text);

                int overlap = (int)num_overlap.Value;
                int sidelap = (int)num_sidelap.Value;

                double viewwidth = 0;
                double viewheight = 0;

                getFOV(flyalt, ref viewwidth, ref viewheight);

                TXT_fovH.Text = viewwidth.ToString("#.#");
                TXT_fovV.Text = viewheight.ToString("#.#");
                // Imperial
                feet_fovH = (viewwidth * 3.2808399f).ToString("#.#");
                feet_fovV = (viewheight * 3.2808399f).ToString("#.#");

                //    mm  / pixels * 100
                TXT_cmpixel.Text = ((viewheight / imageheight) * 100).ToString("0.00 cm");
                // Imperial
                inchpixel = (((viewheight / imageheight) * 100) * 0.393701).ToString("0.00 inches");

                spacingHandlersOn = false;

                if (CHK_camdirection.Checked)
                {
                    NUM_spacing.Value = (decimal)((1 - (overlap / 100.0f)) * viewheight);
                    NUM_Distance.Value = (decimal)((1 - (sidelap / 100.0f)) * viewwidth);
                }
                else
                {
                    NUM_spacing.Value = (decimal)((1 - (overlap / 100.0f)) * viewwidth);
                    NUM_Distance.Value = (decimal)((1 - (sidelap / 100.0f)) * viewheight);
                }
                spacingHandlersOn = true;
            }
            catch { return; }
        }

        // `NUM_spacing.ValueChanged -= / +=` and the same for NUM_Distance, GridUI.cs:1096-1112:
        // while doCalc sets them their change does not regenerate the grid. An exception between
        // the two leaves them off, as it leaves the C#'s handlers detached, until the next doCalc
        // gets through.
        public bool spacingHandlersOn = true;

        // CalcHeadingHold, GridUI.cs:1119-1146. By the time the awaited grid comes back the
        // NumericUpDown's Text is its new Value at DecimalPlaces 0 (UpdateEditText runs as the
        // ValueChanged handler returns at its first await).
        private void CalcHeadingHold()
        {
            int previous = (int)Math.Round(Convert.ToDecimal(NUM_angle.Value.ToString("F0")));
            int current = (int)Math.Round(NUM_angle.Value);

            int change = current - previous;

            if (change > 0) // Positive change
            {
                int val = Convert.ToInt32(TXT_headinghold.Text) + change;
                if (val > 359)
                {
                    val = val - 360;
                }
                TXT_headinghold.Text = val.ToString();
            }

            if (change < 0) // Negative change
            {
                int val = Convert.ToInt32(TXT_headinghold.Text) + change;
                if (val < 0)
                {
                    val = val + 360;
                }
                TXT_headinghold.Text = val.ToString();
            }
        }

        // CMB_camera_SelectedIndexChanged, GridUI.cs:1316-1333
        public void CMB_camera_SelectedIndexChanged()
        {
            if (cameras.ContainsKey(CMB_camera_Text))
            {
                camerainfo camera = cameras[CMB_camera_Text];

                NUM_focallength.Value = (decimal)camera.focallen;
                TXT_imgheight.Text = camera.imageheight.ToString();
                TXT_imgwidth.Text = camera.imagewidth.ToString();
                TXT_sensheight.Text = camera.sensorheight.ToString();
                TXT_senswidth.Text = camera.sensorwidth.ToString();
            }

            domainUpDown1_ValueChanged();
        }

        // CHK_copter_headinghold_CheckedChanged, GridUI.cs:1365-1382: enabling only.
        void CHK_copter_headinghold_CheckedChanged() { }

        // CHK_copter_headingholdlock_CheckedChanged, GridUI.cs:1384-1395
        void CHK_copter_headingholdlock_CheckedChanged()
        {
            if (CHK_copter_headingholdlock.Checked)
            {
            }
            else
            {
                TXT_headinghold.Text = Decimal.Round(NUM_angle.Value).ToString();
            }
        }

        // CMB_startfrom_SelectedIndexChanged, GridUI.cs:1917-1933, with `point` typed.
        public void CMB_startfrom_SelectedIndexChanged()
        {
            if (loading)
                return;

            if (CMB_startfrom_Text == Grid.StartPosition.Point.ToString())
            {
                int pnt = point;

                if (list.Count > pnt)
                    Grid.StartPointLatLngAlt = list[pnt - 1];
            }

            domainUpDown1_ValueChanged();
        }

        // BUT_Accept_Click, GridUI.cs:1595-1880, less the form's closing and the map.
        public void BUT_Accept_Click()
        {
            if (grid != null && grid.Count > 0)
            {
                if (NUM_split.Value > 1 && CHK_toandland.Checked != true)
                {
                    acceptMessage = "You must use Land/RTL to split a mission";
                    return;
                }

                object gridobject = null;

                int wpsplit = (int)Math.Round(grid.Count / NUM_split.Value, MidpointRounding.AwayFromZero);

                List<int> wpsplitstart = new List<int>();

                for (int splitno = 0; splitno < NUM_split.Value; splitno++)
                {
                    int wpstart = wpsplit * splitno;
                    int wpend = wpsplit * (splitno + 1);

                    while (wpstart != 0 && wpstart < grid.Count && grid[wpstart].Tag != "E")
                    {
                        wpstart--;
                    }

                    while (wpend > 0 && wpend < grid.Count && grid[wpend].Tag != "S")
                    {
                        wpend--;
                    }

                    if (CHK_toandland.Checked)
                    {
                        if (firmware == "ArduCopter2")
                        {
                            var wpno = AddWPtoList(TAKEOFF, 20, 0, 0, 0, 0, 0,
                                (int)(30 * 1f), gridobject);

                            wpsplitstart.Add(wpno);
                        }
                        else
                        {
                            var wpno = AddWPtoList(TAKEOFF, 20, 0, 0, 0, 0, 0,
                                (int)(30 * 1f), gridobject);

                            wpsplitstart.Add(wpno);
                        }
                    }

                    if (CHK_usespeed.Checked)
                    {
                        AddWPtoList(DO_CHANGE_SPEED, 0,
                            ((float)NUM_UpDownFlySpeed.Value / 1f), 0, 0, 0, 0, 0,
                            gridobject);
                    }

                    int i = 0;
                    bool startedtrigdist = false;
                    PointLatLngAlt lastplla = PointLatLngAlt.Zero;
                    foreach (var plla in grid)
                    {
                        // skip before start point
                        if (i < wpstart)
                        {
                            i++;
                            continue;
                        }
                        // skip after endpoint
                        if (i >= wpend)
                            break;
                        if (i > wpstart)
                        {
                            // internal point check
                            if (plla.Tag == "M")
                            {
                                if (rad_repeatservo.Checked)
                                {
                                    if (!chk_stopstart.Checked)
                                    {
                                        AddWP(plla.Lng, plla.Lat, plla.Alt, plla.Tag);
                                        AddWPtoList(DO_REPEAT_SERVO,
                                            (float)NUM_reptservo.Value,
                                            (float)num_reptpwm.Value, 1, (float)NUM_repttime.Value, 0, 0, 0,
                                            gridobject);
                                    }
                                }
                                if (rad_digicam.Checked)
                                {
                                    AddWP(plla.Lng, plla.Lat, plla.Alt, plla.Tag);
                                    AddWPtoList(DO_DIGICAM_CONTROL, 1, 0, 0, 0, 0, 1, 0,
                                        gridobject);
                                }
                            }
                            else
                            {
                                // only add points that are ends
                                if (plla.Tag == "S" || plla.Tag == "E")
                                {
                                    if (plla.Lat != lastplla.Lat || plla.Lng != lastplla.Lng ||
                                        plla.Alt != lastplla.Alt)
                                        AddWP(plla.Lng, plla.Lat, plla.Alt, plla.Tag);
                                }

                                // check trigger method
                                if (rad_trigdist.Checked)
                                {
                                    // if stopstart enabled, add wp and trigger start/stop
                                    if (chk_stopstart.Checked)
                                    {
                                        if (plla.Tag == "SM")
                                        {
                                            //  s > sm, need to dup check
                                            if (plla.Lat != lastplla.Lat || plla.Lng != lastplla.Lng ||
                                                plla.Alt != lastplla.Alt)
                                                AddWP(plla.Lng, plla.Lat, plla.Alt, plla.Tag);

                                            AddWPtoList(DO_SET_CAM_TRIGG_DIST,
                                                (float)NUM_spacing.Value,
                                                0, 1, 0, 0, 0, 0, gridobject);
                                        }
                                        else if (plla.Tag == "ME")
                                        {
                                            AddWP(plla.Lng, plla.Lat, plla.Alt, plla.Tag);

                                            AddWPtoList(DO_SET_CAM_TRIGG_DIST, 0, 0, 1, 0,
                                                0, 0, 0, gridobject);
                                        }
                                    }
                                    else
                                    {
                                        // add single start trigger
                                        if (!startedtrigdist)
                                        {
                                            AddWPtoList(DO_SET_CAM_TRIGG_DIST,
                                                (float)NUM_spacing.Value,
                                                0, 1, 0, 0, 0, 0, gridobject);
                                            startedtrigdist = true;
                                        }
                                        else if (plla.Tag == "ME")
                                        {
                                            AddWP(plla.Lng, plla.Lat, plla.Alt, plla.Tag);
                                        }
                                    }
                                }
                                else if (rad_repeatservo.Checked)
                                {
                                    if (chk_stopstart.Checked)
                                    {
                                        if (plla.Tag == "SM")
                                        {
                                            if (plla.Lat != lastplla.Lat || plla.Lng != lastplla.Lng ||
                                                plla.Alt != lastplla.Alt)
                                                AddWP(plla.Lng, plla.Lat, plla.Alt, plla.Tag);

                                            AddWPtoList(DO_REPEAT_SERVO,
                                                (float)NUM_reptservo.Value,
                                                (float)num_reptpwm.Value, 999, (float)NUM_repttime.Value, 0, 0, 0,
                                                gridobject);
                                        }
                                        else if (plla.Tag == "ME")
                                        {
                                            AddWP(plla.Lng, plla.Lat, plla.Alt, plla.Tag);

                                            AddWPtoList(DO_REPEAT_SERVO,
                                                (float)NUM_reptservo.Value,
                                                (float)num_reptpwm.Value, 0, (float)NUM_repttime.Value, 0, 0, 0,
                                                gridobject);
                                        }
                                    }
                                }
                                else if (rad_do_set_servo.Checked)
                                {
                                    if (plla.Tag == "SM")
                                    {
                                        if (plla.Lat != lastplla.Lat || plla.Lng != lastplla.Lng ||
                                            plla.Alt != lastplla.Alt)
                                            AddWP(plla.Lng, plla.Lat, plla.Alt, plla.Tag);

                                        AddWPtoList(DO_SET_SERVO,
                                            (float)num_setservono.Value,
                                            (float)num_setservolow.Value, 0, 0, 0, 0, 0,
                                            gridobject);
                                    }
                                    else if (plla.Tag == "ME")
                                    {
                                        AddWP(plla.Lng, plla.Lat, plla.Alt, plla.Tag);

                                        AddWPtoList(DO_SET_SERVO,
                                            (float)num_setservono.Value,
                                            (float)num_setservohigh.Value, 0, 0, 0, 0, 0,
                                            gridobject);
                                    }
                                }
                            }
                        }
                        else
                        {
                            AddWP(plla.Lng, plla.Lat, plla.Alt, plla.Tag, gridobject);
                        }
                        lastplla = plla;
                        ++i;
                    }

                    // end
                    if (rad_trigdist.Checked)
                    {
                        AddWPtoList(DO_SET_CAM_TRIGG_DIST, 0, 0, 1, 0, 0, 0, 0, gridobject);
                    }

                    if (CHK_usespeed.Checked)
                    {
                        double speed = 0;
                        if (WPNAV_SPEED != null)
                        {
                            speed = WPNAV_SPEED.Value / 100;
                        }
                        else if (WP_SPD != null)
                        {
                            speed = WP_SPD.Value;
                        }
                        if (speed > 0)
                            AddWPtoList(DO_CHANGE_SPEED, 0, speed, 0, 0, 0, 0, 0, gridobject);
                    }

                    if (CHK_toandland.Checked)
                    {
                        if (CHK_toandland_RTL.Checked)
                        {
                            AddWPtoList(RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, gridobject);
                        }
                        else
                        {
                            AddWPtoList(LAND, 0, 0, 0, 0, PlannedHomeLocation.Lng,
                                PlannedHomeLocation.Lat, 0, gridobject);
                        }
                    }
                }

                if (NUM_split.Value > 1)
                {
                    int index = 0;
                    foreach (var i in wpsplitstart)
                    {
                        // add do jump
                        InsertWP(index, DO_JUMP, i + wpsplitstart.Count + 1, 1, 0, 0, 0, 0, 0, gridobject);
                        index++;
                    }
                }

                // save camera fov's for use with footprints
                double fovha = 0;
                double fovva = 0;
                try
                {
                    getFOVangle(ref fovha, ref fovva);

                    if (CHK_camdirection.Checked)
                    {
                        config["camera_fovh"] = fovha.ToString();
                        config["camera_fovv"] = fovva.ToString();
                    }
                    else
                    {
                        config["camera_fovh"] = fovva.ToString();
                        config["camera_fovv"] = fovha.ToString();
                    }
                }
                catch (Exception)
                {
                }

                savesettings();
            }
            else
            {
                acceptMessage = "Bad Grid";
            }
        }

        // savesettings, GridUI.cs:450-510
        void savesettings()
        {
            config["grid_camera"] = CMB_camera_Text;
            config["grid_alt"] = NUM_altitude.Value.ToString();
            config["grid_angle"] = NUM_angle.Value.ToString();
            config["grid_camdir"] = CHK_camdirection.Checked.ToString();

            config["grid_usespeed"] = CHK_usespeed.Checked.ToString();
            config["grid_speed"] = NUM_UpDownFlySpeed.Value.ToString();

            config["grid_dist"] = NUM_Distance.Value.ToString();
            config["grid_overshoot1"] = NUM_overshoot.Value.ToString();
            config["grid_overshoot2"] = NUM_overshoot2.Value.ToString();
            config["grid_leadin1"] = NUM_leadin.Value.ToString();
            config["grid_leadin2"] = NUM_leadin2.Value.ToString();
            config["grid_overlap"] = num_overlap.Value.ToString();
            config["grid_sidelap"] = num_sidelap.Value.ToString();
            config["grid_spacing"] = NUM_spacing.Value.ToString();
            config["grid_crossgrid"] = chk_crossgrid.Checked.ToString();
            config["grid_spiral"] = chk_spiral.Checked.ToString();

            config["grid_startfrom"] = CMB_startfrom_Text;

            config["grid_autotakeoff"] = CHK_toandland.Checked.ToString();
            config["grid_autotakeoff_RTL"] = CHK_toandland_RTL.Checked.ToString();

            config["grid_internals"] = CHK_internals.Checked.ToString();
            config["grid_footprints"] = CHK_footprints.Checked.ToString();
            config["grid_advanced"] = CHK_advanced.Checked.ToString();

            config["grid_trigdist"] = rad_trigdist.Checked.ToString();
            config["grid_digicam"] = rad_digicam.Checked.ToString();
            config["grid_repeatservo"] = rad_repeatservo.Checked.ToString();
            config["grid_breakstopstart"] = chk_stopstart.Checked.ToString();

            // Copter Settings
            config["grid_copter_spline"] = chk_spline.Checked.ToString();
            config["grid_copter_delay"] = NUM_copter_delay.Value.ToString();
            config["grid_copter_headinghold_chk"] = CHK_copter_headinghold.Checked.ToString();

            // Plane Settings
            config["grid_min_lane_separation"] = NUM_Lane_Dist.Value.ToString();

            // Spiral Settings
            config["grid_clockwise_laps"] = NUM_clockwise_laps.Value.ToString();
            config["grid_laps"] = NUM_laps.Value.ToString();
            config["grid_match_spiral_perimeter"] = CHK_match_spiral_perimeter.Checked.ToString();
        }

        // What the operator does to one control, with the events it raises.
        public void Apply(string key, string value)
        {
            foreach (var n in Nums())
            {
                if (n.Name == key)
                {
                    n.Type(value);
                    return;
                }
            }
            var chks = Chks();
            if (chks.ContainsKey(key))
            {
                var chk = chks[key];
                bool want = bool.Parse(value);
                if (key.StartsWith("rad_"))
                {
                    // A click on a radio button checks it and clears the others in groupBox3.
                    if (!want)
                        throw new FormatException("a radio button is clicked on, not off: " + key);
                    foreach (var other in new[] { rad_trigdist, rad_digicam, rad_repeatservo, rad_do_set_servo })
                        if (other != chk)
                            other.Checked = false;
                    chk.Checked = true;
                    chk.OnClick();
                    return;
                }
                if (chk.Checked != want)
                {
                    chk.Checked = want;
                    chk.OnClick();
                }
                return;
            }
            var txts = Txts();
            if (txts.ContainsKey(key))
            {
                txts[key].Text = value;
                return;
            }
            switch (key)
            {
                case "CMB_camera":
                    if (!CMB_camera_Items.Contains(value))
                        throw new FormatException("no camera " + value);
                    if (CMB_camera_Text != value)
                    {
                        CMB_camera_Text = value;
                        CMB_camera_SelectedIndexChanged();
                    }
                    return;
                case "CMB_startfrom":
                    if (CMB_startfrom_Text != value)
                    {
                        CMB_startfrom_Text = ((Grid.StartPosition)Enum.Parse(typeof(Grid.StartPosition), value)).ToString();
                        CMB_startfrom_SelectedIndexChanged();
                    }
                    return;
                case "point":
                    point = int.Parse(value, Inv);
                    return;
            }
            throw new FormatException("unknown control " + key);
        }
    }

    // Reads the case's context and runs it.
    public static void RunCase(string[] words, List<PointLatLngAlt> polygon, string where, string outPath,
        string builtinCameras)
    {
        var dialog = new Dialog();
        var sets = new List<KeyValuePair<string, string>>();
        // The planner's side first: it is there before the dialog opens.
        for (int i = 3; i < words.Length; i++)
        {
            int eq = words[i].IndexOf('=');
            if (eq <= 0)
                throw new FormatException(where + ": expected control=value, got " + words[i]);
            string key = words[i].Substring(0, eq);
            string value = words[i].Substring(eq + 1).Replace("%20", " ");
            switch (key)
            {
                case "HomeLocation":
                    {
                        var parts = value.Split(',');
                        dialog.PlannedHomeLocation = new PointLatLngAlt(double.Parse(parts[0], Inv),
                            double.Parse(parts[1], Inv), parts.Length > 2 ? double.Parse(parts[2], Inv) : 0);
                        break;
                    }
                case "firmware": dialog.firmware = value; break;
                case "rows":
                    dialog.rowsBefore = int.Parse(value, Inv);
                    for (int r = 0; r < dialog.rowsBefore; r++)
                        dialog.rows.Add(-1);
                    break;
                case "WPNAV_SPEED": dialog.WPNAV_SPEED = double.Parse(value, Inv); break;
                case "WP_SPD": dialog.WP_SPD = double.Parse(value, Inv); break;
                default: sets.Add(new KeyValuePair<string, string>(key, value)); break;
            }
        }

        // A static CreateGrid reads for StartPosition.Point, which the dialog sets; each case starts
        // from the value a fresh Mission Planner has.
        Grid.StartPointLatLngAlt = PointLatLngAlt.Zero;

        dialog.Wire();
        dialog.Construct(polygon, builtinCameras);
        dialog.Load();
        foreach (var set in sets)
            dialog.Apply(set.Key, set.Value);
        dialog.BUT_Accept_Click();

        var sb = new StringBuilder();
        sb.Append("# GridUI (Grid/GridUI.cs) from tools/csharp-reference/regen-grid.sh - do not edit\n");
        Row(sb, "case", words[1]);
        foreach (var p in polygon)
            Row(sb, "vertex", D(p.Lat), D(p.Lng));
        Row(sb, "HomeLocation", D(dialog.PlannedHomeLocation.Lat), D(dialog.PlannedHomeLocation.Lng),
            D(dialog.PlannedHomeLocation.Alt));
        Row(sb, "firmware", dialog.firmware);
        Row(sb, "rows", dialog.rowsBefore.ToString(Inv));
        Row(sb, "WPNAV_SPEED", dialog.WPNAV_SPEED.HasValue ? D(dialog.WPNAV_SPEED.Value) : "none");
        Row(sb, "WP_SPD", dialog.WP_SPD.HasValue ? D(dialog.WP_SPD.Value) : "none");
        foreach (var set in sets)
            Row(sb, "set", set.Key, Esc(set.Value));
        foreach (var n in dialog.Nums())
            Row(sb, "control", n.Name, n.Value.ToString(Inv));
        foreach (var chk in dialog.Chks())
            Row(sb, "control", chk.Key, chk.Value.Checked.ToString());
        Row(sb, "control", "CMB_camera", Esc(dialog.CMB_camera_Text));
        Row(sb, "control", "CMB_startfrom", dialog.CMB_startfrom_Text);
        foreach (var txt in dialog.Txts())
            Row(sb, "control", txt.Key, Esc(txt.Value.Text));
        Row(sb, "stat", "lbl_area", Esc(dialog.lbl_area));
        Row(sb, "stat", "lbl_distance", Esc(dialog.lbl_distance));
        Row(sb, "stat", "lbl_spacing", Esc(dialog.lbl_spacing));
        Row(sb, "stat", "lbl_grndres", Esc(dialog.lbl_grndres));
        Row(sb, "stat", "lbl_pictures", Esc(dialog.lbl_pictures));
        Row(sb, "stat", "lbl_strips", Esc(dialog.lbl_strips));
        Row(sb, "stat", "lbl_footprint", Esc(dialog.lbl_footprint));
        Row(sb, "stat", "lbl_distbetweenlines", Esc(dialog.lbl_distbetweenlines));
        Row(sb, "stat", "lbl_flighttime", Esc(dialog.lbl_flighttime));
        Row(sb, "stat", "lbl_photoevery", Esc(dialog.lbl_photoevery));
        Row(sb, "stat", "lbl_turnrad", Esc(dialog.lbl_turnrad));
        Row(sb, "stat", "lbl_gndelev", Esc(dialog.lbl_gndelev));
        Row(sb, "stat", "lbl_minshutter", Esc(dialog.lbl_minshutter));
        Row(sb, "recomputes", dialog.recomputes.ToString(Inv));
        Row(sb, "StartPointLatLngAlt", D(Grid.StartPointLatLngAlt.Lat), D(Grid.StartPointLatLngAlt.Lng));
        Row(sb, "spacing_handlers", dialog.spacingHandlersOn.ToString());
        // The grid's own points are Grid.CreateGrid's, which the grid, corridor and rotary goldens
        // hold point by point; here the count, and the tags in order, run-length coded.
        var grid = dialog.grid ?? new List<PointLatLngAlt>();
        Row(sb, "points", grid.Count.ToString(Inv));
        var tags = new StringBuilder();
        for (int i = 0; i < grid.Count; )
        {
            int j = i;
            while (j < grid.Count && grid[j].Tag == grid[i].Tag)
                j++;
            if (tags.Length > 0)
                tags.Append(' ');
            tags.Append(grid[i].Tag).Append('*').Append((j - i).ToString(Inv));
            i = j;
        }
        Row(sb, "tags", tags.ToString());
        Row(sb, "accept", dialog.acceptMessage == null ? "ok" : Esc(dialog.acceptMessage));
        foreach (var call in dialog.calls)
            sb.Append("call,").Append(call).Append('\n');
        Row(sb, "rows_added", (dialog.rows.Count - dialog.rowsBefore).ToString(Inv));
        Row(sb, "commands", string.Join(" ", dialog.rows.Select(r => r.ToString(Inv)).ToArray()));
        foreach (var kv in dialog.config)
            Row(sb, "setting", kv.Key, Esc(kv.Value));
        File.WriteAllText(outPath, sb.ToString(), new UTF8Encoding(false));
    }

    // The camera list xmlcamera builds from camerasBuiltin.xml, in the order CMB_camera lists it.
    public static void WriteCameras(string builtinCameras, string outPath)
    {
        var dialog = new Dialog();
        dialog.Construct(new List<PointLatLngAlt>(), builtinCameras);
        var sb = new StringBuilder();
        sb.Append("# GridUI.xmlcamera over camerasBuiltin.xml from tools/csharp-reference/regen-grid.sh - do not edit\n");
        foreach (var name in dialog.CMB_camera_Items)
        {
            var c = dialog.cameras[name];
            // The floats, then what CMB_camera_SelectedIndexChanged makes of them: the focal length
            // as (decimal)float, and the four text boxes' float.ToString().
            Row(sb, "camera", Esc(name), F(c.focallen), F(c.imagewidth), F(c.imageheight),
                F(c.sensorwidth), F(c.sensorheight), ((decimal)c.focallen).ToString(Inv),
                c.imagewidth.ToString(), c.imageheight.ToString(), c.sensorwidth.ToString(),
                c.sensorheight.ToString());
        }
        File.WriteAllText(outPath, sb.ToString(), new UTF8Encoding(false));
    }

    // A label's text in a golden line: commas would split it, so they are %2C, and spaces %20.
    static string Esc(string s) { return s.Replace("%", "%25").Replace(",", "%2C").Replace(" ", "%20"); }

    static void Row(StringBuilder sb, string key, params string[] values)
    {
        sb.Append(key);
        foreach (var v in values)
            sb.Append(',').Append(v);
        sb.Append('\n');
    }

    static string D(double d) { return d.ToString("G17", Inv); }

    static string F(float f) { return f.ToString("G9", Inv); }
}
