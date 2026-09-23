// Headless survey-grid oracle: the `grid` verb of PLAN.md §7.1, for §13.3 item 3 and D11.
//
// Runs Mission Planner's own `MissionPlanner.Utilities.Grid.CreateGrid` (ExtLibs/Utilities/Grid.cs)
// over the cases in testdata/grid/cases.txt and writes exactly what it returns, so
// crates/mp-mission/tests/grid_vectors.rs holds the Rust port to the C# rather than to our reading
// of it. The assembly is built from the pinned source tree by regen-grid.sh, not taken from a
// binary distribution. Console-only: Grid.cs, utmpos.cs and PointLatLngAlt.cs never touch WinForms.
//
// Build:  see regen-grid.sh (mcs against the msbuild output of ExtLibs/Utilities)
// Run:    mono MpGrid.exe grid <cases.txt> <outdir>
//
// cases.txt, one directive per line, `#` starts a comment:
//   polygon <name> <lat>,<lng> <lat>,<lng> ...
//   case <name> <polygon> [<parameter>=<value> ...]
// Parameters are named as CreateGrid names them (Grid.cs:354). One that is left out takes the value
// GridUI gives it on a fresh install, so a case reads as "what the user gets unless they change X":
//   altitude            100      NUM_altitude.Value, GridUI.Designer.cs:1333 (metres display unit)
//   distance            50       NUM_Distance.Value, GridUI.Designer.cs:1149
//   spacing             0        NUM_spacing has no Value in GridUI.Designer.cs:1164, so 0
//   angle               longest  GridUI.cs:104, getAngleOfLongestSide (GridUI.cs:1015) via decimal
//   overshoot1          0        NUM_overshoot, no Value in the designer
//   overshoot2          0        NUM_overshoot2, likewise
//   startpos            Home     CMB_startfrom.SelectedIndex = 0, GridUI.cs:101
//   shutter             False    the literal GridUI.cs:614 passes
//   minLaneSeparation   0        NUM_Lane_Dist, no Value in the designer
//   leadin1             0        NUM_leadin, likewise
//   leadin2             0        NUM_leadin2, likewise
//   HomeLocation        0,0      PlannedHomeLocation before a home is planned, CurrentState.cs:41
//   useextendedendpoint False    chk_optimize_for_distance is unchecked in the designer
//   StartPointLatLngAlt 0,0      Grid.StartPointLatLngAlt, Grid.cs:34; read only for startpos=Point
//
// Output, one <outdir>/<case>.csv per case, every line `key,value[,value...]`: the case name, every
// polygon vertex and every argument exactly as CreateGrid received it (so a default the harness
// resolved is recorded, not re-derived), then one `wp,lat,lng,alt,tag` line per returned point.
// Doubles are G17 and floats G9, invariant culture: both always round-trip, which "R" does not on
// every runtime.

using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using MissionPlanner.Utilities;

public static class MpGrid
{
    static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    public static int Main(string[] args)
    {
        // Parsing and formatting must not depend on the machine's locale.
        System.Threading.Thread.CurrentThread.CurrentCulture = Inv;
        if (args.Length != 3 || args[0] != "grid")
        {
            Console.Error.WriteLine("usage: MpGrid grid <cases.txt> <outdir>");
            return 2;
        }
        try
        {
            return RunGrid(args[1], args[2]);
        }
        catch (Exception ex)
        {
            // No partial golden data: a case that cannot be run fails the whole regeneration.
            Console.Error.WriteLine("MpGrid: " + ex);
            return 1;
        }
    }

    static int RunGrid(string casesPath, string outDir)
    {
        var polygons = new Dictionary<string, List<PointLatLngAlt>>();
        var names = new HashSet<string>();
        int count = 0;
        int lineNo = 0;
        foreach (var raw in File.ReadAllLines(casesPath))
        {
            lineNo++;
            var line = raw;
            int hash = line.IndexOf('#');
            if (hash >= 0)
                line = line.Substring(0, hash);
            var words = line.Split(new[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
            if (words.Length == 0)
                continue;
            string where = casesPath + ":" + lineNo;

            if (words[0] == "polygon")
            {
                if (words.Length < 3)
                    throw new FormatException(where + ": polygon needs a name and vertices");
                var poly = new List<PointLatLngAlt>();
                for (int i = 2; i < words.Length; i++)
                    poly.Add(ParseLatLng(words[i], where));
                if (polygons.ContainsKey(words[1]))
                    throw new FormatException(where + ": polygon " + words[1] + " defined twice");
                polygons[words[1]] = poly;
            }
            else if (words[0] == "case")
            {
                if (words.Length < 3)
                    throw new FormatException(where + ": case needs a name and a polygon");
                if (!names.Add(words[1]))
                    throw new FormatException(where + ": case " + words[1] + " defined twice");
                List<PointLatLngAlt> poly;
                if (!polygons.TryGetValue(words[2], out poly))
                    throw new FormatException(where + ": unknown polygon " + words[2]);
                RunCase(words, poly, where, Path.Combine(outDir, words[1] + ".csv"));
                count++;
            }
            else
            {
                throw new FormatException(where + ": unknown directive " + words[0]);
            }
        }
        Console.Error.WriteLine("MpGrid: wrote " + count + " cases to " + outDir);
        return 0;
    }

    static void RunCase(string[] words, List<PointLatLngAlt> polygon, string where, string outPath)
    {
        // GridUI's defaults, see the header. The angle default needs the polygon, so it is resolved
        // after the overrides are read.
        double altitude = 100;
        double distance = 50;
        double spacing = 0;
        double? angle = null;
        double overshoot1 = 0;
        double overshoot2 = 0;
        var startpos = Grid.StartPosition.Home;
        bool shutter = false;
        float minLaneSeparation = 0;
        float leadin1 = 0;
        float leadin2 = 0;
        var home = new PointLatLngAlt();
        bool useextendedendpoint = false;
        var startPoint = PointLatLngAlt.Zero;

        for (int i = 3; i < words.Length; i++)
        {
            int eq = words[i].IndexOf('=');
            if (eq <= 0)
                throw new FormatException(where + ": expected parameter=value, got " + words[i]);
            string key = words[i].Substring(0, eq);
            string value = words[i].Substring(eq + 1);
            switch (key)
            {
                case "altitude": altitude = double.Parse(value, Inv); break;
                case "distance": distance = double.Parse(value, Inv); break;
                case "spacing": spacing = double.Parse(value, Inv); break;
                case "angle": angle = double.Parse(value, Inv); break;
                case "overshoot1": overshoot1 = double.Parse(value, Inv); break;
                case "overshoot2": overshoot2 = double.Parse(value, Inv); break;
                case "startpos":
                    startpos = (Grid.StartPosition)Enum.Parse(typeof(Grid.StartPosition), value);
                    break;
                case "shutter": shutter = bool.Parse(value); break;
                case "minLaneSeparation": minLaneSeparation = float.Parse(value, Inv); break;
                case "leadin1": leadin1 = float.Parse(value, Inv); break;
                case "leadin2": leadin2 = float.Parse(value, Inv); break;
                case "HomeLocation": home = ParseLatLng(value, where); break;
                case "useextendedendpoint": useextendedendpoint = bool.Parse(value); break;
                case "StartPointLatLngAlt": startPoint = ParseLatLng(value, where); break;
                default: throw new FormatException(where + ": unknown parameter " + key);
            }
        }

        // GridUI.cs:104 stores the angle in a NumericUpDown, so it passes through decimal on its way
        // to CreateGrid (GridUI.cs:611).
        double resolvedAngle = angle ?? (double)(decimal)((GetAngleOfLongestSide(polygon) + 360) % 360);

        // A static that CreateGrid reads for startpos=Point (Grid.cs:580). It persists between
        // calls, so every case sets it rather than inheriting the previous case's.
        Grid.StartPointLatLngAlt = startPoint;

        // CreateGrid mutates nothing it is given, but hand it a copy so that stays true of this
        // harness whatever a later Grid.cs does.
        var input = new List<PointLatLngAlt>();
        polygon.ForEach(p => input.Add(new PointLatLngAlt(p)));

        List<PointLatLngAlt> result = Grid.CreateGrid(input, altitude, distance, spacing, resolvedAngle,
            overshoot1, overshoot2, startpos, shutter, minLaneSeparation, leadin1, leadin2, home,
            useextendedendpoint);

        var sb = new StringBuilder();
        sb.Append("# Grid.CreateGrid output from tools/csharp-reference/regen-grid.sh - do not edit\n");
        Row(sb, "case", words[1]);
        foreach (var p in polygon)
            Row(sb, "vertex", D(p.Lat), D(p.Lng));
        Row(sb, "altitude", D(altitude));
        Row(sb, "distance", D(distance));
        Row(sb, "spacing", D(spacing));
        Row(sb, "angle", D(resolvedAngle));
        Row(sb, "overshoot1", D(overshoot1));
        Row(sb, "overshoot2", D(overshoot2));
        Row(sb, "startpos", startpos.ToString());
        Row(sb, "shutter", shutter.ToString());
        Row(sb, "minLaneSeparation", F(minLaneSeparation));
        Row(sb, "leadin1", F(leadin1));
        Row(sb, "leadin2", F(leadin2));
        Row(sb, "HomeLocation", D(home.Lat), D(home.Lng));
        Row(sb, "useextendedendpoint", useextendedendpoint.ToString());
        Row(sb, "StartPointLatLngAlt", D(startPoint.Lat), D(startPoint.Lng));
        Row(sb, "points", result.Count.ToString(Inv));
        foreach (var p in result)
            Row(sb, "wp", D(p.Lat), D(p.Lng), D(p.Alt), p.Tag);

        // Explicit "\n" and no BOM, so the file is byte-identical whichever platform regenerates it.
        File.WriteAllText(outPath, sb.ToString(), new UTF8Encoding(false));
    }

    // GridUI.cs:1015-1033, verbatim apart from being static: the bearing of the polygon's longest
    // side, which the dialog proposes as the initial grid angle.
    static double GetAngleOfLongestSide(List<PointLatLngAlt> list)
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

    static PointLatLngAlt ParseLatLng(string text, string where)
    {
        var parts = text.Split(',');
        if (parts.Length != 2)
            throw new FormatException(where + ": expected lat,lng, got " + text);
        return new PointLatLngAlt(double.Parse(parts[0], Inv), double.Parse(parts[1], Inv));
    }

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
