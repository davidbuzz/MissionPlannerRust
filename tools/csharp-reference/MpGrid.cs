// Copyright (C) 2026 David "Buzz" Bussenschutt
//
// This file is part of MissionPlannerRust, a Rust implementation derived from
// Mission Planner (Copyright (C) 2010-2024 Michael Oborne and contributors,
// https://github.com/ArduPilot/MissionPlanner); NOTICE records the changes.
//
// MissionPlannerRust is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by the
// Free Software Foundation, version 3 of the License.
//
// MissionPlannerRust is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License along with
// MissionPlannerRust. If not, see <https://www.gnu.org/licenses/>.
//
// SPDX-License-Identifier: GPL-3.0-only

// Not taken from Mission Planner: this file is MissionPlannerRust's own, and carries no line of
// Mission Planner's source. It is temporary plumbing - it runs Mission Planner's compiled code
// under mono and records what it does, so the port's completeness and fidelity can be checked
// against the original - and it goes when the port is complete.

// Headless survey-grid oracle: the `grid`, `corridor` and `rotary` verbs of PLAN.md §7.1, for §13.3
// item 3, §13.4 item 8 and Deliverable 11.
//
// Runs Mission Planner's own generators in `MissionPlanner.Utilities.Grid` (ExtLibs/Utilities/Grid.cs)
// - CreateGrid, CreateCorridor and CreateRotary - over the cases in testdata/grid/cases.txt and
// writes exactly what they return, so crates/mp-mission/tests/grid_vectors.rs,
// corridor_vectors.rs and rotary_vectors.rs hold the Rust port to the C# rather than to our reading
// of it. The assembly is built from the pinned source tree by regen-grid.sh, not taken from a binary
// distribution. Console-only: Grid.cs, clipper.cs, utmpos.cs and PointLatLngAlt.cs never touch
// WinForms.
//
// Build:  see regen-grid.sh (mcs against the msbuild output of ExtLibs/Utilities)
// Run:    mono MpGrid.exe <verb> <cases.txt> <outdir>      <verb> is grid, corridor, rotary or offset
//
// cases.txt, one directive per line, `#` starts a comment:
//   polygon <name> <lat>,<lng> <lat>,<lng> ...
//   case <name> <polygon> [<parameter>=<value> ...]        run by the grid verb
//   corridor <name> <polygon> [<parameter>=<value> ...]    run by the corridor verb
//   rotary <name> <polygon> [<parameter>=<value> ...]      run by the rotary verb
//   path <name> <x>,<y> <x>,<y> ...                        ClipperLib.IntPoint coordinates
//   offset <name> <path>[+<path>...] <delta>[/<delta>...]  run by the offset verb
// Each verb runs its own directive and passes over the others; a name is a case of one verb only.
// `accept` lines are the Survey (Grid) dialog's cases: their oracle, which was GridUI.cs's own code
// re-hosted without its form, was deleted on 2026-10-03 (this repository carries no line of Mission
// Planner's source), so every verb passes over them and testdata/grid/golden/accept stands as it
// wrote it at efb0801.
//
// The offset verb is ClipperLib itself (ExtLibs/Utilities/clipper.cs), the offset CreateRotary
// insets with (Grid.cs:248-257): one ClipperOffset built as Grid.cs builds it, every path added
// with JoinType.jtMiter and EndType.etClosedPolygon, then Execute(ref PolyTree, delta) once per
// delta in turn on the same object. Its paths are integers chosen to be exactly collinear and to
// touch, which latitudes and longitudes projected to millimetres almost never are, so it reaches
// the joins, splits and hole fixups of the union that the rotary cases leave alone. Output: the
// arguments, then per delta an `execute,<delta>,<top-level count>` line and one
// `node,<depth>,<x>,<y>,...` line per PolyTree node, depth first in PolyTreeToPaths order
// (clipper.cs:4400-4424).
// Parameters are named as the generator names them (CreateGrid Grid.cs:354, CreateCorridor
// Grid.cs:55, CreateRotary Grid.cs:196). One that is left out takes the value GridUI gives it on a
// fresh install, so a case reads as "what the user gets unless they change X":
//   altitude            100      NUM_altitude.Value, GridUI.Designer.cs:1333 (metres display unit)
//   distance            50       NUM_Distance.Value, GridUI.Designer.cs:1149
//   spacing             0        NUM_spacing has no Value in GridUI.Designer.cs:1164, so 0
//   angle               longest  GridUI.cs:104, getAngleOfLongestSide (GridUI.cs:1015) via decimal
//   overshoot1          0        NUM_overshoot, no Value in the designer
//   overshoot2          0        NUM_overshoot2, likewise
//   startpos            Home     CMB_startfrom.SelectedIndex = 0, GridUI.cs:101
//   shutter             False    the literal GridUI.cs:594, :603 and :614 pass
//   minLaneSeparation   0        NUM_Lane_Dist, no Value in the designer
//   leadin1             0        NUM_leadin, likewise                          (grid)
//   leadin2             0        NUM_leadin2, likewise                         (grid)
//   leadin              0        NUM_leadin                                    (corridor, rotary)
//   HomeLocation        0,0      PlannedHomeLocation before a home is planned, CurrentState.cs:41
//                                                                              (grid, rotary)
//   useextendedendpoint False    chk_optimize_for_distance is unchecked in the designer (grid)
//   width               100      num_corridorwidth.Value, GridUI.Designer.cs:1011, passed through
//                                float as GridUI.cs:596 passes it              (corridor)
//   clockwise_laps      0        NUM_clockwise_laps has no Value, GridUI.Designer.cs:797 (rotary)
//   match_spiral_perimeter False CHK_match_spiral_perimeter is unchecked, GridUI.Designer.cs:781
//                                                                              (rotary)
//   laps                200      NUM_laps.Value, GridUI.Designer.cs:772        (rotary)
//   StartPointLatLngAlt 0,0      Grid.StartPointLatLngAlt, Grid.cs:34; read only for startpos=Point
//                                                                              (grid, rotary)
//
// Output, one <outdir>/<case>.csv per case, every line `key,value[,value...]`: the case name, every
// polygon vertex and every argument exactly as the generator received it (so a default the harness
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
        if (args.Length != 3
            || (args[0] != "grid" && args[0] != "corridor" && args[0] != "rotary" && args[0] != "offset"))
        {
            Console.Error.WriteLine("usage: MpGrid grid|corridor|rotary|offset <cases.txt> <outdir>");
            return 2;
        }
        try
        {
            return RunCases(args[0], args[1], args[2]);
        }
        catch (Exception ex)
        {
            // No partial golden data: a case that cannot be run fails the whole regeneration.
            Console.Error.WriteLine("MpGrid: " + ex);
            return 1;
        }
    }

    static int RunCases(string verb, string casesPath, string outDir)
    {
        // The directive this verb runs.
        string directive = verb == "grid" ? "case" : verb;
        var polygons = new Dictionary<string, List<PointLatLngAlt>>();
        var paths = new Dictionary<string, List<ClipperLib.IntPoint>>();
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
            else if (words[0] == "path")
            {
                if (words.Length < 3)
                    throw new FormatException(where + ": path needs a name and points");
                var path = new List<ClipperLib.IntPoint>();
                for (int i = 2; i < words.Length; i++)
                {
                    var xy = words[i].Split(',');
                    if (xy.Length != 2)
                        throw new FormatException(where + ": expected x,y, got " + words[i]);
                    path.Add(new ClipperLib.IntPoint(long.Parse(xy[0], Inv), long.Parse(xy[1], Inv)));
                }
                if (paths.ContainsKey(words[1]))
                    throw new FormatException(where + ": path " + words[1] + " defined twice");
                paths[words[1]] = path;
            }
            else if (words[0] == "offset")
            {
                if (words.Length != 4)
                    throw new FormatException(where + ": offset needs a name, paths and deltas");
                if (!names.Add(words[1]))
                    throw new FormatException(where + ": case " + words[1] + " defined twice");
                var input = new List<List<ClipperLib.IntPoint>>();
                foreach (var pathName in words[2].Split('+'))
                {
                    List<ClipperLib.IntPoint> path;
                    if (!paths.TryGetValue(pathName, out path))
                        throw new FormatException(where + ": unknown path " + pathName);
                    input.Add(path);
                }
                var deltas = new List<double>();
                foreach (var delta in words[3].Split('/'))
                    deltas.Add(double.Parse(delta, Inv));
                if (verb != "offset")
                    continue;
                RunOffsetCase(words[1], words[2], input, deltas, Path.Combine(outDir, words[1] + ".csv"));
                count++;
            }
            else if (words[0] == "accept")
            {
                // The Survey (Grid) dialog's cases: no verb here runs them (see the header).
                continue;
            }
            else if (words[0] == "case" || words[0] == "corridor" || words[0] == "rotary")
            {
                if (words.Length < 3)
                    throw new FormatException(where + ": " + words[0] + " needs a name and a polygon");
                if (!names.Add(words[1]))
                    throw new FormatException(where + ": case " + words[1] + " defined twice");
                List<PointLatLngAlt> poly;
                if (!polygons.TryGetValue(words[2], out poly))
                    throw new FormatException(where + ": unknown polygon " + words[2]);
                if (words[0] != directive)
                    continue;
                string outPath = Path.Combine(outDir, words[1] + ".csv");
                if (verb == "grid")
                    RunCase(words, poly, where, outPath);
                else if (verb == "corridor")
                    RunCorridorCase(words, poly, where, outPath);
                else
                    RunRotaryCase(words, poly, where, outPath);
                count++;
            }
            else
            {
                throw new FormatException(where + ": unknown directive " + words[0]);
            }
        }
        if (verb == "grid")
            Console.Error.WriteLine("MpGrid: wrote " + count + " cases to " + outDir);
        else
            Console.Error.WriteLine("MpGrid: wrote " + count + " " + verb + " cases to " + outDir);
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
        double resolvedAngle = angle ?? (double)(decimal)((LongestSideBearing(polygon) + 360) % 360);

        // A static that CreateGrid reads for startpos=Point (Grid.cs:580). It persists between
        // calls, so every case sets it rather than inheriting the previous case's.
        Grid.StartPointLatLngAlt = startPoint;

        List<PointLatLngAlt> result = Grid.CreateGrid(Copy(polygon), altitude, distance, spacing, resolvedAngle,
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
        WritePoints(sb, result, outPath);
    }

    // Grid.CreateCorridor (Grid.cs:55), called as GridUI.cs:592-596 calls it.
    static void RunCorridorCase(string[] words, List<PointLatLngAlt> polygon, string where, string outPath)
    {
        var parameters = Parameters(words, where);
        string v;
        double altitude = (v = Take(parameters, "altitude")) != null ? double.Parse(v, Inv) : 100;
        double distance = (v = Take(parameters, "distance")) != null ? double.Parse(v, Inv) : 50;
        double spacing = (v = Take(parameters, "spacing")) != null ? double.Parse(v, Inv) : 0;
        double angle = ResolveAngle(Take(parameters, "angle"), polygon);
        double overshoot1 = (v = Take(parameters, "overshoot1")) != null ? double.Parse(v, Inv) : 0;
        double overshoot2 = (v = Take(parameters, "overshoot2")) != null ? double.Parse(v, Inv) : 0;
        var startpos = (v = Take(parameters, "startpos")) != null
            ? (Grid.StartPosition)Enum.Parse(typeof(Grid.StartPosition), v)
            : Grid.StartPosition.Home;
        bool shutter = (v = Take(parameters, "shutter")) != null ? bool.Parse(v) : false;
        float minLaneSeparation = (v = Take(parameters, "minLaneSeparation")) != null ? float.Parse(v, Inv) : 0;
        // GridUI.cs:596 passes (float)num_corridorwidth.Value for the double parameter.
        float width = (v = Take(parameters, "width")) != null ? float.Parse(v, Inv) : 100;
        float leadin = (v = Take(parameters, "leadin")) != null ? float.Parse(v, Inv) : 0;
        NoneLeft(parameters, where);

        List<PointLatLngAlt> result = Grid.CreateCorridor(Copy(polygon), altitude, distance, spacing, angle,
            overshoot1, overshoot2, startpos, shutter, minLaneSeparation, width, leadin);

        var sb = new StringBuilder();
        sb.Append("# Grid.CreateCorridor output from tools/csharp-reference/regen-grid.sh - do not edit\n");
        Row(sb, "case", words[1]);
        foreach (var p in polygon)
            Row(sb, "vertex", D(p.Lat), D(p.Lng));
        Row(sb, "altitude", D(altitude));
        Row(sb, "distance", D(distance));
        Row(sb, "spacing", D(spacing));
        Row(sb, "angle", D(angle));
        Row(sb, "overshoot1", D(overshoot1));
        Row(sb, "overshoot2", D(overshoot2));
        Row(sb, "startpos", startpos.ToString());
        Row(sb, "shutter", shutter.ToString());
        Row(sb, "minLaneSeparation", F(minLaneSeparation));
        // The double CreateCorridor received, which is the float widened.
        Row(sb, "width", D(width));
        Row(sb, "leadin", F(leadin));
        WritePoints(sb, result, outPath);
    }

    // Grid.CreateRotary (Grid.cs:196), called as GridUI.cs:600-605 calls it.
    static void RunRotaryCase(string[] words, List<PointLatLngAlt> polygon, string where, string outPath)
    {
        var parameters = Parameters(words, where);
        string v;
        double altitude = (v = Take(parameters, "altitude")) != null ? double.Parse(v, Inv) : 100;
        double distance = (v = Take(parameters, "distance")) != null ? double.Parse(v, Inv) : 50;
        double spacing = (v = Take(parameters, "spacing")) != null ? double.Parse(v, Inv) : 0;
        double angle = ResolveAngle(Take(parameters, "angle"), polygon);
        double overshoot1 = (v = Take(parameters, "overshoot1")) != null ? double.Parse(v, Inv) : 0;
        double overshoot2 = (v = Take(parameters, "overshoot2")) != null ? double.Parse(v, Inv) : 0;
        var startpos = (v = Take(parameters, "startpos")) != null
            ? (Grid.StartPosition)Enum.Parse(typeof(Grid.StartPosition), v)
            : Grid.StartPosition.Home;
        bool shutter = (v = Take(parameters, "shutter")) != null ? bool.Parse(v) : false;
        float minLaneSeparation = (v = Take(parameters, "minLaneSeparation")) != null ? float.Parse(v, Inv) : 0;
        float leadin = (v = Take(parameters, "leadin")) != null ? float.Parse(v, Inv) : 0;
        var home = (v = Take(parameters, "HomeLocation")) != null ? ParseLatLng(v, where) : new PointLatLngAlt();
        int clockwiseLaps = (v = Take(parameters, "clockwise_laps")) != null ? int.Parse(v, Inv) : 0;
        bool matchSpiralPerimeter = (v = Take(parameters, "match_spiral_perimeter")) != null ? bool.Parse(v) : false;
        int laps = (v = Take(parameters, "laps")) != null ? int.Parse(v, Inv) : 200;
        var startPoint = (v = Take(parameters, "StartPointLatLngAlt")) != null ? ParseLatLng(v, where) : PointLatLngAlt.Zero;
        NoneLeft(parameters, where);

        // A static that CreateRotary reads for startpos=Point (Grid.cs:241). It persists between
        // calls, so every case sets it rather than inheriting the previous case's.
        Grid.StartPointLatLngAlt = startPoint;

        List<PointLatLngAlt> result = Grid.CreateRotary(Copy(polygon), altitude, distance, spacing, angle,
            overshoot1, overshoot2, startpos, shutter, minLaneSeparation, leadin, home, clockwiseLaps,
            matchSpiralPerimeter, laps);

        var sb = new StringBuilder();
        sb.Append("# Grid.CreateRotary output from tools/csharp-reference/regen-grid.sh - do not edit\n");
        Row(sb, "case", words[1]);
        foreach (var p in polygon)
            Row(sb, "vertex", D(p.Lat), D(p.Lng));
        Row(sb, "altitude", D(altitude));
        Row(sb, "distance", D(distance));
        Row(sb, "spacing", D(spacing));
        Row(sb, "angle", D(angle));
        Row(sb, "overshoot1", D(overshoot1));
        Row(sb, "overshoot2", D(overshoot2));
        Row(sb, "startpos", startpos.ToString());
        Row(sb, "shutter", shutter.ToString());
        Row(sb, "minLaneSeparation", F(minLaneSeparation));
        Row(sb, "leadin", F(leadin));
        Row(sb, "HomeLocation", D(home.Lat), D(home.Lng));
        Row(sb, "clockwise_laps", clockwiseLaps.ToString(Inv));
        Row(sb, "match_spiral_perimeter", matchSpiralPerimeter.ToString());
        Row(sb, "laps", laps.ToString(Inv));
        Row(sb, "StartPointLatLngAlt", D(startPoint.Lat), D(startPoint.Lng));
        WritePoints(sb, result, outPath);
    }

    // ClipperLib.ClipperOffset as Grid.cs:248-257 drives it, over integer paths.
    static void RunOffsetCase(string name, string pathNames, List<List<ClipperLib.IntPoint>> input,
        List<double> deltas, string outPath)
    {
        var sb = new StringBuilder();
        sb.Append("# ClipperLib.ClipperOffset output from tools/csharp-reference/regen-grid.sh - do not edit\n");
        Row(sb, "case", name);
        var offset = new ClipperLib.ClipperOffset();
        foreach (var path in input)
        {
            var copy = new List<ClipperLib.IntPoint>(path);
            var values = new List<string>();
            foreach (var p in copy)
            {
                values.Add(p.X.ToString(Inv));
                values.Add(p.Y.ToString(Inv));
            }
            Row(sb, "path", values.ToArray());
            offset.AddPath(copy, ClipperLib.JoinType.jtMiter, ClipperLib.EndType.etClosedPolygon);
        }
        foreach (var delta in deltas)
        {
            var tree = new ClipperLib.PolyTree();
            offset.Execute(ref tree, delta);
            Row(sb, "execute", D(delta), tree.ChildCount.ToString(Inv));
            foreach (var child in tree.Childs)
                WriteNode(sb, child, 1);
        }
        File.WriteAllText(outPath, sb.ToString(), new UTF8Encoding(false));
    }

    // One PolyNode and, depth first, its children: the order PolyTreeToPaths visits them.
    static void WriteNode(StringBuilder sb, ClipperLib.PolyNode node, int depth)
    {
        var values = new List<string>();
        values.Add(depth.ToString(Inv));
        foreach (var p in node.Contour)
        {
            values.Add(p.X.ToString(Inv));
            values.Add(p.Y.ToString(Inv));
        }
        Row(sb, "node", values.ToArray());
        foreach (var child in node.Childs)
            WriteNode(sb, child, depth + 1);
    }

    // The `parameter=value` words of a corridor or rotary directive, each at most once.
    static Dictionary<string, string> Parameters(string[] words, string where)
    {
        var result = new Dictionary<string, string>();
        for (int i = 3; i < words.Length; i++)
        {
            int eq = words[i].IndexOf('=');
            if (eq <= 0)
                throw new FormatException(where + ": expected parameter=value, got " + words[i]);
            string key = words[i].Substring(0, eq);
            if (result.ContainsKey(key))
                throw new FormatException(where + ": " + key + " given twice");
            result[key] = words[i].Substring(eq + 1);
        }
        return result;
    }

    // The value given for `key`, or null for none; either way the key is used up.
    static string Take(Dictionary<string, string> parameters, string key)
    {
        string value;
        if (!parameters.TryGetValue(key, out value))
            return null;
        parameters.Remove(key);
        return value;
    }

    // A parameter the generator does not take is a typo, not a default.
    static void NoneLeft(Dictionary<string, string> parameters, string where)
    {
        foreach (var key in parameters.Keys)
            throw new FormatException(where + ": unknown parameter " + key);
    }

    // The generators mutate nothing they are given, but hand them a copy so that stays true of this
    // harness whatever a later Grid.cs does.
    static List<PointLatLngAlt> Copy(List<PointLatLngAlt> polygon)
    {
        var input = new List<PointLatLngAlt>();
        polygon.ForEach(p => input.Add(new PointLatLngAlt(p)));
        return input;
    }

    // GridUI.cs:104 stores the angle in a NumericUpDown, so it passes through decimal on its way to
    // the generator (GridUI.cs:593, :601). CreateCorridor and CreateRotary never read it.
    static double ResolveAngle(string value, List<PointLatLngAlt> polygon)
    {
        if (value != null)
            return double.Parse(value, Inv);
        return (double)(decimal)((LongestSideBearing(polygon) + 360) % 360);
    }

    static void WritePoints(StringBuilder sb, List<PointLatLngAlt> result, string outPath)
    {
        Row(sb, "points", result.Count.ToString(Inv));
        foreach (var p in result)
            Row(sb, "wp", D(p.Lat), D(p.Lng), D(p.Alt), p.Tag);

        // Explicit "\n" and no BOM, so the file is byte-identical whichever platform regenerates it.
        File.WriteAllText(outPath, sb.ToString(), new UTF8Encoding(false));
    }

    // The angle the dialog proposes before the operator touches it: the bearing of the polygon's
    // longest side, each corner taken with the corner before it (the first with the last), measured
    // with PointLatLngAlt's own distance and bearing so the number is the one the dialog shows.
    // Written here; the dialog's own method is private to its form.
    static double LongestSideBearing(List<PointLatLngAlt> polygon)
    {
        double bearing = 0;
        double longest = 0;
        for (int i = 0; i < polygon.Count; i++)
        {
            PointLatLngAlt from = polygon[(i + polygon.Count - 1) % polygon.Count];
            PointLatLngAlt to = polygon[i];
            double length = to.GetDistance(from);
            if (length > longest)
            {
                longest = length;
                bearing = to.GetBearing(from);
            }
        }
        return (bearing + 360) % 360;
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
