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

// Headless oracle for the planning screen's map-menu geometry and files: the goldens in
// testdata/planner/golden, which crates/mp-mission's circle, polygon, shapefile and fence_file
// tests hold the Rust port to.
//
// The handlers live in GCSViews/FlightPlanner.cs, a WinForms control mono cannot construct
// headless, so each verb below is the handler's own lines copied out - cited per verb, changed
// only where a grid cell or a dialog stood - run against Mission Planner's own libraries:
// MissionPlanner.Utilities (utmpos, PointLatLngAlt, ClipperLib, MathHelper), ProjNet (the area)
// and DotSpatial.Projections (the shapefile's .prj). What the goldens pin is the arithmetic in the
// C#'s own types and evaluation order - float where it has float - under the runtime Mission
// Planner runs on under Linux.
//
// Build (against the MissionPlanner.Utilities that tools/csharp-reference/regen-grid.sh builds):
//   OUT=~/.cache/mp-csharp-reference/<mp sha>/src/ExtLibs/Utilities/bin/Release/netstandard2.0
//   NS=$(find /usr/lib/mono -path '*/4.5/Facades/netstandard.dll' | head -1)
//   mcs -nologo -nowarn:1685 -out:$OUT/PlannerOracle.exe -r:$OUT/MissionPlanner.Utilities.dll \
//       -r:$OUT/GMap.NET.Core.dll -r:$OUT/System.Memory.dll -r:$OUT/ProjNET.dll \
//       -r:$OUT/GeoAPI.dll -r:$OUT/GeoAPI.CoordinateSystems.dll \
//       -r:$OUT/DotSpatial.Projections.dll -r:$NS tools/csharp-reference/PlannerOracle.cs
//   mono $OUT/PlannerOracle.exe testdata/planner/golden
//
// Every double is written G17 and every float G9, invariant culture: both round-trip.

using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using GMap.NET;
using MissionPlanner.Utilities;
using ProjNet.CoordinateSystems;
using ProjNet.CoordinateSystems.Transformations;
using GeoAPI.CoordinateSystems;
using GeoAPI.CoordinateSystems.Transformations;

public static class PlannerOracle
{
    static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    static string D(double v)
    {
        return v.ToString("G17", Inv);
    }

    public static int Main(string[] args)
    {
        System.Threading.Thread.CurrentThread.CurrentCulture = Inv;
        if (args.Length != 1)
        {
            Console.Error.WriteLine("usage: PlannerOracle <outdir>");
            return 2;
        }
        var outDir = args[0];
        Directory.CreateDirectory(outDir);

        File.WriteAllText(Path.Combine(outDir, "wp_circle.csv"), WpCircles());
        File.WriteAllText(Path.Combine(outDir, "spline_circle.csv"), SplineCircles());
        File.WriteAllText(Path.Combine(outDir, "offset.csv"), Offsets());
        File.WriteAllText(Path.Combine(outDir, "area.csv"), Areas());
        File.WriteAllText(Path.Combine(outDir, "rally.ral"), RallyFile());
        File.WriteAllText(Path.Combine(outDir, "rally_read.csv"), RallyRead());
        File.WriteAllText(Path.Combine(outDir, "polygon.poly"), PolygonFile());
        File.WriteAllText(Path.Combine(outDir, "reproject.csv"), Reprojections());
        return 0;
    }

    // The centre every circle is drawn about: SITL's home at CMAC.
    static readonly PointLatLng Cmac = new PointLatLng(-35.3632621, 149.1652374);

    // createWpCircleToolStripMenuItem_Click, GCSViews/FlightPlanner.cs:2963-3048, from the parse of
    // the four answers on; each row's setfromMap(pll.Lat, pll.Lng, ...) is a `wp` line.
    static string WpCircle(string name, PointLatLng MouseDownEnd, int Radius, int Points, int Direction,
        int startangle)
    {
        var sb = new StringBuilder();
        sb.AppendLine(string.Join(",", "case", name, D(MouseDownEnd.Lat), D(MouseDownEnd.Lng), Radius, Points,
            Direction, startangle));

        double a = startangle;
        double step = 360.0f / Points;
        if (Direction == -1)
        {
            a += 360;
            step *= -1;
        }

        for (; a <= (startangle + 360) && a >= 0; a += step)
        {
            float d = Radius;
            float R = 6371000;

            var lat2 = Math.Asin(Math.Sin(MouseDownEnd.Lat * MathHelper.deg2rad) * Math.Cos(d / R) +
                                 Math.Cos(MouseDownEnd.Lat * MathHelper.deg2rad) * Math.Sin(d / R) *
                                 Math.Cos(a * MathHelper.deg2rad));
            var lon2 = MouseDownEnd.Lng * MathHelper.deg2rad +
                       Math.Atan2(
                           Math.Sin(a * MathHelper.deg2rad) * Math.Sin(d / R) *
                           Math.Cos(MouseDownEnd.Lat * MathHelper.deg2rad),
                           Math.Cos(d / R) - Math.Sin(MouseDownEnd.Lat * MathHelper.deg2rad) * Math.Sin(lat2));

            PointLatLng pll = new PointLatLng(lat2 * MathHelper.rad2deg, lon2 * MathHelper.rad2deg);

            sb.AppendLine(string.Join(",", "wp", D(pll.Lat), D(pll.Lng)));
        }

        return sb.ToString();
    }

    static string WpCircles()
    {
        return WpCircle("default", Cmac, 50, 20, 1, 0)
               + WpCircle("seven_reversed", Cmac, 100, 7, -1, 30)
               + WpCircle("reversed_past_zero", Cmac, 50, 20, -1, 100)
               + WpCircle("no_points", Cmac, 50, 0, 1, 0)
               + WpCircle("negative_start", Cmac, 50, 20, 1, -10)
               + WpCircle("north", new PointLatLng(51.5007292, -0.1246254), 5000, 12, 1, 45);
    }

    // createSplineCircleToolStripMenuItem_Click, GCSViews/FlightPlanner.cs:2857-2961, from the
    // parse of the answers on: `roi` for the AddCommand(DO_SET_ROI, ...) at MouseDownStart, `wp`
    // for each setfromMap(pll.Lat, pll.Lng, stepalt). The C#'s startangle is never parsed and
    // stays 0; Points is 4.
    static string SplineCircle(string name, PointLatLng MouseDownEnd, int Radius, int minalt, int maxalt,
        int altstep)
    {
        var sb = new StringBuilder();
        sb.AppendLine(string.Join(",", "case", name, D(MouseDownEnd.Lat), D(MouseDownEnd.Lng), Radius, minalt,
            maxalt, altstep));
        int Points = 4;
        int startangle = 0;

        double a = startangle;
        double step = 360.0f / Points;

        sb.AppendLine(string.Join(",", "roi", D(MouseDownEnd.Lat), D(MouseDownEnd.Lng)));

        bool startup = true;

        for (int stepalt = minalt; stepalt <= maxalt;)
        {
            for (a = 0; a <= (startangle + 360) && a >= 0; a += step)
            {
                float d = Radius;
                float R = 6371000;

                var lat2 = Math.Asin(Math.Sin(MouseDownEnd.Lat * MathHelper.deg2rad) * Math.Cos(d / R) +
                                     Math.Cos(MouseDownEnd.Lat * MathHelper.deg2rad) * Math.Sin(d / R) *
                                     Math.Cos(a * MathHelper.deg2rad));
                var lon2 = MouseDownEnd.Lng * MathHelper.deg2rad +
                           Math.Atan2(
                               Math.Sin(a * MathHelper.deg2rad) * Math.Sin(d / R) *
                               Math.Cos(MouseDownEnd.Lat * MathHelper.deg2rad),
                               Math.Cos(d / R) - Math.Sin(MouseDownEnd.Lat * MathHelper.deg2rad) * Math.Sin(lat2));

                PointLatLng pll = new PointLatLng(lat2 * MathHelper.rad2deg, lon2 * MathHelper.rad2deg);

                sb.AppendLine(string.Join(",", "wp", D(pll.Lat), D(pll.Lng), stepalt));

                if (!startup)
                    stepalt += altstep / Points;
            }

            // reset back to the start
            if (startup)
                stepalt = minalt;

            // we have finsihed the first run
            startup = false;
        }

        return sb.ToString();
    }

    static string SplineCircles()
    {
        return SplineCircle("default", Cmac, 50, 5, 20, 5)
               + SplineCircle("one_lap_band", Cmac, 80, 10, 10, 8)
               + SplineCircle("inverted_band", Cmac, 50, 30, 20, 5);
    }

    // offsetPolygonToolStripMenuItem_Click, GCSViews/FlightPlanner.cs:3639-3686, from the parse of
    // the answer on; drawnpolygon.Points is `polygon`, and redrawPolygonSurvey's list is the
    // `corner` lines. `empty` where the C# returns with the polygon as it was.
    static string Offset(string name, List<PointLatLng> polygon, double intmeter)
    {
        var sb = new StringBuilder();
        sb.AppendLine(string.Join(",", "case", name, D(intmeter)));
        foreach (var p in polygon)
            sb.AppendLine(string.Join(",", "vertex", D(p.Lat), D(p.Lng)));

        List<PointLatLngAlt> list = new List<PointLatLngAlt>();
        polygon.ForEach(x => { list.Add(x); });

        List<utmpos> ans = new List<utmpos>();

        // utm zone distance calcs will be done in
        int utmzone = list[0].GetUTMZone();

        // utm position list
        List<utmpos> utmpositions = utmpos.ToList(PointLatLngAlt.ToUTM(utmzone, list), utmzone);

        // close the loop if its not already
        if (utmpositions[0] != utmpositions[utmpositions.Count - 1])
            utmpositions.Add(utmpositions[0]); // make a full loop

        ClipperLib.ClipperOffset clipperOffset = new ClipperLib.ClipperOffset();

        clipperOffset.AddPath(utmpositions.Select(a => { return new ClipperLib.IntPoint(a.x * 1000.0, a.y * 1000.0); }).ToList(), ClipperLib.JoinType.jtMiter, ClipperLib.EndType.etClosedPolygon);

        List<utmpos> ans1 = new List<utmpos>();

        ClipperLib.PolyTree tree = new ClipperLib.PolyTree();
        clipperOffset.Execute(ref tree, (Int64)(intmeter * 1000.0));

        if (tree.ChildCount == 0)
        {
            sb.AppendLine("empty");
            return sb.ToString();
        }

        foreach (var treeChild in tree.Childs)
        {
            ans1 = treeChild.Contour.Select(a => new utmpos(a.X / 1000.0, a.Y / 1000.0, utmzone))
                .ToList();

            ans.AddRange(ans1);
        }

        foreach (var plla in ans)
        {
            var a = plla.ToLLA();
            sb.AppendLine(string.Join(",", "corner", D(a.Lat), D(a.Lng)));
        }

        return sb.ToString();
    }

    // Offsets from a click at CMAC: a square of about 110 m, an L that has a reflex corner, and a
    // thin triangle that a large inset removes entirely.
    static List<PointLatLng> Square()
    {
        return new List<PointLatLng>
        {
            new PointLatLng(-35.3627, 149.1646), new PointLatLng(-35.3627, 149.1658),
            new PointLatLng(-35.3637, 149.1658), new PointLatLng(-35.3637, 149.1646)
        };
    }

    static List<PointLatLng> Ell()
    {
        return new List<PointLatLng>
        {
            new PointLatLng(-35.3620, 149.1640), new PointLatLng(-35.3620, 149.1650),
            new PointLatLng(-35.3630, 149.1650), new PointLatLng(-35.3630, 149.1670),
            new PointLatLng(-35.3640, 149.1670), new PointLatLng(-35.3640, 149.1640)
        };
    }

    static List<PointLatLng> Sliver()
    {
        return new List<PointLatLng>
        {
            new PointLatLng(-35.3630, 149.1650), new PointLatLng(-35.3631, 149.1680),
            new PointLatLng(-35.3632, 149.1650)
        };
    }

    static string Offsets()
    {
        return Offset("square_out", Square(), 10)
               + Offset("square_in", Square(), -10)
               + Offset("square_zero", Square(), 0)
               + Offset("ell_out", Ell(), 5.5)
               + Offset("ell_in", Ell(), -20)
               + Offset("sliver_gone", Sliver(), -30);
    }

    // areaToolStripMenuItem_Click and calcpolygonarea, GCSViews/FlightPlanner.cs:1760-1773 and
    // 1986-2036: the message's text, with a `text` line per line of it.
    static double calcpolygonarea(List<PointLatLng> polygon)
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

        int utmzone = (int) ((polygon[0].Lng - -186.0) / 6.0);

        IProjectedCoordinateSystem utm = ProjectedCoordinateSystem.WGS84_UTM(utmzone,
            polygon[0].Lat < 0 ? false : true);

        ICoordinateTransformation trans = ctfac.CreateFromCoordinateSystems(wgs84, utm);

        double prod1 = 0;
        double prod2 = 0;

        for (int a = 0; a < (polygon.Count - 1); a++)
        {
            double[] pll1 = {polygon[a].Lng, polygon[a].Lat};
            double[] pll2 = {polygon[a + 1].Lng, polygon[a + 1].Lat};

            double[] p1 = trans.MathTransform.Transform(pll1);
            double[] p2 = trans.MathTransform.Transform(pll2);

            prod1 += p1[0] * p2[1];
            prod2 += p1[1] * p2[0];
        }

        double answer = (prod1 - prod2) / 2;

        if (polygon[0] == polygon[polygon.Count - 1])
            polygon.RemoveAt(polygon.Count - 1); // unmake a full loop

        return answer;
    }

    static string Area(string name, List<PointLatLng> polygon)
    {
        var sb = new StringBuilder();
        sb.AppendLine(string.Join(",", "case", name));
        foreach (var p in polygon)
            sb.AppendLine(string.Join(",", "vertex", D(p.Lat), D(p.Lng)));

        double aream2 = Math.Abs((double) calcpolygonarea(polygon));

        double areaa = aream2 * 0.000247105;

        double areaha = aream2 * 1e-4;

        double areasqf = aream2 * 10.7639;

        var text = "Area: " + aream2.ToString("0") + " m2\n\t" + areaa.ToString("0.00") + " Acre\n\t" +
                   areaha.ToString("0.00") + " Hectare\n\t" + areasqf.ToString("0") + " sqf";
        sb.AppendLine("m2," + D(aream2));
        foreach (var line in text.Split('\n'))
            sb.AppendLine("text," + line);
        return sb.ToString();
    }

    static string Areas()
    {
        // A hectare-sized field, the L, and a field of a few square metres whose figures round.
        return Area("square", Square())
               + Area("ell", Ell())
               + Area("small", new List<PointLatLng>
               {
                   new PointLatLng(-35.36326, 149.16523), new PointLatLng(-35.36326, 149.16526),
                   new PointLatLng(-35.36329, 149.16526)
               })
               + Area("empty", new List<PointLatLng>());
    }

    // The rally markers the file verbs use: a whole-number click, one with every digit a click
    // carries, and one near zero on both axes.
    static readonly double[][] RallyMarkers =
    {
        new[] {-35.3632621, 149.1652374, 100},
        new[] {-35.36326234567891, 149.16523741234567, 60},
        new[] {-0.00001, 0.000015, 0},
    };

    // saveToFileToolStripMenuItem1_Click, GCSViews/FlightPlanner.cs:6044-6055, with
    // Application.ProductVersion written as "0.0.0" - the header the tests compare up to the
    // version.
    static string RallyFile()
    {
        var sw = new StringWriter(Inv);
        sw.NewLine = "\n";
        sw.WriteLine("#saved by Mission Planner " + "0.0.0");
        foreach (var m in RallyMarkers)
        {
            var Position = new PointLatLng(m[0], m[1]);
            int Alt = (int) m[2];
            sw.WriteLine("{0}\t{1}\t{2}\t{3}\t{4}\t{5}\t{6}", "RALLY",
                Position.Lat.ToString(CultureInfo.InvariantCulture),
                Position.Lng.ToString(CultureInfo.InvariantCulture),
                Alt.ToString(CultureInfo.InvariantCulture), 0, 0, 0);
        }

        return sw.ToString();
    }

    // loadFromFileToolStripMenuItem1_Click, GCSViews/FlightPlanner.cs:4426-4461, over RallyFile's
    // text and three hand rows: each marker's Position and Alt as GMapMarkerRallyPt keeps them.
    static string RallyRead()
    {
        var text = RallyFile() + "RALLY\t-35.12345678\t149.98765432\t123.9\t-4.5\t270.7\t1\n"
                   + "ANYTHING 51.5 -0.125 7 0 0 0\n";
        var sb = new StringBuilder();
        foreach (var line in text.Split('\n'))
        {
            if (line == "" || line.StartsWith("#"))
                continue;
            string[] items = line.Split(new[] {' ', '\t'}, StringSplitOptions.RemoveEmptyEntries);

            // mavlink_rally_point_t's fields, with their types (int, int, short, short, ushort, byte).
            RallyPoint rally = new RallyPoint();

            rally.lat = (int) (float.Parse(items[1], CultureInfo.InvariantCulture) * 1e7);
            rally.lng = (int) (float.Parse(items[2], CultureInfo.InvariantCulture) * 1e7);
            rally.alt = (short) float.Parse(items[3], CultureInfo.InvariantCulture);
            rally.break_alt = (short) float.Parse(items[4], CultureInfo.InvariantCulture);
            rally.land_dir = (ushort) float.Parse(items[5], CultureInfo.InvariantCulture);
            rally.flags = byte.Parse(items[6], CultureInfo.InvariantCulture);

            var plla = new PointLatLngAlt(rally.lat / 1e7, rally.lng / 1e7, rally.alt);
            sb.AppendLine(string.Join(",", "marker", D(plla.Lat), D(plla.Lng), (int) plla.Alt, rally.break_alt,
                rally.land_dir, rally.flags));
        }

        return sb.ToString();
    }

    struct RallyPoint
    {
        public int lat;
        public int lng;
        public short alt;
        public short break_alt;
        public ushort land_dir;
        public byte flags;
    }

    // savePolygonToolStripMenuItem_Click, GCSViews/FlightPlanner.cs:5892-5933, over the L.
    static string PolygonFile()
    {
        var sw = new StringWriter(Inv);
        sw.NewLine = "\n";
        sw.WriteLine("#saved by Mission Planner " + "0.0.0");
        var points = Ell();
        points.Add(new PointLatLng(-35.36326234567891, 149.16523741234567));
        foreach (var pll in points)
        {
            sw.WriteLine(pll.Lat.ToString(CultureInfo.InvariantCulture) + " " +
                         pll.Lng.ToString(CultureInfo.InvariantCulture));
        }

        PointLatLng pll2 = points[0];

        sw.WriteLine(pll2.Lat.ToString(CultureInfo.InvariantCulture) + " " +
                     pll2.Lng.ToString(CultureInfo.InvariantCulture));
        return sw.ToString();
    }

    // fromSHPToolStripMenuItem_Click's reprojection, GCSViews/FlightPlanner.cs:3547-3584:
    // ParseEsriString on the .prj's first line, then ReprojectPoints to WGS 1984 per point.
    static string Reproject(string name, string esri, double x, double y)
    {
        DotSpatial.Projections.ProjectionInfo pStart = new DotSpatial.Projections.ProjectionInfo();
        DotSpatial.Projections.ProjectionInfo pESRIEnd =
            DotSpatial.Projections.KnownCoordinateSystems.Geographic.World.WGS1984;
        pStart.ParseEsriString(esri);
        double[] xyarray = {x, y};
        double[] zarray = {0};
        DotSpatial.Projections.Reproject.ReprojectPoints(xyarray, zarray, pStart, pESRIEnd, 0, 1);
        return string.Join(",", "point", name, D(x), D(y), D(xyarray[1]), D(xyarray[0])) + "\n";
    }

    const string Utm55S =
        "PROJCS[\"WGS_1984_UTM_Zone_55S\",GEOGCS[\"GCS_WGS_1984\",DATUM[\"D_WGS_1984\",SPHEROID[\"WGS_1984\",6378137.0,298.257223563]],PRIMEM[\"Greenwich\",0.0],UNIT[\"Degree\",0.0174532925199433]],PROJECTION[\"Transverse_Mercator\"],PARAMETER[\"False_Easting\",500000.0],PARAMETER[\"False_Northing\",10000000.0],PARAMETER[\"Central_Meridian\",147.0],PARAMETER[\"Scale_Factor\",0.9996],PARAMETER[\"Latitude_Of_Origin\",0.0],UNIT[\"Meter\",1.0]]";

    const string Utm30N =
        "PROJCS[\"WGS_1984_UTM_Zone_30N\",GEOGCS[\"GCS_WGS_1984\",DATUM[\"D_WGS_1984\",SPHEROID[\"WGS_1984\",6378137.0,298.257223563]],PRIMEM[\"Greenwich\",0.0],UNIT[\"Degree\",0.0174532925199433]],PROJECTION[\"Transverse_Mercator\"],PARAMETER[\"False_Easting\",500000.0],PARAMETER[\"False_Northing\",0.0],PARAMETER[\"Central_Meridian\",-3.0],PARAMETER[\"Scale_Factor\",0.9996],PARAMETER[\"Latitude_Of_Origin\",0.0],UNIT[\"Meter\",1.0]]";

    const string Wgs84 =
        "GEOGCS[\"GCS_WGS_1984\",DATUM[\"D_WGS_1984\",SPHEROID[\"WGS_1984\",6378137.0,298.257223563]],PRIMEM[\"Greenwich\",0.0],UNIT[\"Degree\",0.0174532925199433]]";

    static string Reprojections()
    {
        return Reproject("utm55s", Utm55S, 695400.0, 6084100.0)
               + Reproject("utm55s", Utm55S, 695530.25, 6084205.5)
               + Reproject("utm55s", Utm55S, 500000.0, 10000000.0)
               + Reproject("utm30n", Utm30N, 699316.0, 5710164.0)
               + Reproject("utm30n", Utm30N, 300000.0, 4000000.0)
               + Reproject("wgs84", Wgs84, 149.1652374, -35.3632621);
    }
}
