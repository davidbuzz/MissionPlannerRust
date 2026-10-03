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

// Headless projection oracle: the `projection` verb of PLAN.md §7.1, for §13.3 item 8 and Deliverable 8.
//
// Runs the projections the map and the planner actually use - GMap.NET's MercatorProjection
// (ExtLibs/GMap.NET.Core/GMap.NET.Projections/MercatorProjection.cs) and PointLatLngAlt's spherical
// geodesy and UTM (ExtLibs/Utilities/PointLatLngAlt.cs, utmpos.cs, and the ProjNet behind them) -
// over the coordinates in testdata/projection/points.txt and writes exactly what they return, so
// crates/mp-units/tests/projection.rs holds the Rust to the C# rather than to our reading of it.
// Built by regen-projection.sh from the same msbuild output of ExtLibs/Utilities as MpGrid.cs.
//
// Build:  see regen-projection.sh (mcs against the msbuild output of ExtLibs/Utilities)
// Run:    mono MpProjection.exe projection <points.txt> <outdir>
//
// points.txt, one directive per line, `#` starts a comment:
//   point   <lat> <lng>
//   lattice <lat from> <lat to> <lng from> <lng to> <step>   every from + i*step <= to, both axes,
//                                                           latitude-major; stepped in decimal so
//                                                           the ends are hit exactly
//   pair    <lat1> <lng1> <lat2> <lng2>
//   newpos  <lat> <lng> <bearing> <distance>
//
// Output, three files in <outdir>, every line `key,value[,value...]`:
//   mercator.csv  `size,<zoom>,<w>,<h>` for each zoom (GetTileMatrixSizePixel), then per point
//                 `point,<lat>,<lng>` followed, for each zoom in the order of the size lines, by
//                 FromLatLngToPixel's `<x>,<y>` and FromPixelToLatLng's `<lat>,<lng>` of that pixel.
//   geodesy.csv   `pair,<lat1>,<lng1>,<lat2>,<lng2>,<GetDistance>,<GetBearing>,<lat>,<lng>` for every
//                 `pair` directive and every two consecutive points, the last two being p1.newpos
//                 of that bearing and distance; `newpos,<lat>,<lng>,<bearing>,<distance>,<lat>,<lng>`
//                 for every `newpos` directive.
//   utm.csv       per point, `utm,<lat>,<lng>,<zone>,<x>,<y>,<lat>,<lng>`: new utmpos(point) - its
//                 zone is GetUTMZone, negative in the south - and that utmpos's ToLLA(); or
//                 `utm,<lat>,<lng>,error,<exception type>` if ProjNet refuses.
// Doubles are G17 in the invariant culture, as MpGrid.cs writes them: it always round-trips, which
// "R" does not on every runtime.

using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using GMap.NET.Projections;
using MissionPlanner.Utilities;

public static class MpProjection
{
    static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    // The zooms of PLAN §13.3 item 8, and 30. GMap returns whole pixels, so the continuous
    // projection can only be read through that rounding; 30 is the deepest zoom whose arithmetic
    // is still exact (`1 << zoom` is an int, MercatorProjection.cs:97), 2^38 pixels across the
    // world, so it is the finest readout GMap offers: 0.15 mm on the ground at the equator.
    static readonly int[] Zooms = { 1, 10, 16, 20, 30 };

    public static int Main(string[] args)
    {
        // Parsing and formatting must not depend on the machine's locale.
        System.Threading.Thread.CurrentThread.CurrentCulture = Inv;
        if (args.Length != 3 || args[0] != "projection")
        {
            Console.Error.WriteLine("usage: MpProjection projection <points.txt> <outdir>");
            return 2;
        }
        try
        {
            return Run(args[1], args[2]);
        }
        catch (Exception ex)
        {
            // No partial golden data: a directive that cannot be run fails the whole regeneration.
            Console.Error.WriteLine("MpProjection: " + ex);
            return 1;
        }
    }

    static int Run(string pointsPath, string outDir)
    {
        var points = new List<PointLatLngAlt>();
        var pairs = new List<PointLatLngAlt[]>();
        var newposes = new List<double[]>();
        int lineNo = 0;
        foreach (var raw in File.ReadAllLines(pointsPath))
        {
            lineNo++;
            var line = raw;
            int hash = line.IndexOf('#');
            if (hash >= 0)
                line = line.Substring(0, hash);
            var words = line.Split(new[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
            if (words.Length == 0)
                continue;
            string where = pointsPath + ":" + lineNo;

            switch (words[0])
            {
                case "point":
                    Expect(words, 3, where);
                    points.Add(new PointLatLngAlt(Num(words[1], where), Num(words[2], where)));
                    break;
                case "lattice":
                    Expect(words, 6, where);
                    Lattice(points, Dec(words[1], where), Dec(words[2], where), Dec(words[3], where),
                        Dec(words[4], where), Dec(words[5], where), where);
                    break;
                case "pair":
                    Expect(words, 5, where);
                    pairs.Add(new[]
                    {
                        new PointLatLngAlt(Num(words[1], where), Num(words[2], where)),
                        new PointLatLngAlt(Num(words[3], where), Num(words[4], where)),
                    });
                    break;
                case "newpos":
                    Expect(words, 5, where);
                    newposes.Add(new[]
                    {
                        Num(words[1], where), Num(words[2], where), Num(words[3], where),
                        Num(words[4], where),
                    });
                    break;
                default:
                    throw new FormatException(where + ": unknown directive " + words[0]);
            }
        }

        Directory.CreateDirectory(outDir);
        Write(Path.Combine(outDir, "mercator.csv"), Mercator(points));
        Write(Path.Combine(outDir, "geodesy.csv"), Geodesy(points, pairs, newposes));
        Write(Path.Combine(outDir, "utm.csv"), Utm(points));
        Console.Error.WriteLine("MpProjection: " + points.Count + " points, " +
            (pairs.Count + points.Count - 1) + " pairs, " + newposes.Count + " newpos");
        return 0;
    }

    static StringBuilder Mercator(List<PointLatLngAlt> points)
    {
        var projection = MercatorProjection.Instance;
        var sb = Header("MercatorProjection.FromLatLngToPixel and FromPixelToLatLng");
        foreach (int zoom in Zooms)
        {
            var size = projection.GetTileMatrixSizePixel(zoom);
            Row(sb, "size", zoom.ToString(Inv), L(size.Width), L(size.Height));
        }
        foreach (var p in points)
        {
            var row = new List<string> { D(p.Lat), D(p.Lng) };
            foreach (int zoom in Zooms)
            {
                var pixel = projection.FromLatLngToPixel(p.Lat, p.Lng, zoom);
                var back = projection.FromPixelToLatLng(pixel.X, pixel.Y, zoom);
                row.Add(L(pixel.X));
                row.Add(L(pixel.Y));
                row.Add(D(back.Lat));
                row.Add(D(back.Lng));
            }
            Row(sb, "point", row.ToArray());
        }
        return sb;
    }

    static StringBuilder Geodesy(List<PointLatLngAlt> points, List<PointLatLngAlt[]> pairs,
        List<double[]> newposes)
    {
        var sb = Header("PointLatLngAlt.GetDistance, GetBearing and newpos");
        var all = new List<PointLatLngAlt[]>(pairs);
        for (int i = 1; i < points.Count; i++)
            all.Add(new[] { points[i - 1], points[i] });
        foreach (var pair in all)
        {
            var a = pair[0];
            var b = pair[1];
            double distance = a.GetDistance(b);
            double bearing = a.GetBearing(b);
            var there = a.newpos(bearing, distance);
            Row(sb, "pair", D(a.Lat), D(a.Lng), D(b.Lat), D(b.Lng), D(distance), D(bearing),
                D(there.Lat), D(there.Lng));
        }
        foreach (var n in newposes)
        {
            var there = new PointLatLngAlt(n[0], n[1]).newpos(n[2], n[3]);
            Row(sb, "newpos", D(n[0]), D(n[1]), D(n[2]), D(n[3]), D(there.Lat), D(there.Lng));
        }
        return sb;
    }

    static StringBuilder Utm(List<PointLatLngAlt> points)
    {
        var sb = Header("new utmpos(PointLatLngAlt) and utmpos.ToLLA");
        foreach (var p in points)
        {
            try
            {
                var utm = new utmpos(p);
                var back = utm.ToLLA();
                Row(sb, "utm", D(p.Lat), D(p.Lng), utm.zone.ToString(Inv), D(utm.x), D(utm.y),
                    D(back.Lat), D(back.Lng));
            }
            catch (Exception ex)
            {
                // Recorded rather than fatal: which coordinates ProjNet refuses is itself the
                // behaviour a port has to reproduce.
                Row(sb, "utm", D(p.Lat), D(p.Lng), "error", ex.GetType().Name);
            }
        }
        return sb;
    }

    static void Lattice(List<PointLatLngAlt> points, decimal lat0, decimal lat1, decimal lng0,
        decimal lng1, decimal step, string where)
    {
        if (step <= 0 || lat1 < lat0 || lng1 < lng0)
            throw new FormatException(where + ": lattice needs from <= to and a positive step");
        for (decimal lat = lat0; lat <= lat1; lat += step)
            for (decimal lng = lng0; lng <= lng1; lng += step)
                // Through the decimal's own digits, so the double is the one the same text in a
                // `point` line would give.
                points.Add(new PointLatLngAlt(Num(lat.ToString(Inv), where), Num(lng.ToString(Inv), where)));
    }

    static void Expect(string[] words, int count, string where)
    {
        if (words.Length != count)
            throw new FormatException(where + ": " + words[0] + " takes " + (count - 1) + " numbers");
    }

    static double Num(string text, string where)
    {
        double value;
        if (!double.TryParse(text, NumberStyles.Float, Inv, out value))
            throw new FormatException(where + ": " + text + " is not a number");
        return value;
    }

    static decimal Dec(string text, string where)
    {
        decimal value;
        if (!decimal.TryParse(text, NumberStyles.Float, Inv, out value))
            throw new FormatException(where + ": " + text + " is not a number");
        return value;
    }

    static StringBuilder Header(string what)
    {
        var sb = new StringBuilder();
        sb.Append("# ").Append(what).Append(" from tools/csharp-reference/regen-projection.sh - do not edit\n");
        return sb;
    }

    static void Row(StringBuilder sb, string key, params string[] values)
    {
        sb.Append(key);
        foreach (var v in values)
            sb.Append(',').Append(v);
        sb.Append('\n');
    }

    // Explicit "\n" and no BOM, so the file is byte-identical whichever platform regenerates it.
    static void Write(string path, StringBuilder sb)
    {
        File.WriteAllText(path, sb.ToString(), new UTF8Encoding(false));
    }

    static string D(double d) { return d.ToString("G17", Inv); }

    static string L(long l) { return l.ToString(Inv); }
}
