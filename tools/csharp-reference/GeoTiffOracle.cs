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

// Headless GeoTIFF oracle: Mission Planner's own GeoTiff (ExtLibs/Utilities/GeoTiff.cs) and the
// srtm.getAltitude that asks it first (ExtLibs/Utilities/srtm.cs:116-150), for
// crates/mp-terrain/tests/geotiff.rs.
//
// Runs the classes out of the msbuild output of ExtLibs/Utilities - the build regen-srtm.sh makes,
// with LibTiff.Net, DotSpatial.Projections and Microsoft.Extensions.Caching.Memory beside it -
// with srtm.datadirectory pointed at a scratch copy of testdata/geotiff/*.tif, srtm's two servers
// pointed at a loopback port nothing listens on, and GMap.NET in CacheOnly mode, so a lookup that
// falls through to SRTM queues nothing and nothing leaves the machine.
//
// Build:  see regen-geotiff.sh (mcs against the msbuild output of ExtLibs/Utilities)
// Run:    mono GeoTiffOracle.exe <testdata/geotiff> <empty scratch dir> <out oracle.txt>
//
// The index order. GeoTiff's static constructor lists the directory with Directory.GetFiles, which
// on Windows returns NTFS's order - by name, compared as upper case - and under mono on Linux
// returns readdir's, which depends on the file system and is not stable. Only a point two files
// both cover depends on it, and Mission Planner is a Windows program, so the harness sorts
// GeoTiff.index into NTFS's order (ordinal, upper-cased) before the first lookup; the port lists
// in that order (geotiff.rs says why).
//
// oracle.txt, one record per line, fields separated by single spaces, strings in double quotes:
//   epsg <code> ok "<Name>" "<Transform.Name>" "<ToProj4String()>"
//   epsg <code> throws <exception type>
//                           ProjectionInfo.FromEpsgCode for the codes LoadFile's branches name
//   geog "<proj4>" <code>...               the codes 4000-4999 whose FromEpsgCode has that definition
//   esri "<string>" ok|throws ...          ProjectionInfo.FromEsriString, as the epsg records
//   proj4 "<string>" ok|throws ...         ProjectionInfo.FromProj4String, as the epsg records
//   fwd <code> <lng> <lat> <x> <y>         Reproject.ReprojectPoints from WGS 1984 to FromEpsgCode(code)
//   inv <code> <x> <y> <lng> <lat>         and back
//   skipped "<file>" <exception type> "<message>"
//                           a .tif generateIndex left out: LoadFile threw (asked again, here)
//   index <n> "<file>" <width> <height> <bits> <type> <Area.Lat> <Area.Lng> <Area.WidthLng>
//         <Area.HeightLat> <x> <y> <xscale> <yscale> <ProjectedCSTypeGeoKey>
//         <GTRasterTypeGeoKey> <ProjCoordTransGeoKey> <UTMZone> null|"<srcProjection.Name>"
//         null|"<srcProjection.Transform.Name>"
//                           GeoTiff.index after the sort, every field getAltitude and DEM read
//   param "<file>" <IsLatLon> <Transform.Proj4Name> <DatumType> <ToWGS84> <a> <b> <Radians>
//         <Meridian.Longitude> <Unit.Meters> <CentralMeridian> <Lam0> <LatitudeOfOrigin> <Phi0>
//         <ScaleFactor> <FalseEasting> <FalseNorthing> <Over> <Geoc> <IsSouth> <Zone>
//                           the entry's srcProjection, every value Reproject reads from it
//   tostring <v> <s>        s = v.ToString(), the runtime's default double format
//   geokey "<file>" <id> u16|string|double <value>
//                           geotiffdata.GeoKeys, in key order; strings with \ and " escaped
//   dem "<line>"            the line DEM (temp.cs:903-904) builds for the entry, CRLF dropped
//   alt <lat> <lng> <currenttype> <alt> "<altsource>" | throws <type> "<message>"
//       srtm <currenttype> <alt> "<altsource>"
//                           GeoTiff.getAltitude(lat, lng), then srtm.getAltitude(lat, lng)
// Doubles are G17 in the invariant culture, which always round-trips.

using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
using DotSpatial.Projections;
using GMap.NET;
using MissionPlanner.Utilities;

public static class GeoTiffOracle
{
    static readonly CultureInfo Inv = CultureInfo.InvariantCulture;
    static StreamWriter Out;

    public static int Main(string[] args)
    {
        Thread.CurrentThread.CurrentCulture = Inv;
        Thread.CurrentThread.CurrentUICulture = Inv;
        if (args.Length != 3)
        {
            Console.Error.WriteLine("usage: GeoTiffOracle <testdata/geotiff> <scratch dir> <out.txt>");
            return 2;
        }
        try
        {
            Oracle(args[0], args[1], args[2]);
        }
        catch (Exception ex)
        {
            // No partial golden data: a step that fails fails the whole regeneration.
            Console.Error.WriteLine("GeoTiffOracle: " + ex);
            Environment.Exit(1);
        }
        // srtm's queue thread is still running; it is not the harness's to wait for.
        Environment.Exit(0);
        return 0;
    }

    static string G(double v) { return v.ToString("G17", Inv); }

    static string Q(string s)
    {
        if (s == null) return "null";
        return "\"" + s.Replace("\\", "\\\\").Replace("\"", "\\\"").Replace("\r", "\\r")
            .Replace("\n", "\\n") + "\"";
    }

    static string Proj(string kind, string arg, Func<ProjectionInfo> make)
    {
        try
        {
            var p = make();
            return string.Format("{0} {1} ok {2} {3} {4}", kind, arg, Q(p.Name),
                Q(p.Transform == null ? null : p.Transform.Name), Q(p.ToProj4String()));
        }
        catch (Exception ex)
        {
            return string.Format("{0} {1} throws {2}", kind, arg, ex.GetType().Name);
        }
    }

    // The next double toward +infinity and toward -infinity.
    static double Up(double v)
    {
        if (v == 0) return double.Epsilon;
        long bits = BitConverter.DoubleToInt64Bits(v);
        return BitConverter.Int64BitsToDouble(v > 0 ? bits + 1 : bits - 1);
    }

    static double Down(double v) { return -Up(-v); }

    // A pseudo-random stream, only to spread points over an area; the points are written out, so
    // nothing has to reproduce it.
    static ulong seed = 0x6e0717ff;
    static double Next()
    {
        seed = seed * 6364136223846793005UL + 1442695040888963407UL;
        return (seed >> 11) * (1.0 / (1UL << 53));
    }

    static void Alt(double lat, double lng)
    {
        var sb = new StringBuilder();
        sb.Append("alt ").Append(G(lat)).Append(' ').Append(G(lng)).Append(' ');
        try
        {
            var r = GeoTiff.getAltitude(lat, lng);
            sb.Append(r.currenttype).Append(' ').Append(G(r.alt)).Append(' ').Append(Q(r.altsource));
        }
        catch (Exception ex)
        {
            sb.Append("throws ").Append(ex.GetType().Name).Append(' ').Append(Q(ex.Message));
        }
        var s = srtm.getAltitude(lat, lng);
        sb.Append(" srtm ").Append(s.currenttype).Append(' ').Append(G(s.alt)).Append(' ')
            .Append(Q(s.altsource));
        Out.WriteLine(sb.ToString());
    }

    static int[] EpsgCodes()
    {
        var codes = new List<int> { 0, 4326, 4283, 4258, 4269, 4167, 4019, 4030, 4322, 4979, 32767 };
        for (int c = 32601; c <= 32760; c++) codes.Add(c);
        for (int c = 3038; c <= 3051; c++) codes.Add(c);
        for (int c = 25828; c <= 25838; c++) codes.Add(c);
        for (int c = 28348; c <= 28358; c++) codes.Add(c);
        return codes.ToArray();
    }

    static void Oracle(string data, string cache, string outPath)
    {
        Directory.CreateDirectory(cache);
        if (Directory.GetFileSystemEntries(cache).Length != 0)
            throw new Exception(cache + " is not empty");
        var names = Directory.GetFiles(data, "*.tif").Select(Path.GetFileName)
            .OrderBy(n => n, StringComparer.Ordinal).ToList();
        if (names.Count == 0) throw new Exception("no .tif in " + data);
        foreach (var name in names)
            File.Copy(Path.Combine(data, name), Path.Combine(cache, name));

        // Before anything touches GeoTiff, whose static constructor lists this directory.
        srtm.datadirectory = cache;
        srtm.baseurl1sec = "http://127.0.0.1:9/SRTM1/";
        srtm.baseurl = "http://127.0.0.1:9/SRTM3/";
        GMaps.Instance.Mode = AccessMode.CacheOnly;

        using (Out = new StreamWriter(outPath, false, new UTF8Encoding(false)))
        {
            Out.NewLine = "\n";

            // What DotSpatial does with the codes LoadFile's branches can hand it.
            foreach (var code in EpsgCodes())
                Out.WriteLine(Proj("epsg", code.ToString(Inv), () => ProjectionInfo.FromEpsgCode(code)));
            // The geographic codes DotSpatial defines as latitude and longitude on WGS 1984 or GRS
            // 1980 (the same equatorial radius) with no datum shift but the zero one: every code
            // from 4000 to 4999 whose definition is one of these four.
            var forms = new[] { " +proj=longlat +datum=WGS84 +no_defs", " +proj=longlat +ellps=WGS84 +no_defs",
                " +proj=longlat +ellps=GRS80 +no_defs", " +proj=longlat +towgs84=0,0,0 +ellps=GRS80 +no_defs" };
            var byForm = forms.ToDictionary(f => f, f => new List<int>());
            for (int code = 4000; code <= 4999; code++)
            {
                string p4;
                try { p4 = ProjectionInfo.FromEpsgCode(code).ToProj4String(); }
                catch (Exception) { continue; }
                if (byForm.ContainsKey(p4)) byForm[p4].Add(code);
            }
            foreach (var f in forms)
                Out.WriteLine("geog {0}{1}", Q(f), string.Concat(byForm[f].Select(c => " " + c.ToString(Inv))));
            foreach (var s in new[] {
                         "+proj=utm +zone=55 +ellps=WGS84 +datum=WGS84 +units=m +no_defs ",
                         "+proj=utm +zone=-55 +ellps=WGS84 +datum=WGS84 +units=m +no_defs ",
                         "+proj=utm +zone=90 +ellps=WGS84 +datum=WGS84 +units=m +no_defs ",
                         "+proj=utm +zone=32 +ellps=GRS80 +units=m +no_defs ",
                         "+proj=stere +lat_ts=70 +lat_0=90 +lon_0=-45 +x_0=0 +y_0=0 +ellps=WGS84 +datum=WGS84 +units=m +no_defs " })
                Out.WriteLine(Proj("proj4", Q(s), () => ProjectionInfo.FromProj4String(s)));

            // The polar stereographic case formats GeoKeys' doubles with ToString() into a proj4
            // string; what the runtime's default format keeps of a double.
            foreach (var v in new[] { 70.0, -71.0, 70.123456789012345, 0.1 + 0.2, -45.5, 1e-7, 123456789012345678.0 })
                Out.WriteLine("tostring {0} {1}", G(v), v.ToString());

            // Reprojection itself, both ways, for the codes the test files use.
            var wgs = KnownCoordinateSystems.Geographic.World.WGS1984;
            var probes = new[] {
                new { code = 4326, lng = 149.123456789, lat = -35.123456789 },
                new { code = 4283, lng = 152.05, lat = -30.05 },
                new { code = 4019, lng = 154.05, lat = -30.05 },
                new { code = 32755, lng = 149.1, lat = -35.3 },
                new { code = 32755, lng = 141.0, lat = -0.001 },
                new { code = 32755, lng = 153.9, lat = -79.0 },
                new { code = 28355, lng = 149.2, lat = -35.2 },
                new { code = 25832, lng = 9.0, lat = 49.6 },
                new { code = 3044, lng = 9.3, lat = 49.8 },
                new { code = 32633, lng = 13.5, lat = 52.3 },
                new { code = 32633, lng = 21.0, lat = 70.0 } };
            foreach (var p in probes)
            {
                var proj = ProjectionInfo.FromEpsgCode(p.code);
                var xy = new[] { p.lng, p.lat };
                Reproject.ReprojectPoints(xy, null, wgs, proj, 0, 1);
                Out.WriteLine("fwd {0} {1} {2} {3} {4}", p.code, G(p.lng), G(p.lat), G(xy[0]), G(xy[1]));
                var back = new[] { xy[0], xy[1] };
                Reproject.ReprojectPoints(back, null, proj, wgs, 0, 1);
                Out.WriteLine("inv {0} {1} {2} {3} {4}", p.code, G(xy[0]), G(xy[1]), G(back[0]), G(back[1]));
            }

            // The static constructor: every *.tif in srtm.datadirectory through LoadFile.
            List<GeoTiff.geotiffdata> index = GeoTiff.index;
            lock (index)
                index.Sort((a, b) => string.CompareOrdinal(
                    Path.GetFileName(a.FileName).ToUpperInvariant(),
                    Path.GetFileName(b.FileName).ToUpperInvariant()));

            // FromEsriString on the citations LoadFile hands it, and on an empty one.
            var citations = new List<string> { "" };
            foreach (var e in index)
                if (e.GeoKeys.ContainsKey(GeoTiff.geotiffdata.GKID.PCSCitationGeoKey))
                    citations.Add(e.GeoKeys[GeoTiff.geotiffdata.GKID.PCSCitationGeoKey].ToString());
            foreach (var s in citations.Distinct())
                Out.WriteLine(Proj("esri", Q(s), () => ProjectionInfo.FromEsriString(s)));

            // The ones it left out, and why: LoadFile again, which throws again.
            foreach (var name in names)
            {
                if (index.Any(e => Path.GetFileName(e.FileName) == name)) continue;
                try
                {
                    new GeoTiff.geotiffdata().LoadFile(Path.Combine(cache, name));
                    throw new Exception(name + " loaded on the second try");
                }
                catch (Exception ex)
                {
                    if (ex.Message.EndsWith("second try")) throw;
                    Out.WriteLine("skipped {0} {1} {2}", Q(name), ex.GetType().Name, Q(ex.Message));
                }
            }

            int n = 0;
            foreach (var e in index)
            {
                var name = Path.GetFileName(e.FileName);
                Out.WriteLine("index {0} {1} {2} {3} {4} {5} {6} {7} {8} {9} {10} {11} {12} {13} {14} {15} {16} {17} {18} {19}",
                    n++, Q(name), e.width, e.height, e.bits, e.type, G(e.Area.Lat), G(e.Area.Lng),
                    G(e.Area.WidthLng), G(e.Area.HeightLat), G(e.x), G(e.y), G(e.xscale), G(e.yscale),
                    e.ProjectedCSTypeGeoKey, e.GTRasterTypeGeoKey, e.ProjCoordTransGeoKey, e.UTMZone,
                    Q(e.srcProjection == null ? null : e.srcProjection.Name),
                    Q(e.srcProjection == null || e.srcProjection.Transform == null ? null : e.srcProjection.Transform.Name));
                foreach (var kv in e.GeoKeys.OrderBy(kv => (int)kv.Key))
                {
                    var v = kv.Value;
                    string kind = v is ushort ? "u16" : v is string ? "string" : v is double ? "double" : v.GetType().Name;
                    string text = v is double ? G((double)v) : v is string ? Q((string)v) : Convert.ToString(v, Inv);
                    Out.WriteLine("geokey {0} {1} {2} {3}", Q(name), (int)kv.Key, kind, text);
                }
                // What DotSpatial made of the projection: every value Reproject reads.
                var sp = e.srcProjection;
                if (sp != null)
                {
                    var d = sp.GeographicInfo.Datum;
                    Func<double?, string> N = v => v.HasValue ? G(v.Value) : "null";
                    Out.WriteLine("param {0} {1} {2} {3} {4} {5} {6} {7} {8} {9} {10} {11} {12} {13} {14} {15} {16} {17} {18} {19} {20}",
                        Q(name), sp.IsLatLon, sp.Transform == null ? "null" : sp.Transform.Proj4Name,
                        d.DatumType, d.ToWGS84 == null ? "null" : string.Join(",", d.ToWGS84.Select(G)),
                        G(d.Spheroid.EquatorialRadius), G(d.Spheroid.PolarRadius),
                        G(sp.GeographicInfo.Unit.Radians), G(sp.GeographicInfo.Meridian.Longitude),
                        sp.Unit == null ? "null" : G(sp.Unit.Meters), N(sp.CentralMeridian), G(sp.Lam0),
                        N(sp.LatitudeOfOrigin), G(sp.Phi0), G(sp.ScaleFactor), N(sp.FalseEasting),
                        N(sp.FalseNorthing), sp.Over, sp.Geoc, sp.IsSouth,
                        sp.Zone.HasValue ? sp.Zone.Value.ToString(Inv) : "null");
                }
                // temp.cs:903-904, as DEM formats it, with the path made relative to the directory.
                var line = String.Format("{0} = {1} = {2}*{3} {4} {8}\r\n", name, e.Area, e.width, e.height, e.bits,
                    e.xscale, e.yscale, e.zscale, e.srcProjection?.Name ?? e.srcProjection?.Transform?.Name);
                Out.WriteLine("dem {0}", Q(line.TrimEnd('\r', '\n')));
            }

            // The lookups, file by file.
            foreach (var e in index)
            {
                var area = e.Area;
                double top = area.Top, bottom = area.Bottom, left = area.Left, right = area.Right;
                double h = area.HeightLat, w = area.WidthLng;
                if (h == 0 || w == 0)
                {
                    // An empty area holds nothing; one point where its corner is.
                    Alt(top, left);
                    continue;
                }
                foreach (var fy in new[] { 0.1, 0.5, 0.9 })
                foreach (var fx in new[] { 0.1, 0.5, 0.9 })
                    Alt(top - fy * h, left + fx * w);
                for (int k = 0; k < 12; k++)
                    Alt(top - Next() * h, left + Next() * w);
                var lats = new[] { top, Down(top), bottom, Up(bottom), top + h * 1e-3, bottom - h * 1e-3 };
                var lngs = new[] { left, Up(left), right, Down(right), left - w * 1e-3, right + w * 1e-3 };
                foreach (var lat in lats)
                foreach (var lng in lngs)
                    Alt(lat, lng);
                // On the samples, where the bilinear weights are 0 and 1, for a file whose rows and
                // columns are latitude and longitude.
                if (e.srcProjection == null || e.srcProjection.IsLatLon)
                {
                    foreach (var r in new[] { 0, 1, e.height / 2, e.height - 2, e.height - 1 })
                    foreach (var c in new[] { 0, 1, e.width / 2, e.width - 2, e.width - 1 })
                        Alt(e.y - r * (e.height * e.yscale) / (e.height - 1),
                            e.x + c * (e.width * e.xscale) / (e.width - 1));
                }
            }

            // Around geo_west_area's no-data block (rows 10-11, columns 4-6) and its threshold
            // cells (rows 2-3, columns 15-16), between samples and on them.
            var west = index.First(e => Path.GetFileName(e.FileName) == "geo_west_area.tif");
            foreach (var cell in new[] { new[] { 10.0, 4.0 }, new[] { 9.5, 3.5 }, new[] { 11.5, 6.5 },
                         new[] { 12.0, 7.0 }, new[] { 2.0, 15.0 }, new[] { 2.5, 15.5 }, new[] { 2.0, 15.5 },
                         new[] { 2.5, 15.0 }, new[] { 2.25, 15.75 }, new[] { 3.0, 16.0 } })
                Alt(west.y - cell[0] * (west.height * west.yscale) / (west.height - 1),
                    west.x + cell[1] * (west.width * west.xscale) / (west.width - 1));

            // The edge geo_west_area and geo_east_point share.
            var east = index.First(e => Path.GetFileName(e.FileName) == "geo_east_point.tif");
            foreach (var lng in new[] { west.Area.Right, Down(west.Area.Right), Up(west.Area.Right),
                         east.Area.Left, Down(east.Area.Left), Up(east.Area.Left) })
            foreach (var lat in new[] { -35.05, -35.1 })
                Alt(lat, lng);

            // Outside every file.
            Alt(0, 0);
            Alt(-90, 180);
            Alt(double.NaN, 149.1);
        }
    }
}
