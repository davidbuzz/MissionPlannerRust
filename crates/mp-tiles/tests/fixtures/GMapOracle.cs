// What GMap.NET itself says about its providers: the differential oracle for
// crates/mp-tiles/tests/providers.rs, whose fixture gmap-oracle.tsv is this program's output.
//
// It loads Mission Planner's GMap.NET.Core.dll, walks GMapProviders.List - the list
// FlightPlanner.cs:177 hands to comboBoxMapType - and for the providers it names below calls each
// one's private MakeTileImageUrl through reflection, which is the function GetTileImage builds its
// request from. Nothing is fetched: OnInitialized is never called, so every version is the
// hard-coded one, and no provider is given to a map control.
//
// Regenerate (mono 6.12; the DLL and BouncyCastle.Cryptography.dll from a Mission Planner install,
// here the copy in ~/Downloads):
//
//   mcs -r:$MP/GMap.NET.Core.dll -r:/usr/lib/mono/4.5/Facades/netstandard.dll GMapOracle.cs
//   MONO_PATH=$MP mono GMapOracle.exe > gmap-oracle.tsv
//
// The output is tab-separated, one record per line:
//   year     <DateTime.Today.Year, which every Copyright was formatted with>
//   list     <index> <Name> <MaxZoom or null> <RefererUrl> <Overlays' Names joined by +>
//   provider <Name> <Version> <Copyright>
//   url      <Name> <x> <y> <zoom> <url>
using System;
using System.Reflection;
using GMap.NET;
using GMap.NET.MapProviders;

class GMapOracle
{
    static void Main()
    {
        Console.WriteLine("year\t" + DateTime.Today.Year);
        int i = 0;
        foreach (var p in GMapProviders.List)
        {
            var overlays = p.Overlays == null ? "" : string.Join("+", Array.ConvertAll(p.Overlays, o => o.Name));
            var maxZoom = p.MaxZoom.HasValue ? p.MaxZoom.Value.ToString() : "null";
            Console.WriteLine("list\t" + i + "\t" + p.Name + "\t" + maxZoom + "\t" + p.RefererUrl + "\t" + overlays);
            i++;
        }

        var tiles = new long[][] {
            new long[] { 0, 0, 0 }, new long[] { 1, 2, 5 }, new long[] { 3, 1, 2 }, new long[] { 2, 1, 2 },
            new long[] { 7, 5, 3 }, new long[] { 2, 3, 2 }, new long[] { 511, 340, 10 },
            new long[] { 15089, 9814, 14 }, new long[] { 59922, 39658, 16 }, new long[] { 12345, 67890, 17 },
            new long[] { 120000, 90000, 18 }, new long[] { 200000, 150000, 19 },
            new long[] { 3879839, 2430784, 22 }, new long[] { 4194303, 4194303, 22 },
        };
        foreach (var name in new string[] { "GoogleMap", "GoogleSatelliteMap", "GoogleHybridMap", "GoogleTerrainMap", "BingMap", "BingSatelliteMap", "BingHybridMap" })
        {
            var p = GMapProviders.List.Find(x => x.Name == name);
            var version = p.GetType().GetField("Version").GetValue(p);
            Console.WriteLine("provider\t" + name + "\t" + version + "\t" + p.Copyright);
            var make = p.GetType().GetMethod("MakeTileImageUrl", BindingFlags.NonPublic | BindingFlags.Instance);
            foreach (var t in tiles)
            {
                // GetTileImage passes GMapProvider.LanguageStr, which Mission Planner leaves at "en".
                var url = (string)make.Invoke(p, new object[] { new GPoint(t[0], t[1]), (int)t[2], GMapProvider.LanguageStr });
                Console.WriteLine("url\t" + name + "\t" + t[0] + "\t" + t[1] + "\t" + t[2] + "\t" + url);
            }
        }
    }
}
