// Headless terrain oracle: Mission Planner's own srtm.getAltitude (ExtLibs/Utilities/srtm.cs), for
// crates/mp-terrain/tests/oracle.rs.
//
// Runs the class out of the msbuild output of ExtLibs/Utilities - the same build regen-grid.sh makes
// - with srtm.datadirectory pointed at a scratch copy of testdata/srtm/ and its two servers
// (srtm.baseurl1sec, srtm.baseurl) pointed at a web server this harness runs on 127.0.0.1, so the
// download queue is driven for real - listings fetched and cached, the .hgt.zip fetched, written and
// extracted, the ocean rule reached, a failing server left at the head of the queue - without
// anything leaving the machine. Every answer is written to one file, in the order it was asked, and
// the Rust replays the same sequence against the same inputs.
//
// Build:  see regen-srtm.sh (mcs against the msbuild output of ExtLibs/Utilities)
// Run:    mono SrtmOracle.exe mksynthetic <out.hgt.zip>
//         mono SrtmOracle.exe oracle <testdata/srtm> <empty scratch dir> <out oracle.txt>
//
// mksynthetic  writes N00W001.hgt, a 3-arc-second (1201x1201) tile of known values - a slope with
//              a void block, a lone void, cells either side of the -1000 threshold and the two
//              extremes of a short - zipped as the server zips a tile. Deterministic in content; the
//              zip carries a timestamp, so regen-srtm.sh makes it only when it is missing.
// oracle       the scratch directory is filled as a user's cache would be: N00W001.hgt extracted
//              from its zip with FastZip.ExtractZip (what srtm.gethgt does, srtm.cs:664-667), and
//              the other fixtures copied. S28E153.hgt is not copied: the harness's server serves
//              testdata/srtm/S28E153.hgt.zip and srtm's own queue downloads it.
//
// oracle.txt, one record per line, fields separated by single spaces:
//   format <v> <s>          s = Math.Round(v, 0).ToString("00"), srtm.cs:283-284's expression
//   cast <v> <i>            i = (int)Math.Floor(v), srtm.cs:106-107's expression
//   fdiv <a> <b> <i>        i = (int)(a / b) for floats a and b parsed from the text, srtm.cs:348-349
//   mode cacheonly|server   GMaps.Instance.Mode set to CacheOnly or ServerAndCache
//   baseurl <url>           srtm.baseurl set; {base} stands for http://127.0.0.1:<port>, the
//                           harness's server, and {broken} for a second port that reads a request
//                           and resets the connection
//   alt <lat> <lng> <zoom> <currenttype> <alt> "<altsource>"   one srtm.getAltitude call
//   queue [<name>...]       the contents of srtm's private download queue, in order
//   run                     the harness waited for the queue thread to take the head of the queue
//                           (until the queue emptied, or the server saw the next request that
//                           fails)
//   requests [<path>...]    the paths the server was asked for since the last `requests`
//   file <name> <length>    a file in the cache directory, in ordinal order of name
//   listing <name> <length> <lines> "<first>" "<last>"   a listing file srtm cached there, by
//                           File.ReadAllLines, with the server's address as {base}
// Doubles are G17 in the invariant culture: it always round-trips, which "R" does not on every
// runtime. The port is always five digits, so a listing file's length does not depend on it.

using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Reflection;
using System.Text;
using System.Threading;
using GMap.NET;
using ICSharpCode.SharpZipLib.Zip;
using MissionPlanner.Utilities;

public static class SrtmOracle
{
    static readonly CultureInfo Inv = CultureInfo.InvariantCulture;
    static StreamWriter Out;
    static string Base;
    static string Broken;
    static string Cache;
    static byte[] TileZip;
    static readonly List<string> Requests = new List<string>();
    static int RequestsPrinted;

    public static int Main(string[] args)
    {
        Thread.CurrentThread.CurrentCulture = Inv;
        Thread.CurrentThread.CurrentUICulture = Inv;
        try
        {
            if (args.Length == 2 && args[0] == "mksynthetic")
            {
                MkSynthetic(args[1]);
                return 0;
            }
            if (args.Length == 4 && args[0] == "oracle")
            {
                Oracle(args[1], args[2], args[3]);
                // The queue thread and the server thread are still running; neither is the
                // harness's to wait for.
                Environment.Exit(0);
            }
        }
        catch (Exception ex)
        {
            // No partial golden data: a step that fails fails the whole regeneration.
            Console.Error.WriteLine("SrtmOracle: " + ex);
            Environment.Exit(1);
        }
        Console.Error.WriteLine("usage: SrtmOracle mksynthetic <out.hgt.zip>");
        Console.Error.WriteLine("       SrtmOracle oracle <testdata/srtm> <scratch dir> <out.txt>");
        return 2;
    }

    // ---------------------------------------------------------------- the synthetic tile

    // Row 0 is the north edge and each row runs west to east, as an SRTM .hgt does. Duplicated in
    // crates/mp-terrain/tests/oracle.rs only as a check that the committed zip is this.
    public static short Synthetic(int col, int row)
    {
        if (col >= 600 && col <= 602 && row >= 600 && row <= 602) return -32768;
        if (col == 100 && row == 1100) return -32768;
        if (row == 300 && col == 900) return -1000;
        if (row == 300 && col == 901) return -1001;
        if (row == 301 && col == 900) return -999;
        if (row == 301 && col == 901) return -1000;
        if (row == 50 && col == 50) return 32767;
        if (row == 50 && col == 51) return -32767;
        return (short)((col * 3 + row * 7) % 2000 - 200);
    }

    static void MkSynthetic(string zipPath)
    {
        var dir = Path.Combine(Path.GetTempPath(), "srtm-synthetic-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(dir);
        try
        {
            var bytes = new byte[1201 * 1201 * 2];
            int at = 0;
            for (int row = 0; row < 1201; row++)
            for (int col = 0; col < 1201; col++)
            {
                short v = Synthetic(col, row);
                bytes[at++] = (byte)((v >> 8) & 0xff);
                bytes[at++] = (byte)(v & 0xff);
            }
            File.WriteAllBytes(Path.Combine(dir, "N00W001.hgt"), bytes);
            if (File.Exists(zipPath)) File.Delete(zipPath);
            new FastZip().CreateZip(zipPath, dir, false, "");
        }
        finally
        {
            Directory.Delete(dir, true);
        }
    }

    // ---------------------------------------------------------------- the server

    // The listings, byte for byte what crates/mp-terrain/tests/oracle.rs serves. SRTM3's index holds
    // everything getListing filters (srtm.cs:723-728), a HREF in capitals, a single-quoted href the
    // pattern does not match, an empty one, the README getListing pads (srtm.cs:742-743) and a
    // "bios" entry it never fetches (srtm.cs:682-683). With SRTM1 that is nine listings (srtm.cs:621,
    // list.Count >= 9), and the names in them total 38001, one more than the rule needs.
    const string Srtm3Index =
        "<html><body><h1>Index of /SRTM3/</h1>\n" +
        "<a href=\"../\">Parent Directory</a>\n" +
        "<a href=\"Region1/\">Region1/</a>\n" +
        "<a href=\"Region2/\">Region2/</a>\n" +
        "<A HREF=\"Region3/\">Region3/</A>\n" +
        "<a href=\"Region4/\">Region4/</a>\n" +
        "<a href=\"Region5/\">Region5/</a>\n" +
        "<a href=\"Region6/\">Region6/</a>\n" +
        "<a href=\"http://elsewhere.invalid/Region7/\">elsewhere</a>\n" +
        "<a href=\"/srtm/version2_1/\">version 2.1</a>\n" +
        "<a href=\"README.txt\">README.txt</a>\n" +
        "<a href=\"bios\">bios</a>\n" +
        "<a href='single/'>single quotes</a>\n" +
        "<a href=\"\">empty</a>\n" +
        "</body></html>\n";

    const string Readme = "Nothing in here is a link.\n";

    static string Listing(IEnumerable<string> names)
    {
        var sb = new StringBuilder("<html><body>\n");
        foreach (var name in names)
            sb.Append("<a href=\"").Append(name).Append("\">").Append(name).Append("</a>\n");
        return sb.Append("</body></html>\n").ToString();
    }

    static IEnumerable<string> Srtm1Names()
    {
        for (int i = 0; i < 20000; i++)
        {
            yield return "X" + i.ToString("00000", Inv) + ".hgt.zip";
            if (i == 9999) yield return "S28E153.hgt.zip";
        }
    }

    static IEnumerable<string> RegionNames(int region)
    {
        for (int i = 0; i < 3000; i++)
            yield return "Y" + region.ToString(Inv) + "_" + i.ToString("0000", Inv) + ".hgt.zip";
    }

    static byte[] Route(string path)
    {
        if (path == "/SRTM3/") return Encoding.UTF8.GetBytes(Srtm3Index);
        if (path == "/SRTM3/README.txt") return Encoding.UTF8.GetBytes(Readme);
        if (path == "/SRTM1/") return Encoding.UTF8.GetBytes(Listing(Srtm1Names()));
        if (path == "/SRTM1/S28E153.hgt.zip") return TileZip;
        if (path == "/EMPTY/") return new byte[0];
        for (int region = 1; region <= 6; region++)
            if (path == "/SRTM3/Region" + region + "/")
                return Encoding.UTF8.GetBytes(Listing(RegionNames(region)));
        return null;
    }

    static void Serve(HttpListenerContext ctx)
    {
        var path = ctx.Request.RawUrl;
        lock (Requests) Requests.Add(path);
        var body = Route(path);
        if (body == null)
        {
            ctx.Response.StatusCode = 404;
            body = Encoding.UTF8.GetBytes("<html><body>not found</body></html>\n");
        }
        ctx.Response.ContentLength64 = body.Length;
        ctx.Response.OutputStream.Write(body, 0, body.Length);
        ctx.Response.OutputStream.Close();
    }

    static int FreePort()
    {
        int port;
        do
        {
            var probe = new TcpListener(IPAddress.Loopback, 0);
            probe.Start();
            port = ((IPEndPoint)probe.LocalEndpoint).Port;
            probe.Stop();
        } while (port < 10000);
        return port;
    }

    // Reads the request line, logs its path and resets the connection without answering, so
    // HttpClient.GetAsync throws. (HttpListenerResponse.Abort does not do that under mono: the client
    // sees an empty 200, which is what /EMPTY/ serves on purpose.)
    static void StartBrokenServer()
    {
        var listener = new TcpListener(IPAddress.Loopback, FreePort());
        listener.Start();
        Broken = "http://127.0.0.1:" + ((IPEndPoint)listener.LocalEndpoint).Port.ToString(Inv);
        new Thread(() =>
        {
            while (true)
            {
                TcpClient client;
                try { client = listener.AcceptTcpClient(); }
                catch { return; }
                try
                {
                    var line = new StreamReader(client.GetStream()).ReadLine() ?? "";
                    var parts = line.Split(' ');
                    lock (Requests) Requests.Add(parts.Length > 1 ? parts[1] : line);
                    client.Client.LingerState = new LingerOption(true, 0);
                    client.Close();
                }
                catch (Exception ex) { Console.Error.WriteLine("broken: " + ex.Message); }
            }
        }) { IsBackground = true }.Start();
    }

    static void StartServer()
    {
        int port = FreePort();
        Base = "http://127.0.0.1:" + port.ToString(Inv);
        var listener = new HttpListener();
        listener.Prefixes.Add(Base + "/");
        listener.Start();
        new Thread(() =>
        {
            while (true)
            {
                HttpListenerContext ctx;
                try { ctx = listener.GetContext(); }
                catch { return; }
                try { Serve(ctx); }
                catch (Exception ex) { Console.Error.WriteLine("serve: " + ex.Message); }
            }
        }) { IsBackground = true }.Start();
    }

    // ---------------------------------------------------------------- the records

    static string G(double v) { return v.ToString("G17", Inv); }

    static void Alt(double lat, double lng, double zoom = 16)
    {
        var r = srtm.getAltitude(lat, lng, zoom);
        Out.WriteLine("alt {0} {1} {2} {3} {4} \"{5}\"", G(lat), G(lng), G(zoom), r.currenttype,
            G(r.alt), r.altsource);
    }

    static List<string> Queue()
    {
        var t = typeof(srtm);
        var q = (List<string>)t.GetField("queue", BindingFlags.NonPublic | BindingFlags.Static).GetValue(null);
        var l = t.GetField("objlock", BindingFlags.NonPublic | BindingFlags.Static).GetValue(null);
        lock (l) return q.ToList();
    }

    static void PrintQueue() { Out.WriteLine("queue" + string.Concat(Queue().Select(x => " " + x))); }

    static void PrintRequests()
    {
        lock (Requests)
        {
            Out.WriteLine("requests" + string.Concat(Requests.Skip(RequestsPrinted).Select(x => " " + x)));
            RequestsPrinted = Requests.Count;
        }
    }

    static int CountRequests(string path) { lock (Requests) return Requests.Count(x => x == path); }

    static void Mode(AccessMode mode)
    {
        GMaps.Instance.Mode = mode;
        Out.WriteLine("mode " + (mode == AccessMode.CacheOnly ? "cacheonly" : "server"));
    }

    static void WaitFor(Func<bool> done, string what)
    {
        var until = DateTime.UtcNow.AddSeconds(180);
        while (!done())
        {
            if (DateTime.UtcNow > until) throw new TimeoutException("waiting for " + what);
            Thread.Sleep(50);
        }
        Out.WriteLine("run");
    }

    static void PrintFiles()
    {
        foreach (var path in Directory.GetFiles(Cache).OrderBy(Path.GetFileName, StringComparer.Ordinal))
        {
            var name = Path.GetFileName(path);
            var length = new FileInfo(path).Length;
            if (name.EndsWith(".hgt") || name.EndsWith(".zip") || name.EndsWith(".asc"))
            {
                Out.WriteLine("file {0} {1}", name, length);
                continue;
            }
            var lines = File.ReadAllLines(path);
            Func<string, string> norm = s => s.Replace(Base, "{base}");
            Out.WriteLine("listing {0} {1} {2} \"{3}\" \"{4}\"", name, length, lines.Length,
                lines.Length > 0 ? norm(lines[0]) : "", lines.Length > 0 ? norm(lines[lines.Length - 1]) : "");
        }
    }

    // A pseudo-random stream, only to spread points over a tile; the points are written out, so
    // nothing has to reproduce it.
    static ulong seed = 0x5eed;
    static double Next()
    {
        seed = seed * 6364136223846793005UL + 1442695040888963407UL;
        return (seed >> 11) * (1.0 / (1UL << 53));
    }

    static double Down(double v) { return BitConverter.Int64BitsToDouble(BitConverter.DoubleToInt64Bits(v) - (v > 0 ? 1 : -1)); }

    // ---------------------------------------------------------------- the run

    static void Oracle(string data, string cache, string outPath)
    {
        Cache = cache;
        Directory.CreateDirectory(cache);
        if (Directory.GetFileSystemEntries(cache).Length != 0)
            throw new Exception(cache + " is not empty");
        new FastZip().ExtractZip(Path.Combine(data, "N00W001.hgt.zip"), cache, "");
        foreach (var name in new[] { "N01W001.hgt", "N02W001.hgt", "srtm_41_10.asc", "srtm_41_11.asc",
                     "srtm_42_10.asc", "srtm_42_11.asc" })
            File.Copy(Path.Combine(data, name), Path.Combine(cache, name));
        TileZip = File.ReadAllBytes(Path.Combine(data, "S28E153.hgt.zip"));
        StartServer();
        StartBrokenServer();

        // MainV2.cs:737 sets it with no trailing separator; srtm puts one between (srtm.cs:179).
        // Set before the first getAltitude, which is when GeoTiff and DTED index the directory.
        srtm.datadirectory = cache;
        srtm.baseurl1sec = Base + "/SRTM1/";
        srtm.baseurl = Base + "/SRTM3/";

        using (Out = new StreamWriter(outPath, false, new UTF8Encoding(false)))
        {
            Out.NewLine = "\n";

            // The runtime's own arithmetic in srtm's two expressions that depend on it.
            foreach (var v in new[] { 0.0, -0.0, 0.5, 1.5, 2.5, -0.5, -0.3, -1.5, -2.5, -5.68,
                         0.49999999999999994, 0.50000000000000011, 1.4999999999999998, 10.5, 36.5,
                         37.5, 40.5, 41.5, 72.5, 1e-300, -1e-300 })
                Out.WriteLine("format {0} {1}", G(v), Math.Round(v, 0).ToString("00"));
            foreach (var v in new[] { double.NaN, double.PositiveInfinity, double.NegativeInfinity, 1e10,
                         -1e10, 2147483647.5, 2147483648.0, -2147483648.5, -2147483649.0, 4294967331.5,
                         -0.0, -1e-300, 90.5, -90.5 })
                Out.WriteLine("cast {0} {1}", G(v), ((int)Math.Floor(v)).ToString(Inv));
            // srtm.cs:348-349 divides a float by a float and truncates: whether the runtime does
            // that in single precision decides which row an ASCII lookup reads. Parsed, so the
            // compiler cannot fold them.
            foreach (var pair in new[] { "1 0.1", "0.3 0.1", "2.3 0.5", "0.7 0.000833333", "1.1 0.000833333",
                         "3 0.3", "0.6 0.2" })
            {
                var ab = pair.Split(' ');
                float a = float.Parse(ab[0], Inv), b = float.Parse(ab[1], Inv);
                Out.WriteLine("fdiv {0} {1} {2}", ab[0], ab[1], ((int)(a / b)).ToString(Inv));
            }

            Mode(AccessMode.CacheOnly);

            // N00W001: the synthetic 3-arc-second tile.
            for (int i = 0; i <= 10; i++)
            for (int j = 0; j <= 10; j++)
                Alt(i / 10.0, -1 + j / 10.0);
            for (int k = 0; k < 300; k++)
                Alt(Next(), -1 + Next());
            // Around each special cell, at and between samples.
            foreach (var cell in new[] { new[] { 601, 601 }, new[] { 600, 600 }, new[] { 100, 1100 },
                         new[] { 900, 300 }, new[] { 50, 50 } })
            foreach (var dr in new[] { -1.0, -0.5, 0.0, 0.25, 0.5, 1.0 })
            foreach (var dc in new[] { -1.0, -0.5, 0.0, 0.25, 0.5, 1.0 })
                Alt(1 - (cell[1] + dr) / 1200.0, -1 + (cell[0] + dc) / 1200.0);
            // Exactly between the four threshold cells, and a quarter of the way.
            Alt(1 - 300.5 / 1200.0, -1 + 900.5 / 1200.0);
            Alt(1 - 300.25 / 1200.0, -1 + 900.75 / 1200.0);
            // Edges: of the tile, of a double, and of the next tiles.
            foreach (var lat in new[] { 0.0, 1e-300, 0.5, Down(1.0), 1.0 - 1e-12, 1.0, -1e-17 })
            foreach (var lng in new[] { -1.0, Down(-1.0), -0.5, -1e-12, -1e-17, -4.9e-324, -0.0, 0.0 })
                Alt(lat, lng);

            // The wrong-length and empty tiles.
            Alt(1.5, -0.5);
            Alt(2.5, -0.5);

            // The ASCII grids. Each is read once per process (see the report in oracle.rs); every
            // repeat is written too.
            Alt(12.3, 21.2);
            Alt(12.3, 21.2);
            Alt(13.1, 20.6);
            Alt(5.2, 21.0);
            Alt(8.3, 27.7);
            Alt(11.0, 25.5);

            // Outside what a tile name can say.
            foreach (var p in new[] { new[] { 91.0, 0.5 }, new[] { -91.5, 0.5 }, new[] { 0.5, 181.0 },
                         new[] { 0.5, -181.5 }, new[] { 90.5, 0.5 }, new[] { -90.5, 180.5 },
                         new[] { double.NaN, 0.5 }, new[] { 0.5, double.NaN },
                         new[] { double.PositiveInfinity, 0.5 }, new[] { 0.5, double.NegativeInfinity },
                         new[] { 1e10, 0.5 }, new[] { 0.5, -1e10 } })
                Alt(p[0], p[1]);

            // A missing tile: CacheOnly, then a zoom below 7, queue nothing.
            Alt(5.5, 5.5);
            PrintQueue();
            Mode(AccessMode.ServerAndCache);
            Alt(5.5, 5.5, 6.99);
            PrintQueue();

            // A download: the tile is queued, answered Invalid while it is, fetched and extracted.
            Alt(-27.5, 153.5);
            PrintQueue();
            Alt(-27.5, 153.5);
            WaitFor(() => Queue().Count == 0, "S28E153 to download");
            PrintRequests();
            PrintQueue();
            PrintFiles();

            // S28E153: the real 1-arc-second tile, now in the cache.
            Mode(AccessMode.CacheOnly);
            for (int i = 0; i <= 10; i++)
            for (int j = 0; j <= 10; j++)
                Alt(-28 + i / 10.0, 153 + j / 10.0);
            for (int k = 0; k < 400; k++)
                Alt(-28 + Next(), 153 + Next());
            foreach (var lat in new[] { -28.0, -27.5, Down(-27.0), -27.000000001, -27.0 })
            foreach (var lng in new[] { 153.0, 153.5, Down(154.0), 153.999999999, 154.0 })
                Alt(lat, lng);
            for (int k = 0; k < 20; k++)
            {
                // Exactly on samples, where the weights are 0 and 1.
                int row = (int)(Next() * 3600), col = (int)(Next() * 3600);
                Alt(-27 - row / 3600.0, 153 + col / 3600.0);
            }
            Alt(-27.5, 153.5, 0);

            // The ocean rule: every listing walked, nothing matched.
            Mode(AccessMode.ServerAndCache);
            Alt(-40.5, -140.5, 6.99);
            PrintQueue();
            Alt(-40.5, -140.5, 7);
            PrintQueue();
            WaitFor(() => Queue().Count == 0, "S41W141 to be found in no listing");
            PrintRequests();
            PrintFiles();
            Alt(-40.5, -140.5);
            Alt(-40.9, -140.1, 1);

            // A server that answers with nothing: an empty listing is written, the tile is taken
            // off the queue as if it had been looked for, and the next question queues it again -
            // and fetches the listing again, because an empty one is not a cached one
            // (srtm.cs:692).
            srtm.baseurl = Base + "/EMPTY/";
            Out.WriteLine("baseurl {base}/EMPTY/");
            Alt(5.5, 5.5);
            PrintQueue();
            WaitFor(() => Queue().Count == 0, "the empty listing");
            PrintRequests();
            PrintQueue();
            Alt(5.5, 5.5);
            PrintQueue();
            WaitFor(() => Queue().Count == 0, "the empty listing again");
            PrintRequests();
            PrintFiles();

            // A server that drops the connection: GetAsync throws, getListing rethrows
            // (srtm.cs:748-751), and the tile stays at the head of the queue and blocks the next.
            srtm.baseurl = Broken + "/BROKEN/";
            Out.WriteLine("baseurl {broken}/BROKEN/");
            Alt(7.5, 7.5);
            PrintQueue();
            WaitFor(() => CountRequests("/BROKEN/") >= 1, "the first failure");
            Thread.Sleep(500);
            PrintRequests();
            PrintQueue();
            Alt(8.5, 8.5);
            PrintQueue();
            WaitFor(() => CountRequests("/BROKEN/") >= 2, "the second failure");
            Thread.Sleep(500);
            PrintRequests();
            PrintQueue();
            Alt(7.5, 7.5);
        }
    }
}
