// Headless Geo Reference Images oracle: Mission Planner's own GeoRefImageBase
// (ExtLibs/Utilities/GeoRefImageBase.cs), driven the way the form's two buttons drive it
// (GeoRef/georefimage.cs:140-250 "Process", 268-319 "GeoTag Images"), for crates/mp-georef.
//
// The form itself (GeoRef/georefimage.cs, in MissionPlanner.exe) is not driven: its constructor
// builds WinForms controls and reads MainV2.instance's map. Everything it does with the photos
// is a call into GeoRefImageBase, which is in MissionPlanner.Utilities.dll with no WinForms at
// all, so this harness is the form's click handlers with the controls replaced by arguments -
// each line cited - and GeoRefImageBase itself runs unchanged.
//
// Build:  see regen-georef.sh (mcs against the msbuild output of ExtLibs/Utilities)
// Run:    mono GeorefOracle.exe case <dir> <time|cam|trig> <log> <photos> [key=value ...]
//         mono GeorefOracle.exe estimate <out.txt> <log> <photos> [key=value ...]
//         mono GeorefOracle.exe mktrig <in.bin> <out.bin>
//         mono GeorefOracle.exe srtm <tile.hgt.zip> <dir>
//         mono GeorefOracle.exe bintolog <in.bin> <out.log>
//         mono GeorefOracle.exe roundtrip <out.txt>
//         mono GeorefOracle.exe phototimes <dir> <out.txt>
//         mono GeorefOracle.exe geotag <dir> <out dir> <lat> <lon> <alt>
//
// case      copies the photos (*.jpg) and the log into <dir>, then does what "Process" does in
//           that mode - doworkGPSOFFSET, doworkCAM or doworkTRIG, then CreateReportFiles into
//           <dir> - and what "GeoTag Images" does after it - WriteCoordinatesToImage for every
//           matched photo into <dir>/geotagged. It writes messages.txt, every AppendText the two
//           buttons make, and state.txt, what GeoRefImageBase holds afterwards (below), and
//           deletes the copies of the photos and the log. Keys, with the form's defaults:
//             amsl=1 (CHECK_AMSLAlt_Use, checked in the constructor, georefimage.cs:30-33)
//             lag=0 (TXT_shutterLag), minshutter=0.5 (num_minshutter), dropstart=0, dropend=0,
//             gps=GPS (chk_usegps2 unchecked), usecam=0 (chk_cammsg), offset=0 (TXT_offsetseconds),
//             rotation=90 hfov=200 vfov=130 (num_camerarotation, num_hfov, num_vfov, from
//             Georefimage.Designer.cs:199-252), camgpsalt=0 (chk_camusegpsalt),
//             triggpsalt=0 (chk_trigusergpsalt), basealt=0 (txt_basealt)
// estimate  "Estimate Offset" (georefimage.cs:258-266): EstimateOffset's lines and its answer.
// mktrig    camera.bin with every CAM message relabelled TRIG - the two share a format in
//           ArduPilot - so TRIG mode has a recorded log to read; a message is found by walking the
//           log from FMT length to FMT length, as ArduPilot wrote it, never by searching bytes.
// srtm      unzips a terrain tile the way srtm.gethgt does (FastZip.ExtractZip, srtm.cs:664-667),
//           for srtm.getAltitude, which ImageProjection.calcIntersection asks (the ground under
//           each photo's footprint, GeoRefImageBase.cs:1466, ImageProjection.cs:354-393).
// bintolog  BinaryLog.ConvertBin, for a text log of the same flight.
// roundtrip ToString("R") of log-shaped and random doubles and of floats (what the XmlSerializer
//           of the positions writes numbers with).
// phototimes getPhotoTime of every file in <dir>, and what DateTime.Today was.
// geotag    WriteCoordinatesToImage of every .jpg in <dir> at one position.
//
// doworkGPSOFFSET matches the photos in a Parallel.ForEach (GeoRefImageBase.cs:685-726), so the
// order of picturesInfo, and of its "Photo ..." lines, is whatever the threads make it. This
// harness puts both in the photos' sorted order - the order a single thread gives, and the one the
// port uses - before CreateReportFiles. Nothing else is reordered.
//
// state.txt: one line per entry of vehicleLocations, camLocations and picturesInfo in their
// dictionary order, and one per photo's getPhotoTime:
//   vehicle|cam <key> <ticks> <kind> <lat> <lon> <altamsl> <relalt> <gpsalt> <salt> <roll> <pitch> <yaw>
//   picture <file> <shotticks> <ticks> <kind> <lat> <lon> <altamsl> <relalt> <gpsalt> <salt> <roll> <pitch> <yaw>
//   phototime <file> <ticks> <kind>
// Doubles are G17 and floats G9, in the invariant culture: both round-trip.
//
// Run under TZ=UTC and the invariant culture: the log's times are converted to local time and
// back (DFLog.cs:702-711, GeoRefImageBase.cs:228), a tlog's to local time (MavlinkParse.cs:146-148),
// and location.tel and the .jxl format doubles in the current culture.

using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
using ICSharpCode.SharpZipLib.Zip;
using MissionPlanner.GeoRef;
using MissionPlanner.Utilities;

// getPhotoTime and readCAMMsgInLog are protected; nothing else is reached this way.
public class OracleGeoRef : GeoRefImageBase
{
    public DateTime PhotoTime(string fn)
    {
        return getPhotoTime(fn);
    }
}

public static class GeorefOracle
{
    static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    public static int Main(string[] args)
    {
        Thread.CurrentThread.CurrentCulture = Inv;
        Thread.CurrentThread.CurrentUICulture = Inv;
        try
        {
            if (args.Length >= 5 && args[0] == "case")
            {
                Case(args[1], args[2], args[3], args[4], Keys(args.Skip(5)));
                Environment.Exit(0);
            }
            if (args.Length >= 4 && args[0] == "estimate")
            {
                Estimate(args[1], args[2], args[3], Keys(args.Skip(4)));
                Environment.Exit(0);
            }
            if (args.Length == 3 && args[0] == "mktrig")
            {
                MkTrig(args[1], args[2]);
                return 0;
            }
            if (args.Length == 3 && args[0] == "srtm")
            {
                new FastZip().ExtractZip(args[1], args[2], "");
                return 0;
            }
            if (args.Length == 3 && args[0] == "bintolog")
            {
                // "Convert .Bin to .Log" with no flight-mode names wired (onFlightMode is null,
                // BinaryLog.cs:559), which only changes MODE lines, which nothing here reads.
                BinaryLog.ConvertBin(args[1], args[2]);
                return 0;
            }
            if (args.Length == 2 && args[0] == "roundtrip")
            {
                RoundTrip(args[1]);
                return 0;
            }
            if (args.Length == 3 && args[0] == "phototimes")
            {
                PhotoTimes(args[1], args[2]);
                return 0;
            }
            if (args.Length == 6 && args[0] == "geotag")
            {
                Geotag(args[1], args[2], double.Parse(args[3], Inv), double.Parse(args[4], Inv),
                    double.Parse(args[5], Inv));
                return 0;
            }
        }
        catch (Exception ex)
        {
            Console.Error.WriteLine("GeorefOracle: " + ex);
            Environment.Exit(1);
        }
        Console.Error.WriteLine("usage: GeorefOracle case <dir> <time|cam|trig> <log> <photos> [key=value ...]");
        Console.Error.WriteLine("       GeorefOracle estimate <out.txt> <log> <photos> [key=value ...]");
        Console.Error.WriteLine("       GeorefOracle mktrig <in.bin> <out.bin>");
        Console.Error.WriteLine("       GeorefOracle srtm <tile.hgt.zip> <dir>");
        return 2;
    }

    static Dictionary<string, string> Keys(IEnumerable<string> args)
    {
        var keys = new Dictionary<string, string>
        {
            {"amsl", "1"}, {"lag", "0"}, {"minshutter", "0.5"}, {"dropstart", "0"}, {"dropend", "0"},
            {"gps", "GPS"}, {"usecam", "0"}, {"offset", "0"}, {"rotation", "90"}, {"hfov", "200"},
            {"vfov", "130"}, {"camgpsalt", "0"}, {"triggpsalt", "0"}, {"basealt", "0"}, {"srtm", ""},
        };
        foreach (var arg in args)
        {
            var at = arg.IndexOf('=');
            keys[arg.Substring(0, at)] = arg.Substring(at + 1);
        }
        return keys;
    }

    static void Srtm(string dir)
    {
        if (dir == "")
            return;
        // A missing tile is queued for download; nothing may leave the machine.
        srtm.datadirectory = dir;
        srtm.baseurl1sec = "http://127.0.0.1:9/SRTM1/";
        srtm.baseurl = "http://127.0.0.1:9/SRTM3/";
    }

    static void Case(string dir, string modeName, string logSource, string photos,
        Dictionary<string, string> keys)
    {
        Srtm(keys["srtm"]);
        if (Directory.Exists(dir))
            Directory.Delete(dir, true);
        Directory.CreateDirectory(dir);
        var copied = new List<string>();
        foreach (var photo in Directory.GetFiles(photos, "*.jpg"))
        {
            var to = Path.Combine(dir, Path.GetFileName(photo));
            File.Copy(photo, to);
            copied.Add(to);
        }
        // The log beside the outputs, so doworkGPSOFFSET's <log>.xml lands there too.
        var logFilePath = Path.Combine(dir, Path.GetFileName(logSource));
        File.Copy(logSource, logFilePath);
        string dirPictures = dir;

        var georef = new OracleGeoRef();
        var messages = new List<string>();
        Action<string> AppendText = text => { lock (messages) messages.Add(text); };

        // The form's settings (georefimage.cs:33, 360-374, 388-391).
        georef.useAMSLAlt = keys["amsl"] == "1";
        int.TryParse(keys["lag"], NumberStyles.Integer, Inv, out georef.millisShutterLag);
        georef.minshutter = (double)decimal.Parse(keys["minshutter"], Inv);
        string useGpsOrGps2 = keys["gps"];
        bool usecam = keys["usecam"] == "1";
        int dropstart = int.Parse(keys["dropstart"], Inv);
        int dropend = int.Parse(keys["dropend"], Inv);
        double rotation = (double)decimal.Parse(keys["rotation"], Inv);
        double hfov = (double)decimal.Parse(keys["hfov"], Inv);
        double vfov = (double)decimal.Parse(keys["vfov"], Inv);
        bool camgpsalt = keys["camgpsalt"] == "1";
        bool triggpsalt = keys["triggpsalt"] == "1";

        var mode = modeName == "time" ? PROCESSING_MODE.TIME_OFFSET
            : modeName == "cam" ? PROCESSING_MODE.CAM_MSG : PROCESSING_MODE.TRIG;

        // BUT_doit_Click (georefimage.cs:150-200).
        float seconds = 0;
        if (mode == PROCESSING_MODE.TIME_OFFSET)
        {
            if (float.TryParse(keys["offset"], NumberStyles.Float, CultureInfo.InvariantCulture,
                    out seconds) == false)
            {
                AppendText("Offset number not in correct format. Use . as decimal separator\n");
                return;
            }
        }
        try
        {
            switch (mode)
            {
                case PROCESSING_MODE.TIME_OFFSET:
                    georef.picturesInfo = georef.doworkGPSOFFSET(logFilePath, dirPictures, seconds,
                        useGpsOrGps2, usecam, AppendText);
                    if (georef.picturesInfo != null)
                    {
                        Sequential(georef, messages, dirPictures);
                        georef.CreateReportFiles(georef.picturesInfo, dirPictures, seconds, rotation,
                            hfov, vfov, AppendText, kml => { });
                    }
                    break;
                case PROCESSING_MODE.CAM_MSG:
                    georef.picturesInfo = georef.doworkCAM(logFilePath, dirPictures, useGpsOrGps2,
                        AppendText, dropstart, dropend);
                    if (georef.picturesInfo != null)
                        georef.CreateReportFiles(georef.picturesInfo, dirPictures, seconds, rotation,
                            hfov, vfov, AppendText, kml => { }, camgpsalt);
                    break;
                case PROCESSING_MODE.TRIG:
                    georef.picturesInfo = georef.doworkTRIG(logFilePath, dirPictures, useGpsOrGps2,
                        AppendText, dropstart, dropend);
                    if (georef.picturesInfo != null)
                        georef.CreateReportFiles(georef.picturesInfo, dirPictures, seconds, rotation,
                            hfov, vfov, AppendText, kml => { }, triggpsalt);
                    break;
            }
        }
        catch (Exception ex)
        {
            // The form prints ex.ToString(); the stack is the runtime's, not the program's.
            AppendText("Error " + ex.GetType().FullName + ": " + ex.Message);
        }

        // BUT_Geotagimages_Click (georefimage.cs:268-319).
        string rootFolder = dirPictures;
        string geoTagFolder = rootFolder + Path.DirectorySeparatorChar + "geotagged";
        if (Directory.Exists(geoTagFolder))
            Directory.Delete(geoTagFolder, true);
        Directory.CreateDirectory(geoTagFolder);
        if (georef.picturesInfo == null)
        {
            AppendText("no valid matchs");
        }
        else
        {
            foreach (PictureInformation picInfo in georef.picturesInfo.Values)
            {
                if (camgpsalt && mode == PROCESSING_MODE.CAM_MSG)
                    georef.WriteCoordinatesToImage(picInfo.Path, picInfo.Lat, picInfo.Lon, picInfo.GPSAlt,
                        dirPictures, AppendText);
                else if (triggpsalt && mode == PROCESSING_MODE.TRIG)
                    georef.WriteCoordinatesToImage(picInfo.Path, picInfo.Lat, picInfo.Lon, picInfo.GPSAlt,
                        dirPictures, AppendText);
                else if (georef.useAMSLAlt)
                    georef.WriteCoordinatesToImage(picInfo.Path, picInfo.Lat, picInfo.Lon,
                        double.Parse(keys["basealt"]) + picInfo.AltAMSL, dirPictures, AppendText);
                else
                    georef.WriteCoordinatesToImage(picInfo.Path, picInfo.Lat, picInfo.Lon, picInfo.RelAlt,
                        dirPictures, AppendText);
            }
            AppendText("GeoTagging FINISHED \n\n");
        }

        var state = new StringBuilder();
        Locations(state, "vehicle", georef.vehicleLocations);
        Locations(state, "cam", georef.camLocations);
        if (georef.picturesInfo != null)
            foreach (var p in georef.picturesInfo.Values)
                state.Append("picture " + Path.GetFileName(p.Path) + " " + p.ShotTimeReportedByCamera.Ticks +
                             " " + Location(p) + "\n");
        foreach (var photo in copied.OrderBy(p => p, StringComparer.Ordinal))
        {
            var t = georef.PhotoTime(photo);
            state.Append("phototime " + Path.GetFileName(photo) + " " + t.Ticks + " " + t.Kind + "\n");
        }
        File.WriteAllText(Path.Combine(dir, "state.txt"), state.ToString());
        File.WriteAllText(Path.Combine(dir, "messages.txt"), string.Concat(messages).Replace(dir, "{dir}"));

        foreach (var photo in copied)
            File.Delete(photo);
        File.Delete(logFilePath);
    }

    // The photos' sorted order for picturesInfo and its "Photo ..." lines, in place of the order
    // doworkGPSOFFSET's Parallel.ForEach happened to leave them in.
    static void Sequential(OracleGeoRef georef, List<string> messages, string dir)
    {
        var files = Directory.GetFiles(dir, "*.jpg").ToList();
        files.Sort((x, y) =>
        {
            var a = georef.PhotoTime(x).CompareTo(georef.PhotoTime(y));
            return a != 0 ? a : x.CompareTo(y);
        });
        var sorted = new Dictionary<string, PictureInformation>();
        foreach (var f in files)
            if (georef.picturesInfo.ContainsKey(f))
                sorted.Add(f, georef.picturesInfo[f]);
        georef.picturesInfo = sorted;

        var photoLines = messages.Where(m => m.StartsWith("Photo ")).ToList();
        var rest = messages.Where(m => !m.StartsWith("Photo ")).ToList();
        photoLines.Sort((a, b) =>
        {
            int ia = files.FindIndex(f => a.StartsWith("Photo " + Path.GetFileNameWithoutExtension(f) + " "));
            int ib = files.FindIndex(f => b.StartsWith("Photo " + Path.GetFileNameWithoutExtension(f) + " "));
            return ia.CompareTo(ib);
        });
        messages.Clear();
        messages.AddRange(rest);
        messages.AddRange(photoLines);
    }

    static string Location(SingleLocation l)
    {
        return l.Time.Ticks + " " + l.Time.Kind + " " + D(l.Lat) + " " + D(l.Lon) + " " + D(l.AltAMSL) + " " +
               D(l.RelAlt) + " " + D(l.GPSAlt) + " " + D(l.SAlt) + " " + F(l.Roll) + " " + F(l.Pitch) + " " +
               F(l.Yaw);
    }

    static void Locations(StringBuilder state, string name, Dictionary<long, VehicleLocation> list)
    {
        if (list == null)
            return;
        foreach (var kv in list)
            state.Append(name + " " + kv.Key + " " + Location(kv.Value) + "\n");
    }

    static string D(double v)
    {
        return v.ToString("G17", Inv);
    }

    static string F(float v)
    {
        return v.ToString("G9", Inv);
    }

    static void Estimate(string outFile, string log, string photos, Dictionary<string, string> keys)
    {
        // BUT_estoffset_Click (georefimage.cs:258-266).
        var georef = new OracleGeoRef();
        var messages = new StringBuilder();
        Action<string> AppendText = text => messages.Append(text);
        try
        {
            double offset = georef.EstimateOffset(log, photos, keys["gps"], keys["usecam"] == "1", AppendText);
            AppendText("Offset around :  " + offset.ToString(CultureInfo.InvariantCulture) + "\n\n");
        }
        catch (Exception ex)
        {
            AppendText("Error " + ex.GetType().FullName + ": " + ex.Message);
        }
        File.WriteAllText(outFile, messages.ToString());
    }

    // ToString("R") - what XmlConvert.ToString, and so the XmlSerializer of the positions beside
    // the log, writes - of doubles shaped like a log's (1e-7 and 1e-2 steps, through double.Parse
    // as DFLogBuffer's text goes) and of random ones, and of floats: "<bits hex> <R>" per line,
    // "F <bits hex> <R>" for a float.
    static void RoundTrip(string output)
    {
        var rnd = new Random(12345);
        using (var w = new StreamWriter(output))
        {
            w.NewLine = "\n";
            for (int i = 0; i < 4000; i++)
            {
                double v;
                switch (i % 5)
                {
                    case 0: v = rnd.Next(-900000000, 900000000) * 1e-7; break;
                    case 1: v = rnd.Next(-1800000000, 1800000000) / 1e7; break;
                    case 2:
                        v = double.Parse((rnd.Next(-100000, 100000) / 100.0).ToString("F2", Inv), Inv);
                        break;
                    case 3: v = (rnd.NextDouble() - 0.5) * Math.Pow(10, rnd.Next(-8, 12)); break;
                    default:
                        v = double.Parse((rnd.Next(-900000000, 900000000) / 1e7).ToString("F7", Inv), Inv);
                        break;
                }
                w.WriteLine(BitConverter.DoubleToInt64Bits(v).ToString("X16") + " " + v.ToString("R", Inv));
            }
            var r2 = new Random(999);
            for (int i = 0; i < 1000; i++)
            {
                float f = (float)((r2.NextDouble() - 0.5) * Math.Pow(10, r2.Next(-6, 8)));
                w.WriteLine("F " + BitConverter.ToInt32(BitConverter.GetBytes(f), 0).ToString("X8") + " " +
                            f.ToString("R", Inv));
            }
        }
    }

    // getPhotoTime of every file in <dir>, by ordinal name: "<name> <ticks> <kind>", after a line
    // "today <ticks>" - what DateTime.Today was, which a date no format reads comes out as.
    static void PhotoTimes(string dir, string output)
    {
        var georef = new OracleGeoRef();
        var lines = new StringBuilder();
        lines.Append("today " + DateTime.Today.Ticks + "\n");
        foreach (var file in Directory.GetFiles(dir).OrderBy(f => f, StringComparer.Ordinal))
        {
            var t = georef.PhotoTime(file);
            lines.Append(Path.GetFileName(file) + " " + t.Ticks + " " + t.Kind + "\n");
        }
        File.WriteAllText(output, lines.ToString());
    }

    // WriteCoordinatesToImage (GeoRefImageBase.cs:1167-1231) of every .jpg in <dir>, by ordinal
    // name, at one position, into <out>/geotagged, with its lines in <out>/messages.txt. The
    // photos are copied into <out> and deleted after, as `case` does.
    static void Geotag(string dir, string output, double lat, double lon, double alt)
    {
        if (Directory.Exists(output))
            Directory.Delete(output, true);
        Directory.CreateDirectory(output);
        var georef = new OracleGeoRef();
        var messages = new StringBuilder();
        Action<string> AppendText = text => messages.Append(text);
        var copied = new List<string>();
        foreach (var file in Directory.GetFiles(dir, "*.jpg").OrderBy(f => f, StringComparer.Ordinal))
        {
            var to = Path.Combine(output, Path.GetFileName(file));
            File.Copy(file, to);
            copied.Add(to);
        }
        foreach (var file in copied)
            georef.WriteCoordinatesToImage(file, lat, lon, alt, output, AppendText);
        File.WriteAllText(Path.Combine(output, "messages.txt"), messages.ToString().Replace(output, "{dir}"));
        foreach (var file in copied)
            File.Delete(file);
    }

    // camera.bin with each CAM message's type byte set to TRIG's.
    static void MkTrig(string input, string output)
    {
        var data = File.ReadAllBytes(input);
        var lengths = new Dictionary<byte, int>();
        byte cam = 0, trig = 0;
        int pos = 0;
        int relabelled = 0;
        while (pos + 3 <= data.Length)
        {
            if (data[pos] != 0xA3 || data[pos + 1] != 0x95)
                throw new InvalidDataException("no message header at " + pos);
            byte type = data[pos + 2];
            if (type == 0x80)
            {
                byte defined = data[pos + 3];
                lengths[defined] = data[pos + 4];
                string name = Encoding.ASCII.GetString(data, pos + 5, 4).TrimEnd('\0');
                if (name == "CAM") cam = defined;
                if (name == "TRIG") trig = defined;
                pos += 89;
                continue;
            }
            if (!lengths.ContainsKey(type))
                throw new InvalidDataException("no format for type " + type + " at " + pos);
            // SITL was stopped mid-write: the last message is cut short, and left as it is.
            if (pos + lengths[type] > data.Length)
                break;
            if (type == cam && cam != 0 && trig != 0)
            {
                data[pos + 2] = trig;
                relabelled++;
            }
            pos += lengths[type];
        }
        File.WriteAllBytes(output, data);
        Console.WriteLine("relabelled " + relabelled + " CAM messages as TRIG");
    }
}
