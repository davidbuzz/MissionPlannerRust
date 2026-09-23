// Headless dataflash-log oracle: the `bintolog`, `dflogtokml`, `matlab` and `loganalysis` verbs, for
// D14 and the flight screen's DataFlash Logs page (GCSViews/FlightData.cs:1082-1098, 1135-1197,
// 1311-1385, 1387-1390).
//
// Runs Mission Planner's own converters out of the msbuild output of ExtLibs/Utilities - the same
// build regen-grid.sh makes - over a log and writes exactly what they write, so
// crates/mp-log/tests/convert.rs, crates/mp-log/tests/matlab.rs and crates/mp-kml/tests/dflog.rs
// hold the Rust to the C# rather than to our reading of it. Console-only: BinaryLog.cs,
// DFLogBuffer.cs, DFLog.cs, LogOutput.cs and MatLab.cs never touch WinForms.
//
// Build:  see regen-log.sh (mcs against the msbuild output of ExtLibs/Utilities)
// Run:    mono MpLog.exe bintolog    <log.bin> <out.log> <ParameterMetaDataBackup.xml>
//         mono MpLog.exe dflogtokml  <log> <ParameterMetaDataBackup.xml>
//         mono MpLog.exe matlab      <log> <ParameterMetaDataBackup.xml>
//         mono MpLog.exe loganalysis <analyzer.xml> <out.txt>
//         mono MpLog.exe mkedge      <out.bin>
//
// bintolog     BinaryLog.ConvertBin(in, out), what but_bintolog_Click runs per file
//              (FlightData.cs:1093-1096).
// dflogtokml   the body of but_dflogtokml_Click's loop (FlightData.cs:1154-1193) on one file: every
//              line of a DFLogBuffer (.bin) or of the text (.log) through LogOutput.processLine, then
//              LogOutput.writeKML(<log>.kml). Everything is written beside <log>, as the button
//              writes it, so the caller hands in a copy. writeKML zips the .kml into a .kmz and
//              deletes it (LogOutput.cs:1101-1128); this verb then extracts the zip's entries beside
//              it, the .kml under its own name, and lists them in <kmz>.entries - the zip itself
//              carries DateTime.Now and SharpZipLib's deflate, neither of which is the format.
// matlab       MatLab.ProcessLog(<log>), what MatLabForms.ProcessLog runs per file
//              (Log/MatLabForms.cs:61-66), writing <log>-<lines>.mat beside it.
// loganalysis  the analyzer's report as the button shows it: LogAnalyzer.Results(<xml>) and then the
//              text Controls.LogAnalyzer puts in its text box. Utilities/LogAnalyzer.cs and
//              Controls/LogAnalyzer.cs are in the WinForms application, not ExtLibs, so the two
//              pieces that do no I/O but read the XML and build the text are copied here verbatim,
//              with their lines cited; what they are run against is .NET's own XmlReader, which is
//              the part that has to be exact. CheckLogFile - download LogAnalyzer64.zip, extract it,
//              run runner.exe - is not an oracle's business.
//
// BinaryLog names a flight mode through the static event BinaryLog.onFlightMode, which the
// application wires in MainV2.cs:3394-3418 to ArduPilot.Common.getModesList (ExtLibs/ArduPilot/
// Common.cs:88-183). That assembly is not part of the Utilities build, and getModesList reads the
// parameter metadata through ParameterMetaDataRepository, which prefers whatever
// ParameterMetaData.xml this machine's Mission Planner data directory happens to hold. So the handler
// here is MainV2's, and getModesList's per-firmware rules are copied, with the mode table read from
// the pinned tree's ParameterMetaDataBackup.xml the way ParameterMetaDataRepositoryAPM.
// GetParameterMetaData (ExtLibs/Utilities/ParameterMetaDataRepositoryAPM.cs:69-104) and
// ParameterMetaDataRepository.GetParameterOptionsInt (ParameterMetaDataRepository.cs:74-101) read it:
// the first <vehicle> element with children holding <node> with children, its <Values>, split on ','
// then ':' and trimmed. That is the file mp-vehicle's mode table is generated from.
//
// Run under TZ=UTC and the invariant culture: LogOutput's GPX times are local (DFLog.cs:702-711) and
// its waypoint files format doubles in the current culture (LogOutput.cs:490).

using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Xml;
using System.Xml.Linq;
using ICSharpCode.SharpZipLib.Zip;
using MissionPlanner.Log;
using MissionPlanner.Utilities;

public static class MpLog
{
    static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    public static int Main(string[] args)
    {
        System.Threading.Thread.CurrentThread.CurrentCulture = Inv;
        System.Threading.Thread.CurrentThread.CurrentUICulture = Inv;
        try
        {
            if (args.Length == 4 && args[0] == "bintolog")
            {
                WireFlightModes(args[3]);
                // FlightData.cs:1096
                BinaryLog.ConvertBin(args[1], args[2]);
                return 0;
            }
            if (args.Length == 3 && args[0] == "dflogtokml")
            {
                WireFlightModes(args[2]);
                return DfLogToKml(args[1]);
            }
            if (args.Length == 3 && args[0] == "matlab")
            {
                WireFlightModes(args[2]);
                // Log/MatLabForms.cs:65
                MatLab.ProcessLog(args[1]);
                return 0;
            }
            if (args.Length == 2 && args[0] == "mkedge")
            {
                File.WriteAllBytes(args[1], MkEdge());
                return 0;
            }
            if (args.Length == 3 && args[0] == "loganalysis")
            {
                var text = Report(Results(args[1]));
                File.WriteAllText(args[2], text, new UTF8Encoding(false));
                return 0;
            }
        }
        catch (Exception ex)
        {
            // No partial golden data: a verb that fails fails the whole regeneration.
            Console.Error.WriteLine("MpLog: " + ex);
            return 1;
        }
        Console.Error.WriteLine("usage: MpLog bintolog <log.bin> <out.log> <ParameterMetaDataBackup.xml>");
        Console.Error.WriteLine("       MpLog dflogtokml <log> <ParameterMetaDataBackup.xml>");
        Console.Error.WriteLine("       MpLog matlab <log> <ParameterMetaDataBackup.xml>");
        Console.Error.WriteLine("       MpLog loganalysis <analyzer.xml> <out.txt>");
        Console.Error.WriteLine("       MpLog mkedge <out.bin>");
        return 2;
    }

    // The edge-case log: a synthetic dataflash log that takes every path BinaryLog.cs has and the
    // checked-in logs do not - every field type at its extremes, a few hundred floats and doubles
    // from arbitrary bit patterns, strings with every kind of character the escaping touches, a
    // format with an unknown character, one that runs past its body, one too short to have one, a
    // header BinaryLog's scan misses, a message before its own FMT, a type declared twice, a mode
    // named for a plane, and a message cut off by the end of the file. Deterministic: its bytes are
    // committed as testdata/dataflash/edge.bin and are the input, the C# output the golden.
    static byte[] MkEdge()
    {
        var o = new List<byte>();
        Action<string, int> pad = (value, width) =>
        {
            var bytes = new byte[width];
            for (int i = 0; i < value.Length && i < width; i++)
                bytes[i] = (byte)value[i];
            o.AddRange(bytes);
        };
        Action<byte> head = type => { o.Add(0xA3); o.Add(0x95); o.Add(type); };
        Action<byte, int, string, string, string> fmt = (type, length, name, format, labels) =>
        {
            head(0x80);
            o.Add(type);
            o.Add((byte)length);
            pad(name, 4);
            pad(format, 16);
            pad(labels, 64);
        };
        Action<string, int> text = (value, width) => pad(value, width);

        fmt(0x80, 89, "FMT", "BBnNZ", "Type,Length,Name,Format,Columns");
        fmt(10, 3 + 56, "EDGE", "bBhHiIqQfdgcCeE", "b,B,h,H,i,I,q,Q,f,d,g,c,C,e,E");
        fmt(11, 3 + 4 + 16 + 64 + 1, "STR", "nNZM", "n,N,Z,M");
        fmt(12, 3 + 64, "ARR", "a", "a");
        fmt(13, 3 + 2, "UNK", "BxB", "a,x,b");
        fmt(14, 3 + 2, "SHRT", "I", "I");
        fmt(15, 2, "TINY", "B", "B");
        fmt(16, 3 + 64, "MSG", "Z", "Message");
        fmt(18, 3 + 2, "Q\0Z", "B\0B", "a,b");
        fmt(19, 3 + 2, "HIGH", "B\u00c3", "a,b");

        Action<sbyte, byte, short, ushort, int, uint, long, ulong, float, double, ushort, short, ushort, int, uint> edge =
            (b, B, h, H, i, I, q, Q, f, d, g, c, C, e, E) =>
            {
                head(10);
                o.Add((byte)b); o.Add(B);
                o.AddRange(BitConverter.GetBytes(h)); o.AddRange(BitConverter.GetBytes(H));
                o.AddRange(BitConverter.GetBytes(i)); o.AddRange(BitConverter.GetBytes(I));
                o.AddRange(BitConverter.GetBytes(q)); o.AddRange(BitConverter.GetBytes(Q));
                o.AddRange(BitConverter.GetBytes(f)); o.AddRange(BitConverter.GetBytes(d));
                o.AddRange(BitConverter.GetBytes(g)); o.AddRange(BitConverter.GetBytes(c));
                o.AddRange(BitConverter.GetBytes(C)); o.AddRange(BitConverter.GetBytes(e));
                o.AddRange(BitConverter.GetBytes(E));
            };
        edge(sbyte.MinValue, byte.MaxValue, short.MinValue, ushort.MaxValue, int.MinValue, uint.MaxValue,
            long.MinValue, ulong.MaxValue, float.NaN, double.Epsilon, 0x7C00, short.MinValue, ushort.MaxValue,
            int.MinValue, uint.MaxValue);
        edge(0, 0, 1, 2, 3, 4, 5, 6, -0.0f, -0.0, 0x8000, 0, 0, 0, 0);
        edge(-1, 1, -1, 1, -1, 1, -1, 1, float.MaxValue, double.MaxValue, 0x0001, 12345, 1, -1, 1);
        edge(127, 128, 32767, 32768, int.MaxValue, 2147483648u, long.MaxValue, 9223372036854775808ul,
            float.Epsilon, -double.MaxValue, 0xFC00, -1, 99, 12345678, 0);
        edge(0, 0, 0, 0, 0, 0, 0, 0, float.PositiveInfinity, double.NegativeInfinity, 0x7E00, 5, 5, 5, 5);
        // Arbitrary bit patterns, from a fixed linear congruential sequence.
        ulong state = 0x2545F4914F6CDD1DUL;
        Func<ulong> next = () => { state = state * 6364136223846793005UL + 1442695040888963407UL; return state; };
        for (int n = 0; n < 300; n++)
        {
            var r1 = next();
            var r2 = next();
            float f = BitConverter.ToSingle(BitConverter.GetBytes((uint)(r1 >> 32)), 0);
            double d = BitConverter.Int64BitsToDouble((long)r2);
            if (n % 3 == 1)
            {
                // Values in the range a log actually holds, with every digit significant.
                f = (float)(((long)(r1 >> 11) % 2000000000) / 1000.0);
                d = ((long)(r2 >> 11) % 200000000000000) / 1e7;
            }
            else if (n % 3 == 2)
            {
                f = (float)(((long)(r1 >> 20) % 100000) / 7.0);
                d = ((long)(r2 >> 20) % 1000000000) / 3.0;
            }
            edge((sbyte)r1, (byte)(r1 >> 8), (short)(r1 >> 16), (ushort)(r2 >> 16), (int)(r2 >> 8), (uint)r1,
                (long)r2, r1, f, d, (ushort)(r2 >> 40), (short)(r2 >> 24), (ushort)(r1 >> 48), (int)(r1 >> 3),
                (uint)(r2 >> 5));
        }

        Action<string, byte[], byte[], byte> str = (n4, n16, z64, mode) =>
        {
            head(11);
            text(n4, 4);
            var b16 = new byte[16];
            Array.Copy(n16, b16, Math.Min(16, n16.Length));
            o.AddRange(b16);
            var b64 = new byte[64];
            Array.Copy(z64, b64, Math.Min(64, z64.Length));
            o.AddRange(b64);
            o.Add(mode);
        };
        // No firmware named yet: the mode is its number.
        str("AB", new byte[] { 0x41, 0x42 }, System.Text.Encoding.ASCII.GetBytes("plain text"), 5);
        head(16);
        text("ArduPlane V4.5.7 (1a2b3c4d)", 64);
        str("A\0B", new byte[] { 0x80, 0xFF, 0x7F, 0x41 },
            new byte[] { 0x62, 0x61, 0x63, 0x6B, 0x5C, 0x09, 0x0A, 0x0D, 0x01, 0x7F, 0xC3, 0xA9, 0x00, 0x02 }, 16);
        str("\0\0X\0", new byte[] { 0, 0, 0x42, 0 }, new byte[] { 0, 0, 0x20, 0x21, 0 }, 99);
        str("MODE", new byte[] { 0x4D }, new byte[] { 0x5A }, 5);
        // An all-NUL string: the C# throws formatting it, and the message is dropped.
        str("GONE", new byte[] { 0x47 }, new byte[0], 5);
        head(16);
        text("", 64);

        head(12);
        for (int k = 0; k < 32; k++)
            o.AddRange(BitConverter.GetBytes((short)(k * 2111 - 32768)));
        head(13); o.Add(1); o.Add(2);
        head(14); o.Add(1); o.Add(2);
        head(15);
        head(18); o.Add(7); o.Add(8);
        head(19); o.Add(9); o.Add(10);
        head(200); o.Add(0x11);
        // A3 A3 95: the second A3 resets the scan, so this record is not found.
        o.Add(0xA3); head(13); o.Add(3); o.Add(4);
        head(13); o.Add(5); o.Add(6);
        // A message before its own format.
        head(17); o.AddRange(BitConverter.GetBytes(1234567u));
        fmt(17, 3 + 4, "LATE", "I", "v");
        head(17); o.AddRange(BitConverter.GetBytes(7654321u));
        // Type 10 declared again under a new name: kept out of the cache until an unknown type
        // makes BinaryLog rebuild it from the formats by name.
        fmt(10, 3 + 4, "EDG2", "I", "v");
        edge(1, 2, 3, 4, 5, 6, 7, 8, 1.5f, 2.5, 0x3C00, 1, 2, 3, 4);
        head(201);
        head(10); o.AddRange(BitConverter.GetBytes(42u));
        head(16);
        text("PARM, RATE_RLL_P ArduCopter", 64);
        str("END", new byte[] { 0x45 }, new byte[] { 0x45 }, 5);
        // Cut short by the end of the file: the array reads as far as it goes, then zeros.
        head(12);
        for (int k = 0; k < 5; k++)
            o.AddRange(BitConverter.GetBytes((short)(k - 2)));
        return o.ToArray();
    }

    // MainV2.cs:3394-3418, with getModesList (ExtLibs/ArduPilot/Common.cs:88-183) for the firmwares
    // BinaryLog can guess (BinaryLog.cs:179-201): ArduCopter2, ArduPlane, ArduRover, ArduTracker.
    static void WireFlightModes(string backupXml)
    {
        var doc = XDocument.Load(backupXml);
        BinaryLog.onFlightMode += (firmware, modeno) =>
        {
            try
            {
                if (firmware == "")
                    return null;

                var modes = GetModesList(doc, firmware);
                string currentmode = null;

                foreach (var mode in modes)
                {
                    if (mode.Key == modeno)
                    {
                        currentmode = mode.Value;
                        break;
                    }
                }

                return currentmode;
            }
            catch
            {
                return null;
            }
        };
    }

    // ExtLibs/ArduPilot/Common.cs:141-180
    static List<KeyValuePair<int, string>> GetModesList(XDocument doc, string firmware)
    {
        if (firmware == "ArduPlane")
        {
            var flightModes = GetParameterOptionsInt(doc, "FLTMODE1", firmware);
            flightModes.Add(new KeyValuePair<int, string>(16, "INITIALISING"));
            return flightModes;
        }
        if (firmware == "ArduCopter2")
            return GetParameterOptionsInt(doc, "FLTMODE1", firmware);
        if (firmware == "ArduRover")
            return GetParameterOptionsInt(doc, "MODE1", firmware);
        if (firmware == "ArduTracker")
        {
            var temp = new List<KeyValuePair<int, string>>();
            temp.Add(new KeyValuePair<int, string>(0, "MANUAL"));
            temp.Add(new KeyValuePair<int, string>(1, "STOP"));
            temp.Add(new KeyValuePair<int, string>(2, "SCAN"));
            temp.Add(new KeyValuePair<int, string>(3, "SERVO_TEST"));
            temp.Add(new KeyValuePair<int, string>(10, "AUTO"));
            temp.Add(new KeyValuePair<int, string>(16, "INITIALISING"));
            return temp;
        }
        return null;
    }

    // ParameterMetaDataRepository.cs:74-101 over ParameterMetaDataRepositoryAPM.cs:69-104.
    static List<KeyValuePair<int, string>> GetParameterOptionsInt(XDocument doc, string nodeKey, string vehicle)
    {
        string availableValuesRaw = string.Empty;
        foreach (var element in doc.Element("Params").Elements(vehicle))
        {
            if (element != null && element.HasElements)
            {
                var node = element.Element(nodeKey);
                if (node != null && node.HasElements)
                {
                    var metaValue = node.Element("Values");
                    if (metaValue != null)
                    {
                        availableValuesRaw = metaValue.Value;
                        break;
                    }
                }
            }
        }
        var splitValues = new List<KeyValuePair<int, string>>();
        foreach (string val in availableValuesRaw.Split(new[] {','}, StringSplitOptions.RemoveEmptyEntries))
        {
            try
            {
                string[] valParts = val.Split(new[] {':'});
                splitValues.Add(new KeyValuePair<int, string>(int.Parse(valParts[0].Trim()),
                    (valParts.Length > 1) ? valParts[1].Trim() : valParts[0].Trim()));
            }
            catch
            {
            }
        }
        return splitValues;
    }

    // GCSViews/FlightData.cs:1154-1193, one file.
    //
    // writeKML opens the .kmz at `filename.ToLower()...` - the whole path lower-cased
    // (LogOutput.cs:1105) - which on Windows is the same directory and on Linux is a directory that
    // does not exist unless the path was lower case to begin with. So the verb works from the log's
    // directory with the bare file name, which the fixtures have in lower case.
    static int DfLogToKml(string logpath)
    {
        Directory.SetCurrentDirectory(Path.GetDirectoryName(Path.GetFullPath(logpath)));
        string logfile = "." + Path.DirectorySeparatorChar + Path.GetFileName(logpath);
        LogOutput lo = new LogOutput();
        StreamReader tr;

        if (logfile.ToLower().EndsWith(".bin"))
        {
            using (tr = new StreamReader(logfile))
            {
                DFLogBuffer temp = new DFLogBuffer(tr.BaseStream);

                foreach (var line in temp)
                {
                    lo.processLine(line);
                }

                temp.Dispose();
            }
        }
        else
        {
            using (tr = new StreamReader(logfile))
            {
                while (!tr.EndOfStream)
                {
                    lo.processLine(tr.ReadLine());
                }

                tr.Close();
            }
        }

        lo.writeKML(logfile + ".kml");

        // LogOutput.cs:1105
        var kml = logfile + ".kml";
        var kmz = kml.ToLower().Replace(".log.kml", ".kmz").Replace(".bin.kml", ".kmz");
        var listing = new StringBuilder();
        using (var zip = new ZipFile(kmz))
        {
            foreach (ZipEntry entry in zip)
            {
                listing.Append(entry.Name + "," + entry.Size + "\n");
                var target = entry.Name;
                // The .kml goes back where writeKML wrote it before zipping; the model is the
                // application's own file and is not golden data.
                if (entry.Name.EndsWith(".dae"))
                    continue;
                using (var input = zip.GetInputStream(entry))
                using (var output = File.Create(target))
                    input.CopyTo(output);
            }
        }
        File.WriteAllText(kmz + ".entries", listing.ToString(), new UTF8Encoding(false));
        File.Delete(kmz);
        return 0;
    }

    // --- Utilities/LogAnalyzer.cs:117-240, verbatim but for the logger ---------------------------

    public class analysis
    {
        public string logfile;
        public string sizekb;
        public string sizelines;
        public string duration;
        public string vehicletype;
        public string firmwareversion;
        public string firmwarehash;
        public string hardwaretype;
        public string freemem;
        public string skippedlines;

        public List<result> results = new List<result>();
    }

    public class result
    {
        public string name;
        public string status;
        public string message;
        public string data;
    }

    public static analysis Results(string xmlfile)
    {
        analysis answer = new analysis();

        using (XmlReader reader = XmlReader.Create(xmlfile))
        {
            while (!reader.EOF)
            {
                if (reader.ReadToFollowing("header"))
                {
                    var subtree = reader.ReadSubtree();

                    while (subtree.Read())
                    {
                        subtree.MoveToElement();
                        if (subtree.IsStartElement())
                        {
                            try
                            {
                                switch (subtree.Name.ToLower())
                                {
                                    case "logfile":
                                        answer.logfile = subtree.ReadString();
                                        break;
                                    case "sizekb":
                                        answer.sizekb = subtree.ReadString();
                                        break;
                                    case "sizelines":
                                        answer.sizelines = subtree.ReadString();
                                        break;
                                    case "duration":
                                        answer.duration = subtree.ReadString();
                                        break;
                                    case "vehicletype":
                                        answer.vehicletype = subtree.ReadString();
                                        break;
                                    case "firmwareversion":
                                        answer.firmwareversion = subtree.ReadString();
                                        break;
                                    case "firmwarehash":
                                        answer.firmwarehash = subtree.ReadString();
                                        break;
                                    case "hardwaretype":
                                        answer.hardwaretype = subtree.ReadString();
                                        break;
                                    case "freemem":
                                        answer.freemem = subtree.ReadString();
                                        break;
                                    case "skippedlines":
                                        answer.skippedlines = subtree.ReadString();
                                        break;
                                }
                            }
                            catch (Exception ex)
                            {
                                Console.Error.WriteLine(ex);
                            }
                        }
                    }
                }
                // params - later
                if (reader.ReadToFollowing("results"))
                {
                    var subtree = reader.ReadSubtree();

                    result res = null;

                    while (subtree.Read())
                    {
                        subtree.MoveToElement();
                        if (subtree.IsStartElement())
                        {
                            switch (subtree.Name.ToLower())
                            {
                                case "result":
                                    if (res != null && res.name != "")
                                        answer.results.Add(res);
                                    res = new result();
                                    break;
                                case "name":
                                    res.name = subtree.ReadString();
                                    break;
                                case "status":
                                    res.status = subtree.ReadString();
                                    break;
                                case "message":
                                    res.message = subtree.ReadString();
                                    break;
                                case "data":
                                    res.data = subtree.ReadString();
                                    break;
                            }
                        }
                    }
                }
            }
        }

        return answer;
    }

    // --- Controls/LogAnalyzer.cs:12-31, verbatim but for the text box ---------------------------

    static string Report(analysis analysis)
    {
        var start = String.Format(@"Log File {0}
Size (kb) {1}
No of lines {2}
Duration {3}
Vehicletype {4}
Firmware Version {5}
Firmware Hash {6}
Hardware Type {7}
Free Mem {8}
Skipped Lines {9}
", analysis.logfile, analysis.sizekb, analysis.sizelines, analysis.duration, analysis.vehicletype,
            analysis.firmwareversion, analysis.firmwarehash, analysis.hardwaretype, analysis.freemem,
            analysis.skippedlines).Replace("\n", Environment.NewLine);

        var text = start;

        foreach (var item in analysis.results)
        {
            text += "Test: " + item.name + " = " + item.status + " - " + item.message + "\r\n";
        }

        return text;
    }
}
