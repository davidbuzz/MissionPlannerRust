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

// The Cursor-on-Target text Controls/SerialOutputCoT.cs's getXmlString makes (:263-360), for
// testdata/cot/golden: its serialization re-hosted - the form is WinForms and is not built here -
// over the CoT classes compiled from the reference tree as they stand (ExtLibs/Utilities/CoT).
// Each line of testdata/cot/cases.txt, `|`-separated:
//
//   name|uid|type|how|lat|lng|alt|course|speed|time|indent|takv|callsign|endpoint|vmf
//
// time is UTC as `yyyy-MM-ddTHH:mm:ss.fffffff`; indent and takv are 1 or 0; an empty callsign,
// endpoint or vmf is the grid's empty cell. golden/<name>.xml is the text, its line ends as
// Windows writes them: NewLineChars is set to "\r\n", .NET Framework's Environment.NewLine there,
// where mono's here is "\n" - the C# runs on Windows.
//
//   mcs -r:System.Xml.dll -out:CotOracle.exe CotOracle.cs $MP_SRC/ExtLibs/Utilities/CoT/*.cs
//   mono CotOracle.exe testdata/cot/cases.txt testdata/cot/golden
using System;
using System.Globalization;
using System.IO;
using System.Text;
using System.Xml;
using System.Xml.Serialization;
using MissionPlanner.Utilities.CoT;

// `Utf8StringWriter`. C#: Controls/SerialOutputCoT.cs:458-460
public sealed class Utf8StringWriter : StringWriter
{
    public override Encoding Encoding => Encoding.UTF8;
}

public static class CotOracle
{
    static bool isValidStr(object obj)
    {
        return (obj != null && obj.ToString().Length > 0);
    }

    // C#: Controls/SerialOutputCoT.cs:263-360, the grid's row read as the cells it would hold.
    static string GetXmlString(string uid, string type, string how, double lat, double lng, double alt,
        double course, double speed, DateTime time, bool indent, bool takv, string callsign,
        string endpoint, string uid_vmf)
    {
        if (uid == null || uid.Length <= 0) {
            uid = "";
        }
        if (type == null || type.Length <= 0) {
            type = "";
        }

        CultureInfo culture = new CultureInfo("en-US");
        culture.NumberFormat.NumberGroupSeparator = "";

        string datetimeformat = "yyyy-MM-ddTHH:mm:ss.ffK";

        var cotevent = new @event()
        {
            uid = uid, type = type, time = time.ToString(datetimeformat), start = time.AddSeconds(-5).ToString(datetimeformat),
            stale = time.AddSeconds(120).ToString(datetimeformat), how = how,
            detail = new detail()
            {
                track = new track()
                {
                    course = course.ToString("N2", culture), speed = speed.ToString("N2", culture)
                }
            },
            point = new point()
            {
                lat = lat.ToString("N7", culture), lon = lng.ToString("N7", culture), hae = alt.ToString("N2", culture).PadLeft(5, ' ')
            }
        };

        if (takv)
        {
            cotevent.detail.takv = new takv();
        }
        if (isValidStr(callsign))
        {
            if (cotevent.detail.contact == null) cotevent.detail.contact = new contact();
            cotevent.detail.contact.callsign = callsign;
        }
        if (isValidStr(endpoint))
        {
            if (cotevent.detail.contact == null) cotevent.detail.contact = new contact();
            cotevent.detail.contact.endpoint = endpoint;
        }
        if (isValidStr(uid_vmf))
        {
            cotevent.detail.uid = new uid();
            cotevent.detail.uid.vmf = uid_vmf;
        }

        using (StringWriter textWriter = new Utf8StringWriter())
        {
            XmlWriterSettings xws = new XmlWriterSettings();
            xws.OmitXmlDeclaration = true;
            xws.Indent = indent;
            xws.Encoding = Encoding.UTF8;
            xws.NewLineOnAttributes = indent;
            // Windows' Environment.NewLine, the C#'s there.
            xws.NewLineChars = "\r\n";

            XmlSerializerNamespaces ns = new XmlSerializerNamespaces();
            ns.Add("", "");

            var xtw = XmlTextWriter.Create(textWriter, xws);
            XmlSerializer serializer = new XmlSerializer(typeof(@event));

            xtw.WriteStartDocument(true);

            serializer.Serialize(xtw, cotevent, ns);

            var ans = "<?xml version=\"1.0\" encoding=\"utf-8\" standalone=\"yes\"?>\r\n" + textWriter.ToString();

            return ans;
        }
    }

    public static int Main(string[] args)
    {
        var inv = CultureInfo.InvariantCulture;
        Directory.CreateDirectory(args[1]);
        foreach (var line in File.ReadAllLines(args[0]))
        {
            if (line.Length == 0 || line.StartsWith("#")) continue;
            var f = line.Split('|');
            var time = DateTime.SpecifyKind(
                DateTime.ParseExact(f[9], "yyyy-MM-ddTHH:mm:ss.fffffff", inv), DateTimeKind.Utc);
            var xml = GetXmlString(f[1], f[2], f[3], double.Parse(f[4], inv), double.Parse(f[5], inv),
                double.Parse(f[6], inv), double.Parse(f[7], inv), double.Parse(f[8], inv), time,
                f[10] == "1", f[11] == "1", f[12], f[13], f[14]);
            File.WriteAllText(Path.Combine(args[1], f[0] + ".xml"), xml, new UTF8Encoding(false));
        }
        return 0;
    }
}
