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

// Headless reference dumper for differential testing (DELIVERABLES.md Deliverable 19).
//
// Loads the *shipped* Mission Planner MAVLink.dll and prints what the C# implementation decodes,
// so the Rust codec can be diffed against the real thing rather than against our reading of it.
// Deliberately console-only: it never touches WinForms, so it runs cleanly under mono on Linux
// where the full GUI is unstable.
//
// Build:  mcs -out:MpRefDump.exe -r:<MP>/MAVLink.dll MpRefDump.cs
// Run:    mono MpRefDump.exe infos
//         mono MpRefDump.exe tlog <file.tlog> [max_frames]

using System;
using System.IO;

public static class MpRefDump
{
    public static int Main(string[] args)
    {
        if (args.Length == 0) { Usage(); return 2; }
        switch (args[0])
        {
            case "infos": return DumpInfos();
            case "tlog": return DumpTlog(args);
            case "fields": return DumpFields(args);
            default: Usage(); return 2;
        }
    }

    static void Usage()
    {
        Console.Error.WriteLine("usage: MpRefDump infos | tlog <file> [max_frames] | fields <file> [max_frames]");
    }

    // The authoritative message table as the shipping binary holds it.
    static int DumpInfos()
    {
        Console.WriteLine("id,name,crc_extra,min_len,len");
        foreach (var mi in MAVLink.MAVLINK_MESSAGE_INFOS)
        {
            if (mi.name == null) continue;
            Console.WriteLine("{0},{1},{2},{3},{4}", mi.msgid, mi.name, mi.crc, mi.minlength, mi.length);
        }
        return 0;
    }

    // Decode a telemetry log with the C# parser and emit one row per frame.
    static int DumpTlog(string[] args)
    {
        if (args.Length < 2) { Usage(); return 2; }
        string path = args[1];
        long max = args.Length > 2 ? long.Parse(args[2]) : long.MaxValue;

        // hasTimestamp: tlogs prefix each frame with a big-endian 64-bit microsecond stamp.
        var parser = new MAVLink.MavlinkParse(true);
        long index = 0;

        // MAVLink.dll writes "Unknown Packet <id>" to stdout from inside the library. Our CSV
        // must be data only, so the library's console is pointed at stderr and our rows go to a
        // private handle on the real stdout.
        var data = Console.Out;
        Console.SetOut(Console.Error);

        data.WriteLine("index,msgid,seq,sysid,compid,payload_len,crc16,frame_hex");
        using (var fs = new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.Read))
        {
            while (fs.Position < fs.Length && index < max)
            {
                MAVLink.MAVLinkMessage msg;
                try { msg = parser.ReadPacket(fs); }
                catch (EndOfStreamException) { break; }
                catch (Exception ex)
                {
                    Console.Error.WriteLine("# parse error at {0}: {1}", fs.Position, ex.Message);
                    break;
                }
                if (msg == null || msg.buffer == null) continue;

                data.WriteLine("{0},{1},{2},{3},{4},{5},{6},{7}",
                    index, msg.msgid, msg.seq, msg.sysid, msg.compid,
                    msg.payloadlength, msg.crc16, ToHex(msg.buffer));
                index++;
            }
        }
        Console.Error.WriteLine("# frames={0} badCRC={1} badLength={2}", index, parser.badCRC, parser.badLength);
        return 0;
    }

    // Decode every frame into its C# struct and dump every field by name, via reflection.
    // This is what lets the Rust port be compared field by field rather than byte by byte:
    // a field at the right offset with the wrong name or sign is invisible to a byte diff.
    static int DumpFields(string[] args)
    {
        if (args.Length < 2) { Usage(); return 2; }
        string path = args[1];
        long max = args.Length > 2 ? long.Parse(args[2]) : 3000;

        var infoByMsgId = new System.Collections.Generic.Dictionary<uint, MAVLink.message_info>();
        foreach (var mi in MAVLink.MAVLINK_MESSAGE_INFOS)
            if (mi.name != null) infoByMsgId[mi.msgid] = mi;

        var toStructure = typeof(MAVLink.MAVLinkMessage).GetMethod("ToStructure");
        var parser = new MAVLink.MavlinkParse(true);
        long index = 0;

        var data = Console.Out;
        Console.SetOut(Console.Error);
        data.WriteLine("index,msgid,msgname,field,value");
        using (var fs = new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.Read))
        {
            while (fs.Position < fs.Length && index < max)
            {
                MAVLink.MAVLinkMessage msg;
                try { msg = parser.ReadPacket(fs); }
                catch (Exception) { break; }
                if (msg == null || msg.buffer == null) continue;

                MAVLink.message_info mi;
                if (!infoByMsgId.TryGetValue(msg.msgid, out mi) || mi.type == null) { index++; continue; }

                object decoded;
                try { decoded = toStructure.MakeGenericMethod(mi.type).Invoke(msg, null); }
                catch (Exception) { index++; continue; }

                foreach (var f in mi.type.GetFields())
                {
                    object value = f.GetValue(decoded);
                    data.WriteLine("{0},{1},{2},{3},{4}", index, msg.msgid, mi.name, f.Name, Render(value));
                }
                index++;
            }
        }
        Console.Error.WriteLine("# frames={0}", index);
        return 0;
    }

    // Values are rendered so a Rust-side parser can compare them exactly: round-trippable
    // floats, space-separated arrays, invariant culture throughout.
    static string Render(object value)
    {
        var ci = System.Globalization.CultureInfo.InvariantCulture;
        if (value == null) return "";
        if (value is float) return ((float)value).ToString("R", ci);
        if (value is double) return ((double)value).ToString("R", ci);
        if (value is Array)
        {
            var arr = (Array)value;
            var parts = new string[arr.Length];
            for (int i = 0; i < arr.Length; i++) parts[i] = Render(arr.GetValue(i));
            return string.Join(" ", parts);
        }
        return Convert.ToString(value, ci);
    }

    static string ToHex(byte[] data)
    {
        var c = new char[data.Length * 2];
        const string H = "0123456789abcdef";
        for (int i = 0; i < data.Length; i++)
        {
            c[i * 2] = H[data[i] >> 4];
            c[i * 2 + 1] = H[data[i] & 0xF];
        }
        return new string(c);
    }
}
