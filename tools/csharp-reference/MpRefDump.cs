// Headless reference dumper for differential testing (DELIVERABLES.md D19).
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
            default: Usage(); return 2;
        }
    }

    static void Usage()
    {
        Console.Error.WriteLine("usage: MpRefDump infos | tlog <file> [max_frames]");
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

        Console.WriteLine("index,msgid,seq,sysid,compid,payload_len,crc16,frame_hex");
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

                Console.WriteLine("{0},{1},{2},{3},{4},{5},{6},{7}",
                    index, msg.msgid, msg.seq, msg.sysid, msg.compid,
                    msg.payloadlength, msg.crc16, ToHex(msg.buffer));
                index++;
            }
        }
        Console.Error.WriteLine("# frames={0} badCRC={1} badLength={2}", index, parser.badCRC, parser.badLength);
        return 0;
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
