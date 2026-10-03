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

// Headless MAVFTP oracle: Mission Planner's own MAVFtp (ExtLibs/ArduPilot/Mavlink/MAVFtp.cs), built
// from the pinned tree under mono, for crates/mp-ftp.
//
// Beside MpLog.cs, as the other harnesses are; its output is testdata/ftp/csharp-tables.txt.
// Build (from a copy of ExtLibs, as regen-log.sh makes one):
//
//   msbuild -t:Restore,Build -p:Configuration=Release -p:CopyLocalLockFileAssemblies=true \
//       ExtLibs/ArduPilot/MissionPlanner.ArduPilot.csproj      # needs Resources/quad2.png
//   OUT=ExtLibs/ArduPilot/bin/Release/netstandard2.0
//   mcs -nowarn:1685 -out:$OUT/MpFtp.exe -r:$OUT/MissionPlanner.ArduPilot.dll \
//       -r:$OUT/MAVLink.dll -r:$OUT/MissionPlanner.Comms.dll -r:$OUT/Interfaces.dll \
//       -r:$OUT/MissionPlanner.Utilities.dll -r:<mono>/4.5/Facades/netstandard.dll MpFtp.cs
//
// Run:    LC_ALL=C mono MpFtp.exe tables > testdata/ftp/csharp-tables.txt
//         mono MpFtp.exe sitl <host> <port> <outdir>
//
// tables  what MAVFtp.cs defines without a vehicle: the ToString of every errno, FTPErrorCode and
//         FTPOpcode byte (the words its exceptions use), crc_crc32 over fixed inputs, and the bytes
//         FTPPayloadHeader's conversion makes of sample requests. crates/mp-ftp/tests/csharp.rs
//         holds the Rust to every line.
// sitl    MAVFtp itself over a MAVLinkInterface on a TCP port: the listings of / and @SYS,
//         GetFile of @SYS/uarts.txt (plain, as ConfigSerial.cs:105 reads it) and of
//         @PARAM/param.pck (burst, 64 bytes a read), and kCmdCalcFileCRC32 of param.pck. The files
//         are written to <outdir>; the rest is printed.

using System;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
using MissionPlanner;
using MissionPlanner.ArduPilot.Mavlink;
using MissionPlanner.Comms;

public static class MpFtp
{
    public static int Main(string[] args)
    {
        if (args.Length == 1 && args[0] == "tables")
        {
            Tables();
            return 0;
        }
        if (args.Length == 4 && args[0] == "sitl")
        {
            return Sitl(args[1], args[2], args[3]);
        }
        Console.Error.WriteLine("usage: MpFtp.exe tables | sitl <host> <port> <outdir>");
        return 2;
    }

    static string Hex(byte[] bytes)
    {
        var sb = new StringBuilder();
        foreach (var b in bytes)
            sb.Append(b.ToString("x2"));
        return sb.ToString();
    }

    static void Tables()
    {
        for (int i = 0; i < 256; i++)
            Console.WriteLine("errno " + i + " " + ((MAVFtp.errno)i).ToString());
        for (int i = 0; i < 256; i++)
            Console.WriteLine("error " + i + " " + ((MAVFtp.FTPErrorCode)i).ToString());
        for (int i = 0; i < 256; i++)
            Console.WriteLine("opcode " + i + " " + ((MAVFtp.FTPOpcode)i).ToString());

        var numbered = Enumerable.Range(0, 5000).Select(i => (byte)(i % 251)).ToArray();
        var every = Enumerable.Range(0, 256).Select(i => (byte)i).ToArray();
        Console.WriteLine("crc empty " + MAVFtp.crc_crc32(0, new byte[0]).ToString("x8"));
        Console.WriteLine("crc check " + MAVFtp.crc_crc32(0, Encoding.ASCII.GetBytes("123456789")).ToString("x8"));
        Console.WriteLine("crc check-from-ones " + MAVFtp.crc_crc32(uint.MaxValue, Encoding.ASCII.GetBytes("123456789")).ToString("x8"));
        Console.WriteLine("crc every-byte " + MAVFtp.crc_crc32(0, every).ToString("x8"));
        Console.WriteLine("crc numbered-5000 " + MAVFtp.crc_crc32(0, numbered).ToString("x8"));

        // Requests as the kCmd methods build them, numbered as a fresh MAVFtp numbers them in each
        // of the sequences crates/mp-ftp/tests/csharp.rs runs, through the conversion every send
        // goes through (MAVFtp.cs:2394-2407).
        // GetFile(file, cancel, true, 110): MAVFtp.cs:564, 600-606, 700-707.
        Payload("get-reset", new MAVFtp.FTPPayloadHeader
        {
            opcode = MAVFtp.FTPOpcode.kCmdResetSessions,
            seq_number = 0,
            session = 0
        });
        Payload("get-open", new MAVFtp.FTPPayloadHeader
        {
            opcode = MAVFtp.FTPOpcode.kCmdOpenFileRO,
            data = Encoding.UTF8.GetBytes("@SYS/uarts.txt"),
            seq_number = 1,
            session = 0
        });
        Payload("get-burst", new MAVFtp.FTPPayloadHeader
        {
            opcode = MAVFtp.FTPOpcode.kCmdBurstReadFile,
            seq_number = 2,
            session = 0,
            offset = 0,
            size = 110
        });
        // kCmdListDirectory("/APM/") of seven entries: MAVFtp.cs:1269-1283, 1419-1422.
        Payload("list-timed", new MAVFtp.FTPPayloadHeader
        {
            opcode = MAVFtp.FTPOpcode.kCmdListDirectoryWithTime,
            data = Encoding.UTF8.GetBytes("/APM"),
            seq_number = 0,
            offset = 0
        });
        Payload("list-plain", new MAVFtp.FTPPayloadHeader
        {
            opcode = MAVFtp.FTPOpcode.kCmdListDirectory,
            data = Encoding.UTF8.GetBytes("/APM"),
            seq_number = 1,
            offset = 0
        });
        Payload("list-next", new MAVFtp.FTPPayloadHeader
        {
            opcode = MAVFtp.FTPOpcode.kCmdListDirectory,
            data = Encoding.UTF8.GetBytes("/APM"),
            seq_number = 2,
            offset = 7
        });
        // UploadFile("/APM/up.bin", ...) of 1000 bytes: MAVFtp.cs:1145-1151, 2218-2224, 2335-2339.
        Payload("put-create", new MAVFtp.FTPPayloadHeader
        {
            opcode = MAVFtp.FTPOpcode.kCmdCreateFile,
            data = Encoding.UTF8.GetBytes("/APM/up.bin"),
            seq_number = 1,
            session = 0
        });
        Payload("put-write", new MAVFtp.FTPPayloadHeader
        {
            opcode = MAVFtp.FTPOpcode.kCmdWriteFile,
            seq_number = 2,
            offset = 0,
            session = 0,
            data = numbered.Take(80).ToArray(),
            size = 80
        });
        // kCmdRename, kCmdCalcFileCRC32, kCmdRemoveFile and kCmdTerminateSession on a fresh
        // MAVFtp: MAVFtp.cs:1836-1843, 919-926, 1752-1758, 1965-1970.
        Payload("rename", new MAVFtp.FTPPayloadHeader
        {
            opcode = MAVFtp.FTPOpcode.kCmdRename,
            data = Encoding.UTF8.GetBytes("/APM/a.txt" + "\0" + "/APM/b.txt"),
            seq_number = 0,
            session = 0
        });
        Payload("crc", new MAVFtp.FTPPayloadHeader
        {
            opcode = MAVFtp.FTPOpcode.kCmdCalcFileCRC32,
            data = Encoding.UTF8.GetBytes("@PARAM/param.pck"),
            seq_number = 0,
            session = 0
        });
        Payload("remove-long-path", new MAVFtp.FTPPayloadHeader
        {
            opcode = MAVFtp.FTPOpcode.kCmdRemoveFile,
            data = Encoding.UTF8.GetBytes(new string('a', 300)),
            seq_number = 0,
            session = 0
        });
        Payload("terminate", new MAVFtp.FTPPayloadHeader
        {
            opcode = MAVFtp.FTPOpcode.kCmdTerminateSession,
            seq_number = 0,
            session = 0
        });
        // kCmdReadFile at the last sequence number: MAVFtp.cs:1550-1557.
        Payload("read-65535", new MAVFtp.FTPPayloadHeader
        {
            opcode = MAVFtp.FTPOpcode.kCmdReadFile,
            seq_number = 65535,
            offset = 240,
            session = 0,
            size = 80
        });
    }

    static void Payload(string name, MAVFtp.FTPPayloadHeader header)
    {
        byte[] bytes = header;
        Console.WriteLine("payload " + name + " " + Hex(bytes));
    }

    static int Sitl(string host, string port, string outdir)
    {
        var mav = new MAVLinkInterface();
        var tcp = new TcpSerial { Host = host, Port = port };
        mav.BaseStream = tcp;
        mav.BaseStream.Open();
        // MainV2's SerialReader: read packets for as long as the link is open.
        var reader = new Thread(() =>
        {
            while (mav.BaseStream != null && mav.BaseStream.IsOpen)
            {
                try
                {
                    mav.readPacket();
                }
                catch (Exception)
                {
                }
            }
        }) { IsBackground = true };
        reader.Start();
        var deadline = DateTime.Now.AddSeconds(20);
        while (mav.MAVlist.Count == 0 && DateTime.Now < deadline)
            Thread.Sleep(100);
        var id = mav.MAVlist.First();
        Console.WriteLine("vehicle " + id.sysid + " " + id.compid);
        var ftp = new MAVFtp(mav, id.sysid, id.compid);
        var cancel = new CancellationTokenSource();

        foreach (var dir in new[] { "/", "@SYS" })
        {
            foreach (var entry in ftp.kCmdListDirectory(dir, cancel))
                Console.WriteLine("list " + dir + " " + entry);
        }

        var uarts = ftp.GetFile("@SYS/uarts.txt", cancel, false);
        Console.WriteLine("uarts " + (uarts == null ? -1 : uarts.Length));
        if (uarts != null)
            File.WriteAllBytes(Path.Combine(outdir, "uarts.txt"), uarts.ToArray());

        var param = ftp.GetFile("@PARAM/param.pck", cancel, true, 64);
        Console.WriteLine("param " + (param == null ? -1 : param.Length) + " crc " +
                          (param == null ? "none" : MAVFtp.crc_crc32(0, param.ToArray()).ToString("x8")));
        if (param != null)
            File.WriteAllBytes(Path.Combine(outdir, "param.pck"), param.ToArray());
        uint crc = 0;
        var answered = ftp.kCmdCalcFileCRC32("@PARAM/param.pck", ref crc, cancel);
        Console.WriteLine("vehicle-crc " + answered + " " + crc.ToString("x8"));

        try
        {
            var threads = ftp.GetFile("@SYS/threads.txt", cancel, true);
            Console.WriteLine("threads " + (threads == null ? -1 : threads.Length));
        }
        catch (Exception ex)
        {
            Console.WriteLine("threads threw " + ex.GetType().Name + ": " + ex.Message);
        }

        mav.BaseStream.Close();
        return 0;
    }
}
