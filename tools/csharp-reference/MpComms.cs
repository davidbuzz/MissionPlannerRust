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

// Headless comms oracle: runs Mission Planner's own CommsNTRIP, WebSocket and UdpSerialConnect
// (ExtLibs/Comms/CommsNTRIP.cs, CommsWebSocket.cs, CommsUDPSerialConnect.cs) against peers this
// harness plays on 127.0.0.1, and writes down what crossed the wire, so
// crates/mp-transport/tests/csharp_goldens.rs holds the Rust to the C# rather than to our reading
// of it. Built by regen-comms.sh straight from those sources with csc, against log4net.
//
// Run:    mono MpComms.exe <ntrip|gga|ws|udp> <input or -> <outdir>
//
//   ntrip <ntrip-cases.txt>  one line per case, a line starting `#` is a comment:
//                              request <v1|v2> <url>    the bytes Open(url) sends to the caster,
//                                                       which answers `ICY 200 OK`; `{port}` in
//                                                       the url is the caster's port
//                              response <status line>   what Open does when the caster answers a
//                                                       v2 request for /MOUNT with that line (and,
//                                                       for a source table, a table and a close)
//                            -> ntrip-requests.txt, one `<case>\t<result>` line per case, where the
//                               result is the escaped request, `ok`, or `error <message>`.
//   gga <gga-positions.txt>  one `<lat> <lng> <alt>` per line: each is set on a connected
//                            CommsNTRIP, its 30 s gate is reopened (`_lastnmea` back to MinValue)
//                            and BytesToRead is read, which is what sends the sentence
//                            -> ntrip-gga.txt, `<lat> <lng> <alt>\t<sentence or none>`, then the
//                               cadence lines and the sentence Open itself sends.
//   ws -                     the websocket conversation, step by step -> ws.txt
//   udp -                    UdpSerialConnect's reads, writes, BytesToRead and IsInRange -> udp.txt
//   reconnect -              CommsNTRIP after the caster hangs up -> ntrip-reconnect.txt
//
// Bytes are written escaped: printable ASCII as itself, `\\`, `\r`, `\n`, and `\xHH` for the rest.
// A port number that changes from run to run is written `{port}`; a websocket key, `{key}`; the
// time in a GGA sentence is kept, because the Rust test rebuilds the sentence at that time.

using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Reflection;
using System.Security.Cryptography;
using System.Text;
using System.Threading;
using MissionPlanner.Comms;

public static class MpComms
{
    static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    public static int Main(string[] args)
    {
        Thread.CurrentThread.CurrentCulture = Inv;
        if (args.Length != 3)
        {
            Console.Error.WriteLine("usage: MpComms <ntrip|gga|ws|udp|reconnect> <input or -> <outdir>");
            return 2;
        }
        Directory.CreateDirectory(args[2]);
        switch (args[0])
        {
            case "ntrip": return Ntrip(args[1], Path.Combine(args[2], "ntrip-requests.txt"));
            case "gga": return Gga(args[1], Path.Combine(args[2], "ntrip-gga.txt"));
            case "reconnect": return Reconnect(Path.Combine(args[2], "ntrip-reconnect.txt"));
            case "ws": return Ws(Path.Combine(args[2], "ws.txt"));
            case "udp": return Udp(Path.Combine(args[2], "udp.txt"));
        }
        Console.Error.WriteLine("unknown verb " + args[0]);
        return 2;
    }

    // ---------------------------------------------------------------- helpers

    static string Esc(byte[] data, int count)
    {
        var sb = new StringBuilder();
        for (int i = 0; i < count; i++)
        {
            byte b = data[i];
            if (b == (byte)'\\') sb.Append("\\\\");
            else if (b == (byte)'\r') sb.Append("\\r");
            else if (b == (byte)'\n') sb.Append("\\n");
            else if (b >= 0x20 && b < 0x7f) sb.Append((char)b);
            else sb.Append("\\x").Append(b.ToString("X2", Inv));
        }
        return sb.ToString();
    }

    static string Esc(byte[] data) { return Esc(data, data.Length); }

    static string Esc(string text) { return Esc(Encoding.UTF8.GetBytes(text)); }

    static TcpListener Listen()
    {
        var listener = new TcpListener(IPAddress.Loopback, 0);
        listener.Start();
        return listener;
    }

    static int PortOf(TcpListener listener) { return ((IPEndPoint)listener.LocalEndpoint).Port; }

    /// Waits for a connection while `alive` says the other side may still make one.
    static TcpClient Accept(TcpListener listener, Func<bool> alive, int ms = 5000)
    {
        var deadline = DateTime.UtcNow.AddMilliseconds(ms);
        while (DateTime.UtcNow < deadline)
        {
            if (listener.Pending()) return listener.AcceptTcpClient();
            if (!alive()) return null;
            Thread.Sleep(5);
        }
        return null;
    }

    /// Reads byte by byte until `end` has been read, so nothing after it is consumed.
    static byte[] ReadUntil(Stream s, string end)
    {
        var got = new List<byte>();
        var tail = Encoding.ASCII.GetBytes(end);
        while (true)
        {
            int b = s.ReadByte();
            if (b < 0) break;
            got.Add((byte)b);
            if (got.Count >= tail.Length && got.Skip(got.Count - tail.Length).SequenceEqual(tail)) break;
        }
        return got.ToArray();
    }

    static string ReadLineOrNone(TcpClient c, int ms)
    {
        c.ReceiveTimeout = ms;
        try
        {
            var line = ReadUntil(c.GetStream(), "\n");
            return line.Length == 0 ? "none" : Esc(line);
        }
        catch (IOException) { return "none"; }
    }

    static string Port(string text, int port) { return text.Replace(":" + port.ToString(Inv), ":{port}"); }

    // ---------------------------------------------------------------- ntrip: the request and the answer

    static int Ntrip(string cases, string output)
    {
        var lines = new List<string>();
        foreach (var raw in File.ReadAllLines(cases))
        {
            var line = raw.Trim();
            if (line.Length == 0 || line.StartsWith("#")) continue;
            var verb = line.Split(' ')[0];
            var rest = line.Substring(verb.Length).Trim();
            if (verb == "request")
            {
                var version = rest.Split(' ')[0];
                var url = rest.Substring(version.Length).Trim();
                var result = NtripCase(version == "v1", url, "ICY 200 OK", null);
                lines.Add(line + "\t" + (result[1] == "ok" ? result[0] : result[1]));
            }
            else if (verb == "response")
            {
                string body = rest.Contains("SOURCETABLE")
                    ? "STR;MOUNT;Canberra;RTCM 3.2;1005(10),1077(1);2;GPS+GLO;SNIP;AUS;-35.36;149.17;1;0;sNTRIP;none;B;N;9600;\r\nENDSOURCETABLE\r\n"
                    : null;
                lines.Add(line + "\t" + NtripCase(false, "ntrip://127.0.0.1:{port}/MOUNT", rest, body)[1]);
            }
            else throw new Exception("unknown case " + line);
        }
        File.WriteAllLines(output, lines);
        return 0;
    }

    /// Runs one Open against a caster that answers `status` (then `body`, then hangs up, if there
    /// is a body), and gives back the request it received and what Open did: `ok` or `error ...`.
    static string[] NtripCase(bool v1, string template, string status, string body)
    {
        var listener = Listen();
        int port = PortOf(listener);
        var url = template.Replace("{port}", port.ToString(Inv));
        var nt = new CommsNTRIP();
        nt.ntrip_v1 = v1;
        Exception failure = null;
        var opener = new Thread(() =>
        {
            try { nt.Open(url); }
            catch (Exception e) { failure = e; }
        });
        opener.Start();
        string request = "none";
        using (var peer = Accept(listener, () => opener.IsAlive))
        {
            if (peer != null)
            {
                var stream = peer.GetStream();
                peer.ReceiveTimeout = 5000;
                request = Port(Esc(ReadUntil(stream, "\r\n\r\n")), port);
                var answer = Encoding.ASCII.GetBytes(status + "\r\n" + (body ?? ""));
                stream.Write(answer, 0, answer.Length);
                stream.Flush();
                // The source table's ReadToEnd returns when the caster hangs up.
                if (body != null) peer.Client.Shutdown(SocketShutdown.Send);
                opener.Join(5000);
            }
        }
        opener.Join(5000);
        listener.Stop();
        try { nt.Close(); } catch { }
        var outcome = failure == null
            ? "ok"
            : "error " + failure.GetType().Name + " " + Port(Esc(failure.Message), port);
        return new[] { request, outcome };
    }

    // ---------------------------------------------------------------- ntrip: the GGA sentence

    static readonly FieldInfo LastNmea =
        typeof(CommsNTRIP).GetField("_lastnmea", BindingFlags.NonPublic | BindingFlags.Instance);

    static int Gga(string positions, string output)
    {
        var lines = new List<string>();
        var listener = Listen();
        int port = PortOf(listener);
        var nt = new CommsNTRIP();

        // The sentence Open sends itself, straight after the caster's answer, when the position
        // was set first - as the RTK page sets it (ConfigSerialInjectGPS.cs:339-344).
        nt.lat = -35.363261;
        nt.lng = 149.165230;
        nt.alt = 584.0;
        var opener = new Thread(() => nt.Open("ntrip://127.0.0.1:" + port.ToString(Inv) + "/MOUNT"));
        opener.Start();
        var peer = Accept(listener, () => opener.IsAlive);
        peer.ReceiveTimeout = 5000;
        ReadUntil(peer.GetStream(), "\r\n\r\n");
        var ok = Encoding.ASCII.GetBytes("ICY 200 OK\r\n");
        peer.GetStream().Write(ok, 0, ok.Length);
        opener.Join();
        lines.Add("open -35.363261 149.165230 584.0\t" + ReadLineOrNone(peer, 2000));

        foreach (var raw in File.ReadAllLines(positions))
        {
            var line = raw.Trim();
            if (line.Length == 0 || line.StartsWith("#")) continue;
            var parts = line.Split(new[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
            nt.lat = double.Parse(parts[0], Inv);
            nt.lng = double.Parse(parts[1], Inv);
            nt.alt = double.Parse(parts[2], Inv);
            LastNmea.SetValue(nt, DateTime.MinValue);
            var unused = nt.BytesToRead;
            lines.Add(parts[0] + " " + parts[1] + " " + parts[2] + "\t" + ReadLineOrNone(peer, 300));
        }

        // The cadence: nothing again at once, nothing at 29 s, a sentence at 31 s.
        nt.lat = 1; nt.lng = 1; nt.alt = 1;
        LastNmea.SetValue(nt, DateTime.MinValue);
        var a = nt.BytesToRead;
        lines.Add("cadence first\t" + (ReadLineOrNone(peer, 300) != "none" ? "sent" : "none"));
        a = nt.BytesToRead;
        lines.Add("cadence again\t" + (ReadLineOrNone(peer, 300) != "none" ? "sent" : "none"));
        LastNmea.SetValue(nt, DateTime.Now.AddSeconds(-29));
        a = nt.BytesToRead;
        lines.Add("cadence 29s\t" + (ReadLineOrNone(peer, 300) != "none" ? "sent" : "none"));
        LastNmea.SetValue(nt, DateTime.Now.AddSeconds(-31));
        a = nt.BytesToRead;
        lines.Add("cadence 31s\t" + (ReadLineOrNone(peer, 300) != "none" ? "sent" : "none"));
        // A position of exactly 0,0 is no position: nothing is sent.
        nt.lat = 0; nt.lng = 0;
        LastNmea.SetValue(nt, DateTime.MinValue);
        a = nt.BytesToRead;
        lines.Add("cadence zero\t" + (ReadLineOrNone(peer, 300) != "none" ? "sent" : "none"));

        peer.Close();
        listener.Stop();
        File.WriteAllLines(output, lines);
        return 0;
    }

    // ---------------------------------------------------------------- ntrip: the caster hangs up

    static int Reconnect(string output)
    {
        var lines = new List<string>();
        var listener = Listen();
        int port = PortOf(listener);
        var accepted = 0;
        var nt = new CommsNTRIP();
        var buf = new byte[16];

        // Every connection the caster sees gets a request read and `ICY 200 OK`.
        Func<TcpClient> serve = () =>
        {
            var peer = Accept(listener, () => true, 3000);
            if (peer == null) return null;
            accepted++;
            peer.ReceiveTimeout = 5000;
            ReadUntil(peer.GetStream(), "\r\n\r\n");
            var ok = Encoding.ASCII.GetBytes("ICY 200 OK\r\n");
            peer.GetStream().Write(ok, 0, ok.Length);
            return peer;
        };

        TcpClient current = null;
        var t = new Thread(() => current = serve());
        t.Start();
        nt.Open("ntrip://127.0.0.1:" + port.ToString(Inv) + "/MOUNT");
        t.Join();
        lines.Add("open\taccepted " + accepted + " isopen " + nt.IsOpen);

        for (int round = 1; round <= 4; round++)
        {
            // The caster hangs up, gracefully.
            current.Client.Shutdown(SocketShutdown.Both);
            current.Close();
            Thread.Sleep(200);
            int n;
            try { n = nt.Read(buf, 0, buf.Length); lines.Add("round " + round + " read after hangup\t" + n + " isopen " + nt.IsOpen); }
            catch (Exception e) { lines.Add("round " + round + " read after hangup\terror " + e.Message); }
            // Writes are what notice: the first goes out and draws a reset, the second fails.
            for (int w = 1; w <= 2; w++)
            {
                try { nt.Write("x"); lines.Add("round " + round + " write " + w + "\tisopen " + nt.IsOpen); }
                catch (Exception e) { lines.Add("round " + round + " write " + w + "\terror " + e.Message); }
                Thread.Sleep(200);
            }
            // And the next read reconnects, then throws anyway.
            var before = accepted;
            var server = new Thread(() => current = serve());
            server.Start();
            try { n = nt.Read(buf, 0, buf.Length); lines.Add("round " + round + " read\t" + n); }
            catch (Exception e) { lines.Add("round " + round + " read\terror " + Port(Esc(e.Message), port)); }
            server.Join();
            lines.Add("round " + round + " reconnected\t" + (accepted - before) + " isopen " + nt.IsOpen);
            if (current == null) break;
            // A reconnected link reads again.
            var data = new byte[] { 0xD3, 0x00, 0x13 };
            current.GetStream().Write(data, 0, data.Length);
            Thread.Sleep(100);
            try { n = nt.Read(buf, 0, buf.Length); lines.Add("round " + round + " read again\t" + Esc(buf, n)); }
            catch (Exception e) { lines.Add("round " + round + " read again\terror " + e.Message); }
        }
        try { nt.Close(); } catch { }
        listener.Stop();
        File.WriteAllLines(output, lines);
        return 0;
    }

    // ---------------------------------------------------------------- websocket

    const string Guid = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

    static byte[] ServerFrame(bool fin, int opcode, byte[] payload)
    {
        var f = new List<byte>();
        f.Add((byte)((fin ? 0x80 : 0) | opcode));
        if (payload.Length < 126) f.Add((byte)payload.Length);
        else { f.Add(126); f.Add((byte)(payload.Length >> 8)); f.Add((byte)payload.Length); }
        f.AddRange(payload);
        return f.ToArray();
    }

    /// Reads one frame the client sent and describes it: fin, opcode, whether it was masked, and
    /// the payload unmasked.
    static string ClientFrame(TcpClient c, int ms)
    {
        c.ReceiveTimeout = ms;
        var s = c.GetStream();
        try
        {
            int b0 = s.ReadByte(), b1 = s.ReadByte();
            if (b0 < 0 || b1 < 0) return "eof";
            long len = b1 & 0x7f;
            if (len == 126) len = (s.ReadByte() << 8) | s.ReadByte();
            else if (len == 127) { len = 0; for (int i = 0; i < 8; i++) len = (len << 8) | (long)s.ReadByte(); }
            var mask = new byte[4];
            bool masked = (b1 & 0x80) != 0;
            if (masked) for (int i = 0; i < 4; i++) mask[i] = (byte)s.ReadByte();
            var payload = new byte[len];
            for (int i = 0; i < len; i++) payload[i] = (byte)(s.ReadByte() ^ (masked ? mask[i % 4] : 0));
            return "fin " + ((b0 & 0x80) != 0 ? 1 : 0) + " rsv " + ((b0 >> 4) & 7) + " opcode " + (b0 & 0x0f)
                   + " masked " + (masked ? 1 : 0) + " payload " + Esc(payload);
        }
        catch (IOException) { return "none"; }
    }

    static string Handshake(TcpClient peer, int port)
    {
        peer.ReceiveTimeout = 5000;
        var request = Encoding.ASCII.GetString(ReadUntil(peer.GetStream(), "\r\n\r\n"));
        var key = request.Split(new[] { "\r\n" }, StringSplitOptions.None)
            .First(l => l.StartsWith("Sec-WebSocket-Key:", StringComparison.OrdinalIgnoreCase))
            .Split(':')[1].Trim();
        var accept = Convert.ToBase64String(SHA1.Create().ComputeHash(Encoding.ASCII.GetBytes(key + Guid)));
        var response = Encoding.ASCII.GetBytes("HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: " + accept + "\r\n\r\n");
        peer.GetStream().Write(response, 0, response.Length);
        return Port(Esc(request.Replace(key, "{key}")), port);
    }

    static readonly FieldInfo SocketIo =
        typeof(MissionPlanner.Comms.WebSocket).GetField("socketio", BindingFlags.NonPublic | BindingFlags.Instance);

    static void Until(Func<bool> condition, int ms = 3000)
    {
        var deadline = DateTime.UtcNow.AddMilliseconds(ms);
        while (!condition() && DateTime.UtcNow < deadline) Thread.Sleep(5);
    }

    static int Ws(string output)
    {
        var lines = new List<string>();
        var listener = Listen();
        int port = PortOf(listener);
        var url = "ws://LocalHost:" + port.ToString(Inv) + "/mav/./link?x=1";
        CommsBase.Settings += (name, value, set) => set ? value : (name == "WS_url" ? url : "");
        var ws = new MissionPlanner.Comms.WebSocket();
        Exception failure = null;
        var opener = new Thread(() => { try { ws.Open(); } catch (Exception e) { failure = e; } });
        opener.Start();

        var peer = Accept(listener, () => opener.IsAlive);
        lines.Add("handshake\t" + Handshake(peer, port));
        opener.Join(5000);
        lines.Add("open\tisopen " + ws.IsOpen + (failure != null ? " error " + failure.Message : ""));
        lines.Add("client sends\t" + ClientFrame(peer, 2000));

        // A binary message is the byte stream.
        var s = peer.GetStream();
        var frame = ServerFrame(true, 2, new byte[] { 0xFD, 0x01, 0x02, 0x03 });
        s.Write(frame, 0, frame.Length);
        Until(() => ws.BytesToRead == 4);
        var buf = new byte[64];
        var n = ws.Read(buf, 0, buf.Length);
        lines.Add("server binary FD010203\tread " + Esc(buf, n));

        // A message in two fragments arrives as its bytes, in order.
        frame = ServerFrame(false, 2, new byte[] { 0xFD, 0x05 });
        s.Write(frame, 0, frame.Length);
        frame = ServerFrame(true, 0, new byte[] { 0x06, 0x07 });
        s.Write(frame, 0, frame.Length);
        Until(() => ws.BytesToRead == 4);
        n = ws.Read(buf, 0, buf.Length);
        lines.Add("server binary FD05 then continuation 0607\tread " + Esc(buf, n));

        ws.Write(new byte[] { 0xFD, 0x09, 0x08 }, 0, 3);
        lines.Add("client write FD0908\t" + ClientFrame(peer, 2000));

        // socket.io: the engine.io open packet names the session.
        frame = ServerFrame(true, 1, Encoding.ASCII.GetBytes("0{\"sid\":\"abc\",\"upgrades\":[]}"));
        s.Write(frame, 0, frame.Length);
        Until(() => (bool)SocketIo.GetValue(ws));
        lines.Add("server text 0{sid}\tsocketio " + SocketIo.GetValue(ws));
        ws.Write(new byte[] { 0x01, 0x02, 0x03 }, 0, 3);
        lines.Add("client write 010203\t" + ClientFrame(peer, 2000));

        // The probe's answer.
        frame = ServerFrame(true, 1, Encoding.ASCII.GetBytes("3probe"));
        s.Write(frame, 0, frame.Length);
        lines.Add("server text 3probe\t" + ClientFrame(peer, 2000));
        lines.Add("server text 3probe\t" + ClientFrame(peer, 2000));

        // Ping.
        frame = ServerFrame(true, 9, Encoding.ASCII.GetBytes("hi"));
        s.Write(frame, 0, frame.Length);
        lines.Add("server ping hi\t" + ClientFrame(peer, 2000));

        // Other text is only logged.
        frame = ServerFrame(true, 1, Encoding.ASCII.GetBytes("42[\"x\"]"));
        s.Write(frame, 0, frame.Length);
        lines.Add("server text 42\t" + ClientFrame(peer, 300) + " bytestoread " + ws.BytesToRead);

        // The server drops the connection without a close frame: the reader reconnects.
        peer.Client.LingerState = new LingerOption(true, 0);
        peer.Close();
        var again = Accept(listener, () => true, 3000);
        lines.Add("drop\treconnected " + (again != null));
        if (again != null)
        {
            lines.Add("handshake again\t" + Handshake(again, port));
            lines.Add("client sends again\t" + ClientFrame(again, 2000));
            Until(() => ws.IsOpen);
            lines.Add("isopen again\t" + ws.IsOpen);

            // A close frame: does the client answer, and is it open after?
            frame = ServerFrame(true, 8, new byte[] { 0x03, 0xE8 });
            again.GetStream().Write(frame, 0, frame.Length);
            lines.Add("server close 1000\t" + ClientFrame(again, 1000));
            Until(() => !ws.IsOpen, 1000);
            lines.Add("after close\tisopen " + ws.IsOpen);
            var third = Accept(listener, () => true, 500);
            lines.Add("after close\treconnected " + (third != null));
            again.Close();
        }
        try { ws.Close(); } catch { }
        listener.Stop();
        File.WriteAllLines(output, lines);
        return 0;
    }

    // ---------------------------------------------------------------- udp client

    static int Udp(string output)
    {
        var lines = new List<string>();
        foreach (var address in new[] { "224.0.0.0", "224.0.0.1", "239.255.255.255", "240.0.0.0", "223.255.255.255",
                                        "10.0.0.1", "127.0.0.1", "255.255.255.255", "0.0.0.0", "::1", "fe80::e000:1", "::ffff:224.0.0.1", "ff02::1" })
            lines.Add("isinrange " + address + "\t" + UdpSerialConnect.IsInRange("224.0.0.0", "239.255.255.255", address));

        var peer = new UdpClient(new IPEndPoint(IPAddress.Loopback, 0));
        int port = ((IPEndPoint)peer.Client.LocalEndPoint).Port;
        var u = new UdpSerialConnect();
        u.Open("127.0.0.1", port.ToString(Inv));
        lines.Add("open\tisopen " + u.IsOpen + " portname " + u.PortName.Replace(port.ToString(Inv), "{port}") + " bytestoread " + u.BytesToRead);

        var from = new IPEndPoint(IPAddress.Any, 0);
        u.Write(new byte[] { 1, 2, 3, 4, 5 }, 0, 5);
        var got = peer.Receive(ref from);
        lines.Add("write 0102030405 offset 0 length 5\t" + Esc(got) + (from.Port != port ? " from another port" : " from the same port"));
        u.Write(new byte[] { 1, 2, 3, 4, 5 }, 2, 3);
        got = peer.Receive(ref from);
        lines.Add("write 0102030405 offset 2 length 3\t" + Esc(got));

        var big = Enumerable.Range(0, 100).Select(i => (byte)i).ToArray();
        var small = Enumerable.Range(200, 50).Select(i => (byte)i).ToArray();
        peer.Send(big, big.Length, from);
        peer.Send(small, small.Length, from);
        Thread.Sleep(100);
        lines.Add("two datagrams 100 then 50\tbytestoread " + u.BytesToRead);
        var buf = new byte[256];
        var n = u.Read(buf, 0, 10);
        lines.Add("read 10\tgot " + n + " first " + buf[0] + " last " + buf[n - 1] + " bytestoread " + u.BytesToRead);
        n = u.Read(buf, 0, 200);
        lines.Add("read 200\tgot " + n + " first " + buf[0] + " last " + buf[n - 1] + " bytestoread " + u.BytesToRead);
        var started = DateTime.UtcNow;
        n = u.Read(buf, 0, 10);
        lines.Add("read 10 of nothing\tgot " + n + (DateTime.UtcNow - started > TimeSpan.FromMilliseconds(400) ? " after the timeout" : " at once"));
        u.Close();
        lines.Add("close\tisopen " + u.IsOpen);
        peer.Close();

        // Joining the same group twice on one socket, which the 30 s loop in Open does.
        var m = new UdpClient(0, AddressFamily.InterNetwork);
        try { m.JoinMulticastGroup(IPAddress.Parse("239.255.10.10")); lines.Add("join once\tok"); }
        catch (Exception e) { lines.Add("join once\terror " + e.GetType().Name); }
        try { m.JoinMulticastGroup(IPAddress.Parse("239.255.10.10")); lines.Add("join twice\tok"); }
        catch (Exception e) { lines.Add("join twice\terror " + e.GetType().Name + " " + ((e as SocketException) != null ? ((SocketException)e).SocketErrorCode.ToString() : "")); }
        m.Close();

        File.WriteAllLines(output, lines);
        return 0;
    }
}
