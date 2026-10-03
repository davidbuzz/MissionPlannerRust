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

// Headless CurrentState oracle: Mission Planner's own ExtLibs/ArduPilot/CurrentState.cs fed a
// recorded .tlog, and its getters on constructed inputs, for crates/mp-vehicle/tests/
// current_state_oracle.rs.
//
// Build:  see regen-state.sh (mcs against the msbuild output of ExtLibs/ArduPilot)
// Run:    mono MpState.exe tlog    <log.tlog> <out.csv>
//         mono MpState.exe getters <out.csv>
//         mono MpState.exe fence   <out.csv>
//
// tlog     The Telemetry Logs playback, without its screen: a MAVLinkInterface in log-read mode
//          (Log/MavlinkLog.cs:81-96), one readPacketAsync per packet - which stamps the sender's
//          CurrentState.datetime with the packet's time (MAVLinkInterface.cs:6649) and hands the
//          packet to every CurrentState (MAVLinkInterface.cs:5369, CurrentState.cs:2278) - and
//          after each packet UpdateCurrentSettings(null, true, ...) on every detected vehicle, as
//          Log/MavlinkLog.cs:141-144 and GCSViews/FlightData.cs:3534-3541 do. After each packet
//          the fields of vehicle 1:1 are written as a row, when any but `datetime` changed, keyed by the
//          packet's number and the file position after it - a read that meets bytes that are not
//          MAVLink can take several records into one packet (MAVLinkInterface.cs:5012-5021), and
//          the position is what lines the two up there; then the packet count, and the custom
//          field names the NAMED_VALUE_FLOATs claimed.
//
//          One thing is pinned that the C# leaves to the wall clock: a new CurrentState starts
//          `lastsecondcounter` at DateTime.Now (CurrentState.cs:128), which decides whether the
//          first update ticks the once-a-second counters. On a live link the state is made as its
//          first packet is read, stamped with DateTime.Now (MAVLinkInterface.cs:4721), so the two
//          agree; here the counter is set by reflection to the first packet's time, which is
//          that, instead of whenever this harness happened to run.
//
//          `speedup` is left out: CurrentState.cs:3701 measures the IMU clock against
//          DateTime.Now, so under a replay it says how fast this harness read the file.
//
// getters  The derived getters - timeInAirMinSec, battery_mahperkm, battery_kmleft,
//          DistFromMovingBase, TrackerLocation, gimballat, gimballng - over a grid of inputs set
//          through the public setters, including the ones that divide by zero.
//
// fence    GeoFenceDist (CurrentState.cs:1617-1753) for a set of fences, put in MAVState.fencepoints
//          as MAVLinkInterface.cs:5694 does, and positions around them.
//
// Every float is written as the double it widens to, and every double with "R", so the Rust test
// compares bits. Run under TZ=UTC and HOME pointing at an empty directory: lastlogread is local
// time (MAVLinkInterface.cs:6557), and Settings reads the user's config.xml.

using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;
using MissionPlanner;
using MissionPlanner.Utilities;

public static class MpState
{
    static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    public static int Main(string[] args)
    {
        System.Threading.Thread.CurrentThread.CurrentCulture = Inv;
        System.Threading.Thread.CurrentThread.CurrentUICulture = Inv;
        try
        {
            if (args.Length == 3 && args[0] == "tlog")
                return Tlog(args[1], args[2]);
            if (args.Length == 2 && args[0] == "getters")
                return Getters(args[1]);
            if (args.Length == 2 && args[0] == "fence")
                return Fence(args[1]);
            if (args.Length == 2 && args[0] == "mktlog")
                return MkTlog(args[1]);
        }
        catch (Exception ex)
        {
            Console.Error.WriteLine("MpState: " + ex);
            return 1;
        }
        Console.Error.WriteLine("usage: MpState tlog <log.tlog> <out.csv>");
        Console.Error.WriteLine("       MpState getters <out.csv>");
        Console.Error.WriteLine("       MpState fence <out.csv>");
        Console.Error.WriteLine("       MpState mktlog <out.tlog>");
        return 2;
    }

    static string F(float value)
    {
        return ((double)value).ToString("R", Inv);
    }

    static string D(double value)
    {
        return value.ToString("R", Inv);
    }

    static readonly string[] Columns =
    {
        "datetime_ticks", "alt", "altoffsethome", "verticalspeed", "climbrate", "distTraveled",
        "timeSinceArmInAir", "timeInAir", "timeInAirMinSec", "battery_usedmah", "battery_mahperkm",
        "battery_kmleft", "DistFromMovingBase", "tracker_lat", "tracker_lng", "tracker_alt",
        "GeoFenceDist", "hilch1", "hilch2", "hilch3", "hilch4", "hilch5", "hilch6", "hilch7",
        "hilch8", "customfield0", "customfield1", "customfield2", "customfield3", "customfield4",
        "customfield5", "customfield6", "customfield7", "customfield8", "customfield9",
        "customfield10", "customfield11", "customfield12", "customfield13", "customfield14",
        "customfield15", "customfield16", "customfield17", "customfield18", "customfield19",
        "lowairspeed", "rateattitude", "rateposition", "ratestatus", "ratesensors", "raterc",
        "KIndex", "gimballat", "gimballng", "timesincelastshot", "planned_lat", "planned_lng",
        "planned_alt",
    };

    static string Row(CurrentState cs)
    {
        var tracker = cs.TrackerLocation;
        var planned = cs.PlannedHomeLocation;
        var fields = new List<string>
        {
            cs.datetime.Ticks.ToString(Inv), F(cs.alt), F(cs.altoffsethome), F(cs.verticalspeed),
            F(cs.climbrate), F(cs.distTraveled), F(cs.timeSinceArmInAir), F(cs.timeInAir),
            F(cs.timeInAirMinSec), D(cs.battery_usedmah), D(cs.battery_mahperkm),
            D(cs.battery_kmleft), F(cs.DistFromMovingBase), D(tracker.Lat), D(tracker.Lng),
            D(tracker.Alt), F(cs.GeoFenceDist), cs.hilch1.ToString(Inv), cs.hilch2.ToString(Inv),
            cs.hilch3.ToString(Inv), cs.hilch4.ToString(Inv), cs.hilch5.ToString(Inv),
            cs.hilch6.ToString(Inv), cs.hilch7.ToString(Inv), cs.hilch8.ToString(Inv),
            F(cs.customfield0), F(cs.customfield1), F(cs.customfield2), F(cs.customfield3),
            F(cs.customfield4), F(cs.customfield5), F(cs.customfield6), F(cs.customfield7),
            F(cs.customfield8), F(cs.customfield9), F(cs.customfield10), F(cs.customfield11),
            F(cs.customfield12), F(cs.customfield13), F(cs.customfield14), F(cs.customfield15),
            F(cs.customfield16), F(cs.customfield17), F(cs.customfield18), F(cs.customfield19),
            cs.lowairspeed ? "1" : "0", cs.rateattitude.ToString(Inv),
            cs.rateposition.ToString(Inv), cs.ratestatus.ToString(Inv),
            cs.ratesensors.ToString(Inv), cs.raterc.ToString(Inv), cs.KIndex.ToString(Inv),
            F(cs.gimballat), F(cs.gimballng), D(cs.timesincelastshot), D(planned.Lat),
            D(planned.Lng), D(planned.Alt),
        };
        return string.Join(",", fields);
    }

    static int Tlog(string path, string output)
    {
        var counter = typeof(CurrentState).GetField("lastsecondcounter",
            BindingFlags.NonPublic | BindingFlags.Instance);
        if (counter == null)
            throw new Exception("CurrentState.lastsecondcounter not found");
        var pinned = new HashSet<CurrentState>();

        using (var mine = new MAVLinkInterface())
        using (var w = new StreamWriter(output, false, new UTF8Encoding(false)))
        {
            w.NewLine = "\n";
            // Log/MavlinkLog.cs:81-96
            mine.logplaybackfile =
                new BinaryReader(File.Open(path, FileMode.Open, FileAccess.Read, FileShare.Read));
            mine.logreadmode = true;
            mine.speechenabled = false;

            w.WriteLine("packet,position," + string.Join(",", Columns));
            string last = null;
            long packet = 0;
            while (mine.logplaybackfile.BaseStream.Position < mine.logplaybackfile.BaseStream.Length)
            {
                var message = mine.readPacketAsync().AwaitSync();
                packet++;

                // The sender's state, made by MAVLinkInterface.cs:6649 if it is new: pin its
                // once-a-second counter to its first packet's time.
                if (message.Length > 0 && mine.MAVlist.Contains(message.sysid, message.compid))
                {
                    var cs = mine.MAVlist[message.sysid, message.compid].cs;
                    if (pinned.Add(cs))
                        counter.SetValue(cs, cs.datetime);
                }

                // Log/MavlinkLog.cs:141-144
                foreach (var mav in mine.MAVlist)
                    mav.cs.UpdateCurrentSettings(null, true, mine, mav);

                if (!mine.MAVlist.Contains(1, 1))
                    continue;
                // A row when anything but the clock changed: the clock moves with every packet,
                // and the test knows each packet's time from the log itself.
                var row = Row(mine.MAVlist[1, 1].cs);
                var rest = row.Substring(row.IndexOf(',') + 1);
                if (rest != last)
                {
                    w.WriteLine(packet.ToString(Inv) + "," +
                        mine.logplaybackfile.BaseStream.Position.ToString(Inv) + "," + row);
                    last = rest;
                }
            }
            w.WriteLine("# packets " + packet.ToString(Inv));
            foreach (var pair in CurrentState.custom_field_names)
                w.WriteLine("# " + pair.Key + "=" + pair.Value);
        }
        return 0;
    }

    static int Getters(string output)
    {
        using (var mine = new MAVLinkInterface())
        using (var w = new StreamWriter(output, false, new UTF8Encoding(false)))
        {
            w.NewLine = "\n";
            var cs = mine.MAVlist[1, 1].cs;

            // timeInAirMinSec, CurrentState.cs:1197
            w.WriteLine("minsec,timeInAir,timeInAirMinSec");
            foreach (var t in new float[] { 0, 1, 59, 60, 61, 119.5f, 3599, 3600, 3661.25f, 1e6f, -1, -61 })
            {
                cs.timeInAir = t;
                w.WriteLine("minsec," + F(t) + "," + F(cs.timeInAirMinSec));
            }

            // battery_mahperkm and battery_kmleft, CurrentState.cs:1474, 1479-1480
            w.WriteLine("battery,battery_usedmah,distTraveled,battery_remaining,battery_mahperkm,battery_kmleft");
            foreach (var used in new double[] { 0, 1234.5, 5000 })
                foreach (var dist in new float[] { 0, 1, 2500.25f })
                    foreach (var remaining in new int[] { 0, 50, 99, 100 })
                    {
                        cs.battery_usedmah = used;
                        cs.distTraveled = dist;
                        cs.battery_remaining = remaining;
                        w.WriteLine("battery," + D(used) + "," + F(dist) + "," +
                                    cs.battery_remaining.ToString(Inv) + "," + D(cs.battery_mahperkm) +
                                    "," + D(cs.battery_kmleft));
                    }

            // DistFromMovingBase, CurrentState.cs:1786-1805
            w.WriteLine("base,base_lat,base_lng,lat,lng,DistFromMovingBase");
            var bases = new[] { new double[] { 0, 0 }, new[] { -35.363261, 149.165230 }, new[] { 60.5, -10.25 } };
            var positions = new[] { new double[] { 0, 0 }, new[] { -35.362, 149.166 }, new[] { 0, 149.1 }, new double[] { 61, -11 } };
            foreach (var b in bases)
                foreach (var p in positions)
                {
                    cs.Base = new PointLatLngAlt(b[0], b[1], 12.5);
                    cs.lat = p[0];
                    cs.lng = p[1];
                    w.WriteLine("base," + D(b[0]) + "," + D(b[1]) + "," + D(p[0]) + "," + D(p[1]) + "," +
                                F(cs.DistFromMovingBase));
                }

            // TrackerLocation, CurrentState.cs:1606-1615: the tracker when its longitude is set,
            // otherwise home.
            w.WriteLine("tracker,set_lat,set_lng,set_alt,home_lat,home_lng,home_alt,lat,lng,alt");
            var trackers = new[] { new double[] { 0, 0, 0 }, new[] { -35.1, 0, 7 }, new[] { 0, 149.2, 3 }, new[] { -35.2, 149.3, 584 } };
            foreach (var t in trackers)
            {
                cs.TrackerLocation = new PointLatLngAlt(t[0], t[1], t[2], "");
                cs.HomeLocation = new PointLatLngAlt(-35.36, 149.16, 580.5, "");
                var got = cs.TrackerLocation;
                w.WriteLine("tracker," + D(t[0]) + "," + D(t[1]) + "," + D(t[2]) + "," +
                            D(-35.36) + "," + D(149.16) + "," + D(580.5) + "," + D(got.Lat) + "," +
                            D(got.Lng) + "," + D(got.Alt));
            }
            cs.TrackerLocation = new PointLatLngAlt();

            // gimballat and gimballng, CurrentState.cs:2024-2042: 0 until a point is set, then
            // the point narrowed to a float.
            w.WriteLine("gimbal,lat,lng,gimballat,gimballng");
            w.WriteLine("gimbal,,," + F(cs.gimballat) + "," + F(cs.gimballng));
            cs.GimbalPoint = new PointLatLngAlt(-35.36326123456789, 149.16523098765432, 10);
            w.WriteLine("gimbal," + D(cs.GimbalPoint.Lat) + "," + D(cs.GimbalPoint.Lng) + "," +
                        F(cs.gimballat) + "," + F(cs.gimballng));
        }
        return 0;
    }

    // The fences: each a list of (command, param1, lat, lng) in sequence order.
    static readonly Dictionary<string, double[][]> Fences = new Dictionary<string, double[][]>
    {
        { "empty", new double[0][] },
        // MAV_CMD.FENCE_POLYGON_VERTEX_INCLUSION = 5001, a square about 220 m on a side
        { "inclusion", new[] {
            new double[] { 5001, 4, -35.362, 149.164 }, new double[] { 5001, 4, -35.362, 149.1665 },
            new double[] { 5001, 4, -35.364, 149.1665 }, new double[] { 5001, 4, -35.364, 149.164 } } },
        // FENCE_POLYGON_VERTEX_EXCLUSION = 5002, a triangle
        { "exclusion", new[] {
            new double[] { 5002, 3, -35.3625, 149.1650 }, new double[] { 5002, 3, -35.3625, 149.1660 },
            new double[] { 5002, 3, -35.3635, 149.1655 } } },
        // FENCE_CIRCLE_INCLUSION = 5003 and FENCE_CIRCLE_EXCLUSION = 5004, with a return point
        // (FENCE_RETURN_POINT = 5000) that the property filters out
        { "circles", new[] {
            new double[] { 5000, 0, -35.3630, 149.1650 },
            new double[] { 5003, 300, -35.3630, 149.1650 },
            new double[] { 5004, 25, -35.3632, 149.1652 } } },
        // two inclusion polygons of the same size back to back, then an exclusion circle
        { "mixed", new[] {
            new double[] { 5001, 3, -35.360, 149.160 }, new double[] { 5001, 3, -35.360, 149.170 },
            new double[] { 5001, 3, -35.368, 149.165 },
            new double[] { 5001, 3, -35.361, 149.162 }, new double[] { 5001, 3, -35.361, 149.168 },
            new double[] { 5001, 3, -35.366, 149.165 },
            new double[] { 5004, 40, -35.3635, 149.1655 } } },
        // a polygon whose declared vertex count is short of its vertices: the rest chunk alone
        { "short", new[] {
            new double[] { 5001, 3, -35.362, 149.164 }, new double[] { 5001, 3, -35.362, 149.1665 },
            new double[] { 5001, 3, -35.364, 149.1665 }, new double[] { 5001, 3, -35.364, 149.164 } } },
    };

    // The synthetic flight: a .tlog written here, deterministically, to take the paths the
    // recorded ones do not - a plane that arms, flies a track on a 3D fix, loses the fix, lands,
    // disarms and arms again; a current sensor that reports "none" and then amps, with battery
    // reports between; a climb rate from the altitude before the first VFR_HUD; an airspeed
    // sensor and ARSPD_FBW_MIN for the low-airspeed warning, the sensor going unhealthy; named
    // values, more names than there are custom fields, and a name in lower case; HIL channels,
    // with a NaN and an overflow; a clock that goes back three seconds, and one that jumps a
    // minute and a second. Committed as testdata/currentstate/synthetic.tlog, and played by the
    // `tlog` verb like the recorded ones.
    static int MkTlog(string output)
    {
        var parse = new MAVLink.MavlinkParse();
        byte seq = 0;
        // 2026-01-01T00:00:00.25Z, a quarter second off the seconds so ticks fall between packets.
        ulong t0 = 1767225600250000UL;
        using (var file = new FileStream(output, FileMode.Create, FileAccess.Write))
        {
            Action<ulong, MAVLink.MAVLINK_MSG_ID, object> put = (micros, id, message) =>
            {
                var stamp = BitConverter.GetBytes(micros);
                Array.Reverse(stamp);
                file.Write(stamp, 0, 8);
                var frame = parse.GenerateMAVLinkPacket20(id, message, false, 1, 1, seq++);
                file.Write(frame, 0, frame.Length);
            };
            Func<string, byte[]> name10 = text =>
            {
                var bytes = new byte[10];
                var ascii = Encoding.ASCII.GetBytes(text);
                Array.Copy(ascii, bytes, Math.Min(10, ascii.Length));
                return bytes;
            };
            Func<string, byte[]> name16 = text =>
            {
                var bytes = new byte[16];
                var ascii = Encoding.ASCII.GetBytes(text);
                Array.Copy(ascii, bytes, Math.Min(16, ascii.Length));
                return bytes;
            };
            const uint DiffPressure = 16;
            double lat = -35.3632621, lng = 149.1652374;
            float relalt = 0;
            // The clock's offset from the step's own time: back 3 s at step 900, on 61 s at 1200.
            long offset = 0;
            for (int step = 0; step < 1500; step++)
            {
                if (step == 900)
                    offset -= 3000000;
                if (step == 1200)
                    offset += 61000000;
                ulong now = (ulong)((long)t0 + step * 100000L + offset);
                double t = step / 10.0;
                bool armed = (t >= 10 && t < 60) || (t >= 70 && t < 140);
                bool flying = armed && ((t >= 15 && t < 55) || (t >= 75 && t < 135));
                bool fix = !(t >= 30 && t < 35);

                if (flying)
                {
                    lng += 0.0000110;
                    lat += step % 20 < 10 ? 0.0000031 : -0.0000017;
                    relalt = Math.Min(relalt + 0.37f, 60.5f);
                }
                else if (relalt > 0)
                {
                    relalt = Math.Max(relalt - 0.9f, 0);
                }

                if (step % 10 == 0)
                {
                    put(now, MAVLink.MAVLINK_MSG_ID.HEARTBEAT, new MAVLink.mavlink_heartbeat_t(
                        10, 1, 3, (byte)(armed ? 129 : 1), 4, 3));
                    put(now, MAVLink.MAVLINK_MSG_ID.GPS_RAW_INT, new MAVLink.mavlink_gps_raw_int_t(
                        (ulong)step * 100000UL, (int)Math.Round(lat * 1e7), (int)Math.Round(lng * 1e7),
                        584000 + (int)(relalt * 1000), 121, 200, (ushort)(flying ? 1230 : 0), 9000,
                        (byte)(fix ? 3 : 2), 10, 0, 0, 0, 0, 0, 0));
                    uint health = t >= 90 && t < 95 ? 0 : DiffPressure;
                    short current = t < 15 ? (short)-1 : (short)(flying ? 1234 : 456);
                    put(now, MAVLink.MAVLINK_MSG_ID.SYS_STATUS, new MAVLink.mavlink_sys_status_t(
                        DiffPressure, DiffPressure, health, 200, 12600, current, 0, 0, 0, 0, 0, 0,
                        (sbyte)(100 - step / 20)));
                    put(now, MAVLink.MAVLINK_MSG_ID.RC_CHANNELS_SCALED,
                        new MAVLink.mavlink_rc_channels_scaled_t((uint)step * 100, (short)(step - 700),
                            (short)(-step), 10000, -10000, (short)(step * 7), 0, 1, -1, 0, 255));
                    put(now, MAVLink.MAVLINK_MSG_ID.NAMED_VALUE_FLOAT,
                        new MAVLink.mavlink_named_value_float_t((uint)step * 100, (float)(t * 1.5), name10("alpha")));
                    put(now, MAVLink.MAVLINK_MSG_ID.NAMED_VALUE_FLOAT,
                        new MAVLink.mavlink_named_value_float_t((uint)step * 100, (float)(-t / 3), name10("Beta")));
                }
                if (step % 100 == 50)
                {
                    int[] consumed = { 0, 5, 40, 90, 150, 180, 210, 280, 340, 400, 420, 450, 500, 560, 600 };
                    put(now, MAVLink.MAVLINK_MSG_ID.BATTERY_STATUS, new MAVLink.mavlink_battery_status_t(
                        consumed[step / 100], 0, 2500,
                        new ushort[] { 4200, 4200, 4200, ushort.MaxValue, ushort.MaxValue, ushort.MaxValue,
                            ushort.MaxValue, ushort.MaxValue, ushort.MaxValue, ushort.MaxValue },
                        (short)(flying ? 1234 : 456), 0, 0, 0, (sbyte)(100 - step / 20), 0, 0,
                        new ushort[] { 0, 0, 0, 0 }, 0, 0));
                }
                if (step == 50)
                {
                    put(now, MAVLink.MAVLINK_MSG_ID.PARAM_VALUE,
                        new MAVLink.mavlink_param_value_t(12.0f, 2, 0, name16("ARSPD_FBW_MIN"), 9));
                    put(now, MAVLink.MAVLINK_MSG_ID.PARAM_VALUE,
                        new MAVLink.mavlink_param_value_t(3.0f, 2, 1, name16("SERVO_FOO"), 9));
                }
                if (step == 400)
                {
                    // Twenty-two more names: the fields fill up at twenty with alpha and Beta.
                    for (int n = 0; n < 22; n++)
                        put(now, MAVLink.MAVLINK_MSG_ID.NAMED_VALUE_FLOAT,
                            new MAVLink.mavlink_named_value_float_t(40000, n + 0.5f, name10("n" + n.ToString("00"))));
                }
                if (step % 50 == 25)
                {
                    float[] roll = { 0.5f, -1f, float.NaN, 214748.3648f, 0.12345f, -0.99999f };
                    int k = step / 50 % roll.Length;
                    put(now, MAVLink.MAVLINK_MSG_ID.HIL_CONTROLS, new MAVLink.mavlink_hil_controls_t(
                        (ulong)step * 100000UL, roll[k], -roll[k] / 2, 0.25f, (float)(t / 200), 0, 0, 0, 0, 0, 0));
                }
                // Position at 10 Hz; VFR_HUD from 20 s, so the climb rate before is the C#'s own.
                put(now, MAVLink.MAVLINK_MSG_ID.GLOBAL_POSITION_INT, new MAVLink.mavlink_global_position_int_t(
                    (uint)step * 100, (int)Math.Round(lat * 1e7), (int)Math.Round(lng * 1e7),
                    584000 + (int)(relalt * 1000), (int)(relalt * 1000), (short)(flying ? 1000 : 0), 0, 0, 9000));
                if (t >= 20)
                {
                    float airspeed = flying ? (t >= 100 && t < 110 ? 9.5f : 14.25f) : 0f;
                    put(now, MAVLink.MAVLINK_MSG_ID.VFR_HUD, new MAVLink.mavlink_vfr_hud_t(
                        airspeed, flying ? 12.3f : 0f, relalt, flying ? 0.5f : 0f, 90,
                        (ushort)(flying ? 55 : (armed ? 10 : 0))));
                }
            }
        }
        return 0;
    }

    static int Fence(string output)
    {
        using (var mine = new MAVLinkInterface())
        using (var w = new StreamWriter(output, false, new UTF8Encoding(false)))
        {
            w.NewLine = "\n";
            w.WriteLine("fence,lat,lng,GeoFenceDist");
            var mav = mine.MAVlist[1, 1];
            var positions = new List<double[]>();
            for (int i = -3; i <= 3; i++)
                for (int j = -3; j <= 3; j++)
                    positions.Add(new[] { -35.363 + i * 0.0009, 149.16525 + j * 0.0011 });
            positions.Add(new double[] { 0, 0 });
            positions.Add(new[] { -35.3632, 149.1652 });
            foreach (var fence in Fences)
            {
                // MAVLinkInterface.cs:5641, 5694
                mav.fencepoints.Clear();
                for (int seq = 0; seq < fence.Value.Length; seq++)
                {
                    var item = fence.Value[seq];
                    mav.fencepoints[seq] = new MAVLink.mavlink_mission_item_int_t
                    {
                        seq = (ushort)seq,
                        command = (ushort)item[0],
                        param1 = (float)item[1],
                        x = (int)Math.Round(item[2] * 1e7),
                        y = (int)Math.Round(item[3] * 1e7),
                        mission_type = (byte)MAVLink.MAV_MISSION_TYPE.FENCE,
                    };
                }
                foreach (var p in positions)
                {
                    mav.cs.lat = p[0];
                    mav.cs.lng = p[1];
                    w.WriteLine(fence.Key + "," + D(p[0]) + "," + D(p[1]) + "," + F(mav.cs.GeoFenceDist));
                }
            }
        }
        return 0;
    }
}
