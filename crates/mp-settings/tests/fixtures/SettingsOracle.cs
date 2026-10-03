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

// Drives Mission Planner's own Settings class (MissionPlanner.Utilities.dll) under mono, so the
// bytes Settings.Save writes can be compared with what mp-settings renders.
//
//   mono SettingsOracle.exe <dir> set <file>   each line of <file> is key=value, the value with
//                                              C# escapes (Regex.Unescape: \n, \r, \t, \x01);
//                                              Settings.Instance[key] = value for each, then Save()
//   mono SettingsOracle.exe <dir> resave       Load() <dir>/Mission Planner/config.xml, Save() it
//
// The file lands at <dir>/Mission Planner/config.xml: CustomUserDataDirectory moves
// GetUserDataDirectory there (ExtLibs/Utilities/Settings.cs:344-348).
//
// Built and run (MP = an installed Mission Planner, whose MissionPlanner.Utilities.dll is used):
//   mcs -out:SettingsOracle.exe -r:$MP/MissionPlanner.Utilities.dll \
//       -r:/usr/lib/mono/4.5/Facades/netstandard.dll SettingsOracle.cs
//   MONO_PATH=$MP mono SettingsOracle.exe <dir> set config-saved.txt
using System;
using System.IO;
using System.Text.RegularExpressions;
using MissionPlanner.Utilities;

class SettingsOracle
{
    static void Main(string[] args)
    {
        Settings.CustomUserDataDirectory = args[0];
        var settings = Settings.Instance;
        if (args[1] == "set")
        {
            foreach (var line in File.ReadAllLines(args[2]))
            {
                if (line == "" || line.StartsWith("#"))
                    continue;
                var at = line.IndexOf('=');
                settings[line.Substring(0, at)] = Regex.Unescape(line.Substring(at + 1));
            }
        }
        settings.Save();
    }
}
