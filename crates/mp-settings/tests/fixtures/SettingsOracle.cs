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
