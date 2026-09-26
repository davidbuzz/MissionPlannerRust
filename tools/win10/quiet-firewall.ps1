# Windows Defender Firewall's "has blocked some features of this app" prompt, off by local group
# policy, so a SITL fetched to a new path - every run of sitl.gui and sitl-launch.gui, the suite's
# own on its first run - no longer waits for somebody at the VM to click Allow access (the owner,
# 2026-09-26: "can we change windows defender settings as a matter of policy so thats not
# required").
#
# The firewall stays on, with its defaults: a new app that listens is still blocked from inbound
# connections off this machine - it is only no longer asked about. What the GUI suite needs is
# loopback (127.0.0.1), which the firewall does not filter, and the VM sits behind VirtualBox's
# NAT with only SSH forwarded. Elevated, as the ClaudeConsole window is.
#
#   tools/win10/vm-run.sh quiet-firewall tools/win10/quiet-firewall.ps1
#
# Undone with the same command and -NotifyOnListen NotConfigured.
$ErrorActionPreference = 'Stop'
Set-NetFirewallProfile -PolicyStore localhost -Profile Domain, Private, Public -NotifyOnListen False
"in the local group policy:"
Get-NetFirewallProfile -PolicyStore localhost | Select-Object Name, NotifyOnListen | Format-Table -AutoSize | Out-String -Width 120
# A local policy change is in force from the next policy refresh; ask for it now.
gpupdate /target:computer /force | Out-String
"in force now (policy over local settings):"
Get-NetFirewallProfile -PolicyStore ActiveStore |
    Select-Object Name, Enabled, NotifyOnListen, DefaultInboundAction | Format-Table -AutoSize | Out-String -Width 120
