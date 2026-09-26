# One SITL for the whole Windows GUI suite, as tools/sitl/start-sitl.sh keeps one for the Linux
# suite: the Cygwin ArduCopter the SIMULATION page fetches, and a relay in front of it.
#
#   powershell -File tools\win10\sitl-relay.ps1 -SitlExe <ArduCopter.exe> -SitlDir <dir> -SitlParams <copter.parm>
#
# The Cygwin build exits within three seconds of its first client leaving - measured in tiny10 on
# 2026-09-26, started with the suite's command line and with Mission Planner's `--serial0 tcp:0`
# alike, and not when the process that started it exits - so every GUI script that closed its
# planner took the simulator with it, and the next met a SITL seconds old, with no GPS fix, no
# home and "Not Ready to Arm" (the owner's report of the failures that followed). The Linux SITL
# accepts the next client and carries on, settled for minutes.
#
# So the SITL is started with its TCP ports moved up by $Offset (`--base-port`) and booting at
# once (`--serial0 tcp:0`, as `SITL.cs:664` starts it), and this holds a connection to each of
# them for as long as it runs: the simulator never sees a client leave. It listens on the usual
# ports - 5760, 5762, 5763 - for one client at a time each, passing bytes both ways; what the
# SITL sends while no client is connected is read and dropped, as it would be sent to nobody. A
# new client replaces the one before. Should the SITL exit anyway it is started again in the same
# directory, its parameters kept, and the relay connects to it again.
#
# `relay.status` in the SITL directory holds the running SITL's process id and when it started, for
# gui-test.ps1 to wait until it has settled (its GPS, its home, its prearm checks) before a script
# uses it. The suite stops this process and the SITL when it ends.
param(
    [Parameter(Mandatory = $true)][string]$SitlExe,
    [Parameter(Mandatory = $true)][string]$SitlDir,
    [string]$SitlParams = '',
    [int[]]$Ports = @(5760, 5762, 5763),
    [int]$Offset = 100
)
$ErrorActionPreference = 'Stop'

Add-Type -TypeDefinition @'
using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;

// One relayed port: a connection to the SITL's port held open, reconnected whenever the SITL
// comes back, and the one client on the listening port.
public sealed class SitlRelay
{
    readonly int listenPort, sitlPort;
    readonly object gate = new object();
    TcpClient upstream, client;

    SitlRelay(int listenPort, int sitlPort) { this.listenPort = listenPort; this.sitlPort = sitlPort; }

    public static void Serve(int listenPort, int sitlPort)
    {
        var relay = new SitlRelay(listenPort, sitlPort);
        var listener = new TcpListener(IPAddress.Loopback, listenPort);
        listener.Start();
        Start(relay.Upstream);
        Start(() => relay.Accept(listener));
    }

    static void Start(ThreadStart body) { new Thread(body) { IsBackground = true }.Start(); }

    static void Close(TcpClient connection) { try { if (connection != null) connection.Close(); } catch { } }

    // The SITL's side: connected until it goes, its bytes to the client or to nobody.
    void Upstream()
    {
        var buffer = new byte[65536];
        while (true)
        {
            var connection = new TcpClient();
            try { connection.NoDelay = true; connection.Connect(IPAddress.Loopback, sitlPort); }
            catch { Close(connection); Thread.Sleep(250); continue; }
            lock (gate) { upstream = connection; }
            try
            {
                var stream = connection.GetStream();
                int read;
                while ((read = stream.Read(buffer, 0, buffer.Length)) > 0)
                {
                    TcpClient to;
                    lock (gate) { to = client; }
                    if (to == null) continue;
                    try { to.GetStream().Write(buffer, 0, read); }
                    catch { lock (gate) { if (client == to) client = null; } Close(to); }
                }
            }
            catch { }
            lock (gate) { if (upstream == connection) upstream = null; }
            Close(connection);
            Thread.Sleep(250);
        }
    }

    // The listening side: one client at a time, the newest replacing the one before.
    void Accept(TcpListener listener)
    {
        while (true)
        {
            var accepted = listener.AcceptTcpClient();
            accepted.NoDelay = true;
            TcpClient before;
            lock (gate) { before = client; client = accepted; }
            Close(before);
            Start(() => Downstream(accepted));
        }
    }

    // The client's bytes to the SITL, until the client leaves.
    void Downstream(TcpClient from)
    {
        var buffer = new byte[65536];
        try
        {
            var stream = from.GetStream();
            int read;
            while ((read = stream.Read(buffer, 0, buffer.Length)) > 0)
            {
                TcpClient to;
                lock (gate) { to = upstream; }
                if (to == null) continue;
                try { to.GetStream().Write(buffer, 0, read); } catch { }
            }
        }
        catch { }
        lock (gate) { if (client == from) client = null; }
        Close(from);
    }
}
'@

New-Item -ItemType Directory -Force -Path $SitlDir | Out-Null
$status = Join-Path $SitlDir 'relay.status'
Remove-Item $status -ErrorAction SilentlyContinue
foreach ($port in $Ports) { [SitlRelay]::Serve($port, $port + $Offset) }
Write-Host "relaying $($Ports -join ', ') to the SITL's $(($Ports | ForEach-Object { $_ + $Offset }) -join ', ')"

$arguments = @('--model', 'quad', '--speedup', '1', '--base-port', (5760 + $Offset), '--serial0', 'tcp:0')
if ($SitlParams) { $arguments += @('--defaults', "`"$SitlParams`"") }
while ($true) {
    $sitl = Start-Process -FilePath $SitlExe -WorkingDirectory $SitlDir -WindowStyle Minimized -PassThru -ArgumentList $arguments
    # Windows PowerShell reports no ExitCode for a process whose handle was not read while it ran.
    $null = $sitl.Handle
    "{0} {1}" -f $sitl.Id, $sitl.StartTime.ToUniversalTime().ToString('o') | Set-Content -Path $status
    Write-Host "SITL $($sitl.Id) started in $SitlDir"
    $sitl.WaitForExit()
    Write-Host "SITL $($sitl.Id) exited with $($sitl.ExitCode); starting it again"
    Start-Sleep -Seconds 1
}
