#!/bin/sh
# Tailscale's CLI as the planner asks it, for the GUI scripts of the Tailscale indicator
# (crates/mp-gui/src/tailscale.rs, the owner's feature of 2026-10-14). The planner runs
# `tailscale status --json --peers=false` every few seconds; with MP_TAILSCALE_CLI naming this
# script it is answered here, as Tailscale 1.104's CLI answers, in the state FAKE_TAILSCALE_STATE
# names - `running` when it names none:
#
#   running        on a tailnet as buzz-laptop, 100.101.102.103 and fd7a:115c:a1e0::1234:5678
#   starting       signed in and connecting (Starting)
#   nostate        the daemon's first moments, not yet knowing whether it is signed in (NoState)
#   needslogin     signed out, a sign-in started and its link waiting (NeedsLogin, an AuthURL)
#   loggedout      signed out, no sign-in started (NeedsLogin)
#   needsapproval  signed in, waiting for the tailnet's admin to approve it (NeedsMachineAuth)
#   stopped        turned off (Stopped)
#   down           its daemon not running: nothing on stdout, the CLI's error on stderr, exit 1
#
# The status is `ipnstate.Status` as the CLI prints it - `json.MarshalIndent(st, "", "  ")`, no
# final newline, exit 0 in every backend state (cmd/tailscale/cli/status.go:88-101); a daemon
# that does not answer is diag.go's error on stderr and exit 1 (cmd/tailscale/tailscale.go:22-25).
# Anything but `status --json` is refused with exit 2: the planner may ask nothing else.
#
# In a GUI script (tools/gui-test.sh):
#
#   setup install -m 755 tests/gui/fake-tailscale.sh $WORK/tailscale
#   env MP_TAILSCALE_CLI $WORK/tailscale
#   env FAKE_TAILSCALE_STATE needslogin
#   expect tailscale.state signed-out
#
# The unit test `the_fake_cli_answers_each_state_as_the_real_one` in tailscale.rs runs every
# state through the planner's own run of the CLI.
set -u

state=${FAKE_TAILSCALE_STATE:-running}

asked_json=no
for arg in "$@"; do
    case $arg in
        --json | -json | --json=true | -json=true) asked_json=yes ;;
    esac
done
if [ "${1:-}" != status ] || [ "$asked_json" != yes ]; then
    echo "fake-tailscale: only 'status --json' is answered here, not '$*'" >&2
    exit 2
fi

# The status signed in, with its network map: `BackendState` $1, `Health` $health.
signed_in() {
    cat <<EOF
{
  "Version": "1.104.0-t0123456789-g0123456789",
  "TUN": true,
  "BackendState": "$1",
  "HaveNodeKey": true,
  "AuthURL": "",
  "TailscaleIPs": [
    "100.101.102.103",
    "fd7a:115c:a1e0::1234:5678"
  ],
  "Self": {
    "ID": "nTkGsU6fz711CNTRL",
    "NodeID": 1234567890123456,
    "PublicKey": "nodekey:5f2b0c7e1d9a4b3c8e6f0a1b2c3d4e5f60718293a4b5c6d7e8f90a1b2c3d4e5f",
    "HostName": "buzz-laptop",
    "DNSName": "buzz-laptop.tail1234.ts.net.",
    "OS": "linux",
    "UserID": 9876543210,
    "TailscaleIPs": [
      "100.101.102.103",
      "fd7a:115c:a1e0::1234:5678"
    ],
    "AllowedIPs": [
      "100.101.102.103/32",
      "fd7a:115c:a1e0::1234:5678/128"
    ],
    "Addrs": [
      "203.0.113.7:41641",
      "192.168.1.20:41641"
    ],
    "CurAddr": "",
    "Relay": "syd",
    "PeerRelay": "",
    "RxBytes": 0,
    "TxBytes": 0,
    "Created": "2026-09-01T02:03:04.123456789Z",
    "LastWrite": "0001-01-01T00:00:00Z",
    "LastSeen": "0001-01-01T00:00:00Z",
    "LastHandshake": "0001-01-01T00:00:00Z",
    "Online": true,
    "ExitNode": false,
    "ExitNodeOption": false,
    "Active": false,
    "PeerAPIURL": [
      "http://100.101.102.103:45678",
      "http://[fd7a:115c:a1e0::1234:5678]:45678"
    ],
    "TaildropTarget": 0,
    "NoFileSharingReason": "",
    "Capabilities": [
      "HTTPS://TAILSCALE.COM/s/DEPRECATED-NODE-CAPS#see-https://github.com/tailscale/tailscale/issues/11508",
      "https://tailscale.com/cap/file-sharing"
    ],
    "CapMap": {
      "https://tailscale.com/cap/file-sharing": null
    },
    "InNetworkMap": true,
    "InMagicSock": false,
    "InEngine": false,
    "KeyExpiry": "2027-03-01T02:03:04Z"
  },
  "Health": $health,
  "MagicDNSSuffix": "tail1234.ts.net",
  "CurrentTailnet": {
    "Name": "pilot@example.com",
    "StableID": "TfAkE1CNTRL",
    "MagicDNSSuffix": "tail1234.ts.net",
    "MagicDNSEnabled": true
  },
  "CertDomains": [
    "buzz-laptop.tail1234.ts.net"
  ],
  "ExtraRecords": null,
  "Peer": null,
  "User": {
    "9876543210": {
      "ID": 9876543210,
      "LoginName": "pilot@example.com",
      "DisplayName": "A Pilot"
    }
  },
  "ClientVersion": null
}
EOF
}

# The status with no network map - signed out, or not yet knowing: `BackendState` $1, `AuthURL`
# $2, `Health` $health. `Self` has only the computer's host name and system.
signed_out() {
    cat <<EOF
{
  "Version": "1.104.0-t0123456789-g0123456789",
  "TUN": true,
  "BackendState": "$1",
  "AuthURL": "$2",
  "TailscaleIPs": null,
  "Self": {
    "ID": "",
    "NodeID": 0,
    "PublicKey": "nodekey:0000000000000000000000000000000000000000000000000000000000000000",
    "HostName": "buzz-laptop",
    "DNSName": "",
    "OS": "linux",
    "UserID": 0,
    "TailscaleIPs": null,
    "Addrs": null,
    "CurAddr": "",
    "Relay": "",
    "PeerRelay": "",
    "RxBytes": 0,
    "TxBytes": 0,
    "Created": "0001-01-01T00:00:00Z",
    "LastWrite": "0001-01-01T00:00:00Z",
    "LastSeen": "0001-01-01T00:00:00Z",
    "LastHandshake": "0001-01-01T00:00:00Z",
    "Online": false,
    "ExitNode": false,
    "ExitNodeOption": false,
    "Active": false,
    "PeerAPIURL": null,
    "TaildropTarget": 0,
    "NoFileSharingReason": "",
    "InNetworkMap": false,
    "InMagicSock": false,
    "InEngine": false
  },
  "Health": $health,
  "MagicDNSSuffix": "",
  "CurrentTailnet": null,
  "CertDomains": null,
  "ExtraRecords": null,
  "Peer": null,
  "User": null,
  "ClientVersion": null
}
EOF
}

# health.go's warnings for the states that have one (health/warnings.go:88, 115).
logged_out='[
    "You are logged out."
  ]'
stopped='[
    "Tailscale is stopped."
  ]'

health=null
case $state in
    running) printf '%s' "$(signed_in Running)" ;;
    starting) printf '%s' "$(signed_in Starting)" ;;
    needsapproval) printf '%s' "$(signed_in NeedsMachineAuth)" ;;
    stopped)
        health=$stopped
        printf '%s' "$(signed_in Stopped)"
        ;;
    nostate) printf '%s' "$(signed_out NoState '')" ;;
    needslogin)
        health=$logged_out
        printf '%s' "$(signed_out NeedsLogin https://login.tailscale.com/a/1a2b3c4d5e6f7)"
        ;;
    loggedout)
        health=$logged_out
        printf '%s' "$(signed_out NeedsLogin '')"
        ;;
    down)
        echo "failed to connect to local tailscaled; it doesn't appear to be running (sudo systemctl start tailscaled ?)" >&2
        exit 1
        ;;
    *)
        echo "fake-tailscale: no state '$state': running, starting, nostate, needslogin, loggedout, needsapproval, stopped or down" >&2
        exit 2
        ;;
esac
