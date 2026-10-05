# Using MissionPlannerRust over Tailscale

This guide is for you if you are new to MissionPlannerRust, new to Tailscale, or both. By the end,
the planner on one device - a laptop, or just a web browser - will be connected to a vehicle or a
simulator on another device, wherever each of them is.

## What Tailscale does here

Normally the planner and the vehicle have to be on the same network, or someone has to open ports
on a router. Tailscale builds a private network of your own devices, called your **tailnet**, that
works across the internet. Each device gets an address starting with `100.` and a short name.
Nothing on your tailnet is visible to the internet, only to your own devices.

The desktop planner uses Tailscale like any other network. The browser version needs it for any
vehicle outside the page itself: a web page cannot open a connection to a vehicle on its own, so
the page carries a small Tailscale device of its own.

## Words you will meet

| Word | Meaning |
|---|---|
| tailnet | Your private network: every device signed in to your Tailscale account. |
| device (Tailscale also says *machine*) | A computer, phone or browser page on your tailnet. |
| tailnet address | A device's address on the tailnet, like `100.101.102.103`. It does not change. |
| machine name | A device's short name, like `my-pi`. You can type it wherever an address is asked for. |
| admin console | Tailscale's web page listing your devices: https://login.tailscale.com/admin/machines. Rename or remove devices there. |
| MAVLink | The language the planner and the vehicle speak. Something on the vehicle side has to hand it out on a port. |

## What you need

- A Tailscale account. It is free for personal use.
- The **vehicle-side computer**: the one connected to the vehicle, such as a Raspberry Pi on the
  vehicle or a laptop with a telemetry radio, or a computer running a simulator. It needs Linux,
  macOS or Windows.
- The **planner**: the desktop app, or a recent Chrome, Edge or Firefox for the browser version.

## Step 1: Create your Tailscale account

Go to https://tailscale.com and choose **Get started**. Sign up with an account you already have
(Google, Microsoft, GitHub, Apple and others are offered). Every device you add later signs in with
this same account.

## Step 2: Put the vehicle-side computer on your tailnet

On Linux, including a Raspberry Pi:

```sh
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
```

`tailscale up` prints a link. Open it in any browser and sign in with your account. On macOS or
Windows, install the app from https://tailscale.com/download, open it and sign in.

Then note the computer's name and address:

```sh
tailscale status
```

The first line is this computer: its tailnet address, then its machine name.

```
100.101.102.103  my-pi  you@  linux  -
```

The commands in this guide are for Linux. The same `tailscale` commands work in a terminal on
macOS and Windows, without `sudo`. If the command is not found there, see Tailscale's own docs for
the command line on your system.

## Step 3: Hand out the vehicle's MAVLink on the tailnet

Something on the vehicle-side computer has to offer the vehicle's MAVLink on a port. Pick the case
that fits.

### Case A: one planner at a time, over TCP

Use this if a program on the computer already offers MAVLink on a TCP port. An ArduPilot simulator
(SITL) offers port 5760, and mavlink-router or MAVProxy can be set up to offer one. Share that
port on your tailnet:

```sh
sudo tailscale serve --bg --tcp 5760 tcp://localhost:5760
```

Port 5760 of this computer can now be reached from your tailnet, and only from your tailnet.
`tailscale serve status` shows what is shared ("tailnet only"). If the command prints a link asking
you to enable Serve for your tailnet, open it, enable Serve, and run the command again.

To stop sharing it:

```sh
sudo tailscale serve --tcp=5760 off
```

A port like this takes **one planner at a time**. For more, use case B.

### Case B: several planners at once, with MAVProxy

MAVProxy can give each planner a port of its own:

```sh
mavproxy.py --master tcp:127.0.0.1:5760 \
    --out udpin:0.0.0.0:14560 --out udpin:0.0.0.0:14561 --out udpin:0.0.0.0:14562
```

- `--master` is how MAVProxy reaches the vehicle: `tcp:127.0.0.1:5760` for a simulator, or the
  flight controller's serial port, such as `/dev/ttyACM0` for one plugged in by USB.
- Each `udpin` port serves one planner, which connects to it with **UDPCl** (step 4). Add a port
  for each extra screen.
- MAVProxy listens on every network, your tailnet included, so this case needs no
  `tailscale serve`.

## Step 4: Connect the planner

### In the desktop app

1. Install Tailscale on the planner's computer too, and sign in with the same account (step 2).
2. Start MissionPlannerRust.
3. At the top right, the box to the left of the CONNECT button lists the ways to connect.
   Choose **TCP** for case A, or **UDPCl** for case B.
4. Click **CONNECT**. At "Enter host name/ip", type the vehicle-side computer's machine name
   (`my-pi`) or its tailnet address (`100.101.102.103`), then OK.
5. At "Enter remote port", type `5760` for case A, or one of MAVProxy's ports (`14560`) for
   case B, then OK.

The line under the MissionPlannerRust title now shows the link and a growing count of frames, and
the vehicle's parameters download. The next time, the boxes offer your last answers.

### In a web browser

1. Open the planner's page, https://davidbuzz.github.io/MissionPlannerRust/, on any computer. You
   install nothing. The first visit plays a short demo of the planner flying a simulated copter;
   to skip it, open https://davidbuzz.github.io/MissionPlannerRust/?demo=0 instead.
2. Choose **TCP** or **UDPCl**, click **CONNECT**, and answer the host and port as in the desktop
   app.
3. The first time, a box at the bottom right says "The link needs this page on your tailnet:
   **sign in to Tailscale**". Click it. A new tab opens Tailscale's sign-in: sign in with the same
   account, and it says "Login successful", naming the page's new device. Go back to the planner's
   tab. The connection carries on by itself, and the box shows "Tailscale: on the tailnet as
   mpr-... 100...." for a few seconds.
4. The browser remembers the sign-in, so later visits connect straight away.

Each browser is a device of its own on your tailnet, named `mpr-` and two random words, such as
`mpr-jerboa-stonecat`. You will see them in the admin console, where you can rename or remove
them. Clearing the site's data in the browser makes it a new device at the next visit, so remove
the old one from the console.

In the browser, `127.0.0.1` and `localhost` mean the simulator inside the page itself (the
SIMULATION screen), not your computer. To reach a simulator running on your own computer, put that
computer on your tailnet (step 2) and use its machine name.

## Which way to connect?

| Choice | What it does | Desktop | Browser |
|---|---|---|---|
| TCP | The planner connects to a host and port. Case A. | yes | yes |
| UDPCl | The planner sends first to a host and port, which answers. Case B. | yes | yes |
| UDP | The planner waits on a port of its own for the vehicle to send first. | yes | not yet |
| WS | A WebSocket URL, for a vehicle side that offers one. | yes | yes |

## How fast is it?

When two devices can reach each other directly, the link is about as fast as the network between
them. When they cannot, Tailscale carries the traffic through one of its relay servers, adding a
little delay. A browser always goes through a relay. In our own test, three planners were
connected to one simulated copter, one of them a browser in another city using Tailscale's Sydney
relay. An action taken on one screen showed on the other two in under half a second.

## When something goes wrong

| What you see | What to try |
|---|---|
| In the browser, nothing happens for a few seconds after OK | The first time, the page loads Tailscale (about 8 MB). Wait for the sign-in box. |
| "link failed" | Check both devices show as connected in the admin console. Try the tailnet address instead of the name. On the vehicle side, check `tailscale serve status` lists your port (case A), or that MAVProxy is running (case B). |
| It worked for one planner but not a second one | A TCP port takes one planner at a time (case A). Use case B for more. |
| Connected, but no data, with UDPCl | The port must be one of MAVProxy's `udpin` ports. |
| You are not sure the two devices can reach each other | On the vehicle side, run `tailscale ping <the other device's name>`. |
| The browser asks you to sign in again | Its device was removed or expired in the admin console, or the site's data was cleared. Sign in again. |

## Keeping it safe

- Only devices signed in to your account, and people you choose to share your tailnet with, can
  reach anything. Nothing is opened to the internet.
- Anyone who can reach the vehicle's port can command the vehicle. Share your tailnet only with
  people you trust to fly it.
- To stop: `sudo tailscale serve --tcp=5760 off` (case A), stop MAVProxy (case B), or take the
  computer off the tailnet with `sudo tailscale down`.

## For the curious: page options

Added to the page's address after `?` (several joined with `&`):

- `tscontrol=<url>`: use a coordination server other than Tailscale's own, such as Headscale.
- `tsauthkey=<key>`: join with a Tailscale auth key instead of signing in.
- `tshostname=<name>`: the page's device name, instead of `mpr-` and two random words.
- `demo=0`: skip the first visit's demo.

How the browser's Tailscale works inside is described in [web/README.md](web/README.md).
