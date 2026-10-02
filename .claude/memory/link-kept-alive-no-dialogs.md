---
name: link-kept-alive-no-dialogs
description: Buzz's rulings of 2026-10-03 - a connected link is kept and reconnected aggressively with no dialog (PLAN §12 D23); the take-off altitude is asked once a session (D24); the C# asks every time and never reconnects its main link
metadata:
  type: feedback
---

Two rulings from Buzz's bug reports of 2026-10-03, both divergences from the C# at his word:

1. **A link that was connected stays connected.** "after a vehicle is rebooted or as a result of
   packet loss etc, the connection should be forcibly always kept active, and always re-connect
   and start connected aggressively, no dialog boxes should occur during this reconnection
   period." The link thread keeps its vehicles, parameters and messages and opens its transport
   again by URL every second (`mp_link::RECONNECT_INTERVAL`) until it is back or DISCONNECT is
   pressed; the status line says "reconnecting, try N". Mission Planner's own main link does not
   reconnect (`MainV2.cs:2456-2490`); only its mirror streams do (`CommsTCPSerial.cs:330-355`).
2. **A question asked once a session.** The take-off altitude box ("Enter Alt" / "Enter Takeoff
   Alt") is asked the first time and its answer used for every later TakeOff while the
   application runs, on any vehicle (`fly::takeoff_press`); the C# asks at every press
   (`FlightData.cs:5294`).

**Why:** a reconnection that needs the operator, or a box that repeats a question whose answer has
not changed, costs attention at the moment it matters least; the link's state is on the status
line and the top right of the window already ([[no-dialogs-for-avoidable-errors]]).

**How to apply:** nothing on the reconnection path may prompt; a new transport kind joins the
reopen rule unless, like a replay or a listening socket, opening it again is not a reconnection
(say so at the site). When another box repeats a question whose answer rarely changes within a
session, ask Buzz whether D24's rule extends to it rather than assuming. Related:
[[not-in-the-csharp-not-in-scope]] (a ruling is what makes such a divergence allowed).
