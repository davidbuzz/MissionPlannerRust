---
name: no-desktop-region-captures
description: Never screenshot a region of Buzz's desktop to see a VM or VNC window - it captures whatever is on top, his private windows included; read the app's facts over SSH instead (2026-10-04)
metadata:
  type: feedback
---

On 2026-10-04 Buzz asked me to screenshot tridge's Mac. `screencapture` over SSH fails ("could
not create image from display"). So I ran `ffmpeg -f x11grab` over the region of his Linux
desktop where the Remmina window sat. The first grab worked. The second caught one of his private
conversations, which had come over that region. I deleted it at once. `xwd -id` on the Remmina
window failed too (BadColor on a 32-bit window).

**Why:** a region grab takes whatever is on screen there, not the window I meant. His desktop is
his own; what it shows is private.

**How to apply:**
- To see what the planner shows on a remote machine, read its facts instead. Put
  `MP_FACTS=$HOME/mpr-facts.txt` in its launcher (on tridge's Mac, `~/mpr-show.command` has it
  since 2026-10-04) and read the facts over SSH. `layout.hidden.names` says what is cut off,
  with each control's placement.
- Only screenshot his desktop when he asks for it in that moment, and then grab only the window
  he named. Never grab a screen region.
- If a capture shows anything other than what was asked for, delete it unread and say so.
- See also [[gui-tests-take-the-mouse]].
