//! Sticks read on a thread of their own, and sent the moment they move.
//!
//! DELIVERABLES.md D15 asks for stick input to packet on the wire at p99 under 5 ms. Polling a
//! non-blocking device every 50 ms from gpui's foreground executor cannot get there - the poll
//! interval alone is ten times the budget - and it ties the thing flying the aircraft to how
//! quickly the user interface finishes a frame. So this owns two OS threads:
//!
//! - **read**: blocks in `read(2)` on the device and wakes the moment the kernel has an event. It
//!   decodes, publishes the new position, and goes straight back to blocking. It never calls
//!   anything outside this module, so a slow link or a slow screen cannot delay the next read.
//! - **send**: waits for a new position, a change of state, or the resend interval, whichever is
//!   first, and hands the frame to the caller's sink - the function that puts it on the link.
//!
//! Mission Planner does the first half the same way on Linux - a dedicated thread in a blocking
//! read of `/dev/input/js*` (`// C#: ExtLibs/ArduPilot/Joystick/JoystickLinux.cs:81`, `:101`) - and
//! then throws the latency away twice: a loop that samples that state every 50 ms
//! (`// C#: ExtLibs/ArduPilot/Joystick/JoystickBase.cs:1036`) and a send loop that wakes every
//! 40 ms and sends if 50 ms have passed (`// C#: MainV2.cs:2249`, `:2356`, `:2446`). The channel
//! arithmetic is unchanged from [`Mapping`]; what changed is the plumbing between the device and
//! the sink.
//!
//! # Wiring it in (for the GUI)
//!
//! This replaces, in `mp-gui`, the `linux::Joystick` held by `Sticks`, the `Failsafe` beside it,
//! `Sticks::poll`/`Sticks::send_failed`, and the 50 ms `STICK_POLL` loop spawned on the foreground
//! executor. **That loop must go**: nothing needs to call this periodically, and a caller that
//! also sends what it polls would send every frame twice.
//!
//! ```text
//! // Choosing a device. The sink runs on the send thread, so it captures a Send handle to the
//! // link and the target vehicle, puts frame.channels.0 in an RC_CHANNELS_OVERRIDE, and returns
//! // whether the link took it. It must not block.
//! let reader = StickReader::open(path, Mapping::gamepad(), move |frame| send(frame.channels.0))?;
//!
//! reader.set_enabled(true);   // "fly with sticks"; false returns control and sends the release
//! reader.is_enabled();        // for the "sticks have control" label; goes false on its own
//!                             // when the device is unplugged
//! reader.liveness();          // Poll::Alive / Poll::Gone, same meaning as Joystick::poll
//! reader.reading();           // live axes and buttons for the panel, read at repaint
//! reader.latency();           // LatencyHistogram of read-to-sent, for p50/p99 on screen
//! reader.close();             // at shutdown: waits for the release frames to go out
//! ```
//!
//! On `Poll::Gone` the release has **already been sent** by the time the screen looks; the screen
//! only has to say so and drop the reader. Dropping (or replacing, on choosing another device) a
//! reader that is flying releases too, from its own thread, without blocking the caller.
//!
//! # What goes on the wire, and when
//!
//! - Nothing until [`StickReader::set_enabled`] is called with `true`. Then one frame at once (or
//!   when [`MIN_INTERVAL`] has passed, if a release went out less than that ago).
//! - A frame as soon as the mapped channels change ([`Cause::Changed`]), unless something was sent
//!   less than [`MIN_INTERVAL`] ago; then the change is held until that floor passes and goes out
//!   as whatever the newest position is by then. So an isolated movement is on the wire at once,
//!   and a stick being stirred continuously is sent at most 50 times a second, each frame at most
//!   about 20 ms stale. Bursts coalesce: one `read` drains everything the kernel has queued, and
//!   the send thread always takes the newest position, so a slow sink gets fewer, fresher frames
//!   rather than a backlog of stale ones.
//! - The current channels again whenever [`RESEND`] passes with nothing sent ([`Cause::Refresh`]),
//!   because the device is edge-triggered and a held stick produces no events at all.
//! - On a disconnect, on switching off, or on dropping the reader: [`Channels::release`] at once -
//!   never held by the floor - then repeated at [`RESEND`] until the failsafe's budget is spent
//!   ([`Cause::Release`]). A release the sink refuses is put back on that budget, exactly as
//!   `Sticks::send_failed` did.
//!
//! The decision about *what* to send is still [`Failsafe`], unchanged and fed the same way the
//! 50 ms loop fed it: while the device is alive it is fed, whether or not anything moved.

use crate::event::{self, LEN as EVENT_LEN};
use crate::{Channels, Failsafe, LatencyHistogram, Mapping, Poll, Reading};
use std::io::{ErrorKind, Read};
use std::sync::{Arc, Condvar, Mutex, MutexGuard, PoisonError};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

/// The longest the sticks go without being sent while overrides are on.
///
/// Mission Planner's joystick send rate: `float rate = 50; // 1000 / 50 = 20 hz`
/// (`// C#: MainV2.cs:2249`), gated by `lastjoystick.AddMilliseconds(rate) < DateTime.Now`
/// (`// C#: MainV2.cs:2356`). In Mission Planner it is the *only* send, and because the loop around
/// it sleeps 40 ms (`// C#: MainV2.cs:2446`) the frames actually go out about every 80 ms. Here it
/// is the ceiling: a moving stick is sent as it moves, and this is how long a held one waits to be
/// said again.
pub const RESEND: Duration = Duration::from_millis(50);

/// The shortest gap between two sends while overrides are on: a rate floor, 50 frames a second.
///
/// Mission Planner's floor is 50 ms, because its periodic send is its only send
/// (`// C#: MainV2.cs:2356`), and it also skips a send while the port has 50 bytes queued
/// (`// C#: MainV2.cs:2391`). Sending on change has no floor of its own: a gamepad reports at
/// 250-1000 Hz, and a stick being stirred would put a frame on the link for every report - a few
/// hundred `RC_CHANNELS_OVERRIDE`s a second, more than a 57600-baud radio carries at all, crowding
/// out the heartbeats and commands sharing it.
///
/// 20 ms, because PLAN.md §8.2 already measures stick latency "50 Hz sampled", so 50 frames a second
/// is the rate it assumes. Inside the floor a change is still sent the moment it arrives; only a
/// change that lands within 20 ms of the previous send waits, coalesced with anything newer, for
/// the floor to pass. An isolated movement is therefore not delayed at all, and a stirred stick is
/// at most about 20 ms stale rather than the 50 ms of [`RESEND`]. A release is never held.
///
/// This caps what one joystick asks of the link; it is not a radio budget. Fitting the link's
/// actual capacity - shared with everything else going out - is the link's job: the per-link token
/// bucket with RC override as the top priority in PLAN.md §8.1, which is not built yet.
pub const MIN_INTERVAL: Duration = Duration::from_millis(20);

/// How many `js_event`s one `read` may return.
///
/// The kernel hands back as many queued events as fit, so this is how large a burst is drained in
/// one wake-up. A full sweep of every axis on a gamepad is a few dozen; the sizing is for a device
/// that reports everything at once after a stall, which must still come out as one position rather
/// than several stale ones.
const READ_EVENTS: usize = 128;

/// How long to back off if the device turns out to be non-blocking.
///
/// This reader expects a blocking device - it has a thread to spend on waiting. Handed a
/// non-blocking one, `WouldBlock` is neither a disconnect (the device is there; treating it as gone
/// would hand control back on every idle moment) nor something to retry at once (that spins a
/// core). A millisecond keeps it correct and within budget.
const NONBLOCKING_BACKOFF: Duration = Duration::from_millis(1);

/// Why a frame is being sent.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Cause {
    /// The mapped channels moved.
    Changed,
    /// The channels are where they were: overrides were just switched on, or [`RESEND`] passed
    /// without a change. A held stick emits no events, so this is what keeps it flying.
    Refresh,
    /// Control is being handed back to the transmitter.
    Release,
}

/// One `RC_CHANNELS_OVERRIDE` worth of channels, and where it came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Frame {
    /// What to put in the message, channels 1 to 18.
    pub channels: Channels,
    /// When the read that produced the latest position returned, on the monotonic clock.
    ///
    /// `None` for a release, which carries no position, and before the device has said anything.
    pub read_at: Option<Instant>,
    /// Why it is being sent.
    pub cause: Cause,
}

/// Everything the two threads and the owner share.
#[derive(Debug)]
struct State {
    /// The latest position, as of the latest read.
    reading: Reading,
    /// When that read returned.
    read_at: Option<Instant>,
    /// When the oldest input not yet reflected on the wire was read, or `None` if everything
    /// read so far has been sent.
    ///
    /// The latency sample for a change is taken from here rather than from the latest read: a
    /// change held by [`MIN_INTERVAL`] and then coalesced with newer ones has been waiting since
    /// the first of them, and measuring from the newest would hide exactly the wait the floor adds.
    unsent_since: Option<Instant>,
    /// Set by the read thread when the device goes away, and never cleared.
    gone: bool,
    /// Set when the owner lets go, and never cleared.
    stopping: bool,
    /// Something happened that needs a frame now rather than at the next resend: overrides
    /// switched on or off, the device gone, the owner leaving, or the mapping changed.
    kicked: bool,
    mapping: Mapping,
    failsafe: Failsafe,
    latency: LatencyHistogram,
}

#[derive(Debug)]
struct Shared {
    state: Mutex<State>,
    /// Signalled on every change to `state` the send thread might need to act on.
    wake: Condvar,
}

impl Shared {
    /// Locks the state, surviving a poisoned lock.
    ///
    /// A thread that panicked while holding it leaves the data consistent - every write under this
    /// lock is a single field assignment - and refusing to touch it afterwards would mean refusing
    /// to send the release that the panic makes necessary.
    fn lock(&self) -> MutexGuard<'_, State> {
        self.state.lock().unwrap_or_else(PoisonError::into_inner)
    }
}

/// A joystick being read and sent from threads of its own.
///
/// See the module documentation for how it is wired and what it sends.
#[derive(Debug)]
pub struct StickReader {
    shared: Arc<Shared>,
    /// The send thread, joined by [`StickReader::close`]. The read thread is never joined: it is
    /// blocked in `read` on a device nobody may touch again, and waiting for it would wait for a
    /// person.
    sender: Option<JoinHandle<()>>,
}

impl StickReader {
    /// Opens a joystick node blocking and starts reading it.
    ///
    /// Blocking, unlike [`crate::linux::Joystick::open`]: that one is polled from a UI thread that
    /// must never wait, and this one has a thread whose whole job is to wait, so that it wakes the
    /// instant the kernel has an event rather than at the next poll. A blocking read on
    /// `/dev/input/js*` also returns an error the moment the device is unplugged, which is what
    /// makes the release on disconnect immediate.
    ///
    /// # Errors
    /// If the device cannot be opened - usually permissions, which on most distributions means
    /// the user is not in the `input` group - or a thread cannot be started.
    #[cfg(target_os = "linux")]
    pub fn open<S>(path: &str, mapping: Mapping, sink: S) -> std::io::Result<Self>
    where
        S: FnMut(&Frame) -> bool + Send + 'static,
    {
        let device = std::fs::File::open(path)?;
        Self::spawn(device, mapping, sink)
    }

    /// Starts reading `js_event`s from any blocking source.
    ///
    /// The device is anything that reads like `/dev/input/js*`: the real node, or a socket or pipe
    /// a test writes events into. End of file or an error from it means the device is gone.
    ///
    /// `sink` is called from the send thread with each frame to put on the wire, and returns
    /// whether the link accepted it. It must not block: while it runs, nothing else is sent -
    /// including a release.
    ///
    /// # Errors
    /// If a thread cannot be started.
    pub fn spawn<R, S>(device: R, mapping: Mapping, sink: S) -> std::io::Result<Self>
    where
        R: Read + Send + 'static,
        S: FnMut(&Frame) -> bool + Send + 'static,
    {
        let shared = Arc::new(Shared {
            state: Mutex::new(State {
                reading: Reading::default(),
                read_at: None,
                unsent_since: None,
                gone: false,
                stopping: false,
                kicked: false,
                mapping,
                failsafe: Failsafe::default(),
                latency: LatencyHistogram::new(),
            }),
            wake: Condvar::new(),
        });

        let sender = {
            let shared = Arc::clone(&shared);
            thread::Builder::new()
                .name("sticks-send".to_owned())
                .spawn(move || send_loop(&shared, sink))?
        };
        let mut reader = Self {
            shared: Arc::clone(&shared),
            sender: Some(sender),
        };

        let started = thread::Builder::new()
            .name("sticks-read".to_owned())
            .spawn(move || {
                // Marks the device gone however this thread ends - end of file, an error, or a
                // panic in the decoding - because a read thread that has stopped is a device
                // nobody is listening to, whatever the reason.
                let _gone = MarkGoneOnExit(&shared);
                read_loop(device, &shared);
            });
        if let Err(err) = started {
            // The send thread is idle - nothing has been enabled - so it exits at once.
            reader.stop();
            if let Some(sender) = reader.sender.take() {
                let _ = sender.join();
            }
            return Err(err);
        }
        Ok(reader)
    }

    /// Whether the device is still there, with exactly the meaning of
    /// [`crate::linux::Joystick::poll`]'s answer.
    ///
    /// `Alive` while the read thread is blocked waiting for the device or reading from it -
    /// whether or not anything has moved, because a held stick emits nothing. `Gone` once a read
    /// has returned end of file or an error other than `Interrupted`, which is what the kernel does
    /// when the device is unplugged. Never goes back to `Alive`.
    #[must_use]
    pub fn liveness(&self) -> Poll {
        if self.shared.lock().gone {
            Poll::Gone
        } else {
            Poll::Alive
        }
    }

    /// Turns overrides on or off, and returns whether they are on afterwards.
    ///
    /// Switching on a device that is gone is refused - it returns `false` - for the same reason the
    /// screen refuses to switch on with no device open. Switching off sends the release at once
    /// rather than at the next resend.
    pub fn set_enabled(&self, enabled: bool) -> bool {
        let mut state = self.shared.lock();
        if enabled && (state.gone || state.stopping) {
            return false;
        }
        state.failsafe.set_enabled(enabled);
        state.kicked = true;
        let now_enabled = state.failsafe.is_enabled();
        drop(state);
        self.shared.wake.notify_all();
        now_enabled
    }

    /// Whether overrides are on.
    ///
    /// Goes `false` without being asked when the device is unplugged: the read thread releases
    /// control as it notices, so a screen that asks after a disconnect is told the truth.
    #[must_use]
    pub fn is_enabled(&self) -> bool {
        self.shared.lock().failsafe.is_enabled()
    }

    /// The latest position of every axis and button.
    ///
    /// After a disconnect this is the last position read, as [`crate::linux::Joystick::reading`]
    /// was; clearing the display is the screen's decision.
    #[must_use]
    pub fn reading(&self) -> Reading {
        self.shared.lock().reading.clone()
    }

    /// Replaces the mapping. The new channels go out without waiting for the stick to move, if they
    /// differ from the old - at once, or when [`MIN_INTERVAL`] has passed since the last send.
    pub fn set_mapping(&self, mapping: Mapping) {
        let mut state = self.shared.lock();
        state.mapping = mapping;
        state.kicked = true;
        drop(state);
        self.shared.wake.notify_all();
    }

    /// How long changes took from the read returning to the sink returning, so far.
    ///
    /// Only frames sent because the channels moved, and only ones the sink accepted: a resend of a
    /// position read a second ago is not a second of latency. Each sample runs from the oldest read
    /// the frame delivers, so time spent held by [`MIN_INTERVAL`] is counted. This covers the part
    /// this crate controls; the kernel's wake-up of the blocked read comes before it and is not visible from
    /// user space without evdev timestamps.
    #[must_use]
    pub fn latency(&self) -> LatencyHistogram {
        self.shared.lock().latency.clone()
    }

    /// Lets go of the device and waits for the send thread to finish.
    ///
    /// If overrides were on, that means waiting for the release frames: the first at once, the
    /// rest at [`RESEND`], about a fifth of a second in all. Meant for shutdown, where a process
    /// that exits first would take the release with it. Everywhere else, dropping does the same
    /// without the wait.
    pub fn close(mut self) {
        self.stop();
        if let Some(sender) = self.sender.take() {
            let _ = sender.join();
        }
    }

    fn stop(&self) {
        let mut state = self.shared.lock();
        state.stopping = true;
        state.kicked = true;
        drop(state);
        self.shared.wake.notify_all();
    }
}

impl Drop for StickReader {
    /// Letting go of a reader that is flying hands control back.
    ///
    /// The release goes out from the send thread, which finishes the repeats and exits on its own,
    /// so a screen that drops a reader - on unplug, or on choosing another device - does not stall
    /// for the fifth of a second the repeats take. The read thread leaves at its next event.
    fn drop(&mut self) {
        self.stop();
    }
}

/// Marks the device gone when the read thread ends, however it ends.
struct MarkGoneOnExit<'a>(&'a Shared);

impl Drop for MarkGoneOnExit<'_> {
    fn drop(&mut self) {
        let mut state = self.0.lock();
        state.gone = true;
        // Released here, on the thread that found out, rather than left for the send thread to
        // infer: from this moment `is_enabled` is false for anyone who asks, and the send thread
        // is woken to put the release on the wire.
        state.failsafe.release_now();
        state.kicked = true;
        drop(state);
        self.0.wake.notify_all();
    }
}

/// The read thread: block, decode, publish, repeat.
fn read_loop<R: Read>(mut device: R, shared: &Shared) {
    let mut buffer = [0u8; READ_EVENTS * EVENT_LEN];
    // Bytes of an incomplete event carried over from the previous read, at the front of `buffer`.
    // The real device never splits an event, but a socket or pipe may, and dropping the fragment
    // would misframe every event after it.
    let mut held = 0;
    let mut reading = Reading::default();
    loop {
        let Some(space) = buffer.get_mut(held..) else {
            return;
        };
        match device.read(space) {
            // End of file: the device has gone.
            Ok(0) => return,
            Ok(count) => {
                let read_at = Instant::now();
                let end = held + count;
                let whole = end - end % EVENT_LEN;
                if let Some(events) = buffer.get(..whole) {
                    for chunk in events.chunks_exact(EVENT_LEN) {
                        if let Ok(raw) = <&[u8; EVENT_LEN]>::try_from(chunk) {
                            event::apply(&mut reading, raw);
                        }
                    }
                }
                buffer.copy_within(whole..end, 0);
                held = end - whole;
                if whole == 0 {
                    continue;
                }
                let mut state = shared.lock();
                if state.stopping {
                    return;
                }
                // One publish per read, however many events it carried: the burst is coalesced
                // here, before the send thread ever sees it.
                state.reading.clone_from(&reading);
                state.read_at = Some(read_at);
                state.unsent_since.get_or_insert(read_at);
                drop(state);
                shared.wake.notify_all();
            }
            Err(err) if err.kind() == ErrorKind::Interrupted => {}
            Err(err) if err.kind() == ErrorKind::WouldBlock => {
                if shared.lock().stopping {
                    return;
                }
                thread::sleep(NONBLOCKING_BACKOFF);
            }
            // Anything else is the device going away: unplugged, or the node removed.
            Err(_) => return,
        }
    }
}

/// The send thread: decide, send, wait, repeat.
fn send_loop<S>(shared: &Shared, mut sink: S)
where
    S: FnMut(&Frame) -> bool,
{
    let mut last_send: Option<Instant> = None;
    // The last stick position sent since overrides came on, for telling a change from a repeat.
    let mut last_sticks: Option<Channels> = None;
    let mut state = shared.lock();
    loop {
        let now = Instant::now();
        // Fed while the device is there, whether or not anything moved - the same rule the 50 ms
        // loop followed, and for the same reason: a held stick emits nothing.
        if state.gone || state.stopping {
            state.failsafe.release_now();
        } else {
            state.failsafe.fed(now);
        }
        if !state.failsafe.is_enabled() {
            last_sticks = None;
        }
        let channels = state.mapping.channels(&state.reading);
        let changed =
            state.failsafe.is_live(now) && last_sticks.is_some_and(|last| last != channels);
        if !changed {
            // Whatever was read since the last send moved nothing on the wire - an unmapped
            // button, a stick within one microsecond - so there is nothing waiting to be delivered.
            state.unsent_since = None;
        }
        let since_last = |interval| last_send.is_none_or(|at| now.duration_since(at) >= interval);
        let due = since_last(RESEND);
        // After the lines above, an enabled failsafe is a live one, so a disabled one with frames
        // still to send is releasing.
        let releasing = !state.failsafe.is_enabled() && state.failsafe.releases_pending() > 0;
        let send = if releasing {
            // Never held by the floor: handing control back is not something to rate-limit.
            state.kicked || due
        } else {
            // A change, or a kick (switched on, remapped), goes out at once unless something was
            // sent within MIN_INTERVAL; then it waits for the floor, and the kick stays set so the
            // wait cannot lose it.
            ((state.kicked || changed) && since_last(MIN_INTERVAL)) || due
        };

        let outgoing = if send {
            state.kicked = false;
            state.failsafe.outgoing(now, channels)
        } else {
            None
        };
        if let Some(channels) = outgoing {
            let cause = if channels == Channels::release() {
                Cause::Release
            } else if changed {
                Cause::Changed
            } else {
                Cause::Refresh
            };
            let frame = Frame {
                channels,
                read_at: if cause == Cause::Release {
                    None
                } else {
                    state.read_at
                },
                cause,
            };
            let stopping = state.stopping;
            let unsent_since = state.unsent_since.take();
            // Unlocked while the sink runs, so a slow link never holds up the read thread
            // publishing the next position - which is what lets that position coalesce.
            drop(state);
            let sent = sink(&frame);
            let sent_at = Instant::now();
            state = shared.lock();
            last_send = Some(sent_at);
            if cause != Cause::Release {
                last_sticks = Some(channels);
            }
            if sent
                && cause == Cause::Changed
                && let Some(read_at) = unsent_since
            {
                state
                    .latency
                    .record(sent_at.saturating_duration_since(read_at));
            }
            // A refused release goes back on the budget, as `Sticks::send_failed` did - except
            // while shutting down, where a link refusing everything would otherwise keep this
            // thread retrying forever after its owner has gone.
            if !sent && cause == Cause::Release && !stopping {
                state.failsafe.retry_release();
            }
            // Straight round again: whatever arrived while the sink ran is the newest position,
            // and it goes out as soon as the floor allows rather than after a full resend wait.
            continue;
        }

        let pending = state.failsafe.releases_pending();
        if state.stopping && pending == 0 {
            return;
        }
        state = if state.failsafe.is_enabled() || pending > 0 {
            // A change or kick held by the floor is due when the floor passes; otherwise the next
            // thing due is the resend.
            let held = state.failsafe.is_enabled() && (changed || state.kicked);
            let interval = if held { MIN_INTERVAL } else { RESEND };
            let wait = last_send
                .and_then(|at| at.checked_add(interval))
                .map_or(Duration::ZERO, |next| {
                    next.saturating_duration_since(Instant::now())
                });
            shared
                .wake
                .wait_timeout(state, wait)
                .map_or_else(|poisoned| poisoned.into_inner().0, |(guard, _)| guard)
        } else {
            // Nothing to send until somebody switches overrides on, so no timer at all: an idle
            // joystick costs no wake-ups.
            shared
                .wake
                .wait(state)
                .unwrap_or_else(PoisonError::into_inner)
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::event::{AXIS, BUTTON, INIT};
    use crate::{Binding, RELEASE_REPEATS, Source};
    use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender};

    /// Long enough to wait for something that must happen, on a machine that is busy.
    const PATIENCE: Duration = Duration::from_secs(5);

    /// A device fed from the test, one `read` per chunk sent, like a character device.
    ///
    /// Dropping the sender is an unplug: the next read returns end of file.
    struct FakeDevice {
        chunks: Receiver<std::io::Result<Vec<u8>>>,
        leftover: Vec<u8>,
    }

    impl Read for FakeDevice {
        fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
            if self.leftover.is_empty() {
                match self.chunks.recv() {
                    Ok(Ok(chunk)) => self.leftover = chunk,
                    Ok(Err(err)) => return Err(err),
                    Err(_) => return Ok(0),
                }
            }
            let count = buf.len().min(self.leftover.len());
            buf[..count].copy_from_slice(&self.leftover[..count]);
            self.leftover.drain(..count);
            Ok(count)
        }
    }

    /// Everything the sink was handed, with when.
    struct Wire {
        frames: Receiver<(Instant, Frame)>,
    }

    impl Wire {
        fn next(&self) -> Frame {
            self.frames.recv_timeout(PATIENCE).expect("a frame").1
        }

        /// The next frame of a kind, skipping others - a resend can always be in flight.
        fn next_of(&self, cause: Cause) -> Frame {
            self.next_timed_of(cause).1
        }

        fn next_timed_of(&self, cause: Cause) -> (Instant, Frame) {
            loop {
                let (at, frame) = self.frames.recv_timeout(PATIENCE).expect("a frame");
                if frame.cause == cause {
                    return (at, frame);
                }
            }
        }

        /// Everything that arrives within `window`.
        fn during(&self, window: Duration) -> Vec<Frame> {
            let until = Instant::now() + window;
            let mut frames = Vec::new();
            while let Some(left) = until.checked_duration_since(Instant::now()) {
                match self.frames.recv_timeout(left) {
                    Ok((_, frame)) => frames.push(frame),
                    Err(RecvTimeoutError::Timeout) => break,
                    Err(RecvTimeoutError::Disconnected) => break,
                }
            }
            frames
        }

        fn is_silent_for(&self, window: Duration) -> bool {
            self.frames.recv_timeout(window).is_err()
        }
    }

    /// Channel 1 on axis 0, so a test can move one number and watch it.
    fn one_axis() -> Mapping {
        Mapping {
            bindings: vec![Binding::new(1, Source::Axis(0))],
        }
    }

    fn axis(value: i16) -> Vec<u8> {
        event::encode(0, AXIS, 0, value).to_vec()
    }

    /// A reader on a fake device, with a sink that accepts everything and records it.
    fn rig(mapping: Mapping) -> (StickReader, Sender<std::io::Result<Vec<u8>>>, Wire) {
        rig_with(mapping, |_| true)
    }

    fn rig_with(
        mapping: Mapping,
        mut accept: impl FnMut(&Frame) -> bool + Send + 'static,
    ) -> (StickReader, Sender<std::io::Result<Vec<u8>>>, Wire) {
        let (feed, chunks) = mpsc::channel();
        let (wire, frames) = mpsc::channel();
        let device = FakeDevice {
            chunks,
            leftover: Vec::new(),
        };
        let reader = StickReader::spawn(device, mapping, move |frame| {
            let _ = wire.send((Instant::now(), *frame));
            accept(frame)
        })
        .expect("threads start");
        (reader, feed, Wire { frames })
    }

    /// Waits until the reader has seen a position, so a test does not race the read thread.
    fn wait_for_axis(reader: &StickReader, value: f32) {
        let until = Instant::now() + PATIENCE;
        while (reader.reading().axis(0) - value).abs() > 1e-3 {
            assert!(Instant::now() < until, "the reading never arrived");
            thread::sleep(Duration::from_millis(1));
        }
    }

    /// Nothing is sent until the operator says so, however much the sticks move.
    #[test]
    fn nothing_is_sent_until_switched_on() {
        let (reader, feed, wire) = rig(one_axis());
        for value in [1_000, -20_000, 32_767] {
            feed.send(Ok(axis(value))).expect("feed");
        }
        wait_for_axis(&reader, 1.0);
        assert!(wire.is_silent_for(RESEND * 3), "sent before being enabled");
        assert!(!reader.is_enabled());
    }

    /// Switching on sends at once, and a movement is sent as a change, not at the next tick.
    #[test]
    fn a_movement_is_sent_as_a_change() {
        let (reader, feed, wire) = rig(one_axis());
        assert!(reader.set_enabled(true));
        let first = wire.next();
        assert_eq!(first.cause, Cause::Refresh);
        assert_eq!(first.channels.get(1), Some(crate::mapping::CENTRE_US));

        feed.send(Ok(axis(32_767))).expect("feed");
        let moved = wire.next_of(Cause::Changed);
        assert_eq!(moved.channels.get(1), Some(crate::mapping::MAX_US));
        assert!(moved.read_at.is_some(), "a change says when it was read");
        // The sample is recorded after the sink returns; the next frame proves it has.
        assert_eq!(wire.next().cause, Cause::Refresh);
        assert_eq!(reader.latency().count(), 1, "and its latency is recorded");
    }

    /// The flight-safety property of an edge-triggered device: a stick held still emits nothing,
    /// and it must keep flying rather than being released as though it were unplugged.
    #[test]
    fn a_held_stick_keeps_being_sent_and_is_never_released() {
        let (reader, feed, wire) = rig(one_axis());
        feed.send(Ok(axis(32_767))).expect("feed");
        wait_for_axis(&reader, 1.0);
        assert!(reader.set_enabled(true));
        // Well past STICK_TIMEOUT, with not one event from the device.
        let frames = wire.during(crate::STICK_TIMEOUT * 3);
        assert!(
            frames.len() >= 4,
            "a held stick must be resent at the cadence, got {}",
            frames.len()
        );
        for frame in &frames {
            assert_eq!(frame.cause, Cause::Refresh, "{frame:?}");
            assert_eq!(frame.channels.get(1), Some(crate::mapping::MAX_US));
        }
        assert_eq!(reader.liveness(), Poll::Alive);
        assert!(reader.is_enabled());
        // Not a latency: nothing moved.
        assert_eq!(reader.latency().count(), 0);
        drop(feed);
    }

    /// An unplug hands control back from the reader's own threads. Nothing here calls into the
    /// reader between the unplug and the release - the consumer may be a screen that is busy.
    #[test]
    fn unplugging_releases_without_the_consumer_doing_anything() {
        let (reader, feed, wire) = rig(one_axis());
        assert!(reader.set_enabled(true));
        let _ = wire.next();

        drop(feed);
        let released = wire.next_of(Cause::Release);
        assert_eq!(released.channels, Channels::release());
        assert_eq!(released.read_at, None);

        for _ in 1..RELEASE_REPEATS {
            assert_eq!(wire.next().cause, Cause::Release);
        }
        assert!(
            wire.is_silent_for(RESEND * 4),
            "releases must stop, or the vehicle's own failsafe timer never runs"
        );
        assert_eq!(reader.liveness(), Poll::Gone);
        assert!(!reader.is_enabled());
    }

    /// The release is on its way before the next resend would have been due: a disconnect that
    /// is known is acted on, not waited out.
    #[test]
    fn an_unplug_is_released_promptly_rather_than_on_the_next_tick() {
        let (reader, feed, wire) = rig(one_axis());
        assert!(reader.set_enabled(true));
        let _ = wire.next();
        // Let a resend go past, so the next one is a full interval away when the unplug lands.
        let _ = wire.next_of(Cause::Refresh);

        let unplugged = Instant::now();
        drop(feed);
        let (at, _) = wire.next_timed_of(Cause::Release);
        assert!(
            at.duration_since(unplugged) < crate::STICK_TIMEOUT,
            "the release took {:?}",
            at.duration_since(unplugged)
        );
        drop(reader);
    }

    /// A read error other than `Interrupted` is a disconnect, as it was for `Joystick::poll`.
    #[test]
    fn a_read_error_is_a_disconnect() {
        let (reader, feed, wire) = rig(one_axis());
        assert!(reader.set_enabled(true));
        let _ = wire.next();
        feed.send(Err(std::io::Error::other("no such device")))
            .expect("feed");
        assert_eq!(wire.next_of(Cause::Release).channels, Channels::release());
        assert_eq!(reader.liveness(), Poll::Gone);
    }

    /// `Interrupted` is a signal arriving during the read, not the device leaving.
    #[test]
    fn an_interrupted_read_is_not_a_disconnect() {
        let (reader, feed, wire) = rig(one_axis());
        assert!(reader.set_enabled(true));
        let _ = wire.next();
        feed.send(Err(std::io::ErrorKind::Interrupted.into()))
            .expect("feed");
        feed.send(Ok(axis(32_767))).expect("feed");
        assert_eq!(
            wire.next_of(Cause::Changed).channels.get(1),
            Some(crate::mapping::MAX_US)
        );
        assert_eq!(reader.liveness(), Poll::Alive);
        assert!(reader.is_enabled());
    }

    /// Unplugging while not flying sends nothing: there is nothing to hand back.
    #[test]
    fn unplugging_while_switched_off_sends_nothing() {
        let (reader, feed, wire) = rig(one_axis());
        drop(feed);
        let until = Instant::now() + PATIENCE;
        while reader.liveness() == Poll::Alive {
            assert!(Instant::now() < until, "the unplug was never noticed");
            thread::sleep(Duration::from_millis(1));
        }
        assert!(wire.is_silent_for(RESEND * 3));
        assert!(!reader.set_enabled(true), "a gone device cannot be flown");
        assert!(wire.is_silent_for(RESEND * 2));
    }

    /// Switching off hands control back at once and repeats the release, then goes quiet.
    #[test]
    fn switching_off_releases_at_once_and_repeats() {
        let (reader, _feed, wire) = rig(one_axis());
        assert!(reader.set_enabled(true));
        let _ = wire.next();
        assert!(!reader.set_enabled(false));
        let _ = wire.next_of(Cause::Release);
        let releases: Vec<Frame> = (1..RELEASE_REPEATS).map(|_| wire.next()).collect();
        assert!(
            releases.iter().all(|frame| frame.cause == Cause::Release),
            "{releases:?}"
        );
        assert!(wire.is_silent_for(RESEND * 4));
    }

    /// While releasing, a moving stick must not speed the releases up or send positions: they
    /// go out at the cadence, spaced so one burst of loss cannot take them all.
    #[test]
    fn movement_after_switching_off_sends_no_positions() {
        let (reader, feed, wire) = rig(one_axis());
        assert!(reader.set_enabled(true));
        let _ = wire.next();
        assert!(!reader.set_enabled(false));
        for step in 0..50 {
            feed.send(Ok(axis(step * 600))).expect("feed");
        }
        let frames = wire.during(RESEND * 8);
        // A resend already in the sink when the switch was thrown may land first; after the first
        // release there must be nothing but releases.
        let first_release = frames
            .iter()
            .position(|frame| frame.cause == Cause::Release)
            .expect("a release");
        assert!(first_release <= 1, "{frames:?}");
        let after = &frames[first_release..];
        assert_eq!(after.len(), usize::from(RELEASE_REPEATS), "{frames:?}");
        assert!(after.iter().all(|frame| frame.cause == Cause::Release));
    }

    /// A reader dropped while flying releases, from its own thread, without making the caller
    /// wait. Choosing another device does exactly this.
    #[test]
    fn dropping_a_reader_that_is_flying_releases() {
        let (reader, _feed, wire) = rig(one_axis());
        assert!(reader.set_enabled(true));
        let _ = wire.next();
        let before = Instant::now();
        drop(reader);
        assert!(
            before.elapsed() < RESEND,
            "drop must not wait for the repeats"
        );
        let _ = wire.next_of(Cause::Release);
        for _ in 1..RELEASE_REPEATS {
            assert_eq!(wire.next().cause, Cause::Release);
        }
        assert!(wire.is_silent_for(RESEND * 3));
    }

    /// `close` is for shutdown: when it returns, the release has already gone out.
    #[test]
    fn close_returns_after_the_release_has_been_sent() {
        let (reader, _feed, wire) = rig(one_axis());
        assert!(reader.set_enabled(true));
        let _ = wire.next();
        reader.close();
        let frames: Vec<Frame> = wire.frames.try_iter().map(|(_, frame)| frame).collect();
        let releases = frames
            .iter()
            .filter(|frame| frame.cause == Cause::Release)
            .count();
        assert_eq!(releases, usize::from(RELEASE_REPEATS), "{frames:?}");
    }

    /// A release the link refuses is put back on the budget, as `Sticks::send_failed` did.
    #[test]
    fn a_refused_release_is_sent_again() {
        let mut refused = false;
        let (reader, _feed, wire) = rig_with(one_axis(), move |frame| {
            if frame.cause == Cause::Release && !refused {
                refused = true;
                return false;
            }
            true
        });
        assert!(reader.set_enabled(true));
        let _ = wire.next();
        assert!(!reader.set_enabled(false));
        let releases = wire
            .during(RESEND * (u32::from(RELEASE_REPEATS) + 4))
            .into_iter()
            .filter(|frame| frame.cause == Cause::Release)
            .count();
        assert_eq!(releases, usize::from(RELEASE_REPEATS) + 1);
    }

    /// A burst read in one go is one position on the wire, and it is the last one.
    #[test]
    fn a_burst_is_sent_as_its_latest_position() {
        let (reader, feed, wire) = rig(one_axis());
        assert!(reader.set_enabled(true));
        let _ = wire.next();
        // A hundred events in one read, as a device delivers them after a stall.
        let burst: Vec<u8> = (0..100i16).flat_map(|step| axis(step * 300)).collect();
        feed.send(Ok(burst)).expect("feed");
        let expected = Binding::new(1, Source::Axis(0)).value(&Reading {
            axes: vec![f32::from(99i16 * 300) / 32_767.0],
            buttons: Vec::new(),
        });
        let changes: Vec<Frame> = wire
            .during(RESEND * 3)
            .into_iter()
            .filter(|frame| frame.cause == Cause::Changed)
            .collect();
        assert_eq!(changes.len(), 1, "a burst must not become a queue");
        assert_eq!(changes[0].channels.get(1), Some(expected));
    }

    /// A slow link gets fewer, fresher frames - never a backlog of stale ones.
    #[test]
    fn a_slow_sink_gets_the_latest_position_rather_than_a_queue() {
        let (reader, feed, wire) = rig_with(one_axis(), |_| {
            thread::sleep(Duration::from_millis(20));
            true
        });
        assert!(reader.set_enabled(true));
        let _ = wire.next();
        for step in 1..=60i16 {
            feed.send(Ok(axis(step * 500))).expect("feed");
            thread::sleep(Duration::from_millis(1));
        }
        let expected = Binding::new(1, Source::Axis(0)).value(&Reading {
            axes: vec![f32::from(60i16 * 500) / 32_767.0],
            buttons: Vec::new(),
        });
        let changes: Vec<Frame> = wire
            .during(Duration::from_millis(400))
            .into_iter()
            .filter(|frame| frame.cause == Cause::Changed)
            .collect();
        assert!(
            changes.len() < 30,
            "sixty movements into a 20 ms sink must coalesce, got {}",
            changes.len()
        );
        assert_eq!(
            changes.last().and_then(|frame| frame.channels.get(1)),
            Some(expected),
            "the last thing on the wire is where the stick ended up"
        );
    }

    /// How late past its deadline a held frame may go out, on a busy machine.
    const SLACK: Duration = Duration::from_millis(10);

    /// Enabled, the switch-on frame taken, and far enough past it that the next change is
    /// isolated - nothing sent within `MIN_INTERVAL` - and so goes out at once.
    fn flying_and_quiet(reader: &StickReader, wire: &Wire) {
        assert!(reader.set_enabled(true));
        assert_eq!(wire.next().cause, Cause::Refresh);
        thread::sleep(MIN_INTERVAL + Duration::from_millis(5));
    }

    /// Two changes a millisecond apart: the first goes out at once, the second is held by the
    /// floor and goes out when it passes - once, carrying the second position, and not before.
    #[test]
    fn a_change_inside_the_floor_is_held_until_it_passes() {
        let (reader, feed, wire) = rig(one_axis());
        flying_and_quiet(&reader, &wire);

        let first_written = Instant::now();
        feed.send(Ok(axis(16_000))).expect("feed");
        let (first_at, first) = wire.next_timed_of(Cause::Changed);
        let first_value = first.channels.get(1);
        assert!(first_value > Some(crate::mapping::CENTRE_US), "{first:?}");

        thread::sleep(
            (first_written + Duration::from_millis(1)).saturating_duration_since(Instant::now()),
        );
        feed.send(Ok(axis(-16_000))).expect("feed");

        let (second_at, second) = wire.frames.recv_timeout(PATIENCE).expect("a frame");
        assert_eq!(second.cause, Cause::Changed, "{second:?}");
        assert!(
            second.channels.get(1) < Some(crate::mapping::CENTRE_US),
            "the held frame must carry the second position: {second:?}"
        );
        let gap = second_at.duration_since(first_at);
        assert!(
            gap >= MIN_INTERVAL,
            "sent {gap:?} after the first, inside the floor"
        );
        assert!(
            gap <= MIN_INTERVAL + SLACK,
            "held {gap:?}, well past the floor"
        );
        // Exactly one: nothing else carrying the second position, and no change after it.
        let after = wire.during(RESEND);
        assert!(
            after.iter().all(|frame| frame.cause == Cause::Refresh),
            "{after:?}"
        );
    }

    /// Switching off a millisecond after a change went out: the release is not held by the floor.
    #[test]
    fn a_release_is_not_held_by_the_floor() {
        let (reader, feed, wire) = rig(one_axis());
        flying_and_quiet(&reader, &wire);

        feed.send(Ok(axis(16_000))).expect("feed");
        let (changed_at, _) = wire.next_timed_of(Cause::Changed);
        thread::sleep(
            (changed_at + Duration::from_millis(1)).saturating_duration_since(Instant::now()),
        );
        assert!(!reader.set_enabled(false));

        let (released_at, released) = wire.frames.recv_timeout(PATIENCE).expect("a frame");
        assert_eq!(released.cause, Cause::Release, "{released:?}");
        let gap = released_at.duration_since(changed_at);
        assert!(
            gap < MIN_INTERVAL,
            "the release waited {gap:?} after the last frame - the floor held it"
        );
    }

    /// Stirred continuously, the wire never sees two frames closer than the floor.
    #[test]
    fn a_stirred_stick_never_sends_two_frames_inside_the_floor() {
        let (reader, feed, wire) = rig(one_axis());
        assert!(reader.set_enabled(true));
        let mut times = vec![wire.frames.recv_timeout(PATIENCE).expect("a frame").0];
        for step in 0..200i16 {
            feed.send(Ok(axis(if step % 2 == 0 { 20_000 } else { -20_000 })))
                .expect("feed");
            thread::sleep(Duration::from_millis(1));
        }
        // A fixed window rather than "until quiet": a flying reader resends every RESEND for ever.
        let until = Instant::now() + RESEND * 2;
        while let Some(left) = until.checked_duration_since(Instant::now()) {
            match wire.frames.recv_timeout(left) {
                Ok((at, _)) => times.push(at),
                Err(_) => break,
            }
        }
        assert!(times.len() > 3, "{}", times.len());
        for pair in times.windows(2) {
            let gap = pair[1].duration_since(pair[0]);
            assert!(gap >= MIN_INTERVAL, "two frames {gap:?} apart");
        }
    }

    /// A control nobody mapped moving is not a change on the wire.
    #[test]
    fn an_unmapped_control_does_not_send_a_change() {
        let (reader, feed, wire) = rig(one_axis());
        assert!(reader.set_enabled(true));
        let _ = wire.next();
        feed.send(Ok(event::encode(0, BUTTON, 3, 1).to_vec()))
            .expect("feed");
        let until = Instant::now() + PATIENCE;
        while !reader.reading().button(3) {
            assert!(Instant::now() < until, "the button never arrived");
            thread::sleep(Duration::from_millis(1));
        }
        let frames = wire.during(RESEND * 3);
        assert!(frames.iter().all(|frame| frame.cause == Cause::Refresh));
    }

    /// An event split across two reads - which a socket may do and the kernel does not - is
    /// joined rather than dropped, and does not misframe what follows.
    #[test]
    fn an_event_split_across_reads_is_joined() {
        let (reader, feed, _wire) = rig(one_axis());
        let whole = event::encode(0, AXIS, 0, 32_767);
        feed.send(Ok(whole[..3].to_vec())).expect("feed");
        feed.send(Ok(whole[3..].to_vec())).expect("feed");
        wait_for_axis(&reader, 1.0);
        feed.send(Ok(axis(-32_767))).expect("feed");
        wait_for_axis(&reader, -1.0);
    }

    /// A new mapping goes out without waiting for the stick to move.
    #[test]
    fn a_new_mapping_is_sent_at_once() {
        let (reader, feed, wire) = rig(one_axis());
        feed.send(Ok(axis(32_767))).expect("feed");
        wait_for_axis(&reader, 1.0);
        assert!(reader.set_enabled(true));
        assert_eq!(wire.next().channels.get(1), Some(crate::mapping::MAX_US));
        let mut reversed = Binding::new(1, Source::Axis(0));
        reversed.reversed = true;
        reader.set_mapping(Mapping {
            bindings: vec![reversed],
        });
        let changed = wire.next_of(Cause::Changed);
        assert_eq!(changed.channels.get(1), Some(crate::mapping::MIN_US));
        // Recorded, if at all, before the sender moves on; the next frame proves it has.
        let _ = wire.next();
        assert_eq!(
            reader.latency().count(),
            0,
            "a remap is not stick latency: nothing new was read"
        );
    }

    /// A gamepad as it opens: the init burst puts the triggers at full deflection, and the default
    /// mapping must still send centred sticks, with the unmapped channels ignored on each half by
    /// that half's own convention.
    #[test]
    fn a_gamepad_opening_with_triggers_at_rest_sends_centred_sticks() {
        let (reader, feed, wire) = rig(Mapping::gamepad());
        let init: Vec<u8> = [0i16, 0, -32_767, 0, 0, -32_767]
            .into_iter()
            .zip(0u8..)
            .flat_map(|(value, number)| event::encode(0, AXIS | INIT, number, value))
            .collect();
        feed.send(Ok(init)).expect("feed");
        let until = Instant::now() + PATIENCE;
        while reader.reading().axes.len() < 6 {
            assert!(Instant::now() < until, "the init burst never arrived");
            thread::sleep(Duration::from_millis(1));
        }
        assert!(reader.set_enabled(true));
        let frame = wire.next();
        for channel in 1..=4 {
            assert_eq!(
                frame.channels.get(channel),
                Some(crate::mapping::CENTRE_US),
                "channel {channel} moved with the sticks centred"
            );
        }
        for channel in 5..=8 {
            assert_eq!(frame.channels.get(channel), Some(u16::MAX), "{channel}");
        }
        for channel in 9..=18 {
            assert_eq!(frame.channels.get(channel), Some(0), "{channel}");
        }
    }
}
