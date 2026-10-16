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

//! MAVLink 2 signing on the link: what `ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs` does with
//! it, and the fields `ExtLibs/ArduPilot/Mavlink/MAVState.cs` keeps for it per vehicle. The wire
//! format - the signature block and its SHA-256 - is `mp_mavlink::signing`'s; this is when a frame
//! is signed, with which key, and what is done with one that arrives signed.
//!
//! * Sending, `generatePacket` (`MAVLinkInterface.cs:1345-1433`): a frame for a vehicle whose
//!   `signing` is on carries the signed flag and a signature over the vehicle's key - zeros while
//!   it has none - its `sendlinkid`, a random byte chosen when the vehicle is first heard
//!   (`MAVState.cs:61`), and a timestamp in 10 µs since 2015-01-01 that moves on by one when it
//!   would repeat the last ([`timestamp_at`], [`Signing::sign`]).
//! * Receiving, `readPacketAsync` (`MAVLinkInterface.cs:5035-5065`) and `CheckSignature`
//!   (`:5481-5506`): a signed packet is counted, then checked against its vehicle's key and, if
//!   that fails, against every key in the store; one that passes makes that key the vehicle's,
//!   its link id the vehicle's `linkid`, and turns signing on for it - the C#'s "auto adapt"; one
//!   that passes none is dropped before anything sees it. Unsigned packets are let through
//!   whatever the state, as the C# lets them through; a log being played back is not checked.
//! * `setupSigning` (`:1526-1581`): `SETUP_SIGNING` sent to the vehicle twice, with the key and
//!   the time, or zeros and 0 to clear it; then signing on - the key left for the vehicle's own
//!   signed packets to supply - or off and the key forgotten. Asked from another thread, it is
//!   queued for the link thread, which sends and switches in that order ([`Signing::queue`]).
//! * `Mavlink2Signed`, the signed packets read since the start of the current second
//!   (`:420-424, 4912-4937, 5039`), which the signing window shows.
//! * `MAVState.mavlinkv2` (`MAVState.cs:172`), whether a vehicle is sent MAVLink 2, kept here
//!   because `generatePacket` decides a frame's framing and its signing together (`:1302-1437`):
//!   a vehicle is spoken to as its first packet speaks, and a MAVLink 2 packet from one spoken to
//!   in MAVLink 1 turns it to MAVLink 2 - "once set it cannot be reverted" (`:5016-5027`) - though
//!   `disableSigning` and the Link Stats window's "Mavlink 2" turn it off
//!   (`Controls/ConnectionStats.cs:176-179`), and `enableSigning` on. A frame for a vehicle spoken
//!   to in MAVLink 1 goes as MAVLink 1 - its base payload, never signed - when its message id fits
//!   a byte and neither id is wide ([`Signing::mavlink1`]); else as MAVLink 2, signed or not.
//!
//! The key store, `MAVAuthKeys.Keys`, is static in the C# - one for the application, read by every
//! link - and so is [`auth_keys`] here; the application loads `authkeys.xml` and hands its keys
//! over with [`set_auth_keys`] whenever they change.
//!
//! Where this is not the C#, and why:
//!
//! * a frame's vehicle is the one its `target_system` and `target_component` name - the one
//!   `generatePacket` is called for - and, for a message that names none (the heartbeat) or
//!   names a component never heard, the first vehicle of that system, or on the link, that is
//!   signing. The C#'s heartbeat loop sends one heartbeat per kind of vehicle, signed for those
//!   that sign (`MainV2.cs:2951-2978`); one link here sends one, signed when any vehicle signs,
//!   which a vehicle that does not sign accepts all the same;
//! * a frame's framing goes by the same vehicle, its target's - the component named if it has
//!   been heard, else the system's first heard - and a frame that names none (the heartbeat) goes
//!   as MAVLink 1 only when every vehicle heard is spoken to in MAVLink 1: the C#'s heartbeat loop
//!   sends a MAVLink 1 heartbeat and a MAVLink 2 one when its vehicles differ, one link here sends
//!   one, and a vehicle spoken to in MAVLink 1 reads MAVLink 2 all the same. A vehicle never heard
//!   is sent MAVLink 2, where the C#'s `MAVlist` would make it a state whose `mavlinkv2` is false;
//! * `signingignore` is not ported: nothing in the C# tree sets it.

use mp_os::ReadWrite as _;
use std::collections::BTreeMap;
use std::hash::{BuildHasher as _, Hasher as _};
use std::sync::RwLock;
use web_time::{Duration, SystemTime, UNIX_EPOCH};

use mp_mavlink::{
    Dialect as _, FieldValue, Frame, INCOMPAT_FLAG_SIGNED, MAX_PAYLOAD_LEN, STX_V2, SigningKey,
    encode_v1, v2_layout,
};
use mp_mavlink_dialects::all::{DIALECT, MavMessage, SetupSigning};
use mp_vehicle::VehicleId;

/// The checksum's two bytes.
const CHECKSUM_LEN: usize = 2;
/// Seconds from the Unix epoch to 2015-01-01, `new DateTime(2015, 1, 1)`.
const EPOCH_2015: u64 = 1_420_070_400;

/// `MAVAuthKeys.Keys`' keys.
static AUTH_KEYS: RwLock<Vec<[u8; 32]>> = RwLock::new(Vec::new());

/// Hands every link the store's keys: `MAVAuthKeys.Keys` as it stands. The application calls it
/// when it has loaded the store and whenever a key is added or removed.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVAuthKeys.cs:22, 45-55`
pub fn set_auth_keys(keys: Vec<[u8; 32]>) {
    if let Ok(mut held) = AUTH_KEYS.os_write() {
        *held = keys;
    }
}

/// The keys every link checks a signed packet against, after the vehicle's own.
#[must_use]
pub fn auth_keys() -> Vec<[u8; 32]> {
    AUTH_KEYS
        .os_read()
        .map(|keys| keys.clone())
        .unwrap_or_default()
}

/// `(UInt64)((DateTime.UtcNow - new DateTime(2015, 1, 1)).TotalMilliseconds * 100)` at `since_unix`
/// after the Unix epoch: the signing timestamp, 10 µs since 2015, by the C#'s arithmetic -
/// `TimeSpan.TotalMilliseconds` is the ticks times 1/10000 in a double, so the last digit can be
/// a unit from the exact count. 0 before 2015.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1393, 1552`
#[must_use]
#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss
)] // `(UInt64)` of a double, and ticks as the double the C# makes of them
pub fn timestamp_at(since_unix: Duration) -> u64 {
    let ticks = since_unix.as_nanos() / 100;
    let epoch = u128::from(EPOCH_2015) * 10_000_000;
    if ticks <= epoch {
        return 0;
    }
    let ticks = (ticks - epoch) as f64;
    // `TimeSpan.MillisecondsPerTick`, `1.0 / TicksPerMillisecond`.
    let millis = ticks * (1.0 / 10_000.0);
    (millis * 100.0) as u64
}

/// The signing timestamp now.
#[must_use]
pub fn timestamp_now() -> u64 {
    timestamp_at(
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default(),
    )
}

/// `DateTime.UtcNow.Second`: the seconds field, which the signed-packet count is reset on.
#[must_use]
#[allow(clippy::cast_possible_truncation)] // 0 to 59
pub fn utc_second() -> u8 {
    (SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs()
        % 60) as u8
}

/// A vehicle's signing state: `MAVState`'s `signing`, `signingKey`, `sendlinkid`, `linkid` and
/// `timestamp`.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVState.cs:61-62, 151-162`
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct MavSigning {
    /// `signing`: whether frames to it are signed.
    pub signing: bool,
    /// `signingKey`: the key its packets last passed with, or none.
    pub key: Option<[u8; 32]>,
    /// `sendlinkid`: the link id signed into what is sent to it.
    pub send_link_id: u8,
    /// `linkid`: the link id its last good signed packet carried.
    pub link_id: u8,
    /// `timestamp`: the last timestamp signed into what was sent to it.
    pub timestamp: u64,
}

impl core::fmt::Debug for MavSigning {
    /// Never prints the key.
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("MavSigning")
            .field("signing", &self.signing)
            .field("key", &self.key.map(|_| "<redacted>"))
            .field("send_link_id", &self.send_link_id)
            .field("link_id", &self.link_id)
            .field("timestamp", &self.timestamp)
            .finish()
    }
}

impl MavSigning {
    /// A vehicle first heard: not signing, no key, `sendlinkid = (byte)(new Random().Next(256))`.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVState.cs:61-62`
    #[must_use]
    #[allow(clippy::cast_possible_truncation)] // one random byte
    pub fn new() -> Self {
        let random = std::collections::hash_map::RandomState::new()
            .build_hasher()
            .finish();
        Self {
            signing: false,
            key: None,
            send_link_id: random as u8,
            link_id: 0,
            timestamp: 0,
        }
    }
}

impl Default for MavSigning {
    fn default() -> Self {
        Self::new()
    }
}

/// What `setupSigning(sysid, compid, userseed, key)` was asked.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1526-1581`
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct Setup {
    /// The vehicle.
    pub target: VehicleId,
    /// `shauser`: the key given, cut or padded to 32 bytes, or the SHA-256 of the seed.
    pub key: [u8; 32],
    /// `clearkey`: no key given and an empty seed - signing to be cleared.
    pub clear: bool,
}

impl core::fmt::Debug for Setup {
    /// Never prints the key.
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Setup")
            .field("target", &self.target)
            .field("clear", &self.clear)
            .finish_non_exhaustive()
    }
}

impl Setup {
    /// The key `setupSigning` works out: `key` when there is one, `Array.Resize`d to 32 bytes,
    /// else the SHA-256 of `userseed`'s UTF-8; an empty seed with no key clears signing.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1531-1546`
    #[must_use]
    pub fn new(target: VehicleId, userseed: &str, key: Option<&[u8]>) -> Self {
        let (key, clear) = match key {
            Some(given) => {
                let mut key = [0u8; 32];
                for (slot, byte) in key.iter_mut().zip(given) {
                    *slot = *byte;
                }
                (key, false)
            }
            None => (
                *SigningKey::from_passphrase(userseed).as_bytes(),
                userseed.is_empty(),
            ),
        };
        Self { target, key, clear }
    }

    /// The `SETUP_SIGNING` it sends: the key and `timestamp`, or, clearing, zeros and 0.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1548-1562`
    #[must_use]
    pub const fn message(&self, timestamp: u64) -> MavMessage {
        let (initial_timestamp, secret_key) = if self.clear {
            (0, [0u8; 32])
        } else {
            (timestamp, self.key)
        };
        MavMessage::SetupSigning(SetupSigning {
            initial_timestamp,
            target_system: self.target.payload_target(),
            target_component: self.target.compid,
            secret_key,
        })
    }
}

/// A link's signing: each vehicle's state, the `setupSigning`s waiting for the link thread, and
/// `Mavlink2Signed`; and each vehicle's `mavlinkv2`.
#[derive(Debug, Default)]
pub struct Signing {
    vehicles: BTreeMap<VehicleId, MavSigning>,
    /// `MAVState.mavlinkv2`, for every vehicle heard or switched: whether frames to it are
    /// MAVLink 2. `// C#: ExtLibs/ArduPilot/Mavlink/MAVState.cs:172`
    mavlinkv2: BTreeMap<VehicleId, bool>,
    setups: Vec<Setup>,
    /// `_mavlink2signed`.
    signed: u32,
    /// `_bpstime.Second`: `DateTime.MinValue`'s 0 until the first reset.
    second: u8,
}

impl Signing {
    /// `MAVlist[sysid, compid]`'s signing state, once the vehicle has been heard or signed to.
    #[must_use]
    pub fn vehicle(&self, id: VehicleId) -> Option<MavSigning> {
        self.vehicles.get(&id).copied()
    }

    /// `Mavlink2Signed`: signed packets read since the count last started again.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:420`
    #[must_use]
    pub const fn signed_packets(&self) -> u32 {
        self.signed
    }

    /// `setupSigning`, asked: queued for the link thread, which sends its `SETUP_SIGNING`s and
    /// then switches signing in that order, as the C#'s call does on the caller's thread.
    pub fn queue(&mut self, setup: Setup) {
        self.setups.push(setup);
    }

    /// The `setupSigning`s asked since the last call, oldest first.
    pub fn take_setups(&mut self) -> Vec<Setup> {
        std::mem::take(&mut self.setups)
    }

    /// `setupSigning`'s end, its `SETUP_SIGNING`s sent: `enableSigning`, the key left for the
    /// vehicle's signed packets to supply ("we will auto adapt to this key"), or the key
    /// forgotten and `disableSigning`. Returns `signing` as the C#'s call does.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1568-1581`
    pub fn finish(&mut self, setup: &Setup) -> bool {
        if setup.clear {
            self.state(setup.target).key = None;
            self.disable(setup.target)
        } else {
            self.enable(setup.target)
        }
    }

    /// `enableSigning`: signing on, and MAVLink 2 with it.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1583-1589`
    pub fn enable(&mut self, id: VehicleId) -> bool {
        self.state(id).signing = true;
        self.mavlinkv2.insert(id, true);
        true
    }

    /// `disableSigning`: signing off, and MAVLink 2 with it, until the vehicle's next MAVLink 2
    /// packet ([`Signing::heard`]). `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1591-1597`
    pub fn disable(&mut self, id: VehicleId) -> bool {
        self.state(id).signing = false;
        self.mavlinkv2.insert(id, false);
        false
    }

    /// The Link Stats window's "Signing" ticked or not, `MAV.signing = chk_signing.Checked`: the
    /// flag alone, the key and the framing as they were. `// C#: Controls/ConnectionStats.cs:181-184`
    pub fn set_signing(&mut self, id: VehicleId, on: bool) {
        self.state(id).signing = on;
    }

    /// `MAVlist[sysid, compid].mavlinkv2`: whether frames to it are MAVLink 2; `None` for a vehicle
    /// never heard or switched. `// C#: ExtLibs/ArduPilot/Mavlink/MAVState.cs:172`
    #[must_use]
    pub fn mavlinkv2(&self, id: VehicleId) -> Option<bool> {
        self.mavlinkv2.get(&id).copied()
    }

    /// The Link Stats window's "Mavlink 2" ticked or not, `MAV.mavlinkv2 = chk_mavlink2.Checked`.
    /// `// C#: Controls/ConnectionStats.cs:176-179`
    pub fn set_mavlinkv2(&mut self, id: VehicleId, on: bool) {
        self.mavlinkv2.insert(id, on);
    }

    /// A packet from `id` that has passed its checksum, MAVLink 2 or not: a vehicle first heard is
    /// spoken to as it speaks, and one spoken to in MAVLink 1 that speaks MAVLink 2 is spoken to in
    /// it from then on. Allocates only for a vehicle first heard.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5016-5027`
    pub fn heard(&mut self, id: VehicleId, v2: bool) {
        let held = self.mavlinkv2.entry(id).or_insert(false);
        if !*held {
            *held = v2;
        }
    }

    /// Whether any vehicle is spoken to in MAVLink 1: the link thread looks at every packet's
    /// framing only while one is.
    #[must_use]
    pub fn any_mavlink1(&self) -> bool {
        self.mavlinkv2.values().any(|v2| !v2)
    }

    /// `frame`, written by this link, as MAVLink 1 into `out` - its length - when the vehicle it
    /// is for is spoken to in MAVLink 1 (see the module documentation for which that is), or
    /// `None` to send it as it is: `generatePacket`'s first branch, `!mavlinkv2 && messageType <
    /// 256 && sysid <= 255 && gcssysid <= 255` - here a v2 frame with no flags, neither id wide
    /// nor signed, whose message id fits a byte. Its payload is the message's base fields,
    /// `Array.Resize(ref data, info.minlength)`, and it is never signed. Allocates nothing.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1302-1344`
    pub fn mavlink1(&self, frame: &[u8], out: &mut [u8]) -> Option<usize> {
        if !self.any_mavlink1() || frame.first() != Some(&STX_V2) || *frame.get(2)? != 0 {
            return None;
        }
        let (header_len, msgid) = v2_layout(frame)?;
        if msgid > u32::from(u8::MAX) {
            return None;
        }
        let len = usize::from(*frame.get(1)?);
        let payload = frame.get(header_len..header_len + len)?;
        let v2 = match payload_targets(msgid, payload)? {
            (Some(system), component) if system != 0 => {
                self.framing_for(u32::from(system), component)?
            }
            // For no vehicle in particular: MAVLink 1 only when every vehicle is spoken to in it.
            _ => self.mavlinkv2.values().any(|v2| *v2),
        };
        if v2 {
            return None;
        }
        to_mavlink1(frame, msgid, payload, out)
    }

    /// `mavlinkv2` of the vehicle a frame for `system` and `component` goes to: the component if it
    /// has been heard, else the system's first; `None` for a system never heard.
    fn framing_for(&self, system: u32, component: Option<u8>) -> Option<bool> {
        if let Some(component) = component
            && let Some(v2) = self.mavlinkv2.get(&VehicleId::new(system, component))
        {
            return Some(*v2);
        }
        self.mavlinkv2
            .iter()
            .find(|(id, _)| id.sysid == system)
            .map(|(_, v2)| *v2)
    }

    /// `MAVlist[sysid, compid]`, made when first asked for as the C#'s list makes a hidden one.
    fn state(&mut self, id: VehicleId) -> &mut MavSigning {
        self.vehicles.entry(id).or_default()
    }

    /// A packet read in the second `second` (`DateTime.UtcNow.Second`): a new second starts the
    /// count again.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4912, 4932-4937`
    pub fn new_second(&mut self, second: u8) {
        if self.second != second {
            self.second = second;
            self.signed = 0;
        }
    }

    /// A packet read: true to keep it. An unsigned one is kept; a signed one is counted, then
    /// kept if it passes its vehicle's key or one of `keys` - which becomes the vehicle's, with
    /// its link id, and signing to it on - and dropped if it passes none.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5035-5065, 5481-5506`
    pub fn check(&mut self, frame: &Frame<'_>, keys: &[[u8; 32]]) -> bool {
        let Some(signature) = frame.signature else {
            return true;
        };
        self.signed = self.signed.saturating_add(1);
        let link_id = signature.first().copied().unwrap_or(0);
        let state = self.state(VehicleId::new(frame.sysid, frame.compid));
        let passed = state
            .key
            .into_iter()
            .chain(keys.iter().copied())
            .find(|key| mp_mavlink::verify(&SigningKey::new(*key), frame));
        let Some(key) = passed else {
            return false;
        };
        state.link_id = link_id;
        state.key = Some(key);
        state.signing = true;
        // `enableSigning`, which turns MAVLink 2 on as well.
        // `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1583-1589, 5502`
        self.mavlinkv2
            .insert(VehicleId::new(frame.sysid, frame.compid), true);
        true
    }

    /// [`Signing::check`] against the store [`set_auth_keys`] last handed over.
    pub fn check_against_store(&mut self, frame: &Frame<'_>) -> bool {
        if frame.signature.is_none() {
            return true;
        }
        let Ok(keys) = AUTH_KEYS.os_read() else {
            return self.check(frame, &[]);
        };
        self.check(frame, &keys)
    }

    /// `frame`, written by this link, signed if the vehicle it is for is being signed to - see
    /// the module documentation for which that is - or `None` to send it as it is. `now` gives
    /// the timestamp, asked only when one is signed.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1350-1358, 1381-1432`
    pub fn sign(&mut self, frame: &[u8], now: impl FnOnce() -> u64) -> Option<Vec<u8>> {
        if !self.vehicles.values().any(|state| state.signing) {
            return None;
        }
        let id = self.vehicle_for(target_of(frame))?;
        let state = self.vehicles.get_mut(&id)?;
        if !state.signing {
            return None;
        }
        let mut timestamp = now();
        if timestamp == state.timestamp {
            timestamp += 1;
        }
        state.timestamp = timestamp;
        // `if (signingKey == null || signingKey.Length != 32) signingKey = new byte[32];`
        let key = state.key.unwrap_or([0; 32]);
        sign_frame(frame, &key, state.send_link_id, timestamp)
    }

    /// The vehicle a frame addressed to `target` is sent for.
    fn vehicle_for(&self, target: Option<(u32, Option<u8>)>) -> Option<VehicleId> {
        let signing = |id: &&VehicleId| self.vehicles.get(id).is_some_and(|state| state.signing);
        match target {
            Some((system, component)) => {
                if let Some(component) = component {
                    let id = VehicleId::new(system, component);
                    if self.vehicles.contains_key(&id) {
                        return Some(id);
                    }
                }
                if !self.vehicles.keys().any(|id| id.sysid == system) {
                    return None;
                }
                self.vehicles
                    .keys()
                    .filter(|id| id.sysid == system)
                    .find(signing)
                    .copied()
            }
            None => self.vehicles.keys().find(signing).copied(),
        }
    }
}

/// Who a v2 frame is addressed to, `GetTargetSystem`'s way: the header's target with
/// `TARGET32`, else its message's `target_system` (`target` in the few that name it so), and
/// `target_component` if it has one; `None` for a message that names no system, or names system
/// 0, everyone. `// C#: ExtLibs/Mavlink/MAVLinkMessage.cs:39-47`
fn target_of(frame: &[u8]) -> Option<(u32, Option<u8>)> {
    let len = usize::from(*frame.get(1)?);
    let (header_len, msgid) = v2_layout(frame)?;
    let payload = frame.get(header_len..header_len + len)?;
    // `TARGET32`'s four bytes end the header.
    let wide = (frame.get(2)? & mp_mavlink::INCOMPAT_FLAG_TARGET32 != 0)
        .then(|| frame.get(header_len - 4..header_len))
        .flatten()
        .and_then(|bytes| bytes.try_into().ok())
        .map(u32::from_le_bytes);
    let fields = MavMessage::decode(msgid, payload)?.fields();
    let value = |name: &str| {
        fields
            .iter()
            .find(|(field, _)| *field == name)
            .and_then(|(_, value)| match value {
                FieldValue::Unsigned(value) => u8::try_from(*value).ok(),
                _ => None,
            })
    };
    let system = wide.or_else(|| {
        value("target_system")
            .or_else(|| value("target"))
            .map(u32::from)
    })?;
    (system != 0).then(|| (system, value("target_component")))
}

/// A payload's `target_system` - `target` where a message names it so - and `target_component`,
/// one-byte fields read where the message's fields' wire order puts them, a truncated payload's
/// missing bytes zeros: what [`target_of`] reads from the fields' list, without making one, so a
/// frame's framing is decided without allocating. `None` for a message this dialect does not know.
fn payload_targets(msgid: u32, payload: &[u8]) -> Option<(Option<u8>, Option<u8>)> {
    let fields = MavMessage::decode(msgid, payload)?.field_info();
    let byte_at = |at: usize| Some(payload.get(at).copied().unwrap_or(0));
    let (mut system, mut target, mut component) = (None, None, None);
    let mut at = 0;
    for field in fields {
        if field.array_len == 0 && field.base_type == "uint8_t" {
            match field.name {
                "target_system" => system = byte_at(at),
                "target" => target = byte_at(at),
                "target_component" => component = byte_at(at),
                _ => {}
            }
        }
        at += wire_size(field.base_type) * field.array_len.max(1);
    }
    Some((system.or(target), component))
}

/// The bytes one element of a field of XML type `base_type` takes on the wire.
fn wire_size(base_type: &str) -> usize {
    match base_type {
        "uint64_t" | "int64_t" | "double" => 8,
        "uint32_t" | "int32_t" | "float" => 4,
        "uint16_t" | "int16_t" => 2,
        // uint8_t, int8_t, char, uint8_t_mavlink_version.
        _ => 1,
    }
}

/// A v2 frame with no flags - `payload` its payload, `msgid` its message's id - framed again as
/// MAVLink 1 into `out`, as `generatePacket`'s first branch frames a message: its sequence number
/// and ids, the payload cut or padded to the message's base length, and the checksum over those.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1306-1343`
fn to_mavlink1(frame: &[u8], msgid: u32, payload: &[u8], out: &mut [u8]) -> Option<usize> {
    let info = DIALECT.info(msgid)?;
    let base = usize::from(info.min_len);
    let mut body = [0u8; MAX_PAYLOAD_LEN];
    for (slot, byte) in body.iter_mut().zip(payload.iter().take(base)) {
        *slot = *byte;
    }
    // The v2 header without wide fields: STX, length, two flag bytes, sequence, system, component.
    let (seq, sysid, compid) = (*frame.get(4)?, *frame.get(5)?, *frame.get(6)?);
    encode_v1(out, seq, sysid, compid, msgid, body.get(..base)?, info.crc_extra).ok()
}

/// A MAVLink 2 frame from this link signed: the signed flag set in its header, its checksum
/// worked out again over the header that now carries it, and the 13-byte block - `link_id`, the
/// 48-bit `timestamp`, the first six bytes of SHA-256 over `key`, the frame and those seven -
/// appended. `None` for anything not a whole v2 frame of a message this dialect knows.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1352-1360, 1366-1432`
#[must_use]
pub fn sign_frame(frame: &[u8], key: &[u8; 32], link_id: u8, timestamp: u64) -> Option<Vec<u8>> {
    if frame.first() != Some(&STX_V2) {
        return None;
    }
    let len = usize::from(*frame.get(1)?);
    let (header_len, msgid) = v2_layout(frame)?;
    let crc_extra = DIALECT.crc_extra(msgid)?;
    let end = header_len + len + CHECKSUM_LEN;
    let mut out = Vec::with_capacity(end + mp_mavlink::SIGNATURE_LEN);
    out.extend_from_slice(frame.get(..end)?);
    *out.get_mut(2)? |= INCOMPAT_FLAG_SIGNED;
    let checksum = mp_mavlink::crc::checksum(out.get(1..end - CHECKSUM_LEN)?, crc_extra);
    out.get_mut(end - CHECKSUM_LEN..end)?
        .copy_from_slice(&checksum.to_le_bytes());
    let signature = mp_mavlink::sign(&SigningKey::new(*key), link_id, timestamp, &out);
    out.extend_from_slice(&signature);
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use mp_mavlink::FrameDecoder;
    use mp_mavlink_dialects::all::Heartbeat;

    /// `SHA256("Correct Horse 42!")`, the key `MAVAuthKeys.AddKey("bench", ...)` makes - its
    /// base64 is what the C#'s store holds for it (`oBPRD6pB...`).
    const KEY: [u8; 32] = [
        0xa0, 0x13, 0xd1, 0x0f, 0xaa, 0x41, 0x83, 0x33, 0xbc, 0x4d, 0xb7, 0xfa, 0x9d, 0x0c, 0x49,
        0xe5, 0xa0, 0x90, 0x6a, 0x8a, 0xc9, 0x9a, 0xfe, 0x41, 0x7d, 0x3d, 0xf9, 0x32, 0xbf, 0x68,
        0xc8, 0x33,
    ];

    /// `0x123456789A`, a timestamp in 10 µs units.
    const TIMESTAMP: u64 = 0x0012_3456_789a;

    fn hex(text: &str) -> Vec<u8> {
        (0..text.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&text[i..i + 2], 16).unwrap())
            .collect()
    }

    /// One frame of `message` from `sysid`/`compid` with sequence `seq`, unsigned.
    fn unsigned(sysid: u32, compid: u8, seq: u8, message: &MavMessage) -> Vec<u8> {
        let mut payload = [0u8; 255];
        let len = message.encode(&mut payload);
        let mut out = [0u8; mp_mavlink::MAX_FRAME_LEN];
        let n = mp_mavlink::encode_v2(
            &mut out,
            seq,
            sysid,
            compid,
            message.id(),
            &payload[..len],
            message.crc_extra(),
            0,
        )
        .unwrap();
        out[..n].to_vec()
    }

    fn gcs_heartbeat() -> MavMessage {
        MavMessage::Heartbeat(Heartbeat {
            custom_mode: 0,
            r#type: 6,
            autopilot: 8,
            base_mode: 0,
            system_status: 0,
            mavlink_version: 3,
        })
    }

    /// Each frame `bytes` holds, parsed, handed to `each`.
    fn frames(bytes: &[u8], mut each: impl FnMut(&Frame<'_>)) {
        let mut decoder = FrameDecoder::new();
        decoder.push_and_drain(bytes, &DIALECT, |frame| each(frame));
    }

    /// A frame with both wide fields - a 32-bit sender and `TARGET32` - signed by the link: the
    /// flag set and the checksum worked out where its header ends, the signature over the whole
    /// header; its target read from the header, as `GetTargetSystem` reads it.
    #[test]
    fn a_frame_with_32_bit_ids_is_signed_over_its_whole_header() {
        let command = MavMessage::CommandLong(mp_mavlink_dialects::all::CommandLong {
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
            command: 300,
            target_system: 255,
            target_component: 190,
            confirmation: 1,
        });
        let mut payload = [0u8; 255];
        let len = command.encode(&mut payload);
        let mut out = [0u8; mp_mavlink::MAX_FRAME_LEN];
        let n = mp_mavlink::encode_v2_targeted(
            &mut out,
            9,
            0x0001_0000,
            190,
            command.id(),
            &payload[..len],
            command.crc_extra(),
            0,
            Some(u32::MAX),
        )
        .unwrap();
        assert_eq!(target_of(&out[..n]), Some((u32::MAX, Some(190))));
        let signed = sign_frame(&out[..n], &KEY, 3, TIMESTAMP).expect("signed");
        let (frame, _) = mp_mavlink::parse(&signed, &DIALECT).expect("its checksum holds");
        assert_eq!(frame.sysid, 0x0001_0000);
        assert_eq!(frame.target_system, Some(u32::MAX));
        assert!(mp_mavlink::verify(&SigningKey::new(KEY), &frame));
    }

    /// pymavlink signs the GCS's heartbeat - seq 7, from 255/190, link id 3 - to these bytes
    /// (`MAVLink.signing` with `sign_outgoing`, `pymavlink/dialects/v20/ardupilotmega.py`), as
    /// `generatePacket` lays them out: the flag, the checksum over it, the block.
    #[test]
    fn a_frame_is_signed_as_pymavlink_signs_it() {
        let frame = unsigned(255, 190, 7, &gcs_heartbeat());
        let signed = sign_frame(&frame, &KEY, 3, TIMESTAMP).unwrap();
        assert_eq!(
            signed,
            hex("fd09010007ffbe000000000000000608000003f861039a78563412002698e8c3b37a")
        );
        // And it parses as a signed frame that the key verifies.
        let mut checked = 0;
        frames(&signed, |parsed| {
            assert!(parsed.is_signed());
            assert!(mp_mavlink::verify(&SigningKey::new(KEY), parsed));
            checked += 1;
        });
        assert_eq!(checked, 1);
    }

    /// A vehicle's heartbeat pymavlink signed with the key passes with it, makes it the
    /// vehicle's with its link id, and turns signing to it on; with the wrong key it is dropped
    /// and nothing is adopted. Every signed one is counted, passed or not.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5035-5065, 5481-5506`
    #[test]
    fn a_signed_packet_passes_its_key_and_no_other() {
        let vehicle = hex("fd0901002a01010000000400000002035103036dd500ff7856341200a3e770ff14d6");
        let id = VehicleId::new(1, 1);
        let wrong = *SigningKey::from_passphrase("not the key").as_bytes();

        let mut signing = Signing::default();
        frames(&vehicle, |frame| assert!(!signing.check(frame, &[wrong])));
        assert_eq!(signing.signed_packets(), 1);
        let state = signing.vehicle(id).unwrap();
        assert_eq!((state.signing, state.key), (false, None));

        frames(&vehicle, |frame| {
            assert!(signing.check(frame, &[wrong, KEY]))
        });
        let state = signing.vehicle(id).unwrap();
        assert_eq!(
            (state.signing, state.key, state.link_id),
            (true, Some(KEY), 0)
        );
        assert_eq!(signing.signed_packets(), 2);

        // Its own key first: passes with no store at all.
        frames(&vehicle, |frame| assert!(signing.check(frame, &[])));

        // Unsigned: kept, not counted.
        let plain = unsigned(1, 1, 0, &gcs_heartbeat());
        frames(&plain, |frame| assert!(signing.check(frame, &[])));
        assert_eq!(signing.signed_packets(), 3);

        // A new second starts the count again; the same second does not.
        signing.new_second(5);
        assert_eq!(signing.signed_packets(), 0);
        frames(&vehicle, |frame| assert!(signing.check(frame, &[])));
        signing.new_second(5);
        assert_eq!(signing.signed_packets(), 1);
    }

    /// The timestamp is the C#'s: 10 µs since 2015-01-01 UTC, 0 before, and one sent at the same
    /// instant as the last moves on by one.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1393-1398`
    #[test]
    fn the_timestamp_is_ten_microseconds_since_2015_and_never_repeats() {
        let at = |seconds: u64, micros: u64| {
            timestamp_at(Duration::from_secs(seconds) + Duration::from_micros(micros))
        };
        assert_eq!(at(EPOCH_2015, 0), 0);
        assert_eq!(at(EPOCH_2015 - 1, 0), 0);
        assert_eq!(at(EPOCH_2015 + 1, 0), 100_000);
        assert_eq!(at(EPOCH_2015 + 1, 20), 100_002);
        // 2026-10-02 within a unit of the exact count.
        let exact = (1_790_000_000 - EPOCH_2015) * 100_000 + 12_345;
        assert!(at(1_790_000_000, 123_450).abs_diff(exact) <= 1);

        let mut signing = Signing::default();
        let id = VehicleId::new(1, 1);
        signing.enable(id);
        let frame = unsigned(255, 190, 0, &gcs_heartbeat());
        let first = signing.sign(&frame, || TIMESTAMP).unwrap();
        let second = signing.sign(&frame, || TIMESTAMP).unwrap();
        let third = signing.sign(&frame, || TIMESTAMP + 7).unwrap();
        let stamp = |signed: &[u8]| {
            let mut stamp = 0;
            frames(signed, |frame| {
                stamp = mp_mavlink::signing::signature_meta(frame.signature.unwrap())
                    .unwrap()
                    .1;
            });
            stamp
        };
        assert_eq!(stamp(&first), TIMESTAMP);
        assert_eq!(stamp(&second), TIMESTAMP + 1);
        assert_eq!(stamp(&third), TIMESTAMP + 7);
        assert_eq!(signing.vehicle(id).unwrap().timestamp, TIMESTAMP + 7);
    }

    /// Signing on with no key yet - `setupSigning`'s "auto adapt" - signs with 32 zero bytes and
    /// the vehicle's `sendlinkid`; with signing off nothing is signed.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1403-1413`
    #[test]
    fn a_vehicle_with_no_key_is_signed_to_with_zeros() {
        let mut signing = Signing::default();
        let id = VehicleId::new(1, 1);
        let frame = unsigned(255, 190, 0, &gcs_heartbeat());
        assert_eq!(signing.sign(&frame, || TIMESTAMP), None);
        signing.enable(id);
        let link = signing.vehicle(id).unwrap().send_link_id;
        let signed = signing.sign(&frame, || TIMESTAMP).unwrap();
        assert_eq!(
            signed,
            sign_frame(&frame, &[0; 32], link, TIMESTAMP).unwrap()
        );
        signing.disable(id);
        assert_eq!(signing.sign(&frame, || TIMESTAMP + 5), None);
    }

    /// A frame goes out under the state of the vehicle its target names: one addressed to a
    /// vehicle not signing is not signed while another is, and a heartbeat - no target - goes
    /// under the signing one's.
    #[test]
    fn a_frame_is_signed_for_the_vehicle_it_is_addressed_to() {
        let mut signing = Signing::default();
        let autopilot = VehicleId::new(1, 1);
        let other = VehicleId::new(2, 1);
        signing.enable(autopilot);
        signing.disable(other);
        let to = |target: VehicleId| Setup::new(target, "", None).message(0);
        let to_other = unsigned(255, 190, 0, &to(other));
        let to_autopilot = unsigned(255, 190, 0, &to(autopilot));
        assert_eq!(signing.sign(&to_other, || TIMESTAMP), None);
        assert!(signing.sign(&to_autopilot, || TIMESTAMP).is_some());
        let heartbeat = unsigned(255, 190, 0, &gcs_heartbeat());
        assert!(signing.sign(&heartbeat, || TIMESTAMP + 9).is_some());
        // A component never heard of a system that signs: that system's signing one.
        let to_gimbal = unsigned(255, 190, 0, &to(VehicleId::new(1, 154)));
        assert!(signing.sign(&to_gimbal, || TIMESTAMP + 20).is_some());
        // A system never heard: not signed.
        let to_stranger = unsigned(255, 190, 0, &to(VehicleId::new(9, 1)));
        assert_eq!(signing.sign(&to_stranger, || TIMESTAMP + 30), None);
    }

    /// `setupSigning`: the key given, resized to 32, or the seed's SHA-256; the `SETUP_SIGNING`
    /// pymavlink encodes for that key and time to the same bytes; an empty seed clears - zeros
    /// and 0 sent, then signing off and the key forgotten - and anything else turns signing on
    /// without a key.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1526-1581`
    #[test]
    fn setup_signing_sends_the_key_then_switches() {
        let id = VehicleId::new(1, 1);
        let setup = Setup::new(id, "Correct Horse 42!", None);
        assert_eq!((setup.key, setup.clear), (KEY, false));
        assert_eq!(Setup::new(id, "", Some(&KEY)).key, KEY);
        assert!(!Setup::new(id, "", Some(&KEY)).clear);
        let short = Setup::new(id, "", Some(&[1, 2, 3]));
        assert_eq!(&short.key[..4], &[1, 2, 3, 0]);
        assert_eq!(
            unsigned(255, 190, 0, &setup.message(TIMESTAMP)),
            hex(
                "fd2a000000ffbe0001009a785634120000000101a013d10faa418333bc4db7fa9d0c49e5a0906a8a\
                 c99afe417d3df932bf68c833dc73"
            )
        );

        let mut signing = Signing::default();
        signing.queue(setup);
        assert_eq!(signing.take_setups(), [setup]);
        assert!(signing.take_setups().is_empty());
        assert!(signing.finish(&setup));
        assert_eq!(signing.vehicle(id).unwrap().key, None, "left to adapt");
        assert!(signing.vehicle(id).unwrap().signing);

        // The vehicle signs with it: adopted.
        let vehicle = hex("fd0901002a01010000000400000002035103036dd500ff7856341200a3e770ff14d6");
        frames(&vehicle, |frame| assert!(signing.check(frame, &[KEY])));
        assert_eq!(signing.vehicle(id).unwrap().key, Some(KEY));

        let clear = Setup::new(id, "", None);
        assert!(clear.clear);
        let MavMessage::SetupSigning(sent) = clear.message(TIMESTAMP) else {
            panic!("SETUP_SIGNING");
        };
        assert_eq!((sent.initial_timestamp, sent.secret_key), (0, [0; 32]));
        assert_eq!((sent.target_system, sent.target_component), (1, 1));
        assert!(!signing.finish(&clear));
        let state = signing.vehicle(id).unwrap();
        assert_eq!((state.signing, state.key), (false, None));
    }

    /// For every message of the dialect, the target bytes read where the wire order puts them are
    /// the ones its fields' list names, from a payload whose every byte differs - and from one cut
    /// short, its missing bytes zeros.
    #[test]
    fn the_targets_are_read_where_the_fields_put_them() {
        let named = |message: &MavMessage, name: &str| {
            message
                .fields()
                .into_iter()
                .find(|(field, _)| *field == name)
                .and_then(|(_, value)| match value {
                    FieldValue::Unsigned(value) => u8::try_from(value).ok(),
                    _ => None,
                })
        };
        let mut with_targets = 0;
        for info in DIALECT.messages() {
            let full: Vec<u8> = (0..info.len).map(|at| at.wrapping_add(1)).collect();
            for payload in [&full[..], &full[..usize::from(info.min_len) / 2]] {
                let message = MavMessage::decode(info.id, payload).unwrap();
                let system = named(&message, "target_system").or_else(|| named(&message, "target"));
                let expected = (system, named(&message, "target_component"));
                assert_eq!(
                    payload_targets(info.id, payload),
                    Some(expected),
                    "{} ({})",
                    info.name,
                    payload.len()
                );
                if expected.0.is_some() {
                    with_targets += 1;
                }
            }
        }
        assert!(with_targets > 100, "{with_targets}");
        assert_eq!(payload_targets(u32::MAX, &[]), None);
    }

    /// `mavlinkv2` as the C# keeps it: a vehicle spoken to as its first packet speaks, MAVLink 2
    /// from then on once it speaks it; switched by the window's tick, by `disableSigning` and
    /// `enableSigning`, and by a signed packet that passes.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1583-1597, 5016-5027, 5502`
    #[test]
    fn mavlinkv2_follows_what_the_vehicle_speaks() {
        let mut signing = Signing::default();
        let v1 = VehicleId::new(1, 1);
        let v2 = VehicleId::new(2, 1);
        assert_eq!(signing.mavlinkv2(v1), None);
        assert!(!signing.any_mavlink1());
        signing.heard(v1, false);
        signing.heard(v2, true);
        assert_eq!((signing.mavlinkv2(v1), signing.mavlinkv2(v2)), (Some(false), Some(true)));
        assert!(signing.any_mavlink1());
        signing.heard(v2, false);
        assert_eq!(signing.mavlinkv2(v2), Some(true), "once set it cannot be reverted");
        signing.heard(v1, true);
        assert_eq!(signing.mavlinkv2(v1), Some(true));
        assert!(!signing.any_mavlink1());
        signing.set_mavlinkv2(v1, false);
        assert_eq!(signing.mavlinkv2(v1), Some(false));
        signing.enable(v1);
        assert_eq!(signing.mavlinkv2(v1), Some(true));
        signing.disable(v1);
        assert_eq!(signing.mavlinkv2(v1), Some(false));
        // The window's "Signing": the flag alone.
        signing.set_signing(v1, true);
        assert!(signing.vehicle(v1).unwrap().signing);
        assert_eq!(signing.mavlinkv2(v1), Some(false));
        // A signed packet that passes: `enableSigning`.
        let vehicle = hex("fd0901002a01010000000400000002035103036dd500ff7856341200a3e770ff14d6");
        frames(&vehicle, |frame| assert!(signing.check(frame, &[KEY])));
        assert_eq!(signing.mavlinkv2(v1), Some(true));
    }

    /// What goes as MAVLink 1: a frame to a vehicle spoken to in it - the same sequence number,
    /// ids and message, the base payload, a checksum the parser accepts - and nothing else: a
    /// vehicle spoken to in MAVLink 2, a system never heard, a message whose id needs three bytes,
    /// a wide id; one for no vehicle only when every vehicle is spoken to in MAVLink 1.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1302-1344`
    #[test]
    fn a_frame_to_a_vehicle_spoken_to_in_mavlink_1_is_framed_so() {
        use mp_mavlink::Message as _;
        use mp_mavlink_dialects::all::CommandLong;
        let mut signing = Signing::default();
        let old = VehicleId::new(1, 1);
        let new = VehicleId::new(2, 1);
        let to = |target: VehicleId| {
            MavMessage::CommandLong(CommandLong {
                param1: 1.0,
                param2: 2.0,
                param3: 0.0,
                param4: 0.0,
                param5: 0.0,
                param6: 0.0,
                param7: 0.0,
                command: 520,
                target_system: target.payload_target(),
                target_component: target.compid,
                confirmation: 0,
            })
        };
        let mut out = [0u8; mp_mavlink::MAX_FRAME_LEN];
        let frame = unsigned(255, 190, 9, &to(old));
        assert_eq!(signing.mavlink1(&frame, &mut out), None, "no vehicle heard");

        signing.heard(old, false);
        signing.heard(new, true);
        let n = signing.mavlink1(&frame, &mut out).expect("MAVLink 1");
        let mut parsed = 0;
        frames(&out[..n], |v1| {
            assert_eq!(v1.version, mp_mavlink::MavVersion::V1);
            assert_eq!((v1.seq, v1.sysid, v1.compid, v1.msgid), (9, 255, 190, CommandLong::ID));
            assert_eq!(v1.payload.len(), CommandLong::MIN_LEN);
            assert_eq!(MavMessage::decode(v1.msgid, v1.payload), Some(to(old)));
            assert!(!v1.is_signed());
            parsed += 1;
        });
        assert_eq!(parsed, 1);

        assert_eq!(signing.mavlink1(&unsigned(255, 190, 0, &to(new)), &mut out), None);
        let stranger = VehicleId::new(9, 1);
        assert_eq!(signing.mavlink1(&unsigned(255, 190, 0, &to(stranger)), &mut out), None);
        // A component never heard: its system's first.
        let gimbal = VehicleId::new(1, 154);
        assert!(signing.mavlink1(&unsigned(255, 190, 0, &to(gimbal)), &mut out).is_some());
        // The heartbeat names no vehicle: MAVLink 2 while one is spoken to in it.
        let heartbeat = unsigned(255, 190, 0, &gcs_heartbeat());
        assert_eq!(signing.mavlink1(&heartbeat, &mut out), None);
        signing.set_mavlinkv2(new, false);
        assert!(signing.mavlink1(&heartbeat, &mut out).is_some());
        // A wide id: MAVLink 2 whatever.
        let wide = unsigned(0x0001_0000, 190, 0, &to(old));
        assert_eq!(signing.mavlink1(&wide, &mut out), None);
        // An id past a byte, to the MAVLink 1 vehicle itself: SETUP_SIGNING, 256 (the dialect has
        // no PROTOCOL_VERSION).
        let mut payload = [0u8; 42];
        payload[8] = u8::try_from(old.sysid).expect("a byte");
        payload[9] = old.compid;
        let big = MavMessage::decode(256, &payload).expect("SETUP_SIGNING");
        assert_eq!(signing.mavlink1(&unsigned(255, 190, 0, &big), &mut out), None);
    }

    /// Polls until `check` holds, failing rather than hanging.
    fn until(what: &str, check: impl Fn() -> bool) {
        let deadline = web_time::Instant::now() + Duration::from_secs(5);
        while !check() {
            assert!(
                web_time::Instant::now() < deadline,
                "timed out waiting for {what}"
            );
            wasm_thread::sleep(Duration::from_millis(1));
        }
    }

    /// The next `count` frames the vehicle's end is sent that are this test's own - the
    /// `SETUP_SIGNING`s and the `COMMAND_LONG`s for command 520 - skipping anything the link
    /// sends of its own accord.
    fn sent_to(vehicle: &mut mp_transport::testing::LoopbackEnd, count: usize) -> Vec<Vec<u8>> {
        use mp_mavlink::Message as _;
        use mp_transport::Transport as _;
        let deadline = web_time::Instant::now() + Duration::from_secs(5);
        let mut bytes = Vec::new();
        let mut buf = [0u8; 4096];
        loop {
            let n = vehicle.read(&mut buf).unwrap_or(0);
            bytes.extend_from_slice(&buf[..n]);
            let mut out = Vec::new();
            frames(&bytes, |frame| {
                let ours = frame.msgid == SetupSigning::ID
                    || matches!(
                        MavMessage::decode(frame.msgid, frame.payload),
                        Some(MavMessage::CommandLong(m)) if m.command == 520
                    );
                if ours {
                    out.push(frame.raw.to_vec());
                }
            });
            if out.len() >= count {
                return out;
            }
            assert!(
                web_time::Instant::now() < deadline,
                "timed out waiting for {count} frames"
            );
            wasm_thread::sleep(Duration::from_millis(1));
        }
    }

    /// The whole path, over a real link: a heartbeat signed with a key in no store is dropped;
    /// one signed with a key the store has is heard and teaches the link that key; what the link
    /// sends that vehicle afterwards is signed with it, under the vehicle's `sendlinkid`; and a
    /// `setupSigning` with an empty seed sends its two `SETUP_SIGNING`s - still signed, the switch
    /// coming after them - and then nothing more is signed. The only test that touches the store.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1381-1432, 1526-1581, 5035-5065`
    #[test]
    fn a_link_adapts_to_a_signing_vehicle_and_signs_to_it() {
        use mp_mavlink::Message as _;
        use mp_mavlink_dialects::all::CommandLong;
        use mp_transport::Transport as _;
        use mp_transport::testing::Loopback;

        let key = *SigningKey::from_passphrase("a_link_adapts_to_a_signing_vehicle").as_bytes();
        let mut keys = auth_keys();
        keys.push(key);
        set_auth_keys(keys);
        assert!(auth_keys().contains(&key));

        let (mut vehicle, gcs) = Loopback::pair();
        let link = crate::Link::from_transport(
            Box::new(gcs),
            crate::LinkConfig {
                send_heartbeat: false,
                stream_rate_hz: 0,
                ..crate::LinkConfig::default()
            },
        );
        let sender = link.sender();
        let id = VehicleId::new(1, 1);

        let stranger = sign_frame(&crate::testing::heartbeat(0), &[7; 32], 0, TIMESTAMP).unwrap();
        vehicle.write_all(&stranger).unwrap();
        let known = sign_frame(&crate::testing::heartbeat(1), &key, 0, TIMESTAMP + 1).unwrap();
        vehicle.write_all(&known).unwrap();
        until("the vehicle to be heard", || link.vehicles().contains(&id));
        until("its key to be adopted", || {
            sender
                .signing(id)
                .is_some_and(|state| state.key == Some(key))
        });
        until("its state published", || {
            link.vehicle(id)
                .is_some_and(|handle| handle.load().messages_applied >= 1)
        });
        let state = link.vehicle(id).unwrap().load();
        assert_eq!(
            state.messages_applied, 1,
            "the stranger's heartbeat was dropped"
        );
        let adopted = sender.signing(id).unwrap();
        assert!(adopted.signing);

        let command = MavMessage::CommandLong(CommandLong {
            target_system: 1,
            target_component: 1,
            command: 520,
            confirmation: 0,
            param1: 1.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
        });
        assert!(sender.send(&command));
        let sent = sent_to(&mut vehicle, 1);
        assert_eq!(sent.len(), 1);
        frames(&sent[0], |frame| {
            assert!(frame.is_signed());
            assert!(mp_mavlink::verify(&SigningKey::new(key), frame));
            assert_eq!(frame.signature.unwrap()[0], adopted.send_link_id);
        });

        assert!(!sender.setup_signing(id, "", None));
        until("signing off", || {
            sender.signing(id).is_some_and(|state| !state.signing)
        });
        assert_eq!(sender.signing(id).unwrap().key, None);
        assert!(sender.send(&command));
        let sent = sent_to(&mut vehicle, 3);
        assert_eq!(sent.len(), 3);
        for (index, raw) in sent.iter().enumerate() {
            frames(raw, |frame| {
                let setup = frame.msgid == SetupSigning::ID;
                assert_eq!(setup, index < 2, "two SETUP_SIGNING, then the command");
                assert_eq!(frame.is_signed(), setup, "signed until the switch");
                if setup {
                    assert!(mp_mavlink::verify(&SigningKey::new(key), frame));
                    let Some(MavMessage::SetupSigning(m)) =
                        MavMessage::decode(frame.msgid, frame.payload)
                    else {
                        panic!("SETUP_SIGNING");
                    };
                    assert_eq!((m.secret_key, m.initial_timestamp), ([0; 32], 0));
                }
            });
        }
    }
}
