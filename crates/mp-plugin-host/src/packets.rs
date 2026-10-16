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

//! `SubscribeToPacketType` and `PacketReceived`: the packets a plugin asked for, handed to it.
//!
//! The C#'s plugin subscribes a function on `Host.comPort`, and the link's reader calls it for
//! each packet that matches, on the reader's thread, after the packet is handled
//! (`// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5346-5347, 5508-5577`). Here a plugin's
//! `subscribe-packet` goes into a table its host keeps ([`Subscriptions`]), at once, from the
//! plugin's own thread; the application hands each packet its link reads to the host's
//! [`PacketSink`] on the link's thread; and each subscription the packet matches sends it to its
//! plugin's thread, where its `packet-received` is called. The link never waits on a plugin, so
//! one that is slow holds up only itself; and a plugin that traps in its handler is unloaded,
//! where the C# logs the exception and calls it again with the next packet.
//!
//! The world carries no MAVLink: the application hands the sink a packet's id, sender and
//! payload bytes, which it makes only for a packet some subscription matches.

use mp_os::ReadWrite as _;
use std::sync::mpsc::Sender;
use std::sync::{Arc, PoisonError, RwLock};

use crate::channel::Snapshot;
use crate::runner::Command;

/// One subscription: the C#'s `(msgId, function, exclusive, sysid, compid)` tuple, the function
/// being its plugin's `packet-received`. `exclusive`, which no plugin here passes, is not kept.
/// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5537-5577`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Subscription {
    /// The plugin's index in [`crate::PluginHost::plugins`].
    pub plugin: usize,
    /// `msgId`.
    pub message_id: u32,
    /// `sysid`: 0, with `compid` 0, for the vehicle the link talks to.
    pub sysid: u32,
    /// `compid`.
    pub compid: u8,
}

impl Subscription {
    /// `PacketReceived`'s rule: the message, and either the sender exactly, or - for a
    /// subscription to system 0 component 0 - the vehicle the link talks to, `current` (none
    /// before one is heard).
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5520-5523`
    #[must_use]
    pub fn matches(
        &self,
        message_id: u32,
        sysid: u32,
        compid: u8,
        current: Option<(u32, u8)>,
    ) -> bool {
        self.message_id == message_id
            && ((sysid == self.sysid && compid == self.compid)
                || (self.sysid == 0 && self.compid == 0 && current == Some((sysid, compid))))
    }
}

/// Every plugin's subscriptions, in the order they were made: `MAVLinkInterface.Subscriptions`,
/// shared by the plugins' surfaces, which add to it on their threads, and the host's
/// [`PacketSink`], which reads it on the link's.
#[derive(Debug, Clone, Default)]
pub struct Subscriptions(Arc<RwLock<Vec<Subscription>>>);

impl Subscriptions {
    /// `SubscribeToPacketType`: `Subscriptions.Add(item)`. The same subscription made twice is
    /// held twice, and its plugin handed each packet twice, as the C#'s list holds the tuple
    /// twice. `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5547-5573`
    pub fn add(&self, subscription: Subscription) {
        self.0
            .os_write()
            .unwrap_or_else(PoisonError::into_inner)
            .push(subscription);
    }

    /// Every subscription, in the order made.
    #[must_use]
    pub fn list(&self) -> Vec<Subscription> {
        self.0
            .os_read()
            .unwrap_or_else(PoisonError::into_inner)
            .clone()
    }

    /// Whether any plugin has subscribed to anything.
    #[must_use]
    pub fn any(&self) -> bool {
        !self
            .0
            .os_read()
            .unwrap_or_else(PoisonError::into_inner)
            .is_empty()
    }

    /// Whether any subscription is to message `message_id`: asked of every packet the link
    /// reads, so it neither copies nor allocates.
    fn wants(&self, message_id: u32) -> bool {
        self.0
            .os_read()
            .unwrap_or_else(PoisonError::into_inner)
            .iter()
            .any(|subscription| subscription.message_id == message_id)
    }

    /// The plugin of each subscription a packet matches, in the order they were made.
    fn matching(
        &self,
        message_id: u32,
        sysid: u32,
        compid: u8,
        current: Option<(u32, u8)>,
    ) -> Vec<usize> {
        self.0
            .os_read()
            .unwrap_or_else(PoisonError::into_inner)
            .iter()
            .filter(|subscription| subscription.matches(message_id, sysid, compid, current))
            .map(|subscription| subscription.plugin)
            .collect()
    }
}

/// A packet on its way to a plugin's thread.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct Delivery {
    pub(crate) message_id: u32,
    pub(crate) sysid: u32,
    pub(crate) compid: u8,
    pub(crate) payload: Vec<u8>,
}

/// Where the application hands each packet its link reads: `PacketReceived`, for every plugin
/// of a host. Cheap to clone, and sent to the link's thread.
#[derive(Debug, Clone, Default)]
pub struct PacketSink {
    subscriptions: Subscriptions,
    /// Each plugin's thread, by its index.
    plugins: Arc<[Sender<Command>]>,
    /// What the window last said, for the vehicle the link talks to.
    snapshot: Arc<RwLock<Snapshot>>,
}

impl PacketSink {
    pub(crate) const fn new(
        subscriptions: Subscriptions,
        plugins: Arc<[Sender<Command>]>,
        snapshot: Arc<RwLock<Snapshot>>,
    ) -> Self {
        Self {
            subscriptions,
            plugins,
            snapshot,
        }
    }

    /// The subscriptions it serves.
    pub(crate) const fn subscriptions(&self) -> &Subscriptions {
        &self.subscriptions
    }

    /// `PacketReceived`: a packet the link read - message `message_id` from component `compid`
    /// of system `sysid` - handed to the plugin of each subscription it matches, in the order
    /// they were made, once for each, as the C# calls each one's function. `payload` makes its
    /// bytes, once, and only when a subscription matches. Returns how many it was handed to.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:5508-5534`
    pub fn offer(
        &self,
        message_id: u32,
        sysid: u32,
        compid: u8,
        payload: impl FnOnce() -> Vec<u8>,
    ) -> usize {
        // Most packets are of no message any plugin asked for: nothing more is done for those.
        if !self.subscriptions.wants(message_id) {
            return 0;
        }
        let current = self
            .snapshot
            .os_read()
            .unwrap_or_else(PoisonError::into_inner)
            .vehicle;
        let matched = self
            .subscriptions
            .matching(message_id, sysid, compid, current);
        if matched.is_empty() {
            return 0;
        }
        let bytes = payload();
        let mut handed = 0;
        for plugin in matched {
            let delivery = Delivery {
                message_id,
                sysid,
                compid,
                payload: bytes.clone(),
            };
            // A plugin gone - unloaded, exited - is not an error: its thread no longer listens.
            if self
                .plugins
                .get(plugin)
                .is_some_and(|thread| thread.send(Command::Packet(delivery)).is_ok())
            {
                handed += 1;
            }
        }
        handed
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

    use super::*;
    use std::sync::mpsc::{Receiver, channel};

    /// `MAVLINK_MSG_ID.OPEN_DRONE_ID_ARM_STATUS`.
    const ARM_STATUS: u32 = 12918;
    /// `MAVLINK_MSG_ID.HEARTBEAT`.
    const HEARTBEAT: u32 = 0;

    fn subscription(plugin: usize, message_id: u32, sysid: u32, compid: u8) -> Subscription {
        Subscription {
            plugin,
            message_id,
            sysid,
            compid,
        }
    }

    /// A sink over `plugins` threads, each a receiver the test reads, the window having said
    /// the link talks to `vehicle`.
    fn sink(plugins: usize, vehicle: Option<(u32, u8)>) -> (PacketSink, Vec<Receiver<Command>>) {
        let (senders, receivers): (Vec<_>, Vec<_>) = (0..plugins).map(|_| channel()).unzip();
        let snapshot = Snapshot {
            vehicle,
            ..Snapshot::default()
        };
        (
            PacketSink::new(
                Subscriptions::default(),
                Arc::from(senders),
                Arc::new(RwLock::new(snapshot)),
            ),
            receivers,
        )
    }

    /// What one plugin's thread was handed, in order.
    fn handed(receiver: &Receiver<Command>) -> Vec<Delivery> {
        receiver
            .try_iter()
            .filter_map(|command| match command {
                Command::Packet(delivery) => Some(delivery),
                _ => None,
            })
            .collect()
    }

    /// The rule: the message and the sender exactly; another message, system or component is
    /// not the subscription's. `// C#: MAVLinkInterface.cs:5520-5521`
    #[test]
    fn a_subscription_is_to_its_message_and_its_sender() {
        let odid = subscription(0, ARM_STATUS, 1, 236);
        assert!(odid.matches(ARM_STATUS, 1, 236, None));
        assert!(odid.matches(ARM_STATUS, 1, 236, Some((1, 1))));
        assert!(!odid.matches(HEARTBEAT, 1, 236, None), "another message");
        assert!(!odid.matches(ARM_STATUS, 2, 236, None), "another system");
        assert!(!odid.matches(ARM_STATUS, 1, 237, None), "another component");
    }

    /// System 0 component 0 is the vehicle the link talks to, whoever that is when the packet
    /// comes - and nobody before one is heard. `// C#: MAVLinkInterface.cs:5520-5522`
    #[test]
    fn zero_zero_is_the_vehicle_the_link_talks_to() {
        let current = subscription(0, HEARTBEAT, 0, 0);
        assert!(current.matches(HEARTBEAT, 1, 1, Some((1, 1))));
        assert!(!current.matches(HEARTBEAT, 2, 1, Some((1, 1))));
        assert!(!current.matches(HEARTBEAT, 1, 1, None));
        assert!(!current.matches(ARM_STATUS, 1, 1, Some((1, 1))));
        // Exactly 0, 0 still matches as the first clause has it, though no vehicle sends so.
        assert!(current.matches(HEARTBEAT, 0, 0, None));
    }

    /// A subscribed packet goes to its plugin's thread with its id, sender and bytes; one of a
    /// message nobody asked for goes nowhere and its bytes are never made; one from another
    /// sender goes nowhere.
    #[test]
    fn a_subscribed_packet_is_delivered_and_an_unsubscribed_one_is_not() {
        let (sink, threads) = sink(2, Some((1, 1)));
        sink.subscriptions.add(subscription(1, ARM_STATUS, 1, 236));
        let made = std::cell::Cell::new(0);
        let bytes = || {
            made.set(made.get() + 1);
            vec![1, b'E', 0]
        };
        assert_eq!(sink.offer(ARM_STATUS, 1, 236, bytes), 1);
        assert_eq!(
            sink.offer(HEARTBEAT, 1, 236, || {
                made.set(made.get() + 1);
                Vec::new()
            }),
            0
        );
        assert_eq!(sink.offer(ARM_STATUS, 1, 100, || vec![9]), 0);
        assert_eq!(made.get(), 1, "bytes made for the subscribed packet alone");
        assert!(handed(&threads[0]).is_empty(), "plugin 0 asked for nothing");
        assert_eq!(
            handed(&threads[1]),
            [Delivery {
                message_id: ARM_STATUS,
                sysid: 1,
                compid: 236,
                payload: vec![1, b'E', 0],
            }]
        );
    }

    /// Every matching subscription hands the packet on, in the order made - the same one twice
    /// is two deliveries - and its bytes are made once.
    #[test]
    fn each_matching_subscription_hands_it_on() {
        let (sink, threads) = sink(2, Some((1, 236)));
        sink.subscriptions.add(subscription(0, ARM_STATUS, 1, 236));
        sink.subscriptions.add(subscription(1, ARM_STATUS, 0, 0));
        sink.subscriptions.add(subscription(0, ARM_STATUS, 1, 236));
        let made = std::cell::Cell::new(0);
        let handed_to = sink.offer(ARM_STATUS, 1, 236, || {
            made.set(made.get() + 1);
            vec![0; 51]
        });
        assert_eq!((handed_to, made.get()), (3, 1));
        assert_eq!(handed(&threads[0]).len(), 2);
        assert_eq!(handed(&threads[1]).len(), 1);
        assert_eq!(sink.subscriptions.list().len(), 3);
    }

    /// A plugin whose thread has gone is skipped, and the others still hear.
    #[test]
    fn a_plugin_gone_is_skipped() {
        let (sink, mut threads) = sink(2, None);
        sink.subscriptions.add(subscription(0, ARM_STATUS, 1, 236));
        sink.subscriptions.add(subscription(1, ARM_STATUS, 1, 236));
        sink.subscriptions.add(subscription(7, ARM_STATUS, 1, 236));
        drop(threads.remove(0));
        assert_eq!(sink.offer(ARM_STATUS, 1, 236, || vec![0]), 1);
        assert_eq!(handed(&threads[0]).len(), 1);
    }
}
