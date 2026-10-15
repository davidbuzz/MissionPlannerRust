//! Peer delta update tracking.

use std::{collections::HashSet, net::IpAddr, sync::Arc};

use kameo::{
    actor::ActorRef,
    message::{Context, Message},
    reply::ReplySender,
};
use ts_control::Node;
use ts_transport::PeerId;

use crate::{Error, env::Env};

// MissionPlannerRust: the MagicDNS name table, as Go's tsnet resolves a name it dials
// (ipn/ipnlocal/node_backend.go:776-852, ipn/ipnlocal/resolve.go:54-144).
pub mod magic_dns;
mod peer_db;

pub use peer_db::PeerDb;

// MissionPlannerRust: how many MagicDNS lookups may wait for this node's first network map. A
// caller that gives up (`tailscale::Device::resolve`'s timeout) leaves its request queued, so the
// queue is bounded: past this the oldest is answered "no such name".
const MAX_PENDING_MAGIC_DNS: usize = 64;

/// Actor that tracks peer delta updates and emits new states.
pub struct PeerTracker {
    peer_db: PeerDb,
    seen_state_update: bool,
    pending_requests: Vec<Pending>,
    env: Env,
    // MissionPlannerRust: this node, from the network map - in the MagicDNS table, and its name
    // gives the tailnet's MagicDNS suffix (Go: ipn/ipnlocal/node_backend.go:785, 814-816).
    self_node: Option<Node>,
    // MissionPlannerRust: Go's `nodeByName`, rebuilt from `self_node` and `peer_db` whenever
    // either changes (Go: ipn/ipnlocal/node_backend.go:805-820).
    magic_dns: magic_dns::MagicDnsNames<MagicDnsTarget>,
    // MissionPlannerRust: lookups made before the first network map named this node, answered
    // when it does.
    pending_magic_dns: Vec<(String, ReplySender<Option<Node>>)>,
}

// MissionPlannerRust: what a MagicDNS name names - this node, or a peer in `peer_db`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum MagicDnsTarget {
    SelfNode,
    Peer(PeerId),
}

impl PeerTracker {
    fn peer_by_name_opt(&self, name: &str) -> Option<&Node> {
        let name = name.trim_end_matches('.');
        self.peer_db.get(&name).map(|(_id, node)| node)
    }

    fn peer_by_tailnet_ip_opt(&self, ip: IpAddr) -> Option<&Node> {
        self.peer_db.get(&ip).map(|(_id, node)| node)
    }

    // MissionPlannerRust: `updateNodeByNameLocked` - the table anew from this node and every
    // peer, by their full names (`Node::fqdn` without the trailing dot, Go's `Name()`).
    // Go: ipn/ipnlocal/node_backend.go:805-820
    fn rebuild_magic_dns(&mut self) {
        let this_node = self
            .self_node
            .as_ref()
            .map(|node| (node.fqdn(false), MagicDnsTarget::SelfNode));
        let peers = self
            .peer_db
            .peers()
            .iter()
            .map(|(id, node)| (node.fqdn(false), MagicDnsTarget::Peer(*id)));
        self.magic_dns = magic_dns::MagicDnsNames::build(this_node, peers);
    }

    // MissionPlannerRust: `NodeByName` then `NodeByID` - the node a lookup key names.
    // Go: ipn/ipnlocal/resolve.go:60-72
    fn magic_dns_opt(&self, name: &str) -> Option<Node> {
        match self.magic_dns.get(name)? {
            MagicDnsTarget::SelfNode => self.self_node.clone(),
            MagicDnsTarget::Peer(id) => self.peer_db.get(&id).map(|(_id, node)| node.clone()),
        }
    }

    // MissionPlannerRust: the table after a network map changed this node or its peers, and the
    // lookups that waited for this node answered from it.
    fn update_magic_dns(&mut self) {
        self.rebuild_magic_dns();
        if self.self_node.is_none() {
            return;
        }
        for (name, reply) in core::mem::take(&mut self.pending_magic_dns) {
            reply.send(self.magic_dns_opt(&name));
        }
    }
}

impl kameo::Actor for PeerTracker {
    type Args = Env;
    type Error = Error;

    async fn on_start(env: Self::Args, slf: ActorRef<Self>) -> Result<Self, Self::Error> {
        env.subscribe::<Arc<ts_control::StateUpdate>>(&slf).await?;
        env.register(None, &slf).await?;

        Ok(Self {
            peer_db: PeerDb::default(),
            pending_requests: Default::default(),
            seen_state_update: false,
            env,
            // MissionPlannerRust: no network map yet, so no table.
            self_node: None,
            magic_dns: Default::default(),
            pending_magic_dns: Default::default(),
        })
    }
}

enum Pending {
    PeerByName(PeerByName, ReplySender<Option<Node>>),
    AcceptedRoute(PeerByAcceptedRoute, ReplySender<Vec<Node>>),
    TailnetIp(PeerByTailnetIp, ReplySender<Option<Node>>),
}

// For messages with arguments, a struct is generated with the args as fields. They aren't
// documented, and we can't apply attributes directly to the fields. Hence, wrap in a module where
// docs are turned off everywhere.
#[allow(missing_docs)]
mod msg_impl {
    use std::net::IpAddr;

    use kameo::prelude::DelegatedReply;

    use super::*;

    #[kameo::messages]
    impl PeerTracker {
        /// Lookup a peer by name.
        ///
        /// Waits until we've received at least one peer update from control.
        #[message(ctx)]
        pub async fn peer_by_name(
            &mut self,
            ctx: &mut Context<Self, DelegatedReply<Option<Node>>>,
            name: String,
        ) -> DelegatedReply<Option<Node>> {
            let (deleg, sender) = ctx.reply_sender();
            let Some(sender) = sender else { return deleg };

            if !self.seen_state_update {
                tracing::debug!(query = name, "no peer state seen yet, queueing request");

                self.pending_requests
                    .push(Pending::PeerByName(PeerByName { name }, sender));

                return deleg;
            }

            sender.send(self.peer_by_name_opt(&name).cloned());

            deleg
        }

        /// Lookup all peers that accept packets addressed to the given IP.
        ///
        /// This includes the peer's tailnet address and any subnet routes it provides. Only
        /// the peers with the most specific subnet route match that covers `ip` will be
        /// returned.
        ///
        /// E.g., suppose:
        ///
        /// - We're querying for `10.1.2.3`
        /// - `PeerA` and `PeerB` have accepted routes for `10.1.2.0/24`
        /// - `PeerC` has an accepted route for `10.1.0.0/16`
        ///
        /// Only `PeerA` and `PeerB` will be returned, since they have the most specific
        /// prefix match.
        #[message(ctx)]
        pub fn peer_by_accepted_route(
            &mut self,
            ctx: &mut Context<Self, DelegatedReply<Vec<Node>>>,
            ip: IpAddr,
        ) -> DelegatedReply<Vec<Node>> {
            let (deleg, sender) = ctx.reply_sender();
            let Some(sender) = sender else { return deleg };

            if !self.seen_state_update {
                tracing::debug!(query = %ip, "no peer state seen yet, queueing request");

                self.pending_requests
                    .push(Pending::AcceptedRoute(PeerByAcceptedRoute { ip }, sender));

                return deleg;
            }

            sender.send(
                self.peer_db
                    .get_route(ip.into())
                    .map(|(_id, node)| node.clone())
                    .collect(),
            );

            deleg
        }

        /// Lookup the peer that has the given tailnet IP address.
        #[message(ctx)]
        pub fn peer_by_tailnet_ip(
            &mut self,
            ctx: &mut Context<Self, DelegatedReply<Option<Node>>>,
            ip: IpAddr,
        ) -> DelegatedReply<Option<Node>> {
            let (deleg, sender) = ctx.reply_sender();
            let Some(sender) = sender else { return deleg };

            if !self.seen_state_update {
                tracing::debug!(query = %ip, "no peer state seen yet, queueing request");

                self.pending_requests
                    .push(Pending::TailnetIp(PeerByTailnetIp { ip }, sender));

                return deleg;
            }

            sender.send(self.peer_by_tailnet_ip_opt(ip).cloned());

            deleg
        }

        // MissionPlannerRust: the MagicDNS lookup Go's tsnet makes when it dials a name.
        /// Look up the node a MagicDNS name names: this node or a peer, by its full name or, for
        /// a node in this node's own tailnet, its short name.
        ///
        /// `name` must be a lookup key: lowercase, without a trailing dot
        /// (`magic_dns::canon_map_key`). Waits until the network map has named this node; from
        /// then on answers at once, `None` for a name the table does not hold.
        ///
        /// Go: ipn/ipnlocal/resolve.go:60-72, net/tsdial/tsdial.go:409-413
        #[message(ctx)]
        pub fn resolve_magic_dns(
            &mut self,
            ctx: &mut Context<Self, DelegatedReply<Option<Node>>>,
            name: String,
        ) -> DelegatedReply<Option<Node>> {
            let (deleg, sender) = ctx.reply_sender();
            let Some(sender) = sender else { return deleg };

            if self.self_node.is_none() {
                tracing::debug!(query = name, "no network map yet, queueing magicdns lookup");

                if self.pending_magic_dns.len() >= MAX_PENDING_MAGIC_DNS {
                    let (_name, oldest) = self.pending_magic_dns.remove(0);
                    oldest.send(None);
                }
                self.pending_magic_dns.push((name, sender));

                return deleg;
            }

            sender.send(self.magic_dns_opt(&name));

            deleg
        }
    }
}

pub use msg_impl::*;

#[derive(Debug, Clone, Default)]
pub(crate) struct PeerState {
    #[allow(unused)]
    pub deletions: HashSet<PeerId>,
    #[allow(unused)]
    pub upserts: HashSet<PeerId>,
    pub peers: Arc<PeerDb>,
}

impl Message<Arc<ts_control::StateUpdate>> for PeerTracker {
    type Reply = ();

    async fn handle(
        &mut self,
        msg: Arc<ts_control::StateUpdate>,
        _ctx: &mut Context<Self, Self::Reply>,
    ) {
        // MissionPlannerRust: this node, as the network map names it, for the MagicDNS table.
        // Go: ipn/ipnlocal/node_backend.go:814-816
        if let Some(node) = &msg.node {
            self.self_node = Some(node.clone());
        }

        let Some(peer_update) = &msg.peer_update else {
            // MissionPlannerRust: a network map with no peers in it - the first one of a tailnet
            // with no other node is one - still names this node, which changes the table.
            if msg.node.is_some() {
                self.update_magic_dns();
            }
            return;
        };

        let mut upserts = HashSet::default();
        let mut deletions = HashSet::default();

        match peer_update {
            ts_control::PeerUpdate::Full(new_nodes) => {
                tracing::trace!("full peer update");

                let new_ids = new_nodes
                    .iter()
                    .map(|x| &x.stable_id)
                    .collect::<HashSet<_>>();

                self.peer_db.retain(|id, peer| {
                    let retain = new_ids.contains(&peer.stable_id);

                    if !retain {
                        deletions.insert(id);
                    }

                    retain
                });

                for node in new_nodes {
                    let peer_id = self.peer_db.upsert(node);
                    upserts.insert(peer_id);
                }
            }

            ts_control::PeerUpdate::Delta {
                patch,
                remove,
                upsert,
            } => {
                tracing::trace!("delta peer update");

                for peer in remove {
                    let Some((id, _node)) = self.peer_db.remove(peer) else {
                        tracing::error!(control_node_id = peer, "removed peer was unknown");
                        continue;
                    };

                    deletions.insert(id);
                }

                for peer in upsert {
                    let id = self.peer_db.upsert(peer);

                    upserts.insert(id);
                }

                for update in patch {
                    if let Some(id) = self.peer_db.patch(update) {
                        upserts.insert(id);
                    } else {
                        tracing::warn!(?update, "no peer for update");
                    }
                }
            }
        }

        tracing::debug!(
            n_upsert = upserts.len(),
            n_delete = deletions.len(),
            peer_count = self.peer_db.peers().len(),
            "new peer state"
        );

        self.service_pending_requests();

        // MissionPlannerRust: the MagicDNS table follows the peers.
        self.update_magic_dns();

        if let Err(e) = self
            .env
            .publish(Arc::new(PeerState {
                upserts,
                deletions,
                peers: Arc::new(self.peer_db.clone()),
            }))
            .await
        {
            tracing::error!(error = %e, "publishing peer state update");
        }
    }
}

impl PeerTracker {
    fn service_pending_requests(&mut self) {
        if self.seen_state_update {
            return;
        }

        self.seen_state_update = true;

        if !self.pending_requests.is_empty() {
            tracing::debug!(
                n_pending = self.pending_requests.len(),
                "state update received, servicing pending requests"
            );
        }

        for req in core::mem::take(&mut self.pending_requests) {
            match req {
                Pending::PeerByName(PeerByName { name }, reply) => {
                    reply.send(self.peer_by_name_opt(&name).cloned());
                }
                Pending::TailnetIp(PeerByTailnetIp { ip }, reply) => {
                    reply.send(self.peer_by_tailnet_ip_opt(ip).cloned());
                }
                Pending::AcceptedRoute(PeerByAcceptedRoute { ip }, reply) => {
                    reply.send(
                        self.peer_db
                            .get_route(ip.into())
                            .map(|(_id, node)| node.clone())
                            .collect(),
                    );
                }
            }
        }
    }
}
