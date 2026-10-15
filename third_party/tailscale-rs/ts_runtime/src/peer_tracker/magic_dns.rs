// MissionPlannerRust: this whole file is MissionPlannerRust's addition to tailscale-rs 0.6.1
// (third_party/tailscale-rs/README.md), under tailscale-rs's own BSD-3-Clause licence (LICENSE).

//! MagicDNS names, as Go's tsnet resolves them when it dials a name: no DNS server, but a table
//! of the network map's node names in memory - each node's full name, lowercase and without its
//! trailing dot, and its short name too when it is in this node's own tailnet; this node itself
//! included.
//!
//! Ported from Tailscale's Go client (v1.104.0): `ipn/ipnlocal/node_backend.go` (the table),
//! `ipn/ipnlocal/resolve.go` (the address a name dials), `net/tsdial/dnsmap.go` (the lookup key),
//! `types/netmap/netmap.go` (the tailnet's suffix) and `util/dnsname/dnsname.go` (suffixes). Go
//! also answers `DNS.ExtraRecords` (service addresses); the network map tailscale-rs parses does
//! not carry them, so neither does this.
//!
//! Plain functions over names and addresses, std only, so MissionPlannerRust's workspace tests
//! them where they stand: `crates/mp-tailnet` compiles this file into its own tests (the
//! vendored crates are outside that workspace, and this crate's other tests need dev-dependencies
//! its published manifest does not list).

use std::collections::HashMap;
use std::net::IpAddr;

/// `canonMapKey`: a name as a MagicDNS lookup key - lowercase, one trailing dot removed.
/// Go: net/tsdial/dnsmap.go:14-18
#[must_use]
pub fn canon_map_key(name: &str) -> String {
    name.strip_suffix('.').unwrap_or(name).to_lowercase()
}

/// `MagicDNSSuffixOfNodeName`: the tailnet's MagicDNS suffix from a node's name - everything
/// after its first label, with no dot at either end; a name of one label is its own suffix.
/// Go: types/netmap/netmap.go:256-262
#[must_use]
pub fn magic_dns_suffix_of_node_name(node_name: &str) -> &str {
    let name = node_name.trim_matches('.');
    match name.split_once('.') {
        Some((_, rest)) => rest,
        None => name,
    }
}

/// `dnsname.HasSuffix`: whether `name` ends in `suffix` at a label boundary, a trailing dot on
/// either aside. An empty suffix is no suffix.
/// Go: util/dnsname/dnsname.go:207-213
#[must_use]
pub fn has_suffix(name: &str, suffix: &str) -> bool {
    let name = name.strip_suffix('.').unwrap_or(name);
    let suffix = suffix.strip_suffix('.').unwrap_or(suffix);
    let suffix = suffix.strip_prefix('.').unwrap_or(suffix);
    let base = name.strip_suffix(suffix).unwrap_or(name);
    base.len() < name.len() && base.ends_with('.')
}

/// `dnsname.TrimSuffix`: `name` without `suffix`, if it ends in it, and without a trailing dot.
/// Go: util/dnsname/dnsname.go:218-225
#[must_use]
pub fn trim_suffix<'a>(name: &'a str, suffix: &str) -> &'a str {
    let mut name = name;
    if has_suffix(name, suffix) {
        name = name.strip_suffix('.').unwrap_or(name);
        name = name.strip_suffix(suffix.trim_matches('.')).unwrap_or(name);
    }
    name.strip_suffix('.').unwrap_or(name)
}

/// Go's `nodeBackend.nodeByName`: every MagicDNS name of this node and its peers, to what it
/// names.
#[derive(Debug, Clone)]
pub struct MagicDnsNames<T> {
    by_name: HashMap<String, T>,
}

impl<T> Default for MagicDnsNames<T> {
    fn default() -> Self {
        Self {
            by_name: HashMap::new(),
        }
    }
}

impl<T: Copy> MagicDnsNames<T> {
    /// `updateNodeByNameLocked`: the table from this node's name and its peers', each a node's
    /// full name (`white.tail1234.ts.net`, with or without the trailing dot) and what it names.
    /// The suffix that makes a short name is this node's tailnet's, so with no node there are
    /// only full names. This node goes in first and a peer of the same name after it, as in Go.
    /// The suffix is compared lowercase, as the names are; the control server's are lowercase.
    /// Go: ipn/ipnlocal/node_backend.go:805-820; types/netmap/netmap.go:268-270
    pub fn build(
        this_node: Option<(String, T)>,
        peers: impl IntoIterator<Item = (String, T)>,
    ) -> Self {
        let mut table = Self::default();
        let suffix = this_node
            .as_ref()
            .map(|(name, _)| magic_dns_suffix_of_node_name(name).to_lowercase())
            .unwrap_or_default();
        if let Some((name, target)) = &this_node {
            table.add(name, &suffix, *target);
        }
        for (name, target) in peers {
            table.add(&name, &suffix, target);
        }
        table
    }

    /// `addNodeNameLocked`: a node's full name, and its short name when its full name is under
    /// this node's tailnet's suffix. A node with no name has neither.
    /// Go: ipn/ipnlocal/node_backend.go:776-788
    fn add(&mut self, name: &str, suffix: &str, target: T) {
        if name.is_empty() {
            return;
        }
        let canon = canon_map_key(name);
        let short = has_suffix(&canon, suffix).then(|| trim_suffix(&canon, suffix).to_owned());
        self.by_name.insert(canon, target);
        if let Some(short) = short {
            self.by_name.insert(short, target);
        }
    }

    /// `NodeByName`: what a name names. The name must be a lookup key already - lowercase, no
    /// trailing dot ([`canon_map_key`]); a short name and a full name are both looked up.
    /// Go: ipn/ipnlocal/node_backend.go:836-844
    #[must_use]
    pub fn get(&self, name: &str) -> Option<T> {
        self.by_name.get(name).copied()
    }
}

/// `nodeAddrForNetwork`: which of a node's addresses a dial over `network` ("tcp", "tcp4",
/// "tcp6", "udp", "udp4", "udp6") uses - the first of the family a "4" or "6" asks for; for a
/// network that names no family the first IPv4 address, else the first IPv6 one.
/// Go: ipn/ipnlocal/resolve.go:112-144
#[must_use]
pub fn node_addr_for_network(addresses: &[IpAddr], network: &str) -> Option<IpAddr> {
    let want4 = network.ends_with('4');
    let want6 = network.ends_with('6');
    let mut v6 = None;
    for &ip in addresses {
        if want4 && ip.is_ipv4() {
            return Some(ip);
        }
        if want6 && ip.is_ipv6() {
            return Some(ip);
        }
        if !want4 && !want6 {
            if ip.is_ipv4() {
                return Some(ip);
            }
            if v6.is_none() {
                v6 = Some(ip);
            }
        }
    }
    v6
}

#[cfg(test)]
mod tests {
    use std::net::{Ipv4Addr, Ipv6Addr};

    use super::*;

    /// The key: lowercase, and one trailing dot gone - only one, as Go's `TrimSuffix`.
    #[test]
    fn a_key_is_lowercase_without_its_trailing_dot() {
        assert_eq!(canon_map_key("White.Tail1234.TS.NET."), "white.tail1234.ts.net");
        assert_eq!(canon_map_key("buzz55"), "buzz55");
        assert_eq!(canon_map_key("BUZZ55."), "buzz55");
        assert_eq!(canon_map_key("white.."), "white.");
        assert_eq!(canon_map_key(""), "");
    }

    /// The tailnet's suffix is the node's name after its first label, dots trimmed; a name of one
    /// label is its own.
    #[test]
    fn the_suffix_is_the_name_after_its_first_label() {
        assert_eq!(magic_dns_suffix_of_node_name("mpr-host.tail1234.ts.net."), "tail1234.ts.net");
        assert_eq!(magic_dns_suffix_of_node_name("mpr-host.tail1234.ts.net"), "tail1234.ts.net");
        assert_eq!(magic_dns_suffix_of_node_name("white"), "white");
        assert_eq!(magic_dns_suffix_of_node_name(".white.example."), "example");
        assert_eq!(magic_dns_suffix_of_node_name(""), "");
    }

    /// A suffix counts at a label boundary only, with something before it; trailing dots aside;
    /// an empty suffix never.
    #[test]
    fn a_suffix_is_whole_labels() {
        assert!(has_suffix("white.tail1234.ts.net", "tail1234.ts.net"));
        assert!(has_suffix("white.tail1234.ts.net.", "tail1234.ts.net."));
        assert!(has_suffix("white.tail1234.ts.net", ".tail1234.ts.net"));
        assert!(!has_suffix("whitetail1234.ts.net", "tail1234.ts.net"));
        assert!(!has_suffix("tail1234.ts.net", "tail1234.ts.net"));
        assert!(!has_suffix("white.other.ts.net", "tail1234.ts.net"));
        assert!(!has_suffix("white", ""));
        assert_eq!(trim_suffix("white.tail1234.ts.net", "tail1234.ts.net"), "white");
        assert_eq!(trim_suffix("white.tail1234.ts.net.", ".tail1234.ts.net."), "white");
        assert_eq!(trim_suffix("white.other.ts.net.", "tail1234.ts.net"), "white.other.ts.net");
    }

    /// What the table's entries name in the tests.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum Named {
        ThisNode,
        Peer(u32),
    }

    fn tailnet() -> MagicDnsNames<Named> {
        MagicDnsNames::build(
            Some(("mpr-host.tail1234.ts.net".to_owned(), Named::ThisNode)),
            [
                ("white.tail1234.ts.net".to_owned(), Named::Peer(1)),
                ("Buzz55.Tail1234.ts.net.".to_owned(), Named::Peer(2)),
                // Shared into this tailnet from another: its full name only.
                ("white.other-net.ts.net".to_owned(), Named::Peer(3)),
                ("".to_owned(), Named::Peer(4)),
            ],
        )
    }

    /// Full names always; short names for this tailnet's nodes only, so the shared-in `white`
    /// does not take the short name from this tailnet's; this node itself by both names; keys
    /// lowercase, so a name control sent in capitals is found by its lowercase key.
    #[test]
    fn short_names_are_this_tailnets_and_full_names_everyones() {
        let table = tailnet();
        assert_eq!(table.get("white"), Some(Named::Peer(1)));
        assert_eq!(table.get("white.tail1234.ts.net"), Some(Named::Peer(1)));
        assert_eq!(table.get("white.other-net.ts.net"), Some(Named::Peer(3)));
        assert_eq!(table.get("buzz55"), Some(Named::Peer(2)));
        assert_eq!(table.get("buzz55.tail1234.ts.net"), Some(Named::Peer(2)));
        assert_eq!(table.get("mpr-host"), Some(Named::ThisNode));
        assert_eq!(table.get("mpr-host.tail1234.ts.net"), Some(Named::ThisNode));
        assert_eq!(table.get("other-net"), None);
        assert_eq!(table.get("ts.net"), None);
        assert_eq!(table.get(""), None, "a node with no name has none");
        assert_eq!(table.get("Buzz55"), None, "a key is lowercase");
        assert_eq!(table.get(&canon_map_key("Buzz55.")), Some(Named::Peer(2)));
        assert_eq!(table.get("unknown"), None);
    }

    /// Without this node there is no suffix, so only full names.
    #[test]
    fn without_this_node_only_full_names() {
        let table = MagicDnsNames::build(None, [("white.tail1234.ts.net".to_owned(), 1_u32)]);
        assert_eq!(table.get("white.tail1234.ts.net"), Some(1));
        assert_eq!(table.get("white"), None);
    }

    /// A peer named as this node is goes in after it, so it is the one found, as in Go.
    #[test]
    fn a_peer_of_this_nodes_name_comes_after_it() {
        let table = MagicDnsNames::build(
            Some(("twin.tail1234.ts.net".to_owned(), Named::ThisNode)),
            [("twin.tail1234.ts.net".to_owned(), Named::Peer(9))],
        );
        assert_eq!(table.get("twin"), Some(Named::Peer(9)));
    }

    /// "tcp" and "udp" take the first IPv4 address wherever it is, else the first IPv6; "4" and
    /// "6" their own family or nothing.
    #[test]
    fn ipv4_first_unless_the_network_says() {
        let v4 = IpAddr::V4(Ipv4Addr::new(100, 101, 102, 103));
        let v6 = IpAddr::V6(Ipv6Addr::new(0xfd7a, 0x115c, 0xa1e0, 0, 0, 0, 0, 1));
        assert_eq!(node_addr_for_network(&[v4, v6], "tcp"), Some(v4));
        assert_eq!(node_addr_for_network(&[v6, v4], "tcp"), Some(v4));
        assert_eq!(node_addr_for_network(&[v6, v4], "udp"), Some(v4));
        assert_eq!(node_addr_for_network(&[v6], "tcp"), Some(v6));
        assert_eq!(node_addr_for_network(&[v4, v6], "tcp6"), Some(v6));
        assert_eq!(node_addr_for_network(&[v6, v4], "udp4"), Some(v4));
        assert_eq!(node_addr_for_network(&[v4], "tcp6"), None);
        assert_eq!(node_addr_for_network(&[v6], "udp4"), None);
        assert_eq!(node_addr_for_network(&[], "tcp"), None);
    }
}
