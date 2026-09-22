//! Message metadata lookup.
//!
//! Parsing a frame needs the message's `CRC_EXTRA` seed, which is a property of the *dialect*
//! (common, ardupilotmega, ...). The codec is generic over this so that the hot path never
//! depends on generated code, and so tests and fuzzers can supply tiny synthetic dialects.

/// Static description of one message, mirroring the C# `message_info` record in
/// `ExtLibs/Mavlink/Mavlink.cs`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MessageInfo {
    /// Message id.
    pub id: u32,
    /// Message name as it appears in the XML definition, e.g. `HEARTBEAT`.
    pub name: &'static str,
    /// `CRC_EXTRA` seed derived from the message's field signature.
    pub crc_extra: u8,
    /// Wire length with MAVLink2 trailing-zero truncation applied maximally.
    pub min_len: u8,
    /// Full (untruncated) payload length.
    pub len: u8,
}

/// Anything that can answer "what is the CRC seed for this message id?".
pub trait Dialect {
    /// Returns the `CRC_EXTRA` seed, or `None` if this dialect does not know the message.
    fn crc_extra(&self, msgid: u32) -> Option<u8>;

    /// Returns full metadata when available. Defaults to `None` so minimal dialects stay cheap.
    fn info(&self, _msgid: u32) -> Option<&MessageInfo> {
        None
    }

    /// Human-readable dialect name, used in diagnostics.
    fn name(&self) -> &str {
        "unnamed"
    }
}

/// A dialect backed by a sorted, generated table. Lookup is a binary search over a slice that
/// lives in `.rodata`: no allocation, no hashing, cache-friendly.
#[derive(Debug, Clone, Copy)]
pub struct StaticDialect {
    name: &'static str,
    messages: &'static [MessageInfo],
}

impl StaticDialect {
    /// Wraps a table that **must** be sorted by `id`. Generated tables are sorted by
    /// construction and `debug_assert`ed here; `xtask` re-checks it in CI.
    #[must_use]
    pub const fn new(name: &'static str, messages: &'static [MessageInfo]) -> Self {
        Self { name, messages }
    }

    /// Number of messages in the dialect.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.messages.len()
    }

    /// Whether the dialect is empty.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.messages.is_empty()
    }

    /// All messages, ordered by id.
    #[must_use]
    pub const fn messages(&self) -> &'static [MessageInfo] {
        self.messages
    }

    /// Looks a message up by name. Linear, and not for use on the hot path.
    #[must_use]
    pub fn by_name(&self, name: &str) -> Option<&'static MessageInfo> {
        self.messages.iter().find(|m| m.name == name)
    }

    #[inline]
    fn find(&self, msgid: u32) -> Option<&'static MessageInfo> {
        let idx = self.messages.binary_search_by_key(&msgid, |m| m.id).ok()?;
        self.messages.get(idx)
    }
}

impl Dialect for StaticDialect {
    #[inline]
    fn crc_extra(&self, msgid: u32) -> Option<u8> {
        self.find(msgid).map(|m| m.crc_extra)
    }

    #[inline]
    fn info(&self, msgid: u32) -> Option<&MessageInfo> {
        self.find(msgid)
    }

    fn name(&self) -> &str {
        self.name
    }
}

/// A dialect that accepts every message id with a fixed seed. Only useful for fuzzing and
/// benchmarks, where we care about framing throughput rather than message identity.
#[derive(Debug, Clone, Copy)]
pub struct AnyDialect(pub u8);

impl Dialect for AnyDialect {
    #[inline]
    fn crc_extra(&self, _msgid: u32) -> Option<u8> {
        Some(self.0)
    }

    fn name(&self) -> &str {
        "any"
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    static TABLE: &[MessageInfo] = &[
        MessageInfo {
            id: 0,
            name: "HEARTBEAT",
            crc_extra: 50,
            min_len: 9,
            len: 9,
        },
        MessageInfo {
            id: 1,
            name: "SYS_STATUS",
            crc_extra: 124,
            min_len: 31,
            len: 31,
        },
        MessageInfo {
            id: 30,
            name: "ATTITUDE",
            crc_extra: 39,
            min_len: 28,
            len: 28,
        },
    ];

    #[test]
    fn lookup_hit_and_miss() {
        let d = StaticDialect::new("test", TABLE);
        assert_eq!(d.crc_extra(0), Some(50));
        assert_eq!(d.crc_extra(30), Some(39));
        assert_eq!(d.crc_extra(31), None);
        assert_eq!(d.info(1).map(|m| m.name), Some("SYS_STATUS"));
        assert_eq!(d.by_name("ATTITUDE").map(|m| m.id), Some(30));
    }
}
