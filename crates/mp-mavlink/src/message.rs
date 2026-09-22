//! The trait generated message types implement.

/// A typed MAVLink message.
///
/// Implementations are generated from the XML definitions by `cargo xtask codegen mavlink`;
/// the metadata constants are verified against the shipping C# table.
pub trait Message: Sized {
    /// Message id.
    const ID: u32;
    /// Message name as written in the XML definition.
    const NAME: &'static str;
    /// `CRC_EXTRA` seed for this message.
    const CRC_EXTRA: u8;
    /// Payload length excluding extension fields.
    const MIN_LEN: usize;
    /// Full payload length including extension fields.
    const LEN: usize;

    /// Decodes from a payload, zero-extending a v2-truncated one.
    fn decode(payload: &[u8]) -> Self;

    /// Encodes into `out`, which must be at least [`Self::LEN`] bytes. Returns bytes written.
    fn encode(&self, out: &mut [u8]) -> usize;
}
