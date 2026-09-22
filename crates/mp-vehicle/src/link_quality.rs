//! Packet loss and link health.

/// Link health derived from MAVLink sequence numbers.
///
/// MAVLink stamps every frame with an 8-bit per-sender sequence number, so gaps reveal loss
/// without any cooperation from the vehicle. The subtlety is the wrap: a gap of 250 is far more
/// likely to be a reordered or duplicated packet than 250 genuinely lost frames, so gaps beyond
/// half the sequence space are treated as reordering rather than loss.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct LinkQuality {
    /// Frames received from this sender.
    pub received: u64,
    /// Frames inferred lost from sequence gaps.
    pub lost: u64,
    /// Frames that arrived out of order or duplicated.
    pub out_of_order: u64,
    last_seq: Option<u8>,
}

/// Gaps larger than this are treated as reordering, not loss.
const REORDER_THRESHOLD: u8 = 128;

impl LinkQuality {
    /// Records a received frame's sequence number.
    pub fn record(&mut self, seq: u8) {
        self.received += 1;
        let Some(last) = self.last_seq else {
            self.last_seq = Some(seq);
            return;
        };
        let gap = seq.wrapping_sub(last).wrapping_sub(1);
        // A repeat of the last sequence number, or a gap past half the sequence space, is far
        // more likely to be duplication or reordering than a burst of real loss.
        if seq == last || gap >= REORDER_THRESHOLD {
            self.out_of_order += 1;
        } else if gap > 0 {
            self.lost += u64::from(gap);
        }
        self.last_seq = Some(seq);
    }

    /// Loss as a percentage of frames sent, 0 when nothing has been received.
    #[must_use]
    #[allow(clippy::cast_precision_loss)] // counts stay far below f64's exact integer range
    pub fn loss_percent(&self) -> f64 {
        let sent = self.received + self.lost;
        if sent == 0 {
            return 0.0;
        }
        (self.lost as f64 / sent as f64) * 100.0
    }

    /// Forgets history, e.g. after a reconnect where the sequence restarts.
    pub fn reset(&mut self) {
        *self = Self::default();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_clean_stream_reports_no_loss() {
        let mut q = LinkQuality::default();
        for seq in 0..=255u8 {
            q.record(seq);
        }
        assert_eq!(q.received, 256);
        assert_eq!(q.lost, 0);
        assert!(q.loss_percent() < f64::EPSILON);
    }

    #[test]
    fn gaps_are_counted_as_loss() {
        let mut q = LinkQuality::default();
        q.record(0);
        q.record(3); // 1 and 2 missing
        assert_eq!(q.lost, 2);
        assert_eq!(q.received, 2);
        assert!((q.loss_percent() - 50.0).abs() < 1e-9);
    }

    #[test]
    fn the_sequence_wrap_is_not_a_burst_of_loss() {
        let mut q = LinkQuality::default();
        q.record(254);
        q.record(255);
        q.record(0);
        q.record(1);
        assert_eq!(q.lost, 0, "wrapping from 255 to 0 is normal");
    }

    #[test]
    fn reordering_is_distinguished_from_loss() {
        let mut q = LinkQuality::default();
        q.record(10);
        q.record(9); // arrived late
        assert_eq!(q.lost, 0);
        assert_eq!(q.out_of_order, 1);

        q.record(9); // duplicate
        assert_eq!(q.out_of_order, 2);
    }

    #[test]
    fn loss_across_a_wrap_boundary_is_still_loss() {
        let mut q = LinkQuality::default();
        q.record(253);
        q.record(2); // 254, 255, 0, 1 missing
        assert_eq!(q.lost, 4);
    }
}
