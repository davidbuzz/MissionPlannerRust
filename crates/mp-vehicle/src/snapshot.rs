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

//! Publishing telemetry snapshots to readers that must never block.

use std::sync::Arc;

use arc_swap::ArcSwap;

use crate::state::VehicleState;

/// How many snapshot allocations to recycle.
///
/// One is held by the cell, one by the publisher, and the rest absorb readers that hold a
/// snapshot across a frame. Beyond this the publisher allocates rather than stalling - dropping
/// telemetry to save an allocation would be the wrong trade.
const POOL_SIZE: usize = 8;

/// Writer side of the snapshot bus. Owned by the link thread.
#[derive(Debug)]
pub struct StatePublisher {
    cell: Arc<ArcSwap<VehicleState>>,
    pool: Vec<Arc<VehicleState>>,
    /// The state being mutated per packet. Never shared, so mutation needs no synchronisation.
    pub working: VehicleState,
    allocations: u64,
}

impl StatePublisher {
    /// Creates a publisher and its first snapshot.
    #[must_use]
    pub fn new(initial: VehicleState) -> Self {
        Self {
            cell: Arc::new(ArcSwap::from_pointee(initial)),
            pool: Vec::with_capacity(POOL_SIZE),
            working: initial,
            allocations: 0,
        }
    }

    /// A handle readers can clone.
    #[must_use]
    pub fn handle(&self) -> StateHandle {
        StateHandle {
            cell: Arc::clone(&self.cell),
        }
    }

    /// Publishes the working state as the new snapshot.
    ///
    /// Reuses a pooled allocation when one is unshared, so a steady state of publish-and-read
    /// performs no allocation at all.
    pub fn publish(&mut self) {
        // Find a pooled snapshot nobody is reading any more.
        let reusable = self.pool.iter_mut().find_map(|arc| {
            Arc::get_mut(arc).map(|slot| {
                *slot = self.working;
            })?;
            Some(Arc::clone(arc))
        });

        let snapshot = match reusable {
            Some(arc) => arc,
            None => {
                let arc = Arc::new(self.working);
                if self.pool.len() < POOL_SIZE {
                    self.pool.push(Arc::clone(&arc));
                }
                self.allocations += 1;
                arc
            }
        };
        self.cell.store(snapshot);
    }

    /// How many snapshot allocations have been made. Used by tests to prove the steady state is
    /// allocation-free.
    #[must_use]
    pub const fn allocations(&self) -> u64 {
        self.allocations
    }
}

/// Reader side of the snapshot bus. Cheap to clone and safe to hold across threads.
#[derive(Debug, Clone)]
pub struct StateHandle {
    cell: Arc<ArcSwap<VehicleState>>,
}

impl StateHandle {
    /// Takes the current snapshot. One atomic load; never blocks, never waits on the link thread.
    #[must_use]
    pub fn load(&self) -> Arc<VehicleState> {
        self.cell.load_full()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn readers_see_published_updates() {
        let mut pub_ = StatePublisher::new(VehicleState::new(1, 1));
        let handle = pub_.handle();
        assert_eq!(handle.load().custom_mode, 0);

        pub_.working.custom_mode = 7;
        pub_.publish();
        assert_eq!(handle.load().custom_mode, 7);
    }

    #[test]
    fn a_held_snapshot_is_not_mutated_underneath_the_reader() {
        // The bug this prevents: a UI frame reading latitude from one packet and longitude from
        // the next.
        let mut pub_ = StatePublisher::new(VehicleState::new(1, 1));
        let handle = pub_.handle();

        pub_.working.custom_mode = 1;
        pub_.publish();
        let held = handle.load();

        for mode in 2..50 {
            pub_.working.custom_mode = mode;
            pub_.publish();
        }
        assert_eq!(
            held.custom_mode, 1,
            "a snapshot must be immutable once handed out"
        );
        assert_eq!(handle.load().custom_mode, 49);
    }

    #[test]
    fn steady_state_publishing_stops_allocating() {
        let mut pub_ = StatePublisher::new(VehicleState::new(1, 1));
        let handle = pub_.handle();

        // Warm the pool, with a reader holding a snapshot part of the time.
        for i in 0..64 {
            pub_.working.messages_applied = i;
            pub_.publish();
            let _held = handle.load();
        }
        let after_warmup = pub_.allocations();

        for i in 0..10_000 {
            pub_.working.messages_applied = i;
            pub_.publish();
        }
        assert_eq!(
            pub_.allocations(),
            after_warmup,
            "steady-state publishing must recycle snapshots"
        );
    }

    #[test]
    fn publishing_works_across_threads() {
        let mut pub_ = StatePublisher::new(VehicleState::new(1, 1));
        let handle = pub_.handle();

        let reader = std::thread::spawn(move || {
            let mut last = 0;
            for _ in 0..10_000 {
                let snap = handle.load();
                // Monotonic: a reader must never observe a value going backwards.
                assert!(snap.messages_applied >= last, "snapshot went backwards");
                last = snap.messages_applied;
            }
            last
        });

        for i in 0..10_000 {
            pub_.working.messages_applied = i;
            pub_.publish();
        }
        let _ = reader.join().expect("reader thread");
    }
}
