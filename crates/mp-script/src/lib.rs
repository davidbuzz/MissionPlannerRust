//! The scripting host: the API Mission Planner's Python scripts call.
//!
//! Ported from `Script.cs` @ efb0801 (GPL-3.0-or-later).
//!
//! Mission Planner embeds IronPython and hands scripts a scope containing `Script`, `cs`, `MAV`,
//! `MainV2`, `FlightPlanner`, `FlightData`, `Ports`, `Joystick` and `mavutil`. Nineteen scripts
//! ship with it and are checked in under `testdata/scripts/`; PLAN.md §10.4 records the owner's
//! decision that they must run unmodified, which is why the engine is Python and not something
//! nicer.
//!
//! **Two halves.** [`api`] defines and implements the API surface with the C#'s semantics -
//! including the parts that are surprising - and [`inventory`] measures what the shipped corpus
//! actually needs. [`engine`] is the interpreter: RustPython, embedded with its standard library
//! frozen into the binary, running a script with `Script`, `cs` and the rest in scope and its
//! `print` output captured for the console (the owner's ruling of 2026-09-25, PLAN.md §12 D20).
//! The corpus is Python 2 and RustPython is Python 3, so the shipped scripts are changed to run -
//! each change recorded per script - which D20 allows and PLAN.md §10.4's "unmodified" no longer
//! demands.

pub mod api;
pub mod engine;
pub mod inventory;

pub use api::{Conditional, CsValue, ScriptApi, ScriptHost};
pub use engine::{ScriptRun, run_blocking};
pub use inventory::{Requirement, ScriptRequirements, Surface};
