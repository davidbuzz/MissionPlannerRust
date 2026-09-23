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
//! **This crate is the host half, not the engine.** It defines and implements the API surface with
//! the C#'s semantics - including the parts that are surprising - and it measures what the shipped
//! corpus actually needs (see [`inventory`]). Choosing and wiring an interpreter is the next step
//! and is deliberately not bundled with this one: the compatibility question is "what surface do
//! the scripts touch", and that is answerable, checkable and reviewable without compiling a Python
//! VM into the build.

pub mod api;
pub mod inventory;

pub use api::{Conditional, ScriptApi, ScriptHost};
pub use inventory::{Requirement, ScriptRequirements, Surface};
