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

//! The engine: RustPython running a script the way `Script.cs` runs one under IronPython.
//!
//! `new Script(redirectOutput)` makes an engine, puts `MainV2`, `FlightPlanner`, `FlightData`,
//! `Ports`, `MAV`, `cs`, `Script`, `mavutil` and `Joystick` in the scope, and - when the Scripts
//! tab's "Redirect Program Output" is ticked - routes the engine's output through a writer the
//! console polls; `runScript` executes the file in that scope and shows "Error running script"
//! with the exception when it throws (`Script.cs:19-70, 107-123`). `FlightData` runs it on a
//! thread named "Script Thread (new)" and its Abort button aborts that thread
//! (`GCSViews/FlightData.cs:786-812, 1012-1017, 4727-4732`).
//!
//! Here:
//!
//! * `Script` is [`PyScript`], each method a call on the [`ScriptHost`] the window supplies;
//!   `cs` is [`PyCurrentState`], whose attributes are the host's `cs_field`, and `cs.messages`
//!   a list with `Clear`;
//! * what `LoadAssembly` gives IronPython - `import clr`, `clr.AddReference(...)`,
//!   `import MissionPlanner`, `import MAVLink`, `from MissionPlanner.Utilities import
//!   Locationwp`, `from System import Byte, Func, Array` - and the objects `MAV`, `MainV2`,
//!   `FlightPlanner`, `FlightData`, `Ports` and `Joystick` are `clr/shim.py`, Python run before
//!   the script, each member with the C#'s semantics and the link's side of it through
//!   [`PyLink`] to the host; a name it does not give raises "<name> is not available to scripts
//!   in this version", so a script stops at the member it reached for, named;
//! * `mavutil` is the `Script` object, as the C# binds it, with `mavlink_connection` and
//!   `recv_match` returning `None` as the C#'s return `null` (`Script.cs:92-102`);
//! * output always goes to the run's buffer, which the console shows; the C# writes to the
//!   process's stdout when the box is unticked, which a desktop application has nowhere to
//!   show, nor has it for the constructor's own `print('hello world from python')` and
//!   `print(cs.roll)` (`Script.cs:55-56`), which run before the output is redirected and are
//!   not run here;
//! * a run has an abort flag that `Sleep`, `WaitFor`, `time.sleep` and the `MAV` members that
//!   wait on the link read, and that the host sees too, so a wait on the link is given up when
//!   it is set: the C#'s `Thread.Abort` stops a script anywhere, this stops it at its next wait
//!   or in the one it is in - a script that never waits runs on, and one blocked in a socket's
//!   `recv` stays there;
//! * RustPython is Python 3 and IronPython 2.7 is Python 2: `print 'x'` is a syntax error here,
//!   which is why the shipped scripts are converted (PLAN.md §12 D20).

use mp_os::Lock as _;
use std::fmt;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, PoisonError};
use wasm_thread::JoinHandle;

use rustpython::InterpreterBuilderExt;
use rustpython_vm::builtins::{PyBaseExceptionRef, PyBytesRef, PyStr, PyStrRef};
use rustpython_vm::convert::TryFromObject;
use rustpython_vm::function::{ArgIntoFloat, FuncArgs};
use rustpython_vm::types::GetAttr;
use rustpython_vm::{
    AsObject, InterpreterBuilder, Py, PyObjectRef, PyPayload, PyResult, Settings, VirtualMachine,
    pyclass,
};

use crate::api::{
    CsValue, PositionTarget, ScriptApi, ScriptHost, Timeout, WAIT_FOR_POLL_MS, WpItem,
};

/// The thread's name, `FlightData.cs:790`.
pub const THREAD_NAME: &str = "Script Thread (new)";

/// How long one slice of a `Sleep` is, so an abort is seen within it.
const SLEEP_SLICE_MS: u32 = 20;

/// The MAVLink enums the shim hands scripts, member for member from the C#'s `Mavlink.cs`.
const MAVLINK_ENUMS: &str = include_str!("clr/mavlink_enums.py");

/// `clr`, `System`, `MissionPlanner`, `MAVLink`, and `MAV`, `MainV2`, the screens, `Ports` and
/// `Joystick`: see the file.
const SHIM: &str = include_str!("clr/shim.py");

/// Every .NET name the shipped scripts reach that the shim answers, whole, as
/// [`crate::ScriptRequirements::dotnet_names`] lists them; the engine's tests check that each
/// resolves, and `tests/stock_scripts.rs` that every name the corpus reaches is here or recorded
/// out of scope.
pub const CLR_NAMES: &[&str] = &[
    "MAVLink.MAVLINK_MSG_ID.HEARTBEAT.value__",
    "MAVLink.MAVLINK_MSG_ID.STATUSTEXT",
    "MAVLink.MAVLINK_MSG_ID.STATUSTEXT.value__",
    "MAVLink.MAVLinkMessage",
    "MAVLink.MAV_CMD.DO_DIGICAM_CONTROL",
    "MAVLink.MAV_CMD.DO_DIGICAM_CONTROL.value__",
    "MAVLink.MAV_CMD.TAKEOFF",
    "MAVLink.MAV_CMD.WAYPOINT",
    "MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT",
    "MAVLink.MAV_MOUNT_MODE.NEUTRAL.value__",
    "MAVLink.mavlink_command_long_t",
    "MissionPlanner.Comms",
    "MissionPlanner.MainV2.speechEnable",
    "MissionPlanner.MainV2.speechEngine.SpeakAsync",
    "MissionPlanner.Utilities.Locationwp",
    "MissionPlanner.Utilities.Locationwp.alt.SetValue",
    "MissionPlanner.Utilities.Locationwp.lat.SetValue",
    "MissionPlanner.Utilities.Locationwp.lng.SetValue",
    "System.Action",
    "System.Array",
    "System.Byte",
    "System.Func",
    "clr.AddReference",
    "clr.ClearProfilerData",
];

/// What the shim puts in a script's scope beside `Script` and `mavutil`, by the C#'s names.
/// `// C#: Script.cs:45-53`
const SHIM_BINDINGS: [&str; 7] = [
    "MainV2",
    "FlightPlanner",
    "FlightData",
    "Ports",
    "MAV",
    "cs",
    "Joystick",
];

/// What a run shares between the Python objects and the host.
struct Runtime {
    host: Box<dyn ScriptHost + Send>,
    /// The RC override state, `Script.rc`, kept across `SendRC` calls.
    api: ScriptApi,
    /// The Abort button.
    abort: Arc<AtomicBool>,
}

impl fmt::Debug for Runtime {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Runtime")
            .field("api", &self.api)
            .field("abort", &self.abort.load(Ordering::Relaxed))
            .finish_non_exhaustive()
    }
}

/// The runtime as the Python objects hold it.
type Shared = Arc<Mutex<Runtime>>;

fn lock(shared: &Shared) -> std::sync::MutexGuard<'_, Runtime> {
    shared.os_lock().unwrap_or_else(PoisonError::into_inner)
}

/// A count a script passes: IronPython lets a float stand where the C# takes an `int` or a
/// `short` - `Script.SendRC(3, Script.GetParam('RC3_MIN'), True)` in the shipped example1 hands
/// `GetParam`'s float to `short pwm` - so every count here takes a float and keeps its whole part.
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn whole(value: f64) -> u32 {
    if value.is_nan() || value <= 0.0 {
        0
    } else {
        value.min(f64::from(u32::MAX)) as u32
    }
}

/// The Abort button pressed: `SystemExit`, which ends the script wherever it is waiting.
fn aborted(vm: &VirtualMachine) -> PyResult<()> {
    Err(abort_exception(vm))
}

/// The `SystemExit` the Abort button raises.
fn abort_exception(vm: &VirtualMachine) -> PyBaseExceptionRef {
    vm.new_exception_msg(
        vm.ctx.exceptions.system_exit.to_owned(),
        "script aborted".into(),
    )
}

/// `Script`, and `mavutil`: the object every method of `Script.cs` hangs off.
/// `// C#: Script.cs:12-217`
#[pyclass(module = false, name = "Script")]
#[derive(Debug, PyPayload)]
pub struct PyScript {
    shared: Shared,
}

#[pyclass]
impl PyScript {
    /// `Sleep(ms)`: in slices, so the Abort button is seen within 20 ms.
    /// `// C#: Script.cs:104-107`
    #[pymethod(name = "Sleep")]
    fn sleep(&self, ms: ArgIntoFloat, vm: &VirtualMachine) -> PyResult<()> {
        let mut left = whole(ms.into());
        while left > 0 {
            let slice = left.min(SLEEP_SLICE_MS);
            {
                let mut runtime = lock(&self.shared);
                if runtime.abort.load(Ordering::Relaxed) {
                    return aborted(vm);
                }
                runtime.host.sleep(slice);
            }
            left -= slice;
        }
        Ok(())
    }

    /// `ChangeParam(param, value)`: `setParam`'s answer.
    /// `// C#: Script.cs:136-139`
    #[pymethod(name = "ChangeParam")]
    fn change_param(&self, param: PyStrRef, value: ArgIntoFloat) -> bool {
        #[allow(clippy::cast_possible_truncation)] // the C# takes a float
        let value = f64::from(value) as f32;
        let param = param.to_string_lossy();
        lock(&self.shared).host.change_param(&param, value)
    }

    /// `GetParam(param)`: the value held, 0.0 for a name not held.
    /// `// C#: Script.cs:141-147`
    #[pymethod(name = "GetParam")]
    fn get_param(&self, param: PyStrRef) -> f64 {
        let param = param.to_string_lossy();
        f64::from(lock(&self.shared).host.get_param(&param))
    }

    /// `ChangeMode(mode)`: `setMode`, and true whatever the vehicle did.
    /// `// C#: Script.cs:149-153`
    #[pymethod(name = "ChangeMode")]
    fn change_mode(&self, mode: PyStrRef) -> bool {
        let mode = mode.to_string_lossy();
        lock(&self.shared).host.change_mode(&mode)
    }

    /// `WaitFor(message, timeout)`: the C#'s 5 ms poll over every message so far, the abort seen
    /// at each poll.
    /// `// C#: Script.cs:155-167`
    #[pymethod(name = "WaitFor")]
    fn wait_for(
        &self,
        message: PyStrRef,
        timeout: ArgIntoFloat,
        vm: &VirtualMachine,
    ) -> PyResult<bool> {
        let timeout = whole(timeout.into());
        let mut waited: u32 = 0;
        loop {
            let mut runtime = lock(&self.shared);
            if runtime.abort.load(Ordering::Relaxed) {
                return aborted(vm).map(|()| false);
            }
            if runtime.host.has_message(&message.to_string_lossy()) {
                return Ok(true);
            }
            runtime.host.sleep(WAIT_FOR_POLL_MS);
            waited = waited.saturating_add(WAIT_FOR_POLL_MS);
            if waited > timeout {
                return Ok(false);
            }
        }
    }

    /// `SendRC(channel, pwm, sendnow)`: the channel kept in the override the object holds, and
    /// the whole override sent when asked.
    /// `// C#: Script.cs:169-215`
    #[pymethod(name = "SendRC")]
    fn send_rc(&self, channel: ArgIntoFloat, pwm: ArgIntoFloat, sendnow: bool) -> bool {
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let channel = f64::from(channel) as u8;
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        // `(ushort) pwm` of a `short`: the C#'s own truncation.
        let pwm = f64::from(pwm) as u16;
        let mut runtime = lock(&self.shared);
        runtime.api.set_channel(channel, pwm);
        runtime.host.send_rc(channel, pwm, sendnow)
    }

    /// `mavutil.mavlink_connection(...)`: the C#'s `return null`.
    /// `// C#: Script.cs:92-97`
    #[pymethod]
    fn mavlink_connection(&self, _args: FuncArgs, vm: &VirtualMachine) -> PyObjectRef {
        vm.ctx.none()
    }

    /// `mavutil.recv_match(...)`: the C#'s `return null`.
    /// `// C#: Script.cs:99-102`
    #[pymethod]
    fn recv_match(&self, _args: FuncArgs, vm: &VirtualMachine) -> PyObjectRef {
        vm.ctx.none()
    }
}

/// `cs`: the vehicle's state by field name.
/// `// C#: Script.cs:50`
#[pyclass(module = false, name = "CurrentState")]
#[derive(Debug, PyPayload)]
pub struct PyCurrentState {
    shared: Shared,
}

#[pyclass(with(GetAttr))]
impl PyCurrentState {}

impl GetAttr for PyCurrentState {
    fn getattro(zelf: &Py<Self>, name: &Py<PyStr>, vm: &VirtualMachine) -> PyResult {
        let name = name.to_string_lossy();
        let name: &str = &name;
        if name == "messages" {
            return Ok(PyMessages {
                shared: zelf.shared.clone(),
            }
            .into_pyobject(vm));
        }
        match lock(&zelf.shared).host.cs_field(name) {
            Some(CsValue::Number(value)) => Ok(vm.ctx.new_float(value).into()),
            Some(CsValue::Text(text)) => Ok(vm.ctx.new_str(text).into()),
            Some(CsValue::Flag(flag)) => Ok(vm.ctx.new_bool(flag).into()),
            None => Err(vm.new_attribute_error(format!("cs has no field '{name}'"))),
        }
    }
}

/// `cs.messages`: the status messages, with the `Clear` the corpus calls.
#[pyclass(module = false, name = "MessageList")]
#[derive(Debug, PyPayload)]
pub struct PyMessages {
    shared: Shared,
}

#[pyclass]
impl PyMessages {
    /// `cs.messages.Clear()`.
    #[pymethod(name = "Clear")]
    fn clear(&self) {
        lock(&self.shared).host.clear_messages();
    }
}

/// The link's side of `MAV`: what `clr/shim.py`'s `MAVLinkInterface` calls on the host, a
/// method each, positional arguments as the shim passes them. A host's [`Timeout`] is Python's
/// `TimeoutError` here, which the shim turns into `System.TimeoutException`.
#[pyclass(module = false, name = "Link")]
#[derive(Debug, PyPayload)]
pub struct PyLink {
    shared: Shared,
}

/// One positional argument, converted.
fn arg<T: TryFromObject>(args: &FuncArgs, index: usize, vm: &VirtualMachine) -> PyResult<T> {
    let object = args
        .args
        .get(index)
        .cloned()
        .ok_or_else(|| vm.new_type_error(format!("argument {index} missing")))?;
    T::try_from_object(vm, object)
}

/// The two ids every link member starts with.
fn target_arg(args: &FuncArgs, vm: &VirtualMachine) -> PyResult<(u8, u8)> {
    Ok((arg::<u8>(args, 0, vm)?, arg::<u8>(args, 1, vm)?))
}

/// A number argument, an int or a float.
fn number(args: &FuncArgs, index: usize, vm: &VirtualMachine) -> PyResult<f64> {
    arg::<ArgIntoFloat>(args, index, vm).map(f64::from)
}

/// A truth-value argument.
fn flag(args: &FuncArgs, index: usize, vm: &VirtualMachine) -> PyResult<bool> {
    arg::<PyObjectRef>(args, index, vm)?.try_to_bool(vm)
}

/// A list of numbers as 32-bit floats: the C#'s `float` parameters.
#[allow(clippy::cast_possible_truncation)] // the C#'s `(float)`
fn floats<const N: usize>(
    args: &FuncArgs,
    index: usize,
    vm: &VirtualMachine,
) -> PyResult<[f32; N]> {
    let list = arg::<PyObjectRef>(args, index, vm)?;
    let values = vm.extract_elements_with(&list, |item| {
        ArgIntoFloat::try_from_object(vm, item).map(|value| f64::from(value) as f32)
    })?;
    <[f32; N]>::try_from(values)
        .map_err(|_| vm.new_type_error(format!("argument {index} wants {N} numbers")))
}

/// The host's [`Timeout`] as Python's `TimeoutError`.
fn timed_out(vm: &VirtualMachine, timeout: Timeout) -> PyBaseExceptionRef {
    vm.new_os_subtype_error(vm.ctx.exceptions.timeout_error.to_owned(), None, timeout.0)
        .upcast()
}

/// A `MAV` member that waits on the link, called on the host: the Abort button seen before it
/// starts and again when it returns - a host gives its wait up once the button is pressed - as
/// `SystemExit`, which no `except TimeoutException` in the script catches. The C#'s
/// `Thread.Abort` ends a script inside such a wait as anywhere else.
fn waited<T>(
    shared: &Shared,
    vm: &VirtualMachine,
    call: impl FnOnce(&mut dyn ScriptHost) -> Result<T, Timeout>,
) -> PyResult<T> {
    let mut runtime = lock(shared);
    if runtime.abort.load(Ordering::Relaxed) {
        return Err(abort_exception(vm));
    }
    let answer = call(runtime.host.as_mut());
    if runtime.abort.load(Ordering::Relaxed) {
        return Err(abort_exception(vm));
    }
    answer.map_err(|timeout| timed_out(vm, timeout))
}

#[pyclass]
impl PyLink {
    /// `MAV.sysid`, `MAV.compid`.
    #[pymethod]
    fn target(&self) -> (u8, u8) {
        lock(&self.shared).host.link_target()
    }

    /// `MAV.BaseStream.IsOpen`.
    #[pymethod]
    fn is_open(&self) -> bool {
        lock(&self.shared).host.is_open()
    }

    /// `setParam`: sysid, compid, name, value, force.
    #[pymethod]
    fn set_param(&self, args: FuncArgs, vm: &VirtualMachine) -> PyResult<bool> {
        let target = target_arg(&args, vm)?;
        let name = arg::<PyStrRef>(&args, 2, vm)?.to_string_lossy().into_owned();
        let value = number(&args, 3, vm)?;
        let force = flag(&args, 4, vm)?;
        waited(&self.shared, vm, |host| {
            host.set_param(target, &name, value, force)
        })
    }

    /// `doCommand`: sysid, compid, command, [p1..p7], requireack.
    #[pymethod]
    fn command(&self, args: FuncArgs, vm: &VirtualMachine) -> PyResult<bool> {
        let target = target_arg(&args, vm)?;
        let command = arg::<u16>(&args, 2, vm)?;
        let params = floats::<7>(&args, 3, vm)?;
        let require_ack = flag(&args, 4, vm)?;
        waited(&self.shared, vm, |host| {
            host.command(target, command, params, require_ack)
        })
    }

    /// `doReboot(bootloadermode, true)`.
    #[pymethod]
    fn reboot(&self, bootloader: bool, vm: &VirtualMachine) -> PyResult<bool> {
        waited(&self.shared, vm, |host| host.reboot(bootloader))
    }

    /// `setWPTotal`: sysid, compid, total, mission type.
    #[pymethod]
    fn set_wp_total(&self, args: FuncArgs, vm: &VirtualMachine) -> PyResult<()> {
        let target = target_arg(&args, vm)?;
        let total = arg::<u16>(&args, 2, vm)?;
        let mission_type = arg::<u8>(&args, 3, vm)?;
        waited(&self.shared, vm, |host| {
            host.set_wp_total(target, total, mission_type)
        })
    }

    /// `setWP`: sysid, compid, seq, frame, command, current, autocontinue, [p1..p4], x, y, z,
    /// mission type. The `MAV_MISSION_RESULT` the vehicle answered with.
    #[pymethod]
    #[allow(clippy::cast_possible_truncation)] // the C#'s `(float)`
    fn set_wp(&self, args: FuncArgs, vm: &VirtualMachine) -> PyResult<u8> {
        let target = target_arg(&args, vm)?;
        let item = WpItem {
            seq: arg::<u16>(&args, 2, vm)?,
            frame: arg::<u8>(&args, 3, vm)?,
            command: arg::<u16>(&args, 4, vm)?,
            current: arg::<u8>(&args, 5, vm)?,
            autocontinue: arg::<u8>(&args, 6, vm)?,
            params: floats::<4>(&args, 7, vm)?,
            x: number(&args, 8, vm)? as f32,
            y: number(&args, 9, vm)? as f32,
            z: number(&args, 10, vm)? as f32,
            mission_type: arg::<u8>(&args, 11, vm)?,
        };
        waited(&self.shared, vm, |host| host.set_wp(target, &item))
    }

    /// `setWPACK`: sysid, compid, mission type.
    #[pymethod]
    fn set_wp_ack(&self, args: FuncArgs, vm: &VirtualMachine) -> PyResult<()> {
        let target = target_arg(&args, vm)?;
        let mission_type = arg::<u8>(&args, 2, vm)?;
        lock(&self.shared).host.set_wp_ack(target, mission_type);
        Ok(())
    }

    /// `setWPCurrent`: sysid, compid, index.
    #[pymethod]
    fn set_wp_current(&self, args: FuncArgs, vm: &VirtualMachine) -> PyResult<bool> {
        let target = target_arg(&args, vm)?;
        let seq = arg::<u16>(&args, 2, vm)?;
        waited(&self.shared, vm, |host| host.set_wp_current(target, seq))
    }

    /// `getWP`: sysid, compid, index, mission type. The `Locationwp`'s id, p1 to p4, lat, lng,
    /// alt and frame, for the shim to put in one.
    #[pymethod]
    fn get_wp(&self, args: FuncArgs, vm: &VirtualMachine) -> PyResult<PyObjectRef> {
        let target = target_arg(&args, vm)?;
        let index = arg::<u16>(&args, 2, vm)?;
        let mission_type = arg::<u8>(&args, 3, vm)?;
        let wp = waited(&self.shared, vm, |host| {
            host.get_wp(target, index, mission_type)
        })?;
        let fields: Vec<PyObjectRef> = vec![
            vm.ctx.new_int(wp.id).into(),
            vm.ctx.new_float(f64::from(wp.p1)).into(),
            vm.ctx.new_float(f64::from(wp.p2)).into(),
            vm.ctx.new_float(f64::from(wp.p3)).into(),
            vm.ctx.new_float(f64::from(wp.p4)).into(),
            vm.ctx.new_float(wp.lat).into(),
            vm.ctx.new_float(wp.lng).into(),
            vm.ctx.new_float(f64::from(wp.alt)).into(),
            vm.ctx.new_int(wp.frame).into(),
        ];
        Ok(vm.ctx.new_tuple(fields).into())
    }

    /// `setPositionTargetGlobalInt` for a guided target: sysid, compid, frame, lat, lng, alt.
    #[pymethod]
    fn set_position_target(&self, args: FuncArgs, vm: &VirtualMachine) -> PyResult<bool> {
        let target = target_arg(&args, vm)?;
        let position = PositionTarget {
            frame: arg::<u8>(&args, 2, vm)?,
            lat: number(&args, 3, vm)?,
            lng: number(&args, 4, vm)?,
            alt: number(&args, 5, vm)?,
        };
        Ok(lock(&self.shared)
            .host
            .set_position_target(target, &position))
    }

    /// `BaseStream.Write`: the bytes.
    #[pymethod]
    fn write_raw(&self, data: PyBytesRef) -> bool {
        lock(&self.shared).host.write_raw(data.as_bytes())
    }

    /// `SpeakAsync`'s text, once its checks have passed.
    #[pymethod]
    fn speak(&self, text: PyStrRef) {
        let text = text.to_string_lossy();
        lock(&self.shared).host.speak(&text);
    }

    /// `MainV2.speechEnable` and `MainV2.speech_armed_only` as the user's settings have them.
    #[pymethod]
    fn speech_settings(&self) -> (bool, bool) {
        lock(&self.shared).host.speech_settings()
    }

    /// `MAVlist[sysid, compid].cs.<name>`: sysid, compid, name; `None` for a field the vehicle
    /// does not have.
    #[pymethod]
    fn cs_of(&self, args: FuncArgs, vm: &VirtualMachine) -> PyResult<PyObjectRef> {
        let target = target_arg(&args, vm)?;
        let name = arg::<PyStrRef>(&args, 2, vm)?.to_string_lossy().into_owned();
        Ok(match lock(&self.shared).host.cs_field_of(target, &name) {
            Some(CsValue::Number(value)) => vm.ctx.new_float(value).into(),
            Some(CsValue::Text(text)) => vm.ctx.new_str(text).into(),
            Some(CsValue::Flag(flag)) => vm.ctx.new_bool(flag).into(),
            None => vm.ctx.none(),
        })
    }

    /// `setMode(sysid, compid, mode)`: sysid, compid, mode.
    #[pymethod]
    fn set_mode_of(&self, args: FuncArgs, vm: &VirtualMachine) -> PyResult<()> {
        let target = target_arg(&args, vm)?;
        let mode = arg::<PyStrRef>(&args, 2, vm)?.to_string_lossy().into_owned();
        lock(&self.shared).host.set_mode_of(target, &mode);
        Ok(())
    }
}

/// `sys.stdout` and `sys.stderr`: the run's output buffer, `StringRedirectWriter`.
/// `// C#: Script.cs:58-66; ExtLibs/Utilities/StringRedirectWriter.cs`
#[pyclass(module = false, name = "ScriptOutput")]
#[derive(Debug, PyPayload)]
pub struct PyOutput {
    buffer: Arc<Mutex<String>>,
}

#[pyclass]
impl PyOutput {
    #[pymethod]
    fn write(&self, text: PyStrRef) -> usize {
        let text = text.to_string_lossy();
        self.buffer
            .os_lock()
            .unwrap_or_else(PoisonError::into_inner)
            .push_str(&text);
        text.len()
    }

    #[pymethod]
    fn flush(&self) {}
}

/// The native classes' types made, once each: a `#[pyclass]` outside a `#[pymodule]` has no
/// module to make its type on import, and an instance made before that is the "static type has
/// not been initialized" panic. Only inside an interpreter: the types derive from `object`,
/// which the context's genesis makes.
fn init_types() {
    use rustpython_vm::class::{PyClassImpl, StaticType};
    use rustpython_vm::vm::Context;
    // `make_class` in one: the type made, then its methods and slots attached, as a module's
    // `#[pymodule]` init does for the classes it holds.
    fn make<T: PyClassImpl + StaticType>() {
        if T::static_cell().get().is_none() {
            T::init_builtin_type();
            let ctx: &'static Context = Context::genesis();
            T::extend_class(ctx, T::static_type());
        }
    }
    make::<PyScript>();
    make::<PyCurrentState>();
    make::<PyMessages>();
    make::<PyLink>();
    make::<PyOutput>();
}

/// Runs `source` as `runScript` runs a file, on this thread: the scope built, the output routed
/// to `output`, the script executed. `Ok` when it ran to its end or was aborted; `Err` with the
/// exception's text when it threw, as "Error running script" shows it.
/// `// C#: Script.cs:107-123`
pub fn run_with(
    name: &str,
    source: &str,
    host: Box<dyn ScriptHost + Send>,
    output: Arc<Mutex<String>>,
    abort: Arc<AtomicBool>,
) -> Result<(), String> {
    // A byte-order mark opening a file saved by a Windows editor (`example9 - sitl.py` has one)
    // is not source; CPython drops it when it reads the file, and so does this.
    let source = source.strip_prefix('\u{feff}').unwrap_or(source);
    let shared: Shared = Arc::new(Mutex::new(Runtime {
        host,
        api: ScriptApi::new(),
        abort,
    }));
    let mut settings = Settings::default();
    settings.install_signal_handlers = false;
    settings.isolated = true;
    let interpreter = InterpreterBuilder::new()
        .settings(settings)
        .init_stdlib()
        .build();
    interpreter.enter(|vm| -> Result<(), String> {
        // After genesis: the classes derive from `object`, which the context makes.
        init_types();
        let describe = |exc: PyBaseExceptionRef| -> String {
            let mut text = String::new();
            let _ = vm.write_exception(&mut text, &exc);
            text
        };
        let stdout = PyOutput {
            buffer: Arc::clone(&output),
        }
        .into_pyobject(vm);
        let stderr = PyOutput {
            buffer: Arc::clone(&output),
        }
        .into_pyobject(vm);
        vm.sys_module
            .set_attr("stdout", stdout, vm)
            .map_err(describe)?;
        vm.sys_module
            .set_attr("stderr", stderr, vm)
            .map_err(describe)?;
        // `// C#: Script.cs:35-53`: the assemblies loaded, then the scope's objects.
        let script = PyScript {
            shared: Arc::clone(&shared),
        }
        .into_pyobject(vm);
        let shim = vm.new_scope_with_builtins();
        let natives: [(&str, PyObjectRef); 3] = [
            (
                "_link",
                PyLink {
                    shared: Arc::clone(&shared),
                }
                .into_pyobject(vm),
            ),
            ("_script", script.clone()),
            (
                "_cs",
                PyCurrentState {
                    shared: Arc::clone(&shared),
                }
                .into_pyobject(vm),
            ),
        ];
        for (binding, object) in natives {
            shim.globals
                .set_item(binding, object, vm)
                .map_err(describe)?;
        }
        vm.run_string(shim.clone(), MAVLINK_ENUMS, "mavlink_enums.py".to_owned())
            .map_err(describe)?;
        vm.run_string(shim.clone(), SHIM, "shim.py".to_owned())
            .map_err(describe)?;
        let scope = vm.new_scope_with_builtins();
        for binding in SHIM_BINDINGS {
            let object = shim.globals.get_item(binding, vm).map_err(describe)?;
            scope
                .globals
                .set_item(binding, object, vm)
                .map_err(describe)?;
        }
        scope
            .globals
            .set_item("Script", script.clone(), vm)
            .map_err(describe)?;
        scope
            .globals
            .set_item("mavutil", script, vm)
            .map_err(describe)?;
        match vm.run_string(scope, source, name.to_owned()) {
            Ok(_) => Ok(()),
            Err(exc) => {
                // The Abort button's `SystemExit` is the script stopped, not a script that failed.
                if exc.fast_isinstance(vm.ctx.exceptions.system_exit) {
                    return Ok(());
                }
                Err(describe(exc))
            }
        }
    })
}

/// Runs `source` on this thread and returns what came of it and everything it printed.
pub fn run_blocking(
    name: &str,
    source: &str,
    host: Box<dyn ScriptHost + Send>,
) -> (Result<(), String>, String) {
    let output = Arc::new(Mutex::new(String::new()));
    let result = run_with(
        name,
        source,
        host,
        Arc::clone(&output),
        Arc::new(AtomicBool::new(false)),
    );
    let printed = output
        .os_lock()
        .unwrap_or_else(PoisonError::into_inner)
        .clone();
    (result, printed)
}

/// A script on its own thread, as the Scripts tab runs one: its output as it comes, whether it
/// is still running, its Abort button, and how it ended.
/// `// C#: GCSViews/FlightData.cs:786-812, 1012-1017, 4727-4732, 4757-4776`
#[derive(Debug)]
pub struct ScriptRun {
    output: Arc<Mutex<String>>,
    /// What the console has not yet taken: `StringRedirectWriter.RetrieveWrittenString`.
    taken: usize,
    abort: Arc<AtomicBool>,
    result: Arc<Mutex<Option<Result<(), String>>>>,
    thread: Option<JoinHandle<()>>,
}

impl ScriptRun {
    /// `BUT_run_script_Click`: the script started on "Script Thread (new)".
    pub fn start(name: &str, source: String, host: Box<dyn ScriptHost + Send>) -> Self {
        Self::start_with_abort(name, source, host, Arc::new(AtomicBool::new(false)))
    }

    /// [`ScriptRun::start`] with the Abort button's flag given, so the host can see it too: a
    /// host that waits on something - the window, the link - gives the wait up when it is set.
    pub fn start_with_abort(
        name: &str,
        source: String,
        host: Box<dyn ScriptHost + Send>,
        abort: Arc<AtomicBool>,
    ) -> Self {
        let output = Arc::new(Mutex::new(String::new()));
        let result: Arc<Mutex<Option<Result<(), String>>>> = Arc::new(Mutex::new(None));
        let name = name.to_owned();
        let thread = {
            let output = Arc::clone(&output);
            let abort = Arc::clone(&abort);
            let result = Arc::clone(&result);
            wasm_thread::Builder::new()
                .name(THREAD_NAME.to_owned())
                .spawn(move || {
                    let outcome = run_with(&name, &source, host, output, abort);
                    *result.os_lock().unwrap_or_else(PoisonError::into_inner) = Some(outcome);
                })
                .ok()
        };
        let run = Self {
            output,
            taken: 0,
            abort,
            result,
            thread,
        };
        if run.thread.is_none() {
            *run.result.os_lock().unwrap_or_else(PoisonError::into_inner) =
                Some(Err("the script thread could not be started".to_owned()));
        }
        run
    }

    /// `scriptrunning`: whether the thread is still going.
    #[must_use]
    pub fn is_running(&self) -> bool {
        self.result
            .os_lock()
            .unwrap_or_else(PoisonError::into_inner)
            .is_none()
    }

    /// `BUT_abort_script_Click`: the script stopped at its next `Sleep`, `WaitFor` or `MAV`
    /// member that waits on the link - or inside one it is waiting in, when its host gives the
    /// wait up (see [`ScriptRun::start_with_abort`]).
    pub fn abort(&self) {
        self.abort.store(true, Ordering::Relaxed);
    }

    /// `RetrieveWrittenString`: what was printed since the last call.
    pub fn take_output(&mut self) -> String {
        let output = self.output.os_lock().unwrap_or_else(PoisonError::into_inner);
        let fresh = output.get(self.taken..).unwrap_or("").to_owned();
        self.taken = output.len();
        fresh
    }

    /// Everything printed so far.
    #[must_use]
    pub fn output(&self) -> String {
        self.output
            .os_lock()
            .unwrap_or_else(PoisonError::into_inner)
            .clone()
    }

    /// How the run ended, once it has: `Ok` for a script that finished or was aborted, `Err`
    /// with the exception's text for one that threw.
    #[must_use]
    pub fn result(&self) -> Option<Result<(), String>> {
        self.result
            .os_lock()
            .unwrap_or_else(PoisonError::into_inner)
            .clone()
    }

    /// Waits for the thread to end, for a test.
    pub fn join(&mut self) {
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;
    use std::time::Duration;

    use super::*;
    use crate::api::Locationwp;

    /// A host that records what a script asked of it.
    #[derive(Debug, Default)]
    struct Fake {
        params: BTreeMap<String, f32>,
        messages: Vec<String>,
        fields: BTreeMap<String, CsValue>,
        modes: Vec<String>,
        rc: Vec<(u8, u16, bool)>,
        writes: Vec<(String, f32)>,
        slept_ms: u64,
        cleared: usize,
        /// What `MAV` and `MainV2` asked of the link, one line a call.
        calls: Vec<String>,
        /// Every waited link call times out, as with a vehicle that never answers.
        silent: bool,
        /// The items `setWP` wrote, which `getWP` reads back.
        wps: BTreeMap<u16, WpItem>,
        /// The user's "speechenable" and "speech_armed_only".
        speech: (bool, bool),
        /// `getWP` waits until this is set - the Abort button - and says it has started.
        get_wp_waits: Option<(Arc<AtomicBool>, Arc<AtomicBool>)>,
    }

    impl Fake {
        fn answer<T>(&mut self, call: String, member: &str, value: T) -> Result<T, Timeout> {
            self.calls.push(call);
            if self.silent {
                Err(Timeout::on(member))
            } else {
                Ok(value)
            }
        }
    }

    impl ScriptHost for Fake {
        fn link_target(&self) -> (u8, u8) {
            (1, 1)
        }
        fn is_open(&self) -> bool {
            true
        }
        fn set_param(
            &mut self,
            target: (u8, u8),
            name: &str,
            value: f64,
            force: bool,
        ) -> Result<bool, Timeout> {
            let call = format!("setParam {target:?} {name} {value} {force}");
            self.answer(call, &format!("setParam {name}"), true)
        }
        fn command(
            &mut self,
            target: (u8, u8),
            command: u16,
            params: [f32; 7],
            require_ack: bool,
        ) -> Result<bool, Timeout> {
            let call = format!("doCommand {target:?} {command} {params:?} {require_ack}");
            self.answer(call, "doCommand", true)
        }
        fn set_wp_total(&mut self, target: (u8, u8), total: u16, kind: u8) -> Result<(), Timeout> {
            self.answer(format!("setWPTotal {target:?} {total} {kind}"), "setWPTotal", ())
        }
        fn set_wp(&mut self, target: (u8, u8), item: &WpItem) -> Result<u8, Timeout> {
            self.wps.insert(item.seq, *item);
            self.answer(format!("setWP {target:?} {item:?}"), "setWP", 0)
        }
        fn set_wp_ack(&mut self, target: (u8, u8), kind: u8) {
            self.calls.push(format!("setWPACK {target:?} {kind}"));
        }
        fn set_wp_current(&mut self, target: (u8, u8), seq: u16) -> Result<bool, Timeout> {
            self.answer(format!("setWPCurrent {target:?} {seq}"), "setWPCurrent", true)
        }
        fn get_wp(&mut self, target: (u8, u8), index: u16, kind: u8) -> Result<Locationwp, Timeout> {
            if let Some((abort, started)) = &self.get_wp_waits {
                // A vehicle that never answers, and a host that gives the wait up at the Abort
                // button, as the window's does.
                started.store(true, Ordering::Relaxed);
                while !abort.load(Ordering::Relaxed) {
                    wasm_thread::sleep(Duration::from_millis(1));
                }
                return Err(Timeout::on("getWP"));
            }
            let item = self.wps.get(&index).copied();
            let call = format!("getWP {target:?} {index} {kind}");
            match item {
                Some(item) => self.answer(
                    call,
                    "getWP",
                    Locationwp {
                        id: item.command,
                        p1: item.params[0],
                        p2: item.params[1],
                        p3: item.params[2],
                        p4: item.params[3],
                        lat: f64::from(item.x),
                        lng: f64::from(item.y),
                        alt: item.z,
                        frame: item.frame,
                    },
                ),
                None => {
                    self.calls.push(call);
                    Err(Timeout::on("getWP"))
                }
            }
        }
        fn set_position_target(&mut self, target: (u8, u8), position: &PositionTarget) -> bool {
            self.calls.push(format!("setPositionTarget {target:?} {position:?}"));
            true
        }
        fn write_raw(&mut self, bytes: &[u8]) -> bool {
            self.calls.push(format!("Write {bytes:?}"));
            true
        }
        fn speak(&mut self, text: &str) {
            self.calls.push(format!("Speak {text}"));
        }
        fn speech_settings(&self) -> (bool, bool) {
            self.speech
        }

        fn get_parameter(&self, name: &str) -> Option<f32> {
            self.params.get(name).copied()
        }
        fn change_param(&mut self, name: &str, value: f32) -> bool {
            self.writes.push((name.to_owned(), value));
            self.params.insert(name.to_owned(), value);
            true
        }
        fn change_mode(&mut self, mode: &str) -> bool {
            self.modes.push(mode.to_owned());
            true
        }
        fn has_message(&self, text: &str) -> bool {
            self.messages.iter().any(|message| message.contains(text))
        }
        fn clear_messages(&mut self) {
            self.messages.clear();
            self.cleared += 1;
        }
        fn cs_field(&self, name: &str) -> Option<CsValue> {
            self.fields.get(name).cloned()
        }
        fn send_rc(&mut self, channel: u8, pwm: u16, send_now: bool) -> bool {
            self.rc.push((channel, pwm, send_now));
            true
        }
        fn sleep(&mut self, milliseconds: u32) {
            self.slept_ms += u64::from(milliseconds);
            // A real wait, short, so the abort test has time to press its button.
            wasm_thread::sleep(Duration::from_millis(1));
        }
    }

    /// Runs a script against a fake host and hands back the host, what came of the run and what
    /// was printed. The host is shared so the test can read it afterwards.
    fn run(source: &str, fake: Fake) -> (Arc<Mutex<Fake>>, Result<(), String>, String) {
        struct Through(Arc<Mutex<Fake>>);
        impl fmt::Debug for Through {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                f.write_str("Through")
            }
        }
        impl ScriptHost for Through {
            fn get_parameter(&self, name: &str) -> Option<f32> {
                self.0.os_lock().unwrap().get_parameter(name)
            }
            fn change_param(&mut self, name: &str, value: f32) -> bool {
                self.0.os_lock().unwrap().change_param(name, value)
            }
            fn change_mode(&mut self, mode: &str) -> bool {
                self.0.os_lock().unwrap().change_mode(mode)
            }
            fn has_message(&self, text: &str) -> bool {
                self.0.os_lock().unwrap().has_message(text)
            }
            fn clear_messages(&mut self) {
                self.0.os_lock().unwrap().clear_messages();
            }
            fn cs_field(&self, name: &str) -> Option<CsValue> {
                self.0.os_lock().unwrap().cs_field(name)
            }
            fn send_rc(&mut self, channel: u8, pwm: u16, send_now: bool) -> bool {
                self.0.os_lock().unwrap().send_rc(channel, pwm, send_now)
            }
            fn sleep(&mut self, milliseconds: u32) {
                self.0.os_lock().unwrap().sleep(milliseconds);
            }
            fn link_target(&self) -> (u8, u8) {
                self.0.os_lock().unwrap().link_target()
            }
            fn is_open(&self) -> bool {
                self.0.os_lock().unwrap().is_open()
            }
            fn set_param(
                &mut self,
                target: (u8, u8),
                name: &str,
                value: f64,
                force: bool,
            ) -> Result<bool, Timeout> {
                self.0.os_lock().unwrap().set_param(target, name, value, force)
            }
            fn command(
                &mut self,
                target: (u8, u8),
                command: u16,
                params: [f32; 7],
                require_ack: bool,
            ) -> Result<bool, Timeout> {
                self.0
                    .os_lock()
                    .unwrap()
                    .command(target, command, params, require_ack)
            }
            fn set_wp_total(&mut self, target: (u8, u8), total: u16, kind: u8) -> Result<(), Timeout> {
                self.0.os_lock().unwrap().set_wp_total(target, total, kind)
            }
            fn set_wp(&mut self, target: (u8, u8), item: &WpItem) -> Result<u8, Timeout> {
                self.0.os_lock().unwrap().set_wp(target, item)
            }
            fn set_wp_ack(&mut self, target: (u8, u8), kind: u8) {
                self.0.os_lock().unwrap().set_wp_ack(target, kind);
            }
            fn set_wp_current(&mut self, target: (u8, u8), seq: u16) -> Result<bool, Timeout> {
                self.0.os_lock().unwrap().set_wp_current(target, seq)
            }
            fn get_wp(
                &mut self,
                target: (u8, u8),
                index: u16,
                kind: u8,
            ) -> Result<Locationwp, Timeout> {
                self.0.os_lock().unwrap().get_wp(target, index, kind)
            }
            fn set_position_target(
                &mut self,
                target: (u8, u8),
                position: &PositionTarget,
            ) -> bool {
                self.0.os_lock().unwrap().set_position_target(target, position)
            }
            fn write_raw(&mut self, bytes: &[u8]) -> bool {
                self.0.os_lock().unwrap().write_raw(bytes)
            }
            fn speak(&mut self, text: &str) {
                self.0.os_lock().unwrap().speak(text);
            }
            fn speech_settings(&self) -> (bool, bool) {
                self.0.os_lock().unwrap().speech_settings()
            }
        }
        let shared = Arc::new(Mutex::new(fake));
        let (result, printed) = run_blocking("test.py", source, Box::new(Through(shared.clone())));
        (shared, result, printed)
    }

    /// `print` reaches the console, and the `Script` methods reach the host with the C#'s
    /// semantics: `GetParam` 0.0 for a name not held, `ChangeMode` true, `SendRC` the channel
    /// and the send.
    #[test]
    fn script_methods_reach_the_host_and_print_reaches_the_console() {
        let mut fake = Fake::default();
        fake.params.insert("RC3_MIN".to_owned(), 1100.0);
        let (host, result, printed) = run(
            concat!(
                "print('Start Script')\n",
                "for chan in range(1, 9):\n",
                "    Script.SendRC(chan, 1500, False)\n",
                "Script.SendRC(3, Script.GetParam('RC3_MIN'), True)\n",
                "print(Script.GetParam('NOT_THERE'))\n",
                "print(Script.ChangeMode('AUTO'))\n",
                "print(Script.ChangeParam('FENCE_ENABLE', 1))\n",
                "Script.Sleep(50)\n",
            ),
            fake,
        );
        assert_eq!(result, Ok(()));
        assert_eq!(printed, "Start Script\n0.0\nTrue\nTrue\n");
        let host = host.os_lock().unwrap();
        assert_eq!(host.rc.len(), 9);
        assert_eq!(host.rc[8], (3, 1100, true));
        assert_eq!(host.modes, ["AUTO"]);
        assert_eq!(host.writes, [("FENCE_ENABLE".to_owned(), 1.0)]);
        assert_eq!(host.slept_ms, 50);
    }

    /// `WaitFor` is a substring match over the messages so far, false once the timeout has
    /// passed in 5 ms polls; `cs.messages.Clear()` empties them.
    #[test]
    fn wait_for_polls_the_messages_and_clear_empties_them() {
        let mut fake = Fake::default();
        fake.messages.push("ARMING MOTORS".to_owned());
        let (host, result, printed) = run(
            concat!(
                "print(Script.WaitFor('ARMING', 30000))\n",
                "cs.messages.Clear()\n",
                "print(Script.WaitFor('ARMING', 20))\n",
            ),
            fake,
        );
        assert_eq!(result, Ok(()));
        assert_eq!(printed, "True\nFalse\n");
        let host = host.os_lock().unwrap();
        assert_eq!(host.cleared, 1);
        // Five polls of 5 ms pass 20 ms.
        assert_eq!(host.slept_ms, 25);
    }

    /// `cs.<field>` is the host's state by the C#'s names, a name the state lacks an
    /// `AttributeError` naming it.
    #[test]
    fn cs_fields_are_read_by_name() {
        let mut fake = Fake::default();
        fake.fields
            .insert("lat".to_owned(), CsValue::Number(-35.363_262));
        fake.fields
            .insert("mode".to_owned(), CsValue::Text("Stabilize".to_owned()));
        fake.fields.insert("armed".to_owned(), CsValue::Flag(false));
        let (_, result, printed) = run(
            "print(cs.lat)\nprint(cs.mode)\nprint(cs.armed)\nprint(cs.nosuch)\n",
            fake,
        );
        assert_eq!(printed, "-35.363262\nStabilize\nFalse\n");
        let err = result.expect_err("nosuch");
        assert!(err.contains("AttributeError"), "{err}");
        assert!(err.contains("cs has no field 'nosuch'"), "{err}");
    }

    /// The objects this port does not hand over raise with their name, and a syntax error -
    /// a Python 2 `print` - is the exception's text, as "Error running script" shows it.
    #[test]
    fn unported_objects_and_syntax_errors_are_reported() {
        let (_, result, _) = run("MAV.getParamList()\n", Fake::default());
        let err = result.expect_err("MAV");
        assert!(
            err.contains("RuntimeError: MAV.getParamList is not available to scripts in this version"),
            "{err}"
        );
        let (_, result, _) = run("print 'Start Script'\n", Fake::default());
        let err = result.expect_err("python 2");
        assert!(err.contains("SyntaxError"), "{err}");
        let (_, result, printed) = run("print(mavutil.recv_match(type='HEARTBEAT', blocking=True))\n", Fake::default());
        assert_eq!(result, Ok(()));
        assert_eq!(printed, "None\n");
    }

    /// A run on its thread: the output comes as it is printed, Abort stops a script at its next
    /// `Sleep`, and the run reports it ended without an error.
    #[test]
    fn a_run_on_its_thread_streams_output_and_stops_at_abort() {
        let mut run = ScriptRun::start(
            "loop.py",
            "print('going')\nwhile True:\n    Script.Sleep(1000)\n".to_owned(),
            Box::new(Fake::default()),
        );
        let deadline = web_time::Instant::now() + Duration::from_secs(20);
        while run.output().is_empty() && web_time::Instant::now() < deadline {
            wasm_thread::sleep(Duration::from_millis(10));
        }
        assert_eq!(run.take_output(), "going\n");
        assert_eq!(run.take_output(), "");
        assert!(run.is_running());
        run.abort();
        run.join();
        assert!(!run.is_running());
        assert_eq!(run.result(), Some(Ok(())));
    }

    /// The last line of a failed run: the exception as "Error running script" ends with it.
    fn last_line(result: &Result<(), String>) -> String {
        match result {
            Ok(()) => "ok".to_owned(),
            Err(text) => text.lines().last().unwrap_or("").trim().to_owned(),
        }
    }

    /// Every .NET name the shipped scripts reach that the shim answers resolves after
    /// `import <root>`, as IronPython's loaded assemblies make it resolve.
    #[test]
    fn every_clr_name_the_shim_answers_resolves() {
        for name in CLR_NAMES {
            let root = name.split('.').next().unwrap_or("");
            let (_, result, _) = run(&format!("import {root}\nprint({name})\n"), Fake::default());
            assert_eq!(result, Ok(()), "{name}");
        }
    }

    /// `import clr` and the imports the corpus makes work; a .NET name the shim does not give
    /// stops the script there, named, whether imported, taken by a `from` or reached as an
    /// attribute.
    #[test]
    fn the_clr_imports_work_and_a_name_not_given_is_named() {
        let (_, result, printed) = run(
            concat!(
                "import clr\n",
                "clr.AddReference('MissionPlanner')\n",
                "clr.AddReference('System.Drawing, Version=4.0.0.0, Culture=neutral')\n",
                "clr.ClearProfilerData()\n",
                "import MissionPlanner\n",
                "import MissionPlanner.Comms\n",
                "from MissionPlanner.Utilities import Locationwp\n",
                "import MAVLink\n",
                "from MAVLink import mavlink_command_long_t\n",
                "from System import Byte, Func, Action, Array\n",
                "print(MissionPlanner.Utilities.Locationwp is Locationwp)\n",
                "print(MissionPlanner.MainV2 is MainV2)\n",
            ),
            Fake::default(),
        );
        assert_eq!(result, Ok(()));
        assert_eq!(printed, "True\nTrue\n");
        for (source, named) in [
            (
                "from System.Windows.Forms import Form\n",
                "System.Windows.Forms",
            ),
            ("import System.Diagnostics\n", "System.Diagnostics"),
            ("from System import DateTime\n", "System.DateTime"),
            ("import MissionPlanner\nMissionPlanner.Comms.TcpSerial()\n", "MissionPlanner.Comms.TcpSerial"),
            ("import clr\nclr.References\n", "clr.References"),
            ("MAV.OnPacketReceived += print\n", "MAV.OnPacketReceived"),
            ("MainV2.instance.FlightPlanner.BUT_read_Click(None, None)\n", "FlightPlanner.BUT_read_Click"),
            ("FlightData.gMapControl1\n", "FlightData.gMapControl1"),
            ("MAV.BaseStream.BaudRate\n", "MAV.BaseStream.BaudRate"),
        ] {
            let (_, result, _) = run(source, Fake::default());
            assert_eq!(
                last_line(&result),
                format!("RuntimeError: {named} is not available to scripts in this version"),
                "{source}"
            );
        }
    }

    /// `Locationwp` is a struct: `Set` fills lat, lng, alt, id and frame 3 and hands back a copy;
    /// `Locationwp.x.SetValue(item, v)` changes the item; `item.x = v` is IronPython's refusal
    /// for a value type; the fields keep the C#'s types - `alt` and `p1` 32-bit floats, `id` a
    /// ushort.
    /// `// C#: ExtLibs/Utilities/locationwp.cs:13-22, 199-210`
    #[test]
    fn locationwp_behaves_as_the_c_sharp_struct() {
        let (_, result, printed) = run(
            concat!(
                "from MissionPlanner.Utilities import Locationwp\n",
                "import MAVLink\n",
                "home = Locationwp().Set(-34.9805, 117.8518, 0.1, int(MAVLink.MAV_CMD.WAYPOINT))\n",
                "print(home.lat, home.lng, home.alt, home.id, home.frame)\n",
                "to = Locationwp()\n",
                "print(to.lat, to.id, to.frame, to.p1, to.Tag)\n",
                "Locationwp.id.SetValue(to, int(MAVLink.MAV_CMD.TAKEOFF))\n",
                "Locationwp.p1.SetValue(to, 15)\n",
                "Locationwp.alt.SetValue(to, 50)\n",
                "print(to.id, to.p1, to.alt, Locationwp.alt.GetValue(to))\n",
                "try:\n",
                "    to.lat = 1\n",
                "except ValueError as e:\n",
                "    print(e)\n",
                "try:\n",
                "    Locationwp.id.SetValue(to, 70000)\n",
                "except OverflowError as e:\n",
                "    print(e)\n",
            ),
            Fake::default(),
        );
        assert_eq!(result, Ok(()));
        assert_eq!(
            printed,
            concat!(
                "-34.9805 117.8518 0.10000000149011612 16 3\n",
                "0.0 0 0 0.0 None\n",
                "22 15.0 50.0 50.0\n",
                "Attempt to update field 'lat' on value type 'Locationwp'; value type fields ",
                "cannot be directly modified\n",
                "Value was either too large or too small for a UInt16.\n",
            )
        );
    }

    /// The MAVLink enums are the C#'s: a member is its number (`int()`, `.value__`) and its
    /// name as text; a name the C# does not have is an AttributeError.
    #[test]
    fn the_mavlink_enums_are_the_c_sharp_ones() {
        let (_, result, printed) = run(
            concat!(
                "import MAVLink\n",
                "print(int(MAVLink.MAV_CMD.WAYPOINT), int(MAVLink.MAV_CMD.TAKEOFF))\n",
                "print(MAVLink.MAV_CMD.DO_DIGICAM_CONTROL.value__, MAVLink.MAV_CMD.DO_DIGICAM_CONTROL)\n",
                "print(int(MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT), MAVLink.MAVLINK_MSG_ID.STATUSTEXT.value__)\n",
                "print(MAVLink.MAV_MOUNT_MODE.NEUTRAL.value__, MAVLink.MAVLINK_MSG_ID.HEARTBEAT.value__)\n",
                "MAVLink.MAVLINK_MSG_ID.STATUSTEXT_LONG\n",
            ),
            Fake::default(),
        );
        assert_eq!(printed, "16 22\n203 DO_DIGICAM_CONTROL\n3 253\n1 0\n");
        assert_eq!(
            last_line(&result),
            "AttributeError: type object 'MAVLINK_MSG_ID' has no attribute 'STATUSTEXT_LONG'"
        );
    }

    /// The mission members reach the host as `MAVLinkInterface` makes them: `setWPTotal`'s
    /// count, `setWP`'s `MISSION_ITEM` in the frame passed (not the item's own), its answer a
    /// `MAV_MISSION_RESULT`, `setWPACK`, `getWP` into a `Locationwp`, and `setWPCurrent` with its
    /// three arguments - one argument is IronPython's TypeError.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:2434-2501, 3398-3564, 3753-4235`
    #[test]
    fn the_mission_members_reach_the_host() {
        let (host, result, printed) = run(
            concat!(
                "from MissionPlanner.Utilities import Locationwp\n",
                "import MAVLink\n",
                "to = Locationwp()\n",
                "Locationwp.id.SetValue(to, int(MAVLink.MAV_CMD.TAKEOFF))\n",
                "Locationwp.p1.SetValue(to, 15)\n",
                "Locationwp.alt.SetValue(to, 50)\n",
                "MAV.setWPTotal(2)\n",
                "print(MAV.setWP(Locationwp().Set(-35, 117.8, 50, 16), 0, MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT))\n",
                "print(int(MAV.setWP(to, 1, MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT)))\n",
                "MAV.setWPACK()\n",
                "wp = MAV.getWP(1)\n",
                "print(wp.id, wp.p1, wp.alt, wp.frame)\n",
                "print(MAV.setWPCurrent(MAV.sysidcurrent, MAV.compidcurrent, 1))\n",
                "MAV.setWPCurrent(1)\n",
            ),
            Fake::default(),
        );
        assert_eq!(printed, "MAV_MISSION_ACCEPTED\n0\n22 15.0 50.0 3\nTrue\n");
        assert_eq!(
            last_line(&result),
            "TypeError: setWPCurrent() takes exactly 3 arguments (1 given)"
        );
        let host = host.os_lock().unwrap();
        assert_eq!(host.calls[0], "setWPTotal (1, 1) 2 0");
        assert!(host.calls[1].starts_with("setWP (1, 1) WpItem { seq: 0, frame: 3, command: 16, current: 0, autocontinue: 1, params: [0.0, 0.0, 0.0, 0.0], x: -35.0, y: 117.8, z: 50.0, mission_type: 0 }"), "{}", host.calls[1]);
        assert!(host.calls[2].contains("seq: 1, frame: 3, command: 22"), "{}", host.calls[2]);
        assert!(host.calls[2].contains("params: [15.0, 0.0, 0.0, 0.0], x: 0.0, y: 0.0, z: 50.0"), "{}", host.calls[2]);
        assert_eq!(host.calls[3], "setWPACK (1, 1) 0");
        assert_eq!(host.calls[4], "getWP (1, 1) 1 0");
        assert_eq!(host.calls[5], "setWPCurrent (1, 1) 1");
        assert_eq!(host.calls.len(), 6);
    }

    /// A vehicle that never answers: the member's `TimeoutException`, with the C#'s message.
    #[test]
    fn a_silent_vehicle_is_the_c_sharp_timeout() {
        for (source, member) in [
            ("MAV.setWPTotal(3)\n", "setWPTotal"),
            ("MAV.getWP(0)\n", "getWP"),
            ("MAV.doARM(True)\n", "doCommand"),
            ("MAV.setParam('X', 1)\n", "setParam X"),
        ] {
            let fake = Fake {
                silent: true,
                ..Fake::default()
            };
            let (_, result, _) = run(source, fake);
            assert_eq!(
                last_line(&result),
                format!("TimeoutException: Timeout on read - {member}"),
                "{source}"
            );
        }
        // And a script can catch it by the C#'s name.
        let fake = Fake {
            silent: true,
            ..Fake::default()
        };
        let (_, result, printed) = run(
            "import System\ntry:\n    MAV.getWP(0)\nexcept System.TimeoutException as e:\n    print('caught', e)\n",
            fake,
        );
        assert_eq!(result, Ok(()));
        assert_eq!(printed, "caught Timeout on read - getWP\n");
    }

    /// The command members: `doCommand` the seven params as floats, `doARM` 400 with 1 or 0 and
    /// the magic force numbers, `doReboot` 246 with 1 (3 into the bootloader), `setParam` the
    /// (name, value, force) overload.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1622-1626, 2553-2718`
    #[test]
    fn the_command_members_reach_the_host() {
        let (host, result, printed) = run(
            concat!(
                "import MAVLink\n",
                "print(MAV.doCommand(MAVLink.MAV_CMD.TAKEOFF, 0, 0, 0, 0, 0, 0, 100))\n",
                "print(MAV.doARM(True))\n",
                "MAV.doARM(False, True)\n",
                "MAV.doReboot()\n",
                "MAV.doReboot(True)\n",
                "print(MAV.setParam('INS_ACC_ID', 3081250, True))\n",
                "MAV.setParam('FENCE_ENABLE', 1.0)\n",
            ),
            Fake::default(),
        );
        assert_eq!(result, Ok(()));
        assert_eq!(printed, "True\nTrue\nTrue\n");
        let host = host.os_lock().unwrap();
        assert_eq!(
            host.calls,
            [
                "doCommand (1, 1) 22 [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0] true",
                "doCommand (1, 1) 400 [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] true",
                "doCommand (1, 1) 400 [0.0, 21196.0, 0.0, 0.0, 0.0, 0.0, 0.0] true",
                "doCommand (1, 1) 246 [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] true",
                "doCommand (1, 1) 246 [3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] true",
                "setParam (1, 1) INS_ACC_ID 3081250 true",
                "setParam (1, 1) FENCE_ENABLE 1 false",
            ]
        );
    }

    /// `setGuidedModeWP`: nothing for an item with a zero latitude, longitude or altitude;
    /// GUIDED asked for unless the vehicle is in it; a copter the position target in the item's
    /// own frame, a plane `setWP` with current 2.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:4416-4460`
    #[test]
    fn set_guided_mode_wp_is_the_c_sharp_one() {
        let script = concat!(
            "from MissionPlanner.Utilities import Locationwp\n",
            "item = Locationwp()\n",
            "Locationwp.lat.SetValue(item, -35.36)\n",
            "Locationwp.lng.SetValue(item, 149.16)\n",
            "MAV.setGuidedModeWP(item)\n",
            "Locationwp.alt.SetValue(item, 60)\n",
            "MAV.setGuidedModeWP(item)\n",
            "print(item.id)\n",
        );
        let mut copter = Fake::default();
        copter
            .fields
            .insert("mode".to_owned(), CsValue::Text("Stabilize".to_owned()));
        copter
            .fields
            .insert("firmware".to_owned(), CsValue::Text("ArduCopter2".to_owned()));
        let (host, result, printed) = run(script, copter);
        assert_eq!(result, Ok(()));
        // The script's item keeps its id: the C#'s struct was a copy.
        assert_eq!(printed, "0\n");
        let host = host.os_lock().unwrap();
        assert_eq!(host.modes, ["GUIDED"]);
        assert_eq!(
            host.calls,
            ["setPositionTarget (1, 1) PositionTarget { frame: 0, lat: -35.36, lng: 149.16, alt: 60.0 }"]
        );
        drop(host);

        let mut plane = Fake::default();
        plane
            .fields
            .insert("mode".to_owned(), CsValue::Text("Guided".to_owned()));
        plane
            .fields
            .insert("firmware".to_owned(), CsValue::Text("ArduPlane".to_owned()));
        let (host, result, _) = run(script, plane);
        assert_eq!(result, Ok(()));
        let host = host.os_lock().unwrap();
        assert!(host.modes.is_empty());
        assert_eq!(host.calls.len(), 1);
        assert!(
            host.calls[0].contains("seq: 0, frame: 0, command: 16, current: 2"),
            "{}",
            host.calls[0]
        );
    }

    /// `MAV.BaseStream`: `IsOpen`, and `Write` the bytes of a `byte[]` from `Array[Byte]` as
    /// they are.
    #[test]
    fn the_base_stream_writes_raw_bytes() {
        let (host, result, printed) = run(
            concat!(
                "from System import Byte, Array\n",
                "key = Array[Byte]([0x13, 0x00, 0x00, 0x00, 0x08, 0x00])\n",
                "print(MAV.BaseStream.IsOpen, len(key))\n",
                "MAV.BaseStream.Write(key, 1, 4)\n",
                "Array[Byte]([256])\n",
            ),
            Fake::default(),
        );
        assert_eq!(printed, "True 6\n");
        assert_eq!(
            last_line(&result),
            "OverflowError: Value was either too large or too small for an unsigned byte."
        );
        assert_eq!(host.os_lock().unwrap().calls, ["Write [0, 0, 0, 8]"]);
    }

    /// `SubscribeToPacketType` takes four or five arguments; the corpus's two are IronPython's
    /// TypeError. `Func[...]` wraps a Python callable.
    #[test]
    fn subscribe_to_packet_type_takes_four_arguments() {
        let (_, result, printed) = run(
            concat!(
                "import MAVLink\n",
                "from System import Func\n",
                "def handler(message):\n",
                "    return True\n",
                "f = Func[MAVLink.MAVLinkMessage, bool](handler)\n",
                "print(f(None), f.Invoke(None))\n",
                "MAV.SubscribeToPacketType(MAVLink.MAVLINK_MSG_ID.HEARTBEAT.value__, f)\n",
            ),
            Fake::default(),
        );
        assert_eq!(printed, "True True\n");
        assert_eq!(
            last_line(&result),
            "TypeError: SubscribeToPacketType() takes at least 4 arguments (2 given)"
        );
        let (_, result, _) = run(
            "MAV.SubscribeToPacketType(0, None, 1, 1)\n",
            Fake::default(),
        );
        assert_eq!(
            last_line(&result),
            "RuntimeError: MAV.SubscribeToPacketType is not available to scripts in this version"
        );
    }

    /// `MainV2`: `comPort` is `MAV`, `Comports` and `Ports` the one list holding it,
    /// `Joystick` none; speech off - the settings off - until `speechEnable` is set, then
    /// `SpeakAsync`'s rewording handed to the host; `cs` numbers have `ToString()`.
    /// `// C#: MainV2.cs:401-491; Utilities/Speech.cs:65-80`
    #[test]
    fn main_v2_speech_and_the_scope_objects() {
        let mut fake = Fake::default();
        fake.fields.insert("roll".to_owned(), CsValue::Number(0.0));
        fake.fields.insert("alt".to_owned(), CsValue::Number(12.5));
        fake.fields.insert("big".to_owned(), CsValue::Number(1e20));
        let (host, result, printed) = run(
            concat!(
                "import MissionPlanner\n",
                "print(MainV2.comPort is MAV, MainV2.Comports is Ports, Ports[0] is MAV, Joystick)\n",
                "print(MissionPlanner.MainV2.instance is MainV2, MainV2.speechEnable)\n",
                "MissionPlanner.MainV2.speechEngine.SpeakAsync('not spoken')\n",
                "MissionPlanner.MainV2.speechEnable = True\n",
                "print(MainV2.speechEnable)\n",
                "MissionPlanner.MainV2.speechEngine.SpeakAsync('test ' + cs.roll.ToString())\n",
                "MissionPlanner.MainV2.speechEngine.SpeakAsync('PreArm: 5m to NAV dist')\n",
                "MissionPlanner.MainV2.speechEngine.SpeakAsync('   ')\n",
                "print(cs.alt.ToString(), cs.big.ToString(), cs.alt + 1)\n",
                "MainV2.nosuch\n",
            ),
            fake,
        );
        assert_eq!(
            printed,
            "True True True None\nTrue False\nTrue\n12.5 1E+20 13.5\n"
        );
        assert_eq!(
            last_line(&result),
            "RuntimeError: MainV2.nosuch is not available to scripts in this version"
        );
        assert_eq!(
            host.os_lock().unwrap().calls,
            ["Speak test 0", "Speak Pre Arm: 5 meters to Navigation distance"]
        );
    }

    /// Speech starts as the user's settings have it: "speechenable" on speaks from the first
    /// line, and "speech_armed_only" keeps it quiet while the vehicle is disarmed.
    /// `// C#: MainV2.cs:469-481, 658, 1005-1006`
    #[test]
    fn speech_starts_from_the_users_settings() {
        let script = concat!(
            "print(MainV2.speechEnable, MainV2.speech_armed_only)\n",
            "MainV2.speechEngine.SpeakAsync('hello')\n",
        );
        let (host, result, printed) = run(
            script,
            Fake {
                speech: (true, false),
                ..Fake::default()
            },
        );
        assert_eq!(result, Ok(()));
        assert_eq!(printed, "True False\n");
        assert_eq!(host.os_lock().unwrap().calls, ["Speak hello"]);

        let mut disarmed = Fake {
            speech: (true, true),
            ..Fake::default()
        };
        disarmed
            .fields
            .insert("armed".to_owned(), CsValue::Flag(false));
        let (host, result, printed) = run(script, disarmed);
        assert_eq!(result, Ok(()));
        assert_eq!(printed, "True True\n");
        assert!(host.os_lock().unwrap().calls.is_empty());
    }

    /// `cs` numbers are their C# types: a `float` member's `ToString()` is .NET Framework's
    /// Single "G7", a `double` member's "G15", an integer member's an integer.
    /// `// C#: ExtLibs/ArduPilot/CurrentState.cs:264, 315, 1295, 1498`
    #[test]
    fn cs_numbers_are_their_c_sharp_types() {
        let mut fake = Fake::default();
        fake.fields
            .insert("roll".to_owned(), CsValue::Number(1.234_567_89));
        fake.fields
            .insert("lat".to_owned(), CsValue::Number(-35.363_262_123_4));
        fake.fields
            .insert("battery_remaining".to_owned(), CsValue::Number(87.0));
        fake.fields
            .insert("alt".to_owned(), CsValue::Number(12_345_678.0));
        let (_, result, printed) = run(
            concat!(
                "print(cs.roll.ToString(), cs.lat.ToString(), cs.alt.ToString())\n",
                "print(cs.battery_remaining.ToString(), cs.battery_remaining)\n",
            ),
            fake,
        );
        assert_eq!(result, Ok(()));
        assert_eq!(printed, "1.234568 -35.3632621234 1.234568E+07\n87 87\n");
    }

    /// Each member takes its current overload on a named vehicle as well as the [Obsolete] one
    /// on the vehicle flown, and `setParam` its list of names, each tried until one is set.
    /// `// C#: ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs:1602-1631, 2433-2450, 2621-2685,
    /// 3397-3407, 3974-3988, 4417-4461`
    #[test]
    fn the_members_take_their_overloads_on_a_named_vehicle() {
        let mut fake = Fake::default();
        fake.fields
            .insert("mode".to_owned(), CsValue::Text("Guided".to_owned()));
        fake.fields
            .insert("firmware".to_owned(), CsValue::Text("ArduCopter2".to_owned()));
        let (host, result, printed) = run(
            concat!(
                "import MAVLink\n",
                "from MissionPlanner.Utilities import Locationwp\n",
                "item = Locationwp().Set(-35.36, 149.16, 50, 16)\n",
                "print(MAV.setParam(2, 1, 'RTL_ALT', 1500))\n",
                "print(MAV.setParam(['NOPE', 'RTL_ALT'], 1500))\n",
                "print(MAV.doCommand(2, 1, MAVLink.MAV_CMD.TAKEOFF, 0, 0, 0, 0, 0, 0, 10, False))\n",
                "print(MAV.doARM(2, 1, True, True))\n",
                "print(MAV.setWP(2, 1, item, 3, MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT))\n",
                "print(MAV.getWP(2, 1, 3).alt, MAV.getWP(3, MAVLink.MAV_MISSION_TYPE.FENCE).alt)\n",
                "MAV.setWPACK(2, 1, MAVLink.MAV_MISSION_TYPE.RALLY)\n",
                "MAV.setWPACK()\n",
                "MAV.setGuidedModeWP(2, 1, item)\n",
                "MAV.doARM(1, 2, 3, 4, 5)\n",
            ),
            fake,
        );
        assert_eq!(printed, "True\nTrue\nTrue\nTrue\nMAV_MISSION_ACCEPTED\n50.0 50.0\n");
        assert_eq!(
            last_line(&result),
            "TypeError: doARM() takes at most 4 arguments (5 given)"
        );
        let host = host.os_lock().unwrap();
        let calls: Vec<&str> = host.calls.iter().map(String::as_str).collect();
        assert_eq!(calls[0], "setParam (2, 1) RTL_ALT 1500 false");
        assert_eq!(calls[1], "setParam (1, 1) NOPE 1500 false");
        assert_eq!(
            calls[2],
            "doCommand (2, 1) 22 [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0] false"
        );
        assert_eq!(
            calls[3],
            "doCommand (2, 1) 400 [1.0, 2989.0, 0.0, 0.0, 0.0, 0.0, 0.0] true"
        );
        assert!(calls[4].starts_with("setWP (2, 1) WpItem { seq: 3, frame: 3"), "{}", calls[4]);
        assert_eq!(&calls[5..7], ["getWP (2, 1) 3 0", "getWP (1, 1) 3 1"]);
        assert_eq!(&calls[7..9], ["setWPACK (2, 1) 2", "setWPACK (1, 1) 0"]);
        assert!(calls[9].starts_with("setPositionTarget (2, 1)"), "{}", calls[9]);
        assert_eq!(calls.len(), 10);
    }

    /// The Abort button ends a script waiting on the link: the host gives the wait up and the
    /// script stops there, `SystemExit` - not the `TimeoutException` the script would catch.
    #[test]
    fn abort_ends_a_script_waiting_on_the_link() {
        let abort = Arc::new(AtomicBool::new(false));
        let started = Arc::new(AtomicBool::new(false));
        let fake = Fake {
            get_wp_waits: Some((Arc::clone(&abort), Arc::clone(&started))),
            ..Fake::default()
        };
        let mut run = ScriptRun::start_with_abort(
            "wait.py",
            concat!(
                "import System\n",
                "try:\n",
                "    MAV.getWP(0)\n",
                "except System.TimeoutException:\n",
                "    print('caught')\n",
                "print('after')\n",
            )
            .to_owned(),
            Box::new(fake),
            Arc::clone(&abort),
        );
        let deadline = web_time::Instant::now() + Duration::from_secs(20);
        while !started.load(Ordering::Relaxed) {
            assert!(web_time::Instant::now() < deadline, "getWP never called");
            wasm_thread::sleep(Duration::from_millis(5));
        }
        assert!(run.is_running());
        run.abort();
        run.join();
        assert_eq!(run.result(), Some(Ok(())));
        assert_eq!(run.output(), "");
    }

    /// `time.sleep` is one of the waits: the host's clock, and the Abort button seen there.
    #[test]
    fn time_sleep_is_a_wait_the_abort_reaches() {
        let (host, result, _) = run("import time\ntime.sleep(0.25)\n", Fake::default());
        assert_eq!(result, Ok(()));
        assert_eq!(host.os_lock().unwrap().slept_ms, 250);
        let mut run = ScriptRun::start(
            "loop.py",
            "import time\nprint('going')\nwhile True:\n    time.sleep(1)\n".to_owned(),
            Box::new(Fake::default()),
        );
        let deadline = web_time::Instant::now() + Duration::from_secs(20);
        while run.output().is_empty() && web_time::Instant::now() < deadline {
            wasm_thread::sleep(Duration::from_millis(10));
        }
        run.abort();
        run.join();
        assert_eq!(run.result(), Some(Ok(())));
    }
}
