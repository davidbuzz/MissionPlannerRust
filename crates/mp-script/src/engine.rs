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
//!   a list with `Clear`; `MAV`, `MainV2`, `FlightPlanner`, `FlightData`, `Ports` and `Joystick`
//!   are [`PyUnported`], which raises with the object's name on any use - a written divergence,
//!   the C# handing scripts the whole application over .NET;
//! * `mavutil` is the `Script` object, as the C# binds it, with `mavlink_connection` and
//!   `recv_match` returning `None` as the C#'s return `null` (`Script.cs:92-102`);
//! * output always goes to the run's buffer, which the console shows; the C# writes to the
//!   process's stdout when the box is unticked, which a desktop application has nowhere to show;
//! * a run has an abort flag that `Sleep` and `WaitFor` read: the C#'s `Thread.Abort` stops a
//!   script anywhere, this stops it at its next wait - a script that never waits runs on;
//! * RustPython is Python 3 and IronPython 2.7 is Python 2: `print 'x'` is a syntax error here,
//!   which is why the shipped scripts are converted (PLAN.md §12 D20).

use std::fmt;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, PoisonError};
use std::thread::JoinHandle;

use rustpython::InterpreterBuilderExt;
use rustpython_vm::builtins::{PyBaseExceptionRef, PyStr, PyStrRef};
use rustpython_vm::function::{ArgIntoFloat, FuncArgs};
use rustpython_vm::types::GetAttr;
use rustpython_vm::{
    AsObject, InterpreterBuilder, Py, PyObjectRef, PyPayload, PyResult, Settings, VirtualMachine,
    pyclass,
};

use crate::api::{CsValue, ScriptApi, ScriptHost, WAIT_FOR_POLL_MS};

/// The thread's name, `FlightData.cs:790`.
pub const THREAD_NAME: &str = "Script Thread (new)";

/// How long one slice of a `Sleep` is, so an abort is seen within it.
const SLEEP_SLICE_MS: u32 = 20;

/// What the C# says of an object a script reaches that this port does not hand over.
fn unported_text(name: &str) -> String {
    format!("{name} is not available to scripts in this version")
}

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
    shared.lock().unwrap_or_else(PoisonError::into_inner)
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
    Err(vm.new_exception_msg(
        vm.ctx.exceptions.system_exit.to_owned(),
        "script aborted".into(),
    ))
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

/// An object the C# hands scripts and this port does not: `MAV`, `MainV2`, `FlightPlanner`,
/// `FlightData`, `Ports`, `Joystick`. Any attribute raises with the name, so a script's failure
/// says what it reached for.
#[pyclass(module = false, name = "Unported")]
#[derive(Debug, PyPayload)]
pub struct PyUnported {
    name: &'static str,
}

#[pyclass(with(GetAttr))]
impl PyUnported {}

impl GetAttr for PyUnported {
    fn getattro(zelf: &Py<Self>, _name: &Py<PyStr>, vm: &VirtualMachine) -> PyResult {
        Err(vm.new_runtime_error(unported_text(zelf.name)))
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
            .lock()
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
    make::<PyUnported>();
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
        let scope = vm.new_scope_with_builtins();
        // `// C#: Script.cs:45-53`
        let script = PyScript {
            shared: Arc::clone(&shared),
        }
        .into_pyobject(vm);
        let bindings: [(&str, PyObjectRef); 9] = [
            ("MainV2", PyUnported { name: "MainV2" }.into_pyobject(vm)),
            (
                "FlightPlanner",
                PyUnported {
                    name: "FlightPlanner",
                }
                .into_pyobject(vm),
            ),
            ("FlightData", PyUnported { name: "FlightData" }.into_pyobject(vm)),
            ("Ports", PyUnported { name: "Ports" }.into_pyobject(vm)),
            ("MAV", PyUnported { name: "MAV" }.into_pyobject(vm)),
            (
                "cs",
                PyCurrentState {
                    shared: Arc::clone(&shared),
                }
                .into_pyobject(vm),
            ),
            ("Script", script.clone()),
            ("mavutil", script),
            ("Joystick", PyUnported { name: "Joystick" }.into_pyobject(vm)),
        ];
        for (binding, object) in bindings {
            scope
                .globals
                .set_item(binding, object, vm)
                .map_err(describe)?;
        }
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
        .lock()
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
        let output = Arc::new(Mutex::new(String::new()));
        let abort = Arc::new(AtomicBool::new(false));
        let result: Arc<Mutex<Option<Result<(), String>>>> = Arc::new(Mutex::new(None));
        let name = name.to_owned();
        let thread = {
            let output = Arc::clone(&output);
            let abort = Arc::clone(&abort);
            let result = Arc::clone(&result);
            std::thread::Builder::new()
                .name(THREAD_NAME.to_owned())
                .spawn(move || {
                    let outcome = run_with(&name, &source, host, output, abort);
                    *result.lock().unwrap_or_else(PoisonError::into_inner) = Some(outcome);
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
            *run.result.lock().unwrap_or_else(PoisonError::into_inner) =
                Some(Err("the script thread could not be started".to_owned()));
        }
        run
    }

    /// `scriptrunning`: whether the thread is still going.
    #[must_use]
    pub fn is_running(&self) -> bool {
        self.result
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .is_none()
    }

    /// `BUT_abort_script_Click`: the script stopped at its next `Sleep` or `WaitFor`.
    pub fn abort(&self) {
        self.abort.store(true, Ordering::Relaxed);
    }

    /// `RetrieveWrittenString`: what was printed since the last call.
    pub fn take_output(&mut self) -> String {
        let output = self.output.lock().unwrap_or_else(PoisonError::into_inner);
        let fresh = output.get(self.taken..).unwrap_or("").to_owned();
        self.taken = output.len();
        fresh
    }

    /// Everything printed so far.
    #[must_use]
    pub fn output(&self) -> String {
        self.output
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .clone()
    }

    /// How the run ended, once it has: `Ok` for a script that finished or was aborted, `Err`
    /// with the exception's text for one that threw.
    #[must_use]
    pub fn result(&self) -> Option<Result<(), String>> {
        self.result
            .lock()
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
    }

    impl ScriptHost for Fake {
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
            std::thread::sleep(Duration::from_millis(1));
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
                self.0.lock().unwrap().get_parameter(name)
            }
            fn change_param(&mut self, name: &str, value: f32) -> bool {
                self.0.lock().unwrap().change_param(name, value)
            }
            fn change_mode(&mut self, mode: &str) -> bool {
                self.0.lock().unwrap().change_mode(mode)
            }
            fn has_message(&self, text: &str) -> bool {
                self.0.lock().unwrap().has_message(text)
            }
            fn clear_messages(&mut self) {
                self.0.lock().unwrap().clear_messages();
            }
            fn cs_field(&self, name: &str) -> Option<CsValue> {
                self.0.lock().unwrap().cs_field(name)
            }
            fn send_rc(&mut self, channel: u8, pwm: u16, send_now: bool) -> bool {
                self.0.lock().unwrap().send_rc(channel, pwm, send_now)
            }
            fn sleep(&mut self, milliseconds: u32) {
                self.0.lock().unwrap().sleep(milliseconds);
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
        let host = host.lock().unwrap();
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
        let host = host.lock().unwrap();
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
        let (_, result, _) = run("MAV.setParam('X', 1)\n", Fake::default());
        let err = result.expect_err("MAV");
        assert!(err.contains("MAV is not available to scripts in this version"), "{err}");
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
        let deadline = std::time::Instant::now() + Duration::from_secs(20);
        while run.output().is_empty() && std::time::Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(10));
        }
        assert_eq!(run.take_output(), "going\n");
        assert_eq!(run.take_output(), "");
        assert!(run.is_running());
        run.abort();
        run.join();
        assert!(!run.is_running());
        assert_eq!(run.result(), Some(Ok(())));
    }
}
