//! The AT command layer: `RFDLib`'s token waits and AT client (`SikRadio/RFDLib/IO/ATCommand.cs`,
//! `SikRadio/RFDLib/IO/SerialPort/SerialPort.cs`), `TSession`'s own token wait
//! (`SikRadio/RFD900.cs:94-137`), and the page's `doCommand` (`Radio/Sikradio.cs:1938-2000`),
//! which the page's handlers use beside the client and which differs from it: it sends a bare
//! line first, reads whole lines, and tries once more when the echo or the answer is missing.
//!
//! The C#'s `TimeoutException` from a read is `None` here; a port that fails is an `io::Error`,
//! as the C# throws.

use std::io;
use std::time::Duration;

use crate::{Port, Wire};

/// `TSerialPort.WaitForToken`: `ReadExisting` every 50 ms until what has come holds `token`, or
/// `max_wait_ms` passes. Whether it came, and everything read. `ReadTimeout` is set to the wait
/// for its length and put back.
/// `// C#: SikRadio/RFDLib/IO/SerialPort/SerialPort.cs:55-89`
///
/// # Errors
/// The port's.
pub fn wait_for_token<P: Port>(
    wire: &mut Wire<P>,
    token: Option<&str>,
    max_wait_ms: u64,
) -> io::Result<(bool, String)> {
    let max_wait = Duration::from_millis(max_wait_ms);
    let start = wire.now();
    let timeout = wire.read_timeout();
    wire.set_read_timeout(max_wait);
    let mut temp = String::new();
    while wire.now().saturating_sub(start) < max_wait {
        temp.push_str(&wire.read_existing()?);
        if let Some(token) = token
            && temp.contains(token)
        {
            wire.set_read_timeout(timeout);
            return Ok((true, temp));
        }
        wire.sleep(50);
    }
    wire.set_read_timeout(timeout);
    Ok((false, temp))
}

/// `TSession.WaitForAnyOfTheseTokens`: bytes read one at a time, each within the whole wait,
/// until one of the tokens is in what has come - its index - or a read times out or the wait
/// passes (`None`). A read that times out leaves `ReadTimeout` at the wait, as the C#'s `catch`
/// returns before putting it back; the byte read once the wait has passed is dropped, as the
/// C#'s loop condition drops it.
/// `// C#: SikRadio/RFD900.cs:94-137`
///
/// # Errors
/// The port's.
pub fn wait_for_any_token<P: Port>(
    wire: &mut Wire<P>,
    tokens: &[&str],
    max_wait_ms: u64,
) -> io::Result<Option<usize>> {
    let max_wait = Duration::from_millis(max_wait_ms);
    let start = wire.now();
    let timeout = wire.read_timeout();
    wire.set_read_timeout(max_wait);
    let mut temp = String::new();
    while wire.now().saturating_sub(start) < max_wait {
        loop {
            let Some(byte) = wire.read_byte()? else {
                return Ok(None);
            };
            if wire.now().saturating_sub(start) >= max_wait {
                break;
            }
            temp.push(char::from(byte));
            if let Some(index) = tokens.iter().position(|token| temp.contains(token)) {
                wire.set_read_timeout(timeout);
                return Ok(Some(index));
            }
        }
        wire.sleep(50);
    }
    wire.set_read_timeout(timeout);
    Ok(None)
}

/// `TSession.WaitForToken`.
/// `// C#: SikRadio/RFD900.cs:81-84`
///
/// # Errors
/// The port's.
pub fn wait_for<P: Port>(wire: &mut Wire<P>, token: &str, max_wait_ms: u64) -> io::Result<bool> {
    Ok(wait_for_any_token(wire, &[token], max_wait_ms)?.is_some())
}

/// The AT command client: `RFDLib.IO.ATCommand.TClient`, as `TSession` makes it - echoes
/// expected, `\r\n` terminated, a one-second timeout - with `TATCClient`'s answer of a multipoint
/// radio taken from after its `[n]`.
/// `// C#: SikRadio/RFDLib/IO/ATCommand.cs:8-179; SikRadio/RFD900.cs:33-37, 2754-2773`
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AtClient {
    /// `Terminator`.
    pub terminator: &'static str,
    /// `Echoes`.
    pub echoes: bool,
    /// `Timeout`, in milliseconds.
    pub timeout_ms: u64,
}

impl Default for AtClient {
    /// `TSession`'s: `Echoes = true`, `Terminator = "\r\n"`, `Timeout = 1000`.
    /// `// C#: SikRadio/RFD900.cs:33-37`
    fn default() -> Self {
        Self {
            terminator: "\r\n",
            echoes: true,
            timeout_ms: 1000,
        }
    }
}

impl AtClient {
    /// `EliminateEcho`: the command read back character by character. A character that differs
    /// fails it; a timeout before the echo is whole does not - the C#'s loop ends there and
    /// returns true.
    /// `// C#: SikRadio/RFDLib/IO/ATCommand.cs:54-96`
    fn eliminate_echo<P: Port>(&self, wire: &mut Wire<P>, complete: &str) -> io::Result<bool> {
        if self.echoes {
            wire.set_read_timeout_ms(self.timeout_ms);
            for expected in complete.chars() {
                let Some(got) = wire.read_char()? else {
                    break;
                };
                if got != expected {
                    return Ok(false);
                }
            }
        }
        Ok(true)
    }

    /// `TClient.DoQuery`: the command sent, its echo read back, then the answer - up to the
    /// terminator when `wait_for_terminator`, whatever came in the timeout otherwise. "" when the
    /// echo differed.
    /// `// C#: SikRadio/RFDLib/IO/ATCommand.cs:115-152`
    fn base_query<P: Port>(
        &self,
        wire: &mut Wire<P>,
        command: &str,
        wait_for_terminator: bool,
    ) -> io::Result<String> {
        let complete = format!("{command}{}", self.terminator);
        wire.discard_in_buffer()?;
        wire.write_str(&complete)?;
        if !self.eliminate_echo(wire, &complete)? {
            return Ok(String::new());
        }
        if wait_for_terminator {
            let (got, result) = wait_for_token(wire, Some(self.terminator), self.timeout_ms)?;
            if got {
                // `Result.Split(Terminator)[0]`.
                return Ok(result
                    .split(self.terminator)
                    .next()
                    .unwrap_or_default()
                    .to_owned());
            }
            Ok(result)
        } else {
            let (_, result) = wait_for_token(wire, None, self.timeout_ms)?;
            Ok(result)
        }
    }

    /// `TATCClient.DoQuery`: [`AtClient::base_query`], with a multipoint radio's `[n]` taken off
    /// the front when something follows it.
    /// `// C#: SikRadio/RFD900.cs:2761-2772`
    ///
    /// # Errors
    /// The port's.
    pub fn do_query<P: Port>(
        &self,
        wire: &mut Wire<P>,
        command: &str,
        wait_for_terminator: bool,
    ) -> io::Result<String> {
        let raw = self.base_query(wire, command, wait_for_terminator)?;
        Ok(strip_node(&raw).to_owned())
    }

    /// `TClient.DoCommand`: the query's answer holds "OK".
    /// `// C#: SikRadio/RFDLib/IO/ATCommand.cs:96-104`
    ///
    /// # Errors
    /// The port's.
    pub fn do_command<P: Port>(&self, wire: &mut Wire<P>, command: &str) -> io::Result<bool> {
        Ok(self.do_query(wire, command, true)?.contains("OK"))
    }

    /// `TClient.DoQueryWithMultiLineResponse`: lines gathered while each wait - the timeout, then
    /// 500 ms - finds a terminator; what came in a wait that found none is dropped. `TATCClient`
    /// does not override it, so a multipoint `[n]` stays.
    /// `// C#: SikRadio/RFDLib/IO/ATCommand.cs:154-179`
    ///
    /// # Errors
    /// The port's.
    pub fn do_query_with_multi_line_response<P: Port>(
        &self,
        wire: &mut Wire<P>,
        command: &str,
    ) -> io::Result<String> {
        let complete = format!("{command}{}", self.terminator);
        wire.discard_in_buffer()?;
        wire.write_str(&complete)?;
        if !self.eliminate_echo(wire, &complete)? {
            return Ok(String::new());
        }
        let mut result = String::new();
        let mut timeout = self.timeout_ms;
        loop {
            let (got, temp) = wait_for_token(wire, Some(self.terminator), timeout)?;
            if !got {
                break;
            }
            result.push_str(&temp);
            timeout = 500;
        }
        Ok(result)
    }
}

/// `TATCClient`'s rule: an answer starting `[` with a `]` that something follows is taken from
/// after the `]`.
/// `// C#: SikRadio/RFD900.cs:2765-2771`
#[must_use]
pub fn strip_node(raw: &str) -> &str {
    if raw.starts_with('[')
        && let Some(close) = raw.find(']')
        && close + 1 < raw.len()
    {
        return raw.get(close + 1..).unwrap_or(raw);
    }
    raw
}

/// `Serial_ReadLine`: bytes until a `\n` - kept - or `ReadTimeout` passes.
/// `// C#: Radio/Sikradio.cs:1911-1928`
///
/// # Errors
/// The port's.
pub fn serial_read_line<P: Port>(wire: &mut Wire<P>) -> io::Result<String> {
    let deadline = wire.now() + wire.read_timeout();
    let mut line = String::new();
    loop {
        let now = wire.now();
        if now >= deadline {
            break;
        }
        // The C# polls `BytesToRead` until the deadline; a read that waits for the next byte
        // until then is the same loop without the spin.
        if let Some(byte) = wire.read_byte_within(deadline - now)? {
            line.push(char::from(byte));
            if byte == b'\n' {
                break;
            }
        }
    }
    Ok(line)
}

/// `doCommand`: the page's command. "Doing Command <cmd>" in `lbl_status`; a bare `\r\n` and its
/// line read, 50 ms, then the command; its echo line must hold the command, then one line is the
/// answer - or, `multi_line`, every line that comes before a second of quiet. A missing echo or an
/// empty one-line answer is tried once more (`level` 1 is that second try); "" after it, or when
/// the port is closed.
/// `// C#: Radio/Sikradio.cs:1929-2000`
///
/// # Errors
/// The port's.
pub fn do_command<P: Port>(
    wire: &mut Wire<P>,
    status: &mut dyn FnMut(&str),
    cmd: &str,
    multi_line: bool,
    level: u8,
) -> io::Result<String> {
    if !wire.is_open() {
        return Ok(String::new());
    }
    wire.discard_in_buffer()?;
    status(&format!("Doing Command {cmd}"));
    wire.set_read_timeout_ms(1000);
    wire.write_str("\r\n")?;
    serial_read_line(wire)?;
    wire.sleep(50);
    wire.write_str(&format!("{cmd}\r\n"))?;
    wire.set_read_timeout_ms(1000);
    // Command echo.
    let echo = serial_read_line(wire)?;
    if echo.contains(cmd) {
        let mut value = String::new();
        if multi_line {
            let deadline = wire.now() + Duration::from_millis(1000);
            while wire.bytes_to_read()? > 0 || wire.now() < deadline {
                value.push_str(&serial_read_line(wire)?);
            }
        } else {
            value = serial_read_line(wire)?;
            if value.is_empty() && level == 0 {
                return do_command(wire, status, cmd, multi_line, 1);
            }
        }
        return Ok(value);
    }
    wire.discard_in_buffer()?;
    // Try again.
    if level == 0 {
        return do_command(wire, status, cmd, multi_line, 1);
    }
    Ok(String::new())
}

/// `RemoveMultiPointLocalNodeID`: a reply starting `[` with a `]`, trimmed, taken from after it.
/// `// C#: Radio/Sikradio.cs:2521-2535`
#[must_use]
pub fn remove_multipoint_node_id(reply: &str) -> String {
    let reply = reply.trim();
    if reply.len() > 1
        && reply.starts_with('[')
        && let Some(close) = reply.find(']')
    {
        return reply.get(close + 1..).unwrap_or_default().to_owned();
    }
    reply.to_owned()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::radio::Radio;

    fn wire() -> Wire<Radio> {
        let mut radio = Radio::rfd900p();
        radio.enter_command_mode();
        Wire::new(radio)
    }

    /// The client reads the echo back and answers with the line after it; `TATCClient` takes a
    /// multipoint node off the front.
    #[test]
    fn the_client_skips_the_echo_and_reads_one_line() {
        let mut wire = wire();
        let client = AtClient::default();
        assert_eq!(client.do_query(&mut wire, "ATI2", true).unwrap(), "130");
        assert!(wire.port().commands().contains(&"ATI2".to_owned()));
        assert!(client.do_command(&mut wire, "ATS3=25").unwrap());
        assert!(!client.do_command(&mut wire, "ATS99=1").unwrap(), "ERROR");
        assert_eq!(strip_node("[1]130"), "130");
        assert_eq!(strip_node("[1]"), "[1]", "nothing after the node");
        assert_eq!(remove_multipoint_node_id(" [2] 0123 "), " 0123");
    }

    /// The multi-line query gathers every line; the page's `doCommand` reads its lines the same.
    #[test]
    fn multi_line_answers_are_gathered() {
        let mut wire = wire();
        let client = AtClient::default();
        let all = client
            .do_query_with_multi_line_response(&mut wire, "ATI5")
            .unwrap();
        assert!(
            all.starts_with("S0:FORMAT=25\r\nS1:SERIAL_SPEED=57\r\n"),
            "{all}"
        );
        let mut said = Vec::new();
        let answer =
            do_command(&mut wire, &mut |s| said.push(s.to_owned()), "ATI5", true, 0).unwrap();
        assert_eq!(answer, all);
        assert_eq!(said, ["Doing Command ATI5"]);
        let one = do_command(&mut wire, &mut |_| {}, "ATI", false, 0).unwrap();
        assert_eq!(one, "RFD SiK 2.65 on RFD900P\r\n");
    }

    /// No answer: `doCommand` tries once more, then gives "".
    #[test]
    fn a_silent_radio_is_asked_twice() {
        let mut wire = Wire::new(Radio::rfd900p());
        let mut said = Vec::new();
        let answer =
            do_command(&mut wire, &mut |s| said.push(s.to_owned()), "ATI", false, 0).unwrap();
        assert_eq!(answer, "");
        assert_eq!(said.len(), 2, "the second try");
    }

    /// The token waits: found, and not found within the wait, on the port's clock.
    #[test]
    fn the_token_waits_end_at_the_token_or_the_wait() {
        let mut wire = Wire::new(Radio::rfd900p());
        wire.write_str("+++").unwrap();
        let before = wire.now();
        assert!(wait_for(&mut wire, "OK\r\n", 3000).unwrap());
        assert!(
            wire.now() - before >= Duration::from_millis(1000),
            "the guard time"
        );
        let before = wire.now();
        assert!(!wait_for(&mut wire, "RFD", 400).unwrap());
        assert!(wire.now() - before >= Duration::from_millis(400));
        assert_eq!(
            wire.read_timeout(),
            Duration::from_millis(400),
            "not put back"
        );
        let (got, text) = wait_for_token(&mut wire, Some("x"), 200).unwrap();
        assert!(!got);
        assert_eq!(text, "");
    }
}
