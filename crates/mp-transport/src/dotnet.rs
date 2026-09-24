//! The pieces of .NET the C# transports lean on without saying so, done the way .NET does them.
//!
//! `CommsNTRIP` and `WebSocket` hand their URL to `System.Uri` and put what it gives back on the
//! wire: its lower-cased `Host`, its `Port` (with the scheme's default), its canonical
//! `PathAndQuery`. `CommsNTRIP` formats the GGA sentence's numbers with custom format strings like
//! `"0000.00"`, which round differently from Rust's `{:.2}`. Each is reproduced here to the
//! behaviour recorded from the C# under mono in `testdata/comms/golden/` (made by
//! `tools/csharp-reference/regen-comms.sh`), because the bytes a caster or a websocket server sees
//! are what has to match.

use std::fmt;

/// What `System.Uri` makes of an absolute URL, as far as the transports read it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct Uri {
    /// `Uri.Scheme`: lower case.
    pub(crate) scheme: String,
    /// `Uri.UserInfo`: as written, still escaped; empty when there is none.
    pub(crate) user_info: String,
    /// `Uri.Host`: lower case; an IPv6 literal keeps its brackets.
    pub(crate) host: String,
    /// `Uri.Port`: the one written, else the scheme's default, else none (.NET's -1).
    pub(crate) port: Option<u16>,
    /// `Uri.PathAndQuery`: escaped, canonical, never empty; the fragment is not part of it.
    pub(crate) path_and_query: String,
}

/// Why `new Uri(...)` throws, in its own words.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum UriError {
    /// No `scheme://` at the front.
    Format,
    /// The host is empty or holds characters a host cannot.
    Host,
    /// The port is not a number from 0 to 65535.
    Port,
}

impl fmt::Display for UriError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // The UriFormatException messages, as ntrip-requests.txt recorded them.
        f.write_str(match self {
            Self::Format => "Invalid URI: The format of the URI could not be determined.",
            Self::Host => "Invalid URI: The hostname could not be parsed.",
            Self::Port => "Invalid URI: Invalid port specified.",
        })
    }
}

impl std::error::Error for UriError {}

impl Uri {
    /// `new Uri(text)` for an absolute URL with an authority.
    ///
    /// Covers what the transports are given: `scheme://[userinfo@]host[:port][/path][?query]
    /// [#fragment]`. Not covered, because nothing here is ever given it: .NET's rewriting of IPv4
    /// in other notations (`0x7f.1` becomes `127.0.0.1`) and IDN hosts (`IdnHost`).
    pub(crate) fn parse(text: &str) -> Result<Self, UriError> {
        let text = text.trim();
        let (scheme, rest) = text.split_once(':').ok_or(UriError::Format)?;
        let scheme_ok = scheme
            .chars()
            .next()
            .is_some_and(|c| c.is_ascii_alphabetic())
            && scheme
                .chars()
                .all(|c| c.is_ascii_alphanumeric() || matches!(c, '+' | '-' | '.'));
        let rest = rest
            .strip_prefix("//")
            .filter(|_| scheme_ok)
            .ok_or(UriError::Format)?;
        let scheme = scheme.to_ascii_lowercase();

        // The authority ends at the path, the query or the fragment; http's family also takes a
        // backslash as a path separator.
        let end = rest.find(['/', '?', '#', '\\']).unwrap_or(rest.len());
        let (authority, tail) = rest.split_at(end);

        // One `@` ends the user info. A second one lands in the host, which cannot hold it - the
        // reason CommsNTRIP escapes the first of two (CommsNTRIP.cs:139-148).
        let (user_info, host_port) = match authority.split_once('@') {
            Some((user, host)) => (user, host),
            None => ("", authority),
        };
        if host_port.contains('@') {
            return Err(UriError::Host);
        }

        let (host, port_text) = if host_port.starts_with('[') {
            let close = host_port.find(']').ok_or(UriError::Host)?;
            let (host, after) = host_port.split_at(close + 1);
            match after.strip_prefix(':') {
                Some(port) => (host, Some(port)),
                None if after.is_empty() => (host, None),
                None => return Err(UriError::Host),
            }
        } else {
            match host_port.rsplit_once(':') {
                Some((host, port)) => (host, Some(port)),
                None => (host_port, None),
            }
        };
        let host_ok = !host.is_empty()
            && (host.starts_with('[')
                || host
                    .chars()
                    .all(|c| c.is_alphanumeric() || matches!(c, '-' | '.' | '_')));
        if !host_ok {
            return Err(UriError::Host);
        }

        let port = match port_text {
            // `host:` with nothing after it is the default port.
            None | Some("") => default_port(&scheme),
            Some(digits) if digits.bytes().all(|b| b.is_ascii_digit()) => {
                Some(digits.parse::<u16>().map_err(|_| UriError::Port)?)
            }
            Some(_) => return Err(UriError::Port),
        };

        // The fragment is not part of PathAndQuery; the query starts at the first `?`.
        let tail = tail.split_once('#').map_or(tail, |(before, _)| before);
        let marks = unescapes_marks(tail);
        let (path, query) = match tail.split_once('?') {
            Some((path, query)) => (path, Some(query)),
            None => (tail, None),
        };
        let mut path_and_query = remove_dot_segments(&canonical(path, true, marks));
        if path_and_query.is_empty() {
            path_and_query.push('/');
        }
        if let Some(query) = query {
            path_and_query.push('?');
            path_and_query.push_str(&canonical(query, false, marks));
        }

        Ok(Self {
            scheme,
            user_info: user_info.to_owned(),
            host: host.to_ascii_lowercase(),
            port,
            path_and_query,
        })
    }
}

/// The schemes whose default port .NET knows, of those the transports are handed.
fn default_port(scheme: &str) -> Option<u16> {
    match scheme {
        "http" | "ws" => Some(80),
        "https" | "wss" => Some(443),
        _ => None,
    }
}

/// Whether .NET writes an escape of this byte back as the byte itself: RFC 3986's unreserved set.
const fn unreserved(byte: u8) -> bool {
    byte.is_ascii_alphanumeric() || matches!(byte, b'-' | b'.' | b'_' | b'~')
}

/// Whether .NET also unescapes `!`, `*`, `'`, `(`, `)` and `:` in this path and query.
///
/// It does once anything in them is unescaped or is past ASCII - an escaped unreserved character,
/// a raw non-ASCII one, or a UTF-8 escape - and then does it throughout: mono gives `/x%21` for
/// `/x%21`, but `/xA!` for `/x%41%21` and `/x!?qA` for `/x%21?q%41`. Mission Planner's own
/// `PercentEncode` escapes every character past ASCII to one byte, and never escapes an unreserved
/// one, so for NTRIP it applies only to a mount point whose Latin-1 characters happen to spell
/// UTF-8.
fn unescapes_marks(tail: &str) -> bool {
    let bytes = tail.as_bytes();
    bytes.iter().enumerate().any(|(at, &byte)| {
        if byte >= 0x80 {
            return true;
        }
        match escape_at(bytes, at) {
            Some(decoded) if unreserved(decoded) => true,
            Some(lead) if lead >= 0xC0 => {
                // A UTF-8 lead byte whose continuation bytes are escaped after it.
                let mut run = vec![lead];
                let mut next = at + 3;
                while let Some(more) = escape_at(bytes, next) {
                    run.push(more);
                    next += 3;
                }
                match std::str::from_utf8(&run) {
                    Ok(_) => true,
                    Err(error) => error.valid_up_to() > 0,
                }
            }
            _ => false,
        }
    })
}

/// The marks [`unescapes_marks`] names.
const fn mark(byte: u8) -> bool {
    matches!(byte, b'!' | b'*' | b'\'' | b'(' | b')' | b':')
}

/// Escapes a path or a query as `System.Uri` does.
///
/// An escape of an unreserved character is unescaped (and of a mark too, when `marks`), a `%`
/// that starts no escape becomes `%25`,
/// the characters RFC 3986 leaves out of a URL (space, `"`, `<`, `>`, `^`, `` ` ``, `{`, `|`, `}`)
/// and anything past ASCII are escaped as UTF-8, and a backslash is a `/` in a path and `%5C` in a
/// query. Everything else is kept as written.
fn canonical(text: &str, is_path: bool, marks: bool) -> String {
    let bytes = text.as_bytes();
    let mut out = String::with_capacity(text.len());
    let mut i = 0;
    while let Some(&byte) = bytes.get(i) {
        match byte {
            b'%' => {
                let escape = bytes
                    .get(i + 1..i + 3)
                    .and_then(|hex| std::str::from_utf8(hex).ok())
                    .filter(|hex| hex.bytes().all(|b| b.is_ascii_hexdigit()))
                    .and_then(|hex| u8::from_str_radix(hex, 16).ok());
                match escape {
                    Some(decoded) if unreserved(decoded) || (marks && mark(decoded)) => {
                        out.push(char::from(decoded));
                        i += 3;
                    }
                    Some(_) => {
                        out.push_str(text.get(i..i + 3).unwrap_or_default());
                        i += 3;
                    }
                    None => {
                        out.push_str("%25");
                        i += 1;
                    }
                }
            }
            b'\\' if is_path => {
                out.push('/');
                i += 1;
            }
            b' '
            | b'"'
            | b'<'
            | b'>'
            | b'^'
            | b'`'
            | b'{'
            | b'|'
            | b'}'
            | b'\\'
            | 0..=0x1F
            | 0x7F.. => {
                push_escape(&mut out, byte);
                i += 1;
            }
            _ => {
                out.push(char::from(byte));
                i += 1;
            }
        }
    }
    out
}

fn push_escape(out: &mut String, byte: u8) {
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    out.push('%');
    for nibble in [byte >> 4, byte & 0x0F] {
        out.push(char::from(
            HEX.get(usize::from(nibble)).copied().unwrap_or(b'0'),
        ));
    }
}

/// RFC 3986 section 5.2.4, which .NET applies to a path after unescaping it: `/./` goes, and
/// `/x/../` takes `x` with it. `//` is kept.
fn remove_dot_segments(path: &str) -> String {
    let mut input = path;
    let mut output: Vec<&str> = Vec::new();
    // Each output entry is a segment with its leading `/`.
    while !input.is_empty() {
        if let Some(rest) = input.strip_prefix("../") {
            input = rest;
        } else if let Some(rest) = input.strip_prefix("./") {
            input = rest;
        } else if input.starts_with("/./") {
            input = input.get(2..).unwrap_or_default();
        } else if input == "/." {
            input = "/";
        } else if input.starts_with("/../") {
            input = input.get(3..).unwrap_or_default();
            output.pop();
        } else if input == "/.." {
            input = "/";
            output.pop();
        } else if input == "." || input == ".." {
            input = "";
        } else {
            // Move the first segment, with its leading slash, to the output.
            let start = usize::from(input.starts_with('/'));
            let end = input
                .get(start..)
                .and_then(|after| after.find('/'))
                .map_or(input.len(), |at| at + start);
            output.push(input.get(..end).unwrap_or_default());
            input = input.get(end..).unwrap_or_default();
        }
    }
    output.concat()
}

/// `Uri.UnescapeDataString`: every escape decoded, except bytes that do not make UTF-8, which are
/// left escaped.
pub(crate) fn unescape_data_string(text: &str) -> String {
    let bytes = text.as_bytes();
    let mut out = String::with_capacity(text.len());
    let mut i = 0;
    while let Some(&byte) = bytes.get(i) {
        if byte != b'%' {
            // Copy the whole character this byte starts.
            let next = (i + 1..=bytes.len())
                .find(|&end| text.is_char_boundary(end))
                .unwrap_or(bytes.len());
            out.push_str(text.get(i..next).unwrap_or_default());
            i = next;
            continue;
        }
        // A run of escapes, decoded together so a multi-byte character comes back whole.
        let mut run = Vec::new();
        let mut spans = Vec::new();
        let mut j = i;
        while let Some(decoded) = escape_at(bytes, j) {
            run.push(decoded);
            spans.push(j);
            j += 3;
        }
        if run.is_empty() {
            out.push('%');
            i += 1;
            continue;
        }
        let mut k = 0;
        while k < run.len() {
            let rest = run.get(k..).unwrap_or_default();
            let valid = match std::str::from_utf8(rest) {
                Ok(all) => all.len(),
                Err(error) => error.valid_up_to(),
            };
            if valid > 0 {
                out.push_str(
                    std::str::from_utf8(rest.get(..valid).unwrap_or_default()).unwrap_or_default(),
                );
                k += valid;
            } else {
                // Not UTF-8: the escape stays as it was written.
                let at = spans.get(k).copied().unwrap_or(i);
                out.push_str(text.get(at..at + 3).unwrap_or_default());
                k += 1;
            }
        }
        i = j;
    }
    out
}

fn escape_at(bytes: &[u8], at: usize) -> Option<u8> {
    if bytes.get(at) != Some(&b'%') {
        return None;
    }
    let hex = bytes.get(at + 1..at + 3)?;
    let hex = std::str::from_utf8(hex).ok()?;
    if !hex.bytes().all(|b| b.is_ascii_hexdigit()) {
        return None;
    }
    u8::from_str_radix(hex, 16).ok()
}

/// `value.ToString(format, CultureInfo.InvariantCulture)` for a custom format of `int_digits`
/// zeros, a point and `decimals` zeros, such as `"0000.00"`.
///
/// .NET first takes the double to 15 significant digits and then rounds those half away from
/// zero, so `2.675` (stored as 2.67499999...) becomes `"2.68"` where Rust's `{:.2}` gives `2.67`,
/// and `0.125` becomes `"0.13"` where Rust gives `0.12`. A negative number that rounds to zero
/// loses its sign: `-0.004` is `"0.00"`. All three are in ntrip-gga.txt.
pub(crate) fn format_fixed(value: f64, int_digits: usize, decimals: usize) -> String {
    if value.is_nan() {
        return "NaN".to_owned();
    }
    if value.is_infinite() {
        return if value < 0.0 { "-Infinity" } else { "Infinity" }.to_owned();
    }
    // Fifteen significant digits and the power of ten of the first.
    let scientific = format!("{:.14e}", value.abs());
    let (mantissa, exponent) = scientific.split_once('e').unwrap_or(("0", "0"));
    let exponent: i64 = exponent.parse().unwrap_or(0);
    let digits: Vec<u8> = mantissa
        .bytes()
        .filter(u8::is_ascii_digit)
        .map(|digit| digit - b'0')
        .collect();

    // Digits kept: those before the point and `decimals` after it.
    let kept = exponent + 1 + i64::try_from(decimals).unwrap_or(0);
    if kept > 38 {
        // Past what the arithmetic below holds; not a latitude, a longitude or an altitude.
        return format!("{value:.decimals$}");
    }
    let mut scaled: u128 = 0;
    for position in 0..kept.max(0) {
        let digit = usize::try_from(position)
            .ok()
            .and_then(|at| digits.get(at))
            .copied()
            .unwrap_or(0);
        scaled = scaled * 10 + u128::from(digit);
    }
    let rounding = usize::try_from(kept)
        .ok()
        .and_then(|at| digits.get(at))
        .copied()
        .unwrap_or(0);
    if kept >= 0 && rounding >= 5 {
        scaled += 1;
    }

    let unit = 10u128.pow(u32::try_from(decimals).unwrap_or(0));
    let whole = scaled / unit;
    let fraction = scaled % unit;
    let sign = if value < 0.0 && scaled != 0 { "-" } else { "" };
    if decimals == 0 {
        format!("{sign}{whole:0int_digits$}")
    } else {
        format!("{sign}{whole:0int_digits$}.{fraction:0decimals$}")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn uri(text: &str) -> Uri {
        Uri::parse(text).unwrap_or_else(|e| panic!("{text}: {e}"))
    }

    // Each expectation below is what mono's System.Uri printed for the same input.

    #[test]
    fn host_port_and_path_come_out_as_dotnet_gives_them() {
        let u = uri("http://Me%40x.com:p%21@RTK2GO.com:2101/MOUNT%20a/./b/../c");
        assert_eq!(u.host, "rtk2go.com");
        assert_eq!(u.port, Some(2101));
        assert_eq!(u.path_and_query, "/MOUNT%20a/c");
        assert_eq!(u.user_info, "Me%40x.com:p%21");
        assert_eq!(unescape_data_string(&u.user_info), "Me@x.com:p!");

        assert_eq!(uri("http://host/M").port, Some(80));
        assert_eq!(uri("http://host:2101").path_and_query, "/");
        assert_eq!(uri("ws://Host/path").port, Some(80));
        assert_eq!(uri("wss://Host/path").port, Some(443));
        assert_eq!(uri("WS://host/a").scheme, "ws");
        assert_eq!(uri("http://host:/M").port, Some(80));
        assert_eq!(uri("ntrip://host/M").port, None);
        assert_eq!(uri("http://:pw@host/M").user_info, ":pw");
        assert_eq!(uri("http://ho_st/M").host, "ho_st");
        assert_eq!(uri("ws://[::1]:81/a").host, "[::1]");
    }

    #[test]
    fn what_dotnet_refuses_is_refused() {
        assert_eq!(Uri::parse("http://a:b@c@host:2101/M"), Err(UriError::Host));
        assert_eq!(Uri::parse("http://%5B::1%5D:2101/M"), Err(UriError::Host));
        assert_eq!(Uri::parse("http://host:99999/M"), Err(UriError::Port));
        assert_eq!(Uri::parse("rtk2go.com:2101/MOUNT"), Err(UriError::Format));
    }

    #[test]
    fn paths_and_queries_are_canonical_as_dotnet_makes_them() {
        assert_eq!(
            uri("http://host:2101/%41%2B%21%3F%25").path_and_query,
            "/A%2B!%3F%25"
        );
        assert_eq!(uri("http://host:5000/a%21e").path_and_query, "/a%21e");
        assert_eq!(uri("http://host:5000/x%41%21").path_and_query, "/xA!");
        assert_eq!(uri("http://host:5000/x%21?q%41").path_and_query, "/x!?qA");
        assert_eq!(uri("http://host:5000/p?x%21").path_and_query, "/p?x%21");
        assert_eq!(
            uri("http://host:5000/x%C3%A9%21").path_and_query,
            "/x%C3%A9!"
        );
        assert_eq!(uri("http://host:5000/x%21é").path_and_query, "/x!%C3%A9");
        assert_eq!(uri("http://host:5000/x%E9%21").path_and_query, "/x%E9%21");
        assert_eq!(uri("http://host:5000/x%2f%3a").path_and_query, "/x%2f%3a");
        assert_eq!(uri("http://host:5000/x%2e%21").path_and_query, "/x.!");
        assert_eq!(
            uri("http://host:5000/x%41%21%3A%2A%27%28%29%40%24").path_and_query,
            "/xA!:*'()%40%24"
        );
        assert_eq!(
            uri("ws://h:81/a b?x=1&y=é#frag").path_and_query,
            "/a%20b?x=1&y=%C3%A9"
        );
        assert_eq!(uri("http://host:2101//a//b/").path_and_query, "//a//b/");
        assert_eq!(uri("http://host:2101/a/b/..").path_and_query, "/a/");
        assert_eq!(uri("http://host:2101/..").path_and_query, "/");
        assert_eq!(uri("http://host:2101/a.").path_and_query, "/a.");
        assert_eq!(uri("ws://host/./a/../../b/").path_and_query, "/b/");
        assert_eq!(uri("ws://host/a/.").path_and_query, "/a/");
        assert_eq!(uri("http://host:5000/a/%2E%2E/b").path_and_query, "/b");
        assert_eq!(uri("ws://host:5000/a%zzb").path_and_query, "/a%25zzb");
        assert_eq!(uri("ws://host:5000/a%").path_and_query, "/a%25");
        assert_eq!(uri("ws://host:5000/a?b?c/d").path_and_query, "/a?b?c/d");
        assert_eq!(uri("ws://host:5000/a?%2E%2E/b").path_and_query, "/a?../b");
        assert_eq!(uri("ws://host:5000?q").path_and_query, "/?q");
        assert_eq!(uri("ws://host:5000/a#f?x").path_and_query, "/a");
        assert_eq!(uri("ws://host:5000/x\\y").path_and_query, "/x/y");
        assert_eq!(uri("ws://host:5000/p?x\\y").path_and_query, "/p?x%5Cy");
        for (raw, escaped) in [
            ('"', "%22"),
            ('<', "%3C"),
            ('>', "%3E"),
            ('^', "%5E"),
            ('`', "%60"),
            ('{', "%7B"),
            ('|', "%7C"),
            ('}', "%7D"),
        ] {
            assert_eq!(
                uri(&format!("ws://host:5000/x{raw}y")).path_and_query,
                format!("/x{escaped}y")
            );
        }
    }

    #[test]
    fn unescaping_leaves_what_is_not_utf8_escaped() {
        assert_eq!(unescape_data_string("p%21ss"), "p!ss");
        assert_eq!(unescape_data_string("caf%C3%A9"), "café");
        assert_eq!(unescape_data_string("caf%E9"), "caf%E9");
        assert_eq!(unescape_data_string("100%"), "100%");
        assert_eq!(unescape_data_string("a%2"), "a%2");
        assert_eq!(unescape_data_string("é%40"), "é@");
    }

    #[test]
    fn numbers_format_as_dotnet_formats_them() {
        assert_eq!(format_fixed(2.675, 1, 2), "2.68");
        assert_eq!(format_fixed(0.125, 1, 2), "0.13");
        assert_eq!(format_fixed(1.005, 1, 2), "1.01");
        assert_eq!(format_fixed(-0.004, 1, 2), "0.00");
        assert_eq!(format_fixed(-45.5, 1, 2), "-45.50");
        assert_eq!(format_fixed(584.0, 1, 2), "584.00");
        assert_eq!(format_fixed(3521.8, 4, 2), "3521.80");
        assert_eq!(format_fixed(9.0, 5, 2), "00009.00");
        assert_eq!(format_fixed(0.005, 4, 2), "0000.01");
        assert_eq!(format_fixed(0.0049, 4, 2), "0000.00");
        assert_eq!(format_fixed(0.0, 4, 2), "0000.00");
        assert_eq!(format_fixed(99.999, 1, 2), "100.00");
        assert_eq!(format_fixed(f64::NAN, 1, 2), "NaN");
    }
}
