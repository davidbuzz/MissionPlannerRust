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

//! The main window's connection controls: `ConnectionControl` - the port box `cmb_Connection`
//! and the baud box `cmb_Baud` - and the CONNECT button `MenuConnect`, with what pressing it does.
//! Ported from `MainV2.cs` @ efb0801 (GPL-3.0-only): `PopulateSerialportList` (:1283-1300),
//! `MenuConnect_Click` and `Connect` (:1841-1880), `doDisconnect` (:1389-1447), `doConnect`
//! (:1448-1700), `CMB_serialport_SelectedIndexChanged` (:1962-1984), `CMB_baudrate_TextChanged`
//! (:4333-4350); `Controls/ConnectionControl.cs`; the transports' `Open` prompts in
//! `ExtLibs/Comms/CommsTCPSerial.cs:101-145`, `CommsUdpSerial.cs:100-125`,
//! `CommsUDPSerialConnect.cs:130-150`, `CommsWebSocket.cs:100-110`.
//!
//! The port box lists the serial ports between `AUTO` and `TCP`, `UDP`, `UDPCl`, `WS`; the baud
//! box holds the sixteen rates of the `.resx` and is off for the ports that have no baud. CONNECT
//! opens what the boxes say - a serial port at its baud, or TCP, UDP, UDPCl and WS after their
//! questions (host and port, a local port, a URL, each offering what the settings last saved) -
//! and becomes DISCONNECT; DISCONNECT closes the link, asking first when the model is still
//! moving. Every press saves the settings, as `MenuConnect_Click` does.
//!
//! What is here is the pure part: the lists, the rules, the questions and the URL the answers
//! make. The drawing and the link itself are in `main.rs`.

/// `cmb_Baud`'s items.
/// `// C#: Controls/ConnectionControl.resx (cmb_Baud.Items..Items15)`
pub(crate) const BAUDS: [&str; 16] = [
    "1200", "2400", "4800", "9600", "19200", "38400", "57600", "111100", "115200", "230400",
    "460800", "500000", "625000", "921600", "1000000", "1500000",
];

/// The port box's entries after the serial ports.
/// `// C#: MainV2.cs:1295-1299`
pub(crate) const NETWORK_PORTS: [&str; 4] = ["TCP", "UDP", "UDPCl", "WS"];

/// `Strings.CONNECTc` and `DISCONNECTc`, the button's two texts.
/// `// C#: ExtLibs/Strings/Strings.resx:271-279`
pub(crate) const CONNECT: &str = "CONNECT";
pub(crate) const DISCONNECT: &str = "DISCONNECT";
/// `Strings.Stillmoving`, asked before disconnecting from a moving model, under `Strings.Disconnect`.
/// `// C#: MainV2.cs:1851-1857; ExtLibs/Strings/Strings.resx:274-296`
pub(crate) const STILL_MOVING: &str =
    "Your model is still moving are you sure you want to disconnect?";
pub(crate) const DISCONNECT_TITLE: &str = "Disconnect";
/// `Strings.InvalidBaudRate`, for a baud box that is not a number.
/// `// C#: MainV2.cs:4335-4339; ExtLibs/Strings/Strings.resx:177-179`
pub(crate) const INVALID_BAUD_RATE: &str = "Invalid BaudRate";
/// `comPort.MAV.cs.groundspeed > 4`: faster than this, disconnecting is asked about.
/// `// C#: MainV2.cs:1851`
pub(crate) const STILL_MOVING_SPEED: f64 = 4.0;

/// `PopulateSerialportList`: `AUTO`, the serial ports as the system lists them, then the network
/// kinds. `AUTO` is listed as the C# lists it, and refused when chosen: its port scan
/// (`CommsSerialScan`) is not ported.
/// `// C#: MainV2.cs:1291-1300`
#[must_use]
pub(crate) fn port_list(serial_ports: &[String]) -> Vec<String> {
    let mut list = vec!["AUTO".to_owned()];
    list.extend(serial_ports.iter().cloned());
    list.extend(NETWORK_PORTS.iter().map(|name| (*name).to_owned()));
    list
}

/// `CMB_serialport_SelectedIndexChanged`: the baud box is off for the kinds that have no baud.
/// `// C#: MainV2.cs:1967-1974`
#[must_use]
pub(crate) fn baud_enabled(port: &str) -> bool {
    !matches!(port, "UDP" | "UDPCl" | "TCP" | "AUTO")
}

/// `CMB_baudrate_TextChanged`: the text must parse as an integer, else "Invalid BaudRate"; the
/// digits alone are then kept.
/// `// C#: MainV2.cs:4333-4350`
///
/// # Errors
///
/// The status line's words (a box in the C#, never here - the owner's ruling of 2026-09-25).
pub(crate) fn baud_changed(text: &str) -> Result<String, &'static str> {
    if text.trim().parse::<i32>().is_err() {
        return Err(INVALID_BAUD_RATE);
    }
    Ok(text.chars().filter(char::is_ascii_digit).collect())
}

/// What the port box names.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Kind {
    /// A serial device.
    Serial,
    /// `TcpSerial`: a TCP client.
    Tcp,
    /// `UdpSerial`: a UDP listener.
    Udp,
    /// `UdpSerialConnect`: a UDP client.
    UdpClient,
    /// `WebSocket`.
    WebSocket,
    /// The serial scan, not ported.
    Auto,
}

/// `doConnect`'s `switch (portname)`.
/// `// C#: MainV2.cs:1452-1526`
#[must_use]
pub(crate) fn kind(port: &str) -> Kind {
    match port {
        "TCP" => Kind::Tcp,
        "UDP" => Kind::Udp,
        "UDPCl" => Kind::UdpClient,
        "WS" => Kind::WebSocket,
        "AUTO" => Kind::Auto,
        _ => Kind::Serial,
    }
}

/// One of the `InputBox`es a transport's `Open` shows: its title, its words, the settings key
/// whose value it offers, and what it offers when the key is empty.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct Question {
    /// The box's title.
    pub(crate) title: &'static str,
    /// Its words.
    pub(crate) text: &'static str,
    /// The `Settings` key it reads and, once answered, writes.
    pub(crate) key: &'static str,
    /// The transport's own default.
    pub(crate) default: &'static str,
}

/// The questions a kind asks before opening, in the order its `Open` asks them.
/// `// C#: ExtLibs/Comms/CommsTCPSerial.cs:112-125; CommsUdpSerial.cs:110-115;
/// CommsUDPSerialConnect.cs:136-146; CommsWebSocket.cs:103-106`
#[must_use]
pub(crate) fn questions(kind: Kind) -> Vec<Question> {
    match kind {
        Kind::Tcp => vec![
            Question {
                title: "remote host",
                text: "Enter host name/ip (ensure remote end is already started)",
                key: "TCP_host",
                default: "127.0.0.1",
            },
            Question {
                title: "remote Port",
                text: "Enter remote port",
                key: "TCP_port",
                default: "5760",
            },
        ],
        Kind::Udp => vec![Question {
            title: "Listern Port",
            text: "Enter Local port (ensure remote end is already sending)",
            key: "UDP_port",
            default: "14550",
        }],
        Kind::UdpClient => vec![
            Question {
                title: "remote host",
                text: "Enter host name/ip (ensure remote end is already started)",
                key: "UDP_host",
                default: "127.0.0.1",
            },
            Question {
                title: "remote Port",
                text: "Enter remote port",
                key: "UDP_port",
                default: "14550",
            },
        ],
        Kind::WebSocket => vec![Question {
            title: "remote host",
            text: "Enter url (eg http://user:pass@host:port/wspath)",
            key: "WS_url",
            default: "",
        }],
        Kind::Serial | Kind::Auto => Vec::new(),
    }
}

/// A question's OK: the box's text kept as `InputBox` keeps every titled answer - the transports'
/// `OnInputBoxShow` is `Program.CommsBaseOnInputBoxShow`, an `InputBox.Show` - and the answer,
/// trimmed, under the question's settings key. Returns the answer.
/// `// C#: Program.cs:312, 564-566; ExtLibs/Comms/CommsBase.cs:33-39; ExtLibs/Controls/InputBox.cs:73-84, 178-184`
pub(crate) fn answered(
    settings: &mut crate::settings::Persisted,
    question: &Question,
    text: &str,
) -> String {
    crate::config::optional::remember_answer(settings, question.title, question.text, text);
    let answer = text.trim().to_owned();
    settings.set(question.key, answer.clone());
    answer
}

/// The link URL the box and the answers make, as this application's transports spell one:
/// `serial:<port>:<baud>`, `tcp:host:port`, `udp:0.0.0.0:port`, `udpcl:host:port`, or the URL
/// typed for WS. `None` for AUTO, and for a network kind whose answers are not there yet.
#[must_use]
pub(crate) fn url(kind: Kind, port: &str, baud: &str, answers: &[String]) -> Option<String> {
    let answer = |index: usize| answers.get(index).map(|answer| answer.trim());
    match kind {
        Kind::Serial => Some(format!("serial:{port}:{baud}")),
        Kind::Tcp => Some(format!("tcp:{}:{}", answer(0)?, answer(1)?)),
        Kind::Udp => Some(format!("udp:0.0.0.0:{}", answer(0)?)),
        Kind::UdpClient => Some(format!("udpcl:{}:{}", answer(0)?, answer(1)?)),
        Kind::WebSocket => Some(answer(0)?.to_owned()),
        Kind::Auto => None,
    }
}

/// `Connect`'s first check: a moving model is asked about before disconnecting.
/// `// C#: MainV2.cs:1851-1857`
#[must_use]
pub(crate) fn asks_before_disconnecting(connected: bool, groundspeed: f64) -> bool {
    connected && groundspeed > STILL_MOVING_SPEED
}

/// A network kind's questions on their way to being answered.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct Asking {
    /// What is being opened.
    pub(crate) kind: Kind,
    /// The questions, in order.
    pub(crate) questions: Vec<Question>,
    /// The answers so far.
    pub(crate) answers: Vec<String>,
}

impl Asking {
    /// The question due now, if one is.
    #[must_use]
    pub(crate) fn current(&self) -> Option<&Question> {
        self.questions.get(self.answers.len())
    }
}

/// The connection controls' state.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct ConnectBox {
    /// `cmb_Connection.Text`, `MainV2.comPortName`.
    pub(crate) port: String,
    /// `cmb_Baud.Text`.
    pub(crate) baud: String,
    /// The port box's list, filled when the box is clicked.
    pub(crate) ports: Vec<String>,
    /// Whether the port list is open.
    pub(crate) ports_open: bool,
    /// Whether the baud list is open.
    pub(crate) bauds_open: bool,
    /// A network kind's questions being asked.
    pub(crate) asking: Option<Asking>,
    /// "Your model is still moving ..." showing.
    pub(crate) still_moving: bool,
}

impl ConnectBox {
    /// The boxes as the settings left them: `comport` and its baud.
    #[must_use]
    pub(crate) fn new(port: &str, baud: &str) -> Self {
        Self {
            port: port.to_owned(),
            baud: baud.to_owned(),
            ports: Vec::new(),
            ports_open: false,
            bauds_open: false,
            asking: None,
            still_moving: false,
        }
    }
}

/// What a UI test asserts on.
pub(crate) fn record_facts(state: &ConnectBox, connected: bool) {
    use crate::facts::record;
    record("link.port", &state.port);
    record("link.baud", &state.baud);
    record("link.button", if connected { DISCONNECT } else { CONNECT });
    record(
        "link.prompt",
        state.asking.as_ref().and_then(Asking::current).map_or_else(
            || {
                if state.still_moving {
                    DISCONNECT_TITLE.to_owned()
                } else {
                    "none".to_owned()
                }
            },
            |question| question.title.to_owned(),
        ),
    );
    record("link.ports", state.ports.len());
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_port_list_is_auto_the_ports_then_the_network_kinds() {
        let ports = vec!["/dev/ttyACM0".to_owned(), "/dev/ttyUSB0".to_owned()];
        assert_eq!(
            port_list(&ports),
            vec![
                "AUTO",
                "/dev/ttyACM0",
                "/dev/ttyUSB0",
                "TCP",
                "UDP",
                "UDPCl",
                "WS"
            ]
        );
        assert_eq!(port_list(&[]).len(), 5);
    }

    #[test]
    fn the_baud_box_is_off_for_the_kinds_without_one() {
        for port in ["UDP", "UDPCl", "TCP", "AUTO"] {
            assert!(!baud_enabled(port), "{port}");
        }
        assert!(baud_enabled("/dev/ttyACM0"));
        assert!(baud_enabled("WS"));
    }

    #[test]
    fn a_baud_that_is_not_a_number_is_invalid_and_digits_are_kept() {
        assert_eq!(baud_changed("115200"), Ok("115200".to_owned()));
        assert_eq!(baud_changed(" 57600 "), Ok("57600".to_owned()));
        assert_eq!(baud_changed("fast"), Err(INVALID_BAUD_RATE));
        assert_eq!(baud_changed(""), Err(INVALID_BAUD_RATE));
    }

    #[test]
    fn each_kind_asks_its_transports_questions_and_makes_its_url() {
        assert_eq!(kind("TCP"), Kind::Tcp);
        assert_eq!(kind("/dev/ttyACM0"), Kind::Serial);
        assert!(questions(Kind::Serial).is_empty());
        let tcp = questions(Kind::Tcp);
        assert_eq!(tcp.len(), 2);
        assert_eq!(tcp[0].title, "remote host");
        assert_eq!(tcp[0].default, "127.0.0.1");
        assert_eq!(tcp[1].default, "5760");
        assert_eq!(questions(Kind::Udp)[0].title, "Listern Port");
        assert_eq!(questions(Kind::UdpClient)[1].key, "UDP_port");
        assert_eq!(
            url(Kind::Serial, "/dev/ttyACM0", "115200", &[]),
            Some("serial:/dev/ttyACM0:115200".to_owned())
        );
        assert_eq!(
            url(
                Kind::Tcp,
                "TCP",
                "",
                &["127.0.0.1".to_owned(), "5760".to_owned()]
            ),
            Some("tcp:127.0.0.1:5760".to_owned())
        );
        assert_eq!(url(Kind::Tcp, "TCP", "", &["127.0.0.1".to_owned()]), None);
        assert_eq!(
            url(Kind::Udp, "UDP", "", &["14550".to_owned()]),
            Some("udp:0.0.0.0:14550".to_owned())
        );
        assert_eq!(
            url(
                Kind::UdpClient,
                "UDPCl",
                "",
                &["10.0.0.5".to_owned(), "14550".to_owned()]
            ),
            Some("udpcl:10.0.0.5:14550".to_owned())
        );
        assert_eq!(
            url(Kind::WebSocket, "WS", "", &["ws://h:1/p".to_owned()]),
            Some("ws://h:1/p".to_owned())
        );
        assert_eq!(url(Kind::Auto, "AUTO", "", &[]), None);
    }

    #[test]
    fn a_moving_model_is_asked_about_before_disconnecting() {
        assert!(asks_before_disconnecting(true, 4.5));
        assert!(!asks_before_disconnecting(true, 4.0));
        assert!(!asks_before_disconnecting(false, 9.0));
    }

    /// A question's OK keeps the text under `InputBox`'s key for it - the caption and question
    /// with all but letters and digits taken out - and the trimmed answer under its own key.
    /// Cancel calls nothing, so keeps nothing. The facts publish every key.
    /// `// C#: Program.cs:564-566; ExtLibs/Controls/InputBox.cs:73-84, 178-184`
    #[test]
    fn an_answer_is_kept_under_the_input_box_key() {
        let mut keys: Vec<String> = [Kind::Tcp, Kind::Udp, Kind::UdpClient, Kind::WebSocket]
            .into_iter()
            .flat_map(questions)
            .map(|q| crate::config::optional::answers_key(q.title, q.text))
            .collect();
        keys.sort();
        keys.dedup();
        assert_eq!(
            keys,
            [
                "InputBoxListernPortEnterLocalportensureremoteendisalreadysending",
                "InputBoxremotePortEnterremoteport",
                "InputBoxremotehostEnterhostnameipensureremoteendisalreadystarted",
                "InputBoxremotehostEnterurleghttpuserpasshostportwspath",
            ]
        );
        for key in &keys {
            assert!(crate::settings::PUBLISHED.contains(&key.as_str()), "{key}");
        }
        let mut settings = crate::settings::Persisted::at(None);
        let tcp = questions(Kind::Tcp);
        assert_eq!(answered(&mut settings, &tcp[0], " 10.0.0.5 "), "10.0.0.5");
        assert_eq!(settings.get("TCP_host"), Some("10.0.0.5"));
        assert_eq!(settings.get(&keys[2]), Some("+10.0.0.5+"));
        assert_eq!(answered(&mut settings, &tcp[1], "5760"), "5760");
        assert_eq!(settings.get(&keys[1]), Some("5760"));
    }

    #[test]
    fn asking_walks_the_questions_in_order() {
        let mut asking = Asking {
            kind: Kind::Tcp,
            questions: questions(Kind::Tcp),
            answers: Vec::new(),
        };
        assert_eq!(asking.current().map(|q| q.title), Some("remote host"));
        asking.answers.push("h".to_owned());
        assert_eq!(asking.current().map(|q| q.title), Some("remote Port"));
        asking.answers.push("1".to_owned());
        assert!(asking.current().is_none());
    }
}
