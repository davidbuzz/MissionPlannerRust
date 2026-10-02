//! What the two serial output windows share - `Controls/SerialOutputPass.cs` (the Mavlink
//! Mirror) and `Controls/SerialOutputNMEA.cs`: the port list, the baud list, the ways a port
//! name is turned into a stream, and the combo they draw.
//!
//! * `CMB_serialport.Items`: `SerialPort.GetPortNames()`, then `"TCP Host - <port>"`, `"TCP
//!   Client"`, `"UDP Host - <port>"` and `"UDP Client"` (`SerialOutputPass.cs:27-31`,
//!   `SerialOutputNMEA.cs:26-30`);
//! * `CMB_baudrate.Items`: 4800 to 115200 (both `.resx`);
//! * the stream a name opens (`BUT_connect_Click`'s `switch`): a TCP host listens on the window's
//!   port and takes the client that connects; a TCP client asks for its host and port, as
//!   `TcpSerial.Open` asks, under the window's `ConfigRef`; a UDP host asks for its port and
//!   binds it, as `UdpSerial.Open` asks; a UDP client asks for its host and port, as
//!   `UdpSerialConnect.Open` asks; anything else is a serial port of that name at the baud.
//!
//! Where this is not the C#: the questions are the application's input boxes over the window,
//! asked one after the other (the transports' `InputBox.Show`); a TCP host takes one client for
//! each Connect, where `BeginAcceptTcpClient` goes on accepting; `ReadTimeout` and the serial
//! port's handshaking are the transport's own.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use std::sync::mpsc::{Receiver, TryRecvError};

use gpui::{AnyElement, Context, SharedString, div, prelude::*, px, rgb};
use mp_transport::{SerialTransport, TcpTransport, Transport, UdpClientTransport, UdpTransport};

use super::optional::InputBox;
use crate::MissionPlanner;
use crate::settings::Persisted;
use crate::ui::theme;

/// `CMB_baudrate.Items`. `// C#: Controls/SerialOutputNMEA.resx; SerialOutputPass.resx`
pub const BAUDS: [&str; 8] = [
    "4800", "9600", "14400", "19200", "28800", "38400", "57600", "115200",
];

/// `Strings.Connect` and `Strings.Stop`.
pub const CONNECT: &str = "Connect";
pub const STOP: &str = "Stop";
/// `Strings.InvalidPortName`, `Strings.InvalidBaudRate`.
pub const INVALID_PORT_NAME: &str = "Invalid PortName";
pub const INVALID_BAUD_RATE: &str = "Invalid BaudRate";
/// The box when the stream will not open.
/// `// C#: Controls/SerialOutputNMEA.cs:105; SerialOutputPass.cs:112`
pub const ERROR_CONNECTING: &str =
    "Error Connecting\nif using com0com please rename the ports to COM??";

/// `CMB_serialport.Items`: the serial ports present, then the four network kinds with the
/// window's host port in two of their names.
#[must_use]
pub fn port_items(host_port: u16) -> Vec<String> {
    let mut items: Vec<String> = mp_transport::list_ports()
        .into_iter()
        .map(|port| port.name)
        .collect();
    items.push(format!("TCP Host - {host_port}"));
    items.push("TCP Client".to_owned());
    items.push(format!("UDP Host - {host_port}"));
    items.push("UDP Client".to_owned());
    items
}

/// What `CMB_serialport.Text` names.
/// `// C#: Controls/SerialOutputNMEA.cs:44-78; SerialOutputPass.cs:54-98`
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Kind {
    /// `"TCP Host - <port>"` or `"TCP Host"`: a listener on the window's port.
    TcpHost,
    /// `"TCP Client"`.
    TcpClient,
    /// `"UDP Host - <port>"`: `UdpSerial`, which asks for its port.
    UdpHost,
    /// `"UDP Client"`.
    UdpClient,
    /// A serial port of this name.
    Serial(String),
}

impl Kind {
    /// The `switch` on the text.
    #[must_use]
    pub fn of(text: &str) -> Self {
        match text {
            "TCP Client" => Self::TcpClient,
            "UDP Client" => Self::UdpClient,
            t if t.starts_with("TCP Host") => Self::TcpHost,
            t if t.starts_with("UDP Host") => Self::UdpHost,
            other => Self::Serial(other.to_owned()),
        }
    }

    /// The questions the kind's `Open` asks, with the settings keys `ConfigRef` names.
    /// `// C#: ExtLibs/Comms/CommsTCPSerial.cs:112-125; CommsUdpSerial.cs:110-115;
    /// CommsUDPSerialConnect.cs:136-146`
    #[must_use]
    pub fn questions(&self, config_ref: &str) -> Vec<Question> {
        match self {
            Self::TcpClient => vec![
                Question {
                    title: "remote host",
                    text: "Enter host name/ip (ensure remote end is already started)",
                    key: format!("TCP_host{config_ref}"),
                    default: "127.0.0.1",
                },
                Question {
                    title: "remote Port",
                    text: "Enter remote port",
                    key: format!("TCP_port{config_ref}"),
                    default: "5760",
                },
            ],
            Self::UdpHost => vec![Question {
                title: "Listern Port",
                text: "Enter Local port (ensure remote end is already sending)",
                key: format!("UDP_port{config_ref}"),
                default: "14550",
            }],
            Self::UdpClient => vec![
                Question {
                    title: "remote host",
                    text: "Enter host name/ip (ensure remote end is already started)",
                    key: format!("UDP_host{config_ref}"),
                    default: "127.0.0.1",
                },
                Question {
                    title: "remote Port",
                    text: "Enter remote port",
                    key: format!("UDP_port{config_ref}"),
                    default: "14550",
                },
            ],
            Self::TcpHost | Self::Serial(_) => Vec::new(),
        }
    }
}

/// One of a transport's `InputBox`es: its caption, its words, the settings key it offers and
/// writes back, and its default.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Question {
    pub title: &'static str,
    pub text: &'static str,
    pub key: String,
    pub default: &'static str,
}

/// A question up: the box, and whose question it is.
#[derive(Debug)]
pub struct Asking {
    pub question: Question,
    pub input: InputBox,
}

/// The questions a Connect still has to ask, and the answers so far.
#[derive(Debug, Default)]
pub struct Questions {
    pub pending: Vec<Question>,
    pub answers: Vec<String>,
    pub asking: Option<Asking>,
}

impl Questions {
    /// The next question up, offering the settings' value or the default; `false` when none is
    /// left.
    pub fn ask_next(&mut self, persisted: &Persisted) -> bool {
        if self.pending.is_empty() {
            return false;
        }
        let question = self.pending.remove(0);
        let offered = persisted
            .get(&question.key)
            .map_or(question.default, |v| v)
            .to_owned();
        let title: &'static str = question.title;
        let text: &'static str = question.text;
        self.asking = Some(Asking {
            input: InputBox::new(title, text, &offered),
            question,
        });
        true
    }

    /// OK: the answer kept under the question's key and in the list; Cancel ends the questions.
    pub fn answered(&mut self, ok: bool, persisted: &mut Persisted) -> bool {
        let Some(asking) = self.asking.take() else {
            return false;
        };
        if !ok {
            self.pending.clear();
            self.answers.clear();
            return false;
        }
        let answer = asking.input.field.value().trim().to_owned();
        asking.input.remember(persisted);
        persisted.set(&asking.question.key, answer.clone());
        self.answers.push(answer);
        true
    }
}

/// A stream being opened on its thread: what comes back.
pub type Opening = Receiver<Result<Box<dyn Transport>, String>>;

/// The stream a kind opens with its answers, off the UI thread: a TCP host listens on
/// `host_port` for its client; a serial port opens at `baud`.
/// `// C#: Controls/SerialOutputNMEA.cs:44-108`
pub fn open(kind: Kind, host_port: u16, baud: u32, answers: Vec<String>) -> Opening {
    let (send, receive) = std::sync::mpsc::channel();
    let spawned = std::thread::Builder::new()
        .name("mp-serial-output".to_owned())
        .spawn(move || {
            let answer = |index: usize| answers.get(index).map(String::as_str).unwrap_or("");
            let port_number = |text: &str| {
                text.trim()
                    .parse::<u16>()
                    .map_err(|_| "Input string was not in a correct format.".to_owned())
            };
            let opened: Result<Box<dyn Transport>, String> = match kind {
                Kind::TcpHost => TcpTransport::listen(host_port)
                    .map(|s| Box::new(s) as Box<dyn Transport>)
                    .map_err(|e| e.to_string()),
                Kind::TcpClient => port_number(answer(1)).and_then(|port| {
                    TcpTransport::connect(answer(0), port)
                        .map(|s| Box::new(s) as Box<dyn Transport>)
                        .map_err(|e| e.to_string())
                }),
                Kind::UdpHost => port_number(answer(0)).and_then(|port| {
                    UdpTransport::bind("0.0.0.0", port)
                        .map(|s| Box::new(s) as Box<dyn Transport>)
                        .map_err(|e| e.to_string())
                }),
                Kind::UdpClient => port_number(answer(1)).and_then(|port| {
                    UdpClientTransport::open(answer(0), port)
                        .map(|s| Box::new(s) as Box<dyn Transport>)
                        .map_err(|e| e.to_string())
                }),
                Kind::Serial(name) => SerialTransport::open(&name, baud)
                    .map(|s| Box::new(s) as Box<dyn Transport>)
                    .map_err(|e| e.to_string()),
            };
            let _ = send.send(opened);
        });
    if spawned.is_err() {
        let (send, receive) = std::sync::mpsc::channel();
        let _ = send.send(Err("no thread".to_owned()));
        return receive;
    }
    receive
}

/// What an opening has come to, if it has.
pub fn poll(opening: &Opening) -> Option<Result<Box<dyn Transport>, String>> {
    match opening.try_recv() {
        Ok(opened) => Some(opened),
        Err(TryRecvError::Empty) => None,
        Err(TryRecvError::Disconnected) => Some(Err("no answer".to_owned())),
    }
}

/// `int.Parse(CMB_baudrate.Text)`: `Strings.InvalidBaudRate` for anything else.
pub fn baud_of(text: &str) -> Result<u32, String> {
    text.trim()
        .parse::<u32>()
        .map_err(|_| INVALID_BAUD_RATE.to_owned())
}

/// An item's probe id: `<combo>-<index>`, and for the four network kinds `<combo>-<the words
/// joined by ->` as well - `nmea-CMB_serialport-UDP-Client` - since the serial ports before
/// them are this machine's.
#[must_use]
pub fn item_id(combo: &str, index: usize, text: &str) -> String {
    if text.starts_with("TCP ") || text.starts_with("UDP ") {
        let words: Vec<&str> = text.split([' ', '-']).filter(|w| !w.is_empty()).collect();
        format!("{combo}-{}", words.join("-"))
    } else {
        format!("{combo}-{index}")
    }
}

/// A combo as the windows draw one: the text, an arrow, and its list below while it is open.
#[allow(clippy::too_many_arguments)] // the combo's parts
pub fn combo(
    id: &str,
    text: &str,
    items: &[String],
    open: bool,
    enabled: bool,
    (x, y, width, height): (f32, f32, f32, f32),
    on_toggle: impl Fn(&mut MissionPlanner) + 'static,
    on_choose: impl Fn(&mut MissionPlanner, usize) + 'static + Clone,
    cx: &mut Context<MissionPlanner>,
) -> AnyElement {
    let box_id = id.to_owned();
    let mut element = div().absolute().left(px(x)).top(px(y)).w(px(width));
    let head = crate::probe::measured(box_id.clone(), div())
        .id(SharedString::from(box_id))
        .w(px(width))
        .h(px(height))
        .px_1()
        .flex()
        .items_center()
        .justify_between()
        .bg(rgb(if enabled { theme::ACTION } else { theme::PANEL }))
        .border_1()
        .border_color(rgb(theme::BORDER))
        .rounded_sm()
        .text_xs()
        .whitespace_nowrap()
        .overflow_hidden()
        .text_color(rgb(if enabled { theme::TEXT } else { theme::DIM }))
        // The text kept inside the box: a long port name would otherwise widen what the probe
        // measures past the arrow.
        .child(
            div()
                .flex_1()
                .min_w(px(0.0))
                .overflow_hidden()
                .whitespace_nowrap()
                .child(text.to_owned()),
        )
        .child(div().flex_shrink_0().child("\u{25bc}"));
    let head = if enabled {
        head.cursor_pointer()
            .on_click(cx.listener(move |this, _event, _window, cx| {
                on_toggle(this);
                cx.notify();
            }))
    } else {
        head
    };
    element = element.child(head);
    if open && enabled {
        let list_id = format!("{id}-list");
        let mut list = div()
            .id(SharedString::from(list_id.clone()))
            .flex()
            .flex_col()
            .max_h(px(220.0))
            .overflow_scroll()
            .bg(rgb(theme::PANEL))
            .border_1()
            .border_color(rgb(theme::BORDER))
            .text_xs()
            .text_color(rgb(theme::TEXT));
        for (index, item) in items.iter().enumerate() {
            let item_id = item_id(id, index, item);
            let choose = on_choose.clone();
            list = list.child(
                crate::probe::measured(item_id.clone(), div())
                    .id(SharedString::from(item_id))
                    .px_1()
                    .whitespace_nowrap()
                    .cursor_pointer()
                    .hover(|style| style.bg(rgb(theme::BORDER)))
                    .child(item.clone())
                    .on_click(cx.listener(move |this, _event, _window, cx| {
                        choose(this, index);
                        cx.notify();
                    })),
            );
        }
        // Measured as the box it scrolls in, not as its items, which run on below it.
        element = element.child(crate::probe::measured(list_id, div()).child(list));
    }
    element.into_any_element()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The names, as the `switch` reads them, and each kind's questions under the window's
    /// `ConfigRef`.
    #[test]
    fn the_kinds_and_their_questions_are_the_transports() {
        assert_eq!(Kind::of("TCP Host - 14551"), Kind::TcpHost);
        assert_eq!(Kind::of("TCP Host"), Kind::TcpHost);
        assert_eq!(Kind::of("TCP Client"), Kind::TcpClient);
        assert_eq!(Kind::of("UDP Host - 14550"), Kind::UdpHost);
        assert_eq!(Kind::of("UDP Client"), Kind::UdpClient);
        assert_eq!(
            Kind::of("/dev/ttyACM0"),
            Kind::Serial("/dev/ttyACM0".to_owned())
        );
        let tcp = Kind::TcpClient.questions("_SerialOutputNMEATCP");
        assert_eq!(tcp.len(), 2);
        assert_eq!(tcp[0].key, "TCP_host_SerialOutputNMEATCP");
        assert_eq!(tcp[1].key, "TCP_port_SerialOutputNMEATCP");
        assert_eq!(tcp[1].default, "5760");
        let udp = Kind::UdpHost.questions("");
        assert_eq!(udp.len(), 1);
        assert_eq!(udp[0].key, "UDP_port");
        assert_eq!(udp[0].title, "Listern Port");
        assert!(Kind::TcpHost.questions("").is_empty());
        let items = port_items(14551);
        let tail: Vec<&str> = items
            .iter()
            .rev()
            .take(4)
            .rev()
            .map(String::as_str)
            .collect();
        assert_eq!(
            tail,
            [
                "TCP Host - 14551",
                "TCP Client",
                "UDP Host - 14551",
                "UDP Client"
            ]
        );
        assert_eq!(
            item_id("nmea-CMB_serialport", 7, "UDP Client"),
            "nmea-CMB_serialport-UDP-Client"
        );
        assert_eq!(
            item_id("nmea-CMB_serialport", 4, "TCP Host - 14551"),
            "nmea-CMB_serialport-TCP-Host-14551"
        );
        assert_eq!(
            item_id("nmea-CMB_serialport", 0, "/dev/ttyACM0"),
            "nmea-CMB_serialport-0"
        );
        assert_eq!(baud_of("57600"), Ok(57600));
        assert_eq!(baud_of("fast"), Err(INVALID_BAUD_RATE.to_owned()));
    }

    /// The questions asked in turn, offering the settings' value, each answer kept under its
    /// key; Cancel ends them.
    #[test]
    fn questions_are_asked_in_turn_and_their_answers_kept() {
        let mut persisted = Persisted::at(None);
        persisted.set("UDP_host", "10.0.0.2");
        let mut questions = Questions {
            pending: Kind::UdpClient.questions(""),
            ..Questions::default()
        };
        assert!(questions.ask_next(&persisted));
        assert_eq!(
            questions
                .asking
                .as_ref()
                .map(|a| a.input.field.value().to_owned()),
            Some("10.0.0.2".to_owned())
        );
        assert!(questions.answered(true, &mut persisted));
        assert!(questions.ask_next(&persisted));
        assert_eq!(
            questions
                .asking
                .as_ref()
                .map(|a| a.input.field.value().to_owned()),
            Some("14550".to_owned())
        );
        if let Some(asking) = questions.asking.as_mut() {
            asking.input.field.set("14560");
        }
        assert!(questions.answered(true, &mut persisted));
        assert!(!questions.ask_next(&persisted));
        assert_eq!(questions.answers, ["10.0.0.2", "14560"]);
        assert_eq!(persisted.get("UDP_port"), Some("14560"));
        let mut cancelled = Questions {
            pending: Kind::TcpClient.questions("_X"),
            ..Questions::default()
        };
        cancelled.ask_next(&persisted);
        assert!(!cancelled.answered(false, &mut persisted));
        assert!(cancelled.pending.is_empty());
    }

    /// A UDP client opens at once; a TCP client to nothing fails with the error's text.
    #[test]
    fn streams_open_on_their_thread() {
        let opening = open(
            Kind::UdpClient,
            0,
            0,
            vec!["127.0.0.1".into(), "14599".into()],
        );
        let opened = opening.recv().expect("an answer");
        assert!(opened.is_ok());
        let failed = open(Kind::TcpClient, 0, 0, vec!["127.0.0.1".into(), "1".into()]);
        assert!(failed.recv().expect("an answer").is_err());
        let bad_port = open(Kind::TcpClient, 0, 0, vec!["127.0.0.1".into(), "x".into()]);
        assert_eq!(
            bad_port.recv().expect("an answer").err(),
            Some("Input string was not in a correct format.".to_owned())
        );
    }
}
