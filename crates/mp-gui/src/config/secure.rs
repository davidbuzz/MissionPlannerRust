//! Secure: `GCSViews/ConfigurationView/ConfigSecureAP.cs`, the page Initial Setup lists at the top
//! while no vehicle is connected (`GCSViews/InitialSetup.cs:176-179`, `isDisConnected`), for
//! signing a bootloader and firmware with a key of one's own - ArduPilot's secure boot.
//!
//! What it shows, as its Designer places it in a 511 x 224 page: "Do Only Once", a group with
//! Generate Key; and "Files", a group with Private Key, BootLoader and Firmware, each with a text
//! box beside it - the public key, the bootloader's path and the firmware's path.
//!
//! What its four buttons do (`ConfigSecureAP.cs:30-111`, `ExtLibs/Utilities/SignedFW.cs`):
//!
//! * Generate Key makes an Ed25519 key pair (BouncyCastle's `Ed25519KeyPairGenerator`), asks where
//!   to save it, writes the private key as PEM and as `_private_key.dat` and the public key as
//!   `_public_key.dat` beside it, shows the public key in base64 in the first box and says "Protect
//!   your private key, if lost there is no method to get it back.";
//! * Private Key opens a `.pem` or `.dat` and reads the key pair from it - BouncyCastle's
//!   `PemReader`, or `PRIVATE_KEYV1:` and the base64 seed that `SignedFW.GenerateKey(byte[])` turns
//!   into a key - and shows its public key;
//! * BootLoader opens a `.bin`, finds its key table and writes ArduPilot's three public keys and the
//!   one loaded after it, saving `<name>-signed.bin`;
//! * Firmware opens an `.apj`, inflates its image, signs it with the key loaded (Ed25519 over the
//!   image without its descriptor), writes the signature into the descriptor with the SHA-512 of
//!   the image, and saves `<name>-signed.apj`.
//!
//! Every one of them is Ed25519 key generation, key reading or signing. This application has no
//! Ed25519 implementation: none of its crates depends on one, and a signing key made by curve
//! arithmetic written for the occasion is not a thing to hand anyone to lock their bootloader with.
//! Adding one is a dependency decision this page does not make. So the page is drawn as the
//! Designer draws it and each button is disabled, with the reason at the button; the text boxes
//! are drawn empty, as the Designer leaves them - nothing reads what is typed into them, and
//! without the buttons nothing writes them.
//!
//! The colours are this application's.

// This module is internal to the binary; `pub` here documents intent rather than exporting API.
#![allow(unreachable_pub)]

use gpui::{AnyElement, Div, div, prelude::*, px, rgb};

use crate::ui::{panel, theme};

/// The page's title in Initial Setup's list.
/// `// C#: GCSViews/InitialSetup.cs:178`
pub const TITLE: &str = "Secure";

/// Why every button is disabled.
pub const DISABLED_BECAUSE: &str = "Ed25519 key generation and signing (BouncyCastle in the C#) \
                                    have no implementation in this application";

/// A control's `Location` and `Size`.
pub type Place = (f32, f32, f32, f32);

/// `this.Size`.
/// `// C#: GCSViews/ConfigurationView/ConfigSecureAP.Designer.cs:136`
pub const PAGE_SIZE: (f32, f32) = (511.0, 224.0);

/// A group box: its name, text and place in the page.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Group {
    /// The Designer's name.
    pub name: &'static str,
    /// Its `Text`.
    pub text: &'static str,
    /// Its `Location` and `Size`.
    pub place: Place,
}

/// `groupBox5` and `groupBox4`.
/// `// C#: GCSViews/ConfigurationView/ConfigSecureAP.Designer.cs:86-93, 129-135`
pub const GROUPS: [Group; 2] = [
    Group {
        name: "groupBox5",
        text: "Do Only Once",
        place: (59.0, 3.0, 335.0, 55.0),
    },
    Group {
        name: "groupBox4",
        text: "Files",
        place: (59.0, 64.0, 335.0, 120.0),
    },
];

/// A button: its name, text, the group it is in (an index into [`GROUPS`]), its place in the
/// group, and the handler the Designer wires to it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Button {
    /// The Designer's name.
    pub name: &'static str,
    /// Its `Text`.
    pub text: &'static str,
    /// The group it is in.
    pub group: usize,
    /// Its `Location` and `Size` in the group.
    pub place: Place,
    /// Its `Click` handler.
    pub handler: &'static str,
}

/// The four buttons.
/// `// C#: GCSViews/ConfigurationView/ConfigSecureAP.Designer.cs:50-81, 119-128`
pub const BUTTONS: [Button; 4] = [
    Button {
        name: "but_generatekey",
        text: "Generate Key",
        group: 0,
        place: (6.0, 19.0, 75.0, 23.0),
        handler: "but_generatekey_Click",
    },
    Button {
        name: "but_privkey",
        text: "Private Key",
        group: 1,
        place: (6.0, 19.0, 75.0, 23.0),
        handler: "but_privkey_Click",
    },
    Button {
        name: "but_bootloader",
        text: "BootLoader",
        group: 1,
        place: (6.0, 48.0, 75.0, 23.0),
        handler: "but_bootloader_Click",
    },
    Button {
        name: "but_firmware",
        text: "Firmware",
        group: 1,
        place: (6.0, 77.0, 75.0, 23.0),
        handler: "but_firmware_Click",
    },
];

/// The three text boxes in `groupBox4`: the public key, the bootloader's path, the firmware's.
/// `// C#: GCSViews/ConfigurationView/ConfigSecureAP.Designer.cs:97-115`
pub const TEXT_BOXES: [(&str, Place); 3] = [
    ("txt_pubkey", (88.0, 21.0, 230.0, 20.0)),
    ("txt_bl", (88.0, 51.0, 230.0, 20.0)),
    ("txt_fwapj", (87.0, 80.0, 230.0, 20.0)),
];

/// Facts a UI test asserts on: the page's groups, buttons and text boxes, and that every button
/// is disabled and why. Recorded while the page is shown.
pub fn record_facts(showing: bool) {
    use crate::facts::record;
    record("config.secure.active", showing);
    record(
        "config.secure.groups",
        GROUPS
            .iter()
            .map(|group| group.text)
            .collect::<Vec<_>>()
            .join(","),
    );
    record(
        "config.secure.buttons",
        BUTTONS
            .iter()
            .map(|button| button.text)
            .collect::<Vec<_>>()
            .join(","),
    );
    record("config.secure.buttons.enabled", false);
    record("config.secure.disabled", DISABLED_BECAUSE);
    record(
        "config.secure.boxes",
        TEXT_BOXES
            .iter()
            .map(|(name, _)| *name)
            .collect::<Vec<_>>()
            .join(","),
    );
}

/// An absolutely placed box.
fn at((x, y, width, height): Place) -> Div {
    div()
        .absolute()
        .left(px(x))
        .top(px(y))
        .w(px(width))
        .h(px(height))
}

/// The page, as the Designer lays it out, every button disabled.
/// `// C#: GCSViews/ConfigurationView/ConfigSecureAP.Designer.cs:29-143`
pub fn page() -> AnyElement {
    let mut body = div().relative().w(px(PAGE_SIZE.0)).h(px(PAGE_SIZE.1));
    for (index, group) in GROUPS.iter().enumerate() {
        let mut groupbox = at(group.place)
            .border_1()
            .border_color(rgb(theme::BORDER))
            .rounded_sm()
            .child(
                div()
                    .absolute()
                    .left(px(6.0))
                    .top(px(-8.0))
                    .px_1()
                    .bg(rgb(theme::PANEL))
                    .text_xs()
                    .text_color(rgb(theme::DIM))
                    .child(group.text),
            );
        // Each button disabled: every handler is Ed25519 key generation, reading or signing,
        // which this application has no implementation of (see the module's notes).
        // `// C#: GCSViews/ConfigurationView/ConfigSecureAP.cs:30-111; ExtLibs/Utilities/SignedFW.cs:19-141`
        for button in BUTTONS.iter().filter(|button| button.group == index) {
            groupbox = groupbox.child(
                crate::probe::measured(format!("secure-{}", button.name), at(button.place))
                    .flex()
                    .items_center()
                    .justify_center()
                    .rounded_sm()
                    .border_1()
                    .border_color(rgb(theme::BORDER))
                    .bg(rgb(theme::PANEL))
                    .text_xs()
                    .text_color(rgb(theme::DIM))
                    .child(button.text),
            );
        }
        if index == 1 {
            for (name, place) in TEXT_BOXES {
                groupbox = groupbox.child(
                    crate::probe::measured(format!("secure-{name}"), at(place))
                        .border_1()
                        .border_color(rgb(theme::BORDER))
                        .bg(rgb(theme::BG)),
                );
            }
        }
        body = body.child(groupbox);
    }
    panel(TITLE, body).into_any_element()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The Designer's controls, places, texts and handlers, read from the tree when it is here:
    /// this page's Designer sets them in code rather than in the `.resx`.
    #[test]
    fn the_controls_are_the_designers() {
        let Some(designer) = crate::config_coverage::source::csharp(
            "GCSViews/ConfigurationView/ConfigSecureAP.Designer.cs",
        ) else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        let has = |line: &str| {
            assert!(designer.contains(line), "the Designer has no `{line}`");
        };
        let point = |name: &str, (x, y, w, h): Place| {
            has(&format!(
                "this.{name}.Location = new System.Drawing.Point({x}, {y});"
            ));
            has(&format!(
                "this.{name}.Size = new System.Drawing.Size({w}, {h});"
            ));
        };
        for group in GROUPS {
            point(group.name, group.place);
            has(&format!("this.{}.Text = \"{}\";", group.name, group.text));
        }
        for button in BUTTONS {
            point(button.name, button.place);
            has(&format!("this.{}.Text = \"{}\";", button.name, button.text));
            has(&format!(
                "this.{}.Click += new System.EventHandler(this.{});",
                button.name, button.handler
            ));
            let parent = GROUPS.get(button.group).map_or("", |group| group.name);
            has(&format!(
                "this.{parent}.Controls.Add(this.{});",
                button.name
            ));
        }
        for (name, place) in TEXT_BOXES {
            point(name, place);
            has(&format!("this.groupBox4.Controls.Add(this.{name});"));
        }
        has(&format!(
            "this.Size = new System.Drawing.Size({}, {});",
            PAGE_SIZE.0, PAGE_SIZE.1
        ));
        assert_eq!(designer.matches(" += new ").count(), BUTTONS.len());
    }

    /// Every handler is Ed25519: each calls `SignedFW` or BouncyCastle's Ed25519 types - the
    /// reason each button is disabled.
    #[test]
    fn every_handler_is_ed25519() {
        let Some(source) =
            crate::config_coverage::source::csharp("GCSViews/ConfigurationView/ConfigSecureAP.cs")
        else {
            eprintln!("skipped: the C# tree is not checked out");
            return;
        };
        for button in BUTTONS {
            let start = source
                .find(&format!("void {}(", button.handler))
                .expect("the handler");
            let body = &source[start..];
            let end = body.find("\n        }").expect("its end");
            let body = &body[..end];
            assert!(
                body.contains("SignedFW.") || body.contains("Ed25519"),
                "{} is not Ed25519",
                button.handler
            );
        }
    }

    /// Every fact the script asserts on is recorded here, and the script clicks no button.
    #[test]
    fn the_gui_script_names_facts_this_page_has() {
        let script = include_str!("../../../../tests/gui/config-secure.gui");
        let source = include_str!("secure.rs");
        let mut facts = 0;
        for line in script.lines() {
            let line = line.split('#').next().unwrap_or("");
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some("expect"), Some(key)) if key.starts_with("config.secure.") => {
                    assert!(source.contains(&format!("\"{key}\"")), "{key}");
                    facts += 1;
                }
                (Some("click"), Some(id)) => {
                    assert!(!id.starts_with("secure-"), "the buttons are disabled");
                }
                _ => {}
            }
        }
        assert!(facts >= 5, "{facts} facts");
    }
}
