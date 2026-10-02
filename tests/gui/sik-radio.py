#!/usr/bin/env python3
"""A SiK radio for the GUI script of SETUP's Sik Radio page (Radio/Sikradio.cs): an RFD900+ on
RFD's SiK 2.65, 915 MHz, with a remote one over the air, reached over TCP as the page reaches a
radio when the link is TCP (Radio/Sikradio.cs:236-244).

usage: sik-radio.py --work DIR [--port 5795]

It answers what the page says to a radio:

  transparent mode until `+++` arrives after a second of quiet, then "OK" after another second
  and AT command mode (SikRadio/RFD900.cs:198-208, 409-425); in AT command mode each character
  echoed, a carriage return as "\\r\\n", and the line acted on - ATI, ATI2, ATI3, ATI5, ATI5?
  (with the ranges and options), ATI7, ATI10:n ("ERROR": 2.65 has no ATI10), AT&E?, AT&E=,
  ATSn=v, AT&W, AT&F, ATZ, ATO, AT&T, AT&R, ATPO=1, ATPI=1, AT+Cnn? ("ERROR"), AT&UPDATE; RT...
  the same of the remote radio, its answer a little later;
  the SiK bootloader after AT&UPDATE (Radio/Uploader.cs): GET_SYNC, GET_DEVICE, CHIP_ERASE,
  LOAD_ADDRESS (24 bits: the RFD900+ banks), PROG_MULTI, READ_MULTI, REBOOT;
  the RFD900x bootloader's CHIPID is not answered: this is not an x.

The radios keep their state from one connection to the next, as a radio does when the planner's
link closes and the page opens its own port. At start it writes DIR/rfd900p.hex, an Intel HEX
firmware holding "RFD900P" (what RFD900p.GetFirmwareSearchTokens looks for), for Upload Firmware
(custom).

Listens on 127.0.0.1:PORT, one client at a time, and returns once listening, leaving itself
serving in the background. It goes when DIR does, which the harness removes after the run, or
after five minutes.
"""
import argparse
import os
import socket
import sys
import time

import gui_background

LIFETIME = 300.0
GUARD = 1.0

# Radio/Uploader.cs:35-56
OK, INSYNC, EOC = 0x10, 0x12, 0x20
GET_SYNC, GET_DEVICE, CHIP_ERASE, LOAD_ADDRESS = 0x21, 0x22, 0x23, 0x24
PROG_MULTI, READ_MULTI, REBOOT = 0x27, 0x28, 0x30

# Designator, name, value, what ATI5? adds after the name, what it adds after the value.
PARAMS = [
    (0, "FORMAT", 25, "(R)[0..255]", ""),
    (1, "SERIAL_SPEED", 57, "(N)[1..115]", "{1200,2400,4800,9600,19200,38400,57600,115200,}"),
    (2, "AIR_SPEED", 64, "(N)[2..250]", "{2,4,8,16,19,24,32,48,64,96,128,192,250,}"),
    (3, "NETID", 25, "(N)[0..499]", ""),
    (4, "TXPOWER", 30, "(N)[0..30]", ""),
    (5, "ECC", 0, "(N)[0..1]", ""),
    (6, "MAVLINK", 1, "(N)[0..2]", "{RawData,Mavlink,LowLatency,}"),
    (7, "OPPRESEND", 1, "(N)[0..1]", ""),
    (8, "MIN_FREQ", 915000, "(N)[902000..927000]", ""),
    (9, "MAX_FREQ", 928000, "(N)[903000..928000]", ""),
    (10, "NUM_CHANNELS", 20, "(N)[1..50]", ""),
    (11, "DUTY_CYCLE", 100, "(N)[10..100]", ""),
    (12, "LBT_RSSI", 0, "(N)[0..220]", ""),
    (13, "MANCHESTER", 0, "(N)[0..1]", ""),
    (14, "RTSCTS", 0, "(N)[0..1]", ""),
    (15, "MAX_WINDOW", 131, "(N)[20..400]", ""),
    (16, "ENCRYPTION_LEVEL", 0, "(N)[0..1]", "{Off,128b,}"),
]


class Unit:
    """One radio: its registers, what &W saved, its key."""

    def __init__(self, rssi):
        self.values = {p[0]: p[2] for p in PARAMS}
        self.eeprom = dict(self.values)
        self.key = "0" * 32
        self.rssi = rssi

    def query_lines(self):
        return "".join(
            "S%d:%s%s=%d%s\r\n" % (n, name, rng, self.values[n], opts)
            for n, name, _, rng, opts in PARAMS
        )

    def plain_lines(self):
        return "".join("S%d:%s=%d\r\n" % (n, name, self.values[n]) for n, name, *_ in PARAMS)

    def act(self, rest):
        """The answer to AT + rest, and what it does: (text or None, effect)."""
        if rest == "I":
            return "RFD SiK 2.65 on RFD900P\r\n", None
        if rest == "I2":
            return "130\r\n", None
        if rest == "I3":
            return "145\r\n", None
        if rest == "I5":
            return self.plain_lines(), None
        if rest == "I5?":
            return self.query_lines(), None
        if rest == "I7":
            return self.rssi + "\r\n", None
        if rest.startswith("I10:"):
            return "ERROR\r\n", None
        if rest == "&E?":
            return self.key + "\r\n", None
        if rest.startswith("&E="):
            self.key = rest[3:]
            return "OK\r\n", None
        if rest == "&W":
            self.eeprom = dict(self.values)
            return "OK\r\n", None
        if rest == "&F":
            self.values = {p[0]: p[2] for p in PARAMS}
            return "OK\r\n", None
        if rest == "Z":
            self.values = dict(self.eeprom)
            return None, "reboot"
        if rest == "O":
            return None, "transparent"
        if rest == "&T":
            return None, None
        if rest in ("&R", "PO=1", "PI=1"):
            return "OK\r\n", None
        if rest == "&UPDATE":
            return None, "bootloader"
        if rest.startswith("+C") and rest.endswith("?"):
            return "ERROR\r\n", None
        if rest.startswith("S") and "=" in rest:
            number, _, value = rest[1:].partition("=")
            try:
                number, value = int(number), int(value)
            except ValueError:
                return "ERROR\r\n", None
            if number in self.values:
                self.values[number] = value
                return "OK\r\n", None
        return "ERROR\r\n", None


class Radio:
    """The local radio, its mode, and the remote radio it reaches."""

    def __init__(self):
        self.local = Unit("L/R RSSI: 210/198  L/R noise: 45/40 pkts: 120  txe=0 rxe=0 stx=0 srx=0 ecc=0/0 temp=31 dco=0")
        self.remote = Unit("L/R RSSI: 199/211  L/R noise: 41/44 pkts: 118  txe=0 rxe=0 stx=0 srx=0 ecc=0/0 temp=30 dco=0")
        self.mode = "transparent"
        self.line = ""
        self.last_rx = 0.0
        self.escape_at = None
        self.boot = bytearray()
        self.flash = {}
        self.address = 0
        self.later = []  # (when, bytes): the remote radio's answers on their way

    def feed(self, data, now):
        """Bytes from the page; what goes back at once."""
        out = bytearray()
        quiet = now - self.last_rx
        self.last_rx = now
        if self.mode == "transparent":
            if data == b"+++" and quiet >= GUARD * 0.9:
                self.escape_at = now
            else:
                self.escape_at = None
            return bytes(out)
        if self.mode == "bootloader":
            self.boot.extend(data)
            out.extend(self.bootloader())
            return bytes(out)
        for byte in data:
            if byte == 0x0D:
                out.extend(b"\r\n")
                line, self.line = self.line.strip().upper(), ""
                out.extend(self.execute(line, now))
            elif byte == 0x0A:
                continue
            elif 0x20 <= byte < 0x7F:
                self.line += chr(byte)
                out.append(byte)
        return bytes(out)

    def execute(self, line, now):
        if line.startswith("AT"):
            text, effect = self.local.act(line[2:])
            if effect in ("reboot", "transparent"):
                self.mode = "transparent"
            elif effect == "bootloader":
                self.mode = "bootloader"
                self.boot.clear()
            return (text or "").encode()
        if line.startswith("RT"):
            text, _ = self.remote.act(line[2:])
            if text:
                self.later.append((now + 0.08, text.encode()))
        return b""

    def poll(self, now):
        """What is due now: the "OK" a second after +++, the remote radio's answers."""
        out = bytearray()
        if self.escape_at is not None and now - self.escape_at >= GUARD:
            self.escape_at = None
            self.mode = "command"
            self.line = ""
            out.extend(b"OK\r\n")
        due = [item for item in self.later if item[0] <= now]
        self.later = [item for item in self.later if item[0] > now]
        for _, text in due:
            out.extend(text)
        return bytes(out)

    def bootloader(self):
        out = bytearray()
        while self.boot:
            command = self.boot[0]
            if command in (GET_SYNC, GET_DEVICE, CHIP_ERASE):
                need = 2
            elif command == LOAD_ADDRESS:
                need = 5
            elif command == PROG_MULTI:
                if len(self.boot) < 2:
                    break
                need = 3 + self.boot[1]
            elif command == READ_MULTI:
                need = 3
            elif command == REBOOT:
                need = 1
            else:
                del self.boot[0]
                continue
            if len(self.boot) < need:
                break
            frame = bytes(self.boot[:need])
            del self.boot[:need]
            if command != REBOOT and frame[-1] != EOC:
                continue
            if command == GET_SYNC:
                out.extend([INSYNC, OK])
            elif command == GET_DEVICE:
                out.extend([0x82, 0x91, INSYNC, OK])
            elif command == CHIP_ERASE:
                self.flash.clear()
                out.extend([INSYNC, OK])
            elif command == LOAD_ADDRESS:
                self.address = frame[1] | (frame[2] << 8) | (frame[3] << 16)
                out.extend([INSYNC, OK])
            elif command == PROG_MULTI:
                for byte in frame[2:-1]:
                    self.flash[self.address] = byte
                    self.address += 1
                out.extend([INSYNC, OK])
            elif command == READ_MULTI:
                for _ in range(frame[1]):
                    out.append(self.flash.get(self.address, 0xFF))
                    self.address += 1
                out.extend([INSYNC, OK])
            elif command == REBOOT:
                self.mode = "transparent"
        return bytes(out)


def write_firmware(work):
    """DIR/rfd900p.hex: sixteen-byte data records holding "RFD900P", and the end record."""
    data = b"RFD900P firmware for the Sik Radio page's GUI script.".ljust(64, b"\xff")
    lines = []
    for offset in range(0, len(data), 16):
        chunk = data[offset:offset + 16]
        record = bytes([len(chunk), offset >> 8, offset & 0xFF, 0]) + chunk
        checksum = (-sum(record)) & 0xFF
        lines.append(":" + record.hex().upper() + "%02X" % checksum)
    lines.append(":00000001FF")
    with open(os.path.join(work, "rfd900p.hex"), "w") as f:
        f.write("\n".join(lines) + "\n")


def serve_client(client, radio, work, started):
    client.settimeout(0.02)
    while os.path.isdir(work) and time.monotonic() - started < LIFETIME:
        try:
            data = client.recv(4096)
            if not data:
                return
            out = radio.feed(data, time.monotonic())
        except socket.timeout:
            out = b""
        except OSError:
            return
        out += radio.poll(time.monotonic())
        if out:
            try:
                client.sendall(out)
            except OSError:
                return


def run(listener, work):
    started = time.monotonic()
    radio = Radio()
    listener.settimeout(0.5)
    while os.path.isdir(work) and time.monotonic() - started < LIFETIME:
        try:
            client, _ = listener.accept()
        except socket.timeout:
            continue
        with client:
            serve_client(client, radio, work, started)


def main():
    child = gui_background.child_listener()
    parser = argparse.ArgumentParser()
    parser.add_argument("--work", required=True)
    parser.add_argument("--port", type=int, default=5795)
    args = parser.parse_args()
    if child is not None:
        run(child, args.work)
        return 0

    write_firmware(args.work)
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # The last script's radio may still be leaving: it looks for its directory twice a second.
    deadline = time.monotonic() + 5.0
    while True:
        try:
            listener.bind(("127.0.0.1", args.port))
            break
        except OSError:
            if time.monotonic() > deadline:
                print(f"port {args.port} is busy", file=sys.stderr)
                return 3
            time.sleep(0.2)
    listener.listen(1)

    # Into the background: the harness waits for this command, and the radio must outlive it.
    gui_background.serve_in_background(
        listener, f"sik radio on {args.port}", lambda held: run(held, args.work)
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
