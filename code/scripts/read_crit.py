#!/usr/bin/env python3
"""Read the critical (CRIT) log lines from a viikate board and check it booted.

Every boot logs a CRIT boot marker first thing in main(). This script finds
the CRIT lines in one of two places:

  serial   The board's USB console. Start the script, then plug in or reset
           the board: lines printed before the port is opened are lost.

               read_crit.py serial [--port /dev/ttyACM0] [--timeout 30]

  sd       The microSD card, mounted on this computer. Reads log.txt and its
           rotated files (log.1.txt is newer than log.2.txt, and so on).

               read_crit.py sd /media/$USER/VIIKATE

It prints every CRIT line it finds and exits 0 if the boot marker was among
them, 1 if not. crit.txt on the onboard flash cannot be read from a host.
"""

import argparse
import glob
import os
import re
import sys
import time

BOOT_MARKER = "(half life scientist) everything.. seems to be in order"
CONSOLE_USB_ID = (0x2FE3, 0x0004)
LINE = re.compile(r"^\s*(\d+) (INFO|WARN|CRIT) (.*)$")


def crit_lines(lines):
    for line in lines:
        m = LINE.match(line.rstrip("\r\n"))
        if m and m.group(2) == "CRIT":
            yield int(m.group(1)), m.group(3)


def from_sd(root):
    files = glob.glob(os.path.join(root, "log.*.txt"))
    files.sort(key=lambda p: int(p.rsplit(".", 2)[1]), reverse=True)
    newest = os.path.join(root, "log.txt")
    if os.path.exists(newest):
        files.append(newest)
    if not files:
        sys.exit(f"no log.txt in {root}; is the card mounted there?")

    found = False
    for path in files:
        with open(path, errors="replace") as f:
            for ms, text in crit_lines(f):
                print(f"{os.path.basename(path)}: {ms} CRIT {text}")
                found |= text == BOOT_MARKER
    return found


def find_console():
    from serial.tools import list_ports

    for p in list_ports.comports():
        if (p.vid, p.pid) == CONSOLE_USB_ID:
            return p.device
    return None


def from_serial(port, timeout):
    import serial

    deadline = time.monotonic() + timeout
    print("waiting for the board; plug it in or reset it", file=sys.stderr)
    console = None
    buf = b""
    while time.monotonic() < deadline:
        # The port goes away on every reset; wait for it to come back.
        if console is None:
            dev = port or find_console()
            try:
                console = serial.Serial(dev, 115200, timeout=0.2) if dev else None
            except (serial.SerialException, OSError):
                console = None
            if console is None:
                time.sleep(0.05)
                continue
            buf = b""
        try:
            buf += console.read(4096)
        except (serial.SerialException, OSError):
            console.close()
            console = None
            continue
        *done, buf = buf.split(b"\n")
        for ms, text in crit_lines(l.decode(errors="replace") for l in done):
            print(f"{ms} CRIT {text}")
            if text == BOOT_MARKER:
                return True
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="source", required=True)
    s = sub.add_parser("serial", help="watch the USB console as the board boots")
    s.add_argument("--port", help="serial device (default: find the viikate console)")
    s.add_argument("--timeout", type=float, default=30, help="seconds to wait (default 30)")
    d = sub.add_parser("sd", help="read the logs on a mounted microSD card")
    d.add_argument("root", help="where the card is mounted")
    args = ap.parse_args()

    booted = from_serial(args.port, args.timeout) if args.source == "serial" else from_sd(args.root)
    print("booted: boot marker found" if booted else "NOT booted: no boot marker", file=sys.stderr)
    return 0 if booted else 1


if __name__ == "__main__":
    sys.exit(main())
