"""HIL log test: viikate_log() lines land in the log file on the Teensy's
flash, survive a power cut, and the file rotates without losing or splitting
lines.

Before these tests run, the twister `dut` fixture has flashed the image and
cut and restored USB power (post_flash_script). The image is the flight
application plus a shell (shell.conf); files are read back with `fs cat`.
"""

import re
import secrets
import subprocess
import time
from collections.abc import Callable
from pathlib import Path

from twister_harness import DeviceAdapter, Shell
from twister_harness.fixtures import get_ready_shell

HERE = Path(__file__).resolve().parent
RIG = HERE.parent / "scripts" / "rig.sh"
LOGGER_H = HERE.parents[2] / "app" / "include" / "logger.h"

# Lines main() logs on every boot.
STATUS = re.compile(r"(barometer|imu)\.\.\. (ok|failed)")
# Must match FILL_PADDING in src/log_shell.cpp.
FILL_PADDING = "0123456789abcdefghijklmnopqrstuvwxyz"


def logger_define(name: str) -> str:
    match = re.search(rf"^#define {name} (.+)$", LOGGER_H.read_text(), re.MULTILINE)
    assert match, f"{name} not defined in {LOGGER_H}"
    return match.group(1).strip()


LOG_DIR = logger_define("VIIKATE_LOG_DIR").strip('"')
LOG_FILE = f"{LOG_DIR}/log.txt"
MAX_SIZE = int(eval(logger_define("VIIKATE_LOG_FILE_MAX_SIZE"), {"__builtins__": {}}))
FILE_COUNT = int(logger_define("VIIKATE_LOG_FILE_COUNT"))
# Oldest first, the order lines were written in.
LOG_FILES = [f"{LOG_DIR}/log.{i}.txt" for i in range(FILE_COUNT - 1, 0, -1)] + [LOG_FILE]


def connect_shell(dut: DeviceAdapter) -> Shell:
    shell = get_ready_shell(dut)
    shell.exec_command("shell colors off", print_output=False)
    return shell


def power_cycle(dut: DeviceAdapter) -> Shell:
    """Cut USB power to the board, restore it and reattach the shell."""
    dut.disconnect()
    subprocess.run([str(RIG), "power-cycle"], check=True, timeout=60)
    dut.connect()
    return connect_shell(dut)


def read_file(shell: Shell, path: str) -> list[str] | None:
    """Lines of a file on the board, or None if it does not exist."""
    output = shell.exec_command(f"fs cat {path}", timeout=120, print_output=False)
    lines = [line for line in shell.get_filtered_output(output) if line]
    if any(line.startswith(f"Failed to obtain file {path}") for line in lines):
        return None
    return lines


def read_log(shell: Shell) -> dict[str, list[str]]:
    """Every existing log file, oldest first."""
    files = {}
    for path in LOG_FILES:
        lines = read_file(shell, path)
        if lines is not None:
            files[path] = lines
    return files


def all_lines(files: dict[str, list[str]]) -> list[str]:
    return [line for lines in files.values() for line in lines]


def poll(read: Callable[[], list[str]], done: Callable[[list[str]], bool], timeout_s: float):
    """Re-read until done(lines) holds; return the last read either way."""
    end = time.monotonic() + timeout_s
    while True:
        lines = read()
        if done(lines) or time.monotonic() >= end:
            return lines
        time.sleep(2)


def lines_after(tail: list[str], lines: list[str]) -> list[str] | None:
    """Lines following the last occurrence of tail in lines, None if absent."""
    for start in range(len(lines) - len(tail), -1, -1):
        if lines[start : start + len(tail)] == tail:
            return lines[start + len(tail) :]
    return None


def file_size(lines: list[str]) -> int:
    return sum(len(line.encode()) + 1 for line in lines)


def test_log_survives_power_cut(dut: DeviceAdapter):
    shell = connect_shell(dut)

    def ends_with_status(lines: list[str]) -> bool:
        return len(lines) >= 2 and all(STATUS.fullmatch(line) for line in lines[-2:])

    # main() logs barometer and imu status on boot; wait until it is on flash.
    before = poll(lambda: all_lines(read_log(shell)), ends_with_status, 30)
    assert ends_with_status(before), f"boot status lines not in the log, log ends with {before[-5:]}"

    shell = power_cycle(dut)

    tail = before[-20:]
    after = poll(
        lambda: all_lines(read_log(shell)),
        lambda lines: len(lines_after(tail, lines) or []) >= 2,
        30,
    )
    new = lines_after(tail, after)
    assert new is not None, "lines written before the power cut are gone or corrupted"
    assert len(new) == 2 and all(STATUS.fullmatch(line) for line in new), (
        f"expected this boot's barometer and imu lines after the old log, got {new}"
    )
    assert new[0].startswith("barometer") and new[1].startswith("imu")


def test_log_rotates(dut: DeviceAdapter):
    shell = connect_shell(dut)
    tag = secrets.token_hex(3)
    fill = re.compile(rf"fill {tag} (\d+) {FILL_PADDING}")
    line_size = len(f"fill {tag} 0000 {FILL_PADDING}\n")
    # One and a half files' worth: forces at least one rotation, and with the
    # rotated files kept, every line of this run must still be on the board.
    count = MAX_SIZE * 3 // 2 // line_size
    last = f"fill {tag} {count - 1} {FILL_PADDING}"

    shell.exec_command(f"vlog fill {tag} {count}", timeout=300, print_output=False)
    current = poll(lambda: read_file(shell, LOG_FILE) or [], lambda lines: last in lines, 120)
    assert last in current, "the last filled line never reached the log file"

    # Rotated files must survive a power cut too.
    shell = power_cycle(dut)
    files = read_log(shell)

    assert f"{LOG_DIR}/log.1.txt" in files, f"no rotation after {count} lines; files: {list(files)}"
    assert read_file(shell, f"{LOG_DIR}/log.{FILE_COUNT}.txt") is None, (
        f"more than {FILE_COUNT} log files kept"
    )
    for path, lines in files.items():
        assert file_size(lines) <= MAX_SIZE, f"{path} is {file_size(lines)} bytes, limit {MAX_SIZE}"

    lines = all_lines(files)
    torn = [line for line in lines if tag in line and not fill.fullmatch(line)]
    assert not torn, f"partial or corrupted lines: {torn[:5]}"
    numbers = [int(m.group(1)) for line in lines if (m := fill.fullmatch(line))]
    assert numbers == list(range(count)), (
        f"lines lost, duplicated or reordered: got {len(numbers)} of {count}"
    )
    holding = [path for path, lines in files.items() if any(tag in line for line in lines)]
    assert len(holding) >= 2, f"this run's lines did not span a rotation: {holding}"
