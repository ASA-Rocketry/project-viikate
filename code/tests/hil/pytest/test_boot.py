"""HIL boot test: the flight image must boot on the Teensy 4.1 from a cold power-on.

Before these tests run, the twister `dut` fixture has flashed the image,
cut and restored USB power (post_flash_script), and attached the console.

"Booted" means the kernel got through every init level: the USB CDC console
is started at the APPLICATION init level, after all drivers have probed, so
the board enumerating with the image's VID:PID proves boot did not stall.
"""

import re
import subprocess
import time
from pathlib import Path

from twister_harness import DeviceAdapter

RIG = Path(__file__).resolve().parents[1] / "scripts" / "rig.sh"

# Cold power-on to USB enumeration: ROM boot + CONFIG_BOOT_DELAY (1 s) +
# driver init + USB enumeration, with generous margin.
ENUMERATION_TIMEOUT_S = 20
# How long the board must then stay up without resetting.
STABLE_S = 10

FATAL_PATTERNS = re.compile(r"ZEPHYR FATAL ERROR|Halting system|Kernel panic")


def usb_id(build_dir: Path) -> tuple[str, str]:
    """USB VID/PID the image's CDC ACM console enumerates with, from its .config."""
    config = (build_dir / "zephyr" / ".config").read_text()
    ids = []
    for key in ("CDC_ACM_SERIAL_VID", "CDC_ACM_SERIAL_PID"):
        match = re.search(rf"^CONFIG_{key}=(0x[0-9a-fA-F]+)$", config, re.MULTILINE)
        assert match, f"CONFIG_{key} missing from {build_dir}/zephyr/.config"
        ids.append(f"{int(match.group(1), 16):04x}")
    return ids[0], ids[1]


def usb_device(vid: str, pid: str, timeout_s: float) -> str | None:
    """Return "<bus>-<devnum> <tty>" for the device, or None if it never appeared."""
    result = subprocess.run(
        [str(RIG), "usb-device", vid, pid, "--timeout", str(int(timeout_s))],
        capture_output=True,
        text=True,
        check=False,
    )
    return result.stdout.strip() if result.returncode == 0 else None


def test_cold_boot_enumerates_usb(dut: DeviceAdapter):
    vid, pid = usb_id(Path(dut.device_config.build_dir))
    device = usb_device(vid, pid, ENUMERATION_TIMEOUT_S)
    assert device, (
        f"board did not enumerate as {vid}:{pid} within {ENUMERATION_TIMEOUT_S}s "
        "of power-on: boot stalled before the APPLICATION init level"
    )


def test_stays_up_without_fatal_error(dut: DeviceAdapter):
    vid, pid = usb_id(Path(dut.device_config.build_dir))
    before = usb_device(vid, pid, ENUMERATION_TIMEOUT_S)
    assert before, f"board not enumerated as {vid}:{pid}"

    time.sleep(STABLE_S)

    after = usb_device(vid, pid, 0)
    # A reset re-enumerates the device with a new device number.
    assert after == before, (
        f"board reset or dropped off USB within {STABLE_S}s (before: {before}, after: {after})"
    )

    output = dut.readlines(print_output=True)
    fatal = [line for line in output if FATAL_PATTERNS.search(line)]
    assert not fatal, f"fatal error on console: {fatal}"
