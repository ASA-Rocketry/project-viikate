#!/usr/bin/env python3
"""Build, run and test the Viikate software-in-the-loop simulation.

The unmodified firmware in code/app is built for Zephyr's native_sim board
together with the sim module in code/sim, which provides simulated sensors
and servos driven by a plant model. Linux x86_64 only (use the Dockerfile on
other hosts).

    python3 code/sim/sil.py build --plant toy
    python3 code/sim/sil.py run --plant toy --stop-at 6 \
        --plant-param kick_roll_rate_radps=2
    python3 code/sim/sil.py test

Each plant gets its own build directory, build/sil-<plant>.
"""

import argparse
import csv
import math
import os
import subprocess
import sys
import unittest
from dataclasses import dataclass, field
from pathlib import Path

SIM_DIR = Path(__file__).resolve().parent
CODE_DIR = SIM_DIR.parent
REPO_DIR = CODE_DIR.parent
BUILD_ROOT = REPO_DIR / "build"
PLANTS = ("toy", "rocket")
BOARD = "native_sim/native/64"
RUN_TIMEOUT_S = 300


class SilError(RuntimeError):
    pass


@dataclass
class SimResult:
    plant: str
    interface_version: int
    columns: list
    rows: list = field(default_factory=list)
    log: str = ""

    def column(self, name):
        return [row[name] for row in self.rows]

    def between(self, t0_ms, t1_ms):
        return [row for row in self.rows if t0_ms <= row["t_ms"] <= t1_ms]

    def write_csv(self, path):
        with open(path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=self.columns)
            writer.writeheader()
            writer.writerows(self.rows)


def _check_plant(plant):
    if plant not in PLANTS:
        raise SilError(f"unknown plant {plant!r}, expected one of {PLANTS}")


def build_dir(plant):
    _check_plant(plant)
    return BUILD_ROOT / f"sil-{plant}"


def exe_path(plant):
    return build_dir(plant) / "zephyr" / "zephyr.exe"


def build(plant="toy", pristine=False):
    """Build code/app for native_sim with the sim module and the given
    plant. Returns the path of zephyr.exe."""
    cmd = [
        "west",
        "build",
        "-b",
        BOARD,
        "-d",
        str(build_dir(plant)),
        "-p",
        "always" if pristine else "auto",
        str(CODE_DIR / "app"),
        "--",
        f"-DZEPHYR_EXTRA_MODULES={SIM_DIR}",
        f"-DDTC_OVERLAY_FILE={SIM_DIR / 'native_sim.overlay'}",
        f"-DEXTRA_CONF_FILE={SIM_DIR / 'native_sim.conf'}",
        f"-DCONFIG_VIIKATE_SIM_PLANT_{plant.upper()}=y",
    ]
    env = dict(os.environ, ZEPHYR_TOOLCHAIN_VARIANT="host")
    subprocess.run(cmd, check=True, env=env)
    return exe_path(plant)


def parse_log(text):
    header = None
    rows = []
    for line in text.splitlines():
        if line.startswith("SIMHDR,"):
            fields = line.split(",")
            header = (fields[1], int(fields[2]), fields[3:])
        elif line.startswith("SIM,"):
            if header is None:
                raise SilError("SIM line before SIMHDR")
            values = [float(v) for v in line.split(",")[1:]]
            if len(values) != len(header[2]):
                raise SilError(f"malformed SIM line: {line!r}")
            rows.append(dict(zip(header[2], values)))
    if header is None:
        raise SilError("no SIMHDR line in output, did the sim start?")
    return SimResult(header[0], header[1], header[2], rows, text)


def run(exe, stop_at_s, plant_params=None, servo_signs=None):
    """Run zephyr.exe in simulated time and return the parsed log."""
    cmd = [str(exe), "--no-rt", f"--stop_at={stop_at_s}"]
    if plant_params:
        cmd.append(
            "--plant-params="
            + ",".join(f"{k}={v}" for k, v in plant_params.items())
        )
    if servo_signs:
        cmd.append("--servo-signs=" + ",".join(f"{s:+d}" for s in servo_signs))
    proc = subprocess.run(
        cmd, capture_output=True, text=True, timeout=RUN_TIMEOUT_S
    )
    if proc.returncode != 0:
        raise SilError(
            f"{' '.join(cmd)} exited with {proc.returncode}\n"
            f"{proc.stdout[-2000:]}{proc.stderr[-2000:]}"
        )
    return parse_log(proc.stdout)


def _key_value(text):
    name, sep, value = text.partition("=")
    if not sep:
        raise argparse.ArgumentTypeError(f"expected name=value, got {text!r}")
    return name, float(value)


def _signs(text):
    try:
        signs = [int(s) for s in text.split(",")]
    except ValueError:
        signs = []
    if len(signs) != 4 or any(s not in (1, -1) for s in signs):
        raise argparse.ArgumentTypeError("expected four of +1/-1, e.g. 1,1,-1,-1")
    return signs


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="command", required=True)

    p_build = sub.add_parser("build", help="build zephyr.exe for native_sim")
    p_build.add_argument("--plant", choices=PLANTS, default="toy")
    p_build.add_argument("--pristine", action="store_true")

    p_run = sub.add_parser("run", help="run one scenario")
    p_run.add_argument("--plant", choices=PLANTS, default="toy")
    p_run.add_argument("--stop-at", type=float, default=6.0, help="seconds")
    p_run.add_argument(
        "--plant-param", type=_key_value, action="append", default=[]
    )
    p_run.add_argument("--servo-signs", type=_signs)
    p_run.add_argument("--csv", type=Path, help="write the SIM rows here")
    p_run.add_argument("--no-build", action="store_true")

    sub.add_parser("test", help="build, then run the tests in code/sim/tests")

    args = parser.parse_args(argv)

    if args.command == "build":
        build(args.plant, args.pristine)
        return 0

    if args.command == "run":
        exe = exe_path(args.plant) if args.no_build else build(args.plant)
        result = run(
            exe, args.stop_at, dict(args.plant_param), args.servo_signs
        )
        if args.csv:
            result.write_csv(args.csv)
        else:
            print(",".join(result.columns))
            for row in result.rows:
                print(",".join(f"{row[c]:g}" for c in result.columns))
        return 0

    os.environ["SIL_EXE_TOY"] = str(build("toy"))
    suite = unittest.defaultTestLoader.discover(
        str(SIM_DIR / "tests"), top_level_dir=str(SIM_DIR)
    )
    outcome = unittest.TextTestRunner(verbosity=2).run(suite)
    return 0 if outcome.wasSuccessful() else 1


if __name__ == "__main__":
    sys.exit(main())
