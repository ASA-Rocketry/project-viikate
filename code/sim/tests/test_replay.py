"""The generated rocket plant plus its C adapter, stepped without Zephyr by
code/sim/tools/plant_replay.c, against the reference scenario Simulink
produced in simulation/plant_codegen/reference (generate_plant.m)."""

import csv
import math
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

from tests import common, rocket_design
from tests.common import sil

EXE = None
REF_DIR = sil.REPO_DIR / "simulation" / "plant_codegen" / "reference"
REF_INPUT = REF_DIR / "input.csv"
REF_PARAMS = (REF_DIR / "params.txt").read_text().split()
V1 = rocket_design.ROCKETS_DIR / "viikate_v1.rocket"
V1_MOTOR = rocket_design.ROCKETS_DIR / "motors" / "viikate_v1.eng"
PERTURBED = rocket_design.ROCKETS_DIR / "test" / "perturbed.rocket"
H = math.sqrt(0.5)
G_MODEL = 9.819
# Model inertial frame (x up) to NED.
P = [[0, 0, 1], [0, 1, 0], [-1, 0, 0]]


def setUpModule():
    global EXE
    EXE = common.replay()


def replay(*args):
    return sil.run_replay(EXE, [str(a) for a in args])


def replay_proc(*args, cwd=None):
    """plant_replay as a finished process, for exit codes and stderr."""
    return subprocess.run(
        [str(EXE), *[str(a) for a in args]],
        capture_output=True, text=True, cwd=cwd, timeout=sil.RUN_TIMEOUT_S,
    )


def changed_design(key, count):
    """viikate_v1 with one key changed: x1.2, or 1e-4 for the absent
    products of inertia."""
    lines = V1.read_text().splitlines()
    if key not in rocket_design.load(V1) or key.endswith("_products_kgm2"):
        lines.append(f"{key} = " + ", ".join(["1e-4"] * count))
        return "\n".join(lines) + "\n"
    for i, line in enumerate(lines):
        name, sep, value = line.partition("=")
        if sep and name.strip() == key:
            values = [float(v) * 1.2 for v in value.split("#")[0].split(",")]
            lines[i] = f"{key} = " + ", ".join(repr(v) for v in values)
            return "\n".join(lines) + "\n"
    raise AssertionError(f"{key} not in {V1}")


def read_csv(path):
    with open(path, newline="") as f:
        return [{k: float(v) for k, v in row.items()} for row in csv.DictReader(f)]


def quat_to_matrix(w, x, y, z):
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ]


def isa_pressure(h):
    return 101325 * (1 - 0.0065 * h / 288.15) ** (9.80665 / (287.05 * 0.0065))


class ReplayTest(unittest.TestCase):
    def assertClose(self, actual, expected, tol, what):
        self.assertLessEqual(
            abs(actual - expected), tol, f"{what}: {actual!r} != {expected!r}"
        )

    def test_pad_state_at_t0(self):
        row = replay("--zero-input", "--duration", 1)[0]
        self.assertEqual(row["t_s"], 0.0)
        expected = {
            "pos_n_m": 0, "pos_e_m": 0, "pos_d_m": 0,
            "vel_n_mps": 0, "vel_e_mps": 0, "vel_d_mps": 0,
            "q_w": H, "q_x": 0, "q_y": H, "q_z": 0,
            "sf_x_mps2": G_MODEL, "sf_y_mps2": 0, "sf_z_mps2": 0,
            "pressure_pa": 101325.0, "temperature_c": 15.0,
        }
        for name, value in expected.items():
            self.assertClose(row[name], value, 1e-9, name)

    def test_matches_simulink_reference(self):
        params = (REF_DIR / "params.txt").read_text().split()
        rows = replay(
            "--input", REF_DIR / "input.csv", "--duration", 8, *params
        )
        self.assert_matches_reference(rows, REF_DIR)

    def test_perturbed_matches_simulink_reference(self):
        ref_dir = REF_DIR / "perturbed"
        params = (ref_dir / "params.txt").read_text().split()
        rows = replay(
            "--rocket", PERTURBED, "--input", ref_dir / "input.csv",
            "--duration", 8, *params,
        )
        self.assert_matches_reference(rows, ref_dir)

    def assert_matches_reference(self, rows, ref_dir):
        ref = read_csv(ref_dir / "output.csv")
        self.assertEqual(len(rows), len(ref))
        pos0 = [ref[0]["pos_x"], ref[0]["pos_y"], ref[0]["pos_z"]]
        tol = 1e-4
        for got, exp in zip(rows, ref):
            t = exp["t_s"]
            self.assertClose(got["t_s"], t, 1e-9, "t_s")

            def check(name, value):
                self.assertClose(got[name], value, tol, f"{name} at {t} s")

            for axis, i in (("x", 0), ("y", 1), ("z", 2)):
                check(f"sf_{axis}_mps2", exp[f"sf_{axis}"])
                check(f"gyro_{axis}_radps", exp[f"w_{axis}"])
            for i in range(4):
                check(f"fin{i}_rad", exp[f"fin{i}"])
            check("roll_rad", exp["phi"])
            check("pitch_rad", exp["theta"])
            check("yaw_rad", exp["psi"])

            dx = exp["pos_x"] - pos0[0]
            check("pos_n_m", exp["pos_z"] - pos0[2])
            check("pos_e_m", exp["pos_y"] - pos0[1])
            check("pos_d_m", -dx)
            check("vel_n_mps", exp["vel_z"])
            check("vel_e_mps", exp["vel_y"])
            check("vel_d_mps", -exp["vel_x"])
            check("pressure_pa", isa_pressure(dx))

            dcm = [[exp[f"dcm_{r}{c}"] for c in (1, 2, 3)] for r in (1, 2, 3)]
            expected_r = [
                [sum(P[i][k] * dcm[j][k] for k in range(3)) for j in range(3)]
                for i in range(3)
            ]
            got_r = quat_to_matrix(got["q_w"], got["q_x"], got["q_y"], got["q_z"])
            for i in range(3):
                for j in range(3):
                    self.assertClose(
                        got_r[i][j], expected_r[i][j], tol, f"R[{i}][{j}] at {t} s"
                    )

    def test_same_seed_is_reproducible(self):
        first = replay("--zero-input", "--duration", 8)
        second = replay("--zero-input", "--duration", 8)
        self.assertEqual(first, second)

    def test_gust_seed_changes_trajectory(self):
        default = replay("--zero-input", "--duration", 8)
        other = replay("--zero-input", "--duration", 8, "gust_seed_1=7")
        self.assertGreater(
            abs(default[-1]["pos_e_m"] - other[-1]["pos_e_m"]), 1e-6
        )

    def test_low_thrust_stays_on_pad(self):
        rows = replay("--zero-input", "--duration", 6, "thrust_scale=0.05")
        self.assertEqual(rows[-1]["t_s"], 6.0)
        for row in rows:
            for name in ("pos_n_m", "pos_e_m", "pos_d_m"):
                self.assertEqual(row[name], 0.0, f"{name} at {row['t_s']} s")
            self.assertClose(row["sf_x_mps2"], G_MODEL, 1e-9, "sf_x_mps2")

    def test_unknown_parameter_is_rejected(self):
        with self.assertRaises(sil.SilError) as ctx:
            replay("--zero-input", "--duration", 1, "no_such=1")
        self.assertIn("no_such", str(ctx.exception))

    def test_out_of_range_parameter_is_rejected(self):
        for arg, name in (("servo_tau_s=0", "servo_tau_s"),
                          ("ignition_time_s=-1", "ignition_time_s")):
            with self.subTest(arg=arg):
                with self.assertRaises(sil.SilError) as ctx:
                    replay("--zero-input", "--duration", 1, arg)
                self.assertIn(name, str(ctx.exception))

    # --- Design files (--rocket) ---------------------------------------------

    def ref_rows(self, *args):
        """The reference scenario: fins move at 1.5-2.5 s, after the 0.5 s
        ignition (params.txt), so aerodynamic values act on the flight."""
        return replay(
            "--input", REF_INPUT, "--duration", 8, *REF_PARAMS, *args
        )

    def assertRowsClose(self, a, b, tol):
        self.assertEqual(len(a), len(b))
        for ra, rb in zip(a, b):
            for name in ra:
                self.assertClose(ra[name], rb[name], tol, f"{name} at {ra['t_s']} s")

    def test_builtin_matches_v1_file(self):
        self.assertRowsClose(self.ref_rows(), self.ref_rows("--rocket", V1), 1e-9)

    def test_each_design_key_changes_the_flight(self):
        base = {}
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            (tmp / "motors").mkdir()
            shutil.copy(V1_MOTOR, tmp / "motors")
            step = tmp / "step.csv"
            step.write_text(
                "t_s,fin0_rad,fin1_rad,fin2_rad,fin3_rad\n"
                "0,0,0,0,0\n1.5,0.7,0.7,0.7,0.7\n"
            )
            for name, count, _, _ in rocket_design.KEYS:
                with self.subTest(key=name):
                    path = tmp / f"{name}.rocket"
                    path.write_text(changed_design(name, count))
                    # The reference input's 2 degree steps never reach the
                    # 600 deg/s servo rate limit; a 0.7 rad step does.
                    inp = step if name == "servo_rate_limit_dps" else REF_INPUT
                    run = ("--input", inp, "--duration", 8, *REF_PARAMS)
                    if inp not in base:
                        base[inp] = replay("--rocket", V1, *run)
                    rows = replay("--rocket", path, *run)
                    self.assertEqual(len(rows), len(base[inp]))
                    diff = max(
                        abs(r[c] - b[c])
                        for r, b in zip(rows, base[inp])
                        for c in r
                    )
                    self.assertGreater(diff, 1e-9, f"{name} has no effect")

    def test_print_design_matches_python_reader(self):
        for path in (V1, PERTURBED):
            with self.subTest(design=path.name):
                proc = replay_proc("--rocket", path, "--print-design")
                self.assertEqual(proc.returncode, 0, proc.stderr)
                self.assertEqual(
                    proc.stdout,
                    rocket_design.format_design(rocket_design.load(path)),
                )

    def test_print_design_requires_rocket(self):
        proc = replay_proc("--print-design")
        self.assertEqual(proc.returncode, 1)

    def test_bad_design_file_exits_2(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "bad.rocket"
            path.write_text("name = bad\nfoo = 1\n")
            proc = replay_proc("--rocket", path, "--zero-input", "--duration", 1)
            self.assertEqual(proc.returncode, 2)
            self.assertIn(f"{path}:2: unknown key foo", proc.stderr)

    def test_missing_design_file_exits_2(self):
        proc = replay_proc(
            "--rocket", "/nonexistent/x.rocket", "--zero-input", "--duration", 1
        )
        self.assertEqual(proc.returncode, 2)
        self.assertIn("/nonexistent/x.rocket", proc.stderr)

    def test_design_override_order(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            (tmp / "motors").mkdir()
            shutil.copy(V1_MOTOR, tmp / "motors")
            path = tmp / "cd.rocket"
            path.write_text(V1.read_text().replace("cd0 = 0.635", "cd0 = 0.7"))
            # The parameter comes first on the command line, the design file
            # is still applied before it.
            self.assertRowsClose(
                self.ref_rows("cd0=0.7", "--rocket", V1),
                self.ref_rows("--rocket", path),
                1e-12,
            )

    def test_override_that_breaks_design_is_rejected(self):
        proc = replay_proc(
            "--rocket", V1, "--zero-input", "--duration", 1, "mass_fuel_kg=2"
        )
        self.assertEqual(proc.returncode, 3)
        self.assertIn("mass_fuel_kg must be less than mass_full_kg", proc.stderr)

    def test_relative_rocket_path_from_other_cwd(self):
        proc = replay_proc(
            "--rocket", "perturbed.rocket", "--zero-input", "--duration", 2,
            cwd=PERTURBED.parent,
        )
        self.assertEqual(proc.returncode, 0, proc.stderr)
        absolute = replay_proc(
            "--rocket", PERTURBED, "--zero-input", "--duration", 2
        )
        self.assertEqual(proc.stdout, absolute.stdout)

    def test_old_servo_rate_name_is_gone(self):
        proc = replay_proc(
            "--zero-input", "--duration", 1, "servo_rate_limit_radps=10"
        )
        self.assertEqual(proc.returncode, 2)
        replay("--zero-input", "--duration", 1, "servo_rate_limit_dps=600")

    def test_design_scalar_override_reaches_the_plant(self):
        rows = self.ref_rows("cp_aft_of_cg_m=0.2")
        self.assertGreater(
            max(abs(a["pitch_rad"] - b["pitch_rad"])
                for a, b in zip(rows, self.ref_rows())),
            1e-9,
        )

    def test_fuel_above_full_mass_is_rejected(self):
        with self.assertRaises(sil.SilError) as ctx:
            replay("--zero-input", "--duration", 1, "mass_fuel_kg=2")
        self.assertIn(
            "mass_fuel_kg must be less than mass_full_kg", str(ctx.exception)
        )


if __name__ == "__main__":
    unittest.main()
