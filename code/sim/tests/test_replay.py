"""The generated rocket plant plus its C adapter, stepped without Zephyr by
code/sim/tools/plant_replay.c, against the reference scenario Simulink
produced in simulation/plant_codegen/reference (generate_plant.m)."""

import csv
import math
import unittest

from tests import common
from tests.common import sil

EXE = None
REF_DIR = sil.REPO_DIR / "simulation" / "plant_codegen" / "reference"
H = math.sqrt(0.5)
G_MODEL = 9.819
# Model inertial frame (x up) to NED.
P = [[0, 0, 1], [0, 1, 0], [-1, 0, 0]]


def setUpModule():
    global EXE
    EXE = common.replay()


def replay(*args):
    return sil.run_replay(EXE, [str(a) for a in args])


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
        ref = read_csv(REF_DIR / "output.csv")
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

    def test_fuel_above_full_mass_is_rejected(self):
        with self.assertRaises(sil.SilError) as ctx:
            replay("--zero-input", "--duration", 1, "mass_fuel_kg=2")
        self.assertIn(
            "mass_fuel_kg must be less than mass_full_kg", str(ctx.exception)
        )


if __name__ == "__main__":
    unittest.main()
