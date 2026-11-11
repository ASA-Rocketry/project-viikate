"""The unmodified firmware flown against the rocket plant: pad wait,
ignition, boost and coast to apogee.

The zero-fin reference is the same plant stepped by plant_replay with the
fins held at zero, so the comparison measures what the firmware's fin
commands do to the flight.
"""

import math
import unittest

from tests import common
from tests.common import sil

EXE = None
FLIGHT = None
REF = None
IGNITION_MS = 3000  # ignition_time_s default in plant_parameters.m
BURNOUT_MS = 4000  # ignition + 1 s burn time
H = math.sqrt(0.5)
G_MODEL = 9.819
CMDS = ["cmd0_deg", "cmd1_deg", "cmd2_deg", "cmd3_deg"]


def setUpModule():
    global EXE, FLIGHT, REF
    EXE = common.exe("rocket")
    FLIGHT = sil.run(EXE, 20)
    REF = sil.run_replay(
        common.replay(), ["--zero-input", "--duration", "20"]
    )


class FlightTest(unittest.TestCase):
    def test_header(self):
        self.assertEqual(FLIGHT.plant, "rocket_plant")
        self.assertEqual(FLIGHT.interface_version, 2)

    def test_pad_is_quiet_before_ignition(self):
        pad = [row for row in FLIGHT.rows if row["t_ms"] < IGNITION_MS]
        self.assertGreater(len(pad), 250)
        for row in pad:
            t = row["t_ms"]
            for name in ("pos_n_m", "pos_e_m", "pos_d_m"):
                self.assertLess(abs(row[name]), 1e-6, f"{name} at {t} ms")
            for name, value in (("q_w", H), ("q_x", 0), ("q_y", H), ("q_z", 0)):
                self.assertLess(abs(row[name] - value), 1e-6, f"{name} at {t} ms")
            self.assertLess(abs(row["sf_x_mps2"] - G_MODEL), 0.01, f"sf_x at {t} ms")
            self.assertLess(abs(row["sf_y_mps2"]), 0.01, f"sf_y at {t} ms")
            self.assertLess(abs(row["sf_z_mps2"]), 0.01, f"sf_z at {t} ms")
            for name in CMDS:
                self.assertLess(abs(row[name]), 0.5, f"{name} at {t} ms")

    def test_liftoff_follows_ignition(self):
        liftoff = next(row for row in FLIGHT.rows if -row["pos_d_m"] > 1.0)
        self.assertGreater(liftoff["t_ms"], IGNITION_MS)
        self.assertLessEqual(liftoff["t_ms"], IGNITION_MS + 500)

    def test_outputs_are_finite(self):
        for row in FLIGHT.rows:
            for name, value in row.items():
                self.assertTrue(math.isfinite(value), f"{name} at {row['t_ms']}")

    def test_deterministic(self):
        self.assertEqual(sil.run(EXE, 20).rows, FLIGHT.rows)

    def test_reaches_apogee(self):
        common.apogee(FLIGHT.rows)

    def test_tilt_at_burnout_matches_zero_fin_reference(self):
        flight = common.tilt_deg(common.row_at(FLIGHT.rows, BURNOUT_MS))
        ref = common.tilt_deg(common.row_at(REF, BURNOUT_MS))
        self.assertLess(abs(flight - ref), 5.0, f"tilt {flight:.2f} vs {ref:.2f} deg")

    def test_apogee_matches_zero_fin_reference(self):
        flight = common.apogee(FLIGHT.rows)
        ref = common.apogee(REF)
        self.assertLess(
            abs(flight - ref) / ref, 0.05, f"apogee {flight:.1f} vs {ref:.1f} m"
        )

    def test_run_ends_at_ground_impact(self):
        result = sil.run(EXE, 60)
        self.assertEqual(result.end_reason, "ground_impact")
        self.assertLess(result.rows[-1]["t_ms"], 60000)

    def test_low_thrust_hop_ends_at_ground_impact(self):
        # Thrust peaks just above weight: the rocket hops well under 1 m and
        # falls back. Without a ground below the pad it would keep falling.
        result = sil.run(EXE, 20, plant_params={"thrust_scale": 0.15})
        self.assertEqual(result.end_reason, "ground_impact")
        self.assertLess(max(-row["pos_d_m"] for row in result.rows), 1.0)
        self.assertLessEqual(result.rows[-1]["pos_d_m"], 1.0)

    def test_short_run_stays_on_pad(self):
        result = sil.run(EXE, 2)
        self.assertIsNone(result.end_reason)
        for row in result.rows:
            self.assertEqual(row["pos_d_m"], 0.0)
        with self.assertRaises(AssertionError) as ctx:
            common.apogee(result.rows)
        self.assertIn("apogee not reached", str(ctx.exception))

    def test_out_of_range_parameter_stops_the_run(self):
        with self.assertRaises(sil.SilError) as ctx:
            sil.run(EXE, 1, plant_params={"servo_tau_s": 0})
        self.assertIn("servo_tau_s", str(ctx.exception))


if __name__ == "__main__":
    unittest.main()
