"""The unmodified firmware flown against the rocket plant: pad wait,
ignition, boost and coast to apogee, once for every design in
simulation/rockets/*.rocket (test designs in simulation/rockets/test/ are
not flown).

The zero-fin reference is the same plant and design stepped by plant_replay
with the fins held at zero, so the comparison measures what the firmware's
fin commands do to the flight.
"""

import math
import tempfile
import unittest
from pathlib import Path

from tests import common, rocket_design
from tests.common import sil

EXE = None
IGNITION_MS = 3000  # ignition_time_s default in plant_parameters.m
H = math.sqrt(0.5)
G_MODEL = 9.819
CMDS = ["cmd0_deg", "cmd1_deg", "cmd2_deg", "cmd3_deg"]
V1 = rocket_design.ROCKETS_DIR / "viikate_v1.rocket"


def setUpModule():
    global EXE
    EXE = common.exe("rocket")


class DesignFlight:
    """Flight-envelope tests for one design; PATH is set per subclass."""

    PATH = None

    @classmethod
    def setUpClass(cls):
        cls.design = rocket_design.load(cls.PATH)
        cls.burnout_ms = common.burnout_ms(cls.PATH, IGNITION_MS)
        cls.flight = sil.run(EXE, 20, rocket=cls.PATH)
        cls.ref = sil.run_replay(
            common.replay(),
            ["--zero-input", "--duration", "20", "--rocket", str(cls.PATH)],
        )

    def test_simrocket_names_design(self):
        self.assertEqual(self.flight.rocket, self.design["name"])

    def test_pad_is_quiet_before_ignition(self):
        pad = [row for row in self.flight.rows if row["t_ms"] < IGNITION_MS]
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
        liftoff = next(row for row in self.flight.rows if -row["pos_d_m"] > 1.0)
        self.assertGreater(liftoff["t_ms"], IGNITION_MS)
        self.assertLessEqual(liftoff["t_ms"], IGNITION_MS + 500)

    def test_outputs_are_finite(self):
        for row in self.flight.rows:
            for name, value in row.items():
                self.assertTrue(math.isfinite(value), f"{name} at {row['t_ms']}")

    def test_deterministic(self):
        self.assertEqual(sil.run(EXE, 20, rocket=self.PATH).rows, self.flight.rows)

    def test_reaches_apogee(self):
        common.apogee(self.flight.rows)

    def test_tilt_at_burnout_matches_zero_fin_reference(self):
        flight = common.tilt_deg(common.row_at(self.flight.rows, self.burnout_ms))
        ref = common.tilt_deg(common.row_at(self.ref, self.burnout_ms))
        self.assertLess(abs(flight - ref), 5.0, f"tilt {flight:.2f} vs {ref:.2f} deg")

    @unittest.expectedFailure
    def test_apogee_matches_zero_fin_reference(self):
        """Known firmware failure: for viikate_v1, apogee 217.6 m vs 229.7 m
        (-5.3%).

        The firmware's roll estimate follows the launch-frame Euler roll that
        appears when the rocket pitches and yaws (true roll rate stays under
        0.1 dps), and its roll loop answers with a fin pattern that pitches
        and yaws this airframe, costing altitude. The result depends on the
        servo sign convention, which is not yet confirmed: with all four
        signs negated the v1 flight passes at +3.7%. Remove the marker once
        the servo signs are settled and the roll estimate ignores pitch/yaw.
        An unexpected success for a design means the marker needs review.
        """
        flight = common.apogee(self.flight.rows)
        ref = common.apogee(self.ref)
        self.assertLess(
            abs(flight - ref) / ref, 0.05, f"apogee {flight:.1f} vs {ref:.1f} m"
        )

    def test_run_ends_at_ground_impact(self):
        result = sil.run(EXE, 60, rocket=self.PATH)
        self.assertEqual(result.end_reason, "ground_impact")
        self.assertLess(result.rows[-1]["t_ms"], 60000)


for _path in rocket_design.flight_designs():
    _name = f"FlightTest_{_path.stem}"
    globals()[_name] = type(_name, (DesignFlight, unittest.TestCase), {"PATH": _path})
del _path, _name


class BridgeTest(unittest.TestCase):
    """Behaviour of the bridge and the built-in design (viikate_v1)."""

    def test_header(self):
        result = sil.run(EXE, 1)
        self.assertEqual(result.plant, "rocket_plant")
        self.assertEqual(result.interface_version, 3)

    def test_builtin_run_reports_v1(self):
        self.assertEqual(sil.run(EXE, 1).rocket, "viikate_v1")

    def test_bad_rocket_file_stops_the_run(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "bad.rocket"
            path.write_text("name = bad\nfoo = 1\n")
            with self.assertRaises(sil.SilError) as ctx:
                sil.run(EXE, 1, rocket=path)
        self.assertIn("SIMERR", str(ctx.exception))
        self.assertIn(f"{path}:2: unknown key foo", str(ctx.exception))

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

    def test_design_key_override(self):
        heavy = sil.run(EXE, 6, rocket=V1, plant_params={"mass_full_kg": 2.0})
        normal = sil.run(EXE, 6, rocket=V1)
        self.assertLess(
            -heavy.rows[-1]["pos_d_m"], -normal.rows[-1]["pos_d_m"] - 1.0
        )


if __name__ == "__main__":
    unittest.main()
