"""Smoke tests: the firmware runs closed-loop against the toy roll plant.

Run with `python3 code/sim/sil.py test` (builds first), or with SIL_EXE_TOY
set to an existing zephyr.exe:
`python3 -m unittest discover -s code/sim/tests -t code/sim`.
"""

import math
import unittest

from tests import common
from tests.common import sil

EXE = None
KICK_MS = 2500  # toy plant default kick_time_s
LOG_PERIOD_MS = 10  # CONFIG_VIIKATE_SIM_LOG_PERIOD_MS
SERVO_LIMIT_DEG = 45.0  # output_limit in code/app/src/guidance/pid.c


def setUpModule():
    global EXE
    EXE = common.exe("toy")


def max_abs(rows, names):
    return max(abs(row[n]) for row in rows for n in names)


CMDS = ["cmd0_deg", "cmd1_deg", "cmd2_deg", "cmd3_deg"]


class SmokeTest(unittest.TestCase):
    def test_runs_to_stop_time(self):
        result = sil.run(EXE, 3)
        self.assertEqual(result.plant, "toy_roll_plant")
        self.assertEqual(result.interface_version, 2)
        self.assertIn("IMU initialized", result.log)
        self.assertIn("Barometer initialized", result.log)
        # The tick exactly at --stop_at is not run; allow one log period.
        self.assertGreaterEqual(result.rows[-1]["t_ms"], 3000 - LOG_PERIOD_MS)
        self.assertLessEqual(result.rows[-1]["t_ms"], 3000)

    def test_toy_reports_launch_frame_euler(self):
        result = sil.run(EXE, 3)
        self.assertIn("pitch_deg", result.columns)
        self.assertEqual(max_abs(result.rows, ["pitch_deg", "yaw_deg"]), 0.0)

    def test_outputs_are_finite(self):
        result = sil.run(EXE, 6)
        for row in result.rows:
            for name, value in row.items():
                self.assertTrue(math.isfinite(value), f"{name} at {row['t_ms']}")

    def test_deterministic(self):
        first = sil.run(EXE, 4)
        second = sil.run(EXE, 4)
        self.assertEqual(first.rows, second.rows)

    def test_non_finite_plant_output_stops_the_run(self):
        # airspeed 0 makes the toy plant's roll damping 0/0
        with self.assertRaises(sil.SilError) as ctx:
            sil.run(EXE, 3, plant_params={"airspeed_mps": 0.0})
        self.assertIn("SIMERR,non_finite_output", str(ctx.exception))

    def test_normal_run_has_no_end_marker(self):
        self.assertIsNone(sil.run(EXE, 3).end_reason)

    def test_unknown_plant_param_is_rejected(self):
        with self.assertRaises(sil.SilError):
            sil.run(EXE, 1, plant_params={"no_such_param": 1.0})

    def test_firmware_commands_servos_after_roll_disturbance(self):
        """The loop is closed through the firmware: no command before the
        kick, a command within 100 ms after it."""
        result = sil.run(EXE, 4)
        before = result.between(0, KICK_MS)
        after = result.between(KICK_MS, KICK_MS + 100)
        self.assertLess(max_abs(before, CMDS), 1e-6)
        self.assertGreater(max_abs(after, CMDS), 0.5)

    @unittest.expectedFailure
    def test_roll_disturbance_is_rejected(self):
        """Known failure: the firmware does not damp a 1 rad/s roll kick.

        Today's mixing in code/app/src/guidance/pid.c (+,+,-,-) cancels the
        roll moment in the plant's fin geometry, and main.c passes radians
        into roll_control_update(), which expects degrees. Remove the
        expectedFailure marker once roll control works; the test then has
        to pass.
        """
        result = sil.run(EXE, 6)
        late = result.between(5000, 6000)
        self.assertLess(max_abs(late, ["roll_rate_dps"]), 5.0)
        self.assertLess(max_abs(late, CMDS), SERVO_LIMIT_DEG)


if __name__ == "__main__":
    unittest.main()
