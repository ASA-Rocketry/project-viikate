"""Helpers shared by the SIL tests."""

import math
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import sil  # noqa: E402


def exe(plant):
    """zephyr.exe for a plant: $SIL_EXE_<PLANT> if set, else build it."""
    var = f"SIL_EXE_{plant.upper()}"
    if var in os.environ:
        return Path(os.environ[var])
    return sil.build(plant)


def replay():
    """plant_replay: $SIL_REPLAY if set, else build it."""
    if "SIL_REPLAY" in os.environ:
        return Path(os.environ["SIL_REPLAY"])
    return sil.build_replay()


def tilt_deg(row):
    """Angle between the rocket axis and vertical, from the NED quaternion."""
    c = 2.0 * (row["q_w"] * row["q_y"] - row["q_x"] * row["q_z"])
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def apogee(rows):
    """Highest altitude above the pad in metres. Raises AssertionError if the
    rocket never left the pad or is still climbing in the last row."""
    altitudes = [-row["pos_d_m"] for row in rows]
    top = max(range(len(altitudes)), key=altitudes.__getitem__)
    if altitudes[top] <= 0.0 or top == len(altitudes) - 1:
        raise AssertionError("apogee not reached")
    return altitudes[top]


def row_at(rows, t_ms):
    """The row at t_ms, from a SIL log (t_ms) or a replay (t_s)."""
    for row in rows:
        t = row["t_ms"] if "t_ms" in row else row["t_s"] * 1000.0
        if round(t) == t_ms:
            return row
    raise AssertionError(f"no row at {t_ms} ms")
