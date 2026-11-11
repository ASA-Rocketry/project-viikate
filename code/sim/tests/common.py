"""Helpers shared by the SIL tests."""

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
