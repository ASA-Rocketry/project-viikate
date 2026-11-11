"""Python reader for rocket design files, used by the tests.

Follows the same rules as code/sim/plant/rocket/rocket_design.c and gives
the same canonical text (format_design), which test_design.py and
test_replay.py compare against the C parser.
"""

import math
import re
from pathlib import Path

ROCKETS_DIR = Path(__file__).resolve().parents[3] / "simulation" / "rockets"
FILE_MAX = 65536
THRUST_POINTS = 128

# Numeric keys in key-table order: (name, count, rule, optional).
KEYS = [
    ("burn_time_s", 1, "positive", True),
    ("mass_full_kg", 1, "positive", False),
    ("mass_fuel_kg", 1, "non_negative", False),
    ("inertia_full_kgm2", 3, "any", False),
    ("inertia_empty_kgm2", 3, "any", False),
    ("inertia_full_products_kgm2", 3, "any", True),
    ("inertia_empty_products_kgm2", 3, "any", True),
    ("length_m", 1, "positive", False),
    ("frontal_area_m2", 1, "positive", False),
    ("cd0", 1, "non_negative", False),
    ("cn_alpha", 1, "non_negative", False),
    ("cp_aft_of_cg_m", 1, "any", False),
    ("cmq", 1, "any", False),
    ("cnr", 1, "any", False),
    ("clp", 1, "any", False),
    ("roll_ref_length_m", 1, "positive", False),
    ("canard_area_m2", 1, "positive", False),
    ("canard_cl_alpha", 1, "non_negative", False),
    ("canard_fwd_of_cg_m", 1, "any", False),
    ("canard_lift_radius_m", 1, "positive", False),
    ("servo_tau_s", 1, "positive", False),
    ("servo_rate_limit_dps", 1, "positive", False),
]
SCALAR_KEYS = [name for name, count, _, _ in KEYS if count == 1]

_NUMBER = re.compile(r"[+-]?(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?")
_NAME = re.compile(r"[a-z0-9_]{1,63}")


def _number(text):
    text = text.strip()
    if not _NUMBER.fullmatch(text):
        raise ValueError
    value = float(text)
    if not math.isfinite(value):
        raise ValueError
    return value


def _ok(value, rule):
    return {"any": True, "non_negative": value >= 0, "positive": value > 0}[rule]


def _lines(text):
    for number, line in enumerate(text.split("\n"), 1):
        yield number, line[:-1] if line.endswith("\r") else line


def parse(text, path):
    """Parse .rocket text; returns a dict of the keys (no thrust table)."""
    if len(text.encode()) > FILE_MAX:
        raise ValueError(f"{path}: file too large")
    table = {name: (count, rule) for name, count, rule, _ in KEYS}
    d = {}
    for number, line in _lines(text):
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        if "=" not in line:
            raise ValueError(f"{path}:{number}: expected key = value")
        key, value = (s.strip() for s in line.split("=", 1))
        if key not in table and key not in ("name", "motor"):
            raise ValueError(f"{path}:{number}: unknown key {key}")
        if key in d:
            raise ValueError(f"{path}:{number}: duplicate key {key}")
        if key == "name":
            if not _NAME.fullmatch(value):
                raise ValueError(f"{path}:{number}: bad name")
            d[key] = value
            continue
        if key == "motor":
            if not value or value.startswith("/") or ".." in value.split("/"):
                raise ValueError(
                    f"{path}:{number}: motor path must be relative without .."
                )
            d[key] = value
            continue
        count, rule = table[key]
        items = value.split(",")
        if len(items) != count:
            raise ValueError(f"{path}:{number}: expected {count} values for {key}")
        try:
            values = [_number(item) for item in items]
        except ValueError:
            raise ValueError(f"{path}:{number}: bad number for {key}") from None
        if not all(_ok(v, rule) for v in values):
            raise ValueError(f"{path}:{number}: {key} out of range")
        d[key] = values if count > 1 else values[0]
    for key in ("name", "motor"):
        if key not in d:
            raise ValueError(f"{path}: missing key {key}")
    for name, count, _, optional in KEYS:
        if name not in d:
            if not optional:
                raise ValueError(f"{path}: missing key {name}")
            if count > 1:
                d[name] = [0.0] * count
    return d


def parse_eng(text, path):
    """Thrust points of a RASP .eng file, after inserting (0, 0)."""
    if len(text.encode()) > FILE_MAX:
        raise ValueError(f"{path}: file too large")
    header = False
    points = []
    for number, line in _lines(text):
        fields = line.split(";", 1)[0].split()
        if not fields:
            continue
        if not header:
            if len(fields) != 7:
                raise ValueError(f"{path}:{number}: missing motor header")
            header = True
            continue
        try:
            if len(fields) != 2:
                raise ValueError
            t, n = _number(fields[0]), _number(fields[1])
            if t < 0:
                raise ValueError
        except ValueError:
            if len(fields) == 7 and points:
                raise ValueError(
                    f"{path}:{number}: file holds more than one motor"
                ) from None
            raise ValueError(f"{path}:{number}: bad thrust point") from None
        if len(points) == THRUST_POINTS - 1:
            raise ValueError(
                f"{path}:{number}: too many thrust points (max {THRUST_POINTS - 1})"
            )
        if points and t <= points[-1][0]:
            raise ValueError(f"{path}:{number}: thrust times must increase")
        if n < 0:
            raise ValueError(f"{path}:{number}: thrust must be >= 0")
        points.append((t, n))
    if not header:
        raise ValueError(f"{path}: missing motor header")
    if len(points) < 2:
        raise ValueError(f"{path}: need at least 2 thrust points")
    if points[0][0] > 0:
        points.insert(0, (0.0, 0.0))
    return points


def load(path):
    """Read a .rocket file and its motor. Returns the design dict with
    'thrust_points' and burn_time_s defaulted."""
    path = Path(path)
    d = parse(path.read_text(), str(path))
    motor = path.parent / d["motor"]
    d["thrust_points"] = parse_eng(motor.read_text(), str(motor))
    if "burn_time_s" not in d:
        d["burn_time_s"] = d["thrust_points"][-1][0]
    if not d["mass_fuel_kg"] < d["mass_full_kg"]:
        raise ValueError(f"{path}: mass_fuel_kg must be less than mass_full_kg")
    return d


def format_design(d):
    """Canonical text, identical to rocket_design_format() in C."""
    lines = [f"name = {d['name']}", f"motor = {d['motor']}"]
    for name, count, _, _ in KEYS:
        values = d[name] if count > 1 else [d[name]]
        lines.append(f"{name} = " + ", ".join("%.17g" % v for v in values))
    for t, n in d["thrust_points"]:
        lines.append("thrust_point = %.17g, %.17g" % (t, n))
    return "\n".join(lines) + "\n"


def flight_designs():
    """The designs the flight tests fly: simulation/rockets/*.rocket."""
    return sorted(ROCKETS_DIR.glob("*.rocket"))
