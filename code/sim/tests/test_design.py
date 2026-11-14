"""Rocket design and .eng motor parser (code/sim/plant/rocket/rocket_design.c),
called through ctypes, and its agreement with the Python reader the tests
use (tests/rocket_design.py)."""

import ctypes
import tempfile
import unittest
from pathlib import Path

from tests import rocket_design
from tests.common import sil

LIB = None
V1 = rocket_design.ROCKETS_DIR / "viikate_v1.rocket"
PERTURBED = rocket_design.ROCKETS_DIR / "test" / "perturbed.rocket"
V1_T0_THRUST = 65.133111111111106
N_POINTS = 128


class Design(ctypes.Structure):
    """Mirror of struct rocket_design."""

    _fields_ = [
        ("name", ctypes.c_char * 64),
        ("motor", ctypes.c_char * 256),
        ("burn_time_given", ctypes.c_int),
        ("burn_time_s", ctypes.c_double),
        ("mass_full_kg", ctypes.c_double),
        ("mass_fuel_kg", ctypes.c_double),
        ("inertia_full_kgm2", ctypes.c_double * 3),
        ("inertia_empty_kgm2", ctypes.c_double * 3),
        ("inertia_full_products_kgm2", ctypes.c_double * 3),
        ("inertia_empty_products_kgm2", ctypes.c_double * 3),
        *[
            (name, ctypes.c_double)
            for name in (
                "length_m", "frontal_area_m2", "cd0", "cn_alpha",
                "cp_aft_of_cg_m", "cmq", "cnr", "clp", "roll_ref_length_m",
                "canard_area_m2", "canard_cl_alpha", "canard_fwd_of_cg_m",
                "canard_lift_radius_m", "servo_tau_s", "servo_rate_limit_dps",
            )
        ],
        ("thrust_count", ctypes.c_int),
        ("thrust_t_s", ctypes.c_double * N_POINTS),
        ("thrust_n", ctypes.c_double * N_POINTS),
    ]


def setUpModule():
    global LIB
    LIB = ctypes.CDLL(str(sil.build_design_lib()))
    LIB.rocket_design_sizeof.restype = ctypes.c_size_t
    assert LIB.rocket_design_sizeof() == ctypes.sizeof(Design)
    parse_args = [
        ctypes.c_char_p, ctypes.c_size_t, ctypes.c_char_p,
        ctypes.POINTER(Design), ctypes.c_char_p, ctypes.c_size_t,
    ]
    LIB.rocket_design_parse.argtypes = parse_args
    LIB.rocket_motor_parse_eng.argtypes = parse_args
    LIB.rocket_design_check.argtypes = [
        ctypes.POINTER(Design), ctypes.c_char_p, ctypes.c_size_t,
    ]
    LIB.rocket_design_format.argtypes = [
        ctypes.POINTER(Design), ctypes.c_char_p, ctypes.c_size_t,
    ]
    LIB.rocket_design_motor_path.argtypes = [
        ctypes.c_char_p, ctypes.POINTER(Design), ctypes.c_char_p, ctypes.c_size_t,
    ]


def load_c(path):
    """Parse a design and its motor with the C library, as the plant does.
    Returns (rc, error text, canonical text, struct)."""
    path = Path(path)
    d = Design()
    err = ctypes.create_string_buffer(512)
    text = path.read_bytes()
    rc = LIB.rocket_design_parse(text, len(text), str(path).encode(), d, err, 512)
    if rc != 0:
        return rc, err.value.decode(), None, d
    motor = ctypes.create_string_buffer(512)
    assert LIB.rocket_design_motor_path(str(path).encode(), d, motor, 512) == 0
    mtext = Path(motor.value.decode()).read_bytes()
    rc = LIB.rocket_motor_parse_eng(mtext, len(mtext), motor.value, d, err, 512)
    if rc != 0:
        return rc, err.value.decode(), None, d
    rc = LIB.rocket_design_check(d, err, 512)
    if rc != 0:
        return rc, err.value.decode(), None, d
    out = ctypes.create_string_buffer(65536)
    assert LIB.rocket_design_format(d, out, 65536) == 0
    return 0, "", out.value.decode(), d


class DesignFileTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self._tmp.name)
        (self.dir / "motors").mkdir()
        (self.dir / "motors" / "viikate_v1.eng").write_bytes(
            (rocket_design.ROCKETS_DIR / "motors" / "viikate_v1.eng").read_bytes()
        )

    def tearDown(self):
        self._tmp.cleanup()

    def write(self, text, name="d.rocket"):
        path = self.dir / name
        path.write_bytes(text.encode() if isinstance(text, str) else text)
        return path

    def v1_text(self):
        return V1.read_text()

    def with_line(self, key, line):
        """v1 with the line for key replaced (or appended if key is absent)."""
        lines = self.v1_text().splitlines()
        hits = [i for i, l in enumerate(lines) if l.split("=")[0].strip() == key]
        if hits:
            lines[hits[0]] = line
        else:
            lines.append(line)
        return "\n".join(lines) + "\n"

    def error(self, text):
        rc, err, _, _ = load_c(self.write(text))
        self.assertEqual(rc, -1, "expected the design to be rejected")
        return err

    def motor_error(self, eng):
        (self.dir / "motors" / "m.eng").write_text(eng)
        rc, err, _, _ = load_c(
            self.write(self.with_line("motor", "motor = motors/m.eng"))
        )
        self.assertEqual(rc, -1, "expected the motor to be rejected")
        return err

    def eng(self, points, header="m 0 0 0 0 0 test"):
        return header + "\n" + "".join(f"{t} {n}\n" for t, n in points)

    # --- Valid files ---------------------------------------------------------

    def test_v1_round_trips(self):
        rc, err, text, d = load_c(V1)
        self.assertEqual(rc, 0, err)
        self.assertEqual(text, rocket_design.format_design(rocket_design.load(V1)))
        self.assertEqual(d.burn_time_s, 1.0)
        self.assertEqual(d.name, b"viikate_v1")

    def test_python_reader_matches_c_for_every_design(self):
        for path in (V1, PERTURBED):
            with self.subTest(design=path.name):
                rc, err, text, _ = load_c(path)
                self.assertEqual(rc, 0, err)
                self.assertEqual(
                    text, rocket_design.format_design(rocket_design.load(path))
                )

    def test_comments_and_whitespace(self):
        text = self.v1_text().replace("cd0 = 0.635", "  cd0\t=  0.635   # drag")
        text = "\n# header comment\n\n" + text
        _, err, got, _ = load_c(self.write(text))
        _, _, want, _ = load_c(V1)
        self.assertEqual(got, want, err)

    def test_crlf_files_parse(self):
        (self.dir / "motors" / "viikate_v1.eng").write_bytes(
            (rocket_design.ROCKETS_DIR / "motors" / "viikate_v1.eng")
            .read_bytes()
            .replace(b"\n", b"\r\n")
        )
        _, err, got, _ = load_c(self.write(self.v1_text().replace("\n", "\r\n")))
        _, _, want, _ = load_c(V1)
        self.assertEqual(got, want, err)

    def test_parser_builds_with_gnu_dialect(self):
        # GCC's default dialect and _GNU_SOURCE pull in <sys/types.h>, whose
        # POSIX names (key_t, ...) the parser must not reuse.
        out = sil.REPLAY_BUILD_DIR / "rocket_design_gnu.o"
        sil._host_cc(
            [sil.ROCKET_DIR / "rocket_design.c"],
            out,
            ["-std=gnu11", "-D_GNU_SOURCE", "-c"],
        )
        self.assertTrue(out.exists())

    # --- Design errors -------------------------------------------------------

    def test_unknown_key(self):
        lines = self.v1_text().splitlines()
        lines.insert(2, "foo = 1")
        path = self.write("\n".join(lines) + "\n")
        _, err, _, _ = load_c(path)
        self.assertEqual(err, f"{path}:3: unknown key foo")

    def test_duplicate_key(self):
        err = self.error(self.v1_text() + "cd0 = 0.7\n")
        self.assertRegex(err, r":\d+: duplicate key cd0$")

    def test_missing_key(self):
        path = self.write(self.with_line("cd0", ""))
        _, err, _, _ = load_c(path)
        self.assertEqual(err, f"{path}: missing key cd0")

    def test_wrong_list_length(self):
        err = self.error(
            self.with_line("inertia_full_kgm2", "inertia_full_kgm2 = 0.0027, 0.177")
        )
        self.assertRegex(err, r":\d+: expected 3 values for inertia_full_kgm2$")

    def test_non_finite_and_suffixed_numbers_rejected(self):
        for value in ("nan", "inf", "1.2kg", "", "0x"):
            with self.subTest(value=value):
                err = self.error(self.with_line("cd0", f"cd0 = {value}"))
                self.assertRegex(err, r":\d+: bad number for cd0$")

    def test_range_rules(self):
        for name, count, rule, _ in rocket_design.KEYS:
            if rule == "any":
                continue
            bad = 0 if rule == "positive" else -1e-9
            with self.subTest(key=name):
                value = ", ".join([str(bad)] * count)
                err = self.error(self.with_line(name, f"{name} = {value}"))
                self.assertRegex(err, rf":\d+: {name} out of range$")

    def test_fuel_not_below_full_mass(self):
        err = self.error(self.with_line("mass_fuel_kg", "mass_fuel_kg = 1.352"))
        self.assertEqual(err, "mass_fuel_kg must be less than mass_full_kg")

    def test_inertia_not_positive_definite(self):
        err = self.error(
            self.v1_text() + "inertia_full_products_kgm2 = 0.1, 0, 0\n"
        )
        self.assertEqual(err, "inertia_full_kgm2 is not positive-definite")

    def test_motor_path_rules(self):
        for motor in ("/abs.eng", "a/../b.eng", "..", ""):
            with self.subTest(motor=motor):
                err = self.error(self.with_line("motor", f"motor = {motor}"))
                self.assertRegex(err, r":\d+: motor path must be relative without \.\.$")

    def test_bad_name(self):
        for name in ("V1", "v-1", ""):
            with self.subTest(name=name):
                err = self.error(self.with_line("name", f"name = {name}"))
                self.assertRegex(err, r":\d+: bad name$")

    def test_file_too_large(self):
        path = self.write(self.v1_text() + "#" * 70000 + "\n")
        _, err, _, _ = load_c(path)
        self.assertEqual(err, f"{path}: file too large")

    def test_line_without_equals(self):
        err = self.error(self.v1_text() + "cd0 0.6\n")
        self.assertRegex(err, r":\d+: expected key = value$")

    # --- Motor files -----------------------------------------------------------

    def test_eng_inserts_zero_point(self):
        rc, err, _, d = load_c(PERTURBED)
        self.assertEqual(rc, 0, err)
        self.assertEqual(d.thrust_count, 21)
        self.assertEqual((d.thrust_t_s[0], d.thrust_n[0]), (0.0, 0.0))
        self.assertAlmostEqual(d.thrust_t_s[1], 0.06, delta=1e-15)

    def test_eng_keeps_explicit_zero_point(self):
        _, err, _, d = load_c(V1)
        self.assertEqual(d.thrust_count, 16, err)
        self.assertEqual(d.thrust_t_s[0], 0.0)
        self.assertEqual(d.thrust_n[0], V1_T0_THRUST)

    def test_eng_padding(self):
        _, err, _, d = load_c(V1)
        last = d.thrust_t_s[d.thrust_count - 1]
        self.assertEqual(last, 1.001, err)
        for k in range(d.thrust_count, N_POINTS):
            self.assertEqual(d.thrust_t_s[k], last + (k - d.thrust_count + 1))
            self.assertEqual(d.thrust_n[k], 0.0)

    def test_eng_too_many_points(self):
        points = [(0.01 * (i + 1), 10) for i in range(128)]
        err = self.motor_error(self.eng(points))
        self.assertRegex(err, r"m\.eng:129: too many thrust points \(max 127\)$")
        (self.dir / "motors" / "m.eng").write_text(self.eng(points[:127]))
        rc, err, _, d = load_c(
            self.write(self.with_line("motor", "motor = motors/m.eng"))
        )
        self.assertEqual(rc, 0, err)
        self.assertEqual(d.thrust_count, 128)

    def test_eng_times_must_increase(self):
        err = self.motor_error(self.eng([(0.1, 5), (0.1, 6)]))
        self.assertRegex(err, r":3: thrust times must increase$")

    def test_eng_negative_thrust(self):
        err = self.motor_error(self.eng([(0.1, 5), (0.2, -1)]))
        self.assertRegex(err, r":3: thrust must be >= 0$")

    def test_eng_one_point(self):
        err = self.motor_error(self.eng([(0.1, 5)]))
        self.assertRegex(err, r"m\.eng: need at least 2 thrust points$")

    def test_eng_missing_header(self):
        err = self.motor_error("; only a comment\n0.1 5\n0.2 0\n")
        self.assertRegex(err, r"m\.eng:2: missing motor header$")
        err = self.motor_error("; nothing\n")
        self.assertRegex(err, r"m\.eng: missing motor header$")

    def test_eng_bad_thrust_point(self):
        err = self.motor_error(self.eng([(0.1, 5)]) + "0.2 x\n")
        self.assertRegex(err, r":3: bad thrust point$")

    def test_multi_motor_eng_rejected(self):
        one = self.eng([(0.1, 5), (0.2, 0)])
        err = self.motor_error(one + self.eng([(0.1, 7), (0.3, 0)], "n 0 0 0 0 0 x"))
        self.assertRegex(err, r":4: file holds more than one motor$")

    def test_burn_time_defaults_to_last_point(self):
        _, err, _, d = load_c(PERTURBED)
        self.assertEqual(d.burn_time_s, 1.2, err)
        self.assertEqual(d.burn_time_given, 0)
        self.assertEqual(rocket_design.load(PERTURBED)["burn_time_s"], 1.2)


if __name__ == "__main__":
    unittest.main()
