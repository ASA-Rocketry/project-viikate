"""Frame mapping and standard atmosphere used by the rocket plant adapter
(code/sim/plant/rocket/rocket_frames.c), called through ctypes."""

import ctypes
import math
import unittest

from tests.common import sil

LIB = None
H = math.sqrt(0.5)


def setUpModule():
    global LIB
    LIB = ctypes.CDLL(str(sil.build_frames_lib()))
    vec3 = ctypes.c_double * 3
    LIB.rocket_model_to_ned.argtypes = [vec3, vec3]
    LIB.rocket_ned_to_model.argtypes = [vec3, vec3]
    LIB.rocket_dcm_be_to_quat_ned.argtypes = [ctypes.c_double * 9, ctypes.c_double * 4]
    LIB.rocket_isa_pressure_pa.argtypes = [ctypes.c_double]
    LIB.rocket_isa_pressure_pa.restype = ctypes.c_double
    LIB.rocket_isa_temperature_c.argtypes = [ctypes.c_double]
    LIB.rocket_isa_temperature_c.restype = ctypes.c_double


def model_to_ned(v):
    out = (ctypes.c_double * 3)()
    LIB.rocket_model_to_ned((ctypes.c_double * 3)(*v), out)
    return list(out)


def ned_to_model(v):
    out = (ctypes.c_double * 3)()
    LIB.rocket_ned_to_model((ctypes.c_double * 3)(*v), out)
    return list(out)


def quat(dcm_col_major):
    out = (ctypes.c_double * 4)()
    LIB.rocket_dcm_be_to_quat_ned((ctypes.c_double * 9)(*dcm_col_major), out)
    return list(out)


def roll_dcm(phi):
    """Inertial-to-body DCM for a roll phi about body x, column-major."""
    c, s = math.cos(phi), math.sin(phi)
    return [1, 0, 0, 0, c, -s, 0, s, c]


class FramesTest(unittest.TestCase):
    def assertVecAlmostEqual(self, a, b, tol):
        self.assertEqual(len(a), len(b))
        for x, y in zip(a, b):
            self.assertAlmostEqual(x, y, delta=tol, msg=f"{a} != {b}")

    def test_model_to_ned(self):
        self.assertEqual(model_to_ned([1, 2, 3]), [3, 2, -1])

    def test_ned_to_model_is_inverse(self):
        v = [0.3, -4.0, 7.0]
        self.assertEqual(ned_to_model(model_to_ned(v)), v)

    def test_pad_attitude(self):
        identity = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        self.assertVecAlmostEqual(quat(identity), [H, 0, H, 0], 1e-12)

    def test_roll_about_rocket_axis(self):
        phi = 0.3
        c, s = math.cos(phi / 2), math.sin(phi / 2)
        self.assertVecAlmostEqual(
            quat(roll_dcm(phi)), [H * c, H * s, H * c, -H * s], 1e-12
        )

    def test_quaternion_is_normalised_with_positive_w(self):
        q = quat(roll_dcm(3.0))
        self.assertAlmostEqual(math.sqrt(sum(x * x for x in q)), 1.0, delta=1e-12)
        self.assertGreaterEqual(q[0], 0.0)

    def test_pressure_at_pad(self):
        self.assertEqual(LIB.rocket_isa_pressure_pa(0.0), 101325.0)

    def test_pressure_inverts_firmware_formula(self):
        p = LIB.rocket_isa_pressure_pa(378.0)
        # pressure_to_altitude() in code/app/src/main.c
        h = (288.15 / 0.0065) * (1 - (p / 101325) ** (287.05 * 0.0065 / 9.80665))
        self.assertAlmostEqual(h, 378.0, delta=1e-9)

    def test_temperature(self):
        self.assertAlmostEqual(LIB.rocket_isa_temperature_c(1000.0), 8.5, delta=1e-12)


if __name__ == "__main__":
    unittest.main()
