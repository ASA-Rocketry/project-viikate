#include "rocket_frames.h"

#include <math.h>

#define ISA_P0_PA 101325.0
#define ISA_T0_K 288.15
#define ISA_LAPSE_KPM 0.0065
#define ISA_G_MPS2 9.80665
#define ISA_R_JPKGK 287.05
#define KELVIN_OFFSET 273.15

/* Rotation from the model inertial frame to NED: v_ned = P * v_model. */
static const double model_to_ned[3][3] = {
    {0.0, 0.0, 1.0},
    {0.0, 1.0, 0.0},
    {-1.0, 0.0, 0.0},
};

void rocket_model_to_ned(const double v_model[3], double v_ned[3]) {
    v_ned[0] = v_model[2];
    v_ned[1] = v_model[1];
    v_ned[2] = -v_model[0];
}

void rocket_ned_to_model(const double v_ned[3], double v_model[3]) {
    v_model[0] = -v_ned[2];
    v_model[1] = v_ned[1];
    v_model[2] = v_ned[0];
}

void rocket_dcm_be_to_quat_ned(const double dcm_be[9], double quat_wxyz[4]) {
    /* Body to NED: R = P * transpose(dcm_be). dcm_be[c * 3 + r] is row r,
     * column c, so transpose(dcm_be)[k][j] = dcm_be[k * 3 + j]. */
    double r[3][3];
    double w, x, y, z, s, norm;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            r[i][j] = 0.0;
            for (int k = 0; k < 3; k++) {
                r[i][j] += model_to_ned[i][k] * dcm_be[k * 3 + j];
            }
        }
    }

    const double trace = r[0][0] + r[1][1] + r[2][2];
    if (trace > 0.0) {
        s = 2.0 * sqrt(trace + 1.0);
        w = 0.25 * s;
        x = (r[2][1] - r[1][2]) / s;
        y = (r[0][2] - r[2][0]) / s;
        z = (r[1][0] - r[0][1]) / s;
    } else if (r[0][0] > r[1][1] && r[0][0] > r[2][2]) {
        s = 2.0 * sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]);
        w = (r[2][1] - r[1][2]) / s;
        x = 0.25 * s;
        y = (r[0][1] + r[1][0]) / s;
        z = (r[0][2] + r[2][0]) / s;
    } else if (r[1][1] > r[2][2]) {
        s = 2.0 * sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]);
        w = (r[0][2] - r[2][0]) / s;
        x = (r[0][1] + r[1][0]) / s;
        y = 0.25 * s;
        z = (r[1][2] + r[2][1]) / s;
    } else {
        s = 2.0 * sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]);
        w = (r[1][0] - r[0][1]) / s;
        x = (r[0][2] + r[2][0]) / s;
        y = (r[1][2] + r[2][1]) / s;
        z = 0.25 * s;
    }

    norm = sqrt(w * w + x * x + y * y + z * z);
    if (w < 0.0) {
        norm = -norm;
    }
    quat_wxyz[0] = w / norm;
    quat_wxyz[1] = x / norm;
    quat_wxyz[2] = y / norm;
    quat_wxyz[3] = z / norm;
}

double rocket_isa_pressure_pa(double altitude_m) {
    const double ratio = 1.0 - ISA_LAPSE_KPM * altitude_m / ISA_T0_K;

    return ISA_P0_PA * pow(ratio, ISA_G_MPS2 / (ISA_R_JPKGK * ISA_LAPSE_KPM));
}

double rocket_isa_temperature_c(double altitude_m) {
    return ISA_T0_K - ISA_LAPSE_KPM * altitude_m - KELVIN_OFFSET;
}
