#include <math.h>

#include "linmath.h"
#include "utest.h"

#define TEST_EPS 1e-18f

static const mat4x4 a = {
    {1.f, 5.f, 9.f, 13.f},
    {2.f, 6.f, 10.f, 14.f},
    {3.f, 7.f, 11.f, 15.f},
    {4.f, 8.f, 12.f, 16.f},
};

static void
expect_mat_eq(int *utest_result, mat4x4 const got, mat4x4 const want) {
    int c, r;
    for (c = 0; c < 4; ++c) {
        for (r = 0; r < 4; ++r) {
            ASSERT_NEAR(want[c][r], got[c][r], TEST_EPS);
        }
    }
}

UTEST(matmul, identity_left) {
    mat4x4 id, r;
    mat4x4_identity(id);
    mat4x4_mul(r, id, a);
    expect_mat_eq(utest_result, r, a);
}

UTEST(matmul, identity_right) {
    mat4x4 id, r;
    mat4x4_identity(id);
    mat4x4_mul(r, a, id);
    expect_mat_eq(utest_result, r, a);
}

UTEST(matmul, diagonal_right) {
    mat4x4 d = {
        {1.f, 0.f, 0.f, 0.f},
        {0.f, 2.f, 0.f, 0.f},
        {0.f, 0.f, 3.f, 0.f},
        {0.f, 0.f, 0.f, 4.f},
    };
    mat4x4 ad = {
        {1.f, 5.f, 9.f, 13.f},
        {4.f, 12.f, 20.f, 28.f},
        {9.f, 21.f, 33.f, 45.f},
        {16.f, 32.f, 48.f, 64.f},
    };
    mat4x4 r;

    mat4x4_mul(r, a, d);
    expect_mat_eq(utest_result, r, ad);
}

UTEST(matmul, diagonal_left) {
    mat4x4 d = {
        {1.f, 0.f, 0.f, 0.f},
        {0.f, 2.f, 0.f, 0.f},
        {0.f, 0.f, 3.f, 0.f},
        {0.f, 0.f, 0.f, 4.f},
    };
    mat4x4 da = {
        {1.f, 10.f, 27.f, 52.f},
        {2.f, 12.f, 30.f, 56.f},
        {3.f, 14.f, 33.f, 60.f},
        {4.f, 16.f, 36.f, 64.f},
    };
    mat4x4 r;

    mat4x4_mul(r, d, a);
    expect_mat_eq(utest_result, r, da);
}

UTEST(matmul, vec4_product) {
    vec4 v = {1.f, 2.f, 3.f, 4.f};
    vec4 want = {30.f, 70.f, 110.f, 150.f};
    vec4 r;
    int i;

    mat4x4_mul_vec4(r, a, v);
    for (i = 0; i < 4; ++i) {
        ASSERT_NEAR(want[i], r[i], TEST_EPS);
    }
}

UTEST_MAIN()
