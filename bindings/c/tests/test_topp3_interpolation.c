/**
 * @file test_topp3_interpolation.c
 * @brief Smoke tests for the TOPP3/COPP3 interpolation C ABI.
 *
 * The C API intentionally takes explicit `s/a/b/num_stationary_*` profile
 * parts instead of requiring callers to construct an internal profile object.
 */

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "copp/copp.h"

static int expect_ok(enum CoppStatus status, const char *name)
{
    if (status != COPP_STATUS_OK) {
        fprintf(stderr, "%s failed: %s\n", name, copp_status_message(status));
        return 0;
    }
    return 1;
}

static void assert_close(double actual, double expected, double tol)
{
    assert(fabs(actual - expected) <= tol);
}

static void test_zero_stationary(void)
{
    enum { NUM_POINTS = 11 };
    double s[NUM_POINTS];
    double a[NUM_POINTS];
    double b[NUM_POINTS];

    for (size_t i = 0; i < NUM_POINTS; ++i) {
        s[i] = (double)i / (double)(NUM_POINTS - 1);
        a[i] = 1.0;
        b[i] = 0.0;
    }

    struct CoppSliceF64 s_slice = {s, NUM_POINTS};
    struct CoppSliceF64 a_slice = {a, NUM_POINTS};
    struct CoppSliceF64 b_slice = {b, NUM_POINTS};
    struct CoppVecF64 t_s = {NULL, 0, 0};
    struct CoppVecF64 s_t = {NULL, 0, 0};
    double t_final = 0.0;

    enum CoppStatus status = copp_s_to_t_3rd(
        s_slice,
        a_slice,
        b_slice,
        0,
        0,
        0.0,
        &t_final,
        &t_s
    );
    assert(expect_ok(status, "copp_s_to_t_3rd zero-stationary"));
    assert_close(t_final, 1.0, 1e-12);
    assert(t_s.len == NUM_POINTS);
    for (size_t i = 0; i < NUM_POINTS; ++i) {
        assert_close(t_s.data[i], s[i], 1e-12);
    }

    struct CoppSliceF64 t_s_slice = {t_s.data, t_s.len};
    status = copp_t_to_s_uniform_3rd(
        s_slice,
        a_slice,
        b_slice,
        0,
        0,
        t_s_slice,
        0.0,
        0.1,
        true,
        &s_t
    );
    assert(expect_ok(status, "copp_t_to_s_uniform_3rd zero-stationary"));
    assert(s_t.len == NUM_POINTS);
    for (size_t i = 0; i < NUM_POINTS; ++i) {
        assert_close(s_t.data[i], s[i], 1e-10);
    }

    printf("TOPP3 zero-stationary final time: %.17g\n", t_final);

    copp_vec_f64_free(s_t);
    copp_vec_f64_free(t_s);
}

static void test_stationary_boundary(void)
{
    enum { NUM_POINTS = 6 };
    double s[NUM_POINTS] = {0.0, 0.2, 0.4, 0.6, 0.8, 1.0};
    double a[NUM_POINTS] = {0.0, 1.0, 1.0, 1.0, 1.0, 0.0};
    double b[NUM_POINTS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    struct CoppSliceF64 s_slice = {s, NUM_POINTS};
    struct CoppSliceF64 a_slice = {a, NUM_POINTS};
    struct CoppSliceF64 b_slice = {b, NUM_POINTS};
    struct CoppVecF64 t_s = {NULL, 0, 0};
    struct CoppVecF64 s_t = {NULL, 0, 0};
    double t_final = 0.0;

    enum CoppStatus status = copp_s_to_t_3rd(
        s_slice,
        a_slice,
        b_slice,
        1,
        1,
        0.0,
        &t_final,
        &t_s
    );
    assert(expect_ok(status, "copp_s_to_t_3rd stationary-boundary"));
    assert_close(t_final, 1.8, 1e-12);
    assert(t_s.len == NUM_POINTS);

    struct CoppSliceF64 t_s_slice = {t_s.data, t_s.len};
    status = copp_t_to_s_non_uniform_3rd(
        s_slice,
        a_slice,
        b_slice,
        1,
        1,
        t_s_slice,
        t_s_slice,
        &s_t
    );
    assert(expect_ok(status, "copp_t_to_s_non_uniform_3rd stationary-boundary"));
    assert(s_t.len == NUM_POINTS);
    for (size_t i = 0; i < NUM_POINTS; ++i) {
        assert_close(s_t.data[i], s[i], 1e-9);
    }

    printf("TOPP3 stationary-boundary final time: %.17g\n", t_final);

    copp_vec_f64_free(s_t);
    copp_vec_f64_free(t_s);
}

static void test_force_positive_a(void)
{
    enum { NUM_POINTS = 5 };
    double s[NUM_POINTS] = {0.0, 0.25, 0.5, 0.75, 1.0};
    double a[NUM_POINTS] = {0.01, 0.01, 0.01, 0.01, 0.01};
    double b[NUM_POINTS] = {0.0, -1.0, 1.0, 0.0, 0.0};

    struct CoppSliceF64 s_slice = {s, NUM_POINTS};
    struct CoppSliceMutF64 a_slice = {a, NUM_POINTS};
    struct CoppSliceMutF64 b_slice = {b, NUM_POINTS};
    bool succeed = false;

    enum CoppStatus status = copp_force_positive_a_3rd(
        s_slice,
        a_slice,
        b_slice,
        0,
        0,
        1e-6,
        &succeed
    );
    assert(expect_ok(status, "copp_force_positive_a_3rd"));
    assert(succeed);
    for (size_t i = 0; i < NUM_POINTS; ++i) {
        assert(isfinite(a[i]));
        assert(isfinite(b[i]));
        assert(a[i] >= 0.0);
    }

    printf("TOPP3 force_positive_a succeed: %d\n", succeed ? 1 : 0);
}

int main(void)
{
    test_zero_stationary();
    test_stationary_boundary();
    test_force_positive_a();
    return 0;
}
