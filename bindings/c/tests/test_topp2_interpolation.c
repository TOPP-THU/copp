/**
 * @file test_topp2_interpolation.c
 * @brief Smoke test for the TOPP2 interpolation C ABI.
 *
 * This test builds a uniform station grid on [0, 1], samples a deterministic
 * random nonnegative `a(s)` profile, calls `copp_s_to_t_2nd`, prints the final
 * traversal time, then round-trips through `copp_t_to_s_non_uniform_2nd`.
 */

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "copp/copp.h"

int main(void) {
    enum { NUM_POINTS = 101 };
    double s[NUM_POINTS];
    double a[NUM_POINTS];

    srand(20260514u);
    for (size_t i = 0; i < NUM_POINTS; ++i) {
        s[i] = (double)i / (double)(NUM_POINTS - 1);
        a[i] = ((double)rand() + 1.0) / ((double)RAND_MAX + 1.0);
    }

    struct CoppSliceF64 s_slice = { s, NUM_POINTS };
    struct CoppSliceF64 a_slice = { a, NUM_POINTS };
    double t_final = 0.0;
    struct CoppVecF64 t_s = { NULL, 0, 0 };
    struct CoppVecF64 s_t = { NULL, 0, 0 };

    enum CoppStatus status = copp_s_to_t_2nd(
        s_slice,
        a_slice,
        0.0,
        &t_final,
        &t_s
    );

    if (status != COPP_STATUS_OK) {
        fprintf(stderr, "copp_s_to_t_2nd failed: %s\n", copp_status_message(status));
        return 1;
    }

    assert(t_final > 0.0);
    assert(t_final < DBL_MAX);
    assert(t_s.data != NULL);
    assert(t_s.len == NUM_POINTS);

    printf("TOPP2 final time: %.17g\n", t_final);

    struct CoppSliceF64 t_s_slice = { t_s.data, t_s.len };
    status = copp_t_to_s_non_uniform_2nd(
        s_slice,
        a_slice,
        t_s_slice,
        t_s_slice,
        &s_t
    );

    if (status != COPP_STATUS_OK) {
        fprintf(stderr, "copp_t_to_s_non_uniform_2nd failed: %s\n", copp_status_message(status));
        copp_vec_f64_free(t_s);
        return 1;
    }

    assert(s_t.data != NULL);
    assert(s_t.len == NUM_POINTS);
    double max_abs_error = 0.0;
    for (size_t i = 0; i < NUM_POINTS; ++i) {
        const double abs_error = fabs(s_t.data[i] - s[i]);
        if (abs_error > max_abs_error) {
            max_abs_error = abs_error;
        }
        assert(abs_error < 1e-8);
    }
    printf("TOPP2 round-trip max |s_t - s|: %.17g\n", max_abs_error);

    copp_vec_f64_free(s_t);
    copp_vec_f64_free(t_s);
    return 0;
}
