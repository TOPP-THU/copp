/**
 * @file test_path_fitting.c
 * @brief Smoke test for interpolating and tolerance-bounded waypoint paths.
 */

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "copp/copp.h"

enum
{
    DIM = 2,
    NUM_WAYPOINTS = 5
};

/* Column-major two-axis polyline with a corner at the middle waypoint. */
static const double waypoints[DIM * NUM_WAYPOINTS] = {
    0.0, 0.0,
    0.25, 0.4,
    0.5, 0.5,
    0.75, 0.4,
    1.0, 0.0,
};

static int expect_ok(enum CoppStatus status, const char *call)
{
    if (status != COPP_STATUS_OK)
    {
        fprintf(stderr, "%s failed: %s\n", call, copp_status_message(status));
        return 1;
    }
    return 0;
}

static int expect_status(enum CoppStatus status, enum CoppStatus expected, const char *call)
{
    if (status != expected)
    {
        fprintf(stderr,
                "%s returned '%s', expected '%s'\n",
                call,
                copp_status_message(status),
                copp_status_message(expected));
        return 1;
    }
    return 0;
}

static double matrix_get(const struct CoppMatrixF64 *matrix, size_t row, size_t col)
{
    return matrix->data[row + col * matrix->rows];
}

static struct CoppMatrixViewF64 waypoints_view(void)
{
    return COPP_MATRIX_VIEW_F64_COLUMN_MAJOR(waypoints, DIM, NUM_WAYPOINTS);
}

static int evaluate_q(const struct CoppPath *path,
                      const double *s,
                      size_t n,
                      struct CoppMatrixF64 *out_q)
{
    struct CoppMatrixF64 dq = {NULL, 0, 0, 0};
    struct CoppMatrixF64 ddq = {NULL, 0, 0, 0};
    enum CoppStatus status =
        copp_path_evaluate_up_to_2nd(path, (struct CoppSliceF64){s, n}, out_q, &dq, &ddq);
    copp_matrix_f64_free(dq);
    copp_matrix_f64_free(ddq);
    return expect_ok(status, "copp_path_evaluate_up_to_2nd");
}

static int run_interpolating_alias_test(void)
{
    const double s_eval[NUM_WAYPOINTS] = {0.0, 0.25, 0.5, 0.75, 1.0};
    struct CoppPathOptions options;
    struct CoppPath *interpolating = NULL;
    struct CoppPath *alias = NULL;
    struct CoppMatrixF64 q_interpolating = {NULL, 0, 0, 0};
    struct CoppMatrixF64 q_alias = {NULL, 0, 0, 0};
    struct CoppSmoothingReport report = {{NULL, 0, 0}, 0, 0, 0, 0, 0, {NULL, 0, 0}};
    bool has_report = true;
    int rc = 1;

    if (expect_ok(copp_path_default_options(0.0, 1.0, &options), "copp_path_default_options") ||
        expect_ok(copp_path_from_waypoints_interpolating(waypoints_view(), options, &interpolating),
                  "copp_path_from_waypoints_interpolating") ||
        expect_ok(copp_path_from_waypoints(waypoints_view(), options, &alias),
                  "copp_path_from_waypoints") ||
        evaluate_q(interpolating, s_eval, NUM_WAYPOINTS, &q_interpolating) ||
        evaluate_q(alias, s_eval, NUM_WAYPOINTS, &q_alias))
    {
        goto cleanup;
    }

    assert(q_interpolating.rows == DIM && q_interpolating.cols == NUM_WAYPOINTS);
    assert(q_alias.rows == DIM && q_alias.cols == NUM_WAYPOINTS);
    for (size_t j = 0; j < NUM_WAYPOINTS; ++j)
    {
        for (size_t axis = 0; axis < DIM; ++axis)
        {
            const double value = matrix_get(&q_interpolating, axis, j);
            assert(value == matrix_get(&q_alias, axis, j));
            assert(fabs(value - waypoints[axis + j * DIM]) < 1e-10);
        }
    }

    if (expect_ok(copp_path_smoothing_report(interpolating, &has_report, &report),
                  "copp_path_smoothing_report interpolating"))
    {
        goto cleanup;
    }
    assert(!has_report);
    assert(report.axes.data == NULL && report.axes.len == 0);
    assert(report.max_errors.data == NULL && report.max_errors.len == 0);
    assert(report.segments == 0 && report.checked_intervals == 0);
    rc = 0;

cleanup:
    copp_smoothing_report_free(report);
    copp_matrix_f64_free(q_alias);
    copp_matrix_f64_free(q_interpolating);
    copp_path_free(alias);
    copp_path_free(interpolating);
    return rc;
}

static int run_fitting_test(void)
{
    const size_t axes[DIM] = {1, 0};
    const double tolerances[DIM] = {1e-3, 2e-3};
    const double parameters[NUM_WAYPOINTS] = {2.0, 2.5, 3.0, 3.5, 4.0};
    struct CoppSmoothingOptions options;
    struct CoppPath *path = NULL;
    struct CoppMatrixF64 q = {NULL, 0, 0, 0};
    struct CoppSmoothingReport report = {{NULL, 0, 0}, 0, 0, 0, 0, 0, {NULL, 0, 0}};
    bool has_report = false;
    double s_min = 0.0;
    double s_max = 0.0;
    int rc = 1;

    if (expect_ok(copp_smoothing_default_options(&options), "copp_smoothing_default_options"))
    {
        return 1;
    }
    assert(options.tolerance == 1e-3);
    assert(options.tolerance_per_axis.data == NULL && options.tolerance_per_axis.len == 0);
    assert(options.axes.data == NULL && options.axes.len == 0);
    assert(options.parameters.data == NULL && options.parameters.len == 0);
    assert(options.max_refinements == 20);
    assert(options.max_segments == 20000);
    assert(options.out_of_range_mode == COPP_PATH_OUT_OF_RANGE_MODE_ERROR);

    /* Tolerance entries follow the order of `axes`, not the row index. */
    options.axes = (struct CoppSliceUsize){axes, DIM};
    options.tolerance_per_axis = (struct CoppSliceF64){tolerances, DIM};
    options.parameters = (struct CoppSliceF64){parameters, NUM_WAYPOINTS};
    if (expect_ok(copp_path_from_waypoints_fitting(waypoints_view(), options, &path),
                  "copp_path_from_waypoints_fitting"))
    {
        goto cleanup;
    }
    assert(path != NULL);

    if (expect_ok(copp_path_s_range(path, &s_min, &s_max), "copp_path_s_range"))
    {
        goto cleanup;
    }
    assert(s_min == parameters[0]);
    assert(s_max == parameters[NUM_WAYPOINTS - 1]);

    if (expect_ok(copp_path_smoothing_report(path, &has_report, &report),
                  "copp_path_smoothing_report"))
    {
        goto cleanup;
    }
    assert(has_report);
    assert(report.axes.len == DIM && report.max_errors.len == DIM);
    for (size_t i = 0; i < DIM; ++i)
    {
        assert(report.axes.data[i] == axes[i]);
        assert(isfinite(report.max_errors.data[i]));
        assert(report.max_errors.data[i] <= tolerances[i]);
    }
    assert(report.segments > 0);
    assert(report.interpolated_segments == 0);
    assert(report.checked_intervals > 0);

    /* The first and last waypoints are retained. */
    const double s_ends[2] = {s_min, s_max};
    if (evaluate_q(path, s_ends, 2, &q))
    {
        goto cleanup;
    }
    for (size_t axis = 0; axis < DIM; ++axis)
    {
        assert(fabs(matrix_get(&q, axis, 0) - waypoints[axis]) < 1e-9);
        assert(fabs(matrix_get(&q, axis, 1) - waypoints[axis + (NUM_WAYPOINTS - 1) * DIM]) <
               1e-9);
    }

    printf("fitted path: segments=%zu, refinements=%zu, max_errors=[%.3e, %.3e]\n",
           report.segments,
           report.refinements,
           report.max_errors.data[0],
           report.max_errors.data[1]);
    rc = 0;

cleanup:
    copp_matrix_f64_free(q);
    copp_smoothing_report_free(report);
    copp_path_free(path);
    return rc;
}

static int run_fitting_failure_test(void)
{
    const size_t out_of_range_axis[1] = {DIM};
    struct CoppSmoothingOptions options;
    struct CoppPath *path = NULL;
    struct CoppSmoothingReport report = {{NULL, 0, 0}, 0, 0, 0, 0, 0, {NULL, 0, 0}};

    assert(COPP_STATUS_PATH_SMOOTHING == 309);
    assert(strcmp(copp_status_message(COPP_STATUS_PATH_SMOOTHING),
                  "path error: smoothing failed") == 0);

    if (expect_ok(copp_smoothing_default_options(&options), "copp_smoothing_default_options"))
    {
        return 1;
    }
    options.axes = (struct CoppSliceUsize){out_of_range_axis, 1};
    if (expect_status(copp_path_from_waypoints_fitting(waypoints_view(), options, &path),
                      COPP_STATUS_PATH_SMOOTHING,
                      "copp_path_from_waypoints_fitting invalid axes"))
    {
        copp_path_free(path);
        return 1;
    }
    assert(path == NULL);
    assert(copp_last_error_code() == COPP_STATUS_PATH_SMOOTHING);
    assert(copp_last_error_message_len() > 0);

    if (expect_ok(copp_smoothing_default_options(&options), "copp_smoothing_default_options"))
    {
        return 1;
    }
    options.max_segments = 0;
    if (expect_status(copp_path_from_waypoints_fitting(waypoints_view(), options, &path),
                      COPP_STATUS_PATH_SMOOTHING,
                      "copp_path_from_waypoints_fitting max_segments"))
    {
        copp_path_free(path);
        return 1;
    }
    assert(path == NULL);

    if (expect_status(copp_smoothing_default_options(NULL),
                      COPP_STATUS_NULL_POINTER,
                      "copp_smoothing_default_options NULL") ||
        expect_status(copp_path_smoothing_report(NULL, NULL, &report),
                      COPP_STATUS_NULL_POINTER,
                      "copp_path_smoothing_report NULL"))
    {
        return 1;
    }
    return 0;
}

int main(void)
{
    if (run_interpolating_alias_test() || run_fitting_test() || run_fitting_failure_test())
    {
        return 1;
    }

    printf("Path fitting C smoke test passed\n");
    return 0;
}
