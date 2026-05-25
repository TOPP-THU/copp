/**
 * @file test_path_evaluation.c
 * @brief Smoke test for waypoint spline path construction and evaluation.
 */

#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "copp/copp.h"

static int expect_ok(enum CoppStatus status, const char *call)
{
    if (status != COPP_STATUS_OK)
    {
        fprintf(stderr, "%s failed: %s\n", call, copp_status_message(status));
        return 1;
    }
    return 0;
}

static double matrix_get(const struct CoppMatrixF64 *matrix, size_t row, size_t col)
{
    return matrix->data[row + col * matrix->rows];
}

static double lissajous_value(size_t axis, double s)
{
    const double pi = 3.14159265358979323846;
    const double omega[3] = {2.0 * pi, 1.3 * pi, 0.9 * pi};
    const double phase[3] = {0.0, 0.3, 0.7};
    const double scale[3] = {0.20, 0.15, 0.10};
    return scale[axis] * sin(omega[axis] * s + phase[axis]);
}

static enum CoppStatus polynomial_evaluate_3rd(
    void *user_data,
    size_t dim,
    size_t n,
    const double *s,
    double *q,
    double *dq,
    double *ddq,
    double *dddq)
{
    (void)user_data;
    if (dim != 2)
    {
        return COPP_STATUS_PATH_DIMENSION_MISMATCH;
    }

    for (size_t j = 0; j < n; ++j)
    {
        const double x = s[j];
        const size_t col = j * dim;

        q[col] = x * x * x;
        dq[col] = 3.0 * x * x;
        ddq[col] = 6.0 * x;
        dddq[col] = 6.0;

        q[col + 1] = x * x + 1.0;
        dq[col + 1] = 2.0 * x;
        ddq[col + 1] = 2.0;
        dddq[col + 1] = 0.0;
    }

    return COPP_STATUS_OK;
}

static enum CoppStatus polynomial_evaluate_2nd(
    void *user_data,
    size_t dim,
    size_t n,
    const double *s,
    double *q,
    double *dq,
    double *ddq)
{
    (void)user_data;
    if (dim != 2)
    {
        return COPP_STATUS_PATH_DIMENSION_MISMATCH;
    }

    for (size_t j = 0; j < n; ++j)
    {
        const double x = s[j];
        const size_t col = j * dim;

        q[col] = x * x;
        dq[col] = 2.0 * x;
        ddq[col] = 2.0;

        q[col + 1] = x + 1.0;
        dq[col + 1] = 1.0;
        ddq[col + 1] = 0.0;
    }

    return COPP_STATUS_OK;
}

static int run_evaluator_path_test(void)
{
    const double s_eval[3] = {-1.0, 0.0, 0.5};
    struct CoppPath *path = NULL;
    struct CoppMatrixF64 q2 = {NULL, 0, 0, 0};
    struct CoppMatrixF64 dq2 = {NULL, 0, 0, 0};
    struct CoppMatrixF64 ddq2 = {NULL, 0, 0, 0};
    struct CoppMatrixF64 q3 = {NULL, 0, 0, 0};
    struct CoppMatrixF64 dq3 = {NULL, 0, 0, 0};
    struct CoppMatrixF64 ddq3 = {NULL, 0, 0, 0};
    struct CoppMatrixF64 dddq3 = {NULL, 0, 0, 0};

    enum CoppStatus status = copp_path_from_evaluator_3rd(
        2,
        -1.0,
        1.0,
        NULL,
        polynomial_evaluate_3rd,
        NULL,
        &path);
    if (expect_ok(status, "copp_path_from_evaluator_3rd"))
    {
        return 1;
    }

    struct CoppSliceF64 s_eval_slice = {s_eval, 3};
    status = copp_path_evaluate_up_to_2nd(path, s_eval_slice, &q2, &dq2, &ddq2);
    if (expect_ok(status, "copp_path_evaluate_up_to_2nd evaluator"))
    {
        copp_path_free(path);
        return 1;
    }

    status = copp_path_evaluate_up_to_3rd(path, s_eval_slice, &q3, &dq3, &ddq3, &dddq3);
    if (expect_ok(status, "copp_path_evaluate_up_to_3rd evaluator"))
    {
        copp_matrix_f64_free(q2);
        copp_matrix_f64_free(dq2);
        copp_matrix_f64_free(ddq2);
        copp_path_free(path);
        return 1;
    }

    assert(q2.rows == 2 && q2.cols == 3);
    assert(dq2.rows == 2 && dq2.cols == 3);
    assert(ddq2.rows == 2 && ddq2.cols == 3);
    assert(q3.rows == 2 && q3.cols == 3);
    assert(dq3.rows == 2 && dq3.cols == 3);
    assert(ddq3.rows == 2 && ddq3.cols == 3);
    assert(dddq3.rows == 2 && dddq3.cols == 3);

    for (size_t j = 0; j < 3; ++j)
    {
        const double x = s_eval[j];
        assert(fabs(matrix_get(&q2, 0, j) - x * x * x) < 1e-12);
        assert(fabs(matrix_get(&dq2, 0, j) - 3.0 * x * x) < 1e-12);
        assert(fabs(matrix_get(&ddq2, 0, j) - 6.0 * x) < 1e-12);
        assert(fabs(matrix_get(&q3, 1, j) - (x * x + 1.0)) < 1e-12);
        assert(fabs(matrix_get(&dq3, 1, j) - 2.0 * x) < 1e-12);
        assert(fabs(matrix_get(&ddq3, 1, j) - 2.0) < 1e-12);
        assert(fabs(matrix_get(&dddq3, 0, j) - 6.0) < 1e-12);
        assert(fabs(matrix_get(&dddq3, 1, j)) < 1e-12);
    }

    copp_matrix_f64_free(q3);
    copp_matrix_f64_free(dq3);
    copp_matrix_f64_free(ddq3);
    copp_matrix_f64_free(dddq3);
    copp_matrix_f64_free(q2);
    copp_matrix_f64_free(dq2);
    copp_matrix_f64_free(ddq2);
    copp_path_free(path);
    return 0;
}

static int run_evaluator_2nd_path_test(void)
{
    const double s_eval[3] = {-1.0, 0.0, 0.5};
    struct CoppPath *path = NULL;
    struct CoppMatrixF64 q2 = {NULL, 0, 0, 0};
    struct CoppMatrixF64 dq2 = {NULL, 0, 0, 0};
    struct CoppMatrixF64 ddq2 = {NULL, 0, 0, 0};
    struct CoppMatrixF64 q3 = {NULL, 0, 0, 0};
    struct CoppMatrixF64 dq3 = {NULL, 0, 0, 0};
    struct CoppMatrixF64 ddq3 = {NULL, 0, 0, 0};
    struct CoppMatrixF64 dddq3 = {NULL, 0, 0, 0};

    enum CoppStatus status = copp_path_from_evaluator_2nd(
        2,
        -1.0,
        1.0,
        polynomial_evaluate_2nd,
        NULL,
        &path);
    if (expect_ok(status, "copp_path_from_evaluator_2nd"))
    {
        return 1;
    }

    struct CoppSliceF64 s_eval_slice = {s_eval, 3};
    status = copp_path_evaluate_up_to_2nd(path, s_eval_slice, &q2, &dq2, &ddq2);
    if (expect_ok(status, "copp_path_evaluate_up_to_2nd 2nd evaluator"))
    {
        copp_path_free(path);
        return 1;
    }

    status = copp_path_evaluate_up_to_3rd(path, s_eval_slice, &q3, &dq3, &ddq3, &dddq3);
    assert(status == COPP_STATUS_PATH_UNSUPPORTED_DERIVATIVE_ORDER);
    assert(q3.data == NULL && dq3.data == NULL && ddq3.data == NULL && dddq3.data == NULL);

    assert(q2.rows == 2 && q2.cols == 3);
    assert(dq2.rows == 2 && dq2.cols == 3);
    assert(ddq2.rows == 2 && ddq2.cols == 3);

    for (size_t j = 0; j < 3; ++j)
    {
        const double x = s_eval[j];
        assert(fabs(matrix_get(&q2, 0, j) - x * x) < 1e-12);
        assert(fabs(matrix_get(&dq2, 0, j) - 2.0 * x) < 1e-12);
        assert(fabs(matrix_get(&ddq2, 0, j) - 2.0) < 1e-12);
        assert(fabs(matrix_get(&q2, 1, j) - (x + 1.0)) < 1e-12);
        assert(fabs(matrix_get(&dq2, 1, j) - 1.0) < 1e-12);
        assert(fabs(matrix_get(&ddq2, 1, j)) < 1e-12);
    }

    copp_matrix_f64_free(q2);
    copp_matrix_f64_free(dq2);
    copp_matrix_f64_free(ddq2);
    copp_path_free(path);
    return 0;
}

static int run_padded_column_major_waypoint_test(void)
{
    enum
    {
        DIM = 2,
        NUM_WAYPOINTS = 4,
        LEADING_DIM = 3
    };
    const double s_min = 0.0;
    const double s_max = 1.0;
    double waypoints[LEADING_DIM * NUM_WAYPOINTS] = {0.0};
    double s_eval[NUM_WAYPOINTS];
    struct CoppPathOptions options;
    struct CoppPath *path = NULL;
    struct CoppMatrixF64 q = {NULL, 0, 0, 0};
    struct CoppMatrixF64 dq = {NULL, 0, 0, 0};
    struct CoppMatrixF64 ddq = {NULL, 0, 0, 0};

    for (size_t j = 0; j < NUM_WAYPOINTS; ++j)
    {
        const double u = (double)j / (double)(NUM_WAYPOINTS - 1);
        s_eval[j] = u;
        waypoints[j * LEADING_DIM] = u;
        waypoints[j * LEADING_DIM + 1] = u * u + 1.0;
        waypoints[j * LEADING_DIM + 2] = -123.0;
    }

    enum CoppStatus status = copp_path_default_options(s_min, s_max, &options);
    if (expect_ok(status, "copp_path_default_options padded"))
    {
        return 1;
    }

    struct CoppMatrixViewF64 waypoints_view = {
        waypoints,
        DIM,
        NUM_WAYPOINTS,
        COPP_MATRIX_LAYOUT_COLUMN_MAJOR,
        LEADING_DIM};
    status = copp_path_from_waypoints(waypoints_view, options, &path);
    if (expect_ok(status, "copp_path_from_waypoints padded column-major"))
    {
        return 1;
    }

    struct CoppSliceF64 s_eval_slice = {s_eval, NUM_WAYPOINTS};
    status = copp_path_evaluate_up_to_2nd(path, s_eval_slice, &q, &dq, &ddq);
    if (expect_ok(status, "copp_path_evaluate_up_to_2nd padded column-major"))
    {
        copp_path_free(path);
        return 1;
    }

    assert(q.rows == DIM && q.cols == NUM_WAYPOINTS);
    for (size_t j = 0; j < NUM_WAYPOINTS; ++j)
    {
        assert(fabs(matrix_get(&q, 0, j) - waypoints[j * LEADING_DIM]) < 1e-10);
        assert(fabs(matrix_get(&q, 1, j) - waypoints[j * LEADING_DIM + 1]) < 1e-10);
    }

    copp_matrix_f64_free(q);
    copp_matrix_f64_free(dq);
    copp_matrix_f64_free(ddq);
    copp_path_free(path);
    return 0;
}

int main(void)
{
    enum
    {
        DIM = 3,
        NUM_WAYPOINTS = 9,
        NUM_DENSE_SAMPLES = 17
    };
    const double s_min = 0.0;
    const double s_max = 1.0;

    double waypoints[DIM * NUM_WAYPOINTS];
    double s_eval[NUM_WAYPOINTS];
    double s_dense[NUM_DENSE_SAMPLES];
    for (size_t j = 0; j < NUM_WAYPOINTS; ++j)
    {
        const double u = (double)j / (double)(NUM_WAYPOINTS - 1);
        const double x = s_min + (s_max - s_min) * u;
        s_eval[j] = x;
        for (size_t axis = 0; axis < DIM; ++axis)
        {
            waypoints[j + axis * NUM_WAYPOINTS] = lissajous_value(axis, x);
        }
    }
    for (size_t j = 0; j < NUM_DENSE_SAMPLES; ++j)
    {
        const double u = (double)j / (double)(NUM_DENSE_SAMPLES - 1);
        s_dense[j] = s_min + (s_max - s_min) * u;
    }

    struct CoppPath *path = NULL;
    struct CoppMatrixF64 q2 = {NULL, 0, 0, 0};
    struct CoppMatrixF64 dq2 = {NULL, 0, 0, 0};
    struct CoppMatrixF64 ddq2 = {NULL, 0, 0, 0};
    struct CoppMatrixF64 q3 = {NULL, 0, 0, 0};
    struct CoppMatrixF64 dq3 = {NULL, 0, 0, 0};
    struct CoppMatrixF64 ddq3 = {NULL, 0, 0, 0};
    struct CoppMatrixF64 dddq3 = {NULL, 0, 0, 0};
    struct CoppMatrixF64 q_dense = {NULL, 0, 0, 0};
    struct CoppMatrixF64 dq_dense = {NULL, 0, 0, 0};
    struct CoppMatrixF64 ddq_dense = {NULL, 0, 0, 0};
    struct CoppPathOptions options;

    enum CoppStatus status = copp_path_default_options(s_min, s_max, &options);
    if (expect_ok(status, "copp_path_default_options"))
    {
        return 1;
    }
    options.order = 5;
    options.out_of_range_mode = COPP_PATH_OUT_OF_RANGE_MODE_ERROR;
    options.parametrization = COPP_PATH_PARAMETRIZATION_UNIFORM;

    if (run_padded_column_major_waypoint_test())
    {
        return 1;
    }

    struct CoppMatrixViewF64 waypoints_view =
        COPP_MATRIX_VIEW_F64_ROW_MAJOR(waypoints, DIM, NUM_WAYPOINTS);
    status = copp_path_from_waypoints(waypoints_view, options, &path);
    if (expect_ok(status, "copp_path_from_waypoints"))
    {
        return 1;
    }

    size_t dim = 0;
    status = copp_path_dim(path, &dim);
    if (expect_ok(status, "copp_path_dim"))
    {
        copp_path_free(path);
        return 1;
    }
    assert(dim == DIM);

    double got_s_min = 0.0;
    double got_s_max = 0.0;
    status = copp_path_s_range(path, &got_s_min, &got_s_max);
    if (expect_ok(status, "copp_path_s_range"))
    {
        copp_path_free(path);
        return 1;
    }
    assert(fabs(got_s_min - s_min) < 1e-12);
    assert(fabs(got_s_max - s_max) < 1e-12);

    struct CoppSliceF64 s_eval_slice = {s_eval, NUM_WAYPOINTS};
    status = copp_path_evaluate_up_to_2nd(path, s_eval_slice, &q2, &dq2, &ddq2);
    if (expect_ok(status, "copp_path_evaluate_up_to_2nd"))
    {
        copp_path_free(path);
        return 1;
    }
    assert(q2.rows == DIM && q2.cols == NUM_WAYPOINTS);
    assert(dq2.rows == DIM && dq2.cols == NUM_WAYPOINTS);
    assert(ddq2.rows == DIM && ddq2.cols == NUM_WAYPOINTS);
    for (size_t j = 0; j < NUM_WAYPOINTS; ++j)
    {
        for (size_t axis = 0; axis < DIM; ++axis)
        {
            const double expected = lissajous_value(axis, s_eval[j]);
            assert(fabs(matrix_get(&q2, axis, j) - expected) < 1e-8);
            assert(isfinite(matrix_get(&dq2, axis, j)));
            assert(isfinite(matrix_get(&ddq2, axis, j)));
        }
    }

    status = copp_path_evaluate_up_to_3rd(path, s_eval_slice, &q3, &dq3, &ddq3, &dddq3);
    if (expect_ok(status, "copp_path_evaluate_up_to_3rd"))
    {
        copp_matrix_f64_free(q2);
        copp_matrix_f64_free(dq2);
        copp_matrix_f64_free(ddq2);
        copp_path_free(path);
        return 1;
    }
    assert(q3.rows == DIM && q3.cols == NUM_WAYPOINTS);
    assert(dq3.rows == DIM && dq3.cols == NUM_WAYPOINTS);
    assert(ddq3.rows == DIM && ddq3.cols == NUM_WAYPOINTS);
    assert(dddq3.rows == DIM && dddq3.cols == NUM_WAYPOINTS);
    for (size_t j = 0; j < NUM_WAYPOINTS; ++j)
    {
        for (size_t axis = 0; axis < DIM; ++axis)
        {
            assert(isfinite(matrix_get(&q3, axis, j)));
            assert(isfinite(matrix_get(&dq3, axis, j)));
            assert(isfinite(matrix_get(&ddq3, axis, j)));
            assert(isfinite(matrix_get(&dddq3, axis, j)));
        }
    }

    struct CoppSliceF64 s_dense_slice = {s_dense, NUM_DENSE_SAMPLES};
    status = copp_path_evaluate_up_to_2nd(path, s_dense_slice, &q_dense, &dq_dense, &ddq_dense);
    if (expect_ok(status, "copp_path_evaluate_up_to_2nd dense"))
    {
        copp_matrix_f64_free(q3);
        copp_matrix_f64_free(dq3);
        copp_matrix_f64_free(ddq3);
        copp_matrix_f64_free(dddq3);
        copp_matrix_f64_free(q2);
        copp_matrix_f64_free(dq2);
        copp_matrix_f64_free(ddq2);
        copp_path_free(path);
        return 1;
    }
    assert(q_dense.rows == DIM && q_dense.cols == NUM_DENSE_SAMPLES);
    assert(dq_dense.rows == DIM && dq_dense.cols == NUM_DENSE_SAMPLES);
    assert(ddq_dense.rows == DIM && ddq_dense.cols == NUM_DENSE_SAMPLES);
    for (size_t j = 0; j < NUM_DENSE_SAMPLES; ++j)
    {
        for (size_t axis = 0; axis < DIM; ++axis)
        {
            assert(isfinite(matrix_get(&q_dense, axis, j)));
            assert(isfinite(matrix_get(&dq_dense, axis, j)));
            assert(isfinite(matrix_get(&ddq_dense, axis, j)));
        }
    }

    if (run_evaluator_path_test())
    {
        copp_matrix_f64_free(q_dense);
        copp_matrix_f64_free(dq_dense);
        copp_matrix_f64_free(ddq_dense);
        copp_matrix_f64_free(q3);
        copp_matrix_f64_free(dq3);
        copp_matrix_f64_free(ddq3);
        copp_matrix_f64_free(dddq3);
        copp_matrix_f64_free(q2);
        copp_matrix_f64_free(dq2);
        copp_matrix_f64_free(ddq2);
        copp_path_free(path);
        return 1;
    }

    if (run_evaluator_2nd_path_test())
    {
        copp_matrix_f64_free(q_dense);
        copp_matrix_f64_free(dq_dense);
        copp_matrix_f64_free(ddq_dense);
        copp_matrix_f64_free(q3);
        copp_matrix_f64_free(dq3);
        copp_matrix_f64_free(ddq3);
        copp_matrix_f64_free(dddq3);
        copp_matrix_f64_free(q2);
        copp_matrix_f64_free(dq2);
        copp_matrix_f64_free(ddq2);
        copp_path_free(path);
        return 1;
    }

    printf("Path C evaluation smoke test passed: dim=%zu, waypoint q.cols=%zu, dense q.cols=%zu\n",
           dim,
           q2.cols,
           q_dense.cols);

    copp_matrix_f64_free(q_dense);
    copp_matrix_f64_free(dq_dense);
    copp_matrix_f64_free(ddq_dense);
    copp_matrix_f64_free(q3);
    copp_matrix_f64_free(dq3);
    copp_matrix_f64_free(ddq3);
    copp_matrix_f64_free(dddq3);
    copp_matrix_f64_free(q2);
    copp_matrix_f64_free(dq2);
    copp_matrix_f64_free(ddq2);
    copp_path_free(path);
    return 0;
}
