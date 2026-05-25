/**
 * @file test_robot_raw_constraints.c
 * @brief Smoke test for low-level raw constraint C ABI functions.
 */

#include <assert.h>
#include <float.h>
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

int main(void)
{
    enum
    {
        DIM = 1,
        NUM_POINTS = 21,
        NUM_ACC_ROWS = 2,
        NUM_JERK_ROWS = 1
    };

    double s[NUM_POINTS];
    double amax[NUM_POINTS];
    double acc_a[NUM_ACC_ROWS * NUM_POINTS];
    double acc_b[NUM_ACC_ROWS * NUM_POINTS];
    double acc_max[NUM_ACC_ROWS * NUM_POINTS];
    double jerk_a[NUM_JERK_ROWS * NUM_POINTS];
    double jerk_b[NUM_JERK_ROWS * NUM_POINTS];
    double jerk_c[NUM_JERK_ROWS * NUM_POINTS];
    double jerk_d[NUM_JERK_ROWS * NUM_POINTS];
    double jerk_max[NUM_JERK_ROWS * NUM_POINTS];

    for (size_t j = 0; j < NUM_POINTS; ++j)
    {
        s[j] = (double)j / (double)(NUM_POINTS - 1);
        amax[j] = 4.0;

        acc_a[0 + j * NUM_ACC_ROWS] = 0.0;
        acc_a[1 + j * NUM_ACC_ROWS] = 0.0;
        acc_b[0 + j * NUM_ACC_ROWS] = 1.0;
        acc_b[1 + j * NUM_ACC_ROWS] = -1.0;
        acc_max[0 + j * NUM_ACC_ROWS] = 2.0;
        acc_max[1 + j * NUM_ACC_ROWS] = 2.0;

        jerk_a[0 + j * NUM_JERK_ROWS] = 0.0;
        jerk_b[0 + j * NUM_JERK_ROWS] = 0.0;
        jerk_c[0 + j * NUM_JERK_ROWS] = 0.0;
        jerk_d[0 + j * NUM_JERK_ROWS] = 0.0;
        jerk_max[0 + j * NUM_JERK_ROWS] = 100.0;
    }

    struct CoppRobot *robot = NULL;
    struct CoppVecF64 a = {NULL, 0, 0};
    struct CoppVecF64 t_s = {NULL, 0, 0};
    struct CoppVecF64 s_export = {NULL, 0, 0};
    struct CoppVecF64 amax_export = {NULL, 0, 0};
    struct CoppMatrixF64 acc_a_export = {NULL, 0, 0, 0};
    struct CoppMatrixF64 acc_b_export = {NULL, 0, 0, 0};
    struct CoppMatrixF64 acc_max_export = {NULL, 0, 0, 0};
    struct CoppMatrixF64 jerk_a_export = {NULL, 0, 0, 0};
    struct CoppMatrixF64 jerk_b_export = {NULL, 0, 0, 0};
    struct CoppMatrixF64 jerk_c_export = {NULL, 0, 0, 0};
    struct CoppMatrixF64 jerk_d_export = {NULL, 0, 0, 0};
    struct CoppMatrixF64 jerk_max_export = {NULL, 0, 0, 0};
    struct Topp2RaOptions options;
    size_t dim = 0;
    size_t len = 0;
    size_t capacity = 0;
    size_t idx_s_start = 1;
    size_t idx_s_end = 0;
    size_t amax_rows = 0;
    size_t acc_rows = 0;
    size_t jerk_rows = 0;
    bool is_empty = true;
    double s0 = NAN;
    double amax0 = NAN;

    enum CoppStatus status = copp_robot_create(DIM, NUM_POINTS, &robot);
    if (expect_ok(status, "copp_robot_create"))
    {
        return 1;
    }

    struct CoppSliceF64 s_slice = {s, NUM_POINTS};
    status = copp_robot_append_s(robot, s_slice);
    if (expect_ok(status, "copp_robot_append_s"))
    {
        copp_robot_free(robot);
        return 1;
    }
    status = copp_robot_dim(robot, &dim);
    if (expect_ok(status, "copp_robot_dim"))
    {
        copp_robot_free(robot);
        return 1;
    }
    status = copp_robot_len(robot, &len);
    if (expect_ok(status, "copp_robot_len"))
    {
        copp_robot_free(robot);
        return 1;
    }
    status = copp_robot_capacity(robot, &capacity);
    if (expect_ok(status, "copp_robot_capacity"))
    {
        copp_robot_free(robot);
        return 1;
    }
    status = copp_robot_is_empty(robot, &is_empty);
    if (expect_ok(status, "copp_robot_is_empty"))
    {
        copp_robot_free(robot);
        return 1;
    }
    status = copp_robot_idx_s_start(robot, &idx_s_start);
    if (expect_ok(status, "copp_robot_idx_s_start"))
    {
        copp_robot_free(robot);
        return 1;
    }
    status = copp_robot_idx_s_end(robot, &idx_s_end);
    if (expect_ok(status, "copp_robot_idx_s_end"))
    {
        copp_robot_free(robot);
        return 1;
    }
    status = copp_robot_get_s(robot, 0, &s0);
    if (expect_ok(status, "copp_robot_get_s"))
    {
        copp_robot_free(robot);
        return 1;
    }
    status = copp_robot_s_vec(robot, 0, NUM_POINTS, &s_export);
    if (expect_ok(status, "copp_robot_s_vec"))
    {
        copp_robot_free(robot);
        return 1;
    }
    if (dim != DIM || len != NUM_POINTS || capacity < NUM_POINTS || is_empty ||
        idx_s_start != 0 || idx_s_end != NUM_POINTS || s0 != 0.0 ||
        s_export.len != NUM_POINTS)
    {
        fprintf(stderr, "unexpected robot metadata after append_s\n");
        copp_vec_f64_free(s_export);
        copp_robot_free(robot);
        return 1;
    }
    copp_vec_f64_free(s_export);

    struct CoppMatrixViewF64 amax_view =
        COPP_MATRIX_VIEW_F64_COLUMN_MAJOR(amax, 1, NUM_POINTS);
    status = copp_add_raw_constraint_1st(robot, 0, amax_view);
    if (expect_ok(status, "copp_add_raw_constraint_1st"))
    {
        copp_robot_free(robot);
        return 1;
    }
    status = copp_robot_get_amax(robot, 0, &amax0);
    if (expect_ok(status, "copp_robot_get_amax"))
    {
        copp_robot_free(robot);
        return 1;
    }
    status = copp_robot_amax_vec(robot, 0, NUM_POINTS, &amax_export);
    if (expect_ok(status, "copp_robot_amax_vec"))
    {
        copp_robot_free(robot);
        return 1;
    }
    if (amax0 != 4.0 || amax_export.len != NUM_POINTS)
    {
        fprintf(stderr, "unexpected amax export\n");
        copp_vec_f64_free(amax_export);
        copp_robot_free(robot);
        return 1;
    }
    copp_vec_f64_free(amax_export);

    struct CoppMatrixViewF64 acc_a_view =
        COPP_MATRIX_VIEW_F64_COLUMN_MAJOR(acc_a, NUM_ACC_ROWS, NUM_POINTS);
    struct CoppMatrixViewF64 acc_b_view =
        COPP_MATRIX_VIEW_F64_COLUMN_MAJOR(acc_b, NUM_ACC_ROWS, NUM_POINTS);
    struct CoppMatrixViewF64 acc_max_view =
        COPP_MATRIX_VIEW_F64_COLUMN_MAJOR(acc_max, NUM_ACC_ROWS, NUM_POINTS);
    status = copp_add_raw_constraint_2nd(robot, 0, acc_a_view, acc_b_view, acc_max_view);
    if (expect_ok(status, "copp_add_raw_constraint_2nd"))
    {
        copp_robot_free(robot);
        return 1;
    }
    status = copp_robot_constraint_rows(robot, &amax_rows, &acc_rows, &jerk_rows);
    if (expect_ok(status, "copp_robot_constraint_rows"))
    {
        copp_robot_free(robot);
        return 1;
    }
    if (amax_rows != 1 || acc_rows < NUM_ACC_ROWS)
    {
        fprintf(stderr, "unexpected constraint row capacities\n");
        copp_robot_free(robot);
        return 1;
    }
    status = copp_robot_acc_constraints_at(
        robot, 0, &acc_a_export, &acc_b_export, &acc_max_export);
    if (expect_ok(status, "copp_robot_acc_constraints_at"))
    {
        copp_robot_free(robot);
        return 1;
    }
    if (acc_a_export.rows != NUM_ACC_ROWS || acc_a_export.cols != 1 ||
        acc_b_export.rows != NUM_ACC_ROWS || acc_max_export.rows != NUM_ACC_ROWS)
    {
        fprintf(stderr, "unexpected exported acceleration constraint shape\n");
        copp_matrix_f64_free(acc_a_export);
        copp_matrix_f64_free(acc_b_export);
        copp_matrix_f64_free(acc_max_export);
        copp_robot_free(robot);
        return 1;
    }
    copp_matrix_f64_free(acc_a_export);
    copp_matrix_f64_free(acc_b_export);
    copp_matrix_f64_free(acc_max_export);

    struct CoppMatrixViewF64 jerk_a_view =
        COPP_MATRIX_VIEW_F64_COLUMN_MAJOR(jerk_a, NUM_JERK_ROWS, NUM_POINTS);
    struct CoppMatrixViewF64 jerk_b_view =
        COPP_MATRIX_VIEW_F64_COLUMN_MAJOR(jerk_b, NUM_JERK_ROWS, NUM_POINTS);
    struct CoppMatrixViewF64 jerk_c_view =
        COPP_MATRIX_VIEW_F64_COLUMN_MAJOR(jerk_c, NUM_JERK_ROWS, NUM_POINTS);
    struct CoppMatrixViewF64 jerk_d_view =
        COPP_MATRIX_VIEW_F64_COLUMN_MAJOR(jerk_d, NUM_JERK_ROWS, NUM_POINTS);
    struct CoppMatrixViewF64 jerk_max_view =
        COPP_MATRIX_VIEW_F64_COLUMN_MAJOR(jerk_max, NUM_JERK_ROWS, NUM_POINTS);
    status = copp_add_raw_constraint_3rd(
        robot, 0, jerk_a_view, jerk_b_view, jerk_c_view, jerk_d_view, jerk_max_view);
    if (expect_ok(status, "copp_add_raw_constraint_3rd"))
    {
        copp_robot_free(robot);
        return 1;
    }
    status = copp_robot_jerk_constraints_at(
        robot,
        0,
        &jerk_a_export,
        &jerk_b_export,
        &jerk_c_export,
        &jerk_d_export,
        &jerk_max_export);
    if (expect_ok(status, "copp_robot_jerk_constraints_at"))
    {
        copp_robot_free(robot);
        return 1;
    }
    if (jerk_a_export.rows != NUM_JERK_ROWS || jerk_a_export.cols != 1 ||
        jerk_b_export.rows != NUM_JERK_ROWS ||
        jerk_c_export.rows != NUM_JERK_ROWS ||
        jerk_d_export.rows != NUM_JERK_ROWS ||
        jerk_max_export.rows != NUM_JERK_ROWS)
    {
        fprintf(stderr, "unexpected exported jerk constraint shape\n");
        copp_matrix_f64_free(jerk_a_export);
        copp_matrix_f64_free(jerk_b_export);
        copp_matrix_f64_free(jerk_c_export);
        copp_matrix_f64_free(jerk_d_export);
        copp_matrix_f64_free(jerk_max_export);
        copp_robot_free(robot);
        return 1;
    }
    copp_matrix_f64_free(jerk_a_export);
    copp_matrix_f64_free(jerk_b_export);
    copp_matrix_f64_free(jerk_c_export);
    copp_matrix_f64_free(jerk_d_export);
    copp_matrix_f64_free(jerk_max_export);

    struct Topp2Problem problem = {robot, 0, NUM_POINTS - 1, 0.0, 0.0};
    status = topp2_ra_default_options(&options);
    if (expect_ok(status, "topp2_ra_default_options"))
    {
        copp_robot_free(robot);
        return 1;
    }
    status = topp2_ra(problem, options, &a);
    if (expect_ok(status, "topp2_ra"))
    {
        copp_robot_free(robot);
        return 1;
    }
    if (a.data == NULL || a.len != NUM_POINTS)
    {
        fprintf(stderr, "unexpected raw-constraint TOPP2-RA profile shape\n");
        copp_vec_f64_free(a);
        copp_robot_free(robot);
        return 1;
    }

    double t_final = 0.0;
    struct CoppSliceF64 a_slice = {a.data, a.len};
    status = copp_s_to_t_2nd(s_slice, a_slice, 0.0, &t_final, &t_s);
    if (expect_ok(status, "copp_s_to_t_2nd"))
    {
        copp_vec_f64_free(a);
        copp_robot_free(robot);
        return 1;
    }
    if (!isfinite(t_final) || t_final <= 0.0 || t_final >= DBL_MAX || t_s.len != NUM_POINTS)
    {
        fprintf(stderr, "invalid raw-constraint TOPP2 time profile\n");
        copp_vec_f64_free(t_s);
        copp_vec_f64_free(a);
        copp_robot_free(robot);
        return 1;
    }

    printf("Raw constraint C smoke test passed: t_final=%.17g, a.len=%zu\n",
           t_final,
           a.len);

    status = copp_robot_pop_back_n(robot, 1);
    if (expect_ok(status, "copp_robot_pop_back_n"))
    {
        copp_vec_f64_free(t_s);
        copp_vec_f64_free(a);
        copp_robot_free(robot);
        return 1;
    }
    status = copp_robot_len(robot, &len);
    if (expect_ok(status, "copp_robot_len"))
    {
        copp_vec_f64_free(t_s);
        copp_vec_f64_free(a);
        copp_robot_free(robot);
        return 1;
    }
    assert(len == NUM_POINTS - 1);
    status = copp_robot_clear_constraints(robot, true);
    if (expect_ok(status, "copp_robot_clear_constraints"))
    {
        copp_vec_f64_free(t_s);
        copp_vec_f64_free(a);
        copp_robot_free(robot);
        return 1;
    }
    status = copp_robot_is_empty(robot, &is_empty);
    if (expect_ok(status, "copp_robot_is_empty"))
    {
        copp_vec_f64_free(t_s);
        copp_vec_f64_free(a);
        copp_robot_free(robot);
        return 1;
    }
    assert(is_empty);

    copp_vec_f64_free(t_s);
    copp_vec_f64_free(a);
    copp_robot_free(robot);
    return 0;
}
