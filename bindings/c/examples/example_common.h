#ifndef COPP_C_EXAMPLE_COMMON_H
#define COPP_C_EXAMPLE_COMMON_H

/*
 * Shared helpers for the C tutorial examples.
 *
 * The repository examples all start from the same deterministic 3-axis
 * Lissajous path:
 *
 *   q0(s) = sin(2*pi*s + 0.0)
 *   q1(s) = sin(3*pi*s + 0.3)
 *   q2(s) = sin(5*pi*s + 0.7)
 *
 * The C API can model the same path without discretization error by creating
 * a callback-backed `CoppPath`. The callbacks below provide q, dq/ds,
 * d2q/ds2, and d3q/ds3 directly. Keeping this common setup in one header lets
 * each example focus on the solver-specific problem descriptor and options.
 *
 * For ordinary C users this callback path is only one construction option.
 * If you do not have analytic derivatives or automatic differentiation, pass
 * waypoint columns to `copp_path_from_waypoints` instead. COPP builds a spline
 * path, and the same `copp_robot_sample_path_2nd` / `copp_robot_sample_path_3rd`
 * calls used by these examples can sample derivatives from that spline.
 */

#include <float.h>
#include <math.h>
#include <stdio.h>

#include "copp/copp.h"

enum
{
    EXAMPLE_DIM = 3,
    EXAMPLE_NUM_POINTS = 1001
};

static int example_expect_ok(enum CoppStatus status, const char *call)
{
    if (status != COPP_STATUS_OK)
    {
        fprintf(stderr, "%s failed: %s\n", call, copp_status_message(status));
        return 1;
    }
    return 0;
}

static void example_fill_stations(double *s, size_t n)
{
    for (size_t j = 0; j < n; ++j)
    {
        s[j] = (double)j / (double)(n - 1);
    }
}

static void example_eval_lissajous_sample(double s,
                                          double *q,
                                          double *dq,
                                          double *ddq,
                                          double *dddq)
{
    const double pi = 3.14159265358979323846;
    const double freq[EXAMPLE_DIM] = {2.0 * pi, 3.0 * pi, 5.0 * pi};
    const double phase[EXAMPLE_DIM] = {0.0, 0.3, 0.7};

    for (size_t i = 0; i < EXAMPLE_DIM; ++i)
    {
        const double w = freq[i];
        const double x = s * w + phase[i];
        const double sin_x = sin(x);
        const double cos_x = cos(x);
        const double w_sq = w * w;

        q[i] = sin_x;
        dq[i] = cos_x * w;
        ddq[i] = -sin_x * w_sq;
        if (dddq != NULL)
        {
            dddq[i] = (-cos_x * w_sq) * w;
        }
    }
}

static enum CoppStatus example_evaluate_path_2nd(void *user_data,
                                                 size_t dim,
                                                 size_t n,
                                                 const double *s,
                                                 double *q,
                                                 double *dq,
                                                 double *ddq)
{
    (void)user_data;
    if (dim != EXAMPLE_DIM)
    {
        return COPP_STATUS_INVALID_ARGUMENT;
    }
    if (n > 0 && (s == NULL || q == NULL || dq == NULL || ddq == NULL))
    {
        return COPP_STATUS_NULL_POINTER;
    }

    for (size_t j = 0; j < n; ++j)
    {
        double q_col[EXAMPLE_DIM];
        double dq_col[EXAMPLE_DIM];
        double ddq_col[EXAMPLE_DIM];
        example_eval_lissajous_sample(s[j], q_col, dq_col, ddq_col, NULL);
        for (size_t i = 0; i < EXAMPLE_DIM; ++i)
        {
            q[i + j * dim] = q_col[i];
            dq[i + j * dim] = dq_col[i];
            ddq[i + j * dim] = ddq_col[i];
        }
    }

    return COPP_STATUS_OK;
}

static enum CoppStatus example_evaluate_path_3rd(void *user_data,
                                                 size_t dim,
                                                 size_t n,
                                                 const double *s,
                                                 double *q,
                                                 double *dq,
                                                 double *ddq,
                                                 double *dddq)
{
    (void)user_data;
    if (dim != EXAMPLE_DIM)
    {
        return COPP_STATUS_INVALID_ARGUMENT;
    }
    if (n > 0 && (s == NULL || q == NULL || dq == NULL || ddq == NULL || dddq == NULL))
    {
        return COPP_STATUS_NULL_POINTER;
    }

    for (size_t j = 0; j < n; ++j)
    {
        double q_col[EXAMPLE_DIM];
        double dq_col[EXAMPLE_DIM];
        double ddq_col[EXAMPLE_DIM];
        double dddq_col[EXAMPLE_DIM];
        example_eval_lissajous_sample(s[j], q_col, dq_col, ddq_col, dddq_col);
        for (size_t i = 0; i < EXAMPLE_DIM; ++i)
        {
            q[i + j * dim] = q_col[i];
            dq[i + j * dim] = dq_col[i];
            ddq[i + j * dim] = ddq_col[i];
            dddq[i + j * dim] = dddq_col[i];
        }
    }

    return COPP_STATUS_OK;
}

static int example_create_analytic_path_2nd(struct CoppPath **out_path)
{
    *out_path = NULL;
    enum CoppStatus status = copp_path_from_evaluator_2nd(
        EXAMPLE_DIM,
        0.0,
        1.0,
        example_evaluate_path_2nd,
        NULL,
        out_path);
    return example_expect_ok(status, "copp_path_from_evaluator_2nd");
}

static int example_create_analytic_path_3rd(struct CoppPath **out_path)
{
    *out_path = NULL;
    enum CoppStatus status = copp_path_from_evaluator_3rd(
        EXAMPLE_DIM,
        0.0,
        1.0,
        example_evaluate_path_2nd,
        example_evaluate_path_3rd,
        NULL,
        out_path);
    return example_expect_ok(status, "copp_path_from_evaluator_3rd");
}

static int example_add_second_order_limits(struct CoppRobot *robot, size_t n)
{
    double vel_max[EXAMPLE_DIM] = {1.0, 1.0, 1.0};
    double vel_min[EXAMPLE_DIM] = {-1.0, -1.0, -1.0};
    double acc_max[EXAMPLE_DIM] = {1.0, 1.0, 1.0};
    double acc_min[EXAMPLE_DIM] = {-1.0, -1.0, -1.0};

    enum CoppStatus status = copp_add_axial_velocity_limits(
        robot,
        0,
        n,
        (struct CoppSliceF64){vel_max, EXAMPLE_DIM},
        (struct CoppSliceF64){vel_min, EXAMPLE_DIM});
    if (example_expect_ok(status, "copp_add_axial_velocity_limits"))
    {
        return 1;
    }

    status = copp_add_axial_acceleration_limits(
        robot,
        0,
        n,
        (struct CoppSliceF64){acc_max, EXAMPLE_DIM},
        (struct CoppSliceF64){acc_min, EXAMPLE_DIM});
    return example_expect_ok(status, "copp_add_axial_acceleration_limits");
}

static int example_add_third_order_limits(struct CoppRobot *robot, size_t n)
{
    double jerk_max[EXAMPLE_DIM] = {1.0, 1.0, 1.0};
    double jerk_min[EXAMPLE_DIM] = {-1.0, -1.0, -1.0};

    if (example_add_second_order_limits(robot, n))
    {
        return 1;
    }

    enum CoppStatus status = copp_add_axial_jerk_limits(
        robot,
        0,
        n,
        (struct CoppSliceF64){jerk_max, EXAMPLE_DIM},
        (struct CoppSliceF64){jerk_min, EXAMPLE_DIM});
    return example_expect_ok(status, "copp_add_axial_jerk_limits");
}

static int example_create_robot_2nd(const struct CoppPath *path,
                                    const double *s,
                                    size_t n,
                                    struct CoppRobot **out_robot)
{
    *out_robot = NULL;
    enum CoppStatus status = copp_robot_create(EXAMPLE_DIM, n, out_robot);
    if (example_expect_ok(status, "copp_robot_create"))
    {
        return 1;
    }

    status = copp_robot_append_s(*out_robot, (struct CoppSliceF64){s, n});
    if (example_expect_ok(status, "copp_robot_append_s"))
    {
        copp_robot_free(*out_robot);
        *out_robot = NULL;
        return 1;
    }

    status = copp_robot_sample_path_2nd(*out_robot, path, 0, n);
    if (example_expect_ok(status, "copp_robot_sample_path_2nd") ||
        example_add_second_order_limits(*out_robot, n))
    {
        copp_robot_free(*out_robot);
        *out_robot = NULL;
        return 1;
    }

    return 0;
}

static int example_create_robot_3rd(const struct CoppPath *path,
                                    const double *s,
                                    size_t n,
                                    struct CoppRobot **out_robot)
{
    *out_robot = NULL;
    enum CoppStatus status = copp_robot_create(EXAMPLE_DIM, n, out_robot);
    if (example_expect_ok(status, "copp_robot_create"))
    {
        return 1;
    }

    status = copp_robot_append_s(*out_robot, (struct CoppSliceF64){s, n});
    if (example_expect_ok(status, "copp_robot_append_s"))
    {
        copp_robot_free(*out_robot);
        *out_robot = NULL;
        return 1;
    }

    status = copp_robot_sample_path_3rd(*out_robot, path, 0, n);
    if (example_expect_ok(status, "copp_robot_sample_path_3rd") ||
        example_add_third_order_limits(*out_robot, n))
    {
        copp_robot_free(*out_robot);
        *out_robot = NULL;
        return 1;
    }

    return 0;
}

static int example_solve_topp2_seed(struct CoppRobot *robot, size_t n, struct CoppVecF64 *out_a)
{
    struct Topp2RaOptions options;
    enum CoppStatus status = topp2_ra_default_options(&options);
    if (example_expect_ok(status, "topp2_ra_default_options"))
    {
        return 1;
    }

    struct Topp2Problem problem = {robot, 0, n - 1, 0.0, 0.0};
    status = topp2_ra(problem, options, out_a);
    if (example_expect_ok(status, "topp2_ra"))
    {
        return 1;
    }
    if (out_a->data == NULL || out_a->len != n)
    {
        fprintf(stderr, "unexpected TOPP2 seed length\n");
        return 1;
    }
    return 0;
}

static int example_seed_third_order_problem(struct CoppRobot *robot,
                                            size_t n,
                                            struct CoppVecF64 *out_a_seed)
{
    if (example_solve_topp2_seed(robot, n, out_a_seed))
    {
        return 1;
    }

    enum CoppStatus status =
        copp_robot_amax_substitute(robot, (struct CoppSliceF64){out_a_seed->data, out_a_seed->len}, 0);
    return example_expect_ok(status, "copp_robot_amax_substitute");
}

static int example_time_from_second_order(const double *s,
                                          size_t n,
                                          struct CoppVecF64 a,
                                          double *out_t_final,
                                          struct CoppVecF64 *out_t_s)
{
    enum CoppStatus status = copp_s_to_t_2nd(
        (struct CoppSliceF64){s, n},
        (struct CoppSliceF64){a.data, a.len},
        0.0,
        out_t_final,
        out_t_s);
    if (example_expect_ok(status, "copp_s_to_t_2nd"))
    {
        return 1;
    }
    if (!isfinite(*out_t_final) || *out_t_final <= 0.0 || *out_t_final >= DBL_MAX)
    {
        fprintf(stderr, "invalid second-order time profile\n");
        return 1;
    }
    return 0;
}

static int example_time_from_third_order(const double *s,
                                         size_t n,
                                         struct CoppProfile3rd profile,
                                         double *out_t_final,
                                         struct CoppVecF64 *out_t_s)
{
    enum CoppStatus status = copp_s_to_t_3rd(
        (struct CoppSliceF64){s, n},
        (struct CoppSliceF64){profile.a.data, profile.a.len},
        (struct CoppSliceF64){profile.b.data, profile.b.len},
        profile.num_stationary_start,
        profile.num_stationary_end,
        0.0,
        out_t_final,
        out_t_s);
    if (example_expect_ok(status, "copp_s_to_t_3rd"))
    {
        return 1;
    }
    if (!isfinite(*out_t_final) || *out_t_final <= 0.0 || *out_t_final >= DBL_MAX)
    {
        fprintf(stderr, "invalid third-order time profile\n");
        return 1;
    }
    return 0;
}

static int example_interpolate_second_order(const double *s,
                                            size_t n,
                                            struct CoppVecF64 a,
                                            struct CoppVecF64 t_s,
                                            struct CoppVecF64 *out_s_t)
{
    enum CoppStatus status = copp_t_to_s_uniform_2nd(
        (struct CoppSliceF64){s, n},
        (struct CoppSliceF64){a.data, a.len},
        (struct CoppSliceF64){t_s.data, t_s.len},
        0.0,
        1e-3,
        true,
        out_s_t);
    return example_expect_ok(status, "copp_t_to_s_uniform_2nd");
}

static int example_interpolate_third_order(const double *s,
                                           size_t n,
                                           struct CoppProfile3rd profile,
                                           struct CoppVecF64 t_s,
                                           struct CoppVecF64 *out_s_t)
{
    enum CoppStatus status = copp_t_to_s_uniform_3rd(
        (struct CoppSliceF64){s, n},
        (struct CoppSliceF64){profile.a.data, profile.a.len},
        (struct CoppSliceF64){profile.b.data, profile.b.len},
        profile.num_stationary_start,
        profile.num_stationary_end,
        (struct CoppSliceF64){t_s.data, t_s.len},
        0.0,
        1e-3,
        true,
        out_s_t);
    return example_expect_ok(status, "copp_t_to_s_uniform_3rd");
}

#endif /* COPP_C_EXAMPLE_COMMON_H */
