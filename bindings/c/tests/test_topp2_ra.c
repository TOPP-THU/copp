/**
 * @file test_topp2_ra.c
 * @brief End-to-end smoke test for TOPP2-RA using a waypoint spline path.
 */

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
        DIM = 3,
        NUM_WAYPOINTS = 8,
        NUM_POINTS = 201
    };
    const double pi = 3.14159265358979323846;

    double waypoints[DIM * NUM_WAYPOINTS];
    for (size_t j = 0; j < NUM_WAYPOINTS; ++j)
    {
        const double s = (double)j / (double)(NUM_WAYPOINTS - 1);
        waypoints[0 + j * DIM] = 0.20 * sin(2.0 * pi * s);
        waypoints[1 + j * DIM] = 0.15 * cos(1.5 * pi * s);
        waypoints[2 + j * DIM] = 0.10 * s * (1.0 - s);
    }

    double s[NUM_POINTS];
    double vel_max_grid[DIM * NUM_POINTS];
    double vel_min_grid[DIM * NUM_POINTS];
    double acc_max_grid[DIM * NUM_POINTS];
    double acc_min_grid[DIM * NUM_POINTS];
    double jerk_max_grid[DIM * NUM_POINTS];
    double jerk_min_grid[DIM * NUM_POINTS];
    for (size_t j = 0; j < NUM_POINTS; ++j)
    {
        s[j] = (double)j / (double)(NUM_POINTS - 1);
        for (size_t i = 0; i < DIM; ++i)
        {
            vel_max_grid[j + i * NUM_POINTS] = 10.0;
            vel_min_grid[j + i * NUM_POINTS] = -10.0;
            acc_max_grid[i + j * DIM] = 50.0;
            acc_min_grid[i + j * DIM] = -50.0;
            jerk_max_grid[i + j * DIM] = 1000.0;
            jerk_min_grid[i + j * DIM] = -1000.0;
        }
    }

    struct CoppPath *path = NULL;
    struct CoppRobot *robot = NULL;
    struct CoppVecF64 a = {NULL, 0, 0};
    struct CoppVecF64 t_s = {NULL, 0, 0};
    struct CoppVecF64 s_t = {NULL, 0, 0};
    struct CoppPathOptions path_options;
    struct Topp2RaOptions ra_options;

    struct CoppMatrixViewF64 waypoints_view =
        COPP_MATRIX_VIEW_F64_COLUMN_MAJOR(waypoints, DIM, NUM_WAYPOINTS);
    enum CoppStatus status = copp_path_default_options(0.0, 1.0, &path_options);
    if (expect_ok(status, "copp_path_default_options"))
    {
        return 1;
    }
    status = copp_path_from_waypoints(waypoints_view, path_options, &path);
    if (expect_ok(status, "copp_path_from_waypoints"))
    {
        return 1;
    }

    status = copp_robot_create(DIM, NUM_POINTS, &robot);
    if (expect_ok(status, "copp_robot_create"))
    {
        copp_path_free(path);
        return 1;
    }

    struct CoppSliceF64 s_slice = {s, NUM_POINTS};
    status = copp_robot_append_s(robot, s_slice);
    if (expect_ok(status, "copp_robot_append_s"))
    {
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }

    status = copp_robot_sample_path_2nd(robot, path, 0, NUM_POINTS);
    if (expect_ok(status, "copp_robot_sample_path_2nd"))
    {
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }
    status = copp_robot_sample_path_3rd(robot, path, 0, NUM_POINTS);
    if (expect_ok(status, "copp_robot_sample_path_3rd"))
    {
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }

    struct CoppMatrixViewF64 vel_max_view =
        COPP_MATRIX_VIEW_F64_ROW_MAJOR(vel_max_grid, DIM, NUM_POINTS);
    struct CoppMatrixViewF64 vel_min_view =
        COPP_MATRIX_VIEW_F64_ROW_MAJOR(vel_min_grid, DIM, NUM_POINTS);
    status = copp_add_axial_velocity_limits_matrix(robot, 0, vel_max_view, vel_min_view);
    if (expect_ok(status, "copp_add_axial_velocity_limits_matrix"))
    {
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }

    struct CoppMatrixViewF64 acc_max_view =
        COPP_MATRIX_VIEW_F64_COLUMN_MAJOR(acc_max_grid, DIM, NUM_POINTS);
    struct CoppMatrixViewF64 acc_min_view =
        COPP_MATRIX_VIEW_F64_COLUMN_MAJOR(acc_min_grid, DIM, NUM_POINTS);
    status = copp_add_axial_acceleration_limits_matrix(robot, 0, acc_max_view, acc_min_view);
    if (expect_ok(status, "copp_add_axial_acceleration_limits_matrix"))
    {
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }

    struct CoppMatrixViewF64 jerk_max_view =
        COPP_MATRIX_VIEW_F64_COLUMN_MAJOR(jerk_max_grid, DIM, NUM_POINTS);
    struct CoppMatrixViewF64 jerk_min_view =
        COPP_MATRIX_VIEW_F64_COLUMN_MAJOR(jerk_min_grid, DIM, NUM_POINTS);
    status = copp_add_axial_jerk_limits_matrix(robot, 0, jerk_max_view, jerk_min_view);
    if (expect_ok(status, "copp_add_axial_jerk_limits_matrix"))
    {
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }

    struct Topp2Problem problem = {robot, 0, NUM_POINTS - 1, 0.0, 0.0};
    status = topp2_ra_default_options(&ra_options);
    if (expect_ok(status, "topp2_ra_default_options"))
    {
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }
    status = topp2_ra(problem, ra_options, &a);
    if (expect_ok(status, "topp2_ra"))
    {
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }
    if (a.data == NULL || a.len != NUM_POINTS)
    {
        fprintf(stderr, "unexpected TOPP2-RA a profile shape\n");
        copp_vec_f64_free(a);
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }

    double t_final = 0.0;
    struct CoppSliceF64 a_slice = {a.data, a.len};
    status = copp_s_to_t_2nd(s_slice, a_slice, 0.0, &t_final, &t_s);
    if (expect_ok(status, "copp_s_to_t_2nd"))
    {
        copp_vec_f64_free(a);
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }
    if (!isfinite(t_final) || t_final <= 0.0 || t_final >= DBL_MAX || t_s.len != NUM_POINTS)
    {
        fprintf(stderr, "invalid TOPP2 time profile from sampled path\n");
        copp_vec_f64_free(t_s);
        copp_vec_f64_free(a);
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }
    struct CoppSliceF64 t_s_slice = {t_s.data, t_s.len};
    status = copp_t_to_s_uniform_2nd(s_slice, a_slice, t_s_slice, 0.0, 1e-3, true, &s_t);
    if (expect_ok(status, "copp_t_to_s_uniform_2nd"))
    {
        copp_vec_f64_free(t_s);
        copp_vec_f64_free(a);
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }
    if (s_t.data == NULL || s_t.len == 0)
    {
        fprintf(stderr, "invalid TOPP2 s(t) profile\n");
        copp_vec_f64_free(s_t);
        copp_vec_f64_free(t_s);
        copp_vec_f64_free(a);
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }

    printf("TOPP2-RA C smoke test passed: t_final=%.17g, s_t.len=%zu, a.len=%zu\n",
           t_final,
           s_t.len,
           a.len);

    copp_vec_f64_free(s_t);
    copp_vec_f64_free(t_s);
    copp_vec_f64_free(a);
    copp_robot_free(robot);
    copp_path_free(path);
    return 0;
}
