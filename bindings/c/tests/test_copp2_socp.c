/**
 * @file test_copp2_socp.c
 * @brief End-to-end smoke test for COPP2-SOCP.
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

static enum CoppStatus point_inverse_dynamics(void *user_data,
                                              size_t dim,
                                              const double *q,
                                              const double *dq,
                                              const double *ddq,
                                              double *tau)
{
    (void)user_data;
    (void)q;
    (void)dq;
    if (dim > 0 && (ddq == NULL || tau == NULL))
    {
        return COPP_STATUS_NULL_POINTER;
    }
    for (size_t i = 0; i < dim; ++i)
    {
        tau[i] = ddq[i];
    }
    return COPP_STATUS_OK;
}

int main(void)
{
    enum
    {
        DIM = 3,
        NUM_WAYPOINTS = 8,
        NUM_POINTS = 81
    };
    const double pi = 3.14159265358979323846;

    double waypoints[DIM * NUM_WAYPOINTS];
    for (size_t j = 0; j < NUM_WAYPOINTS; ++j)
    {
        const double s_waypoint = (double)j / (double)(NUM_WAYPOINTS - 1);
        waypoints[0 + j * DIM] = 0.20 * sin(2.0 * pi * s_waypoint);
        waypoints[1 + j * DIM] = 0.15 * cos(1.5 * pi * s_waypoint);
        waypoints[2 + j * DIM] = 0.10 * s_waypoint * (1.0 - s_waypoint);
    }

    double s[NUM_POINTS];
    for (size_t j = 0; j < NUM_POINTS; ++j)
    {
        s[j] = (double)j / (double)(NUM_POINTS - 1);
    }

    double vel_max[DIM] = {10.0, 10.0, 10.0};
    double vel_min[DIM] = {-10.0, -10.0, -10.0};
    double acc_max[DIM] = {50.0, 50.0, 50.0};
    double acc_min[DIM] = {-50.0, -50.0, -50.0};
    double torque_max_grid[DIM * NUM_POINTS];
    double torque_min_grid[DIM * NUM_POINTS];
    double normalize[DIM] = {1.0, 1.0, 1.0};

    for (size_t j = 0; j < NUM_POINTS; ++j)
    {
        for (size_t i = 0; i < DIM; ++i)
        {
            torque_max_grid[i + j * DIM] = 1.0e6;
            torque_min_grid[i + j * DIM] = -1.0e6;
        }
    }

    struct CoppPath *path = NULL;
    struct CoppRobot *robot = NULL;
    struct CoppVecF64 a_thermal = {NULL, 0, 0};
    struct CoppVecF64 a_tv = {NULL, 0, 0};
    struct CoppVecF64 t_s = {NULL, 0, 0};
    struct Copp2SocpResult expert_result = {0};
    struct CoppPathOptions path_options;
    struct CoppClarabelOptions socp_options;

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
    status = copp_robot_set_inverse_dynamics(robot, point_inverse_dynamics, NULL);
    if (expect_ok(status, "copp_robot_set_inverse_dynamics"))
    {
        copp_robot_free(robot);
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

    struct CoppSliceF64 vel_max_slice = {vel_max, DIM};
    struct CoppSliceF64 vel_min_slice = {vel_min, DIM};
    status = copp_add_axial_velocity_limits(robot, 0, NUM_POINTS, vel_max_slice, vel_min_slice);
    if (expect_ok(status, "copp_add_axial_velocity_limits"))
    {
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }

    struct CoppSliceF64 acc_max_slice = {acc_max, DIM};
    struct CoppSliceF64 acc_min_slice = {acc_min, DIM};
    status = copp_add_axial_acceleration_limits(robot, 0, NUM_POINTS, acc_max_slice, acc_min_slice);
    if (expect_ok(status, "copp_add_axial_acceleration_limits"))
    {
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }

    struct CoppMatrixViewF64 torque_max_view =
        COPP_MATRIX_VIEW_F64_COLUMN_MAJOR(torque_max_grid, DIM, NUM_POINTS);
    struct CoppMatrixViewF64 torque_min_view =
        COPP_MATRIX_VIEW_F64_COLUMN_MAJOR(torque_min_grid, DIM, NUM_POINTS);
    status = copp_add_axial_torque_limits_matrix(robot, 0, torque_max_view, torque_min_view);
    if (expect_ok(status, "copp_add_axial_torque_limits_matrix"))
    {
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }

    status = copp_clarabel_default_options(&socp_options);
    if (expect_ok(status, "copp_clarabel_default_options"))
    {
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }
    socp_options.clarabel_settings.max_iter = 300;

    struct CoppSliceF64 empty = {NULL, 0};
    struct CoppSliceF64 normalize_slice = {normalize, DIM};
    struct CoppObjective thermal_objectives[2] = {
        {COPP_OBJECTIVE_KIND_TIME, 1.0, empty, empty, empty},
        {COPP_OBJECTIVE_KIND_THERMAL_ENERGY, 1.0, empty, empty, normalize_slice},
    };
    struct Copp2Problem thermal_problem = {
        robot,
        0,
        NUM_POINTS - 1,
        0.0,
        0.0,
        thermal_objectives,
        2,
    };
    status = copp2_socp(thermal_problem, socp_options, &a_thermal);
    if (expect_ok(status, "copp2_socp thermal"))
    {
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }
    if (a_thermal.data == NULL || a_thermal.len != NUM_POINTS)
    {
        fprintf(stderr, "unexpected COPP2-SOCP thermal a profile shape\n");
        copp_vec_f64_free(a_thermal);
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }

    struct CoppSliceF64 a_slice = {a_thermal.data, a_thermal.len};
    double t_final = 0.0;
    status = copp_s_to_t_2nd(s_slice, a_slice, 0.0, &t_final, &t_s);
    if (expect_ok(status, "copp_s_to_t_2nd"))
    {
        copp_vec_f64_free(a_thermal);
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }
    if (!isfinite(t_final) || t_final <= 0.0 || t_final >= DBL_MAX || t_s.len != NUM_POINTS)
    {
        fprintf(stderr, "invalid COPP2-SOCP thermal time profile\n");
        copp_vec_f64_free(t_s);
        copp_vec_f64_free(a_thermal);
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }

    status = copp2_socp_expert(thermal_problem, socp_options, &expert_result);
    if (expect_ok(status, "copp2_socp_expert thermal"))
    {
        copp_vec_f64_free(t_s);
        copp_vec_f64_free(a_thermal);
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }
    if (!expert_result.has_a || expert_result.a.data == NULL || expert_result.a.len != NUM_POINTS ||
        expert_result.x.data == NULL || expert_result.x.len < NUM_POINTS ||
        expert_result.z.data == NULL || expert_result.s.data == NULL ||
        !isfinite(expert_result.obj_val) || expert_result.solve_time < 0.0 ||
        expert_result.objective_terms.data == NULL || expert_result.objective_terms.len != 2 ||
        !isfinite(expert_result.objective_value) || expert_result.linsolver.nnz_a == 0)
    {
        fprintf(stderr, "invalid COPP2-SOCP expert result\n");
        copp2_socp_result_free(expert_result);
        copp_vec_f64_free(t_s);
        copp_vec_f64_free(a_thermal);
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }
    if (expert_result.solver_status != COPP_CLARABEL_SOLVER_STATUS_SOLVED &&
        expert_result.solver_status != COPP_CLARABEL_SOLVER_STATUS_ALMOST_SOLVED)
    {
        fprintf(stderr, "unexpected COPP2-SOCP expert solver status: %d\n",
                (int)expert_result.solver_status);
        copp2_socp_result_free(expert_result);
        copp_vec_f64_free(t_s);
        copp_vec_f64_free(a_thermal);
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }
    copp2_socp_result_free(expert_result);

    struct CoppObjective tv_objectives[2] = {
        {COPP_OBJECTIVE_KIND_TIME, 1.0, empty, empty, empty},
        {COPP_OBJECTIVE_KIND_TOTAL_VARIATION_TORQUE, 1.0, empty, empty, normalize_slice},
    };
    struct Copp2Problem tv_problem = {
        robot,
        0,
        NUM_POINTS - 1,
        0.0,
        0.0,
        tv_objectives,
        2,
    };
    status = copp2_socp(tv_problem, socp_options, &a_tv);
    if (expect_ok(status, "copp2_socp total_variation_torque"))
    {
        copp_vec_f64_free(t_s);
        copp_vec_f64_free(a_thermal);
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }
    if (a_tv.data == NULL || a_tv.len != NUM_POINTS)
    {
        fprintf(stderr, "unexpected COPP2-SOCP total-variation a profile shape\n");
        copp_vec_f64_free(a_tv);
        copp_vec_f64_free(t_s);
        copp_vec_f64_free(a_thermal);
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }

    printf("COPP2-SOCP C smoke test passed: t_final=%.17g, thermal.len=%zu, tv.len=%zu\n",
           t_final,
           a_thermal.len,
           a_tv.len);

    copp_vec_f64_free(a_tv);
    copp_vec_f64_free(t_s);
    copp_vec_f64_free(a_thermal);
    copp_robot_free(robot);
    copp_path_free(path);
    return 0;
}
