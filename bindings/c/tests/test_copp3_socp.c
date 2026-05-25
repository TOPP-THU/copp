/**
 * @file test_copp3_socp.c
 * @brief End-to-end smoke test for COPP3-SOCP.
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
        NUM_POINTS = 51
    };
    const double pi = 3.14159265358979323846;

    double waypoints[DIM * NUM_WAYPOINTS];
    for (size_t j = 0; j < NUM_WAYPOINTS; ++j)
    {
        const double s_waypoint = (double)j / (double)(NUM_WAYPOINTS - 1);
        waypoints[0 + j * DIM] = 0.15 * sin(2.0 * pi * s_waypoint);
        waypoints[1 + j * DIM] = 0.12 * cos(1.5 * pi * s_waypoint);
        waypoints[2 + j * DIM] = 0.08 * s_waypoint * (1.0 - s_waypoint);
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
    double jerk_max[DIM] = {500.0, 500.0, 500.0};
    double jerk_min[DIM] = {-500.0, -500.0, -500.0};
    double normalize[DIM] = {1.0, 1.0, 1.0};

    struct CoppPath *path = NULL;
    struct CoppRobot *robot = NULL;
    struct CoppVecF64 a_seed = {NULL, 0, 0};
    struct CoppVecF64 t_s = {NULL, 0, 0};
    struct CoppProfile3rd profile = {{NULL, 0, 0}, {NULL, 0, 0}, 0, 0};
    struct CoppProfile3rd profile_tv = {{NULL, 0, 0}, {NULL, 0, 0}, 0, 0};
    struct CoppProfile3rd callback_profile = {{NULL, 0, 0}, {NULL, 0, 0}, 0, 0};
    struct Copp3SocpResult expert_result = {0};
    struct CoppPathOptions path_options;
    struct Topp2RaOptions ra_options;
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

    struct CoppSliceF64 s_slice = {s, NUM_POINTS};
    status = copp_robot_append_s(robot, s_slice);
    if (expect_ok(status, "copp_robot_append_s"))
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

    status = copp_add_axial_velocity_limits(
        robot,
        0,
        NUM_POINTS,
        (struct CoppSliceF64){vel_max, DIM},
        (struct CoppSliceF64){vel_min, DIM});
    if (expect_ok(status, "copp_add_axial_velocity_limits"))
    {
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }
    status = copp_add_axial_acceleration_limits(
        robot,
        0,
        NUM_POINTS,
        (struct CoppSliceF64){acc_max, DIM},
        (struct CoppSliceF64){acc_min, DIM});
    if (expect_ok(status, "copp_add_axial_acceleration_limits"))
    {
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }
    status = copp_add_axial_jerk_limits(
        robot,
        0,
        NUM_POINTS,
        (struct CoppSliceF64){jerk_max, DIM},
        (struct CoppSliceF64){jerk_min, DIM});
    if (expect_ok(status, "copp_add_axial_jerk_limits"))
    {
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }

    status = topp2_ra_default_options(&ra_options);
    if (expect_ok(status, "topp2_ra_default_options"))
    {
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }
    struct Topp2Problem topp2_problem = {robot, 0, NUM_POINTS - 1, 0.0, 0.0};
    status = topp2_ra(topp2_problem, ra_options, &a_seed);
    if (expect_ok(status, "topp2_ra"))
    {
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }
    assert(a_seed.data != NULL);
    assert(a_seed.len == NUM_POINTS);

    status = copp_clarabel_default_options(&socp_options);
    if (expect_ok(status, "copp_clarabel_default_options"))
    {
        copp_vec_f64_free(a_seed);
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }
    socp_options.clarabel_settings.max_iter = 400;

    struct CoppSliceF64 empty = {NULL, 0};
    struct CoppSliceF64 normalize_slice = {normalize, DIM};
    struct CoppObjective objectives[2] = {
        {COPP_OBJECTIVE_KIND_TIME, 1.0, empty, empty, empty},
        {COPP_OBJECTIVE_KIND_THERMAL_ENERGY, 0.1, empty, empty, normalize_slice},
    };
    struct Copp3Problem problem = {
        robot,
        0,
        (struct CoppSliceF64){a_seed.data, a_seed.len},
        0.0,
        0.0,
        0.0,
        0.0,
        1,
        1,
        1e-10,
        objectives,
        2,
    };
    status = copp3_socp(problem, socp_options, &profile);
    if (expect_ok(status, "copp3_socp thermal"))
    {
        copp_vec_f64_free(a_seed);
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }
    assert(profile.a.data != NULL);
    assert(profile.b.data != NULL);
    assert(profile.a.len == NUM_POINTS);
    assert(profile.b.len == NUM_POINTS);

    double t_final = 0.0;
    status = copp_s_to_t_3rd(
        s_slice,
        (struct CoppSliceF64){profile.a.data, profile.a.len},
        (struct CoppSliceF64){profile.b.data, profile.b.len},
        profile.num_stationary_start,
        profile.num_stationary_end,
        0.0,
        &t_final,
        &t_s);
    if (expect_ok(status, "copp_s_to_t_3rd"))
    {
        copp_profile_3rd_free(profile);
        copp_vec_f64_free(a_seed);
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }
    assert(isfinite(t_final));
    assert(t_final > 0.0);
    assert(t_final < DBL_MAX);
    assert(t_s.len == NUM_POINTS);

    status = copp3_socp_expert(problem, socp_options, &expert_result);
    if (expect_ok(status, "copp3_socp_expert thermal"))
    {
        copp_vec_f64_free(t_s);
        copp_profile_3rd_free(profile);
        copp_vec_f64_free(a_seed);
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }
    assert(expert_result.has_profile);
    assert(expert_result.profile.a.len == NUM_POINTS);
    assert(expert_result.profile.b.len == NUM_POINTS);
    assert(expert_result.x.data != NULL);
    assert(expert_result.z.data != NULL);
    assert(expert_result.s.data != NULL);
    assert(expert_result.objective_terms.data != NULL);
    assert(expert_result.objective_terms.len == 2);
    assert(isfinite(expert_result.objective_value));
    copp3_socp_result_free(expert_result);

    struct CoppObjective tv_objectives[2] = {
        {COPP_OBJECTIVE_KIND_TIME, 1.0, empty, empty, empty},
        {COPP_OBJECTIVE_KIND_TOTAL_VARIATION_TORQUE, 0.1, empty, empty, normalize_slice},
    };
    struct Copp3Problem tv_problem = problem;
    tv_problem.objectives = tv_objectives;
    status = copp3_socp(tv_problem, socp_options, &profile_tv);
    if (expect_ok(status, "copp3_socp total_variation_torque"))
    {
        copp_vec_f64_free(t_s);
        copp_profile_3rd_free(profile);
        copp_vec_f64_free(a_seed);
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }
    assert(profile_tv.a.len == NUM_POINTS);
    assert(profile_tv.b.len == NUM_POINTS);

    status = copp_robot_set_inverse_dynamics(robot, point_inverse_dynamics, NULL);
    if (expect_ok(status, "copp_robot_set_inverse_dynamics"))
    {
        copp_profile_3rd_free(profile_tv);
        copp_vec_f64_free(t_s);
        copp_profile_3rd_free(profile);
        copp_vec_f64_free(a_seed);
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }
    status = copp3_socp(problem, socp_options, &callback_profile);
    if (expect_ok(status, "copp3_socp callback inverse_dynamics"))
    {
        copp_profile_3rd_free(profile_tv);
        copp_vec_f64_free(t_s);
        copp_profile_3rd_free(profile);
        copp_vec_f64_free(a_seed);
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }
    assert(callback_profile.a.len == NUM_POINTS);
    assert(callback_profile.b.len == NUM_POINTS);
    copp_profile_3rd_free(callback_profile);

    printf("COPP3-SOCP C smoke test passed: t_final=%.17g, len=%zu, tv.len=%zu\n",
           t_final,
           profile.a.len,
           profile_tv.a.len);

    copp_profile_3rd_free(profile_tv);
    copp_vec_f64_free(t_s);
    copp_profile_3rd_free(profile);
    copp_vec_f64_free(a_seed);
    copp_robot_free(robot);
    copp_path_free(path);
    return 0;
}
