/**
 * @file test_topp3_lp.c
 * @brief End-to-end smoke test for TOPP3-LP and Clarabel profile helpers.
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

    struct CoppPath *path = NULL;
    struct CoppRobot *robot = NULL;
    struct CoppVecF64 a_seed = {NULL, 0, 0};
    struct CoppVecF64 t_s = {NULL, 0, 0};
    struct CoppProfile3rd profile = {{NULL, 0, 0}, {NULL, 0, 0}, 0, 0};
    struct CoppProfile3rd helper_profile = {{NULL, 0, 0}, {NULL, 0, 0}, 0, 0};
    struct Copp3SocpResult expert_result = {0};
    struct CoppPathOptions path_options;
    struct Topp2RaOptions ra_options;
    struct CoppClarabelOptions lp_options;

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

    status = copp_clarabel_default_options(&lp_options);
    if (expect_ok(status, "copp_clarabel_default_options"))
    {
        copp_vec_f64_free(a_seed);
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }
    lp_options.clarabel_settings.max_iter = 400;

    struct Topp3Problem problem = {
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
    };
    status = topp3_lp(problem, lp_options, &profile);
    if (expect_ok(status, "topp3_lp"))
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

    status = topp3_lp_expert(problem, lp_options, &expert_result);
    if (expect_ok(status, "topp3_lp_expert"))
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

    status = copp_clarabel_solution_to_profile_3rd(
        s_slice,
        (struct CoppSliceF64){expert_result.x.data, expert_result.x.len},
        expert_result.profile.num_stationary_start,
        expert_result.profile.num_stationary_end,
        &helper_profile);
    if (expect_ok(status, "copp_clarabel_solution_to_profile_3rd"))
    {
        copp3_socp_result_free(expert_result);
        copp_vec_f64_free(t_s);
        copp_profile_3rd_free(profile);
        copp_vec_f64_free(a_seed);
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }
    assert(helper_profile.a.len == NUM_POINTS);
    assert(helper_profile.b.len == NUM_POINTS);

    /*
     * Adaptive linearization: re-solve around the accepted profile by pairing
     * its `a` with its `b`.  An empty `b_linearization` keeps the direct mode
     * used by the first solve.
     */
    struct CoppProfile3rd adaptive = {{NULL, 0, 0}, {NULL, 0, 0}, 0, 0};
    struct CoppProfile3rd rejected = {{NULL, 0, 0}, {NULL, 0, 0}, 0, 0};
    struct Topp3Problem adaptive_problem = problem;
    adaptive_problem.a_linearization = (struct CoppSliceF64){profile.a.data, profile.a.len};
    adaptive_problem.b_linearization = (struct CoppSliceF64){profile.b.data, profile.b.len};
    status = topp3_lp(adaptive_problem, lp_options, &adaptive);
    if (expect_ok(status, "topp3_lp adaptive re-linearization"))
    {
        copp_profile_3rd_free(helper_profile);
        copp3_socp_result_free(expert_result);
        copp_vec_f64_free(t_s);
        copp_profile_3rd_free(profile);
        copp_vec_f64_free(a_seed);
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }
    assert(adaptive.a.len == NUM_POINTS);
    assert(adaptive.b.len == NUM_POINTS);

    /* A `b_linearization` shorter than `a_linearization` is rejected. */
    adaptive_problem.b_linearization.len = profile.b.len - 1;
    status = topp3_lp(adaptive_problem, lp_options, &rejected);
    assert(status == COPP_STATUS_SOLVER_INVALID_INPUT);
    assert(rejected.a.data == NULL);

    /* The accepted profile must stay inside the robot constraints. */
    double exceed_1order = NAN;
    double exceed_2order = NAN;
    double exceed_3order = NAN;
    status = copp_robot_exceed_topp3(
        robot,
        problem.idx_s_start,
        (struct CoppSliceF64){adaptive.a.data, adaptive.a.len},
        (struct CoppSliceF64){adaptive.b.data, adaptive.b.len},
        adaptive.num_stationary_start,
        adaptive.num_stationary_end,
        &exceed_1order,
        &exceed_2order,
        &exceed_3order);
    if (expect_ok(status, "copp_robot_exceed_topp3"))
    {
        copp_profile_3rd_free(adaptive);
        copp_profile_3rd_free(helper_profile);
        copp3_socp_result_free(expert_result);
        copp_vec_f64_free(t_s);
        copp_profile_3rd_free(profile);
        copp_vec_f64_free(a_seed);
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }
    printf("TOPP3-LP adaptive profile exceed: %.3e, %.3e, %.3e\n",
           exceed_1order,
           exceed_2order,
           exceed_3order);
    assert(exceed_1order <= 1e-6);
    assert(exceed_2order <= 1e-6);
    assert(exceed_3order <= 1e-6);

    /* Profiles of different lengths report NaN with an OK status. */
    status = copp_robot_exceed_topp3(
        robot,
        problem.idx_s_start,
        (struct CoppSliceF64){adaptive.a.data, adaptive.a.len},
        (struct CoppSliceF64){adaptive.b.data, adaptive.b.len - 1},
        adaptive.num_stationary_start,
        adaptive.num_stationary_end,
        &exceed_1order,
        &exceed_2order,
        &exceed_3order);
    if (expect_ok(status, "copp_robot_exceed_topp3 mismatched lengths"))
    {
        copp_profile_3rd_free(adaptive);
        copp_profile_3rd_free(helper_profile);
        copp3_socp_result_free(expert_result);
        copp_vec_f64_free(t_s);
        copp_profile_3rd_free(profile);
        copp_vec_f64_free(a_seed);
        copp_robot_free(robot);
        copp_path_free(path);
        return 1;
    }
    assert(isnan(exceed_1order) && isnan(exceed_2order) && isnan(exceed_3order));

    printf("TOPP3-LP C smoke test passed: t_final=%.17g, len=%zu\n",
           t_final,
           profile.a.len);

    copp_profile_3rd_free(rejected);
    copp_profile_3rd_free(adaptive);
    copp_profile_3rd_free(helper_profile);
    copp3_socp_result_free(expert_result);
    copp_vec_f64_free(t_s);
    copp_profile_3rd_free(profile);
    copp_vec_f64_free(a_seed);
    copp_robot_free(robot);
    copp_path_free(path);
    return 0;
}
