/**
 * @file test_reach_set2.c
 * @brief Smoke test for TOPP2 reachable-set C ABI entry points.
 */

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

static int check_reach_set(const char *name,
                           const struct CoppReachSet2Result *result,
                           size_t expected_len,
                           int expect_start_boundary)
{
    if (result->a_max.data == NULL || result->a_min.data == NULL ||
        result->a_max.len != expected_len || result->a_min.len != expected_len)
    {
        fprintf(stderr, "%s has unexpected shape\n", name);
        return 1;
    }

    for (size_t i = 0; i < expected_len; ++i)
    {
        const double a_max = result->a_max.data[i];
        const double a_min = result->a_min.data[i];
        if (!isfinite(a_max) || !isfinite(a_min) || a_min > a_max + 1e-8)
        {
            fprintf(stderr,
                    "%s invalid interval at %zu: a_min=%.17g, a_max=%.17g\n",
                    name,
                    i,
                    a_min,
                    a_max);
            return 1;
        }
    }

    if (fabs(result->a_max.data[expected_len - 1]) > 1e-8 ||
        fabs(result->a_min.data[expected_len - 1]) > 1e-8)
    {
        fprintf(stderr, "%s does not enforce terminal boundary\n", name);
        return 1;
    }

    if (expect_start_boundary &&
        (fabs(result->a_max.data[0]) > 1e-8 || fabs(result->a_min.data[0]) > 1e-8))
    {
        fprintf(stderr, "%s does not enforce start boundary\n", name);
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

    struct CoppPath *path = NULL;
    struct CoppRobot *robot = NULL;
    struct CoppReachSet2Result backward = {{NULL, 0, 0}, {NULL, 0, 0}};
    struct CoppReachSet2Result bidirectional = {{NULL, 0, 0}, {NULL, 0, 0}};
    struct CoppPathOptions path_options;
    struct Topp2RaOptions ra_options;
    int failed = 1;

    struct CoppMatrixViewF64 waypoints_view =
        COPP_MATRIX_VIEW_F64_COLUMN_MAJOR(waypoints, DIM, NUM_WAYPOINTS);
    enum CoppStatus status = copp_path_default_options(0.0, 1.0, &path_options);
    if (expect_ok(status, "copp_path_default_options"))
    {
        goto cleanup;
    }
    status = copp_path_from_waypoints(waypoints_view, path_options, &path);
    if (expect_ok(status, "copp_path_from_waypoints"))
    {
        goto cleanup;
    }

    status = copp_robot_create(DIM, NUM_POINTS, &robot);
    if (expect_ok(status, "copp_robot_create"))
    {
        goto cleanup;
    }

    struct CoppSliceF64 s_slice = {s, NUM_POINTS};
    status = copp_robot_append_s(robot, s_slice);
    if (expect_ok(status, "copp_robot_append_s"))
    {
        goto cleanup;
    }

    status = copp_robot_sample_path_2nd(robot, path, 0, NUM_POINTS);
    if (expect_ok(status, "copp_robot_sample_path_2nd"))
    {
        goto cleanup;
    }

    struct CoppSliceF64 vel_max_slice = {vel_max, DIM};
    struct CoppSliceF64 vel_min_slice = {vel_min, DIM};
    status = copp_add_axial_velocity_limits(robot, 0, NUM_POINTS, vel_max_slice, vel_min_slice);
    if (expect_ok(status, "copp_add_axial_velocity_limits"))
    {
        goto cleanup;
    }

    struct CoppSliceF64 acc_max_slice = {acc_max, DIM};
    struct CoppSliceF64 acc_min_slice = {acc_min, DIM};
    status = copp_add_axial_acceleration_limits(robot, 0, NUM_POINTS, acc_max_slice, acc_min_slice);
    if (expect_ok(status, "copp_add_axial_acceleration_limits"))
    {
        goto cleanup;
    }

    struct Topp2Problem problem = {robot, 0, NUM_POINTS - 1, 0.0, 0.0};
    status = topp2_ra_default_options(&ra_options);
    if (expect_ok(status, "topp2_ra_default_options"))
    {
        goto cleanup;
    }

    status = copp_reach_set2_backward(problem, ra_options, &backward);
    if (expect_ok(status, "copp_reach_set2_backward"))
    {
        goto cleanup;
    }
    if (check_reach_set("backward reach set", &backward, NUM_POINTS, 0))
    {
        goto cleanup;
    }

    status = copp_reach_set2_bidirectional(problem, ra_options, &bidirectional);
    if (expect_ok(status, "copp_reach_set2_bidirectional"))
    {
        goto cleanup;
    }
    if (check_reach_set("bidirectional reach set", &bidirectional, NUM_POINTS, 1))
    {
        goto cleanup;
    }

    printf("reach_set2 C smoke test passed: backward.len=%zu, bidirectional.len=%zu\n",
           backward.a_max.len,
           bidirectional.a_max.len);
    failed = 0;

cleanup:
    copp_reach_set2_result_free(bidirectional);
    copp_reach_set2_result_free(backward);
    copp_robot_free(robot);
    copp_path_free(path);
    return failed;
}
