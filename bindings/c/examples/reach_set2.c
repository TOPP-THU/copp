#include "example_common.h"

int main(void)
{
    /*
     * 1) Deterministic 3-axis Lissajous path q(s), s in [0, 1].
     */
    double s[EXAMPLE_NUM_POINTS];
    struct CoppPath *path = NULL;
    struct CoppRobot *robot = NULL;
    struct CoppReachSet2Result reach_back = {{NULL, 0, 0}, {NULL, 0, 0}};
    struct CoppReachSet2Result reach_bidir = {{NULL, 0, 0}, {NULL, 0, 0}};
    int rc = 1;

    example_fill_stations(s, EXAMPLE_NUM_POINTS);
    if (example_create_analytic_path_2nd(&path))
    {
        goto cleanup;
    }

    /*
     * 2) Build robot constraints (3-axis), then apply symmetric limits
     * velocity/acceleration = [-1, 1].
     */
    if (example_create_robot_2nd(path, s, EXAMPLE_NUM_POINTS, &robot))
    {
        goto cleanup;
    }

    /*
     * 3) Build TOPP2 problem and compute reachable sets. The backward-only
     * pass constrains the terminal boundary; the bidirectional pass constrains
     * both the start and terminal boundaries.
     */
    struct Topp2RaOptions options;
    enum CoppStatus status = topp2_ra_default_options(&options);
    if (example_expect_ok(status, "topp2_ra_default_options"))
    {
        goto cleanup;
    }

    struct Topp2Problem problem = {robot, 0, EXAMPLE_NUM_POINTS - 1, 0.0, 0.0};
    status = copp_reach_set2_backward(problem, options, &reach_back);
    if (example_expect_ok(status, "copp_reach_set2_backward"))
    {
        goto cleanup;
    }

    status = copp_reach_set2_bidirectional(problem, options, &reach_bidir);
    if (example_expect_ok(status, "copp_reach_set2_bidirectional"))
    {
        goto cleanup;
    }

    /*
     * 4) Print the tutorial summary.
     */
    const size_t k0 = 0;
    const size_t km = EXAMPLE_NUM_POINTS / 2;
    const size_t k1 = EXAMPLE_NUM_POINTS - 1;

    printf("reach_set2 done.\n");
    printf("dim = %d, N = %d\n", EXAMPLE_DIM, EXAMPLE_NUM_POINTS);
    printf("backward-only: a_max.len() = %zu, a_min.len() = %zu\n",
           reach_back.a_max.len,
           reach_back.a_min.len);
    printf("bidirectional: a_max.len() = %zu, a_min.len() = %zu\n",
           reach_bidir.a_max.len,
           reach_bidir.a_min.len);
    printf("bidirectional bounds @k=0/mid/end: [%.6f, %.6f], [%.6f, %.6f], [%.6f, %.6f]\n",
           reach_bidir.a_min.data[k0],
           reach_bidir.a_max.data[k0],
           reach_bidir.a_min.data[km],
           reach_bidir.a_max.data[km],
           reach_bidir.a_min.data[k1],
           reach_bidir.a_max.data[k1]);
    rc = 0;

cleanup:
    copp_reach_set2_result_free(reach_bidir);
    copp_reach_set2_result_free(reach_back);
    copp_robot_free(robot);
    copp_path_free(path);
    return rc;
}
