#include "example_common.h"

int main(void)
{
    /*
     * 1) Deterministic 3-axis Lissajous path q(s), s in [0, 1].
     */
    double s[EXAMPLE_NUM_POINTS];
    struct CoppPath *path = NULL;
    struct CoppRobot *robot = NULL;
    struct CoppVecF64 a_ra0 = {NULL, 0, 0};
    struct CoppProfile3rd profile_lp1 = {{NULL, 0, 0}, {NULL, 0, 0}, 0, 0};
    struct CoppProfile3rd profile_lp2 = {{NULL, 0, 0}, {NULL, 0, 0}, 0, 0};
    struct CoppVecF64 t_s1 = {NULL, 0, 0};
    struct CoppVecF64 t_s2 = {NULL, 0, 0};
    struct CoppVecF64 s_t1 = {NULL, 0, 0};
    struct CoppVecF64 s_t2 = {NULL, 0, 0};
    int rc = 1;

    example_fill_stations(s, EXAMPLE_NUM_POINTS);
    if (example_create_analytic_path_3rd(&path))
    {
        goto cleanup;
    }

    /*
     * 2) Build robot constraints (3-axis), then apply symmetric limits
     * velocity/acceleration/jerk = [-1, 1].
     */
    if (example_create_robot_3rd(path, s, EXAMPLE_NUM_POINTS, &robot))
    {
        goto cleanup;
    }

    /*
     * 3) Solve TOPP2-RA to get a feasible linearization profile for TOPP3.
     */
    if (example_seed_third_order_problem(robot, EXAMPLE_NUM_POINTS, &a_ra0))
    {
        goto cleanup;
    }

    /*
     * 4) Solve TOPP3-LP with the linearization profile from TOPP2-RA.
     */
    struct CoppClarabelOptions options_lp;
    enum CoppStatus status = copp_clarabel_default_options(&options_lp);
    if (example_expect_ok(status, "copp_clarabel_default_options"))
    {
        goto cleanup;
    }
    options_lp.allow_almost_solved = true;

    struct Topp3Problem problem1 = {
        robot,
        0,
        (struct CoppSliceF64){a_ra0.data, a_ra0.len},
        0.0,
        0.0,
        0.0,
        0.0,
        1,
        1,
        1e-10,
    };
    status = topp3_lp(problem1, options_lp, &profile_lp1);
    if (example_expect_ok(status, "topp3_lp first iteration"))
    {
        goto cleanup;
    }

    double t_final1 = 0.0;
    if (example_time_from_third_order(s, EXAMPLE_NUM_POINTS, profile_lp1, &t_final1, &t_s1) ||
        example_interpolate_third_order(s, EXAMPLE_NUM_POINTS, profile_lp1, t_s1, &s_t1))
    {
        goto cleanup;
    }

    printf("TOPP3-LP done. (The first-iteration)\n");
    printf("dim = %d, N = %d\n", EXAMPLE_DIM, EXAMPLE_NUM_POINTS);
    printf("t_final = %.6f s\n", t_final1);
    printf("a_profile.len() = %zu\n", profile_lp1.a.len);
    printf("b_profile.len() = %zu\n", profile_lp1.b.len);
    printf("s(t) samples = %zu\n", s_t1.len);

    /*
     * 5) Solve TOPP3-LP with the first LP profile as the new linearization
     * point, matching the tutorial's second SCP iteration.
     */
    struct Topp3Problem problem2 = {
        robot,
        0,
        (struct CoppSliceF64){profile_lp1.a.data, profile_lp1.a.len},
        0.0,
        0.0,
        0.0,
        0.0,
        1,
        1,
        1e-10,
    };
    status = topp3_lp(problem2, options_lp, &profile_lp2);
    if (example_expect_ok(status, "topp3_lp second iteration"))
    {
        goto cleanup;
    }

    double t_final2 = 0.0;
    if (example_time_from_third_order(s, EXAMPLE_NUM_POINTS, profile_lp2, &t_final2, &t_s2) ||
        example_interpolate_third_order(s, EXAMPLE_NUM_POINTS, profile_lp2, t_s2, &s_t2))
    {
        goto cleanup;
    }

    printf("---------\nTOPP3-LP done. (The second-iteration)\n");
    printf("dim = %d, N = %d\n", EXAMPLE_DIM, EXAMPLE_NUM_POINTS);
    printf("t_final = %.6f s <= %.6f s\n", t_final2, t_final1);
    printf("a_profile.len() = %zu\n", profile_lp2.a.len);
    printf("b_profile.len() = %zu\n", profile_lp2.b.len);
    printf("s(t) samples = %zu\n", s_t2.len);
    rc = 0;

cleanup:
    copp_vec_f64_free(s_t2);
    copp_vec_f64_free(s_t1);
    copp_vec_f64_free(t_s2);
    copp_vec_f64_free(t_s1);
    copp_profile_3rd_free(profile_lp2);
    copp_profile_3rd_free(profile_lp1);
    copp_vec_f64_free(a_ra0);
    copp_robot_free(robot);
    copp_path_free(path);
    return rc;
}
