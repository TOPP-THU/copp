#include "example_common.h"

int main(void)
{
    /*
     * 1) Deterministic 3-axis Lissajous path q(s), s in [0, 1].
     */
    double s[EXAMPLE_NUM_POINTS];
    double normalize[EXAMPLE_DIM] = {1.0, 1.0, 1.0};
    struct CoppPath *path = NULL;
    struct CoppRobot *robot = NULL;
    struct CoppVecF64 a_socp = {NULL, 0, 0};
    struct CoppVecF64 t_s = {NULL, 0, 0};
    struct CoppVecF64 s_t = {NULL, 0, 0};
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
     * 3) Build COPP2 problem and solve COPP2-SOCP with Clarabel.
     *
     * The objective is 1.0 * time + 0.1 * thermal energy. With no
     * inverse-dynamics callback installed, torque objectives use point
     * dynamics (`tau = ddq`).
     */
    struct CoppClarabelOptions options;
    enum CoppStatus status = copp_clarabel_default_options(&options);
    if (example_expect_ok(status, "copp_clarabel_default_options"))
    {
        goto cleanup;
    }
    options.allow_almost_solved = true;

    struct CoppSliceF64 empty = {NULL, 0};
    struct CoppSliceF64 normalize_slice = {normalize, EXAMPLE_DIM};
    struct CoppObjective objectives[2] = {
        {COPP_OBJECTIVE_KIND_TIME, 1.0, empty, empty, empty},
        {COPP_OBJECTIVE_KIND_THERMAL_ENERGY, 0.1, empty, empty, normalize_slice},
    };
    struct Copp2Problem problem = {
        robot,
        0,
        EXAMPLE_NUM_POINTS - 1,
        0.0,
        0.0,
        objectives,
        2,
    };

    status = copp2_socp(problem, options, &a_socp);
    if (example_expect_ok(status, "copp2_socp"))
    {
        goto cleanup;
    }

    /*
     * 4) Post-process COPP2-SOCP results: a(s) -> t(s) -> s(t).
     */
    double t_final = 0.0;
    if (example_time_from_second_order(s, EXAMPLE_NUM_POINTS, a_socp, &t_final, &t_s) ||
        example_interpolate_second_order(s, EXAMPLE_NUM_POINTS, a_socp, t_s, &s_t))
    {
        goto cleanup;
    }

    /*
     * 5) Print the tutorial summary.
     */
    printf("COPP2-SOCP done.\n");
    printf("dim = %d, N = %d\n", EXAMPLE_DIM, EXAMPLE_NUM_POINTS);
    printf("t_final = %.6f s\n", t_final);
    printf("a_profile.len() = %zu\n", a_socp.len);
    printf("s(t) samples = %zu\n", s_t.len);
    rc = 0;

cleanup:
    copp_vec_f64_free(s_t);
    copp_vec_f64_free(t_s);
    copp_vec_f64_free(a_socp);
    copp_robot_free(robot);
    copp_path_free(path);
    return rc;
}
