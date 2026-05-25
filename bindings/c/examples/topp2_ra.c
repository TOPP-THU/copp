#include "example_common.h"

int main(void)
{
    /*
     * 1) Deterministic 3-axis Lissajous path q(s), s in [0, 1].
     *
     * This example uses a callback path equivalent to the native parametric
     * path example, so the analytic q, dq, and ddq values are the same.
     */
    double s[EXAMPLE_NUM_POINTS];
    struct CoppPath *path = NULL;
    struct CoppRobot *robot = NULL;
    struct CoppVecF64 a_ra = {NULL, 0, 0};
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
     * 3) Solve TOPP2-RA with boundary values a(0) = 0 and a(1) = 0.
     * The output profile stores a[k] = (ds/dt)^2 at each station.
     */
    if (example_solve_topp2_seed(robot, EXAMPLE_NUM_POINTS, &a_ra))
    {
        goto cleanup;
    }

    /*
     * 4) Post-process TOPP2-RA results: a(s) -> t(s) -> s(t).
     * `t_s[k]` is the time when station s[k] is reached. `s_t` uses a
     * uniform time grid with dt = 1e-3 s, which is useful for plotting and
     * downstream control.
     */
    double t_final = 0.0;
    if (example_time_from_second_order(s, EXAMPLE_NUM_POINTS, a_ra, &t_final, &t_s) ||
        example_interpolate_second_order(s, EXAMPLE_NUM_POINTS, a_ra, t_s, &s_t))
    {
        goto cleanup;
    }

    /*
     * 5) Print the tutorial summary.
     */
    printf("TOPP2-RA done.\n");
    printf("dim = %d, N = %d\n", EXAMPLE_DIM, EXAMPLE_NUM_POINTS);
    printf("t_final = %.6f s\n", t_final);
    printf("a_profile.len() = %zu\n", a_ra.len);
    printf("s(t) samples = %zu\n", s_t.len);
    rc = 0;

cleanup:
    copp_vec_f64_free(s_t);
    copp_vec_f64_free(t_s);
    copp_vec_f64_free(a_ra);
    copp_robot_free(robot);
    copp_path_free(path);
    return rc;
}
