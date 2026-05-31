"""Use TOPP3-SOCP to convert a JAX-defined path into a jerk-limited time-optimal trajectory.

The example builds a TOPP2-RA seed, solves a third-order ``(a,b)`` profile with
Clarabel, and performs one refinement around the first solution.
"""

import numpy as np

import copp_py as copp


def main() -> None:
    try:
        import jax
        import jax.numpy as jnp
    except ImportError as exc:
        raise SystemExit(
            'Install JAX to run this example: python -m pip install "copp-py[jax]"'
        ) from exc

    jax.config.update("jax_enable_x64", True)

    dim = 3
    n = 1001
    dt = 1.0e-3

    # 1) Define q(s). Path.from_jax differentiates it up to third order.
    def q_fn(s):
        freq = jnp.array([2.0 * jnp.pi, 3.0 * jnp.pi, 5.0 * jnp.pi], dtype=jnp.float64)
        phase = jnp.array([0.0, 0.3, 0.7], dtype=jnp.float64)
        return jnp.sin(freq * s + phase)

    path = copp.Path.from_jax(q_fn, 0.0, 1.0)
    s = np.linspace(0.0, 1.0, n, dtype=np.float64)

    # 2) Build robot constraints (3-axis), then apply symmetric limits
    # velocity/acceleration/jerk = [-1, 1].
    robot = copp.Robot(dim, capacity=n)
    robot.append_s(s)
    robot.set_q_from_path_3rd(path, 0, n)

    upper = np.ones(dim, dtype=np.float64)
    lower = -upper
    robot.add_velocity_limits(upper, lower, start_idx_s=0, length=n)
    robot.add_acceleration_limits(upper, lower, start_idx_s=0, length=n)
    robot.add_jerk_limits(upper, lower, start_idx_s=0, length=n)

    # 3) Use TOPP2-RA as the initial a(s) linearization for the third-order model.
    topp2_problem = copp.solver.topp2_ra.Problem(
        robot.constraints,
        idx_s_interval=(0, n - 1),
        a_boundary=(0.0, 0.0),
    )
    a_ra0 = copp.solver.topp2_ra.solve(topp2_problem, copp.solver.topp2_ra.Options())
    robot.constraints.amax_substitute(a_ra0, 0)

    # 4) Solve TOPP3-SOCP twice, refreshing the a(s) linearization once.
    options = copp.solver.topp3_socp.Options(allow_almost_solved=True)

    problem1 = copp.solver.topp3_socp.Problem(
        robot.constraints,
        a_ra0,
        idx_s_start=0,
        a_boundary=(0.0, 0.0),
        b_boundary=(0.0, 0.0),
        num_stationary_max=1,
    )
    profile1 = copp.solver.topp3_socp.solve(problem1, options)
    t_final1, t_s1 = copp.interpolation.s_to_t_topp3(s, profile1, 0.0)
    s_t1 = copp.interpolation.t_to_s_topp3_uniform(
        s,
        profile1,
        t_s1,
        dt,
        t0=0.0,
        include_final=True,
    )

    problem2 = copp.solver.topp3_socp.Problem(
        robot.constraints,
        profile1.a,
        idx_s_start=0,
        a_boundary=(0.0, 0.0),
        b_boundary=(0.0, 0.0),
        num_stationary_max=1,
    )
    profile2 = copp.solver.topp3_socp.solve(problem2, options)
    t_final2, t_s2 = copp.interpolation.s_to_t_topp3(s, profile2, 0.0)
    s_t2 = copp.interpolation.t_to_s_topp3_uniform(
        s,
        profile2,
        t_s2,
        dt,
        t0=0.0,
        include_final=True,
    )

    # 5) Print the tutorial summary.
    print("TOPP3-SOCP done. (The first iteration)")
    print(f"dim = {dim}, N = {n}")
    print(f"t_final = {t_final1:.6f} s")
    print(f"a_profile.len() = {len(profile1.a)}")
    print(f"b_profile.len() = {len(profile1.b)}")
    print(f"s(t) samples = {len(s_t1)}")
    print("---------")
    print("TOPP3-SOCP done. (The second iteration)")
    print(f"t_final = {t_final2:.6f} s <= {t_final1:.6f} s")
    print(f"a_profile.len() = {len(profile2.a)}")
    print(f"b_profile.len() = {len(profile2.b)}")
    print(f"s(t) samples = {len(s_t2)}")


if __name__ == "__main__":
    main()
