"""Use TOPP2-RA to convert a JAX-defined path into a second-order time-optimal trajectory.

The example constrains each axis by velocity and acceleration limits in
``[-1, 1]``, then converts the returned ``a(s)`` profile into ``t(s)`` and
uniform ``s(t)`` samples.
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
    # velocity/acceleration = [-1, 1].
    robot = copp.Robot(dim, capacity=n)
    robot.append_s(s)
    robot.set_q_from_path_2nd(path, 0, n)

    upper = np.ones(dim, dtype=np.float64)
    lower = -upper
    robot.add_velocity_limits(upper, lower, start_idx_s=0, length=n)
    robot.add_acceleration_limits(upper, lower, start_idx_s=0, length=n)

    # 3) Solve TOPP2-RA with boundary values a(0) = 0 and a(1) = 0.
    problem = copp.solver.topp2_ra.Problem(
        robot.constraints,
        idx_s_interval=(0, n - 1),
        a_boundary=(0.0, 0.0),
    )
    options = copp.solver.topp2_ra.Options()
    a_ra = copp.solver.topp2_ra.solve(problem, options)

    # 4) Post-process TOPP2-RA results: a(s) -> t(s) -> s(t).
    t_final, t_s = copp.interpolation.s_to_t_topp2(s, a_ra, 0.0)
    s_t = copp.interpolation.t_to_s_topp2_uniform(
        s,
        a_ra,
        t_s,
        dt,
        t0=0.0,
        include_final=True,
    )

    # 5) Print the tutorial summary.
    print("TOPP2-RA done.")
    print(f"dim = {dim}, N = {n}")
    print(f"t_final = {t_final:.6f} s")
    print(f"a_profile.len() = {len(a_ra)}")
    print(f"s(t) samples = {len(s_t)}")


if __name__ == "__main__":
    main()
