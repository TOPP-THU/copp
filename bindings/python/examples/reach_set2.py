"""Compute second-order reachable sets for a JAX-defined path.

The example applies velocity and acceleration limits in ``[-1, 1]``, then
compares backward-only and bidirectional bounds on ``a(s) = dot(s)^2``.
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

    # 3) Build TOPP2 problem and compute reachable sets.
    problem = copp.solver.reach_set2.Problem(
        robot.constraints,
        idx_s_interval=(0, n - 1),
        a_boundary=(0.0, 0.0),
    )
    options = copp.solver.reach_set2.Options()

    # Backward-only reachability constrains the terminal boundary.
    reach_back = copp.solver.reach_set2.backward(problem, options)
    # Bidirectional reachability constrains both start and terminal boundaries.
    reach_bidir = copp.solver.reach_set2.bidirectional(problem, options)

    # 4) Print the tutorial summary.
    print("reach_set2 done.")
    print(f"dim = {dim}, N = {n}")
    print(
        "backward-only: "
        f"a_max.len() = {len(reach_back.a_max)}, "
        f"a_min.len() = {len(reach_back.a_min)}"
    )
    print(
        "bidirectional: "
        f"a_max.len() = {len(reach_bidir.a_max)}, "
        f"a_min.len() = {len(reach_bidir.a_min)}"
    )

    k0 = 0
    km = n // 2
    k1 = n - 1
    print(
        "bidirectional bounds @k=0/mid/end: "
        f"[{reach_bidir.a_min[k0]:.6f}, {reach_bidir.a_max[k0]:.6f}], "
        f"[{reach_bidir.a_min[km]:.6f}, {reach_bidir.a_max[km]:.6f}], "
        f"[{reach_bidir.a_min[k1]:.6f}, {reach_bidir.a_max[k1]:.6f}]"
    )


if __name__ == "__main__":
    main()
