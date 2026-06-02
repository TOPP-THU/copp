"""TOPP3-LP: Time-optimal 3rd-order path parameterization via LP.

Replicates examples/topp3_lp.rs with matplotlib visualization.
Uses SCP (2 iterations) for jerk-constraint linearization.
"""

import numpy as np
import copp
from _common import (
    lissajous_waypoints, plot_joint_kinematics,
    plot_a_profile, plot_b_profile, plot_s_of_t, make_figure, save_figure,
)

# ─── 1) Build path from waypoints ───
DIM = 3
N = 1001
waypoints = lissajous_waypoints(n_waypoints=200)
path = copp.Path.from_waypoints(waypoints)

s = np.linspace(*path.s_range, N)
derivs = path.evaluate_up_to_3rd(s)

# ─── 2) Build robot constraints: v,a,j in [-1, 1] ───
robot = copp.Robot(dim=DIM, capacity=N)
robot.with_s(s)
robot.with_q(derivs.q, derivs.dq, derivs.ddq, derivs.dddq)
robot.with_axial_velocity(np.ones(DIM), -np.ones(DIM))
robot.with_axial_acceleration(np.ones(DIM), -np.ones(DIM))
robot.with_axial_jerk(np.ones(DIM), -np.ones(DIM))

# ─── 3) TOPP2-RA for initial linearization ───
idx_s_start, idx_s_final = 0, N - 1
a_ra = copp.topp2_ra(robot, idx_s_start, idx_s_final, 0.0, 0.0)

robot.amax_substitute(a_ra, 0)

# ─── 4) TOPP3-LP iteration 1 ───
options = copp.ClarabelOptions(
    allow_almost_solved=True, allow_insufficient_progress=True
)
a1, b1, ns1 = copp.topp3_lp(robot, 0, a_ra, 0.0, 0.0, 0.0, 0.0, options)
t_final1, t_s1 = copp.s_to_t_topp3(s, a1, b1, ns1)

# ─── 5) TOPP3-LP iteration 2 (SCP) ───
a2, b2, ns2 = copp.topp3_lp(robot, 0, a1, 0.0, 0.0, 0.0, 0.0, options)
t_final2, t_s2 = copp.s_to_t_topp3(s, a2, b2, ns2)

dt = 1e-3
s_t = copp.t_to_s_topp3(s, a2, b2, ns2, t_s2, dt=dt)
t_grid = np.arange(len(s_t)) * dt

print(f"TOPP3-LP done. dim={DIM}, N={N}")
print(f"  Iter 1: t_final = {t_final1:.6f} s")
print(f"  Iter 2: t_final = {t_final2:.6f} s")
print(f"  s(t) samples = {len(s_t)}")

# ─── 6) Visualization ───
fig, axes = make_figure(
    f"TOPP3-LP  (iter1={t_final1:.4f}s → iter2={t_final2:.4f}s)"
)

plot_a_profile(
    axes[0, 0], s,
    (a_ra, {"color": "k", "linestyle": "--", "linewidth": 0.6, "alpha": 0.5, "label": "TOPP2-RA"}),
    (a1, {"color": "b", "linewidth": 0.6, "alpha": 0.7, "label": "LP iter1"}),
    (a2, {"color": "r", "linewidth": 0.8, "label": "LP iter2"}),
)
plot_b_profile(
    axes[0, 1], s,
    (b1, {"color": "b", "linewidth": 0.6, "alpha": 0.7, "label": "LP iter1"}),
    (b2, {"color": "r", "linewidth": 0.8, "label": "LP iter2"}),
)
plot_s_of_t(axes[1, 0], t_grid, s_t, suffix=" (iter2)")

q_t = path.evaluate_q(np.asarray(s_t))
plot_joint_kinematics(
    axes[1, 1], axes[2, 0], axes[2, 1], q_t.q, t_grid, dt, DIM, suffix=" (iter2)"
)

save_figure(fig, "topp3_lp")
