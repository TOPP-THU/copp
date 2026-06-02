"""COPP2-SOCP: Convex-objective 2nd-order path parameterization via SOCP.

Replicates examples/copp2_socp.rs with matplotlib visualization.
Objective: 1.0 * time + 0.1 * thermal_energy.
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
derivs = path.evaluate_up_to_2nd(s)

# ─── 2) Build robot constraints: v,a in [-1, 1] ───
robot = copp.Robot(dim=DIM, capacity=N)
robot.with_s(s)
robot.with_q(derivs.q, derivs.dq, derivs.ddq)
robot.with_axial_velocity(np.ones(DIM), -np.ones(DIM))
robot.with_axial_acceleration(np.ones(DIM), -np.ones(DIM))

# ─── 3) Solve COPP2-SOCP ───
objectives = [
    copp.Objective.time(1.0),
    copp.Objective.thermal_energy(0.1, [1.0] * DIM),
]
idx_s_start, idx_s_final = 0, N - 1
a_start, a_final = 0.0, 0.0
options = copp.ClarabelOptions(allow_almost_solved=True)

a = copp.copp2_socp(
    robot, idx_s_start, idx_s_final, a_start, a_final, objectives, options
)

# ─── 4) Post-process ───
t_final, t_s = copp.s_to_t_topp2(s, a)
dt = 1e-3
s_t = copp.t_to_s_topp2(s, a, t_s, dt=dt)
t_grid = np.arange(len(s_t)) * dt

print(f"COPP2-SOCP done. dim={DIM}, N={N}")
print(f"  t_final = {t_final:.6f} s")
print(f"  s(t) samples = {len(s_t)}")

# ─── 5) Visualization ───
fig, axes = make_figure(f"COPP2-SOCP  (t_final = {t_final:.4f} s)")

plot_a_profile(axes[0, 0], s, (a, {"color": "b", "linewidth": 0.8}))
b = copp.a_to_b_topp2(s, a)
plot_b_profile(axes[0, 1], s, (b, {"color": "g", "linewidth": 0.8}))
plot_s_of_t(axes[1, 0], t_grid, s_t)

q_t = path.evaluate_q(np.asarray(s_t))
plot_joint_kinematics(axes[1, 1], axes[2, 0], axes[2, 1], q_t.q, t_grid, dt, DIM)

save_figure(fig, "copp2_socp")
