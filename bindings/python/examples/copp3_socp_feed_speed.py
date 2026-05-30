"""COPP3-SOCP with path feed-speed constraint via amax_substitute.

Demonstrates how to impose a path-parameter speed limit ṡ ≤ ṡ_max(s)
by converting it into a first-order bound a = ṡ² ≤ ṡ_max².

In this example the feed-speed profile varies along the path:
  - Normal segments: ṡ ≤ 1.0
  - A "slow zone" in the middle (s ∈ [0.3, 0.7]): ṡ ≤ 0.4

This is common in CNC machining or robotic welding where corners or
critical regions require reduced TCP speed.

The joint limits are intentionally loose and the objective is time-only so
the feed-speed ceiling is the active constraint in the plot.
"""

import numpy as np
import matplotlib.pyplot as plt
import copp
from _common import lissajous_waypoints, plot_joint_kinematics, OUTPUTS_DIR

# ─── 1) Build path from waypoints ───
DIM = 3
N = 1001
waypoints = lissajous_waypoints(n_waypoints=200)
path = copp.Path.from_waypoints(waypoints)

s = np.linspace(*path.s_range, N)
derivs = path.evaluate_up_to_3rd(s)

# ─── 2) Build robot constraints ───
robot = copp.Robot(dim=DIM, capacity=N)
robot.with_s(s)
robot.with_q(derivs.q, derivs.dq, derivs.ddq, derivs.dddq)

# Keep the robot-side bounds wide enough that the path feed-speed limit below
# becomes visible. The Lissajous spline has large path derivatives, so the
# default [-1, 1] bounds would cap ṡ near 0.06 before the feed limit can bind.
VEL_LIMIT = 25.0
ACC_LIMIT = 8_000.0
JERK_LIMIT = 10_000_000.0

robot.with_axial_velocity(
    np.full(DIM, VEL_LIMIT), -np.full(DIM, VEL_LIMIT)
)
robot.with_axial_acceleration(
    np.full(DIM, ACC_LIMIT), -np.full(DIM, ACC_LIMIT)
)
robot.with_axial_jerk(
    np.full(DIM, JERK_LIMIT), -np.full(DIM, JERK_LIMIT)
)

# ─── 3) Path feed-speed constraint: ṡ ≤ ṡ_max(s) ───
# Build a per-station speed limit profile.
# The constraint is: a[k] = ṡ² ≤ ṡ_max(s_k)²
sdot_max = np.ones(N)  # default: ṡ ≤ 1.0 everywhere

# Slow zone: reduce feed speed in the middle section
slow_mask = (s >= 0.3) & (s <= 0.7)
sdot_max[slow_mask] = 0.4

# Convert to amax = ṡ_max² and apply
amax_feed = sdot_max ** 2
robot.amax_substitute(amax_feed, idx_from=0)

# ─── 4) TOPP2-RA for initial linearization ───
idx_s_interval = (0, N - 1)
a_boundary = (0.0, 0.0)
a_ra = copp.topp2_ra(robot, idx_s_interval, a_boundary)

# Use RA result as tighter amax for COPP3
robot.amax_substitute(a_ra, 0)

# ─── 5) COPP3-SOCP iteration 1 ───
objectives = [copp.Objective.time(1.0)]
options = copp.ClarabelOptions(allow_almost_solved=True)

a1, b1, ns1 = copp.copp3_socp(
    robot, 0, a_ra, (0.0, 0.0), (0.0, 0.0), objectives, options
)
t_final1, t_s1 = copp.s_to_t_topp3(s, a1, b1, ns1)

# ─── 6) COPP3-SOCP iteration 2 (SCP) ───
a2, b2, ns2 = copp.copp3_socp(
    robot, 0, a1, (0.0, 0.0), (0.0, 0.0), objectives, options
)
t_final2, t_s2 = copp.s_to_t_topp3(s, a2, b2, ns2)

dt = 1e-3
s_t = copp.t_to_s_topp3(s, a2, b2, ns2, t_s2, dt=dt)
t_grid = np.arange(len(s_t)) * dt

print(f"COPP3-SOCP (feed-speed constrained) done. dim={DIM}, N={N}")
print(f"  Iter 1: t_final = {t_final1:.6f} s")
print(f"  Iter 2: t_final = {t_final2:.6f} s")
print(f"  s(t) samples = {len(s_t)}")

# ─── 7) Visualization ───
fig, axes = plt.subplots(3, 2, figsize=(12, 10))
fig.suptitle(
    f"COPP3-SOCP with feed-speed limit  "
    f"(iter1={t_final1:.4f}s → iter2={t_final2:.4f}s)",
    fontsize=13,
)

# a(s) with feed-speed ceiling
axes[0, 0].fill_between(s, 0, amax_feed, alpha=0.15, color="orange",
                        label="ṡ² ceiling (feed limit)")
axes[0, 0].plot(s, a_ra, "k--", linewidth=0.6, alpha=0.5, label="TOPP2-RA")
axes[0, 0].plot(s, a1, "b-", linewidth=0.6, alpha=0.7, label="COPP3 iter1")
axes[0, 0].plot(s, a2, "r-", linewidth=0.8, label="COPP3 iter2")
axes[0, 0].set_xlabel("s")
axes[0, 0].set_ylabel("a(s) = ṡ²")
axes[0, 0].set_title("Speed-squared profiles")
axes[0, 0].legend(fontsize=8)
axes[0, 0].grid(True, alpha=0.3)

# ṡ(s) — actual feed speed vs limit
sdot_actual = np.sqrt(np.maximum(a2, 0.0))
axes[0, 1].plot(s, sdot_max, "k--", linewidth=1.0, label="ṡ_max (feed limit)")
axes[0, 1].plot(s, sdot_actual, "r-", linewidth=0.8, label="ṡ (actual)")
axes[0, 1].set_xlabel("s")
axes[0, 1].set_ylabel("ṡ(s)")
axes[0, 1].set_title("Path feed speed")
axes[0, 1].legend()
axes[0, 1].grid(True, alpha=0.3)

# s(t)
axes[1, 0].plot(t_grid, s_t, "r-", linewidth=0.8)
axes[1, 0].set_xlabel("t [s]")
axes[1, 0].set_ylabel("s(t)")
axes[1, 0].set_title("Path parameter vs time (iter2)")
axes[1, 0].grid(True, alpha=0.3)

# q(t), qd(t), qdd(t)
q_t = path.evaluate_q(np.asarray(s_t))
plot_joint_kinematics(
    axes[1, 1], axes[2, 0], axes[2, 1], q_t.q, t_grid, dt, DIM,
    suffix=" (iter2)"
)

plt.tight_layout()
plt.savefig(OUTPUTS_DIR / "copp3_socp_feed_speed.png", dpi=150)
print(f"  Saved: {OUTPUTS_DIR / 'copp3_socp_feed_speed.png'}")
