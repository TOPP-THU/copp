"""Reach-Set 2: Backward and bidirectional reachable sets.

Replicates examples/reach_set2.rs with matplotlib visualization.
"""

import numpy as np
import matplotlib.pyplot as plt
import copp
from _common import lissajous_waypoints, save_figure

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

# ─── 3) Compute reachable sets ───
idx_s_interval = (0, N - 1)
a_boundary = (0.0, 0.0)

a_min_back, a_max_back = copp.reach_set2_backward(
    robot, idx_s_interval, a_boundary
)
a_min_bidir, a_max_bidir = copp.reach_set2_bidirectional(
    robot, idx_s_interval, a_boundary
)

print(f"reach_set2 done. dim={DIM}, N={N}")
print(f"  backward:      a_max.len={len(a_max_back)}")
print(f"  bidirectional: a_max.len={len(a_max_bidir)}")

# ─── 4) Visualization ───
fig, axes = plt.subplots(1, 2, figsize=(12, 5))
fig.suptitle("Reachable Sets (a = ṡ²)", fontsize=14)

axes[0].fill_between(s, a_min_back, a_max_back, alpha=0.3, color="blue")
axes[0].plot(s, a_max_back, "b-", linewidth=0.8, label="a_max")
axes[0].plot(s, a_min_back, "b--", linewidth=0.8, label="a_min")
axes[0].set_xlabel("s")
axes[0].set_ylabel("a(s) = ṡ²")
axes[0].set_title("Backward-only reachable set")
axes[0].legend()
axes[0].grid(True, alpha=0.3)

axes[1].fill_between(s, a_min_bidir, a_max_bidir, alpha=0.3, color="green")
axes[1].plot(s, a_max_bidir, "g-", linewidth=0.8, label="a_max")
axes[1].plot(s, a_min_bidir, "g--", linewidth=0.8, label="a_min")
axes[1].set_xlabel("s")
axes[1].set_ylabel("a(s) = ṡ²")
axes[1].set_title("Bidirectional reachable set")
axes[1].legend()
axes[1].grid(True, alpha=0.3)

save_figure(fig, "reach_set2")
