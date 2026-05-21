"""Shared utilities for copp Python examples."""

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

OUTPUTS_DIR = Path(__file__).parent.parent / "outputs"
OUTPUTS_DIR.mkdir(exist_ok=True)


def lissajous_waypoints(n_waypoints: int = 200) -> np.ndarray:
    """Generate a 3-axis Lissajous path as waypoints (3, n_waypoints).

    Matches the Rust examples: sin(2πs), sin(3πs+0.3), sin(5πs+0.7).
    """
    s = np.linspace(0, 1, n_waypoints)
    q = np.array([
        np.sin(2 * np.pi * s + 0.0),
        np.sin(3 * np.pi * s + 0.3),
        np.sin(5 * np.pi * s + 0.7),
    ])
    return q


def plot_joint_kinematics(
    ax_q,
    ax_qd,
    ax_qdd,
    q_arr: np.ndarray,
    t_grid: np.ndarray,
    dt: float,
    dim: int,
    suffix: str = "",
) -> None:
    """Plot joint positions, velocities, and accelerations.

    Args:
        ax_q: Matplotlib axes for q(t).
        ax_qd: Matplotlib axes for q̇(t).
        ax_qdd: Matplotlib axes for q̈(t).
        q_arr: Joint positions, shape (dim, T).
        t_grid: Time grid, shape (T,).
        dt: Time step used for numerical differentiation.
        dim: Number of joints.
        suffix: Optional string appended to subplot titles.
    """
    for i in range(dim):
        ax_q.plot(t_grid, q_arr[i], linewidth=0.8, label=f"q{i+1}")
    ax_q.set_xlabel("t [s]")
    ax_q.set_ylabel("q(t)")
    ax_q.set_title(f"Joint positions{suffix}")
    ax_q.legend()
    ax_q.grid(True, alpha=0.3)

    qd = np.gradient(q_arr, dt, axis=1)
    for i in range(dim):
        ax_qd.plot(t_grid, qd[i], linewidth=0.8, label=f"q̇{i+1}")
    ax_qd.set_xlabel("t [s]")
    ax_qd.set_ylabel("q̇(t)")
    ax_qd.set_title(f"Joint velocities{suffix}")
    ax_qd.legend()
    ax_qd.grid(True, alpha=0.3)

    qdd = np.gradient(qd, dt, axis=1)
    for i in range(dim):
        ax_qdd.plot(t_grid, qdd[i], linewidth=0.8, label=f"q̈{i+1}")
    ax_qdd.set_xlabel("t [s]")
    ax_qdd.set_ylabel("q̈(t)")
    ax_qdd.set_title(f"Joint accelerations{suffix}")
    ax_qdd.legend()
    ax_qdd.grid(True, alpha=0.3)


# ─── Visualization helpers ───


def plot_a_profile(ax, s, *curves) -> None:
    """Plot speed-squared profile a(s) on the given axes.

    Args:
        ax: Matplotlib axes.
        s: Path parameter array.
        *curves: Each curve is (a_array, style_dict) where style_dict
                 is passed to ax.plot (e.g. color, linewidth, label, alpha).
    """
    for a_arr, style in curves:
        ax.plot(s, a_arr, **style)
    ax.set_xlabel("s")
    ax.set_ylabel("a(s) = ṡ²")
    ax.set_title("Speed-squared profile" if len(curves) == 1 else "Speed-squared profiles")
    if len(curves) > 1:
        ax.legend()
    ax.grid(True, alpha=0.3)


def plot_b_profile(ax, s, *curves) -> None:
    """Plot acceleration profile b(s) on the given axes.

    Args:
        ax: Matplotlib axes.
        s: Path parameter array.
        *curves: Each curve is (b_array, style_dict).
    """
    for b_arr, style in curves:
        ax.plot(s[:len(b_arr)], b_arr, **style)
    ax.set_xlabel("s")
    ax.set_ylabel("b(s) = s̈")
    ax.set_title("Acceleration profile" if len(curves) == 1 else "Acceleration profiles")
    if len(curves) > 1:
        ax.legend()
    ax.grid(True, alpha=0.3)


def plot_s_of_t(ax, t_grid, s_t, suffix: str = "") -> None:
    """Plot path parameter s(t) on the given axes."""
    ax.plot(t_grid, s_t, "r-", linewidth=0.8)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("s(t)")
    ax.set_title(f"Path parameter vs time{suffix}")
    ax.grid(True, alpha=0.3)


def make_figure(suptitle: str):
    """Create a standard 3×2 figure for path parameterization examples.

    Returns (fig, axes) with shape (3, 2).
    """
    fig, axes = plt.subplots(3, 2, figsize=(12, 10))
    fig.suptitle(suptitle, fontsize=14)
    return fig, axes


def save_figure(fig, name: str) -> None:
    """tight_layout + save to OUTPUTS_DIR/{name}.png."""
    fig.tight_layout()
    path = OUTPUTS_DIR / f"{name}.png"
    fig.savefig(path, dpi=150)
    print(f"  Saved: {path}")
