from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .core import MatrixLayout, MatrixLayoutLike


class Constraints:
    """Raw constraint buffer or reference proxy.

    ``Constraints`` can be constructed independently for TOPP-only workflows,
    or obtained from ``robot.constraints`` when a ``Robot`` also owns path
    derivative data and torque limits. Independent constraints use the same
    point-mass-backed storage path as ``Robot``.
    """

    dim: int
    """Robot/path dimension."""

    len: int
    """Number of stored station samples."""

    capacity: int
    """Allocated station-buffer capacity."""

    is_empty: bool
    """Whether no station samples are stored."""

    idx_s_range: tuple[int, int]
    """Active global station-id range ``(idx_s_start, idx_s_end)``."""

    def __init__(self, dim: int, *, capacity: int | None = None) -> None:
        """Construct an independent raw constraint buffer.

        Parameters
        ----------
        dim:
            Positive robot/path dimension.
        capacity:
            Optional initial station-buffer capacity.
        """
        ...

    def append_s(self, s: ArrayLike) -> None:
        """Append a strictly increasing station grid segment.

        ``s`` may be any one-dimensional ArrayLike value convertible to
        ``float64``. If the buffer already contains stations, ``s[0]`` must be
        greater than the current last stored station.
        """
        ...

    def s_values(
        self,
        idx_s_from: int | None = None,
        idx_s_to: int | None = None,
    ) -> NDArray[np.float64]:
        """Return stored station values over ``[idx_s_from, idx_s_to)``."""
        ...

    def amax_values(
        self,
        idx_s_from: int | None = None,
        idx_s_to: int | None = None,
    ) -> NDArray[np.float64]:
        """Return first-order upper bounds over ``[idx_s_from, idx_s_to)``."""
        ...

    def amax_substitute(self, amax: ArrayLike, idx_s: int) -> None:
        """Overwrite first-order upper bounds from ``idx_s``.

        ``amax`` must be convertible to a one-dimensional ``float64`` array
        whose length fits the stored station interval.
        """
        ...

    def clear(self, *, keep_idx_s: bool = False) -> None:
        """Clear all stored constraints and, unless requested, station data."""
        ...

    def pop_front_n(self, n_cols: int) -> None:
        """Remove ``n_cols`` station samples from the front."""
        ...

    def pop_back_n(self, n_cols: int) -> None:
        """Remove ``n_cols`` station samples from the back."""
        ...

    def pop_front_until(self, idx_s_cut: int) -> None:
        """Remove front samples until the kept window starts at ``idx_s_cut``."""
        ...

    def pop_back_until(self, idx_s_cut: int) -> None:
        """Remove back samples until the kept window ends before ``idx_s_cut``."""
        ...

    def add_constraint_1st(
        self,
        amax: ArrayLike,
        idx_s: int,
        *,
        layout: MatrixLayoutLike = MatrixLayout.SAMPLE_MAJOR,
    ) -> None:
        """Add or tighten raw first-order upper-bound rows."""
        ...

    def add_constraint_2nd(
        self,
        acc_a: ArrayLike,
        acc_b: ArrayLike,
        acc_max: ArrayLike,
        idx_s: int,
        *,
        is_negative: bool = False,
        layout: MatrixLayoutLike = MatrixLayout.SAMPLE_MAJOR,
    ) -> None:
        """Add raw second-order rows ``acc_a * a + acc_b * b <= acc_max``.

        ``is_negative=True`` forwards the Rust sign-flip mode used when
        constructing lower-bound rows from the same expression.
        """
        ...

    def add_constraint_3rd(
        self,
        jerk_a: ArrayLike,
        jerk_b: ArrayLike,
        jerk_c: ArrayLike,
        jerk_d: ArrayLike,
        jerk_max: ArrayLike,
        idx_s: int,
        *,
        is_negative: bool = False,
        layout: MatrixLayoutLike = MatrixLayout.SAMPLE_MAJOR,
    ) -> None:
        """Add raw third-order jerk rows.

        Rows follow the Rust constraint model
        ``sqrt(a) * (jerk_a*a + jerk_b*b + jerk_c*c + jerk_d) <= jerk_max``.
        ``is_negative=True`` forwards the Rust sign-flip mode.
        """
        ...
