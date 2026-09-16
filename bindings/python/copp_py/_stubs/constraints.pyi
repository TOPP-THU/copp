from __future__ import annotations

from typing import overload

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .core import MatrixLayout, MatrixLayoutLike
from .interpolation import Profile3rd


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

    def exceed_topp2(
        self,
        a: ArrayLike,
        *,
        idx_s_start: int = 0,
    ) -> tuple[float, float]:
        """Return maximum TOPP2 constraint violations of an ``a`` profile.

        The path acceleration of each interval is reconstructed by the TOPP2
        relation ``b[k] = (a[k+1] - a[k]) / (2 * ds[k])``, and the
        second-order rows of both endpoint stations are checked against that
        interval's ``b``.

        Parameters
        ----------
        a:
            One-dimensional node profile ``a = (ds/dt)^2`` convertible to
            ``float64``, sampled on the stations starting at ``idx_s_start``.
        idx_s_start:
            Global station index of ``a[0]``.

        Returns
        -------
        tuple[float, float]
            ``(exceed_1st, exceed_2nd)``. Each value is ``<= 0`` when the
            profile is feasible and positive when violated. The first-order
            term covers ``0 <= a[k] <= amax[k]``. Both values are ``NaN`` if
            the station range is unavailable or ``b`` cannot be reconstructed,
            i.e. fewer than two stations are given, the station grid is not
            strictly increasing, or ``a`` contains non-finite values.
        """
        ...

    @overload
    def exceed_topp3(
        self,
        profile: Profile3rd,
        /,
        *,
        idx_s_start: int = 0,
    ) -> tuple[float, float, float]:
        """Return maximum TOPP3 constraint violations of a ``Profile3rd``.

        ``profile`` supplies ``a``, ``b``, and ``num_stationary`` together;
        passing ``b`` or ``num_stationary`` as well raises ``ValueError``.
        See the array overload for the returned values.
        """
        ...

    @overload
    def exceed_topp3(
        self,
        a: ArrayLike,
        b: ArrayLike,
        *,
        num_stationary: tuple[int, int] | None = None,
        idx_s_start: int = 0,
    ) -> tuple[float, float, float]:
        """Return maximum TOPP3 constraint violations of an ``(a, b)`` profile.

        The third-order term uses the original nonlinear ``sqrt(a)`` form,
        not the linearized rows, so it audits the profile that is actually
        delivered. Run it after post-processing such as
        ``Profile3rd.force_positive_a``. The third-order term skips the two
        stationary boundary blocks described by ``num_stationary``.

        Parameters
        ----------
        a:
            One-dimensional node profile ``a = (ds/dt)^2`` convertible to
            ``float64``.
        b:
            Node profile ``b = dds/dt`` with the same length as ``a``.
        num_stationary:
            Stationary boundary interval counts ``(start, end)``. ``None``
            means ``(0, 0)``.
        idx_s_start:
            Global station index of ``a[0]``.

        Returns
        -------
        tuple[float, float, float]
            ``(exceed_1st, exceed_2nd, exceed_3rd)``. Each value is ``<= 0``
            when the profile is feasible and positive when violated. All
            values are ``NaN`` if fewer than two stations are given, the
            station range is unavailable, the station grid is not strictly
            increasing, ``a`` and ``b`` disagree in length, or ``a`` or ``b``
            contains non-finite values.
        """
        ...
