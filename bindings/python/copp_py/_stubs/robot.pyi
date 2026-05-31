from __future__ import annotations

from typing import Callable, Protocol

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .core import LimitLike, MatrixLayout, MatrixLayoutLike
from .constraints import Constraints
from .path import Path

class _InverseDynamicsObject(Protocol):
    """Object protocol accepted by ``Robot`` for inverse dynamics."""

    def inverse_dynamics(
        self,
        q: ArrayLike,
        dq: ArrayLike,
        ddq: ArrayLike,
    ) -> ArrayLike:
        """Return torque values for one robot state."""
        ...


InverseDynamicsLike = (
    Callable[[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]], ArrayLike]
    | _InverseDynamicsObject
)


class Robot:
    """Robot model and station-indexed constraint buffer.

    ``Robot`` owns a Rust robot wrapper and its constraint storage. When
    ``inverse_dynamics`` is omitted, torque-related methods use the point-mass
    convention ``tau = ddq``. Passing a Python callable or an object with an
    ``inverse_dynamics(q, dq, ddq)`` method enables user-defined dynamics.

    The public API uses Python-style method names such as ``append_s`` and
    ``add_velocity_limits`` while preserving Rust semantics. Matrices default
    to ``MatrixLayout.SAMPLE_MAJOR``: rows are station samples and columns are
    robot dimensions or constraint rows.
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

    has_inverse_dynamics: bool
    """Whether a Python inverse-dynamics callback is installed."""

    constraints: Constraints
    """Reference proxy for raw constraint-buffer operations.

    Each access returns a fresh proxy object that mutates the same robot-owned
    constraint buffer.
    """

    def __init__(
        self,
        dim: int,
        *,
        capacity: int | None = None,
        inverse_dynamics: InverseDynamicsLike | None = None,
    ) -> None:
        """Construct a robot.

        Parameters
        ----------
        dim:
            Positive robot/path dimension.
        capacity:
            Optional initial station-buffer capacity.
        inverse_dynamics:
            Optional callable ``f(q, dq, ddq)`` or object with an
            ``inverse_dynamics(q, dq, ddq)`` method. The callback receives
            contiguous one-dimensional ``float64`` arrays with shape
            ``(dim,)`` and must return values convertible to a ``float64``
            vector of shape ``(dim,)``.

        Raises
        ------
        ValueError
            If ``dim``, ``capacity``, or the callback protocol is invalid.
        """
        ...

    @staticmethod
    def point_mass(dim: int, *, capacity: int | None = None) -> Robot:
        """Construct an explicit point-mass robot with ``tau = ddq``.

        This is a convenience alias for ``Robot(dim, capacity=capacity)``.
        """
        ...

    def __len__(self) -> int:
        """Return the number of stored station samples."""
        ...

    def set_inverse_dynamics(
        self,
        inverse_dynamics: InverseDynamicsLike,
    ) -> None:
        """Replace the inverse-dynamics callback.

        Existing station and constraint data are preserved. Future torque-limit
        construction uses the new callback.
        """
        ...

    def clear_inverse_dynamics(self) -> None:
        """Restore point-mass torque evaluation ``tau = ddq``."""
        ...

    def append_s(self, s: ArrayLike) -> None:
        """Append a strictly increasing station grid segment.

        ``s`` may be any one-dimensional ArrayLike value convertible to
        ``float64``. If the robot already contains stations, ``s[0]`` must be
        greater than the current last stored station.

        Raises
        ------
        ValueError
            If ``s`` cannot be converted to a one-dimensional ``float64`` array.
        CoppError
            If the Rust constraint buffer rejects the grid.
        """
        ...

    def set_q(
        self,
        q: ArrayLike,
        dq: ArrayLike,
        ddq: ArrayLike,
        idx_s: int,
        *,
        dddq: ArrayLike | None = None,
        layout: MatrixLayoutLike = MatrixLayout.SAMPLE_MAJOR,
    ) -> None:
        """Store discrete path derivatives over an existing station interval.

        With ``SAMPLE_MAJOR`` layout, each matrix has shape
        ``(n_samples, dim)``. With ``DIM_MAJOR`` layout, each matrix has shape
        ``(dim, n_samples)``. ``idx_s`` is the global start station id; it has
        no default to avoid accidental writes to the wrong station window.
        """
        ...

    def set_q_from_path_2nd(
        self,
        path: Path,
        idx_s_from: int,
        idx_s_to: int,
    ) -> None:
        """Sample ``path`` up to second order and store ``q``, ``dq``, ``ddq``.

        The half-open interval ``[idx_s_from, idx_s_to)`` must already exist in
        the robot station buffer. Third-order derivative data is cleared over
        the written interval.
        """
        ...

    def set_q_from_path_3rd(
        self,
        path: Path,
        idx_s_from: int,
        idx_s_to: int,
    ) -> None:
        """Sample ``path`` up to third order and store all derivative matrices."""
        ...

    def add_velocity_limits(
        self,
        upper: LimitLike,
        lower: LimitLike,
        *,
        start_idx_s: int,
        length: int | None = None,
        layout: MatrixLayoutLike = MatrixLayout.SAMPLE_MAJOR,
    ) -> None:
        """Add axial velocity limits over an existing station interval.

        ``upper`` values must be positive and ``lower`` values must be
        negative. One-dimensional arrays must have length ``dim`` and are
        broadcast over the station interval; two-dimensional arrays must match
        the selected layout and explicit station length. Dimension broadcasting
        is intentionally not supported in the first Python version.
        """
        ...

    def add_acceleration_limits(
        self,
        upper: LimitLike,
        lower: LimitLike,
        *,
        start_idx_s: int,
        length: int | None = None,
        layout: MatrixLayoutLike = MatrixLayout.SAMPLE_MAJOR,
    ) -> None:
        """Add axial acceleration limits over an existing station interval."""
        ...

    def add_jerk_limits(
        self,
        upper: LimitLike,
        lower: LimitLike,
        *,
        start_idx_s: int,
        length: int | None = None,
        layout: MatrixLayoutLike = MatrixLayout.SAMPLE_MAJOR,
    ) -> None:
        """Add axial jerk limits over an existing station interval."""
        ...

    def add_torque_limits(
        self,
        upper: LimitLike,
        lower: LimitLike,
        *,
        start_idx_s: int,
        length: int | None = None,
        layout: MatrixLayoutLike = MatrixLayout.SAMPLE_MAJOR,
    ) -> None:
        """Add axial torque limits using point or Python inverse dynamics.

        Any exception raised by the Python inverse-dynamics callback is
        propagated unchanged. Wrapper-level callback return-format errors are
        reported as ``ValueError``; Rust core errors are reported as
        ``CoppError``.
        """
        ...
