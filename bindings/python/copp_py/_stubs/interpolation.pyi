from __future__ import annotations

from typing import Any, Callable, ClassVar, Literal, Protocol, TypedDict, overload

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .core import CoppError

class Profile3rd:
    """Node-based third-order TOPP/COPP profile.

    ``Profile3rd`` stores owned copies of ``a`` and ``b`` node profiles plus
    stationary boundary interval counts. It is used by third-order
    interpolation and, later, TOPP3/COPP3 solvers.

    Parameters
    ----------
    a:
        One-dimensional ArrayLike value convertible to ``float64`` for
        ``a = (ds/dt)^2``.
    b:
        One-dimensional ArrayLike value convertible to ``float64`` for
        ``b = dds/dt``.
        Must have the same length as ``a``.
    num_stationary:
        Stationary boundary interval counts ``(start, end)``.

    Examples
    --------
    >>> import numpy as np
    >>> import copp_py as copp
    >>> profile = copp.Profile3rd(
    ...     np.array([1.0, 1.0, 1.0], dtype=np.float64),
    ...     np.array([0.0, 0.0, 0.0], dtype=np.float64),
    ... )
    >>> profile.num_stationary
    (0, 0)
    """

    def __init__(
        self,
        a: ArrayLike,
        b: ArrayLike,
        num_stationary: tuple[int, int] = (0, 0),
    ) -> None:
        """Create a third-order profile from node samples."""
        ...

    @property
    def a(self) -> NDArray[np.float64]:
        """Copy of the node profile ``a = (ds/dt)^2``."""
        ...

    @property
    def b(self) -> NDArray[np.float64]:
        """Copy of the node profile ``b = dds/dt``."""
        ...

    @property
    def num_stationary(self) -> tuple[int, int]:
        """Stationary boundary interval counts as ``(start, end)``."""
        ...

    @property
    def num_stationary_start(self) -> int:
        """Number of stationary intervals at the start boundary."""
        ...

    @property
    def num_stationary_end(self) -> int:
        """Number of stationary intervals at the end boundary."""
        ...

    @property
    def len(self) -> int:
        """Number of station nodes in the profile."""
        ...

    def __len__(self) -> int:
        ...

    def __bool__(self) -> bool:
        ...



def a_to_b_topp2(
    s: ArrayLike,
    a: ArrayLike,
) -> NDArray[np.float64]:
    """Compute segment profile ``b`` from a TOPP2/COPP2 node profile ``a``.

    For each segment ``[s[k], s[k + 1]]``, this returns
    ``b[k] = 0.5 * (a[k + 1] - a[k]) / (s[k + 1] - s[k])``. In the TOPP2/COPP2
    convention, ``a`` stores ``ds/dt`` squared at station nodes, while ``b``
    stores ``dds/dt`` on path segments.

    Parameters
    ----------
    s:
        One-dimensional ArrayLike station grid convertible to ``float64``.
        Must be strictly increasing and contain at least two entries.
    a:
        One-dimensional ArrayLike node profile convertible to ``float64`` and
        with ``len(a) == len(s)``. Values must be finite.

    Returns
    -------
    numpy.ndarray
        Segment profile with dtype ``float64`` and length ``len(s) - 1``.

    Raises
    ------
    ValueError
        If an input cannot be converted to a one-dimensional ``float64`` array.
    CoppError
        If the COPP core rejects the grid or profile.
    """
    ...


def s_to_t_topp2(
    s: ArrayLike,
    a: ArrayLike,
    t0: float = 0.0,
) -> tuple[float, NDArray[np.float64]]:
    """Integrate a TOPP2/COPP2 node profile ``a(s)`` into arrival times.

    The returned array ``t_s`` satisfies ``t_s[i] == t(s[i])`` and starts from
    ``t_s[0] == t0``. The scalar ``t_final`` is equal to ``t_s[-1]``.

    Parameters
    ----------
    s:
        One-dimensional ArrayLike station grid convertible to ``float64``.
        Must be strictly increasing and contain at least two entries.
    a:
        One-dimensional ArrayLike node profile convertible to ``float64`` and
        with ``len(a) == len(s)``. Values must be finite and non-negative. Each
        interval must have a positive finite speed denominator.
    t0:
        Initial time assigned to ``s[0]``.

    Returns
    -------
    tuple[float, numpy.ndarray]
        ``(t_final, t_s)`` where ``t_s`` has the same length as ``s``.

    Raises
    ------
    ValueError
        If an input cannot be converted to a one-dimensional ``float64`` array.
    CoppError
        If the COPP core rejects the grid, profile, or initial time.
    """
    ...


def t_to_s_topp2(
    s: ArrayLike,
    a: ArrayLike,
    t_s: ArrayLike,
    *,
    t0: float = 0.0,
    dt: float | None = None,
    include_final: bool = True,
    t_sample: ArrayLike | None = None,
) -> NDArray[np.float64]:
    """Compatibility wrapper for sampling the inverse TOPP2/COPP2 mapping.

    New code should prefer ``t_to_s_topp2_uniform`` or
    ``t_to_s_topp2_samples`` so the sampling mode is explicit.

    Exactly one sampling mode must be selected:

    - pass ``dt`` for a uniform time grid starting at ``t0``;
    - pass ``t_sample`` for an explicit strictly increasing nonuniform time
      grid.

    Out-of-range query times are returned as ``NaN`` by the COPP core.

    Parameters
    ----------
    s:
        One-dimensional ArrayLike station grid convertible to ``float64``.
    a:
        One-dimensional ArrayLike TOPP2/COPP2 node profile convertible to
        ``float64``. Must have the same length as ``s``.
    t_s:
        One-dimensional ArrayLike arrival-time profile convertible to
        ``float64``. Usually returned by ``s_to_t_topp2``. Must have the same
        length as ``s`` and be strictly increasing.
    t0:
        Start time for uniform-grid sampling. Ignored when ``t_sample`` is
        provided.
    dt:
        Positive uniform time step. Mutually exclusive with ``t_sample``.
    include_final:
        When using ``dt``, append ``s[-1]`` if the generated uniform grid does
        not already include the final station.
    t_sample:
        Explicit nonuniform query times. Must be one-dimensional, finite,
        non-empty, and strictly increasing. Mutually exclusive with ``dt``.

    Returns
    -------
    numpy.ndarray
        Sampled station values ``s(t)`` with dtype ``float64``.

    Raises
    ------
    ValueError
        If inputs cannot be converted to one-dimensional ``float64`` arrays,
        if neither sampling mode is selected, or if both ``dt`` and
        ``t_sample`` are provided.
    CoppError
        If the COPP core rejects the grid, profile, arrival times, or sampling
        parameters.
    """
    ...


def t_to_s_topp2_uniform(
    s: ArrayLike,
    a: ArrayLike,
    t_s: ArrayLike,
    dt: float,
    *,
    t0: float = 0.0,
    include_final: bool = True,
) -> NDArray[np.float64]:
    """Sample inverse TOPP2/COPP2 mapping ``s(t)`` on a uniform time grid.

    Parameters are the same as ``t_to_s_topp2``'s uniform mode, but ``dt`` is a
    required positional or keyword argument and no ``t_sample`` argument exists.
    """
    ...


def t_to_s_topp2_samples(
    s: ArrayLike,
    a: ArrayLike,
    t_s: ArrayLike,
    t_sample: ArrayLike,
) -> NDArray[np.float64]:
    """Sample inverse TOPP2/COPP2 mapping ``s(t)`` at explicit query times.

    ``t_sample`` must be one-dimensional, finite, non-empty, and strictly
    increasing.
    """
    ...


def s_to_t_topp3(
    s: ArrayLike,
    profile: Profile3rd,
    t0: float = 0.0,
) -> tuple[float, NDArray[np.float64]]:
    """Integrate a TOPP3/COPP3 profile into arrival times.

    The input profile is node-based: both ``profile.a`` and ``profile.b`` must
    have the same length as ``s``. The returned array ``t_s`` satisfies
    ``t_s[i] == t(s[i])`` and starts from ``t_s[0] == t0``. The scalar
    ``t_final`` is equal to ``t_s[-1]``.

    Parameters
    ----------
    s:
        One-dimensional ArrayLike station grid convertible to ``float64``.
    profile:
        Third-order profile containing node samples of ``a = (ds/dt)^2`` and
        ``b = dds/dt`` plus stationary boundary counts.
    t0:
        Initial time assigned to ``s[0]``.

    Returns
    -------
    tuple[float, numpy.ndarray]
        ``(t_final, t_s)`` where ``t_s`` has the same length as ``s``.

    Raises
    ------
    ValueError
        If ``s`` cannot be converted to a one-dimensional ``float64`` array.
    CoppError
        If the COPP core rejects the grid, profile, stationary counts, or
        initial time.

    Examples
    --------
    >>> import numpy as np
    >>> import copp_py as copp
    >>> s = np.array([0.0, 0.5, 1.0], dtype=np.float64)
    >>> profile = copp.Profile3rd(
    ...     np.array([1.0, 1.0, 1.0], dtype=np.float64),
    ...     np.array([0.0, 0.0, 0.0], dtype=np.float64),
    ... )
    >>> t_final, t_s = copp.interpolation.s_to_t_topp3(s, profile)
    >>> t_final
    1.0
    """
    ...


def t_to_s_topp3(
    s: ArrayLike,
    profile: Profile3rd,
    t_s: ArrayLike,
    *,
    t0: float = 0.0,
    dt: float | None = None,
    include_final: bool = True,
    t_sample: ArrayLike | None = None,
) -> NDArray[np.float64]:
    """Compatibility wrapper for sampling the inverse TOPP3/COPP3 mapping.

    New code should prefer ``t_to_s_topp3_uniform`` or
    ``t_to_s_topp3_samples`` so the sampling mode is explicit.

    Exactly one sampling mode must be selected:

    - pass ``dt`` for a uniform time grid starting at ``t0``;
    - pass ``t_sample`` for an explicit strictly increasing nonuniform time
      grid.

    Out-of-range query times are returned as ``NaN`` by the COPP core.

    Parameters
    ----------
    s:
        One-dimensional ArrayLike station grid convertible to ``float64``.
    profile:
        Third-order profile used to integrate and invert the time mapping.
    t_s:
        One-dimensional ArrayLike arrival-time profile convertible to
        ``float64``. Usually returned by ``s_to_t_topp3``. Must have the same
        length as ``s`` and be strictly increasing.
    t0:
        Start time for uniform-grid sampling. Ignored when ``t_sample`` is
        provided.
    dt:
        Positive uniform time step. Mutually exclusive with ``t_sample``.
    include_final:
        When using ``dt``, append ``s[-1]`` if the generated uniform grid does
        not already include the final station.
    t_sample:
        Explicit nonuniform query times. Must be one-dimensional, finite,
        non-empty, and strictly increasing. Mutually exclusive with ``dt``.

    Returns
    -------
    numpy.ndarray
        Sampled station values ``s(t)`` with dtype ``float64``.

    Raises
    ------
    ValueError
        If inputs cannot be converted to one-dimensional ``float64`` arrays,
        if neither sampling mode is selected, or if both ``dt`` and
        ``t_sample`` are provided.
    CoppError
        If the COPP core rejects the grid, profile, arrival times, or sampling
        parameters.

    Examples
    --------
    >>> import numpy as np
    >>> import copp_py as copp
    >>> s = np.array([0.0, 0.5, 1.0], dtype=np.float64)
    >>> profile = copp.Profile3rd(
    ...     np.array([1.0, 1.0, 1.0], dtype=np.float64),
    ...     np.array([0.0, 0.0, 0.0], dtype=np.float64),
    ... )
    >>> _, t_s = copp.interpolation.s_to_t_topp3(s, profile)
    >>> copp.interpolation.t_to_s_topp3_uniform(s, profile, t_s, 0.25)
    array([0.  , 0.25, 0.5 , 0.75, 1.  ])
    """
    ...


def t_to_s_topp3_uniform(
    s: ArrayLike,
    profile: Profile3rd,
    t_s: ArrayLike,
    dt: float,
    *,
    t0: float = 0.0,
    include_final: bool = True,
) -> NDArray[np.float64]:
    """Sample inverse TOPP3/COPP3 mapping ``s(t)`` on a uniform time grid.

    Parameters are the same as ``t_to_s_topp3``'s uniform mode, but ``dt`` is a
    required positional or keyword argument and no ``t_sample`` argument exists.
    """
    ...


def t_to_s_topp3_samples(
    s: ArrayLike,
    profile: Profile3rd,
    t_s: ArrayLike,
    t_sample: ArrayLike,
) -> NDArray[np.float64]:
    """Sample inverse TOPP3/COPP3 mapping ``s(t)`` at explicit query times.

    ``t_sample`` must be one-dimensional, finite, non-empty, and strictly
    increasing.
    """
    ...
