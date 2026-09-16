use crate::diag::PathError;
use crate::path::OutOfRangeMode;
use crate::path::autodiff::Jet3;
use crate::path::smoothing::{SmoothedSpline, SmoothingConfig, SmoothingReport};
use crate::path::spline::{SplineConfig, SplinePath};
use nalgebra::{DMatrix, DMatrixView};
use rayon::prelude::*;
use std::sync::Arc;

const EPS_RANGE: f64 = 1e-12;

/// Shared analytic path function used by [`Path::from_parametric`].
///
/// The input is a seeded [`Jet3`] scalar representing path parameter `s`, and
/// the returned vector contains one [`Jet3`] per path dimension.
pub type ParametricFn = Arc<dyn Fn(Jet3) -> Vec<Jet3> + Send + Sync>;

/// User-provided path evaluator with explicit derivatives up to second order.
///
/// Implement this trait when path derivatives are already available from an
/// external model, library, or hand-written analytic formula. Unlike
/// [`Path::from_parametric`], no automatic differentiation is performed: the
/// evaluator writes `q`, `dq`, and `ddq` directly into pre-allocated
/// column-major buffers.
///
/// Buffer layout is always `dim x s.len()` in column-major order:
/// `buffer[row + col * dim]` corresponds to path dimension `row` at sample
/// `s[col]`. Empty `s` slices are valid no-ops.
///
/// See [`Path::from_evaluator_2nd`] for a complete constructor example.
pub trait PathEvaluator2nd: Send + Sync {
    /// Return the path dimension.
    ///
    /// The dimension must remain stable for the lifetime of the evaluator and
    /// must be greater than zero.
    fn dim(&self) -> usize;

    /// Evaluate position only.
    ///
    /// The default implementation calls [`PathEvaluator2nd::evaluate_up_to_2nd`]
    /// with temporary derivative buffers. Override this method if computing
    /// only `q` is substantially cheaper for the evaluator.
    fn evaluate_q(&self, s: &[f64], q: &mut [f64]) -> Result<(), PathError> {
        let mut dq = vec![0.0; q.len()];
        let mut ddq = vec![0.0; q.len()];
        self.evaluate_up_to_2nd(s, q, &mut dq, &mut ddq)
    }

    /// Evaluate `q`, `dq`, and `ddq` at all supplied path parameters.
    ///
    /// All output buffers have length `dim() * s.len()` and use column-major
    /// layout.
    fn evaluate_up_to_2nd(
        &self,
        s: &[f64],
        q: &mut [f64],
        dq: &mut [f64],
        ddq: &mut [f64],
    ) -> Result<(), PathError>;
}

/// User-provided path evaluator with explicit derivatives up to third order.
///
/// This extends [`PathEvaluator2nd`] with jerk-level path derivatives. Use it
/// when the path will be sampled by TOPP3/COPP3 workflows or any API that calls
/// [`Path::evaluate_up_to_3rd`].
///
/// See [`Path::from_evaluator_3rd`] for a complete constructor example.
pub trait PathEvaluator3rd: PathEvaluator2nd {
    /// Evaluate `q`, `dq`, `ddq`, and `dddq` at all supplied path parameters.
    ///
    /// All output buffers have length `dim() * s.len()` and use column-major
    /// layout.
    fn evaluate_up_to_3rd(
        &self,
        s: &[f64],
        q: &mut [f64],
        dq: &mut [f64],
        ddq: &mut [f64],
        dddq: &mut [f64],
    ) -> Result<(), PathError>;
}

/// Compatibility alias for third-order explicit path evaluators.
///
/// New code should prefer [`PathEvaluator2nd`] or [`PathEvaluator3rd`] to make
/// the supported derivative order explicit.
pub trait PathEvaluator: PathEvaluator3rd {}

impl<T: PathEvaluator3rd + ?Sized> PathEvaluator for T {}

/// Output of path evaluation.
///
/// `dq`, `ddq`, `dddq` are `None` when the evaluation did not request them
/// (e.g. `evaluate_q` only fills `q`; `evaluate_up_to_2nd` fills `q/dq/ddq`).
#[derive(Debug)]
pub struct PathDerivatives {
    /// Position samples with shape `(dim, s.len())`.
    pub q: DMatrix<f64>,
    /// First derivative samples `dq/ds`, populated by second- and third-order evaluation.
    pub dq: Option<DMatrix<f64>>,
    /// Second derivative samples `d^2q/ds^2`, populated by second- and third-order evaluation.
    pub ddq: Option<DMatrix<f64>>,
    /// Third derivative samples `d^3q/ds^3`, populated only by third-order evaluation.
    pub dddq: Option<DMatrix<f64>>,
}

/// How many derivative orders to compute.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Order {
    Zero,  // q only
    Two,   // q, dq, ddq
    Three, // q, dq, ddq, dddq
}

/// Unified path abstraction over parametric, spline, and evaluator representations.
///
/// Construct via [`Path::from_parametric`](crate::path::Path::from_parametric),
/// [`Path::from_waypoints_interpolating`](crate::path::Path::from_waypoints_interpolating),
/// [`Path::from_waypoints_fitting`](crate::path::Path::from_waypoints_fitting),
/// [`Path::from_evaluator_2nd`](crate::path::Path::from_evaluator_2nd), or
/// [`Path::from_evaluator_3rd`](crate::path::Path::from_evaluator_3rd), then
/// query a batch of parameter values with the `evaluate_*` family of methods.
///
/// The valid parameter domain is `[s_min, s_max]` (set at construction time).
/// Out-of-range behaviour is controlled by [`OutOfRangeMode`](crate::path::OutOfRangeMode): the default is to
/// return an error; it can be changed to silent clamping.
pub struct Path {
    dim: usize,
    s_min: f64,
    s_max: f64,
    out_of_range_mode: OutOfRangeMode,
    repr: PathRepr,
}

enum PathRepr {
    /// Closure-based path evaluated via third-order forward AD.
    Parametric(ParametricFn),
    /// Piecewise-polynomial path built from waypoints.
    Spline(SplinePath),
    /// Tolerance-fitted selected axes plus waypoint interpolation of unselected axes.
    Smoothed(Box<SmoothedSpline>),
    /// User-provided path evaluator with explicit derivatives up to second order.
    Evaluator2nd(Arc<dyn PathEvaluator2nd>),
    /// User-provided path evaluator with explicit derivatives up to third order.
    Evaluator3rd(Arc<dyn PathEvaluator3rd>),
}

impl Path {
    /// Construct a tolerance-bounded `C4` quintic approximation of a waypoint polyline.
    ///
    /// This constructor is intended for sampled or programmed paths whose
    /// selected coordinates may be denoised or rounded within explicit absolute
    /// tolerances. Use
    /// [`Path::from_waypoints_interpolating`](crate::path::Path::from_waypoints_interpolating)
    /// when every waypoint must be interpolated.
    /// For a borrowed compatible strided column-major matrix, use
    /// [`Path::from_waypoints_fitting_view`].
    ///
    /// # API stability
    ///
    /// This constructor is currently **unstable**. Its name, signature,
    /// configuration, and algorithm-selection interface may change as additional
    /// waypoint-fitting methods are introduced.
    ///
    /// # Reference path and tolerances
    ///
    /// `waypoints` has shape `(dim, n_points)` and each **column** is one ordered
    /// waypoint. A common parameter is assigned to every column by
    /// [`SmoothingConfig::parameters`](crate::path::SmoothingConfig::parameters),
    /// or uniformly on `[0, 1]` by default. Adjacent columns are linearly
    /// interpolated at that common parameter to form the reference polyline.
    ///
    /// For each selected axis `j`, successful construction numerically audits
    /// the complete-domain contract
    ///
    /// $$
    /// \max_s |q_j(s)-r_j(s)| \le \varepsilon_j.
    /// $$
    ///
    /// This is a same-parameter, per-axis absolute bound in input-coordinate
    /// units. It is not merely a waypoint check, a nearest-distance/Hausdorff
    /// bound, or a Cartesian error after forward kinematics.
    ///
    /// # Continuity and derivatives
    ///
    /// Selected axes use one adaptive nonuniform quintic B-spline with simple
    /// interior knots, giving `C4` continuity. Their first and last positions
    /// are fixed to the input, while endpoint derivatives are determined by the
    /// fit. Unselected axes use a separate quintic `C4` interpolant through all
    /// input columns, with zero first and second parameter derivatives at both
    /// endpoints. Positional endpoint equalities are subject only to ordinary
    /// floating-point roundoff.
    ///
    /// Returned derivatives are with respect to the configured path parameter
    /// `s`, not time. Smoothing therefore does not replace time parameterization
    /// or impose physical velocity, acceleration, or jerk limits.
    ///
    /// # Construction method
    ///
    /// 1. Parameters are mapped to `[0, 1]`; every selected coordinate is
    ///    shifted by its first value and divided by its tolerance.
    /// 2. A straight-chord fast path is tried first. Otherwise a shared slope
    ///    corridor seeds nonuniform knots near changes in direction.
    /// 3. An open degree-five B-spline is fitted to the reference polyline with
    ///    a span-scaled third-parameter-derivative penalty. Compact banded normal
    ///    equations exploit the six-function local support.
    /// 4. Every spline-span/reference-segment intersection is converted to a
    ///    degree-five Bernstein error polynomial. Its coefficients provide an
    ///    exact-arithmetic sufficient bound over the complete intersection.
    /// 5. Failed regions receive new knots and local control-point refits until
    ///    the computed bounds pass or a configured budget is exhausted.
    /// 6. Coordinates are restored to physical units and the entire domain is
    ///    audited again before the path is returned.
    ///
    /// The least-squares fit does not directly impose hard error constraints;
    /// the tolerance behavior comes from this audit/refinement/rejection loop.
    /// Bernstein bounds are computed with ordinary `f64` arithmetic and are
    /// therefore numerical audit results, not outward-rounded formal proofs.
    ///
    /// # Arguments
    ///
    /// - `waypoints`: finite `(dim, n_points)` matrix with `dim > 0` and
    ///   `n_points >= 2`; each column is one waypoint;
    /// - `cfg`: selected axes, their tolerances, common waypoint parameters,
    ///   refinement budgets, and out-of-range query behavior.
    ///
    /// # Returns
    ///
    /// A unified [`Path`] that supports position and derivatives through
    /// [`Path::evaluate_up_to_3rd`]. Construction diagnostics and final error
    /// bounds are available from [`Path::smoothing_report`].
    ///
    /// # Errors
    ///
    /// Returns [`PathError::Smoothing`](crate::diag::PathError::Smoothing) when:
    ///
    /// - waypoint values, dimensions, axis indices, tolerances, or parameters
    ///   violate their input contracts;
    /// - the selected-axis span count exceeds the configured segment budget;
    /// - the banded fit encounters an unsupported or non-finite numerical system;
    /// - refinement exhausts its pass budget or parameter resolution; or
    /// - the final physical-unit whole-interval audit does not satisfy tolerance.
    ///
    /// An unchecked or silently relaxed approximation is never returned.
    ///
    /// # Limitations
    ///
    /// The adaptive knot layout is heuristic and does not guarantee the minimum
    /// span count. Bernstein bounds use ordinary floating-point arithmetic, not
    /// outward-rounded interval arithmetic. No guarantee is made for forward
    /// kinematics, collision avoidance, monotonicity, dynamics, or traversal
    /// time.
    ///
    /// # Example
    ///
    /// ```rust
    /// # fn main() -> Result<(), copp::diag::PathError> {
    /// use copp::path::{Path, SmoothingConfig, SmoothingTolerance};
    /// use nalgebra::DMatrix;
    ///
    /// let waypoints = DMatrix::from_row_slice(
    ///     2,
    ///     3,
    ///     &[
    ///         0.0, 0.5, 1.0,
    ///         0.0, 1.0, 2.0,
    ///     ],
    /// );
    /// let config = SmoothingConfig {
    ///     axes: Some(vec![0, 1]),
    ///     tolerance: SmoothingTolerance::PerAxis(vec![0.001, 0.01]),
    ///     parameters: Some(vec![2.0, 3.0, 4.0]),
    ///     ..Default::default()
    /// };
    ///
    /// let path = Path::from_waypoints_fitting(&waypoints, config)?;
    /// let values = path.evaluate_up_to_3rd(&[2.0, 3.0, 4.0])?;
    /// let report = path.smoothing_report().unwrap();
    ///
    /// assert_eq!(values.q.shape(), (2, 3));
    /// assert_eq!(report.axes, vec![0, 1]);
    /// assert!(report.max_errors[0] <= 0.001);
    /// assert!(report.max_errors[1] <= 0.01);
    /// # Ok(())
    /// # }
    /// ```
    pub fn from_waypoints_fitting(
        waypoints: &DMatrix<f64>,
        cfg: SmoothingConfig,
    ) -> Result<Self, PathError> {
        Self::from_waypoints_fitting_view(waypoints.as_view(), cfg)
    }

    /// Construct a tolerance-bounded `C4` quintic approximation from a borrowed
    /// waypoint matrix view.
    ///
    /// This accepts nalgebra views such as `waypoints.as_view()` and compatible
    /// strided column-major views. Construction reads the view and stores an
    /// owned fitted representation in the returned [`Path`], which does not
    /// borrow the input afterwards. See [`Path::from_waypoints_fitting`] for the
    /// complete fitting semantics, configuration, errors, and limitations.
    ///
    /// # API stability
    ///
    /// This view constructor has the same unstable API status as
    /// [`Path::from_waypoints_fitting`].
    pub fn from_waypoints_fitting_view(
        waypoints: DMatrixView<'_, f64>,
        cfg: SmoothingConfig,
    ) -> Result<Self, PathError> {
        let dim = waypoints.nrows();
        let spline = SmoothedSpline::build(waypoints, &cfg)?;
        Ok(Self {
            dim,
            s_min: spline.start,
            s_max: spline.end,
            out_of_range_mode: cfg.out_of_range_mode,
            repr: PathRepr::Smoothed(Box::new(spline)),
        })
    }

    /// Return construction diagnostics for a tolerance-fitted waypoint path.
    ///
    /// # Returns
    ///
    /// - `Some(report)` for a path successfully constructed by either
    ///   [`Path::from_waypoints_fitting`] or [`Path::from_waypoints_fitting_view`];
    /// - `None` for analytic, strictly interpolated, and user-evaluator paths.
    ///
    /// [`SmoothingReport::max_errors`](crate::path::SmoothingReport::max_errors)
    /// contains Bernstein-derived whole-domain numerical bounds in physical input
    /// units and follows the order of
    /// [`SmoothingReport::axes`](crate::path::SmoothingReport::axes).
    /// `refinements`, `fitting_rows`, and `checked_intervals` are cumulative work
    /// counters; axis and segment fields describe the final representation.
    /// None of these values is an optimality or run-time guarantee.
    pub fn smoothing_report(&self) -> Option<&SmoothingReport> {
        match &self.repr {
            PathRepr::Smoothed(spline) => Some(&spline.report),
            _ => None,
        }
    }

    /// Build a parametric path from an analytic closure.
    ///
    /// Derivatives up to third order are computed automatically via [`Jet3`](crate::path::Jet3)
    /// forward-mode AD.  The closure only needs to express `q(s)` symbolically;
    /// no manual differentiation is required.
    ///
    /// # Arguments
    /// - `q_fn`  : closure mapping scalar `s` to a `dim`-dimensional position vector
    /// - `s_min` : lower bound of the path parameter
    /// - `s_max` : upper bound of the path parameter (`s_max > s_min` required)
    ///
    /// # Errors
    /// - [`PathError::InvalidRange`](crate::diag::PathError::InvalidRange)     : `s_min >= s_max` or either value is non-finite
    /// - [`PathError::InvalidDimension`](crate::diag::PathError::InvalidDimension) : closure returned an empty vector
    ///
    /// # Example
    /// The example below builds a two-dimensional analytic path and evaluates
    /// derivatives produced by automatic differentiation.
    ///
    /// ```rust
    /// # fn main() -> Result<(), copp::diag::CoppError> {
    /// use copp::path::autodiff::Jet3;
    /// use copp::path::{cos, sin, Path};
    ///
    /// let path = Path::from_parametric(|s: Jet3| vec![sin(s), cos(s)], 0.0, 1.0)?;
    ///
    /// let s = [0.0, 0.25, 0.5, 0.75, 1.0];
    /// let out = path.evaluate_up_to_3rd(s.as_slice())?;
    /// assert_eq!(out.q.shape(), (2, 5));
    /// assert!(out.dddq.is_some());
    /// # Ok(())
    /// # }
    /// ```
    pub fn from_parametric<F>(q_fn: F, s_min: f64, s_max: f64) -> Result<Self, PathError>
    where
        F: Fn(Jet3) -> Vec<Jet3> + Send + Sync + 'static,
    {
        validate_range(s_min, s_max)?;

        let sample = q_fn(Jet3::constant((s_min + s_max) * 0.5));
        if sample.is_empty() {
            return Err(PathError::InvalidDimension { dim: 0 });
        }
        let dim = sample.len();

        Ok(Self {
            dim,
            s_min,
            s_max,
            out_of_range_mode: OutOfRangeMode::Error,
            repr: PathRepr::Parametric(Arc::new(q_fn)),
        })
    }

    /// Build a path from an evaluator that provides explicit derivatives up to second order.
    ///
    /// Use this constructor when derivatives are already available from an
    /// external source and automatic differentiation is not desired. The
    /// evaluator is owned by the returned [`Path`] through an internal
    /// [`Arc`], so the path can be passed around without borrowing the original
    /// value.
    ///
    /// # Arguments
    /// - `evaluator`: object that writes column-major derivative buffers
    /// - `s_min`: lower bound of the path parameter
    /// - `s_max`: upper bound of the path parameter (`s_max > s_min` required)
    ///
    /// # Errors
    /// - [`PathError::InvalidRange`](crate::diag::PathError::InvalidRange): `s_min >= s_max` or either value is non-finite
    /// - [`PathError::InvalidDimension`](crate::diag::PathError::InvalidDimension): evaluator dimension is zero
    ///
    /// # Example
    /// The example below wraps a two-dimensional external evaluator that writes
    /// explicit `q/dq/ddq` buffers in column-major order.
    ///
    /// ```rust
    /// # fn main() -> Result<(), copp::diag::CoppError> {
    /// use copp::diag::PathError;
    /// use copp::path::{Path, PathEvaluator2nd};
    ///
    /// struct NormalizedEvaluator2nd;
    ///
    /// impl PathEvaluator2nd for NormalizedEvaluator2nd {
    ///     fn dim(&self) -> usize {
    ///         2
    ///     }
    ///
    ///     fn evaluate_up_to_2nd(
    ///         &self,
    ///         s: &[f64],
    ///         q: &mut [f64],
    ///         dq: &mut [f64],
    ///         ddq: &mut [f64],
    ///     ) -> Result<(), PathError> {
    ///         for (col, &sj) in s.iter().enumerate() {
    ///             let row0 = 2 * col;
    ///             q[row0] = 0.5 * sj * sj;
    ///             q[row0 + 1] = sj;
    ///             dq[row0] = sj;
    ///             dq[row0 + 1] = 1.0;
    ///             ddq[row0] = 1.0;
    ///             ddq[row0 + 1] = 0.0;
    ///         }
    ///         Ok(())
    ///     }
    /// }
    ///
    /// let path = Path::from_evaluator_2nd(NormalizedEvaluator2nd, 0.0, 1.0)?;
    /// let out = path.evaluate_up_to_2nd(&[0.0, 0.5])?;
    /// assert_eq!(out.q.shape(), (2, 2));
    /// assert_eq!(out.q[(0, 1)], 0.125);
    /// assert!(out.dddq.is_none());
    /// # Ok(())
    /// # }
    /// ```
    pub fn from_evaluator_2nd<E>(evaluator: E, s_min: f64, s_max: f64) -> Result<Self, PathError>
    where
        E: PathEvaluator2nd + 'static,
    {
        Self::from_shared_evaluator_2nd(Arc::new(evaluator), s_min, s_max)
    }

    /// Build a path from a shared explicit-derivative evaluator up to second order.
    ///
    /// This is the same representation as [`Path::from_evaluator_2nd`], but accepts
    /// an already shared evaluator. It is useful when multiple paths or
    /// application components need to hold the same evaluator object.
    ///
    /// # Errors
    /// - [`PathError::InvalidRange`](crate::diag::PathError::InvalidRange): `s_min >= s_max` or either value is non-finite
    /// - [`PathError::InvalidDimension`](crate::diag::PathError::InvalidDimension): evaluator dimension is zero
    pub fn from_shared_evaluator_2nd(
        evaluator: Arc<dyn PathEvaluator2nd>,
        s_min: f64,
        s_max: f64,
    ) -> Result<Self, PathError> {
        validate_range(s_min, s_max)?;

        let dim = evaluator.dim();
        if dim == 0 {
            return Err(PathError::InvalidDimension { dim });
        }

        Ok(Self {
            dim,
            s_min,
            s_max,
            out_of_range_mode: OutOfRangeMode::Error,
            repr: PathRepr::Evaluator2nd(evaluator),
        })
    }

    /// Build a path from an evaluator that provides explicit derivatives up to third order.
    ///
    /// This is the constructor to use when the path will be evaluated by
    /// third-order APIs such as TOPP3/COPP3 sampling.  For TOPP2/COPP2-only
    /// usage, [`Path::from_evaluator_2nd`] avoids requiring a third derivative
    /// implementation.
    ///
    /// # Errors
    /// - [`PathError::InvalidRange`](crate::diag::PathError::InvalidRange): `s_min >= s_max` or either value is non-finite
    /// - [`PathError::InvalidDimension`](crate::diag::PathError::InvalidDimension): evaluator dimension is zero
    ///
    /// # Example
    /// The example below extends a two-dimensional explicit evaluator to third
    /// order by writing jerk samples into the `dddq` buffer.
    ///
    /// ```rust
    /// # fn main() -> Result<(), copp::diag::CoppError> {
    /// use copp::diag::PathError;
    /// use copp::path::{Path, PathEvaluator2nd, PathEvaluator3rd};
    ///
    /// struct NormalizedEvaluator3rd;
    ///
    /// impl PathEvaluator2nd for NormalizedEvaluator3rd {
    ///     fn dim(&self) -> usize {
    ///         2
    ///     }
    ///
    ///     fn evaluate_up_to_2nd(
    ///         &self,
    ///         s: &[f64],
    ///         q: &mut [f64],
    ///         dq: &mut [f64],
    ///         ddq: &mut [f64],
    ///     ) -> Result<(), PathError> {
    ///         for (col, &sj) in s.iter().enumerate() {
    ///             let row0 = 2 * col;
    ///             q[row0] = 0.5 * sj * sj;
    ///             q[row0 + 1] = sj;
    ///             dq[row0] = sj;
    ///             dq[row0 + 1] = 1.0;
    ///             ddq[row0] = 1.0;
    ///             ddq[row0 + 1] = 0.0;
    ///         }
    ///         Ok(())
    ///     }
    /// }
    ///
    /// impl PathEvaluator3rd for NormalizedEvaluator3rd {
    ///     fn evaluate_up_to_3rd(
    ///         &self,
    ///         s: &[f64],
    ///         q: &mut [f64],
    ///         dq: &mut [f64],
    ///         ddq: &mut [f64],
    ///         dddq: &mut [f64],
    ///     ) -> Result<(), PathError> {
    ///         self.evaluate_up_to_2nd(s, q, dq, ddq)?;
    ///         dddq.fill(0.0);
    ///         Ok(())
    ///     }
    /// }
    ///
    /// let path = Path::from_evaluator_3rd(NormalizedEvaluator3rd, 0.0, 1.0)?;
    /// let out = path.evaluate_up_to_3rd(&[0.0, 0.5])?;
    /// assert_eq!(out.q.shape(), (2, 2));
    /// assert_eq!(out.q[(0, 1)], 0.125);
    /// assert!(out.dddq.is_some());
    /// # Ok(())
    /// # }
    /// ```
    pub fn from_evaluator_3rd<E>(evaluator: E, s_min: f64, s_max: f64) -> Result<Self, PathError>
    where
        E: PathEvaluator3rd + 'static,
    {
        Self::from_shared_evaluator_3rd(Arc::new(evaluator), s_min, s_max)
    }

    /// Build a path from a shared explicit-derivative evaluator up to third order.
    ///
    /// # Errors
    /// - [`PathError::InvalidRange`](crate::diag::PathError::InvalidRange): `s_min >= s_max` or either value is non-finite
    /// - [`PathError::InvalidDimension`](crate::diag::PathError::InvalidDimension): evaluator dimension is zero
    pub fn from_shared_evaluator_3rd(
        evaluator: Arc<dyn PathEvaluator3rd>,
        s_min: f64,
        s_max: f64,
    ) -> Result<Self, PathError> {
        validate_range(s_min, s_max)?;

        let dim = evaluator.dim();
        if dim == 0 {
            return Err(PathError::InvalidDimension { dim });
        }

        Ok(Self {
            dim,
            s_min,
            s_max,
            out_of_range_mode: OutOfRangeMode::Error,
            repr: PathRepr::Evaluator3rd(evaluator),
        })
    }

    /// Build a path from a third-order explicit-derivative evaluator.
    ///
    /// This compatibility constructor is equivalent to
    /// [`Path::from_evaluator_3rd`]. New code should prefer
    /// [`Path::from_evaluator_2nd`] or [`Path::from_evaluator_3rd`] to make the
    /// supported derivative order explicit.
    pub fn from_evaluator<E>(evaluator: E, s_min: f64, s_max: f64) -> Result<Self, PathError>
    where
        E: PathEvaluator3rd + 'static,
    {
        Self::from_evaluator_3rd(evaluator, s_min, s_max)
    }

    /// Build a path from a shared third-order explicit-derivative evaluator.
    ///
    /// This compatibility constructor is equivalent to
    /// [`Path::from_shared_evaluator_3rd`].
    pub fn from_shared_evaluator(
        evaluator: Arc<dyn PathEvaluator3rd>,
        s_min: f64,
        s_max: f64,
    ) -> Result<Self, PathError> {
        Self::from_shared_evaluator_3rd(evaluator, s_min, s_max)
    }

    /// Build a spline path that interpolates every waypoint in a matrix.
    ///
    /// Every input column is a hard positional interpolation condition: at that
    /// waypoint's assigned parameter, the returned path passes through the
    /// column up to floating-point roundoff. This differs from
    /// [`Path::from_waypoints_fitting`], which may deviate from interior
    /// waypoints within configured tolerances.
    ///
    /// # API stability
    ///
    /// This constructor is currently **unstable**. Its name, signature,
    /// configuration, and algorithm-selection interface may change as additional
    /// waypoint-interpolation methods are introduced.
    ///
    /// Internally, the current algorithm solves the Hermite spline system with an O(N) block-Thomas
    /// algorithm; all dimensions are solved in parallel.
    /// The default configuration ([`SplineConfig::default`](crate::path::SplineConfig::default)) uses a quintic
    /// (order-5) spline with `s in [0, 1]`.
    ///
    /// # Arguments
    /// - `waypoints` : matrix of shape `(dim, n_points)`; each column is one waypoint
    /// - `cfg`       : spline configuration (order, parameter range, boundary derivatives, out-of-range mode)
    ///
    /// # Errors
    /// - [`PathError::InvalidDimension`](crate::diag::PathError::InvalidDimension)   : `waypoints` has zero rows
    /// - [`PathError::NotEnoughWaypoints`](crate::diag::PathError::NotEnoughWaypoints) : fewer than 2 columns
    /// - [`PathError::InvalidOrder`](crate::diag::PathError::InvalidOrder)       : `order < 3`
    /// - [`PathError::InvalidRange`](crate::diag::PathError::InvalidRange)       : invalid parameter range
    /// - [`PathError::SingularSystem`](crate::diag::PathError::SingularSystem)     : spline system is singular (extremely rare)
    ///
    /// # Example
    /// The example below builds a spline through two-dimensional waypoints and
    /// evaluates position samples.
    ///
    /// ```rust
    /// # fn main() -> Result<(), copp::diag::CoppError> {
    /// use copp::path::{Path, SplineConfig};
    /// use nalgebra::DMatrix;
    ///
    /// let waypoints = DMatrix::from_row_slice(
    ///     2,
    ///     5,
    ///     &[
    ///         0.0, 0.25, 0.5, 0.75, 1.0,
    ///         0.0, 0.1, -0.1, 0.2, 0.0,
    ///     ],
    /// );
    /// let path = Path::from_waypoints_interpolating(&waypoints, SplineConfig::default())?;
    ///
    /// let s = [0.0, 0.5, 1.0];
    /// let out = path.evaluate_q(s.as_slice())?;
    /// assert_eq!(out.q.shape(), (2, 3));
    /// assert!(out.dq.is_none());
    /// # Ok(())
    /// # }
    /// ```
    pub fn from_waypoints_interpolating(
        waypoints: &DMatrix<f64>,
        cfg: SplineConfig,
    ) -> Result<Self, PathError> {
        Self::from_waypoints_interpolating_view(waypoints.as_view(), cfg)
    }

    /// Build a waypoint-interpolating spline from a borrowed matrix view.
    ///
    /// This accepts nalgebra views such as `waypoints.as_view()` and compatible
    /// strided column-major views. See [`Path::from_waypoints_interpolating`] for
    /// the full interpolation semantics, configuration, error conditions, and
    /// unstable API status.
    pub fn from_waypoints_interpolating_view(
        waypoints: DMatrixView<'_, f64>,
        cfg: SplineConfig,
    ) -> Result<Self, PathError> {
        if waypoints.nrows() == 0 {
            return Err(PathError::InvalidDimension {
                dim: waypoints.nrows(),
            });
        }
        if waypoints.ncols() < 2 {
            return Err(PathError::NotEnoughWaypoints {
                n: waypoints.ncols(),
            });
        }
        if cfg.order < 3 {
            return Err(PathError::InvalidOrder { order: cfg.order });
        }
        validate_range(cfg.s_min, cfg.s_max)?;

        let spline = SplinePath::from_waypoints_view(waypoints, &cfg)?;

        Ok(Self {
            dim: waypoints.nrows(),
            s_min: spline.s_min,
            s_max: spline.s_max,
            out_of_range_mode: spline.out_of_range_mode,
            repr: PathRepr::Spline(spline),
        })
    }

    /// Deprecated compatibility alias for [`Path::from_waypoints_interpolating`].
    #[doc(hidden)]
    #[deprecated(since = "0.2.3", note = "use Path::from_waypoints_interpolating")]
    pub fn from_waypoints(waypoints: &DMatrix<f64>, cfg: SplineConfig) -> Result<Self, PathError> {
        Self::from_waypoints_interpolating(waypoints, cfg)
    }

    /// Deprecated compatibility alias for [`Path::from_waypoints_interpolating_view`].
    #[doc(hidden)]
    #[deprecated(since = "0.2.3", note = "use Path::from_waypoints_interpolating_view")]
    pub fn from_waypoints_view(
        waypoints: DMatrixView<'_, f64>,
        cfg: SplineConfig,
    ) -> Result<Self, PathError> {
        Self::from_waypoints_interpolating_view(waypoints, cfg)
    }

    /// Returns the spatial dimension (number of joints) of the path.
    #[inline(always)]
    pub fn dim(&self) -> usize {
        self.dim
    }

    /// Returns the valid parameter range `(s_min, s_max)`.
    #[inline(always)]
    pub fn s_range(&self) -> (f64, f64) {
        (self.s_min, self.s_max)
    }

    /// Evaluate position `q` only at the query points (cheapest; no derivatives).
    ///
    /// # Arguments
    /// - `s` : one-dimensional parameter samples (length `N`)
    ///
    /// # Returns
    /// [`PathDerivatives`](crate::path::PathDerivatives) with `dq / ddq / dddq` all `None`;
    /// `q` has shape `(dim, N)`.
    ///
    /// # Errors
    /// - [`PathError::OutOfRangeS`] : a query value is out of range (only in `Error` mode)
    pub fn evaluate_q(&self, s: &[f64]) -> Result<PathDerivatives, PathError> {
        self.evaluate_impl(s, Order::Zero)
    }

    /// Evaluate position, velocity, and acceleration (`q`, `dq`, `ddq`); jerk is not computed.
    ///
    /// # Arguments
    /// - `s` : one-dimensional parameter samples (length `N`)
    ///
    /// # Returns
    /// [`PathDerivatives`](crate::path::PathDerivatives) with `dddq = None`;
    /// `q / dq / ddq` each have shape `(dim, N)`.
    pub fn evaluate_up_to_2nd(&self, s: &[f64]) -> Result<PathDerivatives, PathError> {
        self.evaluate_impl(s, Order::Two)
    }

    /// Evaluate position and all three derivative orders (`q`, `dq`, `ddq`, `dddq`).
    ///
    /// This is the most expensive evaluation method.  If jerk is not needed,
    /// prefer [`evaluate_up_to_2nd`].
    ///
    /// # Arguments
    /// - `s` : one-dimensional parameter samples (length `N`)
    ///
    /// # Returns
    /// [`PathDerivatives`](crate::path::PathDerivatives) with all four fields populated; each matrix has shape `(dim, N)`.
    ///
    /// # Errors
    /// - [`PathError::OutOfRangeS`] : a query value is out of range
    ///
    /// # Example
    /// The example below evaluates an analytic path up to jerk using automatic
    /// differentiation.
    ///
    /// ```rust, no_run
    /// use copp::path::{Path, sin, cos};
    /// use copp::path::autodiff::Jet3;
    ///
    /// let path = Path::from_parametric(
    ///     |s: Jet3| vec![sin(s), cos(s)],
    ///     0.0, 1.0,
    /// ).unwrap();
    ///
    /// let s = [0.0, 0.25, 0.5, 0.75, 1.0];
    /// let out = path.evaluate_up_to_3rd(&s).unwrap();
    ///
    /// let dq   = out.dq.as_ref().unwrap();
    /// let dddq = out.dddq.as_ref().unwrap();
    /// // dim 0 is sin(s); its first derivative is cos(s)
    /// assert!((dq[(0, 0)] - 1.0_f64.cos()).abs() < 1e-10);
    /// // dim 1 is cos(s); its third derivative is sin(s)
    /// assert!((dddq[(1, 0)] - 0.0_f64.sin()).abs() < 1e-10);
    /// ```
    ///
    /// [`evaluate_up_to_2nd`]: Path::evaluate_up_to_2nd
    pub fn evaluate_up_to_3rd(&self, s: &[f64]) -> Result<PathDerivatives, PathError> {
        self.evaluate_impl(s, Order::Three)
    }

    // ── internal ─────────────────────────────────────────────────────────────

    fn evaluate_impl(&self, s: &[f64], order: Order) -> Result<PathDerivatives, PathError> {
        let n = s.len();
        let dim = self.dim;

        // Allocate output buffers; skip higher-order buffers when not needed.
        let mut q = vec![0.0f64; dim * n];
        let mut dq = (order != Order::Zero).then(|| vec![0.0f64; dim * n]);
        let mut ddq = (order != Order::Zero).then(|| vec![0.0f64; dim * n]);
        let mut dddq = (order == Order::Three).then(|| vec![0.0f64; dim * n]);

        match &self.repr {
            PathRepr::Parametric(eval_fn) => {
                eval_parametric(
                    eval_fn,
                    self,
                    dim,
                    (s, &mut q, &mut dq, &mut ddq, &mut dddq),
                )?;
            }
            PathRepr::Spline(spline) => {
                eval_spline(spline, self, dim, (s, &mut q, &mut dq, &mut ddq, &mut dddq))?;
            }
            PathRepr::Smoothed(spline) => {
                // Nonuniform fitted spans are located per query. Validate or
                // clamp each public parameter first; `SmoothedSpline::evaluate`
                // then normalizes it and applies derivative chain-rule scaling.
                for (j, &value) in s.iter().enumerate() {
                    if !value.is_finite() {
                        return Err(PathError::OutOfRangeS {
                            s_min: self.s_min,
                            s_max: self.s_max,
                            index: j,
                            value,
                        });
                    }
                    let x = self.validate_s(value, j)?;
                    let range = j * dim..(j + 1) * dim;
                    spline.evaluate(
                        x,
                        &mut q[range.clone()],
                        dq.as_mut().map(|v| &mut v[range.clone()]),
                        ddq.as_mut().map(|v| &mut v[range.clone()]),
                        dddq.as_mut().map(|v| &mut v[range]),
                    );
                }
            }
            PathRepr::Evaluator2nd(evaluator) => {
                eval_evaluator_2nd(
                    evaluator.as_ref(),
                    self,
                    dim,
                    (s, &mut q, &mut dq, &mut ddq, &mut dddq),
                )?;
            }
            PathRepr::Evaluator3rd(evaluator) => {
                eval_evaluator_3rd(
                    evaluator.as_ref(),
                    self,
                    dim,
                    (s, &mut q, &mut dq, &mut ddq, &mut dddq),
                )?;
            }
        }

        Ok(PathDerivatives {
            q: DMatrix::from_vec(dim, n, q),
            dq: dq.map(|v| DMatrix::from_vec(dim, n, v)),
            ddq: ddq.map(|v| DMatrix::from_vec(dim, n, v)),
            dddq: dddq.map(|v| DMatrix::from_vec(dim, n, v)),
        })
    }

    #[inline(always)]
    fn validate_s(&self, s: f64, index: usize) -> Result<f64, PathError> {
        match self.out_of_range_mode {
            OutOfRangeMode::Error => {
                if s < self.s_min - EPS_RANGE || s > self.s_max + EPS_RANGE {
                    return Err(PathError::OutOfRangeS {
                        s_min: self.s_min,
                        s_max: self.s_max,
                        index,
                        value: s,
                    });
                }
                Ok(s.clamp(self.s_min, self.s_max))
            }
            OutOfRangeMode::Clamp => Ok(s.clamp(self.s_min, self.s_max)),
        }
    }
}

// ── free evaluation functions ─────────────────────────────────────────────────

/// The input of evaluation functions.
type EvalInput<'a> = (
    &'a [f64],                // s
    &'a mut [f64],            // q
    &'a mut Option<Vec<f64>>, // dq
    &'a mut Option<Vec<f64>>, // ddq
    &'a mut Option<Vec<f64>>, // dddq
);

/// Evaluate a parametric path into pre-allocated column-major buffers.
///
/// Per-column parallelism via Rayon: validate + AD-evaluate + write in one pass.
/// No intermediate `Vec<Vec<Jet3>>` allocation; results go directly into output buffers.
fn eval_parametric(
    eval_fn: &ParametricFn,
    path: &Path,
    dim: usize,
    input_eval: EvalInput,
) -> Result<(), PathError> {
    let (s_values, q, dq, ddq, dddq) = input_eval;
    let n = s_values.len();

    // Chunk each output buffer by `dim` so column j maps to slice [j*dim .. (j+1)*dim].
    // When a derivative level is not requested (`None`), we still need an
    // `IndexedParallelIterator` of the same length for `zip`; a Vec<None> is the
    // simplest way to satisfy Rayon's type constraints here.
    let dq_chunks: Vec<Option<&mut [f64]>> = dq.as_deref_mut().map_or_else(
        || (0..n).map(|_| None).collect(),
        |v| v.chunks_mut(dim).map(Some).collect(),
    );
    let ddq_chunks: Vec<Option<&mut [f64]>> = ddq.as_deref_mut().map_or_else(
        || (0..n).map(|_| None).collect(),
        |v| v.chunks_mut(dim).map(Some).collect(),
    );
    let dddq_chunks: Vec<Option<&mut [f64]>> = dddq.as_deref_mut().map_or_else(
        || (0..n).map(|_| None).collect(),
        |v| v.chunks_mut(dim).map(Some).collect(),
    );

    s_values
        .par_iter()
        .enumerate()
        .zip(q.par_chunks_mut(dim))
        .zip(dq_chunks.into_par_iter())
        .zip(ddq_chunks.into_par_iter())
        .zip(dddq_chunks.into_par_iter())
        .map(
            |(((((j, &s_raw), q_col), mut dq_col), mut ddq_col), mut dddq_col)| {
                let s_curr = path.validate_s(s_raw, j)?;
                let vals = eval_fn(Jet3::seed(s_curr));
                if vals.len() != dim {
                    return Err(PathError::DimensionMismatch);
                }
                for (i, jet) in vals.iter().enumerate() {
                    q_col[i] = jet.v;
                    if let Some(ref mut b) = dq_col {
                        b[i] = jet.d1;
                    }
                    if let Some(ref mut b) = ddq_col {
                        b[i] = jet.d2;
                    }
                    if let Some(ref mut b) = dddq_col {
                        b[i] = jet.d3;
                    }
                }
                Ok(())
            },
        )
        .collect()
}

/// Evaluate spline representation into pre-allocated column-major buffers.
///
/// Dispatches to [`SplinePath::eval_at`](crate::path::spline::SplinePath::eval_at)`::<ORDER>` with the minimum derivative
/// order that satisfies the request: zero run-time branching per sample:
///   - [`Order::Zero`](Order::Zero) => `eval_at::<0>` (q only)
///   - [`Order::Two`](Order::Two) => `eval_at::<2>` (q, dq, ddq)
///   - [`Order::Three`](Order::Three) => `eval_at::<3>` (q, dq, ddq, dddq)
fn eval_spline(
    spline: &SplinePath,
    path: &Path,
    dim: usize,
    input_eval: EvalInput,
) -> Result<(), PathError> {
    let (s_values, q, dq, ddq, dddq) = input_eval;
    match (dq.as_deref_mut(), ddq.as_deref_mut(), dddq.as_deref_mut()) {
        // ── Order::Zero: q only ───────────────────────────────────────────
        (None, None, None) => s_values
            .par_iter()
            .enumerate()
            .zip(q.par_chunks_mut(dim))
            .map(|((j, &s_raw), q_col)| -> Result<(), PathError> {
                let s_curr = path.validate_s(s_raw, j)?;
                // eval_at::<0> only writes q_col; the derivative slices are never
                // accessed, so zero-length arrays satisfy the borrow checker.
                let (mut no_dq, mut no_ddq, mut no_dddq): ([f64; 0], [f64; 0], [f64; 0]) =
                    ([], [], []);
                spline.eval_at::<0>(s_curr, dim, q_col, &mut no_dq, &mut no_ddq, &mut no_dddq);
                Ok(())
            })
            .collect(),
        // ── Order::Two: q, dq, ddq ────────────────────────────────────────
        (Some(dq_buf), Some(ddq_buf), None) => {
            let dq_chunks: Vec<&mut [f64]> = dq_buf.chunks_mut(dim).collect();
            let ddq_chunks: Vec<&mut [f64]> = ddq_buf.chunks_mut(dim).collect();
            s_values
                .par_iter()
                .enumerate()
                .zip(q.par_chunks_mut(dim))
                .zip(dq_chunks.into_par_iter())
                .zip(ddq_chunks.into_par_iter())
                .map(
                    |((((j, &s_raw), q_col), dq_col), ddq_col)| -> Result<(), PathError> {
                        let s_curr = path.validate_s(s_raw, j)?;
                        let mut s4 = [];
                        spline.eval_at::<2>(s_curr, dim, q_col, dq_col, ddq_col, &mut s4);
                        Ok(())
                    },
                )
                .collect()
        }
        // ── Order::Three: q, dq, ddq, dddq ───────────────────────────────
        (Some(dq_buf), Some(ddq_buf), Some(dddq_buf)) => {
            let dq_chunks: Vec<&mut [f64]> = dq_buf.chunks_mut(dim).collect();
            let ddq_chunks: Vec<&mut [f64]> = ddq_buf.chunks_mut(dim).collect();
            let dddq_chunks: Vec<&mut [f64]> = dddq_buf.chunks_mut(dim).collect();
            s_values
                .par_iter()
                .enumerate()
                .zip(q.par_chunks_mut(dim))
                .zip(dq_chunks.into_par_iter())
                .zip(ddq_chunks.into_par_iter())
                .zip(dddq_chunks.into_par_iter())
                .map(
                    |(((((j, &s_raw), q_col), dq_col), ddq_col), dddq_col)| -> Result<(), PathError> {
                        let s_curr = path.validate_s(s_raw, j)?;
                        spline.eval_at::<3>(s_curr, dim, q_col, dq_col, ddq_col, dddq_col);
                        Ok(())
                    },
                )
                .collect()
        }
        // Unreachable: evaluate_impl only produces the three patterns above.
        _ => unreachable!("unexpected dq/ddq/dddq combination"),
    }
}

// ── helpers ───────────────────────────────────────────────────────────────────

/// Evaluate a user-provided second-order explicit-derivative evaluator.
///
/// The evaluator receives already validated/clamped path parameters. It is
/// called once for the whole batch so custom evaluators can use their own
/// vectorized implementation.
fn eval_evaluator_2nd(
    evaluator: &dyn PathEvaluator2nd,
    path: &Path,
    dim: usize,
    input_eval: EvalInput,
) -> Result<(), PathError> {
    let (s_values, q, dq, ddq, dddq) = input_eval;
    let s_valid = s_values
        .iter()
        .enumerate()
        .map(|(j, &s)| path.validate_s(s, j))
        .collect::<Result<Vec<_>, _>>()?;

    if evaluator.dim() != dim {
        return Err(PathError::DimensionMismatch);
    }

    match (dq.as_deref_mut(), ddq.as_deref_mut(), dddq.as_deref_mut()) {
        (None, None, None) => evaluator.evaluate_q(&s_valid, q),
        (Some(dq_buf), Some(ddq_buf), None) => {
            evaluator.evaluate_up_to_2nd(&s_valid, q, dq_buf, ddq_buf)
        }
        (Some(_), Some(_), Some(_)) => Err(PathError::UnsupportedDerivativeOrder {
            requested: 3,
            available: 2,
        }),
        // Unreachable: evaluate_impl only produces the three patterns above.
        _ => unreachable!("unexpected dq/ddq/dddq combination"),
    }
}

/// Evaluate a user-provided third-order explicit-derivative evaluator.
///
/// The evaluator receives already validated/clamped path parameters. It is
/// called once for the whole batch so custom evaluators can use their own
/// vectorized implementation.
fn eval_evaluator_3rd(
    evaluator: &dyn PathEvaluator3rd,
    path: &Path,
    dim: usize,
    input_eval: EvalInput,
) -> Result<(), PathError> {
    let (s_values, q, dq, ddq, dddq) = input_eval;
    let s_valid = s_values
        .iter()
        .enumerate()
        .map(|(j, &s)| path.validate_s(s, j))
        .collect::<Result<Vec<_>, _>>()?;

    if evaluator.dim() != dim {
        return Err(PathError::DimensionMismatch);
    }

    match (dq.as_deref_mut(), ddq.as_deref_mut(), dddq.as_deref_mut()) {
        (None, None, None) => evaluator.evaluate_q(&s_valid, q),
        (Some(dq_buf), Some(ddq_buf), None) => {
            evaluator.evaluate_up_to_2nd(&s_valid, q, dq_buf, ddq_buf)
        }
        (Some(dq_buf), Some(ddq_buf), Some(dddq_buf)) => {
            evaluator.evaluate_up_to_3rd(&s_valid, q, dq_buf, ddq_buf, dddq_buf)
        }
        // Unreachable: evaluate_impl only produces the three patterns above.
        _ => unreachable!("unexpected dq/ddq/dddq combination"),
    }
}

fn validate_range(s_min: f64, s_max: f64) -> Result<(), PathError> {
    if !s_min.is_finite() || !s_max.is_finite() || s_max <= s_min {
        return Err(PathError::InvalidRange { s_min, s_max });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::PathDerivatives;
    use crate::path::{
        Jet3, Path as PathModel, PathError, PathEvaluator2nd, PathEvaluator3rd, SplineConfig, cos,
        exp, sin,
    };
    use nalgebra::{Const, DMatrix, DMatrixView, Dyn};
    use plotters::prelude::*;
    use rand::RngExt;
    use std::error::Error;
    use std::fs::create_dir_all;
    use std::hint::black_box;
    use std::path::Path as StdPath;
    use std::time::Instant;

    const DIM: usize = 6;

    fn make_s(n: usize) -> DMatrix<f64> {
        DMatrix::<f64>::from_fn(1, n, |_, j| j as f64 / (n - 1) as f64)
    }

    fn make_parametric_path() -> Result<PathModel, PathError> {
        PathModel::from_parametric(
            |s: Jet3| {
                vec![
                    sin(s),
                    cos(s),
                    exp(0.3 * s) - 1.0,
                    s + 0.1 * s * s - 0.01 * s * s * s * s,
                    sin(2.0 * s) + 0.15 * cos(3.0 * s),
                    sin(s) * cos(s),
                ]
            },
            0.0,
            1.0,
        )
    }

    struct PolynomialEvaluator;
    struct QuadraticEvaluator2nd;

    impl PathEvaluator2nd for PolynomialEvaluator {
        fn dim(&self) -> usize {
            2
        }

        fn evaluate_up_to_2nd(
            &self,
            s: &[f64],
            q: &mut [f64],
            dq: &mut [f64],
            ddq: &mut [f64],
        ) -> Result<(), PathError> {
            if q.len() != 2 * s.len() || dq.len() != q.len() || ddq.len() != q.len() {
                return Err(PathError::DimensionMismatch);
            }

            for (j, &x) in s.iter().enumerate() {
                let col = 2 * j;
                q[col] = x * x * x;
                dq[col] = 3.0 * x * x;
                ddq[col] = 6.0 * x;

                q[col + 1] = x * x + 1.0;
                dq[col + 1] = 2.0 * x;
                ddq[col + 1] = 2.0;
            }
            Ok(())
        }
    }

    impl PathEvaluator3rd for PolynomialEvaluator {
        fn evaluate_up_to_3rd(
            &self,
            s: &[f64],
            q: &mut [f64],
            dq: &mut [f64],
            ddq: &mut [f64],
            dddq: &mut [f64],
        ) -> Result<(), PathError> {
            self.evaluate_up_to_2nd(s, q, dq, ddq)?;
            if dddq.len() != 2 * s.len() {
                return Err(PathError::DimensionMismatch);
            }

            for j in 0..s.len() {
                let col = 2 * j;
                dddq[col] = 6.0;
                dddq[col + 1] = 0.0;
            }
            Ok(())
        }
    }

    impl PathEvaluator2nd for QuadraticEvaluator2nd {
        fn dim(&self) -> usize {
            1
        }

        fn evaluate_up_to_2nd(
            &self,
            s: &[f64],
            q: &mut [f64],
            dq: &mut [f64],
            ddq: &mut [f64],
        ) -> Result<(), PathError> {
            if q.len() != s.len() || dq.len() != q.len() || ddq.len() != q.len() {
                return Err(PathError::DimensionMismatch);
            }

            for (j, &x) in s.iter().enumerate() {
                q[j] = x * x + 1.0;
                dq[j] = 2.0 * x;
                ddq[j] = 2.0;
            }
            Ok(())
        }
    }

    fn make_waypoints(n_pts: usize) -> DMatrix<f64> {
        let mut rng = rand::rng();
        // Random-walk waypoints: each row is one DOF, each column is a waypoint.
        let mut waypoints = DMatrix::<f64>::zeros(DIM, n_pts);
        for mut row in waypoints.row_iter_mut() {
            row[0] = rng.random_range(-1.0..1.0);
            for j in 1..n_pts {
                let step = rng.random_range(-0.35..0.35);
                row[j] = row[j - 1] + step;
            }
        }
        waypoints
    }

    #[test]
    fn test_waypoints_view_interpolates_padded_column_major() -> Result<(), PathError> {
        const DIM_LOCAL: usize = 2;
        const N_PTS: usize = 4;
        const LEADING_DIM: usize = 3;
        let data = [
            0.0, 1.0, -99.0, //
            0.5, 1.5, -99.0, //
            1.0, 2.0, -99.0, //
            1.5, 2.5, -99.0,
        ];
        let waypoints = DMatrixView::from_slice_with_strides_generic(
            &data,
            Dyn(DIM_LOCAL),
            Dyn(N_PTS),
            Const::<1>,
            Dyn(LEADING_DIM),
        );

        let path =
            PathModel::from_waypoints_interpolating_view(waypoints, SplineConfig::default())?;
        let s = [0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0];
        let out = path.evaluate_q(&s)?;

        for j in 0..N_PTS {
            assert!((out.q[(0, j)] - data[j * LEADING_DIM]).abs() < 1e-10);
            assert!((out.q[(1, j)] - data[j * LEADING_DIM + 1]).abs() < 1e-10);
        }

        Ok(())
    }

    #[test]
    fn test_evaluator_path_explicit_derivatives() -> Result<(), PathError> {
        let path = PathModel::from_evaluator_3rd(PolynomialEvaluator, -1.0, 1.0)?;
        let s = [-1.0, 0.0, 0.5];

        let out = path.evaluate_up_to_3rd(&s)?;
        let dq = out.dq.as_ref().unwrap();
        let ddq = out.ddq.as_ref().unwrap();
        let dddq = out.dddq.as_ref().unwrap();

        for (j, &x) in s.iter().enumerate() {
            assert!((out.q[(0, j)] - x.powi(3)).abs() < 1e-12);
            assert!((dq[(0, j)] - 3.0 * x * x).abs() < 1e-12);
            assert!((ddq[(0, j)] - 6.0 * x).abs() < 1e-12);
            assert!((dddq[(0, j)] - 6.0).abs() < 1e-12);

            assert!((out.q[(1, j)] - (x * x + 1.0)).abs() < 1e-12);
            assert!((dq[(1, j)] - 2.0 * x).abs() < 1e-12);
            assert!((ddq[(1, j)] - 2.0).abs() < 1e-12);
            assert!(dddq[(1, j)].abs() < 1e-12);
        }

        let q_only = path.evaluate_q(&s)?;
        assert!(q_only.dq.is_none());
        assert!(q_only.ddq.is_none());
        assert!(q_only.dddq.is_none());
        assert!((q_only.q[(0, 2)] - 0.125).abs() < 1e-12);

        Ok(())
    }

    #[test]
    fn test_evaluator_path_2nd_does_not_require_3rd() -> Result<(), PathError> {
        let path = PathModel::from_evaluator_2nd(QuadraticEvaluator2nd, -1.0, 1.0)?;
        let s = [-1.0, 0.0, 0.5];

        let out = path.evaluate_up_to_2nd(&s)?;
        let dq = out.dq.as_ref().unwrap();
        let ddq = out.ddq.as_ref().unwrap();

        assert!(out.dddq.is_none());
        assert!((out.q[(0, 2)] - 1.25).abs() < 1e-12);
        assert!((dq[(0, 2)] - 1.0).abs() < 1e-12);
        assert!((ddq[(0, 2)] - 2.0).abs() < 1e-12);

        let err = path.evaluate_up_to_3rd(&s).unwrap_err();
        match err {
            PathError::UnsupportedDerivativeOrder {
                requested: 3,
                available: 2,
            } => {}
            other => panic!("unexpected error: {other}"),
        }

        Ok(())
    }

    #[test]
    fn test_parametric_autodiff_dim6() -> Result<(), PathError> {
        let path = make_parametric_path()?;

        let n = 300;
        let s = make_s(n);
        let out = path.evaluate_up_to_3rd(s.as_slice())?;
        let dq = out.dq.as_ref().unwrap();
        let ddq = out.ddq.as_ref().unwrap();
        let dddq = out.dddq.as_ref().unwrap();

        // Build expected values for all query points and check all 4 derivative orders.
        s.as_slice().iter().enumerate().for_each(|(j, &x)| {
            let e03x = (0.3 * x).exp();
            let expected_q = [
                x.sin(),
                x.cos(),
                e03x - 1.0,
                x + 0.1 * x * x - 0.01 * x * x * x * x,
                (2.0 * x).sin() + 0.15 * (3.0 * x).cos(),
                x.sin() * x.cos(),
            ];
            let expected_dq = [
                x.cos(),
                -x.sin(),
                0.3 * e03x,
                1.0 + 0.2 * x - 0.04 * x * x * x,
                2.0 * (2.0 * x).cos() - 0.45 * (3.0 * x).sin(),
                (2.0 * x).cos(),
            ];
            let expected_ddq = [
                -x.sin(),
                -x.cos(),
                0.09 * e03x,
                0.2 - 0.12 * x * x,
                -4.0 * (2.0 * x).sin() - 1.35 * (3.0 * x).cos(),
                -2.0 * (2.0 * x).sin(),
            ];
            let expected_dddq = [
                -x.cos(),
                x.sin(),
                0.027 * e03x,
                -0.24 * x,
                -8.0 * (2.0 * x).cos() + 4.05 * (3.0 * x).sin(),
                -4.0 * (2.0 * x).cos(),
            ];

            for i in 0..DIM {
                assert!(
                    (out.q[(i, j)] - expected_q[i]).abs() < 1e-10,
                    "q    dim={i} idx={j}"
                );
                assert!(
                    (dq[(i, j)] - expected_dq[i]).abs() < 1e-10,
                    "dq   dim={i} idx={j}"
                );
                assert!(
                    (ddq[(i, j)] - expected_ddq[i]).abs() < 1e-10,
                    "ddq  dim={i} idx={j}"
                );
                assert!(
                    (dddq[(i, j)] - expected_dddq[i]).abs() < 1e-10,
                    "dddq dim={i} idx={j}"
                );
            }
        });

        Ok(())
    }

    #[test]
    fn test_evaluate_q_only() -> Result<(), PathError> {
        let path = make_parametric_path()?;
        let n = 100;
        let s = make_s(n);
        let out = path.evaluate_q(s.as_slice())?;

        assert!(out.dq.is_none());
        assert!(out.ddq.is_none());
        assert!(out.dddq.is_none());

        for j in 0..n {
            let x = s[(0, j)];
            assert!((out.q[(0, j)] - x.sin()).abs() < 1e-10);
            assert!((out.q[(1, j)] - x.cos()).abs() < 1e-10);
        }

        Ok(())
    }

    #[test]
    fn test_evaluate_up_to_2nd() -> Result<(), PathError> {
        let path = make_parametric_path()?;
        let n = 100;
        let s = make_s(n);
        let out = path.evaluate_up_to_2nd(s.as_slice())?;
        let dq = out.dq.as_ref().unwrap();
        let ddq = out.ddq.as_ref().unwrap();

        assert!(out.dddq.is_none());

        for j in 0..n {
            let x = s[(0, j)];
            assert!((out.q[(0, j)] - x.sin()).abs() < 1e-10);
            assert!((dq[(0, j)] - x.cos()).abs() < 1e-10);
            assert!((ddq[(0, j)] - (-x.sin())).abs() < 1e-10);
        }

        Ok(())
    }

    #[test]
    fn test_quintic_spline_interpolates_waypoints_dim6() -> Result<(), PathError> {
        let n_pts = 25;
        let waypoints = make_waypoints(n_pts);

        let cfg = SplineConfig::default();
        let path = PathModel::from_waypoints_interpolating(&waypoints, cfg)?;

        let s = DMatrix::<f64>::from_fn(1, n_pts, |_, j| j as f64 / (n_pts - 1) as f64);
        let out = path.evaluate_up_to_3rd(s.as_slice())?;
        let dq = out.dq.as_ref().unwrap();
        let ddq = out.ddq.as_ref().unwrap();
        let dddq = out.dddq.as_ref().unwrap();

        // The spline must interpolate every waypoint exactly (up to floating-point rounding)
        // and all derivatives must be finite (no blowup).
        for (i, j) in (0..DIM).flat_map(|i| (0..n_pts).map(move |j| (i, j))) {
            assert!((out.q[(i, j)] - waypoints[(i, j)]).abs() < 1e-8);
            assert!(dq[(i, j)].is_finite());
            assert!(ddq[(i, j)].is_finite());
            assert!(dddq[(i, j)].is_finite());
        }

        Ok(())
    }

    #[test]
    fn test_s_out_of_range_error_dim6() -> Result<(), PathError> {
        let waypoints = make_waypoints(12);
        let path = PathModel::from_waypoints_interpolating(&waypoints, SplineConfig::default())?;
        let s = DMatrix::<f64>::from_row_slice(1, 3, &[-0.1, 0.5, 1.1]);
        let err = path.evaluate_up_to_3rd(s.as_slice()).unwrap_err();
        match err {
            PathError::OutOfRangeS { .. } => {}
            _ => panic!("expected OutOfRangeS"),
        }
        Ok(())
    }

    #[test]
    fn test_benchmark_parametric_and_spline_dim6() -> Result<(), PathError> {
        let n_eval = 3000;
        let n_repeat = 8;
        let s = make_s(n_eval);

        let start = Instant::now();
        let param_path = make_parametric_path()?;
        let tc_build_param = start.elapsed().as_secs_f64() * 1e3;

        let start = Instant::now();
        for _ in 0..n_repeat {
            let out = param_path.evaluate_up_to_3rd(s.as_slice())?;
            black_box(out.q[(0, 0)]);
        }
        let tc_eval_param = start.elapsed().as_secs_f64() * 1e3 / n_repeat as f64;
        crate::verbosity_log!(
            crate::diag::Verbosity::Summary,
            "[bench][parametric][dim=6] build={tc_build_param:.3} ms eval={tc_eval_param:.3} ms (N={n_eval})"
        );

        let n_waypoints_list = [16usize, 32, 64, 128, 192, 256, 512, 1024];
        for &n_pts in &n_waypoints_list {
            let waypoints = make_waypoints(n_pts);
            let start = Instant::now();
            let spline_path =
                PathModel::from_waypoints_interpolating(&waypoints, SplineConfig::default())?;
            let tc_build = start.elapsed().as_secs_f64() * 1e3;

            let start = Instant::now();
            for _ in 0..n_repeat {
                let out = spline_path.evaluate_up_to_3rd(s.as_slice())?;
                black_box(out.q[(0, 0)]);
            }
            let tc_eval = start.elapsed().as_secs_f64() * 1e3 / n_repeat as f64;

            crate::verbosity_log!(
                crate::diag::Verbosity::Summary,
                "[bench][spline][dim=6][n_pts={n_pts}] build={tc_build:.3} ms eval={tc_eval:.3} ms"
            );
        }

        Ok(())
    }

    #[test]
    fn test_plot_parametric_and_spline_derivatives() -> Result<(), Box<dyn Error>> {
        let dir = "data/path_plots";
        create_dir_all(dir)?;

        let n = 600;
        let s = make_s(n);
        let s_vec: Vec<f64> = (0..n).map(|j| s[(0, j)]).collect();

        let param_path = make_parametric_path()?;
        let param_out = param_path.evaluate_up_to_3rd(s.as_slice())?;
        plot_grid_4x6(
            &format!("{dir}/parametric_dim6_grid.png"),
            "parametric dim=6",
            &s_vec,
            &param_out,
            None,
        )?;

        let n_pts = 10;
        let waypoints = make_waypoints(n_pts);
        let spline_path =
            PathModel::from_waypoints_interpolating(&waypoints, SplineConfig::default())?;
        let spline_out = spline_path.evaluate_up_to_3rd(s.as_slice())?;
        let wp_s: Vec<f64> = (0..n_pts).map(|j| j as f64 / (n_pts - 1) as f64).collect();
        plot_grid_4x6(
            &format!("{dir}/spline_order5_dim6_grid.png"),
            "spline order=5 dim=6",
            &s_vec,
            &spline_out,
            Some((&wp_s, &waypoints)),
        )?;

        Ok(())
    }

    fn plot_grid_4x6(
        file: &str,
        title: &str,
        s: &[f64],
        data: &PathDerivatives,
        waypoints: Option<(&[f64], &DMatrix<f64>)>,
    ) -> Result<(), Box<dyn Error>> {
        if let Some(parent) = StdPath::new(file).parent() {
            create_dir_all(parent)?;
        }

        let root = BitMapBackend::new(file, (2400, 1400)).into_drawing_area();
        root.fill(&WHITE)?;

        let empty = DMatrix::<f64>::zeros(0, 0);
        let dq = data.dq.as_ref().unwrap_or(&empty);
        let ddq = data.ddq.as_ref().unwrap_or(&empty);
        let dddq = data.dddq.as_ref().unwrap_or(&empty);

        let areas = root.split_evenly((4, DIM));
        let mats = [&data.q, dq, ddq, dddq];
        let row_names = ["q", "dq", "ddq", "dddq"];

        for row in 0..4 {
            for col in 0..DIM {
                let area = &areas[row * DIM + col];
                let series = mat_row(mats[row], col);
                let (mut y_min, mut y_max) = min_max_slice(&series);
                if (y_max - y_min).abs() < 1e-12 {
                    y_min -= 1.0;
                    y_max += 1.0;
                } else {
                    let pad = 0.08 * (y_max - y_min);
                    y_min -= pad;
                    y_max += pad;
                }

                let mut chart = ChartBuilder::on(area)
                    .margin(8)
                    .caption(
                        format!("{} j{}", row_names[row], col + 1),
                        ("sans-serif", 16),
                    )
                    .x_label_area_size(24)
                    .y_label_area_size(38)
                    .build_cartesian_2d(s[0]..s[s.len() - 1], y_min..y_max)?;

                chart
                    .configure_mesh()
                    .x_desc(if row == 3 { "s" } else { "" })
                    .y_desc("")
                    .draw()?;

                chart.draw_series(LineSeries::new(
                    (0..s.len()).map(|j| (s[j], series[j])),
                    &BLUE,
                ))?;

                if row == 0
                    && let Some((s_wp, q_wp)) = waypoints
                {
                    chart.draw_series(
                        s_wp.iter()
                            .zip(q_wp.row(col).iter())
                            .map(|(&xs, &ys)| Circle::new((xs, ys), 2, RED.filled())),
                    )?;
                }
            }
        }

        root.titled(title, ("sans-serif", 28))?;
        root.present()?;
        Ok(())
    }

    fn mat_row(mat: &DMatrix<f64>, row: usize) -> Vec<f64> {
        mat.row(row).iter().copied().collect()
    }

    fn min_max_slice(data: &[f64]) -> (f64, f64) {
        data.iter()
            .copied()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(mn, mx), v| {
                (mn.min(v), mx.max(v))
            })
    }
}
