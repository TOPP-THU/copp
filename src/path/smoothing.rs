//! Tolerance-bounded waypoint fitting with adaptive nonuniform quintic B-splines.
//!
//! Most users enter this module through
//! [`Path::from_waypoints_fitting`](crate::path::Path::from_waypoints_fitting) or
//! its borrowed-view companion
//! [`Path::from_waypoints_fitting_view`](crate::path::Path::from_waypoints_fitting_view).
//! The constructor is intended for noisy samples and programmed polylines; its
//! fitted curve may deviate from interior waypoints within explicit per-axis
//! tolerances. Use
//! [`Path::from_waypoints_interpolating`](crate::path::Path::from_waypoints_interpolating)
//! instead when every waypoint must be interpolated exactly.
//!
//! # Geometric contract
//!
//! Let `p_i` be the input columns and let `s_i` be their common parameters. On
//! `s in [s_i, s_{i+1}]`, the reference is the linearly interpolated polyline
//!
//! $$
//! r(s) = (1 - \theta) p_i + \theta p_{i+1}, \qquad
//! \theta = \frac{s-s_i}{s_{i+1}-s_i}.
//! $$
//!
//! For every selected axis `j`, a successful construction satisfies the
//! numerically audited bound
//!
//! $$
//! \max_s |q_j(s)-r_j(s)| \le \varepsilon_j.
//! $$
//!
//! This is a same-parameter, per-axis absolute-error box. It is not a nearest
//! distance to the polyline, a Cartesian/FK error, or a sum-of-squares bound.
//! Coordinates and tolerances retain the caller's units; angle unwrapping and
//! unit conversion remain the caller's responsibility.
//!
//! # Method
//!
//! 1. Normalize the parameter range to `[0, 1]`, subtract the first selected
//!    coordinate, and divide each selected axis by its tolerance.
//! 2. Accept a single straight span when the endpoint chord already lies in the
//!    tolerance tube, otherwise seed nonuniform knots with a linear-time slope
//!    corridor heuristic.
//! 3. Fit one open degree-five B-spline. Simple interior knots give `C4`
//!    continuity, while clamped endpoint control points preserve endpoint
//!    positions.
//! 4. Minimize a data term plus a span-scaled third-parameter-derivative
//!    penalty. Dense knot spans use spacing-weighted sample rows; sparse spans
//!    integrate against the complete reference polyline with Gauss quadrature.
//! 5. Convert the error on every reference/spline intersection to Bernstein
//!    form. The largest absolute Bernstein coefficient is a sufficient bound
//!    over the whole intersection, including extrema between input points.
//! 6. Insert knots near failed bounds, refit only overlapping control-point
//!    windows, and repeat until the audit passes or a configured budget is
//!    exhausted.
//! 7. Restore physical units and perform one final full-domain audit before
//!    returning the path.
//!
//! The least-squares solve itself does **not** contain hard tolerance
//! constraints. Instead, the audit/refinement loop rejects every detected
//! violation or numerical failure; there is no unchecked fallback path. As
//! detailed below, the ordinary-`f64` audit is not a formal proof certificate.
//!
//! # Continuity and derivatives
//!
//! Selected axes share one knot vector and form a quintic `C4` path. Their
//! endpoint positions are fixed, but endpoint derivatives are determined by the
//! fit. Unselected axes are represented by a separate quintic `C4` interpolant
//! through every input waypoint, with zero first and second parameter
//! derivatives at both endpoints.
//!
//! All reported derivatives are with respect to the path parameter `s`, not
//! time. In particular, penalizing `q'''(s)` is a geometric smoothing strategy;
//! it does not by itself minimize or bound physical jerk after time
//! parameterization.
//!
//! # Numerical properties
//!
//! Degree-five local support gives the normal matrix five nonzero subdiagonals.
//! A diagonally scaled banded Cholesky factorization is reused across selected
//! axes, avoiding a dense `M x M` normal matrix. With `N` input points, `M`
//! controls, and `d` selected axes, band storage and factorization are `O(M)` and
//! the multiple right-hand sides are `O(M d)` for this fixed degree.
//! Assembly/auditing otherwise traverses the relevant samples and spans in
//! approximately `O((N + M) d)` logical work, with logarithmic factors from the
//! implementation's parameter/span searches.
//!
//! The complete adaptive procedure is not globally `O(N)`: repeated knot
//! insertion, local reassembly, and interval audits depend on the requested
//! tolerance and refinement history. The compact solver uses normal equations,
//! which can square the original least-squares condition number; there is no
//! dense QR/SVD fallback when the banded factorization becomes unsupported.
//!
//! Bernstein bounds are computed with ordinary floating-point arithmetic, not
//! outward-rounded interval arithmetic. They are conservative polynomial bounds
//! in exact arithmetic, but the report is a numerical audit rather than a
//! formal proof certificate.

use super::OutOfRangeMode;
use crate::diag::PathError;
use nalgebra::DMatrixView;

// --- Public configuration and diagnostics -----------------------------------

/// Absolute deviation limits for the axes selected for smoothing.
///
/// Tolerances are expressed in the corresponding input-coordinate units and
/// must be finite and strictly positive. A scalar tolerance is broadcast to all
/// selected axes; a vector tolerance follows the exact order of
/// [`SmoothingConfig::axes`].
///
/// If `axes` is `None`, the selected order is `0..waypoints.nrows()`. Therefore
/// a [`SmoothingTolerance::PerAxis`] vector then follows the natural matrix-row
/// order.
#[derive(Clone, Debug)]
pub enum SmoothingTolerance {
    /// Broadcast one absolute tolerance to every selected axis.
    ///
    /// For example, `SmoothingTolerance::Uniform(1e-3)` permits a deviation of
    /// `1e-3` in each selected axis's own unit.
    Uniform(f64),
    /// Store one absolute tolerance per selected axis.
    ///
    /// The vector length must equal the number of selected axes. With
    /// `axes = Some(vec![4, 0])`, entries `0` and `1` apply to input rows `4`
    /// and `0`, respectively.
    PerAxis(Vec<f64>),
}

impl From<f64> for SmoothingTolerance {
    fn from(value: f64) -> Self {
        Self::Uniform(value)
    }
}

/// Configuration for tolerance-bounded waypoint fitting.
///
/// Error is measured against the input polyline at the same parameter over the
/// complete parameter domain, not only at waypoint columns. No kinematics,
/// angle unwrapping, nearest-point search, or unit conversion is performed.
///
/// # Defaults
///
/// - smooth every input row;
/// - use absolute tolerance `0.001` on each selected row;
/// - assign waypoint parameters uniformly on `[0, 1]`;
/// - allow at most 20 local refinement passes and 20,000 selected-axis spans;
/// - reject queries outside the parameter domain.
///
/// # Example
///
/// This example smooths translational rows `0..=2` to `1e-3` input units and
/// rotary rows `3..=4` to `1e-2` input units. Tolerance entries follow `axes`
/// order.
///
/// ```rust
/// use copp::path::{Path, SmoothingConfig, SmoothingTolerance};
/// use nalgebra::DMatrix;
///
/// # fn main() -> Result<(), copp::diag::PathError> {
/// let waypoints = DMatrix::from_fn(5, 4, |axis, point| {
///     axis as f64 * 0.1 + point as f64
/// });
/// let config = SmoothingConfig {
///     axes: Some(vec![0, 1, 2, 3, 4]),
///     tolerance: SmoothingTolerance::PerAxis(vec![
///         0.001, 0.001, 0.001, 0.01, 0.01,
///     ]),
///     ..Default::default()
/// };
///
/// let path = Path::from_waypoints_fitting(&waypoints, config)?;
/// assert_eq!(path.smoothing_report().unwrap().axes, vec![0, 1, 2, 3, 4]);
/// # Ok(())
/// # }
/// ```
#[derive(Clone, Debug)]
pub struct SmoothingConfig {
    /// Maximum absolute deviation from the reference polyline.
    ///
    /// The default is [`SmoothingTolerance::Uniform`]`(0.001)`. Every value is
    /// interpreted in the corresponding input row's unit.
    pub tolerance: SmoothingTolerance,
    /// Input rows allowed to deviate from their reference polylines.
    ///
    /// `None` selects every row. An explicit vector must be nonempty and contain
    /// distinct, in-range row indices. Its order also defines the order of
    /// [`SmoothingTolerance::PerAxis`] values and [`SmoothingReport::max_errors`].
    ///
    /// Unselected rows retain quintic `C4` interpolation through all input
    /// columns, with zero first and second parameter derivatives at both ends.
    pub axes: Option<Vec<usize>>,
    /// Common parameter assigned to each input column.
    ///
    /// An explicit vector must have `waypoints.ncols()` finite, strictly
    /// increasing entries. Its first and last entries become the public path
    /// range. `None` assigns columns uniformly on `[0, 1]`.
    ///
    /// Relative parameter spacing affects the reference correspondence, data
    /// weights, smoothing penalty, and fitted geometry; it is not merely output
    /// metadata. A positive affine rescaling leaves the internally normalized
    /// geometry unchanged but rescales derivatives returned with respect to `s`.
    pub parameters: Option<Vec<f64>>,
    /// Maximum number of adaptive knot-refinement passes.
    ///
    /// The default is 20. Reaching this limit before the continuous interval
    /// audit passes returns [`PathError::Smoothing`](crate::diag::PathError::Smoothing).
    pub max_refinements: usize,
    /// Maximum number of polynomial spans used by the selected axes.
    ///
    /// The default is 20,000. Spans belonging to the separate unselected-axis
    /// interpolant are reported independently and do not consume this budget.
    pub max_segments: usize,
    /// Behavior when evaluating outside the configured parameter domain.
    ///
    /// The default is [`OutOfRangeMode::Error`]. Clamping affects later queries,
    /// not fitting or auditing.
    pub out_of_range_mode: OutOfRangeMode,
}

impl Default for SmoothingConfig {
    fn default() -> Self {
        Self {
            tolerance: 0.001.into(),
            axes: None,
            parameters: None,
            max_refinements: 20,
            max_segments: 20_000,
            out_of_range_mode: OutOfRangeMode::Error,
        }
    }
}

/// Diagnostics recorded while constructing a smoothed waypoint path.
///
/// Obtain this report with [`Path::smoothing_report`](crate::path::Path::smoothing_report).
/// `axes`, `segments`, and `interpolated_segments` describe the final
/// representation. `refinements` records completed passes, while `fitting_rows`
/// and `checked_intervals` accumulate work from the initial construction and
/// subsequent local refinement. These values help explain tolerance cost and
/// budget usage; they are not optimality or run-time guarantees.
///
/// In exact arithmetic, the Bernstein convex-hull property supplies a sufficient
/// whole-interval bound. `max_errors` records the corresponding values computed
/// in physical input units with ordinary floating-point arithmetic rather than
/// outward-rounded interval arithmetic; they are numerical audit results, not
/// formal certificates or Cartesian/FK errors.
#[derive(Clone, Debug)]
pub struct SmoothingReport {
    /// Selected input-row indices, in tolerance/report order.
    pub axes: Vec<usize>,
    /// Number of final polynomial spans shared by the selected axes.
    ///
    /// This count is not guaranteed to be minimal or monotone as tolerance
    /// changes because initial knot placement and refinement are adaptive.
    pub segments: usize,
    /// Number of spans in the separate unselected-axis interpolant.
    ///
    /// This is zero when every axis is selected and is not included in
    /// [`SmoothingReport::segments`] or the configured segment budget.
    pub interpolated_segments: usize,
    /// Number of completed local knot-refinement passes.
    pub refinements: usize,
    /// Number of discrete or quadrature fitting rows assembled.
    ///
    /// Rows assembled again during local refits are counted again; this is a
    /// work statistic, not the number of distinct input points. Quadrature rows
    /// used only for the third-derivative penalty are not included.
    pub fitting_rows: usize,
    /// Number of reference-polyline intervals visited by numerical audits.
    ///
    /// Revisited intervals and the final physical-unit full audit are included.
    pub checked_intervals: usize,
    /// Final whole-domain numerical absolute-error bound for each selected axis.
    ///
    /// Values use physical input units and follow [`SmoothingReport::axes`]
    /// order. They are Bernstein-derived numerical bounds, not sampled maxima
    /// or outward-rounded certificates.
    pub max_errors: Vec<f64>,
}

// --- Shared numerical primitives --------------------------------------------

/// Convert any construction-stage failure into the single public smoothing
/// error variant. Callers either receive a fully audited path or this error;
/// there is deliberately no unchecked fallback representation.
fn fail(message: impl Into<String>) -> PathError {
    PathError::Smoothing {
        message: message.into(),
    }
}

/// Fixed B-spline degree used by the fitting and unselected-axis interpolants.
const P: usize = 5;

/// Factorials `0!` through `5!`, used for derivatives and binomial factors.
const FACT: [f64; 6] = [1., 1., 2., 6., 24., 120.];

/// Return the binomial coefficient `n choose k` for degrees up to five.
fn choose(n: usize, k: usize) -> f64 {
    FACT[n] / (FACT[k] * FACT[n - k])
}

// --- Quintic B-spline representation ----------------------------------------

/// Open nonuniform degree-five B-spline shared by `d` coordinate axes.
///
/// The knot vector `t` contains six repeated knots at both endpoints and only
/// simple interior knots. Consequently the represented path interpolates its
/// first/last control points and is `C4` across every interior knot.
///
/// Control points use control-major, axis-minor layout:
/// `c[control * d + axis]`. All selected axes share `t`, which preserves their
/// common parameterization and permits one normal matrix to serve every axis.
#[derive(Clone)]
struct BSpline {
    /// Nondecreasing open knot vector on the normalized domain `[0, 1]`.
    t: Vec<f64>,
    /// Flat control-point coordinates in control-major, axis-minor order.
    c: Vec<f64>,
    /// Number of coordinate axes stored per control point.
    d: usize,
}

impl BSpline {
    /// Return the number of control points.
    fn m(&self) -> usize {
        self.c.len() / self.d
    }

    /// Locate the knot span containing `x`.
    ///
    /// Endpoint queries are clamped to the first/last evaluable span. Callers
    /// validate the public parameter range before reaching this kernel.
    fn span(&self, x: f64) -> usize {
        self.t
            .partition_point(|v| *v <= x)
            .saturating_sub(1)
            .clamp(P, self.m() - 1)
    }

    /// Evaluate the six nonzero B-spline basis functions at `x`.
    ///
    /// Returns the global index of the first active control point together with
    /// basis values `B[start..start+6](x)`. The triangular recurrence is the
    /// allocation-free degree-five Cox--de Boor basis algorithm.
    fn values(&self, x: f64) -> (usize, [f64; 6]) {
        let span = self.span(x);
        let mut b = [0.; 6];
        b[0] = 1.;
        let mut left = [0.; 6];
        let mut right = [0.; 6];
        for j in 1..=P {
            left[j] = x - self.t[span + 1 - j];
            right[j] = self.t[span + j] - x;
            let mut saved = 0.;
            for r in 0..j {
                let v = b[r] / (right[r + 1] + left[j - r]);
                b[r] = saved + right[r + 1] * v;
                saved = left[j - r] * v;
            }
            b[j] = saved;
        }
        (span - P, b)
    }

    /// Expand the six active basis functions into Taylor coefficients at `x`.
    ///
    /// `returned[i][r]` is the coefficient of `(xi-x)^r` for active basis `i`;
    /// multiplying it by `r!` yields the `r`-th derivative at `x`. Keeping the
    /// complete degree-five expansion on the stack supports exact span-to-power
    /// conversion and derivative quadrature without finite differences.
    fn powers(&self, span: usize, x: f64) -> [[f64; 6]; 6] {
        let mut b = [[0.; 6]; 6];
        b[0][0] = 1.;
        for j in 1..=P {
            let mut saved = [0.; 6];
            for (r, row) in b.iter_mut().enumerate().take(j) {
                let left = x - self.t[span + 1 - j + r];
                let right = self.t[span + r + 1] - x;
                let den = left + right;
                let mut next = [0.; 6];
                let mut upper = [0.; 6];
                for k in 0..=j {
                    let v = row[k] / den;
                    let prev = if k > 0 { row[k - 1] / den } else { 0. };
                    next[k] = saved[k] + right * v - prev;
                    upper[k] = left * v + prev;
                }
                *row = next;
                saved = upper;
            }
            b[j] = saved;
        }
        b
    }

    /// Insert one simple knot without changing the represented curve.
    ///
    /// The control-point update is the standard local B-spline knot-insertion
    /// recurrence. It adds one degree of freedom for the following refit while
    /// preserving the pre-insertion geometry and `C4` continuity.
    fn insert(&mut self, x: f64) {
        let k = self.span(x);
        let m = self.m();
        let mut q = vec![0.; (m + 1) * self.d];
        q[..(k - P + 1) * self.d].copy_from_slice(&self.c[..(k - P + 1) * self.d]);
        q[(k + 1) * self.d..].copy_from_slice(&self.c[k * self.d..]);
        for i in k - P + 1..=k {
            let a = (x - self.t[i]) / (self.t[i + P] - self.t[i]);
            for j in 0..self.d {
                q[i * self.d + j] =
                    (1. - a) * self.c[(i - 1) * self.d + j] + a * self.c[i * self.d + j];
            }
        }
        self.t.insert(k + 1, x);
        self.c = q;
    }

    /// Convert all nonempty knot spans into local quintic power polynomials.
    ///
    /// For span `ell`, coefficients are stored for the normalized local
    /// coordinate `u = (x-breaks[ell]) / h`. The output layout is span-major,
    /// axis-minor and is optimized for audit and query-time Horner evaluation.
    fn polynomial(&self) -> Polynomial {
        let mut breaks = Vec::new();
        let mut coefficients = Vec::new();
        for span in P..self.m() {
            let a = self.t[span];
            let h = self.t[span + 1] - a;
            if h <= 0. {
                continue;
            }
            breaks.push(a);
            let basis = self.powers(span, a);
            for axis in 0..self.d {
                let mut p = [0.; 6];
                for (r, row) in basis.iter().enumerate() {
                    for k in 0..6 {
                        p[k] += row[k] * self.c[(span - P + r) * self.d + axis];
                    }
                }
                let mut scale = 1.;
                for v in &mut p {
                    *v *= scale;
                    scale *= h;
                }
                coefficients.push(p);
            }
        }
        breaks.push(1.);
        Polynomial {
            breaks,
            coefficients,
            d: self.d,
        }
    }
}

// --- Banded normal-equation solver ------------------------------------------

/// Active block of a symmetric normal system with five stored subdiagonals.
///
/// For active global controls `[a, z)`, local row `i` stores the lower triangle
/// as `h[i][k] = H[i, i-k]`, `k=0..=5`. The B-spline basis is common to all
/// coordinates, so the matrix is shared while `rhs` contains `d` interleaved
/// right-hand sides.
///
/// `add` assembles normal-equation increments around the current control vector;
/// `solve` applies diagonal equilibration followed by an in-place banded
/// Cholesky factorization and forward/back substitution.
struct Normal {
    /// Lower symmetric band, from the diagonal through the fifth subdiagonal.
    h: Vec<[f64; 6]>,
    /// Interleaved right-hand sides in active-control-major order.
    rhs: Vec<f64>,
    /// Global index of the first active control point.
    a: usize,
    /// Number of coordinate right-hand sides.
    d: usize,
}

impl Normal {
    /// Allocate the active-control system for the half-open range `[a, z)`.
    fn new(a: usize, z: usize, d: usize) -> Self {
        Self {
            h: vec![[0.; 6]; z - a],
            rhs: vec![0.; (z - a) * d],
            a,
            d,
        }
    }

    /// Add one weighted least-squares row to the normal system.
    ///
    /// `b` contains six consecutive basis/derivative values beginning at
    /// `start`, `target` contains one desired value per coordinate, and `c` is
    /// the current global control vector. Controls outside this system's active
    /// range remain frozen: they contribute to the residual but receive no
    /// matrix or right-hand-side update.
    fn add(&mut self, start: usize, b: &[f64; 6], target: &[f64], weight: f64, c: &[f64]) {
        for i in 0..6 {
            let global = start + i;
            if global < self.a || global >= self.a + self.h.len() {
                continue;
            }
            let row = global - self.a;
            for j in 0..=i {
                let col = start + j;
                if col >= self.a {
                    self.h[row][i - j] += weight * b[i] * b[j];
                }
            }
        }
        for axis in 0..self.d {
            let value = (0..6)
                .map(|j| b[j] * c[(start + j) * self.d + axis])
                .sum::<f64>();
            let residual = weight * (target[axis] - value);
            for (i, value) in b.iter().enumerate() {
                let global = start + i;
                if global >= self.a && global < self.a + self.h.len() {
                    self.rhs[(global - self.a) * self.d + axis] += value * residual;
                }
            }
        }
    }

    /// Solve the equilibrated symmetric positive-definite band system.
    ///
    /// Returns active control **increments**, interleaved by coordinate. A zero
    /// or non-finite diagonal, loss of positive definiteness, or non-finite
    /// solution is reported as [`PathError::Smoothing`] rather than falling back
    /// to a dense or unchecked solve.
    fn solve(mut self) -> Result<Vec<f64>, PathError> {
        let n = self.h.len();
        let scales: Vec<_> = self.h.iter().map(|r| r[0].sqrt()).collect();
        if scales.iter().any(|s| !s.is_finite() || *s <= 0.) {
            return Err(fail("unsupported knot layout"));
        }
        for i in 0..n {
            for k in 0..=P.min(i) {
                self.h[i][k] /= scales[i] * scales[i - k];
            }
            for axis in 0..self.d {
                self.rhs[i * self.d + axis] /= scales[i];
            }
        }
        for i in 0..n {
            for j in i.saturating_sub(P)..=i {
                let mut v = self.h[i][i - j];
                for k in i.saturating_sub(P)..j {
                    v -= self.h[i][i - k] * self.h[j][j - k];
                }
                if i == j {
                    if !v.is_finite() || v <= 0. {
                        return Err(fail("non-positive banded factorization"));
                    }
                    self.h[i][0] = v.sqrt();
                } else {
                    self.h[i][i - j] = v / self.h[j][0];
                }
            }
        }
        for i in 0..n {
            for axis in 0..self.d {
                for k in 1..=P.min(i) {
                    self.rhs[i * self.d + axis] -= self.h[i][k] * self.rhs[(i - k) * self.d + axis];
                }
                self.rhs[i * self.d + axis] /= self.h[i][0];
            }
        }
        for i in (0..n).rev() {
            for axis in 0..self.d {
                for j in i + 1..(i + P + 1).min(n) {
                    self.rhs[i * self.d + axis] -= self.h[j][j - i] * self.rhs[j * self.d + axis];
                }
                self.rhs[i * self.d + axis] /= self.h[i][0];
            }
        }
        for (i, scale) in scales.iter().enumerate() {
            for axis in 0..self.d {
                self.rhs[i * self.d + axis] /= scale;
            }
        }
        if self.rhs.iter().any(|v| !v.is_finite()) {
            return Err(fail("non-finite coefficients"));
        }
        Ok(self.rhs)
    }
}

/// Six-point Gauss--Legendre nodes mapped from `[-1, 1]` to `[0, 1]`.
///
/// Six points integrate polynomials through degree eleven exactly. On sparse
/// spans this covers the squared difference between a quintic and a line,
/// whose degree is at most ten.
const GX: [f64; 6] = [
    0.033765242898423975,
    0.16939530676686776,
    0.3806904069584015,
    0.6193095930415985,
    0.8306046932331322,
    0.966234757101576,
];

/// Weights paired with [`GX`] on `[0, 1]`.
const GW: [f64; 6] = [
    0.08566224618958517,
    0.1803807865240693,
    0.2339569672863455,
    0.2339569672863455,
    0.1803807865240693,
    0.08566224618958517,
];

/// Fit an active control-point window while freezing all other controls.
///
/// For the normalized selected-axis curve `q_hat`, this assembles the
/// incremental normal equations for
///
/// $$
/// E(C) = E_{data}(C)
///      + \lambda \sum_l h_l^6
///        \int_{t_l}^{t_{l+1}} \lVert q_hat'''(x) \rVert_2^2\,dx.
/// $$
///
/// The data term is selected independently for every nonempty knot span:
///
/// - spans containing at least eight waypoint samples use trapezoidal
///   parameter-spacing weights at those samples;
/// - sparse spans are split at reference-polyline vertices and use six-point
///   Gauss--Legendre quadrature over every intersection, so long line segments
///   still influence the fit even when they contain no interior samples.
///
/// The third-derivative penalty uses three-point Gauss--Legendre quadrature,
/// exact for its degree-four squared integrand. The `h_l^6` strategy balances
/// differently sized local spans; it is not a global time-domain minimum-jerk
/// objective.
///
/// # Arguments
///
/// - `b`: complete spline and current control vector;
/// - `s`, `y`: normalized parameters and point-major selected coordinates;
/// - `a`, `z`: half-open global control range `[a, z)` allowed to change;
/// - `strength`: current value of the derivative-penalty weight `lambda`.
///
/// # Returns
///
/// `(first, last, rows)` identifies the half-open waypoint range touched by the
/// active support and the number of discrete/quadrature rows assembled. The
/// range lets the refinement loop conservatively re-audit neighboring reference
/// intervals.
fn fit(
    b: &mut BSpline,
    s: &[f64],
    y: &[f64],
    a: usize,
    z: usize,
    strength: f64,
) -> Result<(usize, usize, usize), PathError> {
    let lo = b.t[a];
    let hi = b.t[z + P];
    let first = s.partition_point(|v| *v < lo);
    let last = s.partition_point(|v| *v <= hi);
    let mut normal = Normal::new(a, z, b.d);
    let mut rows = 0;
    // Density is local to each knot span: a globally dense trajectory can still
    // acquire a short refined span with no input vertices inside it.
    let mut dense = vec![false; b.m()];
    for (span, populated) in dense.iter_mut().enumerate().skip(P) {
        let begin = s.partition_point(|v| *v < b.t[span]);
        let end = s.partition_point(|v| *v < b.t[span + 1]);
        *populated = end - begin >= 8;
    }
    // Dense spans use sample rows with trapezoidal parameter-measure weights.
    for i in first..last {
        if !dense[b.span(s[i])] {
            continue;
        }
        let weight = if i == 0 {
            (s[1] - s[0]) / 2.
        } else if i + 1 == s.len() {
            (s[i] - s[i - 1]) / 2.
        } else {
            (s[i + 1] - s[i - 1]) / 2.
        };
        let (start, basis) = b.values(s[i]);
        normal.add(start, &basis, &y[i * b.d..(i + 1) * b.d], weight, &b.c);
        rows += 1;
    }
    // Sparse spans are split wherever the piecewise-linear reference changes.
    // Six quadrature rows per intersection integrate the normal terms exactly.
    let mut target = vec![0.; b.d];
    for (span, populated) in dense.iter().enumerate().skip(P) {
        let left = b.t[span];
        let right = b.t[span + 1];
        if *populated || right <= left || left >= hi || right <= lo {
            continue;
        }
        let begin = s.partition_point(|v| *v <= left);
        let end = s.partition_point(|v| *v < right);
        let mut mesh = Vec::with_capacity(end - begin + 2);
        mesh.push(left);
        mesh.extend_from_slice(&s[begin..end]);
        mesh.push(right);
        for pair in mesh.windows(2) {
            let h = pair[1] - pair[0];
            for r in 0..6 {
                let x = pair[0] + GX[r] * h;
                let i = s
                    .partition_point(|v| *v <= x)
                    .saturating_sub(1)
                    .min(s.len() - 2);
                let f = (x - s[i]) / (s[i + 1] - s[i]);
                for axis in 0..b.d {
                    target[axis] =
                        y[i * b.d + axis] + f * (y[(i + 1) * b.d + axis] - y[i * b.d + axis]);
                }
                let (start, basis) = b.values(x);
                normal.add(start, &basis, &target, h * GW[r], &b.c);
                rows += 1;
            }
        }
    }
    // Three Gauss--Legendre nodes integrate the squared quadratic q''' basis.
    // `h^7 = h^6 * h`: the final factor is the quadrature change of variables.
    let zeros = vec![0.; b.d];
    for span in P..b.m() {
        let left = b.t[span];
        let h = b.t[span + 1] - left;
        if h <= 0. || left >= hi || b.t[span + 1] <= lo {
            continue;
        }
        for (x, w) in [
            (0.1127016653792583, 5. / 18.),
            (0.5, 4. / 9.),
            (0.8872983346207417, 5. / 18.),
        ] {
            let power = b.powers(span, left + x * h);
            let basis = std::array::from_fn(|i| 6. * power[i][3]);
            normal.add(span - P, &basis, &zeros, strength * h.powi(7) * w, &b.c);
        }
    }
    let delta = normal.solve()?;
    for (i, v) in delta.iter().enumerate() {
        b.c[a * b.d + i] += v;
    }
    Ok((first, last, rows))
}

// --- Piecewise-polynomial evaluation and auditing ---------------------------

/// Piecewise quintic power representation on normalized parameter `[0, 1]`.
///
/// `breaks` has one more entry than the number of spans. Coefficients use
/// span-major, axis-minor layout; each `[f64; 6]` stores ascending powers of
/// `u = (x-breaks[span]) / (breaks[span+1]-breaks[span])`.
#[derive(Clone)]
struct Polynomial {
    /// Strictly increasing nonempty-span boundaries.
    breaks: Vec<f64>,
    /// Local ascending-power coefficients, grouped by span then axis.
    coefficients: Vec<[f64; 6]>,
    /// Number of stored coordinate axes per span.
    d: usize,
}

impl Polynomial {
    /// Return the number of nonempty polynomial spans.
    fn segments(&self) -> usize {
        self.breaks.len() - 1
    }

    /// Locate the polynomial span containing normalized parameter `x`.
    fn span(&self, x: f64) -> usize {
        self.breaks
            .partition_point(|v| *v <= x)
            .saturating_sub(1)
            .min(self.segments() - 1)
    }

    /// Evaluate one axis through third order on a known span.
    ///
    /// Horner evaluation is performed separately for each derivative order.
    /// Division by `h^r` converts derivatives from local `u` to normalized
    /// global parameter `x`. The returned array is `[q, q', q'', q''']`.
    fn at(&self, span: usize, x: f64, axis: usize) -> [f64; 4] {
        let h = self.breaks[span + 1] - self.breaks[span];
        let u = (x - self.breaks[span]) / h;
        let p = &self.coefficients[span * self.d + axis];
        let mut out = [0.; 4];
        for r in 0..4 {
            let mut v = 0.;
            for k in (r..6).rev() {
                v = v * u + p[k] * FACT[k] / FACT[k - r];
            }
            out[r] = v / h.powi(r as i32);
        }
        out
    }
}

/// Bound the selected-axis error over complete reference-polyline intervals.
///
/// The routine walks reference intervals and intersecting polynomial spans with
/// monotone cursors; it does not scan every spline span for every input interval.
/// On each intersection, it reparameterizes the quintic error to
/// `tau in [0, 1]` and converts ascending power coefficients `a_j` into degree-five
/// Bernstein coefficients
///
/// $$
/// \beta_k = \sum_{j=0}^k
///           \frac{\binom{k}{j}}{\binom{5}{j}} a_j.
/// $$
///
/// Since Bernstein basis functions are nonnegative and sum to one,
/// `max_k |beta_k|` bounds `|q-r|` everywhere on the intersection. This catches
/// between-sample extrema, although the convex-hull bound may overestimate the
/// true maximum.
///
/// `first..last` is a half-open range of reference intervals. `bounds` is
/// reference-interval-major and axis-minor; updated entries receive the largest
/// Bernstein bound seen on that reference interval. `witness[i]` receives the
/// midpoint of the intersection producing the largest coefficient and is only
/// a refinement hint, not an estimated maximizer.
///
/// In exact arithmetic the convex-hull inequality is a sufficient upper bound.
/// Here arithmetic is ordinary `f64`: construction rejects any candidate whose
/// computed audit fails, but the result is not an outward-rounded formal proof.
fn audit(
    p: &Polynomial,
    s: &[f64],
    y: &[f64],
    first: usize,
    last: usize,
    bounds: &mut [f64],
    witness: &mut [f64],
) -> Result<(), PathError> {
    let mut span = p.span(s[first]);
    for i in first..last {
        bounds[i * p.d..(i + 1) * p.d].fill(0.);
        let mut peak = -1.;
        let mut left = s[i];
        while left < s[i + 1] {
            while span + 1 < p.segments() && p.breaks[span + 1] <= left {
                span += 1;
            }
            let right = s[i + 1].min(p.breaks[span + 1]);
            if right <= left {
                return Err(fail("parameter resolution exhausted during audit"));
            }
            let h = p.breaks[span + 1] - p.breaks[span];
            let u = (left - p.breaks[span]) / h;
            let w = (right - left) / h;
            for axis in 0..p.d {
                let poly = &p.coefficients[span * p.d + axis];
                // Restrict the span-local polynomial to this intersection and
                // express it in a fresh power basis over tau in [0, 1].
                let mut power = [0.; 6];
                for (j, value) in power.iter_mut().enumerate() {
                    for (k, coefficient) in poly.iter().enumerate().skip(j) {
                        *value += choose(k, j) * coefficient * u.powi((k - j) as i32);
                    }
                    *value *= w.powi(j as i32);
                }
                let delta = y[(i + 1) * p.d + axis] - y[i * p.d + axis];
                // Subtract the same-parameter reference line. Only the constant
                // and linear power coefficients change.
                power[0] -= y[i * p.d + axis] + (left - s[i]) / (s[i + 1] - s[i]) * delta;
                power[1] -= (right - left) / (s[i + 1] - s[i]) * delta;
                for k in 0..6 {
                    // Convert power coefficient prefix 0..=k to Bernstein beta_k.
                    let mut v = 0.;
                    for (j, value) in power.iter().enumerate().take(k + 1) {
                        v += value * choose(k, j) / choose(P, j);
                    }
                    if !v.is_finite() {
                        return Err(fail("non-finite interval error bound"));
                    }
                    bounds[i * p.d + axis] = bounds[i * p.d + axis].max(v.abs());
                    if v.abs() > peak {
                        peak = v.abs();
                        witness[i] = (left + right) / 2.;
                    }
                }
            }
            left = right;
        }
    }
    Ok(())
}

// --- Knot initialization and local refinement -------------------------------

/// Seed nonuniform interior knots with a shared multi-axis slope corridor.
///
/// Starting at an anchor waypoint, each later normalized point contributes a
/// per-axis slope interval corresponding to a `0.65` error allowance. The scan
/// intersects these intervals until the current endpoint slope no longer fits;
/// the previous point then becomes a new anchor. Every point is processed at
/// most twice, giving `O(s.len() * d)` corridor work.
///
/// Around every internal anchor, three candidate knots are placed symmetrically.
/// Their width is capped both by neighboring parameter gaps (`0.45`) and by the
/// largest cross-axis slope jump (`0.35 / ((35/256) * jump)`), concentrating
/// resolution around sharp changes. Knots closer than `128 * f64::EPSILON` or
/// too close to an endpoint are discarded.
///
/// These constants define an initialization heuristic only. The returned knots
/// neither certify tolerance nor minimize the final number of spans; [`audit`]
/// and adaptive refinement establish the construction contract.
fn seed(s: &[f64], y: &[f64], d: usize) -> Vec<f64> {
    let mut indices = vec![0];
    let mut anchor = 0;
    let mut i = 1;
    let mut lo = vec![f64::NEG_INFINITY; d];
    let mut hi = vec![f64::INFINITY; d];
    while i < s.len() {
        let ds = s[i] - s[anchor];
        let mut ok = true;
        for axis in 0..d {
            let slope = (y[i * d + axis] - y[anchor * d + axis]) / ds;
            lo[axis] = lo[axis].max(slope - 0.65 / ds);
            hi[axis] = hi[axis].min(slope + 0.65 / ds);
            ok &= slope >= lo[axis] && slope <= hi[axis];
        }
        if ok {
            i += 1;
        } else {
            anchor = i - 1;
            indices.push(anchor);
            lo.fill(f64::NEG_INFINITY);
            hi.fill(f64::INFINITY);
        }
    }
    indices.push(s.len() - 1);
    let mut knots = Vec::new();
    for triple in indices.windows(3) {
        let [a, b, c] = [triple[0], triple[1], triple[2]];
        let h0 = s[b] - s[a];
        let h1 = s[c] - s[b];
        let mut jump: f64 = 0.;
        for axis in 0..d {
            jump = jump.max(
                ((y[c * d + axis] - y[b * d + axis]) / h1
                    - (y[b * d + axis] - y[a * d + axis]) / h0)
                    .abs(),
            );
        }
        let width = (0.45 * h0.min(h1)).min(0.35 / ((35. / 256.) * jump));
        knots.extend([s[b] - width, s[b], s[b] + width]);
    }
    knots.sort_by(f64::total_cmp);
    let mut kept = Vec::new();
    let gap = 128. * f64::EPSILON;
    for x in knots {
        if x > gap && x < 1. - gap && kept.last().is_none_or(|v| x - v > gap) {
            kept.push(x);
        }
    }
    kept
}

/// Merge control-point windows affected by newly inserted knots.
///
/// A quintic basis has support across six knot spans. Each proposed location is
/// expanded to a conservative active range, endpoint controls are excluded, and
/// ranges with overlapping support are coalesced. Controls outside the returned
/// half-open `[a, z)` windows remain frozen during local refits.
fn regions(b: &BSpline, x: &[f64]) -> Vec<(usize, usize)> {
    let mut out: Vec<(usize, usize)> = Vec::new();
    for value in x {
        let k = b.span(*value);
        let a = k.saturating_sub(2 * P).max(1);
        let z = (k + P + 1).min(b.m() - 1);
        if let Some(last) = out.last_mut()
            && a <= last.1 + P
        {
            last.1 = last.1.max(z);
            continue;
        }
        out.push((a, z));
    }
    out
}

// --- Complete smoothing representation --------------------------------------

/// Fully constructed representation behind `PathRepr::Smoothed`.
///
/// Both polynomial sets use physical coordinate values over normalized internal
/// parameter `[0, 1]`. `axes` and `other` map their compact polynomial-axis
/// indices back to rows of the public `Path` output buffers. The original
/// parameter endpoints are retained so evaluation can apply the chain rule.
pub(super) struct SmoothedSpline {
    /// Tolerance-fitted polynomial shared by the selected axes.
    selected: Polynomial,
    /// Waypoint-interpolating polynomial for unselected axes, when any exist.
    ///
    /// The historical field name means "not selected for smoothing"; values
    /// between waypoints are quintic interpolation, not the original polyline.
    unchanged: Option<Polynomial>,
    /// Original matrix-row indices represented by `selected`, in configured order.
    axes: Vec<usize>,
    /// Original matrix-row indices represented by `unchanged`, in row order.
    other: Vec<usize>,
    /// Lower endpoint of the caller-visible path parameter range.
    pub(super) start: f64,
    /// Upper endpoint of the caller-visible path parameter range.
    pub(super) end: f64,
    /// Final representation metadata, numerical bounds, and cumulative work.
    pub(super) report: SmoothingReport,
}

impl SmoothedSpline {
    /// Construct and audit a tolerance-bounded waypoint approximation.
    ///
    /// The implementation follows a validate, fit, audit, and refine sequence:
    ///
    /// 1. validate matrix/configuration contracts and resolve selected axes;
    /// 2. normalize the common parameter and each selected coordinate by its
    ///    tolerance;
    /// 3. take the endpoint-chord fast path or seed a nonuniform knot vector;
    /// 4. fit the selected axes and audit every continuous reference interval;
    /// 5. insert knots and locally refit failed regions within both budgets;
    /// 6. restore physical units, repeat the complete audit, and construct the
    ///    unselected-axis interpolant.
    ///
    /// Any invalid input, numerical failure, exhausted budget, or failed final
    /// audit returns [`PathError::Smoothing`]. No partially fitted object escapes.
    pub(super) fn build(
        points: DMatrixView<'_, f64>,
        cfg: &SmoothingConfig,
    ) -> Result<Self, PathError> {
        // Validate the common matrix and establish the configured axis mapping.
        let d = points.nrows();
        let n = points.ncols();
        if d == 0 || n < 2 || points.iter().any(|v| !v.is_finite()) {
            return Err(fail(
                "expected finite (dim, n_points) matrix, dim > 0, n_points >= 2",
            ));
        }
        let axes = cfg.axes.clone().unwrap_or_else(|| (0..d).collect());
        let mut seen = vec![false; d];
        if axes.is_empty() {
            return Err(fail("axes must not be empty"));
        }
        for &a in &axes {
            if a >= d || seen[a] {
                return Err(fail("axes must be distinct valid indices"));
            }
            seen[a] = true;
        }
        let eps = match &cfg.tolerance {
            SmoothingTolerance::Uniform(x) => vec![*x; axes.len()],
            SmoothingTolerance::PerAxis(v) => v.clone(),
        };
        if eps.len() != axes.len() || eps.iter().any(|v| !v.is_finite() || *v <= 0.) {
            return Err(fail("positive tolerance required for each selected axis"));
        }
        // Normalize the caller's common parameter to the internal unit interval.
        // The original endpoints are kept for query-time derivative scaling.
        let parameters = cfg
            .parameters
            .clone()
            .unwrap_or_else(|| (0..n).map(|i| i as f64 / (n - 1) as f64).collect());
        if parameters.len() != n
            || parameters.iter().any(|v| !v.is_finite())
            || parameters.windows(2).any(|v| v[1] <= v[0])
        {
            return Err(fail(
                "parameters must be finite, strictly increasing and match the point count",
            ));
        }
        let start = parameters[0];
        let end = parameters[n - 1];
        let length = end - start;
        let s: Vec<_> = parameters.iter().map(|v| (v - start) / length).collect();
        if !length.is_finite() || s.windows(2).any(|v| v[1] <= v[0]) {
            return Err(fail("parameter range cannot be represented reliably"));
        }
        // Center each selected coordinate at its first value and divide by its
        // tolerance. The admissible error is now one on every selected axis.
        let k = axes.len();
        let mut y = vec![0.; n * k];
        for i in 0..n {
            for a in 0..k {
                y[i * k + a] = (points[(axes[a], i)] - points[(axes[a], 0)]) / eps[a];
            }
        }
        if y.iter().any(|v| !v.is_finite()) {
            return Err(fail("coordinate/tolerance scale overflow"));
        }
        // A 0.95 threshold leaves margin below the unit normalized tolerance.
        // The chord/reference difference is linear on each input interval, so
        // checking all vertices is sufficient for this special candidate.
        let linear = (0..n)
            .all(|i| (0..k).all(|a| (y[i * k + a] - s[i] * y[(n - 1) * k + a]).abs() <= 0.95));
        let knots = if linear { Vec::new() } else { seed(&s, &y, k) };
        if knots.len() + 1 > cfg.max_segments {
            return Err(fail("initial segment budget exceeded"));
        }
        // Open degree-five knot vector. Greville control abscissae reproduce the
        // endpoint chord exactly and provide a stable initial iterate for fitting.
        let mut t = vec![0.; 6];
        t.extend(knots);
        t.extend([1.; 6]);
        let mut c = vec![0.; (t.len() - 6) * k];
        for i in 0..t.len() - 6 {
            let u = t[i + 1..i + 6].iter().sum::<f64>() / 5.;
            for a in 0..k {
                c[i * k + a] = u * y[(n - 1) * k + a];
            }
        }
        let mut b = BSpline { t, c, d: k };
        let mut report = SmoothingReport {
            axes: axes.clone(),
            segments: 0,
            interpolated_segments: 0,
            refinements: 0,
            fitting_rows: 0,
            checked_intervals: 0,
            max_errors: vec![0.; k],
        };
        // Endpoint controls 0 and m-1 remain frozen, preserving both input
        // endpoint positions. The first global fit uses lambda = 1e-3.
        if !linear {
            let m = b.m();
            report.fitting_rows += fit(&mut b, &s, &y, 1, m - 1, 0.001)?.2;
        }
        let mut bounds = vec![0.; (n - 1) * k];
        let mut witness = vec![0.; n - 1];
        audit(&b.polynomial(), &s, &y, 0, n - 1, &mut bounds, &mut witness)?;
        report.checked_intervals += n - 1;
        // Refine until the computed normalized audit bound is at most 0.99. We
        // select every interval above 0.9 to batch near-limit regions and retain
        // headroom below the physical-unit acceptance threshold.
        loop {
            let worst = bounds.iter().copied().fold(0., f64::max);
            if worst <= 0.99 {
                break;
            }
            if report.refinements >= cfg.max_refinements {
                return Err(fail(format!(
                    "refinement limit reached; error/tolerance={worst}"
                )));
            }
            let mut locations = Vec::new();
            for i in 0..n - 1 {
                if bounds[i * k..(i + 1) * k].iter().any(|v| *v > 0.9) {
                    let span = b.span(witness[i]);
                    locations.push((b.t[span] + b.t[span + 1]) / 2.);
                }
            }
            locations.sort_by(f64::total_cmp);
            locations.dedup();
            if b.m() - 5 + locations.len() > cfg.max_segments {
                return Err(fail("refinement segment budget exceeded"));
            }
            // Knot insertion first preserves the current curve exactly; the
            // subsequent active-window solve uses the new local freedom.
            for &x in &locations {
                let span = b.span(x);
                if x <= b.t[span] || x >= b.t[span + 1] {
                    return Err(fail("parameter resolution exhausted"));
                }
                b.insert(x);
            }
            report.refinements += 1;
            for (a, z) in regions(&b, &locations) {
                let (first, last, rows) = fit(
                    &mut b,
                    &s,
                    &y,
                    a,
                    z,
                    // Relax smoothness geometrically as fidelity becomes the
                    // limiting requirement in repeatedly refined regions.
                    0.001 * 0.25f64.powi(report.refinements as i32),
                )?;
                report.fitting_rows += rows;
                let left = first.saturating_sub(1);
                let right = last.min(n - 1);
                audit(
                    &b.polynomial(),
                    &s,
                    &y,
                    left,
                    right,
                    &mut bounds,
                    &mut witness,
                )?;
                report.checked_intervals += right - left;
            }
        }
        // Restore selected axes to physical units, then repeat a full audit.
        // This catches scaling roundoff and stale local-bound bookkeeping before
        // a successful Path can be returned.
        for i in 0..b.m() {
            for a in 0..k {
                b.c[i * k + a] = b.c[i * k + a] * eps[a] + points[(axes[a], 0)];
            }
        }
        let selected = b.polynomial();
        for i in 0..n {
            for a in 0..k {
                y[i * k + a] = points[(axes[a], i)];
            }
        }
        audit(&selected, &s, &y, 0, n - 1, &mut bounds, &mut witness)?;
        report.checked_intervals += n - 1;
        for a in 0..k {
            report.max_errors[a] = (0..n - 1).map(|i| bounds[i * k + a]).fold(0., f64::max);
            if report.max_errors[a] > eps[a] {
                return Err(fail("final physical-unit interval audit failed"));
            }
        }
        // Rows excluded from smoothing still share the same public parameter,
        // but use a separate waypoint-interpolating polynomial and segment count.
        let other: Vec<_> = (0..d).filter(|a| !seen[*a]).collect();
        let unchanged = if other.is_empty() {
            None
        } else {
            Some(interpolate(points, &s, &other)?)
        };
        report.segments = selected.segments();
        report.interpolated_segments = unchanged.as_ref().map_or(0, Polynomial::segments);
        Ok(Self {
            selected,
            unchanged,
            axes,
            other,
            start,
            end,
            report,
        })
    }

    /// Evaluate physical coordinates and optional derivatives at one parameter.
    ///
    /// `x` has already been validated or clamped by `Path`. It is mapped from
    /// `[start, end]` to the internal unit interval; an order-`r` derivative is
    /// then multiplied by `(end-start)^(-r)`. Compact selected/unselected values
    /// are scattered back to their original axis positions in the caller's
    /// column-major output buffers.
    pub(super) fn evaluate(
        &self,
        x: f64,
        q: &mut [f64],
        dq: Option<&mut [f64]>,
        ddq: Option<&mut [f64]>,
        dddq: Option<&mut [f64]>,
    ) {
        let u = (x - self.start) / (self.end - self.start);
        let scale = 1. / (self.end - self.start);
        let mut outputs = [Some(q), dq, ddq, dddq];
        for (poly, axes) in [
            (Some(&self.selected), &self.axes),
            (self.unchanged.as_ref(), &self.other),
        ] {
            if let Some(p) = poly {
                let span = p.span(u);
                for (i, &axis) in axes.iter().enumerate() {
                    let values = p.at(span, u, i);
                    let mut f = 1.;
                    for r in 0..4 {
                        if let Some(out) = outputs[r].as_deref_mut() {
                            out[axis] = values[r] * f;
                        }
                        f *= scale;
                    }
                }
            }
        }
    }
}

/// Build the waypoint-interpolating quintic `C4` polynomial for unselected axes.
///
/// One simple internal knot is placed at every interior input parameter. The
/// system contains one position equation per waypoint plus zero first- and
/// second-parameter-derivative equations at both endpoints, matching the number
/// of controls. This path is independent of selected-axis tolerance and may use
/// a different number of spans from the fitted polynomial.
fn interpolate(
    points: DMatrixView<'_, f64>,
    s: &[f64],
    axes: &[usize],
) -> Result<Polynomial, PathError> {
    let d = axes.len();
    let n = s.len();
    let mut t = vec![0.; 6];
    t.extend_from_slice(&s[1..n - 1]);
    t.extend([1.; 6]);
    let mut b = BSpline {
        c: vec![0.; (t.len() - 6) * d],
        t,
        d,
    };
    let mut system = Normal::new(0, b.m(), d);
    let mut target = vec![0.; d];
    for i in 0..n {
        for a in 0..d {
            target[a] = points[(axes[a], i)];
        }
        let (start, v) = b.values(s[i]);
        system.add(start, &v, &target, 1., &b.c);
    }
    target.fill(0.);
    for (x, h) in [(0., s[1] - s[0]), (1., s[n - 1] - s[n - 2])] {
        let span = b.span(x);
        let power = b.powers(span, x);
        for r in 1..=2 {
            let v = std::array::from_fn(|i| power[i][r] * FACT[r] * h.powi(r as i32));
            system.add(span - P, &v, &target, 1., &b.c);
        }
    }
    b.c = system.solve()?;
    Ok(b.polynomial())
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::DMatrix;

    #[test]
    fn basis_partition_derivatives_and_insertion() {
        let mut t = vec![0.; 6];
        t.extend([0.01, 0.03, 0.14, 0.39, 0.43, 0.88]);
        t.extend([1.; 6]);
        let c: Vec<_> = (0..t.len() - 6).map(|i| (i as f64 * 1.73).sin()).collect();
        let mut b = BSpline { t, c, d: 1 };
        let before = b.polynomial();
        for i in 0..101 {
            let x = i as f64 / 100.;
            let (start, v) = b.values(x);
            let power = b.powers(start + P, x);
            assert!((v.iter().sum::<f64>() - 1.).abs() < 1e-12);
            for j in 0..6 {
                assert!((v[j] - power[j][0]).abs() < 1e-12);
            }
            for r in 1..6 {
                let sum = power.iter().map(|v| v[r]).sum::<f64>();
                let scale = power.iter().map(|v| v[r].abs()).sum::<f64>().max(1.);
                assert!(sum.abs() < 1e-12 * scale);
            }
        }
        for x in [0.005, 0.2, 0.5, 0.99] {
            b.insert(x);
        }
        let after = b.polynomial();
        for i in 0..301 {
            let x = i as f64 / 300.;
            let a = before.at(before.span(x), x, 0);
            let z = after.at(after.span(x), x, 0);
            for r in 0..4 {
                assert!((a[r] - z[r]).abs() < 1e-7 * (1. + a[r].abs()));
            }
        }
    }

    #[test]
    fn audit_sees_interior_extrema_and_polynomial_continuity() {
        // Endpoints are zero; a point-only audit would accept this curve.
        let p = Polynomial {
            breaks: vec![0., 1.],
            coefficients: vec![[0., 4., -4., 0., 0., 0.]],
            d: 1,
        };
        let mut bounds = vec![0.];
        let mut witness = vec![0.];
        audit(&p, &[0., 1.], &[0., 0.], 0, 1, &mut bounds, &mut witness).unwrap();
        assert!(bounds[0] >= 1.);
        let mut t = vec![0.; 6];
        t.extend([0.11, 0.37, 0.51, 0.91]);
        t.extend([1.; 6]);
        let b = BSpline {
            c: (0..t.len() - 6).map(|i| (i as f64).cos()).collect(),
            t,
            d: 1,
        };
        let poly = b.polynomial();
        for i in 1..poly.segments() {
            let left = &poly.coefficients[i - 1];
            let right = &poly.coefficients[i];
            let hl = poly.breaks[i] - poly.breaks[i - 1];
            let hr = poly.breaks[i + 1] - poly.breaks[i];
            for r in 0..=4 {
                let a = (r..6).map(|k| left[k] * FACT[k] / FACT[k - r]).sum::<f64>()
                    / hl.powi(r as i32);
                let b = right[r] * FACT[r] / hr.powi(r as i32);
                assert!((a - b).abs() < 1e-8 * (1. + a.abs()));
            }
        }
    }

    #[test]
    fn banded_solver_matches_dense_cholesky() {
        let n = 17;
        let mut h = Normal::new(0, n, 2);
        let mut dense = DMatrix::zeros(n, n);
        for i in 0..n {
            h.h[i][0] = 9.;
            dense[(i, i)] = 9.;
            for k in 1..=P.min(i) {
                let value = 0.2 / (k as f64);
                h.h[i][k] = value;
                dense[(i, i - k)] = value;
                dense[(i - k, i)] = value;
            }
            h.rhs[2 * i] = (i as f64).sin();
            h.rhs[2 * i + 1] = (i as f64).cos();
        }
        let rhs = DMatrix::from_fn(n, 2, |i, a| h.rhs[2 * i + a]);
        let expected = dense.cholesky().unwrap().solve(&rhs);
        let actual = h.solve().unwrap();
        for i in 0..n {
            for a in 0..2 {
                assert!((actual[2 * i + a] - expected[(i, a)]).abs() < 1e-13);
            }
        }
    }

    #[test]
    fn constructed_curve_error_checked_at_quartic_stationary_points() {
        let points = DMatrix::from_row_slice(2, 4, &[0., 1., 1., 2., 0., 0., 1., 1.]);
        let cfg = SmoothingConfig::default();
        let fitted = SmoothedSpline::build(points.as_view(), &cfg).unwrap();
        let p = &fitted.selected;
        let samples: [f64; 4] = [0., 1. / 3., 2. / 3., 1.];
        let mut actual = [0_f64; 2];
        for i in 0..3 {
            for span in 0..p.segments() {
                let a = p.breaks[span];
                let h = p.breaks[span + 1] - a;
                let left = samples[i].max(a);
                let right = samples[i + 1].min(a + h);
                if right <= left {
                    continue;
                }
                for axis in 0..2 {
                    let slope =
                        (points[(axis, i + 1)] - points[(axis, i)]) / (samples[i + 1] - samples[i]);
                    let mut error = p.coefficients[span * 2 + axis];
                    error[0] -= points[(axis, i)] + slope * (a - samples[i]);
                    error[1] -= slope * h;
                    let roots = roots::find_roots_quartic(
                        5. * error[5],
                        4. * error[4],
                        3. * error[3],
                        2. * error[2],
                        error[1],
                    );
                    let lo = (left - a) / h;
                    let hi = (right - a) / h;
                    for u in [lo, hi].iter().chain(roots.as_ref().iter()) {
                        if *u < lo || *u > hi {
                            continue;
                        }
                        let value = error.iter().rev().fold(0., |v, c| v * u + c);
                        actual[axis] = actual[axis].max(value.abs());
                    }
                }
            }
        }
        for (axis, &actual) in actual.iter().enumerate() {
            assert!(actual <= 0.001);
            assert!(actual <= fitted.report.max_errors[axis] + 1e-10);
        }
    }
}
