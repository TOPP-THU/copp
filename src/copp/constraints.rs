//! Constraint storage and access layer for TOPP/COPP pipelines.
//!
//! # Method identity
//! This module provides a circular-buffer based constraint container shared by:
//! - **TOPP2 / COPP2** (first-order + second-order constraints),
//! - **TOPP3 / COPP3** (plus nonlinear / linearized third-order constraints).
//!
//! # Data model (math + discrete code view)
//! Continuous/discrete state definition:
//! - $a(s) = \dot{s}^2$;
//! - $b(s) = \ddot{s} = \frac{1}{2}\frac{\mathrm{d}a}{\mathrm{d}s}$;
//! - $c(s) = \frac{\dddot{s}}{\dot{s}} = \frac{\mathrm{d}b}{\mathrm{d}s}$.
//!
//! Continuous/discrete state definition at station $s_k$:
//! - $a_k = \dot{s}_k^2$;
//! - $b_k = \ddot{s}_k$;
//! - $c_k = \frac{\dddot{s}_k}{\dot{s}_k}$.
//!
//! Discrete code symbols in this module:
//! - `a[k]` corresponds to $a_k$,
//! - `b[k]` corresponds to $b_k$,
//! - `c[k]` corresponds to $c_k$.
//!
//! Constraint families:
//! - first-order rows: $a(s) \le a\_{\text{max}}(s)$;
//! - second-order rows: $f\_a(s) a(s) + f\_b(s) b(s) \le f\_{\text{max}}(s)$;
//! - third-order rows: $\sqrt{a(s)}(g\_a(s) a(s) + g\_b(s) b(s) + g\_c(s) c(s) + g\_d(s)) \le g\_{\text{max}}(s)$;
//! - linearized third-order rows: $h\_a(s) a(s) + h\_b(s) b(s) + h\_c(s) c(s) \le h\_{\text{max}}(s)$.
//!
//! # API layering
//! - Public safe getters `get_*` (e.g. [`get_s`](Constraints::get_s), [`get_acc_constraints`](Constraints::get_acc_constraints), [`get_jerk_constraints`](Constraints::get_jerk_constraints))
//!   return `Result<_, ConstraintError>` with explicit bounds contract.
//! - Internal fast getters `*_unchecked` are `pub(crate)` and require caller-side precondition guarantees.
//!
//! # User guidance
//! - For most users, prefer [`Robot`](crate::robot::Robot) as the entry point so
//!   constraints can be expressed with physical semantics ([`with_axial_velocity`](crate::robot::Robot::with_axial_velocity),
//!   [`with_axial_acceleration`](crate::robot::Robot::with_axial_acceleration), torque-related APIs).
//! - Direct manipulation of [`Constraints`](crate::constraints::Constraints) is recommended for advanced users who
//!   need maximum flexibility and custom low-level constraint composition.
//!
//! # Contract summary
//! - Public APIs validate station range before indexing.
//! - Internal unchecked APIs are for hot paths and guarded by debug assertions.
//! - Linearized jerk access requires builders to call
//!   [`build_with_linearization(Topp3)`](crate::solver::topp3_socp::Topp3ProblemBuilder::build_with_linearization) or
//!   [`build_with_linearization(Copp3)`](crate::solver::copp3_socp::Copp3ProblemBuilder::build_with_linearization) beforehand.
//! - For robust solver behavior, keep zero-state `a=b=c=0` strictly feasible at every station.
//!   In practice this means every active scalar RHS must stay strictly positive:
//!   `amax > 0`, `acc_max > 0`, and `jerk_max > 0` (after sign normalization).

use crate::copp::copp2::stable::basic::a_to_b_topp2;
use crate::diag::{ConstraintError, CoppError};
use crate::path::Path;
use core::f64;
use itertools::{Itertools, izip};
use nalgebra::{Const, DMatrix, DMatrixView, Dyn, Matrix, RowDVector, ViewStorage};
use std::cmp::{max, min};
use std::collections::BTreeMap;
use std::ops::Bound::{Excluded, Included, Unbounded};

/// Small numerical threshold used by feasibility and bound computations.
///
/// The value is intentionally conservative and only serves as a tolerance
/// around near-zero comparisons (for example, branch decisions on inequality
/// coefficients). It is **not** a global optimization tolerance.
const EPSILON_NUMERIC: f64 = 1E-10;

/// Constraint storage and query object used by TOPP/COPP solvers.
///
/// # Mathematical symbols (with code mapping)
/// For each path station $s_k$:
/// - $a_k = \dot{s}_k^2$ (squared path speed), mapped to code symbol `a[k]`.
/// - $b_k = \ddot{s}_k = \frac{1}{2}\frac{\mathrm{d}a}{\mathrm{d}s}(s_k)$ (path acceleration), mapped to `b[k]`.
/// - $c_k = \frac{\dddot{s}_k}{\dot{s}_k} = \frac{\mathrm{d}b}{\mathrm{d}s}(s_k)$ (normalized jerk term), mapped to `c[k]`.
///
/// # Constraint families
/// - First-order: `0 <= a[k] <= amax[k]`
/// - Second-order: `acc_a[k]*a[k] + acc_b[k]*b[k] <= acc_max[k]`
/// - Third-order (nonlinear):
///   `sqrt(a[k])*(jerk_a[k]*a[k] + jerk_b[k]*b[k] + jerk_c[k]*c[k] + jerk_d[k]) <= jerk_max[k]`
/// - Third-order (linearized):
///   `jerk_a_linear[k]*a[k] + jerk_b[k]*b[k] + jerk_c[k]*c[k] <= jerk_max_linear[k]`
///
/// # Storage model
/// All station-wise arrays are stored as circular column-major matrices. Logical
/// station index range is `[idx_s, idx_s + len)`, and logical column `i` maps to
/// physical column `(head_col + i) % capacity_col`.
///
/// # API contract
/// - `get_*` methods are safe public accessors and return `Result<_, ConstraintError>`.
/// - `*_unchecked` methods are internal fast-path helpers. Callers must satisfy
///   preconditions; debug builds assert them.
///
/// # Example
/// The example below constructs low-level constraints directly, without going
/// through [`Robot`](crate::robot::Robot).
///
/// ```rust
/// # fn main() -> Result<(), copp::diag::CoppError> {
/// use copp::constraints::Constraints;
/// use nalgebra::DMatrix;
///
/// let mut constraints = Constraints::with_capacity(2, 8);
///
/// let s = [0.0, 0.5, 1.0];
/// constraints.with_s(s.as_slice())?;
///
/// let amax = [1.0, 0.8, 1.0];
/// constraints.with_constraint_1order(amax.as_slice(), 0)?;
///
/// let acc_a = DMatrix::from_row_slice(2, 3, &[
///     0.0, 0.0, 0.0,
///     0.0, 0.0, 0.0,
/// ]);
/// let acc_b = DMatrix::from_row_slice(2, 3, &[
///     1.0, 1.0, 1.0,
///     -1.0, -1.0, -1.0,
/// ]);
/// let acc_max = DMatrix::from_row_slice(2, 3, &[
///     2.0, 2.0, 2.0,
///     2.0, 2.0, 2.0,
/// ]);
///
/// constraints.with_constraint_2order(
///     &acc_a.as_view(),
///     &acc_b.as_view(),
///     &acc_max.as_view(),
///     0,
///     false,
/// )?;
///
/// assert_eq!(constraints.len(), 3);
/// assert_eq!(constraints.get_s(1)?, 0.5);
/// # Ok(())
/// # }
/// ```
#[derive(Clone)]
pub struct Constraints {
    /// Allocated circular-buffer capacity in **columns**.
    pub(crate) capacity_col: usize,
    /// Path dimension / degrees of freedom (`DoF`).
    dim: usize,
    /// Path station grid values (`1 x capacity_col`).
    s: DMatrix<f64>,
    /// Configuration values `q(s)` (`dim x capacity_col`).
    pub(crate) q: DMatrix<f64>,
    /// First derivative `dq/ds` (`dim x capacity_col`).
    pub(crate) dq: DMatrix<f64>,
    /// Second derivative `d2q/ds2` (`dim x capacity_col`).
    pub(crate) ddq: DMatrix<f64>,
    /// Third derivative `d3q/ds3` (`dim x capacity_col`).
    pub(crate) dddq: DMatrix<f64>,
    /// First-order upper bound `amax` (`1 x capacity_col`).
    amax: DMatrix<f64>,
    /// Second-order coefficient `acc_a`.
    acc_a: DMatrix<f64>,
    /// Second-order coefficient `acc_b`.
    acc_b: DMatrix<f64>,
    /// Second-order right-hand side bound.
    acc_max: DMatrix<f64>,
    /// Nonlinear third-order coefficient for `a`.
    jerk_a: DMatrix<f64>,
    /// Nonlinear third-order coefficient for `b`.
    jerk_b: DMatrix<f64>,
    /// Nonlinear third-order coefficient for `c`.
    jerk_c: DMatrix<f64>,
    /// Nonlinear third-order constant term.
    jerk_d: DMatrix<f64>,
    /// Nonlinear third-order right-hand side bound.
    jerk_max: DMatrix<f64>,
    /// Linearized third-order coefficient for `a`.
    jerk_a_linear: DMatrix<f64>,
    /// Linearized third-order right-hand side bound.
    jerk_max_linear: DMatrix<f64>,
    /// Piecewise-constant valid row counts for `q`, `dq`, `ddq`.
    valid_rows_q: ValidRows,
    /// Piecewise-constant valid row counts for `dddq`.
    valid_rows_dddq: ValidRows,
    /// Piecewise-constant valid row counts for second-order constraints.
    valid_rows_acc: ValidRows,
    /// Piecewise-constant valid row counts for third-order constraints.
    valid_rows_jerk: ValidRows,
    /// Valid station-id interval for linearized jerk constraints `[left, right)`.
    valid_ids_linear_jerk: (usize, usize),
    /// Floor applied to `a` before evaluating `1/sqrt(a)` in third-order constraints.
    ///
    /// Set by [`Constraints::linearize_constraint_3order_with_floor`] and read back by
    /// [`Constraints::stationary_constraint_topp3`], so that both paths floor the
    /// *same* quantity. Flooring `sqrt(a)` instead of `a` squares the effective
    /// floor and inflates `a^{-3/2}` by the square of that factor.
    a_linearization_floor: f64,
    /// Physical column in circular buffer that corresponds to logical offset `0`.
    head_col: usize,
    /// Number of valid logical columns currently stored.
    len: usize,
    /// Global station id of the first logical column (`head_col`).
    idx_s: usize,
}

/// Piecewise-constant row-validity map.
///
/// - Key (`usize`): right boundary `idx_s_right` (exclusive upper station id).
/// - Value: `(idx_s_left, n_rows)` meaning stations in
///   `[idx_s_left, idx_s_right)` have exactly `n_rows` valid rows.
///
/// The map is maintained as a contiguous partition without overlaps.
type ValidRows = BTreeMap<usize, (usize, usize)>;

/// Borrowed 2D matrix view type accepted by constraint-ingestion APIs.
///
/// Internally this is a dynamic nalgebra matrix view with column stride support,
/// allowing callers to pass slices, vectors, matrix columns, or full views
/// without allocation.
pub type InputMatrix<'a> = Matrix<f64, Dyn, Dyn, ViewStorage<'a, f64, Dyn, Dyn, Const<1>, Dyn>>;

/// Conversion helper trait for 1D-like first-order inputs.
///
/// This trait normalizes different containers into a `1 x N` [`InputMatrix`](crate::constraints::InputMatrix) view.
/// It is used by APIs such as `with_s()` and `with_constraint_1order()`.
pub trait AsInputMatrix1D {
    /// Borrow input data as a `1 x N` matrix view.
    fn as_input_matrix(&self) -> InputMatrix<'_>;
}

impl AsInputMatrix1D for InputMatrix<'_> {
    fn as_input_matrix(&self) -> InputMatrix<'_> {
        *self
    }
}

impl AsInputMatrix1D for [f64] {
    fn as_input_matrix(&self) -> InputMatrix<'_> {
        DMatrixView::from_slice(self, 1, self.len())
    }
}

impl AsInputMatrix1D for Vec<f64> {
    fn as_input_matrix(&self) -> InputMatrix<'_> {
        DMatrixView::from_slice(self, 1, self.len())
    }
}

/// Linearization point of one third-order row at one station.
///
/// See [`Constraints::linearize_constraint_3order_adaptive`] for the derivation.
/// `None` asks the caller to keep the uniform anchor: a non-finite `jerk_max` is
/// dropped downstream anyway, and a non-finite `amax` would let the anchor run to
/// infinity, which collapses the row to `value <= -jerk_d`.
#[inline]
fn adaptive_anchor_3order(
    row: (f64, f64, f64, f64, f64),
    state: (f64, f64, f64),
    amax: f64,
) -> Option<f64> {
    let (jerk_a, jerk_b, jerk_c, jerk_d, jerk_max) = row;
    let (a_lin, b_lin, c_lin) = state;
    if !jerk_max.is_finite() || !amax.is_finite() {
        return None;
    }
    let value = jerk_a * a_lin + jerk_b * b_lin + jerk_c * c_lin + jerk_d;
    if value <= 0.0 {
        // Slack at the reference state whatever `a` is, and it stays slack for
        // every anchor at or above `a_lin`.  Anchoring high is what removes the
        // `a <= 3 * a_lin` cap this row would otherwise contribute.
        return Some(amax);
    }
    // `a_active` is the `a` at which the row becomes active with `(b, c)` frozen,
    // so `a_lin <= a_active` is exactly "the reference state satisfies the
    // nonlinear row".  Anchors keeping that property form an interval spanned by
    // the two, and the geometric mean sits inside it.
    let a_active_sqrt = jerk_max / value;
    let a_active = a_active_sqrt * a_active_sqrt;
    if a_lin <= 0.0 {
        // The geometric mean degenerates to zero; `a_active` stays admissible
        // because the reference state then satisfies the row for every anchor up
        // to `2.25 * a_active`.
        return Some(a_active.min(amax));
    }
    Some((a_lin.sqrt() * a_active_sqrt).min(amax))
}

impl Constraints {
    /// Default number of constraint stations preallocated for a new container.
    pub const DEFAULT_CAPACITY: usize = 1000;

    /// Construct a new container with default column capacity.
    ///
    /// # Parameters
    /// - `dim`: path dimension / DoF.
    ///
    /// # Notes
    /// Equivalent to `with_capacity(dim, DEFAULT_CAPACITY)`.
    pub fn new(dim: usize) -> Self {
        Self::with_capacity(dim, Self::DEFAULT_CAPACITY)
    }

    /// Construct a new container with explicit column capacity.
    ///
    /// # Parameters
    /// - `dim`: path dimension / DoF.
    /// - `capacity_col`: initial number of allocated columns.
    ///
    /// # Initialization policy
    /// - Bound matrices are initialized to neutral values (`infinity` where applicable).
    /// - Valid-row maps start empty and are progressively populated by `with_s()`.
    pub fn with_capacity(dim: usize, capacity_col: usize) -> Self {
        Constraints {
            capacity_col,
            dim,
            s: DMatrix::<f64>::zeros(1, capacity_col),
            q: DMatrix::<f64>::zeros(dim, capacity_col),
            dq: DMatrix::<f64>::zeros(dim, capacity_col),
            ddq: DMatrix::<f64>::zeros(dim, capacity_col),
            dddq: DMatrix::<f64>::zeros(dim, capacity_col),
            amax: DMatrix::<f64>::from_element(1, capacity_col, f64::INFINITY),
            acc_a: DMatrix::<f64>::zeros(2 * dim, capacity_col),
            acc_b: DMatrix::<f64>::zeros(2 * dim, capacity_col),
            acc_max: DMatrix::<f64>::from_element(2 * dim, capacity_col, f64::INFINITY),
            jerk_a: DMatrix::<f64>::zeros(2 * dim, capacity_col),
            jerk_b: DMatrix::<f64>::zeros(2 * dim, capacity_col),
            jerk_c: DMatrix::<f64>::zeros(2 * dim, capacity_col),
            jerk_d: DMatrix::<f64>::zeros(2 * dim, capacity_col),
            jerk_max: DMatrix::<f64>::from_element(2 * dim, capacity_col, f64::INFINITY),
            jerk_a_linear: DMatrix::<f64>::zeros(2 * dim, capacity_col),
            jerk_max_linear: DMatrix::<f64>::from_element(2 * dim, capacity_col, f64::INFINITY),
            valid_rows_acc: BTreeMap::new(),
            valid_rows_jerk: BTreeMap::new(),
            valid_rows_q: BTreeMap::new(),
            valid_rows_dddq: BTreeMap::new(),
            valid_ids_linear_jerk: (0, 0),
            a_linearization_floor: EPSILON_NUMERIC,
            head_col: 0,
            len: 0,
            idx_s: 0,
        }
    }

    /// Calculate the physical column index in the circular buffer given a logical offset `col`.
    /// This uses `head_col` as the memory offset (bias) for the circular buffer.
    #[inline(always)]
    fn idx(&self, col: usize) -> usize {
        (self.head_col + col) % self.capacity_col
    }

    /// Current number of logical stations stored in the buffer.
    #[inline(always)]
    pub fn len(&self) -> usize {
        self.len
    }

    /// Path dimension / DoF.
    #[inline(always)]
    pub(crate) fn dim(&self) -> usize {
        self.dim
    }

    /// Allocated circular-buffer capacity in columns.
    #[inline(always)]
    pub(crate) fn capacity(&self) -> usize {
        self.capacity_col
    }

    /// Whether no logical stations are currently stored.
    #[inline(always)]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Get the column index in the circular buffer for a given `idx_s` in the original path.
    ///
    /// # Preconditions
    /// Caller must guarantee `idx_s` is within `[self.idx_s, self.idx_s + self.len)`.
    #[inline(always)]
    pub(crate) fn col_at_idx_s_unchecked(&self, idx_s: usize) -> usize {
        debug_assert!(
            self.check_s_in_bounds(idx_s, 1).is_ok(),
            "col_at_idx_s_unchecked called with out-of-bounds idx_s={idx_s}"
        );
        self.idx(idx_s - self.idx_s)
    }

    /// Get the path s value at index `idx_s` without bounds checking.
    ///
    /// # Preconditions
    /// Caller must guarantee `idx_s` is within `[self.idx_s, self.idx_s + self.len)`.
    #[inline(always)]
    pub(crate) fn s_unchecked(&self, idx_s: usize) -> f64 {
        debug_assert!(
            self.check_s_in_bounds(idx_s, 1).is_ok(),
            "s_unchecked called with out-of-bounds idx_s={idx_s}"
        );
        self.s[(0, self.col_at_idx_s_unchecked(idx_s))]
    }

    /// Get station value `s[idx_s]` with bounds validation.
    ///
    /// # Errors
    /// Returns [`ConstraintError::OutOfSBounds`](crate::diag::ConstraintError::OutOfSBounds) if `idx_s` is outside
    /// `[idx_s_start(), idx_s_end())`.
    #[inline(always)]
    pub fn get_s(&self, idx_s: usize) -> Result<f64, ConstraintError> {
        self.check_s_in_bounds(idx_s, 1)?;
        Ok(self.s_unchecked(idx_s))
    }

    /// Number of rows currently allocated for first-order constraints.
    ///
    /// Normally this is `1`, but the method is intentionally generic.
    #[inline(always)]
    pub fn amax_rows(&self) -> usize {
        self.amax.nrows()
    }

    /// Number of rows currently allocated for second-order constraints.
    #[inline(always)]
    pub fn acc_rows(&self) -> usize {
        self.acc_a.nrows()
    }

    /// Number of rows currently allocated for third-order constraints.
    #[inline(always)]
    pub fn jerk_rows(&self) -> usize {
        self.jerk_a.nrows()
    }

    /// Export station values in half-open interval `[idx_s_from, idx_s_to)`.
    ///
    /// # Parameters
    /// - `idx_s_from`: global start station id (inclusive).
    /// - `idx_s_to`: global end station id (exclusive).
    ///
    /// # Returns
    /// A contiguous vector of station values with length `idx_s_to - idx_s_from`.
    ///
    /// # Errors
    /// - [`ConstraintError::EmptyInterval`](crate::diag::ConstraintError::EmptyInterval) when `idx_s_from >= idx_s_to`.
    /// - [`ConstraintError::OutOfSBounds`](crate::diag::ConstraintError::OutOfSBounds) if the interval is outside stored data.
    pub fn s_vec(&self, idx_s_from: usize, idx_s_to: usize) -> Result<Vec<f64>, ConstraintError> {
        if idx_s_from >= idx_s_to {
            return Err(ConstraintError::EmptyInterval {
                start: idx_s_from,
                end: idx_s_to,
            });
        }
        self.check_s_in_bounds(idx_s_from, idx_s_to - idx_s_from)?;
        let s_raw_slice = self.s.as_slice();
        let start_idx = self.idx(idx_s_from - self.idx_s);
        let ncols_mat = self.s.ncols();
        let ncols_data = idx_s_to - idx_s_from;
        let mut result = vec![0.0; ncols_data];
        let func = |start_idx_: usize, ncols_: usize, offset: usize| {
            result[offset..(offset + ncols_)]
                .copy_from_slice(&s_raw_slice[start_idx_..(start_idx_ + ncols_)]);
        };
        Self::circular_process(ncols_mat, start_idx, ncols_data, func);
        Ok(result)
    }

    /// Export first-order upper bounds in `[idx_s_from, idx_s_to)`.
    ///
    /// Semantics and error behavior are identical to `s_vec()`.
    pub fn amax_vec(
        &self,
        idx_s_from: usize,
        idx_s_to: usize,
    ) -> Result<Vec<f64>, ConstraintError> {
        if idx_s_from >= idx_s_to {
            return Err(ConstraintError::EmptyInterval {
                start: idx_s_from,
                end: idx_s_to,
            });
        }
        self.check_s_in_bounds(idx_s_from, idx_s_to - idx_s_from)?;
        let amax_raw_slice = self.amax.as_slice();
        let start_idx = self.idx(idx_s_from - self.idx_s);
        let ncols_mat = self.amax.ncols();
        let ncols_data = idx_s_to - idx_s_from;
        let mut result = vec![0.0; ncols_data];
        let func = |start_idx_: usize, ncols_: usize, offset: usize| {
            result[offset..(offset + ncols_)]
                .copy_from_slice(&amax_raw_slice[start_idx_..(start_idx_ + ncols_)]);
        };
        Self::circular_process(ncols_mat, start_idx, ncols_data, func);
        Ok(result)
    }

    /// Get the amax value at index `idx_s` without bounds checking.
    ///
    /// # Preconditions
    /// Caller must guarantee `idx_s` is within `[self.idx_s, self.idx_s + self.len)`.
    #[inline(always)]
    pub(crate) fn amax_unchecked(&self, idx_s: usize) -> f64 {
        debug_assert!(
            self.check_s_in_bounds(idx_s, 1).is_ok(),
            "amax_unchecked called with out-of-bounds idx_s={idx_s}"
        );
        self.amax[(0, self.idx(idx_s - self.idx_s))]
    }

    /// Get first-order upper bound `amax[idx_s]` with bounds validation.
    ///
    /// # Errors
    /// Returns [`ConstraintError::OutOfSBounds`](crate::diag::ConstraintError::OutOfSBounds) if `idx_s` is invalid.
    #[inline(always)]
    pub fn get_amax(&self, idx_s: usize) -> Result<f64, ConstraintError> {
        self.check_s_in_bounds(idx_s, 1)?;
        Ok(self.amax_unchecked(idx_s))
    }

    /// Global start station id (inclusive) of current logical window.
    #[inline(always)]
    pub fn idx_s_start(&self) -> usize {
        self.idx_s
    }

    /// Global end station id (exclusive) of current logical window.
    #[inline(always)]
    pub fn idx_s_end(&self) -> usize {
        self.idx_s + self.len
    }

    /// Get the second-order constraints at index `idx_s` without bounds checking.
    ///
    /// # Preconditions
    /// Caller must guarantee `idx_s` is within `[self.idx_s, self.idx_s + self.len)`.
    pub(crate) fn acc_constraints_unchecked<'a>(
        &'a self,
        idx_s: usize,
    ) -> (InputMatrix<'a>, InputMatrix<'a>, InputMatrix<'a>) {
        debug_assert!(
            self.check_s_in_bounds(idx_s, 1).is_ok(),
            "acc_constraints_unchecked called with out-of-bounds idx_s={idx_s}"
        );
        // Find the valid_rows for idx_s
        let (_, &(_, valid_rows)) = self
            .valid_rows_acc
            .range((Excluded(idx_s), Unbounded))
            .next()
            .unwrap_or((&0, &(0, 0)));
        // Return the constraints
        let idx = self.idx(idx_s - self.idx_s);
        (
            self.acc_a.view((0, idx), (valid_rows, 1)),
            self.acc_b.view((0, idx), (valid_rows, 1)),
            self.acc_max.view((0, idx), (valid_rows, 1)),
        )
    }

    /// Get second-order row views at station `idx_s`.
    ///
    /// # Returns
    /// `(acc_a_col, acc_b_col, acc_max_col)` where each matrix is a `valid_rows x 1`
    /// view into internal storage.
    ///
    /// # Errors
    /// Returns [`ConstraintError::OutOfSBounds`](crate::diag::ConstraintError::OutOfSBounds) if `idx_s` is invalid.
    pub fn get_acc_constraints<'a>(
        &'a self,
        idx_s: usize,
    ) -> Result<(InputMatrix<'a>, InputMatrix<'a>, InputMatrix<'a>), ConstraintError> {
        self.check_s_in_bounds(idx_s, 1)?;
        Ok(self.acc_constraints_unchecked(idx_s))
    }

    /// Get the third-order constraints at index `idx_s` without bounds checking.
    ///
    /// # Preconditions
    /// Caller must guarantee `idx_s` is within `[self.idx_s, self.idx_s + self.len)`.
    pub(crate) fn jerk_constraints_unchecked<'a>(
        &'a self,
        idx_s: usize,
    ) -> (
        InputMatrix<'a>,
        InputMatrix<'a>,
        InputMatrix<'a>,
        InputMatrix<'a>,
        InputMatrix<'a>,
    ) {
        debug_assert!(
            self.check_s_in_bounds(idx_s, 1).is_ok(),
            "jerk_constraints_unchecked called with out-of-bounds idx_s={idx_s}"
        );
        // Find the valid_rows for idx_s
        let (_, &(_, valid_rows)) = self
            .valid_rows_jerk
            .range((Excluded(idx_s), Unbounded))
            .next()
            .unwrap_or((&0, &(0, 0)));
        // Return the constraints
        let idx = self.idx(idx_s - self.idx_s);
        (
            self.jerk_a.view((0, idx), (valid_rows, 1)),
            self.jerk_b.view((0, idx), (valid_rows, 1)),
            self.jerk_c.view((0, idx), (valid_rows, 1)),
            self.jerk_d.view((0, idx), (valid_rows, 1)),
            self.jerk_max.view((0, idx), (valid_rows, 1)),
        )
    }

    /// Get nonlinear third-order row views at station `idx_s`.
    ///
    /// # Returns
    /// `(jerk_a, jerk_b, jerk_c, jerk_d, jerk_max)`, each a `valid_rows x 1` view.
    ///
    /// # Errors
    /// Returns [`ConstraintError::OutOfSBounds`](crate::diag::ConstraintError::OutOfSBounds) if `idx_s` is invalid.
    pub fn get_jerk_constraints<'a>(
        &'a self,
        idx_s: usize,
    ) -> Result<
        (
            InputMatrix<'a>,
            InputMatrix<'a>,
            InputMatrix<'a>,
            InputMatrix<'a>,
            InputMatrix<'a>,
        ),
        ConstraintError,
    > {
        self.check_s_in_bounds(idx_s, 1)?;
        Ok(self.jerk_constraints_unchecked(idx_s))
    }

    /// Get the linearized third-order constraints at index `idx_s` without bounds checking.
    ///
    /// This accessor assumes linearized jerk constraints are prepared via
    /// [`Topp3ProblemBuilder::build_with_linearization`](crate::solver::topp3_socp::Topp3ProblemBuilder::build_with_linearization)
    /// or [`Copp3ProblemBuilder::build_with_linearization`](crate::solver::copp3_socp::Copp3ProblemBuilder::build_with_linearization).
    ///
    /// # Preconditions
    /// Caller must guarantee:
    /// - `idx_s` is within `[self.idx_s, self.idx_s + self.len)`
    /// - `idx_s` is within `self.valid_ids_linear_jerk`
    pub(crate) fn jerk_linear_constraints_unchecked<'a>(
        &'a self,
        idx_s: usize,
    ) -> (
        InputMatrix<'a>,
        InputMatrix<'a>,
        InputMatrix<'a>,
        InputMatrix<'a>,
    ) {
        debug_assert!(
            self.check_s_in_bounds(idx_s, 1).is_ok(),
            "jerk_linear_constraints_unchecked called with out-of-bounds idx_s={idx_s}"
        );
        debug_assert!(
            idx_s >= self.valid_ids_linear_jerk.0 && idx_s < self.valid_ids_linear_jerk.1,
            "jerk_linear_constraints_unchecked called outside linearized range: idx_s={idx_s}, valid=[{}, {})",
            self.valid_ids_linear_jerk.0,
            self.valid_ids_linear_jerk.1
        );
        // Find the valid_rows for idx_s
        let (_, &(_, valid_rows)) = self
            .valid_rows_jerk
            .range((Excluded(idx_s), Unbounded))
            .next()
            .unwrap_or((&0, &(0, 0)));
        // Return the constraints
        let idx = self.idx(idx_s - self.idx_s);
        (
            self.jerk_a_linear.view((0, idx), (valid_rows, 1)),
            self.jerk_b.view((0, idx), (valid_rows, 1)),
            self.jerk_c.view((0, idx), (valid_rows, 1)),
            self.jerk_max_linear.view((0, idx), (valid_rows, 1)),
        )
    }

    /// Get linearized third-order row views at station `idx_s`.
    ///
    /// The linearized rows are prepared by
    /// [`Topp3ProblemBuilder::build_with_linearization`](crate::solver::topp3_socp::Topp3ProblemBuilder::build_with_linearization)
    /// or [`Copp3ProblemBuilder::build_with_linearization`](crate::solver::copp3_socp::Copp3ProblemBuilder::build_with_linearization).
    /// Call this accessor when solver-side code needs to inspect the affine
    /// third-order rows generated from the latest reference profile.
    ///
    /// # Returns
    /// `(jerk_a_linear, jerk_b, jerk_c, jerk_max_linear)`, each a `valid_rows x 1`
    /// view into internal storage.
    ///
    /// # Errors
    /// - [`ConstraintError::OutOfSBounds`](crate::diag::ConstraintError::OutOfSBounds) if `idx_s` is outside current station window.
    /// - [`ConstraintError::LinearJerkNotAvailable`](crate::diag::ConstraintError::LinearJerkNotAvailable) if `idx_s` is not covered by the
    ///   latest linearization interval.
    pub fn get_jerk_linear_constraints<'a>(
        &'a self,
        idx_s: usize,
    ) -> Result<
        (
            InputMatrix<'a>,
            InputMatrix<'a>,
            InputMatrix<'a>,
            InputMatrix<'a>,
        ),
        ConstraintError,
    > {
        self.check_s_in_bounds(idx_s, 1)?;
        if idx_s < self.valid_ids_linear_jerk.0 || idx_s >= self.valid_ids_linear_jerk.1 {
            return Err(ConstraintError::LinearJerkNotAvailable {
                idx_s,
                valid_range: self.valid_ids_linear_jerk,
            });
        }
        Ok(self.jerk_linear_constraints_unchecked(idx_s))
    }

    /// Ensure buffer capacity is at least `new_capacity` columns.
    ///
    /// # Growth strategy
    /// If expansion is required, target capacity is
    /// `max(new_capacity, 2 * current_capacity + 1)`.
    ///
    /// # Guarantees
    /// - Logical order of existing data is preserved.
    /// - `head_col` is reset to `0` after re-layout.
    /// - All backing matrices (`s`, derivative buffers, and constraint buffers)
    ///   are expanded consistently.
    #[inline(always)]
    pub fn expand_capacity(&mut self, new_capacity: usize) {
        if new_capacity <= self.capacity_col {
            return;
        }
        let new_capacity = max(new_capacity, 2 * self.capacity_col + 1);

        // Copy data from old matrices to new matrices.
        let id_from = self.head_col;
        let len_copy = min(self.len, self.capacity_col - self.head_col);
        let matrices_to_move = [
            (&mut self.s, 0.0),
            (&mut self.q, 0.0),
            (&mut self.dq, 0.0),
            (&mut self.ddq, 0.0),
            (&mut self.dddq, 0.0),
            (&mut self.amax, f64::INFINITY),
            (&mut self.acc_a, 0.0),
            (&mut self.acc_b, 0.0),
            (&mut self.acc_max, f64::INFINITY),
            (&mut self.jerk_a, 0.0),
            (&mut self.jerk_b, 0.0),
            (&mut self.jerk_c, 0.0),
            (&mut self.jerk_d, 0.0),
            (&mut self.jerk_max, f64::INFINITY),
            (&mut self.jerk_a_linear, 0.0),
            (&mut self.jerk_max_linear, f64::INFINITY),
        ];
        for (mat, default_val) in matrices_to_move.into_iter() {
            let mut new_mat = DMatrix::<f64>::from_element(mat.nrows(), new_capacity, default_val);
            // Copy the data in two parts to handle the circular buffer wrap-around.
            new_mat
                .columns_mut(0, len_copy)
                .copy_from(&mat.columns(id_from, len_copy));
            // Copy the remaining part if needed.
            if len_copy < self.len {
                new_mat
                    .columns_mut(len_copy, self.len - len_copy)
                    .copy_from(&mat.columns(0, self.len - len_copy));
            }
            *mat = new_mat;
        }

        self.capacity_col = new_capacity;
        self.head_col = 0;
    }

    /// Append strictly increasing station samples to the logical tail.
    ///
    /// # Parameters
    /// - `s_new`: a `1 x N` station segment; accepted via [`AsInputMatrix1D`](crate::constraints::AsInputMatrix1D).
    ///
    /// For the higher-level robot wrapper, [`Robot::with_s`](crate::robot::Robot::with_s)
    /// delegates to this method.
    ///
    /// # Behavior
    /// - Rejects non-increasing input.
    /// - Rejects overlap with existing tail station (`s_new[0]` must be greater
    ///   than current last station when the buffer is non-empty).
    /// - Expands capacity proactively.
    /// - Appends zero-valid-row segments into all validity maps so downstream
    ///   `with_q` / `with_constraint_*` calls can progressively fill data.
    ///
    /// # Errors
    /// Returns [`ConstraintError::NonIncreasingS`](crate::diag::ConstraintError::NonIncreasingS) on monotonicity violations.
    ///
    /// # Returns
    /// Returns `&mut Self` for chaining on success.
    pub fn with_s<T: AsInputMatrix1D + ?Sized>(
        &mut self,
        s_new: &T,
    ) -> Result<&mut Self, ConstraintError> {
        let s_new = s_new.as_input_matrix();
        if s_new.ncols() == 0 {
            return Ok(self);
        }
        // Check if s_new[0] > self.s[last]
        if self.len > 0 && s_new[0] <= self.s[self.idx(self.len - 1)] {
            return Err(ConstraintError::NonIncreasingS { index: 0 });
        }
        // Check if s_new is strictly increasing
        let check_s_new_increasing = s_new
            .iter()
            .zip(s_new.iter().skip(1))
            .enumerate()
            .find(|(_, (prev, curr))| prev >= curr);
        if let Some((index, _)) = check_s_new_increasing {
            return Err(ConstraintError::NonIncreasingS { index });
        }
        // Expand capacity if needed
        self.expand_capacity(self.len + s_new.ncols() * 2);
        // Add new s values
        let start_idx = self.idx(self.len);
        Self::copy_from_matrix_to_cirmatrix(&mut self.s, &s_new, start_idx);
        // Update valid_rows
        Self::push_back_valid_rows(&mut self.valid_rows_q, s_new.ncols(), 0);
        Self::push_back_valid_rows(&mut self.valid_rows_dddq, s_new.ncols(), 0);
        Self::push_back_valid_rows(&mut self.valid_rows_acc, s_new.ncols(), 0);
        Self::push_back_valid_rows(&mut self.valid_rows_jerk, s_new.ncols(), 0);

        self.len += s_new.ncols();

        Ok(self)
    }

    /// Copy a dense matrix block into a circular matrix starting at `start_idx`.
    ///
    /// # Parameters
    /// - `mat`: destination circular matrix.
    /// - `data`: source matrix view (`rows x cols`).
    /// - `start_idx`: destination physical start column in `mat`.
    ///
    /// # Notes
    /// - Automatically grows destination row count if needed.
    /// - Handles wrap-around by splitting into at most two contiguous writes.
    fn copy_from_matrix_to_cirmatrix(mat: &mut DMatrix<f64>, data: &InputMatrix, start_idx: usize) {
        if data.ncols() == 0 {
            return;
        }
        if data.nrows() > mat.nrows() {
            mat.resize_vertically_mut(data.nrows(), 0.0);
        }
        let ncols_mat = mat.ncols();
        let func = |start_idx: usize, ncols: usize, offset: usize| {
            mat.view_mut((0, start_idx), (data.nrows(), ncols))
                .copy_from(&data.columns(offset, ncols));
        };
        Self::circular_process(ncols_mat, start_idx, data.ncols(), func);
    }

    /// Execute a callback over a circular column interval as up to two segments.
    ///
    /// # Parameters
    /// - `ncols_mat`: total columns in circular matrix.
    /// - `start_idx`: physical start column.
    /// - `ncols_data`: logical number of columns to process.
    /// - `func`: callback invoked as `(segment_start, segment_len, source_offset)`.
    ///
    /// # Contract
    /// - Exactly one callback if no wrap occurs.
    /// - Exactly two callbacks if wrap occurs.
    /// - `source_offset` is suitable for indexing source arrays/views.
    #[inline(always)]
    pub(crate) fn circular_process<F>(
        ncols_mat: usize,
        start_idx: usize,
        ncols_data: usize,
        mut func: F,
    ) where
        F: FnMut(usize, usize, usize),
    {
        if ncols_mat - start_idx >= ncols_data {
            // The range is within the current matrix
            func(start_idx, ncols_data, 0);
        } else {
            // The range is out of the current matrix
            let len_first = ncols_mat - start_idx;
            func(start_idx, len_first, 0);
            let len_second = ncols_data - len_first;
            func(0, len_second, len_first);
        }
    }

    /// Append per-column row blocks into an existing circular matrix region.
    ///
    /// # Intended usage
    /// This helper is used by `with_constraint_2order()` / `with_constraint_3order()`
    /// after `update_valid_rows()` has already increased row counts for the target
    /// station interval.
    ///
    /// # Parameters
    /// - `mat`: destination circular matrix containing stacked rows.
    /// - `valid_rows`: updated row-partition map for the destination interval.
    /// - `data`: newly added row block to be appended in each covered station.
    /// - `start_idx`: physical destination column for `start_idx_s`.
    /// - `start_idx_s`: global station id corresponding to `start_idx`.
    /// - `is_negative`: if `true`, copied block is sign-flipped after insertion.
    ///
    /// # Layout rule
    /// Newly inserted rows are written at the bottom of each station column, i.e.
    /// row range `[num_valid_rows - data.nrows(), num_valid_rows)`.
    fn concat_from_matrix_to_cirmatrix(
        mat: &mut DMatrix<f64>,
        valid_rows: &ValidRows,
        data: &InputMatrix,
        start_idx: usize,
        start_idx_s: usize,
        is_negative: bool,
    ) {
        if data.ncols() == 0 || data.nrows() == 0 {
            return;
        }
        let ncols_mat = mat.ncols();
        for (&idx_s_right, &(idx_s_left, num_valid_rows)) in
            valid_rows.range((Excluded(start_idx_s), Included(start_idx_s + data.ncols())))
        {
            let idx_left = idx_s_left - start_idx_s;
            let idx_right = idx_s_right - start_idx_s;
            let start_idx_here = start_idx + idx_left;
            let ncols_data = idx_right - idx_left;
            if mat.nrows() < num_valid_rows {
                mat.resize_vertically_mut(num_valid_rows, 0.0);
            }
            let func = |start_idx_: usize, ncols: usize, offset: usize| {
                mat.view_mut(
                    (num_valid_rows - data.nrows(), start_idx_),
                    (data.nrows(), ncols),
                )
                .copy_from(&data.view((0, idx_left + offset), (data.nrows(), ncols)));
                if is_negative {
                    mat.view_mut(
                        (num_valid_rows - data.nrows(), start_idx_),
                        (data.nrows(), ncols),
                    )
                    .scale_mut(-1.0);
                }
            };
            Self::circular_process(ncols_mat, start_idx_here, ncols_data, func);
        }
    }

    /// Append a right-side interval with constant valid-row count.
    ///
    /// # Parameters
    /// - `valid_rows`: piecewise-constant map to update.
    /// - `n_cols`: number of appended stations.
    /// - `num_valid_rows`: row count assigned to each appended station.
    ///
    /// If the new interval has the same row count as the current tail interval,
    /// both are merged to keep the map compact.
    fn push_back_valid_rows(valid_rows: &mut ValidRows, n_cols: usize, num_valid_rows: usize) {
        if n_cols == 0 {
            return;
        }
        if valid_rows.is_empty() {
            valid_rows.insert(n_cols, (0, num_valid_rows));
            return;
        }
        let (&last_idx_right, &(last_idx_left, last_n_rows)) = valid_rows.iter().last().unwrap();
        let (idx_left, n_cols_) = if last_n_rows == num_valid_rows {
            // Remove the last entry
            valid_rows.pop_last();
            (last_idx_left, n_cols + last_idx_right - last_idx_left)
        } else {
            (last_idx_right, n_cols)
        };
        // Insert a new entry
        valid_rows.insert(idx_left + n_cols_, (idx_left, num_valid_rows));
    }

    /// Validate whether `[idx_left, idx_left + len)` is inside current station window.
    ///
    /// # Notes
    /// This check uses `checked_add` to avoid potential `usize` overflow in range-end
    /// computations. If overflow occurs, the interval is treated as out-of-bounds.
    #[inline(always)]
    pub(crate) fn check_s_in_bounds(
        &self,
        idx_left: usize,
        len: usize,
    ) -> Result<(), ConstraintError> {
        let idx_right = idx_left.checked_add(len);
        let valid_right = self.idx_s.checked_add(self.len);
        if let (Some(idx_right), Some(valid_right)) = (idx_right, valid_right)
            && idx_left >= self.idx_s
            && idx_right <= valid_right
        {
            Ok(())
        } else {
            Err(ConstraintError::OutOfSBounds {
                idx_s: idx_left,
                len,
            })
        }
    }

    /// Write path geometry derivatives on a station interval.
    ///
    /// This is the low-level entry point for users who populate
    /// [`Constraints`](crate::constraints::Constraints) directly. Robot-centric
    /// workflows usually call [`Robot::with_q`](crate::robot::Robot::with_q),
    /// [`Robot::with_q_from_path_2nd`](crate::robot::Robot::with_q_from_path_2nd),
    /// or [`Robot::with_q_from_path_3rd`](crate::robot::Robot::with_q_from_path_3rd),
    /// which forward into this storage layer.
    ///
    /// # Parameters
    /// - `q_new`: configuration values (`dim x N`).
    /// - `dq_new`: first derivatives (`dim x N`).
    /// - `ddq_new`: second derivatives (`dim x N`).
    /// - `dddq_new`: optional third derivatives (`dim x N`).
    /// - `idx_s`: global start station id (inclusive).
    ///
    /// # Behavior
    /// - Performs shape checks and bounds checks.
    /// - Overwrites corresponding circular-buffer ranges.
    /// - Marks second-order derivative data as fully valid (`n_rows = dim`)
    ///   over the updated range.
    /// - If `dddq_new` is provided, writes it and marks third-order derivative
    ///   data as valid over the updated range.
    /// - If `dddq_new` is `None`, clears third-order derivative availability
    ///   over the updated range.
    ///
    /// # Errors
    /// - [`ConstraintError::NoMatchDimensions`](crate::diag::ConstraintError::NoMatchDimensions) on shape mismatch.
    /// - [`ConstraintError::OutOfSBounds`](crate::diag::ConstraintError::OutOfSBounds) if target interval is invalid.
    ///
    /// # Returns
    /// Returns `&mut Self` for chaining on success.
    ///
    /// # Example
    /// The example below writes one-dimensional path geometry into a directly
    /// constructed constraint container.
    ///
    /// ```rust
    /// # fn main() -> Result<(), copp::diag::CoppError> {
    /// use copp::constraints::Constraints;
    /// use nalgebra::DMatrix;
    ///
    /// let mut constraints = Constraints::with_capacity(2, 3);
    /// let s = [0.0, 0.5, 1.0];
    /// constraints.with_s(s.as_slice())?;
    ///
    /// let q = DMatrix::from_row_slice(2, 3, &[
    ///     0.0, 0.125, 0.5,
    ///     1.0, 1.0, 1.0,
    /// ]);
    /// let dq = DMatrix::from_row_slice(2, 3, &[
    ///     0.0, 0.5, 1.0,
    ///     0.0, 0.0, 0.0,
    /// ]);
    /// let ddq = DMatrix::from_row_slice(2, 3, &[
    ///     1.0, 1.0, 1.0,
    ///     0.0, 0.0, 0.0,
    /// ]);
    ///
    /// constraints.with_q(&q.as_view(), &dq.as_view(), &ddq.as_view(), None, 0)?;
    /// assert_eq!(constraints.len(), 3);
    /// # Ok(())
    /// # }
    /// ```
    pub fn with_q(
        &mut self,
        q_new: &InputMatrix,
        dq_new: &InputMatrix,
        ddq_new: &InputMatrix,
        dddq_new: Option<&InputMatrix>,
        idx_s: usize,
    ) -> Result<&mut Self, ConstraintError> {
        // Check dimensions and bounds
        if q_new.nrows() != self.dim
            || dq_new.shape() != q_new.shape()
            || ddq_new.shape() != q_new.shape()
            || dddq_new.is_some_and(|d| d.shape() != q_new.shape())
        {
            return Err(ConstraintError::NoMatchDimensions);
        }
        self.check_s_in_bounds(idx_s, dq_new.ncols())?;
        if dq_new.ncols() == 0 {
            return Ok(self);
        }
        // Add new derivatives
        let start_idx = self.idx(idx_s - self.idx_s);

        Self::copy_from_matrix_to_cirmatrix(&mut self.q, q_new, start_idx);
        Self::copy_from_matrix_to_cirmatrix(&mut self.dq, dq_new, start_idx);
        Self::copy_from_matrix_to_cirmatrix(&mut self.ddq, ddq_new, start_idx);
        // Update valid_rows_dq
        Self::update_valid_rows(
            &mut self.valid_rows_q,
            idx_s,
            idx_s + dq_new.ncols(),
            self.dim,
            ModeUpdateValidRows::SetValues,
        );
        Self::merge_valid_rows(&mut self.valid_rows_q, idx_s, idx_s + dq_new.ncols());

        if let Some(dddq_new) = dddq_new {
            Self::copy_from_matrix_to_cirmatrix(&mut self.dddq, dddq_new, start_idx);
            Self::update_valid_rows(
                &mut self.valid_rows_dddq,
                idx_s,
                idx_s + dddq_new.ncols(),
                self.dim,
                ModeUpdateValidRows::SetValues,
            );
            Self::merge_valid_rows(&mut self.valid_rows_dddq, idx_s, idx_s + dddq_new.ncols());
        } else {
            self.clear_dddq(idx_s, dq_new.ncols())?;
        }

        Ok(self)
    }

    /// Clear third-derivative data availability over a station interval.
    ///
    /// # Parameters
    /// - `idx_s`: global start station id (inclusive).
    /// - `len`: number of station samples to clear.
    ///
    /// # Behavior
    /// - Validates that `[idx_s, idx_s + len)` is inside the stored station range.
    /// - Marks `dddq` as unavailable over that interval.
    /// - Zeros the stored `dddq` values in the covered circular-buffer columns.
    fn clear_dddq(&mut self, idx_s: usize, len: usize) -> Result<&mut Self, ConstraintError> {
        self.check_s_in_bounds(idx_s, len)?;
        if len == 0 {
            return Ok(self);
        }

        let start_idx = self.idx(idx_s - self.idx_s);
        let ncols_mat = self.dddq.ncols();
        let nrows = self.dddq.nrows();
        Self::circular_process(ncols_mat, start_idx, len, |start_idx_, ncols, _| {
            self.dddq
                .view_mut((0, start_idx_), (nrows, ncols))
                .fill(0.0);
        });

        let idx_to = idx_s + len;
        Self::update_valid_rows(
            &mut self.valid_rows_dddq,
            idx_s,
            idx_to,
            0,
            ModeUpdateValidRows::SetValues,
        );
        Self::merge_valid_rows(&mut self.valid_rows_dddq, idx_s, idx_to);

        Ok(self)
    }

    /// Sample a path over a stored station interval and write derivatives up to second order.
    ///
    /// # Parameters
    /// - `path`: geometric path to evaluate at stored station samples.
    /// - `idx_s_from`: global start station id (inclusive).
    /// - `idx_s_to`: global end station id (exclusive).
    ///
    /// # Behavior
    /// - Exports station samples from `[idx_s_from, idx_s_to)`.
    /// - Evaluates `q`, `dq`, and `ddq` with [`Path::evaluate_up_to_2nd`](crate::path::Path::evaluate_up_to_2nd).
    /// - Writes the evaluated derivatives into the circular constraint buffers.
    /// - Marks second-order path-derivative data as valid over the interval.
    /// - Clears third-order path-derivative data over the interval.
    ///
    /// # Errors
    /// - [`CoppError::ConstraintError`](crate::diag::CoppError::ConstraintError) if the station interval is empty, out of
    ///   bounds, or the evaluated path dimension does not match `self.dim`.
    /// - [`CoppError::PathError`](crate::diag::CoppError::PathError) if path evaluation fails, for example because a
    ///   stored station is outside the path range.
    ///
    /// # Returns
    /// Returns `&mut Self` for chaining on success.
    pub(crate) fn with_q_from_path_2nd(
        &mut self,
        path: &Path,
        idx_s_from: usize,
        idx_s_to: usize,
    ) -> Result<&mut Self, CoppError> {
        let s = self.s_vec(idx_s_from, idx_s_to)?;
        let derivs = path.evaluate_up_to_2nd(&s)?;
        self.with_q(
            &derivs.q.as_view(),
            &derivs.dq.as_ref().unwrap().as_view(),
            &derivs.ddq.as_ref().unwrap().as_view(),
            None,
            idx_s_from,
        )?;
        Ok(self)
    }

    /// Sample a path over a stored station interval and write derivatives up to third order.
    ///
    /// # Parameters
    /// - `path`: geometric path to evaluate at stored station samples.
    /// - `idx_s_from`: global start station id (inclusive).
    /// - `idx_s_to`: global end station id (exclusive).
    ///
    /// # Behavior
    /// - Exports station samples from `[idx_s_from, idx_s_to)`.
    /// - Evaluates `q`, `dq`, `ddq`, and `dddq` with [`Path::evaluate_up_to_3rd`](crate::path::Path::evaluate_up_to_3rd).
    /// - Writes the evaluated derivatives into the circular constraint buffers.
    /// - Marks both second- and third-order path-derivative data as valid over
    ///   the interval.
    ///
    /// # Errors
    /// - [`CoppError::ConstraintError`](crate::diag::CoppError::ConstraintError) if the station interval is empty, out of
    ///   bounds, or the evaluated path dimension does not match `self.dim`.
    /// - [`CoppError::PathError`](crate::diag::CoppError::PathError) if path evaluation fails, for example because a
    ///   stored station is outside the path range.
    ///
    /// # Returns
    /// Returns `&mut Self` for chaining on success.
    pub(crate) fn with_q_from_path_3rd(
        &mut self,
        path: &Path,
        idx_s_from: usize,
        idx_s_to: usize,
    ) -> Result<&mut Self, CoppError> {
        let s = self.s_vec(idx_s_from, idx_s_to)?;
        let derivs = path.evaluate_up_to_3rd(&s)?;
        self.with_q(
            &derivs.q.as_view(),
            &derivs.dq.as_ref().unwrap().as_view(),
            &derivs.ddq.as_ref().unwrap().as_view(),
            derivs.dddq.as_ref().map(|m| m.as_view()).as_ref(),
            idx_s_from,
        )?;
        Ok(self)
    }

    /// Add / tighten first-order bound `amax` over an interval.
    ///
    /// # Parameters
    /// - `amax_new`: candidate upper bounds as `R x N`; each column is reduced
    ///   to its minimum before being fused into storage.
    /// - `idx_s`: global start station id.
    ///
    /// # Fusion rule
    /// Stored value is updated as `self.amax = min(self.amax, amax_new_reduced)`.
    ///
    /// # Errors
    /// - [`ConstraintError::OutOfSBounds`](crate::diag::ConstraintError::OutOfSBounds) if interval is invalid.
    /// - [`ConstraintError::NonPositiveA`](crate::diag::ConstraintError::NonPositiveA) if any reduced bound is non-positive.
    ///
    /// # Returns
    /// Returns `&mut Self` for chaining on success.
    pub fn with_constraint_1order<T: AsInputMatrix1D + ?Sized>(
        &mut self,
        amax_new: &T,
        idx_s: usize,
    ) -> Result<&mut Self, ConstraintError> {
        let amax_new = amax_new.as_input_matrix();
        // Check bounds
        self.check_s_in_bounds(idx_s, amax_new.ncols())?;
        if amax_new.ncols() == 0 || amax_new.nrows() == 0 {
            return Ok(self);
        }
        let amax_row: RowDVector<f64> = RowDVector::from_iterator(
            amax_new.ncols(),
            amax_new.column_iter().map(|col| col.min()),
        );
        if amax_row.min() <= 0.0 {
            return Err(ConstraintError::NonPositiveA);
        }
        // Update amax
        let start_idx = self.idx(idx_s - self.idx_s);
        let ncols_mat = self.amax.ncols();
        let ncols_data = amax_row.ncols();
        let func = |start_idx_: usize, ncols: usize, offset: usize| {
            self.amax
                .columns_mut(start_idx_, ncols)
                .iter_mut()
                .zip(amax_row.columns(offset, ncols).iter())
                .for_each(|(a_self, &a_new)| {
                    *a_self = a_self.min(a_new);
                });
        };
        Self::circular_process(ncols_mat, start_idx, ncols_data, func);

        Ok(self)
    }

    /// Append second-order inequality rows over station interval starting at `idx_s`.
    ///
    /// # Model
    /// For each station column, rows satisfy:
    /// `acc_a * a + acc_b * b <= acc_max`.
    ///
    /// # Parameters
    /// - `acc_a_new`, `acc_b_new`, `acc_max_new`: same-shape matrices (`R x N`).
    /// - `idx_s`: global start station id.
    /// - `is_negative`: whether to negate inserted rows (used to build symmetric
    ///   upper/lower bounds from one physical expression).
    ///
    /// # Behavior
    /// - Increases row counts by `R` on affected stations.
    /// - Appends new rows below existing rows per station.
    /// - Merges adjacent validity intervals when row counts match.
    ///
    /// # Errors
    /// - [`ConstraintError::NoMatchDimensions`](crate::diag::ConstraintError::NoMatchDimensions) for shape mismatch.
    /// - [`ConstraintError::OutOfSBounds`](crate::diag::ConstraintError::OutOfSBounds) for invalid interval.
    ///
    /// # Returns
    /// Returns `&mut Self` for chaining on success.
    pub fn with_constraint_2order(
        &mut self,
        acc_a_new: &InputMatrix,
        acc_b_new: &InputMatrix,
        acc_max_new: &InputMatrix,
        idx_s: usize,
        is_negative: bool,
    ) -> Result<&mut Self, ConstraintError> {
        // Check dimensions and bounds
        if acc_a_new.shape() != acc_b_new.shape() || acc_a_new.shape() != acc_max_new.shape() {
            return Err(ConstraintError::NoMatchDimensions);
        }
        self.check_s_in_bounds(idx_s, acc_a_new.ncols())?;
        if acc_a_new.ncols() == 0 || acc_a_new.nrows() == 0 {
            return Ok(self);
        }
        // Add new second-order constraints
        let start_idx = self.idx(idx_s - self.idx_s);
        Self::update_valid_rows(
            &mut self.valid_rows_acc,
            idx_s,
            idx_s + acc_a_new.ncols(),
            acc_a_new.nrows(),
            ModeUpdateValidRows::AddValues,
        );

        [
            (&mut self.acc_a, &acc_a_new),
            (&mut self.acc_b, &acc_b_new),
            (&mut self.acc_max, &acc_max_new),
        ]
        .into_iter()
        .for_each(|(mat_self, mat_new)| {
            Self::concat_from_matrix_to_cirmatrix(
                mat_self,
                &self.valid_rows_acc,
                mat_new,
                start_idx,
                idx_s,
                is_negative,
            )
        });

        Self::merge_valid_rows(&mut self.valid_rows_acc, idx_s, idx_s + acc_a_new.ncols());
        Ok(self)
    }

    /// Append third-order nonlinear inequality rows over station interval.
    ///
    /// # Model
    /// `sqrt(a) * (jerk_a*a + jerk_b*b + jerk_c*c + jerk_d) <= jerk_max`
    ///
    /// # Parameters
    /// - `jerk_*_new`: same-shape matrices (`R x N`).
    /// - `idx_s`: global start station id.
    /// - `is_negative`: if `true`, inserted rows are sign-flipped.
    ///
    /// # Side effects
    /// If the inserted interval overlaps current `valid_ids_linear_jerk`, the
    /// linearization-valid interval is cleared because source nonlinear rows changed.
    ///
    /// # Errors
    /// - [`ConstraintError::NoMatchDimensions`](crate::diag::ConstraintError::NoMatchDimensions) for shape mismatch.
    /// - [`ConstraintError::OutOfSBounds`](crate::diag::ConstraintError::OutOfSBounds) for invalid interval.
    ///
    /// # Returns
    /// Returns `&mut Self` for chaining on success.
    ///
    /// # Example
    /// The example below installs a single jerk upper-bound row at three
    /// stations. After changing third-order rows, build a TOPP3/COPP3 problem
    /// with `build_with_linearization()` before reading linearized jerk rows.
    ///
    /// ```rust
    /// # fn main() -> Result<(), copp::diag::CoppError> {
    /// use copp::constraints::Constraints;
    /// use nalgebra::DMatrix;
    ///
    /// let mut constraints = Constraints::with_capacity(2, 3);
    /// let s = [0.0, 0.5, 1.0];
    /// constraints.with_s(s.as_slice())?;
    ///
    /// let zero = DMatrix::from_row_slice(2, 3, &[
    ///     0.0, 0.0, 0.0,
    ///     0.0, 0.0, 0.0,
    /// ]);
    /// let jerk_c = DMatrix::from_row_slice(2, 3, &[
    ///     1.0, 1.0, 1.0,
    ///     -1.0, -1.0, -1.0,
    /// ]);
    /// let jerk_max = DMatrix::from_row_slice(2, 3, &[
    ///     5.0, 5.0, 5.0,
    ///     5.0, 5.0, 5.0,
    /// ]);
    ///
    /// constraints.with_constraint_3order(
    ///     &zero.as_view(),
    ///     &zero.as_view(),
    ///     &jerk_c.as_view(),
    ///     &zero.as_view(),
    ///     &jerk_max.as_view(),
    ///     0,
    ///     false,
    /// )?;
    /// assert_eq!(constraints.get_jerk_constraints(1)?.0.nrows(), 2);
    /// # Ok(())
    /// # }
    /// ```
    #[allow(clippy::too_many_arguments)]
    pub fn with_constraint_3order(
        &mut self,
        jerk_a_new: &InputMatrix,
        jerk_b_new: &InputMatrix,
        jerk_c_new: &InputMatrix,
        jerk_d_new: &InputMatrix,
        jerk_max_new: &InputMatrix,
        idx_s: usize,
        is_negative: bool,
    ) -> Result<&mut Self, ConstraintError> {
        // Check dimensions and bounds
        if [&jerk_b_new, &jerk_c_new, &jerk_d_new, &jerk_max_new]
            .iter()
            .any(|mat| mat.shape() != jerk_a_new.shape())
        {
            return Err(ConstraintError::NoMatchDimensions);
        }
        self.check_s_in_bounds(idx_s, jerk_a_new.ncols())?;
        if jerk_a_new.ncols() == 0 || jerk_a_new.nrows() == 0 {
            return Ok(self);
        }
        if self.valid_ids_linear_jerk.1 > idx_s
            && self.valid_ids_linear_jerk.0 < idx_s + jerk_a_new.ncols()
        {
            // clear the valid linear jerk constraints
            self.valid_ids_linear_jerk = (0, 0);
        }
        // Add new third-order constraints
        let start_idx = self.idx(idx_s - self.idx_s);

        Self::update_valid_rows(
            &mut self.valid_rows_jerk,
            idx_s,
            idx_s + jerk_a_new.ncols(),
            jerk_a_new.nrows(),
            ModeUpdateValidRows::AddValues,
        );

        [
            (&mut self.jerk_a, &jerk_a_new),
            (&mut self.jerk_b, &jerk_b_new),
            (&mut self.jerk_c, &jerk_c_new),
            (&mut self.jerk_d, &jerk_d_new),
            (&mut self.jerk_max, &jerk_max_new),
        ]
        .into_iter()
        .for_each(|(mat_self, mat_new)| {
            Self::concat_from_matrix_to_cirmatrix(
                mat_self,
                &self.valid_rows_jerk,
                mat_new,
                start_idx,
                idx_s,
                is_negative,
            )
        });

        // Keep the linearized pair row-compatible with the rows just added.
        let rows = self.jerk_a.nrows();
        if self.jerk_a_linear.nrows() < rows {
            self.jerk_a_linear.resize_vertically_mut(rows, 0.0);
            self.jerk_max_linear
                .resize_vertically_mut(rows, f64::INFINITY);
        }

        Self::merge_valid_rows(&mut self.valid_rows_jerk, idx_s, idx_s + jerk_a_new.ncols());

        Ok(self)
    }

    /// Linearize third-order jerk constraints around a reference profile `a_linear`.
    ///
    /// # Purpose
    /// The original third-order inequality contains a nonlinear factor `1/sqrt(a)`:
    /// `sqrt(a) * (jerk_a*a + jerk_b*b + jerk_c*c + jerk_d) <= jerk_max`, i.e.,
    /// `jerk_a*a + jerk_b*b + jerk_c*c + jerk_d <= jerk_max / sqrt(a)`.
    ///
    /// This method performs a first-order affine approximation around reference `a_linear`,
    /// and writes the result into internal linearized buffers:
    /// - `jerk_a_linear`
    /// - `jerk_max_linear`
    ///
    /// so downstream LP/SOCP/RA stages can read linear constraints through
    /// `get_jerk_linear_constraints()`.
    /// Public callers normally trigger this through
    /// [`Topp3ProblemBuilder::build_with_linearization`](crate::solver::topp3_socp::Topp3ProblemBuilder::build_with_linearization)
    /// or [`Copp3ProblemBuilder::build_with_linearization`](crate::solver::copp3_socp::Copp3ProblemBuilder::build_with_linearization).
    ///
    /// # Numerical safety
    /// - Negative `a_linear` is rejected.
    /// - `a_linear == 0` is allowed.
    /// - To avoid singularity of `1/sqrt(a)` near zero, this method uses
    ///   `1.0 / max(a_linear, a_linearization_floor).sqrt()`.
    ///
    /// # Parameters
    /// - `a_linear`: reference profile for linearization.
    /// - `start_idx_s`: global start index of `a_linear`.
    /// - `a_linearization_floor`: strictly positive denominator floor for
    ///   `1/sqrt(a)` evaluation.
    pub(crate) fn linearize_constraint_3order_with_floor(
        &mut self,
        a_linear: &[f64],
        start_idx_s: usize,
        a_linearization_floor: f64,
    ) -> Result<(), ConstraintError> {
        if a_linear.is_empty() {
            return Ok(());
        }
        self.check_s_in_bounds(start_idx_s, a_linear.len())?;
        if a_linearization_floor <= 0.0 {
            return Err(ConstraintError::NonPositiveLinearizationFloor);
        }
        if a_linear.iter().any(|&a| a < 0.0) {
            return Err(ConstraintError::NonPositiveA);
        }

        // Record the floor so `stationary_constraint_topp3` floors the same quantity.
        self.a_linearization_floor = a_linearization_floor;

        let a_linear_half = a_linear
            .iter()
            .map(|a| 1.0 / a.max(a_linearization_floor).sqrt())
            .collect_vec(); // 1 / sqrt(a_linear)
        let a_linear_one_half = a_linear_half.iter().map(|a| *a * *a * *a).collect_vec(); // 1 / (a_linear)^(3/2)

        let start_idx = self.idx(start_idx_s - self.idx_s);
        for (&idx_s_right, &(idx_s_left, num_valid_rows)) in self
            .valid_rows_jerk
            .range((Excluded(start_idx_s), Unbounded))
        {
            if idx_s_left >= start_idx_s + a_linear.len() {
                break;
            }
            let idx_left = idx_s_left.max(start_idx_s) - start_idx_s;
            let idx_right = idx_s_right.min(start_idx_s + a_linear.len()) - start_idx_s;
            let start_idx_here = start_idx + idx_left;
            let ncols_data = idx_right - idx_left;
            if self.jerk_a_linear.nrows() < num_valid_rows {
                self.jerk_a_linear
                    .resize_vertically_mut(num_valid_rows, 0.0);
                self.jerk_max_linear
                    .resize_vertically_mut(num_valid_rows, f64::INFINITY);
            }
            // jerk_a * a[k] + jerk_b * b[k] + jerk_c * c[k] + jerk_d <= jerk_max / sqrt(a[k])
            // jerk_a * a[k] + jerk_b * b[k] + jerk_c * c[k] <= jerk_max * (1.5 * a_linear_half - 0.5 * a_linear_one_half * a[k]) - jerk_d
            // (jerk_a + 0.5 * jerk_max * a_linear_one_half) * a[k] + jerk_b * b[k] + jerk_c * c[k] <= 1.5 * jerk_max * a_linear_half - jerk_d
            let func = |start_idx_: usize, ncols: usize, offset: usize| {
                // jerk_a_linear = jerk_a + 0.5 * jerk_max * a_linear_one_half
                self.jerk_a_linear
                    .view_mut((0, start_idx_), (num_valid_rows, ncols))
                    .copy_from(&self.jerk_a.view((0, start_idx_), (num_valid_rows, ncols)));
                self.jerk_a_linear
                    .column_iter_mut()
                    .zip(self.jerk_max.column_iter())
                    .skip(start_idx_)
                    .take(ncols)
                    .zip(a_linear_one_half[(idx_left + offset)..(idx_left + offset + ncols)].iter())
                    .for_each(|((mut jerk_a_linear, jerk_max), &a_lin_o_h)| {
                        jerk_a_linear.axpy(0.5 * a_lin_o_h, &jerk_max, 1.0);
                    });
                // jerk_max_linear = 1.5 * jerk_max * a_linear_half - jerk_d
                self.jerk_max_linear
                    .view_mut((0, start_idx_), (num_valid_rows, ncols))
                    .copy_from(&self.jerk_d.view((0, start_idx_), (num_valid_rows, ncols)));
                self.jerk_max_linear
                    .view_mut((0, start_idx_), (num_valid_rows, ncols))
                    .neg_mut();
                self.jerk_max_linear
                    .column_iter_mut()
                    .zip(self.jerk_max.column_iter())
                    .skip(start_idx_)
                    .take(ncols)
                    .zip(a_linear_half[(idx_left + offset)..(idx_left + offset + ncols)].iter())
                    .for_each(|((mut jerk_max_linear, jerk_max), &a_lin_h)| {
                        jerk_max_linear.axpy(1.5 * a_lin_h, &jerk_max, 1.0);
                    });
            };
            Self::circular_process(self.jerk_max.ncols(), start_idx_here, ncols_data, func);
        }

        self.valid_ids_linear_jerk = (start_idx_s, start_idx_s + a_linear.len());
        Ok(())
    }

    /// Linearize third-order rows at a per-row anchor recovered from `a_linear`.
    ///
    /// The tangent of `1/sqrt(a)` at `a_L` lies below `1/sqrt(a)` for every
    /// `a_L > 0`, so any anchor keeps the linearized row an inner approximation of
    /// `sqrt(a) * (jerk_a * a + jerk_b * b + jerk_c * c + jerk_d) <= jerk_max`.
    /// One anchor per station is nevertheless harmful when that anchor is small:
    /// the upper and lower row of one axis differ only by an overall sign, so
    /// adding them cancels `b` and `c` and leaves `a <= 3 * a_L` whatever the path
    /// derivatives are, including on an axis that does not move at all.  Anchoring
    /// each row where that row is nearly active breaks the cancellation.
    ///
    /// Writing `value := jerk_a * a + jerk_b * b + jerk_c * c + jerk_d` at the state
    /// recovered from `a_linear`, a row is anchored
    ///
    /// - at `amax` when `value <= 0`: the row is slack there for every `a`, and it
    ///   stays slack for every anchor at or above `a_linear[k]`;
    /// - at `sqrt(a_linear[k] * a_active)` when `value > 0`, where
    ///   `a_active := (jerk_max / value)^2` is the `a` at which the row becomes
    ///   active with `(b, c)` frozen.  `a_linear[k] <= a_active` is exactly "the
    ///   reference state satisfies the nonlinear row", the anchors preserving that
    ///   property form an interval spanned by the two, and the geometric mean sits
    ///   inside it.
    ///
    /// # Numerical safety
    /// - `b_linear` must belong to the same feasible profile as `a_linear`; the
    ///   anchors are only guaranteed to keep that profile feasible when it does.
    /// - `value <= 0` is tested before `a_active` is formed, so `a_linear[k] == 0`
    ///   together with `value == 0` cannot produce `sqrt(0 * inf)`.
    /// - Rows whose `jerk_max` or `amax` is not finite keep the uniform anchor.
    /// - The anchor is floored by `a_linearization_floor` exactly as in
    ///   [`Self::linearize_constraint_3order_with_floor`].
    ///
    /// # Parameters
    /// - `a_linear`: reference profile.
    /// - `b_linear`: path acceleration of the same feasible profile as `a_linear`.
    /// - `start_idx_s`: global start index of both slices.
    /// - `a_linearization_floor`: strictly positive denominator floor for
    ///   `1/sqrt(a)` evaluation.
    /// - `num_stationary`: effective boundary pair. Its one-sided
    ///   `delta b / delta s` values are excluded because stationary intervals
    ///   use the separate constant-time-jerk boundary model.
    pub(crate) fn linearize_constraint_3order_adaptive(
        &mut self,
        a_linear: &[f64],
        b_linear: &[f64],
        start_idx_s: usize,
        a_linearization_floor: f64,
        num_stationary: (usize, usize),
    ) -> Result<(), ConstraintError> {
        if a_linear.is_empty() {
            return Ok(());
        }
        self.check_s_in_bounds(start_idx_s, a_linear.len())?;
        if a_linearization_floor <= 0.0 {
            return Err(ConstraintError::NonPositiveLinearizationFloor);
        }
        if a_linear.iter().any(|&a| a < 0.0) {
            return Err(ConstraintError::NonPositiveA);
        }

        // Record the floor so `stationary_constraint_topp3` floors the same quantity.
        self.a_linearization_floor = a_linearization_floor;

        let n = a_linear.len();
        let ordinary_interval_end = n - 1 - num_stationary.1;

        // `c` is the one-sided difference of `b` the model uses.  One stored row is
        // emitted once with the forward stencil and once with the backward one, so
        // the anchor takes the smaller of the two candidates and stays admissible
        // under both: each admissible set is an interval containing `a_linear[k]`.
        // Stationary boundary intervals use the separate constant-time-jerk model
        // in `stationary_constraint_topp3`; their delta-b/delta-s is not this
        // model's `c` and must not participate in adaptive anchor selection.
        let max_rows = self.jerk_a.nrows();
        let mut anchor_half = DMatrix::zeros(max_rows, n);
        let mut anchor_one_half = DMatrix::zeros(max_rows, n);
        for k in 0..n {
            let idx_s = start_idx_s + k;
            let amax = self.amax_unchecked(idx_s);
            let (jerk_a, jerk_b, jerk_c, jerk_d, jerk_max) = self.jerk_constraints_unchecked(idx_s);
            let mut c_stencils = [f64::NAN; 2];
            if k >= num_stationary.0 && k < ordinary_interval_end {
                let ds = self.s_unchecked(idx_s + 1) - self.s_unchecked(idx_s);
                c_stencils[0] = (b_linear[k + 1] - b_linear[k]) / ds;
            }
            if k > num_stationary.0 && k <= ordinary_interval_end {
                let ds = self.s_unchecked(idx_s) - self.s_unchecked(idx_s - 1);
                c_stencils[1] = (b_linear[k] - b_linear[k - 1]) / ds;
            }
            for row in 0..jerk_max.nrows() {
                let coefficients = (
                    jerk_a[(row, 0)],
                    jerk_b[(row, 0)],
                    jerk_c[(row, 0)],
                    jerk_d[(row, 0)],
                    jerk_max[(row, 0)],
                );
                let mut anchor: Option<f64> = None;
                let mut keep_uniform = false;
                for &c_lin in c_stencils.iter().filter(|c| c.is_finite()) {
                    match adaptive_anchor_3order(
                        coefficients,
                        (a_linear[k], b_linear[k], c_lin),
                        amax,
                    ) {
                        Some(value) => {
                            anchor = Some(anchor.map_or(value, |held: f64| held.min(value)));
                        }
                        None => {
                            keep_uniform = true;
                            break;
                        }
                    }
                }
                let anchor = if keep_uniform {
                    a_linear[k]
                } else {
                    anchor.unwrap_or(a_linear[k])
                };
                let half = 1.0 / anchor.max(a_linearization_floor).sqrt();
                anchor_half[(row, k)] = half;
                anchor_one_half[(row, k)] = half * half * half;
            }
        }

        let start_idx = self.idx(start_idx_s - self.idx_s);
        let anchor_half = &anchor_half;
        let anchor_one_half = &anchor_one_half;
        for (&idx_s_right, &(idx_s_left, num_valid_rows)) in self
            .valid_rows_jerk
            .range((Excluded(start_idx_s), Unbounded))
        {
            if idx_s_left >= start_idx_s + n {
                break;
            }
            let idx_left = idx_s_left.max(start_idx_s) - start_idx_s;
            let idx_right = idx_s_right.min(start_idx_s + n) - start_idx_s;
            let start_idx_here = start_idx + idx_left;
            let ncols_data = idx_right - idx_left;
            if self.jerk_a_linear.nrows() < num_valid_rows {
                self.jerk_a_linear
                    .resize_vertically_mut(num_valid_rows, 0.0);
                self.jerk_max_linear
                    .resize_vertically_mut(num_valid_rows, f64::INFINITY);
            }
            // Same algebra as `linearize_constraint_3order_with_floor`, but the
            // anchor is per row, so the scalar of `axpy` becomes a column of
            // `anchor_*` and the update turns into a component-wise multiply-add.
            // The access pattern is unchanged: one `copy_from` and one pass.
            let func = |start_idx_: usize, ncols: usize, offset: usize| {
                // jerk_a_linear = jerk_a + 0.5 * jerk_max * anchor_one_half
                self.jerk_a_linear
                    .view_mut((0, start_idx_), (num_valid_rows, ncols))
                    .copy_from(&self.jerk_a.view((0, start_idx_), (num_valid_rows, ncols)));
                self.jerk_a_linear
                    .view_mut((0, start_idx_), (num_valid_rows, ncols))
                    .zip_zip_apply(
                        &self.jerk_max.view((0, start_idx_), (num_valid_rows, ncols)),
                        &anchor_one_half.view((0, idx_left + offset), (num_valid_rows, ncols)),
                        |jerk_a_linear, jerk_max, a_lin_o_h| {
                            *jerk_a_linear += 0.5 * jerk_max * a_lin_o_h;
                        },
                    );
                // jerk_max_linear = 1.5 * jerk_max * anchor_half - jerk_d
                self.jerk_max_linear
                    .view_mut((0, start_idx_), (num_valid_rows, ncols))
                    .copy_from(&self.jerk_d.view((0, start_idx_), (num_valid_rows, ncols)));
                self.jerk_max_linear
                    .view_mut((0, start_idx_), (num_valid_rows, ncols))
                    .neg_mut();
                self.jerk_max_linear
                    .view_mut((0, start_idx_), (num_valid_rows, ncols))
                    .zip_zip_apply(
                        &self.jerk_max.view((0, start_idx_), (num_valid_rows, ncols)),
                        &anchor_half.view((0, idx_left + offset), (num_valid_rows, ncols)),
                        |jerk_max_linear, jerk_max, a_lin_h| {
                            *jerk_max_linear += 1.5 * jerk_max * a_lin_h;
                        },
                    );
            };
            Self::circular_process(self.jerk_max.ncols(), start_idx_here, ncols_data, func);
        }

        self.valid_ids_linear_jerk = (start_idx_s, start_idx_s + n);
        Ok(())
    }

    /// Update valid-row map on interval `[idx_from, idx_to)`.
    ///
    /// # Parameters
    /// - `valid_rows`: map to modify.
    /// - `idx_from`: inclusive start station id.
    /// - `idx_to`: exclusive end station id.
    /// - `n_rows`: row count argument applied per `mode`.
    /// - `mode`: [`SetValues`](ModeUpdateValidRows::SetValues) to overwrite, [`AddValues`](ModeUpdateValidRows::AddValues) to accumulate.
    ///
    /// # Invariant handling
    /// The function splits leaves at boundaries when necessary so update is exact
    /// on the requested half-open interval.
    fn update_valid_rows(
        valid_rows: &mut ValidRows,
        idx_from: usize,
        idx_to: usize,
        n_rows: usize,
        mode: ModeUpdateValidRows,
    ) {
        // Locate idx_to and split the leaf
        if let Some((&idx_right, (idx_left, n_rows_right))) = valid_rows.range_mut(idx_to..).next()
            && idx_right != idx_to
        {
            // Split the entry at idx_to
            let idx_left_old = *idx_left;
            let n_rows_old = *n_rows_right;
            *idx_left = idx_to;
            valid_rows.insert(idx_to, (idx_left_old, n_rows_old));
        }
        // Locate idx_from and split the leaf
        if let Some((_, (idx_left, n_rows_left))) =
            valid_rows.range_mut((Excluded(idx_from), Unbounded)).next()
            && *idx_left != idx_from
        {
            // Split the entry at idx_from
            let idx_left_old = *idx_left;
            let n_rows_old = *n_rows_left;
            *idx_left = idx_from;
            valid_rows.insert(idx_from, (idx_left_old, n_rows_old));
        }
        // Update entries in [idx_from, idx_to)
        let iter = valid_rows.range_mut((Excluded(idx_from), Included(idx_to)));
        match mode {
            ModeUpdateValidRows::SetValues => {
                for (_, (_, n_rows_entry)) in iter {
                    *n_rows_entry = n_rows;
                }
            }
            ModeUpdateValidRows::AddValues => {
                for (_, (_, n_rows_entry)) in iter {
                    *n_rows_entry += n_rows;
                }
            }
        }
    }

    /// Merge adjacent map leaves with identical row counts near update boundaries.
    ///
    /// This post-processing keeps [`ValidRows`] compact after splits and updates.
    fn merge_valid_rows(valid_rows: &mut ValidRows, idx_from: usize, idx_to: usize) {
        // Merge those near idx_from
        if let Some((&idx_right, &(idx_left, nrows))) =
            valid_rows.range((Excluded(idx_from), Unbounded)).next()
        {
            // Merge those after idx_from
            let mut idx_right = idx_right;
            while let Some((&idx_right_new, &(_, n_rows_new))) =
                valid_rows.range((Excluded(idx_right), Unbounded)).next()
            {
                if n_rows_new != nrows {
                    break;
                }
                valid_rows.remove(&idx_right);
                idx_right = idx_right_new;
            }
            if let Some((idx_left_final, _)) = valid_rows.get_mut(&idx_right) {
                *idx_left_final = idx_left;
            }

            // Merge those before idx_from
            let mut idx_left = idx_left;
            while let Some((&idx_right_new, &(idx_left_new, n_rows_new))) = valid_rows
                .range((Unbounded, Included(idx_left)))
                .next_back()
            {
                if n_rows_new != nrows {
                    break;
                }
                valid_rows.remove(&idx_right_new);
                idx_left = idx_left_new;
            }
            if let Some((idx_left_final, _)) = valid_rows.get_mut(&idx_right) {
                *idx_left_final = idx_left;
            }
        }

        // Merge those near idx_to
        if let Some((&idx_right, &(idx_left, nrows))) =
            valid_rows.range((Included(idx_to), Unbounded)).next()
        {
            // Merge those before idx_to
            let mut id_left = idx_left;
            while let Some((&idx_right_new, &(idx_left_new, n_rows_new))) =
                valid_rows.range((Unbounded, Included(id_left))).next_back()
            {
                if n_rows_new != nrows {
                    break;
                }
                valid_rows.remove(&idx_right_new);
                id_left = idx_left_new;
            }
            if let Some((idx_left_final, _)) = valid_rows.get_mut(&idx_right) {
                *idx_left_final = id_left;
            }
            // Merge those after idx_to
            let mut idx_right = idx_right;
            while let Some((&idx_right_new, &(_, n_rows_new))) =
                valid_rows.range((Excluded(idx_right), Unbounded)).next()
            {
                if n_rows_new != nrows {
                    break;
                }
                valid_rows.remove(&idx_right);
                idx_right = idx_right_new;
            }
            if let Some((idx_left_final, _)) = valid_rows.get_mut(&idx_right) {
                *idx_left_final = id_left;
            }
        }
    }

    /// Check whether `q`, `dq`, and `ddq` are fully available in `[start_idx_s, end_idx_s)`.
    ///
    /// Returns `true` only when each covered station has `n_rows == dim` in
    /// `valid_rows_q`.
    pub(crate) fn check_given_q(&self, start_idx_s: usize, end_idx_s: usize) -> bool {
        let (&end_idx_s_new, _) = self.valid_rows_q.range(end_idx_s..).next().unwrap();
        self.valid_rows_q
            .range((Excluded(start_idx_s), Included(end_idx_s_new)))
            .into_iter()
            .all(|(&_, &(_, n_rows))| n_rows == self.dim)
    }

    /// Check whether `dddq` is fully available in `[start_idx_s, end_idx_s)`.
    ///
    /// Returns `true` only when each covered station has `n_rows == dim` in
    /// `valid_rows_dddq`.
    pub(crate) fn check_given_dddq(&self, start_idx_s: usize, end_idx_s: usize) -> bool {
        let (&end_idx_s_new, _) = self.valid_rows_dddq.range(end_idx_s..).next().unwrap();
        self.valid_rows_dddq
            .range((Excluded(start_idx_s), Included(end_idx_s_new)))
            .into_iter()
            .all(|(&_, &(_, n_rows))| n_rows == self.dim)
    }

    /// Remove a prefix of logical stations from the front.
    ///
    /// # Modes
    /// - [`ModePopConstraints::CutAtIdxS`](crate::constraints::ModePopConstraints::CutAtIdxS)`(cut)`: keep stations with `id >= cut`.
    /// - [`ModePopConstraints::PopNCols`](crate::constraints::ModePopConstraints::PopNCols)`(n)`: remove first `n` logical stations.
    ///
    /// # Notes
    /// - `amax` values in removed columns are reset to `+inf`.
    /// - Valid-row maps are trimmed and re-anchored.
    /// - `idx_s` increases and `head_col` advances accordingly.
    pub fn pop_front(&mut self, mode: ModePopConstraints) {
        // Determine ncols to pop and idx_s_cut
        let idx_s_cut = match mode {
            ModePopConstraints::CutAtIdxS(idx_s_cut) => min(idx_s_cut, self.idx_s + self.len),
            ModePopConstraints::PopNCols(n_cols) => self.idx_s + min(n_cols, self.len),
        };
        if idx_s_cut <= self.idx_s {
            return;
        }
        let ncols = idx_s_cut - self.idx_s;
        if ncols >= self.len {
            self.clear(true);
            return;
        }
        // Update amax
        let ncols_mat = self.amax.ncols();
        let start_idx = self.idx(0);
        let clear_amax = |start_idx: usize, ncols: usize, _: usize| {
            self.amax.columns_mut(start_idx, ncols).fill(f64::INFINITY)
        };
        Self::circular_process(ncols_mat, start_idx, ncols, clear_amax);
        // Update valid_rows
        let pop_front_valid_rows = |valid_rows: &mut ValidRows, key_cut: usize| {
            let tree_to_keep = valid_rows.split_off(&(key_cut + 1));
            *valid_rows = tree_to_keep;
            if let Some((&_, (idx_s_left, _))) = valid_rows.iter_mut().next() {
                *idx_s_left = key_cut;
            }
        };
        pop_front_valid_rows(&mut self.valid_rows_acc, idx_s_cut);
        pop_front_valid_rows(&mut self.valid_rows_jerk, idx_s_cut);
        pop_front_valid_rows(&mut self.valid_rows_q, idx_s_cut);
        pop_front_valid_rows(&mut self.valid_rows_dddq, idx_s_cut);
        // Update parameters
        self.head_col = self.idx(ncols);
        self.idx_s += ncols;
        self.len -= ncols;
    }

    /// Remove a suffix of logical stations from the back.
    ///
    /// # Modes
    /// - [`ModePopConstraints::CutAtIdxS`](crate::constraints::ModePopConstraints::CutAtIdxS)`(cut)`: keep stations with `id < cut`.
    /// - [`ModePopConstraints::PopNCols`](crate::constraints::ModePopConstraints::PopNCols)`(n)`: remove last `n` logical stations.
    ///
    /// # Notes
    /// - `amax` values in removed columns are reset to `+inf`.
    /// - Valid-row maps are trimmed to new right boundary.
    /// - `idx_s` is unchanged; only `len` shrinks.
    pub fn pop_back(&mut self, mode: ModePopConstraints) {
        // Determine ncols to pop
        let idx_s_cut = match mode {
            ModePopConstraints::CutAtIdxS(idx_s_cut) => max(idx_s_cut, self.idx_s),
            ModePopConstraints::PopNCols(n_cols) => self.idx_s + self.len - min(n_cols, self.len),
        };
        if idx_s_cut >= self.idx_s + self.len {
            return;
        }
        let ncols = self.idx_s + self.len - idx_s_cut;
        if ncols >= self.len {
            self.clear(true);
            return;
        }
        // Update amax
        let ncols_mat = self.amax.ncols();
        let start_idx = self.idx(self.len - ncols);
        let clear_amax = |start_idx: usize, ncols: usize, _: usize| {
            self.amax.columns_mut(start_idx, ncols).fill(f64::INFINITY)
        };
        Self::circular_process(ncols_mat, start_idx, ncols, clear_amax);
        // Update valid_rows
        let pop_back_valid_rows = |valid_rows: &mut ValidRows, key_cut: usize| {
            valid_rows.split_off(&key_cut);
            if let Some((&idx_s_right, &(idx_s_left, n_rows))) = valid_rows.iter().last() {
                valid_rows.remove(&idx_s_right);
                valid_rows.insert(key_cut, (idx_s_left, n_rows));
            }
        };
        pop_back_valid_rows(&mut self.valid_rows_acc, idx_s_cut);
        pop_back_valid_rows(&mut self.valid_rows_jerk, idx_s_cut);
        pop_back_valid_rows(&mut self.valid_rows_q, idx_s_cut);
        pop_back_valid_rows(&mut self.valid_rows_dddq, idx_s_cut);
        // Update parameters
        self.len -= ncols;
    }

    /// Reset logical content and validity maps.
    ///
    /// # Parameters
    /// - `keep_idx_s`: when `true`, preserve current global station origin;
    ///   otherwise reset it to `0`.
    ///
    /// # Notes
    /// `amax` is reinitialized to `+inf`; other matrices are kept allocated and may
    /// retain old values outside the active logical window.
    pub fn clear(&mut self, keep_idx_s: bool) {
        self.head_col = 0;
        self.len = 0;
        if !keep_idx_s {
            self.idx_s = 0;
        }
        self.valid_rows_acc.clear();
        self.valid_rows_jerk.clear();
        self.valid_rows_q.clear();
        self.valid_rows_dddq.clear();
        self.amax.fill(f64::INFINITY);
    }

    /// Compute total row count over station interval in a [`ValidRows`] map.
    ///
    /// Returns:
    /// `sum_{k in [idx_from, idx_to)} valid_rows(k)`.
    fn count_rows(valid_rows: &ValidRows, idx_from: usize, idx_to: usize) -> usize {
        let mut n_rows_total: usize = 0;
        for (&idx_right, &(idx_left, n_rows)) in
            valid_rows.range((Excluded(idx_from), Included(idx_to)))
        {
            n_rows_total += (idx_right.min(idx_to) - idx_left.max(idx_from)) * n_rows;
        }
        n_rows_total
    }

    /// Total second-order row count in `[idx_from, idx_to)`.
    pub(crate) fn count_rows_acc(&self, idx_from: usize, idx_to: usize) -> usize {
        Self::count_rows(&self.valid_rows_acc, idx_from, idx_to)
    }

    /// Total third-order row count in `[idx_from, idx_to)`.
    pub(crate) fn count_rows_jerk(&self, idx_from: usize, idx_to: usize) -> usize {
        Self::count_rows(&self.valid_rows_jerk, idx_from, idx_to)
    }

    /// Overwrite first-order bounds in `[idx_from, idx_from + amax_new.len())`.
    ///
    /// # Parameters
    /// - `amax_new`: replacement values.
    /// - `idx_from`: global start station id.
    ///
    /// # Errors
    /// Returns [`ConstraintError::OutOfSBounds`](crate::diag::ConstraintError::OutOfSBounds) if target range is invalid.
    pub fn amax_substitute(
        &mut self,
        amax_new: &[f64],
        idx_from: usize,
    ) -> Result<(), ConstraintError> {
        let ncols_data = amax_new.len();
        self.check_s_in_bounds(idx_from, ncols_data)?;
        if ncols_data == 0 {
            return Ok(());
        }
        // Step 1. copy old amax
        let start_idx = self.idx(idx_from - self.idx_s);
        let ncols_mat = self.amax.ncols();
        // Step 2. substitute new amax
        let func = |start_idx_: usize, ncols: usize, offset: usize| {
            self.amax
                .columns_mut(start_idx_, ncols)
                .copy_from_slice(&amax_new[offset..(offset + ncols)]);
        };
        Self::circular_process(ncols_mat, start_idx, ncols_data, func);
        Ok(())
    }

    /// Bounds on a stationary template's anchor from its zero-speed endpoint.
    ///
    /// For anchor value `A` at signed distance `D`, the template has
    /// `sqrt(a) * c = A^(3/2) / (4.5 * D^2)`, including at the endpoint.
    /// The other terms in the original jerk row vanish there; substituting
    /// `a = 0` into an ordinary finite-c row would incorrectly drop this limit.
    /// Both the start and end templates have a positive path-jerk limit.
    pub(crate) fn stationary_boundary_jerk_anchor_bounds(
        &self,
        idx_s_boundary: usize,
        ds_anchor: f64,
    ) -> (f64, f64) {
        let distance = ds_anchor.abs();
        if !distance.is_finite() || distance == 0.0 {
            return (f64::NEG_INFINITY, f64::INFINITY);
        }
        let (_, _, jerk_c, _, jerk_max) = self.jerk_constraints_unchecked(idx_s_boundary);
        let mut upper = f64::INFINITY;
        let mut lower = 0.0_f64;
        let distance_cbrt = distance.cbrt();
        let distance_factor = (4.5_f64.cbrt() * distance_cbrt) * distance_cbrt;
        for (&coefficient, &rhs) in jerk_c.iter().zip(jerk_max.iter()) {
            if rhs == f64::INFINITY {
                continue;
            }
            if !rhs.is_finite() || !coefficient.is_finite() {
                return (f64::NEG_INFINITY, f64::INFINITY);
            }
            if coefficient == 0.0 {
                if rhs < 0.0 {
                    return (f64::NEG_INFINITY, f64::INFINITY);
                }
                continue;
            }
            if coefficient > 0.0 && rhs < 0.0 {
                return (f64::NEG_INFINITY, f64::INFINITY);
            }
            if coefficient < 0.0 && rhs >= 0.0 {
                continue;
            }
            // Take cube roots before forming the ratio or D^2, avoiding
            // overflow/underflow when the resulting anchor bound is finite.
            let root_a = (rhs.abs().cbrt() / coefficient.abs().cbrt()) * distance_factor;
            let bound = root_a * root_a;
            if coefficient > 0.0 {
                upper = upper.min(bound);
            } else {
                lower = lower.max(bound);
            }
        }
        // Only compensate for the fixed cbrt/product evaluation above. Do
        // not erase a valid singleton/tiny interval by independently moving
        // two opposite rows inward; the final row audit retains its existing
        // arithmetic allowance for that case.
        let inset = 64.0 * f64::EPSILON;
        let inset_upper = if upper.is_finite() && upper > 0.0 {
            (upper * (1.0 - inset)).next_down().max(0.0)
        } else {
            upper
        };
        let inset_lower = if lower.is_finite() && lower > 0.0 {
            (lower * (1.0 + inset)).next_up()
        } else {
            lower
        };
        if inset_lower <= inset_upper {
            (inset_upper, inset_lower)
        } else {
            (upper, lower)
        }
    }

    /// Derive equivalent scalar bounds for stationary boundary handling in TOPP3.
    ///
    /// # Purpose
    /// Condenses first/second/third-order constraints over a stationary boundary
    /// stencil into a single interval bound on one representative `a` variable.
    ///
    /// # Returns
    /// `(amax_stationary, amin_stationary)` such that:
    /// - forward (`REV = false`): `amin <= a[num_stationary] <= amax`
    /// - reverse (`REV = true`): `amin <= a[n - num_stationary] <= amax`
    ///
    /// Returns `(<= 0)` when prerequisite station range is unavailable.
    pub(crate) fn stationary_constraint_topp3<const REV: bool>(
        &self,
        a_linear: &[f64],
        idx_s_start: usize,
        num_stationary: usize,
    ) -> (f64, f64) {
        if num_stationary == 0 {
            return (f64::INFINITY, 0.0);
        }
        let n = a_linear.len() - 1;
        let (id_start, id_a) = if REV {
            if self
                .check_s_in_bounds(idx_s_start + n - num_stationary, num_stationary + 1)
                .is_err()
            {
                return (f64::INFINITY, 0.0);
            }
            (n, n - num_stationary)
        } else {
            if self
                .check_s_in_bounds(idx_s_start, num_stationary + 1)
                .is_err()
            {
                return (f64::INFINITY, 0.0);
            }
            (0, num_stationary)
        };
        // Preconditions already verified above, so `*_unchecked` access is valid here.
        let s_start = self.s_unchecked(idx_s_start + id_start);
        let ds_stationary = self.s_unchecked(idx_s_start + id_a) - s_start;
        let (mut amax_stationary, mut amin_stationary) =
            self.stationary_boundary_jerk_anchor_bounds(idx_s_start + id_start, ds_stationary);
        if amax_stationary < amin_stationary {
            return (amax_stationary, amin_stationary);
        }

        // ds2start_alpha is the collection of (ds_to_start=s_curr-s_start, alpha) for each stationary interval
        let map_ds2start_alpha = |i_s: (usize, &f64)| {
            // Input: (i_s: (i, &a_linear_curr))
            let s = self.s_unchecked(idx_s_start + i_s.0);
            let ds_to_start = s - s_start;
            let mut alpha = ds_to_start / ds_stationary;
            alpha *= alpha.cbrt();
            // Floor `a`, not `sqrt(a)`: flooring the root squares the effective floor
            // on `a` (1e-10 on the root == 1e-20 on `a`) and inflates `a^{-3/2}` by
            // 1e15. `linearize_constraint_3order_with_floor` floors `a` directly, so
            // both third-order paths must use the same recorded floor.
            let a_linear_half = 1.0 / i_s.1.max(self.a_linearization_floor).sqrt();
            let a_linear_one_half = a_linear_half * a_linear_half * a_linear_half;
            (i_s.0, ds_to_start, alpha, a_linear_half, a_linear_one_half)
        };
        // (i, ds_to_start, alpha, a_linear_half, a_linear_one_half)
        let vec_prepare: Vec<(usize, f64, f64, f64, f64)> = if REV {
            a_linear
                .iter()
                .enumerate()
                .rev()
                .skip(1)
                .take(num_stationary)
                .map(map_ds2start_alpha)
                .collect()
        } else {
            a_linear
                .iter()
                .enumerate()
                .skip(1)
                .take(num_stationary)
                .map(map_ds2start_alpha)
                .collect()
        };
        // 1st-order constraints at the start
        vec_prepare.iter().for_each(|(i, _, alpha, _, _)| {
            let amax_curr = self.amax_unchecked(idx_s_start + i);
            if amax_curr.is_finite() {
                amax_stationary = amax_stationary.min(alpha * amax_curr);
            }
        });
        // 2nd-order constraints at the start
        vec_prepare
            .iter()
            .for_each(|(i, ds_to_start, alpha, _, _)| {
                let (acc_a, acc_b, acc_max) = self.acc_constraints_unchecked(idx_s_start + i);
                izip!(acc_a.iter(), acc_b.iter(), acc_max.iter()).for_each(
                    |(&a_coeff, &b_coeff, &amax_curr)| {
                        if amax_curr.is_finite() {
                            let coef = (a_coeff + b_coeff / (1.5 * ds_to_start)) * alpha;
                            if coef > EPSILON_NUMERIC {
                                amax_stationary = amax_stationary.min(amax_curr / coef);
                            } else if coef < -EPSILON_NUMERIC {
                                amin_stationary = amin_stationary.max(amax_curr / coef);
                            }
                        }
                    },
                )
            });
        // 3rd-order constraints at the start
        for &(i, ds_to_start, alpha, a_linear_half, a_linear_one_half) in vec_prepare.iter() {
            let (jerk_a, jerk_b, jerk_c, jerk_d, jerk_max) =
                self.jerk_constraints_unchecked(idx_s_start + i);
            for (&a_coeff, &b_coeff, &c_coeff, &d_coeff, &jmax_curr) in izip!(
                jerk_a.iter(),
                jerk_b.iter(),
                jerk_c.iter(),
                jerk_d.iter(),
                jerk_max.iter()
            ) {
                if jmax_curr.is_finite() {
                    let coeff: f64 = (a_coeff
                        + b_coeff / (1.5 * ds_to_start)
                        + c_coeff / (4.5 * ds_to_start * ds_to_start)
                        + 0.5 * jmax_curr * a_linear_one_half)
                        * alpha;
                    if coeff > EPSILON_NUMERIC {
                        amax_stationary = amax_stationary
                            .min((1.5 * jmax_curr * a_linear_half - d_coeff) / coeff);
                    } else if coeff < -EPSILON_NUMERIC {
                        amin_stationary = amin_stationary
                            .max((1.5 * jmax_curr * a_linear_half - d_coeff) / coeff);
                    }
                }
            }
        }

        (amax_stationary, amin_stationary)
    }

    /// Evaluate maximum violation magnitudes of the TOPP2 constraints for a profile.
    ///
    /// `a_profile` holds `a = s_dot^2` on the stations starting at `idx_s_start`.
    /// The path acceleration of each interval is reconstructed from it by the
    /// TOPP2 relation `b[k] = (a[k+1] - a[k]) / (2 * ds[k])`, and the acceleration
    /// rows of both endpoint stations are checked against that interval's `b`.
    ///
    /// # Returns
    /// `(exceed_1order, exceed_2order)`, each `<= 0` when fully feasible and
    /// positive when violated. The first-order term covers `0 <= a[k] <= amax[k]`.
    /// Returns `NaN`s if the station range is unavailable or `b` cannot be
    /// reconstructed from `a_profile`, e.g. because it contains a non-finite value.
    pub fn exceed_topp2(&self, idx_s_start: usize, a_profile: &[f64]) -> (f64, f64) {
        let Ok(s) = self.s_vec(idx_s_start, idx_s_start + a_profile.len()) else {
            return (f64::NAN, f64::NAN);
        };
        let Ok(b_profile) = a_to_b_topp2(&s, a_profile) else {
            return (f64::NAN, f64::NAN);
        };
        let mut excced_1order = -a_profile
            .iter()
            .fold(f64::INFINITY, |a, &b| a.min(b))
            .min(0.0);
        for (i, &a) in a_profile.iter().enumerate() {
            let idx_s = idx_s_start + i;
            let amax_curr = self.amax_unchecked(idx_s);
            if amax_curr.is_finite() {
                excced_1order = excced_1order.max(a - amax_curr);
            }
        }
        let mut excced_2order: f64 = 0.0;
        for (i, (a, &b)) in a_profile.windows(2).zip(b_profile.iter()).enumerate() {
            let idx_s = idx_s_start + i;
            let (acc_a_curr, acc_b_curr, acc_max_curr) = self.acc_constraints_unchecked(idx_s);
            for (&acc_a, &acc_b, &acc_max) in
                izip!(acc_a_curr.iter(), acc_b_curr.iter(), acc_max_curr.iter())
            {
                if acc_max.is_finite() {
                    excced_2order = excced_2order.max(acc_a * a[0] + acc_b * b - acc_max);
                }
            }
            let (acc_a_next, acc_b_next, acc_max_next) = self.acc_constraints_unchecked(idx_s + 1);
            for (&acc_a, &acc_b, &acc_max) in
                izip!(acc_a_next.iter(), acc_b_next.iter(), acc_max_next.iter())
            {
                if acc_max.is_finite() {
                    excced_2order = excced_2order.max(acc_a * a[1] + acc_b * b - acc_max);
                }
            }
        }

        (excced_1order, excced_2order)
    }

    /// Evaluate maximum violation magnitudes of the TOPP3 constraints for a profile.
    ///
    /// The third-order term uses the original `sqrt(a)` form, not the linearized
    /// one, so this certifies the profile that is actually delivered rather than
    /// the model the solver optimized. Run it after any post-processing such as
    /// [`force_positive_a`](crate::solver::topp3_socp::force_positive_a), which rewrites
    /// `a` and `b` without knowing about the acceleration and jerk limits.
    ///
    /// The third-order term covers only the intervals the solver models with a
    /// constant `c`, i.e. it skips the two stationary blocks. There `b` follows the
    /// `sdddot`-constant law and `c = 2a/(9 d^2)`, so a finite difference of `b`
    /// across a block edge overestimates `c` by a factor of three and would report
    /// a violation that the model never claimed.
    ///
    /// # Returns
    /// `(exceed_1order, exceed_2order, exceed_3order)`, each `<= 0` when fully
    /// feasible and positive when violated. Returns `NaN`s if the station range is
    /// unavailable, the two profiles disagree in length, or either profile
    /// contains a non-finite value.
    pub fn exceed_topp3(
        &self,
        idx_s_start: usize,
        a_profile: &[f64],
        b_profile: &[f64],
        num_stationary: (usize, usize),
    ) -> (f64, f64, f64) {
        let n = a_profile.len();
        if n < 2
            || b_profile.len() != n
            || !a_profile.iter().chain(b_profile).all(|v| v.is_finite())
        {
            return (f64::NAN, f64::NAN, f64::NAN);
        }
        let Ok(s) = self.s_vec(idx_s_start, idx_s_start + n) else {
            return (f64::NAN, f64::NAN, f64::NAN);
        };

        // First order: 0 <= a[k] <= amax[k].
        let mut exceed_1order: f64 = 0.0;
        for (i, &a) in a_profile.iter().enumerate() {
            exceed_1order = exceed_1order.max(-a);
            let amax = self.amax_unchecked(idx_s_start + i);
            if amax.is_finite() {
                exceed_1order = exceed_1order.max(a - amax);
            }
        }

        // Second order: acc_a * a[k] + acc_b * b[k] <= acc_max.
        let mut exceed_2order: f64 = 0.0;
        for (i, (&a, &b)) in a_profile.iter().zip(b_profile.iter()).enumerate() {
            let (acc_a, acc_b, acc_max) = self.acc_constraints_unchecked(idx_s_start + i);
            for (&r_a, &r_b, &r_max) in izip!(acc_a.iter(), acc_b.iter(), acc_max.iter()) {
                if r_max.is_finite() {
                    exceed_2order = exceed_2order.max(r_a * a + r_b * b - r_max);
                }
            }
        }

        // Third order: sqrt(a[k]) * (jerk_a * a[k] + jerk_b * b[k] + jerk_c * c + jerk_d) <= jerk_max.
        // Each interval contributes its own `c` at both of its endpoints, matching
        // how the solver emits the rows.
        let mut exceed_3order: f64 = 0.0;
        let k_first = num_stationary.0;
        let k_last = (n - 1).saturating_sub(num_stationary.1);
        for k in k_first..k_last {
            let ds = s[k + 1] - s[k];
            if ds <= 0.0 {
                return (f64::NAN, f64::NAN, f64::NAN);
            }
            let c = (b_profile[k + 1] - b_profile[k]) / ds;
            for i in [k, k + 1] {
                let a = a_profile[i];
                // Negative `a` is already reported by the first-order term, and
                // sqrt() of it would poison the maximum. Non-finite input was
                // rejected above, because f64::max drops NaN.
                if a < 0.0 {
                    continue;
                }
                let b = b_profile[i];
                let a_sqrt = a.sqrt();
                let (jerk_a, jerk_b, jerk_c, jerk_d, jerk_max) =
                    self.jerk_constraints_unchecked(idx_s_start + i);
                for (&r_a, &r_b, &r_c, &r_d, &r_max) in izip!(
                    jerk_a.iter(),
                    jerk_b.iter(),
                    jerk_c.iter(),
                    jerk_d.iter(),
                    jerk_max.iter()
                ) {
                    if r_max.is_finite() {
                        exceed_3order =
                            exceed_3order.max(a_sqrt * (r_a * a + r_b * b + r_c * c + r_d) - r_max);
                    }
                }
            }
        }

        (exceed_1order, exceed_2order, exceed_3order)
    }

    /// Test-only projection of `a_ori` toward feasible profile `a_fea` for TOPP2.
    ///
    /// # Returns
    /// Interpolation factor applied to `a_ori` (in `[0, 1]` in typical cases).
    ///
    /// # Errors
    /// Returns [`ConstraintError::InfeasibleReference`](crate::diag::ConstraintError::InfeasibleReference) if provided `a_fea` itself
    /// violates active constraints.
    #[cfg(test)]
    pub(crate) fn project_to_feasible_topp2(
        &self,
        a_ori: &mut [f64],
        a_fea: &[f64],
        idx_s_start: usize,
    ) -> Result<f64, ConstraintError> {
        if a_ori.len() != a_fea.len() {
            return Err(ConstraintError::NoMatchDimensions);
        }
        if a_ori.is_empty() {
            return Ok(1.0);
        }
        if a_ori.len() == 1 {
            let a_ori = a_ori.first_mut().unwrap();
            let a_fea = a_fea.first().unwrap();
            return if (*a_ori - a_fea).abs() > EPSILON_NUMERIC {
                *a_ori = *a_fea;
                Ok(0.0)
            } else {
                return Ok(1.0);
            };
        }
        // Modify a_ori to meet the boundary conditions
        let s = self
            .s_vec(idx_s_start, idx_s_start + a_ori.len())
            .map_err(|_| ConstraintError::NoGivenQInfo)?;
        let delta_a0 = a_fea.first().unwrap() - a_ori.first().unwrap();
        let delta_af = a_fea.last().unwrap() - a_ori.last().unwrap();
        // linear interpolation
        let &s0 = s.first().unwrap();
        let ds_tol_down = 1.0 / (s.last().unwrap() - s0);
        for (a_ori, &s) in a_ori.iter_mut().zip(s.iter()) {
            let alpha = (s - s0) * ds_tol_down;
            *a_ori += alpha * delta_af + (1.0 - alpha) * delta_a0;
        }
        // Project: first-order
        let mut alpha: f64 = 1.0;
        for (i, (&a_o, &a_f)) in a_ori.iter().zip(a_fea.iter()).enumerate() {
            let idx_s = idx_s_start + i;
            let amax_curr = self.amax_unchecked(idx_s);
            if amax_curr.is_finite() {
                let exceed_ori = a_o - amax_curr;
                let exceed_fea = a_f - amax_curr;
                if exceed_fea > 0.0 {
                    crate::verbosity_log!(
                        crate::diag::Verbosity::Debug,
                        "a_ori[{i}] = {a_o}, a_fea[{i}] = {a_f}, amax_curr = {amax_curr}"
                    );
                    return Err(ConstraintError::InfeasibleReference);
                }
                if exceed_ori > 0.0 {
                    alpha = alpha.min(-exceed_fea / (exceed_ori - exceed_fea));
                }
            }
        }
        // Project: second-order
        let b_ori = a_to_b_topp2(&s, a_ori).map_err(|_| ConstraintError::NoMatchDimensions)?;
        let b_fea = a_to_b_topp2(&s, a_fea).map_err(|_| ConstraintError::NoMatchDimensions)?;
        for (i, (a_o, a_f, &b_o, &b_f)) in izip!(
            a_ori.windows(2),
            a_fea.windows(2),
            b_ori.iter(),
            b_fea.iter()
        )
        .enumerate()
        {
            let idx_s = idx_s_start + i;
            let (acc_a_curr, acc_b_curr, acc_max_curr) = self.acc_constraints_unchecked(idx_s);
            for (&acc_a, &acc_b, &acc_max) in
                izip!(acc_a_curr.iter(), acc_b_curr.iter(), acc_max_curr.iter())
            {
                if acc_max.is_finite() {
                    let exceed_ori = acc_a * a_o[0] + acc_b * b_o - acc_max;
                    let exceed_fea = acc_a * a_f[0] + acc_b * b_f - acc_max;
                    if exceed_fea > 0.0 {
                        crate::verbosity_log!(
                            crate::diag::Verbosity::Debug,
                            "a_ori[{}] = {}, b_ori[{}] = {}, exceed_ori={}, a_fea[{}] = {}, b_fea[{}] = {}, exceed_fea = {}",
                            i,
                            a_o[0],
                            i,
                            b_o,
                            exceed_ori,
                            i,
                            a_f[0],
                            i,
                            b_f,
                            exceed_fea
                        );
                        return Err(ConstraintError::InfeasibleReference);
                    }
                    if exceed_ori > 0.0 {
                        alpha = alpha.min(-exceed_fea / (exceed_ori - exceed_fea));
                    }
                }
            }
            let (acc_a_next, acc_b_next, acc_max_next) = self.acc_constraints_unchecked(idx_s + 1);
            for (&acc_a, &acc_b, &acc_max) in
                izip!(acc_a_next.iter(), acc_b_next.iter(), acc_max_next.iter())
            {
                if acc_max.is_finite() {
                    let exceed_ori = acc_a * a_o[1] + acc_b * b_o - acc_max;
                    let exceed_fea = acc_a * a_f[1] + acc_b * b_f - acc_max;
                    if exceed_fea > 0.0 {
                        crate::verbosity_log!(
                            crate::diag::Verbosity::Debug,
                            "a_ori[{}] = {}, b_ori[{}] = {}, exceed_ori={}, a_fea[{}] = {}, b_fea[{}] = {}, exceed_fea = {}",
                            i + 1,
                            a_o[1],
                            i,
                            b_o,
                            exceed_ori,
                            i + 1,
                            a_f[1],
                            i,
                            b_f,
                            exceed_fea
                        );
                        return Err(ConstraintError::InfeasibleReference);
                    }
                    if exceed_ori > 0.0 {
                        alpha = alpha.min(-exceed_fea / (exceed_ori - exceed_fea));
                    }
                }
            }
        }
        alpha = 1.0 - alpha;
        for (a_o, a_f) in a_ori.iter_mut().zip(a_fea.iter()) {
            *a_o += alpha * (a_f - *a_o);
        }

        Ok(alpha)
    }

    /// Expand second-order TOPP2 constraints at edge `idx_s -> idx_s+1` into linear form.
    ///
    /// # Output format
    /// Push tuples `(coef_left, coef_right, rhs)` into `a_b`.
    /// - `REV = false`: `coef_left * a[idx_s] + coef_right * a[idx_s+1] <= rhs`
    /// - `REV = true`:  `coef_left * a[idx_s+1] + coef_right * a[idx_s] <= rhs`
    ///
    /// The function appends rows and does not clear `a_b`.
    pub(crate) fn fill_acc_topp2<const REV: bool>(
        &self,
        a_b: &mut Vec<(f64, f64, f64)>,
        idx_s: usize,
    ) {
        let ds_double_down = 0.5 / (self.s_unchecked(idx_s + 1) - self.s_unchecked(idx_s));
        let (acc_a_curr, acc_b_curr, acc_max_curr) = self.acc_constraints_unchecked(idx_s);
        let (acc_a_next, acc_b_next, acc_max_next) = self.acc_constraints_unchecked(idx_s + 1);
        if REV {
            for (&acc_a, &acc_b, &acc_max) in
                izip!(acc_a_curr.iter(), acc_b_curr.iter(), acc_max_curr.iter())
            {
                // acc_a * a_curr + acc_b * b_curr <= acc_max
                // acc_a * a_curr + acc_b * (a_next - a_curr) * ds_double_down <= acc_max
                // acc_b * ds_double_down * a_next + (acc_a - acc_b * ds_double_down) * a_curr <= acc_max
                if acc_max.is_finite() {
                    let acc_b_scaled = acc_b * ds_double_down;
                    a_b.push((acc_b_scaled, acc_a - acc_b_scaled, acc_max));
                }
            }
            for (&acc_a, &acc_b, &acc_max) in
                izip!(acc_a_next.iter(), acc_b_next.iter(), acc_max_next.iter())
            {
                // acc_a * a_next + acc_b * b_curr <= acc_max
                // acc_a * a_next + acc_b * (a_next - a_curr) * ds_double_down <= acc_max
                // (acc_a +acc_b * ds_double_down) * a_next - acc_b * ds_double_down *a_curr <= acc_max
                if acc_max.is_finite() {
                    let acc_b_scaled = acc_b * ds_double_down;
                    a_b.push((acc_a + acc_b_scaled, -acc_b_scaled, acc_max));
                }
            }
        } else {
            for (&acc_a, &acc_b, &acc_max) in
                izip!(acc_a_curr.iter(), acc_b_curr.iter(), acc_max_curr.iter())
            {
                // acc_a * a_curr + acc_b * b_curr <= acc_max
                // acc_a * a_curr + acc_b * (a_next - a_curr) * ds_double_down <= acc_max
                // acc_b * ds_double_down * a_next + (acc_a - acc_b * ds_double_down) * a_curr <= acc_max
                if acc_max.is_finite() {
                    let acc_b_scaled = acc_b * ds_double_down;
                    a_b.push((acc_a - acc_b_scaled, acc_b_scaled, acc_max));
                }
            }
            for (&acc_a, &acc_b, &acc_max) in
                izip!(acc_a_next.iter(), acc_b_next.iter(), acc_max_next.iter())
            {
                // acc_a * a_next + acc_b * b_curr <= acc_max
                // acc_a * a_next + acc_b * (a_next - a_curr) * ds_double_down <= acc_max
                // (acc_a +acc_b * ds_double_down) * a_next - acc_b * ds_double_down *a_curr <= acc_max
                if acc_max.is_finite() {
                    let acc_b_scaled = acc_b * ds_double_down;
                    a_b.push((-acc_b_scaled, acc_a + acc_b_scaled, acc_max));
                }
            }
        }
    }
}

/// The mode for updating valid rows.
enum ModeUpdateValidRows {
    SetValues,
    AddValues,
}

/// The mode for popping constraints.
pub enum ModePopConstraints {
    /// [`ModePopConstraints::CutAtIdxS`](crate::constraints::ModePopConstraints::CutAtIdxS)`(cut)`: keep stations with `id >= cut` or `id < cut`.
    CutAtIdxS(usize),
    /// [`ModePopConstraints::PopNCols`](crate::constraints::ModePopConstraints::PopNCols)`(n)`: remove first or last `n` logical stations.
    PopNCols(usize),
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::robot::Robot;

    fn stationary_boundary_fixture(
        s: &[f64],
        boundary: usize,
        coefficient: f64,
        rhs: f64,
    ) -> Result<Constraints, ConstraintError> {
        let mut constraints = Constraints::with_capacity(1, s.len());
        constraints.with_s(s)?;
        let zero = DMatrix::zeros(1, 1);
        let jerk_c = DMatrix::from_element(1, 1, coefficient);
        // This finite term vanishes at the zero-speed boundary; it must not
        // enter the anchor bound through a floored inverse-speed formula.
        let jerk_d = DMatrix::from_element(1, 1, 1.0e100);
        let jerk_max = DMatrix::from_element(1, 1, rhs);
        constraints.with_constraint_3order(
            &zero.as_view(),
            &zero.as_view(),
            &jerk_c.as_view(),
            &jerk_d.as_view(),
            &jerk_max.as_view(),
            boundary,
            false,
        )?;
        Ok(constraints)
    }

    #[test]
    fn stationary_boundary_jerk_bounds_cover_head_tail_and_multiple_intervals()
    -> Result<(), ConstraintError> {
        let s = [0.0, 0.25, 1.0, 1.5, 2.0];
        let a_linear = [0.0, 1.0, 1.0, 1.0, 0.0];
        for reverse in [false, true] {
            let boundary = if reverse { s.len() - 1 } else { 0 };
            let constraints = stationary_boundary_fixture(&s, boundary, 0.75, 3.0)?;
            for count in [1, 2] {
                let anchor = if reverse { boundary - count } else { count };
                let distance = (s[anchor] - s[boundary]).abs();
                let (upper, lower) = if reverse {
                    constraints.stationary_constraint_topp3::<true>(&a_linear, 0, count)
                } else {
                    constraints.stationary_constraint_topp3::<false>(&a_linear, 0, count)
                };
                let expected = (4.5 * distance * distance * 3.0 / 0.75).cbrt().powi(2);
                assert_eq!(lower, 0.0);
                assert!(upper > 0.0 && upper <= expected);
                assert!((upper / expected - 1.0).abs() < 1.0e-12);
                let path_jerk = (upper.sqrt() / distance / 4.5) * (upper / distance);
                assert!(0.75 * path_jerk <= 3.0);
            }
            let opposite = if reverse {
                constraints.stationary_constraint_topp3::<false>(&a_linear, 0, 1)
            } else {
                constraints.stationary_constraint_topp3::<true>(&a_linear, 0, 1)
            };
            assert_eq!(opposite, (f64::INFINITY, 0.0));
        }
        Ok(())
    }

    #[test]
    fn stationary_boundary_jerk_bounds_handle_signed_and_disabled_rows()
    -> Result<(), ConstraintError> {
        let s = [0.0, 1.0];
        for (coefficient, rhs) in [
            (0.0, 0.0),
            (0.0, 1.0),
            (-1.0, 0.0),
            (-1.0, 1.0),
            (f64::NAN, f64::INFINITY),
        ] {
            let constraints = stationary_boundary_fixture(&s, 0, coefficient, rhs)?;
            assert_eq!(
                constraints.stationary_boundary_jerk_anchor_bounds(0, 1.0),
                (f64::INFINITY, 0.0),
            );
        }
        for (coefficient, rhs) in [
            (0.0, -1.0),
            (1.0, -1.0),
            (1.0, f64::NAN),
            (1.0, f64::NEG_INFINITY),
            (f64::INFINITY, 1.0),
        ] {
            let constraints = stationary_boundary_fixture(&s, 0, coefficient, rhs)?;
            let (upper, lower) = constraints.stationary_boundary_jerk_anchor_bounds(0, 1.0);
            assert!(upper < lower);
        }
        let mut constraints = stationary_boundary_fixture(&s, 0, -2.0, -4.0)?;
        let (upper, lower) = constraints.stationary_boundary_jerk_anchor_bounds(0, -1.0);
        assert_eq!(upper, f64::INFINITY);
        assert!(lower > 0.0);
        assert!(-2.0 * (lower.sqrt() / 4.5) * lower <= -4.0);
        let zero = DMatrix::zeros(1, 1);
        let positive_c = DMatrix::from_element(1, 1, 2.0);
        let positive_rhs = DMatrix::from_element(1, 1, 4.0);
        constraints.with_constraint_3order(
            &zero.as_view(),
            &zero.as_view(),
            &positive_c.as_view(),
            &zero.as_view(),
            &positive_rhs.as_view(),
            0,
            false,
        )?;
        let (upper, lower) = constraints.stationary_boundary_jerk_anchor_bounds(0, -1.0);
        assert!(upper.is_finite() && upper > 0.0);
        assert_eq!(upper, lower);
        assert!((2.0 * (upper.sqrt() / 4.5) * upper - 4.0).abs() < 1.0e-12);
        let constraints = stationary_boundary_fixture(&s, 0, 1.0, 0.0)?;
        assert_eq!(
            constraints.stationary_boundary_jerk_anchor_bounds(0, 1.0),
            (0.0, 0.0)
        );
        Ok(())
    }

    #[test]
    fn stationary_boundary_jerk_bounds_keep_small_coefficients_and_scaled_limits()
    -> Result<(), ConstraintError> {
        let s = [0.0, 1.0];
        let constraints = stationary_boundary_fixture(&s, 0, 1.0e-20, 1.0e-20)?;
        let (upper, _) = constraints.stationary_boundary_jerk_anchor_bounds(0, 1.0);
        assert!((upper / 4.5_f64.cbrt().powi(2) - 1.0).abs() < 1.0e-12);
        let constraints = stationary_boundary_fixture(&s, 0, 1.0e300, 1.0e-300)?;
        let (upper, _) = constraints.stationary_boundary_jerk_anchor_bounds(0, 1.0e150);
        let expected = (4.5e-300_f64).cbrt().powi(2);
        assert!(upper.is_finite() && upper > 0.0);
        assert!((upper / expected - 1.0).abs() < 1.0e-12);
        Ok(())
    }

    #[test]
    fn adaptive_linearization_ignores_stationary_boundary_stencils() -> Result<(), ConstraintError>
    {
        let mut constraints = Constraints::with_capacity(1, 3);
        constraints.with_s(&[0.0, 1.0, 2.0][..])?;
        constraints.amax.fill(10.0);

        let zero = DMatrix::zeros(1, 1);
        let jerk_c = DMatrix::from_element(1, 1, 1.0);
        let jerk_max = DMatrix::from_element(1, 1, 0.5);
        constraints.with_constraint_3order(
            &zero.as_view(),
            &zero.as_view(),
            &jerk_c.as_view(),
            &zero.as_view(),
            &jerk_max.as_view(),
            1,
            false,
        )?;

        let cases = [
            (
                [2.8233333333333333, 1.0, 0.0],
                [-1.1566666666666667, -2.0 / 3.0, 0.0],
                (0, 1),
                0.49,
            ),
            (
                [0.0, 1.0, 2.8233333333333333],
                [0.0, 2.0 / 3.0, 1.1566666666666667],
                (1, 0),
                0.49,
            ),
            ([1.49, 1.0, 1.49], [-0.49, 0.0, 0.49], (0, 0), 0.49),
        ];
        for (a, b, num_stationary, c_ordinary) in cases {
            constraints.linearize_constraint_3order_adaptive(&a, &b, 0, 1.0e-10, num_stationary)?;
            let (jerk_a, jerk_b, jerk_c, jerk_max) =
                constraints.jerk_linear_constraints_unchecked(1);
            let lhs = jerk_a[(0, 0)] * a[1] + jerk_b[(0, 0)] * b[1] + jerk_c[(0, 0)] * c_ordinary;
            assert!(
                lhs <= jerk_max[(0, 0)],
                "ordinary-side witness was excluded for num_stationary={num_stationary:?}: \
                 lhs={lhs}, rhs={}",
                jerk_max[(0, 0)]
            );
        }
        Ok(())
    }

    /// Three stations with `a <= 4`, `b <= 1` and `sqrt(a) * c <= 1` everywhere.
    fn exceed_topp3_fixture() -> Result<Constraints, ConstraintError> {
        let mut constraints = Constraints::with_capacity(1, 3);
        constraints.with_s(&[0.0, 1.0, 2.0][..])?;
        constraints.amax.fill(4.0);
        let zero = DMatrix::zeros(1, 3);
        let one = DMatrix::from_element(1, 3, 1.0);
        constraints.with_constraint_2order(
            &zero.as_view(),
            &one.as_view(),
            &one.as_view(),
            0,
            false,
        )?;
        constraints.with_constraint_3order(
            &zero.as_view(),
            &zero.as_view(),
            &one.as_view(),
            &zero.as_view(),
            &one.as_view(),
            0,
            false,
        )?;
        Ok(constraints)
    }

    #[test]
    fn exceed_topp3_keeps_finite_profiles() -> Result<(), ConstraintError> {
        let constraints = exceed_topp3_fixture()?;
        let a = [1.0, 1.0, 1.0];
        assert_eq!(
            constraints.exceed_topp3(0, &a, &[0.0, 0.0, 0.0], (0, 0)),
            (0.0, 0.0, 0.0)
        );
        // b[2] = 4 exceeds 1 by 3, and c = 2 gives sqrt(1) * 2 - 1 = 1.
        assert_eq!(
            constraints.exceed_topp3(0, &a, &[0.0, 2.0, 4.0], (0, 0)),
            (0.0, 3.0, 1.0)
        );
        Ok(())
    }

    #[test]
    fn exceed_topp3_reports_non_finite_profiles_as_nan() -> Result<(), ConstraintError> {
        let constraints = exceed_topp3_fixture()?;
        let finite_a = [1.0, 1.0, 1.0];
        let finite_b = [0.0, 0.0, 0.0];
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let bad_a = [1.0, bad, 1.0];
            let bad_b = [0.0, bad, 0.0];
            for (a, b) in [(&bad_a, &finite_b), (&finite_a, &bad_b), (&bad_a, &bad_b)] {
                let (e1, e2, e3) = constraints.exceed_topp3(0, a, b, (0, 0));
                assert!(
                    e1.is_nan() && e2.is_nan() && e3.is_nan(),
                    "a={a:?}, b={b:?} gave ({e1}, {e2}, {e3})"
                );
            }
        }
        Ok(())
    }

    #[test]
    fn test_constraints() -> Result<(), ConstraintError> {
        let n: usize = 100;
        let qs = [|s: f64| s.sin(), |s: f64| s.cos()];
        let dqs = [|s: f64| s.cos(), |s: f64| -s.sin()];
        let ddqs = [|s: f64| -s.sin(), |s: f64| -s.cos()];
        let dddqs = [|s: f64| -s.cos(), |s: f64| s.sin()];
        let mut robot = Robot::with_capacity(2, 10);

        let s = DMatrix::<f64>::from_fn(1, n, |_r, c| (c as f64) * 0.1);
        let q = DMatrix::<f64>::from_fn(2, n, |r, c| qs[r]((c as f64) * 0.1));
        let dq = DMatrix::<f64>::from_fn(2, n, |r, c| dqs[r]((c as f64) * 0.1));
        let ddq = DMatrix::<f64>::from_fn(2, n, |r, c| ddqs[r]((c as f64) * 0.1));
        let dddq = DMatrix::<f64>::from_fn(2, n, |r, c| dddqs[r]((c as f64) * 0.1));
        // Test idx_s after adding s
        robot.constraints.with_s(&s.columns(0, 5))?;
        assert_eq!(robot.constraints.idx_s, 0);
        // Test adding q
        assert!(
            robot
                .constraints
                .with_q(
                    &q.as_view(),
                    &dq.as_view(),
                    &ddq.as_view(),
                    Some(&dddq.as_view()),
                    0,
                )
                .is_err()
        );
        crate::verbosity_log!(
            crate::diag::Verbosity::Summary,
            "valid_rows_q before adding q:"
        );
        println_validrows(&robot.constraints.valid_rows_q);
        println_constraints(&robot.constraints);
        robot.constraints.with_q(
            &q.columns(0, 4),
            &dq.columns(0, 4),
            &ddq.columns(0, 4),
            Some(&dddq.columns(0, 4)),
            0,
        )?;
        crate::verbosity_log!(
            crate::diag::Verbosity::Summary,
            "valid_rows_q after adding q:"
        );
        println_validrows(&robot.constraints.valid_rows_q);
        println_constraints(&robot.constraints);
        assert_eq!(robot.constraints.valid_rows_q.len(), 2);
        // Test adding non-increasing s
        assert!(robot.constraints.with_s(&s.columns(3, 5)).is_err());
        // Test adding more s
        crate::verbosity_log!(
            crate::diag::Verbosity::Summary,
            "capacity before adding more s: {}",
            robot.constraints.capacity_col
        );
        robot.constraints.with_s(&s.columns(5, 23))?;
        crate::verbosity_log!(
            crate::diag::Verbosity::Summary,
            "capacity after adding more s: {}",
            robot.constraints.capacity_col
        );
        // Test with axial velocity constraints
        let axial_velocity_max = DMatrix::<f64>::from_element(2, 21, 1.0);
        let axial_velocity_min = DMatrix::<f64>::from_element(2, 21, -2.0);
        crate::verbosity_log!(
            crate::diag::Verbosity::Summary,
            "valid_rows_q after adding new s:"
        );
        println_validrows(&robot.constraints.valid_rows_q);
        assert!(
            robot
                .with_axial_velocity(
                    &axial_velocity_max.as_view(),
                    &axial_velocity_min.as_view(),
                    1,
                )
                .is_err()
        );
        robot.constraints.with_q(
            &q.columns(4, 24),
            &dq.columns(4, 24),
            &ddq.columns(4, 24),
            Some(&dddq.columns(4, 24)),
            4,
        )?;
        robot.with_axial_velocity(
            &axial_velocity_max.as_view(),
            &axial_velocity_min.as_view(),
            1,
        )?;
        crate::verbosity_log!(
            crate::diag::Verbosity::Summary,
            "valid_rows_q after adding new q:"
        );
        println_validrows(&robot.constraints.valid_rows_q);
        println_constraints(&robot.constraints);
        crate::verbosity_log!(
            crate::diag::Verbosity::Summary,
            "amax={:?}",
            robot.constraints.amax
        );
        assert!(robot.constraints.amax[(0, 9)].is_finite());
        // Test with_axial_acceleration constraints
        let axial_acceleration_max = DMatrix::<f64>::from_element(2, 15, 0.5);
        let axial_acceleration_min = DMatrix::<f64>::from_element(2, 15, -0.5);
        robot.with_axial_acceleration(
            &axial_acceleration_max.as_view(),
            &axial_acceleration_min.as_view(),
            5,
        )?;
        crate::verbosity_log!(
            crate::diag::Verbosity::Summary,
            "valid_rows_acc after adding axial acceleration:"
        );
        println_validrows(&robot.constraints.valid_rows_acc);
        // Test popping front constraints
        robot
            .constraints
            .pop_front(ModePopConstraints::PopNCols(10));
        crate::verbosity_log!(
            crate::diag::Verbosity::Summary,
            "After popping front 10 cols:"
        );
        println_constraints(&robot.constraints);
        crate::verbosity_log!(
            crate::diag::Verbosity::Summary,
            "amax={:?}",
            robot.constraints.amax
        );
        crate::verbosity_log!(
            crate::diag::Verbosity::Summary,
            "valid_rows_q after popping front:"
        );
        println_validrows(&robot.constraints.valid_rows_q);
        crate::verbosity_log!(
            crate::diag::Verbosity::Summary,
            "valid_rows_acc after popping front:"
        );
        println_validrows(&robot.constraints.valid_rows_acc);
        assert!(!robot.constraints.amax[(0, 9)].is_finite());
        assert_eq!(robot.constraints.idx_s, 10);
        // Test CircularMatrix indexing and multi-acc-constraints
        let axial_acceleration_max = DMatrix::<f64>::from_element(2, 28, 0.5);
        let axial_acceleration_min = DMatrix::<f64>::from_element(2, 28, -0.5);
        robot.constraints.with_s(&s.columns(28, 26))?;
        robot.constraints.with_q(
            &q.columns(28, 26),
            &dq.columns(28, 26),
            &ddq.columns(28, 26),
            Some(&dddq.columns(28, 26)),
            28,
        )?;
        robot.with_axial_acceleration(
            &axial_acceleration_max.as_view(),
            &axial_acceleration_min.as_view(),
            19,
        )?;
        crate::verbosity_log!(
            crate::diag::Verbosity::Summary,
            "valid_rows_acc after adding more axial acceleration:"
        );
        println_validrows(&robot.constraints.valid_rows_acc);
        println_constraints(&robot.constraints);
        // Test with axial velocity again
        robot.with_axial_velocity(
            &axial_velocity_max.as_view(),
            &axial_velocity_min.as_view(),
            15,
        )?;
        crate::verbosity_log!(
            crate::diag::Verbosity::Summary,
            "amax={:?}",
            robot.constraints.amax
        );
        // Test pop back
        robot.constraints.pop_back(ModePopConstraints::PopNCols(25));
        crate::verbosity_log!(
            crate::diag::Verbosity::Summary,
            "After popping back 15 cols:"
        );
        println_constraints(&robot.constraints);
        crate::verbosity_log!(
            crate::diag::Verbosity::Summary,
            "amax={:?}",
            robot.constraints.amax
        );
        crate::verbosity_log!(
            crate::diag::Verbosity::Summary,
            "valid_rows_q after popping back:"
        );
        println_validrows(&robot.constraints.valid_rows_q);
        crate::verbosity_log!(
            crate::diag::Verbosity::Summary,
            "valid_rows_acc after popping back:"
        );
        println_validrows(&robot.constraints.valid_rows_acc);

        Ok(())
    }

    fn println_validrows(valid_rows: &ValidRows) {
        for (k, v) in valid_rows.iter() {
            print!("  [{},{}): {}, ", v.0, k, v.1);
        }
        crate::verbosity_log!(crate::diag::Verbosity::Summary, "");
    }

    fn println_constraints(constraints: &Constraints) {
        crate::verbosity_log!(
            crate::diag::Verbosity::Summary,
            "Constraints: idx_s: {}, len: {}, capacity_col: {}, head_col: {}",
            constraints.idx_s,
            constraints.len,
            constraints.capacity_col,
            constraints.head_col
        );
    }
}
