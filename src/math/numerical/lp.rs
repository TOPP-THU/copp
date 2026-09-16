//! Incremental linear-programming kernels in 1D/2D.
//!
//! # Method identity
//! The functions in this module are low-level numeric engines used by TOPP/COPP
//! planners. They operate on half-space form constraints and provide:
//! - direct 1D/2D incremental LP solves,
//! - warm-start interfaces,
//! - helper normalization and tiny linear-system primitives.

use crate::math::numerical::cross_product_2d;
use core::f64;

/// Safety multiplier used when propagating tolerance across dimension reductions.
const EPS_SCALE: f64 = 10.0;
/// Near-zero threshold for feasibility/normalization branch decisions.
pub(crate) const EPS_ZERO: f64 = 1e-9;
/// Lower bound for coefficient normalization denominators.
const EPS_NORMALIZE: f64 = 1e-3;
/// Initial half-width of the box the incremental LP kernels pose their problems
/// over, and the sentinel value of a warm start that means "start beyond the
/// set in the objective direction". The box grows by [`LP_BOX_GROWTH`] whenever
/// an answer lands on its edge, so this is a starting point, not a limit.
pub(crate) const LP_BOUND: f64 = 1e6;
/// Relative floating-point tolerance (about 450 ulps) for cancellation-sensitive
/// comparisons: a quantity computed as a difference of terms is treated as zero
/// when it is below `EPS_ROUNDING` times the magnitude of those terms.
///
/// This is deliberately not the feasibility tolerance. A violation at the
/// rounding level of `a*x + b*y - c` carries no information about the row, and
/// pivoting onto it breaks the incremental invariant when the row is nearly
/// parallel to the active one; a violation above it is real and must pivot,
/// however small it is in absolute terms.
const EPS_ROUNDING: f64 = 1e-13;

/// Growth factor of the implicit box when an answer lands on its edge.
///
/// `LP_BOUND` is not a guard against unbounded problems, it is part of the
/// problem statement, and the incremental solvers start from its edge and can
/// only move inwards. A problem whose optimum lies beyond it therefore gets the
/// box edge as its answer: with the path parameter in millimetres `a = sdot^2`
/// passes `1e6` at 1000 mm/s, and the profile was observed to stick there. An
/// answer on the box is the signature of that, so the box grows and the solve
/// repeats; only an unbounded problem keeps growing to the cap.
const LP_BOX_GROWTH: f64 = 1e3;
/// How many times the box may grow before the answer is taken as it is; an
/// unbounded problem is then reported at the edge of a `1e24` box.
const LP_BOX_GROWTH_STEPS: usize = 6;
/// Fraction of the box half-width beyond which an answer counts as sitting on
/// the box. Once the box has grown past the problem, a genuine optimum lies
/// well inside it (`LP_BOX_GROWTH` is `1e3`, the threshold one half).
const LP_BOX_SATURATION: f64 = 0.5;

/// Whether a two-dimensional answer sits on the box: a bounded problem whose
/// answer does was clipped by it, and an unbounded one has nowhere else to go.
#[inline(always)]
fn lp_2d_saturated(x: f64, y: f64, bound: f64) -> bool {
    let edge = LP_BOX_SATURATION * bound;
    x.is_finite() && y.is_finite() && (y >= edge || x.abs() >= edge)
}

/// Error bound for the sign of a 2x2 determinant, as a multiple of the sum of
/// the magnitudes of the two products that cancel into it.
///
/// This is Shewchuk's `ccwerrboundA = (3 + 16*eps)*eps` with `eps = 2^-53`, the
/// bound under which the floating-point sign of `orient2d` is proven reliable
/// (Shewchuk, *Adaptive Precision Floating-Point Arithmetic and Fast Robust
/// Geometric Predicates*, DCG 18(3), 1997). Comparing the determinant against a
/// fixed epsilon instead would make the test depend on how the rows happen to
/// be scaled: a determinant of `1e-12` is decisive for rows of magnitude
/// `1e-8` and pure noise for rows of magnitude `1e3`.
const CCW_ERRBOUND_A: f64 = 3.330_669_073_875_47e-16;

/// LP tolerance bundle for numerical-robustness controls.
///
/// This mirrors the options naming style used in `src/copp` modules
/// (`*Options` + `*OptionsBuilder`) so tolerance policies are explicit and
/// configurable.
#[derive(Clone, Copy, Debug)]
pub(crate) struct LpToleranceOptions {
    /// Base feasibility tolerance used by LP inequality checks.
    pub(crate) feas_tol: f64,
    /// Tolerance scale factor used when reducing dimension (2D->1D).
    pub(crate) reduce_dim_scale: f64,
}

/// Builder for [`LpToleranceOptions`].
pub(crate) struct LpToleranceOptionsBuilder {
    pub feas_tol: f64,
    pub reduce_dim_scale: f64,
}

impl Default for LpToleranceOptionsBuilder {
    fn default() -> Self {
        Self {
            feas_tol: EPS_ZERO,
            reduce_dim_scale: EPS_SCALE,
        }
    }
}

impl LpToleranceOptionsBuilder {
    #[inline(always)]
    pub(crate) fn builder() -> Self {
        Self::default()
    }

    #[inline(always)]
    pub(crate) fn feas_tol(mut self, feas_tol: f64) -> Self {
        self.feas_tol = feas_tol;
        self
    }

    #[inline(always)]
    pub(crate) fn reduce_dim_scale(mut self, reduce_dim_scale: f64) -> Self {
        self.reduce_dim_scale = reduce_dim_scale;
        self
    }

    #[inline(always)]
    pub(crate) fn build(self) -> LpToleranceOptions {
        LpToleranceOptions {
            feas_tol: self.feas_tol,
            reduce_dim_scale: self.reduce_dim_scale,
        }
    }
}

impl LpToleranceOptions {
    #[inline(always)]
    pub(crate) fn with_feas_tol(feas_tol: f64) -> Self {
        LpToleranceOptionsBuilder::builder()
            .feas_tol(feas_tol)
            .reduce_dim_scale(EPS_SCALE)
            .build()
    }
}

#[inline(always)]
/// Scale each 2D half-space row to a unit normal, so that its right-hand side
/// becomes the signed distance from the origin to its boundary.
///
/// Rows with no normal at all are mapped to zeros; they carry no direction and
/// no point can violate them.
///
/// The divisor is the row's own norm and nothing else. Clamping it from below —
/// as this did, at `EPS_NORMALIZE` — leaves a row of norm `1e-10` at norm
/// `1e-7` rather than at 1, so every absolute tolerance downstream is inflated
/// by `1e-3 / norm` on that row, up to a factor of a million. The consequence
/// is not a loss of accuracy but a loss of the constraint: multiplying one row
/// by `1e-10` changes nothing geometrically, yet was observed to move the
/// objective from `-6.91e2` to `+4.48e4` because the row stopped being seen.
pub(crate) fn normalize_lp2d(a_b: &mut [(f64, f64, f64)]) {
    for w in a_b.iter_mut() {
        let norm = (w.0 * w.0 + w.1 * w.1).sqrt();
        *w = if norm > 0.0 && norm.is_finite() {
            let norm_inv = 1.0 / norm;
            (w.0 * norm_inv, w.1 * norm_inv, w.2 * norm_inv)
        } else {
            (0.0, 0.0, 0.0)
        };
    }
}

/// Relative tolerance for calling a cancelled coefficient zero.
///
/// The coefficient reaching [`lp_1d_core`] from [`incre_step_2d`] is a
/// difference of two products, so this is the same question `solve_2x2` asks of
/// its determinant and takes the same bound.
const LP1D_CANCEL_EPS: f64 = CCW_ERRBOUND_A;

/// How many times a row found violated after the incremental pass may be
/// reprocessed. Exact arithmetic needs one; the rest is room for rounding to
/// bounce a pair of rows against each other.
const MAX_DEFERRED_ROUNDS: usize = 4;

/// Target a smaller residual than the public feasibility tolerance when the
/// rare degeneracy recovery path has already paid for a full prefix scan.  The
/// remaining margin lets later reductions accumulate some rounding without
/// immediately turning the repaired row into another violation.
const LP2D_REPAIR_TOL_SCALE: f64 = 0.25;

/// One incremental step: pin row `row` as an equality and optimize along it
/// subject to rows `0..upto`, moving `(x, y)` onto the result.
///
/// Returns `false` when that reveals the system to be infeasible.
///
/// `upto` is separate from `row` so the same step serves both phases: during
/// the forward pass a row is optimized against the rows already seen, and
/// during the deferred correction against all of them.
#[inline(always)]
#[allow(clippy::too_many_arguments)]
fn incre_step_2d<C: Lp2dIncCollector>(
    a_b: &[(f64, f64, f64)],
    row: usize,
    upto: usize,
    x: &mut f64,
    y: &mut f64,
    epsilon: f64,
    tol_1d: &LpToleranceOptions,
    collector: &mut C,
    bound: f64,
) -> bool {
    let (a, b, c) = a_b[row];
    let i = row;
    if a.abs() < EPS_ZERO && b.abs() < EPS_ZERO {
        // 0 <= c
        return c >= -epsilon;
    }
    if a.abs() > b.abs() {
        // a*x + b*y == c
        // x == c/a - (b/a)*y == p*y - q
        let a_inv = 1.0 / a;
        let p = -b * a_inv;
        let q = -c * a_inv;
        // a_*x + b_*y <= c_
        // a_ * (p*y - q) + b_*y <= c_
        // (a_*p + b_)*y <= c_ + a_*q
        let (ymax, _) = lp_1d_core::<_, true, true>(
            a_b.iter().take(upto).map(|&(a_, b_, c_)| {
                let term = a_ * p;
                (term + b_, c_ + a_ * q, term.abs() + b_.abs())
            }),
            tol_1d,
            &mut *collector,
        );
        if ymax.is_nan() {
            return false;
        }
        *y = ymax.min(*y);
        *x = p * *y - q;
        collector.collect_2d_id1(i);
        return true;
    }
    // a*x + b*y == c
    // y == c/b - (a/b)*x == p*x - q
    let b_inv = 1.0 / b;
    let p = -a * b_inv;
    let q = -c * b_inv;
    // a_*x + b_*y <= c_
    // a_ * x + b_ * (p*x - q) <= c_
    // (a_ + b_*p)*x <= c_ + b_*q
    let (mut xmax, mut xmin) = lp_1d_core::<_, false, true>(
        a_b.iter().take(upto).map(|(a_, b_, c_)| {
            let term = b_ * p;
            (a_ + term, c_ + b_ * q, a_.abs() + term.abs())
        }),
        tol_1d,
        &mut *collector,
    );
    if xmax < xmin {
        // `xmax`/`xmin` bound the same variable `x`, so the inconsistency
        // `xmin - xmax` lives in `x` units and the tolerance must be applied there.
        // This branch has `|p| <= 1` (it is the `|a| <= |b|` case), so the previous
        // `p.abs() * (xmin - xmax)` test measured the inconsistency in `y` units and
        // was looser by the unbounded factor `1 / |p|`.
        if xmin - xmax > epsilon.max(EPS_ROUNDING * (xmin.abs() + xmax.abs())) {
            collector.clear();
            collector.collect_2d_id0(i);
            return false;
        }
        xmax = 0.5 * (xmax + xmin);
        xmin = xmax;
    }
    *x = if p > 0.0 {
        collector.collect_2d_id1(i);
        if xmax.is_finite() {
            xmax
        } else {
            xmin.max(bound)
        }
    } else if p < 0.0 {
        collector.collect_2d_id0(i);
        if xmin.is_finite() {
            xmin
        } else {
            xmax.min(-bound)
        }
    } else if xmin > 0.0 {
        collector.collect_2d_id0(i);
        xmin
    } else if xmax < 0.0 {
        collector.collect_2d_id1(i);
        xmax
    } else {
        collector.clear();
        collector.collect_2d_id0(i);
        0.0
    };
    *y = p * *x - q;
    true
}

/// Project only the non-objective coordinate while keeping the current objective
/// value.  The returned point satisfies `rows[..upto]` within `epsilon`.
///
/// In exact arithmetic, if `(x, y)` is optimal for the old prefix, any feasible
/// `(x_new, y)` is also optimal after adding rows: the feasible set only shrank,
/// while its old upper bound `y` is still attained.  Here the same argument is
/// used with the solver's advertised residual tolerance, so the projection adds
/// no objective loss and restores the corresponding numerical invariant.
#[cold]
#[inline(never)]
fn project_x_at_fixed_y(
    a_b: &[(f64, f64, f64)],
    upto: usize,
    x: f64,
    y: f64,
    epsilon: f64,
) -> Option<f64> {
    // The common degeneracy needs no coordinate change at all.  Check this
    // before constructing the interval so a rounded `c + epsilon` cannot lose
    // a tolerance that the solver's own residual test can still see.
    if x.is_finite()
        && a_b
            .iter()
            .take(upto)
            .all(|&(a, b, c)| a * x + b * y - c <= epsilon)
    {
        return Some(x);
    }

    let mut xmin = f64::NEG_INFINITY;
    let mut xmax = f64::INFINITY;

    for &(a, b, c) in a_b.iter().take(upto) {
        if !a.is_finite() || !b.is_finite() || !c.is_finite() {
            return None;
        }
        // At fixed y, a*x + b*y <= c + epsilon is a one-dimensional
        // interval. `mul_add` is deliberately confined to this rare fallback.
        let rhs = (-b).mul_add(y, c + epsilon);
        if !rhs.is_finite() {
            return None;
        }
        if a > 0.0 {
            xmax = xmax.min(rhs / a);
        } else if a < 0.0 {
            xmin = xmin.max(rhs / a);
        } else if rhs < 0.0 {
            return None;
        }
    }
    if xmin > xmax {
        return None;
    }

    let is_prefix_feasible = |x_test: f64| {
        x_test.is_finite()
            && a_b
                .iter()
                .take(upto)
                .all(|&(a, b, c)| a * x_test + b * y - c <= epsilon)
    };

    // Usually the old x is already in the interval (the motivating case is a
    // one-ulp disagreement between equivalent evaluation orders).  Trying the
    // projection and then interior/end-point alternatives also covers a bound
    // rounded by one operation in the opposite direction from the final check.
    let projected = x.max(xmin).min(xmax);
    if is_prefix_feasible(projected) {
        return Some(projected);
    }
    if xmin.is_finite() && xmax.is_finite() {
        let midpoint = 0.5 * xmin + 0.5 * xmax;
        if is_prefix_feasible(midpoint) {
            return Some(midpoint);
        }
    }
    if is_prefix_feasible(xmin) {
        return Some(xmin);
    }
    if is_prefix_feasible(xmax) {
        return Some(xmax);
    }
    None
}

/// Check that a trial collector describes active rows whose normal cone
/// supports the `+y` objective.  A lone row is also valid when its positive
/// y-normal combines with the solver's implicit x box; `y == LP_BOUND` is the
/// corresponding implicit objective-bound case.
#[cold]
#[inline(never)]
#[allow(clippy::too_many_arguments)]
fn active_basis_supports_max_y(
    a_b: &[(f64, f64, f64)],
    upto: usize,
    selected_row: usize,
    x: f64,
    y: f64,
    epsilon: f64,
    ids: (usize, usize),
    bound: f64,
) -> bool {
    let valid0 = ids.0 < upto;
    let valid1 = ids.1 < upto;
    if (!valid0 && !valid1)
        || (selected_row != ids.0 && selected_row != ids.1)
        || (valid0 && (a_b[ids.0].0 * x + a_b[ids.0].1 * y - a_b[ids.0].2).abs() > epsilon)
        || (valid1 && (a_b[ids.1].0 * x + a_b[ids.1].1 * y - a_b[ids.1].2).abs() > epsilon)
    {
        return false;
    }

    if !valid0 || !valid1 || ids.0 == ids.1 {
        let id = if valid0 { ids.0 } else { ids.1 };
        let (a, b, _) = a_b[id];
        return y == bound
            || (b > 0.0 && (a == 0.0 || (a > 0.0 && x == -bound) || (a < 0.0 && x == bound)));
    }

    let (n0, n1) = (a_b[ids.0], a_b[ids.1]);
    let (p, q) = (n0.0 * n1.1, n0.1 * n1.0);
    let det = p - q;
    if !det.is_finite() || det.abs() <= CCW_ERRBOUND_A * (p.abs() + q.abs()) {
        return false;
    }
    // [n0 n1] * lambda = (0, 1).  Nonnegative multipliers are the
    // two-dimensional KKT/normal-cone certificate for maximizing y.
    let (lambda0, lambda1) = (-n1.0 / det, n0.0 / det);
    lambda0.is_finite() && lambda1.is_finite() && lambda0 >= 0.0 && lambda1 >= 0.0
}

/// Deterministically search a prefix by trying every possible active boundary.
/// This is quadratic in the slow path, but each returned candidate has passed
/// the full residual check and an active-basis certificate.  Numerical failure
/// of every candidate is still reported to the caller rather than described as
/// a proof that the mathematical LP is infeasible.
#[cold]
#[inline(never)]
#[allow(clippy::too_many_arguments)]
fn rebuild_2d_prefix(
    a_b: &[(f64, f64, f64)],
    upto: usize,
    x_upper: f64,
    y_upper: f64,
    y_floor: f64,
    epsilon: f64,
    tol_1d: &LpToleranceOptions,
    bound: f64,
) -> Option<(f64, f64, (usize, usize))> {
    let mut best: Option<(f64, f64, (usize, usize))> = None;
    for row in 0..upto {
        let (a_row, b_row, c_row) = a_b[row];
        if a_row.abs() < EPS_ZERO && b_row.abs() < EPS_ZERO {
            // This row has no boundary that could support an optimum.
            continue;
        }
        let (mut x_trial, mut y_trial) = (x_upper, y_upper);
        let mut trial_collector = IndexLpIncCollector::new();
        if !incre_step_2d(
            a_b,
            row,
            upto,
            &mut x_trial,
            &mut y_trial,
            epsilon,
            tol_1d,
            &mut trial_collector,
            bound,
        ) || !x_trial.is_finite()
            || !y_trial.is_finite()
            || y_trial < y_floor
            || y_trial > y_upper
            || (a_row * x_trial + b_row * y_trial - c_row).abs() > epsilon
            || a_b
                .iter()
                .take(upto)
                .any(|&(a, b, c)| a * x_trial + b * y_trial - c > epsilon)
        {
            continue;
        }
        // Downstream indexed callers require the first id to be the real row
        // in a one-row basis; preserve both positions for a two-row basis.
        let ids = if trial_collector.id.0 >= upto && trial_collector.id.1 < upto {
            (trial_collector.id.1, trial_collector.id.0)
        } else {
            (trial_collector.id.0, trial_collector.id.1)
        };
        if !active_basis_supports_max_y(a_b, upto, row, x_trial, y_trial, epsilon, ids, bound) {
            continue;
        }
        // `y_upper` came from the optimum of the previous prefix.  Adding a
        // row cannot improve it, so attaining the same value proves that no
        // later candidate can win and avoids the quadratic tail in the common
        // redundant-row recovery.
        if y_trial == y_upper {
            return Some((x_trial, y_trial, ids));
        }
        if best.is_none_or(|(_, y_best, _)| y_trial > y_best) {
            best = Some((x_trial, y_trial, ids));
        }
    }
    best
}

/// Recover a tolerance-verified optimum/basis candidate for `a_b[..upto]` after
/// the usual pivot failed.  An arbitrary feasible projection is never returned:
/// either the old objective level remains attainable, or the active-boundary
/// scan supplies a verified basis that can be committed to the real collector.
#[cold]
#[inline(never)]
#[allow(clippy::too_many_arguments)]
fn recover_2d_prefix<C: Lp2dIncCollector>(
    a_b: &[(f64, f64, f64)],
    upto: usize,
    x: f64,
    y: f64,
    epsilon: f64,
    tol_1d: &LpToleranceOptions,
    collector: &mut C,
    bound: f64,
) -> Option<(f64, f64)> {
    let fixed_y_repair = project_x_at_fixed_y(a_b, upto, x, y, epsilon);
    let x_seed = fixed_y_repair.unwrap_or(x);
    let y_floor = if fixed_y_repair.is_some() {
        // The old y is still both an upper bound and an attainable lower
        // bound. Permit only the recovery tolerance of reconstruction drift.
        y - epsilon
    } else {
        f64::NEG_INFINITY
    };

    if let Some((x_rebuilt, y_rebuilt, ids)) =
        rebuild_2d_prefix(a_b, upto, x_seed, y, y_floor, epsilon, tol_1d, bound)
    {
        // All numeric work above used a private collector. Commit the verified
        // basis only after a winner exists, so failed recovery is transactional
        // with respect to the caller's active-set state.
        collector.clear();
        if ids.0 < upto {
            collector.collect_2d_id0(ids.0);
        }
        if ids.1 < upto {
            collector.collect_2d_id1(ids.1);
        }
        return Some((x_rebuilt, y_rebuilt));
    }

    // If no alternate boundary survived numerically but the unchanged point
    // itself passed the complete prefix check, retaining it is still a valid
    // tolerance-level certificate and leaves the old collector untouched.
    if fixed_y_repair == Some(x) {
        return Some((x, y));
    }
    None
}

/// Linear programming in 2D plane based on incremental method (only maximize Y).
/// max Y, s.t. Ab[i].0*X+Ab[i].1*Y<=Ab[i].2
/// return (X,Y)
///
/// `bound` is the half-width of the box the problem is posed over, in the
/// caller's coordinates. An answer on the box is a clipped one when the
/// problem is bounded, so the box grows and the solve repeats.
#[inline(always)]
fn lp_2d_incre_max_y_core<C: Lp2dIncCollector, W: WarmStartLp2d, const NORMALIZE: bool>(
    a_b: &mut [(f64, f64, f64)],
    warm_start: &W,
    tol: &LpToleranceOptions,
    mut collector: C,
    mut bound: f64,
) -> (f64, f64) {
    let mut saturated = false;
    for step in 0..=LP_BOX_GROWTH_STEPS {
        collector.clear();
        let (x, y) = lp_2d_incre_max_y_once::<_, _, NORMALIZE>(
            a_b,
            warm_start,
            tol,
            &mut collector,
            bound,
            &mut saturated,
        );
        if !saturated || step == LP_BOX_GROWTH_STEPS {
            return (x, y);
        }
        bound *= LP_BOX_GROWTH;
    }
    unreachable!()
}

/// One solve of [`lp_2d_incre_max_y_core`] over a fixed box; `saturated`
/// reports whether the answer sits on that box.
#[inline(always)]
fn lp_2d_incre_max_y_once<C: Lp2dIncCollector, W: WarmStartLp2d, const NORMALIZE: bool>(
    a_b: &mut [(f64, f64, f64)],
    warm_start: &W,
    tol: &LpToleranceOptions,
    mut collector: C,
    mut bound: f64,
    saturated: &mut bool,
) -> (f64, f64) {
    *saturated = false;
    let epsilon = tol.feas_tol;
    if a_b.len() <= 1 {
        if a_b.len() == 1 {
            let a_b_0 = a_b.first().unwrap();
            if a_b_0.0.abs() < EPS_ZERO && a_b_0.1 > EPS_ZERO {
                // Unbounded
                return (f64::NAN, a_b_0.2 / a_b_0.1);
            }
        }
        // Infeasible
        return (f64::NAN, f64::NAN);
    }

    if NORMALIZE {
        normalize_lp2d(a_b);
    }

    let (mut x, mut y) = warm_start.get_initial_point();
    if x.is_finite() && y.is_finite() && (x.abs() > bound || y.abs() > bound) {
        // A warm start is the optimum of its prefix over the box that solve
        // was posed over; this solve must not pose a smaller one.
        bound = LP_BOX_GROWTH * x.abs().max(y.abs());
    }
    if y == LP_BOUND && bound > LP_BOUND {
        // The sentinel "start beyond the set" has to sit on the box edge that
        // is actually in use, or it is below the optimum and the solver, which
        // only ever moves down, returns the sentinel itself.
        y = bound;
    }
    let tol_1d = LpToleranceOptions::with_feas_tol(epsilon * tol.reduce_dim_scale);
    // Reported rather than returned from inside the loop: the loop holds `a_b`
    // immutably, and restoring the rows needs it back.
    let mut feasible = true;
    for (i, &(a, b, c)) in warm_start.iter_skip(a_b.iter().enumerate()) {
        // a*x + b*y <= c, up to the rounding level of the terms.
        let ax = a * x;
        let by = b * y;
        let residual = ax + by - c;
        if residual > EPS_ROUNDING * (ax.abs() + by.abs() + c.abs()) {
            // Keep the strict incremental pivot for every genuine violation:
            // even a small normal residual can move the objective appreciably
            // when this row is nearly parallel to the objective.  The tolerance
            // is used only to adjudicate a failed reduction.  In that case the
            // current point already satisfies the new row to the solver's
            // advertised accuracy, whereas declaring the whole prefix
            // infeasible would amplify roundoff between almost coincident rows.
            //
            // Trial with a silent collector so a rejected step is transactional;
            // `lp_1d_core` may update active indices before discovering that its
            // interval is empty.
            if residual <= epsilon {
                let (mut x_trial, mut y_trial) = (x, y);
                if !incre_step_2d(
                    a_b,
                    i,
                    i,
                    &mut x_trial,
                    &mut y_trial,
                    epsilon,
                    &tol_1d,
                    &mut SilentLpIncCollector,
                    bound,
                ) {
                    let upto = i + 1;
                    let repair_epsilon = epsilon * LP2D_REPAIR_TOL_SCALE;
                    let repair_tol_1d =
                        LpToleranceOptions::with_feas_tol(repair_epsilon * tol.reduce_dim_scale);
                    if let Some(recovered) = recover_2d_prefix(
                        a_b,
                        upto,
                        x,
                        y,
                        repair_epsilon,
                        &repair_tol_1d,
                        &mut collector,
                        bound,
                    ) {
                        (x, y) = recovered;
                        continue;
                    }

                    // A narrow feasible slice need not have enough floating-
                    // point room for the smaller repair tolerance.  Retrying
                    // with the public tolerance preserves the solver's existing
                    // contract without charging the normal path for recovery.
                    if repair_epsilon != epsilon
                        && let Some(recovered) = recover_2d_prefix(
                            a_b,
                            upto,
                            x,
                            y,
                            epsilon,
                            &tol_1d,
                            &mut collector,
                            bound,
                        )
                    {
                        (x, y) = recovered;
                        continue;
                    }

                    feasible = false;
                    break;
                }
            }
            if !incre_step_2d(
                a_b,
                i,
                i,
                &mut x,
                &mut y,
                epsilon,
                &tol_1d,
                &mut collector,
                bound,
            ) {
                feasible = false;
                break;
            }
        }
    }
    if !feasible {
        collector.clear();
        return (f64::NAN, f64::NAN); // Infeasible
    }

    // Deferred correction. The pass above is optimal for the rows in the order
    // it happened to see them, and a row it processed early can be left
    // violated by a later one when the arithmetic that combined them lost
    // digits. Rather than tightening a tolerance to prevent that, look at the
    // answer and ask which row it violates: that is a comparison between two
    // quantities of the same size, needing no tolerance to interpret, and it is
    // only available once there is an answer to look at.
    //
    // Reprocessing such a row is exact rather than a repair. Seidel's step
    // pins one row as an equality and optimizes along it subject to the rest,
    // which yields the optimum of the whole system whenever the optimum lies on
    // that row -- and it does, precisely because the row is violated at the
    // current point. Taking `a_b.len()` as the horizon rather than the row's
    // own index is what makes "the rest" mean every row.
    //
    // One round suffices in exact arithmetic; the cap is there because rounding
    // can send two rows back and forth.
    for _ in 0..MAX_DEFERRED_ROUNDS {
        let mut worst = 0.0;
        let mut worst_row = usize::MAX;
        for (i, &(a, b, c)) in a_b.iter().enumerate() {
            let ax = a * x;
            let by = b * y;
            let excess = ax + by - c - epsilon.max(EPS_ROUNDING * (ax.abs() + by.abs() + c.abs()));
            if excess > worst {
                worst = excess;
                worst_row = i;
            }
        }
        if worst_row == usize::MAX {
            break;
        }
        crate::verbosity_log!(
            crate::diag::Verbosity::Debug,
            "lp_2d deferred correction fired: row {worst_row} violated by {worst:.3e} beyond tolerance"
        );
        if !incre_step_2d(
            a_b,
            worst_row,
            a_b.len(),
            &mut x,
            &mut y,
            epsilon,
            &tol_1d,
            &mut collector,
            bound,
        ) {
            collector.clear();
            return (f64::NAN, f64::NAN); // Infeasible
        }
    }

    // y = if (y / BOUND - 1.0).abs() < epsilon {
    //     f64::INFINITY
    // } else {
    //     y
    // };
    *saturated = lp_2d_saturated(x, y, bound);
    (x, y)
}

/// Linear programming in 2D plane based on incremental method (only maximize Y).
/// max Y, s.t. Ab[i].0*X+Ab[i].1*Y<=Ab[i].2
/// return (X,Y)
pub(crate) fn lp_2d_incre_max_y<W: WarmStartLp2d, const NORMALIZE: bool>(
    a_b: &mut [(f64, f64, f64)],
    warm_start: &W,
    tol: &LpToleranceOptions,
) -> (f64, f64) {
    lp_2d_incre_max_y_core::<_, _, NORMALIZE>(a_b, warm_start, tol, SilentLpIncCollector, LP_BOUND)
}

/// Linear programming in 1D line.
/// a_b[i].0 * x <= a_b[i].1
/// Return (xmax, xmin) if feasible; otherwise (NaN, NaN).
#[inline(always)]
fn lp_1d_core<C: Lp1dIncCollector, const AUTONAN: bool, const CANCEL: bool>(
    a_b: impl Iterator<Item = (f64, f64, f64)>,
    tol: &LpToleranceOptions,
    mut collector: C,
) -> (f64, f64) {
    let epsilon = tol.feas_tol;
    // Find the maximum y such that a*x <= b
    let mut xmax = f64::INFINITY;
    let mut xmin = -f64::INFINITY;

    for (k, (a, b, cancel_scale)) in C::enumerate(a_b) {
        // println!("\t\t(LP-1D) a={}, b={}", a, b);
        let coefficient_is_zero = if CANCEL {
            a.abs() <= LP1D_CANCEL_EPS * cancel_scale
        } else {
            a.abs() < b.abs().clamp(EPS_NORMALIZE, 1.0) * EPS_ZERO
        };
        if coefficient_is_zero {
            // 0 <= b
            if b < -epsilon {
                return (f64::NAN, f64::NAN); // Infeasible
            }
        } else if a > 0.0 {
            // x <= x_
            if b < xmax * a {
                xmax = b / a;
                collector.collect_1d_id_max(k.get_key());
            }
        } else {
            // x >= x_
            if b < xmin * a {
                xmin = b / a;
                collector.collect_1d_id_min(k.get_key());
            }
        }

        // println!("\t\t(LP-1D) xmax={}, xmin={}, a={}, b={}", xmax, xmin, a, b);
    }
    // println!("\t\t(LP-1D) xmax={}, xmin={}", xmax, xmin);
    if AUTONAN {
        if xmin <= xmax + epsilon.max(EPS_ROUNDING * (xmin.abs() + xmax.abs())) {
            if xmin <= xmax {
                (xmax, xmin)
            } else {
                let x = 0.5 * (xmax + xmin);
                (x, x)
            }
        } else {
            // println!("\t(LP-1D) xmax={}, xmin={}", xmax, xmin);
            (f64::NAN, f64::NAN)
        }
    } else {
        (xmax, xmin)
    }
}

/// Linear programming in 1D line.
/// a_b[i].0 * x <= a_b[i].1
/// Return (xmax, xmin) if feasible; otherwise (NaN, NaN).
#[inline(always)]
pub(crate) fn lp_1d<const AUTONAN: bool>(
    a_b: impl Iterator<Item = (f64, f64)>,
    tol: &LpToleranceOptions,
) -> (f64, f64) {
    lp_1d_core::<_, AUTONAN, false>(a_b.map(|(a, b)| (a, b, 0.0)), tol, SilentLpIncCollector)
}

/// Warm-start interface for 2D incremental LP.
pub(crate) trait WarmStartLp2d {
    /// Initial feasible guess.
    fn get_initial_point(&self) -> (f64, f64);
    /// Iterate constraints while skipping known-prefix constraints if desired.
    fn iter_skip<I>(&self, a_b: I) -> impl Iterator<Item = I::Item>
    where
        I: Iterator;
}

/// 2D warm-start policy with explicit initial point and skip length.
pub(crate) struct Lp2dWarmStart {
    /// Initial point in transformed 2D LP coordinates.
    pub x0: (f64, f64),
    /// Number of leading constraints to skip.
    pub skip: usize,
}
impl WarmStartLp2d for Lp2dWarmStart {
    #[inline(always)]
    fn get_initial_point(&self) -> (f64, f64) {
        self.x0
    }
    #[inline(always)]
    fn iter_skip<I>(&self, a_b: I) -> impl Iterator<Item = I::Item>
    where
        I: Iterator,
    {
        a_b.skip(self.skip)
    }
}

/// Lightweight key adapter for LP collector implementations.
trait Lp1dKey: Copy {
    fn get_key(&self) -> usize;
}
impl Lp1dKey for usize {
    #[inline(always)]
    fn get_key(&self) -> usize {
        *self
    }
}
impl Lp1dKey for () {
    #[inline(always)]
    fn get_key(&self) -> usize {
        0
    }
}
/// Collector interface for 1D incremental LP diagnostics (active indices).
trait Lp1dIncCollector {
    type Key: Lp1dKey;
    fn collect_1d_id_max(&mut self, index: usize);
    fn collect_1d_id_min(&mut self, index: usize);
    fn clear(&mut self);
    fn enumerate<D, I>(a_b: I) -> impl Iterator<Item = (Self::Key, D)>
    where
        I: Iterator<Item = D>;
}
impl<T: Lp1dIncCollector> Lp1dIncCollector for &mut T {
    type Key = T::Key;
    #[inline(always)]
    fn collect_1d_id_max(&mut self, k: usize) {
        (**self).collect_1d_id_max(k);
    }
    #[inline(always)]
    fn collect_1d_id_min(&mut self, k: usize) {
        (**self).collect_1d_id_min(k);
    }
    #[inline(always)]
    fn clear(&mut self) {
        (**self).clear();
    }
    #[inline(always)]
    fn enumerate<D, I>(a_b: I) -> impl Iterator<Item = (Self::Key, D)>
    where
        I: Iterator<Item = D>,
    {
        T::enumerate(a_b)
    }
}
/// Collector interface extending 1D collector with 2D active-set events.
trait Lp2dIncCollector: Lp1dIncCollector {
    fn collect_2d_id1(&mut self, index: usize);
    fn collect_2d_id0(&mut self, index: usize);
}
impl<T: Lp2dIncCollector> Lp2dIncCollector for &mut T {
    #[inline(always)]
    fn collect_2d_id1(&mut self, index: usize) {
        (**self).collect_2d_id1(index);
    }
    #[inline(always)]
    fn collect_2d_id0(&mut self, index: usize) {
        (**self).collect_2d_id0(index);
    }
}
/// Silent collector that does nothing.
struct SilentLpIncCollector;
impl Lp1dIncCollector for SilentLpIncCollector {
    type Key = ();
    #[inline(always)]
    fn collect_1d_id_max(&mut self, _: usize) {}
    #[inline(always)]
    fn collect_1d_id_min(&mut self, _: usize) {}
    #[inline(always)]
    fn clear(&mut self) {}
    #[inline(always)]
    fn enumerate<D, I>(a_b: I) -> impl Iterator<Item = (Self::Key, D)>
    where
        I: Iterator<Item = D>,
    {
        a_b.map(|d| ((), d))
    }
}
impl Lp2dIncCollector for SilentLpIncCollector {
    #[inline(always)]
    fn collect_2d_id1(&mut self, _: usize) {}
    #[inline(always)]
    fn collect_2d_id0(&mut self, _: usize) {}
}

struct IndexLpIncCollector {
    // for 1d: (max_index, min_index)
    // for 2d: (index1, index2)
    pub id: (usize, usize),
}
impl IndexLpIncCollector {
    /// Construct collector with sentinel indices.
    pub fn new() -> Self {
        Self {
            id: (usize::MAX - 1, usize::MAX - 1),
        }
    }
}
impl Lp1dIncCollector for IndexLpIncCollector {
    type Key = usize;
    #[inline(always)]
    fn collect_1d_id_max(&mut self, index: usize) {
        self.id.0 = index;
    }
    #[inline(always)]
    fn collect_1d_id_min(&mut self, index: usize) {
        self.id.1 = index;
    }
    #[inline(always)]
    fn clear(&mut self) {
        self.id = (usize::MAX - 1, usize::MAX - 1);
    }
    #[inline(always)]
    fn enumerate<D, I>(a_b: I) -> impl Iterator<Item = (Self::Key, D)>
    where
        I: Iterator<Item = D>,
    {
        a_b.enumerate()
    }
}
impl Lp2dIncCollector for IndexLpIncCollector {
    #[inline(always)]
    fn collect_2d_id0(&mut self, index: usize) {
        self.id.0 = index;
    }
    #[inline(always)]
    fn collect_2d_id1(&mut self, index: usize) {
        self.id.1 = index;
    }
}

/// Solve 2*2 linear equations: a*x=b.
pub(crate) fn solve_2x2(a: ((f64, f64), (f64, f64)), b: (f64, f64)) -> Option<(f64, f64)> {
    let (p, q) = (a.0.0 * a.1.1, a.0.1 * a.1.0);
    let det = p - q;
    // The sign of `det` is only meaningful while it stands above the rounding
    // of the two products that cancel into it; see [`CCW_ERRBOUND_A`]. The
    // non-finite test is what makes this NaN-safe, since every comparison
    // against NaN is false.
    if !det.is_finite() || det.abs() <= CCW_ERRBOUND_A * (p.abs() + q.abs()) {
        return None; // Singular matrix, or non-finite coefficients
    }
    let det_inv = 1.0 / det;
    let x1 = cross_product_2d((b.0, a.0.1), (b.1, a.1.1)) * det_inv;
    let x2 = cross_product_2d((a.0.0, b.0), (a.1.0, b.1)) * det_inv;
    Some((x1, x2))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_project_x_at_fixed_y_keeps_optimal_value() {
        let rows = [
            (0.0, 1.0, 1.0),    // y <= 1
            (-1.0, 0.0, -0.25), // x >= 0.25
            (1.0, 0.0, 1.0),    // x <= 1
        ];
        let x = project_x_at_fixed_y(&rows, rows.len(), 0.0, 1.0, 0.0).unwrap();
        assert_eq!(x, 0.25);
        assert!(rows.iter().all(|&(a, b, c)| a * x + b * 1.0 <= c));
    }

    #[test]
    fn test_single_active_basis_requires_matching_box_support() {
        let missing = IndexLpIncCollector::new().id.1;
        assert!(!active_basis_supports_max_y(
            &[(1.0, 1.0, 0.0)],
            1,
            0,
            0.0,
            0.0,
            0.0,
            (0, missing),
            LP_BOUND,
        ));
        assert!(active_basis_supports_max_y(
            &[(1.0, 1.0, -LP_BOUND)],
            1,
            0,
            -LP_BOUND,
            0.0,
            0.0,
            (0, missing),
            LP_BOUND,
        ));
    }

    #[test]
    fn test_rebuild_2d_prefix_finds_verified_optimum() {
        let rows = [
            (0.0, 1.0, 1.0),  // y <= 1
            (-1.0, 0.0, 0.0), // x >= 0
            (1.0, 0.0, 1.0),  // x <= 1
            (1.0, 1.0, 0.75), // x + y <= 0.75
        ];
        let tol = LpToleranceOptions::with_feas_tol(1.0e-9);
        let (x, y, _) = rebuild_2d_prefix(
            &rows,
            rows.len(),
            0.0,
            1.0,
            f64::NEG_INFINITY,
            tol.feas_tol,
            &tol,
            LP_BOUND,
        )
        .unwrap();
        assert!(x.abs() <= tol.feas_tol);
        assert!((y - 0.75).abs() <= tol.feas_tol);
        assert!(
            rows.iter()
                .all(|&(a, b, c)| a * x + b * y - c <= tol.feas_tol)
        );
    }

    /// Reproduces a TOPP2-RA backward step that reports an empty reachable set on a
    /// problem that is feasible with a wide margin.  The rows are those
    /// `reach_set2`'s backward pass builds at one station of a real 3-axis toolpath:
    /// two box rows on `a[k+1]`, then the axial-acceleration rows of station `k` and
    /// of station `k + 1` as `fill_acc_topp2::<true>` emits them.  The variables are
    /// `(x, y) = (a[k+1], a[k])` and the objective is `max y`.
    ///
    /// That stretch of path is straight, so `q''` sits at the `1e-12` level and `q'`
    /// is nearly identical at both stations.  The station-`k` and station-`k + 1`
    /// rows of one axis therefore collapse onto the same half-plane: rows 2/3 and
    /// 8/9 are bit-identical, and rows 6/7 and 12/13 differ in the eleventh
    /// significant digit.
    ///
    /// In exact arithmetic the optimum is `(x, y) = (277.777777777777828,
    /// 282.287152777735628)`, lying on rows 0 and 6, with row 12 slack by `1.4e-12`.
    /// The floating-point vertex misses row 12 by about `1e-14`, while one ulp at
    /// `282` is `5.7e-14`, so the two rows are not distinguishable at all; the
    /// old zero-tolerance test at the top of the incremental loop pinned the
    /// iterate to row 12 anyway, and the problem reduced onto that row appeared
    /// empty after the near-parallel cancellation was amplified.
    #[test]
    #[allow(clippy::excessive_precision)]
    fn test_lp_2d_incre_max_y_twin_acceleration_rows() {
        // a * a[k+1] + b * a[k] <= c
        let mut a_b: Vec<(f64, f64, f64)> = [
            // Box rows on a[k+1], skipped by the warm start.
            (1.0, 0.0, 2.77777777777777828e2),
            (-1.0, 0.0, 0.0),
            // Axial acceleration at station k: X, Y, Z, each upper then lower.
            (-6.23700623703079060e1, 6.23700623703079060e1, 3.0e2),
            (6.23700623703079060e1, -6.23700623703079060e1, 3.0e2),
            (0.0, 0.0, 3.0e2),
            (0.0, 0.0, 3.0e2),
            (-6.65280665283281110e1, 6.65280665283338806e1, 3.0e2),
            (6.65280665283281110e1, -6.65280665283338806e1, 3.0e2),
            // Axial acceleration at station k + 1, same order.
            (-6.23700623703079060e1, 6.23700623703079060e1, 3.0e2),
            (6.23700623703079060e1, -6.23700623703079060e1, 3.0e2),
            (0.0, 0.0, 3.0e2),
            (0.0, 0.0, 3.0e2),
            (-6.65280665283189023e1, 6.65280665283248140e1, 3.0e2),
            (6.65280665283189023e1, -6.65280665283248140e1, 3.0e2),
        ]
        .into();

        // `backward_bound_a_next` normalizes the rows, then solves with
        // `NORMALIZE = false`; `reach_set2` runs it with `lp_feas_tol = 1e-8` and
        // starts from the midpoint of the successor interval.
        let tol = LpToleranceOptions::with_feas_tol(1.0e-8);
        normalize_lp2d(&mut a_b);
        let warm_start = Lp2dWarmStart {
            x0: (0.5 * 2.77777777777777828e2, LP_BOUND),
            skip: 2,
        };

        // The optimum the rows actually admit, confirmed independently in exact
        // rational arithmetic.
        let y_true = 2.82287152777735628e2;

        let (x_incre, y_incre) = lp_2d_incre_max_y::<_, false>(&mut a_b, &warm_start, &tol);
        assert!(
            x_incre.is_finite() && y_incre.is_finite(),
            "the incremental solver reported a feasible problem as infeasible"
        );
        assert!(
            (x_incre - 2.77777777777777828e2).abs() < 1.0e-9 && (y_incre - y_true).abs() < 1.0e-9,
            "incremental ({x_incre}, {y_incre}) should match the true optimum y={y_true}"
        );
        assert!(
            a_b.iter()
                .all(|&(a, b, c)| a * x_incre + b * y_incre - c <= tol.feas_tol),
            "the returned point must satisfy every normalized row within feas_tol"
        );

        // The slow recovery must also reconstruct a real supporting basis,
        // which is what the indexed API reports to its callers.
        let repair_epsilon = tol.feas_tol * LP2D_REPAIR_TOL_SCALE;
        let repair_tol_1d =
            LpToleranceOptions::with_feas_tol(repair_epsilon * tol.reduce_dim_scale);
        let mut active = IndexLpIncCollector::new();
        let (x_recovered, y_recovered) = recover_2d_prefix(
            &a_b,
            a_b.len(),
            x_incre,
            y_incre,
            repair_epsilon,
            &repair_tol_1d,
            &mut active,
            LP_BOUND,
        )
        .expect("the alternate active-boundary scan should recover this prefix");
        assert!((x_recovered - x_incre).abs() <= repair_epsilon);
        assert!((y_recovered - y_incre).abs() <= repair_epsilon);
        let (id0, id1) = (active.id.0, active.id.1);
        assert!(id0 < a_b.len() && id1 < a_b.len());
        let (n0, n1) = (a_b[id0], a_b[id1]);
        assert!((n0.0 * x_recovered + n0.1 * y_recovered - n0.2).abs() <= repair_epsilon);
        assert!((n1.0 * x_recovered + n1.1 * y_recovered - n1.2).abs() <= repair_epsilon);
        let det = n0.0 * n1.1 - n0.1 * n1.0;
        let (lambda0, lambda1) = (-n1.0 / det, n0.0 / det);
        assert!(
            det.abs() > CCW_ERRBOUND_A && lambda0 >= -repair_epsilon && lambda1 >= -repair_epsilon,
            "the recovered active normals must support the +y objective"
        );
    }

    /// The box the kernel poses its problem over is a starting point, not a
    /// limit: an answer on its edge is a clipped one, so the box grows and the
    /// solve repeats.
    ///
    /// The rows below admit `max y = 1e8` at `x = 0`, two orders of magnitude
    /// above [`LP_BOUND`]. The first solve can only reach the edge of the
    /// starting box and reports saturation; without the growth loop the answer
    /// would stay at `LP_BOUND`.
    #[test]
    fn test_lp_2d_incre_max_y_grows_the_box_past_its_start() {
        // x + y <= 1e8, -x + y <= 1e8, y >= 0.
        let mut a_b = vec![(1.0, 1.0, 1.0e8), (-1.0, 1.0, 1.0e8), (0.0, -1.0, 0.0)];
        normalize_lp2d(&mut a_b);
        let tol = LpToleranceOptions::with_feas_tol(1.0e-8);
        let warm_start = Lp2dWarmStart {
            x0: (0.0, LP_BOUND),
            skip: 0,
        };

        let (x, y) = lp_2d_incre_max_y::<_, false>(&mut a_b, &warm_start, &tol);
        assert!(
            y > LP_BOUND,
            "the box never grew: y={y} is still the edge of the starting box"
        );
        assert!(
            x.abs() < 1.0 && (y - 1.0e8).abs() < 1.0,
            "expected the apex (0, 1e8), got ({x}, {y})"
        );
        // The rows carry unit normals but sit at a right-hand side of `1e8`, so
        // a residual is only meaningful against that scale, not against the
        // feasibility tolerance the solver was given.
        assert!(
            a_b.iter().all(|&(a, b, c)| a * x + b * y - c <= 1.0e-3),
            "the returned point must satisfy every row at the scale of its rows"
        );
    }
}
