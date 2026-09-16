//! Reachability-analysis solver for second-order time-optimal path parameterization.
//!
//! # Method identity
//! This module implements **Reachability Analysis (RA)** for
//! **Time-Optimal Path Parameterization (TOPP2)** and shares the same state-space
//! conventions used by **Convex-Objective Path Parameterization (COPP2)** components.
//!
//! # Discrete variables (local notation)
//! On a path grid `s[0..=n]`:
//! - `a[k]` denotes $\dot{s}_k^2$;
//! - backward intervals are `[a_min[k], a_max[k]]` from [`reach_set2_backward`](crate::solver::reach_set2::reach_set2_backward);
//! - forward pass selects one feasible state per station, yielding the final profile `a`.
//!
//! # High-level pipeline
//! 1. Build backward reachable intervals by calling [`reach_set2_backward`](crate::solver::reach_set2::reach_set2_backward).
//! 2. Run forward clipping against local constraints and backward intervals.
//! 3. Select maximal feasible `a[k]` at each stage to recover the time-optimal profile.

use super::reach_set2::{ReachSet2Options, reach_set2_backward};
use crate::copp::copp2::formulation::Topp2Problem;
use crate::copp::{ApproxOrdering, approx_order};
use crate::diag::{
    CoppError, DebugVerboser, SilentVerboser, SummaryVerboser, TraceVerboser, Verboser, Verbosity,
    format_duration_human,
};
use crate::math::numerical::{LpToleranceOptions, lp_1d};
use core::f64;
use itertools::izip;

/// Solve TOPP2 with RA and return the profile $a(s)=\dot{s}^2$.
///
/// # Returns
/// Returns `a` such that:
/// - `a[0] = a_start`, `a[n] = a_final`, where `n = idx_s_final - idx_s_start`;
/// - `a` is time-optimal under configured first-/second-order constraints.
///
/// The returned profile can be mapped to `t(s)` by
/// [`s_to_t_topp2`](crate::solver::topp2_ra::s_to_t_topp2), then to sampled
/// `s(t)` by [`t_to_s_topp2`](crate::solver::topp2_ra::t_to_s_topp2).
///
/// # Errors
/// Returns [`CoppError`](crate::diag::CoppError) when backward reachable-set construction fails or when
/// forward pass cannot maintain feasibility under constraints.
///
/// # Contract
/// - station interval and boundary states must be valid for the given constraints;
/// - `options` must contain valid tolerance settings.
pub fn topp2_ra(problem: &Topp2Problem, options: &ReachSet2Options) -> Result<Vec<f64>, CoppError> {
    match options.verbosity {
        Verbosity::Silent => topp2_ra_core(problem, (options, SilentVerboser)),
        Verbosity::Summary => topp2_ra_core(problem, (options, SummaryVerboser::new())),
        Verbosity::Debug => topp2_ra_core(problem, (options, DebugVerboser::new())),
        Verbosity::Trace => topp2_ra_core(problem, (options, TraceVerboser::new())),
    }
}

/// Recover the largest state in the current backward interval whose edge from
/// `a_prev` satisfies every original TOPP2 row within the configured
/// unit-normal tolerance and an independent mixed physical-residual cap.
///
/// The ordinary forward reduction solves `row.0 * a_curr <= row.2 -
/// row.1 * a_prev`.  When `row.0` is the tiny remainder of a cancellation, a
/// last-bit residual in `a_prev` can become a macroscopic interval gap after
/// division.  This cold path avoids that ill-conditioned division as its first
/// decision: it certifies the backward upper endpoint directly.  If necessary,
/// it then intersects the tolerance-expanded half-spaces at fixed `a_prev` and
/// retries their largest point.
///
/// Returning a point is a certificate, not a best-effort fallback: the new
/// point is strictly inside its backward/path bounds, the already-committed
/// predecessor is inside its bounds under the configured state comparison
/// tolerance, and every finite row has been checked again in normalized 2D
/// coordinates.  A second mixed absolute/relative check bounds the
/// unnormalized physical residual even when the row normal is large.  Failure
/// leaves the caller's original infeasibility path unchanged.
#[cold]
#[inline(never)]
#[allow(clippy::too_many_arguments)]
fn recover_forward_intersection(
    rows: &[(f64, f64, f64)],
    a_prev: f64,
    prev_backward: (f64, f64),
    curr_backward: (f64, f64),
    prev_path_max: f64,
    curr_path_max: f64,
    options: &ReachSet2Options,
) -> Option<(f64, f64, f64)> {
    let feas_tol = options.lp_feas_tol;
    if !a_prev.is_finite()
        || !feas_tol.is_finite()
        || feas_tol < 0.0
        || !prev_backward.0.is_finite()
        || !prev_backward.1.is_finite()
        || curr_backward.0.is_nan()
        || curr_backward.1.is_nan()
        || prev_path_max.is_nan()
        || curr_path_max.is_nan()
    {
        return None;
    }

    let inside = |value: f64, lower: f64, upper: f64| {
        value.is_finite()
            && !matches!(
                approx_order(value, lower, options.a_cmp_abs_tol, options.a_cmp_rel_tol,),
                ApproxOrdering::Less
            )
            && !matches!(
                approx_order(value, upper, options.a_cmp_abs_tol, options.a_cmp_rel_tol,),
                ApproxOrdering::Greater
            )
    };
    let prev_lower = prev_backward.0.max(0.0);
    let prev_upper = prev_backward.1.min(prev_path_max);
    if !prev_lower.is_finite() || !prev_upper.is_finite() || !inside(a_prev, prev_lower, prev_upper)
    {
        return None;
    }

    // Use strict bounds for the point that will actually be committed.  The
    // mixed comparison tolerance above only certifies the already-selected
    // predecessor against its stored interval.
    let lower = curr_backward.0.max(0.0);
    let upper = curr_backward.1.min(curr_path_max);
    if !lower.is_finite() || !upper.is_finite() || lower > upper {
        return None;
    }

    let certify = |a_curr: f64| {
        if !a_curr.is_finite() || a_curr < lower || a_curr > upper {
            return None;
        }
        let mut worst_normalized = 0.0_f64;
        let mut worst_raw = 0.0_f64;
        for &(coef_curr, coef_prev, rhs) in rows {
            if !coef_curr.is_finite() || !coef_prev.is_finite() || !rhs.is_finite() {
                return None;
            }
            let norm = coef_curr.hypot(coef_prev);
            if norm == 0.0 {
                // A directionless physical row is feasible only when 0 <= rhs.
                if rhs < 0.0 {
                    return None;
                }
                continue;
            }
            if !norm.is_finite() {
                return None;
            }
            let norm_inv = 1.0 / norm;
            let raw_tol = feas_tol * rhs.abs().max(1.0);
            if !raw_tol.is_finite() {
                return None;
            }
            let normalized_residual =
                (coef_curr * norm_inv) * a_curr + (coef_prev * norm_inv) * a_prev - rhs * norm_inv;
            let raw_residual = coef_curr * a_curr + coef_prev * a_prev - rhs;
            let raw_normalized_residual = raw_residual / norm;
            if !normalized_residual.is_finite()
                || !raw_residual.is_finite()
                || !raw_normalized_residual.is_finite()
                || normalized_residual > feas_tol
                || raw_normalized_residual > feas_tol
                || raw_residual > raw_tol
            {
                return None;
            }
            worst_normalized = worst_normalized
                .max(normalized_residual)
                .max(raw_normalized_residual);
            worst_raw = worst_raw.max(raw_residual);
        }
        Some((a_curr, worst_normalized, worst_raw))
    };

    // This is both the cheapest and the strongest recovery: attaining the
    // backward cap preserves the greedy stage optimum exactly.
    if let Some(certified) = certify(upper) {
        return Some(certified);
    }

    // Search only after the cap itself failed its certificate.  At fixed
    // `a_prev`, each normalized 2D half-space cuts out a 1D interval.  Their
    // intersection with the backward interval therefore gives the complete
    // tolerance-feasible candidate set; its upper endpoint is the maximizer.
    let mut projected_lower = lower;
    let mut projected_upper = upper;
    for &(coef_curr, coef_prev, rhs) in rows {
        let norm = coef_curr.hypot(coef_prev);
        if norm == 0.0 {
            if rhs < 0.0 {
                return None;
            }
            continue;
        }
        if !norm.is_finite() {
            return None;
        }
        let norm_inv = 1.0 / norm;
        let raw_tol = feas_tol * rhs.abs().max(1.0);
        if !raw_tol.is_finite() {
            return None;
        }
        let normalized_tol = feas_tol.min(raw_tol * norm_inv);
        let coef_curr = coef_curr * norm_inv;
        let coef_prev = coef_prev * norm_inv;
        let rhs = rhs * norm_inv + normalized_tol - coef_prev * a_prev;
        if !rhs.is_finite() {
            return None;
        }
        if coef_curr > 0.0 {
            projected_upper = projected_upper.min(rhs / coef_curr);
        } else if coef_curr < 0.0 {
            projected_lower = projected_lower.max(rhs / coef_curr);
        } else if rhs < 0.0 {
            return None;
        }
    }
    if !projected_lower.is_finite()
        || !projected_upper.is_finite()
        || projected_lower > projected_upper
    {
        return None;
    }

    // A division can put the endpoint outside the certificate by much more
    // than one x-ulp when its normalized coefficient is tiny: one residual ulp
    // is then many coordinate ulps.  Try the immediate neighbours first, then
    // establish an interior certified point and bisect back toward the upper
    // endpoint.  Every retained point has passed the complete row certificate.
    let mut candidate = projected_upper;
    for _ in 0..=4 {
        if candidate < projected_lower {
            break;
        }
        if let Some(certified) = certify(candidate) {
            return Some(certified);
        }
        candidate = candidate.next_down();
    }

    let mut certified_lower_x = 0.5 * projected_lower + 0.5 * projected_upper;
    let mut certified_lower = certify(certified_lower_x)?;
    let mut rejected_upper = projected_upper;
    for _ in 0..64 {
        let midpoint = 0.5 * certified_lower_x + 0.5 * rejected_upper;
        if midpoint == certified_lower_x || midpoint == rejected_upper {
            break;
        }
        if let Some(certified) = certify(midpoint) {
            certified_lower_x = midpoint;
            certified_lower = certified;
        } else {
            rejected_upper = midpoint;
        }
    }
    Some(certified_lower)
}

/// Core implementation of Reachability Analysis for TOPP2 with layered verbosity logging.
fn topp2_ra_core(
    problem: &Topp2Problem,
    options_verboser: (&ReachSet2Options, impl Verboser),
) -> Result<Vec<f64>, CoppError> {
    let (options, mut verboser) = options_verboser;
    if verboser.is_enabled(Verbosity::Summary) {
        verboser.record_start_time();
        crate::verbosity_log!(
            Verbosity::Summary,
            "\ntopp2_ra started: {} <= idx_s <= {}, a_start = {}, a_final = {}.",
            problem.idx_s_interval.0,
            problem.idx_s_interval.1,
            problem.a_boundary.0,
            problem.a_boundary.1,
        );
    }

    // Step 1. Compute the backward reachable set.
    let reach_set = reach_set2_backward(problem, options).map_err(|e| {
        if verboser.is_enabled(Verbosity::Debug) {
            crate::verbosity_log!(Verbosity::Debug, "{e:?}");
        } else if verboser.is_enabled(Verbosity::Summary) {
            crate::verbosity_log!(
                Verbosity::Summary,
                "topp2_ra: failed while computing backward reachable set."
            );
        }
        e
    })?;
    let a_max = &reach_set.a_max;
    let a_min = &reach_set.a_min;

    // Step 2. Forward pass to select the maximal feasible state at each grid point.
    if verboser.is_enabled(Verbosity::Debug) {
        crate::verbosity_log!(Verbosity::Debug, "Forward pass started.");
    }

    let (idx_s_start, idx_s_final) = problem.idx_s_interval;
    let n = idx_s_final - idx_s_start;
    let mut a = vec![0.0; n + 1];
    let mut a_prev = problem.a_boundary.0;
    *a.first_mut().unwrap() = a_prev;

    let mut a_b = Vec::<(f64, f64, f64)>::with_capacity(2 * problem.constraints.acc_rows());
    for (k, (a_curr, &a_max_curr_, &a_min_curr_)) in
        izip!(a.iter_mut(), a_max, a_min).enumerate().skip(1)
    {
        let idx_s = idx_s_start + k;
        if verboser.is_enabled(Verbosity::Trace) {
            crate::verbosity_log!(
                Verbosity::Trace,
                "\tForward pass at k = {k} (idx_s = {idx_s}): backward interval {a_min_curr_} <= a[k] <= {a_max_curr_}, a_prev = {a_prev}."
            );
        }

        a_b.clear();
        problem
            .constraints
            .fill_acc_topp2::<true>(&mut a_b, idx_s - 1);
        // a_b.0 * a[k] + a_b.1 * a[k-1] <= a_b.2
        // Preserve the raw interval.  Any exact inversion is sent through the
        // cold 2D certificate below instead of being folded to an unchecked
        // midpoint by LP-1D's AUTONAN tolerance path.
        let (mut a_max_curr, mut a_min_curr) = lp_1d::<false>(
            a_b.iter().map(|&coeffs| {
                // coeffs.0 * a_curr  + coeffs.1* a_prev <= coeffs.2
                // coeffs.0 * a_curr <= coeffs.2 - coeffs.1 * a_prev
                (coeffs.0, coeffs.2 - coeffs.1 * a_prev)
            }),
            &LpToleranceOptions::with_feas_tol(options.lp_feas_tol),
        );

        if verboser.is_enabled(Verbosity::Trace) {
            crate::verbosity_log!(
                Verbosity::Trace,
                "\t\tForward LP result before clipping: {a_min_curr} <= a[k] <= {a_max_curr}."
            );
        }

        // The clipping below destroys the raw one-step range, so keep a copy.
        let (a_max_fwd, a_min_fwd) = (a_max_curr, a_min_curr);
        // `f64::min`/`max` deliberately return the non-NaN operand.  Remember
        // an infeasible 1D result before clipping, otherwise the finite
        // backward bounds would silently turn `(NaN, NaN)` into a seemingly
        // valid interval.
        let forward_lp_failed = a_max_fwd.is_nan() || a_min_fwd.is_nan();
        a_max_curr = a_max_curr.min(a_max_curr_);
        a_min_curr = a_min_curr.max(a_min_curr_);
        if verboser.is_enabled(Verbosity::Trace) {
            crate::verbosity_log!(
                Verbosity::Trace,
                "\t\tAfter clipping with backward reachable set: {a_min_curr} <= a[k] <= {a_max_curr}."
            );
        }

        if forward_lp_failed || a_max_curr < a_min_curr {
            let recovered = if a_max_curr_.is_finite() && a_min_curr_.is_finite() {
                recover_forward_intersection(
                    &a_b,
                    a_prev,
                    (a_min[k - 1], a_max[k - 1]),
                    (a_min_curr_, a_max_curr_),
                    problem.constraints.amax_unchecked(idx_s - 1),
                    problem.constraints.amax_unchecked(idx_s),
                    options,
                )
            } else {
                None
            };
            if let Some((a_recovered, worst_normalized, worst_raw)) = recovered {
                if verboser.is_enabled(Verbosity::Debug) {
                    crate::verbosity_log!(
                        Verbosity::Debug,
                        "Recovered the forward step at idx_s = {idx_s}: a[k] = {a_recovered}, max normalized violation = {worst_normalized:.6e}, max raw violation = {worst_raw:.6e}."
                    );
                }
                a_max_curr = a_recovered;
                a_min_curr = a_recovered;
            } else if !forward_lp_failed
                && !matches!(
                    approx_order(
                        a_max_curr,
                        a_min_curr,
                        // Both endpoints are differences of terms at the scale
                        // of `a_prev`, so their rounding is inherited from it.
                        options
                            .a_cmp_abs_tol
                            .max(options.a_cmp_rel_tol * a_prev.abs()),
                        options.a_cmp_rel_tol,
                    ),
                    ApproxOrdering::Less
                )
            {
                // An inversion within the comparison tolerance is a degenerate
                // interval, not an empty one.
                let a_degenerate = 0.5 * (a_max_curr + a_min_curr);
                if verboser.is_enabled(Verbosity::Debug) {
                    crate::verbosity_log!(
                        Verbosity::Debug,
                        "The forward reachable interval at idx_s = {idx_s} is inverted by {:.3e} at scale {:.3e}; treated as the degenerate point a[k] = {a_degenerate}.",
                        a_min_curr - a_max_curr,
                        a_prev.abs()
                    );
                }
                a_max_curr = a_degenerate;
                a_min_curr = a_degenerate;
            } else {
                // `a_prev` is `a_max_curr` of the previous step, hence already
                // `<= a_max[k - 1]`.  So an empty intersection here can only mean
                // the backward set is not self-consistent: its own cap at `k - 1`
                // cannot reach its own cap at `k`.
                let err = CoppError::Infeasible(
                    "topp2_ra".into(),
                    format!(
                        "The reachable set is empty at index {} during the forward pass where a_max = {}, a_min = {}",
                        idx_s_start + k,
                        a_max_curr,
                        a_min_curr
                    ),
                );
                if verboser.is_enabled(Verbosity::Debug) {
                    crate::verbosity_log!(Verbosity::Debug, "{err:?}");
                } else if verboser.is_enabled(Verbosity::Summary) {
                    crate::verbosity_log!(
                        Verbosity::Summary,
                        "topp2_ra: the forward pass failed at index {idx_s} due to infeasibility."
                    );
                }
                return Err(err);
            }
        }

        if a_max_curr.is_infinite() {
            let err = CoppError::Unbounded(
                "topp2_ra".into(),
                format!(
                    "The reachable set is unbounded at index {} during the forward pass where a_max = {}",
                    idx_s_start + k,
                    a_max_curr
                ),
            );
            if verboser.is_enabled(Verbosity::Debug) {
                crate::verbosity_log!(Verbosity::Debug, "{err:?}");
            } else if verboser.is_enabled(Verbosity::Summary) {
                crate::verbosity_log!(
                    Verbosity::Summary,
                    "topp2_ra: the forward pass failed at index {idx_s} due to unboundedness."
                );
            }
            return Err(err);
        }

        if verboser.is_enabled(Verbosity::Debug)
            && matches!(
                approx_order(
                    a_max_curr,
                    a_min_curr,
                    options.a_cmp_abs_tol,
                    options.a_cmp_rel_tol,
                ),
                ApproxOrdering::Equal
            )
        {
            crate::verbosity_log!(
                Verbosity::Debug,
                "The one-step forward reachable set at k = {k} (idx_s = {idx_s}) is degenerate since a_max and a_min are approximately equal at {}.",
                0.5 * (a_max_curr + a_min_curr)
            );
        }

        a_prev = a_max_curr;
        *a_curr = a_prev;

        if verboser.is_enabled(Verbosity::Trace) {
            crate::verbosity_log!(
                Verbosity::Trace,
                "\t\tSelected maximal feasible state: a[k] = {a_prev}."
            );
        }
    }

    if verboser.is_enabled(Verbosity::Summary) {
        crate::verbosity_log!(
            Verbosity::Summary,
            "topp2_ra: total elapsed time = {}.\n",
            format_duration_human(verboser.elapsed())
        );
    }

    Ok(a)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::copp::InterpolationMode;
    use crate::copp::constraints::Constraints;
    use crate::copp::copp2::stable::basic::{
        Topp2ProblemBuilder, a_to_b_topp2, s_to_t_topp2, t_to_s_topp2,
    };
    use crate::copp::copp2::stable::reach_set2::ReachSet2OptionsBuilder;
    use crate::path::{
        Path, SplineConfig, add_symmetric_axial_limits_for_test, lissajous_path_for_test,
    };
    use crate::robot::robot_core::Robot;
    use nalgebra::DMatrix;
    use std::time::{Duration, Instant};

    #[test]
    fn test_topp2_ra() -> Result<(), CoppError> {
        run_test_topp2_ra_repeated(1, false)
    }

    #[test]
    fn test_recovery_numeric_contract_at_extremes() {
        let options = ReachSet2OptionsBuilder::new().build().unwrap();

        assert!(
            recover_forward_intersection(
                &[],
                0.0,
                (0.0, 0.0),
                (0.0, 1.0),
                f64::NEG_INFINITY,
                1.0,
                &options,
            )
            .is_none(),
            "a non-finite effective predecessor interval must be rejected"
        );

        assert!(
            recover_forward_intersection(
                &[(0.0, 1.0, 1.0)],
                2.0,
                (0.0, 2.0),
                (0.0, 2.0),
                2.0,
                2.0,
                &options,
            )
            .is_none(),
            "a violation independent of the current state must not be swallowed"
        );

        let folded_delta = 0.25 * options.a_cmp_abs_tol;
        assert!(
            recover_forward_intersection(
                &[],
                1.0,
                (1.0 + folded_delta, 1.0 - folded_delta),
                (0.0, 1.0),
                2.0,
                1.0,
                &options,
            )
            .is_some(),
            "a predecessor interval folded within a_cmp_tol remains admissible"
        );

        // Squaring MIN_POSITIVE underflows to zero.  A direct sqrt(a*a+b*b)
        // would therefore discard this nonzero row and incorrectly keep 2.0.
        let (a_curr, normalized, _) = recover_forward_intersection(
            &[(f64::MIN_POSITIVE, 0.0, 0.0)],
            0.0,
            (0.0, 0.0),
            (0.0, 2.0),
            2.0,
            2.0,
            &options,
        )
        .expect("stable row normalization must retain a finite subnormal-scale row");
        assert!(a_curr <= options.lp_feas_tol);
        assert!(normalized <= options.lp_feas_tol);

        // A unit-normal tolerance alone would allow roughly 1e4 of raw
        // violation on this row.  The physical mixed cap keeps it below 1e-8.
        let (a_curr, normalized, raw) = recover_forward_intersection(
            &[(1.0e12, 0.0, 1.0)],
            0.0,
            (0.0, 0.0),
            (0.0, 1.0),
            1.0,
            1.0,
            &options,
        )
        .expect("the physical residual cap still leaves a nonempty interval");
        assert!(a_curr <= (1.0 + options.lp_feas_tol) / 1.0e12);
        assert!(normalized <= options.lp_feas_tol);
        assert!(raw <= options.lp_feas_tol);
    }

    fn constraints_with_first_edge_rows(
        rows: &[(f64, f64, f64)],
        ds: f64,
        path_cap: f64,
    ) -> Constraints {
        let mut constraints = Constraints::with_capacity(1, 3);
        constraints.with_s(&[0.0, ds, 2.0 * ds][..]).unwrap();
        constraints
            .with_constraint_1order(&[path_cap, path_cap, path_cap][..], 0)
            .unwrap();

        // At the left station, fill_acc_topp2::<true> emits
        // (acc_b/(2ds), acc_a-acc_b/(2ds), rhs).  Invert that map so the
        // generated problem contains the requested edge rows exactly up to
        // the multiplication/division round trip under test.
        let acc_a = DMatrix::from_iterator(rows.len(), 1, rows.iter().map(|r| r.0 + r.1));
        let acc_b = DMatrix::from_iterator(rows.len(), 1, rows.iter().map(|r| 2.0 * ds * r.0));
        let acc_max = DMatrix::from_iterator(rows.len(), 1, rows.iter().map(|r| r.2));
        constraints
            .with_constraint_2order(
                &acc_a.as_view(),
                &acc_b.as_view(),
                &acc_max.as_view(),
                0,
                false,
            )
            .unwrap();
        constraints
    }

    fn first_edge_row_after_round_trip(
        (coef_curr, coef_prev, rhs): (f64, f64, f64),
        ds: f64,
    ) -> (f64, f64, f64) {
        let acc_a = coef_curr + coef_prev;
        let acc_b = 2.0 * ds * coef_curr;
        let coef_curr = acc_b / (2.0 * ds);
        (coef_curr, acc_a - coef_curr, rhs)
    }

    #[test]
    fn test_topp2_ra_recovery_distribution_stress() {
        // Each three-station problem is a complete TOPP2 run.  The grid is
        // deterministic so a failing tuple is exactly reproducible, while its
        // axes cover state magnitude, ds scaling, cancelled coefficient size,
        // condition number, interval-gap behavior, and all row permutations.
        let options = ReachSet2OptionsBuilder::new().build().unwrap();
        let epsilon = options.lp_feas_tol;
        let base_cases = [
            (48.0, 32.0, 4.0),
            (100.0, 64.0, 4.0),
            (277.77777777777777, 200.0, 6.0),
            (6144.0, 4096.0, 8.0),
        ];
        let ds_exponents = [-20, 0, 20];
        let row_scale_exponents = [0, 4, 8];
        let condition_exponents = [30, 35, 40];
        let gap_factors = [0.25, 0.75, 1.25, 32.0];
        let permutations = [
            [0, 1, 2],
            [0, 2, 1],
            [1, 0, 2],
            [1, 2, 0],
            [2, 0, 1],
            [2, 1, 0],
        ];
        let mut midpoint_cases = 0usize;
        let mut masked_nan_cases = 0usize;
        let mut total_cases = 0usize;

        for &(path_cap, a_prev, tiny_upper_offset) in &base_cases {
            let robust_upper = path_cap - a_prev;
            for &ds_exponent in &ds_exponents {
                let ds = 2.0_f64.powi(ds_exponent);
                for &row_scale_exponent in &row_scale_exponents {
                    let row_rhs = 300.0 * (a_prev / 64.0) * 2.0_f64.powi(row_scale_exponent);
                    for &condition_exponent in &condition_exponents {
                        let condition = 2.0_f64.powi(condition_exponent);
                        let intended_tiny_upper = robust_upper - tiny_upper_offset;
                        let tiny = row_rhs / (condition * a_prev + intended_tiny_upper);
                        let coef_prev = tiny * condition;
                        let tiny_row = (tiny, coef_prev, row_rhs);
                        let actual_tiny = first_edge_row_after_round_trip(tiny_row, ds);
                        let actual_tiny_upper =
                            (actual_tiny.2 - actual_tiny.1 * a_prev) / actual_tiny.0;

                        assert!(
                            actual_tiny_upper.is_finite()
                                && actual_tiny_upper > 0.0
                                && actual_tiny_upper < robust_upper,
                            "bad generated tiny upper: cap={path_cap:e}, prev={a_prev:e}, ds_exp={ds_exponent}, row_scale_exp={row_scale_exponent}, cond_exp={condition_exponent}, upper={actual_tiny_upper:e}"
                        );

                        for &gap_factor in &gap_factors {
                            let requested_lower = actual_tiny_upper + gap_factor * epsilon;
                            let canonical_rows = [
                                (1.0, 1.0, path_cap),
                                tiny_row,
                                (-1.0, 1.0, a_prev - requested_lower),
                            ];
                            for (permutation_id, permutation) in permutations.iter().enumerate() {
                                let rows = [
                                    canonical_rows[permutation[0]],
                                    canonical_rows[permutation[1]],
                                    canonical_rows[permutation[2]],
                                ];
                                let constraints =
                                    constraints_with_first_edge_rows(&rows, ds, path_cap);
                                let problem = Topp2ProblemBuilder::with_constraint(
                                    &constraints,
                                    (0, 2),
                                    (a_prev, 0.0),
                                )
                                .build()
                                .unwrap();
                                let reach = reach_set2_backward(&problem, &options).unwrap();
                                let mut actual_rows = Vec::new();
                                constraints.fill_acc_topp2::<true>(&mut actual_rows, 0);
                                let lp_tol = LpToleranceOptions::with_feas_tol(epsilon);
                                let reduced_rows =
                                    || actual_rows.iter().map(|&(c, d, rhs)| (c, rhs - d * a_prev));
                                let (raw_max, raw_min) = lp_1d::<false>(reduced_rows(), &lp_tol);
                                assert!(
                                    raw_max < raw_min,
                                    "generated interval is not inverted: cap={path_cap:e}, prev={a_prev:e}, ds_exp={ds_exponent}, row_scale_exp={row_scale_exponent}, cond_exp={condition_exponent}, gap_factor={gap_factor}, perm={permutation_id}, raw=[{raw_min:e},{raw_max:e}]"
                                );

                                // Recreate the previous forward control flow.  For a
                                // sub-tolerance gap it folds the exact inversion to a
                                // midpoint; otherwise it returns NaNs, which f64
                                // clipping silently replaces by the backward interval.
                                let (legacy_max, legacy_min) =
                                    lp_1d::<true>(reduced_rows(), &lp_tol);
                                let legacy_choice = legacy_max.min(reach.a_max[1]);
                                let legacy_lower = legacy_min.max(reach.a_min[1]);
                                assert!(
                                    legacy_choice >= legacy_lower,
                                    "legacy fixture did not survive clipping: cap={path_cap:e}, prev={a_prev:e}, ds_exp={ds_exponent}, row_scale_exp={row_scale_exponent}, cond_exp={condition_exponent}, gap_factor={gap_factor}, perm={permutation_id}"
                                );
                                if legacy_max.is_nan() && legacy_min.is_nan() {
                                    masked_nan_cases += 1;
                                    let gross_residual = legacy_choice + a_prev - path_cap;
                                    assert!(
                                        gross_residual > 0.5 * a_prev,
                                        "legacy NaN masking was not grossly infeasible: residual={gross_residual:e}"
                                    );
                                } else {
                                    midpoint_cases += 1;
                                    assert_eq!(legacy_max, legacy_min);
                                    assert!(
                                        robust_upper - legacy_choice > 0.5 * tiny_upper_offset,
                                        "legacy midpoint was not observably suboptimal: legacy={legacy_choice:e}, robust_upper={robust_upper:e}"
                                    );
                                }

                                let profile = topp2_ra(&problem, &options).unwrap_or_else(|error| {
                                    panic!(
                                        "new solver failed: cap={path_cap:e}, prev={a_prev:e}, ds_exp={ds_exponent}, row_scale_exp={row_scale_exponent}, cond_exp={condition_exponent}, gap_factor={gap_factor}, perm={permutation_id}: {error:?}"
                                    )
                                });
                                assert_eq!(profile.len(), 3);
                                assert_eq!(profile[0], a_prev);
                                assert_eq!(profile[2], 0.0);

                                // The robust row x + a_prev <= path_cap is the
                                // active maximizer after both tolerance policies are
                                // applied.  It gives an analytic oracle independent
                                // of LP-1D and the recovery routine.
                                let oracle = robust_upper + 2.0_f64.sqrt() * epsilon;
                                let oracle_roundoff = 512.0 * f64::EPSILON * oracle.abs().max(1.0);
                                assert!(
                                    (profile[1] - oracle).abs() <= oracle_roundoff,
                                    "not the largest certified state: got={:e}, oracle={oracle:e}, allowance={oracle_roundoff:e}, cap={path_cap:e}, prev={a_prev:e}, ds_exp={ds_exponent}, row_scale_exp={row_scale_exponent}, cond_exp={condition_exponent}, gap_factor={gap_factor}, perm={permutation_id}",
                                    profile[1]
                                );

                                for &(coef_curr, coef_prev, rhs) in &actual_rows {
                                    let raw = coef_curr * profile[1] + coef_prev * profile[0] - rhs;
                                    let norm = coef_curr.hypot(coef_prev);
                                    let normalized = raw / norm;
                                    let normalized_second_order = (coef_curr / norm) * profile[1]
                                        + (coef_prev / norm) * profile[0]
                                        - rhs / norm;
                                    assert!(
                                        normalized <= epsilon
                                            && normalized_second_order <= epsilon
                                            && raw <= epsilon * rhs.abs().max(1.0),
                                        "uncertified row: row=({coef_curr:e},{coef_prev:e},{rhs:e}), raw={raw:e}, norm1={normalized:e}, norm2={normalized_second_order:e}, profile={:?}",
                                        profile
                                    );
                                }
                                let (path_exceed, physical_exceed) =
                                    constraints.exceed_topp2(0, &profile);
                                assert!(path_exceed <= 0.0);
                                assert!(
                                    physical_exceed <= 1.0e-5,
                                    "physical audit failed: exceed={physical_exceed:e}, profile={profile:?}, cap={path_cap:e}, prev={a_prev:e}, ds_exp={ds_exponent}, row_scale_exp={row_scale_exponent}, cond_exp={condition_exponent}, gap_factor={gap_factor}, perm={permutation_id}"
                                );
                                total_cases += 1;
                            }
                        }
                    }
                }
            }
        }

        assert_eq!(total_cases, 2592);
        assert_eq!(midpoint_cases, 1296);
        assert_eq!(masked_nan_cases, 1296);
    }

    #[test]
    #[ignore = "bindings"]
    fn test_topp2_ra_bindings_parity() -> Result<(), CoppError> {
        let dim = 3;
        let num_waypoints = 8;
        let n: usize = 201;
        let pi = std::f64::consts::PI;

        let waypoints = DMatrix::<f64>::from_fn(dim, num_waypoints, |axis, j| {
            let s = j as f64 / (num_waypoints - 1) as f64;
            match axis {
                0 => 0.20 * (2.0 * pi * s).sin(),
                1 => 0.15 * (1.5 * pi * s).cos(),
                2 => 0.10 * s * (1.0 - s),
                _ => unreachable!("dimension is fixed to 3"),
            }
        });
        let path = Path::from_waypoints_interpolating(&waypoints, SplineConfig::default())?;
        let s = DMatrix::<f64>::from_fn(1, n, |_, j| j as f64 / (n - 1) as f64);

        let mut robot = Robot::with_capacity(dim, n);
        robot
            .with_s(&s.as_view())?
            .with_q_from_path_2nd(&path, 0, n)?
            .with_q_from_path_3rd(&path, 0, n)?;
        add_symmetric_axial_limits_for_test(&mut robot, 10.0, 50.0, Some(1000.0))?;

        let topp2_problem = Topp2ProblemBuilder::new(&robot, (0, n - 1), (0.0, 0.0)).build()?;
        let options = ReachSet2OptionsBuilder::new().build()?;
        let a_profile = topp2_ra(&topp2_problem, &options)?;

        let (t_final, t_s) = s_to_t_topp2(s.as_slice(), &a_profile, 0.0)?;
        let s_t = t_to_s_topp2(
            s.as_slice(),
            &a_profile,
            &t_s,
            InterpolationMode::UniformTimeGrid(0.0, 1e-3, true),
        )?;

        crate::verbosity_log!(
            Verbosity::Summary,
            "TOPP2-RA Rust bindings parity test: t_final={:.17}, s_t.len={}",
            t_final,
            s_t.len()
        );

        Ok(())
    }

    /// Conditions: release, --include-ignored, CPU = Intel(R) Core(TM) Ultra 9 285K.
    /// Average over 10000 experiments: tc_topp2_ra = 0.286978 ms, tc_interpolation = 0.002001 ms, t_final = 6.168578
    #[test]
    #[ignore = "slow"]
    fn test_topp2_ra_robust() -> Result<(), CoppError> {
        run_test_topp2_ra_repeated(10000, true)
    }

    fn run_test_topp2_ra_repeated(n_exp: usize, flag_print_step: bool) -> Result<(), CoppError> {
        let mut tc_sum_ra = Duration::ZERO;
        let mut tc_sum_interpolation = Duration::ZERO;
        let mut t_final_sum = 0.0;

        let options = ReachSet2OptionsBuilder::new()
            .lp_feas_tol(1E-9)
            .a_cmp_abs_tol(1E-9)
            .a_cmp_rel_tol(1E-9)
            .verbosity(Verbosity::Summary)
            .build()?;

        for i_exp in 0..n_exp {
            let dim = 7;
            let n: usize = 1000;
            let mut robot = Robot::with_capacity(dim, n);

            let mut rng = rand::rng();
            let (s, path, _, _) = lissajous_path_for_test(dim, n, &mut rng).map_err(|e| {
                CoppError::InvalidInput("lissajous_path_for_test".into(), e.to_string())
            })?;
            robot
                .with_s(&s.as_view())?
                .with_q_from_path_2nd(&path, 0, n)?;
            add_symmetric_axial_limits_for_test(&mut robot, 1.0, 1.0, None)?;

            let start = Instant::now();
            let topp2_problem = Topp2ProblemBuilder::new(&robot, (0, n - 1), (0.0, 0.0)).build()?;
            let a_profile = topp2_ra(&topp2_problem, &options)?;
            let tc_topp2_ra = start.elapsed();

            let b_profile = a_to_b_topp2(s.as_slice(), &a_profile)?;
            assert!(
                !izip!(
                    s.as_slice().windows(2),
                    a_profile.windows(2),
                    b_profile.iter()
                )
                .any(|(s_pair, a_pair, b)| {
                    let ds_double = 2.0 * (s_pair[1] - s_pair[0]);
                    let db = (a_pair[1] - a_pair[0]) / ds_double;
                    (*b - db).abs() > 1e-3
                }),
                "b_profile generation failed!"
            );

            let start = Instant::now();
            let (t_final, t_s) = s_to_t_topp2(s.as_slice(), &a_profile, 0.0)?;
            assert_eq!(t_s.len(), s.ncols());
            let tc_interpolation = start.elapsed();
            let s_t = t_to_s_topp2(
                s.as_slice(),
                &a_profile,
                &t_s,
                InterpolationMode::UniformTimeGrid(0.0, 1E-3, true),
            )?;

            tc_sum_ra += tc_topp2_ra;
            tc_sum_interpolation += tc_interpolation;
            t_final_sum += t_final;

            if flag_print_step && ((i_exp + 1) % 100 == 0) {
                crate::verbosity_log!(
                    Verbosity::Summary,
                    "Exp #{}: tc_topp2_ra = {:.4} ms, tc_interpolation = {:.4} ms, t_final = {:.4} s, s_t.len() = {}",
                    i_exp + 1,
                    tc_topp2_ra.as_secs_f64() * 1E3,
                    tc_interpolation.as_secs_f64() * 1E3,
                    t_final,
                    s_t.len()
                );
            }
        }

        crate::verbosity_log!(
            Verbosity::Summary,
            "Average over {} experiments: tc_topp2_ra = {:.6} ms, tc_interpolation = {:.6} ms, t_final = {:.6}",
            n_exp,
            tc_sum_ra.as_secs_f64() * 1E3 / n_exp as f64,
            tc_sum_interpolation.as_secs_f64() * 1E3 / n_exp as f64,
            t_final_sum / n_exp as f64
        );

        Ok(())
    }
}
