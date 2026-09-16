//! Interpolation and profile-conversion utilities for third-order path parameterization.
//!
//! # Method identity
//! This module serves both:
//! - **Time-Optimal Path Parameterization (TOPP3)** workflows,
//! - **Convex-Objective Path Parameterization (COPP3)** workflows.
//!
//! # Scope
//! This module provides deterministic conversions between:
//! - node profiles `a(s) = \dot{s}^2` and `b(s) = \ddot{s}` sampled on stations,
//! - time mapping `t(s)`,
//! - inverse sampling `s(t)`.
//!
//! # Conventions
//! - Path grid uses station samples `s[0..=n]`.
//! - Both `a` and `b` are node-based in TOPP3/COPP3 (`a.len() == b.len() == s.len()`).
//! - `num_stationary = (head, tail)` indicates stationary boundary counts at start/end.
//! - Ordinary intervals normally use the left polynomial anchor. The last
//!   ordinary interval uses the right boundary anchor; an exact zero endpoint
//!   takes priority (left if both endpoints are zero). Forward/inverse time
//!   and profile SOC checks share this convention. Node pairs must already
//!   satisfy the discrete dynamics up to their constructor's own rounding.
//!
//! # Example
//! The example below converts a third-order profile from station samples to
//! cumulative time and then samples the inverse map `s(t)`.
//!
//! ```rust
//! # fn main() -> Result<(), copp::diag::CoppError> {
//! use copp::InterpolationMode;
//! use copp::solver::topp3_lp::{s_to_t_topp3, t_to_s_topp3, Topp3Profile};
//!
//! let s = [0.0, 0.5, 1.0];
//! let profile = Topp3Profile::new(
//!     vec![1.0, 1.0, 1.0],
//!     vec![0.0, 0.0, 0.0],
//!     (0, 0),
//! );
//!
//! let (_t_final, t_s) = s_to_t_topp3(&s, profile.as_parts(), 0.0)?;
//! let s_t = t_to_s_topp3(
//!     &s,
//!     profile.as_parts(),
//!     &t_s,
//!     InterpolationMode::UniformTimeGrid(0.0, 0.25, true),
//! )?;
//!
//! assert_eq!(s_t.first().copied(), Some(0.0));
//! assert_eq!(s_t.last().copied(), Some(1.0));
//! # Ok(())
//! # }
//! ```

use crate::copp::InterpolationMode;
use crate::copp::copp3::exact_interval::{
    IntervalANonnegativeClassification, certify_interval_endpoint_ln,
    classify_interval_a_nonnegative, exact_interval_negative_m_abs_ln,
    exact_interval_nonnegative_m_ln, exact_interval_positive_d_ln,
    profile_interval_uses_right_anchor,
};
use crate::copp::copp3::{Topp3ProfileMut, Topp3ProfileRef};
use crate::diag::{
    CoppError, check_input_len_at_least, check_input_len_equal, check_input_non_negative,
    check_input_not_empty, check_input_not_nan_infinite, check_input_slice_non_negative,
    check_input_slice_not_nan_infinite, check_input_strictly_increasing,
};
use crate::math::numerical::{EPS_ZERO, solve_2x2};
use itertools::izip;
use nalgebra::ComplexField;

/// Taylor threshold for the natural constant-jerk inverse map.
const TAYLOR_INVERSE_CONSTANT_JERK: f64 = 1E-3;
/// Taylor threshold for the natural constant-jerk interval-time map.
const TAYLOR_INTERVAL_TIME_Z: f64 = 1E-4;
/// The direct `z` expression is not a reliable distance to one inside this
/// envelope. This only selects the exact-sign cold path.
const INTERVAL_TIME_NEAR_ONE_DELTA: f64 = 64.0 * f64::EPSILON;

/// Compute cumulative time profile `t(s)` from a TOPP3/COPP3 profile.
///
/// # Semantics
/// - `t_s[i]` is the time at station `s[i]`.
/// - initial condition is `t_s[0] = t0`.
/// - returns `(t_final, t_s)` where `t_final == *t_s.last().unwrap()`.
///
/// # Input contract
/// - valid when `s.len() >= 2 + profile.2.0 + profile.2.1`;
/// - requires `profile.0.len() == s.len()` and `profile.1.len() == s.len()`;
/// - all inputs must contain only finite values;
/// - `s` must be strictly increasing.
///
/// # Returns
/// Returns `(t_final, t_s)` where `t_s[i]` is cumulative time at `s[i]`.
///
/// # Errors
/// Returns [`CoppError::InvalidInput`](crate::diag::CoppError::InvalidInput) when dimensions, stationary counts,
/// monotonicity, positivity, or numeric finiteness requirements are violated.
///
/// # Contract
/// - `t_s.len() == s.len()` on valid input.
/// - `t_s[0] == t0` on valid input.
pub fn s_to_t_topp3(
    s: &[f64],
    profile: Topp3ProfileRef<'_>,
    t0: f64,
) -> Result<(f64, Vec<f64>), CoppError> {
    check_topp3_sab("s_to_t_topp3", s, profile)?;
    check_input_not_nan_infinite("s_to_t_topp3", "t0", t0)?;
    let (a, b, num_stationary) = profile;
    let mut t_s = Vec::<f64>::with_capacity(s.len()); // t_s[i] = t(s[i]), begin from t0
    let mut t_prev = t0;
    let n = s.len() - 1;
    t_s.push(t_prev);
    if num_stationary.0 > 0 {
        let s0 = s.first().unwrap();
        t_s.resize(1 + num_stationary.0, t_prev);
        for (t_curr, a_curr, s_curr) in izip!(t_s.iter_mut(), a.iter(), s.iter()).skip(1) {
            *t_curr += 3.0 * (s_curr - s0) / a_curr.sqrt();
        }
        t_prev = *t_s.last().unwrap();
    }
    for (id, (s_pair, b_pair, a_pair)) in izip!(s.windows(2), b.windows(2), a.windows(2))
        .enumerate()
        .skip(num_stationary.0)
        .take(n - num_stationary.0 - num_stationary.1)
    {
        t_prev += integral_profile_interval(
            s_pair[1] - s_pair[0],
            a_pair,
            b_pair,
            id + 1 == n - num_stationary.1,
        );
        t_s.push(t_prev);
    }
    if num_stationary.1 > 0 {
        let s_final = s.last().unwrap();
        let t_final =
            t_prev + 3.0 * (s_final - s[n - num_stationary.1]) / a[n - num_stationary.1].sqrt();
        t_s.resize(s.len(), t_final);
        if num_stationary.1 > 1 {
            for (t_curr, a_curr, s_curr) in izip!(t_s.iter_mut(), a.iter(), s.iter())
                .rev()
                .skip(1)
                .take(num_stationary.1 - 1)
            {
                *t_curr += 3.0 * (s_curr - s_final) / a_curr.sqrt();
            }
        }
    }

    let t_final = *t_s.last().unwrap();
    // Hot path: branchless reduction over the whole profile. `&` does not
    // short-circuit, so this stays a straight-line loop that can vectorize;
    // `t_final` is `t_s.last()` and is therefore already covered.
    let all_finite = t_s.iter().fold(true, |acc, value| acc & value.is_finite());
    if !all_finite {
        // Cold path: an error is being returned anyway, so a second, scalar
        // pass to locate the offending station costs nothing that matters.
        // The natural interval kernel diverges when a station has
        // `a == b == 0`, and
        // returns NaN when `a(s)` goes negative between two stations, so the
        // neighbouring `(a, b)` pair is what identifies the cause.
        let index = t_s
            .iter()
            .position(|value| !value.is_finite())
            .unwrap_or(t_s.len() - 1);
        // `t_s[k+1] = t_s[k] + integral(a[k], b[k], b[k+1])`, so the interval
        // that diverged is `[index-1, index]`; the stationary head instead uses
        // `a[index]` directly. Report a window that covers both.
        let lo = index.saturating_sub(1);
        let hi = (index + 1).min(a.len() - 1);
        let window = (lo..=hi)
            .map(|j| {
                format!(
                    "  [{j}] a = {:.6e}, b = {:.6e}, s = {:.6e}",
                    a[j], b[j], s[j]
                )
            })
            .collect::<Vec<_>>()
            .join("\n");
        return Err(CoppError::InvalidInput(
            "s_to_t_topp3".into(),
            format!(
                "computed time profile contains NaN or infinity: first at index {index} \
                 (t = {}), num_stationary = {num_stationary:?}\n{window}",
                t_s[index]
            ),
        ));
    }
    check_input_strictly_increasing("s_to_t_topp3", "t_s", &t_s)?;
    Ok((t_final, t_s))
}

/// Interpolate inverse mapping `s(t)` from a TOPP3/COPP3 profile and sampled `t(s)`.
///
/// # Modes
/// - [`UniformTimeGrid`](crate::InterpolationMode::UniformTimeGrid)`(t0, dt, include_final)`: generate uniform time samples;
/// - `NonUniformTimeGrid(t_sample)`: use caller-provided increasing samples.
///
/// # Input contract
/// - requires `s.len() >= 2`, profile slice lengths equal to `s.len()`, and `t_s.len() == s.len()`;
/// - requires `t_s` strictly increasing;
/// - all profile and time-grid values must be finite.
///
/// # Output semantics
/// - output length matches requested sample count in each mode;
/// - for out-of-range time samples, output entries are `NaN`.
///
/// # Returns
/// Returns sampled `s(t)` values under the requested interpolation `mode`.
///
/// # Errors
/// Returns [`CoppError::InvalidInput`](crate::diag::CoppError::InvalidInput) when dimensions, stationary counts,
/// monotonicity, positivity, or numeric finiteness requirements are violated.
///
/// # Contract
/// - preserves caller time-sample ordering.
/// - malformed input is reported as [`CoppError::InvalidInput`](crate::diag::CoppError::InvalidInput).
pub fn t_to_s_topp3(
    s: &[f64],
    profile: Topp3ProfileRef<'_>,
    t_s: &[f64],
    mode: InterpolationMode<'_>,
) -> Result<Vec<f64>, CoppError> {
    check_topp3_sab("t_to_s_topp3", s, profile)?;
    check_input_len_equal(
        "t_to_s_topp3",
        "`t_s.len()`",
        t_s.len(),
        "`s.len()`",
        s.len(),
    )?;
    check_input_slice_not_nan_infinite("t_to_s_topp3", "t_s", t_s)?;
    check_input_strictly_increasing("t_to_s_topp3", "t_s", t_s)?;
    match mode {
        InterpolationMode::UniformTimeGrid(t0, dt, include_final) => {
            check_input_not_nan_infinite("t_to_s_topp3", "t0", t0)?;
            check_input_not_nan_infinite("t_to_s_topp3", "dt", dt)?;
            if dt <= 0.0 {
                return Err(CoppError::InvalidInput(
                    "t_to_s_topp3".into(),
                    format!("`dt` = {dt} must be positive"),
                ));
            }
            // num_t * dt + t0 <= t_final
            let num_t = ((t_s.last().unwrap() - t0) / dt).floor() as usize;
            let mut s_t = t_to_s_topp3_core(
                s,
                profile,
                t_s,
                (0..num_t).map(|i| t0 + i as f64 * dt),
                num_t,
            );
            if include_final {
                let flag = if s_t.is_empty() {
                    t0 <= *t_s.last().unwrap()
                } else {
                    *s_t.last().unwrap() < *s.last().unwrap()
                };
                if flag {
                    s_t.push(*s.last().unwrap());
                }
            }
            Ok(s_t)
        }
        InterpolationMode::NonUniformTimeGrid(t_sample) => {
            check_input_not_empty("t_to_s_topp3", "`t_sample`", t_sample.len())?;
            check_input_slice_not_nan_infinite("t_to_s_topp3", "t_sample", t_sample)?;
            check_input_strictly_increasing("t_to_s_topp3", "t_sample", t_sample)?;
            Ok(t_to_s_topp3_core(
                s,
                profile,
                t_s,
                t_sample.iter().cloned(),
                t_sample.len(),
            ))
        }
    }
}

/// Core inverse interpolation kernel for [`t_to_s_topp3`](crate::solver::topp3_socp::t_to_s_topp3).
///
/// The public wrapper validates dimensions, finiteness, station ordering, and
/// sample ordering before calling this routine. This core then walks the time
/// samples once and maps each sample into the corresponding station interval.
fn t_to_s_topp3_core(
    s: &[f64],
    profile: Topp3ProfileRef<'_>,
    t_s: &[f64],
    mut t_sample: impl Iterator<Item = f64>,
    len_t_sample: usize,
) -> Vec<f64> {
    let (a, b, num_stationary) = profile;
    // Map t to s
    let &t_start = t_s.first().unwrap();
    let &t_final = t_s.last().unwrap();
    let mut s_t = Vec::<f64>::with_capacity(len_t_sample + 1); // s_t[i] = s(t[i])
    let Some(mut t_curr) = t_sample.next() else {
        return vec![];
    };
    while t_curr < t_start {
        s_t.push(f64::NAN);
        let Some(t) = t_sample.next() else {
            return s_t;
        };
        t_curr = t;
    }

    if num_stationary.0 > 0 {
        let s0 = s.first().unwrap();
        let a_stationary = a[num_stationary.0];
        let t_stationary = t_s[num_stationary.0];
        let d3u_over_6 =
            a_stationary.sqrt() * a_stationary / (27.0 * (s[num_stationary.0] - s0).powi(2));
        while t_curr <= t_stationary {
            s_t.push(s0 + d3u_over_6 * (t_curr - t_start).powi(3));
            let Some(t) = t_sample.next() else {
                return s_t;
            };
            t_curr = t;
        }
    }

    for (id, (s_pair, a_pair, b_pair, t_pair)) in
        izip!(s.windows(2), a.windows(2), b.windows(2), t_s.windows(2))
            .enumerate()
            .skip(num_stationary.0)
            .take(s.len() - num_stationary.0 - num_stationary.1 - 1)
    {
        let h = s_pair[1] - s_pair[0];
        let reverse = profile_interval_uses_right_anchor(
            a_pair[0],
            a_pair[1],
            id + 1 == s.len() - num_stationary.1 - 1,
        );
        while t_curr <= t_pair[1] {
            // Keep the sampled station exact when the query is an existing
            // time knot; the analytic inverse is needed only in the interior.
            let position = if t_curr == t_pair[1] {
                s_pair[1]
            } else if t_curr == t_pair[0] {
                s_pair[0]
            } else if reverse {
                s_pair[1]
                    - inverse_constant_jerk_interval(
                        h,
                        a_pair[1],
                        -b_pair[1],
                        -b_pair[0],
                        t_pair[1] - t_curr,
                    )
            } else {
                s_pair[0]
                    + inverse_constant_jerk_interval(
                        h,
                        a_pair[0],
                        b_pair[0],
                        b_pair[1],
                        t_curr - t_pair[0],
                    )
            };
            s_t.push(position);
            let Some(t) = t_sample.next() else {
                return s_t;
            };
            t_curr = t;
        }
    }

    if num_stationary.1 > 0 {
        let s_final = s.last().unwrap();
        let a_stationary = a[s.len() - num_stationary.1 - 1];
        let d3u_over_6 = a_stationary.sqrt() * a_stationary
            / (27.0 * (s_final - s[s.len() - num_stationary.1 - 1]).powi(2));
        while t_curr <= t_final {
            s_t.push(s_final + d3u_over_6 * (t_curr - t_final).powi(3));
            let Some(t) = t_sample.next() else {
                return s_t;
            };
            t_curr = t;
        }
    }

    s_t.push(f64::NAN);
    while t_sample.next().is_some() {
        s_t.push(f64::NAN);
    }
    s_t
}

/// Profile-level anchor convention matches the final SOC scan. An exact
/// stationary endpoint without its own stationary block always diverges;
/// changing the polynomial's anchor must not erase that double zero.
#[inline]
fn integral_profile_interval(h: f64, a: &[f64], b: &[f64], last_ordinary: bool) -> f64 {
    profile_interval_time(h, a[0], b[0], a[1], b[1], last_ordinary)
}

/// Evaluate one delivered-profile interval with its original endpoint-anchor
/// classification. Callers that validate a subdomain must preserve the
/// full-profile `last_ordinary` flag instead of slicing and reclassifying it.
#[inline]
pub(crate) fn profile_interval_time(
    h: f64,
    a_left: f64,
    b_left: f64,
    a_right: f64,
    b_right: f64,
    last_ordinary: bool,
) -> f64 {
    if (a_left == 0.0 && b_left == 0.0) || (a_right == 0.0 && b_right == 0.0) {
        return f64::INFINITY;
    }
    if profile_interval_uses_right_anchor(a_left, a_right, last_ordinary) {
        integral_constant_jerk_interval(h, a_right, -b_right, -b_left)
    } else {
        integral_constant_jerk_interval(h, a_left, b_left, b_right)
    }
}

/// Stable `psi(z)` in the endpoint form of the natural constant-jerk time
/// integral.  Only the value is needed here; no derivatives are carried.
#[inline(always)]
fn interval_time_psi(z: f64) -> f64 {
    if z.abs() < TAYLOR_INTERVAL_TIME_Z {
        z.mul_add(z.mul_add(z.mul_add(1.0 / 7.0, 1.0 / 5.0), 1.0 / 3.0), 1.0)
    } else {
        let root = z.abs().sqrt();
        if z > 0.0 {
            root.atanh() / root
        } else {
            root.atan() / root
        }
    }
}

#[inline(always)]
fn logaddexp(lhs: f64, rhs: f64) -> f64 {
    if lhs == f64::INFINITY || rhs == f64::INFINITY {
        return f64::INFINITY;
    }
    if lhs == f64::NEG_INFINITY {
        return rhs;
    }
    if rhs == f64::NEG_INFINITY {
        return lhs;
    }
    let high = lhs.max(rhs);
    let low = lhs.min(rhs);
    high + (low - high).exp().ln_1p()
}

#[inline(always)]
fn interval_time_scaled_z(h: f64, b: f64, u: f64, sqrt_sum: f64) -> (f64, f64) {
    if u == b {
        return (0.0, f64::NEG_INFINITY);
    }
    let difference = u - b;
    let ln_abs_difference = if difference.is_finite() {
        difference.abs().ln()
    } else {
        logaddexp(u.abs().ln(), b.abs().ln())
    };
    let ln_magnitude = h.ln() + ln_abs_difference - 2.0 * sqrt_sum.ln();
    let magnitude = ln_magnitude.exp();
    (if u > b { magnitude } else { -magnitude }, ln_magnitude)
}

#[inline(always)]
fn interval_time_scaled_value(h: f64, sqrt_sum: f64, d: f64, psi: f64) -> f64 {
    if d.is_finite() && d > 0.0 {
        let direct = d * (2.0 * psi);
        if direct.is_finite() && direct > 0.0 {
            return direct;
        }
    }
    (std::f64::consts::LN_2 + h.ln() - sqrt_sum.ln() + psi.ln()).exp()
}

#[inline(always)]
fn interval_time_psi_near_one(ln_delta: f64) -> f64 {
    debug_assert!(ln_delta < 0.0);
    let z = (-ln_delta.exp_m1()).clamp(0.0, 1.0);
    let root = z.sqrt();
    (root.ln_1p() - 0.5 * ln_delta) / root
}

/// Traverse one ordinary interval under the natural, unmaterialized model
///
/// `a(x) = a + 2*b*x + (u-b)*x^2/h`, `0 <= x <= h`.
///
/// The exact-sign classifier comes from the shared exact-interval kernel.
/// In particular, finiteness is never decided from the
/// different polynomial obtained by first rounding `(u-b)/h` and then using
/// that rounded coefficient in a quadratic evaluation.
#[inline]
fn integral_constant_jerk_interval(h: f64, a: f64, b: f64, u: f64) -> f64 {
    let Some(classified) = classify_interval_a_nonnegative(h, a, b, u) else {
        return f64::NAN;
    };
    let m = h.mul_add(b, a);
    let a_next = h.mul_add(u, m);
    let exact_endpoint_ln = match certify_interval_endpoint_ln(h, a, b, u, m, a_next) {
        Ok(value) => value,
        Err(()) => return f64::NAN,
    };
    let sqrt_a = a.sqrt();
    let sqrt_next = match exact_endpoint_ln {
        Some(ln_endpoint) if ln_endpoint.is_finite() => (0.5 * ln_endpoint).exp(),
        Some(_) => 0.0,
        None => a_next.sqrt(),
    };
    let curvature = (u - b) / h;

    // Two simple zero endpoints are integrable; a double zero is not.
    if (a == 0.0 && b > 0.0 && u == -b) || (a == 0.0 && sqrt_next == 0.0 && curvature < 0.0) {
        return std::f64::consts::PI / (-curvature).sqrt();
    }
    if (a == 0.0 && b == 0.0 && u >= 0.0) || (sqrt_next == 0.0 && u == 0.0 && b <= 0.0) {
        return f64::INFINITY;
    }
    if matches!(classified, IntervalANonnegativeClassification::InteriorZero) {
        return f64::INFINITY;
    }

    let sqrt_sum = sqrt_a + sqrt_next;
    let d = h / sqrt_sum;
    let d2 = d * d;
    let raw_z = curvature * d2;
    let unsafe_scale =
        d == 0.0 || !d.is_finite() || d2 == 0.0 || !d2.is_finite() || !raw_z.is_finite();
    let (z, ln_abs_z) = if unsafe_scale {
        interval_time_scaled_z(h, b, u, sqrt_sum)
    } else {
        (raw_z, raw_z.abs().ln())
    };
    let near_singularity = if unsafe_scale {
        z > 0.5
    } else {
        z >= 1.0 - INTERVAL_TIME_NEAR_ONE_DELTA
    };

    if near_singularity {
        if !d.is_finite() {
            return f64::INFINITY;
        }
        let ln_delta = match classified {
            IntervalANonnegativeClassification::InteriorPositive => {
                let Some(ln_d) = exact_interval_positive_d_ln(h, a, b, u) else {
                    return f64::NAN;
                };
                let Some(ln_abs_m) = exact_interval_negative_m_abs_ln(h, a, b) else {
                    return f64::NAN;
                };
                let ln_pq = sqrt_a.ln() + sqrt_next.ln();
                std::f64::consts::LN_2 + h.ln() + ln_d
                    - logaddexp(ln_pq, ln_abs_m)
                    - 2.0 * sqrt_sum.ln()
            }
            IntervalANonnegativeClassification::Endpoint => {
                let Some(ln_m) = exact_interval_nonnegative_m_ln(h, a, b) else {
                    return f64::NAN;
                };
                let ln_pq = if sqrt_a == 0.0 || sqrt_next == 0.0 {
                    f64::NEG_INFINITY
                } else {
                    sqrt_a.ln() + sqrt_next.ln()
                };
                std::f64::consts::LN_2 + logaddexp(ln_m, ln_pq) - 2.0 * sqrt_sum.ln()
            }
            IntervalANonnegativeClassification::InteriorZero => return f64::INFINITY,
        };
        if ln_delta.is_finite() && ln_delta < 0.0 {
            let psi = interval_time_psi_near_one(ln_delta);
            return interval_time_scaled_value(h, sqrt_sum, d, psi);
        }
        if !z.is_finite() || z >= 1.0 {
            return f64::INFINITY;
        }
    }

    if unsafe_scale && z.is_sign_negative() && ln_abs_z > 0.5 * f64::MAX.ln() {
        let ln_psi = std::f64::consts::FRAC_PI_2.ln() - 0.5 * ln_abs_z;
        return (std::f64::consts::LN_2 + h.ln() - sqrt_sum.ln() + ln_psi).exp();
    }

    let psi = interval_time_psi(z);
    if unsafe_scale || z > 0.5 {
        interval_time_scaled_value(h, sqrt_sum, d, psi)
    } else {
        2.0 * d * psi
    }
}

/// Invert the same natural ordinary-interval model used by
/// [`integral_constant_jerk_interval`].  The rounded dimensionless `eps` below
/// is only an evaluation parameter for the analytic solution; no second
/// rounded-coefficient polynomial is used for feasibility or interval time.
#[inline(always)]
fn inverse_constant_jerk_interval(h: f64, a: f64, b: f64, u: f64, dt: f64) -> f64 {
    if dt == 0.0 {
        return 0.0;
    }
    let eps = ((u - b) / h) * dt * dt;
    let sqrt_a = a.sqrt();
    let half_b_dt = 0.5 * b * dt;

    if eps.abs() < TAYLOR_INVERSE_CONSTANT_JERK {
        let first = eps.mul_add(
            eps.mul_add(eps.mul_add(1.0 / 5040.0, 1.0 / 120.0), 1.0 / 6.0),
            1.0,
        );
        let second = eps.mul_add(
            eps.mul_add(eps.mul_add(1.0 / 20160.0, 1.0 / 360.0), 1.0 / 12.0),
            1.0,
        );
        dt * sqrt_a.mul_add(first, half_b_dt * second)
    } else if eps > 0.0 {
        let k = eps.sqrt();
        let sinh_half_over_k = (0.5 * k).sinh() / k;
        dt * sqrt_a.mul_add(
            k.sinh() / k,
            2.0 * b * dt * sinh_half_over_k * sinh_half_over_k,
        )
    } else {
        let k = (-eps).sqrt();
        let sin_half_over_k = (0.5 * k).sin() / k;
        dt * sqrt_a.mul_add(
            k.sin() / k,
            2.0 * b * dt * sin_half_over_k * sin_half_over_k,
        )
    }
}

/// Post-process a mutable `(a, b)` profile so that interpolated `a(s)` stays strictly positive per interval.
///
/// This is a numerical safety utility for downstream timing integration on
/// profiles that may be very close to zero due to finite precision.
///
/// # Returns
/// Returns `true` when in-place adjustment succeeds, otherwise `false`.
///
/// # Errors
/// Returns [`CoppError::InvalidInput`](crate::diag::CoppError::InvalidInput) when dimensions, station ordering, profile
/// positivity, stationary counts, or numeric finiteness requirements are violated.
///
/// # Contract
/// - requires `a.len() == b.len() == s.len()` and `s.len() >= 4`;
/// - requires endpoint `a` values to be nonnegative.
pub fn force_positive_a(
    profile: Topp3ProfileMut<'_>,
    s: &[f64],
    a_min: f64,
) -> Result<bool, CoppError> {
    let (a, b, num_stationary) = profile;
    let n = s.len();
    if a.len() != n || b.len() != n {
        return Err(CoppError::InvalidInput(
            "force_positive_a".into(),
            format!(
                "`a.len()` = {} and `b.len()` = {} must equal `s.len()` = {}",
                a.len(),
                b.len(),
                n
            ),
        ));
    }
    if n < 4 {
        return Err(CoppError::InvalidInput(
            "force_positive_a".into(),
            format!("`s.len()` = {n} must be at least 4"),
        ));
    }
    check_stationary_counts("force_positive_a", n, num_stationary)?;
    check_input_slice_not_nan_infinite("force_positive_a", "s", s)?;
    check_input_slice_non_negative("force_positive_a", "a", a)?;
    check_input_slice_not_nan_infinite("force_positive_a", "b", b)?;
    check_input_non_negative("force_positive_a", "a_min", a_min)?;
    check_input_strictly_increasing("force_positive_a", "s", s)?;
    // Now we have a(s[i]) >= 0, and we would like to modify a(s) > 0 for s in (s[i], s[i+1]) if a(s) can be negative for some s in (s[i], s[i+1]).
    let mut flag_succeed = true;
    for i in (num_stationary.0 + 1)..(n - 2 - num_stationary.1) {
        // Consider a[i-1], a[i], a[i+1], a[i+2]
        let b1 = b[i];
        let b2 = b[i + 1];
        if b1 < 0.0 && b2 > 0.0 {
            // a(s) = a[i] + 2 * b[i] * (s - s[i]) + (b[i+1] - b[i]) / ds1 * (s - s[i])^2
            // b[i] ^ 2 < a[i] * (b[i+1] - b[i]) / ds1 should hold
            // b[i] ^ 2 * ds1 < a[i] * (b[i+1] - b[i]) should hold
            let s1 = s[i];
            let s2 = s[i + 1];
            let ds1 = s2 - s1;
            let a1 = a[i];
            let amin = a_min.max(a1.min(a[i + 1]));
            let amin = if amin > 10.0 * EPS_ZERO {
                0.1 * amin
            } else if amin > EPS_ZERO {
                EPS_ZERO
            } else {
                amin
            };
            let da = a1 - amin;
            let db = b2 - b1;
            if b1 * b1 * ds1 >= da * db {
                // a(s) <= 0 holds in (s[i], s[i+1])
                // We add c0 on (s[i-1],s[i+2]), c1 on (s[i],s[i+2]), and c2 on (s[i+1],s[i+2])
                // x[i-1] and x[i+2] should keep the same.
                // (i) --- c0*(s[i+2] - s[i-1]) + c1*(s[i+2] - s[i]) + c2*(s[i+2] - s[i+1]) == 0
                // (ii) --- c0*(s[i+2] - s[i-1])^2 + c1*(s[i+2] - s[i])^2 + c2*(s[i+2] - s[i+1])^2 == 0
                let s0 = s[i - 1];
                let s3 = s[i + 2];
                let delta_s_end = (s3 - s0, s3 - s1, s3 - s2);
                let coeff = match solve_2x2(
                    (
                        (delta_s_end.1, delta_s_end.2),
                        (delta_s_end.1 * delta_s_end.1, delta_s_end.2 * delta_s_end.2),
                    ),
                    (-delta_s_end.0, -delta_s_end.0 * delta_s_end.0),
                ) {
                    Some(coeff) => {
                        // A*[c1;c2] = b*c0
                        coeff
                    }
                    None => {
                        crate::verbosity_log!(
                            crate::diag::Verbosity::Debug,
                            "coeff is None? A = {:?}, b = {:?}",
                            (
                                (delta_s_end.1, delta_s_end.2),
                                (delta_s_end.1 * delta_s_end.1, delta_s_end.2 * delta_s_end.2)
                            ),
                            (-delta_s_end.0, -delta_s_end.0 * delta_s_end.0)
                        );
                        flag_succeed = false;
                        continue;
                    }
                };
                // c1 = coeff.0 * c0, c2 = coeff.1 * c0
                // Changes: a[i] += c0 * (s1-s0)^2, b[i] += c0 * (s1-s0), b[i+1] += c0 * (s2-s0) + c1 * (s2-s1)
                let ds0 = s1 - s0;
                let coeff_c = (ds0 * ds0, ds0, ds0 + ds1 * (1.0 + coeff.0));
                // Changes: a[i] += c0 * coeff_c.0, b[i] += c0 * coeff_c.1, b[i+1] += c0 * coeff_c.2
                // We hope that a(s) = a[i] + 2 * b[i] * (s - s[i]) + (b[i+1] - b[i]) / ds1 * (s - s[i])^2 >= amin holds in (s[i],s[i+1])
                // b[i] ^ 2 * ds1 == (a[i] - amin) * (b[i+1] - b[i]) should hold for new ones.
                // For old ones: (b[i] + coeff_c.1 * c0) ^ 2 * ds1 == (a[i] - amin + coeff_c.0 * c0) * (b[i+1] - b[i] + (coeff_c.2-coeff_c.1) * c0). Now solve c0.
                // (coeff_c.1^2 * c0^2 + 2 * b1 * coeff_c.1 * c0 + b1 ^ 2) * ds1 == coeff_c.0 * (coeff_c.2-coeff_c.1) * c0^2 + (da * (coeff_c.2-coeff_c.1) + coeff_c.0 * db) * c0 + da * db
                // (coeff_c.1^2 * ds1 - coeff_c.0 * (coeff_c.2-coeff_c.1)) * c0^2 + (2 * b1 * coeff_c.1 * ds1 - da * (coeff_c.2-coeff_c.1) - coeff_c.0 * db) * c0 + (b1 * b1 * ds1 - da * db) == 0
                let coeff_solve = (
                    coeff_c.1 * coeff_c.1 * ds1 - coeff_c.0 * (coeff_c.2 - coeff_c.1),
                    2.0 * b1 * coeff_c.1 * ds1 - da * (coeff_c.2 - coeff_c.1) - coeff_c.0 * db,
                    b1 * b1 * ds1 - da * db,
                );
                let norm = coeff_solve.0.abs() + coeff_solve.1.abs() + coeff_solve.2.abs();
                if norm < EPS_ZERO {
                    crate::verbosity_log!(
                        crate::diag::Verbosity::Debug,
                        "norm = {norm} < EPS_ZERO, coeff_solve = {coeff_solve:.8?}"
                    );
                    flag_succeed = false;
                    continue;
                }
                let norm_inv = 1.0 / norm;
                let coeff_solve = (
                    coeff_solve.0 * norm_inv,
                    coeff_solve.1 * norm_inv,
                    coeff_solve.2 * norm_inv,
                );
                // coeff_solve.0 * c0^2 + coeff_solve.1 * c0 + coeff_solve.2 == 0
                let c0 = if coeff_solve.0.abs() > EPS_ZERO {
                    // Use quadratic formula to solve for c0
                    let discriminant =
                        coeff_solve.1 * coeff_solve.1 - 4.0 * coeff_solve.0 * coeff_solve.2;
                    if discriminant < 0.0 {
                        if coeff_c.1.abs() > EPS_ZERO && coeff_c.2.abs() > EPS_ZERO {
                            (-b1 / coeff_c.1).min(b2 / coeff_c.2)
                        } else if coeff_c.1.abs() > EPS_ZERO {
                            -b1 / coeff_c.1
                        } else if coeff_c.2.abs() > EPS_ZERO {
                            b2 / coeff_c.2
                        } else {
                            crate::verbosity_log!(
                                crate::diag::Verbosity::Debug,
                                "discriminant = {discriminant:.8} < 0 for c0 (i={i}): coeff_solve = {coeff_solve:.8?}, coeff_c = {coeff_c:.8?}"
                            );
                            flag_succeed = false;
                            continue;
                        }
                    } else {
                        let sqrt_discriminant = discriminant.sqrt();
                        // c0: (max, min)
                        let c0 = if coeff_solve.0 > 0.0 {
                            (
                                (-coeff_solve.1 + sqrt_discriminant) / (2.0 * coeff_solve.0),
                                (-coeff_solve.1 - sqrt_discriminant) / (2.0 * coeff_solve.0),
                            )
                        } else {
                            (
                                (-coeff_solve.1 - sqrt_discriminant) / (2.0 * coeff_solve.0),
                                (-coeff_solve.1 + sqrt_discriminant) / (2.0 * coeff_solve.0),
                            )
                        };
                        if c0.1 >= 0.0 { c0.1 } else { c0.0 }
                    }
                } else {
                    // Linear case
                    -coeff_solve.2 / coeff_solve.1
                };
                a[i] += coeff_c.0 * c0;
                b[i] += coeff_c.1 * c0;
                b[i + 1] += coeff_c.2 * c0;
                a[i + 1] += (coeff_c.0 + (coeff_c.1 + coeff_c.2) * ds1) * c0;
            }
        }
    }

    Ok(flag_succeed)
}

/// Check the shared TOPP3 profile shape, station counts, and station ordering.
///
/// TOPP3/COPP3 interpolation uses node-based `a(s)` and `b(s)` profiles on the
/// same grid, with optional stationary head/tail sections. This helper keeps
/// those preconditions together before any timing integration is attempted.
fn check_topp3_sab(
    function_name: &str,
    s: &[f64],
    profile: Topp3ProfileRef<'_>,
) -> Result<(), CoppError> {
    let (a, b, num_stationary) = profile;
    check_stationary_counts(function_name, s.len(), num_stationary)?;
    if a.len() != s.len() || b.len() != s.len() {
        return Err(CoppError::InvalidInput(
            function_name.into(),
            format!(
                "`a.len()` = {} and `b.len()` = {} must equal `s.len()` = {}",
                a.len(),
                b.len(),
                s.len()
            ),
        ));
    }
    check_input_slice_not_nan_infinite(function_name, "s", s)?;
    check_input_slice_non_negative(function_name, "a", a)?;
    check_input_slice_not_nan_infinite(function_name, "b", b)?;
    check_input_strictly_increasing(function_name, "s", s)
}

/// Check that stationary head/tail counts leave at least one motion interval.
///
/// The minimum station count is `2 + num_stationary.0 + num_stationary.1`;
/// checked arithmetic is used so pathological `usize` inputs are rejected as
/// invalid input instead of overflowing.
fn check_stationary_counts(
    function_name: &str,
    s_len: usize,
    num_stationary: (usize, usize),
) -> Result<(), CoppError> {
    let Some(min_len) = num_stationary
        .0
        .checked_add(num_stationary.1)
        .and_then(|sum| sum.checked_add(2))
    else {
        return Err(CoppError::InvalidInput(
            function_name.into(),
            "`num_stationary` overflowed while checking dimensions".into(),
        ));
    };
    check_input_len_at_least(function_name, "`s.len()`", s_len, min_len)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_close(actual: f64, expected: f64, relative: f64) {
        let scale = actual.abs().max(expected.abs()).max(1.0);
        assert!(
            (actual - expected).abs() <= relative * scale,
            "actual={actual:.17e}, expected={expected:.17e}"
        );
    }

    #[test]
    fn profile_time_reverse_fixed_zero_round_trip() -> Result<(), CoppError> {
        let s = [0.0_f64, 1.0];
        let b = [-0.1_f64, -0.4];
        let a = [(-1.0_f64).mul_add(b[0] + b[1], 0.0), 0.0];
        assert!(integral_constant_jerk_interval(1.0, a[0], b[0], b[1]).is_nan());
        let (total, knots) = s_to_t_topp3(&s, (&a, &b, (0, 0)), 0.0)?;
        assert!(total.is_finite() && total > 0.0);
        let queries = [0.0, 0.25 * total, 0.5 * total, 0.75 * total, total];
        let positions = t_to_s_topp3(
            &s,
            (&a, &b, (0, 0)),
            &knots,
            InterpolationMode::NonUniformTimeGrid(&queries),
        )?;
        assert_eq!(positions[0], s[0]);
        assert_eq!(positions[4], s[1]);
        assert!(positions.windows(2).all(|pair| pair[0] < pair[1]));
        for (&time, &position) in queries[1..4].iter().zip(&positions[1..4]) {
            let distance = 1.0 - position;
            let reverse_b = -b[1] + (b[1] - b[0]) * distance;
            let remaining = integral_constant_jerk_interval(distance, 0.0, -b[1], reverse_b);
            assert_close(remaining, total - time, 2.0e-13);
        }
        Ok(())
    }

    #[test]
    fn profile_time_preserves_stationary_divergence_and_simple_zeros() {
        let s = [0.0, 1.0];
        for (a, b) in [([0.0, 1.0], [0.0, 1.0]), ([1.0, 0.0], [-1.0, 0.0])] {
            assert!(s_to_t_topp3(&s, (&a, &b, (0, 0)), 0.0).is_err());
        }
        let (total, _) = s_to_t_topp3(&[0.0, 2.0], (&[0.0, 0.0], &[1.0, -1.0], (0, 0)), 0.0)
            .expect("two simple zeros have finite time");
        assert_close(total, std::f64::consts::PI, 4.0 * f64::EPSILON);
    }

    #[test]
    fn profile_anchor_does_not_retry_the_easier_soc_direction() {
        // An inconsistent input must not get accepted by testing both sides
        // and choosing whichever makes the time finite.
        assert!(integral_constant_jerk_interval(1.0, 1.0, -1.0, 1.0).is_finite());
        assert!(s_to_t_topp3(&[0.0, 1.0], (&[1.0, 0.25], &[-1.0, 1.0], (0, 0)), 0.0).is_err());
    }

    #[test]
    fn natural_interval_time_handles_zero_curvature_and_two_simple_zeros() {
        assert_close(
            integral_constant_jerk_interval(3.0, 4.0, 0.0, 0.0),
            1.5,
            4.0 * f64::EPSILON,
        );
        assert_close(
            integral_constant_jerk_interval(2.0, 0.0, 1.0, -1.0),
            std::f64::consts::PI,
            4.0 * f64::EPSILON,
        );
    }

    #[test]
    fn natural_interval_time_does_not_turn_an_exact_double_root_into_a_finite_time() {
        // The ideal curvature is 1/3, which is not binary64. Materializing
        // RN((u-b)/h) first therefore defines a different quadratic. The
        // natural data satisfy D=a*(u-b)-h*b*b=0 exactly and have an interior
        // double root at x=3.
        let (h, a, b, u) = (6.0, 3.0, -1.0, 1.0);
        assert!(matches!(
            classify_interval_a_nonnegative(h, a, b, u),
            Some(IntervalANonnegativeClassification::InteriorZero)
        ));
        assert_eq!(integral_constant_jerk_interval(h, a, b, u), f64::INFINITY);
        assert!(integral_constant_jerk_interval(h, a, b, u.next_up()).is_finite());
        assert!(integral_constant_jerk_interval(h, a, b, u.next_down()).is_nan());

        let s = [0.0, h];
        let acceleration = [a, h.mul_add(u, h.mul_add(b, a))];
        let slope = [b, u];
        assert!(s_to_t_topp3(&s, (&acceleration, &slope, (0, 0)), 0.0).is_err());
    }

    #[test]
    fn natural_time_and_inverse_round_trip_for_all_curvature_signs() -> Result<(), CoppError> {
        let cases = [
            (0.5, 1.0, -0.1, 0.4),
            (0.5, 1.0, 0.2, 0.2),
            (0.5, 1.0, 0.5, -0.2),
        ];
        for (h, a, b, u) in cases {
            let a_next = h.mul_add(u, h.mul_add(b, a));
            let s = [0.0, h];
            let acceleration = [a, a_next];
            let slope = [b, u];
            let (t_final, t_s) = s_to_t_topp3(&s, (&acceleration, &slope, (0, 0)), 0.0)?;
            assert!(t_final.is_finite() && t_final > 0.0);

            let query = [0.0, 0.25 * t_final, 0.5 * t_final, 0.75 * t_final, t_final];
            let sampled = t_to_s_topp3(
                &s,
                (&acceleration, &slope, (0, 0)),
                &t_s,
                InterpolationMode::NonUniformTimeGrid(&query),
            )?;
            assert_eq!(sampled.len(), query.len());
            assert_eq!(sampled[0], 0.0);
            assert_eq!(sampled[query.len() - 1], h);
            for pair in sampled.windows(2) {
                assert!(pair[0] < pair[1], "non-monotone inverse: {sampled:?}");
            }

            for (&time, &x) in query[1..query.len() - 1]
                .iter()
                .zip(&sampled[1..sampled.len() - 1])
            {
                let b_at_x = ((u - b) / h).mul_add(x, b);
                let reconstructed = integral_constant_jerk_interval(x, a, b, b_at_x);
                assert_close(reconstructed, time, 2.0e-13);
            }
        }
        Ok(())
    }

    #[test]
    fn stationary_head_and_tail_convention_is_unchanged() -> Result<(), CoppError> {
        let s = [0.0, 1.0, 2.0, 3.0];
        let a = [0.0, 1.0, 1.0, 0.0];
        let b = [0.0, 0.0, 0.0, 0.0];
        let (t_final, t_s) = s_to_t_topp3(&s, (&a, &b, (1, 1)), 0.0)?;
        assert_eq!(t_final, 7.0);
        assert_eq!(t_s, [0.0, 3.0, 4.0, 7.0]);

        let query = [0.0, 3.0, 3.5, 4.0, 7.0];
        let sampled = t_to_s_topp3(
            &s,
            (&a, &b, (1, 1)),
            &t_s,
            InterpolationMode::NonUniformTimeGrid(&query),
        )?;
        assert_eq!(sampled, [0.0, 1.0, 1.5, 2.0, 3.0]);
        Ok(())
    }
}
