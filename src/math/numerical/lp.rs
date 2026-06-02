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

#[cfg(test)]
use clarabel::algebra::*;
#[cfg(test)]
use clarabel::solver::*;

/// Safety multiplier used when propagating tolerance across dimension reductions.
const EPS_SCALE: f64 = 10.0;
/// Near-zero threshold for feasibility/normalization branch decisions.
pub(crate) const EPS_ZERO: f64 = 1e-9;
/// Lower bound for coefficient normalization denominators.
const EPS_NORMALIZE: f64 = 1e-3;
/// Default box bound used to stabilize unbounded LP directions.
pub(crate) const LP_BOUND: f64 = 1e6;

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

/// Linear programming in 2D plane of extreme direction.
/// max w.0*X+w.1*Y, s.t. Ab[i].0*X+Ab[i].1*Y<=Ab[i].2
/// Given a x_opt=(x_opt,y_opt), determine the tangent direction v1, v2 on the 2D plane (x, y).
/// The cone defined by the **anticlockwise** transition from $v_1$ to $v_2$ contains **feasible** directions.
#[cfg(test)]
pub(crate) fn lp_2d_extreme_direction(
    a_b: &[(f64, f64, f64)],
    x_opt: (f64, f64),
    w: (f64, f64),
    tol: &LpToleranceOptions,
) -> ((f64, f64), (f64, f64)) {
    let epsilon = tol.feas_tol;
    let a_b_act = a_b
        .iter()
        .filter_map(|&(a, b, c)| {
            let norm = (a * a + b * b).sqrt();
            if norm > EPS_ZERO && (a * x_opt.0 + b * x_opt.1 - c).abs() < epsilon * norm {
                Some((a / norm, b / norm))
            } else {
                None
            }
        })
        .collect::<Vec<(f64, f64)>>();
    if a_b_act.len() < 2 {
        return ((-w.1, w.0), (w.1, -w.0));
    }
    let w_norm = (w.0 * w.0 + w.1 * w.1).sqrt();
    let v0 = (-w.1 / w_norm, w.0 / w_norm); // 90 degree anticlockwise rotation
    let mut cos_max: f64 = 1.0;
    let mut cos_min: f64 = -1.0;
    let mut v_max = v0;
    let mut v_min = (-v0.0, -v0.1);
    let mut func = |v: (f64, f64), upper_bound: bool| {
        if cross_product_2d(v0, v) > 0.0 {
            // anticlockwise
            let cos_now = (v.0 * v0.0 + v.1 * v0.1) / (v.0 * v.0 + v.1 * v.1).sqrt();
            if upper_bound {
                if cos_now < cos_max {
                    cos_max = cos_now;
                    v_max = v;
                }
            } else if cos_now > cos_min {
                cos_min = cos_now;
                v_min = v;
            }
        }
    };
    for &(a1, b1) in a_b_act.iter() {
        func((-b1, a1), true);
        func((b1, -a1), false);
    }
    let vmax_norm = (v_max.0 * v_max.0 + v_max.1 * v_max.1).sqrt();
    let vmin_norm = (v_min.0 * v_min.0 + v_min.1 * v_min.1).sqrt();

    (
        (v_max.0 / vmax_norm, v_max.1 / vmax_norm),
        (v_min.0 / vmin_norm, v_min.1 / vmin_norm),
    )
}

/// Linear programming in 2D plane based on geometric method.
/// max J:=w_x*X+w_y*Y, s.t. Ab[i].0*X+Ab[i].1*Y<=Ab[i].2
/// return (X,Y,J)
#[cfg(test)]
pub(crate) fn lp_2d_incre<W: WarmStartLp2d, const NORMALIZE: bool>(
    a_b: &[(f64, f64, f64)],
    w: (f64, f64),
    warm_start: &W,
    tol: &LpToleranceOptions,
    buffer: &mut Vec<(f64, f64, f64)>,
) -> (f64, f64, f64) {
    let (w_x, w_y) = w;
    if w_x.abs() < EPS_ZERO && w_y.abs() < EPS_ZERO {
        return (f64::NAN, f64::NAN, 0.0);
    }
    let w = (w_x * w_x + w_y * w_y).sqrt();
    let w_inv = 1.0 / w;
    let w_x = w_x * w_inv;
    let w_y = w_y * w_inv;

    buffer.clear();
    buffer.extend(
        a_b.iter()
            .map(|&(ax, ay, b)| (ax * w_y - ay * w_x, ax * w_x + ay * w_y, b)),
    );
    let (x, y) = lp_2d_incre_max_y::<_, NORMALIZE>(buffer, &warm_start.rotate((w_x, w_y)), tol);

    (w_y * x + w_x * y, w_y * y - w_x * x, w * y)
}

#[inline(always)]
/// Normalize 2D half-space rows by normal-vector magnitude.
///
/// Degenerate rows are mapped to zeros.
pub(crate) fn normalize_lp2d(a_b: &mut [(f64, f64, f64)]) {
    for w in a_b.iter_mut() {
        let norm = (w.0 * w.0 + w.1 * w.1).sqrt();
        *w = if norm < EPS_ZERO {
            (0.0, 0.0, 0.0)
        } else {
            let norm_inv = 1.0 / norm.max(EPS_NORMALIZE);
            (w.0 * norm_inv, w.1 * norm_inv, w.2 * norm_inv)
        };
    }
}

/// Linear programming in 2D plane based on incremental method (only maximize Y).
/// max Y, s.t. Ab[i].0*X+Ab[i].1*Y<=Ab[i].2
/// return (X,Y)
#[inline(always)]
fn lp_2d_incre_max_y_core<C: Lp2dIncCollector, W: WarmStartLp2d, const NORMALIZE: bool>(
    a_b: &mut [(f64, f64, f64)],
    warm_start: &W,
    tol: &LpToleranceOptions,
    mut collector: C,
) -> (f64, f64) {
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
    let tol_1d = LpToleranceOptions::with_feas_tol(epsilon * tol.reduce_dim_scale);
    for (i, &(a, b, c)) in warm_start.iter_skip(a_b.iter().enumerate()) {
        // a*x + b*y <= c
        if a * x + b * y > c {
            // Infeasible for this constraint
            // Let a*x + b*y == c
            // Apply 1-dim LP
            if a.abs() < EPS_ZERO && b.abs() < EPS_ZERO {
                // 0 <= c
                if c < -epsilon {
                    collector.clear();
                    return (f64::NAN, f64::NAN); // Infeasible
                }
            } else if a.abs() > b.abs() {
                // a*x + b*y == c
                // x == c/a - (b/a)*y == p*y - q
                let a_inv = 1.0 / a;
                let p = -b * a_inv;
                let q = -c * a_inv;
                // a_*x + b_*y <= c_
                // a_ * (p*y - q) + b_*y <= c_
                // (a_*p + b_)*y <= c_ + a_*q
                let (ymax, _) = lp_1d_core::<_, true>(
                    a_b.iter()
                        .take(i)
                        .map(|&(a_, b_, c_)| (a_ * p + b_, c_ + a_ * q)),
                    &tol_1d,
                    &mut collector,
                );
                if ymax.is_nan() {
                    collector.clear();
                    return (f64::NAN, f64::NAN); // Infeasible
                }
                y = ymax.min(y);
                x = p * y - q;
                collector.collect_2d_id1(i);
            } else {
                // a*x + b*y == c
                // y == c/b - (a/b)*x == p*x - q
                let b_inv = 1.0 / b;
                let p = -a * b_inv;
                let q = -c * b_inv;
                // a_*x + b_*y <= c_
                // a_ * x + b_ * (p*x - q) <= c_
                // (a_ + b_*p)*x <= c_ + b_*q
                let (mut xmax, mut xmin) = lp_1d_core::<_, false>(
                    a_b.iter()
                        .take(i)
                        .map(|(a_, b_, c_)| (a_ + b_ * p, c_ + b_ * q)),
                    &tol_1d,
                    &mut collector,
                );
                if xmax < xmin {
                    if p.abs() * (xmin - xmax) > epsilon {
                        collector.clear();
                        collector.collect_2d_id0(i);
                        return (f64::NAN, f64::NAN); // Infeasible
                    } else {
                        xmax = 0.5 * (xmax + xmin);
                        xmin = xmax;
                    }
                }
                x = if p > 0.0 {
                    // if debug {
                    //     println!("\t(LP-2D)7: xmax={}, xmin={}", xmax, xmin);
                    // }
                    collector.collect_2d_id1(i);
                    if xmax.is_finite() {
                        xmax
                    } else {
                        xmin.max(LP_BOUND)
                    }
                } else if p < 0.0 {
                    collector.collect_2d_id0(i);
                    if xmin.is_finite() {
                        xmin
                    } else {
                        xmax.min(-LP_BOUND)
                    }
                } else if xmin > 0.0 {
                    // if debug {
                    //     println!("\t(LP-2D)9: xmax={}, xmin={}", xmax, xmin);
                    // }
                    collector.collect_2d_id0(i);
                    xmin
                } else if xmax < 0.0 {
                    // if debug {
                    //     println!("\t(LP-2D)10: xmax={}, xmin={}", xmax, xmin);
                    // }
                    collector.collect_2d_id1(i);
                    xmax
                } else {
                    // if debug {
                    //     println!("\t(LP-2D)11: xmax={}, xmin={}", xmax, xmin);
                    // }
                    collector.clear();
                    collector.collect_2d_id0(i);
                    0.0
                };
                y = p * x - q;
                // println!("222: x={}, y={}", x, y);
            }
        }
        // if debug {
        //     println!("\t(LP-2D) x_curr={}, y_curr={}", x, y);
        //     // println!(
        //     //     "\t(LP-2D) delta={:?}",
        //     //     a_b.clone()
        //     //         .take(i + 1)
        //     //         .map(|(a_, b_, c_)| -(a_ * x + b_ * y - c_))
        //     //         .collect::<Vec<f64>>()
        //     // );
        // }
    }

    // y = if (y / BOUND - 1.0).abs() < epsilon {
    //     f64::INFINITY
    // } else {
    //     y
    // };
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
    lp_2d_incre_max_y_core::<_, _, NORMALIZE>(a_b, warm_start, tol, SilentLpIncCollector)
}

/// Linear programming in 1D line.
/// a_b[i].0 * x <= a_b[i].1
/// Return (xmax, xmin) if feasible; otherwise (NaN, NaN).
#[inline(always)]
fn lp_1d_core<C: Lp1dIncCollector, const AUTONAN: bool>(
    a_b: impl Iterator<Item = (f64, f64)>,
    tol: &LpToleranceOptions,
    mut collector: C,
) -> (f64, f64) {
    let epsilon = tol.feas_tol;
    // Find the maximum y such that a*x <= b
    let mut xmax = f64::INFINITY;
    let mut xmin = -f64::INFINITY;

    for (k, (a, b)) in C::enumerate(a_b) {
        // println!("\t\t(LP-1D) a={}, b={}", a, b);
        if a.abs() < b.abs().clamp(EPS_NORMALIZE, 1.0) * EPS_ZERO {
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
        if xmin <= xmax + epsilon {
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
    lp_1d_core::<_, AUTONAN>(a_b, tol, SilentLpIncCollector)
}

/// Warm-start interface for 2D incremental LP.
pub(crate) trait WarmStartLp2d {
    /// Initial feasible guess.
    fn get_initial_point(&self) -> (f64, f64);
    /// Iterate constraints while skipping known-prefix constraints if desired.
    fn iter_skip<I>(&self, a_b: I) -> impl Iterator<Item = I::Item>
    where
        I: Iterator;
    /// Rotate warm-start state under objective-space rotation.
    #[cfg(test)]
    fn rotate(&self, w: (f64, f64)) -> impl WarmStartLp2d;
}

/// 2D warm-start policy: always start from default point without skipping.
#[cfg(test)]
pub(crate) struct Lp2dNoWarmStart;
#[cfg(test)]
impl WarmStartLp2d for Lp2dNoWarmStart {
    #[inline(always)]
    fn get_initial_point(&self) -> (f64, f64) {
        (0.0, LP_BOUND)
    }
    #[inline(always)]
    fn iter_skip<I>(&self, a_b: I) -> impl Iterator<Item = I::Item>
    where
        I: Iterator,
    {
        a_b
    }
    #[allow(refining_impl_trait)]
    #[cfg(test)]
    #[inline(always)]
    fn rotate(&self, _w: (f64, f64)) -> Lp2dNoWarmStart {
        Lp2dNoWarmStart
    }
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
    #[allow(refining_impl_trait)]
    #[cfg(test)]
    #[inline(always)]
    fn rotate(&self, w: (f64, f64)) -> Lp2dWarmStart {
        Lp2dWarmStart {
            x0: (
                w.1 * self.x0.0 - w.0 * self.x0.1,
                w.0 * self.x0.0 + w.1 * self.x0.1,
            ),
            skip: self.skip,
        }
    }
}

/// Lightweight key adapter for LP collector implementations.
trait Lp1dKey: Copy {
    fn get_key(&self) -> usize;
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

/// Linear programming in 2D plane based on geometric method.
/// max J:=w_x*X+w_y*Y, s.t. Ab[i].0*X+Ab[i].1*Y<=Ab[i].2
/// w=(w_x, w_y)
/// return (X,Y,J)
#[cfg(test)]
fn lp_2d_geo(
    a_b: impl Iterator<Item = (f64, f64, f64)>,
    len: usize,
    w: (f64, f64),
    tol: &LpToleranceOptions,
) -> (f64, f64, f64) {
    let epsilon = tol.feas_tol;
    let (w_x, w_y) = w;
    if w_x.abs() < epsilon && w_y.abs() < epsilon {
        return (f64::NAN, f64::NAN, 0.0);
    }
    let w = (w_x * w_x + w_y * w_y).sqrt();
    let w_x = w_x / w;
    let w_y = w_y / w;
    let (x, y) = lp_2d_geo_max_y(
        a_b.map(|(ax, ay, b)| (ax * w_y - ay * w_x, ax * w_x + ay * w_y, b)),
        len,
        tol,
    );

    (w_y * x + w_x * y, w_y * y - w_x * x, w * y)
}

/// Linear programming in 2D plane based on geometric method (only maximize Y).
/// max Y, s.t. Ab[i].0*X+Ab[i].1*Y<=Ab[i].2
#[cfg(test)]
fn lp_2d_geo_max_y(
    mut a_b: impl Iterator<Item = (f64, f64, f64)>,
    len: usize,
    tol: &LpToleranceOptions,
) -> (f64, f64) {
    let epsilon = tol.feas_tol;
    if len <= 1 {
        if len == 1 {
            let a_b_0 = a_b.next().unwrap();
            if a_b_0.0.abs() < epsilon && a_b_0.1 > epsilon {
                // Unbounded
                return (f64::NAN, a_b_0.2 / a_b_0.1);
            }
        }
        // Infeasible
        return (f64::NAN, f64::NAN);
    }
    // Step 1. Initialize
    let mut ymax = f64::INFINITY;
    let mut ymin = -f64::INFINITY;
    // max Y, s.t. Ab[i].x*X+Ab[i].y*Y<=Ab[i].z
    let mut kb_plus = Vec::<(f64, f64)>::with_capacity(len); // X <= k_plus[i] * Y + b_plus[i]
    let mut kb_minus = Vec::<(f64, f64)>::with_capacity(len); // X >= k_minus[i] * Y + b_minus[i]
    for (x, y, z) in a_b {
        if x.abs() < epsilon {
            if y.abs() < epsilon {
                if z < -epsilon {
                    // Infeasible
                    return (f64::NAN, f64::NAN);
                }
            } else if y > epsilon {
                ymax = ymax.min(z / y);
            } else {
                ymin = ymin.max(z / y);
            }
        } else if x > epsilon {
            kb_plus.push((-y / x, z / x));
        } else {
            kb_minus.push((-y / x, z / x));
        }
    }

    if ymin >= ymax {
        // Infeasible
        return (f64::NAN, f64::NAN);
    }
    if kb_plus.is_empty() && kb_minus.is_empty() {
        // Unbounded
        return (f64::NAN, ymax);
    }
    if kb_plus.is_empty() {
        // Only kb_minus
        let x = kb_minus
            .iter()
            .map(|&(k, b)| k * ymax + b)
            .min_by(|a, b| a.total_cmp(b))
            .unwrap();
        return (x, ymax);
    }
    if kb_minus.is_empty() {
        // Only kb_plus
        let x = kb_plus
            .iter()
            .map(|&(k, b)| k * ymax + b)
            .max_by(|a, b| a.total_cmp(b))
            .unwrap();
        return (x, ymax);
    }

    // Step 2. Get intersections, both right to left
    let (intersect_plus, (k0_plus, b0_plus), (kf_plus, bf_plus)) = if kb_plus.len() >= 2 {
        piecewise_linear_2d(&kb_plus, true, tol)
    } else {
        (
            Vec::new(),
            *kb_plus.first().unwrap(),
            *kb_plus.last().unwrap(),
        )
    };
    let (intersect_minus, (k0_minus, b0_minus), (kf_minus, bf_minus)) = if kb_minus.len() >= 2 {
        piecewise_linear_2d(&kb_minus, false, tol)
    } else {
        (
            Vec::new(),
            *kb_minus.first().unwrap(),
            *kb_minus.last().unwrap(),
        )
    };

    // Step 3. Find the optimal solution
    if ymax.is_infinite() && k0_plus >= k0_minus - epsilon {
        // Unbounded
        return (f64::NAN, ymax);
    }
    let mut flag_ymax = true;
    if ymax.is_infinite() {
        ymax = (b0_minus - b0_plus) / (k0_plus - k0_minus);
        flag_ymax = false;
    }

    let mut xmax_plus = k0_plus * ymax + b0_plus;
    let mut xmax_minus = k0_minus * ymax + b0_minus;
    if xmax_plus >= xmax_minus - epsilon
        && intersect_plus.last().is_none_or(|p| ymax >= p.0)
        && intersect_minus.last().is_none_or(|p| ymax >= p.0)
    {
        return if ymax >= ymin {
            (0.5 * (xmax_plus + xmax_minus), ymax)
        } else {
            (f64::INFINITY, f64::INFINITY)
        };
    }

    let mut ymax_plus = ymax;
    let mut ymax_minus = ymax;
    let mut it_plus = intersect_plus.iter().rev().peekable();
    let mut it_minus = intersect_minus.iter().rev().peekable();
    let mut p_next_minus = (f64::INFINITY, f64::INFINITY);
    let mut p_next_plus = (f64::INFINITY, f64::INFINITY);

    loop {
        // !(it_plus.peek().is_none() && it_minus.peek().is_none())
        // Consider (YMAX_plus, XMAX_plus)--it_plus;
        // (YMAX_minus, XMAX_minus)--it_minus];
        let flag_empty = it_plus.peek().is_none() && it_minus.peek().is_none();
        while let Some(&p_curr) = it_plus.peek()
            && p_curr.0 >= ymax_plus
        {
            p_next_plus = *p_curr;
            it_plus.next();
        }
        while let Some(&p_curr) = it_minus.peek()
            && p_curr.0 >= ymax_minus
        {
            p_next_minus = *p_curr;
            it_minus.next();
        }

        if flag_ymax {
            flag_ymax = false;
            let (k_plus_, b_plus_) = if let Some(&p_curr) = it_plus.peek() {
                let k = if p_next_plus.0.is_infinite() {
                    k0_plus
                } else {
                    (p_curr.1 - p_next_plus.1) / (p_curr.0 - p_next_plus.0)
                };
                (k, p_curr.1 - k * p_curr.0)
            } else {
                (kf_plus, bf_plus)
            };
            let (k_minus_, b_minus_) = if let Some(&p_curr) = it_minus.peek() {
                let k = if p_next_minus.0.is_infinite() {
                    k0_minus
                } else {
                    (p_curr.1 - p_next_minus.1) / (p_curr.0 - p_next_minus.0)
                };
                (k, p_curr.1 - k * p_curr.0)
            } else {
                (kf_minus, bf_minus)
            };
            let xmax_plus_ = k_plus_ * ymax + b_plus_;
            let xmax_minus_ = k_minus_ * ymax + b_minus_;
            if xmax_plus_ >= xmax_minus_ - epsilon {
                return if ymax >= ymin {
                    (0.5 * (xmax_plus_ + xmax_minus_), ymax)
                } else {
                    (f64::INFINITY, f64::INFINITY)
                };
            }
        }
        if !p_next_plus.0.is_infinite() {
            ymax_plus = p_next_plus.0;
            xmax_plus = p_next_plus.1;
        }
        if !p_next_minus.0.is_infinite() {
            ymax_minus = p_next_minus.0;
            xmax_minus = p_next_minus.1;
        }
        let (k_plus, y_min_plus) = if let Some(&p_curr) = it_plus.peek() {
            ((p_curr.1 - xmax_plus) / (p_curr.0 - ymax_plus), p_curr.0)
        } else {
            (kf_plus, -f64::INFINITY)
        };
        let (k_minus, y_min_minus) = if let Some(&p_curr) = it_minus.peek() {
            ((p_curr.1 - xmax_minus) / (p_curr.0 - ymax_minus), p_curr.0)
        } else {
            (kf_minus, -f64::INFINITY)
        };

        if (k_plus - k_minus).abs() > epsilon {
            let b_plus = if let Some(&p_curr) = it_plus.peek() {
                p_curr.1 - k_plus * p_curr.0
            } else {
                bf_plus
            };
            let b_minus = if let Some(&p_curr) = it_minus.peek() {
                p_curr.1 - k_minus * p_curr.0
            } else {
                bf_minus
            };
            let (y, x) = solve_2x2(((-k_plus, 1.0), (-k_minus, 1.0)), (b_plus, b_minus)).unwrap();
            if y >= y_min_plus.max(y_min_minus) && y <= ymax {
                // Find the intersection
                return if y >= ymin {
                    (x, y)
                } else {
                    (f64::INFINITY, f64::INFINITY)
                };
            }
        }
        if y_min_plus < y_min_minus {
            if let Some(&p_curr) = it_minus.peek() {
                ymax_minus = y_min_minus;
                xmax_minus = p_curr.0;
                p_next_minus = *p_curr;
                it_minus.next();
            }
        } else if let Some(&p_curr) = it_plus.peek() {
            ymax_plus = y_min_plus;
            xmax_plus = p_curr.0;
            p_next_plus = *p_curr;
            it_plus.next();
        }
        if flag_empty {
            break;
        }
    }

    (f64::INFINITY, f64::INFINITY)
}

/// Workspace buffers for geometric 2D contour extraction.
///
/// Stores positive/negative line-intersection sets used by
/// `lp_2d_geo_contour_bounded`.
#[cfg(test)]
type Lp2dGeoContourWorkspace<'a> = (
    &'a mut Vec<(f64, f64)>, // intersect_plus, with_capacity(a_b.len())
    &'a mut Vec<(f64, f64)>, // intersect_minus, with_capacity(a_b.len())
);

/// Contour of linear set in 2D plane based on geometric method.
/// Ab[i].0*X+Ab[i].1*Y<=Ab[i].2
/// If infeasible or unbounded, return empty vec.
/// (x, y) is anticlockwise.
#[cfg(test)]
pub(crate) fn lp_2d_geo_contour_bounded<const CLOCKWISE: bool>(
    a_b: &[(f64, f64, f64)],
    tol: &LpToleranceOptions,
    contour: &mut Vec<(f64, f64)>,
    buffer: Lp2dGeoContourWorkspace,
) {
    let epsilon = tol.feas_tol;
    contour.clear();
    if a_b.len() < 3 {
        return;
    }
    // let debug = a_b.len() == 24
    //     && (a_b[0].2 - 995.8861908753399).abs() < 1E-8
    //     && (a_b[7].2 - 29.51611830471187).abs() < 1E-8;
    // if debug {
    //     println!("Debug in lp_2d_geo_contour_bounded");
    //     println!("a_b={:?}", a_b);
    // }

    // Step 1. Initialize
    let mut ymax = f64::INFINITY;
    let mut ymin = -f64::INFINITY;
    // max Y, s.t. Ab[i].x*X+Ab[i].y*Y<=Ab[i].z
    let (kb_plus, kb_minus) = buffer;
    kb_plus.clear(); // X <= k_plus[i] * Y + b_plus[i]
    kb_minus.clear(); // X >= k_minus[i] * Y + b_minus[i]
    for &(x, y, z) in a_b.iter() {
        if x.abs() < epsilon {
            if y.abs() < epsilon {
                if z < -epsilon {
                    // Infeasible
                    return;
                }
            } else if y > epsilon {
                ymax = ymax.min(z / y);
            } else {
                ymin = ymin.max(z / y);
            }
        } else {
            let x_inv = 1.0 / x;
            if x > epsilon {
                kb_plus.push((-y * x_inv, z * x_inv));
            } else {
                kb_minus.push((-y / x, z / x));
            }
        }
    }

    if ymin >= ymax || kb_plus.is_empty() || kb_minus.is_empty() {
        // Infeasible or Unbounded
        return;
    }

    // if debug {
    //     println!("ymin={}, ymax={}", ymin, ymax);
    //     println!("kb_plus={:?}", kb_plus);
    //     println!("kb_minus={:?}", kb_minus);
    // }

    // Step 2. Get intersections, both right to left
    let (mut intersect_plus, (mut k0_plus, mut b0_plus), (kf_plus, bf_plus)) = if kb_plus.len() >= 2
    {
        piecewise_linear_2d(kb_plus, true, tol)
    } else {
        (
            Vec::new(),
            *kb_plus.first().unwrap(),
            *kb_plus.last().unwrap(),
        )
    };
    let (mut intersect_minus, (mut k0_minus, mut b0_minus), (kf_minus, bf_minus)) =
        if kb_minus.len() >= 2 {
            piecewise_linear_2d(kb_minus, false, tol)
        } else {
            (
                Vec::new(),
                *kb_minus.first().unwrap(),
                *kb_minus.last().unwrap(),
            )
        };

    // if debug {
    //     println!("intersect_plus={:?}", intersect_plus);
    //     println!("intersect_minus={:?}", intersect_minus);
    // }

    // Step 3. Find the optimal solution
    if (ymax.is_infinite() && k0_plus >= k0_minus - epsilon)
        || (ymin.is_infinite() && kf_plus <= kf_minus - epsilon)
    {
        // Unbounded
        return;
    }
    // Step 3.1 Max Y
    let mut flag_ymax_never_initial = true;
    if ymax.is_infinite() {
        ymax = (b0_minus - b0_plus) / (k0_plus - k0_minus);
        flag_ymax_never_initial = false;
    }

    let mut xmax_plus = k0_plus * ymax + b0_plus;
    let mut xmax_minus = k0_minus * ymax + b0_minus;
    let mut flag_ymax_succeed = xmax_plus >= xmax_minus - epsilon
        && intersect_plus.last().is_none_or(|p| ymax >= p.0)
        && intersect_minus.last().is_none_or(|p| ymax >= p.0);
    if flag_ymax_succeed {
        if ymax < ymin {
            return;
        }
    } else {
        let mut ymax_plus = ymax;
        let mut ymax_minus = ymax;
        let mut it_plus = intersect_plus.iter().rev().peekable();
        let mut it_minus = intersect_minus.iter().rev().peekable();
        let mut p_next_minus = (f64::INFINITY, f64::INFINITY);
        let mut p_next_plus = (f64::INFINITY, f64::INFINITY);

        while !flag_ymax_succeed {
            // !(it_plus.peek().is_none() && it_minus.peek().is_none())
            // Consider (YMAX_plus, XMAX_plus)--it_plus;
            //          (YMAX_minus, XMAX_minus)--it_minus];
            let flag_empty = it_plus.peek().is_none() && it_minus.peek().is_none();

            while let Some(&p_curr) = it_plus.peek()
                && p_curr.0 >= ymax_plus
            {
                p_next_plus = *p_curr;
                it_plus.next();
            }
            while let Some(&p_curr) = it_minus.peek()
                && p_curr.0 >= ymax_minus
            {
                p_next_minus = *p_curr;
                it_minus.next();
            }

            if flag_ymax_never_initial {
                flag_ymax_never_initial = false;
                let (k_plus_, b_plus_) = if let Some(&p_curr) = it_plus.peek() {
                    let k = if p_next_plus.0.is_infinite() {
                        k0_plus
                    } else {
                        (p_curr.1 - p_next_plus.1) / (p_curr.0 - p_next_plus.0)
                    };
                    (k, p_curr.1 - k * p_curr.0)
                } else {
                    (kf_plus, bf_plus)
                };
                let (k_minus_, b_minus_) = if let Some(&p_curr) = it_minus.peek() {
                    let k = if p_next_minus.0.is_infinite() {
                        k0_minus
                    } else {
                        (p_curr.1 - p_next_minus.1) / (p_curr.0 - p_next_minus.0)
                    };
                    (k, p_curr.1 - k * p_curr.0)
                } else {
                    (kf_minus, bf_minus)
                };
                let xmax_plus_ = k_plus_ * ymax + b_plus_;
                let xmax_minus_ = k_minus_ * ymax + b_minus_;
                if xmax_plus_ >= xmax_minus_ - epsilon {
                    if ymax < ymin {
                        return;
                    }
                    flag_ymax_succeed = true;
                    xmax_plus = xmax_plus_;
                    xmax_minus = xmax_minus_;
                    k0_plus = k_plus_;
                    b0_plus = b_plus_;
                    k0_minus = k_minus_;
                    b0_minus = b_minus_;
                    break;
                }
            }
            if !p_next_plus.0.is_infinite() {
                ymax_plus = p_next_plus.0;
                xmax_plus = p_next_plus.1;
            }
            if !p_next_minus.0.is_infinite() {
                ymax_minus = p_next_minus.0;
                xmax_minus = p_next_minus.1;
            }
            let (k_plus, y_min_plus) = if let Some(&p_curr) = it_plus.peek() {
                ((p_curr.1 - xmax_plus) / (p_curr.0 - ymax_plus), p_curr.0)
            } else {
                (kf_plus, -f64::INFINITY)
            };
            let (k_minus, y_min_minus) = if let Some(&p_curr) = it_minus.peek() {
                ((p_curr.1 - xmax_minus) / (p_curr.0 - ymax_minus), p_curr.0)
            } else {
                (kf_minus, -f64::INFINITY)
            };

            if (k_plus - k_minus).abs() > epsilon {
                let b_plus = if let Some(&p_curr) = it_plus.peek() {
                    p_curr.1 - k_plus * p_curr.0
                } else {
                    bf_plus
                };
                let b_minus = if let Some(&p_curr) = it_minus.peek() {
                    p_curr.1 - k_minus * p_curr.0
                } else {
                    bf_minus
                };
                let (y, x) =
                    solve_2x2(((-k_plus, 1.0), (-k_minus, 1.0)), (b_plus, b_minus)).unwrap();
                if y >= y_min_plus.max(y_min_minus) && y <= ymax {
                    // Find the intersection
                    if y < ymin {
                        return;
                    }
                    flag_ymax_succeed = true;
                    xmax_plus = x;
                    xmax_minus = x;
                    ymax = y;
                    k0_plus = k_plus;
                    b0_plus = b_plus;
                    k0_minus = k_minus;
                    b0_minus = b_minus;
                    break;
                }
            }
            if y_min_plus < y_min_minus {
                if let Some(&p_curr) = it_minus.peek() {
                    ymax_minus = y_min_minus;
                    xmax_minus = p_curr.0;
                    p_next_minus = *p_curr;
                    it_minus.next();
                }
            } else if let Some(&p_curr) = it_plus.peek() {
                ymax_plus = y_min_plus;
                xmax_plus = p_curr.0;
                p_next_plus = *p_curr;
                it_plus.next();
            }
            if flag_empty {
                break;
            }
        }
    }

    // if debug {
    //     println!(
    //         "ymax={}, xmax_plus={}, xmax_minus={}",
    //         ymax, xmax_plus, xmax_minus
    //     );
    // }

    if flag_ymax_succeed {
        while let Some(p_last) = intersect_plus.last()
            && p_last.0 > ymax
        {
            intersect_plus.pop();
        }
        intersect_plus.push((ymax, xmax_plus));
        while let Some(p_last) = intersect_minus.last()
            && p_last.0 > ymax
        {
            intersect_minus.pop();
        }
        intersect_minus.push((ymax, xmax_minus));
    } else {
        return;
    }

    // Step 3.2 Min Y
    let mut flag_ymin_never_initial = true;
    if ymin.is_infinite() {
        ymin = (bf_minus - bf_plus) / (kf_plus - kf_minus);
        flag_ymin_never_initial = false;
    }

    let mut xmin_plus = kf_plus * ymin + bf_plus;
    let mut xmin_minus = kf_minus * ymin + bf_minus;

    let mut flag_ymin_succeed = xmin_plus >= xmin_minus - epsilon
        && intersect_plus.first().is_none_or(|p| ymin <= p.0)
        && intersect_minus.first().is_none_or(|p| ymin <= p.0);

    if flag_ymin_succeed {
        if ymax < ymin {
            return;
        }
    } else {
        let mut ymin_plus = ymin;
        let mut ymin_minus = ymin;
        let mut it_plus = intersect_plus.iter().peekable();
        let mut it_minus = intersect_minus.iter().peekable();
        let mut p_prev_minus = (f64::INFINITY, f64::INFINITY);
        let mut p_prev_plus = (f64::INFINITY, f64::INFINITY);

        while !flag_ymin_succeed {
            // !(it_plus.peek().is_none() && it_minus.peek().is_none())
            // Consider (YMAX_plus, XMAX_plus)--it_plus;
            //          (YMAX_minus, XMAX_minus)--it_minus];
            let flag_empty = it_plus.peek().is_none() && it_minus.peek().is_none();

            while let Some(&p_curr) = it_plus.peek()
                && p_curr.0 <= ymin_plus
            {
                p_prev_plus = *p_curr;
                it_plus.next();
            }
            while let Some(&p_curr) = it_minus.peek()
                && p_curr.0 <= ymin_minus
            {
                p_prev_minus = *p_curr;
                it_minus.next();
            }

            if flag_ymin_never_initial {
                flag_ymin_never_initial = false;
                let (k_plus_, b_plus_) = if let Some(&p_curr) = it_plus.peek() {
                    let k = if p_prev_plus.0.is_infinite() {
                        kf_plus
                    } else {
                        (p_curr.1 - p_prev_plus.1) / (p_curr.0 - p_prev_plus.0)
                    };
                    (k, p_curr.1 - k * p_curr.0)
                } else {
                    (k0_plus, b0_plus)
                };
                let (k_minus_, b_minus_) = if let Some(&p_curr) = it_minus.peek() {
                    let k = if p_prev_minus.0.is_infinite() {
                        kf_minus
                    } else {
                        (p_curr.1 - p_prev_minus.1) / (p_curr.0 - p_prev_minus.0)
                    };
                    (k, p_curr.1 - k * p_curr.0)
                } else {
                    (k0_minus, b0_minus)
                };
                let xmin_plus_ = k_plus_ * ymin + b_plus_;
                let xmin_minus_ = k_minus_ * ymin + b_minus_;
                if xmin_plus_ >= xmin_minus_ - epsilon {
                    if ymax < ymin {
                        return;
                    }
                    flag_ymin_succeed = true;
                    xmin_plus = xmin_plus_;
                    xmin_minus = xmin_minus_;
                    // kf_plus = k_plus_;
                    // bf_plus = b_plus_;
                    // kf_minus = k_minus_;
                    // bf_minus = b_minus_;
                    break;
                }
            }
            if !p_prev_plus.0.is_infinite() {
                ymin_plus = p_prev_plus.0;
                xmin_plus = p_prev_plus.1;
            }
            if !p_prev_minus.0.is_infinite() {
                ymin_minus = p_prev_minus.0;
                xmin_minus = p_prev_minus.1;
            }
            let (k_plus, y_max_plus) = if let Some(&p_curr) = it_plus.peek() {
                ((p_curr.1 - xmin_plus) / (p_curr.0 - ymin_plus), p_curr.0)
            } else {
                (k0_plus, f64::INFINITY)
            };
            let (k_minus, y_max_minus) = if let Some(&p_curr) = it_minus.peek() {
                ((p_curr.1 - xmin_minus) / (p_curr.0 - ymin_minus), p_curr.0)
            } else {
                (k0_minus, f64::INFINITY)
            };

            if (k_plus - k_minus).abs() > epsilon {
                let b_plus = if let Some(&p_curr) = it_plus.peek() {
                    p_curr.1 - k_plus * p_curr.0
                } else {
                    b0_plus
                };
                let b_minus = if let Some(&p_curr) = it_minus.peek() {
                    p_curr.1 - k_minus * p_curr.0
                } else {
                    b0_minus
                };
                let (y, x) =
                    solve_2x2(((-k_plus, 1.0), (-k_minus, 1.0)), (b_plus, b_minus)).unwrap();
                if y <= y_max_plus.min(y_max_minus) && y >= ymin {
                    // Find the intersection
                    if y > ymax {
                        return;
                    }
                    flag_ymin_succeed = true;
                    xmin_plus = x;
                    xmin_minus = x;
                    ymin = y;
                    // kf_plus = k_plus;
                    // bf_plus = b_plus;
                    // kf_minus = k_minus;
                    // bf_minus = b_minus;
                    break;
                }
            }
            if y_max_plus > y_max_minus {
                if let Some(&p_curr) = it_minus.peek() {
                    ymin_minus = y_max_minus;
                    xmin_minus = p_curr.0;
                    p_prev_minus = *p_curr;
                    it_minus.next();
                }
            } else if let Some(&p_curr) = it_plus.peek() {
                ymin_plus = y_max_plus;
                xmin_plus = p_curr.0;
                p_prev_plus = *p_curr;
                it_plus.next();
            }
            if flag_empty {
                break;
            }
        }
    }

    if !flag_ymin_succeed {
        return;
    }

    contour.push((xmax_plus, ymax));
    if CLOCKWISE {
        while let Some(p_curr) = intersect_plus.pop()
            && p_curr.0 > ymin
        {
            if (contour.last().unwrap().0 - p_curr.1).abs() > epsilon
                || (contour.last().unwrap().1 - p_curr.0).abs() > epsilon
            {
                contour.push((p_curr.1, p_curr.0));
            }
        }
        if (contour.last().unwrap().0 - xmin_plus).abs() > epsilon
            || (contour.last().unwrap().1 - ymin).abs() > epsilon
        {
            contour.push((xmin_plus, ymin));
        }
        if (contour.last().unwrap().0 - xmin_minus).abs() > epsilon
            || (contour.last().unwrap().1 - ymin).abs() > epsilon
        {
            contour.push((xmin_minus, ymin));
        }
        for p_curr in intersect_minus.iter() {
            if p_curr.0 >= ymin
                && ((contour.last().unwrap().0 - p_curr.1).abs() > epsilon
                    || (contour.last().unwrap().1 - p_curr.0).abs() > epsilon)
            {
                contour.push((p_curr.1, p_curr.0));
            }
        }
    } else {
        while let Some(p_curr) = intersect_minus.pop()
            && p_curr.0 > ymin
        {
            if (contour.last().unwrap().0 - p_curr.1).abs() > epsilon
                || (contour.last().unwrap().1 - p_curr.0).abs() > epsilon
            {
                contour.push((p_curr.1, p_curr.0));
            }
        }
        if (contour.last().unwrap().0 - xmin_minus).abs() > epsilon
            || (contour.last().unwrap().1 - ymin).abs() > epsilon
        {
            contour.push((xmin_minus, ymin));
        }
        if (contour.last().unwrap().0 - xmin_plus).abs() > epsilon
            || (contour.last().unwrap().1 - ymin).abs() > epsilon
        {
            contour.push((xmin_plus, ymin));
        }
        for p_curr in intersect_plus.iter() {
            if p_curr.0 >= ymin
                && ((contour.last().unwrap().0 - p_curr.1).abs() > epsilon
                    || (contour.last().unwrap().1 - p_curr.0).abs() > epsilon)
            {
                contour.push((p_curr.1, p_curr.0));
            }
        }
    }

    if contour.len() >= 2
        && ((contour[0].0 - contour.last().unwrap().0).abs() < epsilon
            && (contour[0].1 - contour.last().unwrap().1).abs() < epsilon)
    {
        contour.pop();
    }
}

/// Find the piecewise linear representation to the given constraints.
/// # Arguments
/// `kb` - A vector of tuples representing the coefficients (k, b) of linear functions.
/// `maximize` - true if the piecewise linear representation should maximize the functions, false to minimize.
/// # Returns
/// A vector of tuples (x, y) representing the piecewise linear representation of the constraints.
/// The first tuple is the leftmost line's (k, b), and the last tuple is the rightmost line's (k, b).
/// For maximize==true, 0<=i<size, Y<=k[i]*X+b[i]
/// For maximize==false, 0<=i<size, Y>=k[i]*X+b[i]
#[cfg(test)]
#[allow(clippy::type_complexity)]
fn piecewise_linear_2d(
    kb: &[(f64, f64)],
    maximize: bool,
    tol: &LpToleranceOptions,
) -> (Vec<(f64, f64)>, (f64, f64), (f64, f64)) {
    let epsilon = tol.feas_tol;
    if kb.len() <= 1 {
        return (
            Vec::new(),
            *kb.first().unwrap_or(&(f64::NAN, f64::NAN)),
            *kb.last().unwrap_or(&(f64::NAN, f64::NAN)),
        );
    }
    // Reverse the sign of k and b if minimize
    let mut kb: Vec<(f64, f64)> = if maximize {
        kb.to_owned()
    } else {
        kb.iter().map(|(k, b)| (-k, -b)).collect()
    };
    // Now maximize the function
    // Step 1. sort and remove duplicates
    kb.sort_by(|a, b| a.0.total_cmp(&b.0)); // ascend: k[i] <= k[i+1]
    kb.dedup_by(|next, prev| {
        if (next.0 - prev.0).abs() <= epsilon {
            prev.1 = prev.1.min(next.1);
            true
        } else {
            false
        }
    });
    // Step 2. find the intersection points. from right to left
    let n = kb.len();
    let mut sol = Vec::<(f64, f64)>::with_capacity(n);
    let mut k_left = kb[n - 1].0;
    let k_right = k_left;

    for &(ki, bi) in kb.iter().take(n - 1).rev() {
        // Consider line (ki, bi) and sol.last()
        while let Some(&last_p) = sol.last()
            && last_p.0 * ki + bi <= last_p.1
        {
            sol.pop();
            k_left = if let Some(prev_p) = sol.last() {
                (prev_p.1 - last_p.1) / (prev_p.0 - last_p.0)
            } else {
                k_right
            }
        }
        // Consider line (ki, bi) and line (k_left, b_left)
        let b_left = if sol.is_empty() {
            kb.last().unwrap().1
        } else {
            sol.last().unwrap().1 - k_left * sol.last().unwrap().0
        };
        // Solve 2x2 equations: ki * x + bi = k_left * x + b_left
        if let Some(x) = solve_2x2(((-ki, 1.0), (-k_left, 1.0)), (bi, b_left)) {
            sol.push(x);
            k_left = ki;
        } else {
            // Lines are parallel, skip
            continue;
        }
    }

    if !maximize {
        sol.iter_mut().for_each(|p| {
            p.1 = -p.1;
        });
        kb.first_mut().unwrap().0 *= -1.0;
        kb.last_mut().unwrap().0 *= -1.0;
        kb.first_mut().unwrap().1 *= -1.0;
        kb.last_mut().unwrap().1 *= -1.0;
    }
    (sol, *kb.first().unwrap(), *kb.last().unwrap())
}

/// Solve 2*2 linear equations: a*x=b.
pub(crate) fn solve_2x2(a: ((f64, f64), (f64, f64)), b: (f64, f64)) -> Option<(f64, f64)> {
    let det = cross_product_2d((a.0.0, a.0.1), (a.1.0, a.1.1));
    if det.abs() < f64::EPSILON {
        return None; // Singular matrix
    }
    let det_inv = 1.0 / det;
    let x1 = cross_product_2d((b.0, a.0.1), (b.1, a.1.1)) * det_inv;
    let x2 = cross_product_2d((a.0.0, b.0), (a.1.0, b.1)) * det_inv;
    Some((x1, x2))
}

/// Linear programming in 2D plane, based on clarabel library.
/// max J:=w_x*X+w_y*Y, s.t. Ab[i].0*X+Ab[i].1*Y<=Ab[i].2
/// return (X,Y,J)
#[cfg(test)]
fn lp_2d_clarabel(
    a_b: impl Iterator<Item = (f64, f64, f64)> + Clone,
    len: usize,
    w: (f64, f64),
) -> (f64, f64, f64) {
    if len == 0 {
        return (f64::NAN, f64::NAN, f64::NAN);
    }

    let (w_x, w_y) = w;
    let p_csc = CscMatrix::zeros((2, 2));
    let q = vec![-w_x, -w_y];

    let mut colptr = vec![0];
    let mut rowval = Vec::with_capacity(len * 2);
    let mut nzval = Vec::with_capacity(len * 2);

    for (i, item) in a_b.clone().enumerate().take(len) {
        rowval.push(i);
        nzval.push(item.0);
    }
    colptr.push(rowval.len());

    for (i, item) in a_b.clone().enumerate().take(len) {
        rowval.push(i);
        nzval.push(item.1);
    }
    colptr.push(rowval.len());

    let a_csc = CscMatrix::new(len, 2, colptr, rowval, nzval);

    let b_vec: Vec<f64> = a_b.clone().map(|item| item.2).collect();

    let cones = [NonnegativeConeT(len)];
    let settings = DefaultSettings {
        verbose: false,
        ..DefaultSettings::default()
    };

    let mut solver = DefaultSolver::new(&p_csc, &q, &a_csc, &b_vec, &cones, settings).unwrap();

    solver.solve();

    let solution = &solver.solution;
    match solution.status {
        SolverStatus::Solved => {
            let x = solution.x[0];
            let y = solution.x[1];
            let max_val = w_x * x + w_y * y;
            (x, y, max_val)
        }
        _ => (f64::NAN, f64::NAN, f64::NAN),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::panic;
    use rand::RngExt;
    use std::fs;
    use std::io::{BufWriter, Write};
    use std::path::Path;
    use std::time::Instant;

    #[test]
    fn test_lp_2d_multi() {
        run_test_lp_2d_multi_repeated(1, false);
    }

    /// Conditions: release, --include-ignored, CPU = Intel(R) Core(TM) Ultra 9 285K.
    /// Average time per constraint: geometric = 51.043 ns, incremental = 18.859 ns, clarabel = 1839.307 ns
    #[test]
    #[ignore = "slow"]
    fn test_lp_2d_multi_robust() {
        run_test_lp_2d_multi_repeated(100000, true);
    }

    #[test]
    fn test_lp_2d_contour() {
        run_test_lp_2d_contour_repeated(1, false);
    }

    #[test]
    #[ignore = "slow"]
    fn test_lp_2d_contour_robust() {
        run_test_lp_2d_contour_repeated(100000, true);
    }

    #[test]
    fn test_lp_2d_extreme_direction() {
        run_test_lp_2d_extreme_direction_repeated(1, false);
    }

    /// Average ratio over 100000 experiments: 0.4341553260581677
    #[test]
    #[ignore = "slow"]
    fn test_lp_2d_extreme_direction_robust() {
        run_test_lp_2d_extreme_direction_repeated(100000, true);
    }

    #[test]
    fn test_lp_2d_warm_start() {
        run_test_lp_2d_warm_start_repeated(1, false);
    }

    #[test]
    #[ignore = "slow"]
    fn test_lp_2d_warm_start_robust() {
        run_test_lp_2d_warm_start_repeated(100000, true);
    }

    fn run_test_lp_2d_multi_repeated(n_exp: usize, flag_print_step: bool) {
        let path_str = "data/test/lp_2d_compare.csv";
        let path = Path::new(path_str);
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent).expect("Fail to create directory for lp_2d_compare.csv");
        }
        let file = fs::File::create(path).expect("Fail to create lp_2d_compare.csv");
        let mut writer = BufWriter::new(file);
        writeln!(
            writer,
            "n,geometric time (us),incremental time (us),clarabel time (us),geometric objective,incremental objective,clarabel objective"
        )
        .unwrap();

        let epsilon = 1E-8;

        let mut time_sum_geo = 0.0_f64;
        let mut time_sum_incre = 0.0_f64;
        let mut time_sum_clarabel = 0.0_f64;
        let mut a_b_buffer = Vec::<(f64, f64, f64)>::with_capacity(100);
        for i in 0..n_exp {
            let mut rng = rand::rng();
            let n = rng.random_range(0..100); // number of constraints
            let a_b: Vec<(f64, f64, f64)> = (0..n)
                .map(|_| {
                    (
                        rng.random_range(-1.0..1.0),
                        rng.random_range(-1.0..1.0),
                        rng.random_range(0.0..2.0),
                    )
                })
                .collect();
            let w = (rng.random_range(-1.0..1.0), rng.random_range(-1.0..1.0));
            // let mut a_b: Vec<(f64, f64, f64)> = [
            //     (0.000589574578666598, 1.3263417562226534, -9.95066925246844),
            //     (
            //         0.001768043037305233,
            //         3.9774972097647265,
            //         -29.840522176908607,
            //     ),
            // ]
            // .into();
            // let w = (0.0, 1.0);
            // let n = a_b.len();

            // for j in 0..n {
            //     println!(
            //         "A_b[{}] = Data3d({}, {}, {});",
            //         j, a_b[j].0, a_b[j].1, a_b[j].2
            //     );
            // }

            let start = Instant::now();
            let (x_geo, y_geo, j_geo) = lp_2d_geo(
                a_b.iter().copied(),
                a_b.len(),
                w,
                &LpToleranceOptions::with_feas_tol(epsilon),
            );
            let time_geo = start.elapsed().as_secs_f64() * 1E9; // ns
            // println!(
            //     "Geometric: time = {:?}, x = {}, y = {}, J = {} ({})",
            //     time_geo,
            //     x_geo,
            //     y_geo,
            //     j_geo,
            //     w.0 * x_geo + w.1 * y_geo
            // );
            assert!(
                j_geo.is_nan()
                    || j_geo.is_infinite()
                    || (w.0 * x_geo + w.1 * y_geo - j_geo).abs() < 1E-6
            );

            let start = Instant::now();
            let (x_clarabel, y_clarabel, j_clarabel) =
                lp_2d_clarabel(a_b.iter().copied(), a_b.len(), w);
            let time_clarabel = start.elapsed().as_secs_f64() * 1E9; // ns
            assert!(
                j_clarabel.is_nan()
                    || j_clarabel.is_infinite()
                    || (w.0 * x_clarabel + w.1 * y_clarabel - j_clarabel).abs() < 1E-6
            );
            // println!(
            //     "Clarabel: time = {:?}, x = {}, y = {}, J = {} ({})",
            //     time_clarabel,
            //     x_clarabel,
            //     y_clarabel,
            //     j_clarabel,
            //     w.0 * x_clarabel + w.1 * y_clarabel
            // );

            let start = Instant::now();
            // a_b.shuffle(&mut rng);
            let (x_inc, y_inc, j_inc) = lp_2d_incre::<_, true>(
                &a_b,
                w,
                &Lp2dNoWarmStart,
                &LpToleranceOptions::with_feas_tol(epsilon),
                &mut a_b_buffer,
            );
            let time_incre = start.elapsed().as_secs_f64() * 1E9; // ns
            assert!(
                j_inc.is_nan()
                    || j_inc.is_infinite()
                    || (w.0 * x_inc + w.1 * y_inc - j_inc).abs() < 1E-6
            );

            // println!(
            //     "Clarabel: time = {:?}, x = {}, y = {}, J = {}",
            //     time_clarabel, x_clarabel, y_clarabel, j_clarabel
            // );

            writeln!(
                writer,
                "{n},{time_geo},{time_incre},{time_clarabel},{j_geo},{j_inc},{j_clarabel}"
            )
            .unwrap();

            if flag_print_step && (i + 1) % 1000 == 0 {
                crate::verbosity_log!(
                    crate::diag::Verbosity::Summary,
                    "Exp #{}: time_geo/n: {:.3} ns, time_incre/n: {:.3} ns, time_clarabel/n: {:.3} ns, j_geo: {:.3e}, j_incre: {:.3e}, j_clarabel: {:.3e}",
                    i + 1,
                    time_geo / n.max(1) as f64,
                    time_incre / n.max(1) as f64,
                    time_clarabel / n.max(1) as f64,
                    j_geo,
                    j_inc,
                    j_clarabel
                );
            }
            time_sum_geo += time_geo / n.max(1) as f64;
            time_sum_incre += time_incre / n.max(1) as f64;
            time_sum_clarabel += time_clarabel / n.max(1) as f64;

            let clarabel_failed = j_clarabel.is_nan()
                || j_clarabel.is_infinite()
                || x_clarabel.abs() > 1E8
                || y_clarabel.abs() > 1E8;
            let geo_failed =
                j_geo.is_nan() || j_geo.is_infinite() || x_geo.abs() > 1E8 || y_geo.abs() > 1E8;
            let inc_failed =
                j_inc.is_nan() || j_inc.is_infinite() || x_inc.abs() > 1E8 || y_inc.abs() > 1E8;
            let flag_assert = (clarabel_failed != geo_failed)
                || (clarabel_failed != inc_failed)
                || (!clarabel_failed
                    && ((j_geo - j_clarabel).abs() >= 1E-3 || (j_inc - j_clarabel).abs() >= 1E-3));
            let flag_assert = if flag_assert {
                // println!("a_b = {a_b:?}");
                // for j in 0..n {
                //     println!(
                //         "A_b[{}] = Data3d({}, {}, {});",
                //         j, a_b[j].0, a_b[j].1, a_b[j].2
                //     );
                // }
                // println!("w: {w:?}");
                // println!(
                //     "j_geo-clarabel: {}, j_inc-clarabel: {}",
                //     j_geo - j_clarabel,
                //     j_inc - j_clarabel
                // );
                // println!(
                //     "x_geo = {:?}, x_inc = {:?}, x_clarabel = {:?}",
                //     (x_geo, y_geo),
                //     (x_inc, y_inc),
                //     (x_clarabel, y_clarabel)
                // );
                if !inc_failed {
                    let delta = a_b.iter().map(|(a, b, c)| c - a * x_inc - b * y_inc);
                    let delta_min = delta.clone().fold(f64::INFINITY, |a, b| a.min(b));
                    // println!("Minimum slackness at incremental solution: {delta_min}");
                    if delta_min < -epsilon {
                        crate::verbosity_log!(
                            crate::diag::Verbosity::Debug,
                            "Incremental solution is infeasible!"
                        );
                        true
                    } else {
                        false
                    }
                } else {
                    true
                }
            } else {
                false
            };
            assert!(!flag_assert);
        }
        crate::verbosity_log!(
            crate::diag::Verbosity::Summary,
            "Average time per constraint: geometric = {:.3} ns, incremental = {:.3} ns, clarabel = {:.3} ns",
            time_sum_geo / n_exp as f64,
            time_sum_incre / n_exp as f64,
            time_sum_clarabel / n_exp as f64
        );
    }

    fn run_test_lp_2d_contour_repeated(n_exp: usize, flag_print_step: bool) {
        let mut buffer_intersection_plus = Vec::<(f64, f64)>::with_capacity(106);
        let mut buffer_intersection_minus = Vec::<(f64, f64)>::with_capacity(106);

        let mut a_b_buffer = Vec::<(f64, f64, f64)>::with_capacity(106);

        let epsilon = 1E-9;
        for i in 0..n_exp {
            let mut rng = rand::rng();
            let n = rng.random_range(0..100); // number of constraints

            let mut a_b: Vec<(f64, f64, f64)> = (0..n)
                .map(|_| {
                    (
                        rng.random_range(-1.0..1.0),
                        rng.random_range(-1.0..1.0),
                        rng.random_range(0.0..2.0),
                    )
                })
                .collect();
            a_b.extend([
                (1.0, 0.0, LP_BOUND),
                (-1.0, 0.0, LP_BOUND),
                (0.0, 1.0, LP_BOUND),
                (0.0, -1.0, LP_BOUND),
            ]);
            // let a_b: Vec<(f64, f64, f64)> = [
            //     (1.0, 0.0, 995.8861908753399),
            //     (-1.0, 0.0, -697.1203336127378),
            //     (0.1331968505664206, 0.9910896019024651, 129.69547412483433),
            //     (-0.1331968505664206, -0.9910896019024651, -129.6954741248343),
            //     (4.0512058517625615e-5, 0.9999999991793865, 26.27558522984379),
            //     (0.0, 0.0, 0.0),
            //     (-0.0002148857862811844, 0.999999976912049, 60.48528350326356),
            //     (-3.365994633223831e-8, 0.9999999999999993, 29.51611830471187),
            //     (4.0512058517625615e-5, 0.9999999991793865, 26.27558522984379),
            //     (0.0, 0.0, 0.0),
            //     (-0.0002148857862811844, 0.999999976912049, 60.48528350326356),
            //     (-3.365994633223831e-8, 0.9999999999999993, 29.51611830471187),
            //     (
            //         -0.042685922007249515,
            //         -0.999088540652124,
            //         -39.37634909168048,
            //     ),
            //     (
            //         0.015909459988051884,
            //         5.163472720887478e-14,
            //         47.53203451914932,
            //     ),
            //     (
            //         -0.039990724634325785,
            //         -0.9992000510124194,
            //         -31.318622868363203,
            //     ),
            //     (0.046820810258963015, 0.9989033044928294, 51.746327742644965),
            //     (
            //         0.015909459988047263,
            //         -5.163472720887478e-14,
            //         47.53203451914462,
            //     ),
            //     (0.04950825533983154, 0.9987737144384637, 59.79137099766719),
            //     (
            //         -0.0440595097383263,
            //         -0.9990289082912559,
            //         -40.744391349690204,
            //     ),
            //     (0.007954729994139207, 0.05918946393692667, 39.43364517828572),
            //     (-0.04313208344058018, -0.999069378660999, -34.33229069734884),
            //     (0.04543742551563322, 0.9989671868297332, 50.30540896403363),
            //     (0.007954729994159242, 0.05918946393737094, 39.43364517830605),
            //     (0.0463113429889185, 0.9989270541488816, 56.391565689431246),
            // ]
            // .into();

            let mut contour_geo = Vec::<(f64, f64)>::with_capacity(a_b.len());
            let start = Instant::now();
            lp_2d_geo_contour_bounded::<false>(
                &a_b,
                &LpToleranceOptions::with_feas_tol(epsilon),
                &mut contour_geo,
                (
                    &mut buffer_intersection_plus,
                    &mut buffer_intersection_minus,
                ),
            );
            let time_geo = start.elapsed().as_secs_f64() * 1E6; // us

            if contour_geo.is_empty() {
                let (_, _, xmax) = lp_2d_incre::<_, true>(
                    &a_b,
                    (1.0, 0.0),
                    &Lp2dNoWarmStart,
                    &LpToleranceOptions::with_feas_tol(epsilon),
                    &mut a_b_buffer,
                );
                let (_, _, ymax) = lp_2d_incre::<_, false>(
                    &a_b,
                    (0.0, 1.0),
                    &Lp2dNoWarmStart,
                    &LpToleranceOptions::with_feas_tol(epsilon),
                    &mut a_b_buffer,
                );
                let (_, _, xmin) = lp_2d_incre::<_, false>(
                    &a_b,
                    (-1.0, 0.0),
                    &Lp2dNoWarmStart,
                    &LpToleranceOptions::with_feas_tol(epsilon),
                    &mut a_b_buffer,
                );
                let (_, _, ymin) = lp_2d_incre::<_, false>(
                    &a_b,
                    (0.0, -1.0),
                    &Lp2dNoWarmStart,
                    &LpToleranceOptions::with_feas_tol(epsilon),
                    &mut a_b_buffer,
                );
                let infeasible = xmax.is_nan() || ymax.is_nan() || xmin.is_nan() || ymin.is_nan();
                let unbounded = xmax.is_infinite()
                    || ymax.is_infinite()
                    || xmin.is_infinite()
                    || ymin.is_infinite();

                assert!(
                    infeasible || unbounded,
                    "Failed ({}) at exp {}: xmax={}, ymax={}, xmin={}, ymin={}\na_b={:?}",
                    if infeasible {
                        "infeasible"
                    } else {
                        "unbounded"
                    },
                    i + 1,
                    xmax,
                    ymax,
                    xmin,
                    ymin,
                    a_b,
                );
                if flag_print_step && infeasible {
                    crate::verbosity_log!(
                        crate::diag::Verbosity::Debug,
                        "Exp {}: Infeasible region.",
                        i + 1
                    );
                } else if flag_print_step && unbounded {
                    crate::verbosity_log!(
                        crate::diag::Verbosity::Debug,
                        "Exp {}: Unbounded region.",
                        i + 1
                    );
                }
                continue;
            }

            for j in 0..contour_geo.len() {
                let feasibility = a_b
                    .iter()
                    .all(|(a, b, c)| a * contour_geo[j].0 + b * contour_geo[j].1 <= *c + 1E-6);
                assert!(
                    feasibility,
                    "Failed (infeasibility) at exp {}, point {}: point=({},{})\ncontour = [{:?};{:?}];\n\na_b={:?}",
                    i + 1,
                    j,
                    contour_geo[j].0,
                    contour_geo[j].1,
                    contour_geo.iter().map(|p| p.0).collect::<Vec<f64>>(),
                    contour_geo.iter().map(|p| p.1).collect::<Vec<f64>>(),
                    a_b,
                );
                let next_j = (j + 1) % contour_geo.len();
                let w_test = (
                    contour_geo[next_j].1 - contour_geo[j].1,
                    contour_geo[j].0 - contour_geo[next_j].0,
                );
                let (_, _, j_opt) = lp_2d_incre::<_, false>(
                    &a_b,
                    w_test,
                    &Lp2dNoWarmStart,
                    &LpToleranceOptions::with_feas_tol(epsilon),
                    &mut a_b_buffer,
                );
                let j_contour = w_test.0 * contour_geo[j].0 + w_test.1 * contour_geo[j].1;
                assert!(
                    (j_opt - j_contour).abs() <= 1E-6 * j_opt.abs(),
                    "Failed (optimality) at exp {}, edge {}: j_opt = {}, j_contour = {}\nw=[{},{}]\ncontour = [{:?};{:?}];\n\na_b = {:?}\n",
                    i + 1,
                    j,
                    j_opt,
                    j_contour,
                    w_test.0,
                    w_test.1,
                    contour_geo.iter().map(|p| p.0).collect::<Vec<f64>>(),
                    contour_geo.iter().map(|p| p.1).collect::<Vec<f64>>(),
                    a_b,
                );
            }

            if flag_print_step && (i + 1) % 1000 == 0 {
                crate::verbosity_log!(
                    crate::diag::Verbosity::Summary,
                    "Exp {}: time_cal = {} us, contour.len = {}",
                    i + 1,
                    time_geo,
                    contour_geo.len()
                );
            }

            // Test the reverse direction, which should give the same contour in reverse order
            let mut contour_geo_rev = Vec::<(f64, f64)>::with_capacity(a_b.len());
            lp_2d_geo_contour_bounded::<true>(
                &a_b,
                &LpToleranceOptions::with_feas_tol(epsilon),
                &mut contour_geo_rev,
                (
                    &mut buffer_intersection_plus,
                    &mut buffer_intersection_minus,
                ),
            );
            assert!(contour_geo.len() == contour_geo_rev.len());
            let contour_len = contour_geo.len();
            for i in 0..contour_len {
                let p = contour_geo[i];
                let p_new = contour_geo_rev[(contour_len - i) % contour_len];
                assert!((p.0 - p_new.0).abs() < epsilon);
                assert!((p.1 - p_new.1).abs() < epsilon);
            }
        }
    }

    fn run_test_lp_2d_extreme_direction_repeated(n_exp: usize, flag_print_step: bool) {
        let mut rng = rand::rng();

        let mut ratio_sum = 0.0_f64;
        let mut a_b_buffer = Vec::<(f64, f64, f64)>::with_capacity(106);

        for i_exp in 0..n_exp {
            // 1. Generate a_b, ensure (0,0,0) is feasible and bounded
            let n = rng.random_range(0..100);
            let mut time_sum_direction = 0.0;
            let mut time_sum_lp2d = 0.0;
            let mut n_w = 0;
            let mut a_b: Vec<(f64, f64, f64)> = (0..n)
                .map(|_| {
                    (
                        rng.random_range(-1.0..1.0),
                        rng.random_range(-1.0..1.0),
                        rng.random_range(0.1..2.0),
                    )
                })
                .collect();
            let bound = 1.0;
            a_b.push((1.0, 0.0, bound));
            a_b.push((-1.0, 0.0, bound));
            a_b.push((0.0, 1.0, bound));
            a_b.push((0.0, -1.0, bound));

            // let a_b: Vec<(f64, f64, f64)> = [
            //     (-0.8101855338504307, -0.636376960199049, 0.4481497935390153),
            //     (
            //         -0.5834796271826281,
            //         -0.5765578558113358,
            //         0.36766865821476524,
            //     ),
            //     (0.5987445035089243, -0.8241312626215276, 0.16051253942952126),
            // ]
            // .into();

            if flag_print_step && (i_exp + 1) % 1000 == 0 {
                crate::verbosity_log!(
                    crate::diag::Verbosity::Summary,
                    "Experiment #{}, num_constraints = {}",
                    i_exp + 1,
                    a_b.len()
                );
            }

            // 2. Generate w in 18 directions
            for i in 0..18 {
                let angle =
                    (i as f64) * std::f64::consts::PI * 2.0 / 18.0 + rng.random_range(-0.01..0.01);
                let w_2d = (angle.cos(), angle.sin());
                // let w_2d = (-0.4970776426704771, -0.8677060661060065);

                // Solve LP
                let epsilon = 1e-9;
                let start = std::time::Instant::now();
                let (x, y, j_opt) = if i == 0 {
                    lp_2d_incre::<_, true>(
                        &a_b,
                        w_2d,
                        &Lp2dNoWarmStart,
                        &LpToleranceOptions::with_feas_tol(epsilon),
                        &mut a_b_buffer,
                    )
                } else {
                    lp_2d_incre::<_, false>(
                        &a_b,
                        w_2d,
                        &Lp2dNoWarmStart,
                        &LpToleranceOptions::with_feas_tol(epsilon),
                        &mut a_b_buffer,
                    )
                };
                time_sum_lp2d += start.elapsed().as_secs_f64() * 1e9;
                if j_opt.is_nan() || j_opt.is_infinite() {
                    panic!("LP failed at exp {}, angle {}", i_exp, i);
                }
                let x_opt = (x, y);

                // 3. Project Extremes
                let start = std::time::Instant::now();
                let (v1, v2) = lp_2d_extreme_direction(
                    &a_b,
                    x_opt,
                    w_2d,
                    &LpToleranceOptions::with_feas_tol(1e-6),
                );
                time_sum_direction += start.elapsed().as_secs_f64() * 1e9;
                n_w += 1;

                if v1.0.is_nan() || v2.0.is_nan() || v1.1.is_nan() || v2.1.is_nan() {
                    crate::verbosity_log!(crate::diag::Verbosity::Summary, "a_b = {a_b:?}");
                    crate::verbosity_log!(crate::diag::Verbosity::Summary, "w_2d = {w_2d:?}");
                    crate::verbosity_log!(
                        crate::diag::Verbosity::Summary,
                        "v1 = {v1:?}, v2 = {v2:?}"
                    );
                    panic!(
                        "Extreme direction NAN at exp {}, angle {}. x_opt={:?}",
                        i_exp, i, x_opt
                    );
                }

                // 4. Verification
                for (k, &v) in [v1, v2].iter().enumerate() {
                    // (1) Rotate 90 degree as optimization target direction, determine optimal value is same as x_opt
                    let n1 = if k == 0 { (v.1, -v.0) } else { (-v.1, v.0) };
                    let (_, _, j1) = lp_2d_incre::<_, false>(
                        &a_b,
                        n1,
                        &Lp2dNoWarmStart,
                        &LpToleranceOptions::with_feas_tol(epsilon),
                        &mut a_b_buffer,
                    );
                    let val1 = n1.0 * x + n1.1 * y;

                    // Ideally one of them matches exactly. Due to precision, check closeness.
                    let diff = (j1 - val1).abs();
                    if diff > 1e-5 {
                        crate::verbosity_log!(crate::diag::Verbosity::Summary, "a_b = {a_b:?}");
                        crate::verbosity_log!(crate::diag::Verbosity::Summary, "w_2d = {w_2d:?}");
                        crate::verbosity_log!(
                            crate::diag::Verbosity::Summary,
                            "v1 = {v1:?}, v2 = {v2:?}"
                        );
                        crate::verbosity_log!(crate::diag::Verbosity::Summary, "x_opt = {x_opt:?}");
                        panic!(
                            "Rotation check failed for v{}: diff={}\nExp {}, angle {}, w={:?}, v={:?}",
                            k, diff, i_exp, i, w_2d, v
                        );
                    }

                    // (2) Use lp_1d to determine this direction is feasible
                    // i.e. not NAN and v3max > 0.0
                    let delta_max = a_b
                        .iter()
                        .map(|&(a, b, c)| c - a * x_opt.0 - b * x_opt.1)
                        .fold(f64::INFINITY, f64::min)
                        .abs();
                    // if delta_max > 1e-5 {
                    // println!("delta_max = {}", delta_max);
                    // }
                    let iter = a_b.iter().map(|&(a, b, c)| {
                        (a * v.0 + b * v.1, (c - a * x_opt.0 - b * x_opt.1).max(0.0))
                    });
                    // println!(
                    //     "============Checking feasibility of direction v{} = {:?}, delta_max = {}=============",
                    //     k, v, delta_max
                    // );
                    let (tmax, _tmin) = lp_1d::<true>(
                        iter,
                        &LpToleranceOptions::with_feas_tol((1.1 * delta_max).max(1e-6)),
                    );
                    if !tmax.is_nan() && tmax <= 0.0 {
                        crate::verbosity_log!(crate::diag::Verbosity::Summary, "a_b = {a_b:?}");
                        crate::verbosity_log!(crate::diag::Verbosity::Summary, "w_2d = {w_2d:?}");
                        let a_b_t12 = a_b
                            .iter()
                            .map(|&(a, b, c)| {
                                (a * v.0 + b * v.1, (c - a * x_opt.0 - b * x_opt.1).max(0.0))
                            })
                            .collect::<Vec<(f64, f64)>>();
                        crate::verbosity_log!(
                            crate::diag::Verbosity::Summary,
                            "a_b for t12 = {a_b_t12:?}"
                        );
                        crate::verbosity_log!(
                            crate::diag::Verbosity::Summary,
                            "delta for t12 constraints = {:?}",
                            a_b_t12.iter().map(|(_, b)| *b).collect::<Vec<f64>>()
                        );
                        crate::verbosity_log!(crate::diag::Verbosity::Summary, "x_opt = {x_opt:?}");
                        crate::verbosity_log!(
                            crate::diag::Verbosity::Summary,
                            "k = {k}, v1 = {v1:?}, v2 = {v2:?}"
                        );
                        crate::verbosity_log!(
                            crate::diag::Verbosity::Summary,
                            "tmin = {_tmin}, tmax = {tmax}"
                        );
                        panic!(
                            "Direction v{} not feasible (lp_2d returned NAN). Exp {}, angle {}",
                            k, i_exp, i
                        )
                    }
                }
            }
            if flag_print_step && (i_exp + 1) % 1000 == 0 {
                crate::verbosity_log!(
                    crate::diag::Verbosity::Summary,
                    "Exp {}: cal_time_lp2d: {} ns, cal_time_direction: {} ns, ratio: {}",
                    i_exp + 1,
                    time_sum_lp2d / n_w as f64,
                    time_sum_direction / n_w as f64,
                    time_sum_direction / time_sum_lp2d
                );
            }
            ratio_sum += time_sum_direction / time_sum_lp2d;
        }
        crate::verbosity_log!(
            crate::diag::Verbosity::Summary,
            "Average ratio over {} experiments: {}",
            n_exp,
            ratio_sum / n_exp as f64
        );
    }

    fn run_test_lp_2d_warm_start_repeated(n_exp: usize, flag_print_step: bool) {
        let mut time_sum_warm = 0.0_f64;
        let mut time_sum_cold = 0.0_f64;

        let mut a_b_buffer = Vec::<(f64, f64, f64)>::with_capacity(106);

        let epsilon = 1E-8;
        for i in 0..n_exp {
            let mut rng = rand::rng();
            let n = rng.random_range(0..10); // number of constraints
            let n_warm = rng.random_range(0..=n);
            let a_b: Vec<(f64, f64, f64)> = (0..n)
                .map(|_| {
                    (
                        rng.random_range(-1.0..1.0),
                        rng.random_range(-1.0..1.0),
                        rng.random_range(0.0..2.0),
                    )
                })
                .collect();
            let w = (rng.random_range(-1.0..1.0), rng.random_range(-1.0..1.0));

            // let mut a_b: Vec<(f64, f64, f64)> = [
            //     (0.6826148498882487, 0.15950363408897372, 1.9528195977566853),
            //     (-0.6294516510574168, -0.45007860910959874, 1.53466195227118),
            //     (-0.2644639261220636, 0.6401234871975734, 0.9503190109568345),
            // ]
            // .into();
            // let w = (-0.7671173976232137, 0.45694048421237765);
            // let n_warm = 0;
            // let n = a_b.len();

            let start = Instant::now();
            // a_b.shuffle(&mut rng);
            let (x_cold, y_cold, j_cold) = lp_2d_incre::<_, true>(
                &a_b,
                w,
                &Lp2dNoWarmStart,
                &LpToleranceOptions::with_feas_tol(epsilon),
                &mut a_b_buffer,
            );
            let time_cold = start.elapsed().as_secs_f64() * 1E9; // ns
            assert!(
                j_cold.is_nan()
                    || j_cold.is_infinite()
                    || (w.0 * x_cold + w.1 * y_cold - j_cold).abs() < 1E-6
            );

            let start = Instant::now();
            // a_b.shuffle(&mut rng);
            let (x_mid, y_mid, _j_mid) = lp_2d_incre::<_, false>(
                &a_b[0..n_warm],
                w,
                &Lp2dNoWarmStart,
                &LpToleranceOptions::with_feas_tol(epsilon),
                &mut a_b_buffer,
            );
            let (x_warm, y_warm, j_warm) = if x_mid.is_nan() || y_mid.is_nan() {
                lp_2d_incre::<_, false>(
                    &a_b,
                    w,
                    &Lp2dNoWarmStart,
                    &LpToleranceOptions::with_feas_tol(epsilon),
                    &mut a_b_buffer,
                )
            } else {
                lp_2d_incre::<_, false>(
                    &a_b,
                    w,
                    &Lp2dWarmStart {
                        x0: (x_mid, y_mid),
                        skip: n_warm,
                    },
                    &LpToleranceOptions::with_feas_tol(epsilon),
                    &mut a_b_buffer,
                )
            };
            let time_warm = start.elapsed().as_secs_f64() * 1E9; // ns
            assert!(
                j_warm.is_nan()
                    || j_warm.is_infinite()
                    || (w.0 * x_warm + w.1 * y_warm - j_warm).abs() < 1E-6
            );

            // println!(
            //     "Clarabel: time = {:?}, x = {}, y = {}, J = {}",
            //     time_clarabel, x_clarabel, y_clarabel, j_clarabel
            // );
            if flag_print_step && (i + 1) % 1000 == 0 {
                crate::verbosity_log!(
                    crate::diag::Verbosity::Summary,
                    "Exp #{}: time_cold/n: {:.3} ns, time_warm/n: {:.3} ns, j_cold: {:.3e}, j_warm: {:.3e}",
                    i + 1,
                    time_cold / n.max(1) as f64,
                    time_warm / n.max(1) as f64,
                    j_cold,
                    j_warm,
                );
            }
            time_sum_cold += time_cold / n.max(1) as f64;
            time_sum_warm += time_warm / n.max(1) as f64;

            let cold_failed =
                j_cold.is_nan() || j_cold.is_infinite() || x_cold.abs() > 1E8 || y_cold.abs() > 1E8;
            let warm_failed =
                j_warm.is_nan() || j_warm.is_infinite() || x_warm.abs() > 1E8 || y_warm.abs() > 1E8;
            let flag_assert =
                (warm_failed != cold_failed) || (!warm_failed && ((j_warm - j_cold).abs() >= 1E-3));
            let flag_assert = if flag_assert {
                crate::verbosity_log!(crate::diag::Verbosity::Summary, "a_b = {a_b:?}");
                crate::verbosity_log!(crate::diag::Verbosity::Summary, "w: {w:?}");
                crate::verbosity_log!(crate::diag::Verbosity::Summary, "n_warm: {n_warm}");
                crate::verbosity_log!(
                    crate::diag::Verbosity::Summary,
                    "j_warm-j_cold: {}",
                    j_warm - j_cold,
                );
                crate::verbosity_log!(
                    crate::diag::Verbosity::Summary,
                    "x_warm = {x_warm:<10.3e}, x_inc = {x_cold:<10.3e}"
                );
                crate::verbosity_log!(
                    crate::diag::Verbosity::Summary,
                    "y_warm = {y_warm:<10.3e}, y_inc = {y_cold:<10.3e}"
                );
                if !warm_failed {
                    let delta = a_b.iter().map(|(a, b, c)| c - a * x_cold - b * y_cold);
                    let delta_min = delta.clone().fold(f64::INFINITY, |a, b| a.min(b));
                    crate::verbosity_log!(
                        crate::diag::Verbosity::Summary,
                        "Minimum slackness at incremental solution: {delta_min}"
                    );
                    if delta_min < -epsilon {
                        crate::verbosity_log!(
                            crate::diag::Verbosity::Debug,
                            "Incremental solution is infeasible!"
                        );
                        true
                    } else {
                        false
                    }
                } else {
                    true
                }
            } else {
                false
            };
            assert!(!flag_assert);
        }
        crate::verbosity_log!(
            crate::diag::Verbosity::Summary,
            "Average time per constraint: warm = {:.3} ns, cold = {:.3} ns",
            time_sum_warm / n_exp as f64,
            time_sum_cold / n_exp as f64,
        );
    }
}
