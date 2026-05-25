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
    /// Lower bound used in row normalization to avoid aggressive scaling.
    /// Tolerance scale factor used when reducing dimension (3D->2D->1D).
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
                y = p * x - q;
            }
        }
    }

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
    }
    if AUTONAN {
        if xmin <= xmax + epsilon {
            if xmin <= xmax {
                (xmax, xmin)
            } else {
                let x = 0.5 * (xmax + xmin);
                (x, x)
            }
        } else {
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
