//! Shared Clarabel backend options for TOPP/COPP optimization solvers.
//! This module provides a unified configuration surface for all Clarabel-based backends in this crate (e.g., TOPP2-LP, COPP2-SOCP, TOPP3-LP/SOCP, COPP3-OPT).
//! Design goals:
//! - Keep one stable, user-facing option model across multiple solvers;
//! - Allow advanced users to pass raw [`DefaultSettings<f64>`](clarabel::solver::DefaultSettings) when needed;
//! - Prevent duplicated console output by coordinating crate-level [`Verbosity`](crate::diag::Verbosity) and Clarabel's internal `settings.verbose`.

use crate::copp::copp3::{Topp3Profile, formulation::set_ab_stationary_topp3};
use crate::diag::{CoppError, Verbosity};
use clarabel::solver::{DefaultSettings, DefaultSolution, SolverStatus, SupportedConeT};

/// Return crate default Clarabel settings.
/// Policy:
/// - `verbose = true` in debug builds for local development diagnostics;
/// - `verbose = false` in release builds for cleaner production logs.
/// - all other fields use Clarabel's own defaults ([`Default::default`](std::default::Default::default)).
#[inline]
pub(crate) fn default_clarabel_settings() -> DefaultSettings<f64> {
    DefaultSettings::<f64> {
        verbose: cfg!(debug_assertions),
        equilibrate_max_iter: 20,
        ..Default::default()
    }
}

/// Shared options for Clarabel-based optimization routines.
///
/// Field semantics:
/// - `verbosity`: crate-level diagnostics switch used by this crate's algorithms;
/// - `clarabel_settings`: low-level Clarabel solver configuration passed to [`DefaultSolver::new`](clarabel::solver::DefaultSolver::new).
/// - `allow_*`: policy switches for accepting non-ideal Clarabel statuses when deciding whether an extracted `a` profile is considered usable by wrapper APIs. More details in [`ClarabelOptions::is_allow`](crate::solver::copp2_socp::ClarabelOptions::is_allow).
/// - priority rule at build time: if `verbosity <= Verbosity::Summary`, builder will force `clarabel_settings.verbose = false` to avoid duplicate logs from both layers.
pub struct ClarabelOptions {
    /// Crate-level verbosity for algorithm diagnostics and summaries.
    pub(crate) verbosity: Verbosity,
    /// Raw Clarabel settings to be forwarded to solver construction.
    pub(crate) clarabel_settings: DefaultSettings<f64>,
    /// Allow status [`SolverStatus::AlmostSolved`](clarabel::solver::SolverStatus::AlmostSolved).
    pub(crate) allow_almost_solved: bool,
    /// Allow status [`SolverStatus::MaxIterations`](clarabel::solver::SolverStatus::MaxIterations).
    pub(crate) allow_max_iterations: bool,
    /// Allow status [`SolverStatus::MaxTime`](clarabel::solver::SolverStatus::MaxTime).
    pub(crate) allow_max_time: bool,
    /// Allow status [`SolverStatus::CallbackTerminated`](clarabel::solver::SolverStatus::CallbackTerminated).
    pub(crate) allow_callback_terminated: bool,
    /// Allow status [`SolverStatus::InsufficientProgress`](clarabel::solver::SolverStatus::InsufficientProgress).
    pub(crate) allow_insufficient_progress: bool,
}

impl ClarabelOptions {
    /// Get crate-level verbosity.
    #[inline]
    pub fn verbosity(&self) -> Verbosity {
        self.verbosity
    }

    /// Get immutable reference to raw Clarabel settings.
    #[inline]
    pub fn clarabel_settings(&self) -> &DefaultSettings<f64> {
        &self.clarabel_settings
    }

    /// Return whether a solver status ([`SolverStatus`](clarabel::solver::SolverStatus)) is accepted by current policy.
    /// Policy summary:
    /// - [`SolverStatus::Solved`](clarabel::solver::SolverStatus::Solved) is always accepted;
    /// - selected non-ideal statuses are accepted only if their corresponding `allow_*` switch is enabled;
    /// - all other statuses are rejected.
    ///
    /// Typical wrapper behavior based on this predicate:
    /// - accepted status (`true`) => return `(Some(copp_solution), clarabel_solution)`;
    /// - rejected status (`false`) => return `(None, clarabel_solution)`;
    /// - regardless of acceptance, always return full Clarabel `solution` for advanced users to inspect convergence and diagnostics.
    /// - `copp_solution` is typically `a: Vec<f64>` in TOPP2/COPP2 or [`Topp3Profile`](crate::solver::copp3_socp::Topp3Profile) in TOPP3/COPP3. You can call [`clarabel_to_copp2_solution`](crate::solver::copp2_socp::clarabel_to_copp2_solution) or [`clarabel_to_copp3_solution`](crate::solver::copp3_socp::clarabel_to_copp3_solution) to generate a `copp_solution` from a `clarabel_solution` with a valid `clarabel_solution.x`.
    /// - `clarabel_solution` is the full [`DefaultSolution<f64>`](clarabel::solver::DefaultSolution) returned by Clarabel, which may contain useful information for expert users even when status is not ideal.
    #[inline]
    pub fn is_allow(&self, status: SolverStatus) -> bool {
        match status {
            SolverStatus::Solved => true,
            SolverStatus::AlmostSolved => self.allow_almost_solved,
            SolverStatus::MaxIterations => self.allow_max_iterations,
            SolverStatus::MaxTime => self.allow_max_time,
            SolverStatus::CallbackTerminated => self.allow_callback_terminated,
            SolverStatus::InsufficientProgress => self.allow_insufficient_progress,
            _ => false,
        }
    }
}

/// Builder for [`ClarabelOptions`](crate::solver::copp2_socp::ClarabelOptions).
///
/// Usage policy:
/// - Keep default path minimal for common users ([`new`](ClarabelOptionsBuilder::new));
/// - Allow expert tuning with raw Clarabel settings ([`with_clarabel_setting`](ClarabelOptionsBuilder::with_clarabel_setting));
/// - By default, [`SolverStatus::Solved`](clarabel::solver::SolverStatus::Solved) and
///   [`SolverStatus::AlmostSolved`](clarabel::solver::SolverStatus::AlmostSolved) are accepted; all other `allow_*` switches default to `false`;
/// - Normalize output behavior in [`build`](ClarabelOptionsBuilder::build) by applying verbosity-vs-clarabel logging precedence.
pub struct ClarabelOptionsBuilder {
    /// Raw Clarabel settings that will be embedded into built options.
    pub clarabel_settings: DefaultSettings<f64>,
    /// Whether to accept [`SolverStatus::AlmostSolved`](clarabel::solver::SolverStatus::AlmostSolved) as usable.
    pub allow_almost_solved: bool,
    /// Whether to accept [`SolverStatus::MaxIterations`](clarabel::solver::SolverStatus::MaxIterations) as usable.
    pub allow_max_iterations: bool,
    /// Whether to accept [`SolverStatus::MaxTime`](clarabel::solver::SolverStatus::MaxTime) as usable.
    pub allow_max_time: bool,
    /// Whether to accept [`SolverStatus::CallbackTerminated`](clarabel::solver::SolverStatus::CallbackTerminated) as usable.
    pub allow_callback_terminated: bool,
    /// Whether to accept [`SolverStatus::InsufficientProgress`](clarabel::solver::SolverStatus::InsufficientProgress) as usable.
    pub allow_insufficient_progress: bool,
    /// Verbosity level for diagnostics during reachability analysis.
    pub verbosity: Verbosity,
}

impl Default for ClarabelOptionsBuilder {
    #[inline]
    fn default() -> Self {
        Self {
            verbosity: Verbosity::default(),
            clarabel_settings: default_clarabel_settings(),
            allow_almost_solved: true,
            allow_max_iterations: false,
            allow_max_time: false,
            allow_callback_terminated: false,
            allow_insufficient_progress: false,
        }
    }
}

impl ClarabelOptionsBuilder {
    /// Create a builder with crate default Clarabel settings.
    /// Defaults:
    /// - `verbosity = Verbosity::Silent`;
    /// - `clarabel_settings = default_clarabel_settings()`;
    /// - `allow_almost_solved = true`, all other `allow_*` switches are `false`.
    #[inline]
    pub fn new() -> Self {
        Default::default()
    }

    /// Create a builder from caller-provided Clarabel settings.
    /// This is intended for users who need direct control over Clarabel tolerances, limits, or linear-solver behavior.
    /// # Example
    /// ```rust,ignore
    /// use copp::diag::Verbosity;
    /// use copp::solver::copp2_socp::ClarabelOptionsBuilder;
    /// use clarabel::solver::DefaultSettings;
    ///
    /// let custom = DefaultSettings::<f64> {
    ///     tol_gap_rel: 1e-7,
    ///     max_iter: 300,
    ///     verbose: true,
    ///     ..Default::default()
    /// };
    ///
    /// let options = ClarabelOptionsBuilder::with_clarabel_setting(custom)
    ///     .verbosity(Verbosity::Debug)
    ///     .build()
    ///     .expect("valid options");
    ///
    /// assert!(options.clarabel_settings().tol_gap_rel <= 1.01e-7);
    /// ```
    #[inline]
    pub fn with_clarabel_setting(clarabel_settings: DefaultSettings<f64>) -> Self {
        Self {
            verbosity: Verbosity::default(),
            clarabel_settings,
            allow_almost_solved: true,
            allow_max_iterations: false,
            allow_max_time: false,
            allow_callback_terminated: false,
            allow_insufficient_progress: false,
        }
    }

    /// Set crate-level verbosity used by solver wrappers and algorithm diagnostics.
    #[inline]
    pub fn verbosity(mut self, verbosity: Verbosity) -> Self {
        self.verbosity = verbosity;
        self
    }

    /// Replace Clarabel settings in the builder.
    #[inline]
    pub fn clarabel_setting(mut self, clarabel_settings: DefaultSettings<f64>) -> Self {
        self.clarabel_settings = clarabel_settings;
        self
    }

    /// Allow [`SolverStatus::AlmostSolved`](clarabel::solver::SolverStatus::AlmostSolved).
    /// This is enabled by default; pass `false` for a strict solved-only policy.
    /// Rationale: [`AlmostSolved`](clarabel::solver::SolverStatus::AlmostSolved) usually means the solver is very close to strict tolerances and the returned primal vector is often practically usable.
    ///
    /// Integration contract used by wrapper APIs:
    /// - if status is [`SolverStatus::Solved`](clarabel::solver::SolverStatus::Solved) or enabled by `allow_*`, wrapper returns `Some(a)`;
    /// - otherwise wrapper returns `None`;
    /// - in both branches, full Clarabel `solution` should still be returned to support expert post-analysis.
    #[inline]
    pub fn allow_almost_solved(mut self, allow: bool) -> Self {
        self.allow_almost_solved = allow;
        self
    }

    /// Allow [`SolverStatus::MaxIterations`](clarabel::solver::SolverStatus::MaxIterations).
    /// Meaning: solver stopped because iteration cap was reached.
    #[inline]
    pub fn allow_max_iterations(mut self, allow: bool) -> Self {
        self.allow_max_iterations = allow;
        self
    }

    /// Allow [`SolverStatus::MaxTime`](clarabel::solver::SolverStatus::MaxTime).
    /// Meaning: solver stopped because time budget was reached.
    #[inline]
    pub fn allow_max_time(mut self, allow: bool) -> Self {
        self.allow_max_time = allow;
        self
    }

    /// Allow [`SolverStatus::CallbackTerminated`](clarabel::solver::SolverStatus::CallbackTerminated).
    /// Meaning: solver was terminated by user callback logic.
    #[inline]
    pub fn allow_callback_terminated(mut self, allow: bool) -> Self {
        self.allow_callback_terminated = allow;
        self
    }

    /// Allow [`SolverStatus::InsufficientProgress`](clarabel::solver::SolverStatus::InsufficientProgress).
    /// Meaning: solver progress was judged insufficient before full convergence.
    #[inline]
    pub fn allow_insufficient_progress(mut self, allow: bool) -> Self {
        self.allow_insufficient_progress = allow;
        self
    }

    /// Build finalized [`ClarabelOptions`](crate::solver::copp2_socp::ClarabelOptions).
    /// Normalization rule: if `verbosity <= Verbosity::Summary`,
    /// `clarabel_settings.verbose` is forcibly set to `false`.
    /// This keeps output ownership at crate level and avoids interleaved duplicate logs.
    pub fn build(mut self) -> Result<ClarabelOptions, CoppError> {
        if self.verbosity <= Verbosity::Summary {
            self.clarabel_settings.verbose = false;
        }
        Ok(ClarabelOptions {
            verbosity: self.verbosity,
            clarabel_settings: self.clarabel_settings,
            allow_almost_solved: self.allow_almost_solved,
            allow_max_iterations: self.allow_max_iterations,
            allow_max_time: self.allow_max_time,
            allow_callback_terminated: self.allow_callback_terminated,
            allow_insufficient_progress: self.allow_insufficient_progress,
        })
    }
}

/// (row, col, val, b, cones) for clarabel optimization
pub(crate) type ConstraintsClarabel<'a> = (
    &'a mut Vec<usize>,               // row
    &'a mut Vec<usize>,               // col
    &'a mut Vec<f64>,                 // val
    &'a mut Vec<f64>,                 // b
    &'a mut Vec<SupportedConeT<f64>>, // cones
);

/// (row, col, val, b, cones, q_object) for clarabel optimization
pub(crate) type ObjConsClarabel<'a> = (
    &'a mut Vec<usize>,               // row
    &'a mut Vec<usize>,               // col
    &'a mut Vec<f64>,                 // val
    &'a mut Vec<f64>,                 // b
    &'a mut Vec<SupportedConeT<f64>>, // cones
    &'a mut Vec<f64>,                 // q_object
);

/// Extract a nonnegative `a` profile from Clarabel solution vector with minimal copying.
/// - `s_len` is expected to be `problem.s_len()` (`problem`: [`Topp2Problem`](crate::solver::topp2_ra::Topp2Problem) or [`Copp2Problem`](crate::solver::copp2_socp::Copp2Problem));
/// - values are copied from `solution.x[..min(s_len, solution.x.len())]`;
/// - missing tail is zero-filled;
/// - all entries are clamped to `>= 0`.
///
/// This helper intentionally does **not** decide status acceptance; pair it with [`ClarabelOptions::is_allow`](crate::solver::copp2_socp::ClarabelOptions::is_allow) in caller logic.
#[inline]
pub fn clarabel_to_copp2_solution(s_len: usize, solution: &DefaultSolution<f64>) -> Vec<f64> {
    let mut a_res = vec![0.0; s_len];
    let copy_len = s_len.min(solution.x.len());
    a_res[..copy_len].copy_from_slice(&solution.x[..copy_len]);
    a_res.iter_mut().for_each(|v| *v = v.max(0.0));
    a_res
}

/// Extract a [`Topp3Profile`](crate::solver::topp3_lp::Topp3Profile) from Clarabel decision vector for TOPP3/COPP3.
/// - `solution_x` is expected in the layout `[a[0], ..., a[n], b[0], ..., b[n], x_others]`;
/// - `s` is expected to be the path-grid slice used by the TOPP3/COPP3 optimization interval;
/// - non-stationary segments are copied directly from `solution_x`;
/// - `num_stationary` is carried into the returned profile.
///
/// This helper intentionally does **not** decide status acceptance; pair it with [`ClarabelOptions::is_allow`](crate::solver::copp2_socp::ClarabelOptions::is_allow) in caller logic.
#[inline]
pub fn clarabel_to_copp3_solution(
    solution_x: &[f64],
    s: &[f64],
    num_stationary: (usize, usize),
) -> Topp3Profile {
    let n = s.len() - 1;
    let mut a = vec![0.0; n + 1];
    let mut b = vec![0.0; n + 1];
    if num_stationary.0 > 0 {
        set_ab_stationary_topp3::<true>(
            s,
            &mut a,
            &mut b,
            solution_x[num_stationary.0],
            num_stationary.0,
        );
    }
    if num_stationary.1 > 0 {
        set_ab_stationary_topp3::<false>(
            s,
            &mut a,
            &mut b,
            solution_x[n - num_stationary.1],
            num_stationary.1,
        );
    }
    a[num_stationary.0..=(n - num_stationary.1)]
        .copy_from_slice(&solution_x[num_stationary.0..=(n - num_stationary.1)]);
    b[num_stationary.0..=(n - num_stationary.1)]
        .copy_from_slice(&solution_x[(n + 1 + num_stationary.0)..=(2 * n + 1 - num_stationary.1)]);

    Topp3Profile::new(a, b, num_stationary)
}
