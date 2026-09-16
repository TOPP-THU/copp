//! Problem data models and builders for TOPP3/COPP3.
//!
//! # Notation policy (math + code)
//! To help users map paper notation to API fields without ambiguity,
//! this module follows a dual notation style:
//! - mathematical definition uses KaTeX, e.g. $a_k = \dot{s}_k^2$ and $b_k = \ddot{s}_k$;
//! - discrete implementation uses code symbols, e.g. `a[k]`, `b[k]`, `s[k]`.
//!
//! # Reference
//! Wang, Y., Hu, C., Li, Y., Yu, J., Yan, J., Liang, Y., & Jin, Z. (2026).
//! Online time-optimal trajectory planning along parametric toolpaths with strict constraint
//! satisfaction and certifiable feasibility guarantee.
//! *International Journal of Machine Tools and Manufacture*, 215, 104355.
//! <https://doi.org/10.1016/j.ijmachtools.2025.104355>
//!
//! # Stationary-boundary modeling note
//! This module exposes `num_stationary_max=(start,end)` on builders as a **user input upper
//! bound** and derives effective `num_stationary` during `build_with_linearization()`.
//! For typical users, `num_stationary_max=(1,1)` is recommended.
//!
//! For stationary boundaries (`a=b=0`), using zero stationary intervals is allowed, but the
//! boundary-time model can become ill-conditioned because the regular
//! $c=\frac{\dddot{s}}{\dot{s}}$-constant formulation degenerates near zero speed.
//! A short boundary neighborhood modeled by $\dddot{s}$-constant stationary intervals is the
//! practical remedy.

use crate::copp::CoppObjective;
use crate::copp::constraints::Constraints;
use crate::diag::{CoppError, check_boundary_state_copp3_valid, check_s_interval_valid};
use crate::robot::robot_core::{Robot, RobotBasic, RobotTorque};
use itertools::izip;

const DEFAULT_A_LINEARIZATION_FLOOR: f64 = 1E-10;
const DEFAULT_NUM_STATIONARY_MAX: (usize, usize) = (1, 1);

/// Where the third-order rows of a station take their linearization point.
///
/// The tangent of `1/sqrt(a)` lies below `1/sqrt(a)` for every positive anchor, so
/// both variants keep the linearized rows an inner approximation of the nonlinear
/// ones; they differ only in how tight that approximation is and in which `a` the
/// rows still admit.
///
/// Typical usage:
/// - Default to [`LinearizationModeTopp3::Direct`], which reproduces the historical
///   behaviour.
/// - [`LinearizationModeTopp3::GivenFeasibleAdaptive`] requires `a_linearization` to
///   be the `a` of a previously solved third-order profile and that profile's `b`
///   to be at hand; a lower-order profile discretizes `b` as
///   `(a[k+1] - a[k]) / (2 * ds[k])` rather than through
///   `a[k+1] = a[k] + (b[k] + b[k+1]) * ds[k]`, so its state cannot be used here.
/// - Given those, prefer it whenever `a` spans a wide range over the window, and
///   especially when `a` can reach the `1e-10` scale of `EPS_ZERO`.  The upper and
///   lower row of one axis differ only by an overall sign, so anchoring both at the
///   same small `a_linearization[k]` cancels `b` and `c` and leaves
///   `a[k] <= 3 * a_linearization[k]` -- a cap set by the anchor rather than by the
///   jerk limit, and one that no `b` or `c` can offset.  Adapting each row
///   separately breaks the cancellation.
/// - On a profile that stays well away from zero the two agree closely, and
///   [`LinearizationModeTopp3::Direct`] is the cheaper of the two.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum LinearizationModeTopp3<'a> {
    /// Direct:
    /// every row of a station is linearized at `a_linearization[k]`.
    #[default]
    Direct,
    /// GivenFeasibleAdaptive(b_linearization):
    /// each row is anchored where that row is nearly active at the feasible state
    /// `(a_linearization, b_linearization)`, so a slack row is anchored high and a
    /// tight one stays near the state.  `b_linearization` must be the path
    /// acceleration of the same profile as `a_linearization`, with the same length;
    /// it is normally taken from a previously computed feasible third-order
    /// profile.
    GivenFeasibleAdaptive(&'a [f64]),
}

/// Borrowed third-order TOPP/COPP profile parts.
///
/// This view keeps APIs lightweight for callers that already own separate
/// `a`/`b` slices while preserving the same semantic grouping as
/// [`Topp3Profile`]. The tuple layout is `(a, b, num_stationary)`.
pub type Topp3ProfileRef<'a> = (&'a [f64], &'a [f64], (usize, usize));

/// Mutable borrowed third-order TOPP/COPP profile parts.
///
/// This view is intended for in-place profile post-processing. The tuple
/// layout is `(a, b, num_stationary)`, where only the `a` and `b` slices are
/// mutable and the stationary boundary counts remain immutable metadata.
pub type Topp3ProfileMut<'a> = (&'a mut [f64], &'a mut [f64], (usize, usize));

/// Node-based third-order TOPP/COPP profile.
///
/// The three fields are tied together by convention: `a.len() == b.len()`,
/// and `num_stationary` describes stationary head/tail intervals on the same
/// path grid used with those node profiles.
#[derive(Clone, Debug, PartialEq)]
pub struct Topp3Profile {
    /// Node samples of $a_k = \dot{s}_k^2$.
    pub a: Vec<f64>,
    /// Node samples of $b_k = \ddot{s}_k$.
    pub b: Vec<f64>,
    /// Stationary boundary interval counts as `(head, tail)`.
    ///
    /// `head` counts stationary intervals at the start of the grid, and
    /// `tail` counts stationary intervals at the end. These markers are used
    /// by TOPP3/COPP3 interpolation and timing reconstruction.
    pub num_stationary: (usize, usize),
}

impl Topp3Profile {
    /// Build a third-order profile from owned parts.
    #[inline]
    pub fn new(a: Vec<f64>, b: Vec<f64>, num_stationary: (usize, usize)) -> Self {
        Self {
            a,
            b,
            num_stationary,
        }
    }

    /// Borrow the profile as `(a, b, num_stationary)` parts.
    #[inline]
    pub fn as_parts(&self) -> Topp3ProfileRef<'_> {
        (&self.a, &self.b, self.num_stationary)
    }

    /// Mutably borrow the profile as `(a, b, num_stationary)` parts.
    #[inline]
    pub fn as_parts_mut(&mut self) -> Topp3ProfileMut<'_> {
        (&mut self.a, &mut self.b, self.num_stationary)
    }

    /// Consume the profile and return `(a, b, num_stationary)`.
    #[inline]
    pub fn into_parts(self) -> (Vec<f64>, Vec<f64>, (usize, usize)) {
        (self.a, self.b, self.num_stationary)
    }
}

#[inline(always)]
pub(crate) fn boundary_state_is_stationary(a: f64, b: f64) -> bool {
    a == 0.0 && b == 0.0
}

#[inline(always)]
fn determine_num_stationary_side(a: f64, b: f64, num_stationary_max: usize) -> usize {
    if boundary_state_is_stationary(a, b) {
        num_stationary_max
    } else {
        0
    }
}

#[inline(always)]
fn determine_num_stationary_pair(
    a_boundary: (f64, f64),
    b_boundary: (f64, f64),
    num_stationary_max: (usize, usize),
) -> (usize, usize) {
    (
        determine_num_stationary_side(a_boundary.0, b_boundary.0, num_stationary_max.0),
        determine_num_stationary_side(a_boundary.1, b_boundary.1, num_stationary_max.1),
    )
}

/// Check that the window keeps more station intervals than its two boundary blocks.
///
/// Each boundary reserves `num_stationary.max(1)` intervals, so the window of
/// `s_len` stations must satisfy
/// `s_len - 1 > num_stationary.0.max(1) + num_stationary.1.max(1)`.
fn check_num_station_intervals(
    function_name: &str,
    s_len: usize,
    num_stationary: (usize, usize),
) -> Result<(), CoppError> {
    let num_intervals = s_len.saturating_sub(1);
    let reserved = num_stationary
        .0
        .max(1)
        .saturating_add(num_stationary.1.max(1));
    if num_intervals <= reserved {
        return Err(CoppError::InvalidInput(
            function_name.into(),
            format!(
                "{num_intervals} station intervals must exceed \
                 num_stationary.0.max(1) + num_stationary.1.max(1) = {reserved} \
                 (num_stationary = {num_stationary:?})"
            ),
        ));
    }
    Ok(())
}

/// Check that an adaptive linearization state pairs one `b` with every `a`.
fn check_linearization_mode(
    function_name: &str,
    a_linearization: &[f64],
    linearization_mode: LinearizationModeTopp3<'_>,
) -> Result<(), CoppError> {
    match linearization_mode {
        LinearizationModeTopp3::GivenFeasibleAdaptive(b_linearization)
            if b_linearization.len() != a_linearization.len() =>
        {
            Err(CoppError::InvalidInput(
                function_name.into(),
                format!(
                    "b_linearization.len() = {} must equal a_linearization.len() = {}",
                    b_linearization.len(),
                    a_linearization.len()
                ),
            ))
        }
        _ => Ok(()),
    }
}

/// Prepared TOPP3 problem view.
///
/// # Method identity
/// This is the **read-only runtime view** consumed by TOPP3/COPP3 solvers after
/// third-order constraints have been linearized.
///
/// # Important invariant
/// The builder precomputes linearized jerk buffers in [`Constraints`](crate::constraints::Constraints):
/// - `jerk_a_linear`
/// - `jerk_max_linear`
///
/// and this object subsequently holds only `&Constraints` (non-mutable view).
///
/// # Why this works
/// Original third-order inequality includes a nonlinear denominator term:
/// $$
/// \sqrt{a(s)}\left(g\_a(s) a(s) + g\_b(s) b(s) + g\_c(s) c(s) + g\_d(s)\right) \le g\_{\text{max}}(s).
/// $$
///
/// Around reference `a_linearization`, it is approximated into affine form:
/// $$
/// \left(g\_a(s) + \frac{g\_{\text{max}}(s)}{2a_{\text{lin}}^{3/2}}\right)a + g\_b(s) b + g\_c(s) c
/// \le \frac{3g\_{\text{max}}(s)}{2a_{\text{lin}}^{1/2}} - g\_d(s).
/// $$
/// where $a_{\text{lin}}$ corresponds to code input `a_linearization[k]`.
///
/// The affine coefficients are stored into those two buffers for downstream LP/SOCP/RA use.
pub struct Topp3Problem<'a> {
    pub(crate) constraints: &'a Constraints,
    pub(crate) idx_s_start: usize,
    pub(crate) a_linearization: &'a [f64],
    pub(crate) a_boundary: (f64, f64),
    pub(crate) b_boundary: (f64, f64),
    /// Effective stationary intervals at (start, end), derived in `build_with_linearization()` from
    /// boundary conditions and `num_stationary_max`.
    pub(crate) num_stationary: (usize, usize),
}

/// Builder for [`Topp3Problem`](crate::solver::topp3_socp::Topp3Problem), including optional in-build linearization.
///
/// # Side effect notice
/// [`build_with_linearization`](Topp3ProblemBuilder::build_with_linearization) updates
/// cached affine linearization data inside [`Constraints`](crate::constraints::Constraints).
/// Raw jerk constraints remain unchanged.
///
/// # Why mutable?
/// Third-order jerk rows are stored in nonconvex form after
/// [`Robot::with_axial_jerk`](crate::robot::Robot::with_axial_jerk) or
/// [`Constraints::with_constraint_3order`](crate::constraints::Constraints::with_constraint_3order).
/// [`build_with_linearization`](Topp3ProblemBuilder::build_with_linearization)
/// evaluates an affine approximation around
/// [`a_linearization`](Topp3ProblemBuilder::a_linearization) and
/// writes the generated coefficients into the constraint
/// cache. Rebuild the problem whenever the linearization profile changes.
///
/// # Example
/// The example below builds a TOPP3 problem and lets the builder linearize the third-order constraints.
///
/// ```rust
/// # fn main() -> Result<(), copp::diag::CoppError> {
/// use copp::robot::Robot;
/// use copp::solver::topp3_lp::Topp3ProblemBuilder;
///
/// let mut robot = Robot::with_capacity(2usize, 5);
/// let s = [0.0, 0.25, 0.5, 0.75, 1.0];
/// robot.with_s(s.as_slice())?;
///
/// let a_linearization = [0.0, 0.25, 0.25, 0.25, 0.0];
/// let _problem = Topp3ProblemBuilder::new(
///     &mut robot,
///     0,
///     &a_linearization,
///     (0.0, 0.0),
///     (0.0, 0.0),
/// )
/// .build_with_linearization()?;
/// # Ok(())
/// # }
/// ```
pub struct Topp3ProblemBuilder<'a> {
    /// Mutable constraint storage used to build linearized TOPP3 problem data.
    pub constraints: &'a mut Constraints,
    /// Start station index of the optimization interval.
    pub idx_s_start: usize,
    /// Reference profile `a[k]` used to linearize third-order constraints.
    pub a_linearization: &'a [f64],
    /// Boundary values of `a=(a_start, a_final)`.
    pub a_boundary: (f64, f64),
    /// Boundary values of `b=(b_start, b_final)`.
    pub b_boundary: (f64, f64),
    /// User-input upper bound of stationary intervals at (start, end).
    pub num_stationary_max: (usize, usize),
    /// Denominator floor for stable evaluation of `1/sqrt(a_linearization)` near `a=0`.
    ///
    /// Effective usage in linearization is:
    /// $$
    /// \frac{1}{\sqrt{\max(a_{lin}, a_{floor})}}.
    /// $$
    /// Discrete code form:
    /// `1.0 / max(a_linearization, a_linearization_floor).sqrt()`.
    ///
    /// More details are available in the [`Topp3Problem`](crate::solver::topp3_socp::Topp3Problem) documentation.
    pub a_linearization_floor: f64,

    /// Where the third-order rows take their linearization point.
    ///
    /// Defaults to [`LinearizationModeTopp3::Direct`], which reproduces the historical
    /// behaviour exactly.
    pub linearization_mode: LinearizationModeTopp3<'a>,
}

impl<'a> Topp3ProblemBuilder<'a> {
    /// Create a TOPP3 builder with required fields.
    ///
    /// Defaults:
    /// - `num_stationary_max = (1, 1)`
    /// - `a_linearization_floor = 1E-10`
    pub fn new<M: RobotBasic>(
        robot: &'a mut Robot<M>,
        idx_s_start: usize,
        a_linearization: &'a [f64],
        a_boundary: (f64, f64),
        b_boundary: (f64, f64),
    ) -> Self {
        Self {
            constraints: &mut robot.constraints,
            idx_s_start,
            a_linearization,
            a_boundary,
            b_boundary,
            num_stationary_max: DEFAULT_NUM_STATIONARY_MAX,
            a_linearization_floor: DEFAULT_A_LINEARIZATION_FLOOR,
            linearization_mode: LinearizationModeTopp3::Direct,
        }
    }

    /// Create a TOPP3 builder with required fields.
    ///
    /// Defaults:
    /// - `num_stationary_max = (1, 1)`
    /// - `a_linearization_floor = 1E-10`
    pub fn with_constraint(
        constraints: &'a mut Constraints,
        idx_s_start: usize,
        a_linearization: &'a [f64],
        a_boundary: (f64, f64),
        b_boundary: (f64, f64),
    ) -> Self {
        Self {
            constraints,
            idx_s_start,
            a_linearization,
            a_boundary,
            b_boundary,
            num_stationary_max: DEFAULT_NUM_STATIONARY_MAX,
            a_linearization_floor: DEFAULT_A_LINEARIZATION_FLOOR,
            linearization_mode: LinearizationModeTopp3::Direct,
        }
    }

    /// Set symmetric stationary upper bound: `num_stationary_max=(n,n)`.
    ///
    /// See module-level **Stationary-boundary modeling note** for guidance.
    #[inline]
    pub fn with_num_stationary_max(mut self, num_stationary_max: usize) -> Self {
        self.num_stationary_max = (num_stationary_max, num_stationary_max);
        self
    }

    /// Set asymmetric stationary upper bound: `num_stationary_max=(start,end)`.
    ///
    /// See module-level **Stationary-boundary modeling note** for guidance.
    #[inline]
    pub fn with_num_stationary_max_pair(mut self, num_stationary_max: (usize, usize)) -> Self {
        self.num_stationary_max = num_stationary_max;
        self
    }

    /// Set denominator floor used in third-order linearization near `a = 0`.
    ///
    /// [`build_with_linearization`](Self::build_with_linearization) applies
    /// `1.0 / max(a_linearization[k], floor).sqrt()` when converting nonlinear
    /// jerk rows into affine rows.
    #[inline]
    pub fn with_a_linearization_floor(mut self, floor: f64) -> Self {
        self.a_linearization_floor = floor;
        self
    }

    /// Choose where the third-order rows take their linearization point.
    ///
    /// The default is [`LinearizationModeTopp3::Direct`].  The conditions
    /// [`LinearizationModeTopp3::GivenFeasibleAdaptive`] requires of
    /// `a_linearization`, and when it is worth selecting over the default, are
    /// listed under *Typical usage* on [`LinearizationModeTopp3`].
    #[inline]
    pub fn with_linearization_mode(
        mut self,
        linearization_mode: LinearizationModeTopp3<'a>,
    ) -> Self {
        self.linearization_mode = linearization_mode;
        self
    }

    /// Build a TOPP3 problem and linearize third-order constraints in one step.
    ///
    /// This validates boundaries/interval/station count/floor first, then writes linearized jerk buffers
    /// inside [`Constraints`](crate::constraints::Constraints). The generated rows
    /// can be inspected with
    /// [`Constraints::get_jerk_linear_constraints`](crate::constraints::Constraints::get_jerk_linear_constraints).
    ///
    /// # Rebuilding rule
    /// If a later solver result is used as a new
    /// [`a_linearization`](Topp3ProblemBuilder::a_linearization), create a new
    /// builder and call [`build_with_linearization`](Topp3ProblemBuilder::build_with_linearization)
    /// again before solving.
    pub fn build_with_linearization(self) -> Result<Topp3Problem<'a>, CoppError> {
        check_boundary_state_copp3_valid(self.a_boundary, self.b_boundary)?;
        if self.a_linearization.is_empty() {
            return Err(CoppError::InvalidInput(
                "Topp3ProblemBuilder::build_with_linearization".into(),
                "a_linearization cannot be empty".into(),
            ));
        }
        let idx_s_final = self.idx_s_start + self.a_linearization.len() - 1;
        check_s_interval_valid(
            "Topp3ProblemBuilder::build_with_linearization",
            self.idx_s_start,
            idx_s_final,
        )?;
        if self.a_linearization_floor <= 0.0 {
            return Err(CoppError::InvalidInput(
                "Topp3ProblemBuilder::build_with_linearization".into(),
                format!(
                    "a_linearization_floor must be positive, got {}",
                    self.a_linearization_floor
                ),
            ));
        }

        let num_stationary = determine_num_stationary_pair(
            self.a_boundary,
            self.b_boundary,
            self.num_stationary_max,
        );
        check_num_station_intervals(
            "Topp3ProblemBuilder::build_with_linearization",
            self.a_linearization.len(),
            num_stationary,
        )?;
        check_linearization_mode(
            "Topp3ProblemBuilder::build_with_linearization",
            self.a_linearization,
            self.linearization_mode,
        )?;

        match self.linearization_mode {
            LinearizationModeTopp3::Direct => {
                self.constraints.linearize_constraint_3order_with_floor(
                    self.a_linearization,
                    self.idx_s_start,
                    self.a_linearization_floor,
                )
            }
            LinearizationModeTopp3::GivenFeasibleAdaptive(b_linearization) => {
                self.constraints.linearize_constraint_3order_adaptive(
                    self.a_linearization,
                    b_linearization,
                    self.idx_s_start,
                    self.a_linearization_floor,
                    num_stationary,
                )
            }
        }
        .map_err(|e| {
            CoppError::InvalidInput(
                "Topp3ProblemBuilder::build_with_linearization".into(),
                format!("linearize_constraint_3order failed: {e}"),
            )
        })?;

        Ok(Topp3Problem {
            constraints: &*self.constraints,
            idx_s_start: self.idx_s_start,
            a_linearization: self.a_linearization,
            a_boundary: self.a_boundary,
            b_boundary: self.b_boundary,
            num_stationary,
        })
    }
}

/// The problem of COPP3.
/// # Arguments
/// * `robot` - A robot with torque implemented, which defines the constraints and dynamic of the problem.
/// * `objectives` - Objectives for COPP3 optimization.
/// * `idx_s_start` - The starting index along the path (reached).
/// * `a_linearization` - Linearization reference profile for third-order constraints.
/// * `a_boundary=(a_start,a_final)` - The initial and final acceleration at the start and end of the path.
/// * `b_boundary=(b_start,b_final)` - The initial and final boundary conditions for `b`.
/// * `num_stationary=(start,end)` - Effective stationary intervals derived in `build_with_linearization()`.
pub struct Copp3Problem<'a, M: RobotTorque> {
    pub(crate) robot: &'a mut Robot<M>,
    pub(crate) objectives: &'a [CoppObjective<'a>],
    pub(crate) idx_s_start: usize,
    pub(crate) a_linearization: &'a [f64],
    pub(crate) a_boundary: (f64, f64),
    pub(crate) b_boundary: (f64, f64),
    /// Effective stationary intervals at (start, end), derived in `build_with_linearization()` from
    /// boundary conditions and `num_stationary_max`.
    pub(crate) num_stationary: (usize, usize),
}

/// Builder for [`Copp3Problem`](crate::solver::copp3_socp::Copp3Problem).
///
/// # Why mutable?
/// COPP3 uses the same third-order linearization cache as TOPP3. The builder
/// takes [`&mut Robot<M>`](crate::robot::RobotTorque) because
/// [`build_with_linearization`](Copp3ProblemBuilder::build_with_linearization) converts nonconvex
/// jerk rows into affine rows around [`a_linearization`](Copp3ProblemBuilder::a_linearization) and stores those rows in
/// [`robot.constraints`](crate::robot::Robot::constraints).
/// Rebuild the problem whenever the linearization profile changes.
///
/// # Example
/// The example below builds a COPP3 problem with time and thermal objectives, including the required third-order linearization profile.
///
/// ```rust
/// # fn main() -> Result<(), copp::diag::CoppError> {
/// use copp::robot::Robot;
/// use copp::solver::copp3_socp::{Copp3ProblemBuilder, CoppObjective};
///
/// let mut robot = Robot::with_capacity(2usize, 5);
/// let s = [0.0, 0.25, 0.5, 0.75, 1.0];
/// robot.with_s(s.as_slice())?;
///
/// let a_linearization = [0.0, 0.25, 0.25, 0.25, 0.0];
/// let normalize = [1.0, 1.0];
/// let objectives = [
///     CoppObjective::Time(1.0),
///     CoppObjective::ThermalEnergy(0.1, &normalize),
/// ];
///
/// let _problem = Copp3ProblemBuilder::new(
///     &mut robot,
///     &objectives,
///     0,
///     &a_linearization,
///     (0.0, 0.0),
///     (0.0, 0.0),
/// )
/// .build_with_linearization()?;
/// # Ok(())
/// # }
/// ```
pub struct Copp3ProblemBuilder<'a, M: RobotTorque> {
    /// A robot with torque implemented, which defines the constraints and dynamic of the problem.
    pub robot: &'a mut Robot<M>,
    /// Objectives for COPP3 optimization.
    pub objectives: &'a [CoppObjective<'a>],
    /// The starting index along the path (reached).
    pub idx_s_start: usize,
    /// Linearization reference profile for third-order constraints.
    pub a_linearization: &'a [f64],
    /// `a_boundary=(a_start,a_final)` - The initial and final acceleration at the start and end of the path.
    pub a_boundary: (f64, f64),
    /// `b_boundary=(b_start,b_final)` - The initial and final boundary conditions for `b`.
    pub b_boundary: (f64, f64),
    /// User-input upper bound of stationary intervals at (start, end).
    pub num_stationary_max: (usize, usize),
    /// Denominator floor for stable evaluation of `1/sqrt(a_linearization)` near `a=0`.
    ///
    /// Effective usage in linearization is:
    /// $$
    /// \frac{1}{\sqrt{\max(a_{lin}, a_{floor})}}.
    /// $$
    /// Discrete code form:
    /// `1.0 / max(a_linearization, a_linearization_floor).sqrt()`.
    ///
    /// More details are available in the [`Topp3Problem`](crate::solver::topp3_lp::Topp3Problem) documentation.
    pub a_linearization_floor: f64,

    /// Where the third-order rows take their linearization point.
    ///
    /// Defaults to [`LinearizationModeTopp3::Direct`], which reproduces the historical
    /// behaviour exactly.
    pub linearization_mode: LinearizationModeTopp3<'a>,
}

impl<'a, M: RobotTorque> Copp3ProblemBuilder<'a, M> {
    /// Create a COPP3 builder with required fields.
    ///
    /// Defaults:
    /// - `num_stationary_max = (1, 1)`
    /// - `a_linearization_floor = 1E-10`
    pub fn new(
        robot: &'a mut Robot<M>,
        objectives: &'a [CoppObjective<'a>],
        idx_s_start: usize,
        a_linearization: &'a [f64],
        a_boundary: (f64, f64),
        b_boundary: (f64, f64),
    ) -> Self {
        Self {
            robot,
            objectives,
            idx_s_start,
            a_linearization,
            a_boundary,
            b_boundary,
            num_stationary_max: DEFAULT_NUM_STATIONARY_MAX,
            a_linearization_floor: DEFAULT_A_LINEARIZATION_FLOOR,
            linearization_mode: LinearizationModeTopp3::Direct,
        }
    }

    /// Set symmetric stationary upper bound: `num_stationary_max=(n,n)`.
    ///
    /// See module-level **Stationary-boundary modeling note** for guidance.
    #[inline]
    pub fn with_num_stationary_max(mut self, num_stationary_max: usize) -> Self {
        self.num_stationary_max = (num_stationary_max, num_stationary_max);
        self
    }

    /// Set asymmetric stationary upper bound: `num_stationary_max=(start,end)`.
    ///
    /// See module-level **Stationary-boundary modeling note** for guidance.
    #[inline]
    pub fn with_num_stationary_max_pair(mut self, num_stationary_max: (usize, usize)) -> Self {
        self.num_stationary_max = num_stationary_max;
        self
    }

    /// Set denominator floor used in third-order linearization near `a = 0`.
    ///
    /// [`build_with_linearization`](Self::build_with_linearization) applies
    /// `1.0 / max(a_linearization[k], floor).sqrt()` when converting nonlinear
    /// jerk rows into affine rows.
    #[inline]
    pub fn with_a_linearization_floor(mut self, floor: f64) -> Self {
        self.a_linearization_floor = floor;
        self
    }

    /// Choose where the third-order rows take their linearization point.
    ///
    /// The default is [`LinearizationModeTopp3::Direct`].  The conditions
    /// [`LinearizationModeTopp3::GivenFeasibleAdaptive`] requires of
    /// `a_linearization`, and when it is worth selecting over the default, are
    /// listed under *Typical usage* on [`LinearizationModeTopp3`].
    #[inline]
    pub fn with_linearization_mode(
        mut self,
        linearization_mode: LinearizationModeTopp3<'a>,
    ) -> Self {
        self.linearization_mode = linearization_mode;
        self
    }

    /// Build a validated COPP3 problem and linearize third-order constraints in one step.
    ///
    /// This validates boundaries/interval/station count/floor first, then writes linearized jerk buffers
    /// inside [`Constraints`](crate::constraints::Constraints). The generated rows
    /// can be inspected with
    /// [`Constraints::get_jerk_linear_constraints`](crate::constraints::Constraints::get_jerk_linear_constraints).
    ///
    /// # Rebuilding rule
    /// If a later solver result is used as a new [`a_linearization`](Copp3ProblemBuilder::a_linearization), create a new
    /// builder and call [`build_with_linearization`](Copp3ProblemBuilder::build_with_linearization) again before solving.
    pub fn build_with_linearization(self) -> Result<Copp3Problem<'a, M>, CoppError> {
        check_boundary_state_copp3_valid(self.a_boundary, self.b_boundary)?;
        if self.a_linearization.is_empty() {
            return Err(CoppError::InvalidInput(
                "Copp3ProblemBuilder::build_with_linearization".into(),
                "a_linearization cannot be empty".into(),
            ));
        }
        let idx_s_final = self.idx_s_start + self.a_linearization.len() - 1;
        check_s_interval_valid(
            "Copp3ProblemBuilder::build_with_linearization",
            self.idx_s_start,
            idx_s_final,
        )?;
        self.robot
            .constraints
            .check_s_in_bounds(self.idx_s_start, self.a_linearization.len())?;
        if self.a_linearization_floor <= 0.0 {
            return Err(CoppError::InvalidInput(
                "Copp3ProblemBuilder::build_with_linearization".into(),
                format!(
                    "a_linearization_floor must be positive, got {}",
                    self.a_linearization_floor
                ),
            ));
        }

        let num_stationary = determine_num_stationary_pair(
            self.a_boundary,
            self.b_boundary,
            self.num_stationary_max,
        );
        check_num_station_intervals(
            "Copp3ProblemBuilder::build_with_linearization",
            self.a_linearization.len(),
            num_stationary,
        )?;
        check_linearization_mode(
            "Copp3ProblemBuilder::build_with_linearization",
            self.a_linearization,
            self.linearization_mode,
        )?;

        match self.linearization_mode {
            LinearizationModeTopp3::Direct => self
                .robot
                .constraints
                .linearize_constraint_3order_with_floor(
                    self.a_linearization,
                    self.idx_s_start,
                    self.a_linearization_floor,
                ),
            LinearizationModeTopp3::GivenFeasibleAdaptive(b_linearization) => {
                self.robot.constraints.linearize_constraint_3order_adaptive(
                    self.a_linearization,
                    b_linearization,
                    self.idx_s_start,
                    self.a_linearization_floor,
                    num_stationary,
                )
            }
        }
        .map_err(|e| {
            CoppError::InvalidInput(
                "Copp3ProblemBuilder::build_with_linearization".into(),
                format!("linearize_constraint_3order failed: {e}"),
            )
        })?;

        Ok(Copp3Problem {
            robot: self.robot,
            objectives: self.objectives,
            idx_s_start: self.idx_s_start,
            a_linearization: self.a_linearization,
            a_boundary: self.a_boundary,
            b_boundary: self.b_boundary,
            num_stationary,
        })
    }
}

impl<'a, M: RobotTorque> Copp3Problem<'a, M> {
    /// Update linearization profile.
    #[inline]
    pub fn set_a_linearization(&mut self, a_linearization: &'a [f64]) {
        self.a_linearization = a_linearization;
    }

    /// Backward-compatible alias for `set_a_linearization`.
    #[inline]
    pub fn set_a_linear(&mut self, a_linearization: &'a [f64]) {
        self.set_a_linearization(a_linearization);
    }

    /// Update objective list.
    #[inline]
    pub fn set_objective(&mut self, objective: &'a [CoppObjective<'a>]) {
        self.objectives = objective;
    }

    /// Convert to the TOPP3 view that shares interval/boundary/linearization fields.
    pub fn as_topp3_problem(&self) -> Topp3Problem<'_> {
        Topp3Problem {
            constraints: &self.robot.constraints,
            idx_s_start: self.idx_s_start,
            a_linearization: self.a_linearization,
            a_boundary: self.a_boundary,
            b_boundary: self.b_boundary,
            num_stationary: self.num_stationary,
        }
    }
}

/// Get the weight of `a` for value function.
/// Time loss = \sum_{k=0}^n weight_a[k] / sqrt(a[k]) \approx Time
pub(crate) fn get_weight_a_topp3(s: &[f64], num_stationary: (usize, usize)) -> Vec<f64> {
    let n = s.len() - 1;
    let mut weight_a = vec![0.0; s.len()];
    if num_stationary.0 > 0 {
        weight_a[num_stationary.0] =
            0.5 * (5.0 * s[num_stationary.0] + s[num_stationary.0 + 1] - 6.0 * s[0]);
    }
    if num_stationary.1 > 0 {
        weight_a[n - num_stationary.1] =
            0.5 * (6.0 * s[n] - 5.0 * s[n - num_stationary.1] - s[n - num_stationary.1 - 1]);
    }
    weight_a
        .iter_mut()
        .skip(1)
        .zip(s.windows(3))
        .skip(num_stationary.0)
        .take(n - num_stationary.0 - num_stationary.1 - 1)
        .for_each(|(w_a, s_slice)| {
            *w_a = 0.5 * (s_slice[2] - s_slice[0]);
        });

    weight_a
}

/// Get the weight of `a` for value function.
/// Loss = weight[0] * loss_average_left / sqrt(a[num_stationary.0]) + weight[n] * loss_average_right / sqrt(a[n - num_stationary.1]) + \sum_{k=num_stationary.0}^{n-num_stationary.1-1} weight_a[k] * loss[k] / sqrt(a[k])
pub(crate) fn get_weight_a_copp3(s: &[f64], num_stationary: (usize, usize)) -> Vec<f64> {
    let n = s.len() - 1;
    let mut weight_a = vec![0.0; s.len()];
    if num_stationary.0 > 0 {
        let s_n1 = s[num_stationary.0];
        weight_a[num_stationary.0] = 0.5 * (s[num_stationary.0 + 1] - s_n1);
        weight_a[0] = 3.0 * (s_n1 - s[0]);
    }
    if num_stationary.1 > 0 {
        let s_n2 = s[n - num_stationary.1];
        weight_a[n - num_stationary.1] = 0.5 * (s_n2 - s[n - num_stationary.1 - 1]);
        weight_a[n] = 3.0 * (s[n] - s_n2);
    }
    weight_a
        .iter_mut()
        .skip(1)
        .zip(s.windows(3))
        .skip(num_stationary.0)
        .take(n - num_stationary.0 - num_stationary.1 - 1)
        .for_each(|(w_a, s_slice)| {
            *w_a = 0.5 * (s_slice[2] - s_slice[0]);
        });

    weight_a
}

/// Get the `a` and `b` profiles for stationary intervals.
pub(crate) fn set_ab_stationary_topp3<const START: bool>(
    s: &[f64],
    a: &mut [f64],
    b: &mut [f64],
    a_stationary: f64,
    num_stationary: usize,
) {
    if num_stationary == 0 {
        return;
    }
    if START {
        let s_start = s[0];
        let ds_start = s[num_stationary] - s_start;
        *a.first_mut().unwrap() = 0.0;
        *b.first_mut().unwrap() = 0.0;
        a[num_stationary] = a_stationary;
        b[num_stationary] = a_stationary / (1.5 * ds_start);
        if num_stationary > 1 {
            for (a_k, b_k, &s_k) in izip!(a.iter_mut(), b.iter_mut(), s.iter())
                .skip(1)
                .take(num_stationary - 1)
            {
                let dsk_start = s_k - s_start;
                let mut alpha = dsk_start / ds_start;
                alpha *= alpha.cbrt();
                *a_k = a_stationary * alpha;
                *b_k = *a_k / (1.5 * dsk_start);
            }
        }
    } else {
        let &s_final = s.last().unwrap();
        let n = s.len() - 1;
        let ds_final = s[n - num_stationary] - s_final;
        *a.last_mut().unwrap() = 0.0;
        *b.last_mut().unwrap() = 0.0;
        a[a.len() - 1 - num_stationary] = a_stationary;
        b[b.len() - 1 - num_stationary] = a_stationary / (1.5 * ds_final);
        if num_stationary > 1 {
            for (a_k, b_k, &s_k) in izip!(a.iter_mut().rev(), b.iter_mut().rev(), s.iter().rev())
                .skip(1)
                .take(num_stationary - 1)
            {
                let dsk_final = s_k - s_final;
                let mut alpha = dsk_final / ds_final;
                alpha *= alpha.cbrt();
                *a_k = a_stationary * alpha;
                *b_k = *a_k / (1.5 * dsk_final);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{
        LinearizationModeTopp3, boundary_state_is_stationary, check_linearization_mode,
        check_num_station_intervals, determine_num_stationary_pair,
    };

    #[test]
    fn test_determine_num_stationary_pair_respects_boundary_state() {
        let pair = determine_num_stationary_pair((0.0, 0.0), (0.0, 0.0), (2, 3));
        assert_eq!(pair, (2, 3));

        let pair = determine_num_stationary_pair((1.0, 0.0), (0.0, 0.0), (2, 3));
        assert_eq!(pair, (0, 3));

        let pair = determine_num_stationary_pair((0.0, 1.0), (0.0, 1.0), (2, 3));
        assert_eq!(pair, (2, 0));

        let tiny_a = f64::from_bits(1);
        let tiny_b = f64::from_bits(2);
        let pair = determine_num_stationary_pair((tiny_a, tiny_a), (tiny_b, tiny_b), (2, 3));
        assert_eq!(pair, (0, 0));
        assert!(!boundary_state_is_stationary(tiny_a, tiny_b));
        assert!(boundary_state_is_stationary(-0.0, 0.0));
    }

    #[test]
    fn test_check_num_station_intervals_reserves_both_boundary_blocks() {
        // Each side reserves `num_stationary.max(1)` intervals.
        assert!(check_num_station_intervals("test", 4, (0, 0)).is_ok());
        assert!(check_num_station_intervals("test", 3, (0, 0)).is_err());
        assert!(check_num_station_intervals("test", 4, (1, 1)).is_ok());
        assert!(check_num_station_intervals("test", 3, (1, 1)).is_err());
        assert!(check_num_station_intervals("test", 6, (1, 3)).is_ok());
        assert!(check_num_station_intervals("test", 5, (1, 3)).is_err());
        assert!(check_num_station_intervals("test", 0, (0, 0)).is_err());
        assert!(check_num_station_intervals("test", usize::MAX, (usize::MAX, 1)).is_err());
    }

    #[test]
    fn test_check_linearization_mode_requires_one_b_per_a() {
        let a = [0.25; 5];
        let b = [0.0; 5];
        let b_short = [0.0; 4];
        assert!(check_linearization_mode("test", &a, LinearizationModeTopp3::Direct).is_ok());
        assert!(
            check_linearization_mode(
                "test",
                &a,
                LinearizationModeTopp3::GivenFeasibleAdaptive(&b)
            )
            .is_ok()
        );
        assert!(
            check_linearization_mode(
                "test",
                &a,
                LinearizationModeTopp3::GivenFeasibleAdaptive(&b_short)
            )
            .is_err()
        );
    }
}
