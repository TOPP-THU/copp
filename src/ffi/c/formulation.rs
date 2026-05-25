//! C ABI problem and objective descriptors shared by solver entry points.

use crate::copp::CoppObjective as RustCoppObjective;
use crate::ffi::c::core::status::clear_last_error;
use crate::ffi::c::robot::CoppTorqueForC;
use crate::ffi::c::{CoppRobot, CoppSliceF64, CoppStatus, CoppVecF64};
use crate::solver::copp3_socp::{
    Copp3Problem as RustCopp3Problem, Copp3ProblemBuilder as RustCopp3ProblemBuilder,
};
use crate::solver::topp3_lp::{
    Topp3Problem as RustTopp3Problem, Topp3ProblemBuilder as RustTopp3ProblemBuilder,
};
use std::slice;

/// COPP objective descriptor kind shared by second- and third-order solvers.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CoppObjectiveKind {
    /// Time objective: `weight * integral dt`.
    Time = 0,
    /// Linear objective over solver-specific profile variables.
    ///
    /// For COPP2, `alpha` is node-based over `a` and `beta` is interval-based
    /// over second-order `b`.
    /// For COPP3, both `alpha` and `beta` are node-based over third-order
    /// `a` and `b`.
    Linear = 1,
    /// Thermal-energy objective.
    ThermalEnergy = 2,
    /// Total-variation torque objective.
    TotalVariationTorque = 3,
}

/// Borrowed objective descriptor for COPP solvers.
///
/// For `Time`, only `weight` is used.
/// For `Linear`, `alpha` and `beta` are interpreted by each solver family:
/// COPP2 uses node-based `alpha` and interval-based `beta`, while COPP3 uses
/// node-based `alpha` and node-based `beta`.
/// For `ThermalEnergy` and `TotalVariationTorque`, `normalize` must have
/// length equal to the robot dimension. Torque is evaluated through the robot's
/// stored inverse-dynamics callback when provided, otherwise by the default
/// point model (`tau = ddq`).
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct CoppObjective {
    /// Objective kind.
    pub kind: CoppObjectiveKind,
    /// Objective weight.
    pub weight: f64,
    /// Linear objective coefficient for variable `a`.
    pub alpha: CoppSliceF64,
    /// Linear objective coefficient for variable `b`.
    pub beta: CoppSliceF64,
    /// Torque objective normalization vector.
    pub normalize: CoppSliceF64,
}

impl CoppObjective {
    pub(crate) unsafe fn as_rust_objective<'a>(
        &self,
        allow_total_variation_torque: bool,
    ) -> Result<RustCoppObjective<'a>, CoppStatus> {
        match self.kind {
            CoppObjectiveKind::Time => Ok(RustCoppObjective::Time(self.weight)),
            CoppObjectiveKind::Linear => {
                // SAFETY: The C ABI contract requires non-empty slices to
                // point to valid contiguous `double` arrays for this call.
                let alpha = unsafe { self.alpha.as_slice()? };
                // SAFETY: Same input-slice contract as above.
                let beta = unsafe { self.beta.as_slice()? };
                Ok(RustCoppObjective::Linear(self.weight, alpha, beta))
            }
            CoppObjectiveKind::ThermalEnergy => {
                // SAFETY: The C ABI contract requires non-empty slices to
                // point to valid contiguous `double` arrays for this call.
                let normalize = unsafe { self.normalize.as_slice()? };
                Ok(RustCoppObjective::ThermalEnergy(self.weight, normalize))
            }
            CoppObjectiveKind::TotalVariationTorque => {
                if !allow_total_variation_torque {
                    return Err(CoppStatus::InvalidArgument);
                }
                // SAFETY: The C ABI contract requires non-empty slices to
                // point to valid contiguous `double` arrays for this call.
                let normalize = unsafe { self.normalize.as_slice()? };
                Ok(RustCoppObjective::TotalVariationTorque(
                    self.weight,
                    normalize,
                ))
            }
        }
    }
}

/// Borrowed value descriptor for a COPP2 problem solved from C.
///
/// This is not an owning handle. `robot` and `objectives` are borrowed only
/// during the solver call, and the solver does not store those pointers after
/// the call returns.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct Copp2Problem {
    /// Robot handle that owns the station-indexed constraint storage.
    pub robot: *const CoppRobot,
    /// Closed start station index.
    pub idx_s_start: usize,
    /// Closed final station index.
    pub idx_s_final: usize,
    /// Boundary value `a(idx_s_start) = (ds/dt)^2`.
    pub a_start: f64,
    /// Boundary value `a(idx_s_final) = (ds/dt)^2`.
    pub a_final: f64,
    /// Objective descriptor array.
    pub objectives: *const CoppObjective,
    /// Number of objective descriptors.
    pub num_objectives: usize,
}

/// Borrowed value descriptor for a TOPP2 problem solved from C.
///
/// This is not an owning handle.  `robot` is borrowed only during TOPP2
/// solver calls such as `topp2_ra` and `copp_reach_set2_*`, and no pointer is
/// stored after the call returns.
///
/// Refactor it after the C solver flow is validated.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct Topp2Problem {
    /// Robot handle that owns the station-indexed constraint storage.
    pub robot: *const CoppRobot,
    /// Closed start station index.
    pub idx_s_start: usize,
    /// Closed final station index.
    pub idx_s_final: usize,
    /// Boundary value `a(idx_s_start) = (ds/dt)^2`.
    pub a_start: f64,
    /// Boundary value `a(idx_s_final) = (ds/dt)^2`.
    pub a_final: f64,
}

/// Owned third-order profile returned by TOPP3/COPP3 C ABI solvers.
///
/// `a[k]` stores `dot{s}_k^2`, `b[k]` stores `ddot{s}_k`, and
/// `num_stationary_*` stores the effective stationary head/tail interval
/// counts returned by the solver.  Both vectors are library-owned and must be
/// released together with `copp_profile_3rd_free`.
#[repr(C)]
#[derive(Debug)]
pub struct CoppProfile3rd {
    /// Node-based profile `a[k] = dot{s}_k^2`.
    pub a: CoppVecF64,
    /// Node-based profile `b[k] = ddot{s}_k`.
    pub b: CoppVecF64,
    /// Effective stationary interval count at the start.
    pub num_stationary_start: usize,
    /// Effective stationary interval count at the end.
    pub num_stationary_end: usize,
}

impl CoppProfile3rd {
    pub(crate) const fn empty() -> Self {
        Self {
            a: CoppVecF64::empty(),
            b: CoppVecF64::empty(),
            num_stationary_start: 0,
            num_stationary_end: 0,
        }
    }

    pub(crate) fn from_parts(a: Vec<f64>, b: Vec<f64>, num_stationary: (usize, usize)) -> Self {
        Self {
            a: CoppVecF64::from_vec(a),
            b: CoppVecF64::from_vec(b),
            num_stationary_start: num_stationary.0,
            num_stationary_end: num_stationary.1,
        }
    }

    pub(crate) fn free(self) {
        self.a.free();
        self.b.free();
    }
}

/// Release a third-order profile returned by COPP.
///
/// # Safety
/// `profile` must either be empty/null or have been returned by a successful
/// third-order C ABI solver call.  Passing arbitrary pointers or modified
/// capacity fields is invalid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_profile_3rd_free(profile: CoppProfile3rd) {
    profile.free();
    clear_last_error();
}

/// Borrowed value descriptor for a TOPP3 problem solved from C.
///
/// This is not an owning handle. `robot` and `a_linearization` are borrowed
/// only during the solver call.  Building the internal TOPP3 problem mutates the
/// robot's internal linearized third-order constraint cache.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct Topp3Problem {
    /// Mutable robot handle that owns station-indexed constraint storage.
    pub robot: *mut CoppRobot,
    /// Start station index. The final index is derived from
    /// `a_linearization.len - 1`.
    pub idx_s_start: usize,
    /// Reference profile `a[k]` used to linearize third-order constraints.
    pub a_linearization: CoppSliceF64,
    /// Boundary value `a(idx_s_start) = dot{s}^2`.
    pub a_start: f64,
    /// Boundary value `a(idx_s_final) = dot{s}^2`.
    pub a_final: f64,
    /// Boundary value `b(idx_s_start) = ddot{s}`.
    pub b_start: f64,
    /// Boundary value `b(idx_s_final) = ddot{s}`.
    pub b_final: f64,
    /// User-input stationary upper bound at the start.
    pub num_stationary_max_start: usize,
    /// User-input stationary upper bound at the end.
    pub num_stationary_max_end: usize,
    /// Denominator floor used when linearizing third-order constraints.
    pub a_linearization_floor: f64,
}

impl Topp3Problem {
    /// Convert this C descriptor into an internal TOPP3 problem for one solver call.
    ///
    /// # Safety
    /// `self.robot` must be a valid mutable `CoppRobot` handle and
    /// `self.a_linearization` must be valid for reads during the call.
    pub(crate) unsafe fn build_rust<'a>(self) -> Result<RustTopp3Problem<'a>, CoppStatus> {
        // SAFETY: The C ABI contract requires `robot` to be a live mutable
        // handle for the duration of this call.
        let robot = unsafe { CoppRobot::robot_mut(self.robot) }.ok_or(CoppStatus::NullPointer)?;
        // SAFETY: The C ABI contract requires non-empty slices to point to
        // valid contiguous `double` arrays for the duration of this call.
        let a_linearization = unsafe { self.a_linearization.as_slice()? };

        RustTopp3ProblemBuilder::new(
            robot,
            self.idx_s_start,
            a_linearization,
            (self.a_start, self.a_final),
            (self.b_start, self.b_final),
        )
        .with_num_stationary_max_pair((self.num_stationary_max_start, self.num_stationary_max_end))
        .with_a_linearization_floor(self.a_linearization_floor)
        .build_with_linearization()
        .map_err(|error| CoppStatus::from(&error))
    }
}

/// Borrowed value descriptor for a COPP3 problem solved from C.
///
/// This extends [`Topp3Problem`] with borrowed objective descriptors.  It is
/// not an owning handle; all pointers are used only during the solver call.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct Copp3Problem {
    /// Mutable robot handle that owns station-indexed constraint storage.
    pub robot: *mut CoppRobot,
    /// Start station index. The final index is derived from
    /// `a_linearization.len - 1`.
    pub idx_s_start: usize,
    /// Reference profile `a[k]` used to linearize third-order constraints.
    pub a_linearization: CoppSliceF64,
    /// Boundary value `a(idx_s_start) = dot{s}^2`.
    pub a_start: f64,
    /// Boundary value `a(idx_s_final) = dot{s}^2`.
    pub a_final: f64,
    /// Boundary value `b(idx_s_start) = ddot{s}`.
    pub b_start: f64,
    /// Boundary value `b(idx_s_final) = ddot{s}`.
    pub b_final: f64,
    /// User-input stationary upper bound at the start.
    pub num_stationary_max_start: usize,
    /// User-input stationary upper bound at the end.
    pub num_stationary_max_end: usize,
    /// Denominator floor used when linearizing third-order constraints.
    pub a_linearization_floor: f64,
    /// Objective descriptor array.
    pub objectives: *const CoppObjective,
    /// Number of objective descriptors.
    pub num_objectives: usize,
}

impl Copp3Problem {
    /// Convert borrowed C objective descriptors to internal objective descriptors.
    ///
    /// # Safety
    /// `self.objectives` must be valid for `self.num_objectives` reads when
    /// `self.num_objectives` is non-zero. Any non-empty objective slices must
    /// be valid for reads during the call.
    pub(crate) unsafe fn rust_objectives<'a>(
        self,
        allow_total_variation_torque: bool,
    ) -> Result<Vec<RustCoppObjective<'a>>, CoppStatus> {
        // SAFETY: Objective descriptors are borrowed only during this call.
        let objectives = unsafe { objective_slice(self.objectives, self.num_objectives)? };
        objectives
            .iter()
            .map(|objective| {
                // SAFETY: Objective descriptors and their borrowed slices are
                // used only during this call.
                unsafe { objective.as_rust_objective(allow_total_variation_torque) }
            })
            .collect()
    }

    /// Build an internal COPP3 problem from the C robot model and run `solve` immediately.
    ///
    /// The closure form avoids returning an internal problem that borrows the
    /// temporary converted objective vector.
    ///
    /// # Safety
    /// Same pointer and slice validity requirements as `Topp3Problem` plus
    /// the objective requirements documented by the objective conversion helper.
    pub(crate) unsafe fn with_rust_problem<R>(
        self,
        allow_total_variation_torque: bool,
        solve: impl for<'a> FnOnce(RustCopp3Problem<'a, CoppTorqueForC>) -> Result<R, CoppStatus>,
    ) -> Result<R, CoppStatus> {
        // SAFETY: The C ABI contract requires `robot` to be a live mutable
        // handle for the duration of this call.
        let robot = unsafe { CoppRobot::robot_mut(self.robot) }.ok_or(CoppStatus::NullPointer)?;
        // SAFETY: The C ABI contract requires non-empty slices to point to
        // valid contiguous `double` arrays for the duration of this call.
        let a_linearization = unsafe { self.a_linearization.as_slice()? };
        // SAFETY: Objective descriptors are borrowed only during this call.
        let objectives = unsafe { self.rust_objectives(allow_total_variation_torque)? };

        let problem = RustCopp3ProblemBuilder::new(
            robot,
            &objectives,
            self.idx_s_start,
            a_linearization,
            (self.a_start, self.a_final),
            (self.b_start, self.b_final),
        )
        .with_num_stationary_max_pair((self.num_stationary_max_start, self.num_stationary_max_end))
        .with_a_linearization_floor(self.a_linearization_floor)
        .build_with_linearization()
        .map_err(|error| CoppStatus::from(&error))?;

        solve(problem)
    }
}

/// # Safety
/// When `num_objectives > 0`, `objectives` must point to at least
/// `num_objectives` initialized `CoppObjective` values that remain valid for
/// the returned lifetime. The caller must choose a lifetime no longer than the
/// C-provided storage.
pub(crate) unsafe fn objective_slice<'a>(
    objectives: *const CoppObjective,
    num_objectives: usize,
) -> Result<&'a [CoppObjective], CoppStatus> {
    if num_objectives == 0 {
        return Ok(&[]);
    }
    if objectives.is_null() {
        return Err(CoppStatus::NullPointer);
    }

    // SAFETY: The caller guarantees that `objectives` is valid for
    // `num_objectives` reads when `num_objectives` is non-zero.
    Ok(unsafe { slice::from_raw_parts(objectives, num_objectives) })
}
