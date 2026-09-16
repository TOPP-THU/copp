//! C ABI wrappers for path construction, evaluation, and robot sampling.

use crate::ffi::c::core::status::{clear_last_error, panic_to_status};
use crate::ffi::c::{
    CoppMatrixF64, CoppMatrixViewF64, CoppRobot, CoppSliceF64, CoppSliceUsize, CoppStatus,
    CoppVecF64, CoppVecUsize,
};
use crate::path::{
    Jet3, OutOfRangeMode, Parametrization, Path, PathEvaluator2nd, PathEvaluator3rd,
    SmoothingConfig, SmoothingReport, SmoothingTolerance, SplineConfig,
};
use nalgebra::DMatrix;
use std::{
    ffi::c_void,
    panic::{AssertUnwindSafe, catch_unwind},
    ptr,
    sync::Mutex,
};

/// Opaque C handle for a library-owned `Path`.
///
/// Create with `copp_path_from_waypoints_interpolating`,
/// `copp_path_from_waypoints_fitting`, `copp_path_from_parametric`,
/// `copp_path_from_evaluator_2nd`, or `copp_path_from_evaluator_3rd` and
/// release exactly once with `copp_path_free`. C callers must not inspect or
/// allocate this type directly.
pub struct CoppPath;

struct CoppPathInner {
    path: Path,
}

/// C-compatible third-order automatic-differentiation scalar.
///
/// `v` stores the scalar value, while `d1`, `d2`, and `d3` store derivatives
/// with respect to the path parameter `s`. Parametric path callbacks receive a
/// seeded value (`d1 = 1`) and should return one `CoppJet3` per path dimension.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct CoppJet3 {
    /// Function value.
    pub v: f64,
    /// First derivative with respect to `s`.
    pub d1: f64,
    /// Second derivative with respect to `s`.
    pub d2: f64,
    /// Third derivative with respect to `s`.
    pub d3: f64,
}

impl From<Jet3> for CoppJet3 {
    #[inline(always)]
    fn from(value: Jet3) -> Self {
        Self {
            v: value.v,
            d1: value.d1,
            d2: value.d2,
            d3: value.d3,
        }
    }
}

impl From<CoppJet3> for Jet3 {
    #[inline(always)]
    fn from(value: CoppJet3) -> Self {
        Self {
            v: value.v,
            d1: value.d1,
            d2: value.d2,
            d3: value.d3,
        }
    }
}

/// C callback for evaluating a scalar-parametric path with automatic derivatives.
///
/// The callback receives one seeded [`CoppJet3`] path parameter and must write
/// `dim` [`CoppJet3`] values into `q`. COPP extracts each returned value and
/// its first three derivatives to serve the existing path evaluation and robot
/// sampling APIs.
pub type CoppPathParametricFn = Option<
    unsafe extern "C" fn(
        user_data: *mut c_void,
        dim: usize,
        s: CoppJet3,
        q: *mut CoppJet3,
    ) -> CoppStatus,
>;

/// C callback for evaluating `q`, `dq`, and `ddq`.
///
/// The callback receives `n` path samples in `s` and must write column-major
/// `dim x n` matrices into `q`, `dq`, and `ddq`.
pub type CoppPathEvaluate2ndFn = Option<
    unsafe extern "C" fn(
        user_data: *mut c_void,
        dim: usize,
        n: usize,
        s: *const f64,
        q: *mut f64,
        dq: *mut f64,
        ddq: *mut f64,
    ) -> CoppStatus,
>;

/// C callback for evaluating `q`, `dq`, `ddq`, and `dddq`.
///
/// The callback receives `n` path samples in `s` and must write column-major
/// `dim x n` matrices into `q`, `dq`, `ddq`, and `dddq`.
pub type CoppPathEvaluate3rdFn = Option<
    unsafe extern "C" fn(
        user_data: *mut c_void,
        dim: usize,
        n: usize,
        s: *const f64,
        q: *mut f64,
        dq: *mut f64,
        ddq: *mut f64,
        dddq: *mut f64,
    ) -> CoppStatus,
>;

#[derive(Default)]
struct CallbackScratch {
    dq: Vec<f64>,
    ddq: Vec<f64>,
    dddq: Vec<f64>,
    q_jet: Vec<CoppJet3>,
}

struct CoppParametricPathEvaluator {
    dim: usize,
    evaluate: unsafe extern "C" fn(
        user_data: *mut c_void,
        dim: usize,
        s: CoppJet3,
        q: *mut CoppJet3,
    ) -> CoppStatus,
    user_data: *mut c_void,
    scratch: Mutex<CallbackScratch>,
}

// SAFETY: Calls into the C callback are serialized by `scratch`. The callback
// and `user_data` must remain valid until `copp_path_free`; any concurrent
// access to the pointed-to user data outside COPP is the caller's responsibility.
unsafe impl Send for CoppParametricPathEvaluator {}
// SAFETY: Same reasoning as `Send`; shared evaluator references serialize
// callback calls before touching the opaque C user data.
unsafe impl Sync for CoppParametricPathEvaluator {}

impl CoppParametricPathEvaluator {
    #[inline(always)]
    fn check_output_len(
        &self,
        s: &[f64],
        buffers: &[&[f64]],
    ) -> Result<(), crate::diag::PathError> {
        let expected = self.dim * s.len();
        if buffers.iter().any(|buffer| buffer.len() != expected) {
            return Err(crate::diag::PathError::DimensionMismatch);
        }
        Ok(())
    }

    #[inline(always)]
    fn scratch(&self) -> std::sync::MutexGuard<'_, CallbackScratch> {
        self.scratch
            .lock()
            .unwrap_or_else(|_| std::panic::panic_any(CoppStatus::Panic))
    }

    fn evaluate_common(
        &self,
        s: &[f64],
        q: &mut [f64],
        mut dq: Option<&mut [f64]>,
        mut ddq: Option<&mut [f64]>,
        mut dddq: Option<&mut [f64]>,
    ) -> Result<(), crate::diag::PathError> {
        {
            let mut buffers = Vec::with_capacity(4);
            buffers.push(&*q);
            if let Some(buffer) = dq.as_deref() {
                buffers.push(buffer);
            }
            if let Some(buffer) = ddq.as_deref() {
                buffers.push(buffer);
            }
            if let Some(buffer) = dddq.as_deref() {
                buffers.push(buffer);
            }
            self.check_output_len(s, &buffers)?;
        }

        let mut scratch = self.scratch();
        scratch.q_jet.resize(self.dim, CoppJet3::default());

        for (col, &s_value) in s.iter().enumerate() {
            let seed = CoppJet3::from(Jet3::seed(s_value));
            // SAFETY: The C ABI contract requires the callback to remain valid
            // until `copp_path_free` and to write exactly `dim` `CoppJet3`
            // values into `q`.
            let status = unsafe {
                (self.evaluate)(self.user_data, self.dim, seed, scratch.q_jet.as_mut_ptr())
            };
            CoppCallbackPathEvaluator2nd::callback_status(status)?;

            for row in 0..self.dim {
                let idx = row + col * self.dim;
                let jet = scratch.q_jet[row];
                q[idx] = jet.v;
                if let Some(buffer) = dq.as_deref_mut() {
                    buffer[idx] = jet.d1;
                }
                if let Some(buffer) = ddq.as_deref_mut() {
                    buffer[idx] = jet.d2;
                }
                if let Some(buffer) = dddq.as_deref_mut() {
                    buffer[idx] = jet.d3;
                }
            }
        }

        Ok(())
    }
}

impl PathEvaluator2nd for CoppParametricPathEvaluator {
    #[inline(always)]
    fn dim(&self) -> usize {
        self.dim
    }

    fn evaluate_q(&self, s: &[f64], q: &mut [f64]) -> Result<(), crate::diag::PathError> {
        self.evaluate_common(s, q, None, None, None)
    }

    fn evaluate_up_to_2nd(
        &self,
        s: &[f64],
        q: &mut [f64],
        dq: &mut [f64],
        ddq: &mut [f64],
    ) -> Result<(), crate::diag::PathError> {
        self.evaluate_common(s, q, Some(dq), Some(ddq), None)
    }
}

impl PathEvaluator3rd for CoppParametricPathEvaluator {
    fn evaluate_up_to_3rd(
        &self,
        s: &[f64],
        q: &mut [f64],
        dq: &mut [f64],
        ddq: &mut [f64],
        dddq: &mut [f64],
    ) -> Result<(), crate::diag::PathError> {
        self.evaluate_common(s, q, Some(dq), Some(ddq), Some(dddq))
    }
}

struct CoppCallbackPathEvaluator2nd {
    dim: usize,
    evaluate_2nd: unsafe extern "C" fn(
        user_data: *mut c_void,
        dim: usize,
        n: usize,
        s: *const f64,
        q: *mut f64,
        dq: *mut f64,
        ddq: *mut f64,
    ) -> CoppStatus,
    user_data: *mut c_void,
    scratch: Mutex<CallbackScratch>,
}

// SAFETY: Calls into the C callback are serialized by `scratch`. The C ABI
// contract requires the callback and `user_data` to remain valid until
// `copp_path_free`; concurrent use of the pointed-to user data outside COPP
// is still the caller's responsibility.
unsafe impl Send for CoppCallbackPathEvaluator2nd {}
// SAFETY: Same reasoning as `Send`; shared references serialize callback calls
// before touching the opaque C user data.
unsafe impl Sync for CoppCallbackPathEvaluator2nd {}

impl CoppCallbackPathEvaluator2nd {
    #[inline(always)]
    fn check_output_len(
        &self,
        s: &[f64],
        buffers: &[&[f64]],
    ) -> Result<(), crate::diag::PathError> {
        let expected = self.dim * s.len();
        if buffers.iter().any(|buffer| buffer.len() != expected) {
            return Err(crate::diag::PathError::DimensionMismatch);
        }
        Ok(())
    }

    #[inline(always)]
    fn scratch(&self) -> std::sync::MutexGuard<'_, CallbackScratch> {
        self.scratch
            .lock()
            .unwrap_or_else(|_| std::panic::panic_any(CoppStatus::Panic))
    }

    #[inline(always)]
    fn callback_status(status: CoppStatus) -> Result<(), crate::diag::PathError> {
        if status == CoppStatus::Ok {
            Ok(())
        } else {
            std::panic::panic_any(status)
        }
    }
}

impl PathEvaluator2nd for CoppCallbackPathEvaluator2nd {
    #[inline(always)]
    fn dim(&self) -> usize {
        self.dim
    }

    fn evaluate_q(&self, s: &[f64], q: &mut [f64]) -> Result<(), crate::diag::PathError> {
        self.check_output_len(s, &[q])?;
        let mut scratch = self.scratch();
        scratch.dq.resize(q.len(), 0.0);
        scratch.ddq.resize(q.len(), 0.0);
        // SAFETY: The C ABI contract requires the callback to remain valid
        // until `copp_path_free` and to write exactly the provided buffers.
        let status = unsafe {
            (self.evaluate_2nd)(
                self.user_data,
                self.dim,
                s.len(),
                s.as_ptr(),
                q.as_mut_ptr(),
                scratch.dq.as_mut_ptr(),
                scratch.ddq.as_mut_ptr(),
            )
        };
        Self::callback_status(status)
    }

    fn evaluate_up_to_2nd(
        &self,
        s: &[f64],
        q: &mut [f64],
        dq: &mut [f64],
        ddq: &mut [f64],
    ) -> Result<(), crate::diag::PathError> {
        self.check_output_len(s, &[q, dq, ddq])?;
        let _scratch = self.scratch();
        // SAFETY: The C ABI contract requires the callback to remain valid
        // until `copp_path_free` and to write exactly the provided buffers.
        let status = unsafe {
            (self.evaluate_2nd)(
                self.user_data,
                self.dim,
                s.len(),
                s.as_ptr(),
                q.as_mut_ptr(),
                dq.as_mut_ptr(),
                ddq.as_mut_ptr(),
            )
        };
        Self::callback_status(status)
    }
}

struct CoppCallbackPathEvaluator3rd {
    dim: usize,
    evaluate_2nd: CoppPathEvaluate2ndFn,
    evaluate_3rd: unsafe extern "C" fn(
        user_data: *mut c_void,
        dim: usize,
        n: usize,
        s: *const f64,
        q: *mut f64,
        dq: *mut f64,
        ddq: *mut f64,
        dddq: *mut f64,
    ) -> CoppStatus,
    user_data: *mut c_void,
    scratch: Mutex<CallbackScratch>,
}

// SAFETY: Callback calls are serialized by `scratch`, with the same C ABI
// validity requirements as `CoppCallbackPathEvaluator2nd`.
unsafe impl Send for CoppCallbackPathEvaluator3rd {}
// SAFETY: Same reasoning as `Send`.
unsafe impl Sync for CoppCallbackPathEvaluator3rd {}

impl CoppCallbackPathEvaluator3rd {
    #[inline(always)]
    fn check_output_len(
        &self,
        s: &[f64],
        buffers: &[&[f64]],
    ) -> Result<(), crate::diag::PathError> {
        let expected = self.dim * s.len();
        if buffers.iter().any(|buffer| buffer.len() != expected) {
            return Err(crate::diag::PathError::DimensionMismatch);
        }
        Ok(())
    }

    #[inline(always)]
    fn scratch(&self) -> std::sync::MutexGuard<'_, CallbackScratch> {
        self.scratch
            .lock()
            .unwrap_or_else(|_| std::panic::panic_any(CoppStatus::Panic))
    }
}

impl PathEvaluator2nd for CoppCallbackPathEvaluator3rd {
    #[inline(always)]
    fn dim(&self) -> usize {
        self.dim
    }

    fn evaluate_q(&self, s: &[f64], q: &mut [f64]) -> Result<(), crate::diag::PathError> {
        self.check_output_len(s, &[q])?;
        let mut scratch = self.scratch();
        scratch.dq.resize(q.len(), 0.0);
        scratch.ddq.resize(q.len(), 0.0);
        let status = if let Some(evaluate_2nd) = self.evaluate_2nd {
            // SAFETY: The C ABI contract requires the callback to remain valid
            // until `copp_path_free` and to write exactly the provided buffers.
            unsafe {
                evaluate_2nd(
                    self.user_data,
                    self.dim,
                    s.len(),
                    s.as_ptr(),
                    q.as_mut_ptr(),
                    scratch.dq.as_mut_ptr(),
                    scratch.ddq.as_mut_ptr(),
                )
            }
        } else {
            scratch.dddq.resize(q.len(), 0.0);
            // SAFETY: Same callback contract as above; derivative buffers are
            // temporary because the caller only requested position.
            unsafe {
                (self.evaluate_3rd)(
                    self.user_data,
                    self.dim,
                    s.len(),
                    s.as_ptr(),
                    q.as_mut_ptr(),
                    scratch.dq.as_mut_ptr(),
                    scratch.ddq.as_mut_ptr(),
                    scratch.dddq.as_mut_ptr(),
                )
            }
        };
        CoppCallbackPathEvaluator2nd::callback_status(status)
    }

    fn evaluate_up_to_2nd(
        &self,
        s: &[f64],
        q: &mut [f64],
        dq: &mut [f64],
        ddq: &mut [f64],
    ) -> Result<(), crate::diag::PathError> {
        self.check_output_len(s, &[q, dq, ddq])?;
        let mut scratch = self.scratch();
        let status = if let Some(evaluate_2nd) = self.evaluate_2nd {
            // SAFETY: The C ABI contract requires the callback to remain valid
            // until `copp_path_free` and to write exactly the provided buffers.
            unsafe {
                evaluate_2nd(
                    self.user_data,
                    self.dim,
                    s.len(),
                    s.as_ptr(),
                    q.as_mut_ptr(),
                    dq.as_mut_ptr(),
                    ddq.as_mut_ptr(),
                )
            }
        } else {
            scratch.dddq.resize(q.len(), 0.0);
            // SAFETY: Same callback contract as above; `dddq` is a temporary
            // buffer because the caller only requested derivatives up to 2nd.
            unsafe {
                (self.evaluate_3rd)(
                    self.user_data,
                    self.dim,
                    s.len(),
                    s.as_ptr(),
                    q.as_mut_ptr(),
                    dq.as_mut_ptr(),
                    ddq.as_mut_ptr(),
                    scratch.dddq.as_mut_ptr(),
                )
            }
        };
        CoppCallbackPathEvaluator2nd::callback_status(status)
    }
}

impl PathEvaluator3rd for CoppCallbackPathEvaluator3rd {
    fn evaluate_up_to_3rd(
        &self,
        s: &[f64],
        q: &mut [f64],
        dq: &mut [f64],
        ddq: &mut [f64],
        dddq: &mut [f64],
    ) -> Result<(), crate::diag::PathError> {
        self.check_output_len(s, &[q, dq, ddq, dddq])?;
        let _scratch = self.scratch();
        // SAFETY: The C ABI contract requires the callback to remain valid
        // until `copp_path_free` and to write exactly the provided buffers.
        let status = unsafe {
            (self.evaluate_3rd)(
                self.user_data,
                self.dim,
                s.len(),
                s.as_ptr(),
                q.as_mut_ptr(),
                dq.as_mut_ptr(),
                ddq.as_mut_ptr(),
                dddq.as_mut_ptr(),
            )
        };
        CoppCallbackPathEvaluator2nd::callback_status(status)
    }
}

/// C ABI path out-of-range behavior.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CoppPathOutOfRangeMode {
    /// Return an error when a query parameter is outside `[s_min, s_max]`.
    Error = 0,
    /// Clamp query parameters into `[s_min, s_max]`.
    Clamp = 1,
}

impl TryFrom<CoppPathOutOfRangeMode> for OutOfRangeMode {
    type Error = CoppStatus;

    fn try_from(mode: CoppPathOutOfRangeMode) -> Result<Self, CoppStatus> {
        match mode {
            CoppPathOutOfRangeMode::Error => Ok(Self::Error),
            CoppPathOutOfRangeMode::Clamp => Ok(Self::Clamp),
        }
    }
}

impl From<OutOfRangeMode> for CoppPathOutOfRangeMode {
    fn from(mode: OutOfRangeMode) -> Self {
        match mode {
            OutOfRangeMode::Error => Self::Error,
            OutOfRangeMode::Clamp => Self::Clamp,
        }
    }
}

/// C ABI waypoint-spline parametrization.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CoppPathParametrization {
    /// Uniform parameter spacing between waypoints.
    Uniform = 0,
}

impl TryFrom<CoppPathParametrization> for Parametrization {
    type Error = CoppStatus;

    fn try_from(parametrization: CoppPathParametrization) -> Result<Self, Self::Error> {
        match parametrization {
            CoppPathParametrization::Uniform => Ok(Self::Uniform),
        }
    }
}

/// Options for waypoint spline path construction.
///
/// `start_state` and `end_state` are optional matrix views of shape
/// `dim x ((order - 1) / 2)`. Use an empty matrix view (`data = NULL`,
/// `rows = cols = 0`), such as `COPP_MATRIX_VIEW_F64_COLUMN_MAJOR(NULL, 0, 0)`
/// from C, for zero boundary derivatives.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct CoppPathOptions {
    /// Spline order. Must be odd and at least 3. Common values are 3, 5, and 7.
    pub order: usize,
    /// Lower bound of the path parameter range.
    pub s_min: f64,
    /// Upper bound of the path parameter range.
    pub s_max: f64,
    /// Behavior for out-of-range path evaluation.
    pub out_of_range_mode: CoppPathOutOfRangeMode,
    /// Waypoint parametrization mode.
    pub parametrization: CoppPathParametrization,
    /// Boundary derivatives at `s_min`, or empty for zero boundary derivatives.
    pub start_state: CoppMatrixViewF64,
    /// Boundary derivatives at `s_max`, or empty for zero boundary derivatives.
    pub end_state: CoppMatrixViewF64,
}

impl CoppPathOptions {
    fn default_for_range(s_min: f64, s_max: f64) -> Self {
        Self {
            order: 5,
            s_min,
            s_max,
            out_of_range_mode: CoppPathOutOfRangeMode::Error,
            parametrization: CoppPathParametrization::Uniform,
            start_state: CoppMatrixViewF64::empty(),
            end_state: CoppMatrixViewF64::empty(),
        }
    }

    unsafe fn to_spline_config(self) -> Result<SplineConfig, CoppStatus> {
        Ok(SplineConfig {
            order: self.order,
            parametrization: self.parametrization.try_into()?,
            s_min: self.s_min,
            s_max: self.s_max,
            out_of_range_mode: self.out_of_range_mode.try_into()?,
            // SAFETY: The C ABI contract requires non-empty matrix views to
            // point to valid `double` arrays for their declared layouts.
            start_state: unsafe { optional_matrix(self.start_state)? },
            end_state: unsafe { optional_matrix(self.end_state)? },
        })
    }
}

/// Options for tolerance-bounded waypoint fitting.
///
/// Use `copp_smoothing_default_options` first, then override only the fields
/// you need. Empty borrowed slices (`data = NULL`, `len = 0`) select the
/// documented defaults. Borrowed slices are read only during
/// `copp_path_from_waypoints_fitting`; the returned path does not keep them.
///
/// This options type is currently unstable: its fields and layout may change
/// as additional waypoint path-construction algorithms are introduced.
///
/// # Example
/// The example below fits translational rows `0..=2` to `1e-3` input units and
/// rotary rows `3..=4` to `1e-2` input units. Tolerance entries follow `axes`
/// order.
///
/// ```c
/// size_t axes[] = {0, 1, 2, 3, 4};
/// double tolerances[] = {1e-3, 1e-3, 1e-3, 1e-2, 1e-2};
/// struct CoppSmoothingOptions options;
/// check(copp_smoothing_default_options(&options));
/// options.axes = (struct CoppSliceUsize){axes, 5};
/// options.tolerance_per_axis = (struct CoppSliceF64){tolerances, 5};
/// ```
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct CoppSmoothingOptions {
    /// Absolute tolerance broadcast to every selected axis when
    /// `tolerance_per_axis` is empty.
    ///
    /// The value is expressed in each selected row's input units and must be
    /// finite and strictly positive. The default is `0.001`.
    pub tolerance: f64,
    /// Optional per-axis absolute tolerances.
    ///
    /// When non-empty, this replaces `tolerance`. Its length must equal the
    /// number of selected axes, and entry `i` applies to input row `axes[i]`:
    /// the order follows `axes`, not the row index. When `axes` is empty, the
    /// order is the natural row order `0..rows`. Every entry must be finite and
    /// strictly positive, in the corresponding row's input units.
    pub tolerance_per_axis: CoppSliceF64,
    /// Optional 0-based input rows allowed to deviate from their reference
    /// polylines.
    ///
    /// Empty selects every row. A non-empty list must contain distinct,
    /// in-range row indices; its order also defines the order of
    /// `tolerance_per_axis` and of the report's `axes` and `max_errors`.
    /// Unselected rows keep quintic `C4` interpolation through all waypoint
    /// columns, with zero first and second parameter derivatives at both ends.
    pub axes: CoppSliceUsize,
    /// Optional common path parameter assigned to each waypoint column.
    ///
    /// Empty assigns the columns uniformly on `[0, 1]`. A non-empty list must
    /// contain one finite, strictly increasing entry per column; its first and
    /// last entries become the path parameter range. Relative spacing affects
    /// the fitted geometry, not only the returned parameter range.
    pub parameters: CoppSliceF64,
    /// Maximum number of adaptive knot-refinement passes.
    ///
    /// The default is 20. Reaching this limit before the whole-interval audit
    /// passes returns `COPP_STATUS_PATH_SMOOTHING`.
    pub max_refinements: usize,
    /// Maximum number of polynomial spans used by the selected axes.
    ///
    /// The default is 20000. Spans of the separate unselected-axis
    /// interpolant do not consume this budget.
    pub max_segments: usize,
    /// Behavior for out-of-range evaluation of the returned path.
    ///
    /// The default is `COPP_PATH_OUT_OF_RANGE_MODE_ERROR`. Clamping affects
    /// later queries, not fitting or auditing.
    pub out_of_range_mode: CoppPathOutOfRangeMode,
}

impl CoppSmoothingOptions {
    fn default_options() -> Self {
        let defaults = SmoothingConfig::default();
        let tolerance = match defaults.tolerance {
            SmoothingTolerance::Uniform(tolerance) => tolerance,
            SmoothingTolerance::PerAxis(_) => {
                unreachable!("default smoothing tolerance is uniform")
            }
        };
        Self {
            tolerance,
            tolerance_per_axis: CoppSliceF64 {
                data: ptr::null(),
                len: 0,
            },
            axes: CoppSliceUsize {
                data: ptr::null(),
                len: 0,
            },
            parameters: CoppSliceF64 {
                data: ptr::null(),
                len: 0,
            },
            max_refinements: defaults.max_refinements,
            max_segments: defaults.max_segments,
            out_of_range_mode: defaults.out_of_range_mode.into(),
        }
    }

    /// Convert these C options into an owned smoothing configuration.
    ///
    /// # Safety
    /// Non-empty slices must point to valid contiguous arrays for this call.
    unsafe fn to_smoothing_config(self) -> Result<SmoothingConfig, CoppStatus> {
        // SAFETY: The C ABI contract requires non-empty slices to point to
        // valid contiguous arrays for the duration of this call.
        let tolerance_per_axis = unsafe { self.tolerance_per_axis.as_slice()? };
        // SAFETY: Same input-slice contract as above.
        let axes = unsafe { self.axes.as_slice()? };
        // SAFETY: Same input-slice contract as above.
        let parameters = unsafe { self.parameters.as_slice()? };

        Ok(SmoothingConfig {
            tolerance: if tolerance_per_axis.is_empty() {
                SmoothingTolerance::Uniform(self.tolerance)
            } else {
                SmoothingTolerance::PerAxis(tolerance_per_axis.to_vec())
            },
            axes: (!axes.is_empty()).then(|| axes.to_vec()),
            parameters: (!parameters.is_empty()).then(|| parameters.to_vec()),
            max_refinements: self.max_refinements,
            max_segments: self.max_segments,
            out_of_range_mode: self.out_of_range_mode.try_into()?,
        })
    }
}

/// Diagnostics recorded while constructing a tolerance-fitted waypoint path.
///
/// Obtain this report with `copp_path_smoothing_report`. `axes`, `segments`,
/// and `interpolated_segments` describe the final representation, while
/// `refinements`, `fitting_rows`, and `checked_intervals` are cumulative work
/// counters. None of these values is an optimality or run-time guarantee.
/// Both vectors are library-owned and must be released together with
/// `copp_smoothing_report_free`.
///
/// This report type is currently unstable: its fields and layout may change as
/// additional waypoint path-construction algorithms are introduced.
#[repr(C)]
#[derive(Debug)]
pub struct CoppSmoothingReport {
    /// Selected 0-based input-row indices, in tolerance/report order.
    pub axes: CoppVecUsize,
    /// Number of final polynomial spans shared by the selected axes.
    pub segments: usize,
    /// Number of spans in the separate unselected-axis interpolant.
    ///
    /// This is zero when every row is selected and is not included in
    /// `segments` or the `max_segments` budget.
    pub interpolated_segments: usize,
    /// Number of completed local knot-refinement passes.
    pub refinements: usize,
    /// Number of discrete or quadrature fitting rows assembled, counting rows
    /// assembled again during local refits.
    pub fitting_rows: usize,
    /// Number of reference-polyline intervals visited by numerical audits,
    /// including revisits and the final full audit.
    pub checked_intervals: usize,
    /// Final whole-domain absolute-error bound for each selected axis.
    ///
    /// Values are in input units and follow `axes` order. They are
    /// Bernstein-derived numerical bounds computed with ordinary
    /// floating-point arithmetic, not sampled maxima or formal certificates.
    pub max_errors: CoppVecF64,
}

impl CoppSmoothingReport {
    const fn empty() -> Self {
        Self {
            axes: CoppVecUsize::empty(),
            segments: 0,
            interpolated_segments: 0,
            refinements: 0,
            fitting_rows: 0,
            checked_intervals: 0,
            max_errors: CoppVecF64::empty(),
        }
    }

    fn from_report(report: &SmoothingReport) -> Self {
        Self {
            axes: CoppVecUsize::from_vec(report.axes.clone()),
            segments: report.segments,
            interpolated_segments: report.interpolated_segments,
            refinements: report.refinements,
            fitting_rows: report.fitting_rows,
            checked_intervals: report.checked_intervals,
            max_errors: CoppVecF64::from_vec(report.max_errors.clone()),
        }
    }

    fn free(self) {
        self.axes.free();
        self.max_errors.free();
    }
}

impl CoppPath {
    /// Borrow the wrapped path.
    pub(crate) unsafe fn path<'a>(path: *const Self) -> Option<&'a Path> {
        if path.is_null() {
            return None;
        }

        // SAFETY: Non-null `CoppPath` handles returned by this module are
        // pointers to `CoppPathInner` cast to the public opaque handle type.
        let inner = unsafe { &*(path.cast::<CoppPathInner>()) };
        Some(&inner.path)
    }
}

/// Write default waypoint spline options into `out_options`.
///
/// Defaults are quintic order 5, uniform parametrization, zero boundary
/// derivatives, and out-of-range errors.
///
/// # Safety
/// `out_options` must be valid for one `CoppPathOptions` write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_path_default_options(
    s_min: f64,
    s_max: f64,
    out_options: *mut CoppPathOptions,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    if out_options.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: `out_options` was checked for null above and is expected to
        // be valid for one write by the C ABI contract.
        unsafe {
            out_options.write(CoppPathOptions::default_for_range(s_min, s_max));
        }
        CoppStatus::Ok
    })) {
        Ok(status) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Build an interpolating waypoint spline path.
///
/// `waypoints` must be a matrix view of shape `dim x n_points`, where each
/// column is one waypoint. The spline interpolates every waypoint column.
/// `options.start_state` and `options.end_state` may be empty to use zero
/// boundary derivatives.
///
/// Column-major input with `leading_dim >= rows` avoids a temporary layout
/// copy while building the spline coefficients. Row-major input is accepted
/// and copied once into column-major temporary storage. Spline construction
/// still allocates library-owned polynomial coefficients for the returned path.
///
/// On success, `*out_path` receives a non-null handle that must be released
/// with `copp_path_free`.
///
/// # API stability
/// The waypoint path constructors are currently unstable: their names,
/// signatures, and option types may change as additional waypoint
/// path-construction algorithms are introduced. `copp_path_from_waypoints` is
/// an equivalent alias of this function.
///
/// # Example
/// The example below builds a two-dimensional spline path from column-major
/// waypoint coordinates.
///
/// ```c
/// enum { DIM = 2, NUM_WAYPOINTS = 4 };
/// double waypoints[DIM * NUM_WAYPOINTS] = {
///     0.0, 0.0,
///     0.3, 0.2,
///     0.7, 0.8,
///     1.0, 1.0,
/// };
///
/// struct CoppPathOptions options;
/// struct CoppPath *path = NULL;
/// check(copp_path_default_options(0.0, 1.0, &options));
/// check(copp_path_from_waypoints_interpolating(
///     COPP_MATRIX_VIEW_F64_COLUMN_MAJOR(waypoints, DIM, NUM_WAYPOINTS),
///     options,
///     &path));
///
/// copp_path_free(path);
/// ```
///
/// # Safety
/// Non-empty matrix views in `waypoints` and `options` must point to valid
/// `double` arrays for their declared layouts for the duration of this call.
/// `out_path` must be valid for one `CoppPath*` write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_path_from_waypoints_interpolating(
    waypoints: CoppMatrixViewF64,
    options: CoppPathOptions,
    out_path: *mut *mut CoppPath,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    if out_path.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }

    // SAFETY: `out_path` was checked for null above and is expected to be valid
    // for one pointer write by the C ABI contract.
    unsafe {
        out_path.write(ptr::null_mut());
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: The C ABI contract requires non-empty matrix views to point
        // to valid `double` arrays for the declared layout.
        let waypoints = unsafe { waypoints.as_input_matrix()? };

        // SAFETY: The C ABI contract requires non-empty matrix views inside
        // `options` to be valid for the duration of this call.
        let cfg = unsafe { options.to_spline_config()? };
        let path = Path::from_waypoints_interpolating_view(waypoints.as_view(), cfg)
            .map_err(|error| CoppStatus::from(&error))?;
        let path = Box::new(CoppPathInner { path });

        // SAFETY: Same checked output location as above.
        unsafe {
            out_path.write(Box::into_raw(path).cast::<CoppPath>());
        }
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Build an interpolating waypoint spline path.
///
/// This is an equivalent alias of `copp_path_from_waypoints_interpolating`
/// with identical arguments, behavior, and status codes. Both names remain
/// supported and share the unstable status of the waypoint path constructors.
///
/// # Safety
/// Same requirements as `copp_path_from_waypoints_interpolating`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_path_from_waypoints(
    waypoints: CoppMatrixViewF64,
    options: CoppPathOptions,
    out_path: *mut *mut CoppPath,
) -> CoppStatus {
    // SAFETY: This alias forwards the caller's C ABI contract unchanged.
    unsafe { copp_path_from_waypoints_interpolating(waypoints, options, out_path) }
}

/// Write default tolerance-bounded fitting options into `out_options`.
///
/// Defaults fit every row with absolute tolerance `0.001`, assign waypoint
/// parameters uniformly on `[0, 1]`, allow 20 refinement passes and 20000
/// selected-axis spans, and reject out-of-range queries. All borrowed slices
/// are empty.
///
/// # Safety
/// `out_options` must be valid for one `CoppSmoothingOptions` write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_smoothing_default_options(
    out_options: *mut CoppSmoothingOptions,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    if out_options.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: `out_options` was checked for null above and is expected to
        // be valid for one write by the C ABI contract.
        unsafe {
            out_options.write(CoppSmoothingOptions::default_options());
        }
        CoppStatus::Ok
    })) {
        Ok(status) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Build a tolerance-bounded waypoint-fitting path.
///
/// `waypoints` must be a finite matrix view of shape `dim x n_points` with
/// `dim > 0` and `n_points >= 2`, where each column is one waypoint. Each
/// column receives a common path parameter (`options.parameters`, or uniform
/// on `[0, 1]` when empty), and adjacent columns are joined linearly into a
/// reference polyline. The selected axes (`options.axes`, or every row when
/// empty) are approximated by one adaptive nonuniform quintic B-spline with
/// `C4` continuity whose same-parameter absolute deviation from the reference
/// polyline stays within each axis tolerance. The error is audited over every
/// complete reference interval, not only at the waypoints.
///
/// Unlike `copp_path_from_waypoints_interpolating`, the fitted path does not
/// pass through interior waypoints: selected axes may deviate within their
/// tolerances, while the first and last waypoints are retained. Unselected
/// rows keep quintic `C4` interpolation through every column. Derivatives
/// returned by path evaluation are with respect to the path parameter `s`,
/// not time. Use `copp_path_smoothing_report` to inspect the selected axes,
/// the final per-axis error bounds, and the refinement work.
///
/// Column-major input with `leading_dim >= rows` is read without a temporary
/// layout copy. Row-major input is accepted and copied once into column-major
/// temporary storage. The returned path owns its fitted representation and
/// does not borrow `waypoints` or the slices in `options` after the call.
///
/// On success, `*out_path` receives a non-null handle that must be released
/// with `copp_path_free`.
///
/// # API stability
/// The waypoint path constructors and `CoppSmoothingOptions` /
/// `CoppSmoothingReport` are currently unstable: their names, signatures, and
/// option types may change as additional waypoint path-construction
/// algorithms are introduced.
///
/// # Errors
/// Returns `COPP_STATUS_PATH_SMOOTHING` when the waypoint values, dimensions,
/// axes, tolerances, or parameters violate their contracts, when the
/// selected-axis span count exceeds `options.max_segments`, when refinement
/// exhausts `options.max_refinements` or the parameter resolution, when the
/// banded fit encounters an unusable numerical system, or when the final
/// whole-interval audit fails. An unchecked or relaxed approximation is never
/// returned.
///
/// # Example
/// The example below fits a two-dimensional waypoint path within `1e-3` input
/// units and reads the final error bound of each axis.
///
/// ```c
/// enum { DIM = 2, NUM_WAYPOINTS = 5 };
/// double waypoints[DIM * NUM_WAYPOINTS] = {
///     0.0, 0.0,
///     0.25, 0.4,
///     0.5, 0.5,
///     0.75, 0.4,
///     1.0, 0.0,
/// };
///
/// struct CoppSmoothingOptions options;
/// struct CoppPath *path = NULL;
/// check(copp_smoothing_default_options(&options));
/// options.tolerance = 1e-3;
/// check(copp_path_from_waypoints_fitting(
///     COPP_MATRIX_VIEW_F64_COLUMN_MAJOR(waypoints, DIM, NUM_WAYPOINTS),
///     options,
///     &path));
///
/// bool has_report = false;
/// struct CoppSmoothingReport report = {0};
/// check(copp_path_smoothing_report(path, &has_report, &report));
/// for (size_t i = 0; i < report.axes.len; ++i) {
///     printf("axis %zu: %.3e\n", report.axes.data[i], report.max_errors.data[i]);
/// }
/// copp_smoothing_report_free(report);
/// copp_path_free(path);
/// ```
///
/// # Safety
/// Non-empty matrix views and slices in `waypoints` and `options` must point
/// to valid arrays for their declared layouts for the duration of this call.
/// `out_path` must be valid for one `CoppPath*` write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_path_from_waypoints_fitting(
    waypoints: CoppMatrixViewF64,
    options: CoppSmoothingOptions,
    out_path: *mut *mut CoppPath,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    if out_path.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }

    // SAFETY: `out_path` was checked for null above and is expected to be valid
    // for one pointer write by the C ABI contract.
    unsafe {
        out_path.write(ptr::null_mut());
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: The C ABI contract requires non-empty matrix views to point
        // to valid `double` arrays for the declared layout.
        let waypoints = unsafe { waypoints.as_input_matrix()? };

        // SAFETY: The C ABI contract requires non-empty slices inside
        // `options` to be valid for the duration of this call.
        let cfg = unsafe { options.to_smoothing_config()? };
        let path = Path::from_waypoints_fitting_view(waypoints.as_view(), cfg)
            .map_err(|error| CoppStatus::from(&error))?;
        let path = Box::new(CoppPathInner { path });

        // SAFETY: Same checked output location as above.
        unsafe {
            out_path.write(Box::into_raw(path).cast::<CoppPath>());
        }
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Return the construction report of a tolerance-fitted waypoint path.
///
/// For a path built by `copp_path_from_waypoints_fitting`, `*out_has_report`
/// receives `true` and `*out_report` receives a library-owned report that must
/// be released with `copp_smoothing_report_free`. For any other path,
/// `*out_has_report` receives `false` and `*out_report` receives an empty
/// report with null vectors and zero counters; releasing it is allowed.
///
/// # Safety
/// `path` must be a non-null handle created by this module.
/// `out_has_report` must be valid for one `bool` write and `out_report` for
/// one `CoppSmoothingReport` write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_path_smoothing_report(
    path: *const CoppPath,
    out_has_report: *mut bool,
    out_report: *mut CoppSmoothingReport,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    if out_has_report.is_null() || out_report.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }
    // SAFETY: Output pointers were checked for null above and are expected to
    // be valid for one write each by the C ABI contract.
    unsafe {
        out_has_report.write(false);
        out_report.write(CoppSmoothingReport::empty());
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: The C ABI contract requires `path` to be a live handle
        // created by this module for the duration of this call.
        let path = unsafe { CoppPath::path(path) }.ok_or(CoppStatus::NullPointer)?;
        if let Some(report) = path.smoothing_report() {
            let report = CoppSmoothingReport::from_report(report);
            // SAFETY: Same checked output locations as above.
            unsafe {
                out_report.write(report);
                out_has_report.write(true);
            }
        }
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Release memory owned by a `CoppSmoothingReport`.
///
/// Passing an empty report is allowed and has no effect. Passing an
/// already-freed report is invalid.
///
/// # Safety
/// `report` must either be empty/null or have been returned by
/// `copp_path_smoothing_report`. Passing arbitrary pointers or modified
/// capacity fields is invalid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_smoothing_report_free(report: CoppSmoothingReport) {
    report.free();
    clear_last_error();
}

/// Build a path from a scalar-parametric C callback.
///
/// COPP calls `evaluate` once per path sample with a seeded [`CoppJet3`] value
/// for `s`. The callback writes `dim` output entries describing only `q(s)`;
/// the returned `CoppJet3` derivative fields provide `dq`, `ddq`, and `dddq`.
/// Use the inline helpers in `copp/path.h` such as `copp_add`, `copp_mul`, and
/// `copp_sin` to write formulas without hand-differentiating them.
///
/// The resulting path supports position, second-order, and third-order
/// evaluation and can be sampled by `copp_robot_sample_path_2nd` or
/// `copp_robot_sample_path_3rd`.
///
/// The callback pointer and `user_data` are borrowed by the created path and
/// must remain valid until `copp_path_free` is called. Calls into the callback
/// are serialized per path handle.
///
/// On success, `*out_path` receives a non-null handle that must be released
/// with `copp_path_free`.
///
/// # Example
/// The example below builds a one-dimensional `q(s) = sin(2*pi*s)` path.
///
/// ```c
/// static enum CoppStatus eval_path_parametric(
///     void *user_data,
///     size_t dim,
///     struct CoppJet3 s,
///     struct CoppJet3 *q)
/// {
///     (void)user_data;
///     if (dim != 1 || q == NULL) {
///         return COPP_STATUS_INVALID_ARGUMENT;
///     }
///     q[0] = copp_sin(copp_mul_f64(s, 6.28318530717958647692));
///     return COPP_STATUS_OK;
/// }
///
/// struct CoppPath *path = NULL;
/// check(copp_path_from_parametric(1, 0.0, 1.0, eval_path_parametric, NULL, &path));
/// ```
///
/// # Safety
/// `evaluate` must be non-null and valid until `copp_path_free`.
/// `user_data` must remain valid for all callback calls. `out_path` must be
/// valid for one `CoppPath*` write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_path_from_parametric(
    dim: usize,
    s_min: f64,
    s_max: f64,
    evaluate: CoppPathParametricFn,
    user_data: *mut c_void,
    out_path: *mut *mut CoppPath,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    if out_path.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }

    // SAFETY: `out_path` was checked for null above and is expected to be valid
    // for one pointer write by the C ABI contract.
    unsafe {
        out_path.write(ptr::null_mut());
    }

    match catch_unwind(AssertUnwindSafe(|| {
        let evaluate = evaluate.ok_or(CoppStatus::NullPointer)?;
        let evaluator = CoppParametricPathEvaluator {
            dim,
            evaluate,
            user_data,
            scratch: Mutex::new(CallbackScratch::default()),
        };
        let path = Path::from_evaluator_3rd(evaluator, s_min, s_max)
            .map_err(|error| CoppStatus::from(&error))?;
        let path = Box::new(CoppPathInner { path });

        // SAFETY: Same checked output location as above.
        unsafe {
            out_path.write(Box::into_raw(path).cast::<CoppPath>());
        }
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Build a path from a C callback that provides derivatives up to 2nd order.
///
/// The callback operates on a batch of `n` path samples and must write
/// column-major `dim x n` output matrices (`data[row + col * dim]`) for `q`,
/// `dq`, and `ddq`. The resulting path can be evaluated or sampled up to 2nd
/// order. Third-order evaluation returns
/// `COPP_STATUS_PATH_UNSUPPORTED_DERIVATIVE_ORDER`.
///
/// The callback pointer and `user_data` are borrowed by the created path and
/// must remain valid until `copp_path_free` is called. Calls into the callback
/// are serialized per path handle.
///
/// On success, `*out_path` receives a non-null handle that must be released
/// with `copp_path_free`.
///
/// # Example
/// The example below wraps a one-dimensional external evaluator that provides
/// `q`, `dq`, and `ddq`.
///
/// ```c
/// static enum CoppStatus eval_path_2nd(
///     void *user_data,
///     size_t dim,
///     size_t n,
///     const double *s,
///     double *q,
///     double *dq,
///     double *ddq)
/// {
///     for (size_t j = 0; j < n; ++j) {
///         q[0 + j * dim] = s[j];
///         dq[0 + j * dim] = 1.0;
///         ddq[0 + j * dim] = 0.0;
///     }
///     return COPP_STATUS_OK;
/// }
///
/// struct CoppPath *path = NULL;
/// check(copp_path_from_evaluator_2nd(1, 0.0, 1.0, eval_path_2nd, NULL, &path));
/// ```
///
/// # Safety
/// `evaluate_2nd` must be non-null and valid until `copp_path_free`.
/// `user_data` must remain valid for all callback calls. `out_path` must be
/// valid for one `CoppPath*` write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_path_from_evaluator_2nd(
    dim: usize,
    s_min: f64,
    s_max: f64,
    evaluate_2nd: CoppPathEvaluate2ndFn,
    user_data: *mut c_void,
    out_path: *mut *mut CoppPath,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    if out_path.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }

    // SAFETY: `out_path` was checked for null above and is expected to be valid
    // for one pointer write by the C ABI contract.
    unsafe {
        out_path.write(ptr::null_mut());
    }

    match catch_unwind(AssertUnwindSafe(|| {
        let evaluate_2nd = evaluate_2nd.ok_or(CoppStatus::NullPointer)?;
        let evaluator = CoppCallbackPathEvaluator2nd {
            dim,
            evaluate_2nd,
            user_data,
            scratch: Mutex::new(CallbackScratch::default()),
        };
        let path = Path::from_evaluator_2nd(evaluator, s_min, s_max)
            .map_err(|error| CoppStatus::from(&error))?;
        let path = Box::new(CoppPathInner { path });

        // SAFETY: Same checked output location as above.
        unsafe {
            out_path.write(Box::into_raw(path).cast::<CoppPath>());
        }
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Build a path from C callbacks that provide derivatives up to 3rd order.
///
/// `evaluate_3rd` is required. `evaluate_2nd` may be null; when it is null,
/// second-order evaluation calls `evaluate_3rd` internally and discards `dddq`.
/// Both callbacks operate on a batch of `n` path samples and must write
/// column-major `dim x n` output matrices (`data[row + col * dim]`).
///
/// The callback pointers and `user_data` are borrowed by the created path and
/// must remain valid until `copp_path_free` is called. Calls into the callbacks
/// are serialized per path handle.
///
/// On success, `*out_path` receives a non-null handle that must be released
/// with `copp_path_free`.
///
/// # Example
/// The example below wraps a one-dimensional external evaluator that also
/// provides third derivatives.
///
/// ```c
/// static enum CoppStatus eval_path_3rd(
///     void *user_data,
///     size_t dim,
///     size_t n,
///     const double *s,
///     double *q,
///     double *dq,
///     double *ddq,
///     double *dddq)
/// {
///     for (size_t j = 0; j < n; ++j) {
///         q[0 + j * dim] = s[j] * s[j];
///         dq[0 + j * dim] = 2.0 * s[j];
///         ddq[0 + j * dim] = 2.0;
///         dddq[0 + j * dim] = 0.0;
///     }
///     return COPP_STATUS_OK;
/// }
///
/// struct CoppPath *path = NULL;
/// check(copp_path_from_evaluator_3rd(1, 0.0, 1.0, NULL, eval_path_3rd, NULL, &path));
/// ```
///
/// # Safety
/// `evaluate_3rd` must be non-null and valid until `copp_path_free`.
/// `evaluate_2nd`, when non-null, must follow the same lifetime rule.
/// `user_data` must remain valid for all callback calls. `out_path` must be
/// valid for one `CoppPath*` write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_path_from_evaluator_3rd(
    dim: usize,
    s_min: f64,
    s_max: f64,
    evaluate_2nd: CoppPathEvaluate2ndFn,
    evaluate_3rd: CoppPathEvaluate3rdFn,
    user_data: *mut c_void,
    out_path: *mut *mut CoppPath,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    if out_path.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }

    // SAFETY: `out_path` was checked for null above and is expected to be valid
    // for one pointer write by the C ABI contract.
    unsafe {
        out_path.write(ptr::null_mut());
    }

    match catch_unwind(AssertUnwindSafe(|| {
        let evaluate_3rd = evaluate_3rd.ok_or(CoppStatus::NullPointer)?;
        let evaluator = CoppCallbackPathEvaluator3rd {
            dim,
            evaluate_2nd,
            evaluate_3rd,
            user_data,
            scratch: Mutex::new(CallbackScratch::default()),
        };
        let path = Path::from_evaluator_3rd(evaluator, s_min, s_max)
            .map_err(|error| CoppStatus::from(&error))?;
        let path = Box::new(CoppPathInner { path });

        // SAFETY: Same checked output location as above.
        unsafe {
            out_path.write(Box::into_raw(path).cast::<CoppPath>());
        }
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Return the path dimension.
///
/// # Safety
/// `path` must be a non-null handle created by this module.
/// `out_dim` must be valid for one `size_t` write.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_path_dim(path: *const CoppPath, out_dim: *mut usize) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    if path.is_null() || out_dim.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: `path` was checked for null above; the C ABI contract
        // requires it to be a live handle created by this module.
        let path = unsafe { CoppPath::path(path) }.ok_or(CoppStatus::NullPointer)?;
        // SAFETY: `out_dim` was checked for null above and is expected to be
        // valid for one write by the C ABI contract.
        unsafe {
            out_dim.write(path.dim());
        }
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Return the valid path parameter range.
///
/// # Safety
/// `path` must be a non-null handle created by this module.
/// `out_s_min` and `out_s_max` must be valid for one `double` write each.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_path_s_range(
    path: *const CoppPath,
    out_s_min: *mut f64,
    out_s_max: *mut f64,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    if path.is_null() || out_s_min.is_null() || out_s_max.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: `path` was checked for null above; the C ABI contract
        // requires it to be a live handle created by this module.
        let path = unsafe { CoppPath::path(path) }.ok_or(CoppStatus::NullPointer)?;
        let (s_min, s_max) = path.s_range();
        // SAFETY: output pointers were checked for null above and are expected
        // to be valid for one write each by the C ABI contract.
        unsafe {
            out_s_min.write(s_min);
            out_s_max.write(s_max);
        }
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Evaluate `q`, `dq`, and `ddq` at the supplied path parameters.
///
/// Output matrices are column-major with shape `dim x s.len` and must be
/// released with `copp_matrix_f64_free`.
///
/// # Example
/// The example below evaluates second-order path derivatives and reads one
/// column-major output entry.
///
/// ```c
/// double s_eval[] = {0.0, 0.5, 1.0};
/// struct CoppMatrixF64 q = {0};
/// struct CoppMatrixF64 dq = {0};
/// struct CoppMatrixF64 ddq = {0};
///
/// check(copp_path_evaluate_up_to_2nd(
///     path,
///     (struct CoppSliceF64){s_eval, 3},
///     &q,
///     &dq,
///     &ddq));
///
/// double q_axis0_at_sample1 = q.data[0 + 1 * q.rows];
/// copp_matrix_f64_free(ddq);
/// copp_matrix_f64_free(dq);
/// copp_matrix_f64_free(q);
/// ```
///
/// # Safety
/// `path` must be a non-null handle created by this module.
/// `s.data` must be valid for `s.len` reads when non-empty. Output pointers
/// must be valid for one `CoppMatrixF64` write each.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_path_evaluate_up_to_2nd(
    path: *const CoppPath,
    s: CoppSliceF64,
    out_q: *mut CoppMatrixF64,
    out_dq: *mut CoppMatrixF64,
    out_ddq: *mut CoppMatrixF64,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    let outs = match checked_matrix_outs(&[out_q, out_dq, out_ddq]) {
        Ok(outs) => outs,
        Err(status) => return status,
    };
    for out in outs.iter().copied() {
        // SAFETY: `checked_matrix_outs` returned non-null output locations.
        unsafe {
            CoppMatrixF64::write_empty_to(out);
        }
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: The C ABI contract requires `path` to be a live handle
        // created by this module for the duration of this call.
        let path = unsafe { CoppPath::path(path) }.ok_or(CoppStatus::NullPointer)?;
        // SAFETY: The C ABI contract requires non-empty input slices to point
        // to valid contiguous `double` arrays for the duration of this call.
        let s = unsafe { s.as_slice()? };
        let out = path
            .evaluate_up_to_2nd(s)
            .map_err(|error| CoppStatus::from(&error))?;

        // SAFETY: All output locations were checked and initialized above.
        unsafe {
            CoppMatrixF64::write_matrix_to(outs[0], out.q)?;
            CoppMatrixF64::write_matrix_to(
                outs[1],
                out.dq.ok_or(CoppStatus::PathDimensionMismatch)?,
            )?;
            CoppMatrixF64::write_matrix_to(
                outs[2],
                out.ddq.ok_or(CoppStatus::PathDimensionMismatch)?,
            )?;
        }
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Evaluate `q`, `dq`, `ddq`, and `dddq` at the supplied path parameters.
///
/// Output matrices are column-major with shape `dim x s.len` and must be
/// released with `copp_matrix_f64_free`.
///
/// # Example
/// The example below evaluates a third-order path and releases all returned
/// matrices.
///
/// ```c
/// struct CoppMatrixF64 q = {0}, dq = {0}, ddq = {0}, dddq = {0};
/// check(copp_path_evaluate_up_to_3rd(path, s_slice, &q, &dq, &ddq, &dddq));
/// copp_matrix_f64_free(dddq);
/// copp_matrix_f64_free(ddq);
/// copp_matrix_f64_free(dq);
/// copp_matrix_f64_free(q);
/// ```
///
/// # Safety
/// `path` must be a non-null handle created by this module.
/// `s.data` must be valid for `s.len` reads when non-empty. Output pointers
/// must be valid for one `CoppMatrixF64` write each.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_path_evaluate_up_to_3rd(
    path: *const CoppPath,
    s: CoppSliceF64,
    out_q: *mut CoppMatrixF64,
    out_dq: *mut CoppMatrixF64,
    out_ddq: *mut CoppMatrixF64,
    out_dddq: *mut CoppMatrixF64,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    let outs = match checked_matrix_outs(&[out_q, out_dq, out_ddq, out_dddq]) {
        Ok(outs) => outs,
        Err(status) => return status,
    };
    for out in outs.iter().copied() {
        // SAFETY: `checked_matrix_outs` returned non-null output locations.
        unsafe {
            CoppMatrixF64::write_empty_to(out);
        }
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: The C ABI contract requires `path` to be a live handle
        // created by this module for the duration of this call.
        let path = unsafe { CoppPath::path(path) }.ok_or(CoppStatus::NullPointer)?;
        // SAFETY: The C ABI contract requires non-empty input slices to point
        // to valid contiguous `double` arrays for the duration of this call.
        let s = unsafe { s.as_slice()? };
        let out = path
            .evaluate_up_to_3rd(s)
            .map_err(|error| CoppStatus::from(&error))?;

        // SAFETY: All output locations were checked and initialized above.
        unsafe {
            CoppMatrixF64::write_matrix_to(outs[0], out.q)?;
            CoppMatrixF64::write_matrix_to(
                outs[1],
                out.dq.ok_or(CoppStatus::PathDimensionMismatch)?,
            )?;
            CoppMatrixF64::write_matrix_to(
                outs[2],
                out.ddq.ok_or(CoppStatus::PathDimensionMismatch)?,
            )?;
            CoppMatrixF64::write_matrix_to(
                outs[3],
                out.dddq.ok_or(CoppStatus::PathDimensionMismatch)?,
            )?;
        }
        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

/// Sample a path over an existing robot station interval and store second-order derivatives.
///
/// The interval is `[idx_s_from, idx_s_to)`. The station values must already
/// have been appended to the robot.
///
/// # Example
/// The example below appends a station grid and samples second-order path
/// derivatives into the robot constraint storage.
///
/// ```c
/// double s[] = {0.0, 0.5, 1.0};
/// struct CoppRobot *robot = NULL;
/// check(copp_robot_create(dim, 3, &robot));
/// check(copp_robot_append_s(robot, (struct CoppSliceF64){s, 3}));
/// check(copp_robot_sample_path_2nd(robot, path, 0, 3));
/// ```
///
/// # Safety
/// `robot` and `path` must be non-null handles created by COPP and must remain
/// valid for the duration of this call.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_robot_sample_path_2nd(
    robot: *mut CoppRobot,
    path: *const CoppPath,
    idx_s_from: usize,
    idx_s_to: usize,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    sample_path_common(robot, path, idx_s_from, idx_s_to, 2)
}

/// Sample a path over an existing robot station interval and store third-order derivatives.
///
/// The interval is `[idx_s_from, idx_s_to)`. The station values must already
/// have been appended to the robot.
///
/// # Example
/// The example below samples third-order path derivatives over the current
/// robot station grid.
///
/// ```c
/// check(copp_robot_append_s(robot, (struct CoppSliceF64){s, n}));
/// check(copp_robot_sample_path_3rd(robot, path, 0, n));
/// ```
///
/// # Safety
/// `robot` and `path` must be non-null handles created by COPP and must remain
/// valid for the duration of this call.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_robot_sample_path_3rd(
    robot: *mut CoppRobot,
    path: *const CoppPath,
    idx_s_from: usize,
    idx_s_to: usize,
) -> CoppStatus {
    crate::ffi::c::core::status::clear_last_error();
    sample_path_common(robot, path, idx_s_from, idx_s_to, 3)
}

fn sample_path_common(
    robot: *mut CoppRobot,
    path: *const CoppPath,
    idx_s_from: usize,
    idx_s_to: usize,
    order: u8,
) -> CoppStatus {
    if robot.is_null() || path.is_null() {
        return CoppStatus::NullPointer.into_ffi_status();
    }

    match catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: `robot` and `path` were checked for null above; the C ABI
        // contract requires them to be live handles created by COPP.
        let robot = unsafe { CoppRobot::robot_mut(robot) }.ok_or(CoppStatus::NullPointer)?;
        let path = unsafe { CoppPath::path(path) }.ok_or(CoppStatus::NullPointer)?;

        match order {
            2 => robot.with_q_from_path_2nd(path, idx_s_from, idx_s_to),
            3 => robot.with_q_from_path_3rd(path, idx_s_from, idx_s_to),
            _ => unreachable!("unsupported path sampling order"),
        }
        .map(|_| ())
        .map_err(|error| CoppStatus::from(&error))?;

        Ok(CoppStatus::Ok)
    })) {
        Ok(Ok(status)) | Ok(Err(status)) => status.into_ffi_status(),
        Err(payload) => panic_to_status(payload).into_ffi_status(),
    }
}

fn checked_matrix_outs(
    outs: &[*mut CoppMatrixF64],
) -> Result<Vec<std::ptr::NonNull<CoppMatrixF64>>, CoppStatus> {
    outs.iter()
        .map(|&out| CoppMatrixF64::out_ptr(out))
        .collect()
}

unsafe fn optional_matrix(view: CoppMatrixViewF64) -> Result<Option<DMatrix<f64>>, CoppStatus> {
    if view.rows == 0 && view.cols == 0 {
        return Ok(None);
    }

    // SAFETY: The C ABI contract requires non-empty matrix views to point to
    // valid `double` arrays for the declared layout.
    Ok(Some(unsafe { view.to_dmatrix()? }))
}

/// Release a path handle created by this module.
///
/// Passing null is allowed and has no effect.
///
/// # Safety
/// `path` must either be null or a live path handle that has not already been
/// freed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_path_free(path: *mut CoppPath) {
    if path.is_null() {
        clear_last_error();
        return;
    }

    // SAFETY: The C ABI contract requires `path` to come from `Box::into_raw`
    // in this module and to be freed at most once.
    unsafe {
        drop(Box::from_raw(path.cast::<CoppPathInner>()));
    }
    clear_last_error();
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::path::PathEvaluator2nd;
    use std::{ffi::c_void, mem::MaybeUninit, slice};

    /// Column-major two-axis polyline with a corner at `s = 0.5`.
    const FITTING_WAYPOINTS: [f64; 10] = [0.0, 0.0, 0.25, 0.4, 0.5, 0.5, 0.75, 0.4, 1.0, 0.0];

    fn fitting_waypoints_view() -> CoppMatrixViewF64 {
        CoppMatrixViewF64 {
            data: FITTING_WAYPOINTS.as_ptr(),
            rows: 2,
            cols: FITTING_WAYPOINTS.len() / 2,
            layout: crate::ffi::c::CoppMatrixLayout::ColumnMajor,
            leading_dim: 2,
        }
    }

    fn fit(options: CoppSmoothingOptions) -> (CoppStatus, *mut CoppPath) {
        let mut path = ptr::null_mut();
        let status = unsafe {
            copp_path_from_waypoints_fitting(fitting_waypoints_view(), options, &mut path)
        };
        (status, path)
    }

    fn smoothing_report(path: *const CoppPath) -> (bool, CoppSmoothingReport) {
        let mut has_report = true;
        let mut report = CoppSmoothingReport::empty();
        let status = unsafe { copp_path_smoothing_report(path, &mut has_report, &mut report) };
        assert_eq!(status, CoppStatus::Ok);
        (has_report, report)
    }

    #[test]
    fn smoothing_default_options_match_rust_defaults() {
        let mut out = MaybeUninit::<CoppSmoothingOptions>::uninit();
        let status = unsafe { copp_smoothing_default_options(out.as_mut_ptr()) };
        assert_eq!(status, CoppStatus::Ok);
        let options = unsafe { out.assume_init() };

        let defaults = SmoothingConfig::default();
        let SmoothingTolerance::Uniform(tolerance) = defaults.tolerance else {
            panic!("expected a uniform default tolerance");
        };
        assert_eq!(options.tolerance, tolerance);
        assert_eq!(options.tolerance_per_axis.len, 0);
        assert_eq!(options.axes.len, 0);
        assert_eq!(options.parameters.len, 0);
        assert_eq!(options.max_refinements, defaults.max_refinements);
        assert_eq!(options.max_segments, defaults.max_segments);
        assert_eq!(options.out_of_range_mode, CoppPathOutOfRangeMode::Error);
    }

    #[test]
    fn smoothing_options_map_empty_and_explicit_slices() {
        let mut options = CoppSmoothingOptions::default_options();
        options.tolerance = 2.0e-3;
        let cfg = unsafe { options.to_smoothing_config() }.unwrap();
        assert!(matches!(cfg.tolerance, SmoothingTolerance::Uniform(value) if value == 2.0e-3));
        assert!(cfg.axes.is_none());
        assert!(cfg.parameters.is_none());

        let tolerances = [1.0e-3, 1.0e-2];
        let axes = [1_usize, 0];
        let parameters = [2.0, 3.0, 4.0];
        options.tolerance_per_axis = CoppSliceF64 {
            data: tolerances.as_ptr(),
            len: tolerances.len(),
        };
        options.axes = CoppSliceUsize {
            data: axes.as_ptr(),
            len: axes.len(),
        };
        options.parameters = CoppSliceF64 {
            data: parameters.as_ptr(),
            len: parameters.len(),
        };
        options.max_refinements = 3;
        options.max_segments = 7;
        options.out_of_range_mode = CoppPathOutOfRangeMode::Clamp;

        let cfg = unsafe { options.to_smoothing_config() }.unwrap();
        let SmoothingTolerance::PerAxis(per_axis) = cfg.tolerance else {
            panic!("expected per-axis tolerances");
        };
        assert_eq!(per_axis, tolerances);
        assert_eq!(cfg.axes.as_deref(), Some(&axes[..]));
        assert_eq!(cfg.parameters.as_deref(), Some(&parameters[..]));
        assert_eq!(cfg.max_refinements, 3);
        assert_eq!(cfg.max_segments, 7);
        assert!(matches!(cfg.out_of_range_mode, OutOfRangeMode::Clamp));
    }

    #[test]
    fn fitted_path_has_report_and_interpolated_path_has_none() {
        let mut options = CoppSmoothingOptions::default_options();
        options.tolerance = 1.0e-3;
        let (status, fitted) = fit(options);
        assert_eq!(status, CoppStatus::Ok);

        let (has_report, report) = smoothing_report(fitted);
        assert!(has_report);
        let axes = unsafe { slice::from_raw_parts(report.axes.data, report.axes.len) };
        let max_errors =
            unsafe { slice::from_raw_parts(report.max_errors.data, report.max_errors.len) };
        assert_eq!(axes, [0, 1]);
        assert_eq!(max_errors.len(), 2);
        assert!(max_errors.iter().all(|&error| error <= 1.0e-3));
        assert!(report.segments > 0);
        assert_eq!(report.interpolated_segments, 0);
        unsafe { copp_smoothing_report_free(report) };

        let mut interpolated = ptr::null_mut();
        let status = unsafe {
            copp_path_from_waypoints_interpolating(
                fitting_waypoints_view(),
                CoppPathOptions::default_for_range(0.0, 1.0),
                &mut interpolated,
            )
        };
        assert_eq!(status, CoppStatus::Ok);
        let (has_report, report) = smoothing_report(interpolated);
        assert!(!has_report);
        assert!(report.axes.data.is_null() && report.axes.len == 0);
        assert!(report.max_errors.data.is_null() && report.max_errors.len == 0);
        unsafe { copp_smoothing_report_free(report) };

        unsafe {
            copp_path_free(interpolated);
            copp_path_free(fitted);
        }
    }

    #[test]
    fn fitting_failures_map_to_path_smoothing() {
        let mut options = CoppSmoothingOptions::default_options();
        let axes = [2_usize];
        options.axes = CoppSliceUsize {
            data: axes.as_ptr(),
            len: axes.len(),
        };
        let (status, path) = fit(options);
        assert_eq!(status, CoppStatus::PathSmoothing);
        assert!(path.is_null());

        let mut options = CoppSmoothingOptions::default_options();
        options.max_segments = 0;
        let (status, path) = fit(options);
        assert_eq!(status, CoppStatus::PathSmoothing);
        assert!(path.is_null());

        let options = CoppSmoothingOptions::default_options();
        let status = unsafe {
            copp_path_from_waypoints_fitting(fitting_waypoints_view(), options, ptr::null_mut())
        };
        assert_eq!(status, CoppStatus::NullPointer);
    }

    #[derive(Default)]
    struct CallbackState {
        calls_2nd: usize,
        calls_3rd: usize,
    }

    unsafe extern "C" fn eval_2nd(
        user_data: *mut c_void,
        dim: usize,
        n: usize,
        s: *const f64,
        q: *mut f64,
        dq: *mut f64,
        ddq: *mut f64,
    ) -> CoppStatus {
        let state = unsafe { &mut *(user_data as *mut CallbackState) };
        state.calls_2nd += 1;
        let s = unsafe { slice::from_raw_parts(s, n) };
        let len = dim * n;
        let q = unsafe { slice::from_raw_parts_mut(q, len) };
        let dq = unsafe { slice::from_raw_parts_mut(dq, len) };
        let ddq = unsafe { slice::from_raw_parts_mut(ddq, len) };

        for (col, &s) in s.iter().enumerate().take(n) {
            for row in 0..dim {
                let idx = row + col * dim;
                let value = s + row as f64;
                q[idx] = value;
                dq[idx] = 10.0 + value;
                ddq[idx] = 20.0 + value;
            }
        }
        CoppStatus::Ok
    }

    unsafe extern "C" fn eval_3rd(
        user_data: *mut c_void,
        dim: usize,
        n: usize,
        s: *const f64,
        q: *mut f64,
        dq: *mut f64,
        ddq: *mut f64,
        dddq: *mut f64,
    ) -> CoppStatus {
        let state = unsafe { &mut *(user_data as *mut CallbackState) };
        state.calls_3rd += 1;
        let s = unsafe { slice::from_raw_parts(s, n) };
        let len = dim * n;
        let q = unsafe { slice::from_raw_parts_mut(q, len) };
        let dq = unsafe { slice::from_raw_parts_mut(dq, len) };
        let ddq = unsafe { slice::from_raw_parts_mut(ddq, len) };
        let dddq = unsafe { slice::from_raw_parts_mut(dddq, len) };

        for (col, &s) in s.iter().enumerate().take(n) {
            for row in 0..dim {
                let idx = row + col * dim;
                let value = s + row as f64;
                q[idx] = 100.0 + value;
                dq[idx] = 110.0 + value;
                ddq[idx] = 120.0 + value;
                dddq[idx] = 130.0 + value;
            }
        }
        CoppStatus::Ok
    }

    #[test]
    fn callback_2nd_evaluate_q_reuses_scratch_buffers() {
        let mut state = CallbackState::default();
        let evaluator = CoppCallbackPathEvaluator2nd {
            dim: 2,
            evaluate_2nd: eval_2nd,
            user_data: (&mut state as *mut CallbackState).cast::<c_void>(),
            scratch: Mutex::new(CallbackScratch::default()),
        };

        let s = [0.25, 0.5, 0.75];
        let mut q = vec![0.0; 2 * s.len()];
        evaluator.evaluate_q(&s, &mut q).unwrap();

        assert_eq!(state.calls_2nd, 1);
        assert_eq!(q, [0.25, 1.25, 0.5, 1.5, 0.75, 1.75]);
        let first_capacity = {
            let scratch = evaluator.scratch.lock().unwrap();
            assert_eq!(scratch.dq.len(), q.len());
            assert_eq!(scratch.ddq.len(), q.len());
            scratch.dq.capacity()
        };

        let s = [1.0];
        let mut q = vec![0.0; 2 * s.len()];
        evaluator.evaluate_q(&s, &mut q).unwrap();

        assert_eq!(state.calls_2nd, 2);
        assert_eq!(q, [1.0, 2.0]);
        let scratch = evaluator.scratch.lock().unwrap();
        assert_eq!(scratch.dq.len(), q.len());
        assert_eq!(scratch.ddq.len(), q.len());
        assert!(scratch.dq.capacity() >= first_capacity);
    }

    #[test]
    fn callback_3rd_without_2nd_uses_scratch_for_up_to_2nd_fallback() {
        let mut state = CallbackState::default();
        let evaluator = CoppCallbackPathEvaluator3rd {
            dim: 2,
            evaluate_2nd: None,
            evaluate_3rd: eval_3rd,
            user_data: (&mut state as *mut CallbackState).cast::<c_void>(),
            scratch: Mutex::new(CallbackScratch::default()),
        };
        let s = [0.25, 0.5];
        let len = 2 * s.len();
        let mut q = vec![0.0; len];
        let mut dq = vec![0.0; len];
        let mut ddq = vec![0.0; len];

        evaluator
            .evaluate_up_to_2nd(&s, &mut q, &mut dq, &mut ddq)
            .unwrap();

        assert_eq!(state.calls_2nd, 0);
        assert_eq!(state.calls_3rd, 1);
        assert_eq!(q, [100.25, 101.25, 100.5, 101.5]);
        assert_eq!(dq, [110.25, 111.25, 110.5, 111.5]);
        assert_eq!(ddq, [120.25, 121.25, 120.5, 121.5]);
        let scratch = evaluator.scratch.lock().unwrap();
        assert_eq!(scratch.dddq.len(), len);
    }

    #[test]
    fn callback_3rd_with_2nd_prefers_2nd_for_up_to_2nd() {
        let mut state = CallbackState::default();
        let evaluator = CoppCallbackPathEvaluator3rd {
            dim: 2,
            evaluate_2nd: Some(eval_2nd),
            evaluate_3rd: eval_3rd,
            user_data: (&mut state as *mut CallbackState).cast::<c_void>(),
            scratch: Mutex::new(CallbackScratch::default()),
        };
        let s = [0.25, 0.5];
        let len = 2 * s.len();
        let mut q = vec![0.0; len];
        let mut dq = vec![0.0; len];
        let mut ddq = vec![0.0; len];

        evaluator
            .evaluate_up_to_2nd(&s, &mut q, &mut dq, &mut ddq)
            .unwrap();

        assert_eq!(state.calls_2nd, 1);
        assert_eq!(state.calls_3rd, 0);
        assert_eq!(q, [0.25, 1.25, 0.5, 1.5]);
        assert_eq!(dq, [10.25, 11.25, 10.5, 11.5]);
        assert_eq!(ddq, [20.25, 21.25, 20.5, 21.5]);
        let scratch = evaluator.scratch.lock().unwrap();
        assert_eq!(scratch.dddq.len(), 0);
    }
}
