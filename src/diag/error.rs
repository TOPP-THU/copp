use clarabel::solver::{SolverError as ClarabelSolverError, SolverStatus as ClarabelSolverStatus};
use thiserror::Error;

/// The error type for COPP.
#[derive(Error)]
pub enum CoppError {
    /// Some error occurred in filesystem I/O operations, such as opening/creating log files.
    #[error("I/O error: {0}")]
    IoError(#[from] std::io::Error),
    /// Some error occurred in the Constraint struct, such as invalid input or computation failure.
    #[error("Some error occurred in the Constraint struct: {0}")]
    ConstraintError(#[from] ConstraintError),
    /// Some error occurred in the Path struct, such as invalid input or computation failure.
    #[error("Some error occurred in the Path struct: {0}")]
    PathError(#[from] PathError),
    /// Some error occurred while evaluating user-provided robot inverse dynamics.
    #[error("Robot dynamics error: {0}")]
    RobotDynamicsError(#[from] RobotDynamicsError),
    /// The specified optimization problem is reported infeasible by the backend solver.
    #[error("{0} reported an infeasibility: {1}")]
    Infeasible(String, String),
    /// The specified optimization problem is reported unbounded by the backend solver.
    #[error("{0} reported an unboundedness: {1}")]
    Unbounded(String, String),
    /// The solver/backend rejects the given model or data as invalid input.
    #[error("{0} reported an invalid input: {1}")]
    InvalidInput(String, String),
    /// The solver/backend rejects the provided configuration options.
    #[error("{0} reported an invalid options: {1}")]
    InvalidOptions(String, String),
    /// The Clarabel solver returned a concrete internal solver error.
    #[error("{0} reported an error in Clarabel solver: {1}")]
    ClarabelSolverError(String, #[source] ClarabelSolverError),
    /// The Clarabel solver terminated with a non-success status.
    #[error("{0} reported a failure in Clarabel solver with status {1}")]
    ClarabelSolverStatus(String, ClarabelSolverStatus),
    /// A backend-specific or uncategorized runtime error is reported.
    #[error("{0} reported an error: {1}")]
    Other(String, String),
}

/// Error type for user-provided robot inverse-dynamics evaluation.
///
/// This type intentionally stores a free-form message so robot integrations can
/// report errors from external dynamics libraries without fitting them into a
/// fixed COPP-specific taxonomy.
#[derive(Error, Debug, Clone, PartialEq, Eq)]
#[error("{message}")]
pub struct RobotDynamicsError {
    message: String,
}

impl RobotDynamicsError {
    /// Construct a robot dynamics error from a display-ready message.
    #[inline]
    pub fn new(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
        }
    }

    /// Borrow the underlying error message.
    #[inline]
    pub fn message(&self) -> &str {
        &self.message
    }

    /// Consume the error and return its message.
    #[inline]
    pub fn into_message(self) -> String {
        self.message
    }
}

impl From<String> for RobotDynamicsError {
    #[inline]
    fn from(message: String) -> Self {
        Self::new(message)
    }
}

impl From<&str> for RobotDynamicsError {
    #[inline]
    fn from(message: &str) -> Self {
        Self::new(message)
    }
}

/// Error type for constraint storage/query operations.
///
/// # Usage recommendation
/// For public-facing application code, prefer using [`CoppError`](crate::diag::CoppError)
/// as the unified error type.
///
/// [`ConstraintError`](crate::diag::ConstraintError) is automatically converted into [`CoppError`](crate::diag::CoppError) via
/// `From<ConstraintError> for CoppError`, so `?` can be used directly when
/// your function returns `Result<_, CoppError>`.
#[derive(Error, Debug)]
pub enum ConstraintError {
    /// Input station sequence is not strictly increasing.
    #[error("`s` must be strictly increasing; first violation at local index {index}.")]
    NonIncreasingS {
        /// Local index of the first non-increasing station.
        index: usize,
    },

    /// Input matrix/vector dimensions are incompatible with expected shape.
    #[error("Input dimensions do not match the expected shape.")]
    NoMatchDimensions,

    /// Input ordering contract is violated.
    #[error("Input order does not satisfy the expected contract.")]
    NoMatchOrder,

    /// Signed upper/lower bounds violate strict feasibility contract.
    #[error(
        "`{bound_name}` requires strict signed limits at every station: upper bound > 0 and lower bound < 0."
    )]
    InvalidSignedBounds {
        /// Name of the bound array that violated the signed-bound contract.
        bound_name: &'static str,
    },

    /// Requested station interval is outside currently stored constraints range.
    #[error("Requested station interval is out of bounds: idx_s={idx_s}, len={len}.")]
    OutOfSBounds {
        /// Requested starting station index.
        idx_s: usize,
        /// Number of stations currently stored.
        len: usize,
    },

    /// `a` violates positivity / non-negativity preconditions.
    #[error(
        "Input `a` violates positivity requirements (must be nonnegative, and strictly positive where required)."
    )]
    NonPositiveA,

    /// Linearization floor must be strictly positive.
    #[error("Linearization floor must be strictly positive.")]
    NonPositiveLinearizationFloor,

    /// Required path-derivative data is missing in the requested interval.
    #[error(
        "Required derivative data (`q`, `dq`, `ddq`, `dddq`) is not fully available in the requested interval."
    )]
    NoGivenQInfo,

    /// Linearized jerk constraints are unavailable at the requested station.
    #[error(
        "Linearized jerk constraints are unavailable at idx_s={idx_s}; valid range is [{}, {}).",
        valid_range.0,
        valid_range.1
    )]
    LinearJerkNotAvailable {
        /// Requested station index.
        idx_s: usize,
        /// Half-open station range for which linearized jerk data is available.
        valid_range: (usize, usize),
    },

    /// Dynamic-model data has not been provided.
    #[error("Required dynamic-model information is not available.")]
    NoDynamic,

    /// Reference profile is infeasible under current constraints.
    #[error("Reference profile is infeasible under current constraints.")]
    InfeasibleReference,

    /// Requested interval is empty.
    #[error("Requested interval is empty: {start} <= idx_s < {end}.")]
    EmptyInterval {
        /// Start index of the requested interval.
        start: usize,
        /// End index of the requested interval.
        end: usize,
    },
}

/// Error type for path construction and path evaluation APIs.
///
/// This error is returned by path-related modules such as [`Path`](crate::path::Path)
/// and spline utilities when input data, parameter ranges, or numerical systems
/// are invalid.
#[derive(Error, Debug)]
pub enum PathError {
    /// Path dimension is invalid (typically zero).
    #[error("invalid dimension: {dim}")]
    InvalidDimension {
        /// Requested path dimension.
        dim: usize,
    },
    /// Path parameter range is invalid (must satisfy finite `s_min < s_max`).
    #[error("invalid s range: [{s_min}, {s_max}]")]
    InvalidRange {
        /// Lower endpoint of the invalid parameter range.
        s_min: f64,
        /// Upper endpoint of the invalid parameter range.
        s_max: f64,
    },
    /// Spline order is invalid (must satisfy required minimum/order constraints).
    #[error("invalid spline order: {order}, expected >= 3")]
    InvalidOrder {
        /// Requested spline order.
        order: usize,
    },
    /// Matrix/tensor shapes are incompatible for the requested operation.
    #[error("dimension mismatch")]
    DimensionMismatch,
    /// The path representation cannot provide the requested derivative order.
    #[error("unsupported derivative order: requested {requested}, available {available}")]
    UnsupportedDerivativeOrder {
        /// Requested derivative order.
        requested: usize,
        /// Highest derivative order available from the path representation.
        available: usize,
    },
    /// Waypoint sequence is too short to build a valid path.
    #[error("not enough waypoints: {n}, expected >= 2")]
    NotEnoughWaypoints {
        /// Number of supplied waypoints.
        n: usize,
    },
    /// Query parameter `s` is outside the configured valid interval.
    #[error("s out of range [{s_min}, {s_max}] at index {index}: {value}")]
    OutOfRangeS {
        /// Lower endpoint of the valid parameter range.
        s_min: f64,
        /// Upper endpoint of the valid parameter range.
        s_max: f64,
        /// Index of the out-of-range query value.
        index: usize,
        /// Out-of-range query value.
        value: f64,
    },
    /// Boundary conditions are not supported for the requested spline order.
    #[error("unsupported boundary for order={order}")]
    UnsupportedBoundary {
        /// Requested spline order.
        order: usize,
    },
    /// Internal linear system is singular and cannot be solved robustly.
    #[error("singular linear system")]
    SingularSystem,
}

/// Force the debug format of ToppError to be the same as the display format, which is more concise and user-friendly.
impl std::fmt::Debug for CoppError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self}")
    }
}

/// Check whether a given s_interval is valid.
#[inline(always)]
pub(crate) fn check_s_interval_valid(
    function_name: &str,
    idx_s_start: usize,
    idx_s_final: usize,
) -> Result<(), CoppError> {
    if idx_s_final < idx_s_start + 2 {
        Err(CoppError::InvalidInput(
            function_name.into(),
            format!(
                "The final index {idx_s_final} must be at least two positions after the start index {idx_s_start} in Topp2Problem."
            ),
        ))
    } else {
        Ok(())
    }
}

/// Check the positivity of a given tolerance.
#[inline(always)]
pub(crate) fn check_abs_rel_tol(
    function_name: &str,
    abs_tol_name: &str,
    abs_tol: f64,
    rel_tol_name: &str,
    rel_tol: f64,
) -> Result<(), CoppError> {
    check_options_not_nan_infinite(function_name, abs_tol_name, abs_tol)?;
    check_options_not_nan_infinite(function_name, rel_tol_name, rel_tol)?;
    if abs_tol >= 0.0 && rel_tol >= 0.0 && (abs_tol > f64::EPSILON || rel_tol > f64::EPSILON) {
        Ok(())
    } else {
        Err(CoppError::InvalidOptions(
            function_name.into(),
            format!(
                "At least one of {abs_tol_name} = {abs_tol} and {rel_tol_name} = {rel_tol} must be strictly positive."
            ),
        ))
    }
}

#[inline(always)]
pub(crate) fn check_options_not_nan_infinite(
    function_name: &str,
    var_name: &str,
    var_value: f64,
) -> Result<(), CoppError> {
    if var_value.is_nan() {
        Err(CoppError::InvalidOptions(
            function_name.into(),
            format!("{var_name} = {var_value} must not be NaN",),
        ))
    } else if var_value.is_infinite() {
        Err(CoppError::InvalidOptions(
            function_name.into(),
            format!("{var_name} = {var_value} must not be infinite",),
        ))
    } else {
        Ok(())
    }
}

/// Check that a scalar input value is neither NaN nor infinite.
///
/// This is the input-data counterpart of [`check_not_nan_infinite`], which is
/// reserved for option validation and therefore returns [`InvalidOptions`](CoppError::InvalidOptions).
#[inline(always)]
pub(crate) fn check_input_not_nan_infinite(
    function_name: &str,
    var_name: &str,
    var_value: f64,
) -> Result<(), CoppError> {
    if var_value.is_nan() {
        Err(CoppError::InvalidInput(
            function_name.into(),
            format!("{var_name} = {var_value} must not be NaN"),
        ))
    } else if var_value.is_infinite() {
        Err(CoppError::InvalidInput(
            function_name.into(),
            format!("{var_name} = {var_value} must not be infinite"),
        ))
    } else {
        Ok(())
    }
}

/// Check that every value in an input slice is neither NaN nor infinite.
///
/// The reported variable name includes the first offending local index, which
/// keeps interpolation and solver diagnostics precise without duplicating this
/// scan logic in each module.
#[inline(always)]
pub(crate) fn check_input_slice_not_nan_infinite(
    function_name: &str,
    slice_name: &str,
    values: &[f64],
) -> Result<(), CoppError> {
    if let Some((index, value)) = values.iter().enumerate().find(|(_, value)| value.is_nan()) {
        Err(CoppError::InvalidInput(
            function_name.into(),
            format!("`{slice_name}[{index}]` = {value} must not be NaN"),
        ))
    } else if let Some((index, value)) = values
        .iter()
        .enumerate()
        .find(|(_, value)| value.is_infinite())
    {
        Err(CoppError::InvalidInput(
            function_name.into(),
            format!("`{slice_name}[{index}]` = {value} must not be infinite"),
        ))
    } else {
        Ok(())
    }
}

/// Check that an input slice is strictly increasing.
///
/// This check assumes finiteness has already been verified when NaN-specific
/// diagnostics are needed; otherwise comparisons involving NaN simply fail the
/// ordering contract at the first affected pair.
#[inline(always)]
pub(crate) fn check_input_strictly_increasing(
    function_name: &str,
    slice_name: &str,
    values: &[f64],
) -> Result<(), CoppError> {
    if let Some(index) = values.windows(2).position(|pair| pair[0] >= pair[1]) {
        Err(CoppError::InvalidInput(
            function_name.into(),
            format!(
                "`{slice_name}` must be strictly increasing; first violation at local index {index}."
            ),
        ))
    } else {
        Ok(())
    }
}

/// Check that an input scalar is nonnegative.
///
/// This helper first rejects NaN and infinity so downstream numerical code can
/// safely use ordinary comparisons and square roots.
#[inline(always)]
pub(crate) fn check_input_non_negative(
    function_name: &str,
    var_name: &str,
    var_value: f64,
) -> Result<(), CoppError> {
    check_input_not_nan_infinite(function_name, var_name, var_value)?;
    if var_value < 0.0 {
        Err(CoppError::InvalidInput(
            function_name.into(),
            format!("{var_name} = {var_value} must be nonnegative"),
        ))
    } else {
        Ok(())
    }
}

/// Check that every value in an input slice is nonnegative.
///
/// The function reports the first offending entry and is intended for data
/// profiles such as sampled `a(s)` that must be valid before interpolation.
#[inline(always)]
pub(crate) fn check_input_slice_non_negative(
    function_name: &str,
    slice_name: &str,
    values: &[f64],
) -> Result<(), CoppError> {
    check_input_slice_not_nan_infinite(function_name, slice_name, values)?;
    if let Some((index, value)) = values.iter().enumerate().find(|(_, value)| **value < 0.0) {
        Err(CoppError::InvalidInput(
            function_name.into(),
            format!("`{slice_name}[{index}]` = {value} must be nonnegative"),
        ))
    } else {
        Ok(())
    }
}

/// Check that an input length is at least the required minimum.
///
/// Callers pass display-ready length names such as `` `s.len()` `` so error
/// messages can mirror the notation used in each API contract.
#[inline(always)]
pub(crate) fn check_input_len_at_least(
    function_name: &str,
    len_name: &str,
    len: usize,
    min_len: usize,
) -> Result<(), CoppError> {
    if len < min_len {
        Err(CoppError::InvalidInput(
            function_name.into(),
            format!("{len_name} = {len} must be at least {min_len}"),
        ))
    } else {
        Ok(())
    }
}

/// Check that two input lengths are equal.
///
/// This is used for shape contracts where both the actual and reference lengths
/// are useful to report to the caller.
#[inline(always)]
pub(crate) fn check_input_len_equal(
    function_name: &str,
    lhs_name: &str,
    lhs_len: usize,
    rhs_name: &str,
    rhs_len: usize,
) -> Result<(), CoppError> {
    if lhs_len != rhs_len {
        Err(CoppError::InvalidInput(
            function_name.into(),
            format!("{lhs_name} = {lhs_len} must equal {rhs_name} = {rhs_len}"),
        ))
    } else {
        Ok(())
    }
}

/// Check that an input slice is not empty.
///
/// This helper is for APIs where an empty user-provided sample grid is
/// ambiguous and should be rejected before interpolation starts.
#[inline(always)]
pub(crate) fn check_input_not_empty(
    function_name: &str,
    slice_name: &str,
    len: usize,
) -> Result<(), CoppError> {
    if len == 0 {
        Err(CoppError::InvalidInput(
            function_name.into(),
            format!("{slice_name} must not be empty"),
        ))
    } else {
        Ok(())
    }
}

/// Check the non-negativity.
#[inline(always)]
pub(crate) fn check_non_negative(
    function_name: &str,
    var_name: &str,
    var_value: f64,
) -> Result<(), CoppError> {
    check_options_not_nan_infinite(function_name, var_name, var_value)?;
    if var_value < 0.0 {
        Err(CoppError::InvalidOptions(
            function_name.into(),
            format!("{var_name} = {var_value} must be strictly non-negative"),
        ))
    } else {
        Ok(())
    }
}

/// Check the positivity of a given tolerance.
#[inline(always)]
pub(crate) fn check_strictly_positive(
    function_name: &str,
    var_name: &str,
    var_value: f64,
) -> Result<(), CoppError> {
    check_options_not_nan_infinite(function_name, var_name, var_value)?;
    if var_value < f64::EPSILON {
        Err(CoppError::InvalidOptions(
            function_name.into(),
            format!("{var_name} = {var_value} must be strictly positive"),
        ))
    } else {
        Ok(())
    }
}

#[inline(always)]
pub(crate) fn check_boundary_state_copp3_valid(
    a_boundary: (f64, f64),
    b_boundary: (f64, f64),
) -> Result<(), CoppError> {
    if a_boundary.0 < 0.0 {
        return Err(CoppError::InvalidInput(
            "copp3_socp".into(),
            format!("The initial a = {} must be non-negative.", a_boundary.0),
        ));
    }
    if a_boundary.1 < 0.0 {
        return Err(CoppError::InvalidInput(
            "copp3_socp".into(),
            format!("The terminal a = {} must be non-negative.", a_boundary.1),
        ));
    }
    if a_boundary.0.abs() < f64::EPSILON {
        // If a[0]==0 but b[0]!=0, then a<0 will occur near s_start
        if b_boundary.0.abs() >= f64::EPSILON {
            return Err(CoppError::InvalidInput(
                "copp3_socp".into(),
                format!(
                    "The initial a = {} is zero, so the initial b = {} must also be zero.",
                    a_boundary.0, b_boundary.0
                ),
            ));
        }
    }
    if a_boundary.1.abs() < f64::EPSILON {
        // If a[n]==0 but b[n]!=0, then a<0 will occur near s_final
        if b_boundary.1.abs() >= f64::EPSILON {
            return Err(CoppError::InvalidInput(
                "copp3_socp".into(),
                format!(
                    "The terminal a = {} is zero, so the terminal b = {} must also be zero.",
                    a_boundary.1, b_boundary.1
                ),
            ));
        }
    }
    Ok(())
}
