//! Private Rust/C++ bridge backing the hand-written C++ facade.
//!
//! # Layering
//! Public C++ users should never include or call generated `cxx` artifacts
//! directly. The intended call path is:
//!
//! ```text
//! user C++ code
//!   -> bindings/cpp/include/copp/*.hpp      (stable hand-written facade)
//!   -> bindings/cpp/src/*.cpp              (argument checks and copying)
//!   -> this cxx bridge module              (opaque Rust handles)
//!   -> Rust Path/Robot/Solver implementations
//! ```
//!
//! Keeping this layer private lets the C++ API stay idiomatic (`copp::Path`,
//! `copp::Robot`, namespaces, RAII, overloads) while still using Rust as the
//! source of truth for algorithms and validation.
//!
//! # Data ownership and layout
//! The bridge uses borrowed slices for inputs and boxed opaque Rust result
//! objects for outputs. The C++ facade copies output slices immediately into
//! `std::vector<double>`/`copp::Matrix`, so no Rust-owned slice escapes past the
//! call that produced it. Matrix payloads are column-major unless a
//! [`MatrixDescriptor`] explicitly says otherwise; result matrices follow
//! nalgebra storage and are copied into C++ `Matrix` with the same layout.
//!
//! # Error propagation
//! Rust functions return `Result<T, String>` where the public C++ API should
//! throw `copp::Error`. C++ callbacks may throw `copp::Error` or
//! `std::exception`; `cxx` transports those exceptions as bridge errors, and
//! this module converts them into Rust `PathError` / `RobotDynamicsError` where
//! appropriate.
//!
//! # Thread-safety contract
//! C++ callbacks are stored behind bridge objects that serialize calls with a
//! mutex on the C++ side. Rust traits require `Send + Sync`, so this module
//! marks the bridge wrappers as `Send`/`Sync` with that serialization contract.

#[cxx::bridge(namespace = "copp::bridge")]
pub mod ffi {
    /// Internal matrix-view descriptor passed by the C++ facade.
    ///
    /// `layout` uses the private bridge convention:
    /// - `0`: column-major, element `(r, c)` at `r + c * leading_dim`
    /// - `1`: row-major, element `(r, c)` at `c + r * leading_dim`
    struct MatrixDescriptor {
        rows: usize,
        cols: usize,
        leading_dim: usize,
        layout: u8,
    }

    /// Internal flattened COPP objective descriptor.
    ///
    /// `kind` mirrors Rust/C objective ids:
    /// - `0`: time
    /// - `1`: linear
    /// - `2`: thermal energy
    /// - `3`: total-variation torque
    ///
    /// Vector payloads live in one flat `objective_data` slice. Linear uses
    /// `first = alpha` and `second = beta`; torque objectives use
    /// `first = normalize`.
    struct ObjectiveDescriptor {
        kind: u8,
        weight: f64,
        first_offset: usize,
        first_len: usize,
        second_offset: usize,
        second_len: usize,
    }

    /// Internal direct-solver metadata copied from Clarabel settings/results.
    ///
    /// `method` uses the private bridge convention:
    /// - `0`: auto
    /// - `1`: qdldl
    /// - `2`: faer
    /// - `3`: mkl
    /// - `4`: panua
    struct LinearSolverInfoBridge {
        method: u8,
        threads: usize,
        direct: bool,
        nnz_a: usize,
        nnz_l: usize,
    }

    /// Internal raw Clarabel settings.
    ///
    /// This mirrors `clarabel::solver::DefaultSettings<f64>` and the public
    /// C++ `copp::clarabel::Settings` type.
    struct ClarabelSettingsBridge {
        max_iter: u32,
        time_limit: f64,
        verbose: bool,
        max_step_fraction: f64,
        tol_gap_abs: f64,
        tol_gap_rel: f64,
        tol_feas: f64,
        tol_infeas_abs: f64,
        tol_infeas_rel: f64,
        tol_ktratio: f64,
        reduced_tol_gap_abs: f64,
        reduced_tol_gap_rel: f64,
        reduced_tol_feas: f64,
        reduced_tol_infeas_abs: f64,
        reduced_tol_infeas_rel: f64,
        reduced_tol_ktratio: f64,
        equilibrate_enable: bool,
        equilibrate_max_iter: u32,
        equilibrate_min_scaling: f64,
        equilibrate_max_scaling: f64,
        linesearch_backtrack_step: f64,
        min_switch_step_length: f64,
        min_terminate_step_length: f64,
        max_threads: u32,
        direct_kkt_solver: bool,
        direct_solve_method: u8,
        static_regularization_enable: bool,
        static_regularization_constant: f64,
        static_regularization_proportional: f64,
        dynamic_regularization_enable: bool,
        dynamic_regularization_eps: f64,
        dynamic_regularization_delta: f64,
        iterative_refinement_enable: bool,
        iterative_refinement_reltol: f64,
        iterative_refinement_abstol: f64,
        iterative_refinement_max_iter: u32,
        iterative_refinement_stop_ratio: f64,
        presolve_enable: bool,
        input_sparse_dropzeros: bool,
    }

    /// Internal Clarabel option bundle.
    struct ClarabelOptionsBridge {
        verbosity: u8,
        allow_almost_solved: bool,
        allow_max_iterations: bool,
        allow_max_time: bool,
        allow_callback_terminated: bool,
        allow_insufficient_progress: bool,
        clarabel_settings: ClarabelSettingsBridge,
    }

    /// Internal validated TOPP3/COPP3 problem metadata.
    struct Topp3ProblemInfoBridge {
        s_len: usize,
        idx_s_final: usize,
        num_stationary_start: usize,
        num_stationary_end: usize,
    }

    // C++ callback objects owned by Rust-side path/robot wrappers.
    //
    // These are declared as opaque generated bridge types. Their concrete
    // definitions live in `bindings/cpp/include/copp/detail/*_bridge.hpp` and
    // `bindings/cpp/src/*.cpp`. Every callback uses batch-style buffers: Rust
    // passes all samples in one call, and C++ fills column-major output slices.
    unsafe extern "C++" {
        include!("copp/detail/path_bridge.hpp");
        include!("copp/detail/robot_bridge.hpp");

        // Batch path evaluator with derivatives up to second order.
        type CppPathEvaluator2nd;
        // Batch path evaluator with derivatives up to third order.
        type CppPathEvaluator3rd;
        // Robot inverse-dynamics callback supplied by C++.
        type CppInverseDynamics;

        // Return the evaluator dimension. Used once during path construction.
        fn dim(self: &CppPathEvaluator2nd) -> usize;
        // Fill `q`, `dq`, and `ddq` for all samples in `s`.
        //
        // Buffers are flat column-major matrices of shape `(dim, s.len())`.
        // Any C++ exception is converted by `cxx` into an error result.
        fn evaluate_up_to_2nd(
            self: &CppPathEvaluator2nd,
            s: &[f64],
            q: &mut [f64],
            dq: &mut [f64],
            ddq: &mut [f64],
        ) -> Result<()>;

        // Return the evaluator dimension. Used once during path construction.
        fn dim(self: &CppPathEvaluator3rd) -> usize;
        // Fill second-order outputs from a third-order evaluator.
        //
        // The C++ facade computes and discards `dddq` internally so a
        // third-order path can still satisfy second-order solver requests.
        fn evaluate_up_to_2nd(
            self: &CppPathEvaluator3rd,
            s: &[f64],
            q: &mut [f64],
            dq: &mut [f64],
            ddq: &mut [f64],
        ) -> Result<()>;
        // Fill `q`, `dq`, `ddq`, and `dddq` for all samples in `s`.
        fn evaluate_up_to_3rd(
            self: &CppPathEvaluator3rd,
            s: &[f64],
            q: &mut [f64],
            dq: &mut [f64],
            ddq: &mut [f64],
            dddq: &mut [f64],
        ) -> Result<()>;

        // Return robot dimension / DoF.
        fn dim(self: &CppInverseDynamics) -> usize;
        // Evaluate one inverse-dynamics state.
        //
        // Inputs and output have length `dim`. Without a C++ callback,
        // `CppRobotModel` falls back to point-mass dynamics (`tau = ddq`) and
        // this bridge function is not called.
        fn inverse_dynamics(
            self: &CppInverseDynamics,
            q: &[f64],
            dq: &[f64],
            ddq: &[f64],
            tau: &mut [f64],
        ) -> Result<()>;
    }

    // Rust functions exported to the private C++ facade implementation.
    //
    // The declarations below intentionally mirror the public C++ feature
    // groups: version/options, interpolation, path construction/evaluation,
    // robot/constraint operations, and solver entry points. Public C++ headers
    // wrap these raw bridge functions with stronger types and C++ overloads.
    extern "Rust" {
        type PathEval2Result;
        type PathEval3Result;
        type PathEvalQResult;
        type PathHandle;
        type Profile3rdResult;
        type ReachSet2Result;
        type RobotHandle;
        type Copp2SocpResult;
        type Topp3SolverResult;
        type TimeProfile2Result;
        type VecF64Result;

        // Return the Rust crate version string used by `copp::version()`.
        fn version() -> &'static str;

        // Copy Rust-side Clarabel defaults into the C++ settings struct.
        fn clarabel_default_settings() -> ClarabelSettingsBridge;
        // Copy Rust-side Clarabel option defaults into the C++ options struct.
        fn clarabel_default_options() -> ClarabelOptionsBridge;

        // -----------------------------------------------------------------
        // Interpolation helpers.
        //
        // Inputs are borrowed C++ spans. Outputs are boxed Rust vectors or
        // `TimeProfile2Result`, then copied by `bindings/cpp/src/interpolation.cpp`.
        // TOPP2 uses node profile `a[k] = (ds/dt)^2`. TOPP3 uses node profiles
        // `a[k] = (ds/dt)^2` and `b[k] = d^2s/dt^2`; stationary counts describe
        // how many boundary intervals are exactly stationary.
        // -----------------------------------------------------------------
        fn a_to_b_topp2(s: &[f64], a: &[f64]) -> Result<Box<VecF64Result>>;
        fn s_to_t_topp2(s: &[f64], a: &[f64], t0: f64) -> Box<TimeProfile2Result>;
        fn t_to_s_topp2_uniform(
            s: &[f64],
            a: &[f64],
            t_s: &[f64],
            t0: f64,
            dt: f64,
            include_final: bool,
        ) -> Result<Box<VecF64Result>>;
        fn t_to_s_topp2_samples(
            s: &[f64],
            a: &[f64],
            t_s: &[f64],
            t_sample: &[f64],
        ) -> Result<Box<VecF64Result>>;
        fn s_to_t_topp3(
            s: &[f64],
            a: &[f64],
            b: &[f64],
            num_stationary_start: usize,
            num_stationary_end: usize,
            t0: f64,
        ) -> Box<TimeProfile2Result>;
        fn t_to_s_topp3_uniform(
            s: &[f64],
            a: &[f64],
            b: &[f64],
            num_stationary_start: usize,
            num_stationary_end: usize,
            t_s: &[f64],
            t0: f64,
            dt: f64,
            include_final: bool,
        ) -> Result<Box<VecF64Result>>;
        fn t_to_s_topp3_samples(
            s: &[f64],
            a: &[f64],
            b: &[f64],
            num_stationary_start: usize,
            num_stationary_end: usize,
            t_s: &[f64],
            t_sample: &[f64],
        ) -> Result<Box<VecF64Result>>;
        fn ok(self: &TimeProfile2Result) -> bool;
        fn message(self: &TimeProfile2Result) -> String;
        fn t_final(self: &TimeProfile2Result) -> f64;
        fn t_s(self: &TimeProfile2Result) -> &[f64];
        fn a(self: &Profile3rdResult) -> &[f64];
        fn b(self: &Profile3rdResult) -> &[f64];
        fn num_stationary_start(self: &Profile3rdResult) -> usize;
        fn num_stationary_end(self: &Profile3rdResult) -> usize;

        // -----------------------------------------------------------------
        // Path construction and evaluation.
        //
        // `path_from_waypoints` accepts a column-major view described by
        // `(rows, cols, leading_dim)`. The public C++ layer converts row-major
        // inputs before crossing the bridge. Evaluator paths take ownership of
        // opaque C++ callback objects and invoke them through Rust Path traits.
        // -----------------------------------------------------------------
        fn path_from_waypoints(
            data: &[f64],
            rows: usize,
            cols: usize,
            leading_dim: usize,
            order: usize,
            s_min: f64,
            s_max: f64,
            out_of_range_mode: u8,
            start_state_data: &[f64],
            start_state_rows: usize,
            start_state_cols: usize,
            start_state_leading_dim: usize,
            end_state_data: &[f64],
            end_state_rows: usize,
            end_state_cols: usize,
            end_state_leading_dim: usize,
        ) -> Result<Box<PathHandle>>;
        fn path_from_evaluator_2nd(
            evaluator: UniquePtr<CppPathEvaluator2nd>,
            s_min: f64,
            s_max: f64,
        ) -> Result<Box<PathHandle>>;
        fn path_from_evaluator_3rd(
            evaluator: UniquePtr<CppPathEvaluator3rd>,
            s_min: f64,
            s_max: f64,
        ) -> Result<Box<PathHandle>>;
        fn robot_new(dim: usize, capacity: usize) -> Result<Box<RobotHandle>>;
        fn robot_new_with_inverse_dynamics(
            inverse_dynamics: UniquePtr<CppInverseDynamics>,
            capacity: usize,
        ) -> Result<Box<RobotHandle>>;
        fn dim(self: &PathHandle) -> usize;
        fn s_min(self: &PathHandle) -> f64;
        fn s_max(self: &PathHandle) -> f64;
        fn evaluate_q(self: &PathHandle, s: &[f64]) -> Result<Box<PathEvalQResult>>;
        fn evaluate_up_to_2nd(self: &PathHandle, s: &[f64]) -> Result<Box<PathEval2Result>>;
        fn evaluate_up_to_3rd(self: &PathHandle, s: &[f64]) -> Result<Box<PathEval3Result>>;

        // Borrow position-only evaluation output as a flat column-major slice.
        fn q(self: &PathEvalQResult) -> &[f64];

        // Borrow second-order evaluation outputs as flat column-major slices.
        fn q(self: &PathEval2Result) -> &[f64];
        fn dq(self: &PathEval2Result) -> &[f64];
        fn ddq(self: &PathEval2Result) -> &[f64];

        // Borrow third-order evaluation outputs as flat column-major slices.
        fn q(self: &PathEval3Result) -> &[f64];
        fn dq(self: &PathEval3Result) -> &[f64];
        fn ddq(self: &PathEval3Result) -> &[f64];
        fn dddq(self: &PathEval3Result) -> &[f64];

        // -----------------------------------------------------------------
        // Robot and constraint-buffer operations.
        //
        // The same Rust handle backs both C++ `Robot` and independent
        // `Constraints`. Constraint APIs use global station ids, matching Rust
        // and Python. Matrix inputs are accompanied by `MatrixDescriptor` so the
        // bridge can safely accept both row-major and column-major C++ views.
        // -----------------------------------------------------------------
        fn dim(self: &RobotHandle) -> usize;
        fn len(self: &RobotHandle) -> usize;
        fn capacity(self: &RobotHandle) -> usize;
        fn is_empty(self: &RobotHandle) -> bool;
        fn idx_s_start(self: &RobotHandle) -> usize;
        fn idx_s_end(self: &RobotHandle) -> usize;
        fn s_values(
            self: &RobotHandle,
            idx_s_from: usize,
            idx_s_to: usize,
        ) -> Result<Box<VecF64Result>>;
        fn amax_values(
            self: &RobotHandle,
            idx_s_from: usize,
            idx_s_to: usize,
        ) -> Result<Box<VecF64Result>>;
        fn s_value(self: &RobotHandle, idx_s: usize) -> Result<f64>;
        fn amax_value(self: &RobotHandle, idx_s: usize) -> Result<f64>;
        fn has_inverse_dynamics(self: &RobotHandle) -> bool;
        fn set_inverse_dynamics(
            self: &mut RobotHandle,
            inverse_dynamics: UniquePtr<CppInverseDynamics>,
        ) -> Result<()>;
        fn clear_inverse_dynamics(self: &mut RobotHandle);
        fn append_s(self: &mut RobotHandle, s: &[f64]) -> Result<()>;
        fn topp2_problem_s_len(
            self: &RobotHandle,
            idx_s_start: usize,
            idx_s_final: usize,
            a_start: f64,
            a_final: f64,
        ) -> Result<usize>;
        fn copp2_problem_s_len(
            self: &RobotHandle,
            idx_s_start: usize,
            idx_s_final: usize,
            a_start: f64,
            a_final: f64,
            objectives: &[ObjectiveDescriptor],
            objective_data: &[f64],
        ) -> Result<usize>;

        // -----------------------------------------------------------------
        // TOPP2 / COPP2 solver entry points.
        //
        // Public C++ `Problem` constructors validate interval length first via
        // `*_problem_s_len`. Solver functions then borrow the same constraint
        // window and return either an accepted `a` profile or an expert result.
        // -----------------------------------------------------------------
        fn reach_set2_backward(
            self: &RobotHandle,
            idx_s_start: usize,
            idx_s_final: usize,
            a_start: f64,
            a_final: f64,
            lp_feas_tol: f64,
            a_cmp_abs_tol: f64,
            a_cmp_rel_tol: f64,
            verbosity: u8,
        ) -> Result<Box<ReachSet2Result>>;
        fn reach_set2_bidirectional(
            self: &RobotHandle,
            idx_s_start: usize,
            idx_s_final: usize,
            a_start: f64,
            a_final: f64,
            lp_feas_tol: f64,
            a_cmp_abs_tol: f64,
            a_cmp_rel_tol: f64,
            verbosity: u8,
        ) -> Result<Box<ReachSet2Result>>;
        fn topp2_ra_solve(
            self: &RobotHandle,
            idx_s_start: usize,
            idx_s_final: usize,
            a_start: f64,
            a_final: f64,
            lp_feas_tol: f64,
            a_cmp_abs_tol: f64,
            a_cmp_rel_tol: f64,
            verbosity: u8,
        ) -> Result<Box<VecF64Result>>;
        fn copp2_socp_solve(
            self: &RobotHandle,
            idx_s_start: usize,
            idx_s_final: usize,
            a_start: f64,
            a_final: f64,
            objectives: &[ObjectiveDescriptor],
            objective_data: &[f64],
            options: ClarabelOptionsBridge,
        ) -> Result<Box<VecF64Result>>;
        fn copp2_socp_solve_expert(
            self: &RobotHandle,
            idx_s_start: usize,
            idx_s_final: usize,
            a_start: f64,
            a_final: f64,
            objectives: &[ObjectiveDescriptor],
            objective_data: &[f64],
            options: ClarabelOptionsBridge,
        ) -> Result<Box<Copp2SocpResult>>;

        // -----------------------------------------------------------------
        // TOPP3 / COPP3 solver entry points.
        //
        // Problem preparation immediately linearizes third-order constraints at
        // `a_linearization` and returns inferred stationary counts. RA/reach-set
        // calls optionally receive borrowed `a_given`/`b_given` profiles; the
        // C++ facade owns those buffers when long-lived options are copied.
        // -----------------------------------------------------------------
        fn topp3_problem_prepare(
            self: &mut RobotHandle,
            idx_s_start: usize,
            a_linearization: &[f64],
            a_start: f64,
            a_final: f64,
            b_start: f64,
            b_final: f64,
            num_stationary_max_start: usize,
            num_stationary_max_end: usize,
            a_linearization_floor: f64,
        ) -> Result<Topp3ProblemInfoBridge>;
        fn topp3_lp_solve(
            self: &mut RobotHandle,
            idx_s_start: usize,
            a_linearization: &[f64],
            a_start: f64,
            a_final: f64,
            b_start: f64,
            b_final: f64,
            num_stationary_max_start: usize,
            num_stationary_max_end: usize,
            a_linearization_floor: f64,
            options: ClarabelOptionsBridge,
        ) -> Result<Box<Profile3rdResult>>;
        fn topp3_lp_solve_expert(
            self: &mut RobotHandle,
            idx_s_start: usize,
            a_linearization: &[f64],
            a_start: f64,
            a_final: f64,
            b_start: f64,
            b_final: f64,
            num_stationary_max_start: usize,
            num_stationary_max_end: usize,
            a_linearization_floor: f64,
            options: ClarabelOptionsBridge,
        ) -> Result<Box<Topp3SolverResult>>;
        fn topp3_socp_solve(
            self: &mut RobotHandle,
            idx_s_start: usize,
            a_linearization: &[f64],
            a_start: f64,
            a_final: f64,
            b_start: f64,
            b_final: f64,
            num_stationary_max_start: usize,
            num_stationary_max_end: usize,
            a_linearization_floor: f64,
            options: ClarabelOptionsBridge,
        ) -> Result<Box<Profile3rdResult>>;
        fn topp3_socp_solve_expert(
            self: &mut RobotHandle,
            idx_s_start: usize,
            a_linearization: &[f64],
            a_start: f64,
            a_final: f64,
            b_start: f64,
            b_final: f64,
            num_stationary_max_start: usize,
            num_stationary_max_end: usize,
            a_linearization_floor: f64,
            options: ClarabelOptionsBridge,
        ) -> Result<Box<Topp3SolverResult>>;
        fn copp3_problem_prepare(
            self: &mut RobotHandle,
            idx_s_start: usize,
            a_linearization: &[f64],
            a_start: f64,
            a_final: f64,
            b_start: f64,
            b_final: f64,
            num_stationary_max_start: usize,
            num_stationary_max_end: usize,
            a_linearization_floor: f64,
            objectives: &[ObjectiveDescriptor],
            objective_data: &[f64],
        ) -> Result<Topp3ProblemInfoBridge>;
        fn copp3_socp_solve(
            self: &mut RobotHandle,
            idx_s_start: usize,
            a_linearization: &[f64],
            a_start: f64,
            a_final: f64,
            b_start: f64,
            b_final: f64,
            num_stationary_max_start: usize,
            num_stationary_max_end: usize,
            a_linearization_floor: f64,
            objectives: &[ObjectiveDescriptor],
            objective_data: &[f64],
            options: ClarabelOptionsBridge,
        ) -> Result<Box<Profile3rdResult>>;
        fn copp3_socp_solve_expert(
            self: &mut RobotHandle,
            idx_s_start: usize,
            a_linearization: &[f64],
            a_start: f64,
            a_final: f64,
            b_start: f64,
            b_final: f64,
            num_stationary_max_start: usize,
            num_stationary_max_end: usize,
            a_linearization_floor: f64,
            objectives: &[ObjectiveDescriptor],
            objective_data: &[f64],
            options: ClarabelOptionsBridge,
        ) -> Result<Box<Topp3SolverResult>>;
        fn set_q_2nd(
            self: &mut RobotHandle,
            q_data: &[f64],
            q_desc: MatrixDescriptor,
            dq_data: &[f64],
            dq_desc: MatrixDescriptor,
            ddq_data: &[f64],
            ddq_desc: MatrixDescriptor,
            idx_s: usize,
        ) -> Result<()>;
        fn set_q_3rd(
            self: &mut RobotHandle,
            q_data: &[f64],
            q_desc: MatrixDescriptor,
            dq_data: &[f64],
            dq_desc: MatrixDescriptor,
            ddq_data: &[f64],
            ddq_desc: MatrixDescriptor,
            dddq_data: &[f64],
            dddq_desc: MatrixDescriptor,
            idx_s: usize,
        ) -> Result<()>;
        fn set_q_from_path_2nd(
            self: &mut RobotHandle,
            path: &PathHandle,
            idx_s_from: usize,
            idx_s_to: usize,
        ) -> Result<()>;
        fn set_q_from_path_3rd(
            self: &mut RobotHandle,
            path: &PathHandle,
            idx_s_from: usize,
            idx_s_to: usize,
        ) -> Result<()>;
        fn add_limits_broadcast(
            self: &mut RobotHandle,
            kind: u8,
            upper: &[f64],
            lower: &[f64],
            start_idx_s: usize,
            length: usize,
        ) -> Result<()>;
        fn add_limits_matrix(
            self: &mut RobotHandle,
            kind: u8,
            upper_data: &[f64],
            upper_desc: MatrixDescriptor,
            lower_data: &[f64],
            lower_desc: MatrixDescriptor,
            start_idx_s: usize,
        ) -> Result<()>;
        fn amax_substitute(self: &mut RobotHandle, amax: &[f64], idx_s: usize) -> Result<()>;
        fn clear_constraints(self: &mut RobotHandle, keep_idx_s: bool);
        fn pop_front_n(self: &mut RobotHandle, n_cols: usize);
        fn pop_back_n(self: &mut RobotHandle, n_cols: usize);
        fn pop_front_until(self: &mut RobotHandle, idx_s_cut: usize);
        fn pop_back_until(self: &mut RobotHandle, idx_s_cut: usize);
        fn add_constraint_1st_vector(
            self: &mut RobotHandle,
            amax: &[f64],
            idx_s: usize,
        ) -> Result<()>;
        fn add_constraint_1st_matrix(
            self: &mut RobotHandle,
            amax_data: &[f64],
            amax_desc: MatrixDescriptor,
            idx_s: usize,
        ) -> Result<()>;
        fn add_constraint_2nd(
            self: &mut RobotHandle,
            acc_a_data: &[f64],
            acc_a_desc: MatrixDescriptor,
            acc_b_data: &[f64],
            acc_b_desc: MatrixDescriptor,
            acc_max_data: &[f64],
            acc_max_desc: MatrixDescriptor,
            idx_s: usize,
            is_negative: bool,
        ) -> Result<()>;
        fn add_constraint_3rd(
            self: &mut RobotHandle,
            jerk_a_data: &[f64],
            jerk_a_desc: MatrixDescriptor,
            jerk_b_data: &[f64],
            jerk_b_desc: MatrixDescriptor,
            jerk_c_data: &[f64],
            jerk_c_desc: MatrixDescriptor,
            jerk_d_data: &[f64],
            jerk_d_desc: MatrixDescriptor,
            jerk_max_data: &[f64],
            jerk_max_desc: MatrixDescriptor,
            idx_s: usize,
            is_negative: bool,
        ) -> Result<()>;

        // Borrow a generic vector result.
        fn values(self: &VecF64Result) -> &[f64];
        // Borrow second-order reachable interval arrays.
        fn a_max(self: &ReachSet2Result) -> &[f64];
        fn a_min(self: &ReachSet2Result) -> &[f64];

        // Borrow COPP2-SOCP expert fields.
        fn has_a(self: &Copp2SocpResult) -> bool;
        fn a(self: &Copp2SocpResult) -> &[f64];
        fn x(self: &Copp2SocpResult) -> &[f64];
        fn z(self: &Copp2SocpResult) -> &[f64];
        fn s(self: &Copp2SocpResult) -> &[f64];
        fn solver_status(self: &Copp2SocpResult) -> u8;
        fn obj_val(self: &Copp2SocpResult) -> f64;
        fn obj_val_dual(self: &Copp2SocpResult) -> f64;
        fn solve_time(self: &Copp2SocpResult) -> f64;
        fn iterations(self: &Copp2SocpResult) -> u32;
        fn r_prim(self: &Copp2SocpResult) -> f64;
        fn r_dual(self: &Copp2SocpResult) -> f64;
        fn linsolver(self: &Copp2SocpResult) -> LinearSolverInfoBridge;
        fn has_objective_value(self: &Copp2SocpResult) -> bool;
        fn objective_value(self: &Copp2SocpResult) -> f64;
        fn objective_terms(self: &Copp2SocpResult) -> &[f64];

        // Borrow shared TOPP3/COPP3 expert result fields.
        fn has_profile(self: &Topp3SolverResult) -> bool;
        fn profile_a(self: &Topp3SolverResult) -> &[f64];
        fn profile_b(self: &Topp3SolverResult) -> &[f64];
        fn profile_num_stationary_start(self: &Topp3SolverResult) -> usize;
        fn profile_num_stationary_end(self: &Topp3SolverResult) -> usize;
        fn x(self: &Topp3SolverResult) -> &[f64];
        fn z(self: &Topp3SolverResult) -> &[f64];
        fn s(self: &Topp3SolverResult) -> &[f64];
        fn solver_status(self: &Topp3SolverResult) -> u8;
        fn obj_val(self: &Topp3SolverResult) -> f64;
        fn obj_val_dual(self: &Topp3SolverResult) -> f64;
        fn solve_time(self: &Topp3SolverResult) -> f64;
        fn iterations(self: &Topp3SolverResult) -> u32;
        fn r_prim(self: &Topp3SolverResult) -> f64;
        fn r_dual(self: &Topp3SolverResult) -> f64;
        fn linsolver(self: &Topp3SolverResult) -> LinearSolverInfoBridge;
        fn has_objective_value(self: &Topp3SolverResult) -> bool;
        fn objective_value(self: &Topp3SolverResult) -> f64;
        fn objective_terms(self: &Topp3SolverResult) -> &[f64];
    }
}

use crate::copp::constraints::ModePopConstraints;
use crate::copp::copp2::opt2::copp2_socp::objective_value_copp2_opt;
use crate::copp::copp3::opt3::copp3_socp::objective_value_copp3_opt;
use crate::copp::{ClarabelOptionsBuilder, CoppObjective, InterpolationMode};
use crate::path::{OutOfRangeMode, Path, SplineConfig};
use crate::{
    diag::{PathError, RobotDynamicsError, Verbosity},
    path::{PathEvaluator2nd, PathEvaluator3rd},
    robot::{Robot as RustRobot, RobotBasic, RobotTorque},
};
use clarabel::solver::{DefaultSettings, DefaultSolution, LinearSolverInfo, SolverStatus};
use cxx::UniquePtr;
use nalgebra::{Const, DMatrix, DMatrixView, Dyn};

/// Rust adapter that implements [`PathEvaluator2nd`] by calling a C++ callback.
///
/// The public C++ object owns the user lambda. Rust owns this adapter through
/// `Path::from_evaluator_2nd`, and every evaluation crosses back into C++ once
/// per batch of samples. The `dim` field is cached because Rust path traits need
/// the dimension without making a callback call.
struct CppEvaluator2nd {
    inner: UniquePtr<ffi::CppPathEvaluator2nd>,
    dim: usize,
}

// SAFETY: Calls into the C++ evaluator are serialized by the C++ bridge object.
// The public C++ contract also treats callbacks as owned by `Path` and requires
// user captures to remain valid for the callback lifetime.
unsafe impl Send for CppEvaluator2nd {}
unsafe impl Sync for CppEvaluator2nd {}

impl PathEvaluator2nd for CppEvaluator2nd {
    fn dim(&self) -> usize {
        self.dim
    }

    fn evaluate_up_to_2nd(
        &self,
        s: &[f64],
        q: &mut [f64],
        dq: &mut [f64],
        ddq: &mut [f64],
    ) -> Result<(), PathError> {
        self.inner
            .evaluate_up_to_2nd(s, q, dq, ddq)
            .map_err(|error| PathError::EvaluatorError {
                message: error.to_string(),
            })
    }
}

/// Rust adapter that implements both second- and third-order path traits.
///
/// A third-order C++ evaluator can serve second-order consumers as well. The C++
/// side computes all derivatives and discards `dddq` for second-order calls,
/// keeping callback semantics simple and consistent.
struct CppEvaluator3rd {
    inner: UniquePtr<ffi::CppPathEvaluator3rd>,
    dim: usize,
}

// SAFETY: Calls into the C++ evaluator are serialized by the C++ bridge object.
// The public C++ contract also treats callbacks as owned by `Path` and requires
// user captures to remain valid for the callback lifetime.
unsafe impl Send for CppEvaluator3rd {}
unsafe impl Sync for CppEvaluator3rd {}

impl PathEvaluator2nd for CppEvaluator3rd {
    fn dim(&self) -> usize {
        self.dim
    }

    fn evaluate_up_to_2nd(
        &self,
        s: &[f64],
        q: &mut [f64],
        dq: &mut [f64],
        ddq: &mut [f64],
    ) -> Result<(), PathError> {
        self.inner
            .evaluate_up_to_2nd(s, q, dq, ddq)
            .map_err(|error| PathError::EvaluatorError {
                message: error.to_string(),
            })
    }
}

impl PathEvaluator3rd for CppEvaluator3rd {
    fn evaluate_up_to_3rd(
        &self,
        s: &[f64],
        q: &mut [f64],
        dq: &mut [f64],
        ddq: &mut [f64],
        dddq: &mut [f64],
    ) -> Result<(), PathError> {
        self.inner
            .evaluate_up_to_3rd(s, q, dq, ddq, dddq)
            .map_err(|error| PathError::EvaluatorError {
                message: error.to_string(),
            })
    }
}

/// Rust-side path handle owned by `copp::Path`.
///
/// The C++ facade stores this as `rust::Box<bridge::PathHandle>` inside a PIMPL.
/// It is opaque to public C++ headers, so changing the Rust path representation
/// does not change the public C++ ABI surface.
pub struct PathHandle {
    inner: Path,
}

fn optional_column_major_matrix(
    data: &[f64],
    rows: usize,
    cols: usize,
    leading_dim: usize,
    name: &str,
) -> Result<Option<DMatrix<f64>>, String> {
    if rows == 0 && cols == 0 && data.is_empty() {
        return Ok(None);
    }
    if rows == 0 || cols == 0 {
        return Err(format!(
            "path_from_waypoints: `{name}` must either be empty or have positive rows and cols"
        ));
    }
    if leading_dim < rows {
        return Err(format!(
            "path_from_waypoints: `{name}` leading_dim must be at least rows"
        ));
    }
    let required_len = leading_dim
        .checked_mul(cols.saturating_sub(1))
        .and_then(|prefix| prefix.checked_add(rows))
        .ok_or_else(|| format!("path_from_waypoints: `{name}` matrix view size overflow"))?;
    if data.len() < required_len {
        return Err(format!(
            "path_from_waypoints: `{name}` data is shorter than shape"
        ));
    }

    Ok(Some(DMatrix::from_fn(rows, cols, |row, col| {
        data[row + col * leading_dim]
    })))
}

/// Owned position-only path-evaluation result.
///
/// `q` is a flat column-major matrix with shape `(path.dim(), s.len())`.
pub struct PathEvalQResult {
    q: Vec<f64>,
}

/// Owned second-order path-evaluation result.
///
/// All arrays are flat column-major matrices with identical shape
/// `(path.dim(), s.len())`.
pub struct PathEval2Result {
    q: Vec<f64>,
    dq: Vec<f64>,
    ddq: Vec<f64>,
}

/// Owned third-order path-evaluation result.
///
/// All arrays are flat column-major matrices with identical shape
/// `(path.dim(), s.len())`.
pub struct PathEval3Result {
    q: Vec<f64>,
    dq: Vec<f64>,
    ddq: Vec<f64>,
    dddq: Vec<f64>,
}

/// Time-interpolation result used by TOPP2 and TOPP3 interpolation.
///
/// This object deliberately carries `ok/message` instead of returning
/// `Result<Box<_>, String>` for `s_to_t_*`: the Rust interpolation functions
/// can report a meaningful failed profile while still letting C++ inspect a
/// uniform result shape and convert the message to `copp::Error`.
pub struct TimeProfile2Result {
    ok: bool,
    message: String,
    t_final: f64,
    t_s: Vec<f64>,
}

/// Owned third-order profile copied from Rust solver output.
///
/// `a` and `b` are node-based arrays of equal length. `num_stationary` stores
/// `(start, end)` interval counts so C++ interpolation can reconstruct the same
/// stationary-boundary semantics as Rust/Python.
pub struct Profile3rdResult {
    a: Vec<f64>,
    b: Vec<f64>,
    num_stationary: (usize, usize),
}

/// Shared expert result for TOPP3-LP, TOPP3-SOCP, and COPP3-SOCP.
///
/// `profile` is optional because Clarabel can return diagnostics for statuses
/// that are not accepted by the provided C++ options. Objective diagnostics are
/// present for COPP3 and absent for plain TOPP3.
pub struct Topp3SolverResult {
    profile: Option<Profile3rdResult>,
    x: Vec<f64>,
    z: Vec<f64>,
    s: Vec<f64>,
    solver_status: u8,
    obj_val: f64,
    obj_val_dual: f64,
    solve_time: f64,
    iterations: u32,
    r_prim: f64,
    r_dual: f64,
    linsolver: ffi::LinearSolverInfoBridge,
    objective_value: Option<f64>,
    objective_terms: Vec<f64>,
}

/// C++-backed robot model used by the C++ facade.
///
/// When `inverse_dynamics` is `None`, this model behaves like Rust's `usize`
/// point-mass model (`tau = ddq`). When a C++ callback is installed, torque
/// constraints and objectives delegate each inverse-dynamics evaluation through
/// the private `cxx` bridge.
pub struct CppRobotModel {
    dim: usize,
    inverse_dynamics: Option<UniquePtr<ffi::CppInverseDynamics>>,
}

// SAFETY: Calls into the C++ inverse-dynamics callback are serialized by the
// C++ bridge object. The public C++ contract also requires captured state to
// remain valid for the callback lifetime.
unsafe impl Send for CppRobotModel {}
unsafe impl Sync for CppRobotModel {}

impl CppRobotModel {
    fn new(dim: usize) -> Self {
        Self {
            dim,
            inverse_dynamics: None,
        }
    }

    fn with_inverse_dynamics(
        inverse_dynamics: UniquePtr<ffi::CppInverseDynamics>,
    ) -> Result<Self, String> {
        if inverse_dynamics.is_null() {
            return Err("Robot inverse_dynamics callback is null".into());
        }
        let dim = inverse_dynamics.dim();
        if dim == 0 {
            return Err("Robot inverse_dynamics dimension must be positive".into());
        }
        Ok(Self {
            dim,
            inverse_dynamics: Some(inverse_dynamics),
        })
    }

    fn set_inverse_dynamics(
        &mut self,
        inverse_dynamics: UniquePtr<ffi::CppInverseDynamics>,
    ) -> Result<(), String> {
        if inverse_dynamics.is_null() {
            return Err("Robot inverse_dynamics callback is null".into());
        }
        let dim = inverse_dynamics.dim();
        if dim != self.dim {
            return Err(format!(
                "Robot inverse_dynamics dimension {dim} does not match robot dim {}",
                self.dim
            ));
        }
        self.inverse_dynamics = Some(inverse_dynamics);
        Ok(())
    }

    fn clear_inverse_dynamics(&mut self) {
        self.inverse_dynamics = None;
    }

    fn has_inverse_dynamics(&self) -> bool {
        self.inverse_dynamics.is_some()
    }
}

impl RobotBasic for CppRobotModel {
    fn dim(&self) -> usize {
        self.dim
    }
}

impl RobotTorque for CppRobotModel {
    fn inverse_dynamics(
        &self,
        q: &[f64],
        dq: &[f64],
        ddq: &[f64],
        tau: &mut [f64],
    ) -> Result<(), RobotDynamicsError> {
        if q.len() != self.dim
            || dq.len() != self.dim
            || ddq.len() != self.dim
            || tau.len() != self.dim
        {
            return Err(RobotDynamicsError::new(format!(
                "inverse_dynamics received inconsistent dimensions: expected {}, got q={}, dq={}, ddq={}, tau={}",
                self.dim,
                q.len(),
                dq.len(),
                ddq.len(),
                tau.len()
            )));
        }

        let Some(inverse_dynamics) = &self.inverse_dynamics else {
            tau.copy_from_slice(ddq);
            return Ok(());
        };

        inverse_dynamics
            .inverse_dynamics(q, dq, ddq, tau)
            .map_err(|error| RobotDynamicsError::new(error.to_string()))
    }
}

/// Rust-side robot handle owned by the C++ facade.
pub struct RobotHandle {
    inner: RustRobot<CppRobotModel>,
}

/// Owned vector result returned across the private C++ bridge.
///
/// `cxx` borrows slices from opaque Rust objects safely. The C++ facade copies
/// this slice immediately into `std::vector<double>`, so the Rust box can be
/// dropped right after conversion.
pub struct VecF64Result {
    values: Vec<f64>,
}

/// Owned reachable-set result returned to the C++ solver facade.
///
/// The field order follows the Rust/Python convention: `a_max` first, `a_min`
/// second. C++ copies both slices into `std::vector<double>` immediately.
pub struct ReachSet2Result {
    a_max: Vec<f64>,
    a_min: Vec<f64>,
}

/// Owned COPP2-SOCP expert result returned across the private C++ bridge.
///
/// `a` is populated only when the Clarabel status is accepted by the supplied
/// options. Raw Clarabel vectors and diagnostics are copied regardless of
/// status when the solve itself succeeds.
pub struct Copp2SocpResult {
    a: Option<Vec<f64>>,
    x: Vec<f64>,
    z: Vec<f64>,
    s: Vec<f64>,
    solver_status: u8,
    obj_val: f64,
    obj_val_dual: f64,
    solve_time: f64,
    iterations: u32,
    r_prim: f64,
    r_dual: f64,
    linsolver: ffi::LinearSolverInfoBridge,
    objective_value: Option<f64>,
    objective_terms: Vec<f64>,
}

/// Build a Rust spline path from a C++ waypoint matrix.
///
/// # Inputs
/// - `data`: backing slice for a column-major matrix view.
/// - `rows`: path dimension.
/// - `cols`: number of waypoint columns.
/// - `leading_dim`: physical stride between adjacent columns.
/// - `order`: spline order forwarded to Rust [`SplineConfig`].
/// - `s_min`, `s_max`: path-parameter range.
/// - `out_of_range_mode`: private bridge enum (`0 = Error`, `1 = Clamp`).
///
/// # Output
/// Returns an opaque [`PathHandle`] that owns the Rust path. The C++ facade
/// wraps it in RAII and exposes `copp::Path`.
fn path_from_waypoints(
    data: &[f64],
    rows: usize,
    cols: usize,
    leading_dim: usize,
    order: usize,
    s_min: f64,
    s_max: f64,
    out_of_range_mode: u8,
    start_state_data: &[f64],
    start_state_rows: usize,
    start_state_cols: usize,
    start_state_leading_dim: usize,
    end_state_data: &[f64],
    end_state_rows: usize,
    end_state_cols: usize,
    end_state_leading_dim: usize,
) -> Result<Box<PathHandle>, String> {
    if rows == 0 {
        return Err("path_from_waypoints: `rows` must be positive".into());
    }
    if cols < 2 {
        return Err("path_from_waypoints: `cols` must be at least 2".into());
    }
    if leading_dim < rows {
        return Err("path_from_waypoints: `leading_dim` must be at least `rows`".into());
    }
    let required_len = leading_dim
        .checked_mul(cols - 1)
        .and_then(|prefix| prefix.checked_add(rows))
        .ok_or_else(|| "path_from_waypoints: matrix view size overflow".to_string())?;
    if data.len() < required_len {
        return Err("path_from_waypoints: matrix view data is shorter than shape".into());
    }

    let out_of_range_mode = match out_of_range_mode {
        0 => OutOfRangeMode::Error,
        1 => OutOfRangeMode::Clamp,
        other => {
            return Err(format!(
                "path_from_waypoints: unsupported out_of_range_mode {other}"
            ));
        }
    };

    let waypoints = DMatrixView::from_slice_with_strides_generic(
        data,
        Dyn(rows),
        Dyn(cols),
        Const::<1>,
        Dyn(leading_dim),
    );
    let start_state = optional_column_major_matrix(
        start_state_data,
        start_state_rows,
        start_state_cols,
        start_state_leading_dim,
        "start_state",
    )?;
    let end_state = optional_column_major_matrix(
        end_state_data,
        end_state_rows,
        end_state_cols,
        end_state_leading_dim,
        "end_state",
    )?;
    let config = SplineConfig {
        order,
        s_min,
        s_max,
        out_of_range_mode,
        start_state,
        end_state,
        ..SplineConfig::default()
    };
    let inner = Path::from_waypoints_view(waypoints, config).map_err(|error| error.to_string())?;

    Ok(Box::new(PathHandle { inner }))
}

/// Build a Rust path from a C++ batch evaluator up to second order.
///
/// The callback object is moved into Rust and becomes owned by the resulting
/// path. `s_min < s_max` validation is delegated to the Rust path constructor.
fn path_from_evaluator_2nd(
    evaluator: UniquePtr<ffi::CppPathEvaluator2nd>,
    s_min: f64,
    s_max: f64,
) -> Result<Box<PathHandle>, String> {
    if evaluator.is_null() {
        return Err("path_from_evaluator_2nd: evaluator is null".into());
    }
    let dim = evaluator.dim();
    let evaluator = CppEvaluator2nd {
        inner: evaluator,
        dim,
    };
    let inner =
        Path::from_evaluator_2nd(evaluator, s_min, s_max).map_err(|error| error.to_string())?;

    Ok(Box::new(PathHandle { inner }))
}

/// Build a Rust path from a C++ batch evaluator up to third order.
///
/// A third-order evaluator also satisfies second-order path requests.
fn path_from_evaluator_3rd(
    evaluator: UniquePtr<ffi::CppPathEvaluator3rd>,
    s_min: f64,
    s_max: f64,
) -> Result<Box<PathHandle>, String> {
    if evaluator.is_null() {
        return Err("path_from_evaluator_3rd: evaluator is null".into());
    }
    let dim = evaluator.dim();
    let evaluator = CppEvaluator3rd {
        inner: evaluator,
        dim,
    };
    let inner =
        Path::from_evaluator_3rd(evaluator, s_min, s_max).map_err(|error| error.to_string())?;

    Ok(Box::new(PathHandle { inner }))
}

/// Construct the Rust point-mass robot used by the C++ `Robot` and `Constraints`.
///
/// `capacity == 0` means "use the Rust default", matching the public C++
/// constructor default. Positive values become explicit circular-buffer column
/// capacity.
/// Construct a Rust robot handle with point-mass inverse dynamics.
///
/// `capacity` preallocates station/constraint storage but does not change the
/// logical station count. The returned handle backs both C++ `Robot` and
/// independent `Constraints` depending on which facade object owns it.
fn robot_new(dim: usize, capacity: usize) -> Result<Box<RobotHandle>, String> {
    if dim == 0 {
        return Err("Robot: `dim` must be positive".into());
    }
    let model = CppRobotModel::new(dim);
    let inner = if capacity == 0 {
        RustRobot::new(model)
    } else {
        RustRobot::with_capacity(model, capacity)
    };
    Ok(Box::new(RobotHandle { inner }))
}

/// Construct a Rust robot handle with a C++ inverse-dynamics callback.
/// Construct a Rust robot handle backed by a C++ inverse-dynamics callback.
///
/// The callback dimension is read before construction and must be positive.
/// Torque constraints and torque objectives will cross the bridge for each
/// inverse-dynamics evaluation.
fn robot_new_with_inverse_dynamics(
    inverse_dynamics: UniquePtr<ffi::CppInverseDynamics>,
    capacity: usize,
) -> Result<Box<RobotHandle>, String> {
    let model = CppRobotModel::with_inverse_dynamics(inverse_dynamics)?;
    let inner = if capacity == 0 {
        RustRobot::new(model)
    } else {
        RustRobot::with_capacity(model, capacity)
    };
    Ok(Box::new(RobotHandle { inner }))
}

/// Compute second-order segment profile `b` from node profile `a`.
fn a_to_b_topp2(s: &[f64], a: &[f64]) -> Result<Box<VecF64Result>, String> {
    let values = crate::solver::topp2_ra::a_to_b_topp2(s, a).map_err(|error| error.to_string())?;
    Ok(Box::new(VecF64Result { values }))
}

/// Sample `s(t)` on a generated uniform time grid for a TOPP2 profile.
fn t_to_s_topp2_uniform(
    s: &[f64],
    a: &[f64],
    t_s: &[f64],
    t0: f64,
    dt: f64,
    include_final: bool,
) -> Result<Box<VecF64Result>, String> {
    let values = crate::solver::topp2_ra::t_to_s_topp2(
        s,
        a,
        t_s,
        InterpolationMode::UniformTimeGrid(t0, dt, include_final),
    )
    .map_err(|error| error.to_string())?;
    Ok(Box::new(VecF64Result { values }))
}

/// Sample `s(t)` at explicit query times for a TOPP2 profile.
fn t_to_s_topp2_samples(
    s: &[f64],
    a: &[f64],
    t_s: &[f64],
    t_sample: &[f64],
) -> Result<Box<VecF64Result>, String> {
    let values = crate::solver::topp2_ra::t_to_s_topp2(
        s,
        a,
        t_s,
        InterpolationMode::NonUniformTimeGrid(t_sample),
    )
    .map_err(|error| error.to_string())?;
    Ok(Box::new(VecF64Result { values }))
}

/// Compute cumulative time profile `t(s)` from a TOPP3/COPP3 profile.
fn s_to_t_topp3(
    s: &[f64],
    a: &[f64],
    b: &[f64],
    num_stationary_start: usize,
    num_stationary_end: usize,
    t0: f64,
) -> Box<TimeProfile2Result> {
    match crate::copp::copp3::stable::basic::s_to_t_topp3(
        s,
        (a, b, (num_stationary_start, num_stationary_end)),
        t0,
    ) {
        Ok((t_final, t_s)) => Box::new(TimeProfile2Result {
            ok: true,
            message: String::new(),
            t_final,
            t_s,
        }),
        Err(error) => Box::new(TimeProfile2Result {
            ok: false,
            message: error.to_string(),
            t_final: f64::NAN,
            t_s: Vec::new(),
        }),
    }
}

/// Sample `s(t)` on a generated uniform time grid for a TOPP3/COPP3 profile.
fn t_to_s_topp3_uniform(
    s: &[f64],
    a: &[f64],
    b: &[f64],
    num_stationary_start: usize,
    num_stationary_end: usize,
    t_s: &[f64],
    t0: f64,
    dt: f64,
    include_final: bool,
) -> Result<Box<VecF64Result>, String> {
    let values = crate::copp::copp3::stable::basic::t_to_s_topp3(
        s,
        (a, b, (num_stationary_start, num_stationary_end)),
        t_s,
        InterpolationMode::UniformTimeGrid(t0, dt, include_final),
    )
    .map_err(|error| error.to_string())?;
    Ok(Box::new(VecF64Result { values }))
}

/// Sample `s(t)` at explicit query times for a TOPP3/COPP3 profile.
fn t_to_s_topp3_samples(
    s: &[f64],
    a: &[f64],
    b: &[f64],
    num_stationary_start: usize,
    num_stationary_end: usize,
    t_s: &[f64],
    t_sample: &[f64],
) -> Result<Box<VecF64Result>, String> {
    let values = crate::copp::copp3::stable::basic::t_to_s_topp3(
        s,
        (a, b, (num_stationary_start, num_stationary_end)),
        t_s,
        InterpolationMode::NonUniformTimeGrid(t_sample),
    )
    .map_err(|error| error.to_string())?;
    Ok(Box::new(VecF64Result { values }))
}

/// Convert an internal C++ matrix descriptor plus data slice into an owned
/// column-major nalgebra matrix.
///
/// The public C++ API accepts both column-major and row-major views, with
/// optional leading dimension. The Rust algorithms operate on nalgebra views, so
/// the bridge normalizes user memory into owned column-major `DMatrix` values at
/// the boundary. This keeps the generated `cxx` signature simple and prevents
/// Rust from borrowing strided C++ memory beyond the bridge call.
/// Reconstruct an owned nalgebra matrix from C++ matrix-view metadata.
///
/// # Inputs
/// `data` is the raw backing span described by `desc`. Column-major inputs can
/// be copied with stride-aware column slices; row-major inputs are transposed
/// element-by-element into nalgebra's column-major storage.
///
/// # Errors
/// Returns a string error when the descriptor is empty where a matrix is
/// required, has an invalid leading dimension, uses an unknown layout id, or
/// points past the provided slice.
fn matrix_from_bridge(
    name: &str,
    data: &[f64],
    desc: ffi::MatrixDescriptor,
) -> Result<DMatrix<f64>, String> {
    if desc.rows == 0 || desc.cols == 0 {
        return Ok(DMatrix::zeros(desc.rows, desc.cols));
    }

    let minimum_leading_dim = match desc.layout {
        0 => desc.rows,
        1 => desc.cols,
        other => {
            return Err(format!(
                "{name}: unsupported matrix layout id {other}; expected 0 or 1"
            ));
        }
    };
    if desc.leading_dim < minimum_leading_dim {
        return Err(format!(
            "{name}: leading_dim {} is smaller than required minimum {}",
            desc.leading_dim, minimum_leading_dim
        ));
    }

    let required_len = match desc.layout {
        0 => desc
            .leading_dim
            .checked_mul(desc.cols - 1)
            .and_then(|prefix| prefix.checked_add(desc.rows)),
        1 => desc
            .leading_dim
            .checked_mul(desc.rows - 1)
            .and_then(|prefix| prefix.checked_add(desc.cols)),
        _ => unreachable!("layout checked above"),
    }
    .ok_or_else(|| format!("{name}: matrix view size overflow"))?;
    if data.len() < required_len {
        return Err(format!(
            "{name}: matrix data length {} is shorter than required {}",
            data.len(),
            required_len
        ));
    }

    Ok(DMatrix::from_fn(desc.rows, desc.cols, |row, col| {
        if desc.layout == 0 {
            data[row + col * desc.leading_dim]
        } else {
            data[col + row * desc.leading_dim]
        }
    }))
}

/// Return the number of station columns requested by a broadcast limit.
///
/// A zero `length` is the C++ sentinel for "infer from the current stored
/// station window". The Rust robot APIs require an explicit `ncols`, so the
/// bridge translates the sentinel here.
fn inferred_limit_length(
    robot: &RustRobot<CppRobotModel>,
    start_idx_s: usize,
    length: usize,
) -> usize {
    if length != 0 {
        length
    } else {
        robot.constraints.idx_s_end().saturating_sub(start_idx_s)
    }
}

/// Convert the private bridge verbosity id into Rust diagnostics verbosity.
/// Convert the private bridge verbosity enum into Rust diagnostics verbosity.
fn verbosity_from_bridge(verbosity: u8) -> Result<Verbosity, String> {
    match verbosity {
        0 => Ok(Verbosity::Silent),
        1 => Ok(Verbosity::Summary),
        2 => Ok(Verbosity::Debug),
        3 => Ok(Verbosity::Trace),
        other => Err(format!("unsupported verbosity id {other}")),
    }
}

/// Build validated Rust reach-set/TOPP2-RA options from C++ plain fields.
/// Convert TOPP2 option scalars from C++ into the Rust builder type.
fn reach_set2_options_from_bridge(
    lp_feas_tol: f64,
    a_cmp_abs_tol: f64,
    a_cmp_rel_tol: f64,
    verbosity: u8,
) -> Result<crate::solver::topp2_ra::ReachSet2Options, String> {
    crate::solver::topp2_ra::ReachSet2OptionsBuilder::new()
        .lp_feas_tol(lp_feas_tol)
        .a_cmp_abs_tol(a_cmp_abs_tol)
        .a_cmp_rel_tol(a_cmp_rel_tol)
        .verbosity(verbosity_from_bridge(verbosity)?)
        .build()
        .map_err(|error| error.to_string())
}

fn direct_solve_method_to_bridge(settings: &DefaultSettings<f64>) -> u8 {
    match settings.direct_solve_method.as_str() {
        "qdldl" => 1,
        "faer" => 2,
        "mkl" => 3,
        "panua" => 4,
        _ => 0,
    }
}

fn direct_solve_method_from_bridge(method: u8) -> Result<String, String> {
    match method {
        0 => Ok("auto".to_owned()),
        1 => Ok("qdldl".to_owned()),
        2 => Ok("faer".to_owned()),
        3 => Ok("mkl".to_owned()),
        4 => Ok("panua".to_owned()),
        other => Err(format!(
            "unsupported Clarabel direct solve method id {other}"
        )),
    }
}

fn solver_status_to_bridge(status: SolverStatus) -> u8 {
    match status {
        SolverStatus::Unsolved => 0,
        SolverStatus::Solved => 1,
        SolverStatus::PrimalInfeasible => 2,
        SolverStatus::DualInfeasible => 3,
        SolverStatus::AlmostSolved => 4,
        SolverStatus::AlmostPrimalInfeasible => 5,
        SolverStatus::AlmostDualInfeasible => 6,
        SolverStatus::MaxIterations => 7,
        SolverStatus::MaxTime => 8,
        SolverStatus::NumericalError => 9,
        SolverStatus::InsufficientProgress => 10,
        SolverStatus::CallbackTerminated => 11,
    }
}

fn linear_solver_info_to_bridge(info: LinearSolverInfo) -> ffi::LinearSolverInfoBridge {
    let method = match info.name.as_str() {
        "qdldl" => 1,
        "faer" => 2,
        "mkl" => 3,
        "panua" => 4,
        _ => 0,
    };
    ffi::LinearSolverInfoBridge {
        method,
        threads: info.threads,
        direct: info.direct,
        nnz_a: info.nnzA,
        nnz_l: info.nnzL,
    }
}

fn clarabel_settings_to_bridge(settings: &DefaultSettings<f64>) -> ffi::ClarabelSettingsBridge {
    ffi::ClarabelSettingsBridge {
        max_iter: settings.max_iter,
        time_limit: settings.time_limit,
        verbose: settings.verbose,
        max_step_fraction: settings.max_step_fraction,
        tol_gap_abs: settings.tol_gap_abs,
        tol_gap_rel: settings.tol_gap_rel,
        tol_feas: settings.tol_feas,
        tol_infeas_abs: settings.tol_infeas_abs,
        tol_infeas_rel: settings.tol_infeas_rel,
        tol_ktratio: settings.tol_ktratio,
        reduced_tol_gap_abs: settings.reduced_tol_gap_abs,
        reduced_tol_gap_rel: settings.reduced_tol_gap_rel,
        reduced_tol_feas: settings.reduced_tol_feas,
        reduced_tol_infeas_abs: settings.reduced_tol_infeas_abs,
        reduced_tol_infeas_rel: settings.reduced_tol_infeas_rel,
        reduced_tol_ktratio: settings.reduced_tol_ktratio,
        equilibrate_enable: settings.equilibrate_enable,
        equilibrate_max_iter: settings.equilibrate_max_iter,
        equilibrate_min_scaling: settings.equilibrate_min_scaling,
        equilibrate_max_scaling: settings.equilibrate_max_scaling,
        linesearch_backtrack_step: settings.linesearch_backtrack_step,
        min_switch_step_length: settings.min_switch_step_length,
        min_terminate_step_length: settings.min_terminate_step_length,
        max_threads: settings.max_threads,
        direct_kkt_solver: settings.direct_kkt_solver,
        direct_solve_method: direct_solve_method_to_bridge(settings),
        static_regularization_enable: settings.static_regularization_enable,
        static_regularization_constant: settings.static_regularization_constant,
        static_regularization_proportional: settings.static_regularization_proportional,
        dynamic_regularization_enable: settings.dynamic_regularization_enable,
        dynamic_regularization_eps: settings.dynamic_regularization_eps,
        dynamic_regularization_delta: settings.dynamic_regularization_delta,
        iterative_refinement_enable: settings.iterative_refinement_enable,
        iterative_refinement_reltol: settings.iterative_refinement_reltol,
        iterative_refinement_abstol: settings.iterative_refinement_abstol,
        iterative_refinement_max_iter: settings.iterative_refinement_max_iter,
        iterative_refinement_stop_ratio: settings.iterative_refinement_stop_ratio,
        presolve_enable: settings.presolve_enable,
        input_sparse_dropzeros: settings.input_sparse_dropzeros,
    }
}

fn clarabel_settings_from_bridge(
    settings: ffi::ClarabelSettingsBridge,
) -> Result<DefaultSettings<f64>, String> {
    Ok(DefaultSettings::<f64> {
        max_iter: settings.max_iter,
        time_limit: settings.time_limit,
        verbose: settings.verbose,
        max_step_fraction: settings.max_step_fraction,
        tol_gap_abs: settings.tol_gap_abs,
        tol_gap_rel: settings.tol_gap_rel,
        tol_feas: settings.tol_feas,
        tol_infeas_abs: settings.tol_infeas_abs,
        tol_infeas_rel: settings.tol_infeas_rel,
        tol_ktratio: settings.tol_ktratio,
        reduced_tol_gap_abs: settings.reduced_tol_gap_abs,
        reduced_tol_gap_rel: settings.reduced_tol_gap_rel,
        reduced_tol_feas: settings.reduced_tol_feas,
        reduced_tol_infeas_abs: settings.reduced_tol_infeas_abs,
        reduced_tol_infeas_rel: settings.reduced_tol_infeas_rel,
        reduced_tol_ktratio: settings.reduced_tol_ktratio,
        equilibrate_enable: settings.equilibrate_enable,
        equilibrate_max_iter: settings.equilibrate_max_iter,
        equilibrate_min_scaling: settings.equilibrate_min_scaling,
        equilibrate_max_scaling: settings.equilibrate_max_scaling,
        linesearch_backtrack_step: settings.linesearch_backtrack_step,
        min_switch_step_length: settings.min_switch_step_length,
        min_terminate_step_length: settings.min_terminate_step_length,
        max_threads: settings.max_threads,
        direct_kkt_solver: settings.direct_kkt_solver,
        direct_solve_method: direct_solve_method_from_bridge(settings.direct_solve_method)?,
        static_regularization_enable: settings.static_regularization_enable,
        static_regularization_constant: settings.static_regularization_constant,
        static_regularization_proportional: settings.static_regularization_proportional,
        dynamic_regularization_enable: settings.dynamic_regularization_enable,
        dynamic_regularization_eps: settings.dynamic_regularization_eps,
        dynamic_regularization_delta: settings.dynamic_regularization_delta,
        iterative_refinement_enable: settings.iterative_refinement_enable,
        iterative_refinement_reltol: settings.iterative_refinement_reltol,
        iterative_refinement_abstol: settings.iterative_refinement_abstol,
        iterative_refinement_max_iter: settings.iterative_refinement_max_iter,
        iterative_refinement_stop_ratio: settings.iterative_refinement_stop_ratio,
        presolve_enable: settings.presolve_enable,
        input_sparse_dropzeros: settings.input_sparse_dropzeros,
    })
}

fn clarabel_options_to_bridge(
    options: &crate::copp::ClarabelOptions,
) -> ffi::ClarabelOptionsBridge {
    ffi::ClarabelOptionsBridge {
        verbosity: options.verbosity() as u8,
        allow_almost_solved: options.is_allow(SolverStatus::AlmostSolved),
        allow_max_iterations: options.is_allow(SolverStatus::MaxIterations),
        allow_max_time: options.is_allow(SolverStatus::MaxTime),
        allow_callback_terminated: options.is_allow(SolverStatus::CallbackTerminated),
        allow_insufficient_progress: options.is_allow(SolverStatus::InsufficientProgress),
        clarabel_settings: clarabel_settings_to_bridge(options.clarabel_settings()),
    }
}

fn clarabel_options_from_bridge(
    options: ffi::ClarabelOptionsBridge,
) -> Result<crate::copp::ClarabelOptions, String> {
    let ffi::ClarabelOptionsBridge {
        verbosity,
        allow_almost_solved,
        allow_max_iterations,
        allow_max_time,
        allow_callback_terminated,
        allow_insufficient_progress,
        clarabel_settings,
    } = options;

    ClarabelOptionsBuilder::with_clarabel_setting(clarabel_settings_from_bridge(clarabel_settings)?)
        .verbosity(verbosity_from_bridge(verbosity)?)
        .allow_almost_solved(allow_almost_solved)
        .allow_max_iterations(allow_max_iterations)
        .allow_max_time(allow_max_time)
        .allow_callback_terminated(allow_callback_terminated)
        .allow_insufficient_progress(allow_insufficient_progress)
        .build()
        .map_err(|error| error.to_string())
}

/// Borrow one objective payload slice from the flattened C++ objective buffer.
///
/// Objectives with vector parameters store all vectors in one contiguous C++
/// array. Each [`ObjectiveDescriptor`] carries offsets/lengths into that array.
/// This helper centralizes bounds checking so solver constructors report clear
/// bridge errors instead of panicking.
fn objective_data_slice<'a>(
    data: &'a [f64],
    offset: usize,
    len: usize,
    name: &str,
) -> Result<&'a [f64], String> {
    let end = offset
        .checked_add(len)
        .ok_or_else(|| format!("{name}: objective data range overflow"))?;
    data.get(offset..end).ok_or_else(|| {
        format!(
            "{name}: objective data range [{offset}, {end}) exceeds data length {}",
            data.len()
        )
    })
}

/// Reconstruct Rust [`CoppObjective`] values from compact C++ descriptors.
///
/// Validation that depends on robot dimension or station count is left to Rust
/// solver problem constructors, exactly as in the native Rust API.
fn objectives_from_bridge<'a>(
    objectives: &'a [ffi::ObjectiveDescriptor],
    data: &'a [f64],
) -> Result<Vec<CoppObjective<'a>>, String> {
    objectives
        .iter()
        .enumerate()
        .map(|(i, objective)| match objective.kind {
            0 => Ok(CoppObjective::Time(objective.weight)),
            1 => {
                let alpha = objective_data_slice(
                    data,
                    objective.first_offset,
                    objective.first_len,
                    "Linear.alpha",
                )?;
                let beta = objective_data_slice(
                    data,
                    objective.second_offset,
                    objective.second_len,
                    "Linear.beta",
                )?;
                Ok(CoppObjective::Linear(objective.weight, alpha, beta))
            }
            2 => {
                let normalize = objective_data_slice(
                    data,
                    objective.first_offset,
                    objective.first_len,
                    "ThermalEnergy.normalize",
                )?;
                Ok(CoppObjective::ThermalEnergy(objective.weight, normalize))
            }
            3 => {
                let normalize = objective_data_slice(
                    data,
                    objective.first_offset,
                    objective.first_len,
                    "TotalVariationTorque.normalize",
                )?;
                Ok(CoppObjective::TotalVariationTorque(
                    objective.weight,
                    normalize,
                ))
            }
            other => Err(format!(
                "objectives[{i}]: unsupported objective kind {other}"
            )),
        })
        .collect()
}

fn copp2_result_from_solution(
    a: Option<Vec<f64>>,
    solution: DefaultSolution<f64>,
    linsolver: LinearSolverInfo,
    objective_breakdown: Option<(f64, Vec<f64>)>,
) -> Copp2SocpResult {
    let DefaultSolution {
        x,
        z,
        s,
        status,
        obj_val,
        obj_val_dual,
        solve_time,
        iterations,
        r_prim,
        r_dual,
    } = solution;
    let (objective_value, objective_terms) = match objective_breakdown {
        Some((value, terms)) => (Some(value), terms),
        None => (None, Vec::new()),
    };

    Copp2SocpResult {
        a,
        x,
        z,
        s,
        solver_status: solver_status_to_bridge(status),
        obj_val,
        obj_val_dual,
        solve_time,
        iterations,
        r_prim,
        r_dual,
        linsolver: linear_solver_info_to_bridge(linsolver),
        objective_value,
        objective_terms,
    }
}

fn topp3_problem_info_from_parts(
    idx_s_start: usize,
    s_len: usize,
    num_stationary: (usize, usize),
) -> Result<ffi::Topp3ProblemInfoBridge, String> {
    let idx_s_final = idx_s_start
        .checked_add(s_len.saturating_sub(1))
        .ok_or_else(|| "TOPP3 problem: idx_s_final overflow".to_owned())?;
    Ok(ffi::Topp3ProblemInfoBridge {
        s_len,
        idx_s_final,
        num_stationary_start: num_stationary.0,
        num_stationary_end: num_stationary.1,
    })
}

fn topp3_result_from_solution(
    profile: Option<crate::copp::copp3::Topp3Profile>,
    solution: DefaultSolution<f64>,
    linsolver: LinearSolverInfo,
    objective_breakdown: Option<(f64, Vec<f64>)>,
) -> Topp3SolverResult {
    let DefaultSolution {
        x,
        z,
        s,
        status,
        obj_val,
        obj_val_dual,
        solve_time,
        iterations,
        r_prim,
        r_dual,
    } = solution;
    let (objective_value, objective_terms) = match objective_breakdown {
        Some((value, terms)) => (Some(value), terms),
        None => (None, Vec::new()),
    };

    Topp3SolverResult {
        profile: profile.map(Profile3rdResult::from),
        x,
        z,
        s,
        solver_status: solver_status_to_bridge(status),
        obj_val,
        obj_val_dual,
        solve_time,
        iterations,
        r_prim,
        r_dual,
        linsolver: linear_solver_info_to_bridge(linsolver),
        objective_value,
        objective_terms,
    }
}

fn version() -> &'static str {
    env!("CARGO_PKG_VERSION")
}

fn clarabel_default_settings() -> ffi::ClarabelSettingsBridge {
    let options = ClarabelOptionsBuilder::new()
        .build()
        .expect("default Clarabel options should be valid");
    clarabel_settings_to_bridge(options.clarabel_settings())
}

fn clarabel_default_options() -> ffi::ClarabelOptionsBridge {
    let options = ClarabelOptionsBuilder::new()
        .build()
        .expect("default Clarabel options should be valid");
    clarabel_options_to_bridge(&options)
}

/// Dispatch one physical limit family to the corresponding Rust robot method.
///
/// `kind` is a private bridge enum:
/// - `0`: velocity
/// - `1`: acceleration
/// - `2`: jerk
/// - `3`: torque, using point-mass dynamics or a C++ inverse-dynamics callback
fn add_limits_from_matrices(
    robot: &mut RustRobot<CppRobotModel>,
    kind: u8,
    upper: &DMatrix<f64>,
    lower: &DMatrix<f64>,
    start_idx_s: usize,
) -> Result<(), String> {
    if upper.ncols() == 0 {
        return Ok(());
    }
    match kind {
        0 => robot
            .with_axial_velocity(&upper.as_view(), &lower.as_view(), start_idx_s)
            .map(|_| ())
            .map_err(|error| error.to_string()),
        1 => robot
            .with_axial_acceleration(&upper.as_view(), &lower.as_view(), start_idx_s)
            .map(|_| ())
            .map_err(|error| error.to_string()),
        2 => robot
            .with_axial_jerk(&upper.as_view(), &lower.as_view(), start_idx_s)
            .map(|_| ())
            .map_err(|error| error.to_string()),
        3 => robot
            .with_axial_torque(&upper.as_view(), &lower.as_view(), start_idx_s)
            .map(|_| ())
            .map_err(|error| error.to_string()),
        other => Err(format!("unsupported axial limit kind {other}")),
    }
}

fn s_to_t_topp2(s: &[f64], a: &[f64], t0: f64) -> Box<TimeProfile2Result> {
    match crate::solver::topp2_ra::s_to_t_topp2(s, a, t0) {
        Ok((t_final, t_s)) => Box::new(TimeProfile2Result {
            ok: true,
            message: String::new(),
            t_final,
            t_s,
        }),
        Err(error) => Box::new(TimeProfile2Result {
            ok: false,
            message: error.to_string(),
            t_final: f64::NAN,
            t_s: Vec::new(),
        }),
    }
}

impl PathHandle {
    fn dim(&self) -> usize {
        self.inner.dim()
    }

    fn s_min(&self) -> f64 {
        self.inner.s_range().0
    }

    fn s_max(&self) -> f64 {
        self.inner.s_range().1
    }

    fn evaluate_q(&self, s: &[f64]) -> Result<Box<PathEvalQResult>, String> {
        let out = self
            .inner
            .evaluate_q(s)
            .map_err(|error| error.to_string())?;
        Ok(Box::new(PathEvalQResult {
            q: out.q.as_slice().to_vec(),
        }))
    }

    fn evaluate_up_to_2nd(&self, s: &[f64]) -> Result<Box<PathEval2Result>, String> {
        let out = self
            .inner
            .evaluate_up_to_2nd(s)
            .map_err(|error| error.to_string())?;
        let dq = out
            .dq
            .ok_or_else(|| "evaluate_up_to_2nd: Rust path did not return dq".to_string())?;
        let ddq = out
            .ddq
            .ok_or_else(|| "evaluate_up_to_2nd: Rust path did not return ddq".to_string())?;
        Ok(Box::new(PathEval2Result {
            q: out.q.as_slice().to_vec(),
            dq: dq.as_slice().to_vec(),
            ddq: ddq.as_slice().to_vec(),
        }))
    }

    fn evaluate_up_to_3rd(&self, s: &[f64]) -> Result<Box<PathEval3Result>, String> {
        let out = self
            .inner
            .evaluate_up_to_3rd(s)
            .map_err(|error| error.to_string())?;
        let dq = out
            .dq
            .ok_or_else(|| "evaluate_up_to_3rd: Rust path did not return dq".to_string())?;
        let ddq = out
            .ddq
            .ok_or_else(|| "evaluate_up_to_3rd: Rust path did not return ddq".to_string())?;
        let dddq = out
            .dddq
            .ok_or_else(|| "evaluate_up_to_3rd: Rust path did not return dddq".to_string())?;
        Ok(Box::new(PathEval3Result {
            q: out.q.as_slice().to_vec(),
            dq: dq.as_slice().to_vec(),
            ddq: ddq.as_slice().to_vec(),
            dddq: dddq.as_slice().to_vec(),
        }))
    }
}

impl RobotHandle {
    /// Return robot/path dimension.
    fn dim(&self) -> usize {
        self.inner.dim()
    }

    /// Return the number of stored station samples.
    fn len(&self) -> usize {
        self.inner.constraints.len()
    }

    /// Return allocated station-buffer capacity.
    fn capacity(&self) -> usize {
        self.inner.constraints.capacity()
    }

    /// Return whether the constraint buffer is empty.
    fn is_empty(&self) -> bool {
        self.inner.constraints.is_empty()
    }

    /// Return the first global station id in the active logical window.
    fn idx_s_start(&self) -> usize {
        self.inner.constraints.idx_s_start()
    }

    /// Return the exclusive end global station id in the active logical window.
    fn idx_s_end(&self) -> usize {
        self.inner.constraints.idx_s_end()
    }

    /// Export station samples over `[idx_s_from, idx_s_to)`.
    fn s_values(&self, idx_s_from: usize, idx_s_to: usize) -> Result<Box<VecF64Result>, String> {
        let values = self
            .inner
            .constraints
            .s_vec(idx_s_from, idx_s_to)
            .map_err(|error| error.to_string())?;
        Ok(Box::new(VecF64Result { values }))
    }

    /// Export first-order upper bounds over `[idx_s_from, idx_s_to)`.
    fn amax_values(&self, idx_s_from: usize, idx_s_to: usize) -> Result<Box<VecF64Result>, String> {
        let values = self
            .inner
            .constraints
            .amax_vec(idx_s_from, idx_s_to)
            .map_err(|error| error.to_string())?;
        Ok(Box::new(VecF64Result { values }))
    }

    /// Read one station value by global station id.
    fn s_value(&self, idx_s: usize) -> Result<f64, String> {
        self.inner
            .constraints
            .get_s(idx_s)
            .map_err(|error| error.to_string())
    }

    /// Read one first-order upper bound by global station id.
    fn amax_value(&self, idx_s: usize) -> Result<f64, String> {
        self.inner
            .constraints
            .get_amax(idx_s)
            .map_err(|error| error.to_string())
    }

    /// Return whether a C++ inverse-dynamics callback is installed.
    fn has_inverse_dynamics(&self) -> bool {
        self.inner.model().has_inverse_dynamics()
    }

    /// Replace the C++ inverse-dynamics callback.
    fn set_inverse_dynamics(
        &mut self,
        inverse_dynamics: UniquePtr<ffi::CppInverseDynamics>,
    ) -> Result<(), String> {
        self.inner
            .model_mut()
            .set_inverse_dynamics(inverse_dynamics)
    }

    /// Restore point-mass torque evaluation.
    fn clear_inverse_dynamics(&mut self) {
        self.inner.model_mut().clear_inverse_dynamics();
    }

    /// Append strictly increasing station samples to the robot.
    fn append_s(&mut self, s: &[f64]) -> Result<(), String> {
        self.inner
            .with_s(s)
            .map(|_| ())
            .map_err(|error| error.to_string())
    }

    /// Build a validated Rust TOPP2 problem over this handle's constraint buffer.
    fn with_topp2_problem<R>(
        &self,
        idx_s_start: usize,
        idx_s_final: usize,
        a_start: f64,
        a_final: f64,
        f: impl FnOnce(&crate::solver::topp2_ra::Topp2Problem<'_>) -> Result<R, crate::diag::CoppError>,
    ) -> Result<R, String> {
        let problem = crate::solver::topp2_ra::Topp2ProblemBuilder::with_constraint(
            &self.inner.constraints,
            (idx_s_start, idx_s_final),
            (a_start, a_final),
        )
        .build()
        .map_err(|error| error.to_string())?;
        f(&problem).map_err(|error| error.to_string())
    }

    /// Validate a TOPP2 problem descriptor and return its closed interval length.
    fn topp2_problem_s_len(
        &self,
        idx_s_start: usize,
        idx_s_final: usize,
        a_start: f64,
        a_final: f64,
    ) -> Result<usize, String> {
        self.with_topp2_problem(idx_s_start, idx_s_final, a_start, a_final, |problem| {
            Ok(problem.s_len())
        })
    }

    /// Validate a COPP2 problem descriptor and return its closed interval length.
    #[allow(clippy::too_many_arguments)]
    fn copp2_problem_s_len(
        &self,
        idx_s_start: usize,
        idx_s_final: usize,
        a_start: f64,
        a_final: f64,
        objectives: &[ffi::ObjectiveDescriptor],
        objective_data: &[f64],
    ) -> Result<usize, String> {
        let objectives = objectives_from_bridge(objectives, objective_data)?;
        let problem = crate::solver::copp2_socp::Copp2ProblemBuilder::new(
            &self.inner,
            (idx_s_start, idx_s_final),
            (a_start, a_final),
            &objectives,
        )
        .build()
        .map_err(|error| error.to_string())?;
        Ok(problem.s_len())
    }

    /// Compute backward-only TOPP2 reachable intervals.
    #[allow(clippy::too_many_arguments)]
    fn reach_set2_backward(
        &self,
        idx_s_start: usize,
        idx_s_final: usize,
        a_start: f64,
        a_final: f64,
        lp_feas_tol: f64,
        a_cmp_abs_tol: f64,
        a_cmp_rel_tol: f64,
        verbosity: u8,
    ) -> Result<Box<ReachSet2Result>, String> {
        let options =
            reach_set2_options_from_bridge(lp_feas_tol, a_cmp_abs_tol, a_cmp_rel_tol, verbosity)?;
        self.with_topp2_problem(idx_s_start, idx_s_final, a_start, a_final, |problem| {
            crate::solver::reach_set2::reach_set2_backward(problem, &options)
        })
        .map(|reach| {
            Box::new(ReachSet2Result {
                a_max: reach.a_max,
                a_min: reach.a_min,
            })
        })
    }

    /// Compute bidirectional TOPP2 reachable intervals.
    #[allow(clippy::too_many_arguments)]
    fn reach_set2_bidirectional(
        &self,
        idx_s_start: usize,
        idx_s_final: usize,
        a_start: f64,
        a_final: f64,
        lp_feas_tol: f64,
        a_cmp_abs_tol: f64,
        a_cmp_rel_tol: f64,
        verbosity: u8,
    ) -> Result<Box<ReachSet2Result>, String> {
        let options =
            reach_set2_options_from_bridge(lp_feas_tol, a_cmp_abs_tol, a_cmp_rel_tol, verbosity)?;
        self.with_topp2_problem(idx_s_start, idx_s_final, a_start, a_final, |problem| {
            crate::solver::reach_set2::reach_set2_bidirectional(problem, &options)
        })
        .map(|reach| {
            Box::new(ReachSet2Result {
                a_max: reach.a_max,
                a_min: reach.a_min,
            })
        })
    }

    /// Solve TOPP2-RA and return the node profile `a(s)`.
    #[allow(clippy::too_many_arguments)]
    fn topp2_ra_solve(
        &self,
        idx_s_start: usize,
        idx_s_final: usize,
        a_start: f64,
        a_final: f64,
        lp_feas_tol: f64,
        a_cmp_abs_tol: f64,
        a_cmp_rel_tol: f64,
        verbosity: u8,
    ) -> Result<Box<VecF64Result>, String> {
        let options =
            reach_set2_options_from_bridge(lp_feas_tol, a_cmp_abs_tol, a_cmp_rel_tol, verbosity)?;
        let values =
            self.with_topp2_problem(idx_s_start, idx_s_final, a_start, a_final, |problem| {
                crate::solver::topp2_ra::topp2_ra(problem, &options)
            })?;
        Ok(Box::new(VecF64Result { values }))
    }

    /// Solve COPP2-SOCP and return only the accepted `a(s)` profile.
    #[allow(clippy::too_many_arguments)]
    fn copp2_socp_solve(
        &self,
        idx_s_start: usize,
        idx_s_final: usize,
        a_start: f64,
        a_final: f64,
        objectives: &[ffi::ObjectiveDescriptor],
        objective_data: &[f64],
        options: ffi::ClarabelOptionsBridge,
    ) -> Result<Box<VecF64Result>, String> {
        let objectives = objectives_from_bridge(objectives, objective_data)?;
        let options = clarabel_options_from_bridge(options)?;
        let problem = crate::solver::copp2_socp::Copp2ProblemBuilder::new(
            &self.inner,
            (idx_s_start, idx_s_final),
            (a_start, a_final),
            &objectives,
        )
        .build()
        .map_err(|error| error.to_string())?;
        let values = crate::solver::copp2_socp::copp2_socp(&problem, &options)
            .map_err(|error| error.to_string())?;
        Ok(Box::new(VecF64Result { values }))
    }

    /// Solve COPP2-SOCP and return full Clarabel diagnostics.
    #[allow(clippy::too_many_arguments)]
    fn copp2_socp_solve_expert(
        &self,
        idx_s_start: usize,
        idx_s_final: usize,
        a_start: f64,
        a_final: f64,
        objectives: &[ffi::ObjectiveDescriptor],
        objective_data: &[f64],
        options: ffi::ClarabelOptionsBridge,
    ) -> Result<Box<Copp2SocpResult>, String> {
        let objectives = objectives_from_bridge(objectives, objective_data)?;
        let options = clarabel_options_from_bridge(options)?;
        let problem = crate::solver::copp2_socp::Copp2ProblemBuilder::new(
            &self.inner,
            (idx_s_start, idx_s_final),
            (a_start, a_final),
            &objectives,
        )
        .build()
        .map_err(|error| error.to_string())?;
        let expert = crate::solver::copp2_socp::copp2_socp_expert_with_info(&problem, &options)
            .map_err(|error| error.to_string())?;
        let objective_breakdown = expert.result.as_ref().map(|a| {
            objective_value_copp2_opt(&self.inner, problem.idx_s_interval.0, &objectives, a)
        });
        Ok(Box::new(copp2_result_from_solution(
            expert.result,
            expert.solution,
            expert.linsolver,
            objective_breakdown,
        )))
    }

    #[allow(clippy::too_many_arguments)]
    fn with_topp3_problem<R>(
        &mut self,
        idx_s_start: usize,
        a_linearization: &[f64],
        a_start: f64,
        a_final: f64,
        b_start: f64,
        b_final: f64,
        num_stationary_max_start: usize,
        num_stationary_max_end: usize,
        a_linearization_floor: f64,
        f: impl FnOnce(
            &crate::copp::copp3::stable::basic::Topp3Problem<'_>,
        ) -> Result<R, crate::diag::CoppError>,
    ) -> Result<R, String> {
        let problem = crate::solver::topp3_lp::Topp3ProblemBuilder::with_constraint(
            &mut self.inner.constraints,
            idx_s_start,
            a_linearization,
            (a_start, a_final),
            (b_start, b_final),
        )
        .with_num_stationary_max_pair((num_stationary_max_start, num_stationary_max_end))
        .with_a_linearization_floor(a_linearization_floor)
        .build_with_linearization()
        .map_err(|error| error.to_string())?;
        f(&problem).map_err(|error| error.to_string())
    }

    /// Validate and immediately linearize a TOPP3 problem descriptor.
    #[allow(clippy::too_many_arguments)]
    fn topp3_problem_prepare(
        &mut self,
        idx_s_start: usize,
        a_linearization: &[f64],
        a_start: f64,
        a_final: f64,
        b_start: f64,
        b_final: f64,
        num_stationary_max_start: usize,
        num_stationary_max_end: usize,
        a_linearization_floor: f64,
    ) -> Result<ffi::Topp3ProblemInfoBridge, String> {
        self.with_topp3_problem(
            idx_s_start,
            a_linearization,
            a_start,
            a_final,
            b_start,
            b_final,
            num_stationary_max_start,
            num_stationary_max_end,
            a_linearization_floor,
            |problem| {
                topp3_problem_info_from_parts(
                    problem.idx_s_start,
                    problem.a_linearization.len(),
                    problem.num_stationary,
                )
                .map_err(|error| {
                    crate::diag::CoppError::InvalidInput("topp3_problem".into(), error)
                })
            },
        )
    }

    /// Solve TOPP3-LP and return only the accepted profile.
    #[allow(clippy::too_many_arguments)]
    fn topp3_lp_solve(
        &mut self,
        idx_s_start: usize,
        a_linearization: &[f64],
        a_start: f64,
        a_final: f64,
        b_start: f64,
        b_final: f64,
        num_stationary_max_start: usize,
        num_stationary_max_end: usize,
        a_linearization_floor: f64,
        options: ffi::ClarabelOptionsBridge,
    ) -> Result<Box<Profile3rdResult>, String> {
        let options = clarabel_options_from_bridge(options)?;
        self.with_topp3_problem(
            idx_s_start,
            a_linearization,
            a_start,
            a_final,
            b_start,
            b_final,
            num_stationary_max_start,
            num_stationary_max_end,
            a_linearization_floor,
            |problem| crate::solver::topp3_lp::topp3_lp(problem, &options),
        )
        .map(|profile| Box::new(Profile3rdResult::from(profile)))
    }

    /// Solve TOPP3-LP and return full Clarabel diagnostics.
    #[allow(clippy::too_many_arguments)]
    fn topp3_lp_solve_expert(
        &mut self,
        idx_s_start: usize,
        a_linearization: &[f64],
        a_start: f64,
        a_final: f64,
        b_start: f64,
        b_final: f64,
        num_stationary_max_start: usize,
        num_stationary_max_end: usize,
        a_linearization_floor: f64,
        options: ffi::ClarabelOptionsBridge,
    ) -> Result<Box<Topp3SolverResult>, String> {
        let options = clarabel_options_from_bridge(options)?;
        self.with_topp3_problem(
            idx_s_start,
            a_linearization,
            a_start,
            a_final,
            b_start,
            b_final,
            num_stationary_max_start,
            num_stationary_max_end,
            a_linearization_floor,
            |problem| crate::solver::topp3_lp::topp3_lp_expert_with_info(problem, &options),
        )
        .map(|expert| {
            Box::new(topp3_result_from_solution(
                expert.result,
                expert.solution,
                expert.linsolver,
                None,
            ))
        })
    }

    /// Solve TOPP3-SOCP and return only the accepted profile.
    #[allow(clippy::too_many_arguments)]
    fn topp3_socp_solve(
        &mut self,
        idx_s_start: usize,
        a_linearization: &[f64],
        a_start: f64,
        a_final: f64,
        b_start: f64,
        b_final: f64,
        num_stationary_max_start: usize,
        num_stationary_max_end: usize,
        a_linearization_floor: f64,
        options: ffi::ClarabelOptionsBridge,
    ) -> Result<Box<Profile3rdResult>, String> {
        let options = clarabel_options_from_bridge(options)?;
        self.with_topp3_problem(
            idx_s_start,
            a_linearization,
            a_start,
            a_final,
            b_start,
            b_final,
            num_stationary_max_start,
            num_stationary_max_end,
            a_linearization_floor,
            |problem| crate::solver::topp3_socp::topp3_socp(problem, &options),
        )
        .map(|profile| Box::new(Profile3rdResult::from(profile)))
    }

    /// Solve TOPP3-SOCP and return full Clarabel diagnostics.
    #[allow(clippy::too_many_arguments)]
    fn topp3_socp_solve_expert(
        &mut self,
        idx_s_start: usize,
        a_linearization: &[f64],
        a_start: f64,
        a_final: f64,
        b_start: f64,
        b_final: f64,
        num_stationary_max_start: usize,
        num_stationary_max_end: usize,
        a_linearization_floor: f64,
        options: ffi::ClarabelOptionsBridge,
    ) -> Result<Box<Topp3SolverResult>, String> {
        let options = clarabel_options_from_bridge(options)?;
        self.with_topp3_problem(
            idx_s_start,
            a_linearization,
            a_start,
            a_final,
            b_start,
            b_final,
            num_stationary_max_start,
            num_stationary_max_end,
            a_linearization_floor,
            |problem| crate::solver::topp3_socp::topp3_socp_expert_with_info(problem, &options),
        )
        .map(|expert| {
            Box::new(topp3_result_from_solution(
                expert.result,
                expert.solution,
                expert.linsolver,
                None,
            ))
        })
    }

    #[allow(clippy::too_many_arguments)]
    fn with_copp3_problem<R>(
        &mut self,
        idx_s_start: usize,
        a_linearization: &[f64],
        a_start: f64,
        a_final: f64,
        b_start: f64,
        b_final: f64,
        num_stationary_max_start: usize,
        num_stationary_max_end: usize,
        a_linearization_floor: f64,
        objectives: &[ffi::ObjectiveDescriptor],
        objective_data: &[f64],
        f: impl FnOnce(
            &crate::copp::copp3::stable::basic::Copp3Problem<'_, CppRobotModel>,
            &[CoppObjective<'_>],
        ) -> Result<R, crate::diag::CoppError>,
    ) -> Result<R, String> {
        let objectives = objectives_from_bridge(objectives, objective_data)?;
        crate::copp::validate_copp3_objectives(
            "copp3_problem",
            &objectives,
            self.inner.dim(),
            a_linearization.len(),
        )
        .map_err(|error| error.to_string())?;
        let problem = crate::solver::copp3_socp::Copp3ProblemBuilder::new(
            &mut self.inner,
            &objectives,
            idx_s_start,
            a_linearization,
            (a_start, a_final),
            (b_start, b_final),
        )
        .with_num_stationary_max_pair((num_stationary_max_start, num_stationary_max_end))
        .with_a_linearization_floor(a_linearization_floor)
        .build_with_linearization()
        .map_err(|error| error.to_string())?;
        f(&problem, &objectives).map_err(|error| error.to_string())
    }

    /// Validate and immediately linearize a COPP3 problem descriptor.
    #[allow(clippy::too_many_arguments)]
    fn copp3_problem_prepare(
        &mut self,
        idx_s_start: usize,
        a_linearization: &[f64],
        a_start: f64,
        a_final: f64,
        b_start: f64,
        b_final: f64,
        num_stationary_max_start: usize,
        num_stationary_max_end: usize,
        a_linearization_floor: f64,
        objectives: &[ffi::ObjectiveDescriptor],
        objective_data: &[f64],
    ) -> Result<ffi::Topp3ProblemInfoBridge, String> {
        self.with_copp3_problem(
            idx_s_start,
            a_linearization,
            a_start,
            a_final,
            b_start,
            b_final,
            num_stationary_max_start,
            num_stationary_max_end,
            a_linearization_floor,
            objectives,
            objective_data,
            |problem, _| {
                topp3_problem_info_from_parts(
                    problem.idx_s_start,
                    problem.a_linearization.len(),
                    problem.num_stationary,
                )
                .map_err(|error| {
                    crate::diag::CoppError::InvalidInput("copp3_problem".into(), error)
                })
            },
        )
    }

    /// Solve COPP3-SOCP and return only the accepted profile.
    #[allow(clippy::too_many_arguments)]
    fn copp3_socp_solve(
        &mut self,
        idx_s_start: usize,
        a_linearization: &[f64],
        a_start: f64,
        a_final: f64,
        b_start: f64,
        b_final: f64,
        num_stationary_max_start: usize,
        num_stationary_max_end: usize,
        a_linearization_floor: f64,
        objectives: &[ffi::ObjectiveDescriptor],
        objective_data: &[f64],
        options: ffi::ClarabelOptionsBridge,
    ) -> Result<Box<Profile3rdResult>, String> {
        let options = clarabel_options_from_bridge(options)?;
        self.with_copp3_problem(
            idx_s_start,
            a_linearization,
            a_start,
            a_final,
            b_start,
            b_final,
            num_stationary_max_start,
            num_stationary_max_end,
            a_linearization_floor,
            objectives,
            objective_data,
            |problem, _| crate::solver::copp3_socp::copp3_socp(problem, &options),
        )
        .map(|profile| Box::new(Profile3rdResult::from(profile)))
    }

    /// Solve COPP3-SOCP and return full Clarabel diagnostics.
    #[allow(clippy::too_many_arguments)]
    fn copp3_socp_solve_expert(
        &mut self,
        idx_s_start: usize,
        a_linearization: &[f64],
        a_start: f64,
        a_final: f64,
        b_start: f64,
        b_final: f64,
        num_stationary_max_start: usize,
        num_stationary_max_end: usize,
        a_linearization_floor: f64,
        objectives: &[ffi::ObjectiveDescriptor],
        objective_data: &[f64],
        options: ffi::ClarabelOptionsBridge,
    ) -> Result<Box<Topp3SolverResult>, String> {
        let options = clarabel_options_from_bridge(options)?;
        self.with_copp3_problem(
            idx_s_start,
            a_linearization,
            a_start,
            a_final,
            b_start,
            b_final,
            num_stationary_max_start,
            num_stationary_max_end,
            a_linearization_floor,
            objectives,
            objective_data,
            |problem, _| {
                let expert =
                    crate::solver::copp3_socp::copp3_socp_expert_with_info(problem, &options)?;
                let objective_breakdown = expert
                    .result
                    .as_ref()
                    .map(|profile| objective_value_copp3_opt(problem, profile.as_parts()));
                Ok(topp3_result_from_solution(
                    expert.result,
                    expert.solution,
                    expert.linsolver,
                    objective_breakdown,
                ))
            },
        )
        .map(Box::new)
    }

    /// Store second-order path derivative data.
    fn set_q_2nd(
        &mut self,
        q_data: &[f64],
        q_desc: ffi::MatrixDescriptor,
        dq_data: &[f64],
        dq_desc: ffi::MatrixDescriptor,
        ddq_data: &[f64],
        ddq_desc: ffi::MatrixDescriptor,
        idx_s: usize,
    ) -> Result<(), String> {
        let q = matrix_from_bridge("q", q_data, q_desc)?;
        let dq = matrix_from_bridge("dq", dq_data, dq_desc)?;
        let ddq = matrix_from_bridge("ddq", ddq_data, ddq_desc)?;
        self.inner
            .with_q(&q.as_view(), &dq.as_view(), &ddq.as_view(), None, idx_s)
            .map(|_| ())
            .map_err(|error| error.to_string())
    }

    /// Store third-order path derivative data.
    fn set_q_3rd(
        &mut self,
        q_data: &[f64],
        q_desc: ffi::MatrixDescriptor,
        dq_data: &[f64],
        dq_desc: ffi::MatrixDescriptor,
        ddq_data: &[f64],
        ddq_desc: ffi::MatrixDescriptor,
        dddq_data: &[f64],
        dddq_desc: ffi::MatrixDescriptor,
        idx_s: usize,
    ) -> Result<(), String> {
        let q = matrix_from_bridge("q", q_data, q_desc)?;
        let dq = matrix_from_bridge("dq", dq_data, dq_desc)?;
        let ddq = matrix_from_bridge("ddq", ddq_data, ddq_desc)?;
        let dddq = matrix_from_bridge("dddq", dddq_data, dddq_desc)?;
        self.inner
            .with_q(
                &q.as_view(),
                &dq.as_view(),
                &ddq.as_view(),
                Some(&dddq.as_view()),
                idx_s,
            )
            .map(|_| ())
            .map_err(|error| error.to_string())
    }

    /// Sample a bridge path and store derivatives up to second order.
    fn set_q_from_path_2nd(
        &mut self,
        path: &PathHandle,
        idx_s_from: usize,
        idx_s_to: usize,
    ) -> Result<(), String> {
        self.inner
            .with_q_from_path_2nd(&path.inner, idx_s_from, idx_s_to)
            .map(|_| ())
            .map_err(|error| error.to_string())
    }

    /// Sample a bridge path and store derivatives up to third order.
    fn set_q_from_path_3rd(
        &mut self,
        path: &PathHandle,
        idx_s_from: usize,
        idx_s_to: usize,
    ) -> Result<(), String> {
        self.inner
            .with_q_from_path_3rd(&path.inner, idx_s_from, idx_s_to)
            .map(|_| ())
            .map_err(|error| error.to_string())
    }

    /// Add one physical limit family from broadcast upper/lower vectors.
    fn add_limits_broadcast(
        &mut self,
        kind: u8,
        upper: &[f64],
        lower: &[f64],
        start_idx_s: usize,
        length: usize,
    ) -> Result<(), String> {
        let dim = self.inner.dim();
        if upper.len() != dim || lower.len() != dim {
            return Err(format!(
                "axial limits: expected upper/lower length {dim}, got {}/{}",
                upper.len(),
                lower.len()
            ));
        }

        let ncols = inferred_limit_length(&self.inner, start_idx_s, length);
        if ncols == 0 {
            return Ok(());
        }

        match kind {
            0 => self
                .inner
                .with_axial_velocity((upper, ncols), (lower, ncols), start_idx_s)
                .map(|_| ())
                .map_err(|error| error.to_string()),
            1 => self
                .inner
                .with_axial_acceleration((upper, ncols), (lower, ncols), start_idx_s)
                .map(|_| ())
                .map_err(|error| error.to_string()),
            2 => self
                .inner
                .with_axial_jerk((upper, ncols), (lower, ncols), start_idx_s)
                .map(|_| ())
                .map_err(|error| error.to_string()),
            3 => self
                .inner
                .with_axial_torque((upper, ncols), (lower, ncols), start_idx_s)
                .map(|_| ())
                .map_err(|error| error.to_string()),
            other => Err(format!("unsupported axial limit kind {other}")),
        }
    }

    /// Add one physical limit family from explicit matrices.
    fn add_limits_matrix(
        &mut self,
        kind: u8,
        upper_data: &[f64],
        upper_desc: ffi::MatrixDescriptor,
        lower_data: &[f64],
        lower_desc: ffi::MatrixDescriptor,
        start_idx_s: usize,
    ) -> Result<(), String> {
        let upper = matrix_from_bridge("upper", upper_data, upper_desc)?;
        let lower = matrix_from_bridge("lower", lower_data, lower_desc)?;
        if upper.shape() != lower.shape() {
            return Err(format!(
                "axial limits: upper shape {:?} does not match lower shape {:?}",
                upper.shape(),
                lower.shape()
            ));
        }
        if upper.nrows() != self.inner.dim() {
            return Err(format!(
                "axial limits: expected {} rows, got {}",
                self.inner.dim(),
                upper.nrows()
            ));
        }
        add_limits_from_matrices(&mut self.inner, kind, &upper, &lower, start_idx_s)
    }

    /// Replace stored first-order upper bounds.
    fn amax_substitute(&mut self, amax: &[f64], idx_s: usize) -> Result<(), String> {
        self.inner
            .constraints
            .amax_substitute(amax, idx_s)
            .map_err(|error| error.to_string())
    }

    /// Clear all stored station and constraint data.
    fn clear_constraints(&mut self, keep_idx_s: bool) {
        self.inner.constraints.clear(keep_idx_s);
    }

    /// Pop stations from the front by count.
    fn pop_front_n(&mut self, n_cols: usize) {
        self.inner
            .constraints
            .pop_front(ModePopConstraints::PopNCols(n_cols));
    }

    /// Pop stations from the back by count.
    fn pop_back_n(&mut self, n_cols: usize) {
        self.inner
            .constraints
            .pop_back(ModePopConstraints::PopNCols(n_cols));
    }

    /// Pop front stations until the logical window starts at `idx_s_cut`.
    fn pop_front_until(&mut self, idx_s_cut: usize) {
        self.inner
            .constraints
            .pop_front(ModePopConstraints::CutAtIdxS(idx_s_cut));
    }

    /// Pop back stations until the logical window ends at `idx_s_cut`.
    fn pop_back_until(&mut self, idx_s_cut: usize) {
        self.inner
            .constraints
            .pop_back(ModePopConstraints::CutAtIdxS(idx_s_cut));
    }

    /// Add or tighten first-order constraints from a vector.
    fn add_constraint_1st_vector(&mut self, amax: &[f64], idx_s: usize) -> Result<(), String> {
        if amax.is_empty() {
            return Ok(());
        }
        self.inner
            .constraints
            .with_constraint_1order(amax, idx_s)
            .map(|_| ())
            .map_err(|error| error.to_string())
    }

    /// Add or tighten first-order constraints from a matrix.
    fn add_constraint_1st_matrix(
        &mut self,
        amax_data: &[f64],
        amax_desc: ffi::MatrixDescriptor,
        idx_s: usize,
    ) -> Result<(), String> {
        let amax = matrix_from_bridge("amax", amax_data, amax_desc)?;
        if amax.ncols() == 0 || amax.nrows() == 0 {
            return Ok(());
        }
        self.inner
            .constraints
            .with_constraint_1order(&amax.as_view(), idx_s)
            .map(|_| ())
            .map_err(|error| error.to_string())
    }

    /// Add second-order raw constraint rows.
    fn add_constraint_2nd(
        &mut self,
        acc_a_data: &[f64],
        acc_a_desc: ffi::MatrixDescriptor,
        acc_b_data: &[f64],
        acc_b_desc: ffi::MatrixDescriptor,
        acc_max_data: &[f64],
        acc_max_desc: ffi::MatrixDescriptor,
        idx_s: usize,
        is_negative: bool,
    ) -> Result<(), String> {
        let acc_a = matrix_from_bridge("acc_a", acc_a_data, acc_a_desc)?;
        let acc_b = matrix_from_bridge("acc_b", acc_b_data, acc_b_desc)?;
        let acc_max = matrix_from_bridge("acc_max", acc_max_data, acc_max_desc)?;
        if acc_a.shape() != acc_b.shape() || acc_a.shape() != acc_max.shape() {
            return Err("second-order constraints: matrix shapes do not match".into());
        }
        if acc_a.ncols() == 0 || acc_a.nrows() == 0 {
            return Ok(());
        }
        self.inner
            .constraints
            .with_constraint_2order(
                &acc_a.as_view(),
                &acc_b.as_view(),
                &acc_max.as_view(),
                idx_s,
                is_negative,
            )
            .map(|_| ())
            .map_err(|error| error.to_string())
    }

    /// Add third-order raw constraint rows.
    #[allow(clippy::too_many_arguments)]
    fn add_constraint_3rd(
        &mut self,
        jerk_a_data: &[f64],
        jerk_a_desc: ffi::MatrixDescriptor,
        jerk_b_data: &[f64],
        jerk_b_desc: ffi::MatrixDescriptor,
        jerk_c_data: &[f64],
        jerk_c_desc: ffi::MatrixDescriptor,
        jerk_d_data: &[f64],
        jerk_d_desc: ffi::MatrixDescriptor,
        jerk_max_data: &[f64],
        jerk_max_desc: ffi::MatrixDescriptor,
        idx_s: usize,
        is_negative: bool,
    ) -> Result<(), String> {
        let jerk_a = matrix_from_bridge("jerk_a", jerk_a_data, jerk_a_desc)?;
        let jerk_b = matrix_from_bridge("jerk_b", jerk_b_data, jerk_b_desc)?;
        let jerk_c = matrix_from_bridge("jerk_c", jerk_c_data, jerk_c_desc)?;
        let jerk_d = matrix_from_bridge("jerk_d", jerk_d_data, jerk_d_desc)?;
        let jerk_max = matrix_from_bridge("jerk_max", jerk_max_data, jerk_max_desc)?;
        let shape = jerk_a.shape();
        if [
            jerk_b.shape(),
            jerk_c.shape(),
            jerk_d.shape(),
            jerk_max.shape(),
        ]
        .iter()
        .any(|&other| other != shape)
        {
            return Err("third-order constraints: matrix shapes do not match".into());
        }
        if jerk_a.ncols() == 0 || jerk_a.nrows() == 0 {
            return Ok(());
        }
        self.inner
            .constraints
            .with_constraint_3order(
                &jerk_a.as_view(),
                &jerk_b.as_view(),
                &jerk_c.as_view(),
                &jerk_d.as_view(),
                &jerk_max.as_view(),
                idx_s,
                is_negative,
            )
            .map(|_| ())
            .map_err(|error| error.to_string())
    }
}

impl VecF64Result {
    /// Borrow the owned vector result as a slice for immediate C++ copying.
    fn values(&self) -> &[f64] {
        &self.values
    }
}

impl From<crate::copp::copp3::Topp3Profile> for Profile3rdResult {
    fn from(profile: crate::copp::copp3::Topp3Profile) -> Self {
        let (a, b, num_stationary) = profile.into_parts();
        Self {
            a,
            b,
            num_stationary,
        }
    }
}

impl Profile3rdResult {
    /// Borrow node profile `a(s) = dot{s}^2` for immediate C++ copying.
    fn a(&self) -> &[f64] {
        &self.a
    }

    /// Borrow node profile `b(s) = ddot{s}` for immediate C++ copying.
    fn b(&self) -> &[f64] {
        &self.b
    }

    /// Number of stationary intervals at the beginning of the profile.
    fn num_stationary_start(&self) -> usize {
        self.num_stationary.0
    }

    /// Number of stationary intervals at the end of the profile.
    fn num_stationary_end(&self) -> usize {
        self.num_stationary.1
    }
}

impl ReachSet2Result {
    /// Borrow upper reachable bounds for immediate C++ copying.
    fn a_max(&self) -> &[f64] {
        &self.a_max
    }

    /// Borrow lower reachable bounds for immediate C++ copying.
    fn a_min(&self) -> &[f64] {
        &self.a_min
    }
}

impl Copp2SocpResult {
    fn has_a(&self) -> bool {
        self.a.is_some()
    }

    fn a(&self) -> &[f64] {
        self.a.as_deref().unwrap_or(&[])
    }

    fn x(&self) -> &[f64] {
        &self.x
    }

    fn z(&self) -> &[f64] {
        &self.z
    }

    fn s(&self) -> &[f64] {
        &self.s
    }

    fn solver_status(&self) -> u8 {
        self.solver_status
    }

    fn obj_val(&self) -> f64 {
        self.obj_val
    }

    fn obj_val_dual(&self) -> f64 {
        self.obj_val_dual
    }

    fn solve_time(&self) -> f64 {
        self.solve_time
    }

    fn iterations(&self) -> u32 {
        self.iterations
    }

    fn r_prim(&self) -> f64 {
        self.r_prim
    }

    fn r_dual(&self) -> f64 {
        self.r_dual
    }

    fn linsolver(&self) -> ffi::LinearSolverInfoBridge {
        ffi::LinearSolverInfoBridge {
            method: self.linsolver.method,
            threads: self.linsolver.threads,
            direct: self.linsolver.direct,
            nnz_a: self.linsolver.nnz_a,
            nnz_l: self.linsolver.nnz_l,
        }
    }

    fn has_objective_value(&self) -> bool {
        self.objective_value.is_some()
    }

    fn objective_value(&self) -> f64 {
        self.objective_value.unwrap_or(f64::NAN)
    }

    fn objective_terms(&self) -> &[f64] {
        &self.objective_terms
    }
}

impl Topp3SolverResult {
    fn has_profile(&self) -> bool {
        self.profile.is_some()
    }

    fn profile_a(&self) -> &[f64] {
        self.profile
            .as_ref()
            .map(|profile| profile.a.as_slice())
            .unwrap_or(&[])
    }

    fn profile_b(&self) -> &[f64] {
        self.profile
            .as_ref()
            .map(|profile| profile.b.as_slice())
            .unwrap_or(&[])
    }

    fn profile_num_stationary_start(&self) -> usize {
        self.profile
            .as_ref()
            .map(|profile| profile.num_stationary.0)
            .unwrap_or(0)
    }

    fn profile_num_stationary_end(&self) -> usize {
        self.profile
            .as_ref()
            .map(|profile| profile.num_stationary.1)
            .unwrap_or(0)
    }

    fn x(&self) -> &[f64] {
        &self.x
    }

    fn z(&self) -> &[f64] {
        &self.z
    }

    fn s(&self) -> &[f64] {
        &self.s
    }

    fn solver_status(&self) -> u8 {
        self.solver_status
    }

    fn obj_val(&self) -> f64 {
        self.obj_val
    }

    fn obj_val_dual(&self) -> f64 {
        self.obj_val_dual
    }

    fn solve_time(&self) -> f64 {
        self.solve_time
    }

    fn iterations(&self) -> u32 {
        self.iterations
    }

    fn r_prim(&self) -> f64 {
        self.r_prim
    }

    fn r_dual(&self) -> f64 {
        self.r_dual
    }

    fn linsolver(&self) -> ffi::LinearSolverInfoBridge {
        ffi::LinearSolverInfoBridge {
            method: self.linsolver.method,
            threads: self.linsolver.threads,
            direct: self.linsolver.direct,
            nnz_a: self.linsolver.nnz_a,
            nnz_l: self.linsolver.nnz_l,
        }
    }

    fn has_objective_value(&self) -> bool {
        self.objective_value.is_some()
    }

    fn objective_value(&self) -> f64 {
        self.objective_value.unwrap_or(f64::NAN)
    }

    fn objective_terms(&self) -> &[f64] {
        &self.objective_terms
    }
}

impl PathEvalQResult {
    fn q(&self) -> &[f64] {
        &self.q
    }
}

impl PathEval2Result {
    fn q(&self) -> &[f64] {
        &self.q
    }

    fn dq(&self) -> &[f64] {
        &self.dq
    }

    fn ddq(&self) -> &[f64] {
        &self.ddq
    }
}

impl PathEval3Result {
    fn q(&self) -> &[f64] {
        &self.q
    }

    fn dq(&self) -> &[f64] {
        &self.dq
    }

    fn ddq(&self) -> &[f64] {
        &self.ddq
    }

    fn dddq(&self) -> &[f64] {
        &self.dddq
    }
}

impl TimeProfile2Result {
    fn ok(&self) -> bool {
        self.ok
    }

    fn message(&self) -> String {
        self.message.clone()
    }

    fn t_final(&self) -> f64 {
        self.t_final
    }

    fn t_s(&self) -> &[f64] {
        &self.t_s
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Test purpose: verify the Rust bridge exposes the crate version used by
    /// the public C++ `copp::version()` facade.
    #[test]
    fn version_smoke() {
        assert!(!version().is_empty());
    }

    /// Test purpose: exercise TOPP2 interpolation bridge functions and result
    /// accessors without going through the public C++ wrapper.
    #[test]
    fn s_to_t_topp2_smoke() {
        let s = [0.0, 0.5, 1.0];
        let a = [1.0, 1.0, 1.0];

        let b = a_to_b_topp2(&s, &a).unwrap();
        let result = s_to_t_topp2(&s, &a, 0.0);
        let s_t_uniform = t_to_s_topp2_uniform(&s, &a, result.t_s(), 0.0, 0.25, true).unwrap();
        let t_sample = [0.0, 0.25, 0.5, 0.75, 1.0];
        let s_t_samples = t_to_s_topp2_samples(&s, &a, result.t_s(), &t_sample).unwrap();

        assert_eq!(b.values(), &[0.0, 0.0]);
        assert!(result.ok());
        assert_eq!(result.message(), "");
        assert!((result.t_final() - 1.0).abs() < 1.0e-12);
        assert_eq!(result.t_s(), &[0.0, 0.5, 1.0]);
        assert_eq!(s_t_uniform.values(), &[0.0, 0.25, 0.5, 0.75, 1.0]);
        assert_eq!(s_t_samples.values(), &[0.0, 0.25, 0.5, 0.75, 1.0]);
    }

    /// Test purpose: exercise TOPP3 interpolation bridge functions, including
    /// node-based `a/b` arrays and stationary-count arguments.
    #[test]
    fn s_to_t_topp3_smoke() {
        let s = [0.0, 0.5, 1.0];
        let a = [1.0, 1.0, 1.0];
        let b = [0.0, 0.0, 0.0];

        let result = s_to_t_topp3(&s, &a, &b, 0, 0, 0.0);
        let s_t_uniform =
            t_to_s_topp3_uniform(&s, &a, &b, 0, 0, result.t_s(), 0.0, 0.25, true).unwrap();
        let t_sample = [0.0, 0.25, 0.5, 0.75, 1.0];
        let s_t_samples = t_to_s_topp3_samples(&s, &a, &b, 0, 0, result.t_s(), &t_sample).unwrap();

        assert!(result.ok());
        assert_eq!(result.message(), "");
        assert!((result.t_final() - 1.0).abs() < 1.0e-12);
        assert_eq!(result.t_s(), &[0.0, 0.5, 1.0]);
        assert_eq!(s_t_uniform.values(), &[0.0, 0.25, 0.5, 0.75, 1.0]);
        assert_eq!(s_t_samples.values(), &[0.0, 0.25, 0.5, 0.75, 1.0]);
    }

    /// Test purpose: run TOPP3-LP, TOPP3-SOCP, and COPP3-SOCP through the raw
    /// bridge to ensure profile/expert result copying stays compatible with Rust.
    #[test]
    fn topp3_solvers_smoke() {
        let s = [0.0, 0.5, 1.0];
        let amax = [1.0, 1.0, 1.0];
        let a_linearization = [0.25, 0.25, 0.25];
        let make_options = || {
            let mut options = clarabel_default_options();
            options.allow_almost_solved = true;
            options
        };
        let objectives = [ffi::ObjectiveDescriptor {
            kind: 0,
            weight: 1.0,
            first_offset: 0,
            first_len: 0,
            second_offset: 0,
            second_len: 0,
        }];

        let mut robot = robot_new(1, 3).unwrap();
        robot.append_s(&s).unwrap();
        robot.add_constraint_1st_vector(&amax, 0).unwrap();

        let info = robot
            .topp3_problem_prepare(0, &a_linearization, 0.25, 0.25, 0.0, 0.0, 1, 1, 1.0e-10)
            .unwrap();
        assert_eq!(info.s_len, 3);
        assert_eq!(info.idx_s_final, 2);
        assert_eq!(info.num_stationary_start, 0);
        assert_eq!(info.num_stationary_end, 0);

        let lp = robot
            .topp3_lp_solve(
                0,
                &a_linearization,
                0.25,
                0.25,
                0.0,
                0.0,
                1,
                1,
                1.0e-10,
                make_options(),
            )
            .unwrap();
        assert_eq!(lp.a().len(), 3);
        assert_eq!(lp.b().len(), 3);

        let socp = robot
            .topp3_socp_solve(
                0,
                &a_linearization,
                0.25,
                0.25,
                0.0,
                0.0,
                1,
                1,
                1.0e-10,
                make_options(),
            )
            .unwrap();
        assert_eq!(socp.a().len(), 3);
        assert_eq!(socp.b().len(), 3);

        let copp_info = robot
            .copp3_problem_prepare(
                0,
                &a_linearization,
                0.25,
                0.25,
                0.0,
                0.0,
                1,
                1,
                1.0e-10,
                &objectives,
                &[],
            )
            .unwrap();
        assert_eq!(copp_info.s_len, 3);

        let expert = robot
            .copp3_socp_solve_expert(
                0,
                &a_linearization,
                0.25,
                0.25,
                0.0,
                0.0,
                1,
                1,
                1.0e-10,
                &objectives,
                &[],
                make_options(),
            )
            .unwrap();
        assert!(expert.has_profile());
        assert_eq!(expert.profile_a().len(), 3);
        assert_eq!(expert.profile_b().len(), 3);
        assert!(expert.has_objective_value());
        assert_eq!(expert.objective_terms().len(), 1);
    }

    /// Test purpose: build a Rust path from bridged waypoint data and verify
    /// second-/third-order evaluation result accessors.
    #[test]
    fn path_from_waypoints_smoke() {
        let waypoints = [
            0.0, 0.0, //
            0.5, 0.25, //
            1.0, 1.0,
        ];
        let path = path_from_waypoints(
            &waypoints,
            2,
            3,
            2,
            5,
            0.0,
            1.0,
            0,
            &[],
            0,
            0,
            0,
            &[],
            0,
            0,
            0,
        )
        .unwrap();
        let s = [0.0, 0.5, 1.0];

        let out = path.evaluate_up_to_2nd(&s).unwrap();
        let out3 = path.evaluate_up_to_3rd(&s).unwrap();

        assert_eq!(path.dim(), 2);
        assert_eq!(path.s_min(), 0.0);
        assert_eq!(path.s_max(), 1.0);
        assert_eq!(out.q().len(), 6);
        assert_eq!(out.dq().len(), 6);
        assert_eq!(out.ddq().len(), 6);
        assert!((out.q()[0] - 0.0).abs() < 1.0e-10);
        assert!((out.q()[1] - 0.0).abs() < 1.0e-10);
        assert!((out.q()[2] - 0.5).abs() < 1.0e-10);
        assert!((out.q()[3] - 0.25).abs() < 1.0e-10);
        assert!((out.q()[4] - 1.0).abs() < 1.0e-10);
        assert!((out.q()[5] - 1.0).abs() < 1.0e-10);
        assert_eq!(out3.q().len(), 6);
        assert_eq!(out3.dq().len(), 6);
        assert_eq!(out3.ddq().len(), 6);
        assert_eq!(out3.dddq().len(), 6);
    }

    /// Test purpose: exercise raw robot/constraint operations exposed to C++,
    /// including physical limits, raw constraint rows, and buffer trimming.
    #[test]
    fn robot_constraints_smoke() {
        let s = [0.0, 0.5, 1.0];
        let waypoints = [
            0.0, 0.0, //
            0.5, 0.25, //
            1.0, 1.0,
        ];
        let path = path_from_waypoints(
            &waypoints,
            2,
            3,
            2,
            5,
            0.0,
            1.0,
            0,
            &[],
            0,
            0,
            0,
            &[],
            0,
            0,
            0,
        )
        .unwrap();

        let mut robot = robot_new(2, 3).unwrap();
        robot.append_s(&s).unwrap();
        robot.set_q_from_path_3rd(&path, 0, 3).unwrap();

        let upper = [3.0, 3.0];
        let lower = [-3.0, -3.0];
        robot
            .add_limits_broadcast(0, &upper, &lower, 0, s.len())
            .unwrap();
        robot
            .add_limits_broadcast(1, &upper, &lower, 0, s.len())
            .unwrap();
        robot
            .add_limits_broadcast(2, &upper, &lower, 0, s.len())
            .unwrap();
        robot
            .add_limits_broadcast(3, &upper, &lower, 0, s.len())
            .unwrap();

        assert_eq!(robot.dim(), 2);
        assert_eq!(robot.len(), 3);
        assert_eq!(robot.idx_s_start(), 0);
        assert_eq!(robot.idx_s_end(), 3);
        assert_eq!(robot.s_values(0, 3).unwrap().values(), &s);
        assert!(robot.amax_value(1).unwrap().is_finite());

        let replacement = [1.0, 0.8, 1.0];
        robot.amax_substitute(&replacement, 0).unwrap();
        assert_eq!(robot.amax_values(0, 3).unwrap().values(), &replacement);

        let mut raw = robot_new(1, 3).unwrap();
        raw.append_s(&s).unwrap();
        raw.add_constraint_1st_vector(&replacement, 0).unwrap();
        assert_eq!(raw.amax_values(0, 3).unwrap().values(), &replacement);

        let zero = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let acc_b = [1.0, -1.0, 1.0, -1.0, 1.0, -1.0];
        let max = [2.0, 2.0, 2.0, 2.0, 2.0, 2.0];
        let desc = || ffi::MatrixDescriptor {
            rows: 2,
            cols: 3,
            leading_dim: 2,
            layout: 0,
        };
        raw.add_constraint_2nd(&zero, desc(), &acc_b, desc(), &max, desc(), 0, false)
            .unwrap();
        raw.add_constraint_3rd(
            &zero,
            desc(),
            &zero,
            desc(),
            &acc_b,
            desc(),
            &zero,
            desc(),
            &max,
            desc(),
            0,
            false,
        )
        .unwrap();

        raw.pop_front_n(1);
        assert_eq!(raw.idx_s_start(), 1);
        raw.pop_back_until(2);
        assert_eq!(raw.idx_s_end(), 2);
        raw.clear_constraints(false);
        assert!(raw.is_empty());
    }

    /// Test purpose: solve TOPP2-RA through the Rust bridge and verify returned
    /// second-order profile bounds.
    #[test]
    fn topp2_ra_smoke() {
        let s = [0.0, 0.5, 1.0];
        let amax = [1.0, 1.0, 1.0];

        let mut constraints = robot_new(1, 3).unwrap();
        constraints.append_s(&s).unwrap();
        constraints.add_constraint_1st_vector(&amax, 0).unwrap();

        assert_eq!(constraints.topp2_problem_s_len(0, 2, 0.0, 0.0).unwrap(), 3);

        let reach = constraints
            .reach_set2_backward(0, 2, 0.0, 0.0, 1.0e-8, 1.0e-8, 1.0e-8, 0)
            .unwrap();
        let reach_bi = constraints
            .reach_set2_bidirectional(0, 2, 0.0, 0.0, 1.0e-8, 1.0e-8, 1.0e-8, 0)
            .unwrap();
        let a = constraints
            .topp2_ra_solve(0, 2, 0.0, 0.0, 1.0e-8, 1.0e-8, 1.0e-8, 0)
            .unwrap();

        assert_eq!(reach.a_max().len(), 3);
        assert_eq!(reach.a_min().len(), 3);
        assert_eq!(reach_bi.a_max().len(), 3);
        assert_eq!(reach_bi.a_min().len(), 3);
        assert_eq!(a.values().len(), 3);
        assert!((a.values()[0] - 0.0).abs() < 1.0e-12);
        assert!((a.values()[2] - 0.0).abs() < 1.0e-12);
        assert!(a.values()[1].is_finite());
        assert!(a.values()[1] >= 0.0);
        assert!(a.values()[1] <= 1.0 + 1.0e-8);
    }

    /// Test purpose: solve COPP2-SOCP in normal/expert modes and verify
    /// Clarabel/objective diagnostics at the bridge layer.
    #[test]
    fn copp2_socp_smoke() {
        let s = [0.0, 0.5, 1.0];
        let amax = [1.0, 1.0, 1.0];
        let objectives = [ffi::ObjectiveDescriptor {
            kind: 0,
            weight: 1.0,
            first_offset: 0,
            first_len: 0,
            second_offset: 0,
            second_len: 0,
        }];

        let mut robot = robot_new(1, 3).unwrap();
        robot.append_s(&s).unwrap();
        robot.add_constraint_1st_vector(&amax, 0).unwrap();

        assert_eq!(
            robot
                .copp2_problem_s_len(0, 2, 0.0, 0.0, &objectives, &[])
                .unwrap(),
            3
        );

        let a = robot
            .copp2_socp_solve(0, 2, 0.0, 0.0, &objectives, &[], clarabel_default_options())
            .unwrap();
        assert_eq!(a.values().len(), 3);
        assert!((a.values()[0] - 0.0).abs() < 1.0e-8);
        assert!((a.values()[2] - 0.0).abs() < 1.0e-8);
        assert!(a.values()[1].is_finite());

        let expert = robot
            .copp2_socp_solve_expert(0, 2, 0.0, 0.0, &objectives, &[], clarabel_default_options())
            .unwrap();
        assert!(expert.has_a());
        assert_eq!(expert.a().len(), 3);
        assert!(!expert.x().is_empty());
        let status = expert.solver_status();
        assert!(
            status == solver_status_to_bridge(SolverStatus::Solved)
                || status == solver_status_to_bridge(SolverStatus::AlmostSolved)
        );
        assert!(expert.obj_val().is_finite());
        assert!(expert.has_objective_value());
        assert_eq!(expert.objective_terms().len(), 1);
    }
}
