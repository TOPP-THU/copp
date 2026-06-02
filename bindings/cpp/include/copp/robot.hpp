#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "copp/core.hpp"
#include "copp/path.hpp"

namespace copp
{

    class Robot;
    class ConstraintsRef;

    /// User-supplied inverse dynamics callback.
    ///
    /// The callback receives one robot state and must overwrite every entry of
    /// `tau`. Inputs are borrowed for the duration of the call only. Throw
    /// `copp::Error` or another `std::exception` to report dynamics failures;
    /// the bridge converts the exception into a Rust dynamics error and then
    /// back into `copp::Error` at the public call site.
    ///
    /// @param q Joint position vector with length `dim`.
    /// @param dq Joint velocity vector with length `dim`.
    /// @param ddq Joint acceleration vector with length `dim`.
    /// @param tau Writable torque vector with length `dim`.
    ///
    /// @code
    /// copp::InverseDynamics dynamics = [](auto q, auto dq, auto ddq, auto tau) {
    ///     for (std::size_t i = 0; i < tau.size(); ++i) {
    ///         tau[i] = ddq[i] + 0.1 * dq[i] + std::sin(q[i]);
    ///     }
    /// };
    /// @endcode
    using InverseDynamics = std::function<void(
        Span<const double> q,
        Span<const double> dq,
        Span<const double> ddq,
        Span<double> tau)>;

    namespace detail
    {
        /// Return the private bridge handle stored by `Robot`.
        ///
        /// This is an implementation hook for solver facade translation units.
        /// Public users should pass `Robot` objects to solver problem
        /// constructors instead of calling this function.
        COPP_API const void *robot_handle(const Robot &robot) noexcept;

        /// Return the mutable private bridge handle stored by `Robot`.
        ///
        /// This is used by third-order problem constructors because immediate
        /// linearization updates cached constraint rows.
        COPP_API void *robot_handle_mut(Robot &robot) noexcept;

        /// Return the private bridge handle stored by `ConstraintsRef`.
        ///
        /// This is an implementation hook for solver facade translation units.
        /// Public users should pass `ConstraintsRef` objects to solver problem
        /// constructors instead of calling this function.
        COPP_API const void *constraints_handle(ConstraintsRef constraints) noexcept;

        /// Return the mutable private bridge handle stored by `ConstraintsRef`.
        ///
        /// This is used by third-order problem constructors because immediate
        /// linearization updates cached constraint rows.
        COPP_API void *constraints_handle_mut(ConstraintsRef constraints) noexcept;
    } // namespace detail

    /// Non-owning view of a Rust-backed constraint buffer.
    ///
    /// `ConstraintsRef` is returned by `Robot::constraints()` and by
    /// `Constraints::ref()`. It is intentionally lightweight and copyable: it
    /// only stores a borrowed pointer to an owning C++ facade object. The owner
    /// must outlive every copied reference.
    ///
    /// The first C++ milestone mirrors the Python `Constraints` surface for raw
    /// TOPP constraints: station management, first-order bound replacement, and
    /// raw first-/second-/third-order constraint rows. Physical velocity,
    /// acceleration, jerk, and torque limits live on `Robot`, because they need
    /// path derivative data and robot semantics.
    class COPP_API ConstraintsRef
    {
    public:
        ConstraintsRef() noexcept = default;

        /// Return whether this reference points to a live bridge handle.
        bool valid() const noexcept;

        /// Return the robot/path dimension.
        std::size_t dim() const;

        /// Return the number of stored path stations.
        std::size_t len() const;

        /// Alias for `len()`, useful for STL-style code.
        std::size_t size() const;

        /// Return the allocated station-buffer capacity.
        std::size_t capacity() const;

        /// Return whether no path stations are stored.
        bool is_empty() const;

        /// Return the active global station-id interval `[first, second)`.
        std::pair<std::size_t, std::size_t> idx_s_range() const;

        /// Append strictly increasing path station samples.
        ///
        /// The values are borrowed only for this call. If the buffer is not
        /// empty, the first new sample must be greater than the current last
        /// stored station.
        ///
        /// @param s New station samples in strictly increasing order.
        /// @return `*this` for chaining.
        /// @throws copp::Error if station order or range validation fails.
        ConstraintsRef &append_s(Span<const double> s);

        /// Export stored station samples over `[idx_s_from, idx_s_to)`.
        std::vector<double> s_values(std::size_t idx_s_from, std::size_t idx_s_to) const;

        /// Export first-order upper bounds over `[idx_s_from, idx_s_to)`.
        std::vector<double> amax_values(std::size_t idx_s_from, std::size_t idx_s_to) const;

        /// Read one stored station sample by global station id.
        double s_value(std::size_t idx_s) const;

        /// Read one first-order upper bound by global station id.
        double amax_value(std::size_t idx_s) const;

        /// Overwrite first-order upper bounds starting at `idx_s`.
        ///
        /// Unlike `add_constraint_1st`, this replaces `amax` values instead of
        /// taking the elementwise minimum. It is mainly used by reachability
        /// algorithms that refine an existing profile.
        ConstraintsRef &amax_substitute(Span<const double> amax, std::size_t idx_s);

        /// Clear all stored stations and constraints.
        ///
        /// If `keep_idx_s` is true, the current global station origin is kept;
        /// otherwise it is reset to zero.
        ConstraintsRef &clear(bool keep_idx_s = false);

        /// Remove `n_cols` logical stations from the front.
        ConstraintsRef &pop_front_n(std::size_t n_cols);

        /// Remove `n_cols` logical stations from the back.
        ConstraintsRef &pop_back_n(std::size_t n_cols);

        /// Remove front stations until the kept window starts at `idx_s_cut`.
        ConstraintsRef &pop_front_until(std::size_t idx_s_cut);

        /// Remove back stations until the kept window ends at or before `idx_s_cut`.
        ConstraintsRef &pop_back_until(std::size_t idx_s_cut);

        /// Add or tighten first-order constraints from a vector.
        ///
        /// The input represents one `amax` value per station. Stored `amax` is
        /// updated as the elementwise minimum of the existing and new values.
        ///
        /// @param amax Node-based upper bounds for `a = (ds/dt)^2`.
        /// @param idx_s Global station id of `amax[0]`.
        ConstraintsRef &add_constraint_1st(Span<const double> amax, std::size_t idx_s);

        /// Add or tighten first-order constraints from an `R x N` matrix.
        ///
        /// Each matrix column is reduced to its minimum before being fused into
        /// the stored `amax` row. This matches Rust/Python raw constraint
        /// semantics.
        ConstraintsRef &add_constraint_1st(MatrixView amax, std::size_t idx_s);

        /// Append second-order raw constraint rows.
        ///
        /// For every station column, each row encodes
        /// `acc_a * a + acc_b * b <= acc_max`. If `is_negative` is true, the row
        /// is inserted with signs flipped; this is how lower bounds are
        /// represented internally.
        ///
        /// @param acc_a Matrix of `a` coefficients.
        /// @param acc_b Matrix of `b = dds/dt` coefficients.
        /// @param acc_max Matrix of right-hand sides.
        /// @param idx_s Global station id of the first column.
        /// @param is_negative Whether to insert the sign-flipped lower-bound row.
        ConstraintsRef &add_constraint_2nd(
            MatrixView acc_a,
            MatrixView acc_b,
            MatrixView acc_max,
            std::size_t idx_s,
            bool is_negative = false);

        /// Append third-order raw constraint rows.
        ///
        /// For every station column, each row encodes
        /// `sqrt(a) * (jerk_a*a + jerk_b*b + jerk_c*c + jerk_d) <= jerk_max`.
        /// Linearization is performed later by TOPP3/COPP3 problem builders.
        ///
        /// @param jerk_a Coefficient matrix multiplying `a`.
        /// @param jerk_b Coefficient matrix multiplying `b`.
        /// @param jerk_c Coefficient matrix multiplying third-order control `c`.
        /// @param jerk_d Constant term matrix.
        /// @param jerk_max Right-hand-side matrix.
        /// @param idx_s Global station id of the first column.
        /// @param is_negative Whether to insert the sign-flipped lower-bound row.
        ConstraintsRef &add_constraint_3rd(
            MatrixView jerk_a,
            MatrixView jerk_b,
            MatrixView jerk_c,
            MatrixView jerk_d,
            MatrixView jerk_max,
            std::size_t idx_s,
            bool is_negative = false);

    private:
        explicit ConstraintsRef(void *handle) noexcept;

        void *handle_ = nullptr;

        friend class Robot;
        friend class Constraints;
        friend const void *detail::constraints_handle(ConstraintsRef constraints) noexcept;
        friend void *detail::constraints_handle_mut(ConstraintsRef constraints) noexcept;
    };

    /// Owning raw constraint-buffer facade.
    ///
    /// This class mirrors Python's independently constructed `Constraints`.
    /// Internally it owns the same Rust robot-handle shape as `Robot`, but
    /// exposes only raw constraint-buffer operations. This keeps solver problem
    /// construction uniform: both `Robot::constraints()` and `Constraints::ref()`
    /// produce a `ConstraintsRef`.
    ///
    /// @code
    /// copp::Constraints constraints(1, 3);
    /// constraints.append_s(std::vector<double>{0.0, 0.5, 1.0})
    ///            .add_constraint_1st(std::vector<double>{1.0, 1.0, 1.0}, 0);
    /// auto problem = copp::solver::topp2_ra::Problem{
    ///     constraints.ref(),
    ///     copp::IndexInterval{0, 2},
    /// };
    /// @endcode
    class COPP_API Constraints
    {
    public:
        explicit Constraints(std::size_t dim, std::size_t capacity = 0);

        Constraints(Constraints &&) noexcept;
        Constraints &operator=(Constraints &&) noexcept;
        ~Constraints();

        Constraints(const Constraints &) = delete;
        Constraints &operator=(const Constraints &) = delete;

        /// Borrow this owning object as a solver/constraint reference.
        ConstraintsRef ref() noexcept;

        std::size_t dim() const;
        std::size_t len() const;
        std::size_t size() const;
        std::size_t capacity() const;
        bool is_empty() const;
        std::pair<std::size_t, std::size_t> idx_s_range() const;

        Constraints &append_s(Span<const double> s);
        std::vector<double> s_values(std::size_t idx_s_from, std::size_t idx_s_to) const;
        std::vector<double> amax_values(std::size_t idx_s_from, std::size_t idx_s_to) const;
        double s_value(std::size_t idx_s) const;
        double amax_value(std::size_t idx_s) const;
        Constraints &amax_substitute(Span<const double> amax, std::size_t idx_s);
        Constraints &clear(bool keep_idx_s = false);
        Constraints &pop_front_n(std::size_t n_cols);
        Constraints &pop_back_n(std::size_t n_cols);
        Constraints &pop_front_until(std::size_t idx_s_cut);
        Constraints &pop_back_until(std::size_t idx_s_cut);
        Constraints &add_constraint_1st(Span<const double> amax, std::size_t idx_s);
        Constraints &add_constraint_1st(MatrixView amax, std::size_t idx_s);
        Constraints &add_constraint_2nd(
            MatrixView acc_a,
            MatrixView acc_b,
            MatrixView acc_max,
            std::size_t idx_s,
            bool is_negative = false);
        Constraints &add_constraint_3rd(
            MatrixView jerk_a,
            MatrixView jerk_b,
            MatrixView jerk_c,
            MatrixView jerk_d,
            MatrixView jerk_max,
            std::size_t idx_s,
            bool is_negative = false);

    private:
        struct Impl;

        std::unique_ptr<Impl> impl_;
    };

    /// Robot facade backed by Rust `Robot<CppRobotModel>`.
    ///
    /// By default, torque evaluation uses point-mass dynamics (`tau = ddq`).
    /// Passing or setting an inverse-dynamics callback enables robot-specific
    /// torque limits and torque objectives. The callback is owned by `Robot` and
    /// may be replaced or cleared later; any state captured by reference must
    /// outlive the robot or the period in which torque-related operations run.
    ///
    /// A typical TOPP/COPP setup is:
    ///
    /// @code
    /// copp::Robot robot(2, s.size());
    /// robot.append_s(s)
    ///      .set_q_from_path_2nd(path, 0, s.size())
    ///      .add_velocity_limits(upper, lower, 0, s.size())
    ///      .add_acceleration_limits(upper, lower, 0, s.size());
    /// @endcode
    class COPP_API Robot
    {
    public:
        explicit Robot(std::size_t dim, std::size_t capacity = 0);
        Robot(std::size_t dim, InverseDynamics inverse_dynamics, std::size_t capacity = 0);

        Robot(Robot &&) noexcept;
        Robot &operator=(Robot &&) noexcept;
        ~Robot();

        Robot(const Robot &) = delete;
        Robot &operator=(const Robot &) = delete;

        /// Return a borrowed raw constraint-buffer facade.
        ConstraintsRef constraints() noexcept;

        std::size_t dim() const;
        std::size_t len() const;
        std::size_t size() const;
        std::size_t capacity() const;
        bool is_empty() const;
        std::pair<std::size_t, std::size_t> idx_s_range() const;

        /// Return whether a user inverse-dynamics callback is installed.
        bool has_inverse_dynamics() const;

        /// Replace the inverse-dynamics callback.
        ///
        /// Existing station and constraint data are preserved. Future
        /// torque-limit construction and torque objectives use the new callback.
        Robot &set_inverse_dynamics(InverseDynamics inverse_dynamics);

        /// Restore point-mass torque evaluation (`tau = ddq`).
        Robot &clear_inverse_dynamics();

        /// Append strictly increasing path station samples.
        Robot &append_s(Span<const double> s);

        /// Store path derivatives up to second order over an existing station interval.
        ///
        /// Matrices must have shape `(dim, n)` where `n` is the number of
        /// station columns starting at `idx_s`. These are the geometric
        /// derivatives `q(s)`, `dq/ds`, and `d2q/ds2`.
        Robot &set_q_2nd(
            MatrixView q,
            MatrixView dq,
            MatrixView ddq,
            std::size_t idx_s);

        /// Store path derivatives up to third order over an existing station interval.
        ///
        /// Adds `d3q/ds3`, which is required by TOPP3/COPP3 jerk constraints.
        Robot &set_q_3rd(
            MatrixView q,
            MatrixView dq,
            MatrixView ddq,
            MatrixView dddq,
            std::size_t idx_s);

        /// Sample a path and store derivatives up to second order.
        ///
        /// The station samples must already be stored in `[idx_s_from, idx_s_to)`.
        /// The path is evaluated at those station values and copied into the
        /// robot constraint buffer.
        Robot &set_q_from_path_2nd(const Path &path, std::size_t idx_s_from, std::size_t idx_s_to);

        /// Sample a path and store derivatives up to third order.
        ///
        /// Use this before adding jerk limits or constructing third-order
        /// solver problems.
        Robot &set_q_from_path_3rd(const Path &path, std::size_t idx_s_from, std::size_t idx_s_to);

        /// Add velocity limits from broadcast vectors.
        ///
        /// `upper` and `lower` must have length `dim`. `length == 0` means
        /// "infer from the currently stored station window starting at
        /// `start_idx_s`".
        ///
        /// @param upper Per-axis positive upper limits, length `dim`.
        /// @param lower Per-axis negative lower limits, length `dim`.
        /// @param start_idx_s Global station id where the limits start.
        /// @param length Number of station columns to fill; `0` means infer.
        Robot &add_velocity_limits(
            Span<const double> upper,
            Span<const double> lower,
            std::size_t start_idx_s,
            std::size_t length = 0);

        /// Add velocity limits from explicit `(dim x n)` matrices.
        Robot &add_velocity_limits(MatrixView upper, MatrixView lower, std::size_t start_idx_s);

        /// Add acceleration limits from broadcast vectors.
        ///
        /// Limits are physical joint-space bounds. COPP maps them into
        /// second-order path constraints using the stored `q`, `dq`, and `ddq`.
        Robot &add_acceleration_limits(
            Span<const double> upper,
            Span<const double> lower,
            std::size_t start_idx_s,
            std::size_t length = 0);

        /// Add acceleration limits from explicit `(dim x n)` matrices.
        Robot &add_acceleration_limits(MatrixView upper, MatrixView lower, std::size_t start_idx_s);

        /// Add jerk limits from broadcast vectors.
        ///
        /// Requires third-order path derivatives already stored by
        /// `set_q_3rd` or `set_q_from_path_3rd`.
        Robot &add_jerk_limits(
            Span<const double> upper,
            Span<const double> lower,
            std::size_t start_idx_s,
            std::size_t length = 0);

        /// Add jerk limits from explicit `(dim x n)` matrices.
        Robot &add_jerk_limits(MatrixView upper, MatrixView lower, std::size_t start_idx_s);

        /// Add torque limits from broadcast vectors.
        ///
        /// Uses the installed inverse-dynamics callback when present; otherwise
        /// uses point-mass dynamics (`tau = ddq`).
        ///
        /// @note This operation can be substantially more expensive than
        /// velocity/acceleration/jerk limits when a user callback is installed,
        /// because Rust evaluates inverse dynamics while constructing
        /// station-indexed torque constraints.
        Robot &add_torque_limits(
            Span<const double> upper,
            Span<const double> lower,
            std::size_t start_idx_s,
            std::size_t length = 0);

        /// Add torque limits from explicit `(dim x n)` matrices.
        Robot &add_torque_limits(MatrixView upper, MatrixView lower, std::size_t start_idx_s);

    private:
        struct Impl;

        std::unique_ptr<Impl> impl_;

        friend const void *detail::robot_handle(const Robot &robot) noexcept;
        friend void *detail::robot_handle_mut(Robot &robot) noexcept;
    };

} // namespace copp
