#include "copp/robot.hpp"

// Robot and Constraints facade implementation.
//
// Both C++ `Robot` and independent `Constraints` are backed by the same opaque
// Rust `RobotHandle`. `Robot` adds physical semantics and optional inverse
// dynamics; `Constraints` exposes raw TOPP/COPP rows. This file keeps the public
// classes small and hides all generated cxx types behind PIMPL/opaque handles.

#include <string>
#include <utility>
#include <vector>

#include "rust/cxx.h"
#include "copp/detail/robot_bridge.hpp"
#include "copp/src/ffi/cpp/mod.rs.h"

namespace
{

    enum class LimitKind : uint8_t
    {
        Velocity = 0,
        Acceleration = 1,
        Jerk = 2,
        Torque = 3,
    };

    // Borrow a C++ one-dimensional view for the duration of one bridge call.
    rust::Slice<const double> to_rust_slice(copp::Span<const double> values) noexcept
    {
        return rust::Slice<const double>(values.data(), values.size());
    }

    // Borrow the exact backing span covered by a strided matrix view. Rust uses
    // the accompanying MatrixDescriptor to interpret layout and leading stride.
    rust::Slice<const double> to_rust_slice(copp::MatrixView values) noexcept
    {
        return rust::Slice<const double>(values.data(), values.storage_size());
    }

    // Convert public C++ matrix metadata into the private cxx bridge descriptor.
    copp::bridge::MatrixDescriptor to_bridge(copp::MatrixView values)
    {
        return copp::bridge::MatrixDescriptor{
            values.rows(),
            values.cols(),
            values.leading_dim(),
            values.layout() == copp::MatrixLayout::ColumnMajor ? uint8_t{0} : uint8_t{1},
        };
    }

    std::string to_std_string(const rust::Error &error)
    {
        return std::string(error.what());
    }

    [[noreturn]] void throw_bridge_error(const rust::Error &error)
    {
        throw copp::Error(copp::Status::invalid_input, to_std_string(error));
    }

    // Run a bridge operation whose Rust `Result` is surfaced by cxx as
    // `rust::Error`; the public facade rethrows it as `copp::Error`.
    template <typename Fn>
    void call_void(Fn &&fn)
    {
        try
        {
            fn();
        }
        catch (const rust::Error &error)
        {
            throw_bridge_error(error);
        }
    }

    // Same as call_void, but preserves the successful return value.
    template <typename Fn>
    auto call_value(Fn &&fn) -> decltype(fn())
    {
        try
        {
            return fn();
        }
        catch (const rust::Error &error)
        {
            throw_bridge_error(error);
        }
    }

    // Validate only the C++ view contract. Shape semantics are checked on the
    // Rust side where the target operation knows the expected row/column count.
    void check_matrix_view_storage(copp::MatrixView values, const char *name)
    {
        if (values.data() == nullptr && values.storage_size() != 0)
        {
            throw copp::Error(copp::Status::invalid_input, std::string(name) + " data is null");
        }
        if (values.layout() == copp::MatrixLayout::ColumnMajor && values.cols() != 0 &&
            values.leading_dim() < values.rows())
        {
            throw copp::Error(
                copp::Status::invalid_input,
                std::string(name) + " leading dimension is smaller than rows");
        }
        if (values.layout() == copp::MatrixLayout::RowMajor && values.rows() != 0 &&
            values.leading_dim() < values.cols())
        {
            throw copp::Error(
                copp::Status::invalid_input,
                std::string(name) + " leading dimension is smaller than cols");
        }
    }

    // Convert the opaque pointer stored by ConstraintsRef back to the private
    // generated bridge type, with a public-facing error for default-constructed refs.
    copp::bridge::RobotHandle *checked_handle(void *handle)
    {
        if (handle == nullptr)
        {
            throw copp::Error(copp::Status::invalid_input, "ConstraintsRef is empty");
        }
        return static_cast<copp::bridge::RobotHandle *>(handle);
    }

    const copp::bridge::RobotHandle *checked_handle(const void *handle)
    {
        if (handle == nullptr)
        {
            throw copp::Error(copp::Status::invalid_input, "ConstraintsRef is empty");
        }
        return static_cast<const copp::bridge::RobotHandle *>(handle);
    }

    // Copy a Rust-owned vector result into the public C++ owned vector type.
    std::vector<double> copy_vec(const rust::Box<copp::bridge::VecF64Result> &values)
    {
        const auto slice = values->values();
        return std::vector<double>(slice.begin(), slice.end());
    }

    // Shared implementation for broadcast-vector physical limits.
    void add_limits_broadcast(
        copp::bridge::RobotHandle &robot,
        LimitKind kind,
        copp::Span<const double> upper,
        copp::Span<const double> lower,
        std::size_t start_idx_s,
        std::size_t length)
    {
        call_void([&]
                  { robot.add_limits_broadcast(
                        static_cast<uint8_t>(kind),
                        to_rust_slice(upper),
                        to_rust_slice(lower),
                        start_idx_s,
                        length); });
    }

    // Shared implementation for explicit matrix physical limits.
    void add_limits_matrix(
        copp::bridge::RobotHandle &robot,
        LimitKind kind,
        copp::MatrixView upper,
        copp::MatrixView lower,
        std::size_t start_idx_s)
    {
        check_matrix_view_storage(upper, "upper");
        check_matrix_view_storage(lower, "lower");
        call_void([&]
                  { robot.add_limits_matrix(
                        static_cast<uint8_t>(kind),
                        to_rust_slice(upper),
                        to_bridge(upper),
                        to_rust_slice(lower),
                        to_bridge(lower),
                        start_idx_s); });
    }

} // namespace

namespace copp::bridge
{

    CppInverseDynamics::CppInverseDynamics(
        std::size_t dim,
        copp::InverseDynamics inverse_dynamics)
        : dim_(dim), inverse_dynamics_(std::move(inverse_dynamics))
    {
        if (dim_ == 0)
        {
            throw copp::Error(copp::Status::invalid_input, "inverse_dynamics dimension must be positive");
        }
        if (!inverse_dynamics_)
        {
            throw copp::Error(copp::Status::invalid_input, "inverse_dynamics callback is empty");
        }
    }

    std::size_t CppInverseDynamics::dim() const noexcept
    {
        return dim_;
    }

    void CppInverseDynamics::inverse_dynamics(
        rust::Slice<const double> q,
        rust::Slice<const double> dq,
        rust::Slice<const double> ddq,
        rust::Slice<double> tau) const
    {
        if (q.size() != dim_ || dq.size() != dim_ || ddq.size() != dim_ || tau.size() != dim_)
        {
            throw copp::Error(copp::Status::invalid_input, "inverse_dynamics buffer shape mismatch");
        }

        std::lock_guard<std::mutex> lock(mutex_);
        inverse_dynamics_(
            copp::Span<const double>(q.data(), q.size()),
            copp::Span<const double>(dq.data(), dq.size()),
            copp::Span<const double>(ddq.data(), ddq.size()),
            copp::Span<double>(tau.data(), tau.size()));
    }

} // namespace copp::bridge

namespace copp
{

    ConstraintsRef::ConstraintsRef(void *handle) noexcept : handle_(handle) {}

    namespace detail
    {

        const void *constraints_handle(ConstraintsRef constraints) noexcept
        {
            return constraints.handle_;
        }

        void *constraints_handle_mut(ConstraintsRef constraints) noexcept
        {
            return constraints.handle_;
        }

    } // namespace detail

    bool ConstraintsRef::valid() const noexcept
    {
        return handle_ != nullptr;
    }

    std::size_t ConstraintsRef::dim() const
    {
        return checked_handle(handle_)->dim();
    }

    std::size_t ConstraintsRef::len() const
    {
        return checked_handle(handle_)->len();
    }

    std::size_t ConstraintsRef::size() const
    {
        return len();
    }

    std::size_t ConstraintsRef::capacity() const
    {
        return checked_handle(handle_)->capacity();
    }

    bool ConstraintsRef::is_empty() const
    {
        return checked_handle(handle_)->is_empty();
    }

    std::pair<std::size_t, std::size_t> ConstraintsRef::idx_s_range() const
    {
        const auto *handle = checked_handle(handle_);
        return {handle->idx_s_start(), handle->idx_s_end()};
    }

    ConstraintsRef &ConstraintsRef::append_s(Span<const double> s)
    {
        auto *handle = checked_handle(handle_);
        call_void([&]
                  { handle->append_s(to_rust_slice(s)); });
        return *this;
    }

    std::vector<double> ConstraintsRef::s_values(std::size_t idx_s_from, std::size_t idx_s_to) const
    {
        const auto *handle = checked_handle(handle_);
        auto result = call_value([&]
                                 { return handle->s_values(idx_s_from, idx_s_to); });
        return copy_vec(result);
    }

    std::vector<double> ConstraintsRef::amax_values(std::size_t idx_s_from, std::size_t idx_s_to) const
    {
        const auto *handle = checked_handle(handle_);
        auto result = call_value([&]
                                 { return handle->amax_values(idx_s_from, idx_s_to); });
        return copy_vec(result);
    }

    double ConstraintsRef::s_value(std::size_t idx_s) const
    {
        const auto *handle = checked_handle(handle_);
        return call_value([&]
                          { return handle->s_value(idx_s); });
    }

    double ConstraintsRef::amax_value(std::size_t idx_s) const
    {
        const auto *handle = checked_handle(handle_);
        return call_value([&]
                          { return handle->amax_value(idx_s); });
    }

    ConstraintsRef &ConstraintsRef::amax_substitute(Span<const double> amax, std::size_t idx_s)
    {
        auto *handle = checked_handle(handle_);
        call_void([&]
                  { handle->amax_substitute(to_rust_slice(amax), idx_s); });
        return *this;
    }

    ConstraintsRef &ConstraintsRef::clear(bool keep_idx_s)
    {
        checked_handle(handle_)->clear_constraints(keep_idx_s);
        return *this;
    }

    ConstraintsRef &ConstraintsRef::pop_front_n(std::size_t n_cols)
    {
        checked_handle(handle_)->pop_front_n(n_cols);
        return *this;
    }

    ConstraintsRef &ConstraintsRef::pop_back_n(std::size_t n_cols)
    {
        checked_handle(handle_)->pop_back_n(n_cols);
        return *this;
    }

    ConstraintsRef &ConstraintsRef::pop_front_until(std::size_t idx_s_cut)
    {
        checked_handle(handle_)->pop_front_until(idx_s_cut);
        return *this;
    }

    ConstraintsRef &ConstraintsRef::pop_back_until(std::size_t idx_s_cut)
    {
        checked_handle(handle_)->pop_back_until(idx_s_cut);
        return *this;
    }

    ConstraintsRef &ConstraintsRef::add_constraint_1st(Span<const double> amax, std::size_t idx_s)
    {
        auto *handle = checked_handle(handle_);
        call_void([&]
                  { handle->add_constraint_1st_vector(to_rust_slice(amax), idx_s); });
        return *this;
    }

    ConstraintsRef &ConstraintsRef::add_constraint_1st(MatrixView amax, std::size_t idx_s)
    {
        check_matrix_view_storage(amax, "amax");
        auto *handle = checked_handle(handle_);
        call_void([&]
                  { handle->add_constraint_1st_matrix(to_rust_slice(amax), to_bridge(amax), idx_s); });
        return *this;
    }

    ConstraintsRef &ConstraintsRef::add_constraint_2nd(
        MatrixView acc_a,
        MatrixView acc_b,
        MatrixView acc_max,
        std::size_t idx_s,
        bool is_negative)
    {
        check_matrix_view_storage(acc_a, "acc_a");
        check_matrix_view_storage(acc_b, "acc_b");
        check_matrix_view_storage(acc_max, "acc_max");
        auto *handle = checked_handle(handle_);
        call_void([&]
                  { handle->add_constraint_2nd(
                        to_rust_slice(acc_a),
                        to_bridge(acc_a),
                        to_rust_slice(acc_b),
                        to_bridge(acc_b),
                        to_rust_slice(acc_max),
                        to_bridge(acc_max),
                        idx_s,
                        is_negative); });
        return *this;
    }

    ConstraintsRef &ConstraintsRef::add_constraint_3rd(
        MatrixView jerk_a,
        MatrixView jerk_b,
        MatrixView jerk_c,
        MatrixView jerk_d,
        MatrixView jerk_max,
        std::size_t idx_s,
        bool is_negative)
    {
        check_matrix_view_storage(jerk_a, "jerk_a");
        check_matrix_view_storage(jerk_b, "jerk_b");
        check_matrix_view_storage(jerk_c, "jerk_c");
        check_matrix_view_storage(jerk_d, "jerk_d");
        check_matrix_view_storage(jerk_max, "jerk_max");
        auto *handle = checked_handle(handle_);
        call_void([&]
                  { handle->add_constraint_3rd(
                        to_rust_slice(jerk_a),
                        to_bridge(jerk_a),
                        to_rust_slice(jerk_b),
                        to_bridge(jerk_b),
                        to_rust_slice(jerk_c),
                        to_bridge(jerk_c),
                        to_rust_slice(jerk_d),
                        to_bridge(jerk_d),
                        to_rust_slice(jerk_max),
                        to_bridge(jerk_max),
                        idx_s,
                        is_negative); });
        return *this;
    }

    struct Constraints::Impl
    {
        explicit Impl(rust::Box<bridge::RobotHandle> &&robot) : robot(std::move(robot)) {}

        rust::Box<bridge::RobotHandle> robot;
    };

    Constraints::Constraints(std::size_t dim, std::size_t capacity)
    {
        try
        {
            impl_ = std::make_unique<Impl>(bridge::robot_new(dim, capacity));
        }
        catch (const rust::Error &error)
        {
            throw_bridge_error(error);
        }
    }

    Constraints::Constraints(Constraints &&) noexcept = default;

    Constraints &Constraints::operator=(Constraints &&) noexcept = default;

    Constraints::~Constraints() = default;

    ConstraintsRef Constraints::ref() noexcept
    {
        if (!impl_)
        {
            return ConstraintsRef();
        }
        return ConstraintsRef(&*impl_->robot);
    }

    std::size_t Constraints::dim() const { return ConstraintsRef(impl_ ? &*impl_->robot : nullptr).dim(); }
    std::size_t Constraints::len() const { return ConstraintsRef(impl_ ? &*impl_->robot : nullptr).len(); }
    std::size_t Constraints::size() const { return ConstraintsRef(impl_ ? &*impl_->robot : nullptr).size(); }
    std::size_t Constraints::capacity() const { return ConstraintsRef(impl_ ? &*impl_->robot : nullptr).capacity(); }
    bool Constraints::is_empty() const { return ConstraintsRef(impl_ ? &*impl_->robot : nullptr).is_empty(); }
    std::pair<std::size_t, std::size_t> Constraints::idx_s_range() const
    {
        return ConstraintsRef(impl_ ? &*impl_->robot : nullptr).idx_s_range();
    }
    std::vector<double> Constraints::s_values(std::size_t idx_s_from, std::size_t idx_s_to) const
    {
        return ConstraintsRef(impl_ ? &*impl_->robot : nullptr).s_values(idx_s_from, idx_s_to);
    }
    std::vector<double> Constraints::amax_values(std::size_t idx_s_from, std::size_t idx_s_to) const
    {
        return ConstraintsRef(impl_ ? &*impl_->robot : nullptr).amax_values(idx_s_from, idx_s_to);
    }
    double Constraints::s_value(std::size_t idx_s) const
    {
        return ConstraintsRef(impl_ ? &*impl_->robot : nullptr).s_value(idx_s);
    }
    double Constraints::amax_value(std::size_t idx_s) const
    {
        return ConstraintsRef(impl_ ? &*impl_->robot : nullptr).amax_value(idx_s);
    }

    Constraints &Constraints::append_s(Span<const double> s)
    {
        ref().append_s(s);
        return *this;
    }

    Constraints &Constraints::amax_substitute(Span<const double> amax, std::size_t idx_s)
    {
        ref().amax_substitute(amax, idx_s);
        return *this;
    }

    Constraints &Constraints::clear(bool keep_idx_s)
    {
        ref().clear(keep_idx_s);
        return *this;
    }

    Constraints &Constraints::pop_front_n(std::size_t n_cols)
    {
        ref().pop_front_n(n_cols);
        return *this;
    }

    Constraints &Constraints::pop_back_n(std::size_t n_cols)
    {
        ref().pop_back_n(n_cols);
        return *this;
    }

    Constraints &Constraints::pop_front_until(std::size_t idx_s_cut)
    {
        ref().pop_front_until(idx_s_cut);
        return *this;
    }

    Constraints &Constraints::pop_back_until(std::size_t idx_s_cut)
    {
        ref().pop_back_until(idx_s_cut);
        return *this;
    }

    Constraints &Constraints::add_constraint_1st(Span<const double> amax, std::size_t idx_s)
    {
        ref().add_constraint_1st(amax, idx_s);
        return *this;
    }

    Constraints &Constraints::add_constraint_1st(MatrixView amax, std::size_t idx_s)
    {
        ref().add_constraint_1st(amax, idx_s);
        return *this;
    }

    Constraints &Constraints::add_constraint_2nd(
        MatrixView acc_a,
        MatrixView acc_b,
        MatrixView acc_max,
        std::size_t idx_s,
        bool is_negative)
    {
        ref().add_constraint_2nd(acc_a, acc_b, acc_max, idx_s, is_negative);
        return *this;
    }

    Constraints &Constraints::add_constraint_3rd(
        MatrixView jerk_a,
        MatrixView jerk_b,
        MatrixView jerk_c,
        MatrixView jerk_d,
        MatrixView jerk_max,
        std::size_t idx_s,
        bool is_negative)
    {
        ref().add_constraint_3rd(jerk_a, jerk_b, jerk_c, jerk_d, jerk_max, idx_s, is_negative);
        return *this;
    }

    struct Robot::Impl
    {
        explicit Impl(rust::Box<bridge::RobotHandle> &&robot) : robot(std::move(robot)) {}

        rust::Box<bridge::RobotHandle> robot;
    };

    namespace detail
    {

        const void *robot_handle(const Robot &robot) noexcept
        {
            if (!robot.impl_)
            {
                return nullptr;
            }
            return &*robot.impl_->robot;
        }

        void *robot_handle_mut(Robot &robot) noexcept
        {
            if (!robot.impl_)
            {
                return nullptr;
            }
            return &*robot.impl_->robot;
        }

    } // namespace detail

    Robot::Robot(std::size_t dim, std::size_t capacity)
    {
        try
        {
            impl_ = std::make_unique<Impl>(bridge::robot_new(dim, capacity));
        }
        catch (const rust::Error &error)
        {
            throw_bridge_error(error);
        }
    }

    Robot::Robot(std::size_t dim, InverseDynamics inverse_dynamics, std::size_t capacity)
    {
        try
        {
            impl_ = std::make_unique<Impl>(
                bridge::robot_new_with_inverse_dynamics(
                    std::make_unique<bridge::CppInverseDynamics>(dim, std::move(inverse_dynamics)),
                    capacity));
        }
        catch (const rust::Error &error)
        {
            throw_bridge_error(error);
        }
    }

    Robot::Robot(Robot &&) noexcept = default;

    Robot &Robot::operator=(Robot &&) noexcept = default;

    Robot::~Robot() = default;

    ConstraintsRef Robot::constraints() noexcept
    {
        if (!impl_)
        {
            return ConstraintsRef();
        }
        return ConstraintsRef(&*impl_->robot);
    }

    std::size_t Robot::dim() const { return impl_->robot->dim(); }
    std::size_t Robot::len() const { return impl_->robot->len(); }
    std::size_t Robot::size() const { return len(); }
    std::size_t Robot::capacity() const { return impl_->robot->capacity(); }
    bool Robot::is_empty() const { return impl_->robot->is_empty(); }

    std::pair<std::size_t, std::size_t> Robot::idx_s_range() const
    {
        return {impl_->robot->idx_s_start(), impl_->robot->idx_s_end()};
    }

    bool Robot::has_inverse_dynamics() const
    {
        return impl_->robot->has_inverse_dynamics();
    }

    Robot &Robot::set_inverse_dynamics(InverseDynamics inverse_dynamics)
    {
        const auto dim = this->dim();
        call_void([&]
                  { impl_->robot->set_inverse_dynamics(
                        std::make_unique<bridge::CppInverseDynamics>(dim, std::move(inverse_dynamics))); });
        return *this;
    }

    Robot &Robot::clear_inverse_dynamics()
    {
        impl_->robot->clear_inverse_dynamics();
        return *this;
    }

    Robot &Robot::append_s(Span<const double> s)
    {
        call_void([&]
                  { impl_->robot->append_s(to_rust_slice(s)); });
        return *this;
    }

    Robot &Robot::set_q_2nd(
        MatrixView q,
        MatrixView dq,
        MatrixView ddq,
        std::size_t idx_s)
    {
        check_matrix_view_storage(q, "q");
        check_matrix_view_storage(dq, "dq");
        check_matrix_view_storage(ddq, "ddq");
        call_void([&]
                  { impl_->robot->set_q_2nd(
                        to_rust_slice(q),
                        to_bridge(q),
                        to_rust_slice(dq),
                        to_bridge(dq),
                        to_rust_slice(ddq),
                        to_bridge(ddq),
                        idx_s); });
        return *this;
    }

    Robot &Robot::set_q_3rd(
        MatrixView q,
        MatrixView dq,
        MatrixView ddq,
        MatrixView dddq,
        std::size_t idx_s)
    {
        check_matrix_view_storage(q, "q");
        check_matrix_view_storage(dq, "dq");
        check_matrix_view_storage(ddq, "ddq");
        check_matrix_view_storage(dddq, "dddq");
        call_void([&]
                  { impl_->robot->set_q_3rd(
                        to_rust_slice(q),
                        to_bridge(q),
                        to_rust_slice(dq),
                        to_bridge(dq),
                        to_rust_slice(ddq),
                        to_bridge(ddq),
                        to_rust_slice(dddq),
                        to_bridge(dddq),
                        idx_s); });
        return *this;
    }

    Robot &Robot::set_q_from_path_2nd(const Path &path, std::size_t idx_s_from, std::size_t idx_s_to)
    {
        const auto *path_handle = static_cast<const bridge::PathHandle *>(detail::path_handle(path));
        if (path_handle == nullptr)
        {
            throw Error(Status::invalid_input, "Path bridge handle is null");
        }
        call_void([&]
                  { impl_->robot->set_q_from_path_2nd(*path_handle, idx_s_from, idx_s_to); });
        return *this;
    }

    Robot &Robot::set_q_from_path_3rd(const Path &path, std::size_t idx_s_from, std::size_t idx_s_to)
    {
        const auto *path_handle = static_cast<const bridge::PathHandle *>(detail::path_handle(path));
        if (path_handle == nullptr)
        {
            throw Error(Status::invalid_input, "Path bridge handle is null");
        }
        call_void([&]
                  { impl_->robot->set_q_from_path_3rd(*path_handle, idx_s_from, idx_s_to); });
        return *this;
    }

    Robot &Robot::add_velocity_limits(
        Span<const double> upper,
        Span<const double> lower,
        std::size_t start_idx_s,
        std::size_t length)
    {
        add_limits_broadcast(*impl_->robot, LimitKind::Velocity, upper, lower, start_idx_s, length);
        return *this;
    }

    Robot &Robot::add_velocity_limits(MatrixView upper, MatrixView lower, std::size_t start_idx_s)
    {
        add_limits_matrix(*impl_->robot, LimitKind::Velocity, upper, lower, start_idx_s);
        return *this;
    }

    Robot &Robot::add_acceleration_limits(
        Span<const double> upper,
        Span<const double> lower,
        std::size_t start_idx_s,
        std::size_t length)
    {
        add_limits_broadcast(*impl_->robot, LimitKind::Acceleration, upper, lower, start_idx_s, length);
        return *this;
    }

    Robot &Robot::add_acceleration_limits(MatrixView upper, MatrixView lower, std::size_t start_idx_s)
    {
        add_limits_matrix(*impl_->robot, LimitKind::Acceleration, upper, lower, start_idx_s);
        return *this;
    }

    Robot &Robot::add_jerk_limits(
        Span<const double> upper,
        Span<const double> lower,
        std::size_t start_idx_s,
        std::size_t length)
    {
        add_limits_broadcast(*impl_->robot, LimitKind::Jerk, upper, lower, start_idx_s, length);
        return *this;
    }

    Robot &Robot::add_jerk_limits(MatrixView upper, MatrixView lower, std::size_t start_idx_s)
    {
        add_limits_matrix(*impl_->robot, LimitKind::Jerk, upper, lower, start_idx_s);
        return *this;
    }

    Robot &Robot::add_torque_limits(
        Span<const double> upper,
        Span<const double> lower,
        std::size_t start_idx_s,
        std::size_t length)
    {
        add_limits_broadcast(*impl_->robot, LimitKind::Torque, upper, lower, start_idx_s, length);
        return *this;
    }

    Robot &Robot::add_torque_limits(MatrixView upper, MatrixView lower, std::size_t start_idx_s)
    {
        add_limits_matrix(*impl_->robot, LimitKind::Torque, upper, lower, start_idx_s);
        return *this;
    }

} // namespace copp
