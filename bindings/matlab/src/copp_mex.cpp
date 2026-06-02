#include "mex.hpp"
#include "mexAdapter.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "copp/copp.h"

class MexFunction;

extern "C" CoppStatus matlab_path_evaluator_2nd_callback(void *user_data,
                                                          std::size_t dim,
                                                          std::size_t n,
                                                          const double *s,
                                                          double *q,
                                                          double *dq,
                                                          double *ddq);

extern "C" CoppStatus matlab_path_evaluator_3rd_callback(void *user_data,
                                                          std::size_t dim,
                                                          std::size_t n,
                                                          const double *s,
                                                          double *q,
                                                          double *dq,
                                                          double *ddq,
                                                          double *dddq);
extern "C" CoppStatus matlab_inverse_dynamics_callback(void *user_data,
                                                       std::size_t dim,
                                                       const double *q,
                                                       const double *dq,
                                                       const double *ddq,
                                                       double *tau);

namespace
{
    using matlab::data::Array;
    using matlab::data::ArrayFactory;
    using matlab::data::ArrayType;
    using matlab::data::CharArray;
    using matlab::data::TypedArray;
    using matlab::mex::ArgumentList;

    enum class ValuePolicy
    {
        // Inputs such as s, q, dq, ddq, and solver boundary values must be
        // finite because Rust core validation is intentionally not duplicated
        // at every low-level C ABI boundary.
        FiniteOnly,

        // Limit upper/lower bounds may use +Inf/-Inf for open bounds, but NaN
        // is never meaningful and is rejected before reaching the C ABI.
        AllowInfinity,
    };

    enum class HandleKind
    {
        // Native CoppPath handle owned by the MEX registry.
        Path,

        // Native CoppRobot handle owned by the MEX registry.
        Robot,
    };

    struct PathCallbackContext
    {
        // Back-pointer into this live MEX gateway, used by C callbacks to call
        // MATLAB function handles through MATLABEngine::feval.
        MexFunction *owner;

        // MATLAB evaluator callbacks are kept alive here for exactly as long
        // as the native CoppPath that borrows the C callback/user_data pair.
        Array evaluator_2nd;
        Array evaluator_3rd;
        bool has_evaluator_2nd;
        bool has_evaluator_3rd;

        PathCallbackContext(
            MexFunction *owner_,
            Array evaluator_2nd_,
            Array evaluator_3rd_,
            bool has_evaluator_2nd_,
            bool has_evaluator_3rd_)
            : owner(owner_),
              evaluator_2nd(std::move(evaluator_2nd_)),
              evaluator_3rd(std::move(evaluator_3rd_)),
              has_evaluator_2nd(has_evaluator_2nd_),
              has_evaluator_3rd(has_evaluator_3rd_)
        {
        }
    };

    struct RobotDynamicsCallbackContext
    {
        // MATLAB inverse-dynamics callback retained for exactly as long as the
        // native CoppRobot borrows the C callback/user_data pair.
        MexFunction *owner;
        Array callback;

        RobotDynamicsCallbackContext(MexFunction *owner_, Array callback_)
            : owner(owner_), callback(std::move(callback_))
        {
        }
    };

    struct HandleEntry
    {
        // Runtime type tag used to prevent passing a Robot id where a Path id
        // is expected, and vice versa.
        HandleKind kind;

        // Opaque native pointer. It is freed only through free_entry().
        void *ptr;

        // Optional MATLAB callback state for paths built from evaluators.
        // It must outlive the native path because the C ABI borrows user_data.
        std::unique_ptr<PathCallbackContext> path_callback;

        // Optional MATLAB callback state for Robot inverse dynamics.
        std::unique_ptr<RobotDynamicsCallbackContext> dynamics_callback;
    };

    struct NumericMatrix
    {
        // Column-major double copy of a MATLAB numeric matrix.
        //
        // The Data API may receive single or double arrays. We normalize to
        // double so C ABI calls can always receive CoppMatrixViewF64. MATLAB's
        // iteration order preserves column-major logical order for ordinary
        // arrays, matching CoppMatrixViewF64 with leading_dim = rows.
        std::vector<double> values;

        // Logical row and column count exposed to the C ABI.
        std::size_t rows = 0;
        std::size_t cols = 0;

        CoppMatrixViewF64 view() const
        {
            return CoppMatrixViewF64{
                values.empty() ? nullptr : values.data(),
                rows,
                cols,
                COPP_MATRIX_LAYOUT_COLUMN_MAJOR,
                rows,
            };
        }
    };

    struct ObjectiveStorage
    {
        // Temporary borrowed objective descriptors for one solver call.
        //
        // The descriptor slices point into alpha/beta/normalize below, so this
        // storage object must stay alive until the C ABI solve returns.
        std::vector<double> alpha;
        std::vector<double> beta;
        std::vector<double> normalize;
        std::vector<CoppObjective> objectives;
    };

    class CoppVecGuard
    {
    public:
        // Owned C ABI vector result. The destructor releases the buffer exactly
        // once with copp_vec_f64_free, including on MATLAB error paths.
        CoppVecF64 value{nullptr, 0, 0};

        ~CoppVecGuard()
        {
            copp_vec_f64_free(value);
        }

        CoppVecGuard() = default;
        CoppVecGuard(const CoppVecGuard &) = delete;
        CoppVecGuard &operator=(const CoppVecGuard &) = delete;
    };

    class CoppMatrixGuard
    {
    public:
        // Owned C ABI matrix result. The destructor releases the native buffer
        // exactly once after it has been copied into a MATLAB array.
        CoppMatrixF64 value{nullptr, 0, 0, 0};

        ~CoppMatrixGuard()
        {
            copp_matrix_f64_free(value);
        }

        CoppMatrixGuard() = default;
        CoppMatrixGuard(const CoppMatrixGuard &) = delete;
        CoppMatrixGuard &operator=(const CoppMatrixGuard &) = delete;
    };

    class CoppReachSet2Guard
    {
    public:
        // Owned TOPP2 reach-set result with paired a_max/a_min buffers.
        CoppReachSet2Result value{{nullptr, 0, 0}, {nullptr, 0, 0}};

        ~CoppReachSet2Guard()
        {
            copp_reach_set2_result_free(value);
        }

        CoppReachSet2Guard() = default;
        CoppReachSet2Guard(const CoppReachSet2Guard &) = delete;
        CoppReachSet2Guard &operator=(const CoppReachSet2Guard &) = delete;
    };

    class Copp2SocpResultGuard
    {
    public:
        // Owned expert SOCP result. The C ABI writes an empty result before
        // solving, so freeing default-initialized fields is valid.
        Copp2SocpResult value{};

        ~Copp2SocpResultGuard()
        {
            copp2_socp_result_free(value);
        }

        Copp2SocpResultGuard() = default;
        Copp2SocpResultGuard(const Copp2SocpResultGuard &) = delete;
        Copp2SocpResultGuard &operator=(const Copp2SocpResultGuard &) = delete;
    };

    class CoppProfile3rdGuard
    {
    public:
        // Owned third-order profile result. Both a and b are freed together.
        CoppProfile3rd value{{nullptr, 0, 0}, {nullptr, 0, 0}, 0, 0};

        ~CoppProfile3rdGuard()
        {
            copp_profile_3rd_free(value);
        }

        CoppProfile3rdGuard() = default;
        CoppProfile3rdGuard(const CoppProfile3rdGuard &) = delete;
        CoppProfile3rdGuard &operator=(const CoppProfile3rdGuard &) = delete;
    };

    class Copp3SocpResultGuard
    {
    public:
        Copp3SocpResult value{};

        ~Copp3SocpResultGuard()
        {
            copp3_socp_result_free(value);
        }

        Copp3SocpResultGuard() = default;
        Copp3SocpResultGuard(const Copp3SocpResultGuard &) = delete;
        Copp3SocpResultGuard &operator=(const Copp3SocpResultGuard &) = delete;
    };

    const char *safe_cstr(const char *value)
    {
        return value == nullptr ? "" : value;
    }

    constexpr std::size_t kClarabelSettingsArgCount = 39;
    constexpr std::size_t kClarabelOptionsArgCount = 6 + kClarabelSettingsArgCount;

    const double *slice_data(const std::vector<double> &values)
    {
        return values.empty() ? nullptr : values.data();
    }

    bool is_vector_shape(const Array &array)
    {
        const auto dims = array.getDimensions();
        if (dims.empty())
        {
            return true;
        }
        if (dims.size() == 1)
        {
            return true;
        }
        if (dims.size() == 2)
        {
            return dims[0] == 1 || dims[1] == 1;
        }
        return false;
    }

    std::string lower_ascii(std::string value)
    {
        std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
            return static_cast<char>(std::tolower(c));
        });
        return value;
    }

    const char *kind_name(HandleKind kind)
    {
        switch (kind)
        {
        case HandleKind::Path:
            return "Path";
        case HandleKind::Robot:
            return "Robot";
        }
        return "unknown";
    }

} // namespace

// Private MEX gateway for the MATLAB package.
//
// MATLAB users should call package functions and classes such as
// copp.Robot, copp.Path, and copp.solver.topp2_ra.solve. Those
// wrappers call this single internal gateway with command strings. The command
// strings form the private MATLAB-to-MEX contract:
//
//   version
//       Return the native library version string.
//   last_error
//       Return the current native last-error status and detail message.
//   clarabel_default_options
//       Return shared Clarabel defaults as MATLAB scalars.
//   is_valid, release
//       Inspect or release MEX-registry handles.
//   path_from_waypoints, path_dim, path_s_range
//       Construct and query native CoppPath handles.
//   path_from_evaluator_2nd, path_from_evaluator_3rd
//       Construct callback-backed native CoppPath handles.
//   path_evaluate_up_to_2nd, path_evaluate_up_to_3rd
//       Evaluate native CoppPath handles and copy derivative matrices.
//   robot_create, robot_dim, robot_len, robot_capacity
//       Construct and query native CoppRobot handles.
//   robot_append_s, robot_set_q_2nd, robot_set_q_3rd
//   robot_sample_path_2nd, robot_sample_path_3rd
//       Write station samples and path derivatives.
//   robot_add_velocity_limits, robot_add_acceleration_limits, robot_add_jerk_limits
//   robot_add_torque_limits
//       Add axial bounds.
//   robot_add_raw_constraint_1st/2nd/3rd, robot_clear_constraints,
//   robot_pop_front_n, robot_pop_back_n, robot_set_inverse_dynamics,
//   robot_clear_inverse_dynamics
//       Advanced robot constraint and dynamics operations.
//   topp2_ra_solve
//       Solve TOPP2-RA over a borrowed Robot handle.
//   reach_set2_backward, reach_set2_bidirectional
//       Compute second-order reachable intervals.
//   copp2_socp_solve, copp2_socp_solve_expert
//       Solve COPP2 over a borrowed Robot handle and objective list.
//   topp3_lp_solve, topp3_socp_solve, topp3_lp_solve_expert,
//   topp3_socp_solve_expert
//       Solve TOPP3 Clarabel problems over a borrowed Robot handle.
//   copp3_socp_solve, copp3_socp_solve_expert
//       Solve COPP3 over a borrowed Robot handle and objective list.
//   a_to_b_topp2, s_to_t_topp2, t_to_s_topp2_uniform, t_to_s_topp2_samples
//   s_to_t_topp3, t_to_s_topp3_uniform, t_to_s_topp3_samples
//       Convert and sample second- and third-order profiles.
//
// All native resources are owned by this MEX instance. MATLAB receives only
// uint64 registry ids, never raw pointers.
class MexFunction : public matlab::mex::Function
{
public:
    MexFunction() : matlab_(getEngine()) {}

    ~MexFunction() override
    {
        for (auto &item : handles_)
        {
            free_entry(item.second);
        }
        handles_.clear();
    }

    void operator()(ArgumentList outputs, ArgumentList inputs) override
    {
        if (inputs.empty())
        {
            throw_error("copp:InvalidArgument", "Missing MEX command.");
        }

        const std::string command = command_string(inputs[0]);
        if (command == "version")
        {
            version(outputs, inputs);
            return;
        }
        if (command == "last_error")
        {
            last_error(outputs, inputs);
            return;
        }
        if (command == "clarabel_default_options")
        {
            clarabel_default_options(outputs, inputs);
            return;
        }
        if (command == "is_valid")
        {
            is_valid(outputs, inputs);
            return;
        }
        if (command == "release")
        {
            release(outputs, inputs);
            return;
        }
        if (command == "path_from_waypoints")
        {
            path_from_waypoints(outputs, inputs);
            return;
        }
        if (command == "path_from_evaluator_2nd")
        {
            path_from_evaluator_2nd(outputs, inputs);
            return;
        }
        if (command == "path_from_evaluator_3rd")
        {
            path_from_evaluator_3rd(outputs, inputs);
            return;
        }
        if (command == "path_dim")
        {
            path_dim(outputs, inputs);
            return;
        }
        if (command == "path_s_range")
        {
            path_s_range(outputs, inputs);
            return;
        }
        if (command == "path_evaluate_up_to_2nd")
        {
            path_evaluate_up_to_2nd(outputs, inputs);
            return;
        }
        if (command == "path_evaluate_up_to_3rd")
        {
            path_evaluate_up_to_3rd(outputs, inputs);
            return;
        }
        if (command == "robot_create")
        {
            robot_create(outputs, inputs);
            return;
        }
        if (command == "robot_dim")
        {
            robot_dim(outputs, inputs);
            return;
        }
        if (command == "robot_len")
        {
            robot_len(outputs, inputs);
            return;
        }
        if (command == "robot_capacity")
        {
            robot_capacity(outputs, inputs);
            return;
        }
        if (command == "robot_append_s")
        {
            robot_append_s(outputs, inputs);
            return;
        }
        if (command == "robot_set_q_2nd")
        {
            robot_set_q_2nd(outputs, inputs);
            return;
        }
        if (command == "robot_set_q_3rd")
        {
            robot_set_q_3rd(outputs, inputs);
            return;
        }
        if (command == "robot_sample_path_2nd")
        {
            robot_sample_path_2nd(outputs, inputs);
            return;
        }
        if (command == "robot_sample_path_3rd")
        {
            robot_sample_path_3rd(outputs, inputs);
            return;
        }
        if (command == "robot_add_velocity_limits")
        {
            robot_add_limits(outputs, inputs, true);
            return;
        }
        if (command == "robot_add_acceleration_limits")
        {
            robot_add_limits(outputs, inputs, false);
            return;
        }
        if (command == "robot_add_jerk_limits")
        {
            robot_add_jerk_limits(outputs, inputs);
            return;
        }
        if (command == "robot_add_torque_limits")
        {
            robot_add_torque_limits(outputs, inputs);
            return;
        }
        if (command == "robot_add_raw_constraint_1st")
        {
            robot_add_raw_constraint_1st(outputs, inputs);
            return;
        }
        if (command == "robot_add_raw_constraint_2nd")
        {
            robot_add_raw_constraint_2nd(outputs, inputs);
            return;
        }
        if (command == "robot_add_raw_constraint_3rd")
        {
            robot_add_raw_constraint_3rd(outputs, inputs);
            return;
        }
        if (command == "robot_clear_constraints")
        {
            robot_clear_constraints(outputs, inputs);
            return;
        }
        if (command == "robot_pop_front_n")
        {
            robot_pop_n(outputs, inputs, true);
            return;
        }
        if (command == "robot_pop_back_n")
        {
            robot_pop_n(outputs, inputs, false);
            return;
        }
        if (command == "robot_set_inverse_dynamics")
        {
            robot_set_inverse_dynamics(outputs, inputs);
            return;
        }
        if (command == "robot_clear_inverse_dynamics")
        {
            robot_clear_inverse_dynamics(outputs, inputs);
            return;
        }
        if (command == "topp2_ra_solve")
        {
            topp2_ra_solve(outputs, inputs);
            return;
        }
        if (command == "reach_set2_backward")
        {
            reach_set2(outputs, inputs, false);
            return;
        }
        if (command == "reach_set2_bidirectional")
        {
            reach_set2(outputs, inputs, true);
            return;
        }
        if (command == "copp2_socp_solve")
        {
            copp2_socp_solve(outputs, inputs);
            return;
        }
        if (command == "copp2_socp_solve_expert")
        {
            copp2_socp_solve_expert(outputs, inputs);
            return;
        }
        if (command == "copp3_socp_solve")
        {
            copp3_socp_solve(outputs, inputs);
            return;
        }
        if (command == "copp3_socp_solve_expert")
        {
            copp3_socp_solve_expert(outputs, inputs);
            return;
        }
        if (command == "a_to_b_topp2")
        {
            a_to_b_topp2(outputs, inputs);
            return;
        }
        if (command == "s_to_t_topp2")
        {
            s_to_t_topp2(outputs, inputs);
            return;
        }
        if (command == "t_to_s_topp2_uniform")
        {
            t_to_s_topp2_uniform(outputs, inputs);
            return;
        }
        if (command == "t_to_s_topp2_samples")
        {
            t_to_s_topp2_samples(outputs, inputs);
            return;
        }
        if (command == "topp3_lp_solve")
        {
            topp3_solve(outputs, inputs, false);
            return;
        }
        if (command == "topp3_socp_solve")
        {
            topp3_solve(outputs, inputs, true);
            return;
        }
        if (command == "topp3_lp_solve_expert")
        {
            topp3_solve_expert(outputs, inputs, false);
            return;
        }
        if (command == "topp3_socp_solve_expert")
        {
            topp3_solve_expert(outputs, inputs, true);
            return;
        }
        if (command == "s_to_t_topp3")
        {
            s_to_t_topp3(outputs, inputs);
            return;
        }
        if (command == "t_to_s_topp3_uniform")
        {
            t_to_s_topp3_uniform(outputs, inputs);
            return;
        }
        if (command == "t_to_s_topp3_samples")
        {
            t_to_s_topp3_samples(outputs, inputs);
            return;
        }

        throw_error("copp:InvalidArgument", "Unknown MEX command: " + command + ".");
    }

    CoppStatus evaluate_callback_2nd(PathCallbackContext *context,
                                     std::size_t dim,
                                     std::size_t n,
                                     const double *s,
                                     double *q,
                                     double *dq,
                                     double *ddq) noexcept
    {
        try
        {
            if (context == nullptr || !context->has_evaluator_2nd)
            {
                return callback_error(COPP_STATUS_INVALID_ARGUMENT, "MATLAB second-order path evaluator is not available.");
            }
            const auto results = call_matlab_evaluator(context->evaluator_2nd, 3, n, s);
            if (results.size() != 3)
            {
                return callback_error(COPP_STATUS_INVALID_SHAPE, "MATLAB second-order path evaluator must return q, dq, and ddq.");
            }
            return copy_callback_2nd_results(results, dim, n, q, dq, ddq);
        }
        catch (const std::exception &error)
        {
            return callback_error(COPP_STATUS_SOLVER_OTHER, std::string("MATLAB second-order path evaluator failed: ") + error.what());
        }
        catch (...)
        {
            return callback_error(COPP_STATUS_SOLVER_OTHER, "MATLAB second-order path evaluator failed with an unknown error.");
        }
    }

    CoppStatus evaluate_callback_3rd(PathCallbackContext *context,
                                     std::size_t dim,
                                     std::size_t n,
                                     const double *s,
                                     double *q,
                                     double *dq,
                                     double *ddq,
                                     double *dddq) noexcept
    {
        try
        {
            if (context == nullptr || !context->has_evaluator_3rd)
            {
                return callback_error(COPP_STATUS_INVALID_ARGUMENT, "MATLAB third-order path evaluator is not available.");
            }
            const auto results = call_matlab_evaluator(context->evaluator_3rd, 4, n, s);
            if (results.size() != 4)
            {
                return callback_error(COPP_STATUS_INVALID_SHAPE, "MATLAB third-order path evaluator must return q, dq, ddq, and dddq.");
            }
            CoppStatus status = copy_callback_2nd_results(results, dim, n, q, dq, ddq);
            if (status != COPP_STATUS_OK)
            {
                return status;
            }
            return copy_callback_matrix(results[3], "dddq", dim, n, dddq);
        }
        catch (const std::exception &error)
        {
            return callback_error(COPP_STATUS_SOLVER_OTHER, std::string("MATLAB third-order path evaluator failed: ") + error.what());
        }
        catch (...)
        {
            return callback_error(COPP_STATUS_SOLVER_OTHER, "MATLAB third-order path evaluator failed with an unknown error.");
        }
    }

    CoppStatus evaluate_inverse_dynamics_callback(RobotDynamicsCallbackContext *context,
                                                  std::size_t dim,
                                                  const double *q,
                                                  const double *dq,
                                                  const double *ddq,
                                                  double *tau) noexcept
    {
        try
        {
            if (context == nullptr)
            {
                return callback_error(COPP_STATUS_INVALID_ARGUMENT, "MATLAB inverse-dynamics callback is not available.");
            }
            if (dim > 0 && (q == nullptr || dq == nullptr || ddq == nullptr || tau == nullptr))
            {
                return callback_error(COPP_STATUS_INVALID_ARGUMENT, "MATLAB inverse-dynamics callback received a null state buffer.");
            }

            std::vector<Array> args;
            args.reserve(4);
            args.emplace_back(context->callback);
            args.emplace_back(factory_.createArray<double>({dim, 1}, q, q + dim));
            args.emplace_back(factory_.createArray<double>({dim, 1}, dq, dq + dim));
            args.emplace_back(factory_.createArray<double>({dim, 1}, ddq, ddq + dim));

            const auto results = matlab_->feval(u"feval", 1, std::move(args));
            if (results.size() != 1)
            {
                return callback_error(COPP_STATUS_INVALID_SHAPE, "MATLAB inverse-dynamics callback must return tau.");
            }

            const auto values = copy_real_vector(results[0], "tau", ValuePolicy::FiniteOnly);
            if (values.size() != dim)
            {
                return callback_error(COPP_STATUS_INVALID_SHAPE, "MATLAB inverse-dynamics callback tau must have length dim.");
            }
            std::copy(values.begin(), values.end(), tau);
            return COPP_STATUS_OK;
        }
        catch (const std::exception &error)
        {
            return callback_error(COPP_STATUS_ROBOT_DYNAMICS_ERROR, std::string("MATLAB inverse-dynamics callback failed: ") + error.what());
        }
        catch (...)
        {
            return callback_error(COPP_STATUS_ROBOT_DYNAMICS_ERROR, "MATLAB inverse-dynamics callback failed with an unknown error.");
        }
    }

private:
    ArrayFactory factory_;
    std::shared_ptr<matlab::engine::MATLABEngine> matlab_;

    // Registry from MATLAB-visible uint64 ids to native handles. Each live
    // entry holds one mexLock count so MATLAB cannot clear the MEX file while
    // native resources still exist.
    std::unordered_map<std::uint64_t, HandleEntry> handles_;

    // Monotonic id source. Zero is reserved as the MATLAB-side "no handle"
    // sentinel, so store_handle skips it.
    std::uint64_t next_handle_id_ = 1;

    [[noreturn]] void throw_error(const std::string &identifier, const std::string &message)
    {
        matlab_->feval(u"error", 0, std::vector<Array>{
            factory_.createScalar(identifier),
            factory_.createScalar(message),
        });
        throw std::runtime_error(message);
    }

    void require_input_count(ArgumentList &inputs, std::size_t expected, const std::string &name)
    {
        if (inputs.size() != expected)
        {
            std::ostringstream message;
            message << name << " expects " << expected << " MEX inputs including the command.";
            throw_error("copp:InvalidArgument", message.str());
        }
    }

    void require_output_count_at_most(ArgumentList &outputs, std::size_t max_count, const std::string &name)
    {
        if (outputs.size() > max_count)
        {
            std::ostringstream message;
            message << name << " returns at most " << max_count << " output(s).";
            throw_error("copp:InvalidArgument", message.str());
        }
    }

    void require_output_count_at_least(ArgumentList &outputs, std::size_t min_count, const std::string &name)
    {
        if (outputs.size() < min_count)
        {
            std::ostringstream message;
            message << name << " requires at least " << min_count << " output(s).";
            throw_error("copp:InvalidArgument", message.str());
        }
    }

    std::string command_string(Array input)
    {
        if (input.getType() != ArrayType::CHAR)
        {
            throw_error("copp:InvalidArgument", "MEX command must be a char vector.");
        }
        CharArray chars = std::move(input);
        return chars.toAscii();
    }

    void validate_numeric_value(double value, const std::string &name, ValuePolicy policy)
    {
        if (std::isnan(value))
        {
            throw_error("copp:InvalidArgument", name + " must not contain NaN.");
        }
        if (policy == ValuePolicy::FiniteOnly && !std::isfinite(value))
        {
            throw_error("copp:InvalidArgument", name + " must contain only finite values.");
        }
    }

    template <typename T>
    std::vector<double> copy_numeric_vector(Array input, const std::string &name, ValuePolicy policy)
    {
        if (!is_vector_shape(input))
        {
            throw_error("copp:InvalidArgument", name + " must be a vector.");
        }

        TypedArray<T> typed = std::move(input);
        std::vector<double> values;
        values.reserve(typed.getNumberOfElements());
        for (const auto value : typed)
        {
            const double as_double = static_cast<double>(value);
            validate_numeric_value(as_double, name, policy);
            values.push_back(as_double);
        }
        return values;
    }

    std::vector<double> copy_real_vector(Array input, const std::string &name, ValuePolicy policy = ValuePolicy::FiniteOnly)
    {
        switch (input.getType())
        {
        case ArrayType::DOUBLE:
            return copy_numeric_vector<double>(std::move(input), name, policy);
        case ArrayType::SINGLE:
            return copy_numeric_vector<float>(std::move(input), name, policy);
        default:
            throw_error("copp:InvalidArgument", name + " must be a real double or single vector.");
        }
    }

    template <typename T>
    NumericMatrix copy_numeric_matrix(Array input, const std::string &name, ValuePolicy policy)
    {
        const auto dims = input.getDimensions();
        if (dims.size() > 2)
        {
            throw_error("copp:InvalidArgument", name + " must be a 2-D matrix.");
        }

        NumericMatrix matrix;
        matrix.rows = dims.empty() ? 0 : dims[0];
        matrix.cols = dims.size() < 2 ? 1 : dims[1];

        TypedArray<T> typed = std::move(input);
        matrix.values.reserve(typed.getNumberOfElements());
        for (const auto value : typed)
        {
            const double as_double = static_cast<double>(value);
            validate_numeric_value(as_double, name, policy);
            matrix.values.push_back(as_double);
        }
        return matrix;
    }

    NumericMatrix copy_real_matrix(Array input, const std::string &name, ValuePolicy policy = ValuePolicy::FiniteOnly)
    {
        switch (input.getType())
        {
        case ArrayType::DOUBLE:
            return copy_numeric_matrix<double>(std::move(input), name, policy);
        case ArrayType::SINGLE:
            return copy_numeric_matrix<float>(std::move(input), name, policy);
        default:
            throw_error("copp:InvalidArgument", name + " must be a real double or single matrix.");
        }
    }

    double copy_real_scalar(Array input, const std::string &name, ValuePolicy policy)
    {
        const auto values = copy_real_vector(std::move(input), name, policy);
        if (values.size() != 1)
        {
            throw_error("copp:InvalidArgument", name + " must be a scalar.");
        }
        return values[0];
    }

    double copy_real_scalar(Array input, const std::string &name)
    {
        return copy_real_scalar(std::move(input), name, ValuePolicy::FiniteOnly);
    }

    std::size_t copy_size_scalar(Array input, const std::string &name)
    {
        const double value = copy_real_scalar(std::move(input), name);
        const double max_size = static_cast<double>(std::numeric_limits<std::size_t>::max());
        if (value < 0.0 || value > max_size || std::floor(value) != value)
        {
            throw_error("copp:InvalidArgument", name + " must be a nonnegative integer.");
        }
        return static_cast<std::size_t>(value);
    }

    std::uint32_t copy_u32_scalar(Array input, const std::string &name)
    {
        const std::size_t value = copy_size_scalar(std::move(input), name);
        if (value > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max()))
        {
            throw_error("copp:InvalidArgument", name + " is too large for a uint32 option.");
        }
        return static_cast<std::uint32_t>(value);
    }

    std::vector<std::size_t> copy_size_vector(Array input, const std::string &name)
    {
        const auto values = copy_real_vector(std::move(input), name, ValuePolicy::FiniteOnly);
        const double max_size = static_cast<double>(std::numeric_limits<std::size_t>::max());
        std::vector<std::size_t> out;
        out.reserve(values.size());
        for (const double value : values)
        {
            if (value < 0.0 || value > max_size || std::floor(value) != value)
            {
                throw_error("copp:InvalidArgument", name + " must contain nonnegative integers.");
            }
            out.push_back(static_cast<std::size_t>(value));
        }
        return out;
    }

    bool copy_bool_scalar(Array input, const std::string &name)
    {
        const std::size_t value = copy_size_scalar(std::move(input), name);
        if (value > 1)
        {
            throw_error("copp:InvalidArgument", name + " must be true or false.");
        }
        return value != 0;
    }

    std::uint64_t copy_handle_id(Array input, const std::string &name)
    {
        if (input.getType() != ArrayType::UINT64)
        {
            throw_error("copp:InvalidArgument", name + " must be a uint64 native handle id.");
        }

        TypedArray<std::uint64_t> typed = std::move(input);
        if (typed.getNumberOfElements() != 1)
        {
            throw_error("copp:InvalidArgument", name + " must be a scalar uint64 native handle id.");
        }
        for (const auto value : typed)
        {
            return value;
        }
        return 0;
    }

    std::string last_error_detail()
    {
        std::size_t len = 0;
        const CoppStatus len_status = copp_last_error_message_copy(nullptr, 0, &len);
        if (len_status != COPP_STATUS_OK || len == 0)
        {
            return safe_cstr(copp_last_error_message());
        }

        std::vector<char> buffer(len + 1, '\0');
        const CoppStatus copy_status = copp_last_error_message_copy(buffer.data(), buffer.size(), nullptr);
        if (copy_status != COPP_STATUS_OK)
        {
            return safe_cstr(copp_last_error_message());
        }
        return std::string(buffer.data(), len);
    }

    std::string status_identifier(CoppStatus status)
    {
        const int code = static_cast<int>(status);
        if (code >= 200 && code < 300)
        {
            return "copp:ConstraintError";
        }
        if (code >= 300 && code < 400)
        {
            return "copp:PathError";
        }
        if (code >= 400 && code < 500)
        {
            return "copp:SolverError";
        }
        if (code >= 500 && code < 600)
        {
            return "copp:RobotDynamicsError";
        }
        if (status == COPP_STATUS_NULL_POINTER ||
            status == COPP_STATUS_INVALID_LENGTH ||
            status == COPP_STATUS_INVALID_SHAPE ||
            status == COPP_STATUS_INVALID_ARGUMENT)
        {
            return "copp:InvalidArgument";
        }
        return "copp:NativeError";
    }

    std::string status_message(CoppStatus status)
    {
        std::ostringstream message;
        message << safe_cstr(copp_status_message(status));
        const std::string detail = last_error_detail();
        if (!detail.empty())
        {
            message << ": " << detail;
        }
        return message.str();
    }

    [[noreturn]] void throw_status(CoppStatus status)
    {
        throw_error(status_identifier(status), status_message(status));
    }

    static CoppStatus callback_error(CoppStatus status, const std::string &message)
    {
        copp_set_last_error_message_n(status, message.data(), message.size());
        return status;
    }

    std::vector<Array> call_matlab_evaluator(Array evaluator,
                                             std::size_t output_count,
                                             std::size_t n,
                                             const double *s)
    {
        if (n > 0 && s == nullptr)
        {
            throw std::runtime_error("native path evaluator received a null s buffer.");
        }

        std::vector<Array> args;
        args.reserve(2);
        args.emplace_back(std::move(evaluator));
        if (n == 0)
        {
            args.emplace_back(factory_.createArray<double>({1, 0}));
        }
        else
        {
            args.emplace_back(factory_.createArray<double>({1, n}, s, s + n));
        }
        return matlab_->feval(u"feval", output_count, std::move(args));
    }

    template <typename T>
    CoppStatus copy_callback_numeric_matrix(Array input,
                                            const std::string &name,
                                            std::size_t expected_rows,
                                            std::size_t expected_cols,
                                            double *out)
    {
        const auto dims = input.getDimensions();
        if (dims.size() > 2)
        {
            return callback_error(COPP_STATUS_INVALID_SHAPE, "MATLAB path evaluator output " + name + " must be a 2-D matrix.");
        }

        const std::size_t rows = dims.empty() ? 0 : dims[0];
        const std::size_t cols = dims.size() < 2 ? 1 : dims[1];
        if (rows != expected_rows || cols != expected_cols)
        {
            std::ostringstream message;
            message << "MATLAB path evaluator output " << name
                    << " has shape " << rows << "-by-" << cols
                    << "; expected " << expected_rows << "-by-" << expected_cols << ".";
            return callback_error(COPP_STATUS_INVALID_SHAPE, message.str());
        }

        const std::size_t count = expected_rows * expected_cols;
        if (count > 0 && out == nullptr)
        {
            return callback_error(COPP_STATUS_NULL_POINTER, "native path evaluator received a null output buffer.");
        }

        TypedArray<T> typed = std::move(input);
        std::size_t index = 0;
        for (const auto value : typed)
        {
            const double as_double = static_cast<double>(value);
            if (!std::isfinite(as_double))
            {
                return callback_error(COPP_STATUS_INVALID_ARGUMENT, "MATLAB path evaluator output " + name + " must contain only finite values.");
            }
            out[index++] = as_double;
        }

        if (index != count)
        {
            std::ostringstream message;
            message << "MATLAB path evaluator output " << name
                    << " contained " << index << " elements; expected " << count << ".";
            return callback_error(COPP_STATUS_INVALID_SHAPE, message.str());
        }
        return COPP_STATUS_OK;
    }

    CoppStatus copy_callback_matrix(Array input,
                                    const std::string &name,
                                    std::size_t expected_rows,
                                    std::size_t expected_cols,
                                    double *out)
    {
        switch (input.getType())
        {
        case ArrayType::DOUBLE:
            return copy_callback_numeric_matrix<double>(std::move(input), name, expected_rows, expected_cols, out);
        case ArrayType::SINGLE:
            return copy_callback_numeric_matrix<float>(std::move(input), name, expected_rows, expected_cols, out);
        default:
            return callback_error(COPP_STATUS_INVALID_ARGUMENT, "MATLAB path evaluator output " + name + " must be a real double or single matrix.");
        }
    }

    CoppStatus copy_callback_2nd_results(const std::vector<Array> &results,
                                         std::size_t dim,
                                         std::size_t n,
                                         double *q,
                                         double *dq,
                                         double *ddq)
    {
        CoppStatus status = copy_callback_matrix(results[0], "q", dim, n, q);
        if (status != COPP_STATUS_OK)
        {
            return status;
        }
        status = copy_callback_matrix(results[1], "dq", dim, n, dq);
        if (status != COPP_STATUS_OK)
        {
            return status;
        }
        return copy_callback_matrix(results[2], "ddq", dim, n, ddq);
    }

    Array matrix_output(const CoppMatrixF64 &matrix)
    {
        const std::size_t count = matrix.rows * matrix.cols;
        if (count == 0)
        {
            return factory_.createArray<double>({matrix.rows, matrix.cols});
        }
        return factory_.createArray<double>({matrix.rows, matrix.cols}, matrix.data, matrix.data + count);
    }

    Array usize_column_vector_output(const CoppVecUsize &vec)
    {
        std::vector<double> values;
        values.reserve(vec.len);
        for (std::size_t i = 0; i < vec.len; ++i)
        {
            values.push_back(static_cast<double>(vec.data[i]));
        }
        return factory_.createArray<double>({vec.len, 1}, values.data(), values.data() + values.size());
    }

    HandleKind parse_kind(Array input)
    {
        const std::string kind = lower_ascii(command_string(std::move(input)));
        if (kind == "path")
        {
            return HandleKind::Path;
        }
        if (kind == "robot")
        {
            return HandleKind::Robot;
        }
        throw_error("copp:InvalidArgument", "Unknown native handle kind.");
    }

    CoppPathOutOfRangeMode parse_out_of_range_mode(Array input)
    {
        const std::string mode = lower_ascii(command_string(std::move(input)));
        if (mode == "error")
        {
            return COPP_PATH_OUT_OF_RANGE_MODE_ERROR;
        }
        if (mode == "clamp")
        {
            return COPP_PATH_OUT_OF_RANGE_MODE_CLAMP;
        }
        throw_error("copp:InvalidArgument", "out_of_range_mode must be 'error' or 'clamp'.");
    }

    CoppVerbosity parse_verbosity(Array input)
    {
        const std::string verbosity = lower_ascii(command_string(std::move(input)));
        if (verbosity == "silent")
        {
            return COPP_VERBOSITY_SILENT;
        }
        if (verbosity == "summary")
        {
            return COPP_VERBOSITY_SUMMARY;
        }
        if (verbosity == "debug")
        {
            return COPP_VERBOSITY_DEBUG;
        }
        if (verbosity == "trace")
        {
            return COPP_VERBOSITY_TRACE;
        }
        throw_error("copp:InvalidArgument", "verbosity must be 'silent', 'summary', 'debug', or 'trace'.");
    }

    CoppClarabelDirectSolveMethod parse_direct_solve_method(Array input)
    {
        const std::string method = lower_ascii(command_string(std::move(input)));
        if (method == "auto")
        {
            return COPP_CLARABEL_DIRECT_SOLVE_METHOD_AUTO;
        }
        if (method == "qdldl")
        {
            return COPP_CLARABEL_DIRECT_SOLVE_METHOD_QDLDL;
        }
        if (method == "faer")
        {
            return COPP_CLARABEL_DIRECT_SOLVE_METHOD_FAER;
        }
        if (method == "mkl")
        {
            return COPP_CLARABEL_DIRECT_SOLVE_METHOD_MKL;
        }
        if (method == "panua" || method == "panua_pardiso")
        {
            return COPP_CLARABEL_DIRECT_SOLVE_METHOD_PANUA;
        }
        throw_error("copp:InvalidArgument", "direct_solve_method must be 'auto', 'qdldl', 'faer', 'mkl', or 'panua'.");
    }

    CoppObjectiveKind parse_objective_kind(std::size_t code)
    {
        switch (code)
        {
        case 0:
            return COPP_OBJECTIVE_KIND_TIME;
        case 1:
            return COPP_OBJECTIVE_KIND_LINEAR;
        case 2:
            return COPP_OBJECTIVE_KIND_THERMAL_ENERGY;
        case 3:
            return COPP_OBJECTIVE_KIND_TOTAL_VARIATION_TORQUE;
        default:
            throw_error("copp:InvalidArgument", "Unknown objective kind code.");
        }
    }

    CoppSliceF64 vector_slice(const std::vector<double> &values, std::size_t offset, std::size_t len)
    {
        return CoppSliceF64{
            len == 0 ? nullptr : values.data() + offset,
            len,
        };
    }

    ObjectiveStorage copy_objectives(Array kinds_input,
                                     Array weights_input,
                                     Array alpha_lengths_input,
                                     Array beta_lengths_input,
                                     Array normalize_lengths_input,
                                     Array alpha_data_input,
                                     Array beta_data_input,
                                     Array normalize_data_input)
    {
        const auto kinds = copy_size_vector(std::move(kinds_input), "objective_kinds");
        const auto weights = copy_real_vector(std::move(weights_input), "objective_weights", ValuePolicy::FiniteOnly);
        const auto alpha_lengths = copy_size_vector(std::move(alpha_lengths_input), "objective_alpha_lengths");
        const auto beta_lengths = copy_size_vector(std::move(beta_lengths_input), "objective_beta_lengths");
        const auto normalize_lengths = copy_size_vector(std::move(normalize_lengths_input), "objective_normalize_lengths");

        const std::size_t count = kinds.size();
        if (count == 0)
        {
            throw_error("copp:InvalidArgument", "objectives must not be empty.");
        }
        if (weights.size() != count ||
            alpha_lengths.size() != count ||
            beta_lengths.size() != count ||
            normalize_lengths.size() != count)
        {
            throw_error("copp:InvalidArgument", "objective metadata arrays must have the same length.");
        }

        ObjectiveStorage storage;
        storage.alpha = copy_real_vector(std::move(alpha_data_input), "objective_alpha_data", ValuePolicy::FiniteOnly);
        storage.beta = copy_real_vector(std::move(beta_data_input), "objective_beta_data", ValuePolicy::FiniteOnly);
        storage.normalize = copy_real_vector(std::move(normalize_data_input), "objective_normalize_data", ValuePolicy::FiniteOnly);
        storage.objectives.reserve(count);

        std::size_t alpha_offset = 0;
        std::size_t beta_offset = 0;
        std::size_t normalize_offset = 0;
        for (std::size_t i = 0; i < count; ++i)
        {
            validate_numeric_value(weights[i], "objective_weights", ValuePolicy::FiniteOnly);
            if (weights[i] < 0.0)
            {
                throw_error("copp:InvalidArgument", "objective weights must be nonnegative.");
            }

            const std::size_t alpha_len = alpha_lengths[i];
            const std::size_t beta_len = beta_lengths[i];
            const std::size_t normalize_len = normalize_lengths[i];
            if (alpha_offset + alpha_len > storage.alpha.size() ||
                beta_offset + beta_len > storage.beta.size() ||
                normalize_offset + normalize_len > storage.normalize.size())
            {
                throw_error("copp:InvalidArgument", "objective payload lengths exceed the provided payload data.");
            }

            storage.objectives.push_back(CoppObjective{
                parse_objective_kind(kinds[i]),
                weights[i],
                vector_slice(storage.alpha, alpha_offset, alpha_len),
                vector_slice(storage.beta, beta_offset, beta_len),
                vector_slice(storage.normalize, normalize_offset, normalize_len),
            });

            alpha_offset += alpha_len;
            beta_offset += beta_len;
            normalize_offset += normalize_len;
        }

        if (alpha_offset != storage.alpha.size() ||
            beta_offset != storage.beta.size() ||
            normalize_offset != storage.normalize.size())
        {
            throw_error("copp:InvalidArgument", "objective payload data contains unused trailing values.");
        }

        return storage;
    }

    void free_entry(HandleEntry &entry)
    {
        if (entry.ptr == nullptr)
        {
            return;
        }
        if (entry.kind == HandleKind::Path)
        {
            copp_path_free(static_cast<CoppPath *>(entry.ptr));
        }
        else
        {
            copp_robot_free(static_cast<CoppRobot *>(entry.ptr));
        }
        entry.ptr = nullptr;
        entry.path_callback.reset();
        entry.dynamics_callback.reset();
    }

    std::uint64_t store_handle(HandleKind kind,
                               void *ptr,
                               std::unique_ptr<PathCallbackContext> path_callback = nullptr)
    {
        // Adopt a newly-created native pointer and return the MATLAB registry
        // id that will own it. On success, the MEX file is locked until the id
        // is released.
        if (ptr == nullptr)
        {
            throw_error("copp:NativeError", "Native constructor returned a null handle.");
        }

        for (std::size_t attempts = 0; attempts < std::numeric_limits<std::uint64_t>::max(); ++attempts)
        {
            const std::uint64_t id = next_handle_id_++;
            if (id != 0 && handles_.find(id) == handles_.end())
            {
                handles_.emplace(id, HandleEntry{kind, ptr, std::move(path_callback), nullptr});
                mexLock();
                return id;
            }
        }
        throw_error("copp:NativeError", "Native handle registry exhausted.");
    }

    HandleEntry *find_entry(std::uint64_t id)
    {
        const auto it = handles_.find(id);
        return it == handles_.end() ? nullptr : &it->second;
    }

    void *checked_handle(std::uint64_t id, HandleKind kind)
    {
        // Validate existence and runtime type before any C ABI call. This is
        // the main guard against stale ids and cross-kind misuse.
        HandleEntry *entry = find_entry(id);
        if (entry == nullptr || entry->ptr == nullptr)
        {
            throw_error("copp:InvalidHandle", "Native handle is not valid.");
        }
        if (entry->kind != kind)
        {
            std::ostringstream message;
            message << "Native handle is a " << kind_name(entry->kind)
                    << ", not a " << kind_name(kind) << ".";
            throw_error("copp:InvalidHandle", message.str());
        }
        return entry->ptr;
    }

    bool release_handle(std::uint64_t id)
    {
        // Idempotent registry release used by MATLAB delete()/release().
        // Returns false for already-released handles so finalizers can stay
        // simple and safe.
        const auto it = handles_.find(id);
        if (it == handles_.end())
        {
            return false;
        }

        free_entry(it->second);
        handles_.erase(it);
        mexUnlock();
        return true;
    }

    Array column_vector_output(const CoppVecF64 &vec)
    {
        if (vec.len == 0)
        {
            return factory_.createArray<double>({0, 1});
        }
        return factory_.createArray<double>({vec.len, 1}, vec.data, vec.data + vec.len);
    }

    void write_profile3rd_outputs(ArgumentList outputs, std::size_t offset, const CoppProfile3rd &profile)
    {
        outputs[offset] = column_vector_output(profile.a);
        outputs[offset + 1] = column_vector_output(profile.b);
        outputs[offset + 2] = factory_.createScalar(static_cast<double>(profile.num_stationary_start));
        outputs[offset + 3] = factory_.createScalar(static_cast<double>(profile.num_stationary_end));
    }

    void write_clarabel_options_outputs(ArgumentList outputs, const CoppClarabelOptions &options)
    {
        require_output_count_at_least(outputs, kClarabelOptionsArgCount, "clarabel_default_options");
        require_output_count_at_most(outputs, kClarabelOptionsArgCount, "clarabel_default_options");

        const CoppClarabelSettings &settings = options.clarabel_settings;
        outputs[0] = factory_.createScalar(static_cast<double>(options.verbosity));
        outputs[1] = factory_.createScalar(options.allow_almost_solved);
        outputs[2] = factory_.createScalar(options.allow_max_iterations);
        outputs[3] = factory_.createScalar(options.allow_max_time);
        outputs[4] = factory_.createScalar(options.allow_callback_terminated);
        outputs[5] = factory_.createScalar(options.allow_insufficient_progress);
        outputs[6] = factory_.createScalar(static_cast<double>(settings.max_iter));
        outputs[7] = factory_.createScalar(settings.time_limit);
        outputs[8] = factory_.createScalar(settings.verbose);
        outputs[9] = factory_.createScalar(settings.max_step_fraction);
        outputs[10] = factory_.createScalar(settings.tol_gap_abs);
        outputs[11] = factory_.createScalar(settings.tol_gap_rel);
        outputs[12] = factory_.createScalar(settings.tol_feas);
        outputs[13] = factory_.createScalar(settings.tol_infeas_abs);
        outputs[14] = factory_.createScalar(settings.tol_infeas_rel);
        outputs[15] = factory_.createScalar(settings.tol_ktratio);
        outputs[16] = factory_.createScalar(settings.reduced_tol_gap_abs);
        outputs[17] = factory_.createScalar(settings.reduced_tol_gap_rel);
        outputs[18] = factory_.createScalar(settings.reduced_tol_feas);
        outputs[19] = factory_.createScalar(settings.reduced_tol_infeas_abs);
        outputs[20] = factory_.createScalar(settings.reduced_tol_infeas_rel);
        outputs[21] = factory_.createScalar(settings.reduced_tol_ktratio);
        outputs[22] = factory_.createScalar(settings.equilibrate_enable);
        outputs[23] = factory_.createScalar(static_cast<double>(settings.equilibrate_max_iter));
        outputs[24] = factory_.createScalar(settings.equilibrate_min_scaling);
        outputs[25] = factory_.createScalar(settings.equilibrate_max_scaling);
        outputs[26] = factory_.createScalar(settings.linesearch_backtrack_step);
        outputs[27] = factory_.createScalar(settings.min_switch_step_length);
        outputs[28] = factory_.createScalar(settings.min_terminate_step_length);
        outputs[29] = factory_.createScalar(static_cast<double>(settings.max_threads));
        outputs[30] = factory_.createScalar(settings.direct_kkt_solver);
        outputs[31] = factory_.createScalar(static_cast<double>(settings.direct_solve_method));
        outputs[32] = factory_.createScalar(settings.static_regularization_enable);
        outputs[33] = factory_.createScalar(settings.static_regularization_constant);
        outputs[34] = factory_.createScalar(settings.static_regularization_proportional);
        outputs[35] = factory_.createScalar(settings.dynamic_regularization_enable);
        outputs[36] = factory_.createScalar(settings.dynamic_regularization_eps);
        outputs[37] = factory_.createScalar(settings.dynamic_regularization_delta);
        outputs[38] = factory_.createScalar(settings.iterative_refinement_enable);
        outputs[39] = factory_.createScalar(settings.iterative_refinement_reltol);
        outputs[40] = factory_.createScalar(settings.iterative_refinement_abstol);
        outputs[41] = factory_.createScalar(static_cast<double>(settings.iterative_refinement_max_iter));
        outputs[42] = factory_.createScalar(settings.iterative_refinement_stop_ratio);
        outputs[43] = factory_.createScalar(settings.presolve_enable);
        outputs[44] = factory_.createScalar(settings.input_sparse_dropzeros);
    }

    void write_copp3_result_outputs(ArgumentList outputs, const Copp3SocpResult &result)
    {
        outputs[0] = factory_.createScalar(result.has_profile);
        write_profile3rd_outputs(outputs, 1, result.profile);
        outputs[5] = column_vector_output(result.x);
        outputs[6] = column_vector_output(result.z);
        outputs[7] = column_vector_output(result.s);
        outputs[8] = factory_.createScalar(static_cast<double>(result.solver_status));
        outputs[9] = factory_.createScalar(result.obj_val);
        outputs[10] = factory_.createScalar(result.obj_val_dual);
        outputs[11] = factory_.createScalar(result.solve_time);
        outputs[12] = factory_.createScalar(static_cast<double>(result.iterations));
        outputs[13] = factory_.createScalar(result.r_prim);
        outputs[14] = factory_.createScalar(result.r_dual);
        outputs[15] = factory_.createScalar(result.objective_value);
        outputs[16] = column_vector_output(result.objective_terms);
        outputs[17] = factory_.createScalar(static_cast<double>(result.linsolver.method));
        outputs[18] = factory_.createScalar(static_cast<double>(result.linsolver.threads));
        outputs[19] = factory_.createScalar(result.linsolver.direct);
        outputs[20] = factory_.createScalar(static_cast<double>(result.linsolver.nnz_a));
        outputs[21] = factory_.createScalar(static_cast<double>(result.linsolver.nnz_l));
    }

    void version(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 1, "version");
        require_output_count_at_least(outputs, 1, "version");
        require_output_count_at_most(outputs, 1, "version");
        outputs[0] = factory_.createScalar(std::string(safe_cstr(copp_version())));
    }

    void last_error(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 1, "last_error");
        require_output_count_at_least(outputs, 1, "last_error");
        require_output_count_at_most(outputs, 3, "last_error");

        const CoppStatus status = copp_last_error_code();
        outputs[0] = factory_.createScalar(static_cast<double>(status));
        if (outputs.size() >= 2)
        {
            outputs[1] = factory_.createScalar(std::string(safe_cstr(copp_status_message(status))));
        }
        if (outputs.size() >= 3)
        {
            outputs[2] = factory_.createScalar(last_error_detail());
        }
    }

    void clarabel_default_options(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 1, "clarabel_default_options");
        CoppClarabelOptions options;
        const CoppStatus status = copp_clarabel_default_options(&options);
        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }
        write_clarabel_options_outputs(outputs, options);
    }

    void is_valid(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 3, "is_valid");
        require_output_count_at_least(outputs, 1, "is_valid");
        require_output_count_at_most(outputs, 1, "is_valid");

        const std::uint64_t id = copy_handle_id(inputs[1], "handle_id");
        const HandleKind kind = parse_kind(inputs[2]);
        const HandleEntry *entry = find_entry(id);
        outputs[0] = factory_.createScalar(entry != nullptr && entry->ptr != nullptr && entry->kind == kind);
    }

    void release(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 2, "release");
        require_output_count_at_most(outputs, 1, "release");

        const std::uint64_t id = copy_handle_id(inputs[1], "handle_id");
        const bool released = release_handle(id);
        if (!outputs.empty())
        {
            outputs[0] = factory_.createScalar(released);
        }
    }

    void path_from_waypoints(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 8, "path_from_waypoints");
        require_output_count_at_least(outputs, 1, "path_from_waypoints");
        require_output_count_at_most(outputs, 1, "path_from_waypoints");

        const NumericMatrix waypoints = copy_real_matrix(inputs[1], "waypoints");
        if (waypoints.rows == 0 || waypoints.cols == 0)
        {
            throw_error("copp:InvalidArgument", "waypoints must be a non-empty dim-by-N matrix.");
        }

        const double s_min = copy_real_scalar(inputs[2], "s_min");
        const double s_max = copy_real_scalar(inputs[3], "s_max");
        const std::size_t order = copy_size_scalar(inputs[4], "order");
        const CoppPathOutOfRangeMode out_of_range_mode = parse_out_of_range_mode(inputs[5]);
        const NumericMatrix start_state = copy_real_matrix(inputs[6], "start_state");
        const NumericMatrix end_state = copy_real_matrix(inputs[7], "end_state");

        CoppPathOptions options;
        CoppStatus status = copp_path_default_options(s_min, s_max, &options);
        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }
        options.order = order;
        options.out_of_range_mode = out_of_range_mode;
        options.start_state = start_state.view();
        options.end_state = end_state.view();

        CoppPath *path = nullptr;
        status = copp_path_from_waypoints(waypoints.view(), options, &path);
        if (status != COPP_STATUS_OK)
        {
            const std::string identifier = status_identifier(status);
            const std::string message = status_message(status);
            if (path != nullptr)
            {
                copp_path_free(path);
            }
            throw_error(identifier, message);
        }

        outputs[0] = factory_.createScalar(store_handle(HandleKind::Path, path));
    }

    void path_from_evaluator_2nd(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 5, "path_from_evaluator_2nd");
        require_output_count_at_least(outputs, 1, "path_from_evaluator_2nd");
        require_output_count_at_most(outputs, 1, "path_from_evaluator_2nd");

        const std::size_t dim = copy_size_scalar(inputs[2], "dim");
        if (dim == 0)
        {
            throw_error("copp:InvalidArgument", "dim must be positive.");
        }
        const double s_min = copy_real_scalar(inputs[3], "s_min");
        const double s_max = copy_real_scalar(inputs[4], "s_max");

        auto context = std::make_unique<PathCallbackContext>(
            this,
            inputs[1],
            inputs[1],
            true,
            false);

        CoppPath *path = nullptr;
        const CoppStatus status = copp_path_from_evaluator_2nd(
            dim,
            s_min,
            s_max,
            matlab_path_evaluator_2nd_callback,
            context.get(),
            &path);
        if (status != COPP_STATUS_OK)
        {
            if (path != nullptr)
            {
                copp_path_free(path);
            }
            throw_status(status);
        }

        outputs[0] = factory_.createScalar(store_handle(HandleKind::Path, path, std::move(context)));
    }

    void path_from_evaluator_3rd(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 7, "path_from_evaluator_3rd");
        require_output_count_at_least(outputs, 1, "path_from_evaluator_3rd");
        require_output_count_at_most(outputs, 1, "path_from_evaluator_3rd");

        const bool has_evaluator_2nd = copy_size_scalar(inputs[3], "has_evaluator_2nd") != 0;
        const std::size_t dim = copy_size_scalar(inputs[4], "dim");
        if (dim == 0)
        {
            throw_error("copp:InvalidArgument", "dim must be positive.");
        }
        const double s_min = copy_real_scalar(inputs[5], "s_min");
        const double s_max = copy_real_scalar(inputs[6], "s_max");

        auto context = std::make_unique<PathCallbackContext>(
            this,
            has_evaluator_2nd ? inputs[2] : inputs[1],
            inputs[1],
            has_evaluator_2nd,
            true);

        CoppPath *path = nullptr;
        const CoppStatus status = copp_path_from_evaluator_3rd(
            dim,
            s_min,
            s_max,
            has_evaluator_2nd ? matlab_path_evaluator_2nd_callback : nullptr,
            matlab_path_evaluator_3rd_callback,
            context.get(),
            &path);
        if (status != COPP_STATUS_OK)
        {
            if (path != nullptr)
            {
                copp_path_free(path);
            }
            throw_status(status);
        }

        outputs[0] = factory_.createScalar(store_handle(HandleKind::Path, path, std::move(context)));
    }

    void path_dim(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 2, "path_dim");
        require_output_count_at_least(outputs, 1, "path_dim");
        require_output_count_at_most(outputs, 1, "path_dim");

        const auto id = copy_handle_id(inputs[1], "handle_id");
        const auto *path = static_cast<const CoppPath *>(checked_handle(id, HandleKind::Path));
        std::size_t dim = 0;
        const CoppStatus status = copp_path_dim(path, &dim);
        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }
        outputs[0] = factory_.createScalar(static_cast<double>(dim));
    }

    void path_s_range(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 2, "path_s_range");
        require_output_count_at_least(outputs, 1, "path_s_range");
        require_output_count_at_most(outputs, 1, "path_s_range");

        const auto id = copy_handle_id(inputs[1], "handle_id");
        const auto *path = static_cast<const CoppPath *>(checked_handle(id, HandleKind::Path));
        double s_min = 0.0;
        double s_max = 0.0;
        const CoppStatus status = copp_path_s_range(path, &s_min, &s_max);
        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }
        const double values[2] = {s_min, s_max};
        outputs[0] = factory_.createArray<double>({1, 2}, values, values + 2);
    }

    void path_evaluate_up_to_2nd(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 3, "path_evaluate_up_to_2nd");
        require_output_count_at_least(outputs, 3, "path_evaluate_up_to_2nd");
        require_output_count_at_most(outputs, 3, "path_evaluate_up_to_2nd");

        const auto id = copy_handle_id(inputs[1], "handle_id");
        const auto *path = static_cast<const CoppPath *>(checked_handle(id, HandleKind::Path));
        const auto s = copy_real_vector(inputs[2], "s");

        CoppMatrixGuard q;
        CoppMatrixGuard dq;
        CoppMatrixGuard ddq;
        const CoppStatus status = copp_path_evaluate_up_to_2nd(
            path,
            CoppSliceF64{slice_data(s), s.size()},
            &q.value,
            &dq.value,
            &ddq.value);
        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }

        outputs[0] = matrix_output(q.value);
        outputs[1] = matrix_output(dq.value);
        outputs[2] = matrix_output(ddq.value);
    }

    void path_evaluate_up_to_3rd(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 3, "path_evaluate_up_to_3rd");
        require_output_count_at_least(outputs, 4, "path_evaluate_up_to_3rd");
        require_output_count_at_most(outputs, 4, "path_evaluate_up_to_3rd");

        const auto id = copy_handle_id(inputs[1], "handle_id");
        const auto *path = static_cast<const CoppPath *>(checked_handle(id, HandleKind::Path));
        const auto s = copy_real_vector(inputs[2], "s");

        CoppMatrixGuard q;
        CoppMatrixGuard dq;
        CoppMatrixGuard ddq;
        CoppMatrixGuard dddq;
        const CoppStatus status = copp_path_evaluate_up_to_3rd(
            path,
            CoppSliceF64{slice_data(s), s.size()},
            &q.value,
            &dq.value,
            &ddq.value,
            &dddq.value);
        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }

        outputs[0] = matrix_output(q.value);
        outputs[1] = matrix_output(dq.value);
        outputs[2] = matrix_output(ddq.value);
        outputs[3] = matrix_output(dddq.value);
    }

    void robot_create(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 3, "robot_create");
        require_output_count_at_least(outputs, 1, "robot_create");
        require_output_count_at_most(outputs, 1, "robot_create");

        const std::size_t dim = copy_size_scalar(inputs[1], "dim");
        const std::size_t capacity = copy_size_scalar(inputs[2], "capacity");
        if (dim == 0)
        {
            throw_error("copp:InvalidArgument", "dim must be positive.");
        }

        CoppRobot *robot = nullptr;
        const CoppStatus status = copp_robot_create(dim, capacity, &robot);
        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }
        outputs[0] = factory_.createScalar(store_handle(HandleKind::Robot, robot));
    }

    void robot_dim(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 2, "robot_dim");
        require_output_count_at_least(outputs, 1, "robot_dim");
        require_output_count_at_most(outputs, 1, "robot_dim");

        const auto id = copy_handle_id(inputs[1], "handle_id");
        const auto *robot = static_cast<const CoppRobot *>(checked_handle(id, HandleKind::Robot));
        std::size_t dim = 0;
        const CoppStatus status = copp_robot_dim(robot, &dim);
        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }
        outputs[0] = factory_.createScalar(static_cast<double>(dim));
    }

    void robot_len(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 2, "robot_len");
        require_output_count_at_least(outputs, 1, "robot_len");
        require_output_count_at_most(outputs, 1, "robot_len");

        const auto id = copy_handle_id(inputs[1], "handle_id");
        const auto *robot = static_cast<const CoppRobot *>(checked_handle(id, HandleKind::Robot));
        std::size_t len = 0;
        const CoppStatus status = copp_robot_len(robot, &len);
        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }
        outputs[0] = factory_.createScalar(static_cast<double>(len));
    }

    void robot_capacity(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 2, "robot_capacity");
        require_output_count_at_least(outputs, 1, "robot_capacity");
        require_output_count_at_most(outputs, 1, "robot_capacity");

        const auto id = copy_handle_id(inputs[1], "handle_id");
        const auto *robot = static_cast<const CoppRobot *>(checked_handle(id, HandleKind::Robot));
        std::size_t capacity = 0;
        const CoppStatus status = copp_robot_capacity(robot, &capacity);
        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }
        outputs[0] = factory_.createScalar(static_cast<double>(capacity));
    }

    void robot_append_s(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 3, "robot_append_s");
        require_output_count_at_most(outputs, 0, "robot_append_s");

        const auto id = copy_handle_id(inputs[1], "handle_id");
        auto *robot = static_cast<CoppRobot *>(checked_handle(id, HandleKind::Robot));
        const auto s = copy_real_vector(inputs[2], "s");
        const CoppStatus status = copp_robot_append_s(robot, CoppSliceF64{slice_data(s), s.size()});
        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }
    }

    void robot_set_q_2nd(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 6, "robot_set_q_2nd");
        require_output_count_at_most(outputs, 0, "robot_set_q_2nd");

        const auto id = copy_handle_id(inputs[1], "handle_id");
        auto *robot = static_cast<CoppRobot *>(checked_handle(id, HandleKind::Robot));
        const std::size_t idx_s = copy_size_scalar(inputs[2], "idx_s");
        const NumericMatrix q = copy_real_matrix(inputs[3], "q");
        const NumericMatrix dq = copy_real_matrix(inputs[4], "dq");
        const NumericMatrix ddq = copy_real_matrix(inputs[5], "ddq");
        if (q.rows != dq.rows || q.rows != ddq.rows || q.cols != dq.cols || q.cols != ddq.cols)
        {
            throw_error("copp:InvalidArgument", "q, dq, and ddq must have the same shape.");
        }

        const CoppStatus status = copp_robot_set_q_2nd(robot, idx_s, q.view(), dq.view(), ddq.view());
        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }
    }

    void robot_set_q_3rd(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 7, "robot_set_q_3rd");
        require_output_count_at_most(outputs, 0, "robot_set_q_3rd");

        const auto id = copy_handle_id(inputs[1], "handle_id");
        auto *robot = static_cast<CoppRobot *>(checked_handle(id, HandleKind::Robot));
        const std::size_t idx_s = copy_size_scalar(inputs[2], "idx_s");
        const NumericMatrix q = copy_real_matrix(inputs[3], "q");
        const NumericMatrix dq = copy_real_matrix(inputs[4], "dq");
        const NumericMatrix ddq = copy_real_matrix(inputs[5], "ddq");
        const NumericMatrix dddq = copy_real_matrix(inputs[6], "dddq");
        if (q.rows != dq.rows || q.rows != ddq.rows || q.rows != dddq.rows ||
            q.cols != dq.cols || q.cols != ddq.cols || q.cols != dddq.cols)
        {
            throw_error("copp:InvalidArgument", "q, dq, ddq, and dddq must have the same shape.");
        }

        const CoppStatus status = copp_robot_set_q_3rd(robot, idx_s, q.view(), dq.view(), ddq.view(), dddq.view());
        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }
    }

    void robot_sample_path_2nd(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 5, "robot_sample_path_2nd");
        require_output_count_at_most(outputs, 0, "robot_sample_path_2nd");

        const auto robot_id = copy_handle_id(inputs[1], "robot_id");
        auto *robot = static_cast<CoppRobot *>(checked_handle(robot_id, HandleKind::Robot));
        const auto path_id = copy_handle_id(inputs[2], "path_id");
        const auto *path = static_cast<const CoppPath *>(checked_handle(path_id, HandleKind::Path));
        const std::size_t idx_s_from = copy_size_scalar(inputs[3], "idx_s_from");
        const std::size_t idx_s_to = copy_size_scalar(inputs[4], "idx_s_to");

        const CoppStatus status = copp_robot_sample_path_2nd(robot, path, idx_s_from, idx_s_to);
        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }
    }

    void robot_sample_path_3rd(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 5, "robot_sample_path_3rd");
        require_output_count_at_most(outputs, 0, "robot_sample_path_3rd");

        const auto robot_id = copy_handle_id(inputs[1], "robot_id");
        auto *robot = static_cast<CoppRobot *>(checked_handle(robot_id, HandleKind::Robot));
        const auto path_id = copy_handle_id(inputs[2], "path_id");
        const auto *path = static_cast<const CoppPath *>(checked_handle(path_id, HandleKind::Path));
        const std::size_t idx_s_from = copy_size_scalar(inputs[3], "idx_s_from");
        const std::size_t idx_s_to = copy_size_scalar(inputs[4], "idx_s_to");

        const CoppStatus status = copp_robot_sample_path_3rd(robot, path, idx_s_from, idx_s_to);
        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }
    }

    void robot_add_limits(ArgumentList outputs, ArgumentList inputs, bool velocity)
    {
        require_input_count(inputs, 6, velocity ? "robot_add_velocity_limits" : "robot_add_acceleration_limits");
        require_output_count_at_most(outputs, 0, velocity ? "robot_add_velocity_limits" : "robot_add_acceleration_limits");

        const auto id = copy_handle_id(inputs[1], "handle_id");
        auto *robot = static_cast<CoppRobot *>(checked_handle(id, HandleKind::Robot));
        const std::size_t start_idx_s = copy_size_scalar(inputs[2], "start_idx_s");
        const std::size_t len = copy_size_scalar(inputs[3], "length");
        const bool upper_is_vector = is_vector_shape(inputs[4]);
        const bool lower_is_vector = is_vector_shape(inputs[5]);
        if (upper_is_vector != lower_is_vector)
        {
            throw_error("copp:InvalidArgument", "upper and lower limits must both be vectors or both be matrices.");
        }

        CoppStatus status = COPP_STATUS_OK;
        if (upper_is_vector)
        {
            const auto upper = copy_real_vector(inputs[4], "upper", ValuePolicy::AllowInfinity);
            const auto lower = copy_real_vector(inputs[5], "lower", ValuePolicy::AllowInfinity);
            if (velocity)
            {
                status = copp_add_axial_velocity_limits(
                    robot,
                    start_idx_s,
                    len,
                    CoppSliceF64{slice_data(upper), upper.size()},
                    CoppSliceF64{slice_data(lower), lower.size()});
            }
            else
            {
                status = copp_add_axial_acceleration_limits(
                    robot,
                    start_idx_s,
                    len,
                    CoppSliceF64{slice_data(upper), upper.size()},
                    CoppSliceF64{slice_data(lower), lower.size()});
            }
        }
        else
        {
            const NumericMatrix upper = copy_real_matrix(inputs[4], "upper", ValuePolicy::AllowInfinity);
            const NumericMatrix lower = copy_real_matrix(inputs[5], "lower", ValuePolicy::AllowInfinity);
            if (upper.rows != lower.rows || upper.cols != lower.cols)
            {
                throw_error("copp:InvalidArgument", "upper and lower limit matrices must have the same shape.");
            }
            if (velocity)
            {
                status = copp_add_axial_velocity_limits_matrix(robot, start_idx_s, upper.view(), lower.view());
            }
            else
            {
                status = copp_add_axial_acceleration_limits_matrix(robot, start_idx_s, upper.view(), lower.view());
            }
        }

        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }
    }

    void robot_add_jerk_limits(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 6, "robot_add_jerk_limits");
        require_output_count_at_most(outputs, 0, "robot_add_jerk_limits");

        const auto id = copy_handle_id(inputs[1], "handle_id");
        auto *robot = static_cast<CoppRobot *>(checked_handle(id, HandleKind::Robot));
        const std::size_t start_idx_s = copy_size_scalar(inputs[2], "start_idx_s");
        const std::size_t len = copy_size_scalar(inputs[3], "length");
        const bool upper_is_vector = is_vector_shape(inputs[4]);
        const bool lower_is_vector = is_vector_shape(inputs[5]);
        if (upper_is_vector != lower_is_vector)
        {
            throw_error("copp:InvalidArgument", "upper and lower jerk limits must both be vectors or both be matrices.");
        }

        CoppStatus status = COPP_STATUS_OK;
        if (upper_is_vector)
        {
            const auto upper = copy_real_vector(inputs[4], "upper", ValuePolicy::AllowInfinity);
            const auto lower = copy_real_vector(inputs[5], "lower", ValuePolicy::AllowInfinity);
            status = copp_add_axial_jerk_limits(
                robot,
                start_idx_s,
                len,
                CoppSliceF64{slice_data(upper), upper.size()},
                CoppSliceF64{slice_data(lower), lower.size()});
        }
        else
        {
            const NumericMatrix upper = copy_real_matrix(inputs[4], "upper", ValuePolicy::AllowInfinity);
            const NumericMatrix lower = copy_real_matrix(inputs[5], "lower", ValuePolicy::AllowInfinity);
            if (upper.rows != lower.rows || upper.cols != lower.cols)
            {
                throw_error("copp:InvalidArgument", "upper and lower jerk limit matrices must have the same shape.");
            }
            status = copp_add_axial_jerk_limits_matrix(robot, start_idx_s, upper.view(), lower.view());
        }

        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }
    }

    void robot_add_torque_limits(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 6, "robot_add_torque_limits");
        require_output_count_at_most(outputs, 0, "robot_add_torque_limits");

        const auto id = copy_handle_id(inputs[1], "handle_id");
        auto *robot = static_cast<CoppRobot *>(checked_handle(id, HandleKind::Robot));
        const std::size_t start_idx_s = copy_size_scalar(inputs[2], "start_idx_s");
        const std::size_t len = copy_size_scalar(inputs[3], "length");
        const bool upper_is_vector = is_vector_shape(inputs[4]);
        const bool lower_is_vector = is_vector_shape(inputs[5]);
        if (upper_is_vector != lower_is_vector)
        {
            throw_error("copp:InvalidArgument", "upper and lower torque limits must both be vectors or both be matrices.");
        }

        CoppStatus status = COPP_STATUS_OK;
        if (upper_is_vector)
        {
            const auto upper = copy_real_vector(inputs[4], "upper", ValuePolicy::AllowInfinity);
            const auto lower = copy_real_vector(inputs[5], "lower", ValuePolicy::AllowInfinity);
            status = copp_add_axial_torque_limits(
                robot,
                start_idx_s,
                len,
                CoppSliceF64{slice_data(upper), upper.size()},
                CoppSliceF64{slice_data(lower), lower.size()});
        }
        else
        {
            const NumericMatrix upper = copy_real_matrix(inputs[4], "upper", ValuePolicy::AllowInfinity);
            const NumericMatrix lower = copy_real_matrix(inputs[5], "lower", ValuePolicy::AllowInfinity);
            if (upper.rows != lower.rows || upper.cols != lower.cols)
            {
                throw_error("copp:InvalidArgument", "upper and lower torque limit matrices must have the same shape.");
            }
            status = copp_add_axial_torque_limits_matrix(robot, start_idx_s, upper.view(), lower.view());
        }

        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }
    }

    void robot_add_raw_constraint_1st(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 4, "robot_add_raw_constraint_1st");
        require_output_count_at_most(outputs, 0, "robot_add_raw_constraint_1st");

        const auto id = copy_handle_id(inputs[1], "handle_id");
        auto *robot = static_cast<CoppRobot *>(checked_handle(id, HandleKind::Robot));
        const std::size_t idx_s = copy_size_scalar(inputs[2], "idx_s");
        const NumericMatrix amax = copy_real_matrix(inputs[3], "amax", ValuePolicy::AllowInfinity);

        const CoppStatus status = copp_add_raw_constraint_1st(robot, idx_s, amax.view());
        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }
    }

    void robot_add_raw_constraint_2nd(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 6, "robot_add_raw_constraint_2nd");
        require_output_count_at_most(outputs, 0, "robot_add_raw_constraint_2nd");

        const auto id = copy_handle_id(inputs[1], "handle_id");
        auto *robot = static_cast<CoppRobot *>(checked_handle(id, HandleKind::Robot));
        const std::size_t idx_s = copy_size_scalar(inputs[2], "idx_s");
        const NumericMatrix acc_a = copy_real_matrix(inputs[3], "acc_a");
        const NumericMatrix acc_b = copy_real_matrix(inputs[4], "acc_b");
        const NumericMatrix acc_max = copy_real_matrix(inputs[5], "acc_max", ValuePolicy::AllowInfinity);
        if (acc_a.rows != acc_b.rows || acc_a.cols != acc_b.cols ||
            acc_a.rows != acc_max.rows || acc_a.cols != acc_max.cols)
        {
            throw_error("copp:InvalidArgument", "acc_a, acc_b, and acc_max must have the same shape.");
        }

        const CoppStatus status = copp_add_raw_constraint_2nd(robot, idx_s, acc_a.view(), acc_b.view(), acc_max.view());
        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }
    }

    void robot_add_raw_constraint_3rd(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 8, "robot_add_raw_constraint_3rd");
        require_output_count_at_most(outputs, 0, "robot_add_raw_constraint_3rd");

        const auto id = copy_handle_id(inputs[1], "handle_id");
        auto *robot = static_cast<CoppRobot *>(checked_handle(id, HandleKind::Robot));
        const std::size_t idx_s = copy_size_scalar(inputs[2], "idx_s");
        const NumericMatrix jerk_a = copy_real_matrix(inputs[3], "jerk_a");
        const NumericMatrix jerk_b = copy_real_matrix(inputs[4], "jerk_b");
        const NumericMatrix jerk_c = copy_real_matrix(inputs[5], "jerk_c");
        const NumericMatrix jerk_d = copy_real_matrix(inputs[6], "jerk_d");
        const NumericMatrix jerk_max = copy_real_matrix(inputs[7], "jerk_max", ValuePolicy::AllowInfinity);
        if (jerk_a.rows != jerk_b.rows || jerk_a.cols != jerk_b.cols ||
            jerk_a.rows != jerk_c.rows || jerk_a.cols != jerk_c.cols ||
            jerk_a.rows != jerk_d.rows || jerk_a.cols != jerk_d.cols ||
            jerk_a.rows != jerk_max.rows || jerk_a.cols != jerk_max.cols)
        {
            throw_error("copp:InvalidArgument", "jerk_a, jerk_b, jerk_c, jerk_d, and jerk_max must have the same shape.");
        }

        const CoppStatus status = copp_add_raw_constraint_3rd(
            robot,
            idx_s,
            jerk_a.view(),
            jerk_b.view(),
            jerk_c.view(),
            jerk_d.view(),
            jerk_max.view());
        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }
    }

    void robot_clear_constraints(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 3, "robot_clear_constraints");
        require_output_count_at_most(outputs, 0, "robot_clear_constraints");

        const auto id = copy_handle_id(inputs[1], "handle_id");
        auto *robot = static_cast<CoppRobot *>(checked_handle(id, HandleKind::Robot));
        const bool keep_idx_s = copy_bool_scalar(inputs[2], "keep_idx_s");
        const CoppStatus status = copp_robot_clear_constraints(robot, keep_idx_s);
        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }
    }

    void robot_pop_n(ArgumentList outputs, ArgumentList inputs, bool front)
    {
        const char *name = front ? "robot_pop_front_n" : "robot_pop_back_n";
        require_input_count(inputs, 3, name);
        require_output_count_at_most(outputs, 0, name);

        const auto id = copy_handle_id(inputs[1], "handle_id");
        auto *robot = static_cast<CoppRobot *>(checked_handle(id, HandleKind::Robot));
        const std::size_t n = copy_size_scalar(inputs[2], "n");
        const CoppStatus status = front
                                      ? copp_robot_pop_front_n(robot, n)
                                      : copp_robot_pop_back_n(robot, n);
        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }
    }

    void robot_set_inverse_dynamics(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 3, "robot_set_inverse_dynamics");
        require_output_count_at_most(outputs, 0, "robot_set_inverse_dynamics");

        const auto id = copy_handle_id(inputs[1], "handle_id");
        auto *robot = static_cast<CoppRobot *>(checked_handle(id, HandleKind::Robot));
        HandleEntry *entry = find_entry(id);
        if (entry == nullptr)
        {
            throw_error("copp:InvalidHandle", "Native robot handle is not valid.");
        }

        auto context = std::make_unique<RobotDynamicsCallbackContext>(this, inputs[2]);
        const CoppStatus status = copp_robot_set_inverse_dynamics(
            robot,
            matlab_inverse_dynamics_callback,
            context.get());
        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }
        entry->dynamics_callback = std::move(context);
    }

    void robot_clear_inverse_dynamics(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 2, "robot_clear_inverse_dynamics");
        require_output_count_at_most(outputs, 0, "robot_clear_inverse_dynamics");

        const auto id = copy_handle_id(inputs[1], "handle_id");
        auto *robot = static_cast<CoppRobot *>(checked_handle(id, HandleKind::Robot));
        const CoppStatus status = copp_robot_clear_inverse_dynamics(robot);
        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }
        if (HandleEntry *entry = find_entry(id))
        {
            entry->dynamics_callback.reset();
        }
    }

    void topp2_ra_solve(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 10, "topp2_ra_solve");
        require_output_count_at_least(outputs, 1, "topp2_ra_solve");
        require_output_count_at_most(outputs, 1, "topp2_ra_solve");

        const auto id = copy_handle_id(inputs[1], "handle_id");
        const auto *robot = static_cast<const CoppRobot *>(checked_handle(id, HandleKind::Robot));
        const std::size_t idx_s_start = copy_size_scalar(inputs[2], "idx_s_start");
        const std::size_t idx_s_final = copy_size_scalar(inputs[3], "idx_s_final");
        const double a_start = copy_real_scalar(inputs[4], "a_start");
        const double a_final = copy_real_scalar(inputs[5], "a_final");
        const double lp_feas_tol = copy_real_scalar(inputs[6], "lp_feas_tol");
        const double a_cmp_abs_tol = copy_real_scalar(inputs[7], "a_cmp_abs_tol");
        const double a_cmp_rel_tol = copy_real_scalar(inputs[8], "a_cmp_rel_tol");
        const CoppVerbosity verbosity = parse_verbosity(inputs[9]);

        const Topp2Problem problem{robot, idx_s_start, idx_s_final, a_start, a_final};
        const Topp2RaOptions options{lp_feas_tol, a_cmp_abs_tol, a_cmp_rel_tol, verbosity};
        CoppVecGuard a;
        const CoppStatus status = topp2_ra(problem, options, &a.value);
        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }
        outputs[0] = column_vector_output(a.value);
    }

    void reach_set2(ArgumentList outputs, ArgumentList inputs, bool bidirectional)
    {
        const char *name = bidirectional ? "reach_set2_bidirectional" : "reach_set2_backward";
        require_input_count(inputs, 10, name);
        require_output_count_at_least(outputs, 2, name);
        require_output_count_at_most(outputs, 2, name);

        const auto id = copy_handle_id(inputs[1], "handle_id");
        const auto *robot = static_cast<const CoppRobot *>(checked_handle(id, HandleKind::Robot));
        const std::size_t idx_s_start = copy_size_scalar(inputs[2], "idx_s_start");
        const std::size_t idx_s_final = copy_size_scalar(inputs[3], "idx_s_final");
        const double a_start = copy_real_scalar(inputs[4], "a_start");
        const double a_final = copy_real_scalar(inputs[5], "a_final");
        const double lp_feas_tol = copy_real_scalar(inputs[6], "lp_feas_tol");
        const double a_cmp_abs_tol = copy_real_scalar(inputs[7], "a_cmp_abs_tol");
        const double a_cmp_rel_tol = copy_real_scalar(inputs[8], "a_cmp_rel_tol");
        const CoppVerbosity verbosity = parse_verbosity(inputs[9]);

        const Topp2Problem problem{robot, idx_s_start, idx_s_final, a_start, a_final};
        const Topp2RaOptions options{lp_feas_tol, a_cmp_abs_tol, a_cmp_rel_tol, verbosity};
        CoppReachSet2Guard reach;
        const CoppStatus status = bidirectional
                                      ? copp_reach_set2_bidirectional(problem, options, &reach.value)
                                      : copp_reach_set2_backward(problem, options, &reach.value);
        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }

        outputs[0] = column_vector_output(reach.value.a_max);
        outputs[1] = column_vector_output(reach.value.a_min);
    }

    void copp2_socp_solve(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 14 + kClarabelOptionsArgCount, "copp2_socp_solve");
        require_output_count_at_least(outputs, 1, "copp2_socp_solve");
        require_output_count_at_most(outputs, 1, "copp2_socp_solve");

        const auto id = copy_handle_id(inputs[1], "handle_id");
        const auto *robot = static_cast<const CoppRobot *>(checked_handle(id, HandleKind::Robot));
        const std::size_t idx_s_start = copy_size_scalar(inputs[2], "idx_s_start");
        const std::size_t idx_s_final = copy_size_scalar(inputs[3], "idx_s_final");
        const double a_start = copy_real_scalar(inputs[4], "a_start");
        const double a_final = copy_real_scalar(inputs[5], "a_final");

        ObjectiveStorage objectives = copy_objectives(
            inputs[6],
            inputs[7],
            inputs[8],
            inputs[9],
            inputs[10],
            inputs[11],
            inputs[12],
            inputs[13]);

        const CoppClarabelOptions options = copy_clarabel_options(inputs, 14);

        const Copp2Problem problem{
            robot,
            idx_s_start,
            idx_s_final,
            a_start,
            a_final,
            objectives.objectives.data(),
            objectives.objectives.size(),
        };

        CoppVecGuard a;
        const CoppStatus status = copp2_socp(problem, options, &a.value);
        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }
        outputs[0] = column_vector_output(a.value);
    }

    void copp2_socp_solve_expert(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 14 + kClarabelOptionsArgCount, "copp2_socp_solve_expert");
        require_output_count_at_least(outputs, 19, "copp2_socp_solve_expert");
        require_output_count_at_most(outputs, 19, "copp2_socp_solve_expert");

        const auto id = copy_handle_id(inputs[1], "handle_id");
        const auto *robot = static_cast<const CoppRobot *>(checked_handle(id, HandleKind::Robot));
        const std::size_t idx_s_start = copy_size_scalar(inputs[2], "idx_s_start");
        const std::size_t idx_s_final = copy_size_scalar(inputs[3], "idx_s_final");
        const double a_start = copy_real_scalar(inputs[4], "a_start");
        const double a_final = copy_real_scalar(inputs[5], "a_final");

        ObjectiveStorage objectives = copy_objectives(
            inputs[6],
            inputs[7],
            inputs[8],
            inputs[9],
            inputs[10],
            inputs[11],
            inputs[12],
            inputs[13]);

        const CoppClarabelOptions options = copy_clarabel_options(inputs, 14);

        const Copp2Problem problem{
            robot,
            idx_s_start,
            idx_s_final,
            a_start,
            a_final,
            objectives.objectives.data(),
            objectives.objectives.size(),
        };

        Copp2SocpResultGuard result;
        const CoppStatus status = copp2_socp_expert(problem, options, &result.value);
        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }

        outputs[0] = factory_.createScalar(result.value.has_a);
        outputs[1] = column_vector_output(result.value.a);
        outputs[2] = column_vector_output(result.value.x);
        outputs[3] = column_vector_output(result.value.z);
        outputs[4] = column_vector_output(result.value.s);
        outputs[5] = factory_.createScalar(static_cast<double>(result.value.solver_status));
        outputs[6] = factory_.createScalar(result.value.obj_val);
        outputs[7] = factory_.createScalar(result.value.obj_val_dual);
        outputs[8] = factory_.createScalar(result.value.solve_time);
        outputs[9] = factory_.createScalar(static_cast<double>(result.value.iterations));
        outputs[10] = factory_.createScalar(result.value.r_prim);
        outputs[11] = factory_.createScalar(result.value.r_dual);
        outputs[12] = factory_.createScalar(result.value.objective_value);
        outputs[13] = column_vector_output(result.value.objective_terms);
        outputs[14] = factory_.createScalar(static_cast<double>(result.value.linsolver.method));
        outputs[15] = factory_.createScalar(static_cast<double>(result.value.linsolver.threads));
        outputs[16] = factory_.createScalar(result.value.linsolver.direct);
        outputs[17] = factory_.createScalar(static_cast<double>(result.value.linsolver.nnz_a));
        outputs[18] = factory_.createScalar(static_cast<double>(result.value.linsolver.nnz_l));
    }

    Topp3Problem copy_topp3_problem(ArgumentList inputs, std::size_t offset, std::vector<double> &a_linearization)
    {
        const auto id = copy_handle_id(inputs[offset], "handle_id");
        auto *robot = static_cast<CoppRobot *>(checked_handle(id, HandleKind::Robot));
        const std::size_t idx_s_start = copy_size_scalar(inputs[offset + 1], "idx_s_start");
        a_linearization = copy_real_vector(inputs[offset + 2], "a_linearization");
        const double a_start = copy_real_scalar(inputs[offset + 3], "a_start");
        const double a_final = copy_real_scalar(inputs[offset + 4], "a_final");
        const double b_start = copy_real_scalar(inputs[offset + 5], "b_start");
        const double b_final = copy_real_scalar(inputs[offset + 6], "b_final");
        const std::size_t num_stationary_max_start = copy_size_scalar(inputs[offset + 7], "num_stationary_max_start");
        const std::size_t num_stationary_max_end = copy_size_scalar(inputs[offset + 8], "num_stationary_max_end");
        const double a_linearization_floor = copy_real_scalar(inputs[offset + 9], "a_linearization_floor");

        return Topp3Problem{
            robot,
            idx_s_start,
            CoppSliceF64{slice_data(a_linearization), a_linearization.size()},
            a_start,
            a_final,
            b_start,
            b_final,
            num_stationary_max_start,
            num_stationary_max_end,
            a_linearization_floor,
        };
    }

    Copp3Problem copy_copp3_problem(ArgumentList inputs,
                                   std::size_t offset,
                                   std::vector<double> &a_linearization,
                                   ObjectiveStorage &objectives)
    {
        const Topp3Problem topp3 = copy_topp3_problem(inputs, offset, a_linearization);
        objectives = copy_objectives(
            inputs[offset + 10],
            inputs[offset + 11],
            inputs[offset + 12],
            inputs[offset + 13],
            inputs[offset + 14],
            inputs[offset + 15],
            inputs[offset + 16],
            inputs[offset + 17]);

        return Copp3Problem{
            topp3.robot,
            topp3.idx_s_start,
            topp3.a_linearization,
            topp3.a_start,
            topp3.a_final,
            topp3.b_start,
            topp3.b_final,
            topp3.num_stationary_max_start,
            topp3.num_stationary_max_end,
            topp3.a_linearization_floor,
            objectives.objectives.data(),
            objectives.objectives.size(),
        };
    }

    CoppClarabelOptions copy_clarabel_options(ArgumentList inputs, std::size_t offset)
    {
        CoppClarabelOptions options;
        CoppStatus status = copp_clarabel_default_options(&options);
        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }
        options.verbosity = parse_verbosity(inputs[offset]);
        options.allow_almost_solved = copy_bool_scalar(inputs[offset + 1], "allow_almost_solved");
        options.allow_max_iterations = copy_bool_scalar(inputs[offset + 2], "allow_max_iterations");
        options.allow_max_time = copy_bool_scalar(inputs[offset + 3], "allow_max_time");
        options.allow_callback_terminated = copy_bool_scalar(inputs[offset + 4], "allow_callback_terminated");
        options.allow_insufficient_progress = copy_bool_scalar(inputs[offset + 5], "allow_insufficient_progress");
        CoppClarabelSettings &settings = options.clarabel_settings;
        const std::size_t s = offset + 6;
        settings.max_iter = copy_u32_scalar(inputs[s], "max_iter");
        settings.time_limit = copy_real_scalar(inputs[s + 1], "time_limit", ValuePolicy::AllowInfinity);
        settings.verbose = copy_bool_scalar(inputs[s + 2], "verbose");
        settings.max_step_fraction = copy_real_scalar(inputs[s + 3], "max_step_fraction");
        settings.tol_gap_abs = copy_real_scalar(inputs[s + 4], "tol_gap_abs");
        settings.tol_gap_rel = copy_real_scalar(inputs[s + 5], "tol_gap_rel");
        settings.tol_feas = copy_real_scalar(inputs[s + 6], "tol_feas");
        settings.tol_infeas_abs = copy_real_scalar(inputs[s + 7], "tol_infeas_abs");
        settings.tol_infeas_rel = copy_real_scalar(inputs[s + 8], "tol_infeas_rel");
        settings.tol_ktratio = copy_real_scalar(inputs[s + 9], "tol_ktratio");
        settings.reduced_tol_gap_abs = copy_real_scalar(inputs[s + 10], "reduced_tol_gap_abs");
        settings.reduced_tol_gap_rel = copy_real_scalar(inputs[s + 11], "reduced_tol_gap_rel");
        settings.reduced_tol_feas = copy_real_scalar(inputs[s + 12], "reduced_tol_feas");
        settings.reduced_tol_infeas_abs = copy_real_scalar(inputs[s + 13], "reduced_tol_infeas_abs");
        settings.reduced_tol_infeas_rel = copy_real_scalar(inputs[s + 14], "reduced_tol_infeas_rel");
        settings.reduced_tol_ktratio = copy_real_scalar(inputs[s + 15], "reduced_tol_ktratio");
        settings.equilibrate_enable = copy_bool_scalar(inputs[s + 16], "equilibrate_enable");
        settings.equilibrate_max_iter = copy_u32_scalar(inputs[s + 17], "equilibrate_max_iter");
        settings.equilibrate_min_scaling = copy_real_scalar(inputs[s + 18], "equilibrate_min_scaling");
        settings.equilibrate_max_scaling = copy_real_scalar(inputs[s + 19], "equilibrate_max_scaling");
        settings.linesearch_backtrack_step = copy_real_scalar(inputs[s + 20], "linesearch_backtrack_step");
        settings.min_switch_step_length = copy_real_scalar(inputs[s + 21], "min_switch_step_length");
        settings.min_terminate_step_length = copy_real_scalar(inputs[s + 22], "min_terminate_step_length");
        settings.max_threads = copy_u32_scalar(inputs[s + 23], "max_threads");
        settings.direct_kkt_solver = copy_bool_scalar(inputs[s + 24], "direct_kkt_solver");
        settings.direct_solve_method = parse_direct_solve_method(inputs[s + 25]);
        settings.static_regularization_enable = copy_bool_scalar(inputs[s + 26], "static_regularization_enable");
        settings.static_regularization_constant = copy_real_scalar(inputs[s + 27], "static_regularization_constant");
        settings.static_regularization_proportional = copy_real_scalar(inputs[s + 28], "static_regularization_proportional");
        settings.dynamic_regularization_enable = copy_bool_scalar(inputs[s + 29], "dynamic_regularization_enable");
        settings.dynamic_regularization_eps = copy_real_scalar(inputs[s + 30], "dynamic_regularization_eps");
        settings.dynamic_regularization_delta = copy_real_scalar(inputs[s + 31], "dynamic_regularization_delta");
        settings.iterative_refinement_enable = copy_bool_scalar(inputs[s + 32], "iterative_refinement_enable");
        settings.iterative_refinement_reltol = copy_real_scalar(inputs[s + 33], "iterative_refinement_reltol");
        settings.iterative_refinement_abstol = copy_real_scalar(inputs[s + 34], "iterative_refinement_abstol");
        settings.iterative_refinement_max_iter = copy_u32_scalar(inputs[s + 35], "iterative_refinement_max_iter");
        settings.iterative_refinement_stop_ratio = copy_real_scalar(inputs[s + 36], "iterative_refinement_stop_ratio");
        settings.presolve_enable = copy_bool_scalar(inputs[s + 37], "presolve_enable");
        settings.input_sparse_dropzeros = copy_bool_scalar(inputs[s + 38], "input_sparse_dropzeros");
        return options;
    }

    void copp3_socp_solve(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 19 + kClarabelOptionsArgCount, "copp3_socp_solve");
        require_output_count_at_least(outputs, 4, "copp3_socp_solve");
        require_output_count_at_most(outputs, 4, "copp3_socp_solve");

        std::vector<double> a_linearization;
        ObjectiveStorage objectives;
        const Copp3Problem problem = copy_copp3_problem(inputs, 1, a_linearization, objectives);
        const CoppClarabelOptions options = copy_clarabel_options(inputs, 19);

        CoppProfile3rdGuard profile;
        const CoppStatus status = copp3_socp(problem, options, &profile.value);
        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }

        write_profile3rd_outputs(outputs, 0, profile.value);
    }

    void copp3_socp_solve_expert(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 19 + kClarabelOptionsArgCount, "copp3_socp_solve_expert");
        require_output_count_at_least(outputs, 22, "copp3_socp_solve_expert");
        require_output_count_at_most(outputs, 22, "copp3_socp_solve_expert");

        std::vector<double> a_linearization;
        ObjectiveStorage objectives;
        const Copp3Problem problem = copy_copp3_problem(inputs, 1, a_linearization, objectives);
        const CoppClarabelOptions options = copy_clarabel_options(inputs, 19);

        Copp3SocpResultGuard result;
        const CoppStatus status = copp3_socp_expert(problem, options, &result.value);
        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }

        write_copp3_result_outputs(outputs, result.value);
    }

    void a_to_b_topp2(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 3, "a_to_b_topp2");
        require_output_count_at_least(outputs, 1, "a_to_b_topp2");
        require_output_count_at_most(outputs, 1, "a_to_b_topp2");

        const auto s = copy_real_vector(inputs[1], "s");
        const auto a = copy_real_vector(inputs[2], "a");
        if (s.size() != a.size())
        {
            throw_error("copp:InvalidArgument", "s and a must have the same number of elements.");
        }

        CoppVecGuard b;
        const CoppStatus status = copp_a_to_b_2nd(
            CoppSliceF64{slice_data(s), s.size()},
            CoppSliceF64{slice_data(a), a.size()},
            &b.value);
        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }
        outputs[0] = column_vector_output(b.value);
    }

    void s_to_t_topp2(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 4, "s_to_t_topp2");
        require_output_count_at_least(outputs, 1, "s_to_t_topp2");
        require_output_count_at_most(outputs, 2, "s_to_t_topp2");

        const auto s = copy_real_vector(inputs[1], "s");
        const auto a = copy_real_vector(inputs[2], "a");
        const double t0 = copy_real_scalar(inputs[3], "t0");

        if (s.size() != a.size())
        {
            throw_error("copp:InvalidArgument", "s and a must have the same number of elements.");
        }

        double t_final = 0.0;
        CoppVecGuard t_s;
        const CoppStatus status = copp_s_to_t_2nd(
            CoppSliceF64{slice_data(s), s.size()},
            CoppSliceF64{slice_data(a), a.size()},
            t0,
            &t_final,
            &t_s.value);
        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }

        outputs[0] = factory_.createScalar(t_final);
        if (outputs.size() >= 2)
        {
            outputs[1] = column_vector_output(t_s.value);
        }
    }

    void t_to_s_topp2_uniform(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 7, "t_to_s_topp2_uniform");
        require_output_count_at_least(outputs, 1, "t_to_s_topp2_uniform");
        require_output_count_at_most(outputs, 1, "t_to_s_topp2_uniform");

        const auto s = copy_real_vector(inputs[1], "s");
        const auto a = copy_real_vector(inputs[2], "a");
        const auto t_s = copy_real_vector(inputs[3], "t_s");
        const double t0 = copy_real_scalar(inputs[4], "t0");
        const double dt = copy_real_scalar(inputs[5], "dt");
        const bool include_final = copy_bool_scalar(inputs[6], "include_final");

        if (s.size() != a.size() || s.size() != t_s.size())
        {
            throw_error("copp:InvalidArgument", "s, a, and t_s must have the same number of elements.");
        }

        CoppVecGuard s_t;
        const CoppStatus status = copp_t_to_s_uniform_2nd(
            CoppSliceF64{slice_data(s), s.size()},
            CoppSliceF64{slice_data(a), a.size()},
            CoppSliceF64{slice_data(t_s), t_s.size()},
            t0,
            dt,
            include_final,
            &s_t.value);
        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }
        outputs[0] = column_vector_output(s_t.value);
    }

    void t_to_s_topp2_samples(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 5, "t_to_s_topp2_samples");
        require_output_count_at_least(outputs, 1, "t_to_s_topp2_samples");
        require_output_count_at_most(outputs, 1, "t_to_s_topp2_samples");

        const auto s = copy_real_vector(inputs[1], "s");
        const auto a = copy_real_vector(inputs[2], "a");
        const auto t_s = copy_real_vector(inputs[3], "t_s");
        const auto t_sample = copy_real_vector(inputs[4], "t_sample");

        if (s.size() != a.size() || s.size() != t_s.size())
        {
            throw_error("copp:InvalidArgument", "s, a, and t_s must have the same number of elements.");
        }

        CoppVecGuard s_t;
        const CoppStatus status = copp_t_to_s_non_uniform_2nd(
            CoppSliceF64{slice_data(s), s.size()},
            CoppSliceF64{slice_data(a), a.size()},
            CoppSliceF64{slice_data(t_s), t_s.size()},
            CoppSliceF64{slice_data(t_sample), t_sample.size()},
            &s_t.value);
        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }
        outputs[0] = column_vector_output(s_t.value);
    }

    void topp3_solve(ArgumentList outputs, ArgumentList inputs, bool socp)
    {
        const char *name = socp ? "topp3_socp_solve" : "topp3_lp_solve";
        require_input_count(inputs, 11 + kClarabelOptionsArgCount, name);
        require_output_count_at_least(outputs, 4, name);
        require_output_count_at_most(outputs, 4, name);

        const auto id = copy_handle_id(inputs[1], "handle_id");
        auto *robot = static_cast<CoppRobot *>(checked_handle(id, HandleKind::Robot));
        const std::size_t idx_s_start = copy_size_scalar(inputs[2], "idx_s_start");
        const auto a_linearization = copy_real_vector(inputs[3], "a_linearization");
        const double a_start = copy_real_scalar(inputs[4], "a_start");
        const double a_final = copy_real_scalar(inputs[5], "a_final");
        const double b_start = copy_real_scalar(inputs[6], "b_start");
        const double b_final = copy_real_scalar(inputs[7], "b_final");
        const std::size_t num_stationary_max_start = copy_size_scalar(inputs[8], "num_stationary_max_start");
        const std::size_t num_stationary_max_end = copy_size_scalar(inputs[9], "num_stationary_max_end");
        const double a_linearization_floor = copy_real_scalar(inputs[10], "a_linearization_floor");

        const CoppClarabelOptions options = copy_clarabel_options(inputs, 11);

        const Topp3Problem problem{
            robot,
            idx_s_start,
            CoppSliceF64{slice_data(a_linearization), a_linearization.size()},
            a_start,
            a_final,
            b_start,
            b_final,
            num_stationary_max_start,
            num_stationary_max_end,
            a_linearization_floor,
        };

        CoppProfile3rdGuard profile;
        const CoppStatus status = socp
                                      ? topp3_socp(problem, options, &profile.value)
                                      : topp3_lp(problem, options, &profile.value);
        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }

        write_profile3rd_outputs(outputs, 0, profile.value);
    }

    void topp3_solve_expert(ArgumentList outputs, ArgumentList inputs, bool socp)
    {
        const char *name = socp ? "topp3_socp_solve_expert" : "topp3_lp_solve_expert";
        require_input_count(inputs, 11 + kClarabelOptionsArgCount, name);
        require_output_count_at_least(outputs, 22, name);
        require_output_count_at_most(outputs, 22, name);

        std::vector<double> a_linearization;
        const Topp3Problem problem = copy_topp3_problem(inputs, 1, a_linearization);
        const CoppClarabelOptions options = copy_clarabel_options(inputs, 11);

        Copp3SocpResultGuard result;
        const CoppStatus status = socp
                                      ? topp3_socp_expert(problem, options, &result.value)
                                      : topp3_lp_expert(problem, options, &result.value);
        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }

        write_copp3_result_outputs(outputs, result.value);
    }

    void s_to_t_topp3(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 7, "s_to_t_topp3");
        require_output_count_at_least(outputs, 1, "s_to_t_topp3");
        require_output_count_at_most(outputs, 2, "s_to_t_topp3");

        const auto s = copy_real_vector(inputs[1], "s");
        const auto a = copy_real_vector(inputs[2], "a");
        const auto b = copy_real_vector(inputs[3], "b");
        const std::size_t num_stationary_start = copy_size_scalar(inputs[4], "num_stationary_start");
        const std::size_t num_stationary_end = copy_size_scalar(inputs[5], "num_stationary_end");
        const double t0 = copy_real_scalar(inputs[6], "t0");

        if (s.size() != a.size() || s.size() != b.size())
        {
            throw_error("copp:InvalidArgument", "s, profile.a, and profile.b must have the same number of elements.");
        }

        double t_final = 0.0;
        CoppVecGuard t_s;
        const CoppStatus status = copp_s_to_t_3rd(
            CoppSliceF64{slice_data(s), s.size()},
            CoppSliceF64{slice_data(a), a.size()},
            CoppSliceF64{slice_data(b), b.size()},
            num_stationary_start,
            num_stationary_end,
            t0,
            &t_final,
            &t_s.value);
        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }

        outputs[0] = factory_.createScalar(t_final);
        if (outputs.size() >= 2)
        {
            outputs[1] = column_vector_output(t_s.value);
        }
    }

    void t_to_s_topp3_uniform(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 10, "t_to_s_topp3_uniform");
        require_output_count_at_least(outputs, 1, "t_to_s_topp3_uniform");
        require_output_count_at_most(outputs, 1, "t_to_s_topp3_uniform");

        const auto s = copy_real_vector(inputs[1], "s");
        const auto a = copy_real_vector(inputs[2], "a");
        const auto b = copy_real_vector(inputs[3], "b");
        const std::size_t num_stationary_start = copy_size_scalar(inputs[4], "num_stationary_start");
        const std::size_t num_stationary_end = copy_size_scalar(inputs[5], "num_stationary_end");
        const auto t_s = copy_real_vector(inputs[6], "t_s");
        const double t0 = copy_real_scalar(inputs[7], "t0");
        const double dt = copy_real_scalar(inputs[8], "dt");
        const bool include_final = copy_bool_scalar(inputs[9], "include_final");

        if (s.size() != a.size() || s.size() != b.size() || s.size() != t_s.size())
        {
            throw_error("copp:InvalidArgument", "s, profile.a, profile.b, and t_s must have the same number of elements.");
        }

        CoppVecGuard s_t;
        const CoppStatus status = copp_t_to_s_uniform_3rd(
            CoppSliceF64{slice_data(s), s.size()},
            CoppSliceF64{slice_data(a), a.size()},
            CoppSliceF64{slice_data(b), b.size()},
            num_stationary_start,
            num_stationary_end,
            CoppSliceF64{slice_data(t_s), t_s.size()},
            t0,
            dt,
            include_final,
            &s_t.value);
        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }
        outputs[0] = column_vector_output(s_t.value);
    }

    void t_to_s_topp3_samples(ArgumentList outputs, ArgumentList inputs)
    {
        require_input_count(inputs, 8, "t_to_s_topp3_samples");
        require_output_count_at_least(outputs, 1, "t_to_s_topp3_samples");
        require_output_count_at_most(outputs, 1, "t_to_s_topp3_samples");

        const auto s = copy_real_vector(inputs[1], "s");
        const auto a = copy_real_vector(inputs[2], "a");
        const auto b = copy_real_vector(inputs[3], "b");
        const std::size_t num_stationary_start = copy_size_scalar(inputs[4], "num_stationary_start");
        const std::size_t num_stationary_end = copy_size_scalar(inputs[5], "num_stationary_end");
        const auto t_s = copy_real_vector(inputs[6], "t_s");
        const auto t_sample = copy_real_vector(inputs[7], "t_sample");

        if (s.size() != a.size() || s.size() != b.size() || s.size() != t_s.size())
        {
            throw_error("copp:InvalidArgument", "s, profile.a, profile.b, and t_s must have the same number of elements.");
        }

        CoppVecGuard s_t;
        const CoppStatus status = copp_t_to_s_non_uniform_3rd(
            CoppSliceF64{slice_data(s), s.size()},
            CoppSliceF64{slice_data(a), a.size()},
            CoppSliceF64{slice_data(b), b.size()},
            num_stationary_start,
            num_stationary_end,
            CoppSliceF64{slice_data(t_s), t_s.size()},
            CoppSliceF64{slice_data(t_sample), t_sample.size()},
            &s_t.value);
        if (status != COPP_STATUS_OK)
        {
            throw_status(status);
        }
        outputs[0] = column_vector_output(s_t.value);
    }
};

extern "C" CoppStatus matlab_path_evaluator_2nd_callback(void *user_data,
                                                          std::size_t dim,
                                                          std::size_t n,
                                                          const double *s,
                                                          double *q,
                                                          double *dq,
                                                          double *ddq)
{
    auto *context = static_cast<PathCallbackContext *>(user_data);
    if (context == nullptr || context->owner == nullptr)
    {
        copp_set_last_error_message(
            COPP_STATUS_INVALID_ARGUMENT,
            "MATLAB second-order path evaluator callback context is null.");
        return COPP_STATUS_INVALID_ARGUMENT;
    }
    return context->owner->evaluate_callback_2nd(context, dim, n, s, q, dq, ddq);
}

extern "C" CoppStatus matlab_path_evaluator_3rd_callback(void *user_data,
                                                          std::size_t dim,
                                                          std::size_t n,
                                                          const double *s,
                                                          double *q,
                                                          double *dq,
                                                          double *ddq,
                                                          double *dddq)
{
    auto *context = static_cast<PathCallbackContext *>(user_data);
    if (context == nullptr || context->owner == nullptr)
    {
        copp_set_last_error_message(
            COPP_STATUS_INVALID_ARGUMENT,
            "MATLAB third-order path evaluator callback context is null.");
        return COPP_STATUS_INVALID_ARGUMENT;
    }
    return context->owner->evaluate_callback_3rd(context, dim, n, s, q, dq, ddq, dddq);
}

extern "C" CoppStatus matlab_inverse_dynamics_callback(void *user_data,
                                                       std::size_t dim,
                                                       const double *q,
                                                       const double *dq,
                                                       const double *ddq,
                                                       double *tau)
{
    auto *context = static_cast<RobotDynamicsCallbackContext *>(user_data);
    if (context == nullptr || context->owner == nullptr)
    {
        copp_set_last_error_message(
            COPP_STATUS_INVALID_ARGUMENT,
            "MATLAB inverse-dynamics callback context is null.");
        return COPP_STATUS_INVALID_ARGUMENT;
    }
    return context->owner->evaluate_inverse_dynamics_callback(context, dim, q, dq, ddq, tau);
}
