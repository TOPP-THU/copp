#include "copp/path.hpp"

// Path facade implementation.
//
// This file is the C++ side of the private cxx bridge. Public headers expose
// `copp::Path` and callback types; this translation unit validates C++ views,
// converts them into bridge slices/descriptors, and copies Rust path evaluation
// results back into owned `copp::Matrix` objects.
//
// The key ownership rule is simple: user input buffers only need to live for the
// duration of the public call, while returned `Path` objects own their Rust-side
// path handles through RAII.

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "rust/cxx.h"
#include "copp/detail/path_bridge.hpp"
#include "copp/src/ffi/cpp/mod.rs.h"

namespace
{

    rust::Slice<const double> to_rust_slice(copp::Span<const double> values) noexcept
    {
        return rust::Slice<const double>(values.data(), values.size());
    }

    rust::Slice<const double> to_rust_slice(copp::MatrixView values) noexcept
    {
        return rust::Slice<const double>(values.data(), values.storage_size());
    }

    copp::MatrixView optional_state_view(const std::optional<copp::Matrix> &matrix) noexcept
    {
        if (!matrix)
        {
            return {};
        }
        return matrix->view();
    }

    std::string to_std_string(const rust::Error &error)
    {
        return std::string(error.what());
    }

    uint8_t to_bridge(copp::OutOfRangeMode mode)
    {
        switch (mode)
        {
        case copp::OutOfRangeMode::Error:
            return 0;
        case copp::OutOfRangeMode::Clamp:
            return 1;
        }
        throw copp::Error(copp::Status::invalid_input, "unsupported OutOfRangeMode");
    }

    copp::Matrix copy_to_column_major(copp::MatrixView view)
    {
        copp::Matrix matrix(view.rows(), view.cols());
        if (view.layout() == copp::MatrixLayout::ColumnMajor)
        {
            for (std::size_t col = 0; col < view.cols(); ++col)
            {
                const double *src = view.data() + col * view.leading_dim();
                for (std::size_t row = 0; row < view.rows(); ++row)
                {
                    matrix(row, col) = src[row];
                }
            }
            return matrix;
        }

        for (std::size_t row = 0; row < view.rows(); ++row)
        {
            const double *src = view.data() + row * view.leading_dim();
            for (std::size_t col = 0; col < view.cols(); ++col)
            {
                matrix(row, col) = src[col];
            }
        }
        return matrix;
    }

    void check_matrix_view(copp::MatrixView view, const char *name)
    {
        if (view.rows() == 0 || view.cols() == 0)
        {
            throw copp::Error(copp::Status::invalid_input, std::string(name) + " must be non-empty");
        }
        if (view.data() == nullptr && view.storage_size() != 0)
        {
            throw copp::Error(copp::Status::invalid_input, std::string(name) + " data is null");
        }
        if (view.layout() == copp::MatrixLayout::ColumnMajor && view.leading_dim() < view.rows())
        {
            throw copp::Error(
                copp::Status::invalid_input,
                std::string(name) + " leading dimension is smaller than rows");
        }
        if (view.layout() == copp::MatrixLayout::RowMajor && view.leading_dim() < view.cols())
        {
            throw copp::Error(
                copp::Status::invalid_input,
                std::string(name) + " leading dimension is smaller than cols");
        }
    }

    void check_s_range(double s_min, double s_max, const char *name)
    {
        if (!std::isfinite(s_min) || !std::isfinite(s_max) || s_max <= s_min)
        {
            throw copp::Error(
                copp::Status::invalid_input,
                std::string(name) + " requires finite s_min < s_max");
        }
    }

    copp::Matrix make_matrix(std::size_t rows, std::size_t cols, rust::Slice<const double> data)
    {
        return copp::Matrix(rows, cols, std::vector<double>(data.begin(), data.end()));
    }

} // namespace

namespace copp
{

    namespace bridge
    {

        CppPathEvaluator2nd::CppPathEvaluator2nd(
            std::size_t dim,
            copp::PathEvaluator2nd evaluator)
            : dim_(dim), evaluator_(std::move(evaluator))
        {
            if (dim_ == 0)
            {
                throw copp::Error(copp::Status::invalid_input, "path evaluator dimension must be positive");
            }
            if (!evaluator_)
            {
                throw copp::Error(copp::Status::invalid_input, "path evaluator callback is empty");
            }
        }

        std::size_t CppPathEvaluator2nd::dim() const noexcept
        {
            return dim_;
        }

        void CppPathEvaluator2nd::evaluate_up_to_2nd(
            rust::Slice<const double> s,
            rust::Slice<double> q,
            rust::Slice<double> dq,
            rust::Slice<double> ddq) const
        {
            const std::size_t expected_len = dim_ * s.size();
            if (q.size() != expected_len || dq.size() != expected_len || ddq.size() != expected_len)
            {
                throw copp::Error(copp::Status::invalid_input, "path evaluator output buffer shape mismatch");
            }

            std::lock_guard<std::mutex> lock(mutex_);
            evaluator_(
                copp::Span<const double>(s.data(), s.size()),
                copp::MatrixRef::column_major(q.data(), dim_, s.size()),
                copp::MatrixRef::column_major(dq.data(), dim_, s.size()),
                copp::MatrixRef::column_major(ddq.data(), dim_, s.size()));
        }

        CppPathEvaluator3rd::CppPathEvaluator3rd(
            std::size_t dim,
            copp::PathEvaluator3rd evaluator)
            : dim_(dim), evaluator_(std::move(evaluator))
        {
            if (dim_ == 0)
            {
                throw copp::Error(copp::Status::invalid_input, "path evaluator dimension must be positive");
            }
            if (!evaluator_)
            {
                throw copp::Error(copp::Status::invalid_input, "path evaluator callback is empty");
            }
        }

        std::size_t CppPathEvaluator3rd::dim() const noexcept
        {
            return dim_;
        }

        void CppPathEvaluator3rd::evaluate_up_to_2nd(
            rust::Slice<const double> s,
            rust::Slice<double> q,
            rust::Slice<double> dq,
            rust::Slice<double> ddq) const
        {
            const std::size_t expected_len = dim_ * s.size();
            if (q.size() != expected_len || dq.size() != expected_len || ddq.size() != expected_len)
            {
                throw copp::Error(copp::Status::invalid_input, "path evaluator output buffer shape mismatch");
            }

            std::vector<double> dddq(expected_len);
            std::lock_guard<std::mutex> lock(mutex_);
            evaluator_(
                copp::Span<const double>(s.data(), s.size()),
                copp::MatrixRef::column_major(q.data(), dim_, s.size()),
                copp::MatrixRef::column_major(dq.data(), dim_, s.size()),
                copp::MatrixRef::column_major(ddq.data(), dim_, s.size()),
                copp::MatrixRef::column_major(dddq.data(), dim_, s.size()));
        }

        void CppPathEvaluator3rd::evaluate_up_to_3rd(
            rust::Slice<const double> s,
            rust::Slice<double> q,
            rust::Slice<double> dq,
            rust::Slice<double> ddq,
            rust::Slice<double> dddq) const
        {
            const std::size_t expected_len = dim_ * s.size();
            if (q.size() != expected_len || dq.size() != expected_len || ddq.size() != expected_len ||
                dddq.size() != expected_len)
            {
                throw copp::Error(copp::Status::invalid_input, "path evaluator output buffer shape mismatch");
            }

            std::lock_guard<std::mutex> lock(mutex_);
            evaluator_(
                copp::Span<const double>(s.data(), s.size()),
                copp::MatrixRef::column_major(q.data(), dim_, s.size()),
                copp::MatrixRef::column_major(dq.data(), dim_, s.size()),
                copp::MatrixRef::column_major(ddq.data(), dim_, s.size()),
                copp::MatrixRef::column_major(dddq.data(), dim_, s.size()));
        }

    } // namespace bridge

    struct Path::Impl
    {
        explicit Impl(rust::Box<bridge::PathHandle> &&path) : path(std::move(path)) {}

        rust::Box<bridge::PathHandle> path;
    };

    Path::Path(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

    Path::Path(Path &&) noexcept = default;

    Path &Path::operator=(Path &&) noexcept = default;

    Path::~Path() = default;

    namespace detail
    {

        const void *path_handle(const Path &path) noexcept
        {
            if (!path.impl_)
            {
                return nullptr;
            }
            return &*path.impl_->path;
        }

    } // namespace detail

    Path Path::from_waypoints(
        std::initializer_list<std::initializer_list<double>> waypoints,
        SplineConfig config)
    {
        auto matrix = Matrix::from_columns(waypoints);
        return Path::from_waypoints(matrix.view(), config);
    }

    Path Path::from_parametric(
        PathParametric parametric,
        double s_min,
        double s_max)
    {
        if (!parametric)
        {
            throw Error(Status::invalid_input, "parametric path callback is empty");
        }
        check_s_range(s_min, s_max, "Path::from_parametric");

        const double s_probe = 0.5 * (s_min + s_max);
        std::vector<Jet3> probe;
        try
        {
            probe = parametric(Jet3::constant(s_probe));
        }
        catch (const Error &)
        {
            throw;
        }
        catch (const std::exception &error)
        {
            throw Error(
                Status::invalid_input,
                std::string("parametric path callback failed during dimension probe: ") + error.what());
        }
        const auto dim = probe.size();
        if (dim == 0)
        {
            throw Error(Status::invalid_input, "parametric path dimension must be positive");
        }

        return Path::from_parametric(
            dim,
            s_min,
            s_max,
            [parametric = std::move(parametric), dim](Jet3 s, Span<Jet3> q) {
                const auto values = parametric(s);
                if (values.size() != dim)
                {
                    throw Error(Status::invalid_input, "parametric path callback returned inconsistent dimension");
                }
                for (std::size_t i = 0; i < dim; ++i)
                {
                    q[i] = values[i];
                }
            });
    }

    Path Path::from_parametric(
        std::size_t dim,
        double s_min,
        double s_max,
        PathParametricWriter parametric)
    {
        if (dim == 0)
        {
            throw Error(Status::invalid_input, "parametric path dimension must be positive");
        }
        if (!parametric)
        {
            throw Error(Status::invalid_input, "parametric path callback is empty");
        }
        check_s_range(s_min, s_max, "Path::from_parametric");

        return Path::from_evaluator_3rd(
            dim,
            s_min,
            s_max,
            [dim, parametric = std::move(parametric)](Span<const double> samples,
                                                      MatrixRef q,
                                                      MatrixRef dq,
                                                      MatrixRef ddq,
                                                      MatrixRef dddq) {
                std::vector<Jet3> values(dim);
                for (std::size_t j = 0; j < samples.size(); ++j)
                {
                    parametric(Jet3::seed(samples[j]), Span<Jet3>(values));
                    for (std::size_t i = 0; i < dim; ++i)
                    {
                        const auto value = values[i];
                        q(i, j) = value.v;
                        dq(i, j) = value.d1;
                        ddq(i, j) = value.d2;
                        dddq(i, j) = value.d3;
                    }
                }
            });
    }

    Path Path::from_evaluator_2nd(
        std::size_t dim,
        double s_min,
        double s_max,
        PathEvaluator2nd evaluator)
    {
        try
        {
            auto handle = bridge::path_from_evaluator_2nd(
                std::make_unique<bridge::CppPathEvaluator2nd>(dim, std::move(evaluator)),
                s_min,
                s_max);
            return Path(std::make_unique<Impl>(std::move(handle)));
        }
        catch (const rust::Error &error)
        {
            throw Error(Status::invalid_input, to_std_string(error));
        }
    }

    Path Path::from_evaluator_3rd(
        std::size_t dim,
        double s_min,
        double s_max,
        PathEvaluator3rd evaluator)
    {
        try
        {
            auto handle = bridge::path_from_evaluator_3rd(
                std::make_unique<bridge::CppPathEvaluator3rd>(dim, std::move(evaluator)),
                s_min,
                s_max);
            return Path(std::make_unique<Impl>(std::move(handle)));
        }
        catch (const rust::Error &error)
        {
            throw Error(Status::invalid_input, to_std_string(error));
        }
    }

    Path Path::from_waypoints(MatrixView waypoints, SplineConfig config)
    {
        check_matrix_view(waypoints, "waypoints");

        Matrix owned;
        MatrixView bridge_view = waypoints;
        if (waypoints.layout() == MatrixLayout::RowMajor)
        {
            owned = copy_to_column_major(waypoints);
            bridge_view = owned.view();
        }

        try
        {
            const auto start_state = optional_state_view(config.start_state);
            const auto end_state = optional_state_view(config.end_state);
            auto handle = bridge::path_from_waypoints(
                to_rust_slice(bridge_view),
                bridge_view.rows(),
                bridge_view.cols(),
                bridge_view.leading_dim(),
                config.order,
                config.s_min,
                config.s_max,
                to_bridge(config.out_of_range),
                to_rust_slice(start_state),
                start_state.rows(),
                start_state.cols(),
                start_state.leading_dim(),
                to_rust_slice(end_state),
                end_state.rows(),
                end_state.cols(),
                end_state.leading_dim());
            return Path(std::make_unique<Impl>(std::move(handle)));
        }
        catch (const rust::Error &error)
        {
            throw Error(Status::invalid_input, to_std_string(error));
        }
    }

    std::size_t Path::dim() const
    {
        return impl_->path->dim();
    }

    std::pair<double, double> Path::s_range() const
    {
        return {impl_->path->s_min(), impl_->path->s_max()};
    }

    PathDerivatives Path::evaluate_q(Span<const double> s) const
    {
        try
        {
            auto result = impl_->path->evaluate_q(to_rust_slice(s));
            return PathDerivatives{
                make_matrix(dim(), s.size(), result->q()),
                std::nullopt,
                std::nullopt,
                std::nullopt,
            };
        }
        catch (const rust::Error &error)
        {
            throw Error(Status::invalid_input, to_std_string(error));
        }
    }

    PathDerivatives Path::evaluate_up_to_2nd(Span<const double> s) const
    {
        try
        {
            auto result = impl_->path->evaluate_up_to_2nd(to_rust_slice(s));
            const auto rows = dim();
            const auto cols = s.size();
            return PathDerivatives{
                make_matrix(rows, cols, result->q()),
                make_matrix(rows, cols, result->dq()),
                make_matrix(rows, cols, result->ddq()),
                std::nullopt,
            };
        }
        catch (const rust::Error &error)
        {
            throw Error(Status::invalid_input, to_std_string(error));
        }
    }

    PathDerivatives Path::evaluate_up_to_3rd(Span<const double> s) const
    {
        try
        {
            auto result = impl_->path->evaluate_up_to_3rd(to_rust_slice(s));
            const auto rows = dim();
            const auto cols = s.size();
            return PathDerivatives{
                make_matrix(rows, cols, result->q()),
                make_matrix(rows, cols, result->dq()),
                make_matrix(rows, cols, result->ddq()),
                make_matrix(rows, cols, result->dddq()),
            };
        }
        catch (const rust::Error &error)
        {
            throw Error(Status::invalid_input, to_std_string(error));
        }
    }

} // namespace copp
