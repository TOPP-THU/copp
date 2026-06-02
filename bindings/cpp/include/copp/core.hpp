#pragma once

#include <cstddef>
#include <initializer_list>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#if defined(_WIN32)
#if defined(COPP_CPP_BUILDING_LIBRARY)
#define COPP_API __declspec(dllexport)
#elif defined(COPP_CPP_SHARED) && COPP_CPP_SHARED
#define COPP_API __declspec(dllimport)
#else
#define COPP_API
#endif
#elif defined(__GNUC__) || defined(__clang__)
#define COPP_API __attribute__((visibility("default")))
#else
#define COPP_API
#endif

namespace copp
{

    /// Coarse error category reported by the C++ facade.
    ///
    /// The C++ layer currently converts Rust bridge failures into `Error` with a
    /// copied diagnostic message. `invalid_input` is used for user-facing argument
    /// and model errors; `bridge_error` is reserved for lower-level bridge failures.
    enum class Status
    {
        ok = 0,
        invalid_input,
        bridge_error,
    };

    /// Diagnostic verbosity for solver algorithms.
    ///
    /// Values mirror Rust `copp::diag::Verbosity`. `Silent` is the default
    /// and emits no diagnostic output. Higher levels progressively add summary,
    /// debug, and trace messages from the Rust solver core.
    enum class Verbosity
    {
        Silent = 0,
        Summary = 1,
        Debug = 2,
        Trace = 3,
    };

    /// Return the Rust crate version backing this C++ facade.
    ///
    /// @return A semantic-version string copied from the Rust crate metadata.
    ///
    /// @code
    /// std::cout << "COPP version: " << copp::version() << "\n";
    /// @endcode
    COPP_API std::string version();

    /// Closed station-index interval used by TOPP/COPP problem descriptors.
    ///
    /// The interval is inclusive: `idx_s_start <= k <= idx_s_final`.
    struct IndexInterval
    {
        std::size_t idx_s_start = 0;
        std::size_t idx_s_final = 0;
    };

    /// Second-order endpoint boundary values.
    ///
    /// `a = (ds/dt)^2`; the pair encodes `a(idx_s_start)` and
    /// `a(idx_s_final)`.
    struct Boundary2
    {
        double a_start = 0.0;
        double a_final = 0.0;
    };

    /// Third-order endpoint boundary values.
    ///
    /// `a = (ds/dt)^2` and `b = d^2s/dt^2`. The four fields encode endpoint
    /// values at `idx_s_start` and the inferred final station.
    struct Boundary3
    {
        double a_start = 0.0;
        double a_final = 0.0;
        double b_start = 0.0;
        double b_final = 0.0;
    };

    /// Exception type thrown by the default C++ API.
    ///
    /// The message returned by `what()` is owned by this object and remains valid
    /// for the lifetime of the exception. This is important because the bridge may
    /// produce temporary Rust/C++ error objects internally.
    class Error : public std::runtime_error
    {
    public:
        Error(Status status, std::string message)
            : std::runtime_error(std::move(message)), status_(status) {}

        Status status() const noexcept { return status_; }

    private:
        Status status_;
    };

    /// Memory layout for borrowed matrix views.
    ///
    /// COPP's Rust implementation and `Matrix` storage use column-major layout.
    /// Row-major views are accepted by selected public C++ functions and copied to
    /// column-major storage before crossing the Rust bridge.
    enum class MatrixLayout
    {
        ColumnMajor,
        RowMajor,
    };

    /// Non-owning view of a contiguous one-dimensional array.
    ///
    /// `Span<T>` is the C++17 compatibility equivalent of `std::span<T>`. It does
    /// not allocate, copy, or own memory. The referenced data must remain alive for
    /// the duration of the call that consumes the span.
    ///
    /// Typical inputs are `std::vector<double>`, C arrays, or pointer/length pairs:
    ///
    /// @code
    /// std::vector<double> s{0.0, 0.5, 1.0};
    /// copp::Span<const double> view{s};
    /// auto raw_view = copp::Span<const double>(s.data(), s.size());
    /// @endcode
    ///
    /// @tparam T Element type. Use `const double` for most public inputs.
    template <typename T>
    class Span
    {
    public:
        using element_type = T;
        using pointer = T *;
        using reference = T &;

        constexpr Span() noexcept = default;

        constexpr Span(pointer data, std::size_t size) noexcept : data_(data), size_(size) {}

        template <std::size_t N>
        constexpr Span(element_type (&data)[N]) noexcept : data_(data), size_(N) {}

        template <
            typename Container,
            typename = decltype(std::declval<Container &>().data()),
            typename = decltype(std::declval<Container &>().size())>
        Span(Container &container) noexcept
            : data_(container.data()), size_(static_cast<std::size_t>(container.size())) {}

        template <
            typename Container,
            typename = decltype(std::declval<const Container &>().data()),
            typename = decltype(std::declval<const Container &>().size()),
            typename U = element_type,
            typename = typename std::enable_if<std::is_const<U>::value>::type>
        Span(const Container &container) noexcept
            : data_(container.data()), size_(static_cast<std::size_t>(container.size())) {}

        constexpr pointer data() const noexcept { return data_; }
        constexpr std::size_t size() const noexcept { return size_; }
        constexpr bool empty() const noexcept { return size_ == 0; }

        reference operator[](std::size_t index) const noexcept { return data_[index]; }

    private:
        pointer data_ = nullptr;
        std::size_t size_ = 0;
    };

    /// Non-owning view of a two-dimensional `double` matrix.
    ///
    /// `MatrixView` is for input only. It stores pointer, shape, layout, and
    /// leading dimension. It never owns memory. For column-major data,
    /// `leading_dim` is the physical stride between two adjacent columns. For
    /// row-major data, `leading_dim` is the physical stride between two adjacent
    /// rows.
    ///
    /// The logical element `(row, col)` is interpreted as:
    ///
    /// - column-major: `data[row + col * leading_dim]`
    /// - row-major: `data[col + row * leading_dim]`
    ///
    /// The view only borrows memory; the caller owns the backing data. Functions
    /// that need to keep data after the call copy it into Rust-owned storage.
    ///
    /// @code
    /// std::vector<double> values{
    ///     0.0, 0.0,
    ///     0.5, 0.25,
    ///     1.0, 1.0,
    /// };
    /// auto row_major = copp::MatrixView::row_major(values.data(), 3, 2);
    /// auto path = copp::Path::from_waypoints(row_major);
    /// @endcode
    class MatrixView
    {
    public:
        constexpr MatrixView() noexcept = default;

        constexpr MatrixView(
            const double *data,
            std::size_t rows,
            std::size_t cols,
            MatrixLayout layout,
            std::size_t leading_dim) noexcept
            : data_(data), rows_(rows), cols_(cols), layout_(layout), leading_dim_(leading_dim) {}

        static constexpr MatrixView column_major(
            const double *data,
            std::size_t rows,
            std::size_t cols) noexcept
        {
            return MatrixView(data, rows, cols, MatrixLayout::ColumnMajor, rows);
        }

        static constexpr MatrixView strided_column_major(
            const double *data,
            std::size_t rows,
            std::size_t cols,
            std::size_t leading_dim) noexcept
        {
            return MatrixView(data, rows, cols, MatrixLayout::ColumnMajor, leading_dim);
        }

        static constexpr MatrixView row_major(
            const double *data,
            std::size_t rows,
            std::size_t cols) noexcept
        {
            return MatrixView(data, rows, cols, MatrixLayout::RowMajor, cols);
        }

        constexpr const double *data() const noexcept { return data_; }
        constexpr std::size_t rows() const noexcept { return rows_; }
        constexpr std::size_t cols() const noexcept { return cols_; }
        constexpr MatrixLayout layout() const noexcept { return layout_; }
        constexpr std::size_t leading_dim() const noexcept { return leading_dim_; }

        constexpr std::size_t storage_size() const noexcept
        {
            if (rows_ == 0 || cols_ == 0)
            {
                return 0;
            }
            if (layout_ == MatrixLayout::ColumnMajor)
            {
                return leading_dim_ * (cols_ - 1) + rows_;
            }
            return leading_dim_ * (rows_ - 1) + cols_;
        }

    private:
        const double *data_ = nullptr;
        std::size_t rows_ = 0;
        std::size_t cols_ = 0;
        MatrixLayout layout_ = MatrixLayout::ColumnMajor;
        std::size_t leading_dim_ = 0;
    };

    /// Mutable column-major matrix view used by callback APIs.
    ///
    /// `MatrixRef` is a writable borrowed output buffer. It never owns memory. The
    /// current callback path uses column-major output buffers with shape
    /// `(rows x cols)` and storage index `row + col * leading_dim`.
    ///
    /// @code
    /// auto evaluator = [](copp::Span<const double> s,
    ///                     copp::MatrixRef q,
    ///                     copp::MatrixRef dq,
    ///                     copp::MatrixRef ddq) {
    ///     for (std::size_t j = 0; j < s.size(); ++j) {
    ///         q(0, j) = s[j];
    ///         dq(0, j) = 1.0;
    ///         ddq(0, j) = 0.0;
    ///     }
    /// };
    /// @endcode
    class MatrixRef
    {
    public:
        constexpr MatrixRef() noexcept = default;

        constexpr MatrixRef(
            double *data,
            std::size_t rows,
            std::size_t cols,
            std::size_t leading_dim) noexcept
            : data_(data), rows_(rows), cols_(cols), leading_dim_(leading_dim) {}

        static constexpr MatrixRef column_major(
            double *data,
            std::size_t rows,
            std::size_t cols) noexcept
        {
            return MatrixRef(data, rows, cols, rows);
        }

        static constexpr MatrixRef strided_column_major(
            double *data,
            std::size_t rows,
            std::size_t cols,
            std::size_t leading_dim) noexcept
        {
            return MatrixRef(data, rows, cols, leading_dim);
        }

        constexpr double *data() const noexcept { return data_; }
        constexpr std::size_t rows() const noexcept { return rows_; }
        constexpr std::size_t cols() const noexcept { return cols_; }
        constexpr std::size_t leading_dim() const noexcept { return leading_dim_; }

        double &operator()(std::size_t row, std::size_t col) const noexcept
        {
            return data_[row + col * leading_dim_];
        }

    private:
        double *data_ = nullptr;
        std::size_t rows_ = 0;
        std::size_t cols_ = 0;
        std::size_t leading_dim_ = 0;
    };

    /// Lightweight owned column-major matrix.
    ///
    /// This type is intentionally just storage plus shape. It does not implement
    /// linear algebra. The data layout is always column-major and matches nalgebra:
    /// `matrix(row, col)` is stored at `row + col * rows()`.
    ///
    /// Eigen integration lives in `copp/eigen.hpp` as adapters over this storage
    /// rather than changing the core return type. This keeps the core API usable
    /// without Eigen while still making Eigen views cheap for common users.
    ///
    /// @code
    /// auto waypoints = copp::Matrix::from_columns({
    ///     {0.0, 0.0},
    ///     {0.5, 0.25},
    ///     {1.0, 1.0},
    /// });
    /// std::cout << waypoints(1, 2) << "\n"; // 1.0
    /// @endcode
    class Matrix
    {
    public:
        Matrix() = default;

        Matrix(std::size_t rows, std::size_t cols)
            : rows_(rows), cols_(cols), data_(rows * cols) {}

        Matrix(std::size_t rows, std::size_t cols, std::vector<double> data)
            : rows_(rows), cols_(cols), data_(std::move(data))
        {
            if (data_.size() != rows_ * cols_)
            {
                throw Error(Status::invalid_input, "Matrix data length does not match shape");
            }
        }

        /// Build a matrix from row-major initializer-list notation.
        ///
        /// @param rows Outer list of matrix rows. Every row must have the same
        /// number of columns.
        /// @return An owned column-major `Matrix` containing the same logical
        /// values.
        /// @throws copp::Error if input is empty or not rectangular.
        ///
        /// @code
        /// auto matrix = copp::Matrix::from_rows({
        ///     {0.0, 0.5, 1.0},
        ///     {0.0, 0.25, 1.0},
        /// });
        /// @endcode
        static Matrix from_rows(std::initializer_list<std::initializer_list<double>> rows)
        {
            if (rows.size() == 0)
            {
                throw Error(Status::invalid_input, "Matrix::from_rows requires at least one row");
            }

            const std::size_t row_count = rows.size();
            const std::size_t col_count = rows.begin()->size();
            if (col_count == 0)
            {
                throw Error(Status::invalid_input, "Matrix::from_rows requires at least one column");
            }

            Matrix matrix(row_count, col_count);
            std::size_t row = 0;
            for (const auto &row_values : rows)
            {
                if (row_values.size() != col_count)
                {
                    throw Error(Status::invalid_input, "Matrix::from_rows requires rectangular input");
                }
                std::size_t col = 0;
                for (double value : row_values)
                {
                    matrix(row, col) = value;
                    ++col;
                }
                ++row;
            }
            return matrix;
        }

        /// Build a matrix from column-major initializer-list notation.
        ///
        /// This is convenient for waypoint paths because COPP represents
        /// waypoint matrices as `(dim x n_points)`, where each column is one
        /// waypoint:
        ///
        /// @param columns Outer list of waypoint columns. Every column must have
        /// the same dimension.
        /// @return An owned column-major `Matrix`.
        /// @throws copp::Error if input is empty or not rectangular.
        ///
        /// @code
        /// auto waypoints = copp::Matrix::from_columns({
        ///     {0.0, 0.0},
        ///     {0.5, 0.25},
        ///     {1.0, 1.0},
        /// });
        /// @endcode
        static Matrix from_columns(std::initializer_list<std::initializer_list<double>> columns)
        {
            if (columns.size() == 0)
            {
                throw Error(Status::invalid_input, "Matrix::from_columns requires at least one column");
            }

            const std::size_t col_count = columns.size();
            const std::size_t row_count = columns.begin()->size();
            if (row_count == 0)
            {
                throw Error(Status::invalid_input, "Matrix::from_columns requires at least one row");
            }

            Matrix matrix(row_count, col_count);
            std::size_t col = 0;
            for (const auto &col_values : columns)
            {
                if (col_values.size() != row_count)
                {
                    throw Error(Status::invalid_input, "Matrix::from_columns requires rectangular input");
                }
                std::size_t row = 0;
                for (double value : col_values)
                {
                    matrix(row, col) = value;
                    ++row;
                }
                ++col;
            }
            return matrix;
        }

        std::size_t rows() const noexcept { return rows_; }
        std::size_t cols() const noexcept { return cols_; }
        std::size_t size() const noexcept { return data_.size(); }
        bool empty() const noexcept { return data_.empty(); }

        double *data() noexcept { return data_.data(); }
        const double *data() const noexcept { return data_.data(); }

        std::vector<double> &values() noexcept { return data_; }
        const std::vector<double> &values() const noexcept { return data_; }

        MatrixView view() const noexcept
        {
            return MatrixView::column_major(data(), rows_, cols_);
        }

        double &operator()(std::size_t row, std::size_t col) noexcept
        {
            return data_[row + col * rows_];
        }

        double operator()(std::size_t row, std::size_t col) const noexcept
        {
            return data_[row + col * rows_];
        }

    private:
        std::size_t rows_ = 0;
        std::size_t cols_ = 0;
        std::vector<double> data_;
    };

    /// Lightweight error payload returned by no-throw APIs.
    ///
    /// `status` is intentionally coarse; `message` keeps the detailed Rust/C++
    /// diagnostic suitable for logs or UI display.
    struct ErrorInfo
    {
        Status status = Status::ok;
        std::string message;
    };

    /// Small C++17 expected-like result used by no-throw overloads.
    ///
    /// The throwing API remains the primary surface. No-throw overloads take
    /// `copp::no_throw` as their final argument and return `Expected<T>`.
    /// They are implemented by catching exceptions from the normal API, so this
    /// is not a `-fno-exceptions` build mode.
    ///
    /// @code
    /// auto result = copp::Path::from_waypoints({{0.0}, {1.0}}, copp::no_throw);
    /// if (!result) {
    ///     std::cerr << result.error().message << "\n";
    ///     return;
    /// }
    /// auto path = std::move(result).value();
    /// @endcode
    ///
    /// @tparam T Successful return type.
    template <typename T>
    class Expected
    {
    public:
        Expected(T value) : value_(std::move(value)) {}

        static Expected failure(Status status, std::string message)
        {
            Expected out;
            out.error_ = ErrorInfo{status, std::move(message)};
            return out;
        }

        bool has_value() const noexcept { return value_.has_value(); }
        explicit operator bool() const noexcept { return has_value(); }

        T &value() &
        {
            if (!value_)
            {
                throw Error(error_.status, error_.message);
            }
            return *value_;
        }

        const T &value() const &
        {
            if (!value_)
            {
                throw Error(error_.status, error_.message);
            }
            return *value_;
        }

        T &&value() &&
        {
            if (!value_)
            {
                throw Error(error_.status, error_.message);
            }
            return std::move(*value_);
        }

        const ErrorInfo &error() const noexcept { return error_; }

    private:
        Expected() = default;

        std::optional<T> value_;
        ErrorInfo error_;
    };

    namespace detail
    {
        template <typename Fn>
        auto expected_from(Fn &&fn) -> Expected<typename std::decay<decltype(fn())>::type>
        {
            using T = typename std::decay<decltype(fn())>::type;
            try
            {
                return Expected<T>(fn());
            }
            catch (const Error &error)
            {
                return Expected<T>::failure(error.status(), error.what());
            }
            catch (const std::exception &error)
            {
                return Expected<T>::failure(Status::invalid_input, error.what());
            }
            catch (...)
            {
                return Expected<T>::failure(Status::bridge_error, "unknown C++ exception");
            }
        }
    } // namespace detail

    /// Tag object selecting no-throw overloads.
    ///
    /// Put `copp::no_throw` as the final argument. The function name remains the
    /// same as the throwing API, but the return type becomes `Expected<T>`.
    /// The binding still requires C++ exception support internally.
    struct NoThrowTag
    {
        explicit constexpr NoThrowTag() = default;
    };

    inline constexpr NoThrowTag no_throw{};

} // namespace copp
