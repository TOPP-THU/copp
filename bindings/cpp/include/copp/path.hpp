#pragma once

#include <cmath>
#include <functional>
#include <initializer_list>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "copp/core.hpp"

namespace copp
{

    class Path;

    namespace detail
    {
        /// Return the private bridge handle stored by `Path`.
        ///
        /// This is an implementation hook for other C++ facade translation
        /// units. Public users should not call it; it intentionally returns
        /// `void*` so public headers do not expose generated `cxx` bridge types.
        COPP_API const void *path_handle(const Path &path) noexcept;
    } // namespace detail

    /// Third-order forward-mode automatic-differentiation scalar.
    ///
    /// `v` is the scalar value. `d1`, `d2`, and `d3` are derivatives with
    /// respect to the path parameter `s`. Parametric path callbacks receive a
    /// seeded scalar (`d1 = 1`) and return or write one `Jet3` per path
    /// dimension; COPP extracts all derivatives up to third order.
    ///
    /// @code
    /// auto path = copp::Path::from_parametric(
    ///     [](copp::Jet3 s) {
    ///         return std::vector<copp::Jet3>{s, copp::sin(s), copp::powi(s, 3)};
    ///     },
    ///     0.0,
    ///     1.0);
    /// @endcode
    struct Jet3
    {
        double v = 0.0;
        double d1 = 0.0;
        double d2 = 0.0;
        double d3 = 0.0;

        static constexpr Jet3 constant(double value) noexcept
        {
            return Jet3{value, 0.0, 0.0, 0.0};
        }

        static constexpr Jet3 seed(double value) noexcept
        {
            return Jet3{value, 1.0, 0.0, 0.0};
        }
    };

    inline constexpr Jet3 operator+(Jet3 lhs, Jet3 rhs) noexcept
    {
        return Jet3{
            lhs.v + rhs.v,
            lhs.d1 + rhs.d1,
            lhs.d2 + rhs.d2,
            lhs.d3 + rhs.d3,
        };
    }

    inline constexpr Jet3 operator+(Jet3 lhs, double rhs) noexcept
    {
        return Jet3{lhs.v + rhs, lhs.d1, lhs.d2, lhs.d3};
    }

    inline constexpr Jet3 operator+(double lhs, Jet3 rhs) noexcept
    {
        return rhs + lhs;
    }

    inline constexpr Jet3 operator-(Jet3 lhs, Jet3 rhs) noexcept
    {
        return Jet3{
            lhs.v - rhs.v,
            lhs.d1 - rhs.d1,
            lhs.d2 - rhs.d2,
            lhs.d3 - rhs.d3,
        };
    }

    inline constexpr Jet3 operator-(Jet3 lhs, double rhs) noexcept
    {
        return Jet3{lhs.v - rhs, lhs.d1, lhs.d2, lhs.d3};
    }

    inline constexpr Jet3 operator-(double lhs, Jet3 rhs) noexcept
    {
        return Jet3{lhs - rhs.v, -rhs.d1, -rhs.d2, -rhs.d3};
    }

    inline constexpr Jet3 operator-(Jet3 value) noexcept
    {
        return Jet3{-value.v, -value.d1, -value.d2, -value.d3};
    }

    inline constexpr Jet3 operator*(Jet3 lhs, Jet3 rhs) noexcept
    {
        return Jet3{
            lhs.v * rhs.v,
            lhs.d1 * rhs.v + lhs.v * rhs.d1,
            lhs.d2 * rhs.v + 2.0 * lhs.d1 * rhs.d1 + lhs.v * rhs.d2,
            lhs.d3 * rhs.v + 3.0 * (lhs.d2 * rhs.d1 + lhs.d1 * rhs.d2) + lhs.v * rhs.d3,
        };
    }

    inline constexpr Jet3 operator*(Jet3 lhs, double rhs) noexcept
    {
        return Jet3{
            lhs.v * rhs,
            lhs.d1 * rhs,
            lhs.d2 * rhs,
            lhs.d3 * rhs,
        };
    }

    inline constexpr Jet3 operator*(double lhs, Jet3 rhs) noexcept
    {
        return rhs * lhs;
    }

    inline Jet3 operator/(double lhs, Jet3 rhs) noexcept
    {
        const double inv = 1.0 / rhs.v;
        const double inv2 = inv * inv;
        const double inv3 = inv2 * inv;
        const double inv4 = inv3 * inv;
        const double d1sq = rhs.d1 * rhs.d1;
        return Jet3{
            lhs * inv,
            -lhs * inv2 * rhs.d1,
            lhs * (2.0 * inv3 * d1sq - inv2 * rhs.d2),
            lhs * (-6.0 * inv4 * d1sq * rhs.d1 + 6.0 * inv3 * rhs.d1 * rhs.d2 - inv2 * rhs.d3),
        };
    }

    inline constexpr Jet3 operator/(Jet3 lhs, double rhs) noexcept
    {
        return lhs * (1.0 / rhs);
    }

    inline Jet3 operator/(Jet3 lhs, Jet3 rhs) noexcept
    {
        return lhs * (1.0 / rhs);
    }

    inline Jet3 sin(Jet3 x) noexcept
    {
        const double sv = std::sin(x.v);
        const double cv = std::cos(x.v);
        const double d1sq = x.d1 * x.d1;
        return Jet3{
            sv,
            cv * x.d1,
            -sv * d1sq + cv * x.d2,
            -cv * d1sq * x.d1 - 3.0 * sv * x.d1 * x.d2 + cv * x.d3,
        };
    }

    inline Jet3 cos(Jet3 x) noexcept
    {
        const double sv = std::sin(x.v);
        const double cv = std::cos(x.v);
        const double d1sq = x.d1 * x.d1;
        return Jet3{
            cv,
            -sv * x.d1,
            -cv * d1sq - sv * x.d2,
            sv * d1sq * x.d1 - 3.0 * cv * x.d1 * x.d2 - sv * x.d3,
        };
    }

    inline Jet3 exp(Jet3 x) noexcept
    {
        const double ev = std::exp(x.v);
        const double d1sq = x.d1 * x.d1;
        return Jet3{
            ev,
            ev * x.d1,
            ev * (d1sq + x.d2),
            ev * (d1sq * x.d1 + 3.0 * x.d1 * x.d2 + x.d3),
        };
    }

    inline Jet3 log(Jet3 x) noexcept
    {
        const double inv = 1.0 / x.v;
        const double inv2 = inv * inv;
        const double inv3 = inv2 * inv;
        const double d1sq = x.d1 * x.d1;
        return Jet3{
            std::log(x.v),
            inv * x.d1,
            -inv2 * d1sq + inv * x.d2,
            2.0 * inv3 * d1sq * x.d1 - 3.0 * inv2 * x.d1 * x.d2 + inv * x.d3,
        };
    }

    inline Jet3 ln(Jet3 x) noexcept
    {
        return log(x);
    }

    inline Jet3 sqrt(Jet3 x) noexcept
    {
        const double sqrtv = std::sqrt(x.v);
        const double inv_sqrt = 1.0 / sqrtv;
        const double inv_v_sqrt = inv_sqrt / x.v;
        const double inv_v2_sqrt = inv_v_sqrt / x.v;
        const double d1sq = x.d1 * x.d1;
        return Jet3{
            sqrtv,
            0.5 * inv_sqrt * x.d1,
            -0.25 * inv_v_sqrt * d1sq + 0.5 * inv_sqrt * x.d2,
            0.375 * inv_v2_sqrt * d1sq * x.d1 - 0.75 * inv_v_sqrt * x.d1 * x.d2 +
                0.5 * inv_sqrt * x.d3,
        };
    }

    inline Jet3 powi(Jet3 x, int n) noexcept
    {
        if (n == 0)
        {
            return Jet3::constant(1.0);
        }

        const double nf = static_cast<double>(n);
        const double coeff2 = nf * (nf - 1.0);
        const double coeff3 = coeff2 * (nf - 2.0);
        const double dv = nf * std::pow(x.v, nf - 1.0);
        const double ddv = coeff2 == 0.0 ? 0.0 : coeff2 * std::pow(x.v, nf - 2.0);
        const double dddv = coeff3 == 0.0 ? 0.0 : coeff3 * std::pow(x.v, nf - 3.0);
        const double d1sq = x.d1 * x.d1;
        return Jet3{
            std::pow(x.v, nf),
            dv * x.d1,
            ddv * d1sq + dv * x.d2,
            dddv * d1sq * x.d1 + 3.0 * ddv * x.d1 * x.d2 + dv * x.d3,
        };
    }

    /// Scalar-parametric path callback that returns one `Jet3` per dimension.
    ///
    /// @param s Seeded path parameter. Use normal arithmetic and COPP math
    /// helpers (`sin`, `cos`, `exp`, `log`, `sqrt`, `powi`) to propagate
    /// derivatives.
    /// @return One `Jet3` value per path dimension.
    using PathParametric = std::function<std::vector<Jet3>(Jet3 s)>;

    /// Allocation-free scalar-parametric path callback that writes one `Jet3` per dimension.
    ///
    /// @param s Seeded path parameter.
    /// @param q Writable span with length equal to the dimension passed to
    /// `Path::from_parametric(dim, ...)`.
    using PathParametricWriter = std::function<void(Jet3 s, Span<Jet3> q)>;

    /// Batch second-order evaluator callback.
    ///
    /// The callback receives all query samples at once and must fill `q`, `dq`,
    /// and `ddq`. Output matrices have shape `(dim x s.size())` and
    /// column-major storage. Throw `copp::Error` or any `std::exception` to
    /// report evaluation failure.
    using PathEvaluator2nd = std::function<void(
        Span<const double> s,
        MatrixRef q,
        MatrixRef dq,
        MatrixRef ddq)>;

    /// Batch third-order evaluator callback.
    ///
    /// The callback receives all query samples at once and must fill `q`, `dq`,
    /// `ddq`, and `dddq`. Output matrices have shape `(dim x s.size())` and
    /// column-major storage. A path built from this callback can also be
    /// evaluated with `evaluate_q` and `evaluate_up_to_2nd`.
    using PathEvaluator3rd = std::function<void(
        Span<const double> s,
        MatrixRef q,
        MatrixRef dq,
        MatrixRef ddq,
        MatrixRef dddq)>;

    /// Policy for evaluating a path outside its configured `s` range.
    ///
    /// This is forwarded to the Rust path evaluator. Use `Clamp` for plotting
    /// or UI previews; use `Error` for solver input so invalid station grids are
    /// caught early.
    enum class OutOfRangeMode
    {
        /// Throw `copp::Error` when any query sample is outside `[s_min, s_max]`.
        Error,
        /// Clamp out-of-range query samples to the nearest endpoint.
        Clamp,
    };

    /// Configuration for waypoint spline construction.
    ///
    /// The first C++ Path milestone supports the common waypoint-spline case only.
    /// Defaults match Rust/Python:
    ///
    /// - quintic spline (`order = 5`)
    /// - uniform waypoint parametrization
    /// - parameter range `[0, 1]`
    /// - out-of-range queries are errors
    ///
    struct SplineConfig
    {
        std::size_t order = 5;
        double s_min = 0.0;
        double s_max = 1.0;
        OutOfRangeMode out_of_range = OutOfRangeMode::Error;
        /// Optional boundary derivatives at `s_min`.
        ///
        /// Shape is `(dim, m)` where `m = (order - 1) / 2`. Column `r` stores
        /// the `(r + 1)`-th derivative value at the endpoint. `std::nullopt`
        /// means zero boundary derivatives, matching Rust/Python/C defaults.
        std::optional<Matrix> start_state;
        /// Optional boundary derivatives at `s_max`; same shape convention as
        /// `start_state`.
        std::optional<Matrix> end_state;
    };

    /// Configuration for tolerance-bounded waypoint fitting.
    ///
    /// Used by `Path::from_waypoints_fitting`; mirrors Rust
    /// `copp::path::SmoothingConfig`. Error is the absolute deviation of
    /// each selected axis from the input polyline at the same parameter, in
    /// input units, over the complete parameter domain. No kinematics, angle
    /// unwrapping, nearest-point search, or unit conversion is performed.
    ///
    /// Defaults match Rust/Python:
    ///
    /// - fit every input row
    /// - absolute tolerance `1e-3` on each selected row
    /// - waypoint parameters uniform on `[0, 1]`
    /// - at most 20 refinement passes and 20000 selected-axis spans
    /// - out-of-range queries are errors
    ///
    /// @warning Unstable API; see `Path::from_waypoints_fitting`.
    struct SmoothingConfig
    {
        /// Absolute tolerance applied to every selected axis when
        /// `tolerance_per_axis` is empty. Must be finite and strictly positive.
        double tolerance = 1.0e-3;
        /// Optional per-axis absolute tolerances. When non-empty it replaces
        /// `tolerance`; it needs one finite, strictly positive entry per
        /// selected axis, in `axes` order (not row order). With
        /// `axes == std::nullopt` the order is rows `0..dim-1`.
        std::vector<double> tolerance_per_axis;
        /// 0-based input rows allowed to deviate from their reference
        /// polyline. `std::nullopt` selects every row; an explicit list must be
        /// non-empty, distinct, and in range. Unselected rows keep C4 quintic
        /// interpolation through all waypoint columns.
        std::optional<std::vector<std::size_t>> axes;
        /// Optional common parameter per waypoint column: finite, strictly
        /// increasing, one entry per column. The first and last entries become
        /// the path range. `std::nullopt` assigns columns uniformly on `[0, 1]`.
        std::optional<std::vector<double>> parameters;
        /// Maximum number of adaptive knot-refinement passes.
        std::size_t max_refinements = 20;
        /// Maximum number of polynomial spans used by the selected axes.
        std::size_t max_segments = 20000;
        /// Policy for later queries outside the path range.
        OutOfRangeMode out_of_range = OutOfRangeMode::Error;
    };

    /// Diagnostics recorded while constructing a fitted waypoint path.
    ///
    /// Returned by `Path::smoothing_report()`; mirrors Rust
    /// `copp::path::SmoothingReport`. `axes`, `segments`, and
    /// `interpolated_segments` describe the final representation, while
    /// `refinements`, `fitting_rows`, and `checked_intervals` are cumulative
    /// work counters. None of these values is an optimality or run-time
    /// guarantee.
    struct SmoothingReport
    {
        /// Selected input-row indices, in tolerance/report order.
        std::vector<std::size_t> axes;
        /// Number of final polynomial spans shared by the selected axes.
        std::size_t segments = 0;
        /// Number of spans in the separate unselected-axis interpolant; zero
        /// when every axis is selected.
        std::size_t interpolated_segments = 0;
        /// Number of completed local knot-refinement passes.
        std::size_t refinements = 0;
        /// Number of discrete or quadrature fitting rows assembled.
        std::size_t fitting_rows = 0;
        /// Number of reference-polyline intervals visited by numerical audits.
        std::size_t checked_intervals = 0;
        /// Final whole-domain absolute-error bound for each selected axis, in
        /// input units and `axes` order. These are Bernstein-derived numerical
        /// bounds computed in ordinary floating point, not formal certificates.
        std::vector<double> max_errors;
    };

    /// Result of path evaluation.
    ///
    /// All matrices use shape `(dim, s.size())` and column-major storage. Optional
    /// derivative matrices are populated according to the evaluation method:
    ///
    /// - `evaluate_q`: only `q`
    /// - `evaluate_up_to_2nd`: `q`, `dq`, and `ddq`
    /// - `evaluate_up_to_3rd`: `q`, `dq`, `ddq`, and `dddq`
    struct PathDerivatives
    {
        Matrix q;
        std::optional<Matrix> dq;
        std::optional<Matrix> ddq;
        std::optional<Matrix> dddq;
    };

    /// Geometric path wrapper backed by Rust `Path`.
    ///
    /// `Path` owns a Rust-side path handle through the `cxx` bridge. It is movable
    /// but not copyable. The C++ public API exposes RAII ownership and throws
    /// `copp::Error` on construction or evaluation failure.
    class COPP_API Path
    {
    public:
        /// Build a spline path from waypoint positions.
        ///
        /// Equivalent alias of `from_waypoints_interpolating`: the spline passes
        /// through every waypoint. Both names are supported.
        ///
        /// `waypoints` is a `(dim x n_points)` matrix where each column is one
        /// waypoint. At least one dimension and two waypoint columns are required.
        ///
        /// Column-major views can cross the bridge directly. Row-major views are
        /// accepted for convenience and copied once into column-major storage before
        /// calling Rust.
        ///
        /// The input view only needs to remain valid during this call; the returned
        /// `Path` owns the constructed Rust path.
        ///
        /// @param waypoints Borrowed waypoint matrix.
        /// @param config Spline order, parameter range, and out-of-range policy.
        /// @return Movable owning `Path`.
        /// @throws copp::Error on invalid shape, invalid config, or Rust spline
        /// construction failure.
        ///
        /// @code
        /// auto waypoints = copp::Matrix::from_columns({
        ///     {0.0, 0.0},
        ///     {0.5, 0.25},
        ///     {1.0, 1.0},
        /// });
        /// auto path = copp::Path::from_waypoints(waypoints.view());
        /// @endcode
        static Path from_waypoints(MatrixView waypoints, SplineConfig config = {});
        static Expected<Path> from_waypoints(MatrixView waypoints, SplineConfig config, NoThrowTag);
        static Expected<Path> from_waypoints(MatrixView waypoints, NoThrowTag);

        /// Build a spline path from waypoint-list notation.
        ///
        /// Each inner list is one waypoint vector. The example below creates a
        /// two-dimensional path through three waypoints:
        ///
        /// @param waypoints Outer list of waypoint vectors, not matrix rows.
        /// @param config Spline construction options.
        /// @return Movable owning `Path`.
        /// @throws copp::Error if the waypoint list is empty, ragged, or invalid.
        ///
        /// @code
        /// auto path = copp::Path::from_waypoints({
        ///     {0.0, 0.0},
        ///     {0.5, 0.25},
        ///     {1.0, 1.0},
        /// });
        /// @endcode
        static Path from_waypoints(
            std::initializer_list<std::initializer_list<double>> waypoints,
            SplineConfig config = {});
        static Expected<Path> from_waypoints(
            std::initializer_list<std::initializer_list<double>> waypoints,
            SplineConfig config,
            NoThrowTag);
        static Expected<Path> from_waypoints(
            std::initializer_list<std::initializer_list<double>> waypoints,
            NoThrowTag);

        /// Build a spline path that interpolates every waypoint.
        ///
        /// The path passes through **every** waypoint column at its assigned
        /// parameter, up to floating-point roundoff. Choose it when each
        /// waypoint is a position or event that must be retained; use
        /// `from_waypoints_fitting` when the waypoints form a reference
        /// polyline that may be approximated within tolerances.
        ///
        /// `waypoints` is a `(dim x n_points)` matrix where each column is one
        /// waypoint. At least one dimension and two waypoint columns are
        /// required. Row-major views are copied once into column-major storage;
        /// the view only needs to remain valid during this call.
        ///
        /// @warning Unstable API: the waypoint interpolation and fitting
        /// constructor families (all `from_waypoints*` overloads) may change
        /// names, signatures, configuration types, and method-selection
        /// interfaces in future releases as more algorithms are added.
        ///
        /// @param waypoints Borrowed waypoint matrix.
        /// @param config Spline order, parameter range, boundary states, and
        /// out-of-range policy.
        /// @return Movable owning `Path`.
        /// @throws copp::Error on invalid shape, invalid config, or Rust spline
        /// construction failure.
        ///
        /// @code
        /// auto waypoints = copp::Matrix::from_columns({
        ///     {0.0, 0.0},
        ///     {0.5, 0.25},
        ///     {1.0, 1.0},
        /// });
        /// auto path = copp::Path::from_waypoints_interpolating(waypoints.view());
        /// @endcode
        static Path from_waypoints_interpolating(MatrixView waypoints, SplineConfig config = {});
        static Expected<Path> from_waypoints_interpolating(
            MatrixView waypoints,
            SplineConfig config,
            NoThrowTag);
        static Expected<Path> from_waypoints_interpolating(MatrixView waypoints, NoThrowTag);

        /// Build an interpolating spline path from waypoint-list notation.
        ///
        /// Each inner list is one waypoint vector, not a matrix row.
        ///
        /// @warning Unstable API; see the `MatrixView` overload.
        ///
        /// @code
        /// auto path = copp::Path::from_waypoints_interpolating({
        ///     {0.0, 0.0},
        ///     {0.5, 0.25},
        ///     {1.0, 1.0},
        /// });
        /// @endcode
        static Path from_waypoints_interpolating(
            std::initializer_list<std::initializer_list<double>> waypoints,
            SplineConfig config = {});
        static Expected<Path> from_waypoints_interpolating(
            std::initializer_list<std::initializer_list<double>> waypoints,
            SplineConfig config,
            NoThrowTag);
        static Expected<Path> from_waypoints_interpolating(
            std::initializer_list<std::initializer_list<double>> waypoints,
            NoThrowTag);

        /// Build a tolerance-bounded fitted path from waypoint positions.
        ///
        /// Adjacent waypoint columns form a reference polyline under one common
        /// parameter. The result uses adaptive nonuniform C4 quintic B-splines
        /// and does **not** pass through interior waypoints: selected axes may
        /// deviate from the polyline within their tolerances, while endpoint
        /// positions are retained. The absolute error at the same parameter is
        /// numerically audited over every complete reference interval.
        /// Unselected axes keep C4 quintic interpolation through all columns.
        /// Inspect the achieved bounds with `smoothing_report()`.
        ///
        /// Fitting is geometric preprocessing, not time parameterization; path
        /// derivatives remain derivatives with respect to `s`.
        ///
        /// @warning Unstable API: the waypoint interpolation and fitting
        /// constructor families (all `from_waypoints*` overloads) may change
        /// names, signatures, configuration types, and method-selection
        /// interfaces in future releases as more algorithms are added.
        ///
        /// @param waypoints Borrowed `(dim x n_points)` waypoint matrix.
        /// @param config Tolerances, axis selection, parameters, and budgets.
        /// @return Movable owning `Path` with a populated `smoothing_report()`.
        /// @throws copp::Error on invalid shape or configuration, or when the
        /// tolerances cannot be met within the configured budgets.
        ///
        /// @code
        /// copp::SmoothingConfig config;
        /// config.tolerance = 1.0e-3;
        /// auto path = copp::Path::from_waypoints_fitting(waypoints.view(), config);
        /// auto report = path.smoothing_report(); // report->max_errors[i] <= 1e-3
        /// @endcode
        static Path from_waypoints_fitting(MatrixView waypoints, SmoothingConfig config = {});
        static Expected<Path> from_waypoints_fitting(
            MatrixView waypoints,
            SmoothingConfig config,
            NoThrowTag);
        static Expected<Path> from_waypoints_fitting(MatrixView waypoints, NoThrowTag);

        /// Build a tolerance-bounded fitted path from waypoint-list notation.
        ///
        /// Each inner list is one waypoint vector, not a matrix row.
        ///
        /// @warning Unstable API; see the `MatrixView` overload.
        static Path from_waypoints_fitting(
            std::initializer_list<std::initializer_list<double>> waypoints,
            SmoothingConfig config = {});
        static Expected<Path> from_waypoints_fitting(
            std::initializer_list<std::initializer_list<double>> waypoints,
            SmoothingConfig config,
            NoThrowTag);
        static Expected<Path> from_waypoints_fitting(
            std::initializer_list<std::initializer_list<double>> waypoints,
            NoThrowTag);

        /// Build a path from a scalar-parametric formula.
        ///
        /// The callback receives a seeded `Jet3` value and returns one `Jet3`
        /// per path dimension. The dimension is inferred by probing the
        /// callback once at the midpoint of `[s_min, s_max]`. Derivatives up to
        /// third order are propagated by `Jet3` arithmetic.
        ///
        /// @param parametric Function from scalar `s` to a vector-valued path.
        /// @param s_min Lower bound of the valid path range.
        /// @param s_max Upper bound of the valid path range.
        /// @return Movable owning `Path`.
        /// @throws copp::Error if the callback is empty, returns zero or
        /// inconsistent dimension, throws during probing/evaluation, or the
        /// range is invalid.
        static Path from_parametric(
            PathParametric parametric,
            double s_min,
            double s_max);
        static Expected<Path> from_parametric(
            PathParametric parametric,
            double s_min,
            double s_max,
            NoThrowTag);

        /// Build a path from a scalar-parametric formula that writes into a buffer.
        ///
        /// This overload avoids allocating a `std::vector<Jet3>` on each path
        /// sample. The callback must write exactly `dim` entries into `q`.
        ///
        /// @param dim Path dimension / number of entries written by the callback.
        /// @param s_min Lower bound of the valid path range.
        /// @param s_max Upper bound of the valid path range.
        /// @param parametric Allocation-free writer callback.
        static Path from_parametric(
            std::size_t dim,
            double s_min,
            double s_max,
            PathParametricWriter parametric);
        static Expected<Path> from_parametric(
            std::size_t dim,
            double s_min,
            double s_max,
            PathParametricWriter parametric,
            NoThrowTag);

        /// Build a path from a batch evaluator with explicit derivatives up to second order.
        ///
        /// The returned `Path` owns the callback. `dim` must be positive and
        /// remain consistent with the callback's output writes. The path range
        /// must satisfy finite `s_min < s_max`.
        ///
        /// @param dim Path dimension.
        /// @param s_min Lower range endpoint.
        /// @param s_max Upper range endpoint.
        /// @param evaluator Batch callback that fills `q`, `dq`, and `ddq`.
        /// @return Movable owning `Path`.
        /// @throws copp::Error on invalid dimensions/range or callback failure.
        static Path from_evaluator_2nd(
            std::size_t dim,
            double s_min,
            double s_max,
            PathEvaluator2nd evaluator);
        static Expected<Path> from_evaluator_2nd(
            std::size_t dim,
            double s_min,
            double s_max,
            PathEvaluator2nd evaluator,
            NoThrowTag);

        /// Build a path from a batch evaluator with explicit derivatives up to third order.
        ///
        /// The returned `Path` owns the callback. `dim` must be positive and
        /// remain consistent with the callback's output writes. The path range
        /// must satisfy finite `s_min < s_max`.
        ///
        /// A third-order evaluator can also be used by TOPP2/COPP2 workflows;
        /// the facade requests only the derivatives that each solver needs.
        static Path from_evaluator_3rd(
            std::size_t dim,
            double s_min,
            double s_max,
            PathEvaluator3rd evaluator);
        static Expected<Path> from_evaluator_3rd(
            std::size_t dim,
            double s_min,
            double s_max,
            PathEvaluator3rd evaluator,
            NoThrowTag);

        Path(Path &&) noexcept;
        Path &operator=(Path &&) noexcept;
        ~Path();

        Path(const Path &) = delete;
        Path &operator=(const Path &) = delete;

        /// Return the path dimension.
        ///
        /// @return Number of generalized coordinates in `q(s)`.
        std::size_t dim() const;

        /// Return the valid path-parameter interval `(s_min, s_max)`.
        ///
        /// @return Pair `{s_min, s_max}`.
        std::pair<double, double> s_range() const;

        /// Evaluate position samples only.
        ///
        /// Returns `PathDerivatives` with `q` populated and derivative fields empty.
        ///
        /// @param s Query samples.
        /// @return Column-major matrix `q` with shape `(dim(), s.size())`.
        /// @throws copp::Error if samples are invalid or the underlying
        /// evaluator fails.
        PathDerivatives evaluate_q(Span<const double> s) const;
        Expected<PathDerivatives> evaluate_q(Span<const double> s, NoThrowTag) const;

        /// Evaluate position, first derivative, and second derivative.
        ///
        /// Returns `q`, `dq`, and `ddq`, each with shape `(dim, s.size())`.
        ///
        /// @param s Query samples.
        /// @return `PathDerivatives` with `q`, `dq`, and `ddq` populated.
        PathDerivatives evaluate_up_to_2nd(Span<const double> s) const;
        Expected<PathDerivatives> evaluate_up_to_2nd(Span<const double> s, NoThrowTag) const;

        /// Evaluate position, first derivative, second derivative, and third derivative.
        ///
        /// Returns `q`, `dq`, `ddq`, and `dddq`, each with shape `(dim, s.size())`.
        ///
        /// @param s Query samples.
        /// @return `PathDerivatives` with all derivative matrices populated.
        PathDerivatives evaluate_up_to_3rd(Span<const double> s) const;
        Expected<PathDerivatives> evaluate_up_to_3rd(Span<const double> s, NoThrowTag) const;

        /// Return fitting diagnostics of a path built by `from_waypoints_fitting`.
        ///
        /// @return A copied `SmoothingReport`, or `std::nullopt` for
        /// interpolated, parametric, and evaluator paths.
        std::optional<SmoothingReport> smoothing_report() const;

    private:
        struct Impl;

        explicit Path(std::unique_ptr<Impl> impl);

        std::unique_ptr<Impl> impl_;

        friend const void *detail::path_handle(const Path &path) noexcept;
    };

    inline Expected<Path> Path::from_waypoints(
        MatrixView waypoints,
        SplineConfig config,
        NoThrowTag)
    {
        return detail::expected_from([&] { return from_waypoints(waypoints, config); });
    }

    inline Expected<Path> Path::from_waypoints(MatrixView waypoints, NoThrowTag tag)
    {
        return from_waypoints(waypoints, SplineConfig{}, tag);
    }

    inline Expected<Path> Path::from_waypoints(
        std::initializer_list<std::initializer_list<double>> waypoints,
        SplineConfig config,
        NoThrowTag)
    {
        return detail::expected_from([&] { return from_waypoints(waypoints, config); });
    }

    inline Expected<Path> Path::from_waypoints(
        std::initializer_list<std::initializer_list<double>> waypoints,
        NoThrowTag tag)
    {
        return from_waypoints(waypoints, SplineConfig{}, tag);
    }

    inline Expected<Path> Path::from_waypoints_interpolating(
        MatrixView waypoints,
        SplineConfig config,
        NoThrowTag)
    {
        return detail::expected_from([&] { return from_waypoints_interpolating(waypoints, config); });
    }

    inline Expected<Path> Path::from_waypoints_interpolating(MatrixView waypoints, NoThrowTag tag)
    {
        return from_waypoints_interpolating(waypoints, SplineConfig{}, tag);
    }

    inline Expected<Path> Path::from_waypoints_interpolating(
        std::initializer_list<std::initializer_list<double>> waypoints,
        SplineConfig config,
        NoThrowTag)
    {
        return detail::expected_from([&] { return from_waypoints_interpolating(waypoints, config); });
    }

    inline Expected<Path> Path::from_waypoints_interpolating(
        std::initializer_list<std::initializer_list<double>> waypoints,
        NoThrowTag tag)
    {
        return from_waypoints_interpolating(waypoints, SplineConfig{}, tag);
    }

    inline Expected<Path> Path::from_waypoints_fitting(
        MatrixView waypoints,
        SmoothingConfig config,
        NoThrowTag)
    {
        return detail::expected_from([&] { return from_waypoints_fitting(waypoints, config); });
    }

    inline Expected<Path> Path::from_waypoints_fitting(MatrixView waypoints, NoThrowTag tag)
    {
        return from_waypoints_fitting(waypoints, SmoothingConfig{}, tag);
    }

    inline Expected<Path> Path::from_waypoints_fitting(
        std::initializer_list<std::initializer_list<double>> waypoints,
        SmoothingConfig config,
        NoThrowTag)
    {
        return detail::expected_from([&] { return from_waypoints_fitting(waypoints, config); });
    }

    inline Expected<Path> Path::from_waypoints_fitting(
        std::initializer_list<std::initializer_list<double>> waypoints,
        NoThrowTag tag)
    {
        return from_waypoints_fitting(waypoints, SmoothingConfig{}, tag);
    }

    inline Expected<Path> Path::from_parametric(
        PathParametric parametric,
        double s_min,
        double s_max,
        NoThrowTag)
    {
        return detail::expected_from(
            [&] { return from_parametric(std::move(parametric), s_min, s_max); });
    }

    inline Expected<Path> Path::from_parametric(
        std::size_t dim,
        double s_min,
        double s_max,
        PathParametricWriter parametric,
        NoThrowTag)
    {
        return detail::expected_from(
            [&] { return from_parametric(dim, s_min, s_max, std::move(parametric)); });
    }

    inline Expected<Path> Path::from_evaluator_2nd(
        std::size_t dim,
        double s_min,
        double s_max,
        PathEvaluator2nd evaluator,
        NoThrowTag)
    {
        return detail::expected_from(
            [&] { return from_evaluator_2nd(dim, s_min, s_max, std::move(evaluator)); });
    }

    inline Expected<Path> Path::from_evaluator_3rd(
        std::size_t dim,
        double s_min,
        double s_max,
        PathEvaluator3rd evaluator,
        NoThrowTag)
    {
        return detail::expected_from(
            [&] { return from_evaluator_3rd(dim, s_min, s_max, std::move(evaluator)); });
    }

    inline Expected<PathDerivatives> Path::evaluate_q(Span<const double> s, NoThrowTag) const
    {
        return detail::expected_from([&] { return evaluate_q(s); });
    }

    inline Expected<PathDerivatives> Path::evaluate_up_to_2nd(Span<const double> s, NoThrowTag) const
    {
        return detail::expected_from([&] { return evaluate_up_to_2nd(s); });
    }

    inline Expected<PathDerivatives> Path::evaluate_up_to_3rd(Span<const double> s, NoThrowTag) const
    {
        return detail::expected_from([&] { return evaluate_up_to_3rd(s); });
    }

} // namespace copp
