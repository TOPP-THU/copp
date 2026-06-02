#pragma once

#include <utility>
#include <vector>

#include "copp/core.hpp"

namespace copp
{

    /// Borrowed third-order TOPP/COPP profile view.
    ///
    /// `Profile3rdRef` mirrors Rust's `Topp3ProfileRef`: it borrows node-based
    /// `a(s) = dot{s}^2` and `b(s) = ddot{s}` arrays plus the effective
    /// stationary interval counts at the beginning and end. The referenced
    /// arrays are not copied and must outlive the call that consumes this view.
    ///
    /// The `slice` helpers use inclusive station indices, matching
    /// `IndexInterval`: `slice(i, j)` borrows nodes `i..=j` and adjusts
    /// stationary counts for the local profile.
    struct COPP_API Profile3rdRef
    {
        Span<const double> a;
        Span<const double> b;
        std::size_t num_stationary_start = 0;
        std::size_t num_stationary_end = 0;

        Profile3rdRef() = default;
        Profile3rdRef(Span<const double> a_values, Span<const double> b_values);
        Profile3rdRef(
            Span<const double> a_values,
            Span<const double> b_values,
            std::pair<std::size_t, std::size_t> num_stationary);
        Profile3rdRef(
            Span<const double> a_values,
            Span<const double> b_values,
            std::size_t stationary_start,
            std::size_t stationary_end);

        std::size_t len() const noexcept { return a.size(); }
        std::size_t size() const noexcept { return a.size(); }
        bool empty() const noexcept { return a.empty(); }
        std::pair<std::size_t, std::size_t> num_stationary() const noexcept
        {
            return {num_stationary_start, num_stationary_end};
        }

        /// Borrow a closed station subrange `idx_s_start..=idx_s_final`.
        ///
        /// @return A new non-owning profile view whose `a`/`b` spans point into
        /// the original arrays. Stationary-boundary counts are clipped to the
        /// local subrange.
        /// @throws copp::Error if the interval is empty, out of range, or would
        /// be entirely stationary.
        Profile3rdRef slice(std::size_t idx_s_start, std::size_t idx_s_final) const;

        /// Borrow one segment as the two-node profile `idx_s..=idx_s+1`.
        Profile3rdRef segment(std::size_t idx_s) const;
    };

    /// Owned third-order TOPP/COPP profile.
    ///
    /// `a` and `b` are both node-based and must have the same length as the
    /// station grid `s` used for interpolation. Public fields intentionally stay
    /// writable to match the lightweight Rust/Python storage style; constructors
    /// validate the initial shape, while direct later edits are the user's
    /// responsibility.
    struct COPP_API Profile3rd
    {
        std::vector<double> a;
        std::vector<double> b;
        std::size_t num_stationary_start = 0;
        std::size_t num_stationary_end = 0;

        Profile3rd() = default;
        Profile3rd(std::vector<double> a_values, std::vector<double> b_values);
        Profile3rd(
            std::vector<double> a_values,
            std::vector<double> b_values,
            std::pair<std::size_t, std::size_t> num_stationary);
        Profile3rd(
            std::vector<double> a_values,
            std::vector<double> b_values,
            std::size_t stationary_start,
            std::size_t stationary_end);

        std::size_t len() const noexcept { return a.size(); }
        std::size_t size() const noexcept { return a.size(); }
        bool empty() const noexcept { return a.empty(); }
        std::pair<std::size_t, std::size_t> num_stationary() const noexcept
        {
            return {num_stationary_start, num_stationary_end};
        }

        /// Borrow this owned profile without copying arrays.
        Profile3rdRef ref() const;

        /// Borrow a closed station subrange `idx_s_start..=idx_s_final`.
        Profile3rdRef slice(std::size_t idx_s_start, std::size_t idx_s_final) const;

        /// Borrow one segment as the two-node profile `idx_s..=idx_s+1`.
        Profile3rdRef segment(std::size_t idx_s) const;
    };

} // namespace copp

namespace copp::interpolation
{

    /// Cumulative time profile returned by `s_to_t_topp2`.
    ///
    /// `t_s[i]` is the time at station `s[i]`; `t_final` is equal to the last
    /// entry of `t_s` on success.
    struct TimeProfile
    {
        double t_final = 0.0;
        std::vector<double> t_s;
    };

    /// Options for generating a uniform `s(t)` sampling grid.
    ///
    /// C++17 does not have designated initializers, so this small struct keeps
    /// the uniform-grid API compact while remaining aggregate-initializable.
    struct TimeGridOptions
    {
        double t0 = 0.0;
        bool include_final = true;
    };

    /// Compute segment profile `b` from node profile `a`.
    ///
    /// For every segment `[s[k], s[k + 1]]`, this returns
    /// `b[k] = 0.5 * (a[k + 1] - a[k]) / (s[k + 1] - s[k])`.
    ///
    /// @param s Strictly increasing station grid of length `N`.
    /// @param a Node profile of length `N`.
    /// @return Segment acceleration profile of length `N - 1`.
    /// @throws copp::Error if lengths, station ordering, or finite-value checks fail.
    COPP_API std::vector<double> a_to_b_topp2(Span<const double> s, Span<const double> a);
    inline Expected<std::vector<double>> a_to_b_topp2(
        Span<const double> s,
        Span<const double> a,
        NoThrowTag)
    {
        return detail::expected_from([&] { return a_to_b_topp2(s, a); });
    }

    /// Convert a second-order path profile `a(s) = ds/dt squared` to `t(s)`.
    ///
    /// Input requirements mirror the Rust/Python APIs:
    ///
    /// - `s.size() >= 2`
    /// - `a.size() == s.size()`
    /// - `s` is strictly increasing
    /// - all values are finite
    /// - each segment has a positive finite speed denominator
    ///
    /// The default API throws `copp::Error` when validation or interpolation
    /// fails. The returned vector owns its data.
    ///
    /// @param s Strictly increasing station grid.
    /// @param a Node profile `a[k] = (ds/dt)^2`.
    /// @param t0 Initial time offset.
    /// @return Cumulative time samples `t_s` and final time.
    ///
    /// @code
    /// auto time = copp::interpolation::s_to_t_topp2(s, a);
    /// auto s_t = copp::interpolation::t_to_s_topp2_uniform(s, a, time.t_s, 1.0e-3);
    /// @endcode
    COPP_API TimeProfile s_to_t_topp2(Span<const double> s, Span<const double> a, double t0 = 0.0);
    inline Expected<TimeProfile> s_to_t_topp2(
        Span<const double> s,
        Span<const double> a,
        double t0,
        NoThrowTag)
    {
        return detail::expected_from([&] { return s_to_t_topp2(s, a, t0); });
    }

    inline Expected<TimeProfile> s_to_t_topp2(
        Span<const double> s,
        Span<const double> a,
        NoThrowTag tag)
    {
        return s_to_t_topp2(s, a, 0.0, tag);
    }

    /// Sample the inverse TOPP2 mapping `s(t)` on a uniform time grid.
    ///
    /// `t_s` is normally returned by `s_to_t_topp2`. Out-of-range generated
    /// samples are represented as `NaN` by the Rust core. If
    /// `options.include_final` is true, the final station is appended when the
    /// uniform grid does not land exactly on it.
    COPP_API std::vector<double> t_to_s_topp2_uniform(
        Span<const double> s,
        Span<const double> a,
        Span<const double> t_s,
        double dt,
        TimeGridOptions options = {});
    inline Expected<std::vector<double>> t_to_s_topp2_uniform(
        Span<const double> s,
        Span<const double> a,
        Span<const double> t_s,
        double dt,
        TimeGridOptions options,
        NoThrowTag)
    {
        return detail::expected_from([&] { return t_to_s_topp2_uniform(s, a, t_s, dt, options); });
    }

    inline Expected<std::vector<double>> t_to_s_topp2_uniform(
        Span<const double> s,
        Span<const double> a,
        Span<const double> t_s,
        double dt,
        NoThrowTag tag)
    {
        return t_to_s_topp2_uniform(s, a, t_s, dt, TimeGridOptions{}, tag);
    }

    /// Sample the inverse TOPP2 mapping `s(t)` at explicit query times.
    ///
    /// `t_sample` must be finite, non-empty, and strictly increasing. Values
    /// outside the range of `t_s` are returned as `NaN`.
    COPP_API std::vector<double> t_to_s_topp2_samples(
        Span<const double> s,
        Span<const double> a,
        Span<const double> t_s,
        Span<const double> t_sample);
    inline Expected<std::vector<double>> t_to_s_topp2_samples(
        Span<const double> s,
        Span<const double> a,
        Span<const double> t_s,
        Span<const double> t_sample,
        NoThrowTag)
    {
        return detail::expected_from([&] { return t_to_s_topp2_samples(s, a, t_s, t_sample); });
    }

    /// Convert a third-order profile to cumulative time `t(s)`.
    ///
    /// The public profile object owns `a` and `b`; the `Profile3rdRef` overload
    /// is the zero-copy advanced form used for slicing or external storage. No
    /// raw `(a, b, num_stationary)` overload is exposed publicly.
    ///
    /// @param s Strictly increasing station grid.
    /// @param profile Node-based third-order profile.
    /// @param t0 Initial time offset.
    /// @return Cumulative time samples and final time.
    ///
    /// @code
    /// copp::Profile3rd profile{a_values, b_values, {1, 1}};
    /// auto whole = copp::interpolation::s_to_t_topp3(s, profile);
    /// auto first_segment = copp::interpolation::s_to_t_topp3(
    ///     copp::Span<const double>(s.data(), 2),
    ///     profile.segment(0));
    /// @endcode
    COPP_API TimeProfile s_to_t_topp3(
        Span<const double> s,
        const Profile3rd &profile,
        double t0 = 0.0);
    COPP_API TimeProfile s_to_t_topp3(
        Span<const double> s,
        Profile3rdRef profile,
        double t0 = 0.0);
    inline Expected<TimeProfile> s_to_t_topp3(
        Span<const double> s,
        const Profile3rd &profile,
        double t0,
        NoThrowTag)
    {
        return detail::expected_from([&] { return s_to_t_topp3(s, profile, t0); });
    }

    inline Expected<TimeProfile> s_to_t_topp3(
        Span<const double> s,
        const Profile3rd &profile,
        NoThrowTag tag)
    {
        return s_to_t_topp3(s, profile, 0.0, tag);
    }

    inline Expected<TimeProfile> s_to_t_topp3(
        Span<const double> s,
        Profile3rdRef profile,
        double t0,
        NoThrowTag)
    {
        return detail::expected_from([&] { return s_to_t_topp3(s, profile, t0); });
    }

    inline Expected<TimeProfile> s_to_t_topp3(
        Span<const double> s,
        Profile3rdRef profile,
        NoThrowTag tag)
    {
        return s_to_t_topp3(s, profile, 0.0, tag);
    }

    /// Sample the inverse TOPP3 mapping `s(t)` on a uniform time grid.
    ///
    /// `t_s` is normally returned by `s_to_t_topp3`. Generated out-of-range
    /// samples are represented as `NaN` by the Rust core.
    COPP_API std::vector<double> t_to_s_topp3_uniform(
        Span<const double> s,
        const Profile3rd &profile,
        Span<const double> t_s,
        double dt,
        TimeGridOptions options = {});
    COPP_API std::vector<double> t_to_s_topp3_uniform(
        Span<const double> s,
        Profile3rdRef profile,
        Span<const double> t_s,
        double dt,
        TimeGridOptions options = {});
    inline Expected<std::vector<double>> t_to_s_topp3_uniform(
        Span<const double> s,
        const Profile3rd &profile,
        Span<const double> t_s,
        double dt,
        TimeGridOptions options,
        NoThrowTag)
    {
        return detail::expected_from(
            [&] { return t_to_s_topp3_uniform(s, profile, t_s, dt, options); });
    }

    inline Expected<std::vector<double>> t_to_s_topp3_uniform(
        Span<const double> s,
        const Profile3rd &profile,
        Span<const double> t_s,
        double dt,
        NoThrowTag tag)
    {
        return t_to_s_topp3_uniform(s, profile, t_s, dt, TimeGridOptions{}, tag);
    }

    inline Expected<std::vector<double>> t_to_s_topp3_uniform(
        Span<const double> s,
        Profile3rdRef profile,
        Span<const double> t_s,
        double dt,
        TimeGridOptions options,
        NoThrowTag)
    {
        return detail::expected_from(
            [&] { return t_to_s_topp3_uniform(s, profile, t_s, dt, options); });
    }

    inline Expected<std::vector<double>> t_to_s_topp3_uniform(
        Span<const double> s,
        Profile3rdRef profile,
        Span<const double> t_s,
        double dt,
        NoThrowTag tag)
    {
        return t_to_s_topp3_uniform(s, profile, t_s, dt, TimeGridOptions{}, tag);
    }

    /// Sample the inverse TOPP3 mapping `s(t)` at explicit query times.
    ///
    /// @param s Strictly increasing station grid.
    /// @param profile Node-based third-order profile.
    /// @param t_s Cumulative time samples returned by `s_to_t_topp3`.
    /// @param t_sample Explicit query times. Values outside the interpolation
    /// domain are returned as `NaN`, matching Rust/Python.
    /// @return Interpolated station samples `s(t_sample)`.
    COPP_API std::vector<double> t_to_s_topp3_samples(
        Span<const double> s,
        const Profile3rd &profile,
        Span<const double> t_s,
        Span<const double> t_sample);
    COPP_API std::vector<double> t_to_s_topp3_samples(
        Span<const double> s,
        Profile3rdRef profile,
        Span<const double> t_s,
        Span<const double> t_sample);
    inline Expected<std::vector<double>> t_to_s_topp3_samples(
        Span<const double> s,
        const Profile3rd &profile,
        Span<const double> t_s,
        Span<const double> t_sample,
        NoThrowTag)
    {
        return detail::expected_from([&] { return t_to_s_topp3_samples(s, profile, t_s, t_sample); });
    }

    inline Expected<std::vector<double>> t_to_s_topp3_samples(
        Span<const double> s,
        Profile3rdRef profile,
        Span<const double> t_s,
        Span<const double> t_sample,
        NoThrowTag)
    {
        return detail::expected_from([&] { return t_to_s_topp3_samples(s, profile, t_s, t_sample); });
    }

} // namespace copp::interpolation
