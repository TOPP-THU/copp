#include "copp/interpolation.hpp"

// Interpolation facade implementation.
//
// Rust owns the numerical routines for TOPP2/TOPP3 time conversion. The C++
// layer validates the lightweight `Profile3rd` view contract, forwards borrowed
// slices through cxx, and copies returned vectors so public results remain
// ordinary value types.

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "rust/cxx.h"
#include "copp/src/ffi/cpp/mod.rs.h"

namespace
{

    rust::Slice<const double> to_rust_slice(copp::Span<const double> values) noexcept
    {
        return rust::Slice<const double>(values.data(), values.size());
    }

    std::string to_std_string(const rust::String &value)
    {
        return std::string(value.data(), value.size());
    }

    std::string to_std_string(const rust::Error &error)
    {
        return std::string(error.what());
    }

    std::vector<double> copy_vec(const rust::Box<copp::bridge::VecF64Result> &values)
    {
        const auto slice = values->values();
        return std::vector<double>(slice.begin(), slice.end());
    }

    void check_profile_lengths(const char *function_name, copp::Span<const double> a, copp::Span<const double> b)
    {
        if (a.size() != b.size())
        {
            throw copp::Error(
                copp::Status::invalid_input,
                std::string(function_name) + ": `a.size()` and `b.size()` must match");
        }
    }

    void check_stationary_counts(const char *function_name, std::size_t len, std::size_t start, std::size_t end)
    {
        const bool overflow = start > static_cast<std::size_t>(-1) - end ||
                              start + end > static_cast<std::size_t>(-1) - 2;
        if (overflow || len < start + end + 2)
        {
            throw copp::Error(
                copp::Status::invalid_input,
                std::string(function_name) +
                    ": stationary counts must leave at least one non-stationary interval");
        }
    }

    copp::Span<const double> subspan(copp::Span<const double> values, std::size_t offset, std::size_t length) noexcept
    {
        return copp::Span<const double>(values.data() + offset, length);
    }

    copp::Profile3rdRef checked_profile_ref(const char *function_name, copp::Profile3rdRef profile)
    {
        check_profile_lengths(function_name, profile.a, profile.b);
        check_stationary_counts(
            function_name,
            profile.a.size(),
            profile.num_stationary_start,
            profile.num_stationary_end);
        return profile;
    }

} // namespace

namespace copp
{

    Profile3rdRef::Profile3rdRef(Span<const double> a_values, Span<const double> b_values)
        : Profile3rdRef(a_values, b_values, 0, 0) {}

    Profile3rdRef::Profile3rdRef(
        Span<const double> a_values,
        Span<const double> b_values,
        std::pair<std::size_t, std::size_t> num_stationary)
        : Profile3rdRef(a_values, b_values, num_stationary.first, num_stationary.second) {}

    Profile3rdRef::Profile3rdRef(
        Span<const double> a_values,
        Span<const double> b_values,
        std::size_t stationary_start,
        std::size_t stationary_end)
        : a(a_values),
          b(b_values),
          num_stationary_start(stationary_start),
          num_stationary_end(stationary_end)
    {
        check_profile_lengths("Profile3rdRef", a, b);
    }

    Profile3rdRef Profile3rdRef::slice(std::size_t idx_s_start, std::size_t idx_s_final) const
    {
        check_profile_lengths("Profile3rdRef::slice", a, b);
        check_stationary_counts(
            "Profile3rdRef::slice",
            a.size(),
            num_stationary_start,
            num_stationary_end);
        if (idx_s_start >= idx_s_final)
        {
            throw Error(
                Status::invalid_input,
                "Profile3rdRef::slice requires `idx_s_start < idx_s_final`");
        }
        if (idx_s_final >= a.size())
        {
            throw Error(
                Status::invalid_input,
                "Profile3rdRef::slice index is out of range");
        }

        const std::size_t total_intervals = a.size() - 1;
        const std::size_t local_intervals = idx_s_final - idx_s_start;
        const std::size_t local_start = idx_s_start < num_stationary_start
                                            ? std::min(idx_s_final, num_stationary_start) - idx_s_start
                                            : 0;

        const std::size_t tail_begin = total_intervals - num_stationary_end;
        const std::size_t local_end = idx_s_final > tail_begin
                                          ? idx_s_final - std::max(idx_s_start, tail_begin)
                                          : 0;

        if (local_start + local_end >= local_intervals)
        {
            throw Error(
                Status::invalid_input,
                "Profile3rdRef::slice must leave at least one non-stationary interval");
        }

        return Profile3rdRef(
            subspan(a, idx_s_start, idx_s_final - idx_s_start + 1),
            subspan(b, idx_s_start, idx_s_final - idx_s_start + 1),
            local_start,
            local_end);
    }

    Profile3rdRef Profile3rdRef::segment(std::size_t idx_s) const
    {
        return slice(idx_s, idx_s + 1);
    }

    Profile3rd::Profile3rd(std::vector<double> a_values, std::vector<double> b_values)
        : Profile3rd(std::move(a_values), std::move(b_values), 0, 0) {}

    Profile3rd::Profile3rd(
        std::vector<double> a_values,
        std::vector<double> b_values,
        std::pair<std::size_t, std::size_t> num_stationary)
        : Profile3rd(
              std::move(a_values),
              std::move(b_values),
              num_stationary.first,
              num_stationary.second) {}

    Profile3rd::Profile3rd(
        std::vector<double> a_values,
        std::vector<double> b_values,
        std::size_t stationary_start,
        std::size_t stationary_end)
        : a(std::move(a_values)),
          b(std::move(b_values)),
          num_stationary_start(stationary_start),
          num_stationary_end(stationary_end)
    {
        check_profile_lengths("Profile3rd", Span<const double>(a), Span<const double>(b));
    }

    Profile3rdRef Profile3rd::ref() const
    {
        return Profile3rdRef{
            Span<const double>(a),
            Span<const double>(b),
            num_stationary_start,
            num_stationary_end,
        };
    }

    Profile3rdRef Profile3rd::slice(std::size_t idx_s_start, std::size_t idx_s_final) const
    {
        return ref().slice(idx_s_start, idx_s_final);
    }

    Profile3rdRef Profile3rd::segment(std::size_t idx_s) const
    {
        return ref().segment(idx_s);
    }

} // namespace copp

namespace copp::interpolation
{

    std::vector<double> a_to_b_topp2(Span<const double> s, Span<const double> a)
    {
        try
        {
            return copy_vec(::copp::bridge::a_to_b_topp2(to_rust_slice(s), to_rust_slice(a)));
        }
        catch (const rust::Error &error)
        {
            throw Error(Status::invalid_input, to_std_string(error));
        }
    }

    TimeProfile s_to_t_topp2(Span<const double> s, Span<const double> a, double t0)
    {
        auto result = ::copp::bridge::s_to_t_topp2(to_rust_slice(s), to_rust_slice(a), t0);
        if (!result->ok())
        {
            throw Error(Status::invalid_input, to_std_string(result->message()));
        }

        auto t_s = result->t_s();
        return TimeProfile{
            result->t_final(),
            std::vector<double>(t_s.begin(), t_s.end()),
        };
    }

    std::vector<double> t_to_s_topp2_uniform(
        Span<const double> s,
        Span<const double> a,
        Span<const double> t_s,
        double dt,
        TimeGridOptions options)
    {
        try
        {
            return copy_vec(::copp::bridge::t_to_s_topp2_uniform(
                to_rust_slice(s),
                to_rust_slice(a),
                to_rust_slice(t_s),
                options.t0,
                dt,
                options.include_final));
        }
        catch (const rust::Error &error)
        {
            throw Error(Status::invalid_input, to_std_string(error));
        }
    }

    std::vector<double> t_to_s_topp2_samples(
        Span<const double> s,
        Span<const double> a,
        Span<const double> t_s,
        Span<const double> t_sample)
    {
        try
        {
            return copy_vec(::copp::bridge::t_to_s_topp2_samples(
                to_rust_slice(s),
                to_rust_slice(a),
                to_rust_slice(t_s),
                to_rust_slice(t_sample)));
        }
        catch (const rust::Error &error)
        {
            throw Error(Status::invalid_input, to_std_string(error));
        }
    }

    TimeProfile s_to_t_topp3(Span<const double> s, const Profile3rd &profile, double t0)
    {
        return s_to_t_topp3(s, profile.ref(), t0);
    }

    TimeProfile s_to_t_topp3(Span<const double> s, Profile3rdRef profile, double t0)
    {
        profile = checked_profile_ref("s_to_t_topp3", profile);
        auto result = ::copp::bridge::s_to_t_topp3(
            to_rust_slice(s),
            to_rust_slice(profile.a),
            to_rust_slice(profile.b),
            profile.num_stationary_start,
            profile.num_stationary_end,
            t0);
        if (!result->ok())
        {
            throw Error(Status::invalid_input, to_std_string(result->message()));
        }

        auto t_s = result->t_s();
        return TimeProfile{
            result->t_final(),
            std::vector<double>(t_s.begin(), t_s.end()),
        };
    }

    std::vector<double> t_to_s_topp3_uniform(
        Span<const double> s,
        const Profile3rd &profile,
        Span<const double> t_s,
        double dt,
        TimeGridOptions options)
    {
        return t_to_s_topp3_uniform(s, profile.ref(), t_s, dt, options);
    }

    std::vector<double> t_to_s_topp3_uniform(
        Span<const double> s,
        Profile3rdRef profile,
        Span<const double> t_s,
        double dt,
        TimeGridOptions options)
    {
        profile = checked_profile_ref("t_to_s_topp3_uniform", profile);
        try
        {
            return copy_vec(::copp::bridge::t_to_s_topp3_uniform(
                to_rust_slice(s),
                to_rust_slice(profile.a),
                to_rust_slice(profile.b),
                profile.num_stationary_start,
                profile.num_stationary_end,
                to_rust_slice(t_s),
                options.t0,
                dt,
                options.include_final));
        }
        catch (const rust::Error &error)
        {
            throw Error(Status::invalid_input, to_std_string(error));
        }
    }

    std::vector<double> t_to_s_topp3_samples(
        Span<const double> s,
        const Profile3rd &profile,
        Span<const double> t_s,
        Span<const double> t_sample)
    {
        return t_to_s_topp3_samples(s, profile.ref(), t_s, t_sample);
    }

    std::vector<double> t_to_s_topp3_samples(
        Span<const double> s,
        Profile3rdRef profile,
        Span<const double> t_s,
        Span<const double> t_sample)
    {
        profile = checked_profile_ref("t_to_s_topp3_samples", profile);
        try
        {
            return copy_vec(::copp::bridge::t_to_s_topp3_samples(
                to_rust_slice(s),
                to_rust_slice(profile.a),
                to_rust_slice(profile.b),
                profile.num_stationary_start,
                profile.num_stationary_end,
                to_rust_slice(t_s),
                to_rust_slice(t_sample)));
        }
        catch (const rust::Error &error)
        {
            throw Error(Status::invalid_input, to_std_string(error));
        }
    }

} // namespace copp::interpolation
