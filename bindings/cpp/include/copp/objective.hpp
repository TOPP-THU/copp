#pragma once

#include <initializer_list>
#include <utility>
#include <vector>

#include "copp/core.hpp"

namespace copp
{

    /// Objective kinds accepted by COPP optimization solvers.
    ///
    /// The names mirror Rust/Python `CoppObjective`. COPP2 currently supports
    /// all four standard objectives: time, linear, thermal energy, and
    /// total-variation torque.
    enum class ObjectiveKind
    {
        Time = 0,
        Linear = 1,
        ThermalEnergy = 2,
        TotalVariationTorque = 3,
    };

    /// Owning objective descriptor used by C++ COPP solver problem objects.
    ///
    /// This class only stores weights and coefficient vectors. Validation is
    /// performed when a solver `Problem` is constructed, because the expected
    /// vector lengths depend on robot dimension and the chosen station interval.
    class Objective
    {
    public:
        /// Minimize traversal time.
        ///
        /// @param weight Nonnegative scalar weight.
        static Objective Time(double weight = 1.0)
        {
            return Objective(ObjectiveKind::Time, weight, {}, {}, {});
        }

        /// Add a linear torque objective.
        ///
        /// The objective uses coefficient vectors copied from `alpha` and
        /// `beta`. Expected lengths depend on the solver/problem dimension and
        /// are validated when constructing a COPP problem.
        static Objective Linear(
            double weight,
            Span<const double> alpha,
            Span<const double> beta)
        {
            return Objective(
                ObjectiveKind::Linear,
                weight,
                copy_span(alpha),
                copy_span(beta),
                {});
        }

        static Objective Linear(
            double weight,
            std::initializer_list<double> alpha,
            std::initializer_list<double> beta)
        {
            return Linear(weight, Span<const double>(alpha.begin(), alpha.size()),
                          Span<const double>(beta.begin(), beta.size()));
        }

        /// Penalize normalized squared torque / thermal energy.
        ///
        /// @param weight Nonnegative scalar weight.
        /// @param normalize Per-axis normalization vector copied into the
        /// objective descriptor.
        static Objective ThermalEnergy(double weight, Span<const double> normalize)
        {
            return Objective(
                ObjectiveKind::ThermalEnergy,
                weight,
                {},
                {},
                copy_span(normalize));
        }

        static Objective ThermalEnergy(double weight, std::initializer_list<double> normalize)
        {
            return ThermalEnergy(
                weight,
                Span<const double>(normalize.begin(), normalize.size()));
        }

        /// Penalize total variation of torque along the path.
        ///
        /// This objective is supported by SOCP solvers.
        static Objective TotalVariationTorque(double weight, Span<const double> normalize)
        {
            return Objective(
                ObjectiveKind::TotalVariationTorque,
                weight,
                {},
                {},
                copy_span(normalize));
        }

        static Objective TotalVariationTorque(double weight, std::initializer_list<double> normalize)
        {
            return TotalVariationTorque(
                weight,
                Span<const double>(normalize.begin(), normalize.size()));
        }

        ObjectiveKind kind() const noexcept { return kind_; }
        double weight() const noexcept { return weight_; }

        Span<const double> alpha() const noexcept
        {
            return Span<const double>(alpha_.data(), alpha_.size());
        }

        Span<const double> beta() const noexcept
        {
            return Span<const double>(beta_.data(), beta_.size());
        }

        Span<const double> normalize() const noexcept
        {
            return Span<const double>(normalize_.data(), normalize_.size());
        }

    private:
        Objective(
            ObjectiveKind kind,
            double weight,
            std::vector<double> alpha,
            std::vector<double> beta,
            std::vector<double> normalize)
            : kind_(kind),
              weight_(weight),
              alpha_(std::move(alpha)),
              beta_(std::move(beta)),
              normalize_(std::move(normalize)) {}

        static std::vector<double> copy_span(Span<const double> values)
        {
            if (values.empty())
            {
                return {};
            }
            return std::vector<double>(values.data(), values.data() + values.size());
        }

        ObjectiveKind kind_ = ObjectiveKind::Time;
        double weight_ = 1.0;
        std::vector<double> alpha_;
        std::vector<double> beta_;
        std::vector<double> normalize_;
    };

    /// Python-like objective factory namespace.
    ///
    /// Example:
    /// ```cpp
    /// auto objectives = std::vector<copp::Objective>{
    ///     copp::objective::Time(1.0),
    ///     copp::objective::ThermalEnergy(0.1, {1.0, 1.0}),
    /// };
    /// ```
    namespace objective
    {
        inline Objective Time(double weight = 1.0)
        {
            return Objective::Time(weight);
        }

        inline Objective Linear(
            double weight,
            Span<const double> alpha,
            Span<const double> beta)
        {
            return Objective::Linear(weight, alpha, beta);
        }

        inline Objective Linear(
            double weight,
            std::initializer_list<double> alpha,
            std::initializer_list<double> beta)
        {
            return Objective::Linear(weight, alpha, beta);
        }

        inline Objective ThermalEnergy(double weight, Span<const double> normalize)
        {
            return Objective::ThermalEnergy(weight, normalize);
        }

        inline Objective ThermalEnergy(double weight, std::initializer_list<double> normalize)
        {
            return Objective::ThermalEnergy(weight, normalize);
        }

        inline Objective TotalVariationTorque(double weight, Span<const double> normalize)
        {
            return Objective::TotalVariationTorque(weight, normalize);
        }

        inline Objective TotalVariationTorque(
            double weight,
            std::initializer_list<double> normalize)
        {
            return Objective::TotalVariationTorque(weight, normalize);
        }
    } // namespace objective

} // namespace copp
