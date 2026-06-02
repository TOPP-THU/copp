#pragma once

#ifndef COPP_CPP_WITH_EIGEN
#define COPP_CPP_WITH_EIGEN 1
#endif

#if !COPP_CPP_WITH_EIGEN
#error "copp/eigen.hpp requires COPP_CPP_WITH_EIGEN=1"
#endif

#include <Eigen/Core>

#include "copp/core.hpp"

namespace copp::eigen
{

    /// Dynamic column-major Eigen matrix type matching `copp::Matrix` storage.
    using ColMajorMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>;
    /// Dynamic row-major Eigen matrix accepted by input adapters.
    using RowMajorMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    /// Dynamic Eigen vector accepted by `Span` adapters.
    using Vector = Eigen::Matrix<double, Eigen::Dynamic, 1>;

    /// Borrow a column-major Eigen matrix as a COPP matrix view.
    ///
    /// @param matrix Eigen reference whose inner stride must be one.
    /// @return Non-owning `MatrixView`; the Eigen object must outlive the COPP call.
    /// @throws copp::Error when the view is not contiguous along the inner dimension.
    ///
    /// @code
    /// Eigen::MatrixXd waypoints(2, 3);
    /// auto path = copp::Path::from_waypoints(copp::eigen::matrix_view(waypoints));
    /// @endcode
    inline MatrixView matrix_view(const Eigen::Ref<const ColMajorMatrix> &matrix)
    {
        if (matrix.innerStride() != 1)
        {
            throw Error(Status::invalid_input, "Eigen column-major MatrixView requires innerStride == 1");
        }
        return MatrixView::strided_column_major(
            matrix.data(),
            static_cast<std::size_t>(matrix.rows()),
            static_cast<std::size_t>(matrix.cols()),
            static_cast<std::size_t>(matrix.outerStride()));
    }

    /// Borrow a row-major Eigen matrix as a COPP matrix view.
    ///
    /// Row-major views are useful for user-owned data tables. Public COPP
    /// functions either interpret the row-major descriptor directly or copy once
    /// before entering Rust, depending on the operation.
    inline MatrixView matrix_view(const Eigen::Ref<const RowMajorMatrix> &matrix)
    {
        if (matrix.innerStride() != 1)
        {
            throw Error(Status::invalid_input, "Eigen row-major MatrixView requires innerStride == 1");
        }
        return MatrixView(
            matrix.data(),
            static_cast<std::size_t>(matrix.rows()),
            static_cast<std::size_t>(matrix.cols()),
            MatrixLayout::RowMajor,
            static_cast<std::size_t>(matrix.outerStride()));
    }

    /// Borrow a contiguous Eigen vector as `Span<const double>`.
    inline Span<const double> span(const Eigen::Ref<const Vector> &vector)
    {
        if (vector.innerStride() != 1)
        {
            throw Error(Status::invalid_input, "Eigen vector Span requires innerStride == 1");
        }
        return Span<const double>(vector.data(), static_cast<std::size_t>(vector.size()));
    }

    /// Map a mutable COPP matrix into Eigen without copying.
    ///
    /// The returned `Eigen::Map` borrows `matrix`; do not use it after the
    /// matrix is destroyed or reallocated.
    inline Eigen::Map<ColMajorMatrix> map(Matrix &matrix)
    {
        return Eigen::Map<ColMajorMatrix>(
            matrix.data(),
            static_cast<Eigen::Index>(matrix.rows()),
            static_cast<Eigen::Index>(matrix.cols()));
    }

    /// Map a const COPP matrix into Eigen without copying.
    inline Eigen::Map<const ColMajorMatrix> map(const Matrix &matrix)
    {
        return Eigen::Map<const ColMajorMatrix>(
            matrix.data(),
            static_cast<Eigen::Index>(matrix.rows()),
            static_cast<Eigen::Index>(matrix.cols()));
    }

    /// Copy a COPP matrix into an owning Eigen column-major matrix.
    inline ColMajorMatrix copy(const Matrix &matrix)
    {
        return map(matrix);
    }

} // namespace copp::eigen
