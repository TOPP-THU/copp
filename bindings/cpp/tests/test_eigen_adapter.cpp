// Test purpose: verify the optional Eigen adapter compiles and exposes the
// intended zero-copy MatrixView/Span/Map helpers when COPP_CPP_WITH_EIGEN=ON.
// When the option is OFF, this file intentionally avoids including Eigen so
// default lightweight builds do not require the Eigen headers.

#include "copp/core.hpp"

#ifndef COPP_CPP_WITH_EIGEN
#define COPP_CPP_WITH_EIGEN 0
#endif

#if COPP_CPP_WITH_EIGEN
#include "copp/eigen.hpp"

#include <Eigen/Core>
#include <iostream>
#endif

int main()
{
#if COPP_CPP_WITH_EIGEN
    copp::eigen::ColMajorMatrix waypoints(2, 3);
    waypoints(0, 0) = 0.0;
    waypoints(1, 0) = 0.0;
    waypoints(0, 1) = 0.5;
    waypoints(1, 1) = 0.25;
    waypoints(0, 2) = 1.0;
    waypoints(1, 2) = 1.0;

    const auto view = copp::eigen::matrix_view(waypoints);
    if (view.rows() != 2 || view.cols() != 3 || view.leading_dim() != 2)
    {
        std::cerr << "unexpected column-major Eigen MatrixView shape\n";
        return 1;
    }

    copp::eigen::RowMajorMatrix row_major(2, 2);
    row_major(0, 0) = 1.0;
    row_major(0, 1) = 2.0;
    row_major(1, 0) = 3.0;
    row_major(1, 1) = 4.0;
    const auto row_view = copp::eigen::matrix_view(row_major);
    if (row_view.layout() != copp::MatrixLayout::RowMajor || row_view.leading_dim() != 2)
    {
        std::cerr << "unexpected row-major Eigen MatrixView metadata\n";
        return 1;
    }

    copp::eigen::Vector s(3);
    s(0) = 0.0;
    s(1) = 0.5;
    s(2) = 1.0;
    const auto span = copp::eigen::span(s);
    if (span.size() != 3 || span[2] != 1.0)
    {
        std::cerr << "unexpected Eigen vector span\n";
        return 1;
    }

    copp::Matrix matrix(2, 2);
    matrix(0, 0) = 1.0;
    matrix(1, 0) = 2.0;
    matrix(0, 1) = 3.0;
    matrix(1, 1) = 4.0;
    auto mapped = copp::eigen::map(matrix);
    mapped(0, 1) = 5.0;
    if (matrix(0, 1) != 5.0)
    {
        std::cerr << "Eigen map did not write back to copp::Matrix\n";
        return 1;
    }

    const auto copied = copp::eigen::copy(matrix);
    if (copied.rows() != 2 || copied.cols() != 2 || copied(0, 1) != 5.0)
    {
        std::cerr << "unexpected Eigen copy\n";
        return 1;
    }
#endif

    return 0;
}
