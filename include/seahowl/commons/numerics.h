#pragma once

#include <Eigen/Dense>
#define EIGEN_MATRIXBASE_PLUGIN <chrono/core/ChMatrixEigenExtensions.h>
#define EIGEN_SPARSEMATRIX_PLUGIN <chrono/core/ChSparseMatrixEigenExtensions.h>
#include <Eigen/Sparse>

namespace seahowl {

/** @brief Pi constant. */
constexpr double PI = 3.14159265358979323846;

/** @brief Vector 3D. */
using Vector3d = Eigen::Vector3d;

/** @brief Vector 2D. */
using Vector2d = Eigen::Vector2d;

/** @brief Quaternion. */
using Quaternion = Eigen::Quaterniond;

using AngleAxisd = Eigen::AngleAxisd;

}  // namespace seahowl
