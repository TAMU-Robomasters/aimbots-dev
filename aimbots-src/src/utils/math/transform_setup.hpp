#pragma once

#include "utils/common_types.hpp"

namespace src::Utils::MatrixHelper {

Matrix<float, 4, 4> initTransform();
Matrix<float, 4, 4> initTransform(Matrix<float, 3, 3> R, Vector3f P);
Matrix<float, 4, 4> initTransform(float R[], float P[]);

// convert Position vector between 4x1 and 3x1 (returns a matrix with the name of fucntion)
Matrix<float, 4, 1> P_crop_extend(Matrix<float, 3, 1> P);
Matrix<float, 3, 1> P_crop_extend(Matrix<float, 4, 1> P);

}  // namespace src::Utils::MatrixHelper