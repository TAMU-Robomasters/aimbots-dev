#pragma once

#include "utils/common_types.hpp"

namespace src::utils::MatrixHelper {

// returns combined Rotation Matrix R13 = [R12]*[R23]
Matrix<float, 3, 3> merge_rotation_matrix(Matrix<float, 3, 3> R12, Matrix<float, 3, 3> R23);

Matrix<float, 4, 4> transform_matrix(Matrix<float, 3, 3> R, Matrix<float, 3, 1> P);
Matrix<float, 4, 4> transform_matrix(float R[], float P[]);

}  // namespace src::utils::MatrixHelper