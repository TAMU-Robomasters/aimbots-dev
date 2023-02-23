#pragma once

#include "utils/common_types.hpp"

namespace src::Utils::MatrixHelper {

// returns combined Rotation Matrix R13 = [R12]*[R23]
Matrix<float, 3, 3> merge_rotation_matrix(Matrix<float, 3, 3> R12, Matrix<float, 3, 3> R23);

Matrix<float, 4, 4> transform_matrix(Matrix<float, 3, 3> R, Matrix<float, 3, 1> P);
Matrix<float, 4, 4> transform_matrix(float R[], float P[]);

// convert Position vector between 4x1 and 3x1 (returns a matrix with the name of fucntion)
Matrix<float, 4, 1> P_crop_extend(Matrix<float, 3, 1> P);
Matrix<float, 3, 1> P_crop_extend(Matrix<float, 4, 1> P);

}  // namespace src::Utils::MatrixHelper