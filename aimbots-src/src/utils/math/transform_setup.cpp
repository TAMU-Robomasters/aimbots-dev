#include "utils/common_types.hpp"

namespace src::Utils::MatrixHelper {

// make Gimbal Transformation matrix
// combine rotation matricies
// update Transformation matricies

Matrix<float, 4, 4> initTransform() {
    // clang-format off
    float transform_array[16] = {1.0f, 0.0f, 0.0f, 0.0f,
                                 0.0f, 1.0f, 0.0f, 0.0f,
                                 0.0f, 0.0f, 1.0f, 0.0f,
                                 0.0f, 0.0f, 0.0f, 1.0f};
    // clang-format on
    return Matrix<float, 4, 4>(transform_array);
}

Matrix<float, 4, 4> initTransform(Matrix<float, 3, 3> R, Matrix<float, 3, 1> P) {
    // clang-format off
    float transform_array[16] = {
        R[0][0], R[0][1], R[0][2], P[0][0],
        R[1][0], R[1][1], R[1][2], P[1][0],
        R[2][0], R[2][1], R[2][2], P[2][0],
        0.0f,    0.0f,    0.0f,    1.0f};
    // clang-format on
    return Matrix<float, 4, 4>(transform_array);
}

Matrix<float, 4, 4> initTransform(float R[], float P[]) {
    float transform_array[16] =
        {R[0], R[1], R[2], P[0], R[3], R[4], R[5], P[1], R[6], R[7], R[8], P[2], 0.0f, 0.0f, 0.0f, 1.0f};
    return Matrix<float, 4, 4>(transform_array);
}
Matrix<float, 4, 1> P_crop_extend(Matrix<float, 3, 1> P) {
    float P_4by1[4] = {P[0][0], P[1][0], P[2][0], 1.0f};
    return Matrix<float, 4, 1>(P_4by1);
}

Matrix<float, 3, 1> P_crop_extend(Matrix<float, 4, 1> P) {
    float P_3by1[3] = {P[0][0], P[1][0], P[2][0]};
    return Matrix<float, 3, 1>(P_3by1);
}

}  // namespace src::Utils::MatrixHelper