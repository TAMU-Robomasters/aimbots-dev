#include "utils/common_types.hpp"

namespace src::utils::MatrixHelper {

// make Gimbal Transformation matrix
// combine rotation matricies

// Rotation Merge
Matrix<float, 3, 3> merge_rotation_matrix(Matrix<float, 3, 3> R12, Matrix<float, 3, 3> R23) {
    // Return R13 = [R12]*[R23]
    return R12*R23;

    //Possibly change this to manually set up like this:
    /*
    float rotation_array[9] = {
        c,    -s,   0.0f,
        s,     c,   0.0f,
        0.0f, 0.0f, 1.0f};
    // clang-format on
    return Matrix<float, 3, 3>(rotation_array);
    */
}

Matrix<float, 4, 4> transform_matrix(Matrix<float, 3, 3> R_CG, Matrix<float, 3, 1> P_CG) {

}

// make  


}  // namespace src::utils::MatrixHelper