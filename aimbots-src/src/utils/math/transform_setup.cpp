#include "utils/common_types.hpp"

namespace src::Utils::MatrixHelper {

// make Gimbal Transformation matrix
// combine rotation matricies
// update Transformation matricies

Matrix4f initTransform() {
    // clang-format off
    float transform_array[16] = {1.0f, 0.0f, 0.0f, 0.0f,
                                 0.0f, 1.0f, 0.0f, 0.0f,
                                 0.0f, 0.0f, 1.0f, 0.0f,
                                 0.0f, 0.0f, 0.0f, 1.0f};
    // clang-format on
    return Matrix4f(transform_array);
}

Matrix4f initTransform(Matrix3f R, Vector3f p) {
    //      _____________           ______
    //     | r11 r12 r13 |          | px |
    // R = | r21 r22 r23 |      p = | py |
    //     |_r31_r32_r33_|          |_pz_|
    //      ____
    // T = |R p |
    //     |0_1_|
    // clang-format off
    float transform_array[16] = {
        R[0][0], R[0][1], R[0][2], p[0],
        R[1][0], R[1][1], R[1][2], p[1],
        R[2][0], R[2][1], R[2][2], p[2],
        0.0f,    0.0f,    0.0f,    1.0f};
    // clang-format on
    return Matrix4f(transform_array);
}

Vector4f homogenousCoordinateExtend(Vector3f origin) {
    float arr[4] = {origin.getX(), origin.getY(), origin.getZ(), 1.0f};
    return Vector4f(arr);
}

Vector3f homogenousCoordinateCrop(Vector4f origin) {
    float arr[3] = {origin.getX(), origin.getY(), origin.getZ()};
    return Vector3f(arr);
}

Matrix4f invertTransform(Matrix4f transform) {
    //  ______           _____________
    // | R  r | invert  | R^T -R^T * r|
    // |_0__1_| ----->  |_0_______1___|
    //
    // clang-format off
    float transposedRotationArray[9] = {transform[0][0], transform[1][0], transform[2][0],
                                        transform[0][1], transform[1][1], transform[2][1],
                                        transform[0][2], transform[1][2], transform[2][2]};
    // clang-format on
    Matrix3f RNew = Matrix3f(transposedRotationArray);
    Vector3f r = Vector3f{transform[0][3], transform[1][3], transform[2][3]};

    Vector3f rNew = RNew * r * -1;

    Matrix4f invertedTransform = initTransform(RNew, rNew);
    return invertedTransform;
}

}  // namespace src::Utils::MatrixHelper