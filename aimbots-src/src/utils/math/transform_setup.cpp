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

Matrix4f initTransform(Matrix3f R, Vector3f P) {
    // clang-format off
    float transform_array[16] = {
        R[0][0], R[0][1], R[0][2], P[0],
        R[1][0], R[1][1], R[1][2], P[1],
        R[2][0], R[2][1], R[2][2], P[2],
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
    Matrix3f RNew = transform.subMatrix<3, 3>(3, 3).asTransposed();
    Matrix<float, 3, 1> r = transform.subMatrix<3, 1>(3, 0);

    Vector3f rNew = RNew * r * -1;

    Matrix4f invertedTransform = initTransform(RNew, rNew);
    return invertedTransform;
}

}  // namespace src::Utils::MatrixHelper