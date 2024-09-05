#include "utils/tools/common_types.hpp"
#include "utils/math/matrix_helpers.hpp"

using namespace src::Utils::MatrixHelper;  // for matrix helper functions

namespace src::Utils::MatrixOperations {

// Matrix3f identityMatrix(){
//     float identity_array[9] = {1.0f, 0.0f, 0.0f,
//                                0.0f, 1.0f, 0.0f,
//                                0.0f, 0.0f, 1.0f};
//     return Matrix3f(identity_array);
// }

Matrix4f initialization(Matrix3f R, Vector3f p) {
    //      _____________           ______
    //     | r11 r12 r13 |          | px |
    // R = | r21 r22 r23 |      p = | py |
    //     |_r31_r32_r33_|          |_pz_|
    //      ____
    // T = |R p |
    //     |0_1_|
    // clang-format off
        float transformArray[16] = {
            R[0][0], R[0][1], R[0][2], p[0],
            R[1][0], R[1][1], R[1][2], p[1],
            R[2][0], R[2][1], R[2][2], p[2],
            0.0f,    0.0f,    0.0f,    1.0f};
    // clang-format on
    return Matrix4f(transformArray);
}

Vector3f rotate(Vector3f v, float theta, float phi) {
    return rotationMatrix(theta, LinearAxis::X_AXIS, AngleUnit::Radians) *
           rotationMatrix(phi, LinearAxis::Y_AXIS, AngleUnit::Radians) * v;
}

Vector3f translate(Vector3f v, Vector3f p) {
    // float translatedMatrix[3] = {v[0] + p[0], v[1] + p[1], v[2] + p[2]};
    // return Vector3f(translatedMatrix);
    return v + p;
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

    Matrix4f invertedTransform = initialization(RNew, rNew);
    return invertedTransform;
}

Vector4f homogenousCoordinateExtend(Vector3f origin) {
    float arr[4] = {origin.getX(), origin.getY(), origin.getZ(), 1.0f};
    return Vector4f(arr);
}

Vector3f homogenousCoordinateCrop(Vector4f origin) {
    float arr[3] = {origin.getX(), origin.getY(), origin.getZ()};
    return Vector3f(arr);
}

}  // namespace src::Utils::MatrixOperations