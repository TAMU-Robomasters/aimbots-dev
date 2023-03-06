#pragma once

#include "src/utils/common_types.hpp"

namespace src::Utils::MatrixHelper {

struct Transform3D {
    Matrix<float, 4, 4> transform;

    Transform3D() : transform(initTransform()) {}
    Transform3D(Matrix3f R, Vector3f P) : transform(initTransform(R, P)) {}

    Matrix<float, 4, 1> invert() { return invertTransform(transform); }
    void setInvert() { transform = invertTransform(transform); }
};

Matrix<float, 4, 4> initTransform();
Matrix<float, 4, 4> initTransform(Matrix3f R, Vector3f P);

// convert Position vector between 4x1 and 3x1 (returns a matrix with the name of fucntion)
Matrix<float, 4, 1> homogenousCoordinateExtend(Vector3f P);
Matrix<float, 3, 1> homogenousCoordinateCrop(Vector4f P);

Matrix<float, 4, 1> invertTransform(Matrix<float, 4, 4> transform);

}  // namespace src::Utils::MatrixHelper