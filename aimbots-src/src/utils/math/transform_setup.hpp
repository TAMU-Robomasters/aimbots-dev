#pragma once

#include "utils/common_types.hpp"

namespace src::Utils::MatrixHelper {

Matrix4f initTransform();
Matrix4f initTransform(Matrix3f R, Vector3f P);

// convert Position vector between 4x1 and 3x1 (returns a matrix with the name of fucntion)
Vector4f homogenousCoordinateExtend(Vector3f P);
Vector3f homogenousCoordinateCrop(Vector4f P);

Matrix4f invertTransform(Matrix4f transform);

struct Transform3D {
    Matrix4f transform;

    Transform3D() : transform(initTransform()) {}
    Transform3D(Matrix3f R, Vector3f P) : transform(initTransform(R, P)) {}

    Matrix4f invert() { return invertTransform(transform); }
    void setInvert() { transform = invertTransform(transform); }
};

}  // namespace src::Utils::MatrixHelper