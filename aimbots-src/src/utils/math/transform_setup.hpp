#pragma once

#include "utils/tools/common_types.hpp"

namespace src::Utils::MatrixHelper {

Matrix4f initTransform();
Matrix4f initTransform(Matrix3f R, Vector3f p);

// convert Position vector between 4x1 and 3x1
Vector4f homogenousCoordinateExtend(Vector3f P);
Vector3f homogenousCoordinateCrop(Vector4f P);

Matrix4f invertTransform(Matrix4f transform);

}  // namespace src::Utils::MatrixHelper