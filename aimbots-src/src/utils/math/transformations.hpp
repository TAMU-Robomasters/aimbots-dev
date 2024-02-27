#pragma once

#include "utils/common_types.hpp"

namespace src::Utils::MatrixOperations {
    /*
    * Creates a 4x4 identity matrix
    */
    Matrix4f identityMatrix();

    /*
    * Rotate the matrix
    * @param M      Matrix to rotate
    * @param theta  angle to rotate about the x axis
    * @param phi    angle to roate about the y axis
    */
    Matrix4f rotate(Matrix4f M, float theta, float phi);

    /*
    * Rotate the matrix
    * @param M  Matrix to transform
    * @param p  Translation vector
    */
    Matrix4f translate(Matrix4f M, Vector3f p);
}