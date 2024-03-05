#pragma once

#include "utils/common_types.hpp"

namespace src::Utils::MatrixOperations {
    /**
    * @return   3X3 identity matrix
    **/
    // Matrix3f identityMatrix(); WE DO NOT NEED THIS BECAUSE A PRE-BUILT IDENTITY MATRIX EXISTS ALREADY

    Matrix4f initialization(Matrix3f R, Vector3f p);

    /**
    * Rotate the matrix
    * @param v      Vector to rotate
    * @param theta  angle to rotate about the x axis, in radians
    * @param phi    angle to roate about the y axis, in radians
    * @return       rotated column vector
    **/
    Vector3f rotate(Vector3f v, float theta, float phi);

    /**
    * Translate the matrix
    * @param v  Vector to transform
    * @param p  Translation vector
    * @return   Translated column vector
    **/
    Vector3f translate(Vector3f v, Vector3f p);

    Matrix4f invertTransform(Matrix4f transform);
    
}