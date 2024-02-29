#include "utils/common_types.hpp"
#include "utils/math/matrix_helpers.hpp"

using namespace src::Utils::MatrixHelper; // for matrix helper functions

namespace src::Utils::MatrixOperations {

    Matrix3f identityMatrix(){
        float identity_array[9] = {1.0f, 0.0f, 0.0f,
                                   0.0f, 1.0f, 0.0f,
                                   0.0f, 0.0f, 1.0f};
        return Matrix3f(identity_array);            
    }

    Vector3f rotate(Vector3f v, float theta, float phi) {
        return rotationMatrix(theta, LinearAxis::X_AXIS, AngleUnit::Radians) * rotationMatrix(phi, LinearAxis::Y_AXIS, AngleUnit::Radians) * v;
    }

    Vector3f translate(Vector3f v, Vector3f p) {
        // float translatedMatrix[3] = {v[0] + p[0], v[1] + p[1], v[2] + p[2]};
        // return Vector3f(translatedMatrix);
        return v + p;
    }

}