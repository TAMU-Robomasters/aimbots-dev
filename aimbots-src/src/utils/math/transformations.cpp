#include "utils/common_types.hpp"
#include "utils/math/matrix_helpers.hpp"

using namespace src::Utils::MatrixHelper;// for matrix helper functions

namespace src::Utils::MatrixOperations {


    Vector3f rotate(Vector3f M, float theta, float phi) {

        return rotationMatrix(theta, LinearAxis::X_AXIS, AngleUnit::Radians) * rotationMatrix(phi, LinearAxis::Y_AXIS, AngleUnit::Radians) * M;

    }

}