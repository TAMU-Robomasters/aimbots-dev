#pragma once
#include <utils/common_types.hpp>

#include "src/informants/vision/jetson_communicator.hpp"

/*
Convert enemy data (from CV) from camera space to chassis space (somehow)

CV gives us angles (chassis-relative, I'm assuming)
For depth.. consult Jetson messages?? For now, assuming depth is in imaginary units
*/

namespace src::Informants::vision {
class EnemyDataConversion {
public:
    Matrix<float, 1, 3> updateConversion();

    Matrix<float, 1, 3> getEnemyPosition();

private:
    Matrix<float, 1, 2> angularMatrix_chassisRelative;   // "given"
    Matrix<float,`1, 3> positionMatrix_chassisRelative;  // somehow math this up from angles
};
}  // namespace src::Informants::vision
