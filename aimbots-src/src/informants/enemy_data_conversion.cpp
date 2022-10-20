#include "enemy_data_conversion.hpp"

#include <drivers.hpp>

namespace src::Informants::vision {
EnemyDataConversion::EnemyDataConversion(src::Drivers* drivers) : drivers(drivers) {}

void EnemyDataConversion::updateConversion() {
    if (drivers->cvCommunicator.isJetsonOnline()) {
        // get angles and depth
        angularMatrix = drivers->cvCommunicator.getVisionTargetAngles();
        float targetPitchAngle = angularMatrix[0][src::Informants::vision::pitch];  // I still don't know what space these are in
        float targetYawAngle = angularMatrix[0][src::Informants::vision::yaw];
        float depth = drivers->cvCommunicator.getLastValidMessage().depth;
        // derive XYZ (in what space?!?!)
        float targetXCoord = depth * cos(targetPitchAngle) * sin(targetYawAngle);
        float targetYCoord = depth * cos(targetPitchAngle) * cos(targetYawAngle);
        float targetZCoord = depth * sin(targetPitchAngle);
        positionMatrix[0][X_AXIS] = targetXCoord;
        positionMatrix[0][Y_AXIS] = targetYCoord;
        positionMatrix[0][Z_AXIS] = targetZCoord;
    }
}
}  // namespace src::Informants::vision