#pragma once
#include <utils/common_types.hpp>

#include "src/informants/vision/jetson_communicator.hpp"

/*
Convert enemy data (from CV) from camera space to chassis space (somehow)

CV gives us gimbal-relative angles and depth.
    Angles can be accessed from drivers->cvCommunicator.getVisionTargetAngles()
        For pitch, it would be drivers->cvCommunicator.getVisionTargetAngles()[0][src::Informants::vision::pitch];
        For yaw, it would be drivers->cvCommunicator.getVisionTargetAngles()[0][src::Informants::vision::yaw];
    Depth must be accessed directly from the Jetson messages as follows:
        drivers->cvCommunicator.getLastValidMessage().depth;
*/
/* CV gives us data in camera space-- the yaw angle, pitch angle, and the distance
   So.. this method should convert camera-space angles to camera-space 3D vector, then to chassis space 3D vector (using transformation matrices)
   ^^^ I'm not sure if this is still true 2022-10-19
   */

namespace src::Informants::vision {

enum Axis : uint8_t { X_AXIS = 0, Y_AXIS = 1, Z_AXIS = 2 };

class EnemyDataConversion {
public:
    EnemyDataConversion(src::Drivers* drivers);

    /**
     * @brief Gets latest valid enemy target angles and converts it to XYZ coordinates. Should be called continuously.
     *
     */
    void updateConversion();

    /**
     * @brief Returns enemy position as gimbal-relative XYZ coordinates.
     * The +Y-axis extrudes from the barrel (in direction of bullets), +Z-axis goes up from the barrel, and +X-axis goes right from the barrel.
     *
     * Information may or may not be out-of-date
     */
    Matrix<float, 1, 3> const& getEnemyPosition() { return positionMatrix; }

private:
    src::Drivers* drivers;
    Matrix<float, 1, 2> angularMatrix;   // "given"
    Matrix<float, 1, 3> positionMatrix;  // somehow math this up from angles
};
}  // namespace src::Informants::vision
