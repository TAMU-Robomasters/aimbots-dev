#pragma once
#include <utils/common_types.hpp>
#include <robots/robot-matricies/robot-matricies.hpp>
#include <utils/math/transform_setup.hpp>

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

/*
CV will be giving us (at the very least) an XYZ position vector in CAMERA SPACE
This class should be producing position, velocity, and acceleration vectors for the ENEMY in CHASSIS SPACE (and/or FIELD SPACE)
Additionally, this class should be producing position, velocity, and acceleration vectors for OUR ROBOT in FIELD SPACE (Though I'm not sure if this
class should be called EnemyDataConversion then)
*/
namespace src::Informants {

enum Axis : uint8_t { X_AXIS = 0, Y_AXIS = 1, Z_AXIS = 2 };

// for internal use
struct enemyTimedPosition {
    Matrix<float, 3, 1> position;
    uint32_t timestamp_uS;
};

// for output use
struct enemyTimedData {
    Matrix<float, 3, 1> position;
    Matrix<float, 3, 1> velocity;
    Matrix<float, 3, 1> acceleration;
    float timestamp_uS;  // time that 'best guess' was made
};

class EnemyDataConversion {
public:
    EnemyDataConversion(src::Drivers* drivers);

    /**
     * @brief Gets latest valid enemy target data from CV and stores it in a circular/ring buffer.
     * Should be called continuously.
     *
     */
    void updateEnemyInfo();

    /**
     * @brief Calculates best guess of current enemy position, velocity, and acceleration. Does not need to be called continuously.
     */
    enemyTimedData calculateBestGuess();
    // Matrix<float, 1, 3> const& getEnemyPosition() { return positionMatrix; }

private:
    src::Drivers* drivers;
    static const int BUFFER_SIZE = 10;  // prolly move this to constants at some point or something IDK
    static int size;                    // for internal use to tell the prediction thing how many valid entries we actually have

    // buffer for XYZ + timestamp
    Deque<enemyTimedPosition, BUFFER_SIZE> rawPositionBuffer;

    // we want to scrub/ignore data entries that are too outdated... so this access function only returns the last such and such entries within a
    // given time
    void getLastEntriesWithinTime(uint32_t time, enemyTimedPosition* validPositionArray);
};
}  // namespace src::Informants
