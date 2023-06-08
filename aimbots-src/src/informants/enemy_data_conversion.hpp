#pragma once
#include <vector>

#include <utils/math/transform_setup.hpp>

#include "utils/common_types.hpp"
#include "utils/filters/ema.hpp"

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
This class should be producing position, velocity, and acceleration vectors for the ENEMY in CHASSIS SPACE (and/or FIELD
SPACE) Additionally, this class should be producing position, velocity, and acceleration vectors for OUR ROBOT in FIELD SPACE
(Though I'm not sure if this class should be called EnemyDataConversion then)
*/
namespace src {
class Drivers;
}
namespace src::Informants {

namespace vision {
struct plateKinematicState;
}

// for internal use
struct EnemyTimedPosition {
    Vector3f position;
    uint32_t timestamp_uS;
};

class EnemyDataConversion {
public:
    EnemyDataConversion(src::Drivers* drivers);
    ~EnemyDataConversion() = default;
    /**
     * @brief Gets latest valid enemy target data from CV and stores it in a circular/ring buffer.
     * Should be called continuously.
     *
     */
    void updateEnemyInfo(Vector3f position, uint32_t frameCaptureDelay);

    /**
     * @brief Calculates best guess of current enemy position, velocity, and acceleration. Does not need to be called
     * continuously.
     */
    vision::plateKinematicState calculateBestGuess(int desired_finite_diff_accuracy = 3);

    /**
     * @brief Returns all of the enemy position entries within a certain amount of time (from the internal bounded deque)
     * Does not account for 'skips' in data...(ex: on-and-off jetson connection, robot on fire, etc..)
     *
     * @param time the maximum elapsed time for an entry to be valid
     * @return vector with enemyTimedPositions
     */
    std::vector<EnemyTimedPosition> getLastEntriesWithinTime(float time_seconds);

private:
    src::Drivers* drivers;
    static const int BUFFER_SIZE = 1;       // prolly move this to constants at some point or something IDK
    static constexpr float VALID_TIME = 5;  // max elapsed seconds before an enemy position entry is invalid

    // buffer for XYZ + timestamp
    Deque<EnemyTimedPosition, BUFFER_SIZE> rawPositionBuffer;
    bool prevCVValid, cvValid;

    EMAFilter XPositionFilter, YPositionFilter, ZPositionFilter;
};
}  // namespace src::Informants
