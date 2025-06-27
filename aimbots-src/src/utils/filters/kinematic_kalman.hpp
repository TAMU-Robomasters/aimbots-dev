#pragma once

#include <optional>

#include "utils/tools/common_types.hpp"
#include "utils/kinematics/kinematic_state_vector.hpp"
#include "utils/math/matrix_helpers.hpp"


using namespace src::Utils::MatrixHelper;

namespace src::Utils::Filters {

//TODO: consolidate kalman timeout and jetson communicator tracking target timeout
const int KIN_NUM_STATES = 3;

const uint32_t KALMAN_TIMEOUT_MILLISECONDS =  300;

class KinematicKalman {
public:
    KinematicKalman(
        Vector3f x0,
        const float (&P0)[KIN_NUM_STATES * KIN_NUM_STATES],
        const float (&H0)[KIN_NUM_STATES],
        const float R0,
        const float accelErrManeuver,
        const float accelErrStable
        );

    ~KinematicKalman() = default;

    /**
    * @param[in] dt: The time since the last measurement in seconds
    * @param[in] z_pos: The position of the target in meters
    * @param[in] motorAngularVelocity: The angular velocity of the motors in rads/s
    * @param[in] measurementNoiseFromCamera: The measurement noise from the camera in meters
    */
    void update(float dt, float z_pos, float motorAngularVelocity, float measurementNoiseFromCamera, float targetDistance);
    void predict(float dt, const float targetDistance);

    Vector3f getFuturePrediction(float dt) const;

private:
    //TODO: get rid of kinematic state vector class
    // KinematicStateVector z;
    Vector3f x;  // State Vector
    Matrix3f P;  // Covariance Matrix
    Vector3f H;  // Observation Matrix
    float R;  // Environment Noise Covariance Matrix
    const Vector3f x0;  // Initial State Vector
    const Matrix3f P0;  // Initial Covariance Matrix
    const float R0;  // Initial Measurement Noise
    const float accelErrManeuver;
    const float accelErrStable;

    mutable float accelErr; /*An assumption on how much the acceleration error will be
                     The larger the value, the closer the estimation will be to the position measurements*/

    float errResidual = 0.0f;
    float maneuverProbability = 0.0f;
    const float maneuverProbabilityMax = 1.0f;
    const float maneuverProbabilityStep = 0.15f;
    const float maneuverThreshold = 0.032225f;
    mutable float accelErrDistanceMultiplier = 20.0f;
    

    mutable uint32_t updateCount = 0;
    float lastMeasurement;
    tap::arch::MilliTimeout kalmanOfflineTimeout;
    mutable bool isFirstUpdate;



    Matrix3f stateSpaceMatrix(float dt) const;

    Matrix3f processNoiseCovarianceMatrix(float dt, const float targetDistance) const;

    float measurementNoise(float motorAngularVelocity, float measurementNoiseFromCamera);

    void resetKalman();
};

}  // namespace src::Utils::Filters