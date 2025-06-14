#pragma once

#include <optional>

#include "utils/tools/common_types.hpp"
#include "utils/kinematics/kinematic_state_vector.hpp"
#include "utils/math/matrix_helpers.hpp"

using namespace src::Utils::MatrixHelper;

namespace src::Utils::Filters {

const int KIN_NUM_STATES = 3;

const float KALMAN_TIMEOUT_MILLISECONDS =  500;

class KinematicKalman {
public:
    KinematicKalman(
        Vector3f x0,
        const float (&P0)[KIN_NUM_STATES * KIN_NUM_STATES],
        const float (&H0)[KIN_NUM_STATES],
        const float R0,
        const float AccelErr);

    ~KinematicKalman() = default;

    void update(float dt, float z_pos);
    void predict(float dt);

    Vector3f getFuturePrediction(float dt) const;

    Matrix3f stateSpaceMatrix(float dt) const;

    Matrix3f processNoiseCovarianceMatrix(float dt) const;

    void resetKalman();

private:
    KinematicStateVector z;

    Vector3f x;  // State Vector
    Matrix3f P;  // Covariance Matrix
    Vector3f H;  // Observation Matrix
    float R;  // Environment Noise Covariance Matrix
    const Vector3f x0;  // Initial State Vector
    const Matrix3f P0;  // Initial Covariance Matrix
    float AccelErr; /*An assumption on how much the acceleration error will be
                     The larger the value, the closer the estimation will be to the position measurements*/

    tap::arch::MilliTimeout kalmanOfflineTimeout;
    mutable bool isFirstUpdate;
};

}  // namespace src::Utils::Filters