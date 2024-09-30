#pragma once

#include <optional>

#include "utils/tools/common_types.hpp"
#include "utils/kinematics/kinematic_state_vector.hpp"
#include "utils/math/matrix_helpers.hpp"

using namespace src::Utils::MatrixHelper;

namespace src::Utils::Filters {

const int KIN_NUM_STATES = 3;

class KinematicKalman {
public:
    KinematicKalman(
        Vector3f x0,
        const float (&P0)[KIN_NUM_STATES * KIN_NUM_STATES],
        const float (&H0)[KIN_NUM_STATES * KIN_NUM_STATES],
        const float (&Q0)[KIN_NUM_STATES * KIN_NUM_STATES],
        const float (&R0)[KIN_NUM_STATES * KIN_NUM_STATES]);

    ~KinematicKalman() = default;

    void update(float dt, float zPos, std::optional<float> zVel = std::nullopt, std::optional<float> zAcc = std::nullopt);

    void predict(float dt);

    Vector3f getFuturePrediction(float dt) const;

    Matrix3f stateSpaceMatrix(float dt) const;

private:
    KinematicStateVector z;

    Vector3f x;  // State Vector
    Matrix3f P;  // Covariance Matrix

    Matrix3f H;  // Observation Matrix

    Matrix3f Q;  // System Noise Covariance Matrix
    Matrix3f R;  // Environment Noise Covariance Matrix
};

}  // namespace src::Utils::Filters