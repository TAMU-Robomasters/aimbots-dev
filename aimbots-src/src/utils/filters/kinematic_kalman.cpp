#include "kinematic_kalman.hpp"

using namespace src::Utils::MatrixHelper;

namespace src::Utils::Filters {

KinematicKalman::KinematicKalman(
    Vector3f x0,
    const float (&P0)[KIN_NUM_STATES * KIN_NUM_STATES],
    const float (&H0)[KIN_NUM_STATES * KIN_NUM_STATES],
    const float (&Q0)[KIN_NUM_STATES * KIN_NUM_STATES],
    const float (&R0)[KIN_NUM_STATES * KIN_NUM_STATES])
    : x(x0),
      P(P0),
      H(H0),
      Q(Q0),
      R(R0)  //
{}

void KinematicKalman::update(float dt, float zPos, std::optional<float> zVel, std::optional<float> zAcc) {
    predict(dt);

    z.updateFromPosition(zPos, dt);
    if (zVel.has_value()) {
        z.updateFromVelocity(zVel.value(), false, dt);
    }
    if (zAcc.has_value()) {
        z.updateFromAcceleration(zAcc.value(), false, dt);
    }

    Matrix3f PHt = P * H.asTransposed();

    Vector3f y = z.getStateVector() - H * x;
    Matrix3f S = H * PHt + R;
    Matrix3f K = PHt * asInverted(S);

    x = x + K * y;
    static const Matrix3f I = Matrix3f::identityMatrix();
    P = (I - K * H) * P;
}

void KinematicKalman::predict(float dt) {
    Matrix3f F = stateSpaceMatrix(dt);
    x = F * x;
    P = F * P * F.asTransposed() + Q;
}

Matrix3f FDisplay = Matrix3f::zeroMatrix();
Vector3f KinematicKalman::getFuturePrediction(float dt) const {
    // 未来最高! 未来最高! 未来最高!!!
    Matrix3f F = stateSpaceMatrix(dt);
    FDisplay = F;
    return F * x;
}

Matrix3f KinematicKalman::stateSpaceMatrix(float dt) const {
    // clang-format off
    float FHatArray[KIN_NUM_STATES * KIN_NUM_STATES] = {1, dt, 0.5 * pow2(dt),
                                                        0, 1,  dt,
                                                        0, 0,  1};
    // clang-format on
    return Matrix3f(FHatArray);
}

}  // namespace src::Utils::Filters