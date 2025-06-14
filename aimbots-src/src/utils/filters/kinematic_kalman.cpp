#include "kinematic_kalman.hpp"

using namespace src::Utils::MatrixHelper;

namespace src::Utils::Filters {

// watchable variables
int numOfResetsDisplay = 0;

KinematicKalman::KinematicKalman(
    Vector3f x0,
    const float (&P0)[KIN_NUM_STATES * KIN_NUM_STATES],
    const float (&H0)[KIN_NUM_STATES],
    const float R0,
    const float AccelErr)
    : x(x0),
      P(P0),
      H(H0),
      R(R0),
      x0(x0),
      P0(P0),
      AccelErr(AccelErr),
      kalmanOfflineTimeout(KALMAN_TIMEOUT_MILLISECONDS),
      isFirstUpdate(true)  //
{}

void KinematicKalman::update(float dt, float z_pos) {
    predict(dt);

    Vector3f PHt = P * H;

    float y = z_pos - H * x;
    float S = H * PHt + R;
    float invS = 1.0f / S;
    Vector3f K = PHt * invS;  

    x = x + K * y;
    static const Matrix3f I = Matrix3f::identityMatrix();
    P = (I - K.asMatrix() * H.asTransposedMatrix()) * P;
}

void KinematicKalman::predict(float dt) { // called every time we get a new measurement
    if (kalmanOfflineTimeout.isExpired()) resetKalman();
    kalmanOfflineTimeout.restart(KALMAN_TIMEOUT_MILLISECONDS); // reset timeout

    Matrix3f F = stateSpaceMatrix(dt);
    Matrix3f Q = processNoiseCovarianceMatrix(dt);
    x = F * x;
    P = F * P * F.asTransposed() + Q;
}

Matrix3f FDisplay = Matrix3f::zeroMatrix();

Vector3f KinematicKalman::getFuturePrediction(float dt) const {
    // 未来最高! 未来最高! 未来最高!!! huh?
    Matrix3f F = stateSpaceMatrix(dt);
    FDisplay = F;
    if (isFirstUpdate) {
        isFirstUpdate = false;
        return Vector3f(x.getX(), 0, 0);
    }
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

Matrix3f KinematicKalman::processNoiseCovarianceMatrix(float dt) const {
    // clang-format off
    float QArray[KIN_NUM_STATES * KIN_NUM_STATES] = {0.25 * pow4(dt), 0.5 * pow3(dt), 0.5 * pow2(dt),
                                                      0.5 * pow3(dt), pow2(dt),       dt,
                                                      0.5 * pow2(dt), dt,             1};
    // clang-format on
    return Matrix3f(QArray)*pow2(AccelErr);
}

void KinematicKalman::resetKalman() {
    x = Vector3f(0, 0, 0);
    P = P0;
    numOfResetsDisplay++;
}

}  // namespace src::Utils::Filters