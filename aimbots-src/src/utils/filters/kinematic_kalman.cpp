#include "kinematic_kalman.hpp"

using namespace src::Utils::MatrixHelper;

namespace src::Utils::Filters {

// watchable variables
int numOfResetsDisplay = 0;
Matrix3f FDisplay = Matrix3f::zeroMatrix();
float errResidualDisplay = 0;
float accelErrDisplay = 0;
float accelErrDistanceMultiplierDebug = 0;
bool updateAccelErrDistanceMultiplierDebug = 0;

uint32_t timeoutDisplay = 0;
uint32_t counterDisplay = 0;

KinematicKalman::KinematicKalman(
    Vector3f x0,
    const float (&P0)[KIN_NUM_STATES * KIN_NUM_STATES],
    const float (&H0)[KIN_NUM_STATES],
    const float R0,
    const float accelErrManeuver,
    const float accelErrStable)
    : x(x0),
      P(P0),
      H(H0),
      R(R0),
      x0(x0),
      P0(P0),
      R0(R0),
      accelErrManeuver(accelErrManeuver),
      accelErrStable(accelErrStable),
      kalmanOfflineTimeout(KALMAN_TIMEOUT_MILLISECONDS),
      isFirstUpdate(true)  //
{}

//TODO: add pitch velocity
void KinematicKalman::update(float dt, float z_pos, float motorAngularVelocity, float measurementNoiseFromCamera, float targetDistance) { // called every time we get a new measurement
    if (kalmanOfflineTimeout.isExpired()) {
        resetKalman();
        updateCount = 0;
    }
    kalmanOfflineTimeout.restart(KALMAN_TIMEOUT_MILLISECONDS); // reset timeout
    updateCount++;
    
    predict(dt, targetDistance);

    lastMeasurement = z_pos; 

    Vector3f PHt = P * H;

    R = measurementNoise(motorAngularVelocity, measurementNoiseFromCamera);

    float y = z_pos - H * x; // dot product
    float S = H * PHt + R; 

    float invS = 1.0f / S;
    Vector3f K = PHt * invS;  
    errResidual = pow2(y)*invS;
    errResidualDisplay = errResidual;

    if (errResidual > maneuverThreshold) {
        maneuverProbability += maneuverProbabilityStep;
        if (maneuverProbability > maneuverProbabilityMax) {
            maneuverProbability = maneuverProbabilityMax;
        }
    } else {
        maneuverProbability = 0.0f;
    }

    x = x + K * y;
    static const Matrix3f I = Matrix3f::identityMatrix();
    P = (I - K.asMatrix() * H.asTransposedMatrix()) * P;
}

void KinematicKalman::predict(float dt, const float targetDistance) {
    Matrix3f F = stateSpaceMatrix(dt);
    Matrix3f Q = processNoiseCovarianceMatrix(dt, targetDistance);
    x = F * x;
    P = F * P * F.asTransposed() + Q;
}
//TODO: implement finite differences for inital estimate fix P0
//TODO: implement that^ so that the kf can quickly converge at far distances
// implement divide by zero check

Vector3f KinematicKalman::getFuturePrediction(float dt) const { // called every time in main loop
    timeoutDisplay = kalmanOfflineTimeout.timeRemaining();
    counterDisplay++;
    // 未来最高! 未来最高! 未来最高!!!
    if (kalmanOfflineTimeout.isExpired()) return Vector3f(lastMeasurement, 0, 0);
    Matrix3f F = stateSpaceMatrix(dt);
    FDisplay = F;
    if (updateCount <= 3) {
        isFirstUpdate = false;
        return Vector3f(lastMeasurement, 0, 0);
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

Matrix3f KinematicKalman::processNoiseCovarianceMatrix(float dt, const float targetDistance) const {
    float QArray[KIN_NUM_STATES * KIN_NUM_STATES] = {0.25 * pow4(dt), 0.5 * pow3(dt), 0.5 * pow2(dt),
                                                      0.5 * pow3(dt), pow2(dt),       dt,
                                                      0.5 * pow2(dt), dt,             1};
    accelErr = maneuverProbability * accelErrManeuver + (1.0f - maneuverProbability) * accelErrStable;

    if (updateAccelErrDistanceMultiplierDebug) {
    accelErrDistanceMultiplier = accelErrDistanceMultiplierDebug;
        updateAccelErrDistanceMultiplierDebug = false;
    }
    accelErr /= (accelErrDistanceMultiplier * targetDistance);
    accelErrDisplay = accelErr;
    return Matrix3f(QArray)*pow2(accelErr);
}

float KinematicKalman::measurementNoise(float motorAngularVelocity, float measurementNoiseFromCamera) {
    //measurement noise is proportional to the motor angular velocity
    float alpha = 0.0429718; // gotten from very rough empirical testing // 0.0429718 
    float measurementNoise = measurementNoiseFromCamera + alpha * motorAngularVelocity; 
    return pow2(measurementNoise); // variance
}

void KinematicKalman::resetKalman() {
    x = x0;
    P = P0;
    numOfResetsDisplay++;
}

}  // namespace src::Utils::Filters