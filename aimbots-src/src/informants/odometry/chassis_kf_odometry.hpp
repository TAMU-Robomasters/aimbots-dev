#pragma once

#include "tap/algorithms/kalman_filter.hpp"
#include "tap/algorithms/odometry/chassis_displacement_observer_interface.hpp"
#include "tap/algorithms/odometry/odometry_2d_interface.hpp"
#include "tap/control/chassis/chassis_subsystem_interface.hpp"

#include "utils/tools/common_types.hpp"

namespace src::Informants::Odometry {

class ChassisKFOdometry : public tap::algorithms::odometry::Odometry2DInterface {
public:
    ChassisKFOdometry(float initialXPos = 0.0f, float initialYPos = 0.0f);

    void registerChassisSubsystem(tap::control::chassis::ChassisSubsystemInterface* chassis) { this->chassis = chassis; }

    inline modm::Location2D<float> getCurrentLocation2D() const final { return location; }

    inline modm::Vector2f getCurrentVelocity2D() const final { return velocity; }

    inline uint32_t getLastComputedOdometryTime() const final { return lastComputedOdometryTime; }

    inline float getYaw() const override { return chassisYaw; }

    void update(float chassisYaw, float xChassisAccel, float yChassisAccel);

    void reset();

    void resetAtNewLocation(float newXPos = 0.0f, float newYPos = 0.0f);

    void overrideOdometryPosition(const float positionX, const float positionY) override {};


private:
    float initialXPos;
    float initialYPos;

    enum class OdomState {
        POS_X = 0,
        VEL_X,
        ACC_X,
        POS_Y,
        VEL_Y,
        ACC_Y,
        NUM_STATES,
    };

    enum class OdomInput {
        VEL_X = 0,
        ACC_X,
        VEL_Y,
        ACC_Y,
        NUM_INPUTS,
    };

    static constexpr int STATES_SQUARED = static_cast<int>(OdomState::NUM_STATES) * static_cast<int>(OdomState::NUM_STATES);
    static constexpr int INPUTS_SQUARED = static_cast<int>(OdomInput::NUM_INPUTS) * static_cast<int>(OdomInput::NUM_INPUTS);
    static constexpr int INPUTS_MULT_STATES =
        static_cast<int>(OdomInput::NUM_INPUTS) * static_cast<int>(OdomState::NUM_STATES);

    /// Assumed time difference between calls to `update`, in seconds
    static constexpr float dt = 0.002f;

    // clang-format off
    // VELOCITY-ONLY EXPERIMENT: the accel->vel (dt) and accel->pos (0.5*dt*dt) coupling
    // terms are zeroed so the acceleration state can never affect velocity or position.
    // Position is then a pure integral of the (measured) velocity state.
    // Original values: row0 col2 = 0.5*dt*dt, row1 col2 = dt, row3 col5 = 0.5*dt*dt, row4 col5 = dt.
    static constexpr float KF_A[STATES_SQUARED] = {
        1, dt, 0, 0, 0 , 0,
        0, 1 , 0, 0, 0 , 0,
        0, 0 , 1, 0, 0 , 0,
        0, 0 , 0, 1, dt, 0,
        0, 0 , 0, 0, 1 , 0,
        0, 0 , 0, 0, 0 , 1,
    };
    static constexpr float KF_C[INPUTS_MULT_STATES] = {
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1,
    };
    // Velocity process-noise std = 2 m/s -> variance Q_VEL = 2^2 = 4.
    // Position process noise is derived from it: a velocity disturbance of std sqrt(Q_VEL)
    // displaces position by (that velocity * dt) over one tick, so Var(pos) = Q_VEL * dt^2.
    static constexpr float Q_VEL = 4.0f;
    static constexpr float KF_Q[STATES_SQUARED] = {
        Q_VEL * dt * dt, 0    , 0   , 0              , 0    , 0   ,
        0              , Q_VEL, 0   , 0              , 0    , 0   ,
        0              , 0    , 1E-1, 0              , 0    , 0   ,
        0              , 0    , 0   , Q_VEL * dt * dt, 0    , 0   ,
        0              , 0    , 0   , 0              , Q_VEL, 0   ,
        0              , 0    , 0   , 0              , 0    , 1E-1,
    };
    // Velocity measurement std = 0.5 m/s -> variance R_VEL = 0.5^2 = 0.25.
    static constexpr float R_VEL = 0.25f;
    static constexpr float KF_R[INPUTS_SQUARED] = {
        R_VEL, 0  , 0    , 0  ,
        0    , 1.2, 0    , 0  ,
        0    , 0  , R_VEL, 0  ,
        0    , 0  , 0    , 1.2,
    };
    static constexpr float KF_P0[STATES_SQUARED] = {
        1E3, 0  , 0  , 0  , 0  , 0  ,
        0  , 1E3, 0  , 0  , 0  , 0  ,
        0  , 0  , 1E3, 0  , 0  , 0  ,
        0  , 0  , 0  , 1E3, 0  , 0  ,
        0  , 0  , 0  , 0  , 1E3, 0  ,
        0  , 0  , 0  , 0  , 0  , 1E3,
    };
    // clang-format on

    static constexpr float MAX_ACCELERATION = 8.0f;  // m/s^2

    static constexpr modm::Pair<float, float> CHASSIS_ACCELERATION_TO_MEASUREMENT_COVARIANCE_LUT[] = {
        {0, 1E0},
        {MAX_ACCELERATION, 1E2},
    };

    static constexpr float CHASSIS_WHEEL_ACCELERATION_LOW_PASS_ALPHA = 0.01f;

    tap::control::chassis::ChassisSubsystemInterface* chassis;

    tap::algorithms::KalmanFilter<int(OdomState::NUM_STATES), int(OdomInput::NUM_INPUTS)> kf;

    /// Chassis location in the world frame
    modm::Location2D<float> location;
    /// Chassis velocity in the world frame
    modm::Vector2f velocity;
    // Chassis yaw orientation in world frame (radians)
    float chassisYaw = 0.0f;

    modm::Vector2f chassisMeasuredDeltaVelocity;

    modm::interpolation::Linear<modm::Pair<float, float>> chassisAccelerationToMeasurementCovarianceInterpolator;

    /// Previous time `update` was called, in microseconds
    uint32_t lastComputedOdometryTime = 0;
    modm::Matrix<float, 3, 1> prevChassisVelocity;

    void updateChassisStateFromKF(float chassisYaw);

    void updateMeasurementCovariance(const modm::Matrix<float, 3, 1>& chassisVelocity);
};

}  // namespace src::Informants::Odometry