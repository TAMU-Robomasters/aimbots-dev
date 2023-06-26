#pragma once

#include "tap/algorithms/kalman_filter.hpp"
#include "tap/algorithms/odometry/chassis_displacement_observer_interface.hpp"
#include "tap/algorithms/odometry/odometry_2d_interface.hpp"
#include "tap/control/chassis/chassis_subsystem_interface.hpp"

#include "utils/common_types.hpp"

namespace src::Informants::Odometry {

class ChassisKFOdometry : public tap::algorithms::odometry::Odometry2DInterface {
public:
    ChassisKFOdometry(const tap::control::chassis::ChassisSubsystemInterface& chassis);

    inline modm::Location2D<float> getCurrentLocation2D() const final { return location; }

    inline modm::Vector2f getCurrentVelocity2D() const final { return velocity; }

    inline uint32_t getLastComputedOdometryTime() const final { return lastComputedOdometryTime; }

    inline float getYaw() const override { return chassisYaw; }

    void update(float chassisYaw, float xChassisAccel, float yChassisAccel);

private:
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
    static constexpr float KF_A[STATES_SQUARED] = {
        1, dt, 0.5 * dt * dt, 0, 0 , 0            ,
        0, 1 , dt           , 0, 0 , 0            ,
        0, 0 , 1            , 0, 0 , 0            ,
        0, 0 , 0            , 1, dt, 0.5 * dt * dt,
        0, 0 , 0            , 0, 1 , dt           ,
        0, 0 , 0            , 0, 0 , 1            ,
    };
    static constexpr float KF_C[INPUTS_MULT_STATES] = {
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1,
    };
    static constexpr float KF_Q[STATES_SQUARED] = {
        1E1, 0  , 0   , 0  , 0  , 0   ,
        0  , 1E0, 0   , 0  , 0  , 0   ,
        0  , 0  , 1E-1, 0  , 0  , 0   ,
        0  , 0  , 0   , 1E1, 0  , 0   ,
        0  , 0  , 0   , 0  , 1E0, 0   ,
        0  , 0  , 0   , 0  , 0  , 1E-1,
    };
    static constexpr float KF_R[INPUTS_SQUARED] = {
        1.0, 0  , 0  , 0  ,
        0  , 1.2, 0  , 0  ,
        0  , 0  , 1.0, 0  ,
        0  , 0  , 0  , 1.2,
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

    static constexpr float MAX_ACCELERATION = 8.0f;

    static constexpr modm::Pair<float, float> CHASSIS_ACCELERATION_TO_MEASUREMENT_COVARIANCE_LUT[] = {
        {0, 1E0},
        {MAX_ACCELERATION, 1E2},
    };

    static constexpr float CHASSIS_WHEEL_ACCELERATION_LOW_PASS_ALPHA = 0.01f;

    const tap::control::chassis::ChassisSubsystemInterface& chassis;

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