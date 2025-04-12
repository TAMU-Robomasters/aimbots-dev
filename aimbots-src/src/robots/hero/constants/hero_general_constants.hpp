#pragma once
#include "utils/math/matrix_helpers.hpp"
#include "utils/tools/common_types.hpp"

static constexpr SongTitle STARTUP_SONG = SongTitle::LG_WASH;

static Vector3f IMU_MOUNT_POSITION{-0.0035f, 0.101f, 0.0f};

// Power limiting constants, will explain later
static constexpr float POWER_LIMIT_SAFETY_FACTOR = 0.85f;
static constexpr float STARTING_ENERGY_BUFFER = 60.0f;
static constexpr float ENERGY_BUFFER_LIMIT_THRESHOLD = 60.0f;
static constexpr float ENERGY_BUFFER_CRIT_THRESHOLD = 15.0f;

/**
 * @brief Transformation Matrices, specific to robot
 */

static Vector3f CAMERA_ORIGIN_RELATIVE_TO_TURRET_ORIGIN{
    // in meters
    -0.002201f,  // x
    0.1348f,     // y
    -0.0498f,    // z
};

static Vector3f TURRET_ORIGIN_RELATIVE_TO_CHASSIS_ORIGIN{
    0.0f,  // x
    0.0f,  // y
    0.0f   // z
};

static Vector3f CHASSIS_START_POSITION_RELATIVE_TO_WORLD{
    0.0f,  // x
    0.0f,  // y
    0.0f,  // z
};

static Vector3f BARREL_POSITION_FROM_GIMBAL_ORIGIN{
    -0.001727f,  // x = 0.04498
    0.0f,        // y - does not matter too much because projectile comes out this axis
    -0.00587f,   // z = 0.01683
};

static constexpr float CIMU_CALIBRATION_EULER_X = modm::toRadian(0.0f);
static constexpr float CIMU_CALIBRATION_EULER_Y = modm::toRadian(0.0f);
static constexpr float CIMU_CALIBRATION_EULER_Z = modm::toRadian(-90.0f);

static constexpr float TIMU_CALIBRATION_EULER_X = modm::toRadian(0.0f);
static constexpr float TIMU_CALIBRATION_EULER_Y = modm::toRadian(0.0f);
static constexpr float TIMU_CALIBRATION_EULER_Z = modm::toRadian(0.0f);

static constexpr size_t PROJECTILE_SPEED_QUEUE_SIZE = 3;