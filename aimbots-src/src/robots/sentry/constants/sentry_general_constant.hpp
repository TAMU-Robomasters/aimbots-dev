#pragma once
#include "utils/tools/common_types.hpp"
#include "utils/math/matrix_helpers.hpp"

static constexpr SongTitle STARTUP_SONG = SongTitle::DUNE;

static Vector3f IMU_MOUNT_POSITION{0.0f, 0.0f, 0.0f};

/**
 * @brief Transformation Matrices, specific to robot
 */
// clang-format off

// Updated for 2025 Sentry
static Vector3f CAMERA_ORIGIN_RELATIVE_TO_TURRET_ORIGIN{ // in meters
    -0.022025f, 
    0.139188f,
    0.0047f, 
};

static Vector3f TURRET_ORIGIN_RELATIVE_TO_CHASSIS_ORIGIN{ // not used
    0.0f, // x
    0.0f, // y
    0.0f  // z
};

static Vector3f CHASSIS_START_POSITION_RELATIVE_TO_WORLD{ // not used
    -2.830f, // x
    -0.730f, // y
    0.0f, // z
};

//0.04341 how far apart barrels are
static Vector3f BARREL_POSITION_FROM_GIMBAL_ORIGIN{
    0.0f, // x //0.015727 - (0.5f * 0.04341f)
    0.0f, // y //This doesn't matter, is infinitely long for ballistics purposes
    -0.011049f, // z //-0.011049f
};
// clang-format on

static constexpr float CIMU_CALIBRATION_EULER_X = modm::toRadian(0.0f);
static constexpr float CIMU_CALIBRATION_EULER_Y = modm::toRadian(0.0f);
static constexpr float CIMU_CALIBRATION_EULER_Z = modm::toRadian(0.0f);

static constexpr float TIMU_CALIBRATION_EULER_X = modm::toRadian(0.0f);
static constexpr float TIMU_CALIBRATION_EULER_Y = modm::toRadian(0.0f);
static constexpr float TIMU_CALIBRATION_EULER_Z = modm::toRadian(0.0f);

static constexpr size_t PROJECTILE_SPEED_QUEUE_SIZE = 10;

