#pragma once
#include "utils/tools/common_types.hpp"
#include "utils/math/matrix_helpers.hpp"


static constexpr SongTitle STARTUP_SONG = SongTitle::NONE;

/**
 * @brief Defines the number of motors created for the chassis.
 */



static Vector3f IMU_MOUNT_POSITION{0.0f, 0.0f, 0.0f};



// 1 for no symmetry, 2 for 180 degree symmetry, 4 for 90 degree symmetry
static constexpr uint8_t CHASSIS_SNAP_POSITIONS = 1;



/**
 * @brief Transformation Matrices, specific to robot
 */

// clang-format off
static Vector3f CAMERA_ORIGIN_RELATIVE_TO_TURRET_ORIGIN{ // in meters
    0.047531f, // x
    0.12025f, // y
    -0.017464f,  // z
};

static Vector3f TURRET_ORIGIN_RELATIVE_TO_CHASSIS_ORIGIN{
    0.0f, // x
    0.0f, // y
    0.0f  // z
};

static Vector3f CHASSIS_START_POSITION_RELATIVE_TO_WORLD{
    0.0f, // x
    0.0f, // y
    0.0f, // z
};

static Vector3f BARREL_POSITION_FROM_GIMBAL_ORIGIN{
    0.046250f, //x
    0.012996f, //y  
    0.016861f, //z
};
// clang-format on

static constexpr size_t PROJECTILE_SPEED_QUEUE_SIZE = 10;

static constexpr float CHASSIS_START_ANGLE_WORLD = modm::toRadian(0.0f);  // theta (about z axis)

static constexpr float CIMU_CALIBRATION_EULER_X = modm::toRadian(180.0f);
static constexpr float CIMU_CALIBRATION_EULER_Y = modm::toRadian(0.0f);
static constexpr float CIMU_CALIBRATION_EULER_Z = modm::toRadian(180.0f);