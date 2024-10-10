#pragma once
#include "utils/math/matrix_helpers.hpp"
#include "utils/tools/common_types.hpp"

//#define BARREL_SWAP_COMPATIBLE

// #define TURRET_HAS_IMU
#define GIMBAL_UNTETHERED

static constexpr SongTitle STARTUP_SONG = SongTitle::PACMAN;

/**
 * @brief Defines the number of motors created for the chassis.
 */



static Vector3f IMU_MOUNT_POSITION{0.0992f, 0.0f, 0.0534f};



// 1 for no symmetry, 2 for 180 degree symmetry, 4 for 90 degree symmetry
static constexpr uint8_t CHASSIS_SNAP_POSITIONS = 2;


/**
 * @brief Transformation Matrices, specific to robot
 */

// clang-format off
static Vector3f CAMERA_ORIGIN_RELATIVE_TO_TURRET_ORIGIN{ // in meters
    -0.002201f, // x
    0.1348f, // y
    -0.0498f,  // z
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
    -0.001727f, //x = 0.04498
    0.0f, //y - does not matter too much because projectile comes out this axis
    -0.00587f, //z = 0.01683
};
// clang-format on

static constexpr float CHASSIS_START_ANGLE_WORLD = modm::toRadian(0.0f);  // theta (about z axis)

static constexpr float CIMU_CALIBRATION_EULER_X = modm::toRadian(180.0f);
static constexpr float CIMU_CALIBRATION_EULER_Y = modm::toRadian(0.0f);
static constexpr float CIMU_CALIBRATION_EULER_Z = modm::toRadian(90.0f);

static constexpr float TIMU_CALIBRATION_EULER_X = modm::toRadian(0.0f);
static constexpr float TIMU_CALIBRATION_EULER_Y = modm::toRadian(0.0f);
static constexpr float TIMU_CALIBRATION_EULER_Z = modm::toRadian(0.0f);
static constexpr size_t PROJECTILE_SPEED_QUEUE_SIZE = 10;

// This array holds the IDs of all speed monitor barrels on the robot
static const std::array<BarrelID, 2> BARREL_IDS = {BarrelID::TURRET_17MM_1, BarrelID::TURRET_17MM_2};

