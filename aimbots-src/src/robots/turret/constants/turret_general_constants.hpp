#pragma once

#include "utils/math/matrix_helpers.hpp"
#include "utils/tools/common_types.hpp"



static Vector3f IMU_MOUNT_POSITION{0.0f, 0.0f, 0.0f};

static constexpr float POWER_LIMIT_SAFETY_FACTOR = 0.85f;
static constexpr float STARTING_ENERGY_BUFFER = 60.0f;
static constexpr float ENERGY_BUFFER_LIMIT_THRESHOLD = 60.0f;
static constexpr float ENERGY_BUFFER_CRIT_THRESHOLD = 10.0f;