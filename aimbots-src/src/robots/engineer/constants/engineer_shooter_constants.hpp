#pragma once
#include "utils/math/matrix_helpers.hpp"
#include "utils/tools/common_types.hpp"

static constexpr uint8_t SHOOTER_MOTOR_COUNT = 2;


static constexpr float kGRAVITY = -1500.0f;  // Negative because weight is behind pitch motor
static constexpr float HORIZON_OFFSET = 0.0f;

static constexpr int PROJECTILES_PER_FEEDER_ROTATION = 1;  // Engineer with a glock
