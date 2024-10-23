#pragma once
#include "utils/tools/common_types.hpp"
#include "utils/math/matrix_helpers.hpp"


static constexpr int UNJAM_TIMER_MS = 300;

static constexpr CANBus FEED_BUS = CANBus::CAN_BUS1;

//
static constexpr MotorID FEEDER_ID = MotorID::MOTOR8;
static constexpr uint8_t PROJECTILES_PER_FEEDER_ROTATION = 10;
static constexpr bool FEEDER_DIRECTION = true;
