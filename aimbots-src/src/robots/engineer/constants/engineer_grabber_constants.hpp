#pragma once
#include "utils/math/matrix_helpers.hpp"
#include "utils/tools/common_types.hpp"

#define GRABBER_COMPATIBLE

static constexpr tap::gpio::Pwm::Pin GRABBER_PIN = tap::gpio::Pwm::PWM1;  // DM-MC02 PWM1 (PA0); was Type C C1