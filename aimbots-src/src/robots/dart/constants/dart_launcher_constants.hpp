#pragma once
#include "utils/tools/common_types.hpp"
#include "utils/math/matrix_helpers.hpp"
#include "tap/communication/gpio/pwm.hpp"

#define LAUNCHER_COMPATIBLE

static constexpr tap::gpio::Pwm::Pin LAUNCHER_PIN = tap::gpio::Pwm::C1;

static constexpr float LAUNCHER_PWM_LAUNCH_SPEED = 0.01f;  // pwm percent per millisecond

static constexpr float LAUNCHER_MIN_PWM = DS3218_MIN_PWM;
static constexpr float LAUNCHER_MAX_PWM = DS3218_MAX_PWM;