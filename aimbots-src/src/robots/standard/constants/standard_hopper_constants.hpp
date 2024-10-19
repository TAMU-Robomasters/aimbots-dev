#pragma once
#include "utils/math/matrix_helpers.hpp"
#include "utils/tools/common_types.hpp"

#define HOPPER_LID_COMPATIBLE

// Hopper constants
static constexpr tap::gpio::Pwm::Pin HOPPER_PIN = tap::gpio::Pwm::C1;

static constexpr float HOPPER_PWM_RAMP_SPEED = 0.01f;  // pwm percent per millisecond

static constexpr float HOPPER_MIN_PWM = DS3218_MIN_PWM;
static constexpr float HOPPER_MAX_PWM = DS3218_MAX_PWM;

static constexpr float HOPPER_MIN_ANGLE = 0.0f;
static constexpr float HOPPER_MAX_ANGLE = 270.0f;

static constexpr float HOPPER_OPEN_ANGLE = 33.0f;
static constexpr float HOPPER_CLOSED_ANGLE = 77.0f;

static constexpr uint32_t HOPPER_MIN_ACTION_DELAY = 1000;  // Minimum time in ms between hopper lid flips
