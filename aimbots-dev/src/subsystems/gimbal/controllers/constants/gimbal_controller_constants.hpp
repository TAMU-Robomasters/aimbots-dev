#pragma once

#if defined(TARGET_AERIAL)
#include "aerial_controller_constants.hpp"

#elif defined(TARGET_ENGINEER)
#include "engineer_controller_constants.hpp"

#elif defined(TARGET_SWERVE_ENGINEER)
#include "swerve_engineer_controller_constants.hpp"

#elif defined(TARGET_HERO)
#include "hero_controller_constants.hpp"

#elif defined(TARGET_SENTRY)
#include "sentry_controller_constants.hpp"

#elif defined(TARGET_STANDARD)
#include "standard_controller_constants.hpp"

#elif defined(TARGET_SWERVE_STANDARD)
#include "swerve_standard_controller_constants.hpp"

#else
#error "Unsupported Robot Type!!"

#endif