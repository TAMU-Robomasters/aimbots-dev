 #pragma once

#include "utils/tools/robot_specific_defines.hpp"

#if defined(ALL_STANDARDS)
#include "robots/standard/constants/standard_display_constants.hpp"

#elif defined(ALL_HEROES)
#include "robots/hero/constants/hero_display_constants.hpp"

#elif defined(ALL_SENTRIES)
#include "robots/sentry/constants/sentry_display_constants.hpp"

#elif defined(ALL_ENGINEERS)
#include "robots/engineer/constants/engineer_display_constants.hpp"

#elif defined(ALL_AERIALS)
#include "robots/aerial/constants/aerial_display_constants.hpp"

#elif defined(ALL_DARTS)
#include "robots/dart/constants/dart_display_constants.hpp"

#elif defined(ALL_TESTBENCHES)
#include "robots/testbench/constants/testbench_display_constants.hpp"

#elif defined(ALL_TURRETS)
#include "robots/turret/constants/turret_display_constants.hpp"

#endif

// #ifdef DISPLAY_COMPATIBLE not sure if this exists
//Insert any constants common among all robot displays

