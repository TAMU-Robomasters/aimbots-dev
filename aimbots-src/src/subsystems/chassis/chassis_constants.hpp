#pragma once

#include "utils/tools/robot_specific_defines.hpp"

#if defined(ALL_STANDARDS)
#include "robots/standard/constants/standard_chassis_constants.hpp"

#elif defined(ALL_HEROES)
#include "robots/hero/constants/hero_chassis_constants.hpp"

#elif defined(ALL_SENTRIES)
#include "robots/sentry/constants/sentry_chassis_constants.hpp"

#elif defined(ALL_ENGINEERS)
#include "robots/engineer/constants/engineer_chassis_constants.hpp"

#elif defined(ALL_AERIALS)
#include "robots/aerial/constants/aerial_chassis_constants.hpp"

#elif defined(ALL_DARTS)
#include "robots/dart/constants/dart_chassis_constants.hpp"

#elif defined(ALL_TESTBENCHES)
#include "robots/testbench/constants/testbench_chassis_constants.hpp"

#elif defined(ALL_TURRETS)
#include "robots/turret/constants/turret_chassis_constants.hpp"

#endif

#ifdef CHASSIS_COMPATIBLE

//Insert any constants common among all robot chassises

#endif