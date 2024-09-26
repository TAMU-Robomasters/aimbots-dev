#pragma once

#include "utils/tools/robot_specific_defines.hpp"

#if defined(ALL_STANDARDS)
#include "robots/standard/constants/standard_hopper_constants.hpp"

#elif defined(ALL_HEROES)
#include "robots/hero/constants/hero_hopper_constants.hpp"

#elif defined(ALL_SENTRIES)
#include "robots/sentry/constants/sentry_hopper_constants.hpp"

#elif defined(ALL_ENGINEERS)
#include "robots/engineer/constants/engineer_hopper_constants.hpp"

#elif defined(ALL_AERIALS)
#include "robots/aerial/constants/aerial_hopper_constants.hpp"

#elif defined(ALL_DARTS)
#include "robots/dart/constants/dart_hopper_constants.hpp"

#elif defined(ALL_TESTBENCHES)
#include "robots/testbench/constants/testbench_hopper_constants.hpp"

#elif defined(ALL_TURRETS)
#include "robots/turret/constants/turret_hopper_constants.hpp"

#endif

#ifdef HOPPER_COMPATIBLE

//Insert any constants common among all robot hoppers

#endif