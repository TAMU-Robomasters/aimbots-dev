#pragma once

#include "utils/tools/robot_specific_defines.hpp"

#if defined(ALL_DARTS)
#include "robots/dart/constants/dart_launcher_constants.hpp"

#elif defined(ALL_TESTBENCHES)
#include "robots/testbench/constants/testbench_hopper_constants.hpp"

#endif

#ifdef HOPPER_COMPATIBLE

//Insert any constants common among all robot hoppers

#endif