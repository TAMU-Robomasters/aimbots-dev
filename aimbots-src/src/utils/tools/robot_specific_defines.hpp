#pragma once
#if defined(TARGET_AERIAL)
#include "robots/aerial/aerial_control_interface.hpp"
#include "robots/aerial/constants/aerial_general_constants.hpp"
#define ALL_AERIALS

#elif defined(TARGET_DART)
#include "robots/dart/constants/dart_general_constants.hpp"
#include "robots/dart/dart_control_interface.hpp"
#define ALL_DARTS

#elif defined(TARGET_ENGINEER)
#include "robots/engineer/constants/engineer_general_constants.hpp"
#include "robots/engineer/engineer_control_interface.hpp"
#define ALL_ENGINEERS

#elif defined(TARGET_HERO)
#include "robots/hero/constants/hero_general_constants.hpp"
#include "robots/hero/hero_control_interface.hpp"
#define ALL_HEROES

#elif defined(TARGET_SENTRY)
#include "robots/sentry/constants/sentry_general_constant.hpp"
#include "robots/sentry/sentry_control_interface.hpp"
#define TURRET_IMU
#define ALL_SENTRIES

#elif defined(TARGET_STANDARD_BALTHAZAR)  || defined(TARGET_STANDARD_JERRY)
#include "robots/standard/constants/standard_general_constants.hpp"
#include "robots/standard/standard_control_interface.hpp"
#define ALL_STANDARDS

#elif defined(TARGET_CVTEST_HAN)
#include "robots/testbench/constants/testbench_general_constants.hpp"
#include "robots/testbench/testbench_control_interface.hpp"
#define ALL_TESTBENCHES

#elif defined(TARGET_TURRET)
#include "robots/turret/constants/turret_general_constants.hpp"
#include "robots/turret/turret_control_interface.hpp"
#define ALL_TURRETS

#endif