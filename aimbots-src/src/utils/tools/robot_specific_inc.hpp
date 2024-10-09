#pragma once

#if defined(TARGET_AERIAL)
#include "robots/aerial/aerial_constants.hpp"
#include "robots/aerial/aerial_control_interface.hpp"

#elif defined(TARGET_DART)
#include "robots/dart/dart_constants.hpp"
#include "robots/dart/dart_control_interface.hpp"

#elif defined(TARGET_ENGINEER)
#include "robots/engineer/engineer_constants.hpp"
#include "robots/engineer/engineer_control_interface.hpp"

#elif defined(TARGET_SWERVE_ENGINEER)
#include "robots/engineer-swerve/swerve_engineer_constants.hpp"
#include "robots/engineer-swerve/swerve_engineer_control_interface.hpp"

#elif defined(TARGET_HERO)
#include "robots/hero/hero_constants.hpp"
#include "robots/hero/hero_control_interface.hpp"

#elif defined(TARGET_SENTRY)
#include "robots/sentry/sentry_constants.hpp"
#include "robots/sentry/sentry_control_interface.hpp"

#elif defined(TARGET_STANDARD)
#include "robots/standard/standard_constants.hpp"
#include "robots/standard/standard_control_interface.hpp"

#elif defined(TARGET_STANDARD_BLASTOISE) || defined(TARGET_STANDARD_WARTORTLE) || defined(TARGET_STANDARD_SQUIRTLE)
#include "robots/standard/constants/standard_chassis_constants.hpp"
#include "robots/standard/constants/standard_display_constants.hpp"
#include "robots/standard/constants/standard_feeder_constants.hpp"
#include "robots/standard/constants/standard_general_constants.hpp"
#include "robots/standard/constants/standard_gimbal_constants.hpp"
#include "robots/standard/constants/standard_grabber_constants.hpp"
#include "robots/standard/constants/standard_hopper_constants.hpp"
#include "robots/standard/constants/standard_shooter_constants.hpp"
#include "robots/standard/constants/standard_slide_constants.hpp"
#include "robots/standard/constants/standard_wrist_constants.hpp"
#include "robots/standard/standard_control_interface.hpp"

#elif defined(TARGET_STANDARD_2023)
#include "robots/standard/standard_2023_constants.hpp"
#include "robots/standard/standard_control_interface.hpp"

#elif defined(TARGET_CVTESTBENCH)
#include "robots/testbench/testbench_constants.hpp"
#include "robots/testbench/testbench_control_interface.hpp"

#elif defined(TARGET_TURRET)
#include "robots/turret/turret_constants.hpp"
#include "robots/turret/turret_control_interface.hpp"

#endif