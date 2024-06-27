#pragma once

#if defined(TARGET_AERIAL)
#include "robots/aerial/aerial_constants.hpp"
#include "robots/aerial/aerial_control_interface.hpp"

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

#elif defined(TARGET_STANDARD_BLASTOISE) || defined(TARGET_STANDARD_WARTORTLE) || defined(TARGET_STANDARD_SQUIRTLE) || \
    defined(TARGET_STANDARD_BALANCE)
#include "robots/standard/standard_constants.hpp"
#include "robots/standard/standard_control_interface.hpp"

#elif defined(TARGET_BAL_STANDARD)
#include "robots/standard/bal_standard_constants.hpp"
#include "robots/standard/standard_control_interface.hpp"

#elif defined(TARGET_1V1_STANDARD)
#include "robots/standard/standard_constants.hpp"
#include "robots/standard/standard_control_interface.hpp"

#elif defined(TARGET_STANDARD_2023)
#include "robots/standard/standard_2023_constants.hpp"
#include "robots/standard/standard_control_interface.hpp"

#elif defined(TARGET_CVTESTBENCH)
#include "robots/testbench/cvtestbench_constants.hpp"
#include "robots/testbench/cvtestbench_control_interface.hpp"

#elif defined(TARGET_TURRET)
#include "robots/turret/turret_constants.hpp"
#include "robots/turret/turret_control_interface.hpp"

#endif