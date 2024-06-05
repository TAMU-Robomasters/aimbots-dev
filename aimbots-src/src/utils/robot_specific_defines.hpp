#pragma once
#if defined(TARGET_AERIAL)
#define ALL_AERIALS

#elif defined(TARGET_ENGINEER) || defined(TARGET_SWERVE_ENGINEER)
#define ALL_ENGINEERS

#elif defined(TARGET_HERO)
#define ALL_HEROES

#elif defined(TARGET_SENTRY)
#define ALL_SENTRIES

#elif defined(TARGET_STANDARD_BLASTOISE) || defined(TARGET_STANDARD_WARTORTLE) || defined(TARGET_STANDARD_SQUIRTLE) || \
    defined(TARGET_STANDARD_2023)
#define ALL_STANDARDS

//#elif defined(TARGET_CVTESTBENCH)

//#elif defined(TARGET_TURRET)

#endif