#pragma once

#if defined(TARGET_AERIAL)
/**
 * @brief Aerial Rotation and Distance/Position matrixies
 */
static constexpr float R_cam2gimb[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_cam2gimb[3] = {1, 2, 3};

static constexpr float R_gimb2cam[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_gimb2cam[3] = {-1, -2, -3};

static constexpr float R_chas2gimb[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_chas2gimb[3] = {1, 2, 3};

static constexpr float R_gimb2chas[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_gimb2chas[3] = {-1, -2, -3};

static constexpr float R_chas2field[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_chas2field[3] = {1, 2, 3};

static constexpr float R_field2chas[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_field2chas[3] = {-1, -2, -3};
// }
#endif

#if defined(TARGET_ENGINEER)

/**
 * @brief Engineer Rotation and Distance/Position matrixies
 */
static constexpr float R_cam2gimb[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_cam2gimb[3] = {1, 2, 3};

static constexpr float R_gimb2cam[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_gimb2cam[3] = {-1, -2, -3};

static constexpr float R_chas2gimb[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_chas2gimb[3] = {1, 2, 3};

static constexpr float R_gimb2chas[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_gimb2chas[3] = {-1, -2, -3};

static constexpr float R_chas2field[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_chas2field[3] = {1, 2, 3};

static constexpr float R_field2chas[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_field2chas[3] = {-1, -2, -3};
// }
#endif

#if defined(TARGET_SWERVE_ENGINEER)

/**
 * @brief Swerve Engineer Rotation and Distance/Position matrixies
 */
static constexpr float R_cam2gimb[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_cam2gimb[3] = {1, 2, 3};

static constexpr float R_gimb2cam[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_gimb2cam[3] = {-1, -2, -3};

static constexpr float R_chas2gimb[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_chas2gimb[3] = {1, 2, 3};

static constexpr float R_gimb2chas[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_gimb2chas[3] = {-1, -2, -3};

static constexpr float R_chas2field[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_chas2field[3] = {1, 2, 3};

static constexpr float R_field2chas[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_field2chas[3] = {-1, -2, -3};
// }
#endif

#if defined(TARGET_HERO)

/**
 * @brief Hero Rotation and Distance/Position matrixies
 */
static constexpr float R_cam2gimb[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_cam2gimb[3] = {1, 2, 3};

static constexpr float R_gimb2cam[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_gimb2cam[3] = {-1, -2, -3};

static constexpr float R_chas2gimb[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_chas2gimb[3] = {1, 2, 3};

static constexpr float R_gimb2chas[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_gimb2chas[3] = {-1, -2, -3};

static constexpr float R_chas2field[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_chas2field[3] = {1, 2, 3};

static constexpr float R_field2chas[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_field2chas[3] = {-1, -2, -3};
// }
#endif

#if defined(TARGET_SENTRY)

/**
 * @brief Sentry Rotation and Distance/Position matrixies
 */
static constexpr float R_cam2gimb[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_cam2gimb[3] = {1, 2, 3};

static constexpr float R_gimb2cam[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_gimb2cam[3] = {-1, -2, -3};

static constexpr float R_chas2gimb[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_chas2gimb[3] = {1, 2, 3};

static constexpr float R_gimb2chas[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_gimb2chas[3] = {-1, -2, -3};

static constexpr float R_chas2field[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_chas2field[3] = {1, 2, 3};

static constexpr float R_field2chas[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_field2chas[3] = {-1, -2, -3};
// }
#endif

#if defined(TARGET_STANDARD)
/**
 * @brief Standard Rotation and Distance/Position matrixies
 */
static constexpr float R_cam2gimb[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_cam2gimb[3] = {-0.085, 0.07, 0};
//{0, -0.09907, -0.047841} normally

static constexpr float R_gimb2cam[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_gimb2cam[3] = {0.085, -0.07, 0};
//{0, 0.09907, -0.047841} normally

static constexpr float R_chas2gimb[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_chas2gimb[3] = {1, 2, 3};

static constexpr float R_gimb2chas[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_gimb2chas[3] = {-1, -2, -3};

static constexpr float R_chas2field[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_chas2field[3] = {1, 2, 3};

static constexpr float R_field2chas[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_field2chas[3] = {-1, -2, -3};
// }
#endif

#if defined(TARGET_SWERVE_STANDARD)
/**
 * @brief Swerve Standard Rotation and Distance/Position matrixies
 */
static constexpr float R_cam2gimb[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_cam2gimb[3] = {1, 2, 3};

static constexpr float R_gimb2cam[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_gimb2cam[3] = {-1, -2, -3};

static constexpr float R_chas2gimb[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_chas2gimb[3] = {1, 2, 3};

static constexpr float R_gimb2chas[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_gimb2chas[3] = {-1, -2, -3};

static constexpr float R_chas2field[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_chas2field[3] = {1, 2, 3};

static constexpr float R_field2chas[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static constexpr float P_field2chas[3] = {-1, -2, -3};
// }

#endif

/**
 * @brief IMU Rotation and Distance/Position matrixies
 */

// IMU rotation
static constexpr float R_IMU2chas[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
// IMU positon mounting
static constexpr float P_IMU2chas[3] = {1, 2, 3};
