#pragma once

#include <tap/algorithms/math_user_utils.hpp>

namespace src::Gimbal::Calculations {

/**
 * This came from UW's code. The original source file can be found
 * here: https://github.com/uw-advanced-robotics/aruw-mcb/blob/develop/aruw-mcb-project/src/aruwsrc/control/turret/algorithms/turret_gravity_compensation.hpp
 * 
 * @param[in] cgX The center of gravity relative to the center of the turret's pitch pivot point
 *      in the X (forward/back) direction. The "X" direction lies along the plane that the turret
 *      is pointing. Units in millimeters. Positive is forward, negative is backwards.
 * @param[in] cgZ The center of gravity relative to the center of the turret's pitch pivot point,
 *      in the Z (up/down) direction. The "Z" direction lies perpendicular to the plane that the
 *      turret is pointing. Units in millimeters. Positive is upwards, negative is downwards.
 * @param[in] pitchAngleFromCenter The angle in degrees of the turret pitch, relative to the
 *      horizontal plane.
 * @param[in] gravityCompensationMotorOutputMax The maximum output that will be returned by this
 *      function. Should be equivalent to the output to offset gravity when the center of mass lies
 *      on the same xy-plane as the pivot (i.e.: when the turret's mass exerts the most torque about
 *      it's pivot).
 * @return The gravitational force offset necessary to cancel out gravitational
 *      force of the turret, between [-gravityCompensatorMax, gravityCompensatorMax].
 *      The gravitational force offset is a function of the location of the CG and
 *      the current pitch angle.
 */
float computeGravitationalForceOffset(
    float cgX,
    float cgZ,
    float pitchAngleFromCenter,
    float gravityCompensationMax);

} // namespace src::Gimbal::Calculations