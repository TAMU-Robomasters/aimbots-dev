#include "gimbal_gravity_helper.hpp"

#include <tap/algorithms/math_user_utils.hpp>

float computeGravitationalForceOffset(
    float cgX,
    float cgZ,
    float pitchAngleFromCenter,
    float gravityCompensationMax)
{
    bool isCGXZero = tap::algorithms::compareFloatClose(cgX, 0.0f, 1E-5);
    bool isCGZZero = tap::algorithms::compareFloatClose(cgZ, 0.0f, 1E-5);

    if(isCGXZero && isCGZZero) {
        return 0.0f;
    }

    float gimbalCGPolarTheta = 0.0f;
    if(!isCGXZero) {
        gimbalCGPolarTheta = (cgX > 0.0f) ? atanf(cgZ / cgX) : (atanf(cgZ / cgX) + M_PI);
    } else {
        gimbalCGPolarTheta = copysign(M_PI_2, cgZ);
    }

    return gravityCompensationMax * cosf(gimbalCGPolarTheta + pitchAngleFromCenter);
}
