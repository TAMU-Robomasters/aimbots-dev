//=============================================================================================
// MahonyAHRS.h
//=============================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date          Author          Notes
// 29/09/2011    SOH Madgwick    Initial release
// 02/10/2011    SOH Madgwick    Optimised for reduced CPU load
// 09/06/2020    Matthew Arnold  Update style, use safer casting
//
//=============================================================================================
#ifndef MAHONY_AHRS_H_
#define MAHONY_AHRS_H_

#include <cmath>
#include <optional>

#include "modm/math/geometry/vector3.hpp"

//--------------------------------------------------------------------------------------------
// Variable declaration

class Mahony
{
private:
    float twoKp;           // 2 * proportional gain (Kp)
    float twoKi;           // 2 * integral gain (Ki)
    float q0, q1, q2, q3;  // quaternion of sensor frame relative to auxiliary frame
    float integralFBx, integralFBy, integralFBz;  // integral error terms scaled by Ki
    float invSampleFreq;
    float roll, pitch, yaw;
    char anglesComputed;
    static float invSqrt(float x);
    void computeAngles();

    //-------------------------------------------------------------------------------------------
    // Function declarations

public:
    Mahony();

    void setCalibrationEuler(modm::Vector3f calibrationEuler)
    {
        float x_over_two = calibrationEuler.getX() * 0.5f;
        float y_over_two = calibrationEuler.getY() * 0.5f;
        float z_over_two = calibrationEuler.getZ() * 0.5f;

        float cosx2 = cosf(x_over_two);
        float cosy2 = cosf(y_over_two);
        float cosz2 = cosf(z_over_two);
        float sinx2 = sinf(x_over_two);
        float siny2 = sinf(y_over_two);
        float sinz2 = sinf(z_over_two);

        q0 = cosx2 * cosy2 * cosz2 + sinx2 * siny2 * sinz2;
        q1 = sinx2 * cosy2 * cosz2 - cosx2 * siny2 * sinz2;
        q2 = cosx2 * siny2 * cosz2 + sinx2 * cosy2 * sinz2;
        q3 = cosx2 * cosy2 * sinz2 - sinx2 * siny2 * cosz2;
    }

    void begin(float sampleFrequency, float kp, float ki)
    {
        invSampleFreq = 1.0f / sampleFrequency;
        twoKp = 2.0f * kp;
        twoKi = 2.0f * ki;
    }
    void reset()
    {
        q0 = 1.0f;
        q1 = 0.0f;
        q2 = 0.0f;
        q3 = 0.0f;
        integralFBx = 0.0f;
        integralFBy = 0.0f;
        integralFBz = 0.0f;
        anglesComputed = 0;
        roll = 0.0f;
        pitch = 0.0f;
        yaw = 0.0f;
    }
    void update(
        float gx,
        float gy,
        float gz,
        float ax,
        float ay,
        float az,
        float mx,
        float my,
        float mz);
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);
    float getRoll()
    {
        if (!anglesComputed) computeAngles();
        return roll * 57.29578f;
    }
    float getPitch()
    {
        if (!anglesComputed) computeAngles();
        return pitch * 57.29578f;
    }
    float getYaw()
    {
        if (!anglesComputed) computeAngles();
        return yaw * 57.29578f + 180.0f;
    }
    float getRollRadians()
    {
        if (!anglesComputed) computeAngles();
        return roll;
    }
    float getPitchRadians()
    {
        if (!anglesComputed) computeAngles();
        return pitch;
    }
    float getYawRadians()
    {
        if (!anglesComputed) computeAngles();
        return yaw;
    }
};

#endif  // MAHONY_AHRS_H_
