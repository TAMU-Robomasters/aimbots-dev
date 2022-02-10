#pragma once

#include <drivers.hpp>
#include <tap/control/subsystem.hpp>
#include <utils/common_types.hpp>

namespace src::Gimbal {

class GimbalSubsystem : public tap::control::Subsystem {
public:
    GimbalSubsystem(src::Drivers*);

    void initialize() override;
    void refresh() override;

    const char* getName() override { return "Gimbal"; }

    inline void setYawMotorOutputAngleInDegrees(float angle);
    inline void setYawMotorOutputAngleInRadians(float angle);
    inline void displaceYawMotorOutputAngleInDegrees(float angle);
    inline void displaceYawMotorOutputAngleInRadians(float angle);

    inline void setPitchMotorOutputAngleInDegrees(float angle);
    inline void setPitchMotorOutputAngleInRadians(float angle);
    inline void displacePitchMotorOutputAngleInDegrees(float angle);
    inline void displacePitchMotorOutputAngleInRadians(float angle);

    inline float getCurrentYawAngleInDegrees() const;
    inline float getCurrentYawAngleInRadians() const;
    inline float getCurrentPitchAngleInDegrees() const;
    inline float getCurrentPitchAngleInRadians() const;

    inline float getTargetYawAngleInDegrees() const;
    inline float getTargetPitchAngleInDegrees() const;
    inline float getTargetYawAngleInRadians() const;
    inline float getTargetPitchAngleInRadians() const;

private:
    DJIMotor yawMotor;
    DJIMotor pitchMotor;

    StockPID yawPID;
    StockPID pitchPID;

    float currentYawAngle;   // in Radians
    float currentPitchAngle; // in Radians

    float targetYawAngle;   // in Radians
    float targetPitchAngle; // in Radians

    #include <utils/robot_constants.hpp>
};

}  // namespace src::Gimbal