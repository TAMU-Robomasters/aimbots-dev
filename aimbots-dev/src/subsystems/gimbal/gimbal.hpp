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

    inline void setYawAngleInDegrees(float angle);
    inline void setPitchAngleInDegrees(float angle);
    inline void setYawAngleInRadians(float angle);
    inline void setPitchAngleInRadians(float angle);

    inline void displaceYawAngleInDegrees(float displacement);
    inline void displacePitchAngleInDegrees(float displacement);
    inline void displaceYawAngleInRadians(float displacement);
    inline void displacePitchAngleInRadians(float displacement);

    float getCurrentYawAngleInDegrees() const;
    float getCurrentPitchAngleInDegrees() const;
    float getCurrentYawAngleInRadians() const;
    float getCurrentPitchAngleInRadians() const;
    inline float getTargetYawAngleInDegrees() const;
    inline float getTargetPitchAngleInDegrees() const;
    inline float getTargetYawAngleInRadians() const;
    inline float getTargetPitchAngleInRadians() const;

private:
    DJIMotor yawMotor;
    DJIMotor pitchMotor;

    StockPID yawPID;
    StockPID pitchPID;

    float targetYawAngle;   // in Radians
    float targetPitchAngle; // in Radians

    #include <utils/robot_constants.hpp>
};

}  // namespace src::Gimbal