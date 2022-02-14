#pragma once

#include <drivers.hpp>
#include <modm/math.hpp>
#include <tap/control/subsystem.hpp>
#include <utils/common_types.hpp>

namespace src::Gimbal {

class GimbalSubsystem : public tap::control::Subsystem {
public:
    GimbalSubsystem(src::Drivers*);

    void initialize() override;
    void refresh() override;

    const char* getName() override { return "Gimbal Subsystem"; }

    void setYawMotorOutput(float output);
    void setPitchMotorOutput(float output);

    inline void setTargetYawAngleInDegrees(float angle) { targetYawAngle = modm::toRadian(angle); }
    inline void setTargetYawAngleInRadians(float angle) { targetYawAngle = angle; }
    inline void displacePitchYawAngleInDegrees(float delta) { targetYawAngle = currentYawAngle + modm::toRadian(delta); }
    inline void displacePitchYawAngleInRadians(float delta) { targetYawAngle = currentYawAngle + delta; }

    inline void setTargetPitchAngleInDegrees(float angle) { targetPitchAngle = modm::toRadian(angle); }
    inline void setTargetPitchAngleInRadians(float angle) { targetPitchAngle = angle; }
    inline void displaceTargetPitchAngleInDegrees(float delta) { targetPitchAngle = currentPitchAngle + modm::toRadian(delta); }
    inline void displaceTargetPitchAngleInRadians(float delta) { targetPitchAngle = currentPitchAngle + delta; }

    inline float getCurrentYawAngleInDegrees() const { return modm::toDegree(currentYawAngle); }
    inline float getCurrentYawAngleInRadians() const { return currentYawAngle; }
    inline float getCurrentPitchAngleInDegrees() const { return modm::toDegree(currentPitchAngle); }
    inline float getCurrentPitchAngleInRadians() const { return currentPitchAngle; }

    inline float getTargetYawAngleInDegrees() const { return modm::toDegree(targetYawAngle); }
    inline float getTargetYawAngleInRadians() const { return targetYawAngle; }
    inline float getTargetPitchAngleInDegrees() const { return modm::toDegree(targetPitchAngle); }
    inline float getTargetPitchAngleInRadians() const { return targetPitchAngle; }

private:
    DJIMotor yawMotor;
    DJIMotor pitchMotor;

    float currentYawAngle;   // in Radians
    float currentPitchAngle; // in Radians

    float targetYawAngle;   // in Radians
    float targetPitchAngle; // in Radians

    #include <utils/robot_constants.hpp>
};

}  // namespace src::Gimbal