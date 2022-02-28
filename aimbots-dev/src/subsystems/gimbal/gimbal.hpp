#pragma once

#include <drivers.hpp>
#include <tap/algorithms/contiguous_float.hpp>
#include <tap/algorithms/math_user_utils.hpp>
#include <tap/control/subsystem.hpp>
#include <utils/common_types.hpp>

namespace src::Gimbal {

class GimbalSubsystem : public tap::control::Subsystem {
   public:
    GimbalSubsystem(src::Drivers*);

    void initialize() override;
    void refresh() override;

    const char* getName() override { return "Gimbal Subsystem"; }

    inline bool isOnline() const { return yawMotor.isMotorOnline() && pitchMotor.isMotorOnline(); }

    void setYawMotorOutput(float output);
    void setPitchMotorOutput(float output);

    inline void setTargetYawAngleInDegrees(float angle) { targetYawAngle = modm::toRadian(angle); }
    inline void setTargetYawAngleInRadians(float angle) { targetYawAngle = angle; }
    inline void setTargetPitchAngleInDegrees(float angle) { targetPitchAngle = modm::toRadian(angle); }
    inline void setTargetPitchAngleInRadians(float angle) { targetPitchAngle = angle; }

    inline float getYawMotorVelocity() const { return getMotorVelocity(&yawMotor); }
    inline float getPitchMotorVelocity() const { return getMotorVelocity(&pitchMotor); }

    inline float getCurrentYawAngleInDegrees() const { return modm::toDegree(currentYawAngle.getValue()); }
    inline float getCurrentYawAngleInRadians() const { return currentYawAngle.getValue(); }
    inline float getCurrentPitchAngleInDegrees() const { return modm::toDegree(currentPitchAngle.getValue()); }
    inline float getCurrentPitchAngleInRadians() const { return currentPitchAngle.getValue(); }

    float getCurrentYawAngleFromCenterInDegrees() const;
    float getCurrentYawAngleFromCenterInRadians() const;
    float getCurrentPitchAngleFromCenterInDegrees() const;
    float getCurrentPitchAngleFromCenterInRadians() const;

    inline tap::algorithms::ContiguousFloat const& getContiguousCurrentYawAngle() const { return currentYawAngle; }
    inline tap::algorithms::ContiguousFloat const& getContiguousCurrentPitchAngle() const { return currentPitchAngle; }

    inline float getTargetYawAngleInDegrees() const { return modm::toDegree(targetYawAngle); }
    inline float getTargetYawAngleInRadians() const { return targetYawAngle; }
    inline float getTargetPitchAngleInDegrees() const { return modm::toDegree(targetPitchAngle); }
    inline float getTargetPitchAngleInRadians() const { return targetPitchAngle; }

#include <utils/robot_specific_inc.hpp>

   private:
    DJIMotor yawMotor;
    DJIMotor pitchMotor;

    tap::algorithms::ContiguousFloat currentYawAngle;    // in Radians
    tap::algorithms::ContiguousFloat currentPitchAngle;  // in Radians

    float targetYawAngle;    // in Radians
    float targetPitchAngle;  // in Radians

    static inline float getMotorVelocity(DJIMotor const* motor) {
        return 360 / 60 * motor->getShaftRPM();
    }
};

}  // namespace src::Gimbal