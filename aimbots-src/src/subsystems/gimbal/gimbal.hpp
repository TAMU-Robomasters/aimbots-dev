#pragma once
#ifndef TARGET_ENGINEER

#include <drivers.hpp>
#include <tap/algorithms/contiguous_float.hpp>
#include <tap/algorithms/math_user_utils.hpp>
#include <tap/control/subsystem.hpp>
#include <utils/common_types.hpp>
#include <utils/robot_specific_inc.hpp>

namespace src::Gimbal {

constexpr inline float constAbs(float value)
{
    return (value < 0.0f) ? (value * -1.0f) : value;
}

// NOTE: This function assumes the hardstops are in degrees
constexpr float getPitchMotorDirection()
{
    constexpr float intialDirection = (PITCH_HARDSTOP_HIGH < PITCH_HARDSTOP_LOW) ? 1.0f : -1.0f;

    // If 0 is somewhere in our available arc of pitch, then we need
    // to flip the direction, because the previous condition would be
    // incorrect.
    if constexpr (constAbs(PITCH_HARDSTOP_HIGH - PITCH_HARDSTOP_LOW) > 180.0f) {
        return intialDirection * -1.0f;
    }

    return intialDirection;
}

enum class AngleUnit : uint8_t {
    Degrees,
    Radians,
};

class GimbalSubsystem : public tap::control::Subsystem {
   public:
    GimbalSubsystem(src::Drivers*);

    void initialize() override;
    void refresh() override;

    const char* getName() override { return "Gimbal Subsystem"; }

    inline bool isOnline() const { return yawMotor.isMotorOnline() && pitchMotor.isMotorOnline(); }

    void setYawMotorOutput(float output);
    void setPitchMotorOutput(float output);

    inline float getTargetYawAngle(AngleUnit unit) const { return (unit == AngleUnit::Degrees) ? modm::toDegree(targetYawAngle) : targetYawAngle; }
    inline void setTargetYawAngle(AngleUnit unit, float angle) { targetYawAngle = (unit == AngleUnit::Degrees) ? modm::toRadian(angle) : angle; }
    inline float getTargetPitchAngle(AngleUnit unit) const { return (unit == AngleUnit::Degrees) ? modm::toDegree(targetPitchAngle) : targetPitchAngle; }
    inline void setTargetPitchAngle(AngleUnit unit, float angle) {
        angle = (unit == AngleUnit::Degrees) ? modm::toRadian(angle) : angle;
        targetPitchAngle = ContiguousFloat::limitValue(
            ContiguousFloat(angle, 0, M_TWOPI),
            modm::toRadian((getPitchMotorDirection() > 0) ? PITCH_HARDSTOP_HIGH : PITCH_HARDSTOP_LOW),
            modm::toRadian((getPitchMotorDirection() > 0) ? PITCH_HARDSTOP_LOW : PITCH_HARDSTOP_HIGH)
        );
    }

    inline float getCurrentYawAngle(AngleUnit unit) const { return (unit == AngleUnit::Degrees) ? modm::toDegree(currentYawAngle.getValue()) : currentYawAngle.getValue(); }
    inline float getCurrentPitchAngle(AngleUnit unit) const { return (unit == AngleUnit::Degrees) ? modm::toDegree(currentPitchAngle.getValue()) : currentPitchAngle.getValue(); }

    float getCurrentYawAngleFromCenter(AngleUnit) const;
    float getCurrentPitchAngleFromCenter(AngleUnit) const;

    inline tap::algorithms::ContiguousFloat const& getCurrentYawAngleAsContiguousFloat() const { return currentYawAngle; }
    inline tap::algorithms::ContiguousFloat const& getCurrentPitchAngleAsContiguousFloat() const { return currentPitchAngle; }

    inline float getYawMotorRPM() const { return (yawMotor.isMotorOnline()) ? yawMotor.getShaftRPM() : 0.0f; }
    inline float getPitchMotorRPM() const { return (pitchMotor.isMotorOnline()) ? pitchMotor.getShaftRPM() : 0.0f; }

   private:
    DJIMotor yawMotor;
    DJIMotor pitchMotor;

    tap::algorithms::ContiguousFloat currentYawAngle;    // in Radians
    tap::algorithms::ContiguousFloat currentPitchAngle;  // in Radians

    float targetYawAngle;    // in Radians
    float targetPitchAngle;  // in Radians

    float desiredYawMotorOutput;
    float desiredPitchMotorOutput;
};

}  // namespace src::Gimbal
#endif