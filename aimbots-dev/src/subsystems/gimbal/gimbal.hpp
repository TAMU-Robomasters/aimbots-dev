#pragma once

#include <drivers.hpp>
#include <tap/algorithms/contiguous_float.hpp>
#include <tap/algorithms/math_user_utils.hpp>
#include <tap/control/subsystem.hpp>
#include <utils/common_types.hpp>

namespace src::Gimbal {

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
    inline void  setTargetYawAngle(AngleUnit unit, float angle) { targetYawAngle = (unit == AngleUnit::Degrees) ? modm::toRadian(angle) : angle; }
    inline float getTargetPitchAngle(AngleUnit unit) const { return (unit == AngleUnit::Degrees) ? modm::toDegree(targetPitchAngle) : targetPitchAngle; }
    inline void  setTargetPitchAngle(AngleUnit unit, float angle) { targetPitchAngle = (unit == AngleUnit::Degrees) ? modm::toRadian(angle) : angle; }

    inline float getCurrentYawAngle(AngleUnit unit) const { return (unit == AngleUnit::Degrees) ? modm::toDegree(currentYawAngle.getValue()) : currentYawAngle.getValue(); }
    inline float getCurrentPitchAngle(AngleUnit unit) const { return (unit == AngleUnit::Degrees) ? modm::toDegree(currentPitchAngle.getValue()) : currentPitchAngle.getValue(); }

    float getCurrentYawAngleFromCenter(AngleUnit) const;
    float getCurrentPitchAngleFromCenter(AngleUnit) const;

    inline tap::algorithms::ContiguousFloat const& getCurrentYawAngleAsContiguousFloat() const { return currentYawAngle; }
    inline tap::algorithms::ContiguousFloat const& getCurrentPitchAngleAsContiguousFloat() const { return currentPitchAngle; }

#include <utils/robot_specific_inc.hpp>

   private:
    DJIMotor yawMotor;
    DJIMotor pitchMotor;

    tap::algorithms::ContiguousFloat currentYawAngle;    // in Radians
    tap::algorithms::ContiguousFloat currentPitchAngle;  // in Radians
    float targetYawAngle;    // in Radians
    float targetPitchAngle;  // in Radians
};

}  // namespace src::Gimbal