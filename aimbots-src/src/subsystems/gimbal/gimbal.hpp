#pragma once
#ifndef TARGET_ENGINEER

#include <tap/algorithms/contiguous_float.hpp>
#include <tap/algorithms/math_user_utils.hpp>
#include <tap/control/subsystem.hpp>
#include <utils/common_types.hpp>
#include <utils/robot_specific_inc.hpp>
#include <vector>
#include <complex>
#include <utils/math/root_finding.hpp>

namespace src {
class Drivers;
}

namespace src::Gimbal {

constexpr inline float constAbs(float value) {
    return (value < 0.0f) ? (value * -1.0f) : value;
}

class GimbalSubsystem : public tap::control::Subsystem {
   public:

    struct aimAngles {
        float pitch, yaw;
    };

    GimbalSubsystem(src::Drivers*);

    void initialize() override;
    void refresh() override;

    const char* getName() override { return "Gimbal Subsystem"; }

    inline bool isOnline() const { return yawMotor.isMotorOnline() && pitchMotor.isMotorOnline(); }

    void setYawMotorOutput(float output);
    void setPitchMotorOutput(float output);

    aimAngles getAimAngles(/*Insert Struct Here*/);

    inline float getTargetChassisRelativeYawAngle(AngleUnit unit) const { return (unit == AngleUnit::Degrees) ? modm::toDegree(targetChassisRelativeYawAngle) : targetChassisRelativeYawAngle; }
    inline void setTargetChassisRelativeYawAngle(AngleUnit unit, float angle) {
        angle = (unit == AngleUnit::Degrees) ? modm::toRadian(angle) : angle;
        targetChassisRelativeYawAngle = ContiguousFloat(angle, 0, M_TWOPI).getValue();
    }

    inline float getTargetChassisRelativePitchAngle(AngleUnit unit) const { return (unit == AngleUnit::Degrees) ? modm::toDegree(targetChassisRelativePitchAngle) : targetChassisRelativePitchAngle; }
    inline void setTargetChassisRelativePitchAngle(AngleUnit unit, float angle) {
        angle = (unit == AngleUnit::Degrees) ? modm::toRadian(angle) : angle;
        int status = 0;
        targetChassisRelativePitchAngle = ContiguousFloat::limitValue(
            ContiguousFloat(angle, 0, M_TWOPI),
            modm::toRadian((!PITCH_DIRECTION) ? PITCH_SOFTSTOP_HIGH : PITCH_SOFTSTOP_LOW),
            modm::toRadian((!PITCH_DIRECTION) ? PITCH_SOFTSTOP_LOW : PITCH_SOFTSTOP_HIGH),
            &status);
    }

    inline float getCurrentFieldRelativeYawAngle(AngleUnit unit) const { return (unit == AngleUnit::Degrees) ? modm::toDegree(currentFieldRelativeYawAngle.getValue()) : currentFieldRelativeYawAngle.getValue(); }
    inline float getCurrentChassisRelativeYawAngle(AngleUnit unit) const { return (unit == AngleUnit::Degrees) ? modm::toDegree(currentChassisRelativeYawAngle.getValue()) : currentChassisRelativeYawAngle.getValue(); }
    inline float getCurrentChassisRelativePitchAngle(AngleUnit unit) const { return (unit == AngleUnit::Degrees) ? modm::toDegree(currentChassisRelativePitchAngle.getValue()) : currentChassisRelativePitchAngle.getValue(); }

    float getCurrentYawAngleFromChassisCenter(AngleUnit) const;
    float getCurrentPitchAngleFromChassisCenter(AngleUnit) const;

    inline tap::algorithms::ContiguousFloat const& getCurrentChassisRelativeYawAngleAsContiguousFloat() const { return currentChassisRelativeYawAngle; }
    inline tap::algorithms::ContiguousFloat const& getCurrentFieldRelativeYawAngleAsContiguousFloat() const { return currentFieldRelativeYawAngle; }
    inline tap::algorithms::ContiguousFloat const& getCurrentChassisRelativePitchAngleAsContiguousFloat() const { return currentChassisRelativePitchAngle; }

    inline float getYawMotorRPM() const { return (yawMotor.isMotorOnline()) ? yawMotor.getShaftRPM() : 0.0f; }
    inline float getPitchMotorRPM() const { return (pitchMotor.isMotorOnline()) ? pitchMotor.getShaftRPM() : 0.0f; }

   private:
    src::Drivers* drivers;

    DJIMotor yawMotor;
    DJIMotor pitchMotor;

    tap::algorithms::ContiguousFloat currentFieldRelativeYawAngle;      // In radians
    tap::algorithms::ContiguousFloat currentChassisRelativeYawAngle;    // In radians
    tap::algorithms::ContiguousFloat currentChassisRelativePitchAngle;  // In radians

    float targetChassisRelativeYawAngle;    // in Radians
    float targetChassisRelativePitchAngle;  // in Radians

    float desiredYawMotorOutput;
    float desiredPitchMotorOutput;

    vector<double> bulletDropCoEff;

    
};

}  // namespace src::Gimbal
#endif