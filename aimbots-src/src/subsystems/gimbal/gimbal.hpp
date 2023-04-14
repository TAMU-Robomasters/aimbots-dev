#pragma once

#include <complex>
#include <vector>

#include <tap/algorithms/contiguous_float.hpp>
#include <tap/algorithms/math_user_utils.hpp>
#include <tap/control/subsystem.hpp>
#include <utils/common_types.hpp>
#include <utils/robot_specific_inc.hpp>

namespace src {
class Drivers;
}

namespace src::Gimbal {

enum GimbalAxis { YAW_AXIS = 0, PITCH_AXIS = 1 };

class GimbalSubsystem : public tap::control::Subsystem {
public:
    GimbalSubsystem(src::Drivers*);
    ~GimbalSubsystem() = default;

    void initialize() override;
    void refresh() override;

    const char* getName() override { return "Gimbal Subsystem"; }

    inline bool isOnline() const { return yawMotor.isMotorOnline() && pitchMotor.isMotorOnline(); }

    void setYawMotorOutput(float output);
    void setPitchMotorOutput(float output);

    inline float GimbalSubsystem::getChassisRelativeYawAngle(AngleUnit unit) const {
        return (unit == AngleUnit::Radians) ? currentYawAngle.getValue() : modm::toDegree(currentYawAngle.getValue());
    }

    inline float GimbalSubsystem::getChassisRelativePitchAngle(AngleUnit unit) const {
        return (unit == AngleUnit::Radians) ? currentPitchAngle.getValue() : modm::toDegree(currentPitchAngle.getValue());
    }

    inline tap::algorithms::ContiguousFloat const& getCurrentYawMotorAngleAsContiguousFloat() const {
        return currentYawAngle;
    }
    inline tap::algorithms::ContiguousFloat const& getCurrentPitchMotorAngleAsContiguousFloat() const {
        return currentPitchAngle;
    }

    inline float getYawMotorRPM() const { return (yawMotor.isMotorOnline()) ? yawMotor.getShaftRPM() : 0.0f; }
    inline float getPitchMotorRPM() const { return (pitchMotor.isMotorOnline()) ? pitchMotor.getShaftRPM() : 0.0f; }

    inline float getYawMotorAngleWrapped() const { return (yawMotor.isMotorOnline()) ? yawMotor.getEncoderWrapped() : 0.0f; }
    inline float getPitchMotorAngleWrapped() const {
        return (pitchMotor.isMotorOnline()) ? pitchMotor.getEncoderWrapped() : 0.0f;
    }

    inline float getTargetYawAngle(AngleUnit unit) const {
        return (unit == AngleUnit::Radians) ? targetYawAngle.getValue() : modm::toRadian(targetYawAngle.getValue());
    }
    inline void setTargetYawAngle(AngleUnit unit, float angle) {
        angle = (unit == AngleUnit::Radians) ? angle : modm::toRadian(angle);
        targetYawAngle = ContiguousFloat(angle, -M_PI, M_PI);
    }

    inline float getTargetPitchAngle(AngleUnit unit) const {
        return (unit == AngleUnit::Radians) ? targetPitchAngle.getValue() : modm::toRadian(targetPitchAngle.getValue());
    }
    inline void setTargetPitchAngle(AngleUnit unit, float angle) {
        angle = (unit == AngleUnit::Radians) ? angle : modm::toRadian(angle);
        int status = 0;
        targetPitchAngle.setValue(ContiguousFloat::limitValue(
            ContiguousFloat(angle, 0, M_TWOPI),
            modm::toRadian((!PITCH_DIRECTION) ? PITCH_SOFTSTOP_HIGH : PITCH_SOFTSTOP_LOW),
            modm::toRadian((!PITCH_DIRECTION) ? PITCH_SOFTSTOP_LOW : PITCH_SOFTSTOP_HIGH),
            &status));
    }

private:
    src::Drivers* drivers;

    DJIMotor yawMotor;
    DJIMotor pitchMotor;

    tap::algorithms::ContiguousFloat currentYawAngle;    // In radians
    tap::algorithms::ContiguousFloat currentPitchAngle;  // In radians

    tap::algorithms::ContiguousFloat targetYawAngle;    // in Radians
    tap::algorithms::ContiguousFloat targetPitchAngle;  // in Radians

    float desiredYawMotorOutput;
    float desiredPitchMotorOutput;
};

}  // namespace src::Gimbal