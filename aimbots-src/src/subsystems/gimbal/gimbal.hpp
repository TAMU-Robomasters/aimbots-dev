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

    inline float getCurrentYawAngle(AngleUnit unit) const {
        return (unit == AngleUnit::Degrees) ? modm::toDegree(currentYawAngle.getValue()) : currentYawAngle.getValue();
    }
    inline float getCurrentPitchAngle(AngleUnit unit) const {
        return (unit == AngleUnit::Degrees) ? modm::toDegree(currentPitchAngle.getValue()) : currentPitchAngle.getValue();
    }

    float getChassisRelativeYawAngle(AngleUnit) const;
    float getChassisRelativePitchAngle(AngleUnit) const;

    inline tap::algorithms::ContiguousFloat const& getCurrentYawMotorAngleAsContiguousFloat() const {
        return currentYawAngle;
    }
    inline tap::algorithms::ContiguousFloat const& getCurrentPitchMotorAngleAsContiguousFloat() const {
        return currentPitchAngle;
    }

    inline float getYawMotorRPM() const { return (yawMotor.isMotorOnline()) ? yawMotor.getShaftRPM() : 0.0f; }
    inline float getPitchMotorRPM() const { return (pitchMotor.isMotorOnline()) ? pitchMotor.getShaftRPM() : 0.0f; }

    inline float getTargetYawMotorAngle(AngleUnit unit) const {
        return (unit == AngleUnit::Degrees) ? modm::toDegree(targetYawAngle.getValue()) : targetYawAngle.getValue();
    }
    inline void setTargetYawAngle(AngleUnit unit, float angle) {
        angle = (unit == AngleUnit::Degrees) ? modm::toRadian(angle) : angle;
        targetYawAngle = ContiguousFloat(angle, 0, M_TWOPI);
    }

    inline float getTargetPitchAngle(AngleUnit unit) const {
        return (unit == AngleUnit::Degrees) ? modm::toDegree(targetPitchAngle.getValue()) : targetPitchAngle.getValue();
    }
    inline void setTargetPitchAngle(AngleUnit unit, float angle) {
        angle = (unit == AngleUnit::Degrees) ? modm::toRadian(angle) : angle;
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