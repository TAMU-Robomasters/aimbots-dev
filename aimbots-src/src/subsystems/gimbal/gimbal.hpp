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

class GimbalSubsystem : public tap::control::Subsystem {
public:
    struct aimAngles {
        double pitch, yaw;
    };

    GimbalSubsystem(src::Drivers*);

    void initialize() override;
    void refresh() override;

    const char* getName() override { return "Gimbal Subsystem"; }

    inline bool isOnline() const { return yawMotor.isMotorOnline() && pitchMotor.isMotorOnline(); }

    void setYawMotorOutput(float output);
    void setPitchMotorOutput(float output);

    inline float getCurrentYawMotorAngle(AngleUnit unit) const {
        return (unit == AngleUnit::Degrees) ? modm::toDegree(currentYawMotorAngle.getValue())
                                            : currentYawMotorAngle.getValue();
    }
    inline float getCurrentPitchMotorAngle(AngleUnit unit) const {
        return (unit == AngleUnit::Degrees) ? modm::toDegree(currentPitchMotorAngle.getValue())
                                            : currentPitchMotorAngle.getValue();
    }

    float getChassisRelativeYawAngle(AngleUnit) const;
    float getChassisRelativePitchAngle(AngleUnit) const;

    inline tap::algorithms::ContiguousFloat const& getCurrentYawMotorAngleAsContiguousFloat() const {
        return currentYawMotorAngle;
    }
    inline tap::algorithms::ContiguousFloat const& getCurrentPitchMotorAngleAsContiguousFloat() const {
        return currentPitchMotorAngle;
    }

    inline float getYawMotorRPM() const { return (yawMotor.isMotorOnline()) ? yawMotor.getShaftRPM() : 0.0f; }
    inline float getPitchMotorRPM() const { return (pitchMotor.isMotorOnline()) ? pitchMotor.getShaftRPM() : 0.0f; }

    inline float getTargetYawMotorAngle(AngleUnit unit) const {
        return (unit == AngleUnit::Degrees) ? modm::toDegree(targetYawMotorAngle.getValue())
                                            : targetYawMotorAngle.getValue();
    }
    inline void setTargetYawMotorAngle(AngleUnit unit, float angle) {
        angle = (unit == AngleUnit::Degrees) ? modm::toRadian(angle) : angle;
        targetYawMotorAngle = ContiguousFloat(angle, 0, M_TWOPI);
    }

    inline float getTargetPitchMotorAngle(AngleUnit unit) const {
        return (unit == AngleUnit::Degrees) ? modm::toDegree(targetPitchMotorAngle.getValue())
                                            : targetPitchMotorAngle.getValue();
    }
    inline void setTargetPitchMotorAngle(AngleUnit unit, float angle) {
        angle = (unit == AngleUnit::Degrees) ? modm::toRadian(angle) : angle;
        int status = 0;
        targetPitchMotorAngle.setValue(ContiguousFloat::limitValue(
            ContiguousFloat(angle, 0, M_TWOPI),
            modm::toRadian((!PITCH_DIRECTION) ? PITCH_SOFTSTOP_HIGH : PITCH_SOFTSTOP_LOW),
            modm::toRadian((!PITCH_DIRECTION) ? PITCH_SOFTSTOP_LOW : PITCH_SOFTSTOP_HIGH),
            &status));
    }

private:
    src::Drivers* drivers;

    DJIMotor yawMotor;
    DJIMotor pitchMotor;

    tap::algorithms::ContiguousFloat currentYawMotorAngle;    // In radians
    tap::algorithms::ContiguousFloat currentPitchMotorAngle;  // In radians

    tap::algorithms::ContiguousFloat targetYawMotorAngle;    // in Radians
    tap::algorithms::ContiguousFloat targetPitchMotorAngle;  // in Radians

    float desiredYawMotorOutput;
    float desiredPitchMotorOutput;
};

}  // namespace src::Gimbal