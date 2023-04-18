#pragma once

#include <drivers.hpp>
#include <tap/algorithms/contiguous_float.hpp>
#include <tap/control/subsystem.hpp>
#include <utils/common_types.hpp>
#include <utils/robot_specific_inc.hpp>

namespace src::Gimbal {

enum GimbalAxis { YAW_AXIS = 0, PITCH_AXIS = 1 };

class GimbalSubsystem : public tap::control::Subsystem {
public:
    GimbalSubsystem(src::Drivers*);
    ~GimbalSubsystem() = default;

    void BuildYawMotors() {
        for (auto i = 0; i < YAW_MOTOR_COUNT; i++) {
            yawMotors[i] = new DJIMotor(drivers, YAW_MOTOR_IDS[i], GIMBAL_BUS, YAW_MOTOR_DIRECTIONS[i], YAW_MOTOR_NAMES[i]);
        }
    }

    void BuildPitchMotors() {
        for (auto i = 0; i < PITCH_MOTOR_COUNT; i++) {
            pitchMotors[i] =
                new DJIMotor(drivers, PITCH_MOTOR_IDS[i], GIMBAL_BUS, PITCH_MOTOR_DIRECTIONS[i], PITCH_MOTOR_NAMES[i]);
        }
    }

    /**
     * Allows user to call a DJIMotor member function on all gimbal motors
     *
     * @param function pointer to a member function of DJIMotor
     * @param args arguments to pass to the member function
     */
    template <class... Args>
    void ForAllYawMotors(void (DJIMotor::*func)(Args...), Args... args) {
        for (auto& yawMotor : yawMotors) {
            (yawMotor->*func)(args...);
        }
    }

    template <class... Args>
    void ForAllPitchMotors(void (DJIMotor::*func)(Args...), Args... args) {
        for (auto& pitchMotor : pitchMotors) {
            (pitchMotors->*func)(args...);
        }
    }

    /**
     * Allows user to call a GimbalSubsystem function on all gimbal motors.
     *
     * @param function pointer to a member function of GimbalSubsystem that takes a WheelIndex
     * as its first argument and the motor-per-wheel index as the second argument
     * @param args arguments to pass to the member function
     */
    template <class... Args>
    void ForAllYawMotors(void (GimbalSubsystem::*func)(uint8_t YawIdx, Args...), Args... args) {
        for (uint8_t i = 0; i < YAW_MOTOR_COUNT; i++) {
            (this->*func)(i, args...);
        }
    }
    template <class... Args>
    void ForAllPitchMotors(void (GimbalSubsystem::*func)(uint8_t PitchIdx, Args...), Args... args) {
        for (uint8_t i = 0; i < PITCH_MOTOR_COUNT; i++) {
            (this->*func)(i, args...);
        }
    }

    mockable void initialize() override;
    void refresh() override;

    const char* getName() override { return "Gimbal Subsystem"; }

    inline bool isOnline() const {
        for (auto& yawMotor : yawMotors) {
            if (!yawMotor->isMotorOnline()) {
                return false;
            }
        }
        for (auto& pitchMotor : pitchMotors) {
            if (!pitchMotor->isMotorOnline()) {
                return false;
            }
        }
        return true;
    }

    void setDesiredYawOutput(uint8_t YawIdx, uint16_t output);
    void setDesiredPitchOutput(uint8_t PitchIdx, uint16_t output);

    void setAllDesiredYawOutputs(uint16_t output) { desiredYawMotorOutputs.fill(output); }
    void setAllDesiredPitchOutputs(uint16_t output) { desiredPitchMotorOutputs.fill(output); }

    inline float getYawAxisRPM() const {
        int16_t rpm = 0;
        uint8_t onlineMotors = 0;
        for (auto& yawMotor : yawMotors) {
            if (yawMotor->isMotorOnline()) {
                rpm += yawMotor->getShaftRPM();
                onlineMotors++;
            }
        }
        return (onlineMotors > 0) ? rpm / onlineMotors : 0.0f;
    }
    inline float getPitchAxisRPM() const {
        int16_t rpm = 0;
        uint8_t onlineMotors = 0;
        for (auto& pitchMotor : pitchMotors) {
            if (pitchMotor->isMotorOnline()) {
                rpm += pitchMotor->getShaftRPM();
                onlineMotors++;
            }
        }
        return (onlineMotors > 0) ? rpm / onlineMotors : 0.0f;
    }

    inline float getChassisRelativeYawAngle(AngleUnit unit) const {
        return (unit == AngleUnit::Radians) ? currentYawAngle.getValue() : modm::toDegree(currentYawAngle.getValue());
    }

    inline float getChassisRelativePitchAngle(AngleUnit unit) const {
        return (unit == AngleUnit::Radians) ? currentPitchAngle.getValue() : modm::toDegree(currentPitchAngle.getValue());
    }

    inline tap::algorithms::ContiguousFloat const& getCurrentYawMotorAngleAsContiguousFloat() const {
        return currentYawAngle;
    }
    inline tap::algorithms::ContiguousFloat const& getCurrentPitchMotorAngleAsContiguousFloat() const {
        return currentPitchAngle;
    }

    inline float getYawMotorAngleWrapped(uint8_t YawIdx) const {
        return (yawMotors[YawIdx]->isMotorOnline()) ? yawMotors[YawIdx]->getEncoderWrapped() : 0.0f;
    }
    inline float getPitchMotorAngleWrapped(uint8_t PitchIdx) const {
        return (pitchMotors[PitchIdx]->isMotorOnline()) ? pitchMotors[PitchIdx]->getEncoderWrapped() : 0.0f;
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
        targetPitchAngle.setValue(angle);
    }

    float getYawSetpointError(AngleUnit unit) const {
        return (unit == AngleUnit::Radians) ? targetYawAngle.difference(currentYawAngle)
                                            : modm::toDegree(targetYawAngle.difference(currentYawAngle));
    }
    float getPitchSetpointError(AngleUnit unit) const {
        return (unit == AngleUnit::Radians) ? targetPitchAngle.difference(currentPitchAngle)
                                            : modm::toDegree(targetPitchAngle.difference(currentPitchAngle));
    }

    float getYawMotorSetpointError(uint8_t YawIdx, AngleUnit unit) const;
    float getPitchMotorSetpointError(uint8_t PitchIdx, AngleUnit unit) const;

private:
    src::Drivers* drivers;

    tap::algorithms::ContiguousFloat currentYawAngle;    // In radians
    tap::algorithms::ContiguousFloat currentPitchAngle;  // In radians

    tap::algorithms::ContiguousFloat targetYawAngle;    // in Radians
    tap::algorithms::ContiguousFloat targetPitchAngle;  // in Radians

    std::array<float, YAW_MOTOR_COUNT> desiredYawMotorOutputs;
    std::array<float, PITCH_MOTOR_COUNT> desiredPitchMotorOutputs;

    std::array<DJIMotor*, YAW_MOTOR_COUNT> yawMotors;
    std::array<DJIMotor*, PITCH_MOTOR_COUNT> pitchMotors;

    void setDesiredOutputToYawMotor(uint8_t YawIdx);
    void setDesiredOutputToPitchMotor(uint8_t PitchIdx);
};

}  // namespace src::Gimbal