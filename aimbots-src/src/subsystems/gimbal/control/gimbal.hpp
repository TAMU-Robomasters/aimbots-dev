#pragma once

#include <drivers.hpp>
#include <tap/algorithms/wrapped_float.hpp>
#include <tap/control/subsystem.hpp>
#include <utils/tools/common_types.hpp>
#include <utils/tools/robot_specific_inc.hpp>

#ifdef GIMBAL_COMPATIBLE

// static inline float DJIEncoderValueToRadians(int64_t encoderValue) {
//     return (M_TWOPI * static_cast<float>(encoderValue)) / DJIMotor::ENC_RESOLUTION;
// }

namespace src::Gimbal {

enum GimbalAxis { YAW_AXIS = 0, PITCH_AXIS = 1 };

class GimbalSubsystem : public tap::control::Subsystem {
public:
    GimbalSubsystem(src::Drivers*);
    ~GimbalSubsystem() = default;

    void BuildYawMotors() {
        for (auto i = 0; i < YAW_MOTOR_COUNT; i++) {
            yawMotors[i] =
                new DJIMotor(drivers, YAW_MOTOR_IDS[i], YAW_GIMBAL_BUS, YAW_MOTOR_DIRECTIONS[i], YAW_MOTOR_NAMES[i]);
            currentYawAxisAnglesByMotor[i] = new tap::algorithms::WrappedFloat(0.0f, -M_PI, M_PI);
        }
    }

    void BuildPitchMotors() {
        for (auto i = 0; i < PITCH_MOTOR_COUNT; i++) {
            pitchMotors[i] =
                new DJIMotor(drivers, PITCH_MOTOR_IDS[i], PITCH_GIMBAL_BUS, PITCH_MOTOR_DIRECTIONS[i], PITCH_MOTOR_NAMES[i]);
            currentPitchAxisAnglesByMotor[i] = new tap::algorithms::WrappedFloat(0.0f, -M_PI, M_PI);
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
            (pitchMotor->*func)(args...);
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

    const char* getName() const override { return "Gimbal Subsystem"; }

    inline bool isOnline() const {
        bool pitchOnline = false;
        bool yawOnline = false;

        for (auto& yawMotor : yawMotors) {
            if (yawMotor->isMotorOnline()) {
                yawOnline = true;
            }
        }
        for (auto& pitchMotor : pitchMotors) {
            if (pitchMotor->isMotorOnline()) {
                pitchOnline = true;
            }
        }
        return yawOnline && pitchOnline;
    }

    inline bool isYawMotorOnline(uint8_t YawIdx) const {
        if (YawIdx >= YAW_MOTOR_COUNT) {
            return false;
        }
        return yawMotors[YawIdx]->isMotorOnline();
    }

    inline bool isPitchMotorOnline(uint8_t PitchIdx) const {
        if (PitchIdx >= PITCH_MOTOR_COUNT) {
            return false;
        }
        return pitchMotors[PitchIdx]->isMotorOnline();
    }

    void setDesiredYawMotorOutput(uint8_t YawIdx, float output) { desiredYawMotorOutputs[YawIdx] = output; }
    void setDesiredPitchMotorOutput(uint8_t PitchIdx, float output) { desiredPitchMotorOutputs[PitchIdx] = output; }

    void setAllDesiredYawMotorOutputs(uint16_t output) { desiredYawMotorOutputs.fill(output); }
    void setAllDesiredPitchMotorOutputs(uint16_t output) { desiredPitchMotorOutputs.fill(output); }

    inline int16_t getYawMotorRPM(uint8_t YawIdx) const {
        return (yawMotors[YawIdx]->isMotorOnline()) ? yawMotors[YawIdx]->getShaftRPM() : 0;
    }

    inline int16_t getPitchMotorRPM(uint8_t PitchIdx) const {
        return (pitchMotors[PitchIdx]->isMotorOnline()) ? pitchMotors[PitchIdx]->getShaftRPM() : 0;
    }

    inline int16_t getYawMotorTorque(uint8_t yawIdx) const {
        return yawMotors[yawIdx]->isMotorOnline() ? yawMotors[yawIdx]->getTorque() : 0;
    }

    inline int16_t getPitchMotorTorque(uint8_t pitchIdx) const {
        return pitchMotors[pitchIdx]->isMotorOnline() ? pitchMotors[pitchIdx]->getTorque() : 0;
    }

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

    inline void setYawAxisAngleOffset(float offset) { yawAxisOffset = offset; }

    inline float getCurrentYawAxisAngle(AngleUnit unit) const {
        return (unit == AngleUnit::Radians) ? currentYawAxisAngle.getWrappedValue() + yawAxisOffset
                                            : modm::toDegree(currentYawAxisAngle.getWrappedValue() + yawAxisOffset);
    }

    inline float getCurrentPitchAxisAngle(AngleUnit unit) const {
        return (unit == AngleUnit::Radians) ? currentPitchAxisAngle.getWrappedValue()
                                            : modm::toDegree(currentPitchAxisAngle.getWrappedValue());
    }

    inline tap::algorithms::WrappedFloat const& getCurrentYawAxisAngleAsWrappedFloat() const { return currentYawAxisAngle; }
    inline tap::algorithms::WrappedFloat const& getCurrentPitchAxisAngleAsWrappedFloat() const {
        return currentPitchAxisAngle;
    }

    inline float getYawMotorAngleUnwrapped(uint8_t YawIdx) const {
        return (yawMotors[YawIdx]->isMotorOnline()) ? DJIEncoderValueToRadians(yawMotors[YawIdx]->getEncoderUnwrapped())
                                                    : 0.0f;
    }

    inline float getYawMotorAngleWrapped(uint8_t YawIdx) const {
        return (yawMotors[YawIdx]->isMotorOnline()) ? DJIEncoderValueToRadians(yawMotors[YawIdx]->getEncoderWrapped())
                                                    : 0.0f;
    }
    inline float getPitchMotorAngleWrapped(uint8_t PitchIdx) const {
        return (pitchMotors[PitchIdx]->isMotorOnline())
                   ? DJIEncoderValueToRadians(pitchMotors[PitchIdx]->getEncoderWrapped())
                   : 0.0f;
    }

    inline float getTargetYawAxisAngle(AngleUnit unit) const {
        return (unit == AngleUnit::Radians) ? targetYawAxisAngle.getWrappedValue()
                                            : modm::toDegree(targetYawAxisAngle.getWrappedValue());
    }
    inline void setTargetYawAxisAngle(AngleUnit unit, float angle) {
        angle = (unit == AngleUnit::Radians) ? angle : modm::toRadian(angle);
        targetYawAxisAngle.setWrappedValue(angle);
    }

    inline float getTargetPitchAxisAngle(AngleUnit unit) const {
        return (unit == AngleUnit::Radians) ? targetPitchAxisAngle.getWrappedValue()
                                            : modm::toDegree(targetPitchAxisAngle.getWrappedValue());
    }
    inline void setTargetPitchAxisAngle(AngleUnit unit, float angle) {
        angle = (unit == AngleUnit::Radians) ? angle : modm::toRadian(angle);
        targetPitchAxisAngle.setWrappedValue(tap::algorithms::limitVal(angle, PITCH_AXIS_SOFTSTOP_LOW, PITCH_AXIS_SOFTSTOP_HIGH));
    }

    float getYawSetpointError(AngleUnit unit) const {
        return (unit == AngleUnit::Radians) ? targetYawAxisAngle.minDifference(currentYawAxisAngle)
                                            : modm::toDegree(targetYawAxisAngle.minDifference(currentYawAxisAngle));
    }
    float getPitchSetpointError(AngleUnit unit) const {
        return (unit == AngleUnit::Radians) ? targetPitchAxisAngle.minDifference(currentPitchAxisAngle)
                                            : modm::toDegree(targetPitchAxisAngle.minDifference(currentPitchAxisAngle));
    }

    float getYawMotorSetpointError(uint8_t YawIdx, AngleUnit unit) const;
    float getPitchMotorSetpointError(uint8_t PitchIdx, AngleUnit unit) const;

    // from the buffer. gimbal orientation from the buffer.
    inline std::pair<float, float>& getGimbalOrientation(int index) { return gimbalOrientationBuffer[index]; }

    // put in your time, we get the closest orientation entry at that time.
    inline std::pair<float, float>& getGimbalOrientationAtTime(uint32_t time_ms) {
        // assume 2 ms delay between gimbal updates
        int index = std::min(time_ms / 2, GIMBAL_BUFFER_SIZE - 1);
        return gimbalOrientationBuffer[index];
    }
    // just in case?
    inline void clearGimbalOrientationBuffer() { gimbalOrientationBuffer.clear(); }

private:
    src::Drivers* drivers;

    std::array<DJIMotor*, YAW_MOTOR_COUNT> yawMotors;
    std::array<DJIMotor*, PITCH_MOTOR_COUNT> pitchMotors;

    std::array<tap::algorithms::WrappedFloat*, YAW_MOTOR_COUNT> currentYawAxisAnglesByMotor;  // chassis relative, in radians
    std::array<tap::algorithms::WrappedFloat*, PITCH_MOTOR_COUNT>
        currentPitchAxisAnglesByMotor;  // chassis relative, in radians

    std::array<float, YAW_MOTOR_COUNT> desiredYawMotorOutputs;
    std::array<float, PITCH_MOTOR_COUNT> desiredPitchMotorOutputs;

    tap::algorithms::WrappedFloat currentYawAxisAngle;    // average of currentYawAxisAnglesByMotor
    tap::algorithms::WrappedFloat currentPitchAxisAngle;  // chassis relative, in radians

    float yawAxisOffset = 0;  // in radians

    tap::algorithms::WrappedFloat targetYawAxisAngle;    // chassis relative, in radians
    tap::algorithms::WrappedFloat targetPitchAxisAngle;  // chassis relative, in radians

    void setDesiredOutputToYawMotor(uint8_t YawIdx);
    void setDesiredOutputToPitchMotor(uint8_t PitchIdx);

    static const uint32_t GIMBAL_BUFFER_SIZE = 40;

    // gimbal yaw / pitch buffer
    // yaw is first, pitch is second, respectively in the pair
    Deque<std::pair<float, float>, GIMBAL_BUFFER_SIZE> gimbalOrientationBuffer;
};

}  // namespace src::Gimbal

#endif  // #ifdef GIMBAL_COMPATIBLE