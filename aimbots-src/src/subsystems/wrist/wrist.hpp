#pragma once

#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

#ifdef WRIST_COMPATIBLE

// static inline float DJIEncoderValueToRadians(int64_t encoderValue) {
//     return (M_TWOPI * static_cast<float>(encoderValue)) / DJIMotor::ENC_RESOLUTION;
// }

namespace src::Wrist {

enum MotorIndex {YAW, PITCH, ROLL};

class WristSubsystem : public tap::control::Subsystem {
public:
    WristSubsystem(src::Drivers*);

    void initialize() override;
    void refresh() override;

    bool isMotorOnline(MotorIndex motorIdx) const {
        return motors[motorIdx]->isMotorOnline();
    }

    bool isOnline() const {
        for (auto&& motor : motors)
            if (!motor->isMotorOnline()) return false;

        return true;
    }

    template<class... Args>
    void ForAllWristMotors(DJIMotorFunc<Args...> func, Args... args) {
        for (size_t i = 0; i < WRIST_MOTOR_COUNT; i++)
            (motors[i]->*func)(args...);
    }

    template<class... Args>
    void ForAllWristMotors(void (WristSubsystem::*func)(MotorIndex, Args...), Args... args) {
        for (size_t i = 0; i < WRIST_MOTOR_COUNT; i++) {
            auto mi = static_cast<MotorIndex>(i);
            (this->*func)(mi, args...);
        }
    }

    /**
     * @brief calculates angles based on desired arm grabber position
     * reference frame unknown
     * @param x
     * @param y
     * @param z
     */
    void calculateArmAngles(uint16_t x, uint16_t y, uint16_t z);

    void updateMotorPositionPID(MotorIndex);

    void setDesiredOutput(MotorIndex motorIdx, float desiredOutput) {
        desiredMotorOutputs[motorIdx] = desiredOutput;
    }

    void setDesiredOutputToMotor(MotorIndex motorIdx) {
        motors[motorIdx]->setDesiredOutput(desiredMotorOutputs[motorIdx]);
    }

    /** 
     * Get the current unwrapped radians of the given motor after gear box ratios
     * 
     * @param motorIdx
     */
    float getCurrentAngleUnwrappedRadians(MotorIndex motorIdx) const {
        return DJIEncoderValueToRadians(motors[motorIdx]->getEncoderUnwrapped() / WRIST_GEAR_RATIOS[motorIdx]);
    }

    /**
     * Gets the given motor's current RPM, or 0 if it's offline
     * 
     * @param motorIdx
    */
    int16_t getMotorRPM(MotorIndex motorIdx) const {
        return isMotorOnline(motorIdx) ? motors[motorIdx]->getShaftRPM() : 0;
    }

    /**
     * Gets the given motor's current torque, or 0 if it's offline
     * 
     * @param motorIdx
    */
    int16_t getMotorTorque(MotorIndex motorIdx) const {
        return isMotorOnline(motorIdx) ? motors[motorIdx]->getTorque() : 0;
    }

    /**
     * Sets the desired target angle of the target motor after gear box ratios
     * 
     * @param motorIdx
     * @param angle the angle in radians from -pi to pi
    */
    void setTargetAngle(MotorIndex motorIdx, float angle) {
        targetAngles[motorIdx]->setValue(angle);
    }

private:
    src::Drivers* drivers;

    std::array<DJIMotor*, WRIST_MOTOR_COUNT> motors;
    std::array<SmoothPID*, WRIST_MOTOR_COUNT> positionPIDs;

    /** The target desired angles of each motor AFTER scaling for the gear boxes */
    std::array<ContiguousFloat*, WRIST_MOTOR_COUNT> targetAngles;

    /** The current angles of each motor AFTER scaling for the gear boxes */
    std::array<ContiguousFloat*, WRIST_MOTOR_COUNT> currentAngles;

    std::array<float, WRIST_MOTOR_COUNT> desiredMotorOutputs;

    void updateCurrentAngle(MotorIndex);
};

};  // namespace src::Wrist

#endif  // #ifdef WRIST_COMPATIBLE