#pragma once

#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

#ifdef WRIST_COMPATIBLE

// static inline float DJIEncoderValueToRadians(int64_t encoderValue) {
//     return (M_TWOPI * static_cast<float>(encoderValue)) / DJIMotor::ENC_RESOLUTION;
// }

namespace src::Wrist {

enum MotorIndex { YAW = 0, PITCH = 1, ROLL = 2 };

class WristSubsystem : public tap::control::Subsystem {
public:
    WristSubsystem(src::Drivers* drivers);

    void initialize() override;
    void refresh() override;

    // Returns whether or not ALL motors are online
    inline bool isOnline() const {
        // maybe change idk
        // return yawMotor.isOnline() && pitchMotor.isOnline() && rollMotor.isOnline();
        for (auto&& motors : motors) {
            if (!motors->isMotorOnline()) return false;
        }

        return true;
    }

    // Initializes motor array, angle arrays, positionPID array
    void BuildMotors() {
        for (auto i = 0; i < 3; i++) {
            motors[i] =
                new DJIMotor(drivers, YAW_MOTOR_IDS[i], (CANBus)WRIST_BUS, WRIST_MOTOR_DIRECTIONS[i], WRIST_MOTOR_NAMES[i]);
            currentAngle[i] = new tap::algorithms::ContiguousFloat(0.0f, -M_PI, M_PI);
            // targetAngle[i] = new tap::algorithms::ContiguousFloat(0.0f, -M_PI, M_PI);
            positionPID[i] = new SmoothPID(WRIST_VELOCITY_PID_CONFIG);
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

    // setter functions
    /**
     * we should try to limit everything to one function
     * so only one function to set the angle for each motor
     *
     */

    void setDesiredOutputToMotor(uint8_t idx);
    //
    void updateCurrentMotorAngles();
    void updatePositionPID(int idx);

    /**
     * @brief gets angle but
     *
     * @param motorID
     * @return float
     */
    // #warning engineer wrist doesnt have gear ratios yet
    inline float getCurrentAngleWrapped(uint16_t motorID) const {
        return (DJIEncoderValueToRadians(motors[motorID]->getEncoderUnwrapped()));
    }

    inline int16_t getYawMotorRPM() const { return (motors[YAW]->isMotorOnline()) ? motors[YAW]->getShaftRPM() : 0; }
    inline int16_t getPitchMotorRPM() const { return (motors[PITCH]->isMotorOnline()) ? motors[PITCH]->getShaftRPM() : 0; }
    inline int16_t getRollMotorRPM() const { return (motors[ROLL]->isMotorOnline()) ? motors[ROLL]->getShaftRPM() : 0; }

    inline int16_t getYawMotorTorque() const { return (motors[YAW]->isMotorOnline()) ? motors[YAW]->getTorque() : 0; }
    inline int16_t getPitchMotorTorque() const { return (motors[PITCH]->isMotorOnline()) ? motors[PITCH]->getTorque() : 0; }
    inline int16_t getRollMotorTorque() const { return (motors[ROLL]->isMotorOnline()) ? motors[ROLL]->getTorque() : 0; }

    inline void setTargetAngle(int idx, float angle) { targetAngle[idx]->setValue(angle); }

private:
    src::Drivers* drivers;

    // DJIMotor yawMotor, pitchMotor, rollMotor;
    // SmoothPID yawPID, pitchPID, rollPID;

    std::array<DJIMotor*, 3> motors;
    std::array<SmoothPID*, 3> positionPID;  // "position" means angle
    std::array<ContiguousFloat*, 3> currentAngle;
    std::array<ContiguousFloat*, 3> targetAngle;
    std::array<float, 3> desiredMotorOutputs;

    uint16_t WRIST_BUS = 0;

    // SmoothPID yawPID;
    // SmoothPID pitchPID;
    // SmoothPID rollPID;

    // original was tap::algorithms::ContiguousFloat return type
    float currentYawAngle() { return DJIEncoderValueToRadians(motors[MotorIndex::YAW]->getEncoderUnwrapped()); };  // radians
    float currentPitchAngle() { return DJIEncoderValueToRadians(motors[MotorIndex::PITCH]->getEncoderUnwrapped()); };
    float currentRollAngle() { return DJIEncoderValueToRadians(motors[MotorIndex::ROLL]->getEncoderUnwrapped()); };

    // float targetYawAngle() const { /*return targetYawAngle;*/ }; // radians
    // float targetPitchAngle() const { /*return targetPitchAngle;*/ };
    // float targetRollAngle() const { /*return targetRollAngle;*/ };
};
};  // namespace src::Wrist

#endif  // #ifdef WRIST_COMPATIBLE