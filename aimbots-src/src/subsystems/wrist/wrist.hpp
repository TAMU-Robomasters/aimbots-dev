#pragma once

#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

#ifdef WRIST_COMPATIBLE

namespace src::Wrist {

enum MotorIndex { YAW, PITCH, ROLL };
enum MotorControlMode { VELOCITY, POSITION };

// Can go to starting angle plus/minus this angle (yaw, pitch, roll)
static constexpr float WRIST_ANGLE_RANGES[] = {PI / 4, PI / 4, F32_ABSMAX};
class WristSubsystem : public tap::control::Subsystem {
public:
    WristSubsystem(src::Drivers*);

    void initialize() override;
    void refresh() override;

    template <class... Args>
    using WristFunc = void (WristSubsystem::*)(Args...);

    bool isMotorOnline(MotorIndex motorIdx) const { return motors[motorIdx].isMotorOnline(); }

    bool isOnline() const {
        for (auto&& motor : motors)
            if (!motor.isMotorOnline()) return false;

        return true;
    }

    template <class... Args>
    void ForAllWristMotors(DJIMotorFunc<Args...> func, Args... args) {
        for (size_t i = 0; i < WRIST_MOTOR_COUNT; i++) (motors[i].*func)(args...);
    }

    template <class... Args>
    void ForAllWristMotors(WristFunc<MotorIndex, Args...> wristFunc, Args... args) {
        for (size_t i = 0; i < WRIST_MOTOR_COUNT; i++) {
            auto mi = static_cast<MotorIndex>(i);
            (this->*wristFunc)(mi, args...);
        }
    }

    void setControlMode();

    /**
     * @brief calculates angles based on desired arm grabber position
     * reference frame unknown
     */
    void calculateArmAngles(uint16_t x, uint16_t y, uint16_t z);

    void updateAllPIDs();

    float getScaledUnwrappedRadians(MotorIndex) const;

    float getMotorScaledRadsPs(MotorIndex) const;

    float getScaledUnwrappedRadiansOffset(MotorIndex idx) const {
        return getScaledUnwrappedRadians(idx) - WRIST_MOTOR_OFFSET_ANGLES[idx];
    }

    // Sets the desired target angle of the given motor after gear box ratios
    void setTargetAngle(MotorIndex motorIdx, float angleRadians) {
        float maxAngleFromZero = WRIST_ANGLE_RANGES[motorIdx];
        targetAnglesRads[motorIdx] = std::clamp(angleRadians, -maxAngleFromZero, maxAngleFromZero);
    }

    float getTargetAngle(MotorIndex motorIdx) const { return targetAnglesRads[motorIdx]; }

    void setTargetRPM(MotorIndex motorIdx, float targetRPM) { targetRPMs[motorIdx] = targetRPM; }

    float getTargetRPM(MotorIndex motorIdx) const { return targetRPMs[motorIdx]; }

    // Gets the given motor's current RPM, or 0 if it's offline
    int16_t getMotorRPM(MotorIndex motorIdx) const { return isMotorOnline(motorIdx) ? motors[motorIdx].getShaftRPM() : 0; }

    // Gets the given motor's current torque, or 0 if it's offline
    int16_t getMotorTorque(MotorIndex motorIdx) const { return isMotorOnline(motorIdx) ? motors[motorIdx].getTorque() : 0; }

    // Turn off all motors
    void idle() { desiredMotorOutputs = {}; }

private:
    std::array<DJIMotor, WRIST_MOTOR_COUNT> motors;
    std::array<SmoothPID, WRIST_MOTOR_COUNT> positionPIDs;
    std::array<SmoothPID, WRIST_MOTOR_COUNT> velocityPIDs;

    // The target desired angles of each motor AFTER scaling for the gear boxes
    std::array<float, WRIST_MOTOR_COUNT> targetAnglesRads{};
    std::array<float, WRIST_MOTOR_COUNT> targetRPMs{};
    std::array<float, WRIST_MOTOR_COUNT> desiredMotorOutputs{};

    MotorControlMode motor_control_setting = VELOCITY;

    // Update motor PIDs to reach target angle. Don't call alongside updateMotorPID_velocity
    void updateMotorPID(MotorIndex);

    // Update motor PIDs to reach target velocity. Don't call alongside updateMotorPID!
    void updateMotorPID_velocity(MotorIndex);

    DJIMotor buildMotor(MotorIndex idx) {
        return DJIMotor(drivers, WRIST_MOTOR_IDS[idx], WRIST_BUS, WRIST_MOTOR_DIRECTIONS[idx], WRIST_MOTOR_NAMES[idx]);
    }

    void setDesiredOutputToMotor(MotorIndex motorIdx) {
        if (isMotorOnline(motorIdx)) motors[motorIdx].setDesiredOutput(desiredMotorOutputs[motorIdx]);
    }
};

};  // namespace src::Wrist

#endif  // #ifdef WRIST_COMPATIBLE