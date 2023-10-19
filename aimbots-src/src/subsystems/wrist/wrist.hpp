#pragma once

#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

#ifdef WRIST_COMPATIBLE

namespace src::Wrist {

enum MotorIndex { YAW = 0, PITCH = 1, ROLL = 2 };

class WristSubsystem : public tap::control::Subsystem {
public:
    WristSubsystem(src::Drivers* drivers);

    void initialize() override;
    void refresh() override;
    const char* getName() override { return "Wrist Subsystem"; }

    inline bool isOnline() const {
        // maybe change idk
        // return yawMotor.isOnline() && pitchMotor.isOnline() && rollMotor.isOnline();
        for (auto &&motors : motors)
        {
            if(!motors->isMotorOnline())
                return false;
        }
        
        return true;
    }

    void calculateArmAngles(uint16_t x, uint16_t y, uint16_t z);
    void setArmAngles();  // bruh

    void setTargetYawAngle();
    void setTargetPitchAngle();
    void setTargetRollAngle();

    // getter hell
    inline int16_t getYawMotorRPM() const { return (motors[YAW]->isMotorOnline()) ? motors[YAW]->getShaftRPM() : 0; }
    inline int16_t getPitchMotorRPM() const { return (motors[PITCH]->isMotorOnline()) ? motors[PITCH]->getShaftRPM() : 0; }
    inline int16_t getRollMotorRPM() const { return (motors[ROLL]->isMotorOnline()) ?motors[ROLL]->getShaftRPM() : 0; }

    inline int16_t getYawMotorTorque() const { return (motors[YAW]->isMotorOnline()) ? motors[YAW]->getTorque() : 0; }
    inline int16_t getPitchMotorTorque() const { return (motors[PITCH]->isMotorOnline()) ? motors[PITCH]->getTorque() : 0; }
    inline int16_t getRollMotorTorque() const { return (motors[ROLL]->isMotorOnline()) ? motors[ROLL]->getTorque() : 0; }

    void BuildMotors() {
        for (auto i = 0; i < 3; i++) {
            motors[i] =
                new DJIMotor(drivers, YAW_MOTOR_IDS[i], YAW_GIMBAL_BUS, YAW_MOTOR_DIRECTIONS[i], YAW_MOTOR_NAMES[i]);
            currentAngle[i] = new tap::algorithms::ContiguousFloat(0.0f, -M_PI, M_PI);
            postionPID[i] = new SmoothPID(WRIST_VELOCITY_PID_CONFIG);
        }
    }

private:
    src::Drivers* drivers;

    // DJIMotor yawMotor, pitchMotor, rollMotor;
    // SmoothPID yawPID, pitchPID, rollPID;

    std::array<DJIMotor*, 3> motors;
    std::array<SmoothPID*, 3> postionPID;
    std::array<ContiguousFloat*, 3> currentAngle;

    uint16_t WRIST_BUS = 0;

    //     SmoothPID yawPID;
    //     SmoothPID pitchPID;
    //     SmoothPID rollPID;

    //     tap::algorithms::ContiguousFloat currentYawAngle;  // radians
    //     tap::algorithms::ContiguousFloat currentPitchAngle;
    //     tap::algorithms::ContiguousFloat currentRollAngle;

    //     tap::algorithms::ContiguousFloat targetYawAngle;  // radians
    //     tap::algorithms::ContiguousFloat targetPitchAngle;
    //     tap::algorithms::ContiguousFloat targetRollAngle;
};
};  // namespace src::Wrist

#endif  // #ifdef WRIST_COMPATIBLE