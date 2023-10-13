#pragma once

#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

#ifdef WRIST_COMPATIBLE

namespace src::Wrist {
class WristSubsystem : public tap::control::Subsystem {
public:
    WristSubsystem(src::Drivers* drivers);

    void initialize() override;
    void refresh() override;
    const char* getName() override { return "Wrist Subsystem"; }

    inline bool isOnline() const {
        // maybe change idk
        // return yawMotor.isOnline() && pitchMotor.isOnline() && rollMotor.isOnline();
        return true;
    }

    void calculateArmAngles(uint16_t x, uint16_t y, uint16_t z);
    void setArmAngles();  // bruh

    void setTargetYawAngle();
    void setTargetPitchAngle();
    void setTargetRollAngle();

    // getter hell
    // inline int16_t getYawMotorRPM() const { return (yawMotor.isMotorOnline()) ? yawMotor.getShaftRPM() : 0; }
    // inline int16_t getPitchMotorRPM() const { return (pitchMotor.isMotorOnline()) ? pitchMotor.getShaftRPM() : 0; }
    // inline int16_t getRollMotorRPM() const { return (rollMotor.isMotorOnline()) ? rollMotor.getShaftRPM() : 0; }

    // inline int16_t getYawMotorTorque() const { return (yawMotor.isMotorOnline()) ? yawMotor.getTorque() : 0; }

private:
   src::Drivers* drivers;

//     DJIMotor yawMotor;
//     DJIMotor pitchMotor;
//     DJIMotor rollMotor;

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