#pragma once

#include "tap/control/subsystem.hpp"

#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"
#include "drivers.hpp"

namespace src::Slide {

class SlideSubsystem : public tap::control::Subsystem {
public:
    SlideSubsystem(Drivers*);
    void setTargetPosition(float x , float y, float z);

    mockable void initialize() override;
    mockable void refresh() override;

    mockable void setDesiredOutput();
    void updateSlidePositionPID();
private:
    DJIMotor xMotor, yMotor, zMotor;
    SmoothPID xMotorPID, yMotorPID, zMotorPID;
    float targetX, targetY, targetZ;
    float desiredXRPM, desiredYRPM, desiredZRPM;
};

}; // src::Slider