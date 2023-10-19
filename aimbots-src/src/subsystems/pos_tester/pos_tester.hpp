#pragma once

#include "tap/control/subsystem.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"
#include "drivers.hpp"

//#include "src/utils/pid/smooth_pid_wrap.hpp"

namespace src::PosTester {

class PosTester : public tap::control::Subsystem {
public:
    PosTester(src::Drivers*);

    mockable void initialize() override;
    
    mockable void refresh() override;

    float getMotorPos();

    void setMotorRPM();

    float setTargetPos(float pos);

    void runController();

    const char* getName() { return "pos tester"; }

private:
    DJIMotor motor;
    SmoothPID motorPosPID;

    float targetPos;
    float desiredOutput;
};

};      // src::PosTester