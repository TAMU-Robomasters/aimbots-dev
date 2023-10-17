#pragma once
#include "tap/control/subsystem.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

#include "src/utils/pid/smooth_pid_wrap.hpp"

namespace src::PosTester {

class PosTester : public tap::control::Subsystem {
public:
    PosTester(src::Drivers*);
    void getMotorPos() override;

    void setMotorPos() override;
    
    void initialize() override;
    
    void refresh() override;

    bool setTargetPos() override;

    bool runController() override;

    const char* getName() const override { return "pos tester"; }

private:
    src::Drivers* drivers;

    DjiMotor motor;
    SmoothPID feederPosPID;

    float targetPos;
    float desiredOutput;
};

};      // src::PosTester