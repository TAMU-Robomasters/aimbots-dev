#pragma once

#include "tap/control/subsystem.hpp"

#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"
#include "drivers.hpp"

namespace src::Slide {

enum MotorIndex {
    X = 0,
    Y = 1,
    Z = 2
};

class SlideSubsystem : public tap::control::Subsystem {
public:
    SlideSubsystem(Drivers*);
    void setTargetPosition(float x , float y, float z);

    mockable void initialize() override;
    mockable void refresh() override;

    mockable void setDesiredOutputs();
    void updateSlidePositionPID();

    template<class... Args>
    inline void ForAllSlideMotors(void (DJIMotor::*func)(Args...), Args... args)
    {
        for (auto i = 0; i < 3; i++)
            (motors[i].*func)(args...);
    }

    template<class... Args>
    void ForAllSlideMotors(void (SlideSubsystem::*func)(MotorIndex, Args...), Args... args) {
        for (auto i = 0; i < 3; i++) {
            auto mi = static_cast<MotorIndex>(i);
            (this->*func)(mi, args...);
        }
    }

private:
    DJIMotor motors[3];
    SmoothPID motorPIDs[3];
    float targetPoses[3] {};
    int32_t desiredOutputs[3] {};

    void updateMotorPositionPID(MotorIndex);
};

}; // src::Slider