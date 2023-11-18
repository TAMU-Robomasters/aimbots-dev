#pragma once

#include "tap/control/subsystem.hpp"

#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"
#include "drivers.hpp"

namespace src::Slide {

enum MotorIndex {
    X = 0,
    Z = 1
};

class SlideSubsystem : public tap::control::Subsystem {
public:
    SlideSubsystem(Drivers*);
    void setTargetPosition(float x, float z);

    mockable void initialize() override;
    mockable void refresh() override;

    mockable void setDesiredOutputs();
    void updateSlidePositionPID();

    template<class... Args>
    void ForAllSlideMotors(void (DJIMotor::*func)(Args...), Args... args)
    {
        for (auto i = 0; i < SLIDE_MOTOR_COUNT; i++)
            (motors[i].*func)(args...);
    }

    template<class... Args>
    void ForAllSlideMotors(void (SlideSubsystem::*func)(MotorIndex, Args...), Args... args) {
        for (auto i = 0; i < SLIDE_MOTOR_COUNT; i++) {
            auto mi = static_cast<MotorIndex>(i);
            (this->*func)(mi, args...);
        }
    }

private:
    DJIMotor motors[SLIDE_MOTOR_COUNT];
    SmoothPID motorPIDs[SLIDE_MOTOR_COUNT];
    float targetPoses[SLIDE_MOTOR_COUNT] {};
    int32_t desiredOutputs[SLIDE_MOTOR_COUNT] {};

    void updateMotorPositionPID(MotorIndex);
};

}; // src::Slider