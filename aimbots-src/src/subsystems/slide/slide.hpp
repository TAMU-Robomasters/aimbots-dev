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

    mockable void initialize() override;
    mockable void refresh() override;

    void setTargetPositionMeters(float x, float z);
    void updateAllPIDs();

    template<typename... Args>
    using SlideSubsystemFunc = void (SlideSubsystem::*)(Args...);

    template<class... Args>
    void ForAllSlideMotors(DJIMotorFunc<Args...> func, Args... args)
    {
        for (auto i = 0; i < SLIDE_MOTOR_COUNT; i++)
            (motors[i].*func)(args...);
    }

    template<class... Args>
    void ForAllSlideMotors(SlideSubsystemFunc<MotorIndex, Args...> func, Args... args) {
        for (auto i = 0; i < SLIDE_MOTOR_COUNT; i++) {
            auto mi = static_cast<MotorIndex>(i);
            (this->*func)(mi, args...);
        }
    }

private:
    std::array<DJIMotor, SLIDE_MOTOR_COUNT> motors;
    std::array<SmoothPID, SLIDE_MOTOR_COUNT> motorPIDs;
    std::array<float, SLIDE_MOTOR_COUNT> targetPosesMeters {};
    std::array<int32_t, SLIDE_MOTOR_COUNT> desiredOutputs {};

    void updateMotorPositionPID(MotorIndex);
    void refreshDesiredOutput(MotorIndex);
};

}; // src::Slider