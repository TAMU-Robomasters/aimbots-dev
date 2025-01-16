#pragma once

#include "tap/control/subsystem.hpp"

#include "subsystems/slide/slide_constants.hpp"
#include "utils/tools/common_types.hpp"

#include "drivers.hpp"

#ifdef SLIDE_COMPATIBLE

namespace src::Slide {

enum MotorIndex { X = 0, Z = 1 };

class SlideSubsystem : public tap::control::Subsystem {
public:
    SlideSubsystem(Drivers*);
    ~SlideSubsystem() = default;

    void BuildSlideMotors() {
        for (auto i = 0; i < SLIDE_MOTOR_COUNT; i++) {
            slideMotors[i] = new DJIMotor(drivers, 
                                          SLIDE_MOTOR_IDS[i], 
                                          SLIDE_GIMBAL_BUS, 
                                          SLIDE_MOTOR_DIRECTIONS[i], 
                                          SLIDE_MOTOR_NAMES[i]);
        }
        slideMotorEncoderDisplay[i] = 0.0f;
    }

    mockable void initialize() override;
    mockable void refresh() override;

    void setTargetPositionMeters(float x, float z);
    float getTargetXMeters() const;
    float getTargetZMeters() const;

    void updateAllPIDs();

    template <typename... Args>
    using SlideSubsystemFunc = void (SlideSubsystem::*)(Args...);

    template <class... Args>
    void ForAllSlideMotors(void (DJIMotor::*func)(Args...), Args... args) {
        for (auto& slideMotor : slideMotors) {
            (slideMotor->*func)(args...);
        }
    }

    template <class... Args>
    void ForAllSlideMotors(void (DJIMotor::*func)(Args...), Args... args) {
        for (uint8_t i = 0; i < SLIDE_MOTOR_COUNT; i++) {
            (this->*func)(i, args...);
        }
    }

    void idle() { desiredOutputs = {}; }

private:
    std::array<DJIMotor, SLIDE_MOTOR_COUNT> slideMotors;
    std::array<float, SLIDE_MOTOR_COUNT> targetPosesMeters{};
    std::array<int32_t, SLIDE_MOTOR_COUNT> desiredOutputs{};
    std::array<float, SLIDE_MOTOR_COUNT> slideMotorEncoderDisplay{};

    void updateMotorPositionPID(MotorIndex);
    void refreshDesiredOutput(MotorIndex);
};

};  // namespace src::Slide

#endif