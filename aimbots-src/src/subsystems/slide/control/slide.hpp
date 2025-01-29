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

    // initializes all motors on startup
    // initializes motor encoder display
    //   (no idea what that data is used more. 
    //    it's in the cpp but literally isnt used anywhere(???????????)
    //    (16/jan/2025))
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

    // TODO: compile & debug setAllTargetPositionsMeters ++ setTargetPositionMeters (i didnt have time)
    void setAllTargetPositionsMeters(double[] targetVals);
    void setTargetPositionMeters(MotorIndex, double targetVal);
    
    // TODO: fix these functions. general idea below

    /*
        getTargetPositionMeters( int motorIdx ) {
            return targetPosesMeters[motorIdx]
        }
    */
   
    float getTargetXMeters() const;
    float getTargetZMeters() const;

    void updateAllPIDs();

    template <typename... Args>
    using SlideSubsystemFunc = void (SlideSubsystem::*)(Args...);

    template <class... Args>
    void ForAllSlideMotors(void (DJIMotor::*func)(Args...), Args... args) {
        for (auto& currSlideMotor : slideMotors) {
            (currSlideMotor->*func)(args...);
        }
    }

    template <class... Args>
    void ForAllSlideMotors(void (DJIMotor::*func)(Args...), Args... args) { // is DJIMotor arg correct??
        for (uint8_t i = 0; i < SLIDE_MOTOR_COUNT; i++) {
            auto currMotorIndex = static_cast<MotorIndex>(i);
            (this->*func)(currMotorIndex, args...);
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
    void refreshEncoderDisplay(MotorIndex);
};

};  // namespace src::Slide

#endif