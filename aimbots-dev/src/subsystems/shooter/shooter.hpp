#pragma once
#include <vector>
//
#include "utils/common_types.hpp"
#include "utils/robot_constants.hpp"
//
#include "tap/architecture/clock.hpp"
#include "tap/control/subsystem.hpp"

//#ifndef TARGET_ENGINEER
namespace src::Shooter {

enum MotorIndex {
    TOP = 0,
    BOT = 1
};

class ShooterSubsystem : public tap::control::Subsystem {
   public:
    ShooterSubsystem(tap::Drivers* drivers);

    mockable void initialize() override;
    void refresh() override;

    void setDesiredOutputs();

    void calculateShooter(float RPM_Target);


    float targetRPMs[2];
   private:
    DJIMotor topWheel, bottomWheel;
    DJIMotor* motors[2];
    static constexpr CANBus FLY_BUS = CANBus::CAN_BUS1;
    uint32_t lastTime;
    SmoothPID topWheelPID;
    SmoothPID bottomWheelPID;
    
   public:
    inline int16_t getTopWheelRpmActual() const { return topWheel.getShaftRPM(); }
    inline int16_t getBottomWheelRpmActual() const { return bottomWheel.getShaftRPM(); }
};
};  // namespace src::Shooter

//#endif TARGET_ENGINEER