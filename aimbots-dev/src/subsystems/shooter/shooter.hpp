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
    BOT = 1,
    TOP2 = 2,
    BOT2 = 3
};


class ShooterSubsystem : public tap::control::Subsystem {
   public:
    ShooterSubsystem(tap::Drivers* drivers);

    mockable void initialize() override;
    void refresh() override;

    void setDesiredOutputs();

    void calculateShooter(float RPM_Target);

    void setZeroOutput();


    float targetRPMs[2];
   private:
    DJIMotor topWheel, bottomWheel;
    DJIMotor* motors[2];
    //static constexpr CANBus SHOOTER_BUS = CANBus::CAN_BUS1;
    static constexpr CANBus SHOOTER_BUS = CANBus::CAN_BUS1;
    //emergency testing line for if there is a chassis but no gimbal
    //if the code works, the robot will become extremely violent in a short amount of time
    uint32_t lastTime;
    SmoothPID topWheelPID;
    SmoothPID bottomWheelPID;
    #ifdef TARGET_SENTRY
    DJIMotor topWheel2, bottomWheel2;
    #endif //#ifdef TARGET_SENTRY
   public:
    inline int16_t getTopWheelRpmActual() const { return topWheel.getShaftRPM(); }
    inline int16_t getBottomWheelRpmActual() const { return bottomWheel.getShaftRPM(); }
};
};  // namespace src::Shooter

//#endif TARGET_ENGINEER