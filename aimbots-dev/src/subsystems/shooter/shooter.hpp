#pragma once
#include <vector>
//
#include "utils/common_types.hpp"
#include "utils/robot_constants.hpp"
//
#include "tap/architecture/clock.hpp"
#include "tap/control/subsystem.hpp"
namespace src::Shooter {

    enum MotorIndex{
        TOP = 0,
        BOT = 1
    };

    class ShooterSubsystem : public tap::control::Subsystem {
        public:
        ShooterSubsystem(tap::Drivers* drivers);

        mockable void initialize() override;
        void refresh() override;

        void setDesiredOutputs(float RPM);

        std::vector<float> calculateShooter(float RPM);

        private:
        DJIMotor topWheel, bottomWheel;
        DJIMotor* motors[2];
        float targetRPMs[2];
        static constexpr CANBus FLY_BUS = CANBus::CAN_BUS1;
        float lastTime;
        SmoothPID PID;

        public: 
        inline int16_t getTopWheelRpmActual() const { return topWheel.getShaftRPM(); }
        inline int16_t getBottomWheelRpmActual() const { return bottomWheel.getShaftRPM(); }
    };
}; //namespace src::Shooter