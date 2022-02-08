#pragma once

#include "tap/control/subsystem.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_constants.hpp"

namespace src::Flywheel {

    enum MotorIndex{
        TOP = 0,
        BOT = 1
    };

    class FlywheelSubsystem : public tap::control::Subsystem {
        public:
        FlywheelSubsystem(tap::Drivers* drivers);

        mockable void initialize() override;
        void refresh() override;

        void setDesiredOutputs(float r);

        void calculateFlywheel(float r);

        private:
        DJIMotor topWheel, bottomWheel;
        DJIMotor* motors[2];
        float targetRPMs[2];
        static constexpr CANBus FLY_BUS = CANBus::CAN_BUS1;

        public: 
        inline int16_t getTopWheelRpmActual() const { return topWheel.getShaftRPM(); }
        inline int16_t getBottomWheelRpmActual() const { return bottomWheel.getShaftRPM(); }
    };
}; //namespace src::Flywheel