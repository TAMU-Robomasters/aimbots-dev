#pragma once

#include "tap/control/subsystem.hpp"
#include "tap/motor/m3508_constants.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_constants.hpp"

namespace src::Feeder {

    class FeederSubsystem : public tap::control::Subsystem {
        public:
         FeederSubsystem(
             tap::Drivers* drivers);
        
        template <class... Args>

        mockable void initialize() override;
        void refresh() override;

        void setDesiredOutput(float speed);

    #ifndef ENV_UNIT_TESTS
            private:
    #else
            public:
    #endif     
            static constexpr CANBus FEED_BUS = CANBus::CAN_BUS2;
        
        float targetRPM;
        DJIMotor* feederMotor; 

        //commands
    };

}