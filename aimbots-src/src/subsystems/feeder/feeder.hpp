#pragma once
#include "tap/control/subsystem.hpp"
#include "tap/motor/m3508_constants.hpp"

#include "informants/limit_switch.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

#include "drivers.hpp"

#ifdef FEEDER_COMPATIBLE

namespace src::Feeder {
class FeederSubsystem : public tap::control::Subsystem {
public:
    FeederSubsystem(src::Drivers* drivers);

    void BuildFeederMotors(){
        for (auto i = 0; i < FEEDER_MOTOR_COUNT; i++) {
            feederMotors[i] =
                new DJIMotor(drivers, FEEDER_MOTOR_IDS[i], FEEDER_BUS,FEEDER_DIRECTION[i], FEEDER_MOTOR_NAMES[i]);
        }
    }

    template <class... Args>
    void ForAllFeederMotors(void (FeederSubsystem::*func)(uint8_t FeederIdx, Args...), Args... args) {
        for (uint8_t i = 0; i < FEEDER_MOTOR_COUNT; i++) {
            (this->*func)(i, args...);
        }
    }

    

    mockable void initialize() override;
    mockable void refresh() override;

    inline bool isOnline() const {
        bool feederOnline = false;
   
        for (auto& feederMotor : feederMotors) {
            if (yfeederMotor->isMotorOnline()) {
                feederOnline = true;
            }
        }
        
        return feederOnline;
    }

    inline bool isFeederMotorOnline(uint8_t FeederIdx) const {
        if (FeederIdx >= Feeder_MOTOR_COUNT) {
            return false;
        }
        return feederMotors[feederIdx]->isMotorOnline();
    }


    void updateMotorVelocityPID();

    mockable void setAllDesiredOutput(unit16_t output) { desiredFeederMotorOutputs.fill(output);}

    mockable float setTargetRPM(float rpm);

    float getTargetRPM(uint8_t FeederIdx) const {
        return (feederMotors[FeederIdx]->isMotorOnline()) ? feederMotors[FeederIdx]->getShaftRPM() : 0;
    }

    bool getPressed();

    float getCurrentRPM() const { return feederMotor.getShaftRPM(); }

    


private:

    float targetRPM;
    float desiredOutput;

    SmoothPID feederVelPID;
    DJIMotor feederMotor;
    src::Drivers* drivers;

    src::Informants::LimitSwitch limitSwitch;  // for single-barreled robots
//#endif

    // commands
};

}  // namespace src::Feeder

#endif  // #ifdef FEEDER_COMPATIBLE