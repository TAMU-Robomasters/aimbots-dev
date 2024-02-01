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

    void BuildPIDControllers(){
        for (auto i = 0; i < FEEDER_MOTOR_COUNT; i++) {
            feederVelocityPIDs[i] = new SmoothPID(FEEDER_VELOCITY_PID_CONFIG);
            feederVelocityFilters[i] = new src::Utils::Filters::EMAFilter(0.02);
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


    void updateMotorVelocityPID(uint8_t FeederIdxIdx);

    void setDesiredFeederMotorOutput(uint8_t FeederIdx, float output) {desiredFeederMotorOutputs[FeederIdx] = output;}


    mockable void setAllDesiredFeederOutput(uint16_t output) { desiredFeederMotorOutputs.fill(output);}

    mockable float setTargetRPM(float rpm);

    float getRPM(uint8_t FeederIdx) const {
        return (feederMotors[FeederIdx]->isMotorOnline()) ? feederMotors[FeederIdx]->getShaftRPM() : 0;
    }

    bool getPressed();

   // float getCurrentRPM() const { return feederMotor.getShaftRPM(); }

    int64_t getEncoderUnwrapped() const { return feederMotor.getEncoderUnwrapped() / FEEDER_GEAR_RATIO; }

    int getTotalLimitCount() const;

private:
    src::Drivers* drivers;

    std::array<DJIMotor*, FEEDER_MOTOR_COUNT> feederMotors;

    std::array<float, FEEDER_MOTOR_COUNT> desiredFeederMotorOutputs;

    void setDesiredOutputToFeederMotor(uint8_t FeederIdx);

    float targetRPM;
    float desiredOutput;

    std::array<SmoothPID*, FEEDER_MOTOR_COUNT> feederPIDs;
    DJIMotor feederMotor;

    src::Informants::LimitSwitch limitSwitch;  
//#endif

    // commands
};

}  // namespace src::Feeder

#endif  // #ifdef FEEDER_COMPATIBLE