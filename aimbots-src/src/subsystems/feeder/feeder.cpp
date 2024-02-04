#include "subsystems/feeder/feeder.hpp"

#ifdef FEEDER_COMPATIBLE

namespace src::Feeder {

FeederSubsystem::FeederSubsystem(src::Drivers* drivers)
    : tap::control::Subsystem(drivers),
      drivers(drivers),
      targetRPM(0),
      //desiredOutput(0),
      //feederVelocityPID(FEEDER_VELOCITY_PID_CONFIG),
      //feederMotor(drivers, FEEDER_ID, FEED_BUS, FEEDER_DIRECTION, "Feeder Motor"),
      limitSwitch(static_cast<std::string>("C6"), src::Informants::EdgeType::RISING)
{
    BuildFeederMotors();
}

// Watch Variables
int16_t heatCurrentDisplay = 0;
int16_t barrelDDisplay = 0;

int16_t heatMaxDisplay = 0;

void FeederSubsystem::initialize() {
    

 //   feederMotor.initialize();
    ForAllFeederMotors(&DJIMotor::initialize);
    limitSwitch.initialize();

    
    setAllDesiredFeederMotorOutputs(0);
    ForAllFeederMotors(&FeederSubsystem::setDesiredOutputToFeederMotor);
    
    for(auto i = 0; i < FEEDER_MOTOR_COUNT; i++) {
        feederVelocityPIDs[i]->pid.reset();
    }
}

float feederDesiredOutputDisplay = 0;
float feederShaftRPMDisplay = 0;
bool isFeederOnlineDisplay = false;

// refreshes the velocity PID given the target RPM and the current RPM
void FeederSubsystem::refresh() {
    int feederOnlineCount = 0;
    // feederDesiredOutputDisplay = targetRPM;
    // feederShaftRPMDisplay = feederMotor.getShaftRPM();

    // updateMotorVelocityPID();
    // setDesiredOutput();
    limitSwitch.refresh();

    for (auto i = 0; i < FEEDER_MOTOR_COUNT; i++) {
        //feederVelocityFilters[i]->update(feeder->getRPM(i));
        if (!feederMotors[i]->isMotorOnline()) {
            // tap::buzzer::playNote(&drivers->pwm, 932);
            continue;
        }
        
        feederOnlineCount++;

        setDesiredOutputToFeederMotor(i);
    }



    ForAllFeederMotors(&FeederSubsystem::setDesiredFeederMotorOutput);

}

void FeederSubsystem::updateMotorVelocityPID(uint8_t FeederIdx) {
    if (feederMotors[FeederIdx]->isFeederMotorOnline(FeederIdx)){
        float err = targetRPM - feederMotors[FeederIdx]->getRPM(FeederIdx);
        float 
    }
    // float err = targetRPM - feederMotor.getShaftRPM();
    // feederVelPID.runControllerDerivateError(err);
    // desiredOutput = feederVelPID.getOutput();
}

float FeederSubsystem::setTargetRPM(float rpm) {
    this->targetRPM = rpm;
    return targetRPM;
}

void FeederSubsystem::setDesiredOutputToFeederMotor(uint8_t FeederIdx) {  // takes the input from the velocity PID and sets the motor to that RPM
    feederMotors[FeederIdx]->setDesiredOutput(desiredFeederMotorOutputs[FeederIdx]);
}

bool FeederSubsystem::getPressed() {
    return limitSwitch.readSwitch();


}

}  // namespace src::Feeder

#endif  // #ifdef FEEDER_COMPATIBLE