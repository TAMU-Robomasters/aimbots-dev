#include "subsystems/feeder/feeder.hpp"

#include <utils/common_types.hpp>

#ifdef FEEDER_COMPATIBLE

namespace src::Feeder {

FeederSubsystem::FeederSubsystem(src::Drivers* drivers)
    : tap::control::Subsystem(drivers),
      drivers(drivers),
      //targetRPM(0),
      //desiredOutput(0),
      //feederVelocityPID(FEEDER_VELOCITY_PID_CONFIG),
      //feederMotor(drivers, FEEDER_ID, FEED_BUS, FEEDER_DIRECTION, "Feeder Motor"),
      limitSwitch(static_cast<std::string>("C6"), src::Informants::EdgeType::RISING)
{
    BuildFeederMotors();
    BuildPIDControllers();
}

// Watch Variables
int16_t heatCurrentDisplay = 0;
int16_t barrelDDisplay = 0;
bool MotorOnline = false;

int16_t heatMaxDisplay = 0;
//int watchPids = 0;
float pidTargetDisplay = 0.0f;
bool watchonline = false;

void FeederSubsystem::initialize() {
    
 //   feederMotor.initialize();
    ForAllFeederMotors(&DJIMotor::initialize);
    limitSwitch.initialize();

    
    setAllDesiredFeederMotorOutputs(0);
    ForAllFeederMotors(&FeederSubsystem::setDesiredOutputToFeederMotor);
    
    for(auto i = 0; i < FEEDER_MOTOR_COUNT; i++) {
        feederVelocityPIDs[i]->pid.reset();
  //      watchPids++;
    }
}

float feederShaftRPMDisplay = 0;
bool isFeederOnlineDisplay = false;

// refreshes the velocity PID given the target RPM and the current RPM
void FeederSubsystem::refresh() {
    watchonline = isOnline();
    int feederOnlineCount = 0;
    // feederShaftRPMDisplay = feederMotor.getShaftRPM();

    // updateMotorVelocityPID();
    // setDesiredOutput();
    limitSwitch.refresh();
    MotorOnline = feederMotors[0]->isMotorOnline();

    for (auto i = 0; i < FEEDER_MOTOR_COUNT; i++) {
        //feederVelocityFilters[i]->update(feeder->getRPM(i));
        if (!feederMotors[i]->isMotorOnline()) {
            continue;
        }
        
        feederOnlineCount++;
        updateMotorVelocityPID(i);
        setDesiredOutputToFeederMotor(i);
    }
    //ForAllFeederMotors(&FeederSubsystem::setDesiredFeederMotorOutput);
}

void FeederSubsystem::updateMotorVelocityPID(uint8_t FeederIdx) {
  //  if (feederMotors[FeederIdx]->isMotorOnline()){
  //      float 
     //float err = targetRPM - feederMotor.getShaftRPM();
     //feederVelocityPIDs[FeederIdx].runControllerDerivateError(err);
    float desiredOutput = feederVelocityPIDs[FeederIdx]->runController(
        feederTargetRPMs[FeederIdx] - feederMotors[FeederIdx]->getShaftRPM(),getFeederMotorTorque(FeederIdx));
    setDesiredFeederMotorOutput(FeederIdx, desiredOutput);
}

void FeederSubsystem::setTargetRPM(float rpm, int FeederIdx) {
    feederTargetRPMs[FeederIdx] = rpm;
}

void FeederSubsystem::setDesiredOutputToFeederMotor(uint8_t FeederIdx) {  // takes the input from the velocity PID and sets the motor to that RPM
    feederMotors[FeederIdx]->setDesiredOutput(desiredFeederMotorOutputs[FeederIdx]);
}

bool FeederSubsystem::getPressed() {
    return limitSwitch.readSwitch();


}

}  // namespace src::Feeder

#endif  // #ifdef FEEDER_COMPATIBLE