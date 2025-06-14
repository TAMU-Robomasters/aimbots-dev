#include "subsystems/feeder/control/feeder.hpp"

#include <utils/tools/common_types.hpp>

#ifdef FEEDER_COMPATIBLE

namespace src::Feeder {

FeederSubsystem::FeederSubsystem(src::Drivers* drivers)
    : tap::control::Subsystem(drivers),
      drivers(drivers),
      limitSwitch(static_cast<std::string>("C6"), src::Informants::EdgeType::RISING) {
    BuildFeederMotors();
    BuildPIDControllers();
}

// Watch Variables
int16_t heatCurrentDisplay = 0;
int16_t barrelDDisplay = 0;
bool MotorOnline = false;

int16_t heatMaxDisplay = 0;
// int watchPids = 0;
float pidTargetDisplay = 0.0f;
bool watchonline = false;

void FeederSubsystem::initialize() {
    //   feederMotor.initialize();
    ForAllFeederMotors(&DJIMotor::initialize);
    limitSwitch.initialize();

    setAllDesiredFeederMotorOutputs(0);
    ForAllFeederMotors(&FeederSubsystem::setDesiredOutputToFeederMotor);

    for (auto i = 0; i < FEEDER_MOTOR_COUNT; i++) {
        feederVelocityPIDs[i]->pid.reset();
    }
}

bool rightMouseDisplay = false;
bool eKeyDisplay = false;
bool wKeyDisplay = false;

float feederTargetRPMDisplay = 0;
int feederWatchIdx = 0;  // don't change, only in debug
bool isFeederOnlineDisplay = false;

bool limitSwitchDisplay = false;

// refreshes the velocity PID given the target RPM and the current RPM
void FeederSubsystem::refresh() {
    watchonline = isOnline();
    int feederOnlineCount = 0;

    limitSwitch.refresh();

    limitSwitchDisplay = limitSwitch.readSwitch();
    MotorOnline = feederMotors[0]->isMotorOnline();

    for (auto i = 0; i < FEEDER_MOTOR_COUNT; i++) {
        if (!feederMotors[i]->isMotorOnline()) {
            continue;
        }

        if (i == feederWatchIdx) {
            feederTargetRPMDisplay = feederTargetRPMs[i];
        }

        feederOnlineCount++;
        updateMotorVelocityPID(i);
        setDesiredOutputToFeederMotor(i);
    }

    wKeyDisplay = drivers->remote.keyPressed(Remote::Key::W);
    eKeyDisplay = drivers->remote.keyPressed(Remote::Key::E);
    rightMouseDisplay = drivers->remote.getMouseR();
}

void FeederSubsystem::updateMotorVelocityPID(uint8_t FeederIdx) {
    float desiredOutput = feederVelocityPIDs[FeederIdx]->runController(
        feederTargetRPMs[FeederIdx] - feederMotors[FeederIdx]->getShaftRPM(),
        getFeederMotorTorque(FeederIdx));
    setDesiredFeederMotorOutput(FeederIdx, desiredOutput);
}

void FeederSubsystem::setTargetRPM(uint8_t FeederIdx, float rpm) { feederTargetRPMs[FeederIdx] = rpm; }

void FeederSubsystem::setDesiredOutputToFeederMotor(uint8_t FeederIdx) {
    // takes the input from the velocity PID and sets the motor to that RPM
    feederMotors[FeederIdx]->setDesiredOutput(desiredFeederMotorOutputs[FeederIdx]);
}

bool FeederSubsystem::getPressed() { return limitSwitch.readSwitch(); }

}  // namespace src::Feeder

#endif  // #ifdef FEEDER_COMPATIBLE