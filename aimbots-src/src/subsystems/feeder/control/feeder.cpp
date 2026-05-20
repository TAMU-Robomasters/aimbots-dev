#include "subsystems/feeder/control/feeder.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdlib>

#include <utils/tools/common_types.hpp>

#include "tap/motor/dji_motor.hpp"
#include "robots/sentry/constants/sentry_feeder_constants.hpp"

#ifdef FEEDER_COMPATIBLE

namespace src::Feeder {

FeederSubsystem::FeederSubsystem(src::Drivers* drivers)
    : tap::control::Subsystem(drivers),
      drivers(drivers),
      limitSwitch(static_cast<std::string>("C6"), src::Informants::EdgeType::RISING),
      feederRPMFilter(tap::algorithms::filter::butterworth<ORDER, TYPE, float>(130.0, 0.002)) {
    BuildFeederMotors();
    BuildPIDControllers();
}

// Watch Variables
int16_t heatCurrentDisplay = 0;
int16_t barrelDDisplay = 0;
bool MotorOnline = false;
float desiredOutputDisplay = 0.0;

int16_t heatMaxDisplay = 0;
// int watchPids = 0;
float pidTargetDisplay = 0.0f;
bool watchonline = false;

float feederVelocityP = 0.0f;
float feederVelocityI = 0.0f;
float feederVelocityD = 0.0f;
bool updateFeederVelocityPIDsDebug = false;

uint16_t feederDesiredOutputDebug = 0;
uint16_t feederDesiredOutput = 0;
bool updateFeederDesiredOutputDebug = false;

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
float feederRPMDisplay = 0.0f;
float feederRawRPMDisplay = 0.0f;
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
    feederRawRPMDisplay = feederRPMFilter.getLastFiltered();
    feederRPMDisplayFilter->update(getCurrentRPM(feederWatchIdx));
    feederRPMDisplay = feederRPMDisplayFilter->getValue();
    wKeyDisplay = drivers->remote.keyPressed(Remote::Key::W);
    eKeyDisplay = drivers->remote.keyPressed(Remote::Key::E);
    rightMouseDisplay = drivers->remote.getMouseR();
}

void FeederSubsystem::updateMotorVelocityPID(uint8_t FeederIdx) {
    if (updateFeederVelocityPIDsDebug) {  // for PID tunning through Ozone
        feederVelocityPIDs[FeederIdx]->pid.setP(feederVelocityP);
        feederVelocityPIDs[FeederIdx]->pid.setI(feederVelocityI);
        feederVelocityPIDs[FeederIdx]->pid.setD(feederVelocityD);
        feederVelocityPIDs[FeederIdx]->pid.reset();
        updateFeederVelocityPIDsDebug = false;
    }
    float feedForwardOuptut = 0.0f;

    if (abs(feederTargetRPMs[FeederIdx]) >= 10) {
        feedForwardOuptut = sgn(feederTargetRPMs[FeederIdx]) * feederFeedforward(abs(feederTargetRPMs[FeederIdx]));
    }

    float pidOutput = feederVelocityPIDs[FeederIdx]->runController(
        feederTargetRPMs[FeederIdx] - feederRPMFilter.filterData(feederMotors[FeederIdx]->getShaftRPM()),
        getFeederMotorTorque(FeederIdx));

    float desiredOutput = pidOutput + feedForwardOuptut;

    // if (updateFeederDesiredOutputDebug) {
    //     feederDesiredOutput = feederDesiredOutputDebug;
    //     updateFeederDesiredOutputDebug = false;
    // }
    // if (feederTargetRPMs[FeederIdx] >= 10)  {
    //     desiredOutput = std::min(tap::motor::DjiMotor::MAX_OUTPUT_C610, feederDesiredOutput);
    // }
    if (abs(feederTargetRPMs[FeederIdx]) < 10) {
        desiredOutput = 0;
    }
    desiredOutputDisplay = desiredOutput;
    setDesiredFeederMotorOutput(FeederIdx, desiredOutput);
}

void FeederSubsystem::setTargetRPM(uint8_t FeederIdx, float rpm) { feederTargetRPMs[FeederIdx] = rpm; }
void FeederSubsystem::setAllTargetRPMs(float rpm) {
    for (auto i = 0; i < FEEDER_MOTOR_COUNT; i++) {
        setTargetRPM(i, rpm);
    }
}

void FeederSubsystem::setDesiredOutputToFeederMotor(uint8_t FeederIdx) {
    // takes the input from the velocity PID and sets the motor to that RPM
    feederMotors[FeederIdx]->setDesiredOutput(desiredFeederMotorOutputs[FeederIdx]);
}

bool FeederSubsystem::getPressed() { return limitSwitch.readSwitch(); }

}  // namespace src::Feeder

#endif  // #ifdef FEEDER_COMPATIBLE
