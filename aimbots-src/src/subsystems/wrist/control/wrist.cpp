#include "subsystems/wrist/control/wrist.hpp"

#include <utils/tools/common_types.hpp>

#ifdef WRIST_COMPATIBLE

namespace src::Feeder {

FeederSubsystem::FeederSubsystem(src::Drivers* drivers)
    : tap::control::Subsystem(drivers),
      drivers(drivers) {
    BuildWristMotors();
    BuildPIDControllers();
}

void WristSubsystem::initialize() {
    ForAllWristMotors(&DJIMotor::initialize);

    setAllDesiredWristMotorOutputs(0);
    ForAllWRistMotors(&WristSubsystem::setDesiredOutputToWristMotor);

    for (auto i = 0; i < WRIST_MOTOR_COUNT; i++) {
        wristVelocityPIDs[i]->pid.reset();
    }
}

float wristTargetRPMDisplay = 0;

// refreshes the velocity PID given the target RPM and the current RPM
void WristSubsystem::refresh() {
    double desiredYaw = 0;
    double desiredPitch = 0;
    double motor1CurrAngle = 0;
    double motor2CurrAngle = 0;
    
    vector<double> difference = diffE(desiredYaw, desiredPitch, motor1CurrAngle, motor2CurrAngle);

    for (auto i = 0; i < WRIST_MOTOR_COUNT - 1; i++) {
        if (!wristMotors[i]->isMotorOnline()) {
            continue;
        }

        wristTargetRPMDisplay = wristTargetRPMs[i];

        updateMotorVelocityPID(i, difference[i]);
        setDesiredOutputToWristMotor(i);
    }
}

void WristSubsystem::updateMotorVelocityPID(uint8_t WristIdx, double error) {
    float desiredOutput = wristVelocityPIDs[dx]->runController(
        error,
        getWristMotorTorque(WristIdx));
    setDesiredWristMotorOutput(WristIdx, desiredOutput);
}

void WristSubsystem::setTargetRPM(uint8_t WristIdx, float rpm) { wristTargetRPMs[WristIdx] = rpm; }

void WristSubsystem::setDesiredOutputToWristMotor(uint8_t WristIdx) {
    // takes the input from the velocity PID and sets the motor to that RPM
    wristMotors[WristIdx]->setDesiredOutput(desiredWristMotorOutputs[WristIdx]);
}

//bool FeederSubsystem::getPressed() { return limitSwitch.readSwitch(); }

}  // namespace src::Feeder

#endif  // #ifdef FEEDER_COMPATIBLE