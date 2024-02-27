

#include "subsystems/dartsystem/dart.hpp"

#include <tap/architecture/clock.hpp>
#include <tap/communication/gpio/leds.hpp>

#include "utils/common_types.hpp"

#ifdef DART_COMPATIBLE

namespace src::Dart {

DartSubsystem::DartSubsystem(tap::Drivers* drivers)
    : Subsystem(drivers),
      launch1(drivers, MotorID::MOTOR1, CANBus::CAN_BUS1, true, "Left One"),
      launch2(drivers, MotorID::MOTOR2, CANBus::CAN_BUS1, true, "Left Two"),
      launch3(drivers, MotorID::MOTOR3, CANBus::CAN_BUS1, true, "Left Three"),
      launch4(drivers, MotorID::MOTOR4, CANBus::CAN_BUS1, false, "Right Four"),
      launch5(drivers, MotorID::MOTOR5, CANBus::CAN_BUS1, false, "Right Five"),
      launch6(drivers, MotorID::MOTOR6, CANBus::CAN_BUS1, false, "Right Six"),
      launch1PID(SHOOTER_VELOCITY_PID_CONFIG),
      launch2PID(SHOOTER_VELOCITY_PID_CONFIG),
      launch3PID(SHOOTER_VELOCITY_PID_CONFIG),
      launch4PID(SHOOTER_VELOCITY_PID_CONFIG),
      launch5PID(SHOOTER_VELOCITY_PID_CONFIG),
      launch6PID(SHOOTER_VELOCITY_PID_CONFIG),
      launchTargetRPMs(Matrix<float, LAUNCH_MOTOR_COUNT, 1>::zeroMatrix()),
      launchDesiredOutputs(Matrix<int32_t, LAUNCH_MOTOR_COUNT, 1>::zeroMatrix()),
      motors(Matrix<DJIMotor*, LAUNCH_MOTOR_COUNT, 1>::zeroMatrix()),
      launchVelocityPIDs(Matrix<SmoothPID*, LAUNCH_MOTOR_COUNT, 1>::zeroMatrix()),
      load1(drivers, MotorID::MOTOR6, CANBus::CAN_BUS1, false, "Load Top"),
      load2(drivers, MotorID::MOTOR6, CANBus::CAN_BUS1, false, "Load Top"),
      load1PID(BARREL_SWAP_POSITION_PID_CONFIG),
      load2PID(BARREL_SWAP_POSITION_PID_CONFIG)
//
{
    motors[L_1][0] = &launch1;
    motors[L_2][0] = &launch2; 
    motors[L_3][0] = &launch3;
    motors[L_4][0] = &launch4; 
    motors[L_5][0] = &launch5;
    motors[L_6][0] = &launch6; 
    launchVelocityPIDs[L_1][0] = &launch1PID;
    launchVelocityPIDs[L_2][0] = &launch2PID;
    launchVelocityPIDs[L_3][0] = &launch3PID;
    launchVelocityPIDs[L_4][0] = &launch4PID;
    launchVelocityPIDs[L_5][0] = &launch5PID;
    launchVelocityPIDs[L_6][0] = &launch6PID;
}

void DartSubsystem::initialize() {
    ForAllDartMotors(&DJIMotor::initialize);

    ForAllDartMotors(&DJIMotor::setDesiredOutput, static_cast<int32_t>(0.0f));
}

bool isDartOnlineDisplay = false;
int firstBadMotorDisplay = 8;
// Update the actual RPMs of the motors; the calculation is called from ShooterCommand
void DartSubsystem::refresh() {
    isDartOnlineDisplay = isOnline();
    firstBadMotorDisplay = isOnlineDisplay();
    ForAllDartMotors(&DartSubsystem::setDesiredOutputToMotor);

}

void DartSubsystem::setLoadTargetSpeed(float rpm) {
    load1DOut = rpm;
    load2DOut = rpm;
}

void DartSubsystem::runLoadPIDs() {
    float load1Error = load1DOut - load1.getShaftRPM(); 
    float load2Error = load2DOut - load2.getShaftRPM();

    load1PID.runController(load1Error, load1.getTorque());
    load2PID.runController(load2Error, load2.getTorque());

    float reelError = load1DOut - motors[L_6][0]->getShaftRPM();

    launchVelocityPIDs[L_6][0]->runController(reelError, motors[L_6][0]->getTorque());

    load1.setDesiredOutput(load1PID.getOutput());
    load2.setDesiredOutput(load2PID.getOutput());
    motors[L_6][0]->setDesiredOutput(launchVelocityPIDs[L_6][0]->getOutput());
}

float DartSubsystem::getMotorSpeed(L_MotorIndex motorIdx) const {
    if (motors[motorIdx][0]->isMotorOnline()) {
        return motors[motorIdx][0]->getShaftRPM();
    }
    return 0.0f;
}

void DartSubsystem::updateMotorVelocityPID(L_MotorIndex motorIdx) {
    if (motors[motorIdx][0]->isMotorOnline()) {  // Check if motor is online when getting info from it
        float err = launchTargetRPMs[motorIdx][0] - motors[motorIdx][0]->getShaftRPM();
        float PIDOut = launchVelocityPIDs[motorIdx][0]->runController(err, motors[motorIdx][0]->getTorque());
        setDesiredOutput(motorIdx, PIDOut);
    }
}

void DartSubsystem::setTargetRPM(L_MotorIndex motorIdx, float targetRPM) { launchTargetRPMs[motorIdx][0] = targetRPM; }

float powerDisplay = 0.0f;

void DartSubsystem::setDesiredOutput(L_MotorIndex motorIdx, float desiredOutput) {
    launchDesiredOutputs[motorIdx][0] = static_cast<int32_t>(desiredOutput);
}

void DartSubsystem::setDesiredOutputToMotor(L_MotorIndex motorIdx) {
    if (motors[motorIdx][0]->isMotorOnline()) {  // Check if motor is online when setting to it
        motors[motorIdx][0]->setDesiredOutput(launchDesiredOutputs[motorIdx][0]);
    }
}


int DartSubsystem::isOnlineDisplay() {
        for (auto i = 0; i < LAUNCH_MOTOR_COUNT; i++) {
            if (!motors[i][0]->isMotorOnline()) {
                return i;
            }
        }
        return -1;
    }

};  // namespace src::Shooter

#endif