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

void WristSubsystem::refresh() {
    double desiredYaw = 0;
    double desiredPitch = 0;
    double motor1CurrAngle = 0;
    double motor2CurrAngle = 0;
    
    vector<double> difference = diffE(desiredYaw, desiredPitch, motor1CurrAngle, motor2CurrAngle);

    double bottomPitchDesiredPos = 0;

    double armYawMotorDesired = 0;
    double armYawMotorCurrent = 0;

    double differenceFullPitch = calcPitchTargets(armYawMotorDesired, armYawMotorCurrent);

    double armPitchIntermediateDesired = 0;

    for (auto i = 0; i < WRIST_MOTOR_COUNT; i++) {
        if (!wristMotors[i]->isMotorOnline()) {
            continue;
        }

        switch (i) {
        case (Yaw_Pitch_Motor_1):
            updateMotorVelocityPID(i, difference[i]);
            setDesiredOutputToWristMotor(i);

            break;
        case (Yaw_Pitch_Motor_2):
            updateMotorVelocityPID(i, difference[i]);
            setDesiredOutputToWristMotor(i);

            break;
        case (Yaw_Wrist_Motor):
            updateMotorVelocityPID(i, bottomPitchDesiredPos - getEncoderUnwrapped(i));
            setDesiredOutputToWristMotor(i);

            break;
        case (Arm_Yaw_Motor):
            updateMotorVelocityPID(i, armYawMotorDesired - getEncoderUnwrapped(i));
            setDesiredOutputToWristMotor(i);

            break;
        case (Master_Full_Pitch):
            updateMotorVelocityPID(i, differenceFullPitch);
            setDesiredOutputToWristMotor(i);

            break;
        case (Slave_Full_Pitch):
            updateMotorVelocityPID(i, differenceFullPitch);
            setDesiredOutputToWristMotor(i);

            break;
        case (Intermediate_Pitch):
            updateMotorVelocityPID(i, armPitchIntermediateDesired - getEncoderUnwrapped(i));
            setDesiredOutputToWristMotor(i);

            break;
        }
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