#include "pos_tester.hpp"


namespace src::PosTester {

PosTester::PosTester(src::Drivers* drivers) 
    : Subsystem(drivers),
    targetPos(0),
    desiredOutput(0),
    motorPosPID(MOTOR_VELOCITY_PID_CONFIG),
    motor(drivers, MOTOR_ID, CAN_BUS, MOTOR_DIRECTION, "pos tester") {
    //addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(postester));
}

void PosTester::initialize() { motor.initialize(); }

void PosTester::refresh() {
    runController();
    setTargetPos(targetPos);
}

float PosTester::getMotorPos() { return M_TWOPI * motor.getEncoderUnwrapped() / DJIMotor::ENC_RESOLUTION; }

void PosTester::setMotorRPM() { motor.setDesiredOutput(static_cast<int32_t>(desiredOutput)); }

float PosTester::setTargetPos(float pos) {
    this->targetPos = pos;
    return targetPos;
}

void PosTester::runController() {
    float err = targetPos - getMotorPos();
    motorPosPID.runControllerDerivateError(err);
    desiredOutput = motorPosPID.getOutput();
}

}  // namespace src::Feeder
