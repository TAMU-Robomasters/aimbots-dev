#include "pos_tester.hpp"


namespace src::PosTester {

PosTester::PosTester(src::Drivers* drivers) : 
    drivers(drivers), 
    motor(encoderWrapped),
    feederPosPID(error), 
    {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(postester));
}

void PosTester::getMotorPos() { M_TWOPI * getEncoderUnwrapped() / DJIMotor::ENC_RESOLUTION }

void PosTester::setMotorPos() { postester->setValue(targetPos); }

void PosTester::initialize() { feedermotor->initialize(); }

void PosTester::Refresh() { UNUSED(interrupted); }

bool PosTester::SetTarget() { return true; }

bool PosTester::RunController() { return false; }

}  // namespace src::Feeder