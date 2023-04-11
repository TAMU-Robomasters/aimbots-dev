#include "subsystems/shooter/barrel_swap/swap_mechanism.hpp"
#ifndef ENGINEER
namespace src::Shooter{

    SwapMechanismSubsystem::SwapMechanismSubsystem(src::Drivers* drivers)
    : Subsystem(drivers),
    swapMotor(drivers, SWAP_MOTOR_ID, GIMBAL_BUS, SWAP_DIRECTION),
    currentSwapMotorRelativeAngle(0.0f, 0.0f, M_TWOPI),
    targetSwapMotorRelativeAngle(modm::toRadian(SWAP_START_ANGLE)) {}

    void SwapMechanismSubsystem::initialize() {
        swapMotor.initialize();
    }

    float swapRelativeDisplay = 0.0f;
    float swapOutputDisplay = 0.0f;

    void SwapMechanismSubsystem::refresh() {
        setDesiredOutput();
        // below modified from gimbal.cpp:109
        uint16_t currentSwapEncoderPosition = swapMotor.getEncoderWrapped();
        currentSwapMotorRelativeAngle.setValue(wrappedEncoderValueToRadians(currentSwapEncoderPosition));

        swapRelativeDisplay = modm::toDegree(currentSwapMotorRelativeAngle.getValue());
        swapOutputDisplay = desiredSwapMotorOutput;

        swapMotor.setDesiredOutput(desiredSwapMotorOutput);
    }

    void SwapMechanismSubsystem::setSwapMotorOutput(float output) {
        // m2006 motor
        // this function modified from gimbal.cpp:129
        desiredSwapMotorOutput = tap::algorithms::limitVal(output, -M2006_MAX_OUTPUT, M2006_MAX_OUTPUT);
    }

    float SwapMechanismSubsystem::getCurrentAngleFromChassisCenter(AngleUnit unit) const {
        // this function modified from gimbal.cpp:145
        return tap::algorithms::ContiguousFloat(
                (unit == AngleUnit::Degrees) ? modm::toDegree(currentSwapMotorRelativeAngle.getValue() - modm::toRadian(SWAP_START_ANGLE)) : (currentSwapMotorRelativeAngle.getValue() - modm::toRadian(SWAP_START_ANGLE)),
                (unit == AngleUnit::Degrees) ? -180.0f : -M_PI,
                (unit == AngleUnit::Degrees) ? 180.0f : M_PI)
            .getValue();
        )
    }

}

#endif