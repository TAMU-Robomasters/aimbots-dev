#include "subsystems/wrist/wrist.hpp"

#include "utils/common_types.hpp"

#include <cmath>

#ifdef WRIST_COMPATIBLE



namespace src::Wrist {

WristSubsystem::WristSubsystem(src::Drivers* drivers) : drivers(drivers), Subsystem(drivers) { BuildMotors(); }

void WristSubsystem::initialize() {
    // idk
}

void WristSubsystem::refresh() {
    if (!isOnline()) {
        return;
    }

    /*
    TODO periodically:
    - update current motor angles
    - update target motor angles (the hard math part)
    - run PID controllers
    - update motor outputs
    */
    updateCurrentMotorAngles();

    //get error for each motor

    /*
    for (auto i = 0; i < 3; i++) {
        if (!motors[i]->isMotorOnline()) {
            // tap::buzzer::playNote(&drivers->pwm, 932);
            continue;
        }

        // tap::buzzer::playNote(&drivers->pwm, 0);
        yawOnlineCount++;

        int64_t currentMotorEncoderPosition = motors[i]->getEncoderUnwrapped();

        // https://www.desmos.com/calculator/bducsk7y6v
        float wrappedAxisAngle =
            GIMBAL_YAW_GEAR_RATIO * (DJIEncoderValueToRadians(currentMotorEncoderPosition) - YAW_MOTOR_OFFSET_ANGLES[i]);

        currentAngle[i]->setValue(wrappedAxisAngle);

        axisAngleSum += wrappedAxisAngle;
        ////////////////
        // DEBUG VARS //
        ////////////////
        // currentYawAxisAngleByMotorDisplay = currentYawAxisAnglesByMotor[yawDisplayMotorIdx]->getValue();
        // currentYawMotorAngleDisplay =
        //     modm::toDegree(DJIEncoderValueToRadians(yawMotors[yawDisplayMotorIdx]->getEncoderUnwrapped()));
        // otherYawMotorAngleDisplay =
        //     modm::toDegree(DJIEncoderValueToRadians(yawMotors[abs(yawDisplayMotorIdx - 1)]->getEncoderUnwrapped()));
        // yawOutputDisplay = yawMotors[i]->getOutputDesired();

        // yawAxisMotorSpeedDisplay = yawMotors[yawDisplayMotorIdx]->getShaftRPM();

        // yawDesiredOutputDisplay = desiredYawMotorOutputs[yawDisplayMotorIdx];

        // flush the desired output to the motor
        setDesiredOutputToMotor(i);

        // if (yawOnlineCount > 0) {
        // currentAngle.setValue(yawAxisAngleSum / yawOnlineCount);
    }
    */
}

void WristSubsystem::calculateArmAngles(uint16_t x, uint16_t y, uint16_t z) {
    
    



}

void WristSubsystem::setTargetAngle(int idx, float targetAngle) {
    
    motors[idx].setTargetAngle(targetAngle);
    
    
}



void WristSubsystem::setDesiredOutputToMotor(uint8_t idx) {
    // Clamp output to maximum value that the 6020 can handle
    motors[idx]->setDesiredOutput(
        tap::algorithms::limitVal(desiredMotorOutputs[idx], -GM6020_MAX_OUTPUT, GM6020_MAX_OUTPUT));
}

void WristSubsystem::updateCurrentMotorAngles() {
    for (auto i = 0; i < 3; i++) {
        currentAngle[i]->setValue( (getCurrentAngleWrapped(i) - WRIST_MOTOR_OFFSET_ANGLES[i])*WRIST_GEAR_RATIO);
    }
}

};      // namespace src::Wrist
#endif  // #ifdef WRIST_COMPATIBLE
