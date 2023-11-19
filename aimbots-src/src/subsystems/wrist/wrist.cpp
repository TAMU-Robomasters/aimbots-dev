#include "subsystems/wrist/wrist.hpp"

#include <cmath>

#include "utils/common_types.hpp"

#ifdef WRIST_COMPATIBLE

namespace src::Wrist {

WristSubsystem::WristSubsystem(src::Drivers* drivers)
    : /*drivers(drivers),*/ Subsystem(drivers),
      motors{
          new DJIMotor(drivers, WRIST_YAW_MOTOR_ID, WRIST_BUS, false, "yaw"),      // yaw motor
          new DJIMotor(drivers, WRIST_PITCH_MOTOR_ID, WRIST_BUS, false, "pitch"),  // pitch motor
          new DJIMotor(drivers, WRIST_ROLL_MOTOR_ID, WRIST_BUS, false, "roll")     // roll motor
      },
      positionPID{
          new SmoothPID(WRIST_VELOCITY_PID_CONFIG),
          new SmoothPID(WRIST_VELOCITY_PID_CONFIG),
          new SmoothPID(WRIST_VELOCITY_PID_CONFIG)},
      currentYawAngle(0.0f, -M_PI, M_PI),
      currentPitchAngle(0.0f, -M_PI, M_PI),
      currentRollAngle(0.0f, -M_PI, M_PI),
      targetYawAngle(0.0f, -M_PI, M_PI),
      targetPitchAngle(0.0f, -M_PI, M_PI),
      targetRollAngle(0.0f, -M_PI, M_PI) {
    // BuildMotors();
    //
    currentAngles[YAW] = &currentYawAngle;
    currentAngles[PITCH] = &currentPitchAngle;
    currentAngles[ROLL] = &currentRollAngle;
    //
    targetAngle[YAW] = &targetYawAngle;
    targetAngle[PITCH] = &targetPitchAngle;
    targetAngle[ROLL] = &targetRollAngle;
}

void WristSubsystem::initialize() {
    for (int i = 0; i < 3; i++) {
        motors[i]->initialize();
        motors[i]->setDesiredOutput(0.0f);
    }
}

int yawOnline = 0;
int pitchOnline = 0;
int rollOnline = 0;

int failCounter = 0;

float yawTargetDisplay = 0.0f;

void WristSubsystem::refresh() {
    yawOnline = motors[YAW]->isMotorOnline();
    pitchOnline = motors[PITCH]->isMotorOnline();
    rollOnline = motors[ROLL]->isMotorOnline();

    yawTargetDisplay = targetAngle[YAW]->getValue();

    if (!isOnline()) {
        failCounter++;
        return;
    }

    /*
    TODO periodically:
    - update current motor angles (polling encoders)
    - update target motor angles (the hard math part)
    - run PID controllers
    - update motor outputs
    */

    // poll encoders to get current motor angles
    // this is the angle, in radians, of the wrist head (after the gearbox)
    updateCurrentMotorAngles();

    for (auto i = 0; i < 3; i++) {
        if (!motors[i]->isMotorOnline())
            continue;

        auto mi = static_cast<MotorIndex>(i);
        // float wrappedAxisAngle = getCurrentAngle(mi) - YAW_MOTOR_OFFSET_ANGLES[i];
        // // WRIST_GEAR_RATIO * (DJIEncoderValueToRadians(currentMotorEncoderPosition) - YAW_MOTOR_OFFSET_ANGLES[i]);
        // currentAngles[i]->setValue(wrappedAxisAngle);

        updatePositionPID(i);
        setDesiredOutputToMotor(mi);
    }
}

void WristSubsystem::calculateArmAngles(uint16_t x, uint16_t y, uint16_t z) {
    // delete dis later
    setTargetAngle(PITCH, x);
    setTargetAngle(ROLL, y);
    setTargetAngle(YAW, z);
    // TODO: MATH STUFF LATER
}

// void WristSubsystem::setTargetAngle(int idx, float angle) {
//     // targetAngle[idx] = dynamic_cast<tap::algorithms::ContiguousFloat>(angle);
// }
float yawErrorDisplay = 0.0f;

void WristSubsystem::updatePositionPID(int idx) {
    if (motors[idx]->isMotorOnline()) {
        // float err = currentAngles[idx]->getValue() - targetAngle[idx]->getValue();
        float err = currentAngles[idx]->difference(targetAngle[idx]->getValue());
        if (idx == YAW)
            yawErrorDisplay = err;

        positionPID[idx]->runControllerDerivateError(err);
        desiredMotorOutputs[idx] = positionPID[idx]->getOutput();
    }
}

float yawOutputDisplay;
float pitchOutputDisplay;
float rollOutputDisplay;

void WristSubsystem::setDesiredOutputToMotor(MotorIndex idx) {
    motors[idx]->setDesiredOutput(desiredMotorOutputs[idx]);

    if (idx == YAW) {
        yawOutputDisplay = desiredMotorOutputs[idx];
    }
    if (idx == PITCH) {
        pitchOutputDisplay = desiredMotorOutputs[idx];
    }
    if (idx == ROLL) {
        rollOutputDisplay = desiredMotorOutputs[idx];
    }
}

float currentYaw_display = 0;

float currentYawAngle_d = 0;

// polls motor encoders and stores the data in currentAngle
void WristSubsystem::updateCurrentMotorAngles() {
    for (auto i = 0; i < 3; i++) {
        auto mi = static_cast<MotorIndex>(i);
        currentAngles[i]->setValue((getCurrentAngle(mi) - WRIST_MOTOR_OFFSET_ANGLES[i]) /*/ WRIST_GEAR_RATIO*/);
    }

    currentYaw_display = currentAngles[YAW]->getValue();
}

};  // namespace src::Wrist

#endif  // #ifdef WRIST_COMPATIBLE
