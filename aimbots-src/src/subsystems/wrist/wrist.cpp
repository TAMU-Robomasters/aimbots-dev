#include "subsystems/wrist/wrist.hpp"

#include <cmath>

#include "utils/common_types.hpp"

#ifdef WRIST_COMPATIBLE

namespace src::Wrist {

WristSubsystem::WristSubsystem(src::Drivers* drivers)
    : Subsystem(drivers),
    motors {
        new DJIMotor(drivers, WRIST_YAW_MOTOR_ID, WRIST_BUS, false, "yaw"),      // yaw motor
        new DJIMotor(drivers, WRIST_PITCH_MOTOR_ID, WRIST_BUS, false, "pitch"),  // pitch motor
        new DJIMotor(drivers, WRIST_ROLL_MOTOR_ID, WRIST_BUS, false, "roll")     // roll motor
    },
    positionPIDs {
        new SmoothPID(WRIST_POSITION_PID_CONFIG),
        new SmoothPID(WRIST_POSITION_PID_CONFIG),
        new SmoothPID(WRIST_POSITION_PID_CONFIG)
    },
    targetAngles {
        new ContiguousFloat(0.0f, -M_PI, M_PI),
        new ContiguousFloat(0.0f, -M_PI, M_PI),
        new ContiguousFloat(0.0f, -M_PI, M_PI)
    },
    currentAngles {
        new ContiguousFloat(WRIST_MOTOR_OFFSET_ANGLES[YAW], -M_PI, M_PI),
        new ContiguousFloat(WRIST_MOTOR_OFFSET_ANGLES[PITCH], -M_PI, M_PI),
        new ContiguousFloat(WRIST_MOTOR_OFFSET_ANGLES[ROLL], -M_PI, M_PI)
    }
{
}

void WristSubsystem::initialize() {
    for (int i = 0; i < 3; i++) {
        motors[i]->initialize();
        motors[i]->setDesiredOutput(0.0f);
    }
}

int failCounter = 0;

void WristSubsystem::refresh() {
    if (!isOnline()) {
        failCounter++;
        return;
    }

    updateCurrentMotorAngles();

    for (size_t i = 0; i < motors.size(); i++) {
        if (motors[i]->isMotorOnline()) {
            auto mi = static_cast<MotorIndex>(i);
            updateMotorPositionPID(mi);
            setDesiredOutputToMotor(mi);
        }
    }
}

void WristSubsystem::calculateArmAngles(uint16_t x, uint16_t y, uint16_t z) {
    // delete dis later
    setTargetAngle(PITCH, x);
    setTargetAngle(ROLL, y);
    setTargetAngle(YAW, z);
    // TODO: MATH STUFF LATER
}

void WristSubsystem::updateMotorPositionPID(MotorIndex idx) {
    if (motors[idx]->isMotorOnline()) {
        // TODO: ACTUALLY SET THE GEAR RATIOS PROPERLY AND GENERALLY, IT'S NOT 19
        // ContiguousFloat::difference does (other - this), and we want (target - current)
        float output_err = currentAngles[idx]->difference(targetAngles[idx]->getValue());

        positionPIDs[idx]->runControllerDerivateError(output_err);
        desiredMotorOutputs[idx] = positionPIDs[idx]->getOutput();
    }
}

void WristSubsystem::setDesiredOutputToMotor(MotorIndex idx) {
    motors[idx]->setDesiredOutput(desiredMotorOutputs[idx]);
}

// Updates the current OUTPUT angles scaled after gear boxes
void WristSubsystem::updateCurrentMotorAngles() {
    for (size_t i = 0; i < motors.size(); i++) {
        auto mi = static_cast<MotorIndex>(i);
        // TODO make this more general later, it is NOT 19, TRUST ME
        float angleScaled = getCurrentAngleUnwrappedRadians(mi) / 19.0f;
        float angleOffsetted = angleScaled - WRIST_MOTOR_OFFSET_ANGLES[i];
        currentAngles[i]->setValue(angleOffsetted);
    }
}

};  // namespace src::Wrist

#endif  // #ifdef WRIST_COMPATIBLE
