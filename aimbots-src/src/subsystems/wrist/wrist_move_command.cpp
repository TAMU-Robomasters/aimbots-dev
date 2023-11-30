#include "subsystems/wrist/wrist_move_command.hpp"

#ifdef WRIST_COMPATIBLE

namespace src::Wrist {
WristMoveCommand::WristMoveCommand(
    src::Drivers* drivers, WristSubsystem* wrist, float yaw, float pitch, float roll) 
    : drivers(drivers), wrist(wrist), 
      yaw(yaw), pitch(pitch), roll(roll),
      profileConstraints( //#TODO get real values
          {1.0f,     // max velocity
           2.0f,     // max acceleration
           10.0f}),  // max jerk
      yawProfile(nullptr)
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(wrist));
}

void WristMoveCommand::initialize() {
    float currentYaw = wrist->calculateArmAngles(0, 0, 0); //todo need way to access current yaw
    float yawTarget = yaw - currentYaw; //get offset to target
    wrist->setTargetAngle(YAW, yaw);
    wrist->setTargetAngle(PITCH, pitch);
    wrist->setTargetAngle(ROLL, roll);

// idk why this is angry, ig the S-curve class cant handle different directions so this does
    if (displacementTarget < 0.0f) {
        profilerDirection = -1;
    } else {
        profilerDirection = 1;
    }

    if (yawProfile != nullptr) { //if an old profile exists, kill it
        delete yawProfile;
    }

    //idk why squiggles. Create profile and define start time
    yawProfile = new SCurveMotionProfile(profileConstraints, fabs(yawTarget));

    movementStartTime = tap::arch::clock::getTimeMilliseconds();

}

void WristMoveCommand::execute() {
    float currTime = tap::arch::clock::getTimeMilliseconds();

    auto step = yawProfile->stepAtTime((currTime - movementStartTime) / 1000.0f);

    //TODO: conversion ratios stolen from sentry, need to use actual numbers
    float wheelTargetRPM = ((step.velocity * profilerDirection) / (2.0f * M_PI * WHEEL_RADIUS)) * 60.0f;
    float motorTargetRPM = -(wheelTargetRPM / CHASSIS_GEARBOX_RATIO);
    wrist->setTargetRPMs(motorTargetRPM, 0.0f, 0.0f);

}

void WristMoveCommand::end(bool) {
     if (yawProfile != nullptr) { //kill profile
        delete yawProfile;
    }
    wrist->setTargetRPMs(0.0f, 0.0f, 0.0f); // stop moving
}

};  // namespace src::Wrist

#endif  // #ifdef WRIST_COMPATIBLE