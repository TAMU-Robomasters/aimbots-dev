#pragma once
#ifndef ENGINEER
#include "tap/communication/gpio/leds.hpp"
#include "tap/control/command.hpp"
#include "subsystems/feeder/feeder.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"
#include "drivers.hpp"
#include "informants/limit_switch.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/m3508_constants.hpp"

namespace src::Feeder {

class BurstFeederCommand : public TapCommand {
public:
    BurstFeederCommand(
        src::Drivers*,
        FeederSubsystem*,
        float speed = FEEDER_DEFAULT_RPM,
        float acceptableHeatThreshold = 0.90f,
        int burstLength = DEFAULT_BURST_LENGTH);
    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "Burst Feeder Command"; }

    inline void setBurstLength(int newBurstLength) {
        startingTotalBallCount = feeder->getTotalLimitCount();
        burstLength = newBurstLength;
    }

private:
    src::Drivers* drivers;
    FeederSubsystem* feeder;

    float speed;
    float acceptableHeatThreshold;
    bool canShoot;

    int startingTotalBallCount;
    int burstLength;
};

class FeederSubsystem : public tap::control::Subsystem {
   public:
    FeederSubsystem(
        src::Drivers* drivers);

    mockable void initialize() override;
    mockable void refresh() override;

    void updateMotorVelocityPID();

    mockable void setDesiredOutput();

    mockable float setTargetRPM(float rpm);

    float getTargetRPM() const {
        return targetRPM;
    }

    float getCurrentRPM() const {
        return feederMotor.getShaftRPM();
    }

    int getTotalLimitCount() const;

    bool isBarrelHeatAcceptable(float maxPercentage);

#ifndef ENV_UNIT_TESTS
   private:
#else
   public:
#endif

    float targetRPM;
    float desiredOutput;

    SmoothPID feederVelPID;
    DJIMotor feederMotor;

    src::Informants::LimitSwitch limitSwitchLeft;  // for single-barreled robots
#ifdef TARGET_SENTRY
    src::Informants::LimitSwitch limitSwitchRight;  // for double-barreled robots
#endif

    // commands
};

class FullAutoFeederCommand : public TapCommand {
public:
    FullAutoFeederCommand(src::Drivers*, FeederSubsystem*, float speed = FEEDER_DEFAULT_RPM, float acceptableHeatThreshold = 0.90f);
    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    void setSpeed(float speed) { this->speed = speed; }

    const char* getName() const override { return "run feeder"; }

private:
    src::Drivers* drivers;
    FeederSubsystem* feeder;

    float speed;
    float acceptableHeatThreshold;

    MilliTimeout startupThreshold;
    MilliTimeout unjamTimer;
    float unjamSpeed = 0.0f;
};

class StopFeederCommand : public TapCommand {
public:
    StopFeederCommand(src::Drivers*, FeederSubsystem*);
    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "stop feeder"; }

private:
    src::Drivers* drivers;
    FeederSubsystem* feeder;
};

}  // namespace src::Feeder


#endif
