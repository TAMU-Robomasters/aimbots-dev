#pragma once
#ifndef ENGINEER
#include "tap/communication/gpio/leds.hpp"
#include "tap/control/command.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"
#include "drivers.hpp"
#include "informants/limit_switch.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/m3508_constants.hpp"
#include "robots/hero/hero_constants.hpp"

namespace src::Indexer {

class IndexerSubsystem : public tap::control::Subsystem {
   public:
    IndexerSubsystem(
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
        return indexerMotor.getShaftRPM();
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

    SmoothPID indexerVelPID;
    DJIMotor indexerMotor;

    src::Informants::LimitSwitch limitSwitchLeft;  // for single-barreled robots
#ifdef TARGET_SENTRY
    src::Informants::LimitSwitch limitSwitchRight;  // for double-barreled robots
#endif

    // commands
};

class BurstIndexerCommand : public TapCommand {
public:
    BurstIndexerCommand(
        src::Drivers*,
        IndexerSubsystem*,
        float speed = INDEXER_DEFAULT_RPM,
        float acceptableHeatThreshold = 0.90f,
        int burstLength = DEFAULT_BURST_LENGTH);
    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "Burst Indexer Command"; }

    inline void setBurstLength(int newBurstLength) {
        startingTotalBallCount = indexer->getTotalLimitCount();
        burstLength = newBurstLength;
    }

private:
    src::Drivers* drivers;
    IndexerSubsystem* indexer;

    float speed;
    float acceptableHeatThreshold;
    bool canShoot;

    int startingTotalBallCount;
    int burstLength;
};

class FullAutoIndexerCommand : public TapCommand {
public:
    FullAutoIndexerCommand(src::Drivers*, IndexerSubsystem*, float speed = INDEXER_DEFAULT_RPM, float acceptableHeatThreshold = 0.90f);
    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    void setSpeed(float speed) { this->speed = speed; }

    const char* getName() const override { return "run indexer"; }

private:
    src::Drivers* drivers;
    IndexerSubsystem* indexer;

    float speed;
    float acceptableHeatThreshold;

    MilliTimeout startupThreshold;
    MilliTimeout unjamTimer;
    float unjamSpeed = 0.0f;
};

class StopIndexerCommand : public TapCommand {
public:
    StopIndexerCommand(src::Drivers*, IndexerSubsystem*);
    void initialize() override;

    void execute() override;
    void end(bool interrupted) override;
    bool isReady() override;

    bool isFinished() const override;

    const char* getName() const override { return "stop indexer"; }

private:
    src::Drivers* drivers;
    IndexerSubsystem* indexer;
    
};

}  // namespace src::Indexer


#endif
