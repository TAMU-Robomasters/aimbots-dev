#pragma once

#include "tap/control/command.hpp"

#include "robot_state_outbound_subsystem.hpp"

namespace src::robotStates {

#ifdef TARGET_SENTRY
class TeamMessageStandard : public tap::control::Command {
public:
    TeamMessageStandard(RobotStateOutBoundSubsystem &robotStateOutBoundSubsystem) : sub(robotStateOutBoundSubsystem) {
        this->addSubsyemRequirement(&robotStateOutBoundSubsystem);
    }

    const char *getName() const { return "team standard info"; }
    bool isReady() { return true; }
    void initalize() { this->sub.queueRequest(MessageType::TEAM_MESSAGE_STANDARD); }
    void execute() {}
    void end(bool) {}
    bool isFinished() const { return true; }

private:
    RobotStateOutBoundSubsystem &sub;
};

class TeamMessageHero : public tap::control::Command {
public:
    TeamMessageHero(RobotStateOutBoundSubsystem &robotStateOutBoundSubsystem) : sub(robotStateOutBoundSubsystem) {
        this->addSubsyemRequirement(&robotStateOutBoundSubsystem);
    }

    const char *getName() const { return "team hero info"; }
    bool isReady() { return true; }
    void initalize() { this->sub.queueRequest(MessageType::TEAM_MESSAGE_HERO); }
    void execute() {}
    void end(bool) {}
    bool isFinished() const { return true; }

private:
    RobotStateOutBoundSubsystem &sub;
};

#else
class RobotStateMessage : public tap::control::Command {
public:
    RobotStateMessage(RobotStateOutBoundSubsystem &robotStateOutBoundSubsystem) : sub(robotStateOutBoundSubsystem) {
        this->addSubsyemRequirement(&robotStateOutBoundSubsystem);
    }

    const char *getName() const { return "robot state"; }
    bool isReady() { return true; }
    void initalize() { this->sub.queueRequest(MessageType::ROBOT_STATE); }
    void execute() {}
    void end(bool) {}
    bool isFinished() const { return true; }

private:
    RobotStateOutBoundSubsystem &sub;
};

class EnemyStateMessage : public tap::control::Command {
public:
    EnemyStateMessage(RobotStateOutBoundSubsystem &robotStateOutBoundSubsystem) : sub(robotStateOutBoundSubsystem) {
        this->addSubsyemRequirement(&robotStateOutBoundSubsystem);
    }

    const char *getName() const { return "enemey state"; }
    bool isReady() { return true; }
    void initalize() { this->sub.queueRequest(MessageType::ROBOT_STATE); }
    void execute() {}
    void end(bool) {}
    bool isFinished() const { return true; }

private:
    RobotStateOutBoundSubsystem &sub;
};

#endif
}  // namespace src::robotStates
