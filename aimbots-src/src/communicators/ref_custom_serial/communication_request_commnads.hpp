#pragma once

#include "tap/control/command.hpp"

// this doesn't exist
//  #include "communication_outbound_subsystem.hpp"

#ifdef REF_COMM_COMPATIBLE

namespace src::Communication {

#ifdef ALL_SENTRIES
class TeamMessageStandard : public tap::control::Command {
public:
    TeamMessageStandard(CommunicationOutBoundSubsystem &communicationOutBoundSubsystem)
        : sub(communicationOutBoundSubsystem) {
        this->addSubsyemRequirement(&communicationOutBoundSubsystem);
    }

    const char *getName() const { return "team standard info"; }
    bool isReady() { return true; }
    void initalize() { this->sub.queueRequest(MessageType::TEAM_MESSAGE_STANDARD); }
    void execute() {}
    void end(bool) {}
    bool isFinished() const { return true; }

private:
    CommunicationOutBoundSubsystem &sub;
};

class TeamMessageHero : public tap::control::Command {
public:
    TeamMessageHero(CommunicationOutBoundSubsystem &communicationOutBoundSubsystem) : sub(communicationOutBoundSubsystem) {
        this->addSubsyemRequirement(&communicationOutBoundSubsystem);
    }

    const char *getName() const { return "team hero info"; }
    bool isReady() { return true; }
    void initalize() { this->sub.queueRequest(MessageType::TEAM_MESSAGE_HERO); }
    void execute() {}
    void end(bool) {}
    bool isFinished() const { return true; }

private:
    CommunicationOutBoundSubsystem &sub;
};

#else
class CommunicationMessage : public tap::control::Command {
public:
    communicationMessage(CommunicationOutBoundSubsystem &communicationOutBoundSubsystem)
        : sub(communicationOutBoundSubsystem) {
        this->addSubsyemRequirement(&communicationOutBoundSubsystem);
    }

    const char *getName() const { return "robot state"; }
    bool isReady() { return true; }
    void initalize() { this->sub.queueRequest(MessageType::ROBOT_STATE); }
    void execute() {}
    void end(bool) {}
    bool isFinished() const { return true; }

private:
    CommunicationOutBoundSubsystem &sub;
};

class EnemyStateMessage : public tap::control::Command {
public:
    EnemyStateMessage(CommunicationOutBoundSubsystem &communicationOutBoundSubsystem) : sub(communicationOutBoundSubsystem) {
        this->addSubsyemRequirement(&communicationOutBoundSubsystem);
    }

    const char *getName() const { return "enemey state"; }
    bool isReady() { return true; }
    void initalize() { this->sub.queueRequest(MessageType::ROBOT_STATE); }
    void execute() {}
    void end(bool) {}
    bool isFinished() const { return true; }

private:
    CommunicationOutBoundSubsystem &sub;
};

#endif
}  // namespace src::Communication

#endif  //#ifdef REF_COMM_COMPATIBLE
