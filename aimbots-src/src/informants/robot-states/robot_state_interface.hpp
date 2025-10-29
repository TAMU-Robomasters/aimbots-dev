#pragma once

#ifdef REF_COMM_COMPATIBLE

#include "tap/communication/serial/ref_serial_transmitter.hpp"

#include "informants/communication/communication_message.hpp"
// #include "informants/communication/communication_response_handler.hpp"
// #include "utils/tools/common_types.hpp"
#include "tap/control/subsystem.hpp"

#include "informants/communication/communication_response_handler.hpp"

#include "drivers.hpp"
#include "robot_state.hpp"
using namespace src::Communication;

namespace src::RobotStates {
class RobotStates : public tap::control::Subsystem {
private:
    Matrix<Robot, 2, 9> robotStates;
    tap::Drivers& drivers;
    CommunicationResponseHandler messageHandler;

public:
    RobotStates(tap::Drivers& drivers);
    ~RobotStates();

    void refresh() override;

    void setIndividualRobotState(Robot robot);
    Robot getIndividualRobotState(int robotNumber, Team teamColor);

    void updateRobotState(int robotNumber, Team teamColor, short x, short y, short z, int health);
    void updateRobotStateHealth(int robotNumber, Team teamColor, int health);
    void updateRobotStatePosition(int robotNumber, Team teamColor, short x, short y, short z);

#ifdef ALL_SENTRIES
    void updateTeamMessage();
    uint8_t teamMessage[115];
#endif
#ifdef TARGET_STANDARD
    void updateStandardMessage();
    // uint8_t standardMessage[115];
#endif
#ifdef TARGET_HERO
    void updateHeroMessage();
    // uint8_t heroMessage[115];
#endif
    void respond();

    // void updateRobotStateHero();
    // void updateRobotStateStandard();
    // void updateRobotStateSentry();
};

}  // namespace src::RobotStates

#endif  // #ifdef REF_COMM_COMPATIBLE