#pragma once

#include "utils/common_types.hpp"

#include "drivers.hpp"

using RefSerialRxData = tap::communication::serial::RefSerialData::Rx;

namespace src::Utils {

class RefereeHelperInterface {
public:
    RefereeHelperInterface(src::Drivers* drivers) : drivers(drivers){};
    ~RefereeHelperInterface() = default;

    RefSerialRxData::GameStage getGameStage() {
        auto gameData = drivers->refSerial.getGameData();
        return gameData.gameStage;
    }

protected:
    src::Drivers* drivers;
};

}  // namespace src::Utils
