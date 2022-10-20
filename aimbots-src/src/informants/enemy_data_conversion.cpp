#include "enemy_data_conversion.hpp"

#include <drivers.hpp>

namespace src::Informants {
EnemyDataConversion::EnemyDataConversion(src::Drivers* drivers) : drivers(drivers) {}

void EnemyDataConversion::updateConversion() {
    if (drivers->cvCommunicator.isJetsonOnline() && drivers->cvCommunicator.getLastValidMessage().cvState >= src::Informants::vision::FOUND) {
        // get CV data for enemy position
        float enemyXPos = 0;  // TODO
        float enemyYPos = 0;  // TODO
        float enemyZPos = 0;  // TODO
        float enemyPos[3] = {enemyXPos, enemyYPos, enemyZPos};
        // timestamp
        uint32_t timestamp = tap::arch::clock::getTimeMicroseconds();

        enemyTimedPosition currentData;
        currentData.position = Matrix<float, 1, 3>(enemyPos);
        currentData.timestamp_uS = timestamp;

        // save to buffer, overwriting oldest data as necessary
        rawPositionBuffer.appendOverwrite(currentData);
    }
}

}  // namespace src::Informants