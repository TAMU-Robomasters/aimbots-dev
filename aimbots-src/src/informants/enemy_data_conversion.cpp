#include "enemy_data_conversion.hpp"

#include <drivers.hpp>

namespace src::Informants {
EnemyDataConversion::EnemyDataConversion(src::Drivers* drivers) : drivers(drivers) {}

void EnemyDataConversion::updateEnemyInfo() {
    if (drivers->cvCommunicator.isJetsonOnline() && drivers->cvCommunicator.getLastValidMessage().cvState >= src::Informants::vision::FOUND) {
        // get CV data for enemy position
        float enemyXPos = 0;  // TODO
        float enemyYPos = 0;  // TODO
        float enemyZPos = 0;  // TODO
        float enemyPos[3] = {enemyXPos, enemyYPos, enemyZPos};
        // timestamp
        uint32_t timestamp_uS = tap::arch::clock::getTimeMicroseconds();

        enemyTimedPosition currentData;
        currentData.position = Matrix<float, 1, 3>(enemyPos);
        currentData.timestamp_uS = timestamp_uS;

        // save to buffer, overwriting oldest data as necessary
        rawPositionBuffer.appendOverwrite(currentData);
    }
}

// what are we doing here? even our latest position may be microseconds out of date :( so do some finite difference BS to get position at our CURRENT
// time with n valid datapoints, we can only go up to the (n-1)th derivative. shame.
enemyTimedData EnemyDataConversion::calculateBestGuess() {
    // for the sake of memory, calculating to the nth order derivative will be an in-place destructive algorithm
    // we'll make an array with [position, velocity, acceleration, jerk, snap, crackle, pop] where each entry is the value at latest time (which is
    // not necessarily current time)

    enemyTimedPosition* validPoints = getLastEntriesWithinTime(0.5);  // arbitrary value rn
    enemyTimedPosition derivatives[size];
    for (int n = 1; n < size - 1; n++)  // calculate and save nth order derivatives
    {
        // calculate nth order derivative
        for (int index = 0; index < size - n; index++) {
            enemyTimedPosition val1 = validPoints[index];
            enemyTimedPosition val2 = validPoints[index + 1];
            uint32_t dt_uS = val1.timestamp_uS - val2.timestamp_uS;
            // the names might be 'dx' but this is actually 'dx/dt'
            float dx = (val1.position[0][X_AXIS] - val2.position[0][X_AXIS]) / dt_uS / MICROSECONDS_PER_SECOND;
            float dy = (val1.position[0][Y_AXIS] - val2.position[0][Y_AXIS]) / dt_uS / MICROSECONDS_PER_SECOND;
            float dz = (val1.position[0][Z_AXIS] - val2.position[0][Z_AXIS]) / dt_uS / MICROSECONDS_PER_SECOND;
            // overwrite data point with calculated derivative
            float diff[3] = {dx, dy, dz};
            val1.position = Matrix<float, 1, 3>(diff);
        }
        // save nth order derivative
        derivatives[n] = validPoints[0];
    }
    // do our approximation
    // latest data time to current time
    uint32_t dt_uS = tap::arch::clock::getTimeMicroseconds() - derivatives[0].timestamp_uS;
    Matrix<float, 1, 3> finalGuess_position = Matrix<float, 1, 3>::zeroMatrix();
    Matrix<float, 1, 3> finalGuess_velocity = Matrix<float, 1, 3>::zeroMatrix();
    Matrix<float, 1, 3> finalGuess_acceleration = Matrix<float, 1, 3>::zeroMatrix();

    // iterate through our n derivatives to generate position prediction
    // x = x0 + v0t + a0t^2 / 2 + j0t^3 / 3 .. etc etc
    finalGuess_position = finalGuess_position + derivatives[0].position;  // the 0th derivative .. position
    for (int n = 1; n < size; n++) {
        // integrals for position
        float diff[3];
        float pos_dt = pow(dt_uS, n) / n;  // dt for position use
        diff[X_AXIS] = derivatives[n].position[0][X_AXIS] * pos_dt;
        diff[Y_AXIS] = derivatives[n].position[0][Y_AXIS] * pos_dt;
        diff[Z_AXIS] = derivatives[n].position[0][Z_AXIS] * pos_dt;
        Matrix<float, 1, 3> nth_thing = Matrix<float, 1, 3>(diff);
        finalGuess_position = finalGuess_position + nth_thing;
        // integrals for velocity
        if (n >= 2) {
            float velo_dt = pow(dt_uS, n - 1) / (n - 1);  // dt for velocity use
            diff[X_AXIS] = derivatives[n].position[0][X_AXIS] * velo_dt;
            diff[Y_AXIS] = derivatives[n].position[0][Y_AXIS] * velo_dt;
            diff[Z_AXIS] = derivatives[n].position[0][Z_AXIS] * velo_dt;
            nth_thing = Matrix<float, 1, 3>(diff);
            finalGuess_velocity = finalGuess_velocity + nth_thing;
        }
        if (n >= 3) {
            float accel_dt = pow(dt_uS, n - 2) / (n - 2);  // dt for acceleration use
            diff[X_AXIS] = derivatives[n].position[0][X_AXIS] * accel_dt;
            diff[Y_AXIS] = derivatives[n].position[0][Y_AXIS] * accel_dt;
            diff[Z_AXIS] = derivatives[n].position[0][Z_AXIS] * accel_dt;
            nth_thing = Matrix<float, 1, 3>(diff);
            finalGuess_acceleration = finalGuess_acceleration + nth_thing;
        }
    }
    // after all that, we have a predicted position, velocity, acceleration, and the time of this prediction
    enemyTimedData enemyGuess;
    enemyGuess.position = finalGuess_position;
    enemyGuess.velocity = finalGuess_velocity;
    enemyGuess.acceleration = finalGuess_acceleration;
    enemyGuess.timestamp_uS = tap::arch::clock::getTimeMicroseconds();

    return enemyGuess;
}

enemyTimedPosition* EnemyDataConversion::getLastEntriesWithinTime(uint32_t time_seconds) {
    size = 0;  // private class member
    uint32_t currentTime_uS;
    enemyTimedPosition* validPositionArray;

    // traverse bounded deque until invalid time found
    for (int index = 0; index < BUFFER_SIZE; index++) {
        currentTime_uS = tap::arch::clock::getTimeMicroseconds();
        enemyTimedPosition position = rawPositionBuffer[index];
        if (currentTime_uS - position.timestamp_uS > time_seconds * 1000000) {
            validPositionArray[index] = position;
            size = index;
        } else {
            size = index;
            break;
        }
    }
    return validPositionArray;
}

}  // namespace src::Informants