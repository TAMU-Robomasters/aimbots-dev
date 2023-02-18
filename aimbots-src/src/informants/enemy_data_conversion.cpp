#include "enemy_data_conversion.hpp"

#include <drivers.hpp>

namespace src::Informants {
EnemyDataConversion::EnemyDataConversion(src::Drivers* drivers) : drivers(drivers) {}

    float targetXCoordDisplay_camera = 0;
    float targetYCoordDisplay_camera = 0;
    float targetZCoordDisplay_camera = 0;

    Matrix<float,3,1> enemyPositionDisplay_gimbal;
    float targetXCoordDisplay_gimbal = 0;
    float targetYCoordDisplay_gimbal = 0;
    float targetZCoordDisplay_gimbal = 0;

    float targetXCoordDisplay_chassis = 0;
    float targetYCoordDisplay_chassis = 0;
    float targetZCoordDisplay_chassis = 0;

void EnemyDataConversion::updateEnemyInfo() {
    if (true || drivers->cvCommunicator.isJetsonOnline() && drivers->cvCommunicator.getLastValidMessage().cvState >= src::Informants::vision::FOUND) {
        prev_cv_valid = cv_valid;
        cv_valid = true;

        // if our cv connection just turned back on... throw out the data
        if (cv_valid && !prev_cv_valid) {
            rawPositionBuffer.clear();  // goodbye
        }
        // get CV data for enemy position
        // jetson communicator is outdated, need to talk with CV about new jetson messages with position :(
        float enemyXPos = 0;  // TODO
        float enemyYPos = 0;  // TODO
        float enemyZPos = 0;  // TODO
        float enemyPos[3] = {enemyXPos, enemyYPos, enemyZPos};
        // timestamp
        // !! NOTE !!
        // TIMESTAMPING IS PRONE TO CHANGE, TALK WITH CV!!!
        uint32_t timestamp_uS = tap::arch::clock::getTimeMicroseconds();

        enemyTimedPosition currentData;
        // currentData.position = Matrix<float, 3, 1>(enemyPos);
        updateAndGetEnemyPosition(currentData.position);  // because we're apparently starting sentry testing very very soon
        currentData.timestamp_uS = timestamp_uS;

        // save to buffer, overwriting oldest data as necessary
        rawPositionBuffer.appendOverwrite(currentData);
    } else {
        prev_cv_valid = cv_valid;
        cv_valid = false;
    }
}

vector<enemyTimedPosition> EnemyDataConversion::getLastEntriesWithinTime(float time_seconds) {
    vector<enemyTimedPosition> validPositions;
    //avoid 0 entries, debug only?
    validPositions.push_back(rawPositionBuffer[0]);
    uint32_t currentTime_uS = tap::arch::clock::getTimeMicroseconds();
    // traverse bounded deque until invalid time found
    for (int index = 1; index < BUFFER_SIZE; index++) {
        enemyTimedPosition pos = rawPositionBuffer[index];
        if (currentTime_uS - pos.timestamp_uS < time_seconds * MICROSECONDS_PER_SECOND && sqrt(pow(pos.position[X_AXIS][0],2) + pow(pos.position[Y_AXIS][0],2) + pow(pos.position[Z_AXIS][0] != 0,2)) >= 0) {
            validPositions.push_back(pos);
        } else {
            break;
        }
    }
    return validPositions;
}

// !!! note from future self: stop taking so many damn derivatives
// What are we doing here? even our latest position may be microseconds out of date :( so do some finite difference BS to get position at our CURRENT
// time. With n valid datapoints, we can only go up to the (n-1)th derivative. shame.
enemyTimedData EnemyDataConversion::calculateBestGuess() {
    // for the sake of memory, calculating to the nth order derivative will be an in-place destructive algorithm.
    // we'll make an array with [position, velocity, acceleration, jerk, snap, crackle, pop] where each entry is the value at latest time (which is
    // not necessarily current time)

    //Sets the finite difference accuracy
    const int ACCURACY = 1;

    float order1CoEffs[3][4] = {{0,0,-1,1},
                            {0,0.5,-2,1.5},
                            {(-1.0f/3.0f), (-3.0f/2.0f), (-3), (11.0f/6.0f)}};

    float order2CoEffs[3][5] = {{0,0,1,-2,1},
                            {0,-1,4,-5,2},
                            {(-35.0f/12.0f),(26.0f/3.0f),(-19.0f/2.0f),(14.0f/3.0f),(-11.0f/12.0f)}};



    vector<enemyTimedPosition> validPoints;
    vector<enemyTimedData> calculatedData; 
    validPoints = this->getLastEntriesWithinTime(VALID_TIME);
    int size = validPoints.size();

    float v_guess[3];
    float a_guess[3];

    enemyTimedData enemyGuess;

    for (int p = 0; p < size; p++) {
        Matrix<float, 3, 1> guess_position = Matrix<float, 3, 1>::zeroMatrix();
        Matrix<float, 3, 1> guess_velocity = Matrix<float, 3, 1>::zeroMatrix();
        Matrix<float, 3, 1> guess_acceleration = Matrix<float, 3, 1>::zeroMatrix();

        if (p >= 0) {
            //Position
            guess_position = validPoints[p].position;
        }
        if (p >= (1+ACCURACY)) {
            //Velocity
            
            for (int axis = 0; axis < 3; axis++) {
                v_guess[axis] = (validPoints[p].position[axis][0] - validPoints[p-1].position[axis][0]) / (validPoints[p].timestamp_uS - validPoints[p-1].timestamp_uS);

            }
            guess_velocity = Matrix<float, 3, 1>(v_guess);

        }
        if (p >= (2+ACCURACY)) { 
            //Acceleration
            
            for (int axis = 0; axis < 3; axis++) {
                a_guess[axis] = (validPoints[p-2].position[axis][0] - (2.0f * validPoints[p-1].position[axis][0]) + validPoints[p].position[axis][0]) 
                                /
                                ((validPoints[p-1].timestamp_uS-validPoints[p-2].timestamp_uS) * (validPoints[p].timestamp_uS - validPoints[p-1].timestamp_uS));
            }
            
            guess_acceleration = Matrix<float, 3, 1>(a_guess);
        }

        enemyGuess.position = guess_position;
        enemyGuess.velocity = guess_velocity;
        enemyGuess.acceleration = guess_acceleration;
        enemyGuess.timestamp_uS = validPoints[p].timestamp_uS;

        calculatedData.push_back(enemyGuess);
    }

    return calculatedData[size-1];

    


    /*
    enemyTimedPosition derivatives[size];
    for (int n = 1; n < size; n++)  // calculate and save nth order derivatives
    {
        // calculate nth order derivative
        for (int index = 0; index < size - n; index++) {
            enemyTimedPosition val1 = validPoints[index];
            enemyTimedPosition val2 = validPoints[index + 1];
            uint32_t dt_uS = val1.timestamp_uS - val2.timestamp_uS;
            // the names might be 'dx' but this is actually 'dx/dt' -- dt is seconds!!!
            float dx = (val1.position[X_AXIS][0] - val2.position[X_AXIS][0]) / dt_uS * MICROSECONDS_PER_SECOND;
            float dy = (val1.position[Y_AXIS][0] - val2.position[Y_AXIS][0]) / dt_uS * MICROSECONDS_PER_SECOND;
            float dz = (val1.position[Z_AXIS][0] - val2.position[Z_AXIS][0]) / dt_uS * MICROSECONDS_PER_SECOND;
            // overwrite data point with calculated derivative
            float diff[3] = {dx, dy, dz};
            val1.position = Matrix<float, 3, 1>(diff);
        }
        // save nth order derivative
        derivatives[n] = validPoints[0];
    }
    // do our approximation
    // latest data time to current time
    uint32_t dt_uS = tap::arch::clock::getTimeMicroseconds() - derivatives[0].timestamp_uS;
    Matrix<float, 3, 1> finalGuess_position = Matrix<float, 3, 1>::zeroMatrix();
    Matrix<float, 3, 1> finalGuess_velocity = Matrix<float, 3, 1>::zeroMatrix();
    Matrix<float, 3, 1> finalGuess_acceleration = Matrix<float, 3, 1>::zeroMatrix();

    // iterate through our n derivatives to generate position prediction
    // x = x0 + v0t + a0t^2 / 2 + j0t^3 / 3 .. etc etc
    finalGuess_position = finalGuess_position + derivatives[0].position;  // the 0th derivative .. position
    int the_number = 1;                                                   // within the for loop this should be equal to factorial(n)
    for (int n = 1; n < size; n++) {
        // integrals for position
        float diff[3];
        the_number *= n;
        float pos_dt = pow(dt_uS, n) / the_number;  // dt for position use
        diff[X_AXIS] = derivatives[n].position[X_AXIS][0] * pos_dt;
        diff[Y_AXIS] = derivatives[n].position[Y_AXIS][0] * pos_dt;
        diff[Z_AXIS] = derivatives[n].position[Z_AXIS][0] * pos_dt;
        Matrix<float, 3, 1> nth_thing = Matrix<float, 3, 1>(diff);
        finalGuess_position = finalGuess_position + nth_thing;
        // integrals for velocity
        if (n >= 2) {
            float velo_dt = pow(dt_uS, n - 1) / (the_number - 1);  // dt for velocity use
            diff[X_AXIS] = derivatives[n].position[X_AXIS][0] * velo_dt;
            diff[Y_AXIS] = derivatives[n].position[Y_AXIS][0] * velo_dt;
            diff[Z_AXIS] = derivatives[n].position[Z_AXIS][0] * velo_dt;
            nth_thing = Matrix<float, 3, 1>(diff);
            finalGuess_velocity = finalGuess_velocity + nth_thing;
        }
        if (n >= 3) {
            float accel_dt = pow(dt_uS, n - 2) / (the_number - 2);  // dt for acceleration use
            diff[X_AXIS] = derivatives[n].position[X_AXIS][0] * accel_dt;
            diff[Y_AXIS] = derivatives[n].position[Y_AXIS][0] * accel_dt;
            diff[Z_AXIS] = derivatives[n].position[Z_AXIS][0] * accel_dt;
            nth_thing = Matrix<float, 3, 1>(diff);
            finalGuess_acceleration = finalGuess_acceleration + nth_thing;
        }
    }
    */

    

    /* god I hope I don't have to uncomment this
    // final guess is transformed for now
    Matrix<float, 4, 4> T_cam2gimb = src::utils::MatrixHelper::transform_matrix(R_cam2gimb, P_cam2gimb);
    Matrix<float, 4, 4> T_gimb2chas = src::utils::MatrixHelper::transform_matrix(R_gimb2chas, P_gimb2chas);
    Matrix<float, 3, 3> R_cam2gimb_matrix = Matrix<float, 3, 3>(R_cam2gimb);
    Matrix<float, 3, 3> R_gimb2chas_matrix = Matrix<float, 3, 3>(R_gimb2chas);
    // for Position we use T
    // remove last 1                                                // adds a 1 at end
    enemyGuess.position =
        src::utils::MatrixHelper::P_crop_extend(T_gimb2chas * T_cam2gimb * src::utils::MatrixHelper::P_crop_extend(finalGuess_position));
    // for Acceleration & Velocity we use R
    enemyGuess.velocity = R_gimb2chas_matrix * R_cam2gimb_matrix * finalGuess_velocity;
    enemyGuess.acceleration = R_gimb2chas_matrix * R_cam2gimb_matrix * finalGuess_acceleration;
    */

    // after all that, we have a predicted position, velocity, acceleration, and the time of this prediction
    // in chassis space!!!
    // enemyGuess.position = finalGuess_position;
    // enemyGuess.velocity = finalGuess_velocity;
    // enemyGuess.acceleration = finalGuess_acceleration;
    // enemyGuess.timestamp_uS = tap::arch::clock::getTimeMicroseconds();

    // return enemyGuess;
}

bool EnemyDataConversion::updateAndGetEnemyPosition(Matrix<float, 3, 1>& enemyPosition) {
    if (true || drivers->cvCommunicator.isJetsonOnline()) {
        // get XYZ
        // undebug
        // float targetXCoord = drivers->cvCommunicator.getLastValidMessage().targetX;
        // float targetYCoord = drivers->cvCommunicator.getLastValidMessage().targetY;
        // float targetZCoord = drivers->cvCommunicator.getLastValidMessage().targetZ;

        //debug
        float targetXCoord = 0;
        float targetYCoord = 1;
        float targetZCoord = 0;
        enemyPosition[X_AXIS][0] = targetXCoord;
        enemyPosition[Y_AXIS][0] = targetYCoord;
        enemyPosition[Z_AXIS][0] = targetZCoord;

        targetXCoordDisplay_camera = targetXCoord;
        targetYCoordDisplay_camera = targetYCoord;
        targetZCoordDisplay_camera = targetZCoord;

        // now that we have enemy position (in METERS), transform to chassis space ! ! !
        //update matrices
        updateTransformations();
        //pull matricies
        Matrix<float, 4, 4> T_cam2gimb = src::utils::MatrixHelper::transform_matrix(R_cam2gimb, P_cam2gimb);
        Matrix<float, 4, 4> T_gimb2chas = src::utils::MatrixHelper::transform_matrix(R_gimb2chas, P_gimb2chas);
        Matrix<float, 3, 3> R_cam2gimb_matrix = Matrix<float, 3, 3>(R_cam2gimb);
        Matrix<float, 3, 3> R_gimb2chas_matrix = Matrix<float, 3, 3>(R_gimb2chas);
        // for Position we use T
        // remove last 1                                                // adds a 1 at end
        //debug..
        enemyPositionDisplay_gimbal = src::utils::MatrixHelper::P_crop_extend(T_cam2gimb * src::utils::MatrixHelper::P_crop_extend(enemyPosition));
        enemyPosition =
            src::utils::MatrixHelper::P_crop_extend(T_gimb2chas * T_cam2gimb * src::utils::MatrixHelper::P_crop_extend(enemyPosition));

        targetXCoordDisplay_gimbal = enemyPositionDisplay_gimbal[X_AXIS][0];
        targetYCoordDisplay_gimbal = enemyPositionDisplay_gimbal[Y_AXIS][0];
        targetZCoordDisplay_gimbal = enemyPositionDisplay_gimbal[Z_AXIS][0];

        targetXCoordDisplay_chassis = enemyPosition[X_AXIS][0];
        targetYCoordDisplay_chassis = enemyPosition[Y_AXIS][0];
        targetZCoordDisplay_chassis = enemyPosition[Z_AXIS][0];
        return true;
    }
    return false;
}

float theta_display;
float phi_display;

void EnemyDataConversion::updateTransformations() {
    float theta = gimbal->getCurrentChassisRelativeYawAngle(AngleUnit::Radians) - modm::toRadian(YAW_START_ANGLE);
    float phi = gimbal->getCurrentChassisRelativePitchAngle(AngleUnit::Radians) - modm::toRadian(PITCH_START_ANGLE);
    theta_display = theta;
    phi_display = phi;
    auto cph=cos(phi), sph=sin(phi);
    auto cth=cos(theta), sth=sin(theta);
    R_gimb2chas[0] = cth;
    R_gimb2chas[1] = cph*sth;
    R_gimb2chas[2] = sth*sph;
    R_gimb2chas[3] = -sth;
    R_gimb2chas[4] = cph*cth;
    R_gimb2chas[5] = sph*cth;
    R_gimb2chas[6] = 0;
    R_gimb2chas[7] = -sph;
    R_gimb2chas[8] = cph;
}

}  // namespace src::Informants