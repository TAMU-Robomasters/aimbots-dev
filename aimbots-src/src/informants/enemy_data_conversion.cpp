#include "enemy_data_conversion.hpp"

#include <drivers.hpp>

namespace src::Informants {
EnemyDataConversion::EnemyDataConversion(src::Drivers* drivers) : drivers(drivers) {}

    //watchable variables
    float targetXCoordDisplay_camera = 0;
    float targetYCoordDisplay_camera = 0;
    float targetZCoordDisplay_camera = 0;
    //
    Matrix<float,3,1> enemyPositionDisplay_gimbal;
    float targetXCoordDisplay_gimbal = 0;
    float targetYCoordDisplay_gimbal = 0;
    float targetZCoordDisplay_gimbal = 0;
    //
    float targetXCoordDisplay_chassis = 0;
    float targetYCoordDisplay_chassis = 0;
    float targetZCoordDisplay_chassis = 0;
    //
    int size_watch;
    float dt_vel_watch;
    int accuracy_watch;
    float watchVelX;
    int buffer_size_watch;
    float last_entry_timestamp_watch;

//gather data, transform data, 
void EnemyDataConversion::updateEnemyInfo() {
        // clear buffer if CV connection just turned back on
        prev_cv_valid = cv_valid;
        cv_valid = true;
        if (cv_valid && !prev_cv_valid) { rawPositionBuffer.clear(); } // goodbye 
        
        // get CV data for enemy position
        // undebug (unremove this later)
        float targetXPos = drivers->cvCommunicator.getLastValidMessage().targetX;
        float targetYPos = drivers->cvCommunicator.getLastValidMessage().targetY;
        float targetZPos = drivers->cvCommunicator.getLastValidMessage().targetZ;
        float captureDelay_ms = drivers->cvCommunicator.getLastValidMessage().delay;

        // float targetXPos = 0;
        // float targetYPos = 0;
        // float targetZPos = 0;

        enemyTimedPosition currentData{
            .position = Vector3f({targetXPos, targetYPos, targetZPos}),
            .timestamp_uS = (captureDelay_ms * 1000) + tap::arch::clock::getTimeMicroseconds(),
        };
        
        // now that we have enemy position (in METERS), transform to chassis space ! ! !
        //update matrices
        updateTransformations();
        //pull matricies
        Matrix<float, 4, 4> T_cam2gimb = src::utils::MatrixHelper::transform_matrix(R_cam2gimb, P_cam2gimb);
        Matrix<float, 4, 4> T_gimb2chas = src::utils::MatrixHelper::transform_matrix(R_gimb2chas, P_gimb2chas);
        Matrix<float, 3, 3> R_cam2gimb_matrix = Matrix<float, 3, 3>(R_cam2gimb);
        Matrix<float, 3, 3> R_gimb2chas_matrix = Matrix<float, 3, 3>(R_gimb2chas);
        //save transformed position to data point
        enemyPositionDisplay_gimbal = src::utils::MatrixHelper::P_crop_extend(T_cam2gimb * src::utils::MatrixHelper::P_crop_extend(currentData.position.asMatrix()));
        currentData.position =
            src::utils::MatrixHelper::P_crop_extend(T_gimb2chas * T_cam2gimb * src::utils::MatrixHelper::P_crop_extend(currentData.position.asMatrix()));

        // save data point to buffer (at index 0-- index 0 is NEWEST, index size-1 is OLDEST.)
        // at max capacity, oldest data is overwritten first
        rawPositionBuffer.prependOverwrite(currentData);

        //watchable variables
        buffer_size_watch = rawPositionBuffer.getSize();
        last_entry_timestamp_watch = rawPositionBuffer[0].timestamp_uS;
        //
        targetXCoordDisplay_camera = currentData.position.getX();
        targetYCoordDisplay_camera = currentData.position.getY();
        targetZCoordDisplay_camera = currentData.position.getZ();
        //
        targetXCoordDisplay_gimbal = enemyPositionDisplay_gimbal[X_AXIS][0];
        targetYCoordDisplay_gimbal = enemyPositionDisplay_gimbal[Y_AXIS][0];
        targetZCoordDisplay_gimbal = enemyPositionDisplay_gimbal[Z_AXIS][0];
        //
        targetXCoordDisplay_chassis = currentData.position.getX();
        targetYCoordDisplay_chassis = currentData.position.getY();
        targetZCoordDisplay_chassis = currentData.position.getZ();

    // else {
    //     prev_cv_valid = cv_valid;
    //     cv_valid = false;
    //     rawPositionBuffer.clear();
    // }
}

bool canSendData;

float raw_x_display;
float raw_time_display;
float raw_x_display2;


std::vector<enemyTimedPosition> EnemyDataConversion::getLastEntriesWithinTime(float time_seconds) {
    vector<enemyTimedPosition> validPositions;
    uint32_t currentTime_uS = tap::arch::clock::getTimeMicroseconds();
    raw_x_display = rawPositionBuffer[0].position.getX();
    raw_time_display = rawPositionBuffer[0].timestamp_uS;
    raw_x_display2 = rawPositionBuffer[1].position.getX();
    // traverse bounded deque until invalid time found
    for (int index = 0; index < BUFFER_SIZE; index++) {
        enemyTimedPosition pos = rawPositionBuffer[index];
        canSendData = (currentTime_uS - pos.timestamp_uS < time_seconds * (float)MICROSECONDS_PER_SECOND);
        if ((pow(pos.position.getX(),2) + pow(pos.position.getY(),2) + pow(pos.position.getZ(),2)) > 0) {
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
plateKinematicState EnemyDataConversion::calculateBestGuess(int desired_finite_diff_accuracy) {
    // for the sake of memory, calculating to the nth order derivative will be an in-place destructive algorithm.
    // we'll make an array with [position, velocity, acceleration, jerk, snap, crackle, pop, lock, drop] where each entry is the value at latest time (which is
    // not necessarily current time)

    //Sets the finite difference accuracy
    int finite_diff_accuracy = 3; // 5 position points for accuracy of 3
                      // 4 position points for accuracy of 2
                      // 3 position points for accuracy of 1


    //The first index is for the most recent data set
    float order1CoEffs[3][4] = {{1,-1,0,0},
                            {1.5,-2,0.5,0},
                            {(11.0f/6.0f),(-3),(3.0f/2.0f),(-1.0f/3.0f)}};

    float order2CoEffs[3][5] = {{1,-2,1,0,0},
                            {2,-5,4,-1,0},
                            {(35.0f/12.0f),(-26.0f/3.0f),(19.0f/2.0f),(-14.0f/3.0f),(11.0f/12.0f)}};


    vector<enemyTimedPosition> validPoints;
    validPoints = this->getLastEntriesWithinTime(VALID_TIME);
    int size = validPoints.size();
    //DEBUG
    size_watch=size;

    finite_diff_accuracy = size - 2;
    if (finite_diff_accuracy > 3) {
        finite_diff_accuracy = 3;
    }
    if (finite_diff_accuracy < 1) {
        finite_diff_accuracy = 1;
    }

    // adjust for desired parameter in function call
    if (desired_finite_diff_accuracy < finite_diff_accuracy) {
        finite_diff_accuracy = desired_finite_diff_accuracy;
    }
    
    plateKinematicState enemyGuess;
    

    Vector3f guess_position = {0,0,0};
    Vector3f guess_velocity = {0,0,0};
    Vector3f guess_acceleration = {0,0,0};
    
    float dt_uS = 0;

    accuracy_watch=finite_diff_accuracy;
    
    if (size > 0) {
        //Position
        guess_position = validPoints[0].position;

        // check for Velocity (if-statement nested in Position if-statement)
        if (size > 1) {
            //Velocity
            switch (finite_diff_accuracy){
                case 1: // 2 points
                    dt_uS=(validPoints[0].timestamp_uS - validPoints[1].timestamp_uS) / (float)MICROSECONDS_PER_SECOND;
                    
                    //                   order1CoEffs[Accuracy index][coeff index]*validPoints[point index].position.getX()              
                    guess_velocity.set( (order1CoEffs[0][0]*validPoints[0].position.getX() + order1CoEffs[0][1]*validPoints[1].position.getX()) / (dt_uS),
                                        (order1CoEffs[0][0]*validPoints[0].position.getY() + order1CoEffs[0][1]*validPoints[1].position.getY()) / (dt_uS),
                                        (order1CoEffs[0][0]*validPoints[0].position.getZ() + order1CoEffs[0][1]*validPoints[1].position.getZ()) / (dt_uS));
                    watchVelX=(order1CoEffs[0][0]*validPoints[0].position.getX() + order1CoEffs[0][1]*validPoints[1].position.getX()) / (dt_uS);
                    break;

                case 2: // 3 points
                    dt_uS=(validPoints[0].timestamp_uS - validPoints[2].timestamp_uS) / 2.0f / (float)MICROSECONDS_PER_SECOND;
                    //DEBUG
                    dt_vel_watch=dt_uS;
                    guess_velocity.set( (order1CoEffs[1][0]*validPoints[0].position.getX() + order1CoEffs[1][1]*validPoints[1].position.getX() + order1CoEffs[1][2]*validPoints[2].position.getX()) / (dt_uS),
                                        (order1CoEffs[1][0]*validPoints[0].position.getY() + order1CoEffs[1][1]*validPoints[1].position.getY() + order1CoEffs[1][2]*validPoints[2].position.getY()) / (dt_uS),
                                        (order1CoEffs[1][0]*validPoints[0].position.getZ() + order1CoEffs[1][1]*validPoints[1].position.getZ() + order1CoEffs[1][2]*validPoints[2].position.getZ()) / (dt_uS));
                    watchVelX=(order1CoEffs[1][0]*validPoints[0].position.getX() + order1CoEffs[1][1]*validPoints[1].position.getX() + order1CoEffs[1][2]*validPoints[2].position.getX()) / (dt_uS);
                    break;
                case 3: // 4 points
                    dt_uS=(validPoints[0].timestamp_uS - validPoints[3].timestamp_uS) / 3.0f / (float)MICROSECONDS_PER_SECOND;
                    dt_vel_watch=dt_uS;
                    //                   order1CoEffs[Accuracy index][coeff index]*validPoints[point index].position.getX()              
                    guess_velocity.set( (order1CoEffs[2][0]*validPoints[0].position.getX() + order1CoEffs[2][1]*validPoints[1].position.getX() + order1CoEffs[2][2]*validPoints[2].position.getX() + order1CoEffs[2][3]*validPoints[3].position.getX())/ (dt_uS),
                                        (order1CoEffs[2][0]*validPoints[0].position.getY() + order1CoEffs[2][1]*validPoints[1].position.getY() + order1CoEffs[2][2]*validPoints[2].position.getY() + order1CoEffs[2][3]*validPoints[3].position.getY())/ (dt_uS),
                                        (order1CoEffs[2][0]*validPoints[0].position.getZ() + order1CoEffs[2][1]*validPoints[1].position.getZ() + order1CoEffs[2][2]*validPoints[2].position.getZ() + order1CoEffs[2][3]*validPoints[3].position.getZ())/ (dt_uS));
                    
                    break;
            }
            // check for Acceleration (if-statement nested in Position if-Acceleration)
            if (size > 2) { 
                //Acceleration

                switch(finite_diff_accuracy) {                
                    case 1: //3 points
                        dt_uS = (validPoints[0].timestamp_uS - validPoints[2].timestamp_uS) / 2.0f / (float)MICROSECONDS_PER_SECOND;
                        dt_uS *= dt_uS; //Squaring the value for acceleration
                        guess_acceleration.set( (order2CoEffs[0][0]*validPoints[0].position.getX() + order2CoEffs[0][1]*validPoints[1].position.getX() + order2CoEffs[0][2]*validPoints[2].position.getX())/dt_uS,
                                                (order2CoEffs[0][0]*validPoints[0].position.getY() + order2CoEffs[0][1]*validPoints[1].position.getY() + order2CoEffs[0][2]*validPoints[2].position.getY())/dt_uS,         
                                                (order2CoEffs[0][0]*validPoints[0].position.getZ() + order2CoEffs[0][1]*validPoints[1].position.getZ() + order2CoEffs[0][2]*validPoints[2].position.getZ())/dt_uS);
                        
                    break;
                    case 2: //4 points
                        dt_uS = (validPoints[0].timestamp_uS - validPoints[3].timestamp_uS) / 3.0f / (float)MICROSECONDS_PER_SECOND;
                        dt_uS *= dt_uS; //Squaring the value for acceleration
                        guess_acceleration.set( (order2CoEffs[1][0]*validPoints[0].position.getX() + order2CoEffs[1][1]*validPoints[1].position.getX() + order2CoEffs[1][2]*validPoints[2].position.getX() + order2CoEffs[1][3]*validPoints[3].position.getX())/dt_uS,
                                                (order2CoEffs[1][0]*validPoints[0].position.getY() + order2CoEffs[1][1]*validPoints[1].position.getY() + order2CoEffs[1][2]*validPoints[2].position.getY() + order2CoEffs[1][3]*validPoints[3].position.getY())/dt_uS,         
                                                (order2CoEffs[1][0]*validPoints[0].position.getZ() + order2CoEffs[1][1]*validPoints[1].position.getZ() + order2CoEffs[1][2]*validPoints[2].position.getZ() + order2CoEffs[1][3]*validPoints[3].position.getZ())/dt_uS);
                        
                    break;
                    case 3: //5 points
                        dt_uS = (validPoints[0].timestamp_uS - validPoints[4].timestamp_uS) / 4.0f / (float)MICROSECONDS_PER_SECOND;
                        dt_uS *= dt_uS; //Squaring the value for acceleration
                        guess_acceleration.set( (order2CoEffs[2][0]*validPoints[0].position.getX() + order2CoEffs[2][1]*validPoints[1].position.getX() + order2CoEffs[2][2]*validPoints[2].position.getX() + order2CoEffs[2][3]*validPoints[3].position.getX() + order2CoEffs[2][4]*validPoints[4].position.getX())/dt_uS,
                                                (order2CoEffs[2][0]*validPoints[0].position.getY() + order2CoEffs[2][1]*validPoints[1].position.getY() + order2CoEffs[2][2]*validPoints[2].position.getY() + order2CoEffs[2][3]*validPoints[3].position.getY() + order2CoEffs[2][4]*validPoints[4].position.getY())/dt_uS,         
                                                (order2CoEffs[2][0]*validPoints[0].position.getZ() + order2CoEffs[2][1]*validPoints[1].position.getZ() + order2CoEffs[2][2]*validPoints[2].position.getZ() + order2CoEffs[2][3]*validPoints[3].position.getZ() + order2CoEffs[2][4]*validPoints[4].position.getZ())/dt_uS);
                        
                    break;
                    
                }
                
            } // Acceleration if-statement
        }// Velocity if-statement
    } // Position if-statement

    enemyGuess.position = guess_position;
    enemyGuess.velocity = guess_velocity;
    enemyGuess.acceleration = guess_acceleration;
    enemyGuess.timestamp_uS = validPoints[0].timestamp_uS;

    return enemyGuess;

    


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
            val1.position = Vector3f(diff);
        }
        // save nth order derivative
        derivatives[n] = validPoints[0];
    }
    // do our approximation
    // latest data time to current time
    uint32_t dt_uS = tap::arch::clock::getTimeMicroseconds() - derivatives[0].timestamp_uS;
    Vector3f finalGuess_position;
    Vector3f finalGuess_velocity;
    Vector3f finalGuess_acceleration;

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
        Vector3f nth_thing = Vector3f(diff);
        finalGuess_position = finalGuess_position + nth_thing;
        // integrals for velocity
        if (n >= 2) {
            float velo_dt = pow(dt_uS, n - 1) / (the_number - 1);  // dt for velocity use
            diff[X_AXIS] = derivatives[n].position[X_AXIS][0] * velo_dt;
            diff[Y_AXIS] = derivatives[n].position[Y_AXIS][0] * velo_dt;
            diff[Z_AXIS] = derivatives[n].position[Z_AXIS][0] * velo_dt;
            nth_thing = Vector3f(diff);
            finalGuess_velocity = finalGuess_velocity + nth_thing;
        }
        if (n >= 3) {
            float accel_dt = pow(dt_uS, n - 2) / (the_number - 2);  // dt for acceleration use
            diff[X_AXIS] = derivatives[n].position[X_AXIS][0] * accel_dt;
            diff[Y_AXIS] = derivatives[n].position[Y_AXIS][0] * accel_dt;
            diff[Z_AXIS] = derivatives[n].position[Z_AXIS][0] * accel_dt;
            nth_thing = Vector3f(diff);
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