#include "enemy_data_conversion.hpp"

#include "drivers.hpp"

namespace src::Informants {
EnemyDataConversion::EnemyDataConversion(src::Drivers* drivers)
    : drivers(drivers),
      positionKalman(
          {ExtendedKalman(config.tQPositionKalman, config.tRPositionKalman),
           ExtendedKalman(config.tQPositionKalman, config.tRPositionKalman),
           ExtendedKalman(config.tQPositionKalman, config.tRPositionKalman)}),
      velocityKalman(
          {ExtendedKalman(config.tQVelocityKalman, config.tRVelocityKalman),
           ExtendedKalman(config.tQVelocityKalman, config.tRVelocityKalman),
           ExtendedKalman(config.tQVelocityKalman, config.tRVelocityKalman)}),
      accelKalman(
          {ExtendedKalman(config.tQAccelKalman, config.tRAccelKalman),
           ExtendedKalman(config.tQAccelKalman, config.tRAccelKalman),
           ExtendedKalman(config.tQAccelKalman, config.tRAccelKalman)}) {}

// watchable variables
float targetXCoordDisplay_camera = 0.0f;
float targetYCoordDisplay_camera = 0.0f;
float targetZCoordDisplay_camera = 0.0f;
//
Matrix<float, 3, 1> enemyPositionDisplay_gimbal;
float targetXCoordDisplay_gimbal = 0.0f;
float targetYCoordDisplay_gimbal = 0.0f;
float targetZCoordDisplay_gimbal = 0.0f;
//
float targetXCoordDisplay_chassis = 0.0f;
float targetYCoordDisplay_chassis = 0.0f;
float targetZCoordDisplay_chassis = 0.0f;
//
float gimbalYXWatch = 0.0f;
float gimbalYYWatch = 0.0f;
float gimbalYZWatch = 0.0f;
//
int size_watch;
float dt_vel_watch;
int accuracy_watch;
float watchVelX;
int buffer_size_watch;
float last_entry_timestamp_watch;

float px, py, pz = 0.0f;
float vx, vy, vz = 0.0f;
float ax, ay, az = 0.0f;

// gather data, transform data,
void EnemyDataConversion::updateEnemyInfo(Vector3f position, uint32_t frameCaptureDelay) {
    // clear buffer if CV connection just turned back on
    prev_cv_valid = cv_valid;
    cv_valid = true;
    if (cv_valid && !prev_cv_valid) {
        rawPositionBuffer.clear();
    }  // goodbye

    enemyTimedPosition currentData{
        .position = position,
        .timestamp_uS = static_cast<uint32_t>(tap::arch::clock::getTimeMicroseconds()) -
                        (static_cast<uint32_t>(frameCaptureDelay) * MICROSECONDS_PER_MS),
        // Current time - (how long ago the frame was captured)
    };

    // watchable variables
    targetXCoordDisplay_camera = currentData.position.getX();
    targetYCoordDisplay_camera = currentData.position.getY();
    targetZCoordDisplay_camera = currentData.position.getZ();

    enemyTimedPosition transformedData = currentData;
    // now that we have enemy position (in METERS), transform to chassis space ! ! !
    // THE DESIGN IS VERY HUMAN-CENTERED. THE ROBOT IS THE CENTER OF THE UNIVERSE. THE ENEMY IS THE CENTER OF THE ROBOT.

    // std::pair<float, float> gimbalAngles = gimbal->getGimbalOrientationAtTime(frameCaptureDelay);
    // drivers->kinematicInformant.mirrorPastRobotFrame(frameCaptureDelay);

    enemyTimedPosition gimbalTransformedDataWatch = currentData;
    gimbalTransformedDataWatch.position =
        drivers->kinematicInformant.getRobotFrames()
            .getFrame(Transformers::FrameType::CAMERA_FRAME)
            .getPointInFrame(
                drivers->kinematicInformant.getRobotFrames().getFrame(Transformers::FrameType::GIMBAL_FRAME),
                currentData.position);

    Matrix3f gimbalOrientationWatch =
        drivers->kinematicInformant.getRobotFrames().getFrame(Transformers::FrameType::GIMBAL_FRAME).getOrientation();

    gimbalYXWatch = gimbalOrientationWatch[0][1];
    gimbalYYWatch = gimbalOrientationWatch[1][1];
    gimbalYZWatch = gimbalOrientationWatch[2][1];

    transformedData.position =
        drivers->kinematicInformant.getRobotFrames()
            .getFrame(Transformers::FrameType::CAMERA_AT_CV_UPDATE_FRAME)
            .getPointInFrame(
                drivers->kinematicInformant.getRobotFrames().getFrame(Transformers::FrameType::BALLISTICS_FRAME),
                currentData.position);

    // save data point to buffer (at index 0-- index 0 is NEWEST, index size-1 is OLDEST.)
    // at max capacity, oldest data is overwritten first
    rawPositionBuffer.prependOverwrite(transformedData);

    buffer_size_watch = rawPositionBuffer.getSize();
    last_entry_timestamp_watch = rawPositionBuffer[0].timestamp_uS;

    // watchable variables
    targetXCoordDisplay_chassis = transformedData.position.getX();
    targetYCoordDisplay_chassis = transformedData.position.getY();
    targetZCoordDisplay_chassis = transformedData.position.getZ();

    targetXCoordDisplay_gimbal = gimbalTransformedDataWatch.position.getX();
    targetYCoordDisplay_gimbal = gimbalTransformedDataWatch.position.getY();
    targetZCoordDisplay_gimbal = gimbalTransformedDataWatch.position.getZ();

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
    std::vector<enemyTimedPosition> validPositions;
    uint32_t currentTime_uS = tap::arch::clock::getTimeMicroseconds();
    raw_x_display = rawPositionBuffer[0].position.getX();
    raw_time_display = rawPositionBuffer[0].timestamp_uS;
    raw_x_display2 = rawPositionBuffer[1].position.getX();
    // traverse bounded deque until invalid time found
    for (int index = 0; index < BUFFER_SIZE; index++) {
        enemyTimedPosition pos = rawPositionBuffer[index];
        canSendData = (currentTime_uS - pos.timestamp_uS < time_seconds * (float)MICROSECONDS_PER_SECOND);
        if ((pow(pos.position.getX(), 2) + pow(pos.position.getY(), 2) + pow(pos.position.getZ(), 2)) > 0) {
            validPositions.push_back(pos);
        } else {
            break;
        }
    }
    return validPositions;
}

// !!! note from future self: stop taking so many damn derivatives
// What are we doing here? even our latest position may be microseconds out of date :( so do some finite difference BS to get
// position at our CURRENT time. With n valid datapoints, we can only go up to the (n-1)th derivative. shame.
vision::plateKinematicState EnemyDataConversion::calculateBestGuess(int desired_finite_diff_accuracy) {
    // for the sake of memory, calculating to the nth order derivative will be an in-place destructive algorithm.
    // we'll make an array with [position, velocity, acceleration, jerk, snap, crackle, pop, lock, drop] where each entry is
    // the value at latest time (which is not necessarily current time)

    // Sets the finite difference accuracy
    int finite_diff_accuracy = 1;  // 5 position points for accuracy of 3
                                   // 4 position points for accuracy of 2
                                   // 3 position points for accuracy of 1

    // The first index is for the most recent data set
    float order1CoEffs[3][4] = {{1, -1, 0, 0}, {1.5, -2, 0.5, 0}, {(11.0f / 6.0f), (-3), (3.0f / 2.0f), (-1.0f / 3.0f)}};

    float order2CoEffs[3][5] = {
        {1, -2, 1, 0, 0},
        {2, -5, 4, -1, 0},
        {(35.0f / 12.0f), (-26.0f / 3.0f), (19.0f / 2.0f), (-14.0f / 3.0f), (11.0f / 12.0f)}};

    std::vector<enemyTimedPosition> validPoints;
    validPoints = this->getLastEntriesWithinTime(VALID_TIME);
    int size = validPoints.size();
    // DEBUG
    size_watch = size;

    finite_diff_accuracy = size - 2;
    finite_diff_accuracy = std::min(finite_diff_accuracy, 3);
    finite_diff_accuracy = std::max(finite_diff_accuracy, 1);

    // adjust for desired parameter in function call
    if (desired_finite_diff_accuracy < finite_diff_accuracy) {
        finite_diff_accuracy = desired_finite_diff_accuracy;
    }

    vision::plateKinematicState enemyGuess;

    Vector3f guess_position = {0, 0, 0};
    Vector3f guess_velocity = {0, 0, 0};
    Vector3f guess_acceleration = {0, 0, 0};

    float dt_uS = 0;

    accuracy_watch = finite_diff_accuracy;

    if (size > 0) {
        // Position .. unfiltered!!!
        // guess_position = validPoints[0].position;
        // Position .. filtered!!
        guess_position.setX(positionKalman[0].filterData(validPoints[0].position.getX()));
        guess_position.setY(positionKalman[1].filterData(validPoints[0].position.getY()));
        guess_position.setZ(positionKalman[2].filterData(validPoints[0].position.getZ()));

        // check for Velocity (if-statement nested in Position if-statement)
        if (size > 1) {
            // Velocity
            switch (finite_diff_accuracy) {
                case 1:  // 2 points
                    dt_uS = (validPoints[0].timestamp_uS - validPoints[1].timestamp_uS) / (float)MICROSECONDS_PER_SECOND;

                    //                   order1CoEffs[Accuracy index][coeff index]*validPoints[point index].position.getX()
                    guess_velocity.set(
                        (order1CoEffs[0][0] * validPoints[0].position.getX() +
                         order1CoEffs[0][1] * validPoints[1].position.getX()) /
                            (dt_uS),
                        (order1CoEffs[0][0] * validPoints[0].position.getY() +
                         order1CoEffs[0][1] * validPoints[1].position.getY()) /
                            (dt_uS),
                        (order1CoEffs[0][0] * validPoints[0].position.getZ() +
                         order1CoEffs[0][1] * validPoints[1].position.getZ()) /
                            (dt_uS));
                    watchVelX = (order1CoEffs[0][0] * validPoints[0].position.getX() +
                                 order1CoEffs[0][1] * validPoints[1].position.getX()) /
                                (dt_uS);
                    break;

                case 2:  // 3 points
                    dt_uS =
                        (validPoints[0].timestamp_uS - validPoints[2].timestamp_uS) / 2.0f / (float)MICROSECONDS_PER_SECOND;
                    // DEBUG
                    dt_vel_watch = dt_uS;
                    guess_velocity.set(
                        (order1CoEffs[1][0] * validPoints[0].position.getX() +
                         order1CoEffs[1][1] * validPoints[1].position.getX() +
                         order1CoEffs[1][2] * validPoints[2].position.getX()) /
                            (dt_uS),
                        (order1CoEffs[1][0] * validPoints[0].position.getY() +
                         order1CoEffs[1][1] * validPoints[1].position.getY() +
                         order1CoEffs[1][2] * validPoints[2].position.getY()) /
                            (dt_uS),
                        (order1CoEffs[1][0] * validPoints[0].position.getZ() +
                         order1CoEffs[1][1] * validPoints[1].position.getZ() +
                         order1CoEffs[1][2] * validPoints[2].position.getZ()) /
                            (dt_uS));
                    watchVelX = (order1CoEffs[1][0] * validPoints[0].position.getX() +
                                 order1CoEffs[1][1] * validPoints[1].position.getX() +
                                 order1CoEffs[1][2] * validPoints[2].position.getX()) /
                                (dt_uS);
                    break;
                case 3:  // 4 points
                    dt_uS =
                        (validPoints[0].timestamp_uS - validPoints[3].timestamp_uS) / 3.0f / (float)MICROSECONDS_PER_SECOND;
                    dt_vel_watch = dt_uS;
                    // order1CoEffs[Accuracy index][coeff index]*validPoints[point index].position.getX()
                    guess_velocity.set(
                        (order1CoEffs[2][0] * validPoints[0].position.getX() +
                         order1CoEffs[2][1] * validPoints[1].position.getX() +
                         order1CoEffs[2][2] * validPoints[2].position.getX() +
                         order1CoEffs[2][3] * validPoints[3].position.getX()) /
                            (dt_uS),
                        (order1CoEffs[2][0] * validPoints[0].position.getY() +
                         order1CoEffs[2][1] * validPoints[1].position.getY() +
                         order1CoEffs[2][2] * validPoints[2].position.getY() +
                         order1CoEffs[2][3] * validPoints[3].position.getY()) /
                            (dt_uS),
                        (order1CoEffs[2][0] * validPoints[0].position.getZ() +
                         order1CoEffs[2][1] * validPoints[1].position.getZ() +
                         order1CoEffs[2][2] * validPoints[2].position.getZ() +
                         order1CoEffs[2][3] * validPoints[3].position.getZ()) /
                            (dt_uS));

                    break;
            }
            // check for Acceleration (if-statement nested in Position if-Acceleration)
            if (size > 2) {
                // Acceleration

                switch (finite_diff_accuracy) {
                    case 1:  // 3 points
                        dt_uS = (validPoints[0].timestamp_uS - validPoints[2].timestamp_uS) / 2.0f /
                                (float)MICROSECONDS_PER_SECOND;
                        dt_uS *= dt_uS;  // Squaring the value for acceleration
                        guess_acceleration.set(
                            (order2CoEffs[0][0] * validPoints[0].position.getX() +
                             order2CoEffs[0][1] * validPoints[1].position.getX() +
                             order2CoEffs[0][2] * validPoints[2].position.getX()) /
                                dt_uS,
                            (order2CoEffs[0][0] * validPoints[0].position.getY() +
                             order2CoEffs[0][1] * validPoints[1].position.getY() +
                             order2CoEffs[0][2] * validPoints[2].position.getY()) /
                                dt_uS,
                            (order2CoEffs[0][0] * validPoints[0].position.getZ() +
                             order2CoEffs[0][1] * validPoints[1].position.getZ() +
                             order2CoEffs[0][2] * validPoints[2].position.getZ()) /
                                dt_uS);

                        break;
                    case 2:  // 4 points
                        dt_uS = (validPoints[0].timestamp_uS - validPoints[3].timestamp_uS) / 3.0f /
                                (float)MICROSECONDS_PER_SECOND;
                        dt_uS *= dt_uS;  // Squaring the value for acceleration
                        guess_acceleration.set(
                            (order2CoEffs[1][0] * validPoints[0].position.getX() +
                             order2CoEffs[1][1] * validPoints[1].position.getX() +
                             order2CoEffs[1][2] * validPoints[2].position.getX() +
                             order2CoEffs[1][3] * validPoints[3].position.getX()) /
                                dt_uS,
                            (order2CoEffs[1][0] * validPoints[0].position.getY() +
                             order2CoEffs[1][1] * validPoints[1].position.getY() +
                             order2CoEffs[1][2] * validPoints[2].position.getY() +
                             order2CoEffs[1][3] * validPoints[3].position.getY()) /
                                dt_uS,
                            (order2CoEffs[1][0] * validPoints[0].position.getZ() +
                             order2CoEffs[1][1] * validPoints[1].position.getZ() +
                             order2CoEffs[1][2] * validPoints[2].position.getZ() +
                             order2CoEffs[1][3] * validPoints[3].position.getZ()) /
                                dt_uS);

                        break;
                    case 3:  // 5 points
                        dt_uS = (validPoints[0].timestamp_uS - validPoints[4].timestamp_uS) / 4.0f /
                                (float)MICROSECONDS_PER_SECOND;
                        dt_uS *= dt_uS;  // Squaring the value for acceleration
                        guess_acceleration.set(
                            (order2CoEffs[2][0] * validPoints[0].position.getX() +
                             order2CoEffs[2][1] * validPoints[1].position.getX() +
                             order2CoEffs[2][2] * validPoints[2].position.getX() +
                             order2CoEffs[2][3] * validPoints[3].position.getX() +
                             order2CoEffs[2][4] * validPoints[4].position.getX()) /
                                dt_uS,
                            (order2CoEffs[2][0] * validPoints[0].position.getY() +
                             order2CoEffs[2][1] * validPoints[1].position.getY() +
                             order2CoEffs[2][2] * validPoints[2].position.getY() +
                             order2CoEffs[2][3] * validPoints[3].position.getY() +
                             order2CoEffs[2][4] * validPoints[4].position.getY()) /
                                dt_uS,
                            (order2CoEffs[2][0] * validPoints[0].position.getZ() +
                             order2CoEffs[2][1] * validPoints[1].position.getZ() +
                             order2CoEffs[2][2] * validPoints[2].position.getZ() +
                             order2CoEffs[2][3] * validPoints[3].position.getZ() +
                             order2CoEffs[2][4] * validPoints[4].position.getZ()) /
                                dt_uS);

                        break;
                }

            }  // Acceleration if-statement
        }      // Velocity if-statement
    }          // Position if-statement

    guess_velocity.setX(velocityKalman[0].filterData(guess_velocity.getX()));
    guess_velocity.setY(velocityKalman[1].filterData(guess_velocity.getY()));
    guess_velocity.setZ(velocityKalman[2].filterData(guess_velocity.getZ()));

    guess_acceleration.setX(accelKalman[0].filterData(guess_acceleration.getX()));
    guess_acceleration.setY(accelKalman[1].filterData(guess_acceleration.getY()));
    guess_acceleration.setZ(accelKalman[2].filterData(guess_acceleration.getZ()));

    enemyGuess.position = guess_position;
    enemyGuess.velocity = guess_velocity;
    enemyGuess.acceleration = guess_acceleration;
    enemyGuess.timestamp_uS = validPoints[0].timestamp_uS;

    px = guess_position.getX();
    py = guess_position.getY();
    pz = guess_position.getZ();

    vx = guess_velocity.getX();
    vy = guess_velocity.getY();
    vz = guess_velocity.getZ();

    ax = guess_acceleration.getX();
    ay = guess_acceleration.getY();
    az = guess_acceleration.getZ();

    return enemyGuess;
}

}  // namespace src::Informants