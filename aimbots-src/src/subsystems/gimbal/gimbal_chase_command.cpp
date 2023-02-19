#include "gimbal_chase_command.hpp"

namespace src::Gimbal {
// feed chassis relative controller for sentry, field relative for ground robots
GimbalChaseCommand::GimbalChaseCommand(
    src::Drivers* drivers,
    GimbalSubsystem* gimbalSubsystem,
    GimbalControllerInterface* gimbalController)
    : tap::control::Command(),
      drivers(drivers),
      gimbal(gimbalSubsystem),
      controller(gimbalController)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(gimbal));
}

void GimbalChaseCommand::initialize() {}

float targetPitchAngleDisplay2 = 0.0f;
float targetYawAngleDisplay2 = 0.0f;

float yawOffsetDisplay = 0.0f;
float pitchOffsetDisplay = 0.0f;

float chassisRelativeYawAngleDisplay = 0;
float chassisRelativePitchAngleDisplay = 0;

float posXDisplay_guess;
float posYDisplay_guess;
float posZDisplay_guess;

float veloXDisplay_guess;
float veloYDisplay_guess;
float veloZDisplay_guess;

float accelXDisplay_guess;
float accelYDisplay_guess;
float accelZDisplay_guess;

float timestampDisplay;

src::Informants::vision::CVState cvStateDisplay = src::Informants::vision::CVState::FOUND;

bool jetsonOnlineDisplay = false;

void GimbalChaseCommand::execute() {
    float targetYawAngle = 0.0f;
    float targetPitchAngle = 0.0f;

    Matrix<float, 1, 2> visionTargetAngles = Matrix<float, 1, 2>::zeroMatrix();
    src::Informants::vision::CVState cvState;

    jetsonOnlineDisplay = false;

    // remove the "|| true" for later
    if (drivers->cvCommunicator.isJetsonOnline()) {
        jetsonOnlineDisplay = true;
        cvState = drivers->cvCommunicator.getLastValidMessage().cvState;
        if (cvState == src::Informants::vision::CVState::FOUND) {
            // visionTargetAngles = drivers->cvCommunicator.getVisionTargetAngles();

            cvStateDisplay = cvState;

            //data = drivers->enemyDataConverter.calculateBestGuess(2); // passing 1 as desired_finite_diff_accuracy
            data = drivers->cvCommunicator.getPlateData();
            // debug
            posXDisplay_guess = data.position.getX();
            posYDisplay_guess = data.position.getY();
            posZDisplay_guess = data.position.getZ();

            veloXDisplay_guess = data.velocity.getX();
            veloYDisplay_guess = data.velocity.getY();
            veloZDisplay_guess = data.velocity.getZ();

            accelXDisplay_guess = data.acceleration.getX();
            accelYDisplay_guess = data.acceleration.getY();
            accelZDisplay_guess = data.acceleration.getZ();

            timestampDisplay = data.timestamp_uS;

            ballistics::MeasuredKinematicState targetKinematicState = {
                .position = data.position,
                .velocity = data.velocity,
                .acceleration = data.acceleration,
            };
            
            int64_t forwardProjectionTime = static_cast<int64_t>(data.timestamp_uS) - static_cast<int64_t>(tap::arch::clock::getTimeMicroseconds());

            targetKinematicState.position = targetKinematicState.projectForward(forwardProjectionTime / MICROSECONDS_PER_SECOND);

            float timeOfFlight = 0.0f;

            if (!ballistics::findTargetProjectileIntersection(
                    targetKinematicState,
                    30.0f,
                    3,
                    &targetPitchAngle,
                    &targetYawAngle,
                    &timeOfFlight)) {
                // unable to find intersection
            }

            targetYawAngle = M_PI_2 + M_PI_4 - targetYawAngle;
            targetPitchAngle += M_PI_4;

            targetYawAngleDisplay2 = modm::toDegree(targetYawAngle);
            targetPitchAngleDisplay2 = modm::toDegree(targetPitchAngle);

            // last

            // targetYawAngle = modm::toDegree(visionTargetAngles[0][src::Informants::vision::yaw]);
            // targetPitchAngle = modm::toDegree(visionTargetAngles[0][src::Informants::vision::pitch]);
            // targetYawAngle = aimAtAngles.yaw;
            // targetPitchAngle = aimAtAngles.pitch;

            // yawOffsetDisplay = modm::toDegree(drivers->cvCommunicator.getLastValidMessage().targetYawOffset);
            // pitchOffsetDisplay = modm::toDegree(drivers->cvCommunicator.getLastValidMessage().targetPitchOffset);

            // fieldRelativeYawAngleDisplay = gimbal->getCurrentFieldRelativeYawAngle(AngleUnit::Degrees);
            chassisRelativePitchAngleDisplay = gimbal->getCurrentChassisRelativePitchAngle(AngleUnit::Degrees);
            //fieldRelativeYawAngleDisplay = gimbal->getCurrentYawAngleFromChassisCenter(AngleUnit::Degrees);
            chassisRelativeYawAngleDisplay = gimbal->getCurrentYawAngleFromChassisCenter(AngleUnit::Degrees);

            //targetYawAngleDisplay2 = aimAtAngles.yaw * (180.0f / M_PI);
            //targetPitchAngleDisplay2 = aimAtAngles.pitch * (180.0f / M_PI);
        }

        // Should be absolute angle, will double check

        controller->runYawController(AngleUnit::Radians, targetYawAngle, false);
        controller->runPitchController(AngleUnit::Radians, targetPitchAngle, false);
    }
}

bool GimbalChaseCommand::isReady() { return true; }

bool GimbalChaseCommand::isFinished() const { return false; }

void GimbalChaseCommand::end(bool) {
    gimbal->setYawMotorOutput(0);
    gimbal->setPitchMotorOutput(0);
}
};  // namespace src::Gimbal