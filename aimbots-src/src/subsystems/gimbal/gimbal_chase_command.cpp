#include "gimbal_chase_command.hpp"

namespace src::Gimbal {
// feed chassis relative controller for sentry, field relative for ground robots
GimbalChaseCommand::GimbalChaseCommand(src::Drivers* drivers,
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

float fieldRelativeYawAngleDisplay = 0;
float chassisRelativePitchAngleDisplay = 0;

float posXDisplay_guess = 0;
float posYDisplay_guess = 0;
float posZDisplay_guess = 0;

float veloXDisplay_guess = 0;
float veloYDisplay_guess = 0;
float veloZDisplay_guess = 0;


float accelXDisplay_guess = 0;
float accelYDisplay_guess = 0;
float accelZDisplay_guess = 0;

src::Informants::vision::CVState cvStateDisplay = src::Informants::vision::CVState::FOUND;

bool jetsonOnlineDisplay = false;

void GimbalChaseCommand::execute() {
    float targetYawAngle = 0.0f;
    float targetPitchAngle = 0.0f;

    Matrix<float, 1, 2> visionTargetAngles = Matrix<float, 1, 2>::zeroMatrix();
    src::Informants::vision::CVState cvState;

    jetsonOnlineDisplay = false;

    //remove the "|| true" for later

    if (drivers->cvCommunicator.isJetsonOnline() || true) {
        jetsonOnlineDisplay = true;
        //cvState = drivers->cvCommunicator.getLastValidMessage().cvState;
        if (cvState == src::Informants::vision::CVState::FOUND || true) {
            //visionTargetAngles = drivers->cvCommunicator.getVisionTargetAngles();

            cvStateDisplay = cvState;


            data = drivers->enemyDataConverter.calculateBestGuess();
            //debug
            posXDisplay_guess = data.position[X_AXIS][0];
            posYDisplay_guess = data.position[Y_AXIS][0];
            posZDisplay_guess = data.position[Z_AXIS][0];
            
            veloXDisplay_guess = data.velocity[X_AXIS][0];
            veloYDisplay_guess = data.velocity[Y_AXIS][0];
            veloZDisplay_guess = data.velocity[Z_AXIS][0];
            
            accelXDisplay_guess = data.acceleration[X_AXIS][0];
            accelYDisplay_guess = data.acceleration[X_AXIS][0];
            accelZDisplay_guess = data.acceleration[X_AXIS][0];

            aimAtAngles = gimbal->getAimAngles(data);


            //aimAtAngles = gimbal->aimAtPoint(0,1,0);

            //targetYawAngle = modm::toDegree(visionTargetAngles[0][src::Informants::vision::yaw]);
            //targetPitchAngle = modm::toDegree(visionTargetAngles[0][src::Informants::vision::pitch]);
            //targetYawAngle = aimAtAngles.yaw;
            //targetPitchAngle = aimAtAngles.pitch;

            //yawOffsetDisplay = modm::toDegree(drivers->cvCommunicator.getLastValidMessage().targetYawOffset);
            //pitchOffsetDisplay = modm::toDegree(drivers->cvCommunicator.getLastValidMessage().targetPitchOffset);

            //fieldRelativeYawAngleDisplay = gimbal->getCurrentFieldRelativeYawAngle(AngleUnit::Degrees);
            //chassisRelativePitchAngleDisplay = gimbal->getCurrentChassisRelativePitchAngle(AngleUnit::Degrees);
            fieldRelativeYawAngleDisplay = gimbal->getCurrentYawAngleFromChassisCenter(AngleUnit::Degrees);
            chassisRelativePitchAngleDisplay = gimbal->getCurrentPitchAngleFromChassisCenter(AngleUnit::Degrees);

            targetYawAngleDisplay2 = aimAtAngles.yaw * (180.0f / M_PI);
            targetPitchAngleDisplay2 = aimAtAngles.pitch * (180.0f / M_PI);
        }

        //Should be absolute angle, will double check

        /*
        controller->runYawController(AngleUnit::Degrees, targetYawAngle, true);
        controller->runPitchController(AngleUnit::Degrees, targetPitchAngle, true);
        */
    }
}

bool GimbalChaseCommand::isReady() { return true; }

bool GimbalChaseCommand::isFinished() const { return false; }

void GimbalChaseCommand::end(bool) {
    gimbal->setYawMotorOutput(0);
    gimbal->setPitchMotorOutput(0);
}
};  // namespace src::Gimbal