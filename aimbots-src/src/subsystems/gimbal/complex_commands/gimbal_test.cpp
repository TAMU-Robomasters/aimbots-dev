/*TODO: 
    - test how large the pitch and yaw amplitude can be*/
#include <cmath>
#include "gimbal_test.hpp"
#include "utils/tools/common_types.hpp"
#include "informants/kinematics/coordinate_frame.hpp"
#include "communicators/jetson/jetson_communicator.hpp"

#ifdef GIMBAL_COMPATIBLE
namespace src::Gimbal{
GimbalTestCommand::GimbalTestCommand(
        src::Drivers* drivers, 
        GimbalSubsystem* gimbalSubsystem, 
        GimbalFieldRelativeController* controller, 
        GimbalTestConfig config)
        : drivers(drivers), 
          gimbal(gimbalSubsystem), 
          controller(controller), 
          config(config)
        {
            // ? what does this actually do
            addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(gimbal));
        }  

// Display variables
float currGimbalTestTargetYawAngleDisplay = 0.0f;
float currGimbalTestTargetPitchAngleDisplay = 0.0f;
Matrix4f camToTurretTranformationMatrix = modm::Matrix4f::zeroMatrix();
float camToTurretTranformationDisplay[NUM_TRANSFORM_ELEMENTS] = {0};
float *camToTurretPtr = nullptr;

void GimbalTestCommand::execute() {
    if (!drivers->cvCommunicator.isJetsonOnline()) return;

    // start time when Jetson first comes online
    if (!wasJetsonOnline) {
        wasJetsonOnline = true;
        resetInitTime();
    }

    float yawTargetAngle = getYawTargetAngle();
    float pitchTargetAngle = getPitchTargetAngle();

    currGimbalTestTargetYawAngleDisplay = modm::toDegree(yawTargetAngle);
    currGimbalTestTargetPitchAngleDisplay = modm::toDegree(pitchTargetAngle);

    controller->setTargetYaw(AngleUnit::Radians, yawTargetAngle);
    controller->setTargetPitch(AngleUnit::Radians, pitchTargetAngle);

    controller->runYawController();
    controller->runPitchController();

    // Display current transformation from camera reference frame to turret reference frame
    drivers->kinematicInformant.updateRobotFrames();
    src::Informants::Transformers::CoordinateFrame turretFieldFrame =
        drivers->kinematicInformant.getTurretFrames().getFrame(src::Informants::Transformers::TurretFrameType::TURRET_FIELD_FRAME);

    src::Informants::Transformers::CoordinateFrame turretCameraFrame =
        drivers->kinematicInformant.getTurretFrames().getFrame(src::Informants::Transformers::TurretFrameType::TURRET_CAMERA_FRAME);
    camToTurretTranformationMatrix = turretCameraFrame.getTransformToFrame(turretFieldFrame);
    camToTurretPtr = camToTurretTranformationMatrix.element;
    for (int_fast8_t i = 0; i < NUM_TRANSFORM_ELEMENTS; i++) {
        camToTurretTranformationDisplay[i] = camToTurretPtr[i]; 
    }
    camToTurretPtr = nullptr; 
}

float GimbalTestCommand::getYawTargetAngle() {
    float yawTargetAngle = modm::toRadian(config.yawAmplitudeDegree) * cos(2*M_PI*getRelativeTime()/1000) 
                         + modm::toRadian(config.yawOffsetDegree);
    return yawTargetAngle;
}

float GimbalTestCommand::getPitchTargetAngle() {
    float yawTargetAngle = modm::toRadian(config.pitchAmplitudeDegree) * sin(2*M_PI*getRelativeTime()/1000) 
                         + modm::toRadian(config.pitchOffsetDegree);
    return yawTargetAngle;
}

void GimbalTestCommand::initialize() { initTime = tap::arch::clock::getTimeMilliseconds(); }

bool GimbalTestCommand::isReady() { return true; }

bool GimbalTestCommand::isFinished() const { return false; }

void GimbalTestCommand::end(bool) {
    gimbal->setAllDesiredYawMotorOutputs(0);
    gimbal->setAllDesiredPitchMotorOutputs(0);
}

} // namespace src::Gimbal
#endif // #ifdef GIMBAL_COMPATIBLE
