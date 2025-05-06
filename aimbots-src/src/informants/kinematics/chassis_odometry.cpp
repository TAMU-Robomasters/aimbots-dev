#include "informants/kinematics/chassis_odometry.hpp"

#include "subsystems/chassis/control/chassis.hpp"

#include "drivers.hpp"

namespace src::Informants {

ChassisOdometry::ChassisOdometry(src::Drivers* drivers) : drivers(drivers) {}

//#ifndef TARGET_TURRET
// chassisKFOdometry(-2.830f, -0.730f)
// chassisKFOdometry(3.1f, 3.5f);
//#warning "don't hardcode these values"
//#endif

Vector3f linearIMUAccelerationDisplay;
float linearIMUAccelerationXDisplay = 0.0f;
float linearIMUAccelerationYDisplay = 0.0f;
float linearIMUAccelerationZDisplay = 0.0f;

float chassisAngleXDisplay = 0.0f;
float chassisAngleYDisplay = 0.0f;
float chassisAngleZDisplay = 0.0f;

void ChassisOdometry::updateChassisAcceleration() {
    chassisAngleXDisplay = drivers->kinematicInformant.getIMUData()->getChassisAngularState()[X_AXIS].getPosition();
    chassisAngleYDisplay = drivers->kinematicInformant.getIMUData()->getChassisAngularState()[Y_AXIS].getPosition();
    chassisAngleZDisplay = drivers->kinematicInformant.getIMUData()->getChassisAngularState()[Z_AXIS].getPosition();

    Vector3f linearIMUAcceleration = removeFalseAcceleration(
        drivers->kinematicInformant.getIMUData()->getImuLinearState(),
        drivers->kinematicInformant.getIMUData()->getImuAngularState(),
        IMU_MOUNT_POSITION);

    Vector3f linearChassisAcceleration = drivers->kinematicInformant.getFieldGimbal()
                                             ->getRobotFrames()
                                             .getFrame(Transformers::FrameType::CHASSIS_IMU_FRAME)
                                             .getPointInFrame(
                                                 drivers->kinematicInformant.getFieldGimbal()->getRobotFrames().getFrame(
                                                     Transformers::FrameType::CHASSIS_FRAME),
                                                 linearIMUAcceleration);

    linearIMUAccelerationDisplay = linearIMUAcceleration;
    linearIMUAccelerationXDisplay = linearChassisAcceleration.getX();
    linearIMUAccelerationYDisplay = linearChassisAcceleration.getY();
    linearIMUAccelerationZDisplay = linearChassisAcceleration.getZ();

    drivers->kinematicInformant.getIMUData()->getChassisLinearState()[X_AXIS].updateFromAcceleration(
        linearChassisAcceleration.getX());
    drivers->kinematicInformant.getIMUData()->getChassisLinearState()[Y_AXIS].updateFromAcceleration(
        linearChassisAcceleration.getY());
    drivers->kinematicInformant.getIMUData()->getChassisLinearState()[Z_AXIS].updateFromAcceleration(
        linearChassisAcceleration.getZ());
}

Vector3f wDisplay = {0.0f, 0.0f, 0.0f};
Vector3f alphaDisplay = {0.0f, 0.0f, 0.0f};
Vector3f rDisplay = {0.0f, 0.0f, 0.0f};
Vector3f aDisplay = {0.0f, 0.0f, 0.0f};

float aXDisplay = 0.0f;
float aYDisplay = 0.0f;
float aZDisplay = 0.0f;

Vector3f ChassisOdometry::removeFalseAcceleration(
    Vector<KinematicStateVector, 3> imuLinearKSV,
    Vector<KinematicStateVector, 3> imuAngularKSV,
    Vector3f r) {
    Vector3f w = {
        imuAngularKSV[X_AXIS].getVelocity(),
        imuAngularKSV[Y_AXIS].getVelocity(),
        imuAngularKSV[Z_AXIS].getVelocity()};

    // Vector3f alpha = {
    //     imuAngularKSV[X_AXIS].getAcceleration(),
    //     imuAngularKSV[Y_AXIS].getAcceleration(),
    //     imuAngularKSV[Z_AXIS].getAcceleration()};
    // was broken so i made it 0, requires investigation on why it's broken (was setting stuff to infinity)

    Vector3f alpha = {0.0f, 0.0f, 0.0f};

    Vector3f a = {
        imuLinearKSV[X_AXIS].getAcceleration(),
        imuLinearKSV[Y_AXIS].getAcceleration(),
        imuLinearKSV[Z_AXIS].getAcceleration()};

    aXDisplay = a.getX();
    aYDisplay = a.getY();
    aZDisplay = a.getZ();

    wDisplay = w;
    alphaDisplay = alpha;
    rDisplay = r;
    aDisplay = a;

    Vector3f linearIMUAcceleration = a - (alpha ^ r) - (w ^ (w ^ r));
    return linearIMUAcceleration;
}

float robotLocationXDisplay = 0.0f;
float robotLocationYDisplay = 0.0f;

void ChassisOdometry::updateRobotFrames() {
    // Update IMU Stuff
    drivers->kinematicInformant.getIMUData()->updateIMUKinematicStateVector();

#ifndef TARGET_TURRET
    // Update Chassis Stuff after IMU STUFF
    drivers->kinematicInformant.getIMUData()->updateIMUAngles();
    drivers->kinematicInformant.getChassisOdometry()->updateChassisAcceleration();

    chassisIMUHistoryBuffer.prependOverwrite(
        {drivers->kinematicInformant.getIMUData()->getIMUAngle(PITCH_AXIS, AngleUnit::Radians),
         drivers->kinematicInformant.getIMUData()->getIMUAngle(ROLL_AXIS, AngleUnit::Radians),
         drivers->kinematicInformant.getIMUData()->getIMUAngle(YAW_AXIS, AngleUnit::Radians)});

    chassisKFOdometry->update(
        drivers->kinematicInformant.getIMUData()->getIMUAngle(YAW_AXIS, AngleUnit::Radians),
        drivers->kinematicInformant.getIMUData()->getChassisLinearState()[X_AXIS].getAcceleration(),
        drivers->kinematicInformant.getIMUData()->getChassisLinearState()[Y_AXIS].getAcceleration());

    modm::Location2D<float> robotLocation = chassisKFOdometry->getCurrentLocation2D();

    modm::Location2D<float> robotLocationDisplay;

    robotLocationXDisplay = robotLocation.getX();
    robotLocationYDisplay = robotLocation.getY();

    robotLocationDisplay = robotLocation;
#endif
}

}  // namespace src::Informants