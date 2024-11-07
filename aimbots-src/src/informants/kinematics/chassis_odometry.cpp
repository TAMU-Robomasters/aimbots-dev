#include "chassis_odmetry.hpp"

#pragma once

#include <tap/algorithms/wrapped_float.hpp>

#include "tap/communication/sensors/imu/imu_interface.hpp"

#include "communicators/jetson/jetson_communicator.hpp"
#include "informants/odometry/chassis_kf_odometry.hpp"
#include "utils/kinematics/kinematic_state_vector.hpp"
#include "utils/tools/common_types.hpp"

#include "utils/tools/robot_specific_defines.hpp"
#include "subsystems/chassis/chassis_constants.hpp"

#include "robot_frames.hpp"
#include "turret_frames.hpp"

namespace src {
class Drivers;
}  // namespace src

namespace src::Chassis {
class ChassisSubsystem;
}  // namespace src::Chassis

using namespace src::Utils;

namespace src::Informants {

    void KinematicInformant::updateChassisAcceleration() {
        chassisAngleXDisplay = chassisAngularState[X_AXIS].getPosition();
        chassisAngleYDisplay = chassisAngularState[Y_AXIS].getPosition();
        chassisAngleZDisplay = chassisAngularState[Z_AXIS].getPosition();

        Vector3f linearIMUAcceleration = removeFalseAcceleration(imuLinearState, imuAngularState, IMU_MOUNT_POSITION);

        Vector3f linearChassisAcceleration =
            drivers->kinematicInformant.getRobotFrames()
                .getFrame(Transformers::FrameType::CHASSIS_IMU_FRAME)
                .getPointInFrame(
                    drivers->kinematicInformant.getRobotFrames().getFrame(Transformers::FrameType::CHASSIS_FRAME),
                    linearIMUAcceleration);

        linearIMUAccelerationDisplay = linearIMUAcceleration;
        linearIMUAccelerationXDisplay = linearChassisAcceleration.getX();
        linearIMUAccelerationYDisplay = linearChassisAcceleration.getY();
        linearIMUAccelerationZDisplay = linearChassisAcceleration.getZ();

        chassisLinearState[X_AXIS].updateFromAcceleration(linearChassisAcceleration.getX());
        chassisLinearState[Y_AXIS].updateFromAcceleration(linearChassisAcceleration.getY());
        chassisLinearState[Z_AXIS].updateFromAcceleration(linearChassisAcceleration.getZ());
    }

    Vector3f KinematicInformant::removeFalseAcceleration(
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



} //namespace Informants