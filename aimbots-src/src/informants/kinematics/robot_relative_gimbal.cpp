#include "robot_relative_gimbal.hpp"
#include "kinematic_informant.hpp"

#include "tap/algorithms/math_user_utils.hpp"


#include "subsystems/gimbal/control/gimbal.hpp"
#include "utils/tools/common_types.hpp"


#include "drivers.hpp"

namespace src::Informants {
    
RobotRelativeGimbal::RobotRelativeGimbal(src::Drivers* drivers) : drivers(drivers) // rewrite to go to kinematic informants instead of drivers

    void updateRobotFrames(); //PUT THIS BACK
    // This updates more than just the robot frames, so will also update turret frames
    

    void KinematicInformant::updateRobotFrames() {
        // Update IMU Stuff
        updateIMUKinematicStateVector();

    #ifndef TARGET_TURRET
        // Update Chassis Stuff after IMU STUFF
        updateIMUAngles();
        updateChassisAcceleration();

        chassisIMUHistoryBuffer.prependOverwrite(
            {getIMUAngle(PITCH_AXIS, AngleUnit::Radians),
            getIMUAngle(ROLL_AXIS, AngleUnit::Radians),
            getIMUAngle(YAW_AXIS, AngleUnit::Radians)});

        chassisKFOdometry.update(
            getIMUAngle(YAW_AXIS, AngleUnit::Radians),
            chassisLinearState[X_AXIS].getAcceleration(),
            chassisLinearState[Y_AXIS].getAcceleration());

        // update gimbal orientation buffer
        std::pair<float, float> orientation;
        orientation.first = getCurrentFieldRelativeGimbalYawAngleAsWrappedFloat().getWrappedValue();
        orientation.second = getCurrentFieldRelativeGimbalPitchAngleAsWrappedFloat().getWrappedValue();

        gimbalFieldOrientationBuffer.prependOverwrite(orientation);

        modm::Location2D<float> robotLocation = chassisKFOdometry.getCurrentLocation2D();

        modm::Location2D<float> robotLocationDisplay;
        float robotLocationXDisplay = 0.0f;
        float robotLocationYDisplay = 0.0f;

        robotLocationXDisplay = robotLocation.getX();
        robotLocationYDisplay = robotLocation.getY();

        robotFrames.updateFrames(
            gimbalSubsystem->getCurrentYawAxisAngle(AngleUnit::Radians),
            gimbalSubsystem->getCurrentPitchAxisAngle(AngleUnit::Radians),
            getIMUAngle(YAW_AXIS, AngleUnit::Radians) + CHASSIS_START_ANGLE_WORLD,
            {robotLocation.getX(), robotLocation.getY(), 0},
            AngleUnit::Radians);

        turretFrames.updateFrames(
            getIMUAngle(YAW_AXIS, AngleUnit::Radians),
            getIMUAngle(PITCH_AXIS, AngleUnit::Radians),
            getIMUAngle(ROLL_AXIS, AngleUnit::Radians),
            AngleUnit::Radians);

        robotLocationDisplay = robotLocation;
    #endif
    }
} //namespace Informants