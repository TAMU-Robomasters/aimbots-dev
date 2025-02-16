#include "kinematic_informant.hpp"

#include "subsystems/gimbal/control/gimbal.hpp"
#include "subsystems/chassis/control/chassis.hpp"

#include "drivers.hpp"

namespace src::Informants {

KinematicInformant::KinematicInformant(src::Drivers* drivers)
    :   drivers(drivers),
        fieldRelativeGimbal(drivers),
        chassisOdometry(drivers),
        imuData(drivers),    
        hitTracker(drivers) {}

void KinematicInformant::initialize(float imuFrequency, float imukP, float imukI) {
    drivers->bmi088.initialize(imuFrequency, imukP, imukI);
}

#ifdef CHASSIS_COMPATIBLE
    #ifdef GIMBAL_COMPATIBLE
        void KinematicInformant::registerSubsystems(
            src::Gimbal::GimbalSubsystem* gimbalSubsystem,
            src::Chassis::ChassisSubsystem* chassisSubsystem) {
            this->gimbalSubsystem = gimbalSubsystem;
            this->chassisSubsystem = chassisSubsystem;

            fieldRelativeGimbal.registerSubsystems(gimbalSubsystem, chassisSubsystem);
        }
    #endif
#endif

#ifdef CHASSIS_COMPATIBLE 
    void KinematicInformant::registerSubsystems(
        src::Chassis::ChassisSubsystem* chassisSubsystem) {
        this->chassisSubsystem = chassisSubsystem;

        chassisOdometry.registerSubsystems(chassisSubsystem);
    } 
#endif

#ifdef GIMBAL_COMPATIBLE
    void KinematicInformant::registerSubsystems(
        src::Gimbal::GimbalSubsystem* gimbalSubsystem) {
        this->gimbalSubsystem = gimbalSubsystem;

        hitTracker.registerSubsystems(gimbalSubsystem);
    }
#endif

#ifdef CHASSIS_COMPATIBLE
    #ifdef GIMBAL_COMPATIBLE
        void KinematicInformant::updateData() {
            fieldRelativeGimbal.updateRobotFrames();
            imuData.updateIMUKinematicStateVector();
            imuData.updateIMUAngles();
        }
    #endif
#endif

#ifdef CHASSIS_COMPATIBLE 
    #ifndef GIMBAL_COMPATIBLE
        void KinematicInformant::updateData() {
            chassisOdometry.updateRobotFrames();
            chassisOdometry.updateChassisAcceleration();
            imuData.updateIMUKinematicStateVector();
            imuData.updateIMUAngles();
        }
    #endif
#endif

#ifdef GIMBAL_COMPATIBLE
    #ifndef CHASSIS_COMPATIBLE
    void KinematicInformant::updataData() {
        imuData.updateIMUKinematicStateVector();
        imuData.updateIMUAngles();
    }
    #endif
#endif

}  // namespace src::Informants