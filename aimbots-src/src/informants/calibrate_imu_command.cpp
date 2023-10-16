#include "calibrate_imu_command.hpp"
#if defined(GIMBAL_COMPATIBLE) && defined(CHASSIS_COMPATIBLE)

namespace src::Informants {

using namespace tap::communication::sensors::imu::bmi088;

IMUCalibrateCommand::IMUCalibrateCommand(
    src::Drivers* drivers,
    src::Chassis::ChassisSubsystem* chassis,
    src::Gimbal::GimbalSubsystem* gimbal)
    : drivers(drivers),
      chassis(chassis),
      gimbal(gimbal),
      gimbalController(gimbal)
//
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(gimbal));
}

void IMUCalibrateCommand::initialize() {
    calibrationState = CalibrationState::WAITING_FOR_SYSTEMS_ONLINE;

    gimbalController.setTargetYaw(AngleUnit::Radians, 0.0f);
    gimbalController.setTargetPitch(AngleUnit::Radians, 0.0f);

    chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);

    maximumCalibrationTimer.stop();
}

float chassisVelX, chassisVelY, chassisVelZ = 0.0f;
void IMUCalibrateCommand::execute() {
    switch (calibrationState) {
        case CalibrationState::WAITING_FOR_SYSTEMS_ONLINE: {
            if (gimbal->isOnline()) {
                calibrationState = CalibrationState::LOCKING_CHASSIS;
                maximumCalibrationTimer.restart(MAXIMUM_CALIBRATION_TIME);
            }
            break;
        }

        case CalibrationState::LOCKING_CHASSIS: {
            Vector3f chassisVelocity = chassis->getActualVelocityChassisRelative();
            chassisVelX = chassisVelocity.getX();
            chassisVelY = chassisVelocity.getY();
            chassisVelZ = chassisVelocity.getZ();
            if (fabs(chassisVelocity.getX()) < 1E-2 && fabs(chassisVelocity.getY()) < 1E-2 &&
                fabs(chassisVelocity.getZ()) < 1E-2) {
                calibrationState = CalibrationState::LOCKING_TURRET;
            }
            break;
        }

        case CalibrationState::LOCKING_TURRET: {
            gimbalController.runYawController();
            gimbalController.runPitchController();

            if (gimbalController.allOnlineYawControllersSettled(modm::toRadian(1.0f), 1000) &&
                gimbalController.allOnlinePitchControllersSettled(modm::toRadian(1.0f), 1000)) {
#ifdef TURRET_IMU
                drivers->turretCommunicator.requestTurretIMUCalibrate();
#endif
#ifndef TARGET_TURRET
                drivers->kinematicInformant.recalibrateIMU(
                    {CIMU_CALIBRATION_EULER_X, CIMU_CALIBRATION_EULER_Y, CIMU_CALIBRATION_EULER_Z});
#endif
                calibrationState = CalibrationState::CALIBRATING_IMU;
            }
            break;
        }

        case CalibrationState::CALIBRATING_IMU: {
            if (drivers->kinematicInformant.getIMUState() == Bmi088::ImuState::IMU_CALIBRATED) {
                calibrationState = CalibrationState::WAITING_CALIBRATION_COMPLETE;
                extraCalibrationWaitTimer.restart(WAIT_EXTRA_TIME);
            }
            break;
        }

        case CalibrationState::WAITING_CALIBRATION_COMPLETE: {
            if (extraCalibrationWaitTimer.isExpired()) {
                calibrationState = CalibrationState::DONE;
            }
            break;
        }

        case CalibrationState::DONE:
            drivers->leds.set(tap::gpio::Leds::Green, true);
            break;
    }
}

void IMUCalibrateCommand::end(bool interrupted) {
    gimbal->setAllDesiredPitchMotorOutputs(0);
    gimbal->setAllDesiredYawMotorOutputs(0);

    chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);
}

bool IMUCalibrateCommand::isReady() { return true; }

bool IMUCalibrateCommand::isFinished() const {
    return calibrationState == CalibrationState::DONE || maximumCalibrationTimer.isExpired();
}

}  // namespace src::Informants

#endif