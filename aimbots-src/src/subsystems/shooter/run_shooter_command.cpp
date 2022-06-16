#include "subsystems/shooter/run_shooter_command.hpp"

#include "drivers.hpp"
#include "tap/communication/gpio/leds.hpp"
#include "tap/control/subsystem.hpp"
#include "utils/common_types.hpp"
#include "utils/robot_constants.hpp"

//#ifndef TARGET_ENGINEER

namespace src::Shooter {

// These are the barrel speeds that the related rpms should
// produce.

// FIXME: Verify that these rpms can meet these barrel speeds
static constexpr uint16_t REF_17MM_INIT_BARREL_SPEED = 15;
static constexpr uint16_t REF_17MM_UPGRADE_1_BARREL_SPEED = 18;
static constexpr uint16_t REF_17MM_UPGRADE_2_BARREL_SPEED = 30;

static constexpr uint16_t REF_42MM_INIT_BARREL_SPEED = 10;
static constexpr uint16_t REF_42MM_UPGRADE_1_BARREL_SPEED = 10;
static constexpr uint16_t REF_42MM_UPGRADE_2_BARREL_SPEED = 16;

#warning "These numbers are completely guesses... They need to be verified and tuned per-robot. ( DM Richard if u get this lol :^) )"

static constexpr float FLYWHEEL_17MM_DEFAULT_RPM = 4000.0f;
static constexpr float FLYWHEEL_17MM_UPGRADE_1_RPM = 4600.0f;
static constexpr float FLYWHEEL_17MM_UPGRADE_2_RPM = 8500.0f;

static constexpr float FLYWHEEL_42MM_DEFAULT_RPM = 3000.0f;
static constexpr float FLYWHEEL_42MM_UPGRADE_1_RPM = 3000.0f;
static constexpr float FLYWHEEL_42MM_UPGRADE_2_RPM = 5000.0f;

RunShooterCommand::RunShooterCommand(src::Drivers* drivers, ShooterSubsystem* shooter) {
    this->drivers = drivers;
    this->shooter = shooter;
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(shooter));
}

void RunShooterCommand::initialize() {
    // No initialization needed
}

tap::communication::serial::RefSerialData::Rx::TurretData refSysRobotTurretDataDisplay;

void RunShooterCommand::execute() {
    // Slowest speed we have until we know for sure what it should be
    float flywheelSpeed = FLYWHEEL_42MM_DEFAULT_RPM;

    auto refSysRobotTurretData = drivers->refSerial.getRobotData().turret;
    refSysRobotTurretDataDisplay = refSysRobotTurretData;

    uint16_t speedLimit17mmID1 = refSysRobotTurretData.barrelSpeedLimit17ID1;
    uint16_t speedLimit17mmID2 = refSysRobotTurretData.barrelSpeedLimit17ID2;
    uint16_t speedLimit42mm = refSysRobotTurretData.barrelSpeedLimit42;

    using RefSerialRxData = tap::communication::serial::RefSerialData::Rx;

    // switch (refSysRobotTurretData.bulletType) {
    //     case RefSerialRxData::BulletType::AMMO_17: {
    //         auto mechID = refSysRobotTurretData.launchMechanismID;

    //         if (mechID == RefSerialRxData::MechanismID::TURRET_17MM_1) {
    //             if (speedLimit17mmID1 >= REF_17MM_INIT_BARREL_SPEED) {
    //                 flywheelSpeed = FLYWHEEL_17MM_DEFAULT_RPM;
    //             }

    //             if (speedLimit17mmID1 >= REF_17MM_UPGRADE_1_BARREL_SPEED) {
    //                 flywheelSpeed = FLYWHEEL_17MM_UPGRADE_1_RPM;
    //             }

    //             if (speedLimit17mmID1 >= REF_17MM_UPGRADE_2_BARREL_SPEED) {
    //                 flywheelSpeed = FLYWHEEL_17MM_UPGRADE_2_RPM;
    //             }
    //         } else if (mechID == RefSerialRxData::MechanismID::TURRET_17MM_2) {
    //             if (speedLimit17mmID2 >= REF_17MM_INIT_BARREL_SPEED) {
    //                 flywheelSpeed = FLYWHEEL_17MM_DEFAULT_RPM;
    //             }

    //             if (speedLimit17mmID2 >= REF_17MM_UPGRADE_1_BARREL_SPEED) {
    //                 flywheelSpeed = FLYWHEEL_17MM_UPGRADE_1_RPM;
    //             }

    //             if (speedLimit17mmID2 >= REF_17MM_UPGRADE_2_BARREL_SPEED) {
    //                 flywheelSpeed = FLYWHEEL_17MM_UPGRADE_2_RPM;
    //             }
    //         }
    //         break;
    //     }
    //     case RefSerialRxData::BulletType::AMMO_42: {
    //         if (speedLimit42mm >= REF_42MM_INIT_BARREL_SPEED) {
    //             flywheelSpeed = FLYWHEEL_42MM_DEFAULT_RPM;
    //         }

    //         if (speedLimit42mm >= REF_42MM_UPGRADE_1_BARREL_SPEED) {
    //             flywheelSpeed = FLYWHEEL_42MM_UPGRADE_1_RPM;
    //         }

    //         if (speedLimit42mm >= REF_42MM_UPGRADE_2_BARREL_SPEED) {
    //             flywheelSpeed = FLYWHEEL_42MM_UPGRADE_2_RPM;
    //         }
    //         break;
    //     }

    //     default:
    //         break;
    // }
    auto launcherID = refSysRobotTurretData.launchMechanismID;

    switch (launcherID) {
        case RefSerialRxData::MechanismID::TURRET_17MM_1: {
                if (speedLimit17mmID1 >= REF_17MM_INIT_BARREL_SPEED) {
                    flywheelSpeed = FLYWHEEL_17MM_DEFAULT_RPM;
                }

                if (speedLimit17mmID1 >= REF_17MM_UPGRADE_1_BARREL_SPEED) {
                    flywheelSpeed = FLYWHEEL_17MM_UPGRADE_1_RPM;
                }

                if (speedLimit17mmID1 >= REF_17MM_UPGRADE_2_BARREL_SPEED) {
                    flywheelSpeed = FLYWHEEL_17MM_UPGRADE_2_RPM;
                }
            break;
        }

        case RefSerialRxData::MechanismID::TURRET_17MM_2: {
                if (speedLimit17mmID1 >= REF_17MM_INIT_BARREL_SPEED) {
                    flywheelSpeed = FLYWHEEL_17MM_DEFAULT_RPM;
                }

                if (speedLimit17mmID1 >= REF_17MM_UPGRADE_1_BARREL_SPEED) {
                    flywheelSpeed = FLYWHEEL_17MM_UPGRADE_1_RPM;
                }

                if (speedLimit17mmID1 >= REF_17MM_UPGRADE_2_BARREL_SPEED) {
                    flywheelSpeed = FLYWHEEL_17MM_UPGRADE_2_RPM;
                }
            break;
        }

        case RefSerialRxData::MechanismID::TURRET_42MM: {
            if (speedLimit42mm >= REF_42MM_INIT_BARREL_SPEED) {
                flywheelSpeed = FLYWHEEL_42MM_DEFAULT_RPM;
            }

            if (speedLimit42mm >= REF_42MM_UPGRADE_1_BARREL_SPEED) {
                flywheelSpeed = FLYWHEEL_42MM_UPGRADE_1_RPM;
            }

            if (speedLimit42mm >= REF_42MM_UPGRADE_2_BARREL_SPEED) {
                flywheelSpeed = FLYWHEEL_42MM_UPGRADE_2_RPM;
            }
            break;
        }

        default:
            break;
    }

    shooter->ForAllShooterMotors(&ShooterSubsystem::setTargetRPM, flywheelSpeed);

    shooter->ForAllShooterMotors(&ShooterSubsystem::updateMotorVelocityPID);
}

void RunShooterCommand::end(bool) {
    // No cleanup needed
}

bool RunShooterCommand::isReady() {
    return true;
}

bool RunShooterCommand::isFinished() const {
    return false;
}
}  // namespace src::Shooter