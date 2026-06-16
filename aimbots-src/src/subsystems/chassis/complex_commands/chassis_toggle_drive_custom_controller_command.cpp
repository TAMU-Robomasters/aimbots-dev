#include "chassis_toggle_drive_custom_controller_command.hpp"

#ifdef CHASSIS_COMPATIBLE

#include <cmath>

namespace src::Chassis {

// Debug/watch variables for ozone
bool customToggleTokyoScheduledDisplay = false;
bool customToggleIgnoreScheduledDisplay = false;
bool customToggleFEnabledDisplay = false;
bool customToggleButton1Display = false;
bool customToggleButton2Display = false;
bool customToggleButton4Display = false;
float customToggleManualSpinDisplay = 0.0f;
float customToggleMaxWheelSpeedDisplay = 0.0f;
int customToggleModeDisplay = 0;
int customTogglePreferredDirectionDisplay = 0;

ChassisToggleDriveCustomControllerCommand::ChassisToggleDriveCustomControllerCommand(
    src::Drivers* drivers,
    ChassisSubsystem* chassis,
    Gimbal::GimbalSubsystem* gimbal,
    const TokyoConfig& tokyoConfig,
    bool randomizeSpinRate,
    const SpinRandomizerConfig& randomizerConfig,
    float normalMaxWheelSpeed,
    float highMaxWheelSpeed)
    : TapComprisedCommand(drivers),
      drivers(drivers),
      chassis(chassis),
      ignoreGimbalCommand(drivers, chassis, gimbal),
      tokyoMasterCommand(
          drivers,
          chassis,
          gimbal,
          tokyoConfig,
          0,
          randomizeSpinRate,
          randomizerConfig,
          ChassisTokyoMasterMode::NORMAL,
          0.0f,
          normalMaxWheelSpeed),
      normalMaxWheelSpeed(normalMaxWheelSpeed),
      highMaxWheelSpeed(highMaxWheelSpeed) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
    comprisedCommandScheduler.registerSubsystem(dynamic_cast<tap::control::Subsystem*>(chassis));
}

void ChassisToggleDriveCustomControllerCommand::initialize() {
    tokyoMasterEnabledByF = false;
    wasFPressed = false;
    preferredSpinDirection = 0;
    qPressed.restart(0);
    ePressed.restart(0);
    scheduleIgnoreGimbal(false);
}

float ChassisToggleDriveCustomControllerCommand::applyDeadband(float value, float deadband) const {
    return std::fabs(value) < deadband ? 0.0f : value;
}

void ChassisToggleDriveCustomControllerCommand::scheduleIgnoreGimbal(bool interrupted) {
    descheduleIfScheduled(this->comprisedCommandScheduler, &tokyoMasterCommand, interrupted);
    scheduleIfNotScheduled(this->comprisedCommandScheduler, &ignoreGimbalCommand);
}

void ChassisToggleDriveCustomControllerCommand::scheduleTokyoMaster() {
    descheduleIfScheduled(this->comprisedCommandScheduler, &ignoreGimbalCommand, true);
    scheduleIfNotScheduled(this->comprisedCommandScheduler, &tokyoMasterCommand);
}

void ChassisToggleDriveCustomControllerCommand::execute() {
    if (drivers->remote.keyPressed(Remote::Key::F)) {
        wasFPressed = true;
    }

    if (drivers->remote.keyPressed(Remote::Key::E)) {
        ePressed.restart(800);
    }
    if (drivers->remote.keyPressed(Remote::Key::Q)) {
        qPressed.restart(800);
    }

    if (wasFPressed && !drivers->remote.keyPressed(Remote::Key::F)) {
        wasFPressed = false;
        tokyoMasterEnabledByF = !tokyoMasterEnabledByF;

        if (!ePressed.isExpired()) {
            preferredSpinDirection = 1;
        } else if (!qPressed.isExpired()) {
            preferredSpinDirection = -1;
        } else {
            // 0 = random spin direction
            preferredSpinDirection = 0;
        }
    }

    const bool customControllerConnected = drivers->controlOperatorInterface.isCustomControllerConnected();
    const bool button1 = customControllerConnected && drivers->controlOperatorInterface.customControllerButton1Pressed();
    const bool button2 = customControllerConnected && drivers->controlOperatorInterface.customControllerButton2Pressed();
    const bool button4 = customControllerConnected && drivers->controlOperatorInterface.customControllerButton4Pressed();

    // Button 2 sinusodal mode takes priority over button 1 normal mode. If neither is active but F enabled Tokyo Master,
    // the command uses normal constant spin
    const ChassisTokyoMasterMode mode = button2 ? ChassisTokyoMasterMode::SINUSODAL : ChassisTokyoMasterMode::NORMAL;
    const bool shouldRunTokyoMaster = tokyoMasterEnabledByF || button1 || button2;

    const float manualSpin = customControllerConnected
                                 ? applyDeadband(
                                       drivers->controlOperatorInterface.getCustomControllerManualSpinInput(),
                                       MANUAL_SPIN_DEADBAND)
                                 : 0.0f;

    const float maxWheelSpeed = button4 ? highMaxWheelSpeed : normalMaxWheelSpeed;

    tokyoMasterCommand.configure(mode, manualSpin, maxWheelSpeed, preferredSpinDirection);

    if (shouldRunTokyoMaster) {
        scheduleTokyoMaster();
    } else {
        scheduleIgnoreGimbal(true);
    }

    customToggleTokyoScheduledDisplay = comprisedCommandScheduler.isCommandScheduled(&tokyoMasterCommand);
    customToggleIgnoreScheduledDisplay = comprisedCommandScheduler.isCommandScheduled(&ignoreGimbalCommand);
    customToggleFEnabledDisplay = tokyoMasterEnabledByF;
    customToggleButton1Display = button1;
    customToggleButton2Display = button2;
    customToggleButton4Display = button4;
    customToggleManualSpinDisplay = manualSpin;
    customToggleMaxWheelSpeedDisplay = maxWheelSpeed;
    customToggleModeDisplay = static_cast<int>(mode);
    customTogglePreferredDirectionDisplay = preferredSpinDirection;

    comprisedCommandScheduler.run();
}

void ChassisToggleDriveCustomControllerCommand::end(bool interrupted) {
    descheduleIfScheduled(this->comprisedCommandScheduler, &ignoreGimbalCommand, interrupted);
    descheduleIfScheduled(this->comprisedCommandScheduler, &tokyoMasterCommand, interrupted);
    chassis->setTokyoDrift(false);
    chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);
}

bool ChassisToggleDriveCustomControllerCommand::isReady() { return true; }

bool ChassisToggleDriveCustomControllerCommand::isFinished() const { return false; }

}  // namespace src::Chassis

#endif  // #ifdef CHASSIS_COMPATIBLE
