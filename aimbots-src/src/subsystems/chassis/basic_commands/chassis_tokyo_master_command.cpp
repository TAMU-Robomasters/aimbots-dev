#include "chassis_tokyo_master_command.hpp"

#ifdef GIMBAL_UNTETHERED
#ifdef CHASSIS_COMPATIBLE

#include <cmath>
#include <cstdlib>

#include "subsystems/chassis/control/chassis_helper.hpp"

namespace src::Chassis {

// Debug/watch variables for ozone
float tokyoMasterDesiredXDisplay = 0.0f;
float tokyoMasterDesiredYDisplay = 0.0f;
float tokyoMasterDesiredRotationDisplay = 0.0f;
float tokyoMasterRampTargetDisplay = 0.0f;
float tokyoMasterJoystick2OverrideDisplay = 0.0f;
float tokyoMasterSpinRateModifierDisplay = 1.0f;
float tokyoMasterMaxWheelSpeedDisplay = 0.0f;
int tokyoMasterModeDisplay = 0;
int tokyoMasterDirectionDisplay = 0;
bool tokyoMasterManualOverrideActiveDisplay = false;

// Auto-mode debug/watch variables
bool tokyoMasterAutoDisplay = false;
bool tokyoMasterJetsonOnlineDisplay = false;
float tokyoMasterNavVelCmdXDisplay = 0.0f;
float tokyoMasterNavVelCmdYDisplay = 0.0f;

// Tunable: scales the Jetson velocity command (m/s) into the chassis input range used below. Tune in
// Ozone until the robot's real translation speed matches the commanded m/s.
float tokyoMasterNavVelocityToInputMultiplier = 1.0f;

ChassisTokyoMasterCommand::ChassisTokyoMasterCommand(
    src::Drivers* drivers,
    ChassisSubsystem* chassis,
    src::Gimbal::GimbalSubsystem* gimbal,
    const TokyoConfig& tokyoConfig,
    int spinDirectionOverride,
    bool randomizeSpinRate,
    const SpinRandomizerConfig& randomizerConfig,
    ChassisTokyoMasterMode mode,
    float joystick2OverrideVelocity,
    float maxWheelSpeed,
    bool isAuto)
    : drivers(drivers),
      chassis(chassis),
      gimbal(gimbal),
      tokyoConfig(tokyoConfig),
      spinDirectionOverride(sanitizeSpinDirection(spinDirectionOverride)),
      randomizeSpinRate(randomizeSpinRate),
      randomizerConfig(randomizerConfig),
      mode(mode),
      joystick2OverrideVelocity(joystick2OverrideVelocity),
      maxWheelSpeed(maxWheelSpeed),
      isAuto(isAuto) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
}

void ChassisTokyoMasterCommand::initialize() {
    activeSpinDirection = spinDirectionOverride == 0 ? pickRandomDirection() : spinDirectionOverride;
    lastSpinDirectionOverride = spinDirectionOverride;
    lastMode = mode;
    spinRateModifier = 1.0f;
    spinRateModifierDuration = 0;
    spinRateModifierTimer.restart(0);
    rotationSpeedRamp.reset(chassis->getDesiredRotation());
    lastMaxWheelSpeed = maxWheelSpeed;
    chassis->setTokyoDrift(true);
}

void ChassisTokyoMasterCommand::configure(
    ChassisTokyoMasterMode mode,
    float joystick2OverrideVelocity,
    float maxWheelSpeed,
    int spinDirectionOverride) {
    setMode(mode);
    setJoystick2OverrideVelocity(joystick2OverrideVelocity);
    setMaxWheelSpeed(maxWheelSpeed);
    setSpinDirectionOverride(spinDirectionOverride);
}

void ChassisTokyoMasterCommand::setJoystick2OverrideVelocity(float joystick2OverrideVelocity) {
    this->joystick2OverrideVelocity = limitVal<float>(joystick2OverrideVelocity, -1.0f, 1.0f);
}

void ChassisTokyoMasterCommand::setMaxWheelSpeed(float maxWheelSpeed) {
    this->maxWheelSpeed = maxWheelSpeed > 0.0f ? maxWheelSpeed : 6500.0f;
}

void ChassisTokyoMasterCommand::setSpinDirectionOverride(int spinDirectionOverride) {
    this->spinDirectionOverride = sanitizeSpinDirection(spinDirectionOverride);
}

int ChassisTokyoMasterCommand::pickRandomDirection() const {
    return (rand() - RAND_MAX / 2) < 0 ? 1 : -1;
}

int ChassisTokyoMasterCommand::sanitizeSpinDirection(int spinDirection) const {
    if (spinDirection > 0) {
        return 1;
    }
    if (spinDirection < 0) {
        return -1;
    }
    return 0;
}

void ChassisTokyoMasterCommand::refreshActiveSpinDirectionIfNeeded() {
    const bool spinDirectionOverrideChanged = spinDirectionOverride != lastSpinDirectionOverride;
    const bool modeChanged = mode != lastMode;

    if (spinDirectionOverrideChanged || modeChanged) {
        activeSpinDirection = spinDirectionOverride == 0 ? pickRandomDirection() : spinDirectionOverride;
        lastSpinDirectionOverride = spinDirectionOverride;
        lastMode = mode;
        spinRateModifier = 1.0f;
        spinRateModifierTimer.restart(0);
    }
}

float ChassisTokyoMasterCommand::applyDeadband(float value, float deadband) const {
    return std::fabs(value) < deadband ? 0.0f : value;
}

float ChassisTokyoMasterCommand::getBaseAutoSpinTarget(float maxWheelSpeed) const {
    return maxWheelSpeed * static_cast<float>(activeSpinDirection) * tokyoConfig.rotationalSpeedFractionOfMax;
}

float ChassisTokyoMasterCommand::getRandomizedNormalSpinTarget(float maxWheelSpeed) {
    float rampTarget = getBaseAutoSpinTarget(maxWheelSpeed);

    if (randomizeSpinRate) {
        if (spinRateModifierTimer.isExpired() || spinRateModifierTimer.isStopped()) {
            Helper::randomizeSpinCharacteristics(
                &spinRateModifier,
                &spinRateModifierDuration,
                randomizerConfig);
            spinRateModifierTimer.restart(spinRateModifierDuration);
        }
        rampTarget *= spinRateModifier;
    } else {
        spinRateModifier = 1.0f;
    }

    tokyoMasterSpinRateModifierDisplay = spinRateModifier;
    return rampTarget;
}

float ChassisTokyoMasterCommand::getSinusodalSpinTarget(float maxWheelSpeed) {
    Helper::sinusodalSpinCharacteristics(
        &spinRateModifier,
        &spinRateModifierDuration,
        randomizerConfig,
        true);

    tokyoMasterSpinRateModifierDisplay = spinRateModifier;
    return getBaseAutoSpinTarget(maxWheelSpeed) * spinRateModifier;
}

void ChassisTokyoMasterCommand::execute() {
    refreshActiveSpinDirectionIfNeeded();
    chassis->setTokyoDrift(true);
    tokyoMasterAutoDisplay = isAuto;

    float desiredX = 0.0f;
    float desiredY = 0.0f;

    if (isAuto) {
        // Auto mode: translation is driven by nav2 on the Jetson, not the operator. With no command
        // available the chassis must stop, so bail (still tokyo-drifting from above).
        const bool jetsonOnline = drivers->cvCommunicator.isJetsonOnline();
        tokyoMasterJetsonOnlineDisplay = jetsonOnline;
        if (!jetsonOnline) {
            chassis->setTargetRPMs(0.0f, 0.0f, 0.0f, maxWheelSpeed);
            return;
        }

        // Field-relative chassis velocity command (m/s) from nav2 on the Jetson.
        const modm::Vector2f fieldRelativeVelocity = drivers->cvCommunicator.getDesiredTurretRelativeVelocity();
        tokyoMasterNavVelCmdXDisplay = fieldRelativeVelocity.getX();
        tokyoMasterNavVelCmdYDisplay = fieldRelativeVelocity.getY();

        desiredX = fieldRelativeVelocity.getX() * tokyoMasterNavVelocityToInputMultiplier;
        desiredY = fieldRelativeVelocity.getY() * tokyoMasterNavVelocityToInputMultiplier;
    } else {
        const bool customControllerConnected =
            drivers->controlOperatorInterface.isCustomControllerConnected();

        float unusedRotation = 0.0f;
        Chassis::Helper::getUserDesiredInput(
            drivers,
            chassis,
            &desiredX,
            &desiredY,
            &unusedRotation);

        const float customX = customControllerConnected
            ? applyDeadband(
                  drivers->controlOperatorInterface.getCustomControllerChassisXInput(),
                  TRANSLATION_DEADBAND)
            : 0.0f;

        const float customY = customControllerConnected
            ? applyDeadband(
                  drivers->controlOperatorInterface.getCustomControllerChassisYInput(),
                  TRANSLATION_DEADBAND)
            : 0.0f;

        desiredX = limitVal<float>(desiredX + customX, -1.0f, 1.0f);
        desiredY = limitVal<float>(desiredY + customY, -1.0f, 1.0f);
    }

    desiredX *= tokyoConfig.translationalSpeedMultiplier * maxWheelSpeed;
    desiredY *= tokyoConfig.translationalSpeedMultiplier * maxWheelSpeed;

    // No operator joystick in auto mode, so the manual spin override is ignored; the spin ramp /
    // randomizer / sinusoidal path drives rotation instead.
    const float manualSpin = isAuto ? 0.0f : applyDeadband(joystick2OverrideVelocity, JOYSTICK_OVERRIDE_DEADBAND);
    const bool manualOverrideActive = manualSpin != 0.0f;

    float rampTarget = 0.0f;
    if (manualOverrideActive) {
        // Positive joystick 2 X is positive chassis angular command / CCW
        rampTarget = manualSpin * maxWheelSpeed * tokyoConfig.rotationalSpeedFractionOfMax;
        spinRateModifier = 1.0f;
        tokyoMasterSpinRateModifierDisplay = spinRateModifier;
    } else if (mode == ChassisTokyoMasterMode::SINUSODAL) {
        rampTarget = getSinusodalSpinTarget(maxWheelSpeed);
    } else {
        rampTarget = getRandomizedNormalSpinTarget(maxWheelSpeed);
    }

    const float translationalSpeedThreshold = maxWheelSpeed * tokyoConfig.translationalSpeedMultiplier *
                                              tokyoConfig.translationThresholdToDecreaseRotationSpeed;

    // Match the existing Tokyo behavior: reduce rotation speed when translation is high.
    if (std::fabs(desiredX) > translationalSpeedThreshold || std::fabs(desiredY) > translationalSpeedThreshold) {
        rampTarget *= tokyoConfig.rotationalSpeedMultiplierWhenTranslating;
    }

    rotationSpeedRamp.setTarget(rampTarget);

    // Normal Tokyo behavior ramps rotational speed smoothly. If the parent scheduler has just reduced
    // maxWheelSpeed because chassis power is too high, allow the rotational command to fall much faster
    // so power limiting has an immediate effect instead of waiting on the normal spin ramp.
    const bool maxWheelSpeedReduced = maxWheelSpeed + 1.0f < lastMaxWheelSpeed;
    const bool targetMagnitudeReduced = std::fabs(rampTarget) < std::fabs(rotationSpeedRamp.getValue());
    const float rotationRampIncrement =
        (maxWheelSpeedReduced && targetMagnitudeReduced)
            ? POWER_LIMITED_ROTATION_DECREASE_INCREMENT
            : tokyoConfig.rotationalSpeedIncrement;

    rotationSpeedRamp.update(rotationRampIncrement);
    const float desiredRotation = rotationSpeedRamp.getValue();
    lastMaxWheelSpeed = maxWheelSpeed;

    if (gimbal != nullptr && gimbal->isOnline()) {
        if (isAuto) {
            // The Jetson sends field-relative velocity, so first rotate field -> turret using the
            // turret's field-relative yaw (from its IMU) before the turret -> chassis step below.
            const float turretFieldYaw =
                drivers->kinematicInformant.getCurrentFieldRelativeGimbalYawAngleAsWrappedFloat().getWrappedValue();
            tap::algorithms::rotateVector(&desiredX, &desiredY, -turretFieldYaw);
        }
        // turret-relative -> chassis-relative
        const float yawAngleFromChassisCenter = gimbal->getCurrentYawAxisAngle(AngleUnit::Radians);
        tap::algorithms::rotateVector(&desiredX, &desiredY, yawAngleFromChassisCenter);
    }

    tokyoMasterDesiredXDisplay = desiredX;
    tokyoMasterDesiredYDisplay = desiredY;
    tokyoMasterDesiredRotationDisplay = desiredRotation;
    tokyoMasterRampTargetDisplay = rampTarget;
    tokyoMasterJoystick2OverrideDisplay = joystick2OverrideVelocity;
    tokyoMasterManualOverrideActiveDisplay = manualOverrideActive;
    tokyoMasterMaxWheelSpeedDisplay = maxWheelSpeed;
    tokyoMasterModeDisplay = static_cast<int>(mode);
    tokyoMasterDirectionDisplay = activeSpinDirection;

    chassis->setTargetRPMs(desiredX, desiredY, desiredRotation, maxWheelSpeed);
}

void ChassisTokyoMasterCommand::end(bool interrupted) {
    UNUSED(interrupted);
    chassis->setTokyoDrift(false);
    chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);
}

bool ChassisTokyoMasterCommand::isReady() { return true; }

bool ChassisTokyoMasterCommand::isFinished() const { return false; }

}  // namespace src::Chassis

#endif  // #ifdef CHASSIS_COMPATIBLE
#endif  // #ifdef GIMBAL_UNTETHERED