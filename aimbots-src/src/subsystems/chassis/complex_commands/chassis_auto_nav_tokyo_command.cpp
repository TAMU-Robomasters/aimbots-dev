#include "chassis_auto_nav_tokyo_command.hpp"

#include "subsystems/chassis/control/chassis_helper.hpp"

#ifdef CHASSIS_COMPATIBLE

namespace src::Chassis {

ChassisAutoNavTokyoCommand::ChassisAutoNavTokyoCommand(
    src::Drivers* drivers,
    ChassisSubsystem* chassis,
    SmoothPIDConfig linearPIDConfig,
    const TokyoConfig& tokyoConfig,
    bool randomizeSpinRate,
    const SpinRandomizerConfig& randomizerConfig,
    float linearSettledThreshold)
    : drivers(drivers),
      chassis(chassis),
      autoNavigator(),
      xController(linearPIDConfig),
      yController(linearPIDConfig),
      tokyoConfig(tokyoConfig),
      randomizeSpinRate(randomizeSpinRate),
      randomizerConfig(randomizerConfig),
      linearSettledThreshold(linearSettledThreshold)  //
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
}

void ChassisAutoNavTokyoCommand::initialize() {
    rotationDirection = (rand() - RAND_MAX / 2) < 0 ? 1 : -1;

    rotationSpeedRamp.reset(chassis->getDesiredRotation());

    if (randomizeSpinRate) {
        spinRateModifierTimer.restart(0);
    }

    // modm::Location2D<float> targetLocation({1.5f, 1.5f}, modm::toRadian(90.0f));  // test
    // autoNavigator.setTargetLocation(targetLocation);
}

void ChassisAutoNavTokyoCommand::execute() {
    float xError = 0.0f;
    float yError = 0.0f;
    float rotationError = 0.0f;

    modm::Location2D<float> currentWorldLocation = drivers->kinematicInformant.getRobotLocation2D();
    modm::Vector2f currentWorldVelocity = drivers->kinematicInformant.getRobotVelocity2D();

    autoNavigator.update(currentWorldLocation);

    // rotationError intentionally remains unused if tokyo drifting
    autoNavigator.getDesiredInput(&xError, &yError, &rotationError);

    float desiredX = xController.runController(xError, currentWorldVelocity.getX());
    float desiredY = yController.runController(yError, currentWorldVelocity.getY());

    const float maxWheelSpeed = ChassisSubsystem::getMaxRefWheelSpeed(
        drivers->refSerial.getRefSerialReceivingData(),
        drivers->refSerial.getRobotData().chassis.powerConsumptionLimit);

    desiredX *= tokyoConfig.translationalSpeedMultiplier * maxWheelSpeed;
    desiredY *= tokyoConfig.translationalSpeedMultiplier * maxWheelSpeed;

    const float translationalSpeedThreshold =
        maxWheelSpeed * tokyoConfig.translationalSpeedMultiplier * tokyoConfig.translationThresholdToDecreaseRotationSpeed;

    float rampTarget = maxWheelSpeed * rotationDirection * tokyoConfig.rotationalSpeedFractionOfMax;

    if (randomizeSpinRate) {
        if (spinRateModifierTimer.isExpired() || spinRateModifierTimer.isStopped()) {
            Helper::randomizeSpinCharacteristics(&this->spinRateModifier, &this->spinRateModifierDuration, randomizerConfig);
            spinRateModifierTimer.restart(spinRateModifierDuration);
        }
        rampTarget *= spinRateModifier;
    }

    if (fabs(desiredX) > translationalSpeedThreshold || fabs(desiredY) > translationalSpeedThreshold) {
        // we're moving fast enough to warrant slowing down rotation
        rampTarget *= tokyoConfig.rotationalSpeedMultiplierWhenTranslating;
    }

    rotationSpeedRamp.setTarget(rampTarget);
    rotationSpeedRamp.update(tokyoConfig.rotationalSpeedIncrement);
    float desiredRotation = rotationSpeedRamp.getValue();

    rotateVector(&desiredX, &desiredY, -currentWorldLocation.getOrientation());

    chassis->setTargetRPMs(desiredX, desiredY, desiredRotation);
}

bool ChassisAutoNavTokyoCommand::isReady() { return true; }

bool ChassisAutoNavTokyoCommand::isFinished() const { return false; }

void ChassisAutoNavTokyoCommand::end(bool interrupted) {
    UNUSED(interrupted);
    chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);
}

}  // namespace src::Chassis

#endif //#ifdef CHASSIS_COMPATIBLE