#include "chassis_rail_evade_command.hpp"

#include "modm/platform/random/random_number_generator.hpp"
#include "utils/robot_specific_inc.hpp"


namespace src::Chassis {

static constexpr int32_t MIN_RPM = 4000;
static constexpr int32_t MAX_RPM = 6000;

static constexpr float MIN_TRAVERSE_DISTANCE_MM = 250.0f;
static constexpr float MAX_TRAVERSE_DISTANCE_MM = MIN_TRAVERSE_DISTANCE_MM + 400.0f;

// static constexpr float FULL_RAIL_LENGTH_MM = FULL_RAIL_LENGTH * 1000.0f;
static constexpr float SAFETY_BUFFER = 0.15f;
static constexpr float TURNAROUND_BUFFER = (((WHEELBASE_WIDTH /*+ RAIL_POLE_DIAMETER*/) / 2.0f) + SAFETY_BUFFER) * 1000.0f;

ChassisRailEvadeCommand::ChassisRailEvadeCommand(src::Drivers* drivers, ChassisSubsystem* chassis, float velocityRampValue)
    : drivers(drivers),
      chassis(chassis),
      velocityRampValue(velocityRampValue) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
}

void ChassisRailEvadeCommand::initialize() {
    modm::platform::RandomNumberGenerator::enable();
    changeDirectionForRandomDistance(MIN_TRAVERSE_DISTANCE_MM, MAX_TRAVERSE_DISTANCE_MM);
    velocityRamp.setValue(chassis->getLeftFrontRpmActual());
}

float currPositionDisplay = 0.0f;

void ChassisRailEvadeCommand::execute() {
    // float currRailPosition = drivers->fieldRelativeInformant.getRailRelativeRobotPosition()[0][X] * 1000.0f;

    // currPositionDisplay = drivers->fieldRelativeInformant.getRailRelativeRobotPosition()[0][X];

    // if (hasTraveledDriveDistance(currRailPosition)) changeDirectionForRandomDistance(MIN_TRAVERSE_DISTANCE_MM,
    // MAX_TRAVERSE_DISTANCE_MM);

    // changeDirectionIfCloseToEnd(currRailPosition);

    velocityRamp.update(velocityRampValue);

    chassis->setTargetRPMs(velocityRamp.getValue(), 0.0f, 0.0f);
}

bool ChassisRailEvadeCommand::isReady() { return true; }

bool ChassisRailEvadeCommand::isFinished() const { return false; }

void ChassisRailEvadeCommand::end(bool interrupted) {
    UNUSED(interrupted);
    chassis->setTargetRPMs(0.0f, 0.0f, 0.0f);
}

float distanceToDriveDisplay = 0.0f;

void ChassisRailEvadeCommand::changeDirectionForRandomDistance(
    int32_t minimumDistanceMillimeters,
    int32_t maximumDistanceMillimeters) {
    // lastPositionWhenDirectionChanged = drivers->fieldRelativeInformant.getRailRelativeRobotPosition()[0][X] * 1000.0f;

    currentDesiredRPM = getNewRPM();
    // chassis->setTargetRPMs(currentDesiredRPM, 0, 0);
    velocityRamp.setTarget(currentDesiredRPM);

    distanceToDrive = getRandomIntegerInBounds(minimumDistanceMillimeters, maximumDistanceMillimeters);
    distanceToDriveDisplay = distanceToDrive;
}

float currentRailPositionMillimetersDisplay = 0.0f;

void ChassisRailEvadeCommand::changeDirectionIfCloseToEnd(float currentRailPositionMillimeters) {
    currentRailPositionMillimetersDisplay = currentRailPositionMillimeters;
    if ((currentRailPositionMillimeters <= TURNAROUND_BUFFER && currentDesiredRPM >= 0) ||
        (currentRailPositionMillimeters >= (/*FULL_RAIL_LENGTH_MM*/ 0 - TURNAROUND_BUFFER) && currentDesiredRPM <= 0)) {
        float distanceFromCenter = fabs((/*FULL_RAIL_LENGTH_MM*/ 0 / 2.0f) - currentRailPositionMillimeters);
        float distanceFromOppositeEnd =
            std::max(/*FULL_RAIL_LENGTH_MM*/ 0 - currentRailPositionMillimeters, currentRailPositionMillimeters) -
            TURNAROUND_BUFFER;

        changeDirectionForRandomDistance(distanceFromCenter, distanceFromOppositeEnd);
    }
}

uint32_t ChassisRailEvadeCommand::getRandomInteger() {
    if (modm::platform::RandomNumberGenerator::isReady()) return modm::platform::RandomNumberGenerator::getValue();
    return 0;
}

float randomDisplay = 0.0f;
float baseDisplay = 0.0f;

int32_t ChassisRailEvadeCommand::getRandomIntegerInBounds(int32_t min, int32_t max) {
    uint32_t range = max - min;
    uint32_t random = getRandomInteger();
    randomDisplay = random;
    uint32_t base = random % range;
    baseDisplay = base;

    return static_cast<int32_t>(base) + min;
}

int32_t ChassisRailEvadeCommand::getNewRPM() {
    int32_t rand = getRandomIntegerInBounds(MIN_RPM, MAX_RPM);
    return copysign(rand, -currentDesiredRPM);
}

}  // namespace src::Chassis