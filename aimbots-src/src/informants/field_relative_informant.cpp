#include "field_relative_informant.hpp"

#include "utils/common_types.hpp"
#include "utils/robot_specific_inc.hpp"

namespace src::Informants {

    FieldRelativeInformant::FieldRelativeInformant(src::Drivers * drivers)
        : drivers(drivers),
          robotStartingPosition(ROBOT_STARTING_POSITION),
#ifdef TARGET_SENTRY
          railRelativePosition(Matrix<float, 1, 3>::zeroMatrix()),
#endif
          fieldRelativeRobotPosition(Matrix<float, 1, 3>::zeroMatrix()) {
    }

    void
    FieldRelativeInformant::initialize() {
    }

    float robotPositionXDisplay = 0.0f;
    float robotPositionYDisplay = 0.0f;
    float robotPositionZDisplay = 0.0f;

    void FieldRelativeInformant::updateFieldRelativeRobotPosition(DJIMotor * cMotor) {
#ifdef TARGET_SENTRY  // This will need to be replaced with code that uses the ultrasonics once that works
        // first, get unwrapped motor position
        float motorRevolutionsUnwrapped;
        if (cMotor->isMotorOnline()) {
            motorRevolutionsUnwrapped = static_cast<float>(cMotor->getEncoderUnwrapped()) / static_cast<float>(cMotor->ENC_RESOLUTION);  // current position in motor revolutions
        } else {
            motorRevolutionsUnwrapped = 0.0f;
        }

        // now, convert to unwrapped wheel revolutions
        float wheelRevolutionsUnwrapped = motorRevolutionsUnwrapped * CHASSIS_GEARBOX_RATIO;  // current position in wheel revolutions
        // now, convert to unwrapped wheel rotations to get the rail position
        float currRailPosition = -wheelRevolutionsUnwrapped * (2.0f * M_PI * WHEEL_RADIUS);  // current position in meters

        // set the current rail position to a position matrix relative to the rail
        // Matrix<float, 1, 3> railRelativePosition = Matrix<float, 1, 3>::zeroMatrix();
        railRelativePosition[0][0] = currRailPosition;

        // rotate the matrix by 45 degrees (rail is mounted at 45 degree angle) and add to the robot's starting position
        fieldRelativeRobotPosition = railRelativePosition * src::utils::MatrixHelper::xy_rotation_matrix(AngleUnit::Degrees, 45.0f) + ROBOT_STARTING_POSITION;

        robotPositionXDisplay = fieldRelativeRobotPosition[0][0];
        robotPositionYDisplay = fieldRelativeRobotPosition[0][1];
        robotPositionZDisplay = fieldRelativeRobotPosition[0][2];
#endif
    }

    // gets the angle between the robot's current position and the field coordinate
    float FieldRelativeInformant::getXYAngleToFieldCoordinate(AngleUnit unit, Matrix<float, 1, 3> fieldCoordinate) {
        float xy_angle = src::utils::MatrixHelper::xy_angle_between_locations(AngleUnit::Radians, fieldRelativeRobotPosition, fieldCoordinate);
        if (unit == AngleUnit::Degrees) {
            xy_angle = modm::toDegree(xy_angle);
        }
        return xy_angle;
    }
}  // namespace src::Informants
