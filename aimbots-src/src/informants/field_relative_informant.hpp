#pragma once
#include "utils/common_types.hpp"
#include "informants/ultrasonic_distance_sensor.hpp"

namespace src {
class Drivers;
}

namespace src::Informants {

class FieldRelativeInformant {
   public:
    FieldRelativeInformant(src::Drivers* drivers);
    // DISALLOW_COPY_AND_ASSIGN(FieldRelativeInformant);
    ~FieldRelativeInformant() = default;

    void initialize();

#ifdef TARGET_SENTRY
    void updateFieldRelativeRobotPosition(DJIMotor* cMotor);
#else
    void updateFieldRelativeRobotPosition();
#endif

    Matrix<float, 1, 3> getFieldRelativeRobotPosition() {
        return fieldRelativeRobotPosition;
    }

#ifdef TARGET_SENTRY
    Matrix<float, 1, 3> getRailRelativeRobotPosition() {
        return railRelativePosition;
    }
#endif

    float getXYAngleToFieldCoordinate(AngleUnit unit, Matrix<float, 1, 3> fieldCoordinate);

   private:
    src::Drivers* drivers;

    Matrix<float, 1, 3> robotStartingPosition;

#ifdef TARGET_SENTRY
    Matrix<float, 1, 3> railRelativePosition;
#endif

    Matrix<float, 1, 3> fieldRelativeRobotPosition;

    float wheelOffset;
};

}  // namespace src::Informants
