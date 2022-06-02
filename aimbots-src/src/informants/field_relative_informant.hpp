#pragma once
#include "utils/common_types.hpp"

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

        void updateFieldRelativeRobotPosition(DJIMotor* cMotor);

        Matrix<float, 1, 3> getFieldRelativeRobotPosition() {
            return fieldRelativeRobotPosition;
        }

        float getXYAngleToFieldCoordinate(AngleUnit unit, Matrix<float, 1, 3> fieldCoordinate);

       private:
        src::Drivers* drivers;

        Matrix<float, 1, 3> robotStartingPosition;

#ifdef TARGET_SENTRY
        Matrix<float, 1, 3> railRelativePosition;
#endif

        Matrix<float, 1, 3> fieldRelativeRobotPosition;
    };

}  // namespace src::Informants
