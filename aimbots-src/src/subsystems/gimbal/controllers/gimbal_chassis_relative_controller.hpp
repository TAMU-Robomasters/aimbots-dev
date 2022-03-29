#pragma once

#include <drivers.hpp>
#include <subsystems/gimbal/gimbal.hpp>
#include <utils/common_types.hpp>

namespace src::Gimbal {

class GimbalChassisRelativeController {
   public:
    GimbalChassisRelativeController(GimbalSubsystem*);

    void initialize();

    void runYawController(AngleUnit unit, float targetYawAngle);
    void runPitchController(AngleUnit unit, float targetPitchAngle);

    bool isOnline() const;

   private:
    GimbalSubsystem* gimbal;

    StockPID yawPositionPID;
    StockPID pitchPositionPID;
};

}  // namespace src::Gimbal