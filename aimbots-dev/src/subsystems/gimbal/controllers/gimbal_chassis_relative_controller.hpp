#pragma once

#include <drivers.hpp>
#include <subsystems/gimbal/gimbal.hpp>
#include <utils/common_types.hpp>

namespace src::Gimbal {

class GimbalChassisRelativeController {
   public:
    GimbalChassisRelativeController(GimbalSubsystem*);

    void initialize();

    void runYawController(float targetYawAngle);
    void runPitchController(float targetPitchAngle);

    bool isOnline() const;

   private:
    GimbalSubsystem* gimbal;

    StockPID yawPID;
    StockPID pitchPID;
};

}  // namespace src::Gimbal