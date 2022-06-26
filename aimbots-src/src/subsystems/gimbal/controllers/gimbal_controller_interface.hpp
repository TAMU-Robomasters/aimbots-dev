#pragma once

#include <subsystems/gimbal/gimbal.hpp>

namespace src::Gimbal {

class GimbalControllerInterface {
   public:
    virtual void initialize() = 0;

    virtual void runYawController(AngleUnit unit, float targetYawAngle) = 0;
    virtual void runPitchController(AngleUnit unit, float targetPitchAngle) = 0;

    virtual float getTargetYaw(AngleUnit unit) const = 0;

    virtual bool isOnline() const = 0;
};

}  // namespace src::Gimbal