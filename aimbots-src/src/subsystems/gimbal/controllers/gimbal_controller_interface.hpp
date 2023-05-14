#pragma once

#include <subsystems/gimbal/gimbal.hpp>

namespace src::Gimbal {

class GimbalControllerInterface {
public:
    virtual void initialize() = 0;

    virtual void runYawController(bool vision = false) = 0;
    virtual void runPitchController(bool vision = false) = 0;

    virtual void setTargetYaw(AngleUnit unit, float targetYaw) = 0;
    virtual void setTargetPitch(AngleUnit unit, float targetPitch) = 0;

    virtual float getTargetYaw(AngleUnit unit) const = 0;
    virtual float getTargetPitch(AngleUnit unit) const = 0;

    virtual bool isOnline() const = 0;
};

}  // namespace src::Gimbal