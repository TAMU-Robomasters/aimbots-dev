#pragma once

#include <subsystems/gimbal/gimbal.hpp>
#ifdef GIMBAL_COMPATIBLE

namespace src::Gimbal {

class GimbalControllerInterface {
public:
    virtual void initialize() = 0;

    virtual void runYawController(std::optional<float> velocityLimit = std::nullopt) = 0;
    virtual void runPitchController(std::optional<float> velocityLimit = std::nullopt) = 0;

    virtual void setTargetYaw(AngleUnit unit, float targetYaw) = 0;
    void setTargetYaw(ContiguousFloat targetYaw) { setTargetYaw(AngleUnit::Radians, targetYaw.getValue()); }

    virtual void setTargetPitch(AngleUnit unit, float targetPitch) = 0;
    void setTargetPitch(ContiguousFloat targetPitch) { setTargetPitch(AngleUnit::Radians, targetPitch.getValue()); }

    virtual float getTargetYaw(AngleUnit unit) const = 0;
    virtual float getTargetPitch(AngleUnit unit) const = 0;

    virtual bool isOnline() const = 0;
};

}  // namespace src::Gimbal
#endif