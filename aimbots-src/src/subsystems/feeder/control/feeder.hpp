#pragma once
#include "tap/control/subsystem.hpp"
#include "tap/motor/m3508_constants.hpp"

#include "utils/tools/common_types.hpp"
#include "utils/tools/robot_specific_inc.hpp"

#include "drivers.hpp"

#ifdef FEEDER_COMPATIBLE

namespace src::Feeder {
class FeederSubsystem : public tap::control::Subsystem {
public:
    FeederSubsystem(src::Drivers* drivers);

    mockable void initialize() override;
    mockable void refresh() override;

    //  float getMotorSpeed(FeederIdx feederIdx) const;

    inline bool isOnline() const { return false; }

    void updateMotorVelocityPID();

    void setDesiredFeederMotorOutput();

    void setTargetRPM(float targetRPM);

    float getCurrentRPM() { return 0; }

private:
    src::Drivers* drivers;

    float desiredFeederMotorOutput;

    float feederTargetRPM;

    SmoothPID feederVelocityPID;

    DJIMotor feederMotor;
};

}  // namespace src::Feeder

#endif  // #ifdef FEEDER_COMPATIBLE