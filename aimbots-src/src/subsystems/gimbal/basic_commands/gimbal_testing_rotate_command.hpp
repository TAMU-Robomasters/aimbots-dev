#pragma once

#include "tap/control/command.hpp"

#include "informants/kinematics/enemy_data_conversion.hpp"
#include "subsystems/gimbal/control/gimbal.hpp"
#include "subsystems/gimbal/control/gimbal_chassis_relative_controller.hpp"
#include "subsystems/gimbal/control/gimbal_field_relative_controller.hpp"
#include "utils/ref_system/ref_helper_turreted.hpp"

#include "drivers.hpp"
#ifdef GIMBAL_COMPATIBLE

namespace src::Gimbal {

class GimbalRotateCommand : public tap::control::Command {
public:
    GimbalRotateCommand(
        src::Drivers*,
        GimbalSubsystem*,
        GimbalControllerInterface*,
);

    char const* getName() const override { return "Gimbal Rotate Command"; }

    void initialize() override;
    void execute() override;
    bool isReady() override;
    bool isFinished() const override;
    void end(bool interrupted) override;

private:
    src::Drivers* drivers;

    GimbalSubsystem* gimbal;
    GimbalControllerInterface* controller;
};

}  // namespace src::Gimbal

#endif