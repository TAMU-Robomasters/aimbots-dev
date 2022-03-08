/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef AGITATOR_UNJAM_COMMAND_HPP_
#define AGITATOR_UNJAM_COMMAND_HPP_

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/control/command.hpp"
#include "tap/drivers.hpp"
#include "tap/motor/dji_motor.hpp"

namespace tap
{
namespace control
{
namespace setpoint
{
// Forward declarations
class SetpointSubsystem;

/**
 * Command that takes control of an agitator motor and attempts to unjam it. Whether
 * or not the agitator is actually in a jam condition is not up for this command to
 * determine. It is assumed that unjamming must occur.
 */
class UnjamCommand : public tap::control::Command
{
public:
    /**
     * @param[in] agitator The associated agitator subsystem to control.
     * @param[in] agitatorMaxUnjamAngle The maximum backwards rotation of the agitator
     *      to be used in an unjam step. A random backwards angle is subsequently choosen
     *      each time the agitator unjam command attempts to rotate the agitator backwards.
     * @param[in] agitatorMaxWaitTime The maximum amount of time the controller will
     *      wait for the motor to rotate backwards before commencing with a forward rotation.
     */
    UnjamCommand(
        SetpointSubsystem* agitator,
        float agitatorMaxUnjamAngle,
        uint32_t agitatorMaxWaitTime = AGITATOR_MAX_WAIT_TIME);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char* getName() const override { return "agitator unjam"; }

private:
    static constexpr uint32_t SALVATION_TIMEOUT_MS = 2000;

    static constexpr uint32_t SALVATION_UNJAM_BACK_WAIT_TIME = 1000;

    static constexpr float SETPOINT_TOLERANCE = M_PI / 16.0f;

    /**
     * The maximum time that the command will wait from commanding the agitator to rotate
     * backwards to rotating forwards again.
     */
    static constexpr uint32_t AGITATOR_MAX_WAIT_TIME = 130;

    /**
     * Minimum angle the agitator will rotate backwards when unjamming.
     */
    static constexpr float MIN_AGITATOR_UNJAM_ANGLE = M_PI / 4.0f;

    enum AgitatorUnjamState
    {
        AGITATOR_SALVATION_UNJAM_BACK,
        AGITATOR_UNJAM_BACK,
        AGITATOR_UNJAM_RESET,
        FINISHED
    };

    AgitatorUnjamState currUnjamstate;

    /**
     * Time allowed to rotate back the the `currAgitatorUnjamAngle`.
     */
    tap::arch::MilliTimeout agitatorUnjamRotateTimeout;

    tap::arch::MilliTimeout salvationTimeout;

    /**
     * Usually set to `AGITATOR_MAX_WAIT_TIME`, but can be user defined.
     */
    uint32_t agitatorMaxWaitTime;

    SetpointSubsystem* setpointSubsystem;

    float agitatorUnjamAngleMax;

    float currAgitatorUnjamAngle;

    float agitatorSetpointBeforeUnjam;
};  // class UnjamCommand

}  // namespace setpoint

}  // namespace control

}  // namespace tap

#endif  // AGITATOR_UNJAM_COMMAND_HPP_
