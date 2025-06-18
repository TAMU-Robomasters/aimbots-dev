/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TAPROOT_FUZZY_PD_HPP_
#define TAPROOT_FUZZY_PD_HPP_

#include <cstdint>

#include "tap/algorithms/extended_kalman.hpp"

#include "fuzzy_pd_rule_table.hpp"
#include "smooth_pid.hpp"

namespace tap::algorithms
{
/// Specifies Fuzzy-specific configurable parameters in the PD controller
struct FuzzyPDConfig
{
    float maxError = 0.0f;
    float maxErrorDerivative = 0.0f;
    FuzzyPDRuleTable fuzzyTable;
};

/**
 * Fuzzy PD controller. A controller that is built on top of the SmoothPid object.
 *
 * Fuzzy logic is used to adaptively change the proportional and derivative gains at runtime,
 * whereas they are hard-coded in the SmoothPid object. This allows the controller to adapt to more
 * complex situations (such as overcoming sticktion, varying loads, etc.). Aside from adaptively
 * changing the gains, this controller behaves the same was as the SmoothPid object.
 *
 * For more general information about fuzzy PID, refer to this paper:
 * https://ieeexplore.ieee.org/document/937407. You can also find others online. For a more generic
 * look at fuzzy logic, you can refer to this paper:
 * https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=488475.
 *
 * @note While it is not explicitly disallowed, the integral gain is not intended to be used in this
 * implementation. The FuzzyPDRuleTable is used in this controller, which does not have any fuzzy
 * logic for the integral term.
 */
class FuzzyPD : public SmoothPid
{
public:
    /**
     * @param[in] pdConfig Fuzzy-specific configuration.
     * @param[in] smoothPidConfig PID-specific configuration.
     */
    FuzzyPD(const FuzzyPDConfig &pdConfig, const SmoothPidConfig &smoothPidConfig);

    /**
     * @see SmoothPID::runController. Identical except that before the PID update step, new
     * parameters are computed.
     */
    float runController(float error, float errorDerivative, float dt) override;

private:
    FuzzyPDConfig config;

    void udpatePidGains(float error, float errorDerivative);
};

}  // namespace tap::algorithms

#endif  // TAPROOT_FUZZY_PD_HPP_
