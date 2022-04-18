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

#ifndef TAPROOT_FUZZY_PD_RULE_TABLE_HPP_
#define TAPROOT_FUZZY_PD_RULE_TABLE_HPP_

#include <array>

#include "fuzzy_rule_table.hpp"

namespace tap::algorithms
{
/**
 * Rule table that specifically updates the proportional and derivative gains of a PD controller.
 *
 * For more general information about fuzzy PID, refer to this paper:
 * https://ieeexplore.ieee.org/document/937407. You can also find others online. This is slightly
 * different than what is described in the paper.
 *
 * Uses a fuzzy table with 3 fuzzy members (either negative, zero, or positive). Membership
 * functions are triangular to make math easy. Since the table is computing 2 values, the
 * FuzzyRuleTableInterface is instantiated with the OUTPUTS template parameter set to 2.
 *
 * @see FuzzyPD
 */
class FuzzyPDRuleTable : public FuzzyRuleTableInterface<2>
{
public:
    /**
     * Members associated with membership functions
     * (https://www.mathworks.com/help/fuzzy/trimf.html).
     */
    enum FuzzyMembers
    {
        N = 0,  ///< Negative error
        Z,      ///< Zero error
        P,      ///< Positive error
        NUM_FUZZY_MEMBERS,
    };

    /**
     * Default constructor, if default constructor is used the fuzzy rule table will always set the
     * P and D gains to 0.
     */
    FuzzyPDRuleTable() : kpArray{}, kdArray{} {}

    /**
     * @param[in] kpParams A list of possible proportional parameters that are associated with the
     * output of the membership functions. Should be monotonmically increasing. For example, when
     * the fuzzy rule table measures the fuzzy state to be mostly in the location where there is 0
     * error and 0 error derivative, the "small" proportional gain will be mostly used since the
     * proportional gain doesn't have to be as large.
     * @param[in] kdParams A list of possible derivative parameters that are associated with the
     * output of the membership functions. @see kpParams for more information.
     */
    FuzzyPDRuleTable(
        const std::array<float, NUM_FUZZY_MEMBERS> &kpParams,
        const std::array<float, NUM_FUZZY_MEMBERS> &kdParams)
        : kpArray(kpParams),
          kdArray(kdParams)
    {
    }

    /// @see FuzzyRuleTable::performFuzzyUpdate
    modm::Matrix<float, 2, 1> performFuzzyUpdate(float e, float d) override;

    /// @see FuzzyRuleTable::getFuzzyGains
    inline modm::Matrix<float, 2, 1> getFuzzyGains() const override { return fuzzyGains; }

private:
    /**
     * Uses if-else-then rules to perform error fuzzification. 3 Triangle membership functions are
     * used to perform fuzzy classification. These are used to extrapolate the input error and error
     * derivative into various member states (either positive, negative, or zero)
     *
     * @param[in] e measured error
     * @param[in] d derivative of the error (wrt time)
     */
    void performFuzzification(float e, float d);

    /**
     * Update the fuzzy matrix. Fuzzy matrix multiplication of errorFuzzificationMemberValues and
     * derivativeFuzzificationMemberValues.
     */
    void updateFuzzyMatrix();

    /// Applies defuzzification techniques using the fuzzyMatrix to find the fuzzy gain.
    void performDefuzzification();

    /**
     * Uses if-else-then rules to perform error fuzzification. 3 Triangle membership functions are
     * used to perform fuzzy classification. These are triangle membership functions defined with
     * maximums at -1, 0, 1 and widths of 1 each (so the negative triangle intersects the axis at 0,
     * etc.).
     *
     * @param[in] value The value to extrapolate and use to update the member values. Should usually
     * be withing [-1, 1] (though not strictly necessary).
     * @param[out] fuzzificationMemberValues The member value array that will be updated based on
     * the value.
     */
    static inline void performSingleValueFuzzification(
        const float value,
        std::array<float, NUM_FUZZY_MEMBERS> &fuzzificationMemberValues)
    {
        if (value < 0)
        {
            // value < 0 means the positive triangle function is 0 and the negative and zero members
            // should be updated

            // The value should be mapped from [-1, 0] to [1, 0] (i.e. if -1, the negative member
            // value should be 1)
            fuzzificationMemberValues[N] = std::min(-value, 1.0f);
            // The value should be mapped from [-1, 0] to [0, 1] (i.e. if the value is 0, the zero
            // member value should be 1)
            fuzzificationMemberValues[Z] = std::max(value + 1.0f, 0.0f);
            fuzzificationMemberValues[P] = 0;
        }
        else
        {
            // value > 0 means negative triangle function is 0 and positive and zero members should
            // be updated

            fuzzificationMemberValues[N] = 0;
            // The value should be mapped from [0, 1] to [1, 0] (i.e. if the value is 0, the zero
            // member value should be 1)
            fuzzificationMemberValues[Z] = std::max(1.0f - value, 0.0f);
            // The value should be mapped from [0, 1] to [0, 1] (direct mapping)
            fuzzificationMemberValues[P] = std::min(value, 1.0f);
        }
    }

private:
    std::array<float, NUM_FUZZY_MEMBERS> errorFuzzificationMemberValues = {};
    std::array<float, NUM_FUZZY_MEMBERS> derivativeFuzzificationMemberValues = {};
    std::array<std::array<float, NUM_FUZZY_MEMBERS>, NUM_FUZZY_MEMBERS> fuzzyMatrix = {};
    modm::Matrix<float, 2, 1> fuzzyGains = {};
    std::array<float, NUM_FUZZY_MEMBERS> kpArray;
    std::array<float, NUM_FUZZY_MEMBERS> kdArray;
};
}  // namespace tap::algorithms

#endif  // TAPROOT_FUZZY_PD_RULE_TABLE_HPP_
