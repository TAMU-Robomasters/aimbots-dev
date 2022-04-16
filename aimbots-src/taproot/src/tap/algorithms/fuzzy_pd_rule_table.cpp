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

#include "fuzzy_pd_rule_table.hpp"

using namespace std;

namespace tap::algorithms
{
void FuzzyPDRuleTable::performFuzzification(float e, float d)
{
    performSingleValueFuzzification(e, errorFuzzificationMemberValues);
    performSingleValueFuzzification(d, derivativeFuzzificationMemberValues);
}

void FuzzyPDRuleTable::updateFuzzyMatrix()
{
    // Insert elements into the fuzzy matrix
    // Fuzzy multiply the 2 3x1 fuzzy vectors together to product the single matrix
    for (size_t i = 0; i < NUM_FUZZY_MEMBERS; i++)
    {
        for (size_t j = 0; j < NUM_FUZZY_MEMBERS; j++)
        {
            fuzzyMatrix[i][j] =
                min(errorFuzzificationMemberValues[i], derivativeFuzzificationMemberValues[j]);
        }
    }
}

/// Computes dot product of weights and values, in essence computing the weighted gain, a linear
/// combination of the values
inline float computeGain(
    const array<float, FuzzyPDRuleTable::NUM_FUZZY_MEMBERS> &weights,
    const array<float, FuzzyPDRuleTable::NUM_FUZZY_MEMBERS> &values)
{
    float weightSum = 0;
    for (size_t i = 0; i < weights.size(); i++)
    {
        weightSum += weights[i];
    }

    if (weightSum > 0)
    {
        float gain = 0;
        for (size_t i = 0; i < weights.size(); i++)
        {
            gain += weights[i] * values[i];
        }

        return gain / weightSum;
    }
    else
    {
        return 0;
    }
}

void FuzzyPDRuleTable::performDefuzzification()
{
    // Choosen based on what "feels" right (very experimental)
    // In general, the following rules are followed when selecting which weight is associated with
    // which gain:
    // - the P gain should be small when error & derivative is small
    // - the P gain should be large when the error is large and the derivative indicates the error
    //   is increasing
    // - otherwise the P gain should be medium
    // - the D gain should be small when the error is large and the derivative is increasing the
    //   error or if the error and derivative are close to 0
    // - the D gain should be large when the error is 0 but the derivative is large (either negative
    //   or positive)
    // - otherwise the D gain should be medium

    array<float, NUM_FUZZY_MEMBERS> kpWeights;

    // small weight
    kpWeights[0] = fuzzyMatrix[Z][Z];

    // medium weight
    kpWeights[1] = max(
        max(max(fuzzyMatrix[Z][N], fuzzyMatrix[N][Z]), max(fuzzyMatrix[P][Z], fuzzyMatrix[Z][P])),
        max(fuzzyMatrix[N][N], fuzzyMatrix[P][P]));

    // large weight
    kpWeights[2] = max(fuzzyMatrix[N][P], fuzzyMatrix[P][N]);

    fuzzyGains[0][0] = computeGain(kpWeights, kpArray);

    array<float, NUM_FUZZY_MEMBERS> kdWeights;

    // small weight
    kdWeights[0] = max(max(fuzzyMatrix[N][P], fuzzyMatrix[P][N]), fuzzyMatrix[Z][Z]);
    // kdWeights[0] = max(max(fuzzyMatrix[Z][P], fuzzyMatrix[Z][N]), fuzzyMatrix[Z][Z]);

    // medium weight
    kdWeights[1] =
        max(max(fuzzyMatrix[N][N], fuzzyMatrix[N][Z]), max(fuzzyMatrix[P][Z], fuzzyMatrix[P][P]));

    // large weight
    kdWeights[2] = max(fuzzyMatrix[Z][N], fuzzyMatrix[Z][P]);

    fuzzyGains[1][0] = computeGain(kdWeights, kdArray);
}

modm::Matrix<float, 2, 1> FuzzyPDRuleTable::performFuzzyUpdate(float e, float d)
{
    performFuzzification(e, d);
    updateFuzzyMatrix();
    performDefuzzification();
    return fuzzyGains;
}
}  // namespace tap::algorithms
