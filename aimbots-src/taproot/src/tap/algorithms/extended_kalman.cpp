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

/*
 * Copyright (c) 2019 Sanger_X
 */

#include "extended_kalman.hpp"

namespace tap
{
namespace algorithms
{
ExtendedKalman::ExtendedKalman(float tQ, float tR)
    : xLast(0.0f),
      xMid(0.0f),
      xNow(0.0f),
      pMid(0.0f),
      pNow(0.0f),
      pLast(0.0f),
      kg(0.0f),
      A(1.0f),
      B(0.0f),
      Q(tQ),
      R(tR),
      H(1.0f)
{
}

float ExtendedKalman::filterData(float dat)
{
    xMid = A * xLast;
    pMid = A * pLast + Q;
    kg = pMid / (pMid + R);
    xNow = xMid + kg * (dat - xMid);
    pNow = (1 - kg) * pMid;
    pLast = pNow;
    xLast = xNow;
    return xNow;
}

float ExtendedKalman::getLastFiltered() const { return xLast; }

void ExtendedKalman::reset()
{
    xNow = 0.0f;
    xMid = 0.0f;
    xLast = 0.0f;
    pNow = 0.0f;
    pMid = 0.0f;
    pLast = 0.0f;
}

}  // namespace algorithms

}  // namespace tap
