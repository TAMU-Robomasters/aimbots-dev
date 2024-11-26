#include "kinematic_informant.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "utils/tools/common_types.hpp"
#include "drivers.hpp"

#include "utils/tools/robot_specific_defines.hpp"

namespace src::Informants {

KinematicInformant::KinematicInformant(src::Drivers* drivers)
    : drivers(drivers)

#ifndef TARGET_TURRET
      ,
      // chassisKFOdometry(-2.830f, -0.730f)
      chassisKFOdometry(3.1f, 3.5f)
#warning "don't hardcode these values"
#endif
{
}

void KinematicInformant::initialize(float imuFrequency, float imukP, float imukI) {
    drivers->bmi088.initialize(imuFrequency, imukP, imukI);
}

}  // namespace src::Informants