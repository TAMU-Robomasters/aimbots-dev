#include "kinematic_informant.hpp"

#include "subsystems/gimbal/control/gimbal.hpp"
#include "subsystems/chassis/control/chassis.hpp"

#include "drivers.hpp"

namespace src::Informants {

KinematicInformant::KinematicInformant(src::Drivers* drivers)
    : drivers(drivers) {}

void KinematicInformant::initialize(float imuFrequency, float imukP, float imukI) {
    drivers->bmi088.initialize(imuFrequency, imukP, imukI);
}

}  // namespace src::Informants