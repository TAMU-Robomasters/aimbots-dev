#include "chassis_to_imu.hpp"

#include <tap/algorithms/wrapped_float.hpp>

#include "tap/communication/sensors/imu/imu_interface.hpp"

#include "communicators/jetson/jetson_communicator.hpp"
#include "utils/kinematics/kinematic_state_vector.hpp"
#include "utils/tools/common_types.hpp"

#include "utils/tools/robot_specific_defines.hpp"
#include "subsystems/chassis/chassis_constants.hpp"

namespace src {
class Drivers;
}  // namespace src

namespace src::Chassis {
class ChassisSubsystem;
}  // namespace src::Chassis

using namespace src::Utils;

namespace src::Informants {
    
    ChassisToIMU::ChassisToIMU(src::Drivers* drivers) : drivers(drivers) {} // rewrite to go to kinematic informants instead of drivers


}