#ifndef TARGET_ENGINEER

#include "tap/algorithms/math_user_utils.hpp"

#include "modm/architecture/interface/register.hpp"
#include "modm/math/utils.hpp"

namespace src::communicators::I2C::power_comm {
enum Register : uint8_t{
    DEVICE_ADRESS = 0x6A,
    WHO_AM_I = 0x0F,
    
}
}

#endif