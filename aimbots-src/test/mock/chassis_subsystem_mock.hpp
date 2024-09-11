#ifndef CHASSIS_SUBSYSTEM_MOCK_HPP
#define CHASSIS_SUBSYSTEM_MOCK_HPP

#include <gmock/gmock.h>

#include "drivers.hpp"
#include "subsystems/chassis/control/chassis.hpp"

namespace src::mock {
class chassisSubsystemMock : public src::Chassis::ChassisSubsystem {
   public:
    chassisSubsystemMock(src::Drivers *drivers);
    virtual ~chassisSubsystemMock();

    MOCK_METHOD(void, initialize, (), (override));
    MOCK_METHOD(void, refresh, (), (override));
    MOCK_METHOD(int, getNumChassisMotors, (), (const, override));
    MOCK_METHOD(int16_t, getLeftFrontRpmActual, (), (const, override));
    MOCK_METHOD(int16_t, getLeftBackRpmActual, (), (const, override));
    MOCK_METHOD(int16_t, getRightFrontRpmActual, (), (const, override));
    MOCK_METHOD(int16_t, getRightBackRpmActual, (), (const, override));
};
}  // namespace src::mock

#endif  // CHASSIS_SUBSYSTEM_MOCK_HPP