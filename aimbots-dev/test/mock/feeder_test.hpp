#ifndef FEEDER_TEST_HPP
#define FEEDER_TEST_HPP

#include <gmock/gmock.h>

#include "drivers.hpp"
#include "subsystems/feeder/feeder.hpp"

namespace src::mock {
class feeder_test : public src::Feeder::FeederSubsystem {
   public:
    feeder_test(src::Drivers *drivers);
    virtual ~feeder_test();

    MOCK_METHOD(void, initialize, (), (override));
    MOCK_METHOD(void, refresh, (), (override));
    MOCK_METHOD(int32_t, setDesiredOutput, ());
};
}  // namespace src::mock

#endif  // CHASSIS_SUBSYSTEM_MOCK_HPP