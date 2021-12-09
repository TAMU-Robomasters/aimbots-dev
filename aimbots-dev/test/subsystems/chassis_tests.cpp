#include <gtest/gtest.h>
#include "subsystems/chassis.hpp"
#include "drivers.hpp"

TEST(ChassisSubsystemTest, for_chassis_motors_initialize) {
    src::Drivers d;
    Chassis::ChassisSubsystem c(&d);

    // EXPECT_CALL(c.leftBackWheel, initialize());

    c.initialize();
}