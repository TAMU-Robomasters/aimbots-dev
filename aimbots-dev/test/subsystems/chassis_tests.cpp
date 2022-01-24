#include <gtest/gtest.h>

#include "drivers.hpp"
#include "subsystems/chassis.hpp"

using ::testing::NiceMock;

TEST(ChassisSubsystemTest, for_chassis_motors_initialize) {
    src::Drivers d;
    src::Chassis::ChassisSubsystem c(&d);

    // ON_CALL(c.leftBackWheel, getEncoderUnwrapped);
    EXPECT_CALL(c.leftBackWheel, initialize);
    EXPECT_CALL(c.leftFrontWheel, initialize);
    EXPECT_CALL(c.rightFrontWheel, initialize);
    EXPECT_CALL(c.rightBackWheel, initialize);

    c.initialize();
}