#include <gtest/gtest.h>

#include "drivers.hpp"
#include "subsystems/chassis/chassis.hpp"

using ::testing::NiceMock;

TEST(ChassisSubsystemTest, for_chassis_motors_initialize) {
    src::Drivers d;
    src::Chassis::ChassisSubsystem c(&d);

    // ON_CALL(c.leftBackWheel, getEncoderUnwrapped);

#ifdef TARGET_SENTRY
    EXPECT_CALL(c.railWheel, initialize);
#else
    EXPECT_CALL(c.leftBackWheel, initialize);
    EXPECT_CALL(c.leftFrontWheel, initialize);
    EXPECT_CALL(c.rightFrontWheel, initialize);
    EXPECT_CALL(c.rightBackWheel, initialize);
#ifdef SWERVE
    EXPECT_CALL(c.leftBackYaw, initialize);
    EXPECT_CALL(c.leftFrontYaw, initialize);
    EXPECT_CALL(c.rightFrontYaw, initialize);
    EXPECT_CALL(c.rightBackYaw, initialize);
#endif
#endif
    c.initialize();
}