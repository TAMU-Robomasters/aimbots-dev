#include <gtest/gtest.h>

#include "subsystems/chassis/control/chassis.hpp"

#include "drivers.hpp"

using ::testing::NiceMock;

TEST(ChassisSubsystemTest, initialize_runs_initialize_on_all_motors) {
    src::Drivers d;
    src::Chassis::ChassisSubsystem c(&d);

    // ON_CALL(c.leftBackWheel, getEncoderUnwrapped);

#ifdef ALL_SENTRIES
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