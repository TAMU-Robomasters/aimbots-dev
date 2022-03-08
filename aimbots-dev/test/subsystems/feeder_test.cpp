#include <gtest/gtest.h>

#include "drivers.hpp"
#include "subsystems/feeder/feeder.hpp"

// using ::testing::NiceMock;

TEST(feeder_test, run_feeder_motor_with_rpm_zero) {
    src::Drivers d;
    src::Feeder::FeederSubsystem c(&d);

    EXPECT_CALL(c.feederMotor, initialize);
    c.initialize();
    EXPECT_TRUE(0 == c.setDesiredOutput(0));
}