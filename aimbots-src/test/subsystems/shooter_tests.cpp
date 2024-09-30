#include <gtest/gtest.h>

#include "drivers.hpp"
#include "subsystems/shooter/basic_commands/run_shooter_command.hpp"
#include "subsystems/shooter/control/shooter.hpp"

using ::testing::NiceMock;
TEST(shooter, TEST_CALCULATE_FLYWHEEL) {
    src::Drivers d;
    src::Shooter::ShooterSubsystem c(&d);

    c.calculateShooter(100.0f);

    EXPECT_CALL
    // EXPECT_EQ(c.targetRPMs[0], 110.0f);
    // EXPECT_EQ(c.targetRPMs[1], 95.0f);
}