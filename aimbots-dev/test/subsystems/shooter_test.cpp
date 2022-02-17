#include <gtest/gtest.h>

#include "drivers.hpp"
#include "tap/control/subsystem.hpp"
#include "subsystems/shooter/shooter.hpp"

// using ::testing::NiceMock;
TEST(shooter, TEST_CALCULATE_FLYWHEEL){
    tap::Drivers d;
    src::Shooter::ShooterSubsystem c(&d);
    // src::Shooter::ShooterCommand shooterCMD(&d, &c)
    // c.initialize();

    c.calculateShooter(100.0f);
    // std::cout << "Top RPM: " << rpm[0] << std::endl;
    // float rpm [1] = c.calculateShooter(100);

    EXPECT_EQ(c.targetRPMs[0], 110.0f);
    EXPECT_EQ(c.targetRPMs[1], 95.0f);

}