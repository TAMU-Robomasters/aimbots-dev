#include <gtest/gtest.h>
#include <vector>

#include "drivers.hpp"
#include "tap/control/subsystem.hpp"
#include "subsystems/shooter/shooter.hpp"

// using ::testing::NiceMock;
TEST(shooter, TEST_CALCULATE_FLYWHEEL){
    tap::Drivers d;
    src::Shooter::ShooterSubsystem c(&d);


    c.initialize();    
    std::vector<float> rpm = c.calculateShooter(100.0f);
    // float rpm [1] = c.calculateShooter(100);

    EXPECT_EQ(rpm[0], 110.0f);
    EXPECT_EQ(rpm[1], 95.0f);

}