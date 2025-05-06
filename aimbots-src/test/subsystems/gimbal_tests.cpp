#include <gtest/gtest.h>

#include "subsystems/gimbal/control/gimbal.hpp"

#include "drivers.hpp"

using ::testing::NiceMock;

TEST(GimbalSubsystemTest, initialize_runs_initialize_on_all_motors) {
    src::Drivers d;
    src::Gimbal::GimbalSubsystem g(&d);

    // ON_CALL(c.leftBackWheel, getEncoderUnwrapped);

#ifdef ALL_SENTRIES
    EXPECT_CALL(g., initialize);
#else

#endif
    g.initialize();
}