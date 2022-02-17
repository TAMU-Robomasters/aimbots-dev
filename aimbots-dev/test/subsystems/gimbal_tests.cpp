#include <gtest/gtest.h>

#include "drivers.hpp"
#include "subsystems/gimbal/gimbal.hpp"

using ::testing::NiceMock;

TEST(GimbalSubsystemTest, initialize_runs_initialize_on_all_motors) {
    src::Drivers d;
    src::Gimbal::GimbalSubsystem g(&d);

    // ON_CALL(c.leftBackWheel, getEncoderUnwrapped);

#ifdef TARGET_SENTRY
    EXPECT_CALL(g., initialize);
#else

#endif
    g.initialize();
}