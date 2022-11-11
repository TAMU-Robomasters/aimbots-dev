#include <gmock/gmock.h>

#include "subsystems/shooter/shooter.hpp"

#include "drivers.hpp"

namespace src::mock {
class shooterSubsystemMock : public src::Shooter::ShooterSubsystem {
public:
    shooterSubsystemMock(src::Drivers *drivers);
    virtual ~shooterSubsystemMock();

    MOCK_METHOD(void, initialize, (), (override));
    MOCK_METHOD(void, refresh, (), (override));
    MOCK_METHOD(void, setDesiredOutputs, ());
    MOCK_METHOD(void, calculateShooter, (float RPM_TARGET));
};
}  // namespace src::mock