#pragma once
#include <cstddef>

#include "utils/common_types.hpp"

namespace src::Utils {

template <size_t PROJECTILE_SPEED_HISTORY_SIZE>
class ProjectileLaunchSpeedPredictor {
public:
    ProjectileLaunchSpeedPredictor() = default;
    ~ProjectileLaunchSpeedPredictor() = default;

    inline std::optional<float> getPredictedLaunchSpeed() const {
        if (projectileSpeedHistory.isEmpty()) {
            return lastAllowedProjectileSpeed;
        }
        return pastProjectileVelocitySum / projectileSpeedHistory.getSize();
    }

    void updatePredictedLaunchSpeed(float lastProjectileSpeed, float allowedLaunchSpeed, uint32_t launchDataTimestamp) {
        if (!tap::algorithms::compareFloatClose(allowedLaunchSpeed, lastAllowedProjectileSpeed, 1E-5)) {
            projectileSpeedHistory.clear();
            pastProjectileVelocitySum = 0;
            lastAllowedProjectileSpeed = allowedLaunchSpeed;
        }

        if (previousLaunchDataReceivedTimestamp != launchDataTimestamp) {
            if (projectileSpeedHistory.isFull()) {
                pastProjectileVelocitySum -= projectileSpeedHistory.getFront();
                projectileSpeedHistory.removeFront();
            }

            // Limit the reported projectile speed to the realistic min/maximum incase reported speed is faulty
            const float limitedProjectileSpeed = std::clamp(lastProjectileSpeed, 0.0f, THEORETICAL_MAX_LAUNCH_SPEED);

            pastProjectileVelocitySum += limitedProjectileSpeed;
            projectileSpeedHistory.append(limitedProjectileSpeed);

            previousLaunchDataReceivedTimestamp = launchDataTimestamp;
        }
    }

private:
    Deque<float, PROJECTILE_SPEED_HISTORY_SIZE> projectileSpeedHistory;
    float pastProjectileVelocitySum = 0;  // Keeps sum of projectile speeds in history queue

    float lastAllowedProjectileSpeed = 0.0f;
    float THEORETICAL_MAX_LAUNCH_SPEED = 45.0f;

    uint32_t previousLaunchDataReceivedTimestamp = 0;
};

}  // namespace src::Utils