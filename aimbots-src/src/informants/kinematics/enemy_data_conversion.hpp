#pragma once
#include <vector>

#include <utils/math/transform_setup.hpp>

#include "utils/tools/common_types.hpp"
#include "utils/filters/kinematic_kalman.hpp"
#include "utils/math/dft_helper.hpp"

namespace src {
class Drivers;
}
namespace src::Informants::Vision {

struct VisionTimedPosition {
    Vector3f position;
    uint32_t timestamp_uS;
};
struct PlateKinematicState {
    Vector3f position;
    Vector3f velocity;
    Vector3f acceleration;
    uint32_t timestamp_uS;  // time that 'best guess' was made
};

class VisionDataConversion {
public:
    VisionDataConversion(src::Drivers* drivers);
    ~VisionDataConversion() = default;

    /**
     * @brief Gets latest valid enemy target data from CV, converts it to a chassis-relative kinematic state, and filters it.
     * Should be called on every CV update.
     */
    void updateTargetInfo(Vector3f position);

    PlateKinematicState getCurrentPlateEstimation() const;

    uint32_t getLastFrameCaptureDelay() const { return lastFrameCaptureDelay; }

    bool isLastFrameStale() const;

private:
    src::Drivers* drivers;

    static constexpr float VALID_TIME = 0;  // max elapsed ms before an enemy position entry is invalid

    VisionTimedPosition currTransformedPosition;

    uint32_t lastFrameCaptureDelay = 0;

    src::Utils::Filters::KinematicKalman XPositionFilter, YPositionFilter, ZPositionFilter;

    uint32_t lastUpdateTimestamp_uS = 0;
    uint32_t lastFrameCaptureTimestamp_uS = 0;

    float BASE_HEIGHT_THRESHOLD = 0.00;
};

// clang-format off
static constexpr float KF_P[9] = { // Covariance Matrix. Low uncertainty in posiiton, high uncertainty in velocity and acceleration
    1E-4, 0, 0,
    0, 0.031329, 0,
    0, 0, 234.3961,
};  

static constexpr float KF_H[3] = { // Observation Matrix
    1, 0, 0
};


static constexpr float KF_R = 1E-4;  // Measurement Noise Covariance Matrix
// clang-format on

static constexpr float AccelErr = 4.0f;

}  // namespace src::Informants::Vision