#pragma once
#include <vector>

#include <utils/math/transform_setup.hpp>

#include "utils/common_types.hpp"
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
    void updateTargetInfo(Vector3f position, uint32_t frameCaptureDelay);

    PlateKinematicState getPlatePrediction(uint32_t dt) const;

    uint32_t getLastFrameCaptureDelay() const { return lastFrameCaptureDelay; }

    bool isLastFrameStale() const;

private:
    src::Drivers* drivers;

    static constexpr float VALID_TIME = 0;  // max elapsed ms before an enemy position entry is invalid

    uint32_t lastFrameCaptureDelay = 0;

    src::Utils::Filters::KinematicKalman XPositionFilter, YPositionFilter, ZPositionFilter;

    // 1s sample, 30ms per sample = 33 samples
    SlidingDFT<float, 30> xDFT;
    bool xDFTValid = false;

    uint32_t lastUpdateTimestamp_uS = 0;
    uint32_t lastFrameCaptureTimestamp_uS = 0;

    float BASE_HEIGHT_THRESHOLD = 0.00;
};

// clang-format off
static constexpr float KF_P[9] = { // Covariance Matrix
    1, 0, 0,
    0, 1, 0,
    0, 0, 1,
};

static constexpr float KF_H[9] = { // Observation Matrix
    1, 0, 0,
    0, 1, 0,
    0, 0, 1,
};

static constexpr float KF_Q[9] = { // Environment Noise Covariance Matrix
    1E-1, 0,   0,
    0,    1E0, 0,
    0,    0,   1E1,
};

static constexpr float KF_R[9] = { // Measurement Noise Covariance Matrix
    1E0, 0,   0,
    0,    1E2, 0,
    0,    0,   1E4,
};
// clang-format on

}  // namespace src::Informants::Vision