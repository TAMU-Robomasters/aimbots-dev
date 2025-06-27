#pragma once
#include <vector>

#include <utils/math/transform_setup.hpp>

#include "utils/tools/common_types.hpp"
#include "utils/filters/kinematic_kalman.hpp"
#include "utils/filters/fourth_order_low_pass.hpp"
#include "utils/math/dft_helper.hpp"

namespace src {
class Drivers;
}

namespace src::Informants::Transformers {
class CoordinateFrame;
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

    PlateKinematicState getCurrentPlateEstimation() const;

    uint32_t getLastFrameCaptureDelay() const { return lastFrameCaptureDelay_ms; }

    bool isLastFrameStale() const;

private:
    Vector3f getMeasurementNoiseFromCamera(
        float distanceRelativeToCamera, 
        Transformers::CoordinateFrame* cameraFrame, 
        Transformers::CoordinateFrame* fieldFrame
    );
    
    Vector3f getMeasurementOffsetDueToMotor(
        float distanceRelativeToCamera,
        float yawMotorAngleDisplacement, 
        float pitchMotorAngleDisplacement
    );



    src::Drivers* drivers;

    static constexpr float VALID_TIME = 0;  // max elapsed ms before an enemy position entry is invalid

    VisionTimedPosition currTransformedPosition;

    uint32_t lastFrameCaptureDelay_ms = 0;

    src::Utils::Filters::KinematicKalman XPositionFilter, YPositionFilter, ZPositionFilter;

    uint32_t lastUpdateTimestamp_uS = 0;
    uint32_t lastFrameCaptureTimestamp_uS = 0;

    float BASE_HEIGHT_THRESHOLD = 0.00;
    
    float perviousYawAngle = 0;
    float perviousPitchAngle = 0;

    src::Utils::Filters::targetDistanceFourthOrderLPF targetDistanceFilter;
};

// clang-format off
static constexpr float KF_P[9] = { // Initial covariance matrix. Iniitial estimate of the uncertainty in position, velocity and acceleration
    25, 0, 0,
    0, 31329, 0,
    0, 0, 58522500.0,
};  

static constexpr float KF_H[3] = { // Observation Matrix
    1, 0, 0
};


static constexpr float KF_R = 1E-4;  // Measurement Noise
// clang-format on

static constexpr float accelErrManeuver = 6.0f;
static constexpr float accelErrStable = 0.1f;

}  // namespace src::Informants::Vision