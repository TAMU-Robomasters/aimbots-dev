#include "enemy_data_conversion.hpp"

#include "drivers.hpp"

namespace src::Informants::Vision {
VisionDataConversion::VisionDataConversion(src::Drivers* drivers)
    : drivers(drivers),
      XPositionFilter(Vector3f(0, 0, 0), KF_P, KF_H, KF_Q, KF_R),
      YPositionFilter(Vector3f(0, 0, 0), KF_P, KF_H, KF_Q, KF_R),
      ZPositionFilter(Vector3f(0, 0, 0), KF_P, KF_H, KF_Q, KF_R)  //
{}

// watchable variables
float cameraOriginXDisplay = 0.0f;
float cameraOriginYDisplay = 0.0f;
float cameraOriginZDisplay = 0.0f;

float cameraXYMagDisplay = 0.0f;
float targetXYMagDisplay = 0.0f;

float targetPositionXUnfilteredDisplay = 0.0f;
float targetPositionYUnfilteredDisplay = 0.0f;
float targetPositionZUnfilteredDisplay = 0.0f;

float targetPositionXDisplay = 0.0f;
float targetPositionYDisplay = 0.0f;
float targetPositionZDisplay = 0.0f;

uint32_t currentTimeDisplay = 0;
uint32_t lastFrameCaptureDisplay = 0;

uint8_t DCBinDisplay = 0;
float xDFTMagDisplay[30];

float dampingValue = 0.999f;
float spinMagnitude = 0.0f;

uint32_t plateTimeOffsetDisplay = 0;

float previousPositionMag = 0;

float untransformedDataPosXDisplay = 0.0;
float untransformedDataPosYDisplay = 0.0;
float untransformedDataPosZDisplay = 0.0;

// gather data, transform data,
void VisionDataConversion::updateTargetInfo(Vector3f position, uint32_t frameCaptureDelay) {
    uint32_t currentTime_uS = tap::arch::clock::getTimeMicroseconds();
    currentTimeDisplay = currentTime_uS;

    lastFrameCaptureDelay = frameCaptureDelay + plateTimeOffsetDisplay;
    drivers->kinematicInformant.mirrorPastRobotFrame(lastFrameCaptureDelay + plateTimeOffsetDisplay);

    src::Informants::Transformers::CoordinateFrame turretFieldFrame =
        drivers->kinematicInformant.getTurretFrames().getFrame(Transformers::TurretFrameType::TURRET_FIELD_FRAME);

    src::Informants::Transformers::CoordinateFrame turretCameraFrame =
        drivers->kinematicInformant.getTurretFrames().getFrame(Transformers::TurretFrameType::TURRET_CAMERA_FRAME);

    lastFrameCaptureTimestamp_uS = currentTime_uS - (frameCaptureDelay * MICROSECONDS_PER_MS);

    VisionTimedPosition currentData{
        .position = position,
        .timestamp_uS = lastFrameCaptureTimestamp_uS,
        // Current time - (how long ago the frame was captured)
    };

    // Enemy Position in meters

    VisionTimedPosition transformedPosition{
        .position = turretCameraFrame.getPointInFrame(turretFieldFrame, currentData.position),
        .timestamp_uS = currentData.timestamp_uS,
    };

    cameraOriginXDisplay = turretCameraFrame.getOrigin().getX();
    cameraOriginYDisplay = turretCameraFrame.getOrigin().getY();
    cameraOriginZDisplay = turretCameraFrame.getOrigin().getZ();

    cameraXYMagDisplay = sqrt(pow2(turretCameraFrame.getOrigin().getX()) + pow2(turretCameraFrame.getOrigin().getY()));
    targetXYMagDisplay = sqrt(pow2(transformedPosition.position.getX()) + pow2(transformedPosition.position.getY()));

    untransformedDataPosXDisplay = currentData.position.getX();
    untransformedDataPosYDisplay = currentData.position.getY();
    untransformedDataPosZDisplay = currentData.position.getZ();

    targetPositionXUnfilteredDisplay = transformedPosition.position.getX();
    targetPositionYUnfilteredDisplay = transformedPosition.position.getY();
    targetPositionZUnfilteredDisplay = transformedPosition.position.getZ();

    float dt = static_cast<float>(currentTime_uS - lastUpdateTimestamp_uS) / MICROSECONDS_PER_SECOND;

    // This is just a preventative measure against bad CV data corrupting the kalman filters.
    // Lower the value if issues continue to happen
    float MAX_DELTA = 25;  //(5 meters)^2

    float currPosMag = sqrt(
        pow(transformedPosition.position.getX(), 2) + pow(transformedPosition.position.getY(), 2) +
        pow(transformedPosition.position.getZ(), 2));  // magnitude of the current position of camera

    if (abs(currPosMag) < MAX_DELTA && transformedPosition.position.getZ() < BASE_HEIGHT_THRESHOLD) {
        XPositionFilter.update(dt, transformedPosition.position.getX());  // transformedData -> transformedPosition
        YPositionFilter.update(dt, transformedPosition.position.getY());
        ZPositionFilter.update(dt, transformedPosition.position.getZ());

        xDFT.damping_factor = dampingValue;

        xDFTValid = xDFT.update(XPositionFilter.getFuturePrediction(0).getX());

        // if (xDFTValid) {
        //     spinMagnitude = 0.0f;
        //     // std::complex<float> DC_bin = xDFT.dft[0];
        //     uint8_t highestMagIndex = 0;
        //     float highestMag = 0;

        //     for (size_t i = 0; i < 30; i++) {
        //         // if (std::abs<float>(xDFT.dft[i]) > highestMag) {
        //         //     highestMag = std::abs<float>(xDFT.dft[i]);
        //         //     highestMagIndex = i;
        //         // }
        //         xDFTMagDisplay[i] = std::abs<float>(xDFT.dft[i]);
        //     }

        //     for (size_t i = 1; i < 29; i++) {
        //         spinMagnitude += std::abs<float>(xDFT.dft[i]);
        //     }

        //     DCBinDisplay = highestMagIndex;
        //     // DC_binDisplay = src::Utils::DFTHelper::getDominantFrequency<float, 30>(xDFT.dft);
        // }

        lastUpdateTimestamp_uS = currentTime_uS;
    }
    previousPositionMag = currPosMag;
}

float targetPositionXFutureDisplay = 0.0f;
float targetVelocityXFutureDisplay = 0.0f;
float targetAccelerationXFutureDisplay = 0.0f;

float targetPositionYFutureDisplay = 0.0f;
float targetVelocityYFutureDisplay = 0.0f;
float targetAccelerationYFutureDisplay = 0.0f;

float targetPositionZFutureDisplay = 0.0f;
float targetVelocityZFutureDisplay = 0.0f;
float targetAccelerationZFutureDisplay = 0.0f;

float predictiondTDisplay = 0.0f;

PlateKinematicState VisionDataConversion::getPlatePrediction(uint32_t dt) const {
    lastFrameCaptureDisplay = lastFrameCaptureTimestamp_uS;

    float totalForwardProjectionTime =
        static_cast<float>(dt + (tap::arch::clock::getTimeMicroseconds() - lastFrameCaptureTimestamp_uS)) /
        MICROSECONDS_PER_SECOND;

    predictiondTDisplay = totalForwardProjectionTime;

    Vector3f xPlate = XPositionFilter.getFuturePrediction(0);  // dt / MICROSECONDS_PER_SECOND
    Vector3f yPlate = YPositionFilter.getFuturePrediction(0);  // dt / MICROSECONDS_PER_SECOND
    Vector3f zPlate = ZPositionFilter.getFuturePrediction(0);  // dt / MICROSECONDS_PER_SECOND

    targetPositionXFutureDisplay = xPlate.getX();
    targetVelocityXFutureDisplay = xPlate.getY();
    targetAccelerationXFutureDisplay = xPlate.getZ();

    targetPositionYFutureDisplay = yPlate.getX();
    targetVelocityYFutureDisplay = yPlate.getY();

    targetAccelerationYFutureDisplay = yPlate.getZ();

    targetPositionZFutureDisplay = zPlate.getX();
    targetVelocityZFutureDisplay = zPlate.getY();

    targetAccelerationZFutureDisplay = zPlate.getZ();

    return PlateKinematicState{
        .position = Vector3f(xPlate.getX(), yPlate.getX(), zPlate.getX()),
        .velocity = Vector3f(xPlate.getY(), yPlate.getY(), zPlate.getY()),
        // .velocity = Vector3f(0, 0, 0),
        .acceleration = Vector3f(xPlate.getZ(), yPlate.getZ(), zPlate.getZ()),
        // .acceleration = Vector3f(0, 0, 0),
        .timestamp_uS = tap::arch::clock::getTimeMicroseconds() + dt,
    };
}

bool frameStaleDisplay = false;
bool VisionDataConversion::isLastFrameStale() const {
    frameStaleDisplay =
        (tap::arch::clock::getTimeMicroseconds() - lastFrameCaptureTimestamp_uS) > (VALID_TIME * MICROSECONDS_PER_SECOND);
    return (tap::arch::clock::getTimeMicroseconds() - lastFrameCaptureTimestamp_uS) > (VALID_TIME * MICROSECONDS_PER_SECOND);
}

}  // namespace src::Informants::Vision