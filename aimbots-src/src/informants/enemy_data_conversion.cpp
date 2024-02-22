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
float targetPositionXDisplayWithoutCompensation = 0.0f;
float targetPositionYDisplayWithoutCompensation = 0.0f;
float targetPositionZDisplayWithoutCompensation = 0.0f;

float targetPositionXUnfilteredDisplay = 0.0f;
float targetPositionYUnfilteredDisplay = 0.0f;
float targetPositionZUnfilteredDisplay = 0.0f;

float targetPositionXDisplay = 0.0f;
float targetPositionYDisplay = 0.0f;
float targetPositionZDisplay = 0.0f;

float cameraToGimbalXDisplay = -10.0f;
float cameraToGimbalYDisplay = 10.0f;
float cameraToGimbalZDisplay = -10.0f;

float dtDisplay = 0.1f;

uint32_t currentTimeDisplay = 0;
uint32_t lastFrameCaptureDisplay = 0;

uint8_t DCBinDisplay = 0;
float xDFTMagDisplay[30];

float dampingValue = 0.999f;
float spinMagnitude = 0.0f;

float XFilterReturnDisplay = 0.0f;

uint32_t plateTimeOffsetDisplay = 0;

float valueFromCVDisplay = 0.0f;

// gather data, transform data,
void VisionDataConversion::updateTargetInfo(Vector3f position, uint32_t frameCaptureDelay) {
    uint32_t currentTime_uS = tap::arch::clock::getTimeMicroseconds();
    currentTimeDisplay = currentTime_uS;

    if (abs(position.getX()) > 20.0f) {
        return;
    }

    lastFrameCaptureDelay = frameCaptureDelay + plateTimeOffsetDisplay;
    drivers->kinematicInformant.mirrorPastRobotFrame(lastFrameCaptureDelay + plateTimeOffsetDisplay);

    src::Informants::Transformers::CoordinateFrame gimbalFrame =
        drivers->kinematicInformant.getRobotFrames().getFrame(Transformers::FrameType::GIMBAL_FRAME);

    src::Informants::Transformers::CoordinateFrame cameraFrame =
        drivers->kinematicInformant.getRobotFrames().getFrame(Transformers::FrameType::CAMERA_FRAME);

    src::Informants::Transformers::CoordinateFrame ballisticsFrame =
        drivers->kinematicInformant.getRobotFrames().getFrame(Transformers::FrameType::BALLISTICS_FRAME);

    src::Informants::Transformers::CoordinateFrame cameraAtCVUpdateFrame =
        drivers->kinematicInformant.getRobotFrames().getFrame(Transformers::FrameType::CAMERA_AT_CV_UPDATE_FRAME);

    src::Informants::Transformers::CoordinateFrame chassisFrame =
        drivers->kinematicInformant.getRobotFrames().getFrame(Transformers::FrameType::CHASSIS_FRAME);

    src::Informants::Transformers::CoordinateFrame fieldFrame =
        drivers->kinematicInformant.getRobotFrames().getFrame(Transformers::FrameType::FIELD_FRAME);

    lastFrameCaptureTimestamp_uS = currentTime_uS - (frameCaptureDelay * MICROSECONDS_PER_MS);

    VisionTimedPosition currentData{
        .position = position,
        .timestamp_uS = lastFrameCaptureTimestamp_uS,
        // Current time - (how long ago the frame was captured)
    };

    // now that we have enemy position (in METERS), transform to chassis space ! ! !
    // THE DESIGN IS VERY HUMAN-CENTERED. THE ROBOT IS THE CENTER OF THE UNIVERSE. THE ENEMY IS THE CENTER OF THE ROBOT.

    // drivers->kinematicInformant.mirrorPastRobotFrame(27);

    VisionTimedPosition targetPositionWithoutLagCompensation{
        .position = cameraFrame.getPointInFrame(chassisFrame, currentData.position),
        .timestamp_uS = currentData.timestamp_uS,
    };

    VisionTimedPosition transformedData{
        .position = cameraAtCVUpdateFrame.getPointInFrame(chassisFrame, currentData.position),
        .timestamp_uS = currentData.timestamp_uS,
    };

    //TESTING : going directly from camera frame to chassis frame
    // VisionTimedPosition cameraToGimbal{
    //     .position = cameraAtCVUpdateFrame.getPointInFrame(gimbalFrame, currentData.position),
    //     .timestamp_uS = currentData.timestamp_uS,
    // };

    // cameraToGimbalXDisplay = cameraToGimbal.position.getX();
    // cameraToGimbalYDisplay = cameraToGimbal.position.getY();
    // cameraToGimbalZDisplay = cameraToGimbal.position.getZ();

    valueFromCVDisplay = position.getX();

    targetPositionXDisplayWithoutCompensation = targetPositionWithoutLagCompensation.position.getX();
    targetPositionYDisplayWithoutCompensation = targetPositionWithoutLagCompensation.position.getY();
    targetPositionZDisplayWithoutCompensation = targetPositionWithoutLagCompensation.position.getZ();


    targetPositionXUnfilteredDisplay = transformedData.position.getX();
    targetPositionYUnfilteredDisplay = transformedData.position.getY();
    targetPositionZUnfilteredDisplay = transformedData.position.getZ();

    targetPositionXDisplay = transformedData.position.getX();
    targetPositionYDisplay = transformedData.position.getY();
    targetPositionZDisplay = transformedData.position.getZ();

    float dt = static_cast<float>(currentTime_uS - lastUpdateTimestamp_uS) / MICROSECONDS_PER_SECOND;

    dtDisplay = dt;

    XPositionFilter.update(dt, transformedData.position.getX());
    // YPositionFilter.update(dt, transformedData.position.getY());
    // ZPositionFilter.update(dt, transformedData.position.getZ());

    XFilterReturnDisplay = XPositionFilter.getFuturePrediction(0).getX();

    // For testing later
    // XPositionFilter.update(dt, cameraToGimbal.position.getX());
    // YPositionFilter.update(dt, cameraToGimbal.position.getY());
    // ZPositionFilter.update(dt, cameraToGimbal.position.getZ());

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

/**
 * Input dt is how far in the future to predict the action
*/
PlateKinematicState VisionDataConversion::getPlatePrediction(uint32_t dt) const {
    lastFrameCaptureDisplay = lastFrameCaptureTimestamp_uS;

    //This is the total amout of time to project forward in predictions based on the
    //last time CV communicated to here (usually ~300ms)
    float totalForwardProjectionTime =
        static_cast<float>(dt + (tap::arch::clock::getTimeMicroseconds() - lastFrameCaptureTimestamp_uS)) /
        MICROSECONDS_PER_SECOND; 

    predictiondTDisplay = totalForwardProjectionTime;

    //TODO: take cv frame, convert it to gimbal frame
    //predict the location of the panel
    //kinematic shenanigans

    //                          FIX THIS FUNC vvvvvvv
    Vector3f xPlate = XPositionFilter.getFuturePrediction(-0.005); //expected location of the panels
    // Vector3f yPlate = YPositionFilter.getFuturePrediction(0);
    // Vector3f zPlate = ZPositionFilter.getFuturePrediction(0);

    targetPositionXFutureDisplay = xPlate.getX();
    targetVelocityXFutureDisplay = xPlate.getY();
    targetAccelerationXFutureDisplay = xPlate.getZ();

    // targetPositionYFutureDisplay = yPlate.getX();
    // targetVelocityYFutureDisplay = yPlate.getY();
    // targetAccelerationYFutureDisplay = yPlate.getZ();

    // targetPositionZFutureDisplay = zPlate.getX();
    // targetVelocityZFutureDisplay = zPlate.getY();
    // targetAccelerationZFutureDisplay = zPlate.getZ();

    return PlateKinematicState{
        .position = Vector3f(xPlate.getX(), 0/*yPlate.getX()*/, 0/*zPlate.getX()*/),
        // .velocity = Vector3f(xPlate.getY(), yPlate.getY(), zPlate.getY()),
        .velocity = Vector3f(0, 0, 0),
        // .acceleration = Vector3f(xPlate.getZ(), yPlate.getZ(), zPlate.getZ()),
        .acceleration = Vector3f(0, 0, 0),
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