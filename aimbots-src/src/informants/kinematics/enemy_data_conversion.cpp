#include "enemy_data_conversion.hpp"
#include "drivers.hpp"
#include <cmath>

namespace src::Informants::Vision {
VisionDataConversion::VisionDataConversion(src::Drivers* drivers)
    : drivers(drivers),
      XPositionFilter(Vector3f(0, 0, 0), KF_P, KF_H, KF_R, 12.0f, 2.0f),
      YPositionFilter(Vector3f(0, 0, 0), KF_P, KF_H, KF_R, 12.0f, 2.0f),
      ZPositionFilter(Vector3f(0, 0, 0), KF_P, KF_H, KF_R, 12.0f, 2.0f)  //
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

uint32_t plateTimeOffsetDisplay = 1;
uint32_t forwardProjectionOffset_uS = 0;

float previousPositionMag = 0;

float untransformedDataPosXDisplay = 0.0;
float untransformedDataPosYDisplay = 0.0;
float untransformedDataPosZDisplay = 0.0;

// gather data, transform data,
/**
 * @brief plateTImeOffsetDisplay is used to manually add time to frame delay
 */
void VisionDataConversion::updateTargetInfo(Vector3f position, uint32_t frameCaptureDelay) {
    uint32_t currentTime_uS = tap::arch::clock::getTimeMicroseconds();
    currentTimeDisplay = currentTime_uS;

    lastFrameCaptureDelay_ms = frameCaptureDelay + plateTimeOffsetDisplay; 
    drivers->kinematicInformant.mirrorPastRobotFrame(lastFrameCaptureDelay_ms);

    src::Informants::Transformers::CoordinateFrame turretFieldFrame =
        drivers->kinematicInformant.getTurretFrames().getFrame(Transformers::TurretFrameType::TURRET_FIELD_FRAME);

    src::Informants::Transformers::CoordinateFrame turretCameraFrame =
        drivers->kinematicInformant.getTurretFrames().getFrame(Transformers::TurretFrameType::TURRET_CAMERA_FRAME);

    lastFrameCaptureTimestamp_uS = currentTime_uS - (lastFrameCaptureDelay_ms * MICROSECONDS_PER_MS);


    VisionTimedPosition currentData{
        .position = position,
        .timestamp_uS = lastFrameCaptureTimestamp_uS,
        // Current time - (how long ago the frame was captured)
    };

    currentData.position.y = targetDistanceFilter.processSample(currentData.position.y);

    // Enemy Position in meters
    float currentYawAngle = drivers->kinematicInformant.getCurrentFieldRelativeGimbalYawAngleAsWrappedFloat().getWrappedValue();
    float yawMotorAngleDisplacement = currentYawAngle - perviousYawAngle;
    perviousYawAngle = currentYawAngle;

    float currentPitchAngle = drivers->kinematicInformant.getCurrentFieldRelativeGimbalPitchAngleAsWrappedFloat().getWrappedValue();
    float pitchMotorAngleDisplacement = currentPitchAngle - perviousPitchAngle;
    perviousPitchAngle = currentPitchAngle;

    Vector3f measurementOffset = getMeasurementOffsetDueToMotor(currentData.position.y, yawMotorAngleDisplacement, pitchMotorAngleDisplacement);

    VisionTimedPosition transformedPosition{
        .position = turretCameraFrame.getPointInFrame(turretFieldFrame, currentData.position) /*+ measurementOffset*/,
        .timestamp_uS = currentData.timestamp_uS,
    };

    // TODO: get rid of this maybe?
    currTransformedPosition = transformedPosition;

    Vector3f posVecMath = turretCameraFrame.getOrigin() + currentData.position;

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

    targetPositionXDisplay = posVecMath.getX();
    targetPositionYDisplay = posVecMath.getY();
    targetPositionZDisplay = posVecMath.getZ();

    Vector3f measurementNoiseFromCamera = getMeasurementNoiseFromCamera(currentData.position.y, &turretCameraFrame, &turretFieldFrame);

    float dt = static_cast<float>(currentTime_uS - lastUpdateTimestamp_uS) / MICROSECONDS_PER_SECOND;

    // This is just a preventative measure against bad CV data corrupting the kalman filters.
    // Lower the value if issues continue to happen
    float MAX_DELTA = 25;  //(5 meters)^2

    float currPosMag = sqrt(
        pow(transformedPosition.position.getX(), 2) + pow(transformedPosition.position.getY(), 2) +
        pow(transformedPosition.position.getZ(), 2));  // magnitude of the current position of camera

    if (abs(currPosMag) < MAX_DELTA) {
        // This is assuming the robot is always flat on the ground
        float yawMotorAngularVelocity = RPM_TO_RADPS(drivers->kinematicInformant.getYawMotorAngularVelocity(0));
        float pitchMotorAngularVelocity = RPM_TO_RADPS(drivers->kinematicInformant.getPitchMotorAngularVelocity(0));

        XPositionFilter.update(dt, transformedPosition.position.getX(), /*yawMotorAngularVelocity,*/0, measurementNoiseFromCamera.x, currentData.position.y);  // transformedData -> transformedPosition
        YPositionFilter.update(dt, transformedPosition.position.getY(), /*yawMotorAngularVelocity,*/0, measurementNoiseFromCamera.y, currentData.position.y);
        ZPositionFilter.update(dt, transformedPosition.position.getZ(), /*pitchMotorAngularVelocity,*/0, measurementNoiseFromCamera.z, currentData.position.y); 
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

PlateKinematicState VisionDataConversion::getCurrentPlateEstimation() const {
    lastFrameCaptureDisplay = lastFrameCaptureTimestamp_uS;

    float totalForwardProjectionTime =
        static_cast<float>(tap::arch::clock::getTimeMicroseconds() - lastFrameCaptureTimestamp_uS + forwardProjectionOffset_uS) /
        MICROSECONDS_PER_SECOND;

    predictiondTDisplay = totalForwardProjectionTime;

    Vector3f xPlate = XPositionFilter.getFuturePrediction(totalForwardProjectionTime);
    Vector3f yPlate = YPositionFilter.getFuturePrediction(totalForwardProjectionTime);
    Vector3f zPlate = ZPositionFilter.getFuturePrediction(totalForwardProjectionTime);

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
        // .velocity = Vector3f(0, 0, 0),
        .velocity = Vector3f(xPlate.getY(), yPlate.getY(), zPlate.getY()), 
        // .acceleration = Vector3f(0, 0, 0),
        .acceleration = Vector3f(xPlate.getZ(), yPlate.getZ(), zPlate.getZ()),
        .timestamp_uS = tap::arch::clock::getTimeMicroseconds()
    };
}

bool frameStaleDisplay = false;
bool VisionDataConversion::isLastFrameStale() const {
    frameStaleDisplay =
        (tap::arch::clock::getTimeMicroseconds() - lastFrameCaptureTimestamp_uS) > (VALID_TIME * MICROSECONDS_PER_SECOND);
    return (tap::arch::clock::getTimeMicroseconds() - lastFrameCaptureTimestamp_uS) > (VALID_TIME * MICROSECONDS_PER_SECOND);
}

Vector3f VisionDataConversion::getMeasurementNoiseFromCamera(
    // constants were derived from empirical testing and fitting a curve to the data
    float distanceRelativeToCamera, 
    src::Informants::Transformers::CoordinateFrame* cameraFrame, 
    src::Informants::Transformers::CoordinateFrame* fieldFrame) {
    float exponentialTerm = 0.853;
    float constantNoiseFromCamera = 0.001;
    float noiseDueToDistance = constantNoiseFromCamera * exp(exponentialTerm * distanceRelativeToCamera);
    float b = 0.154471344; // scaling factor

    /*x (right left) and z (up down) should have less noise than y (out in) because the y measurement
    directly relies on the camera depth data which gets noiser as the distance increases*/
    float xNoise = constantNoiseFromCamera + b * noiseDueToDistance;
    float yNoise = constantNoiseFromCamera + noiseDueToDistance;
    float zNoise = constantNoiseFromCamera + b * noiseDueToDistance;

    Vector3f cameraNoise = {xNoise, yNoise, zNoise};

    // all the noise above are in terms of the camera reference frame so we need to transform them to the "field" frame
    return cameraFrame->getPointInFrame(*fieldFrame, cameraNoise);
}

Vector3f VisionDataConversion::getMeasurementOffsetDueToMotor(float distanceRelativeToCamera, float yawMotorAngleDisplacement, float pitchMotorAngleDisplacement) {
    //!!! Assumes that the robot is flat on the ground
    float alpha = 0.33f; // rough empirical testing. scaling factor
    float beta = 0.1f; // scaling factor for everything

    float xOffset = 2 * distanceRelativeToCamera * sinf(yawMotorAngleDisplacement / 2.0f);
    float yOffset = alpha * xOffset;
    float zOffset = 2 * distanceRelativeToCamera * sinf(pitchMotorAngleDisplacement / 2.0f);

    return -beta * Vector3f(xOffset, yOffset, zOffset);
}

}  // namespace src::Informants::Vision
