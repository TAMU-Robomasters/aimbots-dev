#include "ballistics_solver.hpp"
#include "../../taproot/src/tap/algorithms/ballistics.hpp"

namespace src::Informants::Vision {
enum CVState;
}

namespace src::Utils::Ballistics {

BallisticsSolver::BallisticsSolver(src::Drivers *drivers, Vector3f barrelOriginFromGimbalOrigin)
    : drivers(drivers),
      barrelOriginFromGimbalOrigin(barrelOriginFromGimbalOrigin)  //
{}

BallisticsSolver::BallisticsSolution solutionDisplay;
MeasuredKinematicState plateKinematicStateDisplay;

float solveForXDisplay = 0;
float solveForYDisplay = 0;
float solveForZDisplay = 0;

int nothingSeenDisplay = 0;
int jetsonOnlineDisplay = 0;

int failedToFindIntersectionDisplay = 0;

std::optional<BallisticsSolver::BallisticsSolution> BallisticsSolver::solve(std::optional<float> projectileSpeed) {
    nothingSeenDisplay = drivers->cvCommunicator.getLastValidMessage().cvState < src::Informants::Vision::CVState::FOUND;
    jetsonOnlineDisplay = !drivers->cvCommunicator.isJetsonOnline();
    if (!drivers->cvCommunicator.isJetsonOnline() ||
        drivers->cvCommunicator.getLastValidMessage().cvState < src::Informants::Vision::CVState::FOUND) {
        // nothingSeenDisplay = 1;
        return std::nullopt;
    }

    // nothingSeenDisplay = 0;

    // If we have already solved for this target, return the same solution
    if (lastPlatePredictionTime == drivers->cvCommunicator.getLastFoundTargetTime()) {
        return lastBallisticsSolution;
    } else {
        lastPlatePredictionTime = drivers->cvCommunicator.getLastFoundTargetTime();
    }

    //TODO: figure out if we should refactor getPlateState or throw it out
    auto plateKinematicState = drivers->cvCommunicator.getPlateState(0);

    MeasuredKinematicState targetKinematicState = {
        .position = plateKinematicState.position,
        .velocity = plateKinematicState.velocity,
        .acceleration = plateKinematicState.acceleration,
    };

    bulletVelocity = projectileSpeed.value_or(defaultProjectileSpeed);

    solveForXDisplay = targetKinematicState.position.getX();
    solveForYDisplay = targetKinematicState.position.getY();
    solveForZDisplay = targetKinematicState.position.getZ();

    plateKinematicStateDisplay = targetKinematicState;

    lastBallisticsSolution = BallisticsSolution();
    lastBallisticsSolution->horizontalDistanceToTarget = sqrt(pow(targetKinematicState.position.y, 2) + pow(targetKinematicState.position.x, 2));
    failedToFindIntersectionDisplay = 0;
    if (!findTargetProjectileIntersection(
            3,
            &lastBallisticsSolution->pitchAngle,
            &lastBallisticsSolution->yawAngle,
            &lastBallisticsSolution->timeToTarget)) {
        lastBallisticsSolution = std::nullopt;
        failedToFindIntersectionDisplay = 1;
    }

    solutionDisplay = *lastBallisticsSolution;

    return lastBallisticsSolution;
}

float projectedTargetPositionXDisp = 0.0;
float projectedTargetPositionYDisp = 0.0;
float projectedTargetPositionZDisp = 0.0;

bool BallisticsSolver::findTargetProjectileIntersection(
    uint8_t numIterations,
    float *turretPitch,
    float *turretYaw,
    float *projectedTravelTime,
    const float pitchAxisOffset,
    const float maxError,
    const float minDiff
) {

    // Initial guess calculation is based on the target being still, the projectile having no acceleration, and no barrel offset
    // *Assuming target is not moving fast relative to the projectile speed
    // *Assuming projectile speed is fast (pitch angle is small)
    // *Assuming barrel offset is small
    Vector3f initGuess;
    float g = -ACCELERATION_GRAVITY;
    float t = targetKinematicState.position.getLength() / bulletVelocity;
    initGuess.z = t;  // time component
    
    float horizontalDist = sqrt(pow(targetKinematicState.position.x, 2) + pow(targetKinematicState.position.y, 2));
    float pitch = atan((targetKinematicState.position.z - 0.5f * g * t * t) / horizontalDist);
    initGuess.y = pitch;  // pitch component
    
    float yaw = acos(targetKinematicState.position.y / horizontalDist);
    if (targetKinematicState.position.x > 0) {
        yaw *= -1;
    }
    initGuess.x = yaw;  // yaw component

    // Broyden's method implementation
    Matrix3f inverseJacob;
    Vector3f xCurr, xPast, delX, delF, vec;
    float err;

    for (int n = 0; n < numIterations; n++) {
        if (n == 0) {
            inverseJacob = approximateInverseJacobian(initGuess);
            xCurr = initGuess - inverseJacob * ballisticCharacteristicEquation(initGuess);
            xPast = initGuess;
            err = xCurr.getLength();
        } else {
            // Sherman-Morrison Formula
            delX = xCurr - xPast;
            delF = ballisticCharacteristicEquation(xCurr) - ballisticCharacteristicEquation(xPast);
            vec = inverseJacob * delF;
            float denominator = delX * vec; // dot product
            if (compareFloatClose(denominator, 0.0f, 1e-6f)) break;  // Prevent division by zero
            
            inverseJacob = inverseJacob + ((delX - vec).asMatrix() * (delX.asTransposedMatrix() * inverseJacob)) / denominator;
            
            xPast = xCurr;
            xCurr = xCurr - inverseJacob * ballisticCharacteristicEquation(xCurr);

            err = xCurr.getLength();
        }

        // TODO: rethink these conditions
        // ?should we also return true when the err is greater than maxError?
        if (err < maxError && n == numIterations - 1) {
            *turretYaw = xCurr.x;
            *turretPitch = xCurr.y;
            *projectedTravelTime = xCurr.z;
            return true;
        }

        if ((xCurr - xPast).getLength() < minDiff) return false; // There could be numerical instability
    }

    return false;
}

float targetPositionXDisp = 0.0;
float targetPositionYDisp = 0.0;
float targetPositionzDisp = 0.0;

// ! This could maybe have infinity?
Matrix3f BallisticsSolver::approximateInverseJacobian(Vector3f xInit, float stepSize) {
    Vector3f column1 = (ballisticCharacteristicEquation(xInit + Vector3f(stepSize, 0, 0)) - ballisticCharacteristicEquation(xInit)) / stepSize;
    Vector3f column2 = (ballisticCharacteristicEquation(xInit + Vector3f(0, stepSize, 0)) - ballisticCharacteristicEquation(xInit)) / stepSize;
    Vector3f column3 = (ballisticCharacteristicEquation(xInit + Vector3f(0, 0, stepSize)) - ballisticCharacteristicEquation(xInit)) / stepSize;

    float elements[9] = {
        column1.x, column2.x, column3.x,
        column1.y, column2.y, column3.y,
        column1.z, column2.z, column3.z
    };
    
    return inverseMatrix3f(Matrix3f(elements));
}
Matrix3f BallisticsSolver::inverseMatrix3f(Matrix3f matrix) {
    // Calculate determinant
    float det = matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) -
                matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]) +
                matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);

    // TODO: figure out what to return
    // Check if matrix is invertible
    if (compareFloatClose(det, 0.0f, 1e-6f)) {
        // Return identity matrix if not invertible
        return Matrix3f::identityMatrix();
    }

    // Calculate adjugate matrix
    float elements[9] = {
        matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1],
        -(matrix[0][1] * matrix[2][2] - matrix[0][2] * matrix[2][1]),
        matrix[0][1] * matrix[1][2] - matrix[0][2] * matrix[1][1],
        
        -(matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]),
        matrix[0][0] * matrix[2][2] - matrix[0][2] * matrix[2][0],
        -(matrix[0][0] * matrix[1][2] - matrix[0][2] * matrix[1][0]),
        
        matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0],
        -(matrix[0][0] * matrix[2][1] - matrix[0][1] * matrix[2][0]),
        matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]
    };

    return Matrix3f(elements) / det;
}

Vector3f BallisticsSolver::ballisticCharacteristicEquation(Vector3f x) {
    // x = [yaw, pitch, time]
    Vector3f a_t = targetKinematicState.acceleration;
    Vector3f v_t = targetKinematicState.velocity;
    Vector3f p_t = targetKinematicState.position;
    Vector3f b = barrelOriginFromGimbalOrigin;
    float s = bulletVelocity;
    float g = -ACCELERATION_GRAVITY;  // gravity acceleration vector
    
    float f_x = b.x * cos(x.x) - sin(x.x) * (b.y * cos(x.y) - b.z * sin(x.y)) - 
                s * x.z * sin(x.x) * cos(x.y) - p_t.x - v_t.x * x.z - 0.5f * a_t.x * x.z * x.z;
                
    float f_y = b.x * sin(x.x) + cos(x.x) * (b.y * cos(x.y) - b.z * sin(x.y)) + 
                s * x.z * cos(x.x) * cos(x.y) - p_t.y - v_t.y * x.z - 0.5f * a_t.y * x.z * x.z;
                
    float f_z = b.y * sin(x.y) + b.z * cos(x.y) + s * x.z * sin(x.y) + 
                0.5f * (g - a_t.z) * x.z * x.z - p_t.z - v_t.z * x.z;
    
    return Vector3f(f_x, f_y, f_z);
}

}  // namespace src::Utils::Ballistics