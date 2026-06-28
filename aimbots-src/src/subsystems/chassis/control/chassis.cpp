#include "subsystems/chassis/control/chassis.hpp"
#include <cmath>

#include "tap/communication/gpio/leds.hpp"

#include "utils/tools/common_types.hpp"

#include "drivers.hpp"

#ifdef CHASSIS_COMPATIBLE

using namespace tap::algorithms;

namespace src::Chassis {

ChassisSubsystem::ChassisSubsystem(src::Drivers* drivers)
    : ChassisSubsystemInterface(drivers),
      drivers(drivers),

      leftBackWheel(drivers, LEFT_BACK_WHEEL_ID, CHASSIS_BUS, false, "Left Back Wheel Motor"),
      leftFrontWheel(drivers, LEFT_FRONT_WHEEL_ID, CHASSIS_BUS, false, "Left Front Wheel Motor"),
      rightFrontWheel(drivers, RIGHT_FRONT_WHEEL_ID, CHASSIS_BUS, false, "Right Front Wheel Motor"),
      rightBackWheel(drivers, RIGHT_BACK_WHEEL_ID, CHASSIS_BUS, false, "Right Back Wheel Motor"),

      leftBackWheelVelPID(CHASSIS_VELOCITY_PID_CONFIG),
      leftFrontWheelVelPID(CHASSIS_VELOCITY_PID_CONFIG),
      rightFrontWheelVelPID(CHASSIS_VELOCITY_PID_CONFIG),
      rightBackWheelVelPID(CHASSIS_VELOCITY_PID_CONFIG),
#ifdef SWERVE
      leftBackYaw(drivers, LEFT_BACK_YAW_ID, CHASSIS_BUS, false, "Left Back Yaw Motor"),
      leftFrontYaw(drivers, LEFT_FRONT_YAW_ID, CHASSIS_BUS, false, "Left Front Yaw Motor"),
      rightFrontYaw(drivers, RIGHT_FRONT_YAW_ID, CHASSIS_BUS, false, "Right Front Yaw Motor"),
      rightBackYaw(drivers, RIGHT_BACK_YAW_ID, CHASSIS_BUS, false, "Right Back Yaw Motor"),
      leftBackYawPosPID(CHASSIS_YAW_PID_CONFIG),
      leftFrontYawPosPID(CHASSIS_YAW_PID_CONFIG),
      rightFrontYawPosPID(CHASSIS_YAW_PID_CONFIG),
      rightBackYawPosPID(CHASSIS_YAW_PID_CONFIG),
#endif
      targetRPMs(Matrix<float, DRIVEN_WHEEL_COUNT, MOTORS_PER_WHEEL>::zeroMatrix()),
      desiredOutputs(Matrix<float, DRIVEN_WHEEL_COUNT, MOTORS_PER_WHEEL>::zeroMatrix()),
      motors(Matrix<DJIMotor*, DRIVEN_WHEEL_COUNT, MOTORS_PER_WHEEL>::zeroMatrix()),
      velocityPIDs(Matrix<SmoothPID*, DRIVEN_WHEEL_COUNT, MOTORS_PER_WHEEL>::zeroMatrix()),
      powerLimiter(
          drivers,
          STARTING_ENERGY_BUFFER,
          ENERGY_BUFFER_LIMIT_THRESHOLD,
          ENERGY_BUFFER_CRIT_THRESHOLD,
          POWER_LIMIT_SAFETY_FACTOR)
//
{
    motors[LB][0] = &leftBackWheel;
    motors[LF][0] = &leftFrontWheel;
    motors[RF][0] = &rightFrontWheel;
    motors[RB][0] = &rightBackWheel;

    velocityPIDs[LB][0] = &leftBackWheelVelPID;
    velocityPIDs[LF][0] = &leftFrontWheelVelPID;
    velocityPIDs[RF][0] = &rightFrontWheelVelPID;
    velocityPIDs[RB][0] = &rightBackWheelVelPID;

    static constexpr float WHEELBASE_HYPOTENUSE = 2 / (WHEELBASE_WIDTH + WHEELBASE_LENGTH);

    wheelVelToChassisVelMat[X][LF] = 1;
    wheelVelToChassisVelMat[X][RF] = 1;
    wheelVelToChassisVelMat[X][LB] = -1;
    wheelVelToChassisVelMat[X][RB] = -1;

    wheelVelToChassisVelMat[Y][LF] = 1;
    wheelVelToChassisVelMat[Y][RF] = -1;
    wheelVelToChassisVelMat[Y][LB] = 1;
    wheelVelToChassisVelMat[Y][RB] = -1;

    wheelVelToChassisVelMat[R][LF] = 1.0f / WHEELBASE_HYPOTENUSE;
    wheelVelToChassisVelMat[R][RF] = 1.0f / WHEELBASE_HYPOTENUSE;
    wheelVelToChassisVelMat[R][LB] = 1.0f / WHEELBASE_HYPOTENUSE;
    wheelVelToChassisVelMat[R][RB] = 1.0f / WHEELBASE_HYPOTENUSE;

    wheelVelToChassisVelMat *= (WHEEL_RADIUS / 4);
    
    #ifdef SWERVE
    // WARNING: these kinematics are for equidistant swerve modules and assume a unit radius. 

    //magic matrix. Found by doing psuedo-inverse(inverse_swerve_kinematics) in a python file somewhere.
    // forward_swerve_kinematics * [BL x, BL y, FL x, FL y, FR x, FR y, BR x, BR y]^T = least squares chassis velocity
    // Frame: x-right positive, y-forward positive, omega CCW positive.
    static_assert(WHEELBASE_WIDTH == WHEELBASE_LENGTH, "These forward swerve kinematics are made only for equidistant modules");
    const float RADIUS = WHEELBASE_WIDTH * M_SQRT2 / 2.0f;

    forward_swerve_kinematics[0][0] = 0.25;
    forward_swerve_kinematics[0][1] = 0.0;
    forward_swerve_kinematics[0][2] = 0.25;
    forward_swerve_kinematics[0][3] = 0.0;
    forward_swerve_kinematics[0][4] = 0.25;
    forward_swerve_kinematics[0][5] = 0.0;
    forward_swerve_kinematics[0][6] = 0.25;
    forward_swerve_kinematics[0][7] = 0.0;
    forward_swerve_kinematics[1][0] = 0.0;
    forward_swerve_kinematics[1][1] = 0.25;
    forward_swerve_kinematics[1][2] = 0.0;
    forward_swerve_kinematics[1][3] = 0.25;
    forward_swerve_kinematics[1][4] = 0.0;
    forward_swerve_kinematics[1][5] = 0.25;
    forward_swerve_kinematics[1][6] = 0.0;
    forward_swerve_kinematics[1][7] = 0.25;
    forward_swerve_kinematics[2][0] =  0.3535533905932738 / RADIUS; // BL x
    forward_swerve_kinematics[2][1] = -0.3535533905932738 / RADIUS; // BL y
    forward_swerve_kinematics[2][2] = -0.3535533905932738 / RADIUS; // FL x
    forward_swerve_kinematics[2][3] = -0.3535533905932738 / RADIUS; // FL y
    forward_swerve_kinematics[2][4] = -0.3535533905932738 / RADIUS; // FR x
    forward_swerve_kinematics[2][5] =  0.3535533905932738 / RADIUS; // FR y
    forward_swerve_kinematics[2][6] =  0.3535533905932738 / RADIUS; // BR x
    forward_swerve_kinematics[2][7] =  0.3535533905932738 / RADIUS; // BR y
    

    // SWERVE ROBOTS
    motors[LB][1] = &leftBackYaw;
    motors[LF][1] = &leftFrontYaw;
    motors[RF][1] = &rightFrontYaw;
    motors[RB][1] = &rightBackYaw;

    velocityPIDs[LB][1] = &leftBackYawPosPID;
    velocityPIDs[LF][1] = &leftFrontYawPosPID;
    velocityPIDs[RF][1] = &rightFrontYawPosPID;
    velocityPIDs[RB][1] = &rightBackYawPosPID;
#endif
}

void ChassisSubsystem::initialize() {
    ForAllChassisMotors(&DJIMotor::initialize);

    setTargetRPMs(0, 0, 0);
    ForAllChassisMotors(&ChassisSubsystem::setDesiredOutput);
}

int refSerialWorkingDisplay = 0;
uint16_t chassisPowerLimitDisplay = 0;

float motorOutputDisplay = 0.0f;
float yawMotorOutputDisplayRF = 0.0f;
float yawMotorOutputDisplayRB = 0.0f;
float yawMotorOutputDisplayLF = 0.0f;
float yawMotorOutputDisplayLB = 0.0f;

float yawMotorAngleDisplayRF = 0.0f;
float yawMotorAngleDisplayRB = 0.0f;
float yawMotorAngleDisplayLF = 0.0f;
float yawMotorAngleDisplayLB = 0.0f;

float velocityMotorAngleDisplayRF = 0.0f;

#ifdef SWERVE
// [0] = x component (RPM * -sin(yaw)), [1] = y component (RPM * cos(yaw))
float lfWheelVelDisplay[2] = {0.0f, 0.0f};
float rfWheelVelDisplay[2] = {0.0f, 0.0f};
float lbWheelVelDisplay[2] = {0.0f, 0.0f};
float rbWheelVelDisplay[2] = {0.0f, 0.0f};
#endif

Matrix<float, 3, 1> ChassisSubsystem::getActualVelocityChassisRelative() {
#ifndef SWERVE
    Matrix<float, DRIVEN_WHEEL_COUNT, 1> wheelVelocities;
    wheelVelocities[LF][0] = leftFrontWheel.getShaftRPM();
    wheelVelocities[RF][0] = rightFrontWheel.getShaftRPM();
    wheelVelocities[LB][0] = leftBackWheel.getShaftRPM();
    wheelVelocities[RB][0] = rightBackWheel.getShaftRPM();
    return wheelVelToChassisVelMat * convertRawRPM(wheelVelocities);
#endif
// NOTE: im pretty sure this funciton is only used for odom so it's only needed for hero
#ifdef TARGET_SENTRY_SWERVE
    Matrix<float, DRIVEN_WHEEL_COUNT * 2, 1> wheelVelocities;
    // Motors are mounted upside down so CW positive. negate angle so CCW is positive
    float left_front_yaw_actual  = -(motors[LF][1]->getInternalEncoder().getPosition().getWrappedValue() - LEFT_FRONT_ANGEL_OFFSET);
    float right_front_yaw_actual = -(motors[RF][1]->getInternalEncoder().getPosition().getWrappedValue() - RIGHT_FRONT_ANGEL_OFFSET);
    float left_back_yaw_actual   = -(motors[LB][1]->getInternalEncoder().getPosition().getWrappedValue() - LEFT_BACK_ANGEL_OFFSET);
    float right_back_yaw_actual  = -(motors[RB][1]->getInternalEncoder().getPosition().getWrappedValue() - RIGHT_BACK_ANGEL_OFFSET);
#endif 
#ifdef TARGET_HERO
    Matrix<float, DRIVEN_WHEEL_COUNT * 2, 1> wheelVelocities;
    // Motors are mounted upside down so CW positive. negate angle so CCW is positive
    float left_front_yaw_actual  = -(motors[LF][1]->getInternalEncoder().getPosition().getWrappedValue() - LEFT_FRONT_YAW_OFFSET);
    float right_front_yaw_actual = -(motors[RF][1]->getInternalEncoder().getPosition().getWrappedValue() - RIGHT_FRONT_YAW_OFFSET);
    float left_back_yaw_actual   = -(motors[LB][1]->getInternalEncoder().getPosition().getWrappedValue() - LEFT_BACK_YAW_OFFSET);
    float right_back_yaw_actual  = -(motors[RB][1]->getInternalEncoder().getPosition().getWrappedValue() - RIGHT_BACK_YAW_OFFSET);
#endif
#ifdef SWERVE
    // All of these velocities are the rotor RPM not shaftRPM
    wheelVelocities[2*LF+X][0] = leftFrontWheel.getInternalEncoder().getShaftRPM()  * -std::sin(left_front_yaw_actual);
    wheelVelocities[2*LF+Y][0] = leftFrontWheel.getInternalEncoder().getShaftRPM()  *  std::cos(left_front_yaw_actual);
    wheelVelocities[2*RF+X][0] = rightFrontWheel.getInternalEncoder().getShaftRPM() * -std::sin(right_front_yaw_actual);
    wheelVelocities[2*RF+Y][0] = rightFrontWheel.getInternalEncoder().getShaftRPM() *  std::cos(right_front_yaw_actual);
    wheelVelocities[2*LB+X][0] = leftBackWheel.getInternalEncoder().getShaftRPM()   * -std::sin(left_back_yaw_actual);
    wheelVelocities[2*LB+Y][0] = leftBackWheel.getInternalEncoder().getShaftRPM()   *  std::cos(left_back_yaw_actual);
    wheelVelocities[2*RB+X][0] = rightBackWheel.getInternalEncoder().getShaftRPM()  * -std::sin(right_back_yaw_actual);
    wheelVelocities[2*RB+Y][0] = rightBackWheel.getInternalEncoder().getShaftRPM()  *  std::cos(right_back_yaw_actual);

    lfWheelVelDisplay[0] = wheelVelocities[2*LF+X][0]; lfWheelVelDisplay[1] = wheelVelocities[2*LF+Y][0];
    rfWheelVelDisplay[0] = wheelVelocities[2*RF+X][0]; rfWheelVelDisplay[1] = wheelVelocities[2*RF+Y][0];
    lbWheelVelDisplay[0] = wheelVelocities[2*LB+X][0]; lbWheelVelDisplay[1] = wheelVelocities[2*LB+Y][0];
    rbWheelVelDisplay[0] = wheelVelocities[2*RB+X][0]; rbWheelVelDisplay[1] = wheelVelocities[2*RB+Y][0];

    static constexpr float ratio = (CHASSIS_GEARBOX_RATIO * 2.0f * WHEEL_RADIUS * M_PI / 60.0f);
    return forward_swerve_kinematics * (ratio * wheelVelocities);
#endif
}

void ChassisSubsystem::refresh() {
    ForAllChassisMotors(&ChassisSubsystem::updateMotorVelocityPID);

    ForAllChassisMotors(&ChassisSubsystem::setDesiredOutput);

    limitChassisPower();

//     motorOutputDisplay = motors[RF][0]->getOutputDesired();
      #ifdef SWERVE
          yawMotorOutputDisplayRF = motors[RF][1]->getInternalEncoder().getEncoder().getWrappedValue();
          yawMotorOutputDisplayRB = motors[RB][1]->getInternalEncoder().getEncoder().getWrappedValue();
          yawMotorOutputDisplayLF = motors[LF][1]->getInternalEncoder().getEncoder().getWrappedValue();
          yawMotorOutputDisplayLB = motors[LB][1]->getInternalEncoder().getEncoder().getWrappedValue();

          yawMotorAngleDisplayRF = motors[RF][1]->getInternalEncoder().getPosition().getWrappedValue();
          yawMotorAngleDisplayRB = motors[RB][1]->getInternalEncoder().getPosition().getWrappedValue();
          yawMotorAngleDisplayLF = motors[LF][1]->getInternalEncoder().getPosition().getWrappedValue();
          yawMotorAngleDisplayLB = motors[LB][1]->getInternalEncoder().getPosition().getWrappedValue();

          velocityMotorAngleDisplayRF = motors[RF][0]->getInternalEncoder().getPosition().getWrappedValue();

      #endif
 }

void ChassisSubsystem::limitChassisPower() {
    float powerLimitFrac = powerLimiter.getPowerLimitRatio();

    if (compareFloatClose(1.0f, powerLimitFrac, 0.001f)) {
        return;
    }

    float totalError = 0.0f;
    for (size_t i = 0; i < DRIVEN_WHEEL_COUNT; i++) {
        totalError += abs(velocityPIDs[i][0]->getError());
    }

    bool totalErrorZero = compareFloatClose(totalError, 0.0f, 0.001f);

    for (size_t i = 0; i < DRIVEN_WHEEL_COUNT; i++) {
        float velocityErrorFrac =
            totalErrorZero ? (1.0f / DRIVEN_WHEEL_COUNT) : (abs(velocityPIDs[i][0]->getError()) / totalError);

        float modifiedPowerLimitFrac = limitVal(DRIVEN_WHEEL_COUNT * powerLimitFrac * velocityErrorFrac, 0.0f, 1.0f);

        motors[i][0]->setDesiredOutput(motors[i][0]->getOutputDesired() * modifiedPowerLimitFrac);
    }
}

float targetRpmDisplay = 0.0f;
float motorRpmDisplay = 0.0f;
void ChassisSubsystem::updateMotorVelocityPID(WheelIndex WheelIdx, MotorOnWheelIndex MotorPerWheelIdx) {
    float err = 0;
    err = targetRPMs[WheelIdx][MotorPerWheelIdx] - motors[WheelIdx][MotorPerWheelIdx]->getShaftRPM();
    if (MotorPerWheelIdx == DRIVER) {
        err = targetRPMs[WheelIdx][MotorPerWheelIdx] - motors[WheelIdx][MotorPerWheelIdx]->getShaftRPM();
    } else if (MotorPerWheelIdx == YAW) {
        if(drivers->remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::UP || drivers->remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::MID){
            err = targetRPMs[WheelIdx][MotorPerWheelIdx] - motors[WheelIdx][MotorPerWheelIdx]->getInternalEncoder().getEncoder().getWrappedValue();
           if(err >4096){err = -(8192-err);}
        }else{
            err = 0.0f;
        }
        if (abs(err) > 4096) {
            int err_int =
                (((-1 * static_cast<int>(err)) / (abs(static_cast<int>(err)))) * (8192 - static_cast<int>(err))) % 8192;
            err = err_int * 1.0f;
        }
    }

    targetRpmDisplay = targetRPMs[LF][MotorPerWheelIdx];
    motorRpmDisplay = motors[LF][MotorPerWheelIdx]->getShaftRPM();

    velocityPIDs[WheelIdx][MotorPerWheelIdx]->runControllerDerivateError(
        err/*,
        motors[WheelIdx][MotorPerWheelIdx]->getTorque()*/);
    desiredOutputs[WheelIdx][MotorPerWheelIdx] = velocityPIDs[WheelIdx][MotorPerWheelIdx]->getOutput();
}

void ChassisSubsystem::setTargetRPMs(float x, float y, float r) {
    setTargetRPMs(
        x,
        y,
        r,
        ChassisSubsystem::getMaxRefWheelSpeed(
            drivers->refSerial.getRefSerialReceivingData(),
            drivers->refSerial.getRobotData().chassis.powerConsumptionLimit));
}

void ChassisSubsystem::setTargetRPMs(float x, float y, float r, float maxWheelSpeed) {
#if defined(SWERVE)
    calculateSwerve(x, y, r, maxWheelSpeed);
#else
    calculateHolonomic(x, y, r, maxWheelSpeed);
#endif
}

void ChassisSubsystem::setDesiredOutput(WheelIndex WheelIdx, MotorOnWheelIndex MotorPerWheelIdx) {
    motors[WheelIdx][MotorPerWheelIdx]->setDesiredOutput(static_cast<int32_t>(desiredOutputs[WheelIdx][MotorPerWheelIdx]));
}

#ifndef SWERVE
float xInputDisplay = 0.0f;
float yInputDisplay = 0.0f;
float rInputDisplay = 0.0f;
int chassisRotateDisplay;
int leftBackRotationRatioDisplay;
int xDisplay;
int yDisplay;

void ChassisSubsystem::calculateHolonomic(float x, float y, float r, float maxWheelSpeed) {
    xInputDisplay = x;
    yInputDisplay = y;
    rInputDisplay = r;
    // get distance from wheel to center of wheelbase
    float wheelbaseCenterDist = sqrtf(pow2(WHEELBASE_WIDTH / 2.0f) + pow2(WHEELBASE_LENGTH / 2.0f));

    // offset gimbal center from center of wheelbase so we rotate around the gimbal
    float leftFrontRotationRatio = modm::toRadian(wheelbaseCenterDist - GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET);
    float rightFrontRotationRatio = modm::toRadian(wheelbaseCenterDist - GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET);
    float leftBackRotationRatio = modm::toRadian(wheelbaseCenterDist + GIMBAL_X_OFFSET - GIMBAL_Y_OFFSET);
    float rightBackRotationRatio = modm::toRadian(wheelbaseCenterDist + GIMBAL_X_OFFSET + GIMBAL_Y_OFFSET);

    float chassisRotateTranslated = modm::toDegree(r) / wheelbaseCenterDist;

    targetRPMs[LF][0] =
        limitVal<float>(x + y + chassisRotateTranslated * leftFrontRotationRatio, -maxWheelSpeed, maxWheelSpeed);
    targetRPMs[RF][0] =
        limitVal<float>(x - y + chassisRotateTranslated * rightFrontRotationRatio, -maxWheelSpeed, maxWheelSpeed);
    targetRPMs[LB][0] =
        limitVal<float>(-x + y + chassisRotateTranslated * leftBackRotationRatio, -maxWheelSpeed, maxWheelSpeed);
    targetRPMs[RB][0] =
        limitVal<float>(-x - y + chassisRotateTranslated * rightBackRotationRatio, -maxWheelSpeed, maxWheelSpeed);

    desiredRotation = r;
    chassisRotateDisplay = chassisRotateTranslated;
    leftBackRotationRatioDisplay = leftBackRotationRatio;
    xDisplay = x;
    yDisplay = y;
}
#endif

#ifdef SWERVE

float left_front_yaw_actual;
float right_front_yaw_actual;
float left_back_yaw_actual;
float right_back_yaw_actual;

int left_front_yaw_db;
int right_front_yaw_db;
int left_back_yaw_db;
int right_back_yaw_db;

int left_front_yaw;
int right_front_yaw;
int left_back_yaw;
int right_back_yaw;

float target_left_front_yaw;
float target_right_front_yaw;
float target_left_back_yaw;
float target_right_back_yaw;

float left_front_drive;
float right_front_drive;
float left_back_drive;
float right_back_drive;

float prev_left_front_yaw;
float prev_right_front_yaw;
float prev_left_back_yaw;
float prev_right_back_yaw;

bool lockWatch;

static inline float wrap0N(float x, float N) {
    float y = std::fmod(x, N);
    if (y < 0) y += N;
    return y;
}

static inline float wrapToPi(float a) {
    return std::atan2(std::sin(a), std::cos(a));
}

static inline float wrap0To2Pi(float a) {
    return wrap0N(a, 2.0f * static_cast<float>(M_PI));
}

static constexpr float kTicksPerRev = 8192.0f; // why wasn't there a variable for this already?
static constexpr float kTwoPi       = 2.0f * static_cast<float>(M_PI);

float ChassisSubsystem::yawToRad(float ticks, int offsetTicks) {
    const float deltaTicks = wrap0N(ticks - static_cast<float>(offsetTicks), kTicksPerRev);
    return deltaTicks * (kTwoPi / kTicksPerRev);
}

static inline float radToTicksFloat(float radians, int offsetTicks) {
    const float ticks = static_cast<float>(offsetTicks) + radians * (kTicksPerRev / kTwoPi);
    return wrap0N(ticks, kTicksPerRev);
}
static inline int   radToTicksInt  (float radians, int offsetTicks) {
    return static_cast<int>(std::lround(radToTicksFloat(radians, offsetTicks)));
}

void ChassisSubsystem::optimizeSwerve(float& targetRPMDrive, float& targetYaw, float currYaw) {
    const float hysteresis = static_cast<float>(M_PI) * 2.0f / 180.0f; // ~2°
    float delta = wrapToPi(targetYaw - currYaw);

    targetYaw = currYaw + delta;

    //flip wheel direction if turn is greater than 90 + hysteresis degrees
    if (std::fabs(delta) > (static_cast<float>(M_PI) / 2.0f + hysteresis)) {
        targetRPMDrive = -targetRPMDrive;
        targetYaw += (delta > 0.0f) ? -static_cast<float>(M_PI) : static_cast<float>(M_PI);
    }

    targetYaw = wrap0To2Pi(targetYaw);
}


float wheelRPMDisplay = 0.0f;

uint16_t watchLFYaw = 0;
uint16_t watchRFYaw = 0;
uint16_t watchLBYaw = 0;
uint16_t watchRBYaw = 0;

float watchLFYawTarget = 0;
float watchRFYawTarget = 0;
float watchLBYawTarget = 0;
float watchRBYawTarget = 0;

// uint16_t watchLFSpeed = 0;

// for tunning PID system through Ozone

float driveVelocityPDebug = 0.0f;
float driveVelocityIDebug = 0.0f;
float driveVelocityDDebug = 0.0f;
bool updateDriveVelocityPIDsDebug = false;

float steeringPositionPDebug = 0.0f;
float steeringPositionIDebug = 0.0f;
float steeringPositionDDebug = 0.0f;
bool updateSteeringPositionPIDsDebug = false;

float maxWheelSpeedDisplay = 0.0f;

float xDisplay = 0.0f;
float yDisplay = 0.0f;
float rotDisplay = 0.0f;

float outDriveRPMDisplay = 0.0;

void ChassisSubsystem::calculateSwerve(float x, float y, float r, float maxWheelSpeed) {
    // if (updateSteeringPositioPIDsDebug) {
    //     for (size_t i = 0; i < 4; i++) {
    //         velocityPIDs[i][1]->pid.setP(steeringPositionPDebug);
    //         velocityPIDs[i][1]->pid.setI(steeringPositionIDebug);
    //         velocityPIDs[i][1]->pid.setD(steeringPositionDDebug);
    //     }

        
    // }


    xDisplay = x;
    yDisplay = y;
    rotDisplay = r;
    
   // updateSteeringVelocityPIDsDebug = false;

    maxWheelSpeedDisplay = maxWheelSpeed;
    lockWatch = false;

    // deadband system to prevent chassis from updating when x,y and r are less than a specified value
    // tune these to prevent yaw motors from always being in "rotation mode" due to chassis follow gimbal pid
    auto deadband = [](float v, float eps) { return (std::fabs(v) < eps) ? 0.0f : v; };
    static constexpr float kXYDeadband = 10.0f;
    static constexpr float kRDeadband  = 100.0f;
    x = deadband(x, kXYDeadband);
    y = deadband(y, kXYDeadband);
    r = deadband(r, kRDeadband);

    // chassis geometry
    const float halfW = WHEELBASE_WIDTH  * 0.5f;
    const float halfL = WHEELBASE_LENGTH * 0.5f;
    const float wheelbaseCenterDist = std::sqrt(halfW * halfW + halfL * halfL);

    // rotational contribution about center
    const float rx =  r * (WHEELBASE_LENGTH / wheelbaseCenterDist);
    const float ry =  r * (WHEELBASE_WIDTH  / wheelbaseCenterDist);
    const float a = -x + rx;
    const float b = -x - rx;
    const float c =  y - ry;
    const float d =  y + ry;

    // yaw values in radians
    left_front_yaw_actual  = yawToRad(motors[LF][1]->getInternalEncoder().getEncoder().getWrappedValue(),  LEFT_FRONT_YAW_OFFSET);
    right_front_yaw_actual = yawToRad(motors[RF][1]->getInternalEncoder().getEncoder().getWrappedValue(), RIGHT_FRONT_YAW_OFFSET);
    left_back_yaw_actual   = yawToRad(motors[LB][1]->getInternalEncoder().getEncoder().getWrappedValue(),  LEFT_BACK_YAW_OFFSET);
    right_back_yaw_actual  = yawToRad(motors[RB][1]->getInternalEncoder().getEncoder().getWrappedValue(), RIGHT_BACK_YAW_OFFSET);

    watchLFYaw = motors[LF][1]->getInternalEncoder().getEncoder().getUnwrappedValue();
    watchRFYaw = motors[RF][1]->getInternalEncoder().getEncoder().getUnwrappedValue();
    watchLBYaw = motors[LB][1]->getInternalEncoder().getEncoder().getUnwrappedValue();
    watchRBYaw = motors[RB][1]->getInternalEncoder().getEncoder().getUnwrappedValue();

    // last angle input is stored here and only gets updated when a new value is greater than the deadband
    static bool  lastYawInit[4]   = {false, false, false, false};
    static float lastCmdYawRad[4] = {0, 0, 0, 0};

    auto ensureInit = [&](int idx, float actual) {
        if (!lastYawInit[idx]) {
            lastCmdYawRad[idx] = wrap0To2Pi(actual);
            lastYawInit[idx]   = true;
        }
    };
    ensureInit(LF, left_front_yaw_actual);
    ensureInit(RF, right_front_yaw_actual);
    ensureInit(LB, left_back_yaw_actual);
    ensureInit(RB, right_back_yaw_actual);

    // holdAngles = true when x,y and r is close to 0
    static constexpr float kModuleVecEps = 1e-3f;
    const bool noTranslate = (std::hypot(x, y) < 1e-4f);
    const bool noRotate    = (std::fabs(r) < 1e-4f);
    const bool holdAngles  = noTranslate && noRotate;

    // FRC implementation of swerve module computation
    auto computeModule = [&](float vx, float vy,
                             int offsetTicks,
                             float currYawRad,
                             int moduleIdx,
                             float& outDriveRPM,
                             float& outSteerTicksFloat,
                             int&   outSteerDbgInt) {
        float targetYawRad;

        if (holdAngles) {
            // set drive to last valid command above deadband limits
            targetYawRad = lastCmdYawRad[moduleIdx];
            outDriveRPM  = 0.0f;
        } else {
            const float mag = std::hypot(vx, vy);
            //removed limiting wheel speed here, should mormalize all wheels later to max speed
            //outDriveRPM = limitVal<float>(mag, -maxWheelSpeed, maxWheelSpeed);
            outDriveRPM = mag;

            if (mag < kModuleVecEps) {
                // Rare single-module degeneracy: hold its last angle
                targetYawRad = lastCmdYawRad[moduleIdx];
                outDriveRPM  = 0.0f;
            } else {
                targetYawRad = std::atan2(vy, vx) + 1.5f * static_cast<float>(M_PI);
                //optimizeSwerve(outDriveRPM, targetYawRad, currYawRad);
                lastCmdYawRad[moduleIdx] = targetYawRad; // update latch
            }
        }
        outDriveRPMDisplay = outDriveRPM;

        outSteerTicksFloat = radToTicksFloat(targetYawRad, offsetTicks);
        outSteerDbgInt     = static_cast<int>(std::lround(outSteerTicksFloat));
    };

    // LF
    computeModule(b, d, LEFT_FRONT_YAW_OFFSET,  left_front_yaw_actual,
                  LF, targetRPMs[LF][0], targetRPMs[LF][1], left_front_yaw_db);

    // RF
    computeModule(b, c, RIGHT_FRONT_YAW_OFFSET, right_front_yaw_actual,
                  RF, targetRPMs[RF][0], targetRPMs[RF][1], right_front_yaw_db);

    // LB
    computeModule(a, d, LEFT_BACK_YAW_OFFSET,   left_back_yaw_actual,
                  LB, targetRPMs[LB][0], targetRPMs[LB][1], left_back_yaw_db);

    // RB
    computeModule(a, c, RIGHT_BACK_YAW_OFFSET,  right_back_yaw_actual,
                  RB, targetRPMs[RB][0], targetRPMs[RB][1], right_back_yaw_db);

    //Normalize drive motor rpm to the max wheel speed
    float largestRPM = 0.0f;
    for(int i = 0; i < 4; i++){
        if(abs(targetRPMs[i][0]) > largestRPM){
            largestRPM = abs(targetRPMs[i][0]);
        }
    }
    if(largestRPM > maxWheelSpeed){
        for(int i = 0; i < 4; i++){
            targetRPMs[i][0] /= largestRPM;
            targetRPMs[i][0] *= maxWheelSpeed;
        }
    }

    watchRBYawTarget = lastCmdYawRad[RB];
    watchLFYawTarget = lastCmdYawRad[LF];
    watchRFYawTarget = lastCmdYawRad[RF];
    watchLBYawTarget = lastCmdYawRad[LB];
}


#endif

void ChassisSubsystem::calculateRail(float x, float maxWheelSpeed) {
    targetRPMs[RAIL][0] = limitVal<float>(x, -maxWheelSpeed, maxWheelSpeed);
}

float ChassisSubsystem::calculateRotationLimitedTranslationalWheelspeed(
    float chassisRotationDesiredWheelspeed,
    float maxWheelSpeed) {
    // what we will multiply x and y speed by to take into account rotation
    float rTranslationalGain = 1.0f;

    // the x and y movement will be slowed by a fraction of auto rotation amount for maximizing power
    // consumption when the wheel rotation speed for chassis rotation is greater than the MIN_ROTATION_THRESHOLD
    if (fabsf(chassisRotationDesiredWheelspeed) > MIN_ROTATION_THRESHOLD) {
        rTranslationalGain =
            pow2(maxWheelSpeed + MIN_ROTATION_THRESHOLD - fabsf(chassisRotationDesiredWheelspeed) / maxWheelSpeed);

        rTranslationalGain = tap::algorithms::limitVal<float>(rTranslationalGain, 0.0f, 1.0f);
    }
    return rTranslationalGain * maxWheelSpeed;
}

bool ChassisSubsystem::getTokyoDrift() const { return tokyoDrift; }
};  // namespace src::Chassis

#endif  // #ifdef CHASSIS_COMPATIBLE
