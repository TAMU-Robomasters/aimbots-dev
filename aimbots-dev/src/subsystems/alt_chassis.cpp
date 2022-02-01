#include "subsystems/chassis/chassis.hpp"

#include <functional>

// 
// SWERVE
// 
#ifdef SWERVE
    namespace src::Chassis{

        template <class... Args>
        void ChassisSubsystem::ForChassisMotors(void (DJIMotor::*func)(Args...), Args... args){
            for (auto i = 0; i < DRIVEN_WHEEL_COUNT; i++) {
                (motors[i][0]->*func)(args...);
                (motors[i][1]->*func)(args...);
            }
        }

        ChassisSubsystem::ChassisSubsystem(tap::Drivers* drivers) :
            ChassisSubsystemInterface(drivers),
            leftBackYaw(drivers, LEFT_BACK_YAW_ID, CHAS_BUS, false, "Left Back Yaw Motor"),
            leftFrontYaw(drivers, LEFT_FRONT_YAW_ID, CHAS_BUS, false, "Left Front Yaw Motor"),
            rightFrontYaw(drivers, RIGHT_FRONT_YAW_ID, CHAS_BUS, false, "Right Front Yaw Motor"),
            rightBackYaw(drivers, RIGHT_BACK_YAW_ID, CHAS_BUS, false, "Right Back Yaw Motor"),
            targetRPMs(Matrix<float, DRIVEN_WHEEL_COUNT, MOTORS_PER_WHEEL>::zeroMatrix()),
            motors(Matrix<DJIMotor*, DRIVEN_WHEEL_COUNT, MOTORS_PER_WHEEL>::zeroMatrix()),
            wheelLocationMatrix()
        {
            motors[LB][0] = &leftBackWheel;
            motors[LF][0] = &leftFrontWheel;
            motors[RF][0] = &rightFrontWheel;
            motors[RB][0] = &rightBackWheel;

            wheelLocationMatrix[0][0] = -1.0f;
            wheelLocationMatrix[0][1] = 1.0f;
            wheelLocationMatrix[0][2] = WHEELBASE_WIDTH + WHEELBASE_LENGTH;
            wheelLocationMatrix[1][0] = 1.0f;
            wheelLocationMatrix[1][1] = 1.0f;
            wheelLocationMatrix[1][2] = -(WHEELBASE_WIDTH + WHEELBASE_LENGTH);
            wheelLocationMatrix[2][0] = -1.0f;
            wheelLocationMatrix[2][1] = 1.0f;
            wheelLocationMatrix[2][2] = WHEELBASE_WIDTH + WHEELBASE_LENGTH;
            wheelLocationMatrix[3][0] = 1.0f;
            wheelLocationMatrix[3][1] = 1.0f;
            wheelLocationMatrix[3][2] = -(WHEELBASE_WIDTH + WHEELBASE_LENGTH);

            // SWERVE ROBOTS
            motors[LB][1] = &leftBackYaw;
            motors[LF][1] = &leftFrontYaw;
            motors[RF][1] = &rightFrontYaw;
            motors[RB][1] = &rightBackYaw;
        }

        void ChassisSubsystem::initialize() {
            ForChassisMotors(&DJIMotor::initialize);
            setDesiredOutputs(0, 0, 0);
        }

        void ChassisSubsystem::refresh() {
            // update motor rpm based on the robot type?
        }

        void ChassisSubsystem::setDesiredOutputs(float x, float y, float r) {
            // do the calculateSwerve(x, y, r) inline here
        }

    };  // namespace src::Chassis


// 
// 
// NOT SWERVE
// 
// 
#elif defined(TARGET_SENTRY)
    namespace src::Chassis{

        template <class... Args>
        void ChassisSubsystem::ForChassisMotors(void (DJIMotor::*func)(Args...), Args... args){
            for (auto i = 0; i < DRIVEN_WHEEL_COUNT; i++){
                (motors[i][0]->*func)(args...);
            }
        }

        ChassisSubsystem::ChassisSubsystem( tap::Drivers* drivers) :
            ChassisSubsystemInterface(drivers),
            railWheel(drivers, RAIL_WHEEL_ID, CHAS_BUS, false, "Rail Motor"),
            targetRPMs(Matrix<float, DRIVEN_WHEEL_COUNT, MOTORS_PER_WHEEL>::zeroMatrix()),
            motors(Matrix<DJIMotor*, DRIVEN_WHEEL_COUNT, MOTORS_PER_WHEEL>::zeroMatrix()),
            wheelLocationMatrix()
        {
            motors[RAIL][0] = &railWheel;
        }

        void ChassisSubsystem::initialize() {
            ForChassisMotors(&DJIMotor::initialize);
            setDesiredOutputs(0, 0, 0);
        }

        void ChassisSubsystem::refresh() {
            // update motor rpm based on the robot type?
        }

        void ChassisSubsystem::setDesiredOutputs(float x, float y, float r) {
            // do the calculateRail(x) inline here
        }

    };  // namespace src::Chassis

// 
// 
// NOT SWERVE OR SENTRY (Mecanum)
// 
// 
#else
    namespace src::Chassis{

        template <class... Args>
        void ChassisSubsystem::ForChassisMotors(void (DJIMotor::*func)(Args...), Args... args){
            for (auto i = 0; i < DRIVEN_WHEEL_COUNT; i++){
                (motors[i][0]->*func)(args...);
            }
        }

    ChassisSubsystem::ChassisSubsystem(tap::Drivers* drivers)
        : ChassisSubsystemInterface(drivers),
        leftBackWheel(drivers, LEFT_BACK_WHEEL_ID, CHAS_BUS, false, "Left Back Wheel Motor"),
        leftFrontWheel(drivers, LEFT_FRONT_WHEEL_ID, CHAS_BUS, false, "Left Front Wheel Motor"),
        rightFrontWheel(drivers, RIGHT_FRONT_WHEEL_ID, CHAS_BUS, false, "Right Front Wheel Motor"),
        rightBackWheel(drivers, RIGHT_BACK_WHEEL_ID, CHAS_BUS, false, "Right Back Wheel Motor"),
        targetRPMs(Matrix<float, DRIVEN_WHEEL_COUNT, MOTORS_PER_WHEEL>::zeroMatrix()),
        motors(Matrix<DJIMotor*, DRIVEN_WHEEL_COUNT, MOTORS_PER_WHEEL>::zeroMatrix()),
        wheelLocationMatrix()
    //
    {
        motors[LB][0] = &leftBackWheel;
        motors[LF][0] = &leftFrontWheel;
        motors[RF][0] = &rightFrontWheel;
        motors[RB][0] = &rightBackWheel;

        wheelLocationMatrix[0][0] = -1.0f;
        wheelLocationMatrix[0][1] = 1.0f;
        wheelLocationMatrix[0][2] = WHEELBASE_WIDTH + WHEELBASE_LENGTH;
        wheelLocationMatrix[1][0] = 1.0f;
        wheelLocationMatrix[1][1] = 1.0f;
        wheelLocationMatrix[1][2] = -(WHEELBASE_WIDTH + WHEELBASE_LENGTH);
        wheelLocationMatrix[2][0] = -1.0f;
        wheelLocationMatrix[2][1] = 1.0f;
        wheelLocationMatrix[2][2] = WHEELBASE_WIDTH + WHEELBASE_LENGTH;
        wheelLocationMatrix[3][0] = 1.0f;
        wheelLocationMatrix[3][1] = 1.0f;
        wheelLocationMatrix[3][2] = -(WHEELBASE_WIDTH + WHEELBASE_LENGTH);

    }

    void ChassisSubsystem::initialize() {
        ForChassisMotors(&DJIMotor::initialize);
        setDesiredOutputs(0, 0, 0);
    }

    void ChassisSubsystem::refresh() {
        // update motor rpm based on the robot type?
    }

    void ChassisSubsystem::setDesiredOutputs(float x, float y, float r) {
        calculateMecanum(x, y, r);
    }

};  // namespace src::Chassis