// #include "subsystems/shooter/control/shooter.hpp"
// #include "utils/tools/robot_specific_inc.hpp"

// #ifdef SHOOTER_COMPATIBLE

// namespace src::Shooter {
//     ShooterSubsystem::ShooterSubsystem(tap::Drivers* drivers) : 
//         Subsystem(drivers),
//         drivers(drivers),
//         flywheel1(drivers, SHOOTER_1_ID, SHOOTER_BUS, SHOOTER_1_DIRECTION, "flywheel1"),
//         flywheel2(drivers, SHOOTER_2_ID, SHOOTER_BUS, SHOOTER_2_DIRECTION, "flywheel2"),
//         flywheel1PID(SHOOTER_VELOCITY_PID_CONFIG),
//         flywheel2PID(SHOOTER_VELOCITY_PID_CONFIG)
//     {
//         motors[0] = &flywheel1;
//         motors[1] = &flywheel2;
        
//         velocityPIDs[0] = &flywheel1PID;
//         velocityPIDs[1] = &flywheel2PID;

//         targetRPMs[0] = 0;
//         targetRPMs[1] = 0;

//         desiredOutputs[0] = 0;
//         desiredOutputs[1] = 0;
//     }

//     void ShooterSubsystem::initialize() {
//         motors[0]->initialize();
//         motors[1]->initialize();
//     }

//     void ShooterSubsystem::refresh() {
//         for (int i = 0; i < 2; i++) {
//             updateMotorVelocityPID(i);
//             setDesiredOutputToMotor(i);
//         }
//     }

//     float ShooterSubsystem::getMotorRPM(uint8_t motorIdx) const {
//         return motors[motorIdx]->getShaftRPM();
//     }

//     void ShooterSubsystem::updateMotorVelocityPID(uint8_t motorIdx) {
//         velocityPIDs[motorIdx]->runControllerDerivateError(targetRPMs[motorIdx] - getMotorRPM(motorIdx));
//         setDesiredOutput(motorIdx, velocityPIDs[motorIdx]->getOutput());
//     }

//     void ShooterSubsystem::setTargetRPM(uint8_t motorIdx, float targetRPM) {
//         targetRPMs[motorIdx] = targetRPM;
//     }

//     void ShooterSubsystem::setDesiredOutput(uint8_t motorIdx, float desiredOutput){
//         desiredOutputs[motorIdx] = desiredOutput;
//     }

//     void ShooterSubsystem::setDesiredOutputToMotor(uint8_t motorIdx) {
//         motors[motorIdx]->setDesiredOutput(desiredOutputs[motorIdx]);
//     }
// }

// #endif