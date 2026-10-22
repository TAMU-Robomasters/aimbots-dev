#include "training_motor_subsystem.hpp"

#ifdef TARGET_TRAINING

#include "tap/algorithms/math_user_utils.hpp"

// MCUViewer watch variables for the motor itself.
float motorRpmDisplay = 0.0f;     // what the motor says its speed is
float motorOutputDisplay = 0.0f;  // the voltage command we're sending it
int motorTemperatureDisplay = 0;  // degrees C
bool motorOnlineDisplay = false;  // false = no CAN feedback (check power, wiring, ID, bus)

namespace src::Training {

TrainingMotorSubsystem::TrainingMotorSubsystem(src::Drivers* drivers)
    : tap::control::Subsystem(drivers),
      motor(drivers,
            TRAINING_MOTOR_ID,
            TRAINING_MOTOR_BUS,
            TRAINING_MOTOR_INVERTED,
            "Training GM6020",
            false)  // currentControl = false -> voltage control (CAN ID 0x1FF)
{}

void TrainingMotorSubsystem::initialize() {
    // Registers the motor with the CAN TX handler (so it gets sent) and RX handler (so we get feedback).
    motor.initialize();
    desiredOutput = 0.0f;
    motor.setDesiredOutput(0);
}

// Runs every 2 ms whether or not a command is using the subsystem.
void TrainingMotorSubsystem::refresh() {
    const float output = tap::algorithms::limitVal(desiredOutput, -GM6020_MAX_OUTPUT, GM6020_MAX_OUTPUT);
    // This only stores the value. main.cpp's djiMotorTxHandler.encodeAndSendCanData() puts it on the bus.
    motor.setDesiredOutput(static_cast<int32_t>(output));

    motorRpmDisplay = getMotorRPM();
    motorOutputDisplay = output;
    motorTemperatureDisplay = motor.getTemperature();
    motorOnlineDisplay = motor.isMotorOnline();
}

}  // namespace src::Training

#endif  // TARGET_TRAINING
