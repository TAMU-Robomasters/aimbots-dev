# Week 3: Spin a GM6020 with a velocity PID

**Goal:** Move a GM6020 motor at a speed proportional to the right joystick's vertical axis on the remote.

Everything is on the **right switch** this week. You'll remap it (section 4):

| Right switch | Runs |
|---|---|
| UP | tuning command (square wave) |
| DOWN | stick → speed command |
| MID / remote off | nothing, so the motor stops |

---

## 0. Setup/Overview

```bash
git fetch && git switch training/<your-name> && git merge origin/training-2026
```
These are the files you will be using in this assignment
```
robots/training/
├── motor/
│   ├── training_motor_subsystem.*   ← given; set the constants at the top
│   ├── velocity_motor_pid.hpp       ← given
│   └── velocity_motor_pid.cpp       ← YOU (step 1)
└── commands/
    ├── velocity_pid_tuning_command.*  ← given except the square wave (step 2)
    ├── velocity_control_command.hpp   ← given
    └── velocity_control_command.cpp   ← YOU (step 3)
robots/training/training_control.cpp   ← YOU: create the motor objects and remap the right switch (sections 4–5)
```


---

## 1. DJI motors and taproot's `DjiMotor`
Have the [GM6020 user manual](gm6020_user_manual.pdf) open while you work through this assignment.
Every DJI motor is on a **CAN bus**:
- **ID**: every motor has an ID that is set through a dip switch on the bottom of the motor. look at pg. 6 of the manual
- **Feedback:** each GM6020 sends angle, RPM, current and temperature about every 1 ms, on CAN ID `0x204 + ID`.
- **Commands:** one 8-byte message carries the outputs for 4 motors. 

Taproot names motors by **feedback ID** (`MOTOR1` = `0x201` … `MOTOR8` = `0x208`):

| GM6020 DIP ID | 1 | 2 | 3 | 4 |
|---|---|---|---|---|
| Taproot | `MOTOR5` | `MOTOR6` | `MOTOR7` | `MOTOR8` |


`tap::motor::DjiMotor` (`taproot/src/tap/motor/dji_motor.hpp`, alias `DJIMotor`) wraps all of this:

```cpp
DJIMotor motor(drivers, MotorID::MOTOR5, CANBus::CAN_BUS1, /*inverted*/ false, "name", /*currentControl*/ false);
```

| Call | Does |
|---|---|
| `initialize()` | Registers with the CAN TX + RX handlers. Nothing works without it. |
| `setDesiredOutput(x)` | **Stores** x. Doesn't send. |
| `getInternalEncoder().getShaftRPM()` | Speed from the last feedback |
| `isMotorOnline()` | Feedback in the last 100 ms? |


### `TrainingMotorSubsystem` (given, `motor/training_motor_subsystem.*`)

Like week 1's `TrainingBoardSubsystem`, except this one really owns hardware: it holds the `DJIMotor`. You never touch the `DJIMotor` yourself; you go through the subsystem.

**The only thing you edit** is the three constants at the top of `training_motor_subsystem.hpp`. Match them to your motor:

```cpp
static constexpr MotorID TRAINING_MOTOR_ID = MotorID::MOTOR5;    // DIP ID 1 → MOTOR5 (table above)
static constexpr CANBus TRAINING_MOTOR_BUS = CANBus::CAN_BUS1;  // the dev board CAN port the cable is plugged into
static constexpr bool TRAINING_MOTOR_INVERTED = false;          // flip if + spins the wrong way
```

**What it does for you:**
- `initialize()` calls the motor's `initialize()` (registers it on CAN) and sets the output to 0.
- `refresh()` runs **every 2 ms**, called by the scheduler whether or not a command is running. It clamps the output to ±25000, hands it to the motor, and updates `motorRpmDisplay`, `motorOutputDisplay`, `motorOnlineDisplay` and `motorTemperatureDisplay` for MCUViewer.

**How your code uses it:** your PID and both commands get a `TrainingMotorSubsystem* motor` in their constructors and call:

| Call | Does |
|---|---|
| `motor->setDesiredOutput(volts)` | Sets the voltage that the next `refresh()` sends |
| `motor->getMotorRPM()` | Measured speed, RPM |
| `motor->isMotorOnline()` | `true` if the motor answered in the last 100 ms |

**Why both commands require it:** their constructors call `addSubsystemRequirement(motor)` (already written). So the scheduler lets only one of them drive the motor at a time.

> ⚠️ It keeps sending the **last** output forever. A command's `end()` must set it to 0.

---

## 2. PID, and taproot's `SmoothPid`

`error = target − current` (RPM), `output = P + I + D` (voltage).

| Term | What it does | Too much |
|---|---|---|
| P = `kp × error` | pushes harder the further off you are | overshoot, buzzing |
| I = `ki × Σ error·dt` | builds up until leftover error is gone |  overshoot |
| D = `kd × d(error)/dt` | helps reduce buzzing| makes buzzing a lot worse (**You should keep at zero for this assignment**) |

- **P alone settles short:** holding a speed takes a steady voltage, and with P only that needs error ≠ 0. I supplies that voltage.
- **`maxOutput`** clamps the output: your safety net.
- **`maxICumulative`** clamps the I term against **windup**. Make it just big enough to hold top speed.

Use the team wrapper `SmoothPID` (`utils/pid/smooth_pid_wrap.hpp`, around `tap::algorithms::SmoothPid`), same as the gimbal:

```cpp
static constexpr SmoothPIDConfig CONFIG = {.kp = 10.0f, .ki = 0.0f, .kd = 0.0f, .maxICumulative = 0.0f, .maxOutput = 12'000.0f};
SmoothPID pid(CONFIG);
float out = pid.runControllerDerivateError(error);  // handles dt and d(error)/dt
pid.pid.reset();                                     // clear I + last error (.pid = the taproot SmoothPid)
pid.pid.setP(kp);                                    // also setI / setD
```

---
## 3. Step 1: `VelocityMotorPid` (`motor/velocity_motor_pid.cpp`)

A cut-down `GimbalFieldRelativeController` (read its `runYawVelocityController()`): one PID, no IMU, no cascade.

- [ ] `initialize()`: `pid.pid.reset()` **and** `pid.lastTime = now`. Otherwise the first `dt` is however long the command was off, and the I term jumps.
- [ ] `setTargetVelocity()`: store it, clamped to ±`MOTOR_MAX_SPEED_RPM` (90).
- [ ] `runVelocityController()`: optional `velocityLimit` clamp → error → PID → `motor->setDesiredOutput()`.
- [ ] `setPidGains()`: set the gains, then `initialize()`.
- [ ] `isOnline()`

Error is `target − current`

---

## 4. Step 2: tune it

**Square wave** (`getSquareWaveTarget()`): `periodMs` and `timeInPeriodMs` (0 → `periodMs`) are already computed. Return +A for the first half of the period and −A for the second. Each edge is a step, and the step response shows how good your gains are.

### `training_control.cpp`

Go top to bottom through the file's sections.

**1. Includes** (next to your week 1–2 includes):
```cpp
#include "robots/training/commands/velocity_pid_tuning_command.hpp"
#include "robots/training/motor/training_motor_subsystem.hpp"
#include "robots/training/motor/velocity_motor_pid.hpp"
```

**2. "Define subsystems here":** create the motor subsystem under `board`. Make exactly **one**: a second one is a second motor with the same ID, and the board halts at boot.
```cpp
TrainingMotorSubsystem motorSubsystem(drivers());
```

**3. The PID and the tuning config**, right under it (the commands below need them). Your PID gets a pointer to the subsystem, because that's what it drives:
```cpp
static constexpr SmoothPIDConfig VELOCITY_PID_CONFIG = {
    .kp = 10.0f, .ki = 0.0f, .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 12'000.0f,  // ~half of max while tuning
};
VelocityMotorPid velocityPid(drivers(), &motorSubsystem, VELOCITY_PID_CONFIG);

static constexpr VelocityTuningConfig TUNING_CONFIG = {.amplitudeRpm = 60.0f, .frequencyHz = 0.5f};
```

**4. "Define commands here":** the tuning command gets the subsystem (to require it and stop it in `end()`) and the PID (to run it):
```cpp
VelocityPidTuningCommand velocityPidTuningCommand(drivers(), &motorSubsystem, &velocityPid, TUNING_CONFIG);
```

**5. "Define command mappings here": remap the right switch.** Your `rightSwitchUp` mapping runs the LED command. Change its command list to the tuning command:
```cpp
HoldCommandMapping rightSwitchUp(
    drivers(),
    {&velocityPidTuningCommand},  // was {&blinkLedCommand}
    RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP));
```
It's already passed to `addMap()` in `registerIOMappings()`, so nothing changes there. Leave your LED and buzzer command objects in place; they just aren't mapped anymore.

**6. Register and initialize the subsystem** in the functions at the bottom:
```cpp
void registerSubsystems(src::Drivers *drivers) {
    drivers->commandScheduler.registerSubsystem(&board);
    drivers->commandScheduler.registerSubsystem(&motorSubsystem);
}

void initializeSubsystems() {
    board.initialize();
    motorSubsystem.initialize();
}
```
- **Forget `registerSubsystem`:** the scheduler never calls `refresh()`, so nothing reaches the motor. It also refuses to run any command that requires this subsystem.
- **Forget `initialize()`:** the motor is never registered on CAN. `motorOnlineDisplay` stays 0.

**MCUViewer:** check that `motorOnlineDisplay` = 1. Then plot `tuningTargetRpmDisplay`, `tuningCurrentRpmDisplay`, `tuningErrorRpmDisplay` and `tuningOutputDisplay`. An output stuck at ±`maxOutput` means the PID is saturated.

**Live gains:** write `velocityPDebug` / `velocityIDebug` / `velocityDDebug`, then `updateVelocityPidDebug = 1`. For the wave, write `tuningAmplitudeRpmDebug` / `tuningFrequencyHzDebug`, then `updateTuningConfigDebug = 1`. **Live values are lost on reboot**, so copy the good ones into `VELOCITY_PID_CONFIG`.

**Procedure:**
1. **P only.** Raise `kp` until the steps rise fast. Back off when it rings. Note it settles short of the target.
2. **I.** Set `maxICumulative` above the output needed to hold 90 RPM (watch `tuningOutputDisplay`), and rebuild. Raise `ki` until the leftover error goes away without big overshoot.
3. **D:** leave it at 0.
4. Recheck with A = 90 and a faster wave.

**Good** = reaches each target in well under 1 s, little overshoot, no ringing, sits on the target.

---

## 5. Step 3: drive it with the stick (`commands/velocity_control_command.cpp`)

- [ ] `initialize()`: get the controller ready.
- [ ] `execute()`: map `drivers->dt7Remote.getRightVertical()` (−1…1) to ±`MAX_SPEED_RPM`, set the target, run the controller, update `stickTargetRpmDisplay` / `stickCurrentRpmDisplay`.
- [ ] `end()`: stop the motor.
- [ ] `training_control.cpp`:
  - Include `"robots/training/commands/velocity_control_command.hpp"`.
  - Under "Define commands here", create it with the **same** subsystem and PID as the tuning command:
    ```cpp
    VelocityControlCommand velocityControlCommand(drivers(), &motorSubsystem, &velocityPid);
    ```
  - Remap `rightSwitchDown`: change its command list from `{&buzzerCommand}` to `{&velocityControlCommand}`. Its `addMap()` is already there.

Both commands share one `motorSubsystem` (one motor) and one `velocityPid`, so the gains you tuned are the gains the stick uses.

---

## 6. Checkoff
- [ ] Right UP: square wave, and your tuned plot.
- [ ] Right DOWN: speed follows the stick, ±90 RPM at the ends.
- [ ] Right MID or remote off: the motor stops.

---

## 7. Problems

| Symptom | Likely cause |
|---|---|
| `motorOnlineDisplay` = 0 | No 24 V, CAN-H/L swapped, wrong `TRAINING_MOTOR_BUS`/`TRAINING_MOTOR_ID` (the status LED blinks green N times = ID N), or `motorSubsystem.initialize()` missing |
| Online, but the motor never moves | `registerSubsystem(&motorSubsystem)` missing, or the mapping still points at the LED/buzzer command |
| Keeps spinning after the switch moves | `end()` doesn't zero the output |
| Buzzes / shakes | `kp` (or `kd`) too high |
| Never reaches the target | P only (add I), `maxICumulative` too small |
