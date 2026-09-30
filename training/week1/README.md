# Week 1: Blink an LED from the remote

**Goal:** when the **right switch** on the remote is **UP**, one of the dev board's LEDs blinks at 2 Hz (250 ms on, 250 ms off). In any other switch position, the LED is off.

It sounds small, but it touches every layer you'll use on the real robot:

1. **The command/subsystem framework.** How robot behavior is organized and tied to the remote.
2. **`tap::arch::MilliTimeout`.** Keeping time without freezing the robot.
3. **modm GPIO.** Driving a microcontroller pin directly.
4. **Reading the Type C documentation.** Figuring out *which* pin to drive.

You'll edit exactly two places: `training_control.cpp` and the files in `commands/`. Every spot you need to change is marked `TODO(week1)`.

---

## 0. Build the starter code first

```bash
cd aimbots-src
pipenv run scons build robot=TRAINING
```

It should finish with `scons: done building targets.` Flash it with `scons run robot=TRAINING`. Nothing happens on the board yet, and that's expected.

---

## 1. The command/subsystem framework

The robot's code doesn't have a giant `while` loop full of `if` statements. Instead, [Taproot](https://github.com/uw-advanced-robotics/taproot) (the library our code is built on) gives us a **command-based framework**, the same idea as WPILib for FRC, if you've used that.

### The pieces

| Piece | What it is | Real example in our code |
|---|---|---|
| **Subsystem** | A piece of hardware that only one thing should control at a time. It owns motors/sensors and exposes functions like `setTargetRPMs()`. | `ChassisSubsystem`: [subsystems/chassis/control/chassis.hpp](../../aimbots-src/src/subsystems/chassis/control/chassis.hpp) |
| **Command** | A *behavior* that uses one or more subsystems for a while: "drive with the joysticks", "spin in place", "blink the LED". | `ChassisManualDriveCommand`: [subsystems/chassis/basic_commands/chassis_manual_drive_command.cpp](../../aimbots-src/src/subsystems/chassis/basic_commands/chassis_manual_drive_command.cpp) |
| **CommandScheduler** | Runs every 2 ms (500 Hz, see `sendMotorTimeout` in `main.cpp`). Calls `refresh()` on every subsystem and `execute()` on every running command. | `drivers->commandScheduler` |
| **Command mapping** | A rule that says "when the remote is in *this* state, run *these* commands". | the `HoldCommandMapping`s in [robots/testbench/testbench_control.cpp](../../aimbots-src/src/robots/testbench/testbench_control.cpp) |
| **Control file** | One file per robot that creates the subsystems, commands and mappings and registers them. | [robots/standard/standard_control.cpp](../../aimbots-src/src/robots/standard/standard_control.cpp), and now **your** `robots/training/training_control.cpp` |

### A command's lifecycle

```
mapping matches ──► initialize()           once, when the command starts
                    execute()  execute()   every 2 ms while it runs
                    ...
isFinished()==true ─┐
   or mapping stops ├► end(interrupted)    once, when the command stops
   or interrupted  ─┘
```

- `initialize()`: set things up (reset timers, set initial outputs).
- `execute()`: do one *small* slice of work and **return quickly**. It runs 500 times a second.
- `isFinished()`: return `true` when the command is done on its own (like "move the arm to 90°"). Commands that should run until the driver lets go (like "drive") return `false`.
- `end(bool interrupted)`: clean up. Leave the hardware in a safe state (motors stopped, LED off).

### Requirements: why the command needs a subsystem

In the constructor, a command calls `addSubsystemRequirement(subsystem)`. That's how the scheduler guarantees that two commands never fight over the same motors: starting a new command that needs the chassis ends whatever command had the chassis before.

Taproot enforces this strictly. **A command with zero requirements is refused**, and the scheduler raises `"Attempting to add a command without subsystem in the scheduler"` (see `CommandScheduler::addCommand` in `taproot/src/tap/control/command_scheduler.cpp`). So your blink command needs *some* subsystem, and we've given you `TrainingBoardSubsystem`: an empty subsystem that just stands for "this dev board".

> 💡 **Heads up: a subsystem for one LED is weird.**
> On the real robot you'd never wrap a single LED in a subsystem. Subsystems exist to wrap hardware that needs exclusive ownership, which is almost always **motors** (chassis wheels, gimbal yaw/pitch, flywheels, feeder). We're doing it this way *only* so you learn how commands, subsystems, requirements and mappings fit together before there are motors that can hurt someone. You'll write a real motor subsystem in week 3.

### Mappings: tying commands to the remote

The DT7 remote has two 3-position switches, `LEFT_SWITCH` and `RIGHT_SWITCH`, each `UP`, `MID` or `DOWN`. The receiver plugs into the board's **DBUS** port, and `main.cpp` already reads it every loop (`drivers->remote.read()`). You only describe *what should happen*:

```cpp
HoldCommandMapping someName(
    drivers(),
    {&someCommand},                         // the command(s) to run
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));
```

| Mapping type | Behavior |
|---|---|
| `HoldCommandMapping` | Starts the commands when the state matches and **ends them as soon as it stops matching**. ← use this one |
| `ToggleCommandMapping` | First match starts the commands, the next match ends them. |
| `PressCommandMapping` | Starts the commands on a match; they run until `isFinished()` returns true. |

Creating the mapping isn't enough. You also have to register it in `registerIOMappings()` with `drivers->commandMapper.addMap(&someName);`. Look at how `testbench_control.cpp` does it.

---

## 2. Timing without blocking: `tap::arch::MilliTimeout`

The obvious way to blink is:

```cpp
// ❌ DON'T
Led::set();
modm::delay_ms(250);
Led::reset();
modm::delay_ms(250);
```

`delay_ms` **stops the whole CPU**. While your command sleeps for 250 ms, nothing else runs: no remote reading, no motor updates, no IMU. On the real robot that means a chassis that keeps driving with nobody in control. **Never block inside a command.**

Instead, check the clock every loop and act only when enough time has passed. Taproot has a helper for that in [`taproot/src/tap/architecture/timeout.hpp`](../../aimbots-src/taproot/src/tap/architecture/timeout.hpp):

```cpp
#include "tap/architecture/timeout.hpp"

tap::arch::MilliTimeout timer;   // doesn't start counting until restart()

timer.restart(500);   // start (or restart) a 500 ms countdown
timer.isExpired();    // true once 500 ms have passed since restart()
timer.stop();         // stop it; isExpired() returns false while stopped
```

The pattern for "do X every N ms" inside `execute()` is: **if expired → do X → restart**.

(`main.cpp` uses the sibling class `tap::arch::PeriodicMilliTimer` to run the scheduler every 2 ms. Go look at it.)

---

## 3. Driving a pin with modm

[modm](https://modm.io) is the hardware library under Taproot. Every pin on the STM32F407 is a type named `modm::platform::Gpio<Port><Number>`. For example, port **B** pin **13** is `modm::platform::GpioB13`. Everything is a static function, so you never create an object:

```cpp
#include "modm/platform.hpp"

using MyPin = modm::platform::GpioB13;   // give it a readable name

MyPin::setOutput();   // configure the pin as an output (do this before using it)
MyPin::set();         // drive it HIGH (3.3 V)
MyPin::reset();       // drive it LOW (0 V)
MyPin::toggle();      // flip it
MyPin::set(true);     // set(bool) works too
```

**Your job:** figure out which `Gpio??` the LED is on (next section).

> 📎 **Taproot has wrappers for this too.** In normal team code you usually wouldn't call modm directly. Taproot wraps the board's pins in `drivers->leds` (`tap::gpio::Leds`), `drivers->digital`, `drivers->analog` and `drivers->pwm` (see [`taproot/src/tap/communication/gpio/`](../../aimbots-src/taproot/src/tap/communication/gpio/)). Under the hood they call the exact same modm functions, using the pin names in [`taproot/src/tap/board/board.hpp`](../../aimbots-src/taproot/src/tap/board/board.hpp). This week you're deliberately skipping the wrapper so you learn to go from *datasheet → pin → code* yourself. Week 2 uses the `tap::gpio` wrappers. (And no peeking at `board.hpp` for the answer until you've found it in the manual. 😉)

---

## 4. Finding the pin in the Type C documentation

Being able to go from "I want to use *that* thing on the board" to "it's pin P?? on the MCU" is a core embedded skill. Every new sensor, laser or UART you wire up starts this way.

Documents:
- **[RoboMaster Development Board Type C User Manual (PDF)](https://rm-static.djicdn.com/tem/35228/RoboMaster%20Development%20Board%20Type%20C%20User%20Manual.pdf)**: board layout, interface descriptions, pin tables.
- **Board schematic**: in DJI's [Development-Board-C-Examples](https://github.com/RoboMaster/Development-Board-C-Examples) repo. The example projects' `main.h` files also show which pins each peripheral uses.

How to look:
1. Find the **board layout / interface diagram** in the user manual and locate the status LED on the board.
2. Find the section that describes that LED. It tells you what kind of LED it is, **which MCU IO pins** drive each color, and **whether HIGH or LOW turns it on**. Write all of that down.
3. Convert the pin name to modm: a pin written as `PXn` in the manual is `modm::platform::GpioXn`.
4. Cross-check against the schematic if you want to be sure. The net will be labeled something like `LED_R/G/B`.

Pick any color you like. Green is the traditional choice.

---

## 5. The assignment

### Step by step

**In `commands/blink_led_command.hpp`:**
- [ ] Add a `tap::arch::MilliTimeout` member to the class.

**In `commands/blink_led_command.cpp`:**
- [ ] Replace the `using Led = ...` TODO with the pin you found in section 4.
- [ ] `initialize()`: set the pin as an output, turn the LED on, and start the timer with `BLINK_PERIOD_MS`.
- [ ] `execute()`: if the timer expired, toggle the LED and restart the timer.
- [ ] `end()`: turn the LED off.
- [ ] `isFinished()`: decide what it should return, and be ready to explain why.

**In `training_control.cpp`:**
- [ ] Create a `BlinkLedCommand`, passing it `drivers()` and `&board`.
- [ ] Create a `HoldCommandMapping` for `RIGHT_SWITCH` / `UP` that runs your command.
- [ ] Register the mapping in `registerIOMappings()`.

Then build, flash and flip the switch.

### Checkoff
- [ ] Compiles with `pipenv run scons build robot=TRAINING` with no new errors or warnings from your files.
- [ ] Right switch UP → LED blinks at ~2 Hz.
- [ ] Right switch MID or DOWN → LED off (including when you flip it away mid-blink).
- [ ] No `delay` anywhere in your command.
- [ ] You can answer:
  - What happens if `isFinished()` returns `true`? Try it!
  - Why can't a command have zero subsystem requirements?
  - What would go wrong on a real robot if `execute()` called `modm::delay_ms(250)`?

### Stretch goals
- Right switch **MID** → LED solid on (a second command, or a parameter).
- Make the blink period a constructor argument and map UP/MID/DOWN to different speeds.
- Cycle through red → green → blue.
- Use the **left** switch too: one switch picks the color, the other picks the pattern.
- Redo the LED with the Taproot wrapper `drivers->leds.set(...)` instead of modm. Read `leds.cpp` first and check what it expects for "on".

---

## 6. Common problems

| Symptom | Likely cause |
|---|---|
| `'Gpio??' is not a member of 'modm::platform'` | Typo in the pin name. It's `GpioH5`, not `GpioPH5` or `Gpio_H5`. |
| Builds, but nothing happens when you flip the switch | You created the mapping but never called `addMap()` on it. Or it's on the wrong switch. Or the remote/receiver isn't bound/powered (the receiver LED should be solid). |
| LED lights for a split second, then goes dark | `isFinished()` returns `true`, so the command ends right after it starts. |
| LED stays on after flipping the switch away | `end()` doesn't turn it off. |
| LED is on when it should be off and vice versa | Check what the manual says about which logic level turns the LED on. |
| Board seems frozen / remote stops responding | You're blocking (`delay`, `while` loop waiting on time) inside a command. |
| `multiple definition of src::Control::initializeSubsystemCommands` | You built without `robot=TRAINING`, or you edited another robot's control file. |
| Errors in files you didn't touch | Build the untouched starter code on your branch. If that also fails, tell a lead. It's not your fault. |
