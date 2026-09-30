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

It should finish with `scons: done building targets.` Flash it to the board (see the Google Doc for how). Nothing happens on the board yet, and that's expected.

---

## 1. The command/subsystem framework

The robot's code doesn't have a giant `while` loop full of `if` statements. Instead, [Taproot](https://github.com/uw-advanced-robotics/taproot) (the library our code is built on) gives us a **command-based framework**, the same idea as WPILib for FRC, if you've used that.

### The pieces

| Piece | What it is | Real example in our code |
|---|---|---|
| **Subsystem** | A piece of hardware that only one thing should control at a time. It owns motors/sensors and exposes functions like `setHopperAngle()`. | `HopperSubsystem` |
| **Command** | A *behavior* that uses one or more subsystems for a while: "drive with the joysticks", "open the hopper", "blink the LED". | `OpenHopperCommand`, `StopShooterCommand` |
| **CommandScheduler** | Runs every 2 ms (500 Hz). Calls `refresh()` on every subsystem and `execute()` on every running command. | `drivers->commandScheduler.run()` in `main.cpp:141-144` |
| **Command mapping** | A rule that says "when the remote is in *this* state, run *these* commands". | `HoldCommandMapping`s in `testbench_control.cpp:114-123` |
| **Control file** | One file per robot that creates the subsystems, commands and mappings and registers them. | `testbench_control.cpp`, and now **your** `robots/training/training_control.cpp` |

### Example files worth reading

These are short and simple. Open them next to this guide. (Paths are relative to `aimbots-src/src/`. Line numbers were right when this was written, so if they've drifted, search for the function name.)

| What | File : lines | Why it's a good example |
|---|---|---|
| A tiny subsystem | [`subsystems/hopper/control/hopper.cpp:14-24`](../../aimbots-src/src/subsystems/hopper/control/hopper.cpp) | Constructor, `initialize()` and `refresh()` in about 10 lines. `refresh()` just keeps the servo updated. |
| Its header | [`subsystems/hopper/control/hopper.hpp`](../../aimbots-src/src/subsystems/hopper/control/hopper.hpp) | How a subsystem exposes functions for commands to call. |
| A command that **finishes on its own** | [`subsystems/hopper/basic_commands/open_hopper_command.cpp:6-24`](../../aimbots-src/src/subsystems/hopper/basic_commands/open_hopper_command.cpp) | Does its work in `initialize()`; `isFinished()` returns true once the hopper is open. |
| Its header | [`subsystems/hopper/basic_commands/open_hopper_command.hpp:14-31`](../../aimbots-src/src/subsystems/hopper/basic_commands/open_hopper_command.hpp) | The shape of every command class. Yours looks almost identical. |
| A command that **runs until stopped** | [`subsystems/shooter/basic_commands/stop_shooter_command.cpp:15-30`](../../aimbots-src/src/subsystems/shooter/basic_commands/stop_shooter_command.cpp) | Work happens in `execute()`, and `isFinished()` is always false. Closest to your blink command. |
| A command reading the remote | [`subsystems/chassis/basic_commands/chassis_manual_drive_command.cpp:9-37`](../../aimbots-src/src/subsystems/chassis/basic_commands/chassis_manual_drive_command.cpp) | Joystick → chassis each loop; `end()` stops the motors (safe state!). |
| A control file | [`robots/testbench/testbench_control.cpp`](../../aimbots-src/src/robots/testbench/testbench_control.cpp) | Subsystems (`:59-60`), commands (`:90-111`), mappings (`:114-123`), registration (`:126-162`), and the hookup `main.cpp` calls (`:166-175`). A bit outdated, but the layout is exactly what yours uses. |
| Default commands + more mapping types | [`robots/standard/standard_control.cpp`](../../aimbots-src/src/robots/standard/standard_control.cpp) | `setDefaultCommand` (`:351-353`), `HoldRepeatCommandMapping` (`:310`), `PressCommandMapping` on a keyboard combo (`:324`). |
| Where your control file gets called | [`main.cpp:111`](../../aimbots-src/src/main.cpp) | `src::Control::initializeSubsystemCommands(drivers)`. Every robot's control file defines this one function. |

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

Taproot enforces this strictly. **A command with zero requirements is refused.** The scheduler raises `"Attempting to add a command without subsystem in the scheduler"` and never runs it ([`taproot/src/tap/control/command_scheduler.cpp:269-278`](../../aimbots-src/taproot/src/tap/control/command_scheduler.cpp), inside `addCommand`).

**Why?** It's a design choice Taproot made, not a law of nature. WPILib, for example, lets commands have no requirements. Taproot's scheduler is built around one question: *"who owns which hardware right now?"* It tracks running commands by the subsystems they use (a bitmap with one bit per subsystem), and that's what powers its main features:
- **Interrupting:** starting a command that needs the chassis ends whichever command had the chassis before (`command_scheduler.cpp:281-290`). A command that owns nothing could never be interrupted this way.
- **Default commands:** a subsystem that no command is using automatically gets its default command (like "stop the shooter", `standard_control.cpp:351-353`). That only works if the scheduler knows which subsystems are in use.

A command that owns no subsystem doesn't fit that model, so Taproot assumes it's a bug (the comment in the code calls it "undefined control behavior") and rejects it. In real team code, logic that doesn't drive hardware (reading a sensor, doing math) usually goes in a subsystem's `refresh()` or a driver/informant, not in a command.

So your blink command needs *some* subsystem, and we've given you `TrainingBoardSubsystem`: an empty subsystem that just stands for "this dev board".

> 💡 **Heads up: a subsystem for one LED is weird.**
> On the real robot you'd never wrap a single LED in a subsystem. Subsystems exist to wrap hardware that needs exclusive ownership, which is almost always **motors** (chassis wheels, gimbal yaw/pitch, flywheels, feeder). We're doing it this way *only* so you learn how commands, subsystems, requirements and mappings fit together. You'll write a real motor subsystem in week 3.

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
| `HoldCommandMapping` | Starts the commands when the state starts matching and **ends them as soon as it stops matching**. ← use this one |
| `HoldRepeatCommandMapping` | Like Hold, but if the command finishes while you're still holding, it gets started again. |
| `PressCommandMapping` | Starts the commands when the state starts matching; they keep running until `isFinished()` returns true, even after you let go. |
| `ToggleCommandMapping` | Each time the state **starts** matching, it flips: the 1st time starts the commands, the 2nd time ends them, the 3rd starts them again... |

**More on Toggle**, because it's the confusing one. It only remembers "am I on or off?" in a normal variable in RAM, so a reboot resets it to off. Nothing gets saved. And it only reacts to the moment the state *becomes* true, not to how long you hold it. With a switch that means:

```
flip right switch to UP   → command starts
flip it back to MID       → nothing happens, command keeps running
flip it to UP again       → command stops
```

That's awkward on a switch, so Toggle is mostly used for **keyboard keys**: tap G to turn something on, tap G again to turn it off. (The code is short: `taproot/src/tap/control/toggle_command_mapping.cpp`, the `executeCommandMapping` function.)

Creating the mapping isn't enough. You also have to register it in `registerIOMappings()` with `drivers->commandMapper.addMap(&someName);`. See `testbench_control.cpp:155-157`.

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

`delay_ms` **stops the whole CPU**. While your command sleeps for 250 ms, nothing else runs: no remote reading, no motor updates, no IMU. On the real robot this would stop your chassis from moving and would cause the driver to lose control. **Never block inside a command.**

Instead, check the clock every loop and act only when enough time has passed. Taproot has a helper for that in [`taproot/src/tap/architecture/timeout.hpp`](../../aimbots-src/taproot/src/tap/architecture/timeout.hpp):

```cpp
#include "tap/architecture/timeout.hpp"

tap::arch::MilliTimeout timer;   // doesn't start counting until restart()

timer.restart(500);   // start (or restart) a 500 ms countdown
timer.isExpired();    // true once 500 ms have passed since restart()
timer.stop();         // stop it; isExpired() returns false while stopped
```

The pattern for "do X every N ms" inside `execute()` is: **if expired → do X → restart**.

Real uses in our code:
- [`subsystems/gimbal/basic_commands/gimbal_chase_command.cpp`](../../aimbots-src/src/subsystems/gimbal/basic_commands/gimbal_chase_command.cpp): declared as a member in the `.hpp` (`:61`), started in `initialize()` (`:29`), then `isExpired()` / `restart()` in `execute()` (`:89-92`). "Keep chasing the target for a bit after the camera loses it."
- `main.cpp` uses the sibling class `tap::arch::PeriodicMilliTimer` to run the scheduler every 2 ms: declared at `main.cpp:51`, used at `main.cpp:141`. A *periodic* timer restarts itself every time `execute()` returns true.

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

> 📎 **Taproot has wrappers for this too.** In normal team code you usually wouldn't call modm directly. Taproot wraps the board's pins in `drivers->leds` (`tap::gpio::Leds`), `drivers->digital`, `drivers->analog` and `drivers->pwm` (see [`taproot/src/tap/communication/gpio/`](../../aimbots-src/taproot/src/tap/communication/gpio/)). Under the hood they call the exact same modm functions, using the pin names in [`taproot/src/tap/board/board.hpp`](../../aimbots-src/taproot/src/tap/board/board.hpp). For example, `informants/imu/calibrate_imu_command.cpp:90` turns the green LED on with `drivers->leds.set(tap::gpio::Leds::Green, true)`. This week you're deliberately skipping the wrapper so you learn to go from *datasheet → pin → code* yourself. (Try not to peek at `board.hpp` for the answer until you've found it in the manual).

---

## 4. Finding the pin in the Type C documentation

Being able to go from "I want to use *that* thing on the board" to "it's pin P?? on the MCU" is a core embedded skill. Every new sensor, laser or UART you wire up starts this way.

Document: **[RoboMaster Development Board Type C User Manual (PDF)](https://rm-static.djicdn.com/tem/35228/RoboMaster%20Development%20Board%20Type%20C%20User%20Manual.pdf)**, which has the board layout, interface descriptions and pin tables.

How to look:
1. Find the **board layout / interface diagram** in the user manual and locate the status LED on the board.
2. Find the section that describes that LED. It tells you what kind of LED it is, **which MCU IO pins** drive each color, and **whether HIGH or LOW turns it on**. Write all of that down.
3. Convert the pin name to modm: a pin written as `PXn` in the manual is `modm::platform::GpioXn`.

Pick any color you like.

> 📎 **Where do taproot's pin names come from?** [`aimbots-src/project.xml`](../../aimbots-src/project.xml) is the config file for taproot's code generator (`lbuild`). It picks the board (`rm-dev-board-c`) and lists which header pins we use as digital in/out and PWM (lines 25-31). From that, taproot *generates* `board.hpp`, which gives each pin a name and maps it to the real MCU pin. Notice the names in `project.xml` (`C2`, `B13`, ...) are the labels printed on the board's headers, not the MCU pin names. `board.hpp` translates, e.g. `DigitalOutPinC2 = GpioE11`. The LEDs aren't in `project.xml` because they're built into every Type C board, so taproot's board template always defines them. You'll use this file in week 2.

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
- [ ] `isFinished()`: decide what it should return.

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

### Play around try completing these goals
- Right switch **MID** → LED solid on (a second command, or a parameter).
- Make the blink period a constructor argument and map UP/MID/DOWN to different speeds.
- Cycle through red → green → blue.
- Use the **left** switch too: one switch picks the color, the other picks different speeds.
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
