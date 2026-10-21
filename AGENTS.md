# AGENTS.md

Notes for agents and people branching off `feature/dm-mc02-standard`: the Taproot develop upgrade and the move to the Damiao DM-MC02 (STM32H723, Cortex-M7) board.

## Build

```bash
uv run scripts/setup_tools.py        # once: xPack arm-none-eabi-gcc 14.2.1 + OpenOCD into .tools/
cd aimbots-src
uv run scons build robot=SENTRY_SWERVE   # no TARGET_ prefix
```

- **C++23 is required.** Taproot develop builds with `-std=c++23`, so gcc 10 (old Homebrew / GNU Arm Embedded 10.3) fails with `unrecognized command-line option '-std=c++23'`. SConstruct puts the bin dirs in `.tools/path.txt` ahead of `PATH`.
- **uv replaced pipenv/poetry.** `pyproject.toml` + `uv.lock`; Python 3.12 (`.python-version`), SCons 4.7.0, lbuild 1.22.0 (needs Python >= 3.11).
- Regenerate Taproot with `uv run lbuild build` in `aimbots-src/` (source: the `taproot` submodule, `kahzmic/taproot-modm-upgrade` branch `dm-mc02/p3-fdcan`).

### Robot build status

| Builds | Fails for Taproot reasons | Fails for other reasons (missing per-robot constants) |
|---|---|---|
| `STANDARD_BLASTOISE`, `SENTRY_SWERVE`, `HERO` | `AERIAL`, `ENGINEER`, `DART` (command mappings, see below) | `STANDARD_SQUIRTLE`, `SENTRY_BRAVO`, `TURRET`, `CVTEST_*`, plus `CAMERA_*_OFFSET` for `ENGINEER`/`DART` |

`STANDARD_2023` and `STANDARD_2025` were not checked.

## Breaking API changes

**`CommandMapper::addMap()` takes `std::unique_ptr<CommandMapping>`** and mappings take a `const RemoteMapState *` (it keeps the pointer). Pattern used in `standard_control.cpp`, `sentry_control.cpp`, `hero_control.cpp`:

```cpp
#include "tap/control/remote_map_state.hpp"

// Each mapping keeps a pointer to its RemoteMapState, so the states must outlive the mappings.
RemoteMapState leftSwitchUpState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP);

auto leftSwitchUp = std::make_unique<HoldCommandMapping>(
    drivers(),
    std::vector<Command *>{&chassisTokyoCommand, &gimbalChaseCommand2},
    &leftSwitchUpState);

void registerIOMappings(src::Drivers *drivers) {
    drivers->commandMapper.addMap(std::move(leftSwitchUp));
}
```

- Never pass a `RemoteMapState(...)` temporary: it dangles.
- `make_unique` can't deduce a braced list, so the commands need `std::vector<Command *>{...}`.
- A mapping can be added only once (the old double `addMap(&bCtrlPressed)` is gone).

**Ref System v1.3:** `MechanismID::TURRET_17MM_1`/`_2` → `TURRET_17MM`; `heat17ID1`/`heat17ID2` → `heat17`. Sentry's two `BARREL_IDS` both map to `TURRET_17MM`, so the barrel manager sees one shared heat.

**Smaller ones:**
- `ChassisSubsystemInterface::getActualVelocityChassisRelative()` is `const`.
- `getVelocityWorldRelative()` moved to `tap::algorithms::odometry`.
- `tap::control::Command`'s destructor is not virtual: no `override` on command destructors.
- modm removed `GpioOutput()`: use `configure(Gpio::OutputType::PushPull)`.
- `taproot:core` no longer pulls everything in. `project.xml` adds `motor:pwm`, `architecture:profiler`, `algorithms:odometry`, `communication:gpio:analog`, `control:terminal_serial`, `errors`, `motor:terminal_serial`. Add the module if a `tap/...` header goes missing.
- IMU heater defaults to 35 C; `kinematic_informant` sets 50 C (`main.cpp` waits for 50 C).

## Fibers (main loop)

DM-MC02 Taproot runs on modm fibers; protothreads and resumables are fiber-backed.

- `main()` runs `modm::fiber::Scheduler::run()`. The old superloop is `mainLoopFiber` (8 KiB stack, not measured) and yields with `modm::this_fiber::yield()` (was `delay_us(10)`).
- **Don't call a protothread's `run()` from a command's `execute()`.** It blocks the main loop at `PT_WAIT_UNTIL`. `ClientDisplayCommand` runs its protothread as its own fiber and `execute()` only (re)starts it.
- `RF_END()` returns nothing under fibers: use `RF_END_RETURN(false)` where a value is needed.
- Motor loop is 500 Hz (scheduler + PID + CAN TX ~118 us). `motorLoopCount` / `motorLoopMaxPeriodUs` are watch vars.

## DM-MC02 hardware mapping (vs Type C)

| What | Type C | DM-MC02 |
|---|---|---|
| Ref serial | Uart6 | Uart1 |
| Jetson | Uart1 | UART7 (PE8/PE7) |
| Hopper/grabber servo | `Pwm::C1` | `PWM1` (PA0, TIMER2) |
| Feeder limit switch | C6/C7 | PA2 (PWM2 pin as GPIO in) |
| Rev encoder | SPI2 PB12-15, prescaler 16 | SPI3 PC10/11/12, CS PE14, prescaler 32 (2.5 MHz) |
| CAN | CAN1, CAN2 | CAN1, CAN2, CAN3. Old CAN2 users are on CAN3; Standard 2025 yaw is alone on CAN2 |
| LEDs | GPIO LEDs | none (`Leds::Green` calls removed) |
| IST8310 magnetometer | I2C3 | none (driver removed) |
| OpenOCD | stm32f4x | stm32h7x (`openocd.cfg`) |

Rail ultrasonics were dead code and are removed.

## Gotchas

- **Hand edit in generated code:** `aimbots-src/taproot/src/tap/communication/sensors/imu_heater/imu_heater.hpp` has MC02 gains (P 0.01, I 5e-6, max err sum 0.08/I, max out 0.1). **An lbuild regen overwrites it.** Re-apply after regenerating; Taproot's defaults overshoot to 93 C.
- `aimbots-src/taproot/modm/scons/site_tools/size.py` must stay tracked. It was gitignored once and fresh clones failed with `No tool module 'size'`.
- `aimbots-src/taproot/` is generated: change `project.xml` or the submodule, then regenerate, rather than editing it (except the heater above).
