# Aimbots Embedded Training

Three weeks, one assignment per week. You'll write real code in the real team repo (`aimbots-src`, built on [Taproot](https://github.com/uw-advanced-robotics/taproot) and [modm](https://modm.io)) against a bare RoboMaster Development Board Type C and a DT7 remote.

> **Setup (toolchain, pipenv, VS Code, flashing drivers) is in the team Google Doc.** Finish that first. This folder assumes you can already build the robot code.

| Week | Topic | Assignment | Status |
|---|---|---|---|
| 1 | Command/subsystem framework, timers, GPIO, reading the board docs | [Blink an LED from a remote switch](week1/README.md) | ✅ Ready |
| 2 | UART, parsing the DT7 remote (DBUS) yourself, the preprocessor, the buzzer | [Parse the remote, drive the buzzer](week2/README.md) | ✅ Ready |
| 3 | Motor control and PID tuning | [week3](week3/README.md) | 🚧 Under construction |

## The training robot

You are setting up a new *robot type* called `TRAINING`. As far as the code is concerned, it's just a dev board and a remote. Everything you write lives in one folder:

```
aimbots-src/src/robots/training/
├── training_control.cpp          ← YOUR robot's control file: create commands, map them to the remote
├── training_board_subsystem.hpp  ← given to you, don't edit (week 1 explains why it exists)
├── training_config.hpp           ← week 2: picks taproot's remote or yours
├── remote/
│   ├── dt7_remote.hpp            ← week 2: your remote parser
│   └── dt7_remote.cpp
└── commands/
    ├── blink_led_command.hpp     ← YOUR week 1 command
    ├── blink_led_command.cpp
    ├── buzzer_command.hpp        ← YOUR week 2 command
    └── buzzer_command.cpp
```

Everything else (`main.cpp`, the remote driver, the scheduler, the command mapper) is shared team code that already works. Don't touch it. The one exception: in week 2 you fill in a few `TODO(week2)` lines in `drivers.hpp` and `main.cpp`, and only inside the `#ifdef TARGET_TRAINING` fences that are already there. Every file in `robots/training/` is wrapped in `#ifdef TARGET_TRAINING`, so nothing you write will affect the other robots' builds.

## Workflow

1. Make your own branch off the **`training-2026`** branch (not `main`, so the robots' code stays safe):
   ```bash
   git fetch
   git switch training-2026
   git pull
   git switch -c training/<your-name>
   ```
2. Build (from `aimbots-src/`):
   ```bash
   pipenv run scons build robot=TRAINING
   ```
   The starter code compiles as-is. **Check that it does before you change anything**, so you know any later error is yours.
3. Flash the board over the ST-Link:
4. Debug with MCUViewer: any global variable can be added to the watch window while the board runs (the codebase has lots of `...Display` globals for exactly this).
5. Commit and push your branch often. To get checked off, show a lead the working board and your branch.

> ⚠️ Never commit to `main` or `training-2026` directly (only to your own `training/<your-name>` branch), and never commit a change to another robot's files or taproot for a training assignment. `main.cpp` and `drivers.hpp` changes are allowed only inside the week 2 `TODO(week2)` fences.
>
> ⚠️ Switching `robot=` forces a full rebuild, which takes a few minutes. That's normal.
