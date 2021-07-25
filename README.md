[![pipeline status](https://gitlab.com/aruw/controls/aruwlib-template-project/badges/develop/pipeline.svg)](https://gitlab.com/aruw/controls/aruwlib-template-project/-/commits/develop)

<!-- Start sections that may be removed after forking this repository -->
<hr/>

# aruwlib-template-project


This is a blank project fully configured for use of [aruwlib](https://gitlab.com/aruw/controls/aruwlib).
It is designed to be a starting point for your own RoboMaster software projects. Be sure to check
out aruwlib for more information.

**Note:** Currently, aruwlib only supports the RoboMaster Development Board Type A. In the future,
you will be able to configure this repository for other development boards. See
aruw/controls/aruwlib#9.

This template includes the following:
- Code generated from the `aruwlib` and `modm` repositories. This generated code is located in
  `/template-project/aruwlib` and includes the hardware abstraction layer provided by
  [modm](modm.io) as well as aruwlib library code that sits on top of modm.

  See [here](https://gitlab.com/aruw/controls/aruwlib/-/wikis/Code-Generation-in-User-Projects) for
  more information on code generation.
- A `.vscode` folder with C++ configurations for developing in a simulated, test, and hardware
  environment. This also includes convenient VS Code tasks for building code and debug launch
  configuration for debugging the simulated, test, or hardware environment.
- A [Doxygen](https://www.doxygen.nl/index.html) document generation configuration that renders a
  documentation webpage sourced from commented code.
- Various [linting](https://en.wikipedia.org/wiki/Lint_(software)) scripts that are used for
  maintaining high quality source code.
- A [clang format](https://clang.llvm.org/docs/ClangFormat.html) configuration (see
  `.clang-format`). This should be configured based on user preferences.
- Build scripts supporting configurable target profiles, including environment (simulator, tests,
  hardware) and robot type (Standard, Hero, etc.).
- A basic GitLab Continuous Integration (CI) Pipeline that lints the source code, builds all
  targets, and runs your tests (see `.gitlab-ci.yml`).
- Instructions and a configuration file for deploying your software to hardware via the command
  line.

## Usage of the template

This project is provided to reduce the configuration overhead when adopting aruwlib. We recommend
creating a new blank GitLab project and pushing the history of this repository there. This will be
similar to a fork, but omit the "forked from..." badge, which you likely don't want.

Assuming your new project is at `https://gitlab.com/my-team/my-amazing-project`, the setup process
is as follows:

```bash
git clone https://gitlab.com/my-team/my-amazing-project.git
cd my-amazing-project
git remote add template https://gitlab.com/aruw/controls/aruwlib-template-project.git
git pull template develop
# replace "main" with your main branch name of choice
git push --set-upstream origin main
```

If you visit the project's GitLab page, starter files should be present and GitLab will likely have
kicked off a Continuous Integration (CI) Pipeline, indicated by the blue "waiting" icon or a green
"checkmark".

### Configuring your new project

By using this template, your project will start out with a fully-functional aruwlib instance and
build tools. The provided `main.cpp` (`/template-project/src/main.cpp`) includes initialization of
all core systems and drivers. This should allow you to develop control systems starting day one.

_Note: [Issue #3](https://gitlab.com/aruw/controls/aruwlib-template-project/-/issues/3) tracks the
desire to automate the below process. Let us know about your experiences there._

To get started, we suggest using Visual Studio Code to perform a find-and-replace across all files,
swapping your own project name in place of `template-project`. Furthermore, rename the directory
`/template-project` accordingly. Feel free to call it the same name as your repo.

You should also update the license headers in your project. The above find-and-replace should have
updated the project name references. However, you will also want to update the copyright line of
each header:

```
Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
```

To refer to your own team, year and email. Do so in all template source files, **excluding files in
`**/aruwlib`**. Also update `scripts/check_license_headers.py` accordingly.

> **A note on copyright headers:**
> The above suggestions regarding copyright headers are purely for your convenience. You are free to
> decide how you would like to manage these, including omitting the license headers entirely if
> preferred, so long as you abide by the original license terms of the files. You may opt to disable
> the automated license header check; do so by removing the below line from `.gitlab-ci.yml`:
> ```yaml
>     - python3 ./scripts/check_license_headers.py
> ```

To start developing software, place your source code in `/template-project/src` and your tests in
`/template-project/test`. See the [workflow guide](#workflow-guide) for how to build, test, and
deploy your code.

Finally, after you are done with it, we recommend removing the portion of this README between
`<!-- ... -->` comments. The rest of the file is intended to provide a starting point for your team.

## Contacting

If you have any questions please contact us at robomstr@uw.edu.

## Licensing

aruwlib-template-project is covered under the GPL-3.0-or-later with the following exceptions:
- `/aruwlib/modm` and `/template-project/aruwlib/modm` are licensed under MPL 2.0 by the modm
  project. We _are not_ the license holder for these files. See `/modm/LICENSE` for license
  information.
- `/template-project/aruwlib/src/aruwlib/algorithms/MahonyAHRS.h` and
  `/template-project/aruwlib/src/aruwlib/algorithms/MahonyAHRS.cpp` are licensed under the GPL by
  SOH Madgwick. The repo containing this code can be found
  [here](https://github.com/uw-advanced-robotics/MahonyAHRS).

Other RoboMaster teams are invited, and encouraged, to utilize this library. We have licensed this
template project and aruwlib under the GPL to encourage collaboration and open publishing of
RoboMaster controls codebases. **We politely request that other teams choosing to utilize this
library, or parts of it (including its design), open-source their own code in turn.**

<hr/>
<!-- End sections that may be removed after forking this repository -->

## Resources

- **The [aruwlib wiki](https://gitlab.com/aruw/controls/aruwlib/-/wikis/home). It has lots of content and we strongly recommend you browse through it to get a sense of
  what's there.**
- [aruw-edu](https://gitlab.com/aruw/controls/aruw-edu): a hands-on tutorial for building robot code with aruwlib
- [aruw-mcb](https://gitlab.com/aruw/controls/aruw-mcb), ARUW's full robot code project available for reference
- The [generated API documentation for aruwlib](https://aruw.gitlab.io/controls/aruwlib/)
- The [modm website](https://modm.io/) and associated documentation

## New user guide

To develop software for the simulator or unit test environment, a Debian Linux development
environment is necessary. When developing software for the development board, Linux, Windows, or Mac
OS operating systems all work. We recommend working in a Debian Linux environment so you can both
run tests and deploy to the development board.

If you do not have a native Linux environment, we recommend using a virtual machine. We have tested
a virtual machine hosted using [VirtualBox](https://www.virtualbox.org). Once you have a virtual
machine installed on your computer, follow
[this](https://gitlab.com/aruw/controls/aruwlib/-/wikis/Debian-Linux-Setup) guide to set up the
tooling necessary to build and deploy software.

Alternatively, we have guides for developing software in a [Docker
container](https://gitlab.com/aruw/controls/aruwlib/-/wikis/Docker-Container-Setup), a [Windows
machine](https://gitlab.com/aruw/controls/aruwlib/-/wikis/Windows-Setup), or by using [Windows
Subsystem for Linux](https://gitlab.com/aruw/controls/aruwlib/-/wikis/Windows-WSL-Setup). Note that
these have drawbacks because they either do not fully support both running unit tests on your local
machine and deploying to the development board or have not been rigorously tested.

Sometimes setting up your machine can be tricky. If you are having trouble setting up your
environment properly, feel free to ask for help on our [Discord server](https://discord.gg/jjDrGhrjMy).

## Workflow guide

### Getting around VSCode

Microsoft provides a [helpful
website](https://code.visualstudio.com/docs/getstarted/tips-and-tricks) with a number of shortcuts
for getting around VSCode. There are many shortcuts that make programming faster.

### Building code and programming the RoboMaster Development Board

_If you would like to use the terminal instead, see the section "Building and running via the
terminal" below._

1. Make sure you have VSCode opened in the folder `aruwlib-template-project` (**not
   `template-project`**)
2. Connect an ST-Link to the RoboMaster Development Board and your computer.
3. In VSCode, open the Command Palette (<kbd>Ctrl</kbd>+<kbd>shift</kbd>+<kbd>P</kbd>)
4. Find `Tasks: Run Task`. You should see the options below. Select `Program - Debug` or `Program -
   Release`.<br><br>
    <img
    src=https://gitlab.com/aruw/controls/aruw-mcb/uploads/2ffb02c86387916c2c49ac3548151b38/image.png
    height="200px" />

### Debugging with an ST-Link

1. Open the folder `aruw-template-project` in VSCode. Hit the debug tab on the left side or type
   <kbd>Ctrl</kbd>+<kbd>shift</kbd>+<kbd>D</kbd>.
2. Hit the green play arrow on the left top of the screen.
3. See [this
   page](https://gitlab.com/aruw/controls/aruwlib/-/wikis/Software-Tools/Debugging-With-STLink) for
   more information about using the ST-Link for programming the MCB and debugging. <br>
   <img
   src=https://gitlab.com/aruw/controls/aruw-mcb/uploads/1f62ea310a20ee76092fe18de83d14a7/image.png
   height="400px" />

### Debugging with a J-Link

See the [wiki](https://gitlab.com/aruw/controls/aruwlib/-/wikis/Software-Tools/Debugging-With-JLink)
for an explanation on the difference between an ST-Link and J-Link and a step-by-step procedure on
how to use the J-Link.

### Selecting and using robot types

Specify the robot type via the command line when compiling (see
[below](#building-and-running-via-the-terminal)). For vscode IntelliSense, navigate to
`/template-project/robot-type/robot_type.hpp` and change the macro defined in this file.

Each robot is signified by a unique macro which can be checked to special-case code:

```c++
#if defined(TARGET_STANDARD)
// Only included if building for the Standard
initializeStandard();
#endif
```

### How to select an appropriate VSCode C/C++ configuration

This codebase has a number of different build targets (see [this wiki
page](https://gitlab.com/aruw/controls/aruwlib/-/wikis/Build-Targets-Overview) for more
information). Because the build setup is different for the test, sim, and RoboMaster Development
Board (aka MCB) environments, while working on a particular portion of code you may select an
appropriate profile that provides optimal
[intellisense](https://code.visualstudio.com/docs/editor/intellisense). To select a configuration,
in VSCode, type <kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>P</kbd>, then type "C/C++:Select a
Configuration" and hit enter. A dropdown menu will appear where you may choose either the "Test",
"Sim", or "Hardware" configuration.

## Building and running via the terminal

The below commands require that your working directory is `/template-project` (where the
`SConstruct` and `project.xml` files are).

- `lbuild build`: Re-generates out copy of `aruwlib` and `modm`.
- `scons build`: Builds the firmware image for the hardware target. Creates a "release" folder located in `build/hardware/` which contains the final `.elf` file as well as the intermediate object files (`.o`).
- `scons build-tests`: Builds a program which hosts our unit tests. This executable can be run on your host computer (only supported on Linux) and prints results for each unit test run.
- `scons program`: Builds as with `scons build` and then programs the board.
- `scons run-tests`: Builds and runs the unit test program.
- `scons size`: Prints statistics on program size and (statically-)allocated memory. Note that the reported available heap space is an upper bound, and this tool has no way of knowing about the real size of dynamic allocations.

Below is the full usage statement from our scons build environment. Note that you can select the
robot, profile, or whether or not you want profiling to be on using the various options.

```
Usage: scons <target> [profile=<debug|release|fast>] [robot=TARGET_<ROBOT_TYPE>] [profiling=<true|false>]
    "<target>" is one of:
        - "build": build all code for the hardware platform.
        - "run": build all code for the hardware platform, and deploy it to the board via a connected ST-Link.
        - "build-tests": build core code and tests for the current host platform.
        - "run-tests": build core code and tests for the current host platform, and execute them locally with the test runner.
        - "run-tests-gcov": builds core code and tests, executes them locally, and captures and prints code coverage information
        - "build-sim": build all code for the simulated environment, for the current host platform.
        - "run-sim": build all code for the simulated environment, for the current host platform, and execute the simulator locally.
    "TARGET_<ROBOT_TYPE>" is an optional argument that can override whatever robot type has been specified in robot_type.hpp.
        - <ROBOT_TYPE> must be one of the following:
            - STANDARD, DRONE, ENGINEER, SENTRY, HERO:
```
