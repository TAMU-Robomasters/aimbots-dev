[![pipeline
status](https://gitlab.com/aruw/controls/aruwlib-template-project/badges/develop/pipeline.svg)](https://gitlab.com/aruw/controls/aruwlib-template-project/-/commits/develop)

<!-- Start sections that may be removed after forking this repository -->
<hr/>

# aruwlib-template-project


This is a blank project to help people start using
[aruwlib](https://gitlab.com/aruw/controls/aruwlib) in their own projects. Be sure to check out
aruwlib before starting to use this template project.

**Note:** Currently, aruwlib only supports the RoboMaster Development Board Type A, so this project
is configured to work with this MCU. In the future, you will be able to configure this repository to
work with other development boards.

This project includes the following:
- Code generated from the `aruwlib` and `modm` repositories to use in your project. This generated
  code is located in `/template-project/aruwlib` and includes the hardware abstraction layer
  provided by [modm](modm.io) as well as aruwlib library code that sits on top of modm.
- A `.vscode` configuration folder with C++ configurations for developing in a simulated, test, and
  hardware environment. This also includes convenient VS Code tasks for building code and debug
  launch configuration for debugging the simulated, test, or hardware environment.
- A [Doxygen](https://www.doxygen.nl/index.html) document generation configuration that generates
  documentation from commented code.
- Various [linting](https://en.wikipedia.org/wiki/Lint_(software)) scripts that are used for
  maintaining high quality software.
- A [clang format](https://clang.llvm.org/docs/ClangFormat.html) configuration (see
  `.clang-format`). This should be configured based on user preferences.
- Build tools that allow you to build your software with various configurations specified (including
  building for a simulator, tests, hardware, as well as building for a specific robot type).
- A basic GitLab Continuous Integration (CI) Pipeline that lints the source code and builds all
  target configurations (see `.gitlab-ci.yml`).
- Instructions and a configuration file for deploying your software to hardware via the command
  line.

## How to utilize this project

This project is provided to reduce the startup time needed to start using aruwlib. To start using
this project, we recommend creating a new blank project and pushing the history of this repository
to the project. We recommend **not** forking this project. Assuming you have a repository set up
somewhere, to push the history of this project, clone this repository, change the remote to your new
project, and push this repository to your new project.

### Starting to use this project

By default, all the tools necessary for developing software for the development boards will be
generated. By default, all of the software drivers present in `aruwlib` are initialized and updated.
You can see this behavior in `/template-project/src/main.cpp`. This should allow you to develop
control systems starting day one.

To get started, we first recommend find and replacing `template-project` with the name of your
project. Furthermore, rename the directory `/template-project` to the name of your project.

Furthermore, you should update the license headers in this project. To do this, find and replace all
instances of `template-project` with the name of your project. Finally, find and replace `Copyright
(c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>` (excluding any
files in `**/aruwlib`) with the name of your organization and an emil.

To start developing software, place your source code in `/template-project/src` and your tests in
`/template-project/test`. See the [workflow guide](#workflow-guide) for how to build, test, or
deploy your code.

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

Alternatively, we have guides for developing software on a [Docker
container](https://gitlab.com/aruw/controls/aruwlib/-/wikis/Docker-Container-Setup), a [Windows
machine](https://gitlab.com/aruw/controls/aruwlib/-/wikis/Windows-Setup), or by using [Windows
Subsystem for Linux](https://gitlab.com/aruw/controls/aruwlib/-/wikis/Windows-WSL-Setup). Note that
these have drawbacks because they either do not fully support both running unit tests on your local
machine and deploying to the development board or have not been rigorously tested.

Sometimes setting up your machine can be tricky. If you are having trouble setting up your
environment properly, feel free to ask for help on our [discord](https://discord.gg/jjDrGhrjMy).

## Workflow guide

### Getting around VSCode

Microsoft provides a [helpful
website](https://code.visualstudio.com/docs/getstarted/tips-and-tricks) with a number of shortcuts
for getting around VSCode. There are many shortcuts that make programming faster.

### How to build code and program the RoboMaster Development Board

_If you would like to use the terminal instead, see the section "Building and running via the
terminal" below._

1. Make sure you have VSCode opened in the folder `aruw-template-project` (**not
   `template-project`**)
2. Connect an ST-Link to the RoboMaster Development Board and your computer.
3. In VSCode, open the Command Palette (<kbd>Ctrl</kbd>+<kbd>shift</kbd>+<kbd>P</kbd>)
4. Find `Tasks: Run Task`. You should see the options below. Select `Program - Debug` or `Program -
   Release`.<br><br>
    <img
    src=https://gitlab.com/aruw/controls/aruw-mcb/uploads/2ffb02c86387916c2c49ac3548151b38/image.png
    height="200px" />

### How to debug using an ST-Link

1. Open the folder `aruw-template-project` in VSCode. Hit the debug tab on the left side or type
   <kbd>Ctrl</kbd>+<kbd>shift</kbd>+<kbd>D</kbd>.
2. Hit the green play arrow on the left top of the screen.
3. See [this
   page](https://gitlab.com/aruw/controls/aruwlib/-/wikis/Software-Tools/Debugging-With-STLink) for
   more information about using the ST-Link for programming the MCB and debugging. <br>
   <img
   src=https://gitlab.com/aruw/controls/aruw-mcb/uploads/1f62ea310a20ee76092fe18de83d14a7/image.png
   height="400px" />

### How to debug using a J-Link

See the [wiki](https://gitlab.com/aruw/controls/aruwlib/-/wikis/Software-Tools/Debugging-With-JLink)
for an explanation on the difference between an ST-Link and J-Link and a step-by-step procedure on
how to use the J-Link.

### How to select robot type

Navigate to `/template-project/robot-type/robot_type.hpp` and change the macro defined in this file.
Alternatively, specify the robot type via the command line when compiling (see
[below](#building-and-running-via-the-terminal)). Each robot is defiend by a unique macro (for
example, to build standard code the macro `TARGET_STANDARD` is globally defined in the project).

### How to select an appropriate VSCode C/C++ configuration

This codebase has a number of different build targets (see [this wiki
page](https://gitlab.com/aruw/controls/aruwlib/-/wikis/Build-Targets-Overview) for more
information). Because the build setup is different for the test, sim, and RoboMaster Development
Board (aka MCB) environments, while working on a particular portion of code you may select an
appropriate profile that provides optimal
[intellisense](https://code.visualstudio.com/docs/editor/intellisense). To select a configuration,
in VSCode, type <kbd>Ctrl</kbd>+<kbd>Shift</kbd>+<kbd>P</kbd>, then type "C/C++:Select a
Configuration" and hit enter. A dropdown menu will appear where you may choose either the "Test",
"Sim", or "MCB" configuration.

## Building and running via the terminal

The below commands require that your working directory is `/template-project` (where the
`SConstruct` and `project.xml` files are).

To rebuild aruwlib or compile the software, refer to [aruwlib's
README](https://gitlab.com/aruw/controls/aruwlib/-/blob/develop/README.md#user-content-building-and-testing-via-the-terminal).
Also note the following usage statement for using the scons build environment. Note that you can
select the robot, profile, or whether or not you want profiling to be on using the various options.

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
