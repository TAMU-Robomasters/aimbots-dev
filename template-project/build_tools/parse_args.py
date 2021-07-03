# Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
#
# This file is part of aruw-mcb.
#
# aruw-mcb is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# aruw-mcb is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.

from SCons.Script import *
from . import extract_robot_type


CMD_LINE_ARGS                       = 1
TEST_BUILD_TARGET_ACCEPTED_ARGS     = ["build-tests", "run-tests", "run-tests-gcov"]
SIM_BUILD_TARGET_ACCEPTED_ARGS      = ["build-sim", "run-sim"]
HARDWARE_BUILD_TARGET_ACCEPTED_ARGS = ["build", "run", "size", "gdb", "all"]
VALID_BUILD_PROFILES                = ["debug", "release", "fast"]
VALID_PROFILING_TYPES               = ["true", "false"]

USAGE = "Usage: scons <target> [profile=<debug|release|fast>] [robot=TARGET_<ROBOT_TYPE>] [profiling=<true|false>]\n\
    \"<target>\" is one of:\n\
        - \"build\": build all code for the hardware platform.\n\
        - \"run\": build all code for the hardware platform, and deploy it to the board via a connected ST-Link.\n\
        - \"build-tests\": build core code and tests for the current host platform.\n\
        - \"run-tests\": build core code and tests for the current host platform, and execute them locally with the test runner.\n\
        - \"run-tests-gcov\": builds core code and tests, executes them locally, and captures and prints code coverage information\n\
        - \"build-sim\": build all code for the simulated environment, for the current host platform.\n\
        - \"run-sim\": build all code for the simulated environment, for the current host platform, and execute the simulator locally.\n\
    \"TARGET_<ROBOT_TYPE>\" is an optional argument that can override whatever robot type has been specified in robot_type.hpp.\n\
        - <ROBOT_TYPE> must be one of the following:\n\
            - SOLDIER, OLD_SOLDIER, DRONE, ENGINEER, SENTINEL, HERO"


def parse_args():
    args = {
        "TARGET_ENV": "",
        "BUILD_PROFILE": "",
        "PROFILING": "",
        "ROBOT_TYPE": "",
    }
    if len(COMMAND_LINE_TARGETS) > CMD_LINE_ARGS:
        raise Exception("You did not enter the correct number of arguments.\n" + USAGE)

    # Extract the target environment from the first command line argument
    # and determine modm build path as well as add any extra files to ignore
    if len(COMMAND_LINE_TARGETS) != 0:
        build_target = COMMAND_LINE_TARGETS[0]
        if build_target in TEST_BUILD_TARGET_ACCEPTED_ARGS:
            args["TARGET_ENV"] = "tests"
        elif build_target in SIM_BUILD_TARGET_ACCEPTED_ARGS:
            args["TARGET_ENV"] = "sim"
        elif build_target in HARDWARE_BUILD_TARGET_ACCEPTED_ARGS:
            args["TARGET_ENV"] = "hardware"
        else:
            raise Exception("You did not select a valid target.\n" + USAGE)
    else:
        args["TARGET_ENV"] = "hardware"

    # Extract and validate the build profile (either debug or release)
    args["BUILD_PROFILE"] = ARGUMENTS.get("profile", "release")
    if args["BUILD_PROFILE"] not in VALID_BUILD_PROFILES:
        raise Exception("You specified an invalid build profile.\n" + USAGE)

    args["PROFILING"] = ARGUMENTS.get("profiling", "false")
    if args["PROFILING"] not in VALID_PROFILING_TYPES:
        raise Exception("You specified an invalid profiling type.\n" + USAGE)

    # Extract the robot type from either the command line or robot_type.hpp
    args["ROBOT_TYPE"] = extract_robot_type.get_robot_type()

    return args
