# Copyright (c) 2023-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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


from enum import Enum
import subprocess, os
from typing import Callable, List, Optional, Dict
import argparse
import pathlib
import platform


def run(cmd: List[str], cwd: Optional[str] = None, add_env: Optional[Dict[str, str]] = None):
    env = os.environ.copy() | add_env if add_env else None
    subprocess.run(cmd, check=True, cwd=cwd, env=env)


PROJECT_NAME = "AIMBOTS-DEV"
ORGANIZATION_NAME = "TAMU-Robomasters"
# Project directory relative to root repo directory
PROJECT_DIR = "aimbots-src"


def clang_format():
    CLANG_FORMAT_DIRS = [
        "aimbots-src/src/",
        "aimbots-src/test/",
    ]
    print("Running clang format...")
    run(["python", "taproot-scripts/clang_format_all.py", *CLANG_FORMAT_DIRS])


def check_singleton_drivers():
    print("Checking singleton drivers...")
    run(["python", "taproot-scripts/check_singleton_drivers.py", "DoNotUse_getDrivers", "-p", "src"])


def check_license_headers():
    IGNORE_LICENSE = [
        './**/__init__.py',
        'taproot/**/*',
        'taproot-scripts/**/*',
        'aimbots-src/taproot/**/*',
        'docs/**/*',
        'aimbots-src/robot-type/robot_type.hpp',
    ]
    print("Checking license headers...")
    run(["python", "taproot-scripts/check_license_headers.py", "-p", PROJECT_NAME, "-o", ORGANIZATION_NAME, "-i", *IGNORE_LICENSE])


def check_header_guards():
    HEADER_GUARD_CHECK_DIRS = [
        "aimbots-src/src",
        "aimbots-src/test/",
    ]
    IGNORE_HEADER = []
    HEADER_PREFIX = None
    print("Checking header guards...")
    run(["python", "./taproot-scripts/check_header_guard.py", *HEADER_GUARD_CHECK_DIRS, *(["-p", HEADER_PREFIX] if HEADER_PREFIX else []), "-i", *IGNORE_HEADER])


# def check_taproot_submodule():
#     VALID_BRANCHES = ["release", "develop"]
#     print("Checking taproot submodule...")
    # FIXME: Newly written powershell script relies on lbuild build command which is broken for windows
    # Need to replace with below wrapper for lbuild build
    # run(["bash" "./taproot-scripts/check_taproot_submodule.sh", PROJECT_DIR, "taproot", " ".join(VALID_BRANCHES)])


def run_lbuild():
    print("Running lbuild...")
    run(["pipenv", "run", "lbuild", "build"], cwd=PROJECT_DIR)

    def override_windows():
        # Note: The LF/CRLF change should be undone by git automatically when the change is staged but we do it manually to reduce confusion
        LF_TO_CRLF = ["aimbots-src/taproot/modm/ext/gcc/cabi.c"]
        DOUBLE_BACKSLASHES_TO_FORWARD_SLASHES = ["aimbots-src/taproot/modm/openocd.cfg"]
        BACKSLASHES_TO_FORWARD_SLASHES = [
            os.path.join(PROJECT_DIR, "taproot", dir, file) for file in [
                pathlib.Path("project.xml"),
                pathlib.Path("modm/SConscript"),
                pathlib.Path("modm/ext/printf/printf.h"),
            ] for dir in [
                pathlib.Path("."),
                pathlib.Path("sim-modm/hosted-darwin"),
                pathlib.Path("sim-modm/hosted-linux"),
                pathlib.Path("sim-modm/hosted-windows"),
            ]
        ]

        for file_path in LF_TO_CRLF:
            with open(file_path, "rb+") as f:
                content = f.read()
                content = content.replace(b"\r\n", b"\n")
                f.seek(0)
                f.write(content)
                f.truncate()
        
        for file_path in DOUBLE_BACKSLASHES_TO_FORWARD_SLASHES:
            with open(file_path, "r+", encoding="utf8") as f:
                content = f.read()
                content = content.replace("\\\\", "/")
                f.seek(0)
                f.write(content)
                f.truncate()

        for file_path in BACKSLASHES_TO_FORWARD_SLASHES:
            with open(file_path, "r+", encoding="utf8") as f:
                content = f.read()
                content = content.replace("\\", "/")
                f.seek(0)
                f.write(content)
                f.truncate()
    
    if platform.system() == "Windows":
        print("Replacing lbuild windows jankness...")
        override_windows()


class BuildTarget(Enum):
    STANDARD_2023 = "STANDARD_2023"
    STANDAED_BLASTOISE = "STANDARD_BLASTOISE"
    STANDAED_SQUIRTLE = "STANDARD_SQUIRTLE"
    STANDAED_2025 = "STANDARD_2025"
    HERO = "HERO"
    SENTRY_BRAVO = "SENTRY_BRAVO"
    SENTRY_SWERVE = "SENTRY_SWERVE"
    DART = "DART"
    ENGINEER = "ENGINEER"
    AERIAL = "AERIAL"
    CVTEST_LUKE = "CVTEST_LUKE"
    CVTEST_HAN = "CVTEST_HAN"
    CVTEST_CHEWIE = "CVTEST_CHEWIE"
    TURRET = "TURRET"
    all = "all"


def build_mcb(target : Optional[BuildTarget] = None):
    print(f"Checking MCB build for {target.value if target else 'all'}...")
    if not target or target == BuildTarget.all:
        for t in BuildTarget:
            if t != BuildTarget.all:
                build_mcb(t)
    else:
        run(["pipenv", "run", "scons", "build", f"robot={target.value}", "additional-ccflags=-Werror"], cwd=PROJECT_DIR)


def build_and_run_tests(target : Optional[BuildTarget] = None):
    print(f"Checking tests for {target.value if target else 'all'}...")
    if not target or target == BuildTarget.all:
        for t in BuildTarget:
            if t != BuildTarget.all:
                build_and_run_tests(t)
    else:
        run(["pipenv", "run", "scons", "run-tests", f"robot={target.value}"], cwd=PROJECT_DIR)


action_to_method : Dict[str, Callable] = {
    "format" : clang_format,
    "singleton_drivers" : check_singleton_drivers,
    "license" : check_license_headers,
    "header_guards" : check_header_guards,
    # "taproot" : check_taproot_submodule,
    "lbuild" : run_lbuild,
    "build" : build_mcb,
    "test" : build_and_run_tests
}


def main():
    args = parse_args()

    # Switch cwd to aruw-mcb
    print("Switching to aruw-mcb working directory...")
    aruw_mcb_path = os.path.dirname(os.path.realpath(__file__))
    os.chdir(aruw_mcb_path)

    # TODO: clear cached files for scons using --clean flag

    if args.action != "all":
        if args.action in {"build", "test", "sim"}:
            action_to_method[args.action](BuildTarget(args.robot))
        else:
            action_to_method[args.action]()
    else:
        # Format
        clang_format()
        
        # Policy checks
        check_singleton_drivers()
        check_license_headers()
        check_header_guards()
        # check_taproot_submodule()

        # Build
        run_lbuild()
        build_mcb()
        build_and_run_tests()

    # TODO: idk how docs work


def parse_args():
    arg = argparse.ArgumentParser(
        description="Runs all checks.")
    arg.add_argument("action", default=None, help=f"Action to take. If not specified, runs all checks. Must be one of {action_to_method.keys()} or all.")
    arg.add_argument("-r", "--robot", default=None, help="If action is either build, test, or sim, a robot target must be specified.")
    return arg.parse_args()


if __name__ == "__main__":
    main()
