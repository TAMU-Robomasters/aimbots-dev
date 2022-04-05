# Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
#
# This file is part of Taproot.
#
# Taproot is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# Taproot is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with Taproot.  If not, see <https://www.gnu.org/licenses/>.

import os
import subprocess
from SCons.Script import *
import glob


GCOV_OUT_DIR        = os.path.abspath('../gcov-result')
GCOV_OUT_FILE       = 'aaa_run_gcov.output'
SIM_MODM_DIR        = 'Source:sim-modm/'
TEST_DIR            = 'Source:test'
COVERAGE_INFO_FILE  = os.path.join(GCOV_OUT_DIR, 'coverage.info')


def run_gcov(env, source, alias="run_gcov"):
    def run_gcov_action(target, source, env):
        try:
            subprocess.run([source[0].abspath], check=True)
        except subprocess.CalledProcessError as e:
            print(e.output())
            exit(1)

        if not os.path.exists(GCOV_OUT_DIR):
            os.mkdir(GCOV_OUT_DIR)

        # Walk through test build directory and run gcov
        for dp, _, files in os.walk(env["BUILDPATH"]):
            for file in files:
                _, file_extension = os.path.splitext(file)

                full_file = os.path.join(dp, file)

                # The build directory has an identical path relative to the source
                # directory so we can chop off the build directory from the path
                # to get the corresponding source file
                corresponding_source_file = os.path.relpath(full_file, env["BUILDPATH"])

                corresponding_source_file, _ = os.path.splitext(corresponding_source_file)
                corresponding_source_file += '.cpp'

                # Run gcov on the file
                try:
                    result = subprocess.run(['gcov', '-r', '-o', dp, corresponding_source_file], capture_output=True, check=True)
                except subprocess.CalledProcessError as e:
                    print(e.output())
                    exit(1)

                with open(os.path.join(GCOV_OUT_DIR, GCOV_OUT_FILE), 'wb') as f:
                    f.write(result.stdout)

        # Move gcov files to the output directory
        subprocess.run(['mv *.gcov ' + GCOV_OUT_DIR], shell=True)

        # Remove any files that are part of modm's stuff (we don't care about coverage there)
        # Also remove any files in the test directory (we don't care about these either)
        for dp, _, files in os.walk(GCOV_OUT_DIR):
            for file in files:
                abs_file = os.path.join(dp, file)
                with open(abs_file, 'r') as f:
                    line = f.readline()
                    if SIM_MODM_DIR in line or TEST_DIR in line:
                        os.remove(abs_file)

        # Run lcov to generate a list of coverage percentages
        try:
            subprocess.run(['lcov', '-c', '-d', '.', '-o', COVERAGE_INFO_FILE, '--no-external'], check=True, stdout=subprocess.DEVNULL)

            files_to_include = glob.glob(os.path.abspath('src/**/*[.cpp|.hpp]'), recursive=True)
            files_to_include.extend(glob.glob(os.path.abspath('taproot/src/**/*[.cpp|.hpp]'), recursive=True))

            files_to_exclude_from_coverage = glob.glob(os.path.abspath('taproot/src/**/MahonyAHRS.*'), recursive=True)
            files_to_include = [ file for file in files_to_include if file not in files_to_exclude_from_coverage ]

            lcov_only_view_src_cmd = ['lcov', '-e', COVERAGE_INFO_FILE]
            lcov_only_view_src_cmd.extend(files_to_include)
            lcov_only_view_src_cmd.extend(['-o', COVERAGE_INFO_FILE])
            subprocess.run(lcov_only_view_src_cmd, stdout=subprocess.DEVNULL, check=True)

            subprocess.run(['lcov', '--list', COVERAGE_INFO_FILE], check=True)
        except subprocess.CalledProcessError as e:
            print(e.output)
            exit(1)

    action = Action(run_gcov_action, cmdstr="")
    return env.AlwaysBuild(env.Alias(alias, source, action))

def generate(env, **kw):
    env.AddMethod(run_gcov, "RunGCOV")

def exists(env):
    return env.Detect("run_gcov")
