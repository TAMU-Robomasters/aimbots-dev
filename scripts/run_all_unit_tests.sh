#!/bin/bash
#
# This script builds the simulation profile for each robot target and runs
# the corresponding unit tests.
# Assumes you are in the build directory (where the SConstruct file is) 

set -e

targets="\
    TARGET_STANDARD \
    TARGET_ENGINEER \
    TARGET_HERO \
    TARGET_SENTRY \
    TARGET_DRONE"

for target in $targets; do
    echo ===============================================================================
    echo building $target
    echo ===============================================================================
    /usr/bin/env python3 $(which scons) run-tests robot=$target
done
