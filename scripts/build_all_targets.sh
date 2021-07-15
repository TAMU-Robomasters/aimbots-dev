#!/bin/bash
#
# This script builds the hardware profile for each robot target.
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
    /usr/bin/env python3 $(which scons) build robot=$target additional-ccflags=-Werror
done
