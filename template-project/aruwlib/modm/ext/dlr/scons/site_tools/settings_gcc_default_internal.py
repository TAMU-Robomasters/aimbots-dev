#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2016-2017, German Aerospace Center (DLR)
# Copyright (c) 2018, Niklas Hauser
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.
#
# Authors:
# - 2016-2017, Fabian Greif (DLR RY-AVS)
# - 2018, Niklas Hauser

import os

from SCons.Script import *

def strip_binary(env, target, source, options="--strip-unneeded"):
    return env.Command(target,
                       source,
                       Action("$STRIP %s -o %s %s" % (options, target, source[0]),
                              cmdstr="$STRIPCOMSTR"))

def list_symbols(env, source, alias='__symbols'):
    action = Action("$NM %s -S -C --size-sort -td" % source[0].path,
                    cmdstr="$SYMBOLSCOMSTR")
    return env.AlwaysBuild(env.Alias(alias, source, action))


def generate(env, **kw):
    env.Tool('gcc')
    env.Tool('g++')
    env.Tool('gnulink')
    env.Tool('ar')
    env.Tool('as')

    env.Tool('utils_common')
    env.Tool('utils_gcc_version')

    # Define executable name of the compiler
    path = env.get('COMPILERPATH', '')
    prefix = env.get('COMPILERPREFIX', '')
    suffix = env.get('COMPILERSUFFIX', '')
    if suffix != '' and not suffix.startswith('-'):
        suffix = '-' + suffix

    prefix = path + prefix
    env['CC'] = prefix + 'gcc' + suffix
    env['CXX'] = prefix + 'g++' + suffix
    env['AR'] = prefix + 'ar'
    env['RANLIB'] = prefix + 'ranlib'
    if suffix == '':
        env['AS'] = prefix + 'as'
        env['NM'] = prefix + 'nm'
    else:
        env['AS'] = prefix + 'gcc' + suffix
        env['NM'] = prefix + 'gcc-nm' + suffix
        if sys.platform != "darwin":
            env['AR'] = prefix + 'gcc-ar' + suffix
            env['RANLIB'] = prefix + 'gcc-ranlib' + suffix

    env['OBJCOPY'] = prefix + 'objcopy'
    env['OBJDUMP'] = prefix + 'objdump'
    env['SIZE'] = prefix + 'size'
    env['STRIP'] = prefix + 'strip'

    env['LINK'] = env['CXX']

    # Flags for C and C++
    env['CCFLAGS'] = [
        '$CCFLAGS_target',
        '$CCFLAGS_optimize',
        '$CCFLAGS_debug',
        '$CCFLAGS_warning',
        '$CCFLAGS_other'
    ]

    env.SetDefault(CCFLAGS_optimize=['-O2'])
    env.SetDefault(CCFLAGS_debug=['-gdwarf-2'])
    env.SetDefault(CCFLAGS_warning=[
        '-W',
        '-Wall',
        '-Wextra',
        '-Wformat',
        '-Wunused-parameter',
        '-Wundef',
        '-Winit-self',
        '-Wcast-qual',
        '-Wcast-align',
        '-Wpointer-arith',
        '-Wwrite-strings',
        '-Wmissing-declarations',
        '-Wredundant-decls',
        '-Wunused',
        '-Wuninitialized',
        # allow 64-bit integer types even if they are not included in ISO C++98
        '-Wno-long-long',
        '-Wshadow',
#        '-Wconversion',
    ])

    env['GCC_version'] = env.DetectGccVersion()
    if env['GCC_version'] >= 40600:
        # The warning flag has been added with GCC 4.6
        env['CCFLAGS_warning'].append('-Wdouble-promotion')

    # C flags
    env['CFLAGS'] = [
        '$CFLAGS_language',
        '$CFLAGS_dialect',
        '$CFLAGS_warning',
        '$CFLAGS_other',
    ]

    env.SetDefault(CFLAGS_language=[
        '-std=c99',
        '-pedantic',
    ])
    env.SetDefault(CFLAGS_warning=[
        '-Wimplicit',
        '-Wstrict-prototypes',
        '-Wredundant-decls',
        '-Wnested-externs',
    ])

    # C++ flags
    env['CXXFLAGS'] = [
        '$CXXFLAGS_language',
        '$CXXFLAGS_dialect',
        '$CXXFLAGS_warning',
        '$CXXFLAGS_other',
    ]

    env.SetDefault(CXXFLAGS_language=[
        '-std=c++11',
        '-pedantic',
    ])

    env.SetDefault(CXXFLAGS_warning=[
        '-Wold-style-cast',
        '-Woverloaded-virtual',
        '-Wnon-virtual-dtor',
    ])

    # Assembler flags
    env['ASFLAGS'] = [
        '$ASFLAGS_target',
        '$ASFLAGS_other',
    ]

    # Flags for the linker
    env['LINKFLAGS'] = [
        '$CCFLAGS',
        '$LINKFLAGS_target',
        '$LINKFLAGS_optimize',
        '$LINKFLAGS_other'
    ]

    env.SetDefault(LINKFLAGS_other=[
        '-Wl,-Map,${TARGET.base}.map',
    ])

    builder_hex = Builder(
        action=Action("$OBJCOPY -O ihex $SOURCE $TARGET",
        cmdstr="$HEXCOMSTR"),
        suffix=".hex",
        src_suffix="")

    builder_bin = Builder(
        action=Action("$OBJCOPY -O binary $SOURCE $TARGET",
        cmdstr="$BINCOMSTR"),
        suffix=".bin",
        src_suffix="")

    builder_listing = Builder(
        action=Action("$OBJDUMP -x -S -l -w $SOURCE > $TARGET",
        cmdstr="$LSSCOMSTR"),
        suffix=".lss",
        src_suffix="")

    env.Append(BUILDERS={
        'Hex': builder_hex,
        'Bin': builder_bin,
        'Listing': builder_listing
    })

    env.AddMethod(strip_binary, 'Strip')
    env.AddMethod(list_symbols, 'Symbols')


def exists(env):
    return True

