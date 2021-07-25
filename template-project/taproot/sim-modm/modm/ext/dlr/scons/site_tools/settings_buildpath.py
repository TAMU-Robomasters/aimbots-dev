#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2013-2014, 2016-2017, German Aerospace Center (DLR)
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.
#
# Authors:
# - 2013-2014, 2016-2017, Fabian Greif (DLR RY-AVS)

import os
import sys

is_py_3 = (sys.version_info[0] == 3)

def makedirs(path):
    if is_py_3:
        os.makedirs(path, exist_ok=True)
    else:
        while(True):
            try:
                os.makedirs(path)
                break
            except OSError as exc:
                if os.path.isdir(path):
                    break
                if exc.errno == errno.EEXIST:
                    pass
                else:
                    raise

def relocate_to_buildpath(env, path, strip_extension=False):
    """ Relocate path from source directory to build directory
    """
    path = str(path)
    if strip_extension:
        path = os.path.splitext(path)[0]

    # Do not relocate path if is already is inside of the build directory
    if not os.path.abspath(path).startswith(os.path.abspath(env['BUILDPATH'])):
        path = os.path.relpath(path, env['BASEPATH'])
        if path.startswith('..'):
            # if the file is not in a subpath of the current directory
            # build it in the root directory of the build path
            while path.startswith('..'):
                path = path[3:]

        buildpath = os.path.abspath(os.path.join(env['BUILDPATH'], path))
    else:
        buildpath = os.path.abspath(path)

    # We make the dirs manually, so that multiple processes building
    # in the same path are not racing to create the first directory
    makedirs(os.path.dirname(buildpath))
    return buildpath


def generate(env, **kw):
    # These emitters are used to build everything not in place but in a
    # separate build-directory.
    def default_emitter(target, source, env):
        targets = []
        for infile in target:
            # relocate the output to the buildpath
            filename = env.Buildpath(infile.path)
            targets.append(env.File(filename))
        return targets, source

    def shared_emitter(target, source, env):
        targets = []
        for infile in target:
            # relocate the output to the buildpath
            filename = env.Buildpath(infile.path)

            outfile = env.File(filename)
            outfile.attributes.shared = 1

            targets.append(outfile)
        return targets, source

    env['BUILDERS']['Object'].add_emitter('.cpp', default_emitter)
    env['BUILDERS']['Object'].add_emitter('.cc', default_emitter)
    env['BUILDERS']['Object'].add_emitter('.c', default_emitter)
    env['BUILDERS']['Object'].add_emitter('.sx', default_emitter)
    env['BUILDERS']['Object'].add_emitter('.S', default_emitter)

    env['BUILDERS']['SharedObject'].add_emitter('.cpp', shared_emitter)
    env['BUILDERS']['SharedObject'].add_emitter('.cc', shared_emitter)
    env['BUILDERS']['SharedObject'].add_emitter('.c', shared_emitter)

    env['LIBEMITTER'] = default_emitter
    env['PROGEMITTER'] = default_emitter

    env['BUILDPATH_EMITTER'] = default_emitter

    env.AddMethod(relocate_to_buildpath, 'Buildpath')


def exists(env):
    return True
