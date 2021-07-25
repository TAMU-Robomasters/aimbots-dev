# -*- coding: utf-8 -*-
#
# Copyright (c) 2018-2019, Niklas Hauser
# Copyright (c) 2019, Raphael Lehmann
#
# This file is part of the modm project.
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.
# -----------------------------------------------------------------------------

from os.path import abspath, relpath

def build_target(env, sources):
	# Building application
	program = env.Program(target=env["CONFIG_PROJECT_NAME"]+".elf", source=sources)

	env.Alias("qtcreator", env.QtCreatorProject(sources))
	env.Alias("symbols", env.Symbols(program))
	env.Alias("listing", env.Listing(program))
	env.Alias("bin", env.Bin(program))
	env.Alias("build", program)
	env.Alias("run", env.Run(program))
	env.Alias("all", ["build", "run"])
	env.Default("all")
	return program


def generate(env, **kw):
	env.AddMethod(build_target, "BuildTarget")

def exists(env):
	return True