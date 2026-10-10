# -*- coding: utf-8 -*-
#
# Copyright (c) 2018-2021, Niklas Hauser
# Copyright (c) 2019, Raphael Lehmann
#
# This file is part of the modm project.
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.
# -----------------------------------------------------------------------------

from os.path import join, abspath, relpath

def build_target(env, sources):
	# Building application
	program = env.Program(target=env["CONFIG_PROJECT_NAME"]+".elf", source=sources)
	chosen_program = env.ChooseFirmware(program)
	# Clean additional artifacts
	env.Clean(program, join(env["BUILDPATH"], env["CONFIG_PROJECT_NAME"]+".bin"))
	env.Clean(program, join(env["BUILDPATH"], env["CONFIG_PROJECT_NAME"]+".hex"))
	env.Clean(program, join(env["BUILDPATH"], env["CONFIG_PROJECT_NAME"]+".lss"))
	env.Clean(program, join(env["BUILDPATH"], env["CONFIG_PROJECT_NAME"]+".uf2"))

	env.Alias("qtcreator", env.QtCreatorProject(sources))
	env.Alias("symbols", env.Symbols(chosen_program))
	env.Alias("listing", env.Listing(chosen_program))
	env.Alias("bin", env.Bin(chosen_program))
	env.Alias("hex", env.Hex(chosen_program))
	env.Alias("build", program)
	# The executable depends on the linkerscript
	env.Depends(target=program, dependency="$BASEPATH/modm/link/linkerscript.ld")
	env.Alias("size", env.Size(chosen_program))
	env.Alias("uf2", env.UF2(chosen_program))

	env.Alias("log-itm", env.LogItmOpenOcd())
	env.Alias("log-rtt", env.LogRttOpenOcd())
	env.Alias("log-itm-jlink", env.LogItmJLink())
	env.Alias("log-rtt-jlink", env.LogRttJLink())

	env.Alias("artifact", env.CacheArtifact(program))
	env.Alias("program-openocd", [env.ProgramOpenOcd(chosen_program)])
	env.Alias("program-remote", [env.ProgramGdbRemote(chosen_program)])
	env.Alias("program-bmp", [env.ProgramBMP(chosen_program)])
	env.Alias("program-jlink", [env.ProgramJLink(chosen_program)])
	env.Alias('program-dfu', [env.ProgramDFU(env.Bin(chosen_program))])
	env.Alias("debug-openocd", env.DebugOpenOcd(chosen_program))
	env.Alias("debug-remote", env.DebugGdbRemote(chosen_program))
	env.Alias("debug-bmp", env.DebugBMP(chosen_program))
	env.Alias("debug-jlink", env.DebugJLink(chosen_program))
	env.Alias("debug-coredump", env.DebugCoredump(chosen_program))

	env.Alias("reset-openocd", env.ResetOpenOcd())
	env.Alias("reset-bmp", env.ResetBMP())
	env.Alias("reset-jlink", env.ResetJLink())
	env.Alias("reset-remote", env.ResetGdbRemote())

	# Start only OpenOCD to attach a external (remote) debugger
	env.Alias("openocd", env.OpenOcd())
	env.Alias("jlink", env.JLink())

	env.Alias("coredump-openocd", env.CoredumpOpenOcd())
	env.Alias("coredump-bmp", env.CoredumpBMP())
	env.Alias("coredump-jlink", env.CoredumpJLink())

	# Default to OpenOCD
	env.Alias("program", "program-openocd")
	env.Alias("reset", "reset-openocd")
	env.Alias("debug", "debug-openocd")
	env.Alias("coredump", "coredump-openocd")

	env.Alias("all", ["build", "size"])
	env.Default("all")
	return program


def generate(env, **kw):
	env.AddMethod(build_target, "BuildTarget")

def exists(env):
	return True