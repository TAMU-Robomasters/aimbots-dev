#!/usr/bin/python3
#
# Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
#
# This file is part of template-project.
#
# template-project is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# template-project is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with template-project.  If not, see <https://www.gnu.org/licenses/>.

import os
import sys

SCRIPT_DIR = os.path.dirname(__file__)
INPUT_FILE = os.path.join(SCRIPT_DIR, '../template-project/size.txt')
OUTPUT_FILE = os.path.join(SCRIPT_DIR, '../metrics.txt')

if len(sys.argv) != 2:
    print('usage: python3 parse_metrics.py <identifier>')
    sys.exit(1)

with open(INPUT_FILE, 'r') as in_file:
    lines = in_file.readlines()
    with open(OUTPUT_FILE, 'w') as out_file:
        for line in lines[-11:-2]:
            if 'Program:' in line:
                words = line.split()
                out_file.write('{0}_program_size [{1}{2},{3}]\n'
                        .format(sys.argv[1], words[1], words[2], words[3][1:]))
            if 'Data:' in line:
                words = line.split()
                out_file.write('{0}_data_usage [{1}{2},{3}]\n'
                        .format(sys.argv[1], words[1], words[2], words[3][1:]))
            if 'Heap:' in line:
                words = line.split()
                out_file.write('{0}_heap_usage [{1}{2},{3}%]\n'
                        .format(sys.argv[1], words[1], words[2], round(100 - float(words[3][1:5]),2)))
