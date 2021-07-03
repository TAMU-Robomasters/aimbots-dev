#!/usr/bin/python3
#
# Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
# These should be relative to the directory that this script is housed in.
PATHS_TO_CHECK = [ '../template-project/src/',
                   '../template-project/test/' ]
FILES_TO_IGNORE = [ '../template-project/src/drivers.hpp' ]

PATHS_TO_CHECK = [ os.path.join(SCRIPT_DIR, path) for path in PATHS_TO_CHECK ]
FILES_TO_IGNORE = [ os.path.join(SCRIPT_DIR, path) for path in FILES_TO_IGNORE ]

USAGE = "usage: /usr/bin/python3 check_license_headers.py [--update] \n\
options:\n\
    --update Adds licenses to files that don't have a license header (optional)"
LICENSED_SOURCE_FILE_EXTENSIONS = ['.cpp', '.hpp', '.h']
LICENSE_HEADER = '/*\n\
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>\n\
 *\n\
 * This file is part of template-project.\n\
 *\n\
 * template-project is free software: you can redistribute it and/or modify\n\
 * it under the terms of the GNU General Public License as published by\n\
 * the Free Software Foundation, either version 3 of the License, or\n\
 * (at your option) any later version.\n\
 *\n\
 * template-project is distributed in the hope that it will be useful,\n\
 * but WITHOUT ANY WARRANTY; without even the implied warranty of\n\
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n\
 * GNU General Public License for more details.\n\
 *\n\
 * You should have received a copy of the GNU General Public License\n\
 * along with template-project.  If not, see <https://www.gnu.org/licenses/>.\n\
 */\n'

if len(sys.argv) not in [ 1, 2 ]:
    print(USAGE)
    sys.exit(2)

def parse_args():
    update_files = sys.argv[-1] == "--update"
    return update_files

def is_licensed_source_file(path, ignored_files):
    if path in ignored_files:
        return False

    _, file_extension = os.path.splitext(file)
    return file_extension in LICENSED_SOURCE_FILE_EXTENSIONS

def file_has_valid_license_header(file):
    with open(file, 'r') as file_to_check:
        if LICENSE_HEADER not in file_to_check.read():
            return False
    return True

def add_license_to_file(file):
    print("Adding license to {0}".format(file))
    with open(file, 'r+') as file_to_check:
        content = file_to_check.read()
        file_to_check.seek(0, 0)
        file_to_check.write(LICENSE_HEADER.rstrip('\r\n') + '\n' + content)

update_files = parse_args()
files_to_search = []
for path in PATHS_TO_CHECK:
    files_to_search.extend([ os.path.join(dp, f) for dp, dn, filenames in os.walk(path) for f in filenames ])

result = False
for file in files_to_search:
    if is_licensed_source_file(file, FILES_TO_IGNORE):
        if not file_has_valid_license_header(file):
            result = True
            print("{0} does not contain a license header".format(file))
            if update_files:
                add_license_to_file(file)

sys.exit(result)
