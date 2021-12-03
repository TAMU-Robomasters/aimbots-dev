#!/usr/bin/env python3

import os
import sys

SOURCE_FILE_EXTENSIONS = ['.cpp', '.hpp', '.h']

if len(sys.argv) != 2:
    print("usage: /usr/bin/python3 check_license_headers.py <project-src-path>")
    sys.exit(2)

project_path = sys.argv[1]

string_to_check = 'DoNotUse_getDrivers'

files_to_whitelist = [
    'main.cpp',
    'drivers_singleton.cpp',
    'drivers_singleton.hpp',
]

# Add on the project_path to all files in files_to_whitelist
files_to_whitelist = [ os.path.join(project_path, f) for f in files_to_whitelist ]
# Find all files in project_path
files_to_search = [ os.path.join(dp, f) for dp, dn, filenames in os.walk(project_path) for f in filenames ]


def is_source_file_to_check(file, ignored_files):
    if file in ignored_files:
        return False

    _, file_extension = os.path.splitext(file)
    return file_extension in SOURCE_FILE_EXTENSIONS

def check_file(file):
    '''
    Checks if 'file' contains 'string_to_check' and returns true if it does.
    '''
    with open(file, 'r') as file_to_check:
        if string_to_check in file_to_check.read():
            print("{0} contains the function call {1}".format(file, string_to_check))
            return True
    return False

# Iterate through all files, checking for the 'string_to_check'
result = 0
for file in files_to_search:
    if is_source_file_to_check(file, files_to_whitelist):
        result = check_file(file) or result

sys.exit(result)
