#!/usr/bin/env python3
#
# This is a PlatformIO pre-build script.

from configparser import RawConfigParser
from sys import stderr

Import("env", "projenv")

parser = RawConfigParser()
parser.optionxform = str

try:
    with open('.env') as f:
        parser.read_string('[top]\n' + f.read())
        for k, v in parser.items('top'):
            print("Injecting %s into the build dynamically..." % k)
            projenv.ProcessUnFlags(k)
            projenv.Append(CPPDEFINES=(k, env.StringifyMacro(v)))

except OSError as e:
    print("Warning: .env file not found!", file=stderr)
    print('Create a file ".env" in the project dir and put the credentials like KOLAB_PASSWORD in it', file=stderr)
