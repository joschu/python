#!/usr/bin/env python
import sys
from os.path import exists
from time import sleep
import subprocess
from utils.colorize import colorize

command = sys.argv[1]
dependencies = sys.argv[2:]

while True:
    if all(exists(file) for file in dependencies):
        print colorize(command,'red')
        subprocess.check_call(command,shell=True)
        break
    else:
        sleep(.01)