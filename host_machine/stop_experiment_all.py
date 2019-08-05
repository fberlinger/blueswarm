#!/usr/bin/env python3

import subprocess

base_command = "sudo killall python3"

for i in range(101, 110):
    command = "ssh -o ConnectTimeout=5 pi@192.168.0." + str(i) + " '" + base_command + "' > /dev/null 2>&1 &"
    # print(command)
    subprocess.call(command, shell=True)
