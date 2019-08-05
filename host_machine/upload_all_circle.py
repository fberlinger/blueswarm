#!/usr/bin/env python3

import subprocess

for i in range(101, 110):
    command = "scp -o ConnectTimeout=5 main.py pi@192.168.0." + str(i) + ":~/fishfood > /dev/null 2>&1 &"
    # print(command)
    subprocess.call(command, shell=True)

    command = "scp -o ConnectTimeout=5 mask_l.txt pi@192.168.0." + str(i) + ":~/fishfood > /dev/null 2>&1 &"
    # print(command)
    subprocess.call(command, shell=True)

    command = "scp -o ConnectTimeout=5 mask_r.txt pi@192.168.0." + str(i) + ":~/fishfood > /dev/null 2>&1 &"
    # print(command)
    subprocess.call(command, shell=True)

    command = "ssh -o ConnectTimeout=5 pi@192.168.0." + str(i) + " 'flash_leds' > /dev/null 2>&1 &"
    # print(command)
    subprocess.call(command, shell=True)
