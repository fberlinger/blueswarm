import subprocess

base_command = "flash_leds"

for i in range(101, 110):
    command = "ssh -o ConnectTimeout=5 pi@192.168.0." + str(i) + " '" + base_command + "' &"
    print(command)
    subprocess.call(command, shell=True)
