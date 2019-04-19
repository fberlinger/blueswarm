import subprocess

for i in range(101, 110):
    command = "scp -o ConnectTimeout=5 main.py pi@192.168.0." + str(i) + ":~/fishfood &"
    print(command)
    subprocess.call(command, shell=True)
    command = "ssh -o ConnectTimeout=5 pi@192.168.0." + str(i) + " 'flash_leds' &"
    print(command)
    subprocess.call(command, shell=True)
