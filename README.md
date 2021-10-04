# Blueswarm

Blueswarm is the collective of autonomous underwater robots, used to study 3D collective behaviors: https://youtu.be/qVsu49f-Vf0. If you just wanted to simulate behaviors, check out Bluesim.

## Overview

This repository contains two main folders:
1) Code in `fishfood` is written to be uploaded and run on Bluebots. The following prefixes are used:
  - `exp_`: experiment files for different behaviors
  - `lib_`: custom libraries for vision updates, blob detection, fin control, etc.
  - `tst_`: test files for rapid prototyping or assebmly test routines
2) Code in `host_machine` runs on your computer and includes files to simultaneously upload and run code on all robots.

All code is written in Python3 and optimized to run as fast as possible onboard of the autonomous robots.

I will now walk you through an example. Find a complete documentation on Bluebot hardware and software on the Google Drive of the Self-organizing Research Group (private) and visit the "Quick start" folder to get started. 

## Run demo

### How to run a demo with a Bluebot:

1.	Connect a Blueswarm router (BB12) to a power outlet.
2.	Connect your computer to the BlueSwarm WiFi network (PW: Tpossr236).
3.	Pick a healthy Bluebot (07, 09, 10, 13) and switch it on with the custom switch.
  - Switch on the switch. Make sure it’s pointed in the correct direction (fish aligned). Touch the two posterior pins.
4.	SSH into Bluebot
  - On a Mac: open terminal and type >>ssh pi@192.168.0.1xx where xx is the Bluebot ID number found engraved on the right posterior end (e.g., 07)
  -	On a Windows machine: use the PuTTY SSH client
5.	Navigate to the fishfood folder >>cd fishfood
6.	Write and upload your own fun demo script or execute some existing code (e.g., tst_selfie.py, tst_ledson.py, tst_fins.py) >> python3 tst_selfie.py

### Example:
```
ssh pi@192.168.0.107
cd fishfood
python3 tst_ledson.py
```

### Optional: retrieve data that got saved, such as pictures
- On a Mac: use Cyberduck
  -	Choose the SFTP protocol
  - Server: 192.168.0.1xx (where xx is the Bluebot ID number found engraved on the right posterior end, e.g., 07)
  - Username: pi
  - Password: Tpossr236
  - Hit connect
  - Drag files to your computer’s desktop
- On a Windows machine: use the WinSCP file manager

### Don’t forget to turn the robot off!
- In terminal: >>sudo shutdown now
-	Use custom switch to turn it off (red light disappears)

### If your python-script hangs:
Abort with `gpio_cleanup_all`

### Cool demos:
-	tst_ledson.py: Switches the LEDs on for some period of time.
-	tst_fins.py: Runs fins.
-	tst_selfie.py: Takes a picture with each camera.
-	exp_luring.py: Makes Bluebot find and follow the only light source in the environment. Use a lure stick in the tank and switch off the room lights.

## Troubleshooting

### Cannot SSH into Bluebot:
- Is your computer connected to the “Blueswarm” WiFi network?
- Is the Bluebot switched on and has booted?
- Do you have the correct IP-address of the Bluebot you’re trying to connect to?
- Is the Bluebot submerged in water? WiFi doesn’t work underwater.

### My code isn’t running on Bluebot:
- Are you using libraries that might not be pre-installed?
- Are there error statements you can look up?
- Can you test sub-functions individually to isolate the problem?
- Has the environment changed, e.g., room lights?
- Did you specify that you want to execute the code with Python3, i.e., `python3 run_my_code.py`?
- Have you tried rebooting the robot?

### Bluebot sinks to the bottom of the tank:
- Is it neutrally buoyant? Tried to remove some weight blocks?
- Is it leaking?
