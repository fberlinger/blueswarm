# Change WiFi Location to US

This can be done though the GUI. Click the WiFi icon on the upper bar (right hand side), the change WiFi location to US.

# Camera Duplexer Board

Edit the config.txt file and disable the camera led to run the script. The switch signal is using the camera LED GPIO to select between the two cameras:

sudo nano /boot/config.txt

Add the line:

disable_camera_led=1

# WiFi Network

The WiFi location may be set to US by default

Ideally, the robot should have only one WiFi network remembered to avoid potential problems (e.g. if you operate it in the presence of another remembered network which has a stronger signal, it may connect to it). The remembered WiFi network information is stored in the file:

/etc/wpa_config/wpa_config.conf

You should see:

ssid=”BlueSwarm”
wpa=”Tpossr236”

It’s best to leave the robot with a dynamic IP confguration, so if needed to connect to another router, it can simply be assigned an IP by DHCP without having to modify the router settings.

A ‘static’ IP can still be achieved by modifying the router settings. Most routers should have an option to assign a specified IP address to a specified MAC address.

# Enabling SSH and Interfaces

Before you can SSH into the R-Pi, you need to enable the SSH server. In a terminal, type:

sudo raspi-config

Go to “Interfaces”. Go to “P2 SSH” and press Enter. When prompted whether you want the SSH server to be enabled, say Yes. After a few seconds you should get a confirmation that it has been enabled. You can now exit the configuration tool.

# SSH-ing into the BlueBot

In a terminal, type:

ssh pi@<ip_address>

Replacing <ip_address> with the robot’s IP address on the WiFi network. On our swarm this should be 192.168.0.1XX where XX is the robot’s number (01, 02, 03, etc.).

When prompted, enter the pi user’s password, which should have been set to “Tpossr236”. If this password was not set for some reason, try the default password “raspberry”.

# Enabling Interfaces

This step is crucial to enable the Raspberry Pi to communicate with its various peripherals on the robot (cameras, IMU, Transceiver, pressure sensor, photodiode ADC). In a terminal type

Sudo raspi-config

Navigate to Interfaces. Enable the Camera, SSH (if you haven’t already), SPI, I2C and Remote GPIO. Exit the config tool.

# Dropbox Configuration

TODO

# Router Settings

The BlueBot is configured to look for a WiFi network with the SSID “BlueSwarm” and password “Tpossr236” (capitalization is important!).

# Pins

Caudal fin: 20, 21
Dorsal fin: 19, 26
Pectoral left fin: 18, 23
Pectoral right fin: 4, 22

LED: 13

Photodiode: 27

# TODO for next version

- Remove or cover red camera LEDs
- Test LEDs
- Smaller resistors for LEDs, use PWM to control brightness
- Change charging pin
- Change PCB (op-amp fix, ADC fix, new voltage reading cct)

# TODO other
Tracking (run test to see if color is trackable)
