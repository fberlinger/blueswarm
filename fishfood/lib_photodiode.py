"""Photodiode library, reads photodiode value.

Attributes:
    bus (TYPE): Description
"""
import RPi.GPIO as GPIO
import time
import smbus
bus = smbus.SMBus(1) # get I2C bus

class Photodiode():

    """Photodiode measures the brightness in the environment
    
    Attributes:
        brightness (float): Light intensity read from photodiode
    """
    
    def __init__(self):
        """Photodiode is on I2C bus
        """
        self.brightness = 0
        self.update()

    def update(self):
        """Read light intensity from photodiode
        """
        # ADC121C021 address, 0x50(80)
        # Select configuration register, 0x02(02)
        # 0x20(32) Automatic conversion mode enabled
        bus.write_byte_data(0x52, 0x02, 0x20) # 0x50 on BlueBot01

        # ADC121C021 address, 0x50(80)
        # Read data back from 0x00(00), 2 bytes
        # raw_adc MSB, raw_adc LSB
        data = bus.read_i2c_block_data(0x52, 0x00, 2)

        # Convert the data to 12-bits
        raw_adc = (data[0] & 0x0F) * 256 + data[1]

        self.brightness = raw_adc
