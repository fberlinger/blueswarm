"""Surfacepressure library, logs the surface pressure in surface_pressure.txt during boot-up such that it can be discounted from all pressure readings during operation.

Attributes:
    d (): DepthSensor object
"""
from lib_depthsensor import DepthSensor

d = DepthSensor()

with open("/home/pi/fishfood/surface_pressure.txt", 'w') as f:
    f.truncate()
    f.write(str(d.pressure_mbar))
    f.close()
