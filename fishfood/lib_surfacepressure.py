from lib_depthsensor import DepthSensor

d = DepthSensor()

with open("/home/pi/fishfood/surface_pressure.txt", 'w') as f:
    f.truncate()
    f.write(str(d.pressure_mbar))
    f.close()
