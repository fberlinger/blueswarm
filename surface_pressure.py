from psens_01 import DepthSensor

d = DepthSensor()

with open("surface_pressure.txt", 'w') as f:
    f.truncate()
    f.write(str(d.pressure_mbar))
    f.close()
