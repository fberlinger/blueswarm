# Distributed with a free-will license.
# Use it any way you want, profit or free, provided it fits in the licenses of its associated works.
# MS5803_02BA
# This code is designed to work with the MS5803_01BA_I2CS I2C Mini Module available from ControlEverything.com.
# https://www.controleverything.com/content/Analog-Digital-Converters?sku=MS5803-02BA_I2CS#tabs-0-product_tabset-2

import smbus
import time

class DepthSensor():
	def __init__(self):
		self.pressure_mbar = 0
		self.temperature_celsius = 0
		self.depth_mm = 0

		try:
			f = open("surface_pressure.txt", 'r')
			s = f.readline()
			self._surface_pressure = float(s)
		except:
			self._surface_pressure = 0

		# Get I2C bus
		bus = smbus.SMBus(1)

		# MS5803_02BA address, 0x77(118)
		#		0x1E(30)	Reset command
		bus.write_byte(0x77, 0x1E)

		time.sleep(0.5)

		# Read 12 bytes of calibration data
		# Read pressure sensitivity
		data = bus.read_i2c_block_data(0x77, 0xA2, 2)
		self._C1 = data[0] * 256 + data[1]

		# Read pressure offset
		data = bus.read_i2c_block_data(0x77, 0xA4, 2)
		self._C2 = data[0] * 256 + data[1]

		# Read temperature coefficient of pressure sensitivity
		data = bus.read_i2c_block_data(0x77, 0xA6, 2)
		self._C3 = data[0] * 256 + data[1]

		# Read temperature coefficient of pressure offset
		data = bus.read_i2c_block_data(0x77, 0xA8, 2)
		self._C4 = data[0] * 256 + data[1]

		# Read reference temperature
		data = bus.read_i2c_block_data(0x77, 0xAA, 2)
		self._C5 = data[0] * 256 + data[1]

		# Read temperature coefficient of the temperature
		data = bus.read_i2c_block_data(0x77, 0xAC, 2)
		self._C6 = data[0] * 256 + data[1]

		# MS5803_02BA address, 0x77(118)
		#		0x1E(30)	Reset command
		bus.write_byte(0x77, 0x1E)

		time.sleep(0.5)

		self.update()

	def update(self):
		C1 = self._C1
		C2 = self._C2
		C3 = self._C3
		C4 = self._C4
		C5 = self._C5
		C6 = self._C6

		# Get I2C bus
		bus = smbus.SMBus(1)

		# MS5803_02BA address, 0x77(118)
		#		0x40(64)	Pressure conversion(OSR = 256) command
		bus.write_byte(0x77, 0x40)

		time.sleep(0.0006)

		# Read digital pressure value
		# Read data back from 0x00(0), 3 bytes
		# D1 MSB2, D1 MSB1, D1 LSB
		value = bus.read_i2c_block_data(0x77, 0x00, 3)
		D1 = value[0] * 65536 + value[1] * 256 + value[2]

		# MS5803_02BA address, 0x77(118)
		#		0x50(64)	Temperature conversion(OSR = 256) command
		bus.write_byte(0x77, 0x50)

		time.sleep(0.0006)

		# Read digital temperature value
		# Read data back from 0x00(0), 3 bytes
		# D2 MSB2, D2 MSB1, D2 LSB
		value = bus.read_i2c_block_data(0x77, 0x00, 3)
		D2 = value[0] * 65536 + value[1] * 256 + value[2]

		dT = D2 - C5 * 256
		TEMP = 2000 + dT * C6 / 8388608
		OFF = C2 * 131072 + (C4 * dT) / 64
		SENS = C1 * 65536 + (C3 * dT ) / 128
		T2 = 0
		OFF2 = 0
		SENS2 = 0

		if TEMP >= 2000 :
			T2 = 0
			OFF2 = 0
			SENS2 = 0
		elif TEMP < 2000 :
			T2 = (dT * dT) / 2147483648
			OFF2= 61 * ((TEMP - 2000) * (TEMP - 2000)) / 16
			SENS2= 2 * ((TEMP - 2000) * (TEMP - 2000))
			if TEMP < -1500 :
				OFF2 = OFF2 + 20 * ((TEMP + 1500) * (TEMP + 1500))
				SENS2 = SENS2 + 12 * ((TEMP + 1500) * (TEMP +1500))


		TEMP = TEMP - T2
		OFF = OFF - OFF2
		SENS = SENS - SENS2
		pressure = ((((D1 * SENS) / 2097152) - OFF) / 32768.0) / 100.0
		cTemp = TEMP / 100.0
		# fTemp = cTemp * 1.8 + 32

		self.pressure_mbar = pressure
		self.temperature_celsius = cTemp
		self.depth_mm = max(0, (pressure - self._surface_pressure) * 10.197162129779)
