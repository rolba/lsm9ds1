# LSM9DS1
Python library for LSM9DS1 using SMBus python lib. Work in progress.
Tested on Python 3.5

Current interfaces:
I2C

Based on https://github.com/sparkfun/SparkFun_LSM9DS1_Arduino_Library
Based on https://github.com/adafruit/Adafruit_CircuitPython_LSM9DS1

Thank guys for your code!

<b>Usage</b>
# You need SMBus to be installed
from smbus import SMBus
import LSM9DS1_I2C

# LSB ADDRESS - configured by hardware - check your board
A=1 
sensorBus = SMBus(1)
sensorInstance_A = LSM9DS1_I2C(sensorBus, A)   

accVal = sensorInstance_A.acceleration
gyroVal = sensorInstance_A.gyro
magVal = sensorInstance_A.magnetic

print(accVal, gyroVal, magVal)
