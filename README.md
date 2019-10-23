# LSM9DS1
Python library for LSM9DS1 using SMBus python lib. Work in progress.
Tested on Python 3.5
<br>
Current interfaces:
I2C
<br>
Based on https://github.com/sparkfun/SparkFun_LSM9DS1_Arduino_Library<br>
Based on https://github.com/adafruit/Adafruit_CircuitPython_LSM9DS1<br>

Thank guys for your code!<br>

<b>Usage</b><br>
#You need SMBus to be installed<br>
from smbus import SMBus<br>
import LSM9DS1_I2C<br>
<br>
#LSB ADDRESS - configured by hardware - check your board<br>
A=1 <br>
sensorBus = SMBus(1)<br>
sensorInstance_A = LSM9DS1_I2C(sensorBus, A)   <br>
<br>
accVal = sensorInstance_A.acceleration<br>
gyroVal = sensorInstance_A.gyro<br>
magVal = sensorInstance_A.magnetic<br>
<br>
print(accVal, gyroVal, magVal)
