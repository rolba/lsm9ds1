from smbus2 import SMBus

class I2C_Device():
    def __init__(self, busNumber, deviceAddress):
        self._busNumber = busNumber
        self._deviceAddress = deviceAddress
        self._bus = SMBus(busNumber)
        
    def read_i2c_bytes(self, startRegister, length):
        return self._bus.read_i2c_block_data(self._deviceAddress, startRegister, length)
    
    def read_i2c_byte(self, register):
        return self._bus.read_byte_data(self._deviceAddress, register)
    
    def write_i2c_byte(self, register):
        return self._bus.read_byte_data(self._deviceAddress, register)
    
    def close_bus(self):
        self._bus.close()

class LSM9DS1():
    def __init__(self):
        pass
    
    def _read_byte(self, address, sensorType):
        pass
    
    def _read_bytes(self, address, lengts, sensorType):
        pass
    
    def _write_byte(self, address, byte, sensorType):
        pass
    
class LSM9DS1_I2C(LSM9DS1):
    
    def __init__(self, i2cBus = 1):
        self._mag_device = I2C_Device(1, 107)
        pass
    
    def _read_byte(self, address, sensorType):
        pass
    
    def _read_bytes(self, address, lengts, sensorType):
        pass
    
    def _write_byte(self, address, byte, sensorType):
        pass
    
x = LSM9DS1_I2C(1)
print(x._mag_device.read_i2c_bytes(0, 32))
x._mag_device.close_bus()


