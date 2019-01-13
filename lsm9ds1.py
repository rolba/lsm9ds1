from smbus import SMBus
import RPi.GPIO as GPIO
import struct
import time

# Internal ants and register values:
# pylint: disable=bad-whitespace
_LSM9DS1_ADDRESS_ACCELGYRO       = 0x6B
_LSM9DS1_ADDRESS_MAG             = 0x1E
_LSM9DS1_XG_ID                   = 0b01101000
_LSM9DS1_MAG_ID                  = 0b00111101
_LSM9DS1_ACCEL_MG_LSB_2G         = 0.061
_LSM9DS1_ACCEL_MG_LSB_4G         = 0.122
_LSM9DS1_ACCEL_MG_LSB_8G         = 0.244
_LSM9DS1_ACCEL_MG_LSB_16G        = 0.732
_LSM9DS1_MAG_MGAUSS_4GAUSS       = 0.14
_LSM9DS1_MAG_MGAUSS_8GAUSS       = 0.29
_LSM9DS1_MAG_MGAUSS_12GAUSS      = 0.43
_LSM9DS1_MAG_MGAUSS_16GAUSS      = 0.58
_LSM9DS1_GYRO_DPS_DIGIT_245DPS   = 0.00875
_LSM9DS1_GYRO_DPS_DIGIT_500DPS   = 0.01750
_LSM9DS1_GYRO_DPS_DIGIT_2000DPS  = 0.07000
_LSM9DS1_TEMP_LSB_DEGREE_CELSIUS = 8 # 1°C = 8, 25° = 200, etc.
_LSM9DS1_REGISTER_WHO_AM_I_XG    = 0x0F
_LSM9DS1_REGISTER_CTRL_REG1_G    = 0x10
_LSM9DS1_REGISTER_CTRL_REG2_G    = 0x11
_LSM9DS1_REGISTER_CTRL_REG3_G    = 0x12
_LSM9DS1_REGISTER_TEMP_OUT_L     = 0x15
_LSM9DS1_REGISTER_TEMP_OUT_H     = 0x16
_LSM9DS1_REGISTER_STATUS_REG     = 0x17
_LSM9DS1_REGISTER_OUT_X_L_G      = 0x18
_LSM9DS1_REGISTER_OUT_X_H_G      = 0x19
_LSM9DS1_REGISTER_OUT_Y_L_G      = 0x1A
_LSM9DS1_REGISTER_OUT_Y_H_G      = 0x1B
_LSM9DS1_REGISTER_OUT_Z_L_G      = 0x1C
_LSM9DS1_REGISTER_OUT_Z_H_G      = 0x1D
_LSM9DS1_REGISTER_CTRL_REG4      = 0x1E
_LSM9DS1_REGISTER_CTRL_REG5_XL   = 0x1F
_LSM9DS1_REGISTER_CTRL_REG6_XL   = 0x20
_LSM9DS1_REGISTER_CTRL_REG7_XL   = 0x21
_LSM9DS1_REGISTER_CTRL_REG8      = 0x22
_LSM9DS1_REGISTER_CTRL_REG9      = 0x23
_LSM9DS1_REGISTER_CTRL_REG10     = 0x24
_LSM9DS1_REGISTER_OUT_X_L_XL     = 0x28
_LSM9DS1_REGISTER_OUT_X_H_XL     = 0x29
_LSM9DS1_REGISTER_OUT_Y_L_XL     = 0x2A
_LSM9DS1_REGISTER_OUT_Y_H_XL     = 0x2B
_LSM9DS1_REGISTER_OUT_Z_L_XL     = 0x2C
_LSM9DS1_REGISTER_OUT_Z_H_XL     = 0x2D
_LSM9DS1_REGISTER_WHO_AM_I_M     = 0x0F
_LSM9DS1_REGISTER_CTRL_REG1_M    = 0x20
_LSM9DS1_REGISTER_CTRL_REG2_M    = 0x21
_LSM9DS1_REGISTER_CTRL_REG3_M    = 0x22
_LSM9DS1_REGISTER_CTRL_REG4_M    = 0x23
_LSM9DS1_REGISTER_CTRL_REG5_M    = 0x24
_LSM9DS1_REGISTER_STATUS_REG_M   = 0x27
_LSM9DS1_REGISTER_OUT_X_L_M      = 0x28
_LSM9DS1_REGISTER_OUT_X_H_M      = 0x29
_LSM9DS1_REGISTER_OUT_Y_L_M      = 0x2A
_LSM9DS1_REGISTER_OUT_Y_H_M      = 0x2B
_LSM9DS1_REGISTER_OUT_Z_L_M      = 0x2C
_LSM9DS1_REGISTER_OUT_Z_H_M      = 0x2D
_LSM9DS1_REGISTER_CFG_M          = 0x30
_LSM9DS1_REGISTER_INT_SRC_M      = 0x31
_MAGTYPE                         = True
_XGTYPE                          = False
_SENSORS_GRAVITY_STANDARD        = 9.80665

# User facing ants/module globals.
ACCELRANGE_2G                = 0b00 << 3
ACCELRANGE_16G               = 0b01 << 3
ACCELRANGE_4G                = 0b10 << 3
ACCELRANGE_8G                = 0b11 << 3
MAGGAIN_4GAUSS               = 0b00 << 5  # +/- 4 gauss
MAGGAIN_8GAUSS               = 0b01 << 5  # +/- 8 gauss
MAGGAIN_12GAUSS              = 0b10 << 5  # +/- 12 gauss
MAGGAIN_16GAUSS              = 0b11 << 5  # +/- 16 gauss
GYROSCALE_245DPS             = 0b00 << 3  # +/- 245 degrees/s rotation
GYROSCALE_500DPS             = 0b01 << 3  # +/- 500 degrees/s rotation
GYROSCALE_2000DPS            = 0b11 << 3  # +/- 2000 degrees/s rotation
# pylint: enable=bad-whitespace

class I2CDevice():
    def __init__(self, bus, address):
        self.bus = bus
        self.address = address
        
    def read(self, cmd, lenght):
        if lenght<2:
            return self.bus.read_byte_data(self.address, cmd)
        else:
            return self.bus.read_i2c_block_data(self.address, cmd, lenght)
        
    def write(self, cmd, data = []):
        if len(data)<2:
            self.bus.write_byte_data(self.address, cmd, data[0])
        else:
            self.bus.write_i2c_block_data(self.address, cmd, data)
            

class LSM9DS1():
    def __init__(self):
         # soft reset & reboot accel/gyro
        self._write_byte(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG8, [0x05])
        # soft reset & reboot magnetometer
        self._write_byte(_MAGTYPE, _LSM9DS1_REGISTER_CTRL_REG2_M, [0x0C])
        time.sleep(0.1)
         # Check ID registers.
        if self._read_byte(_XGTYPE, _LSM9DS1_REGISTER_WHO_AM_I_XG) != _LSM9DS1_XG_ID or \
           self._read_byte(_MAGTYPE, _LSM9DS1_REGISTER_WHO_AM_I_M) != _LSM9DS1_MAG_ID:
            raise RuntimeError('Could not find LSM9DS1, check wiring!')
        
        # enable gyro continuous
        self._write_byte(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG1_G, [0xC0]) # on XYZ
        # Enable the accelerometer continous
        self._write_byte(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG5_XL, [0x38])
        self._write_byte(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG6_XL, [0xC0])
        # enable mag continuous
        self._write_byte(_MAGTYPE, _LSM9DS1_REGISTER_CTRL_REG3_M, [0x00])
        # Set default ranges for the various sensors
        self._accel_mg_lsb = None
        self._mag_mgauss_lsb = None
        self._gyro_dps_digit = None
        self.accel_range = ACCELRANGE_2G
        self.mag_gain = MAGGAIN_4GAUSS
        self.gyro_scale = GYROSCALE_245DPS

    @property
    def accel_range(self):
        """The accelerometer range.  Must be a value of:
          - ACCELRANGE_2G
          - ACCELRANGE_4G
          - ACCELRANGE_8G
          - ACCELRANGE_16G
        """
        reg = self._read_byte(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG6_XL)
        return (reg & 0b00011000) & 0xFF
    
    @accel_range.setter
    def accel_range(self, val):
        assert val in (ACCELRANGE_2G, ACCELRANGE_4G, ACCELRANGE_8G,
                       ACCELRANGE_16G)
        reg = self._read_byte(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG6_XL)
        reg = (reg & ~(0b00011000)) & 0xFF
        reg |= val
        self._write_byte(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG6_XL, [reg])
        if val == ACCELRANGE_2G:
            self._accel_mg_lsb = _LSM9DS1_ACCEL_MG_LSB_2G
        elif val == ACCELRANGE_4G:
            self._accel_mg_lsb = _LSM9DS1_ACCEL_MG_LSB_4G
        elif val == ACCELRANGE_8G:
            self._accel_mg_lsb = _LSM9DS1_ACCEL_MG_LSB_8G
        elif val == ACCELRANGE_16G:
            self._accel_mg_lsb = _LSM9DS1_ACCEL_MG_LSB_16G
    
    @property
    def mag_gain(self):
        """The magnetometer gain.  Must be a value of:
          - MAGGAIN_4GAUSS
          - MAGGAIN_8GAUSS
          - MAGGAIN_12GAUSS
          - MAGGAIN_16GAUSS
        """
        reg = self._read_byte(_MAGTYPE, _LSM9DS1_REGISTER_CTRL_REG2_M)
        return (reg & 0b01100000) & 0xFF

    @mag_gain.setter
    def mag_gain(self, val):
        assert val in (MAGGAIN_4GAUSS, MAGGAIN_8GAUSS, MAGGAIN_12GAUSS,
                       MAGGAIN_16GAUSS)
        reg = self._read_byte(_MAGTYPE, _LSM9DS1_REGISTER_CTRL_REG2_M)
        reg = (reg & ~(0b01100000)) & 0xFF
        reg |= val
        self._write_byte(_MAGTYPE, _LSM9DS1_REGISTER_CTRL_REG2_M, [reg])
        if val == MAGGAIN_4GAUSS:
            self._mag_mgauss_lsb = _LSM9DS1_MAG_MGAUSS_4GAUSS
        elif val == MAGGAIN_8GAUSS:
            self._mag_mgauss_lsb = _LSM9DS1_MAG_MGAUSS_8GAUSS
        elif val == MAGGAIN_12GAUSS:
            self._mag_mgauss_lsb = _LSM9DS1_MAG_MGAUSS_12GAUSS
        elif val == MAGGAIN_16GAUSS:
            self._mag_mgauss_lsb = _LSM9DS1_MAG_MGAUSS_16GAUSS
            
    @property
    def gyro_scale(self):
        """The gyroscope scale.  Must be a value of:
          - GYROSCALE_245DPS
          - GYROSCALE_500DPS
          - GYROSCALE_2000DPS
        """
        reg = self._read_byte(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG1_G)
        return (reg & 0b00011000) & 0xFF

    @gyro_scale.setter
    def gyro_scale(self, val):
        assert val in (GYROSCALE_245DPS, GYROSCALE_500DPS, GYROSCALE_2000DPS)
        reg = self._read_byte(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG1_G)
        reg = (reg & ~(0b00011000)) & 0xFF
        reg |= val
        self._write_byte(_XGTYPE, _LSM9DS1_REGISTER_CTRL_REG1_G, [reg])
        if val == GYROSCALE_245DPS:
            self._gyro_dps_digit = _LSM9DS1_GYRO_DPS_DIGIT_245DPS
        elif val == GYROSCALE_500DPS:
            self._gyro_dps_digit = _LSM9DS1_GYRO_DPS_DIGIT_500DPS
        elif val == GYROSCALE_2000DPS:
            self._gyro_dps_digit = _LSM9DS1_GYRO_DPS_DIGIT_2000DPS
            
    def read_accel_raw(self):
        """Read the raw accelerometer sensor values and return it as a
        3-tuple of X, Y, Z axis values that are 16-bit unsigned values.  If you
        want the acceleration in nice units you probably want to use the
        accelerometer property!
        """
        # Read the accelerometer
        raw_list = self._read_bytes(_XGTYPE, 0x80 | _LSM9DS1_REGISTER_OUT_X_L_XL, 6)
        raw_x, raw_y, raw_z = struct.unpack_from('<hhh', bytes(raw_list))
        return (raw_x, raw_y, raw_z)

    @property
    def acceleration(self):
        """The accelerometer X, Y, Z axis values as a 3-tuple of
        m/s^2 values.
        """
        raw = self.read_accel_raw()
        return list(map(lambda x: x * self._accel_mg_lsb / 1000.0 * _SENSORS_GRAVITY_STANDARD, raw))

    def read_mag_raw(self):
        """Read the raw magnetometer sensor values and return it as a
        3-tuple of X, Y, Z axis values that are 16-bit unsigned values.  If you
        want the magnetometer in nice units you probably want to use the
        magnetometer property!
        """
        # Read the magnetometer
        raw_list = self._read_bytes(_MAGTYPE, 0x80 | _LSM9DS1_REGISTER_OUT_X_L_M, 6)
        raw_x, raw_y, raw_z = struct.unpack_from('<hhh', bytes(raw_list))
        return (raw_x, raw_y, raw_z)

    @property
    def magnetic(self):
        """The magnetometer X, Y, Z axis values as a 3-tuple of
        gauss values.
        """
        raw = self.read_mag_raw()
        return list(map(lambda x: x * self._mag_mgauss_lsb / 1000.0, raw))

    def read_gyro_raw(self):
        """Read the raw gyroscope sensor values and return it as a
        3-tuple of X, Y, Z axis values that are 16-bit unsigned values.  If you
        want the gyroscope in nice units you probably want to use the
        gyroscope property!
        """
        # Read the gyroscope
        raw_list = self._read_bytes(_XGTYPE, 0x80 | _LSM9DS1_REGISTER_OUT_X_L_G, 6)
        raw_x, raw_y, raw_z = struct.unpack_from('<hhh', bytes(raw_list))
        return (raw_x, raw_y, raw_z)

    @property
    def gyro(self):
        """The gyroscope X, Y, Z axis values as a 3-tuple of
        degrees/second values.
        """
        raw = self.read_gyro_raw()
        return list(map(lambda x: x * self._gyro_dps_digit, raw))
      
    def _write_byte(self, sensorType, register, data = []):
        raise NotImplementedError()
    
    def _read_byte(self, sensorType, register):
        raise NotImplementedError()
    
    def _read_bytes(self, sensorType, register, count):
        raise NotImplementedError()


class LSM9DS1_I2C(LSM9DS1):
    def __init__(self, i2cBus):
        self._mag_device = I2CDevice(i2cBus, _LSM9DS1_ADDRESS_MAG)
        self._xg_device =  I2CDevice(i2cBus, _LSM9DS1_ADDRESS_ACCELGYRO)
        super().__init__()
    
    def _write_byte(self, sensorType, register, data = []):
        if sensorType == _MAGTYPE:
            self._mag_device.write(register, data)
        else:
            self._xg_device.write(register, data)
    
    def _read_byte(self, sensorType, register):
        if sensorType == _MAGTYPE:
            return self._mag_device.read(register, 1)
        else:
            return self._xg_device.read(register, 1)
    
    def _read_bytes(self, sensorType, register, count):
        if sensorType == _MAGTYPE:
            return self._mag_device.read(register, count)
        else:
            return self._xg_device.read(register, count)
        
 