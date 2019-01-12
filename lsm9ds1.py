from smbus import SMBus
import RPi.GPIO as GPIO
import time
ACT_THS             = 0x04
ACT_DUR             = 0x05
INT_GEN_CFG_XL      = 0x06
INT_GEN_THS_X_XL    = 0x07
INT_GEN_THS_Y_XL    = 0x08
INT_GEN_THS_Z_XL    = 0x09
INT_GEN_DUR_XL      = 0x0A
REFERENCE_G         = 0x0B
INT1_CTRL           = 0x0C
INT2_CTRL           = 0x0D
WHO_AM_I_XG         = 0x0F
CTRL_REG1_G         = 0x10
CTRL_REG2_G         = 0x11
CTRL_REG3_G         = 0x12
ORIENT_CFG_G        = 0x13
INT_GEN_SRC_G       = 0x14
OUT_TEMP_L          = 0x15
OUT_TEMP_H          = 0x16
STATUS_REG_0        = 0x17
OUT_X_L_G           = 0x18
OUT_X_H_G           = 0x19
OUT_Y_L_G           = 0x1A
OUT_Y_H_G           = 0x1B
OUT_Z_L_G           = 0x1C
OUT_Z_H_G           = 0x1D
CTRL_REG4           = 0x1E
CTRL_REG5_XL        = 0x1F
CTRL_REG6_XL        = 0x20
CTRL_REG7_XL        = 0x21
CTRL_REG8           = 0x22
CTRL_REG9           = 0x23
CTRL_REG10          = 0x24
INT_GEN_SRC_XL      = 0x26
STATUS_REG_1        = 0x27
OUT_X_L_XL          = 0x28
OUT_X_H_XL          = 0x29
OUT_Y_L_XL          = 0x2A
OUT_Y_H_XL          = 0x2B
OUT_Z_L_XL          = 0x2C
OUT_Z_H_XL          = 0x2D
FIFO_CTRL           = 0x2E
FIFO_SRC            = 0x2F
INT_GEN_CFG_G       = 0x30
INT_GEN_THS_XH_G    = 0x31
INT_GEN_THS_XL_G    = 0x32
INT_GEN_THS_YH_G    = 0x33
INT_GEN_THS_YL_G    = 0x34
INT_GEN_THS_ZH_G    = 0x35
INT_GEN_THS_ZL_G    = 0x36
INT_GEN_DUR_G       = 0x37


OFFSET_X_REG_L_M    = 0x05
OFFSET_X_REG_H_M    = 0x06
OFFSET_Y_REG_L_M    = 0x07
OFFSET_Y_REG_H_M    = 0x08
OFFSET_Z_REG_L_M    = 0x09
OFFSET_Z_REG_H_M    = 0x0A
WHO_AM_I_M          = 0x0F
CTRL_REG1_M         = 0x20
CTRL_REG2_M         = 0x21
CTRL_REG3_M         = 0x22
CTRL_REG4_M         = 0x23
CTRL_REG5_M         = 0x24
STATUS_REG_M        = 0x27
OUT_X_L_M           = 0x28
OUT_X_H_M           = 0x29
OUT_Y_L_M           = 0x2A
OUT_Y_H_M           = 0x2B
OUT_Z_L_M           = 0x2C
OUT_Z_H_M           = 0x2D
INT_CFG_M           = 0x30
INT_SRC_M           = 0x31
INT_THS_L_M         = 0x32
INT_THS_H_M         = 0x33

WHO_AM_I_AG_RSP     = 0x68
WHO_AM_I_M_RSP      = 0x3D

SENSITIVITY_ACCELEROMETER_2  = 0.000061
SENSITIVITY_ACCELEROMETER_4  = 0.000122
SENSITIVITY_ACCELEROMETER_8  = 0.000244
SENSITIVITY_ACCELEROMETER_16 = 0.000732
SENSITIVITY_GYROSCOPE_245    = 0.00875
SENSITIVITY_GYROSCOPE_500    = 0.0175
SENSITIVITY_GYROSCOPE_2000   = 0.07
SENSITIVITY_MAGNETOMETER_4   = 0.00014
SENSITIVITY_MAGNETOMETER_8   = 0.00029
SENSITIVITY_MAGNETOMETER_12  = 0.00043
SENSITIVITY_MAGNETOMETER_16  = 0.00058

class gyro():
    def __init__(self):

        self.gyro_enabled = True;
        self.gyro_enableX = True;
        self.gyro_enableY = True;
        self.gyro_enableZ = True;
        # gyro scale can be 245, 500, or 2000
        self.gyro_scale = 245;
        # gyro sample rate: value between 1-6
        # 1 = 14.9    4 = 238
        # 2 = 59.5    5 = 476
        # 3 = 119     6 = 952
        self.gyro_sampleRate = 6;
        # gyro cutoff frequency: value between 0-3
        # Actual value of cutoff frequency depends
        # on sample rate.
        self.gyro_bandwidth = 0;
        self.gyro_lowPowerEnable = False  ;
        self.gyro_HPFEnable = False ;
        # Gyro HPF cutoff frequency: value between 0-9
        # Actual value depends on sample rate. Only applies
        # if gyroHPFEnable is True.
        self.gyro_HPFCutoff = 0;
        self.gyro_flipX = False ;
        self.gyro_flipY = False ;
        self.gyro_flipZ = False ;
        self.gyro_orientation = 0;
        self.gyro_latchInterrupt = True;
    
class accel():
    def __init__(self):

        self.accel_enabled = True;
        self.accel_enableX = True;
        self.accel_enableY = True;
        self.accel_enableZ = True;
        # accel scale can be 2, 4, 8, or 16
        self.accel_scale = 2;
        # accel sample rate can be 1-6
        # 1 = 10 Hz    4 = 238 Hz
        # 2 = 50 Hz    5 = 476 Hz
        # 3 = 119 Hz   6 = 952 Hz
        self.accel_sampleRate = 6;
        # Accel cutoff freqeuncy can be any value between -1 - 3. 
        # -1 = bandwidth determined by sample rate
        # 0 = 408 Hz   2 = 105 Hz
        # 1 = 211 Hz   3 = 50 Hz
        self.accel_bandwidth = -1;
        self.accel_highResEnable = false;
        # accelHighResBandwidth can be any value between 0-3
        # LP cutoff is set to a factor of sample rate
        # 0 = ODR/50    2 = ODR/9
        # 1 = ODR/100   3 = ODR/400
        self.accel_highResBandwidth = 0;

class mag():
    def __init(self):
        self.mag_enabled = true;
        # mag scale can be 4, 8, 12, or 16
        self.mag_scale = 4;
        # mag data rate can be 0-7
        # 0 = 0.625 Hz  4 = 10 Hz
        # 1 = 1.25 Hz   5 = 20 Hz
        # 2 = 2.5 Hz    6 = 40 Hz
        # 3 = 5 Hz      7 = 80 Hz
        self.mag_sampleRate = 7;
        self.mag_tempCompensationEnable = false;
        # magPerformance can be any value between 0-3
        # 0 = Low power mode      2 = high performance
        # 1 = medium performance  3 = ultra-high performance
        self.mag_XYPerformance = 3;
        self.mag_ZPerformance = 3;
        self.mag_lowPowerEnable = false;
        # magOperatingMode can be 0-2
        # 0 = continuous conversion
        # 1 = single-conversion
        # 2 = power down
        self.mag_operatingMode = 0;

class LSM9DS1():
    def __init__(self, i2c=None, g_addr=0x6B, xm_addr=0x1e):
        if not (i2c):
            raise Exception("must have an i2c or spi object")
        self.i2c = i2c
        self.g_addr = g_addr
        self.xm_addr = xm_addr

#         self.write_reg(self.g_addr, CTRL_REG1_G, 0b00001111)


    def write_reg(self, addr, reg, data):
        self.i2c.write_byte_data(addr, reg, data)
            
    def read_reg(self, addr, reg):
        return self.i2c.read_byte_data(addr, reg)


bus = SMBus(1)

x = LSM9DS1(bus)
print(x.read_reg(x.xm_addr, WHO_AM_I_M))
print(x.read_reg(x.g_addr, WHO_AM_I_XG))

bus.close()
