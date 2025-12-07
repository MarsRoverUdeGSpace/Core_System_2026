
# mpu6500.py — Minimal MicroPython driver for MPU6500 (WHO_AM_I=0x70)
# Works on ESP32/ESP8266 using machine.I2C. Accel+Gyro+Temp (no magnetometer).
# Public domain / CC0.

from micropython import const

_DEFAULT_ADDR     = const(0x68)
_REG_WHO_AM_I     = const(0x75)
_REG_PWR_MGMT_1   = const(0x6B)
_REG_SMPLRT_DIV   = const(0x19)
_REG_CONFIG       = const(0x1A)   # DLPF
_REG_GYRO_CONFIG  = const(0x1B)
_REG_ACCEL_CONFIG = const(0x1C)
_REG_ACCEL_CFG2   = const(0x1D)

_REG_ACCEL_OUT    = const(0x3B)   # 6 bytes
_REG_TEMP_OUT     = const(0x41)   # 2 bytes
_REG_GYRO_OUT     = const(0x43)   # 6 bytes

# Accel full-scale
AFS_2G, AFS_4G, AFS_8G, AFS_16G = 0, 1, 2, 3
# Gyro full-scale
GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS = 0, 1, 2, 3

class MPU6500:
    """
    Minimal driver:
      i2c      : machine.I2C
      addr     : 0x68 (AD0=0) or 0x69 (AD0=1)
      afs/gfs  : accel/gyro range enums (above)
      dlpf     : 0..7 (lower = more filtering; 3 is a sensible default)
      odr_div  : sample-rate divider; sample_rate = 1kHz / (1 + odr_div)
    """
    def __init__(self, i2c, addr=_DEFAULT_ADDR, afs=AFS_2G, gfs=GFS_250DPS, dlpf=3, odr_div=9, check_id=True):
        self.i2c = i2c
        self.addr = addr
        # Wake device
        self._w(_REG_PWR_MGMT_1, b'\x00')
        # DLPF
        self._w(_REG_CONFIG, bytes([dlpf & 7]))
        # Ranges
        self.accel_range(afs)
        self.gyro_range(gfs)
        # ODR
        self._w(_REG_SMPLRT_DIV, bytes([odr_div & 0xFF]))
        # Accel config 2 (defaults OK)
        self._w(_REG_ACCEL_CFG2, b'\x00')

        if check_id:
            who = self._r(_REG_WHO_AM_I, 1)[0]
            if who != 0x70:
                raise OSError("Unexpected WHO_AM_I 0x%02X (expect 0x70)" % who)

        # Optional user calibration offsets (in raw LSB)
        self.accel_offset = (0, 0, 0)
        self.gyro_offset  = (0, 0, 0)

    # -------- Configuration ----------
    def accel_range(self, afs):
        self._w(_REG_ACCEL_CONFIG, bytes([(afs & 3) << 3]))
        self._acc_scale = {AFS_2G:16384.0, AFS_4G:8192.0, AFS_8G:4096.0, AFS_16G:2048.0}[afs]

    def gyro_range(self, gfs):
        self._w(_REG_GYRO_CONFIG, bytes([(gfs & 3) << 3]))
        self._gyro_scale = {GFS_250DPS:131.0, GFS_500DPS:65.5, GFS_1000DPS:32.8, GFS_2000DPS:16.4}[gfs]

    # -------- Measurements ----------
    @property
    def accel(self):
        x, y, z = self._v3(_REG_ACCEL_OUT)
        ox, oy, oz = self.accel_offset
        return ((x-ox)/self._acc_scale, (y-oy)/self._acc_scale, (z-oz)/self._acc_scale)

    @property
    def gyro(self):
        x, y, z = self._v3(_REG_GYRO_OUT)
        ox, oy, oz = self.gyro_offset
        return ((x-ox)/self._gyro_scale, (y-oy)/self._gyro_scale, (z-oz)/self._gyro_scale)

    @property
    def temperature(self):
        # Temp(°C) = TEMP_OUT/333.87 + 21
        t = self._i16(self._r(_REG_TEMP_OUT, 2))
        return t/333.87 + 21.0

    # -------- Low-level -------------
    def _r(self, reg, n):
        return self.i2c.readfrom_mem(self.addr, reg, n)

    def _w(self, reg, data):
        self.i2c.writeto_mem(self.addr, reg, data)

    @staticmethod
    def _i16(bb):
        v = (bb[0] << 8) | bb[1]
        return v - 65536 if v & 0x8000 else v

    def _v3(self, reg):
        d = self._r(reg, 6)
        return (self._i16(d[0:2]), self._i16(d[2:4]), self._i16(d[4:6]))
