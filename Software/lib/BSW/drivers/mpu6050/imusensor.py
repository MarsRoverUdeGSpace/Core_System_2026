# /lib/BSW/drivers/mpu6050/imusensor.py
# Wrapper con driver mínimo puro MicroPython (sin ctypes)

from machine import I2C

_ADDR = 0x68
_PWR_MGMT_1   = 0x6B
_ACCEL_XOUT_H = 0x3B
_GYRO_XOUT_H  = 0x43
_TEMP_OUT_H   = 0x41

def _twos(h, l):
    v = (h << 8) | l
    return v - 65536 if v & 0x8000 else v

class ImuSensor:
    def __init__(self, i2c: I2C, addr=_ADDR):
        self.i2c = i2c
        self.addr = addr
        # Wake up
        self.i2c.writeto_mem(self.addr, _PWR_MGMT_1, b"\x00")

    def _read_vec(self, start_reg):
        data = self.i2c.readfrom_mem(self.addr, start_reg, 6)
        return (_twos(data[0], data[1]),
                _twos(data[2], data[3]),
                _twos(data[4], data[5]))

    @property
    def accel(self):
        ax, ay, az = self._read_vec(_ACCEL_XOUT_H)
        # ±2g => 16384 LSB/g
        return (ax/16384.0, ay/16384.0, az/16384.0)

    @property
    def gyro(self):
        gx, gy, gz = self._read_vec(_GYRO_XOUT_H)
        # ±250 dps => 131 LSB/(°/s)
        return (gx/131.0, gy/131.0, gz/131.0)

    @property
    def temperature_c(self):
        t = self.i2c.readfrom_mem(self.addr, _TEMP_OUT_H, 2)
        tr = _twos(t[0], t[1])
        return tr/340.0 + 36.53
