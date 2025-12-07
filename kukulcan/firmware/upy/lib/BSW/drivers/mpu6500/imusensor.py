from BSW.drivers.mpu6500.mpu6500 import MPU6500, AFS_4G, GFS_500DPS
from time import sleep_ms

class ImuSensor:
    """MPU6500 convenience wrapper."""

    def __init__(self, i2c, afs=AFS_4G, gfs=GFS_500DPS, dlpf=3, odr_div=9):
        self._imu = MPU6500(i2c, afs=afs, gfs=gfs, dlpf=dlpf, odr_div=odr_div)

    def calibrate(self, samples=300, delay_ms=2):
        ax = ay = az = gx = gy = gz = 0.0
        for _ in range(samples):
            a_x, a_y, a_z = self._imu.accel
            g_x, g_y, g_z = self._imu.gyro
            ax += a_x; ay += a_y; az += a_z
            gx += g_x; gy += g_y; gz += g_z
            sleep_ms(delay_ms)
        inv = 1.0 / samples
        ax, ay, az = ax*inv, ay*inv, az*inv
        gx, gy, gz = gx*inv, gy*inv, gz*inv

        self._imu.accel_offset = (
            int(ax * self._imu._acc_scale),
            int(ay * self._imu._acc_scale),
            int((az - 1.0) * self._imu._acc_scale),
        )
        self._imu.gyro_offset = (
            int(gx * self._imu._gyro_scale),
            int(gy * self._imu._gyro_scale),
            int(gz * self._imu._gyro_scale),
        )

    @property
    def accel(self): return self._imu.accel
    @property
    def gyro(self): return self._imu.gyro
    @property
    def temperature_c(self): return self._imu.temperature
