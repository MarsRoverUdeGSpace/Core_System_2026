from time import sleep_ms
from machine import I2C, Pin
from App.ui import UI
from BSW.drivers.bmp280.envsensor import EnvSensor
from BSW.drivers.mpu6500.imusensor import ImuSensor
from BSW.drivers.neo_m8n.gnsssensor import GNSSSensor
from BSW.utils import Button

class App:
    """Main application controller."""

    def __init__(self, scl=4, sda=5, i2c_id=0):
        self._i2c = I2C(i2c_id, scl=Pin(scl), sda=Pin(sda), freq=400_000)
        self._ui = UI(self._i2c)

        self._ui.show_splash("/lib/App/assets/udegspace.pbm", duration_ms=1500)
        self._ui.show_splash("lib/App/assets/marsroverteam.pbm", duration_ms=1500)

        self._env = EnvSensor(self._i2c)
        self._imu = ImuSensor(self._i2c)
        self._gnss = GNSSSensor(1,2)

        self._page = 0  # 0: BMP, 1: IMU, 2: GNSS

        # Button on GPIO3: pull-up, active-low, 10 ms debounce
        self._btn = Button(pin_no=3, debounce_ms=10)

    def run(self, interval_ms=100):  # <<< 100 ms real-time updates
        while True:
            try:
                # page change only on button press
                if self._btn.pressed():
                    self._page = (self._page + 1) % 3

                if self._page == 0:
                    self._ui.show_bmp280(self._env)
                elif self._page == 1:
                    self._ui.show_imu(self._imu)
                else:
                    self._ui.show_gnss(self._gnss.read())

                sleep_ms(interval_ms)  # refresh every 100 ms
            except Exception as exc:
                self._ui.show_error(exc)
                sleep_ms(500)


