from BSW.drivers.bmp280.bmp280 import BMP280
from BSW.utils import pressure_pa_to_alt_m

class EnvSensor:
    """BMP280 convenience wrapper."""

    def __init__(self, i2c, addr=0x76, use_case=5):
        self._bmp = BMP280(i2c, addr=addr, use_case=use_case)
        self._bmp.iir = 4
        self._bmp.standby = 1
        self._bmp.press_os = 5
        self._bmp.temp_os = 2
        self._bmp.normal_measure()

    @property
    def temperature_c(self):
        return self._bmp.temperature

    @property
    def pressure_hpa(self):
        return self._bmp.pressure / 100.0

    @property
    def altitude_m(self):
        return pressure_pa_to_alt_m(self._bmp.pressure)
