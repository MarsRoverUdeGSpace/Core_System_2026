# /lib/BSW/drivers/bme680/envsensor.py
# Wrapper para unificar la API del BME680
# Soporta: BSW.drivers.bme680.bme680 o /lib/bme680.py (robert-hh)

def _get_driver():
    # 1) Driver dentro de BSW
    try:
        from .bme680 import BME680_I2C as _DRV
        return _DRV
    except Exception:
        try:
            from .bme680 import BME680 as _DRV
            return _DRV
        except Exception:
            pass
    # 2) Driver en /lib
    from bme680 import BME680_I2C as _DRV  # si no está, lanzará ImportError
    return _DRV

class EnvSensor:
    def __init__(self, i2c, addr=0x77):
        DRV = _get_driver()
        try:
            self.dev = DRV(i2c=i2c, address=addr)
        except Exception:
            self.dev = DRV(i2c=i2c, address=0x76)
        self.addr = addr

    @property
    def temperature_c(self):
        if hasattr(self.dev, "perform_reading"):
            self.dev.perform_reading()
        # distintos drivers usan nombres distintos
        return float(getattr(self.dev, "temperature", getattr(self.dev, "temp", 25.0)))

    @property
    def pressure_hpa(self):
        if hasattr(self.dev, "perform_reading"):
            self.dev.perform_reading()
        p = float(getattr(self.dev, "pressure", getattr(self.dev, "press", 1013.0)))
        return p/100.0 if p > 2000 else p  # conv Pa->hPa si aplica

    @property
    def altitude_m(self):
        p = self.pressure_hpa
        return 44330.0 * (1.0 - (p / 1013.25) ** 0.1903)
