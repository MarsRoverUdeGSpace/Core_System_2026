from time import ticks_ms, ticks_diff, sleep_ms
from machine import I2C, Pin
from App.ui import UI
from BSW.drivers.bme680.envsensor import EnvSensor
from BSW.drivers.mpu6050.imusensor import ImuSensor
from BSW.drivers.neo_m8n.gnsssensor import GNSSSensor
from BSW.drivers.sysinfo.sysinfosensor import SysInfoSensor


# ==== Pines ====
I2C_ID   = 0
I2C_SCL  = 8
I2C_SDA  = 9
I2C_FREQ = 400_000

GNSS_UART_ID = 1
GNSS_RX      = 18    
GNSS_TX      = 17   
GNSS_BAUD    = 9600

BTN_PIN = 47        

class Button:
    def __init__(self, pin_num, active_low=True, debounce_ms=80):
        self.pin = Pin(pin_num, Pin.IN, Pin.PULL_UP if active_low else Pin.PULL_DOWN)
        self.active_low = active_low
        self.debounce = debounce_ms
        self._last_ts = ticks_ms()
        self._last_raw = self._read_raw()
        self._armed = True

    def _read_raw(self):
        v = self.pin.value()
        return (v == 0) if self.active_low else (v == 1)

    def pressed(self):
        now = ticks_ms()
        if ticks_diff(now, self._last_ts) < self.debounce:
            return False
        raw = self._read_raw()
        if raw != self._last_raw:
            self._last_raw = raw
            self._last_ts = now
            if raw and self._armed:
                self._armed = False
                return True
            if not raw:
                self._armed = True
        return False

class App:
    """Main application controller"""

    def __init__(self,
                 scl=I2C_SCL, sda=I2C_SDA, i2c_id=I2C_ID,
                 gnss_uart_id=GNSS_UART_ID, gnss_rx=GNSS_RX, gnss_tx=GNSS_TX, gnss_baud=GNSS_BAUD):
        # I2C de sensores
        self._i2c = I2C(i2c_id, scl=Pin(scl), sda=Pin(sda), freq=I2C_FREQ)

        # UI 
        self._ui = UI(self._i2c)
        self._ui.show_splash("/lib/App/assets/marsroverteam.pbm", duration_ms=1500)

        # Sensores
        try:
            self._env = EnvSensor(self._i2c)
        except Exception:
            self._env = None
        try:
            self._imu = ImuSensor(self._i2c)
        except Exception:
            self._imu = None

        # GNSS 
        self._gnss = GNSSSensor(rx_pin=gnss_rx, tx_pin=gnss_tx,
                                uart_id=gnss_uart_id, baud=gnss_baud)

        # Botón
        self._btn = Button(BTN_PIN, active_low=True, debounce_ms=80)
        
        #esp32 sys
        self._sys = SysInfoSensor()

        # Páginas (ENV → IMU → GNSS → SYS)
        self._pages = ("ENV", "IMU", "GN1", "GN2", "GN3", "SYS1", "SYS2")
        self._page = 0

        self._show_page()

    def _show_page(self):
        p = self._pages[self._page]  
        if p == "ENV":
            self._ui.show_bmp280(self._env) if self._env else self._ui.show_error("No ENV")
        elif p == "IMU":
            self._ui.show_imu(self._imu) if self._imu else self._ui.show_error("No IMU")
        elif p == "GN1":
            fix = self._gnss.read()
            self._ui.show_gnss1(fix)
        elif p == "GN2":
            fix = self._gnss.read()
            self._ui.show_gnss2(fix)
        elif p == "GN3":
            fix = self._gnss.read()
            self._ui.show_gnss3(fix)
        elif p == "SYS1":
            self._ui.show_sys1(self._sys)
        elif p == "SYS2":
            self._ui.show_sys2(self._sys)



    def run(self, refresh_ms=300):
        """Refresca la vista y permite cambiar con el botón."""
        last = ticks_ms()
        while True:
            try:
                if self._btn.pressed():
                    self._page = (self._page + 1) % len(self._pages)
                    self._show_page()
                    sleep_ms(30)

                now = ticks_ms()
                if ticks_diff(now, last) >= refresh_ms:
                    self._show_page()
                    last = now

                sleep_ms(10)
            except Exception as exc:
                self._ui.show_error(exc)
                sleep_ms(500)

if __name__ == "__main__":
    app = App()
    app.run(refresh_ms=300)
