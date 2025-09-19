from BSW.drivers.ssd1306 import ssd1306
from BSW.utils import draw_lines, draw_pbm
from time import sleep_ms
from machine import Pin, SPI
import math

# ===== Config pantalla =====
W, H = 128, 64
COL_OFF = 4     
GRID_Y  = 10     

# ===== Helpers de dibujo =====
def _t(oled, s, x, y):
    oled.text(str(s), x + COL_OFF, y)

def _sep(oled, y=GRID_Y):
    for x in range(W):
        oled.pixel(x + COL_OFF, y, 1)

def _bar(oled, title, right_txt=None):
    oled.fill(0)
    _t(oled, title, 0, 0)
    if right_txt:
        s = str(right_txt)
        x = W - 2 - COL_OFF - len(s) * 8
        if x < 0: x = 0
        _t(oled, s, x, 0)
    _sep(oled)

def _col(oled, y, l1, v1, l2=None, v2=None, x1=0, xv1=34, x2=72, xv2=98):
    _t(oled, l1, x1, y)
    v1 = str(v1)
    x_val = xv1
    max_px = W - 2 - COL_OFF
    end = x_val + len(v1) * 8
    if end > max_px:
        x_val = max(x1, max_px - len(v1) * 8)
    _t(oled, v1, x_val, y)

    if l2 is not None:
        _t(oled, l2, x2, y)
        v2 = str(v2)
        x_val2 = xv2
        end2 = x_val2 + len(v2) * 8
        if end2 > max_px:
            x_val2 = max(x2, max_px - len(v2) * 8)
        _t(oled, v2, x_val2, y)

def _utc_compact(ts):
    if not isinstance(ts, str) or not ts:
        return None
    s = ts
    if "T" in s:
        s = s.split("T", 1)[1]
    if s.endswith("Z"):
        s = s[:-1]
        return s[:5] + "Z" if len(s) >= 5 else s + "Z"
    return s[:5] if len(s) >= 5 else s

def _lon_dec_for_2m(lat_deg):
    m_per_deg = 111320.0 * math.cos(math.radians(abs(lat_deg)))
    n = int(math.ceil(math.log10(max(m_per_deg / 2.0, 1e-9))))
    return min(6, max(4, n))

# =========================================================
class UI:
    """User Interface with Oled screen"""

    def __init__(self, i2c, w=128, h=64, addr=0x3C):
        # Forzar SPI
        spi = SPI(1, baudrate=10_000_000, polarity=0, phase=0, sck=Pin(36), mosi=Pin(35))
        dc  = Pin(38, Pin.OUT)
        res = Pin(39, Pin.OUT)
        cs  = Pin(37, Pin.OUT)
        try:
            res.value(0); sleep_ms(30); res.value(1); sleep_ms(30)
        except:
            pass
        self._oled = ssd1306.SSD1306_SPI(w, h, spi, dc, res, cs)

    # ---------------- ENV ----------------
    def show_bmp280(self, env):
        o = self._oled
        _bar(o, "ENV")
        try:
            t   = float(env.temperature_c)
            p   = float(env.pressure_hpa)
            alt = float(env.altitude_m)
            _col(o, 16, "T",  f"{t:5.1f} C")
            _col(o, 30, "P",  f"{p:7.1f} hPa")
            _col(o, 44, "Alt", f"{alt:7.1f} m")
        except Exception as exc:
            _t(o, "Sensor error:", 0, 22)
            _t(o, str(exc)[:21],  0, 38)
        o.show()

    # ---------------- IMU ----------------
    def show_imu(self, imu):
        o = self._oled
        _bar(o, "IMU")
        try:
            ax, ay, az = imu.accel
            gx, gy, gz = imu.gyro
            tc = getattr(imu, "temperature_c", None)

            _col(o, 16, "Ax", f"{ax:+.3f}")             
            _col(o, 30, "Ay", f"{ay:+.3f}")
            _col(o, 44, "Az", f"{az:+.3f}", "Gz", f"{gz:+.0f}", x2=88, xv2=108)
            if tc is not None and not math.isnan(tc):
                _col(o, 58, "T", f"{tc:5.1f} C")
        except Exception as exc:
            _t(o, "Sensor error:", 0, 22)
            _t(o, str(exc)[:21],  0, 38)
        o.show()


    # ---------------- GNSS ----------------
    def show_gnss1(self, fix):
        o = self._oled
        o.fill(0)

        utc = _utc_compact(fix.time_utc) if fix else "--:--"
        _bar(o, "Nav Bas", utc)

        if not fix:
            _t(o, "No fix", 0, 28)
            o.show()
            return

        nlat = 5
        nlon = _lon_dec_for_2m(fix.lat)

        _col(o, 16, "v",   f"{fix.speed_kph:4.1f}kph")
        _col(o, 30, "Lat", f"{fix.lat:+.{nlat}f}")
        _col(o, 44, "Lon", f"{fix.lon:+.{nlon}f}")
        o.show()

    def show_gnss2(self, fix):
        o = self._oled
        _bar(o, "Alt/Pres")

        if not fix:
            _t(o, "No fix", 0, 28)
            o.show(); return

        _col(o, 16, "MSL", f"{fix.alt_msl:.0f}m")
        _col(o, 30, "h",   f"{fix.alt_ellipsoid:.0f}m")
        _col(o, 44, "E2S", f"{fix.err_2sigma_m:.1f}m")
        _col(o, 58, "Geo", f"{fix.geoid_sep_m:.0f}m")
        o.show()


    def show_gnss3(self, fix):
        o = self._oled
        _bar(o, "Course/Mov")

        if not fix:
            _t(o, "No fix", 0, 28)
            o.show(); return

        _col(o, 16, "COG", f"{int(fix.cog_deg):03d}deg")
        _col(o, 30, "Var", f"{fix.mag_var_deg:.1f}deg")
        _col(o, 44, "MOV", "YES" if fix.has_motion else "NO")
        o.show()



    def show_gnss(self, fix):
        self.show_gnss1(fix)
        
    
        # ---------------- SYS: 2 páginas ----------------
    def show_sys1(self, sys_sensor):

        try:
            snap = sys_sensor.read()
        except Exception as exc:
            draw_lines(self._oled, ["SYS1", "Error:", str(exc)[:21]])
            return

        o = self._oled
        o.fill(0)

        o.text("SYS1", 2, 0)
        o.hline(0, 10, 128, 1)

        cpu = f"{snap.cpu_mhz}MHz" if snap.cpu_mhz else "-"
        o.text(f"CPU  {cpu}", 2, 18)

        temp = f"{snap.temp_c:.1f}C" if snap.temp_c else "-"
        o.text(f"Temp {temp}", 2, 30)

        mem = snap.heap_k
        o.text(f"Mem  {mem}", 2, 42)

        o.show()



    def show_sys2(self, sys_sensor):

        try:
            snap = sys_sensor.read()
        except Exception as exc:
            draw_lines(self._oled, ["SYS2", "Error:", str(exc)[:21], ""])
            return

        _bar(self._oled, "SYS2")
        rssi = f"{snap.rssi_dbm}dBm" if snap.rssi_dbm else "-"
        up   = snap.uptime_hhmm
        rc   = snap.reset_cause_str or "-"

        _col(self._oled, 16, "RSSI", rssi)
        _col(self._oled, 30, "UP", up)
        _col(self._oled, 44, "RC", rc)
        self._oled.show()


    # -------------- Error y Splash --------------
    def show_error(self, exc):
        _bar(self._oled, "ERR")
        _t(self._oled, "Sensor error:", 0, 22)
        _t(self._oled, str(exc)[:21],  0, 38)
        self._oled.show()

    def show_splash(self, path="/lib/App/assets/udegspace.pbm", duration_ms=2000, invert=True):
        try:
            self._oled.fill(0)
            draw_pbm(self._oled, path, center=True, invert=invert)
            sleep_ms(duration_ms)
        except Exception as exc:
            draw_lines(self._oled, ["Booting...", str(exc)[:21]])
            sleep_ms(1000)
