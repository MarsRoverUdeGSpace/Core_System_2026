from BSW.drivers.ssd1306 import ssd1306
from BSW.utils import draw_lines, draw_pbm
from time import sleep_ms

class UI:
    """User Interface with Oled screen"""

    def __init__(self, i2c, w=128, h=64, addr=0x3C):
        self._oled = ssd1306.SSD1306_I2C(w, h, i2c, addr=addr)

    def show_bmp280(self, env):
        lines = [
            "BMP280",
            f"T:{env.temperature_c:5.1f} C",
            f"P:{env.pressure_hpa:7.1f} hPa",
            f"Alt:{env.altitude_m:7.1f} m",
        ]
        draw_lines(self._oled, lines)

    def show_imu(self, imu):
        ax, ay, az = imu.accel
        gx, gy, gz = imu.gyro
        t = imu.temperature_c
        lines = [
            "MPU6500",
            f"A:{ax:+.2f}\n {ay:+.2f}\n {az:+.2f} g",
            f"G:{gx:+.1f}\n {gy:+.1f}\n {gz:+.1f} deg/s",
            f"T:{t:5.1f} C",
        ]
        draw_lines(self._oled, lines)
        
    def show_gnss(self, fix):
        """Render a compact GNSS view for 128x64."""
        if not fix:
            draw_lines(self._oled, ["NEO-M8N", "No fix", "", ""])
            return

        # Compact time: show HH:MMZ for ISO timestamps; else HH:MM from "HH:MM:SS"
        t_txt = fix.time_utc if isinstance(fix.time_utc, str) else "--"
        t_end = t_txt[-8:] if len(t_txt) >= 8 else t_txt # "HH:MM:SS" tail

        lines = [
            "NEO-M8N",
            f"Lat:{fix.lat:+.6f}\nLon:{fix.lon:+.6f}",
            f"H:{fix.alt_msl:6.1f}m\nh:{fix.alt_ellipsoid:6.1f}m",
            f"N:{fix.geoid_sep_m:5.1f}m +/-:{fix.err_2sigma_m:.0f}m",
            f"v:{fix.speed_kph:4.1f} hdg:{int(fix.cog_deg):03d}",
            f"{t_end}",
        ]
        draw_lines(self._oled, lines)

    def show_error(self, exc):
        draw_lines(self._oled, ["Sensor error:"])
        draw_lines(self._oled, ["", str(exc)[:21]])
        
    
    def show_splash(self, path="/lib/App/assets/udegspace.pbm", duration_ms=2000, invert=True):
        try:
            self._oled.fill(0)
            draw_pbm(self._oled, path, center=True, invert=invert)
            sleep_ms(duration_ms)
        except Exception as exc:
            # Fallback if image missing/corrupt
            draw_lines(self._oled, ["Booting...", str(exc)[:21]])
            sleep_ms(1000)
    