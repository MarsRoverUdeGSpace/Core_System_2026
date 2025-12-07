# Path: lib/BSW/drivers/neo_m8n/gnssensor.py
# Thin, PEP8-friendly wrapper over GNSSReceive.getdata()

from BSW.drivers.neo_m8n.neo_m8n import GNSSReceive

_KNOTS_TO_MPS = 0.514444
_MPS_TO_KPH = 3.6


class GNSSFix:
    """
    Container for one GNSS fix.

    Fields map 1:1 to GNSSReceive.getdata(), plus convenience properties.
    """
    __slots__ = (
        "lat", "lon", "alt_msl", "err_2sigma_m",
        "sog_kn", "cog_deg", "mag_var_deg",
        "geoid_sep_m", "time_utc"
    )

    def __init__(self, lat, lon, alt_msl, err_2sigma_m,
                 sog_kn, cog_deg, mag_var_deg, geoid_sep_m, time_utc):
        self.lat = lat
        self.lon = lon
        self.alt_msl = alt_msl
        self.err_2sigma_m = err_2sigma_m
        self.sog_kn = sog_kn
        self.cog_deg = cog_deg
        self.mag_var_deg = mag_var_deg
        self.geoid_sep_m = geoid_sep_m
        self.time_utc = time_utc  # ISO8601 'YYYY-MM-DDTHH:MM:SSZ' or 'HH:MM:SS' or 0

    # --- conveniences ---
    @property
    def speed_mps(self):
        return self.sog_kn * _KNOTS_TO_MPS

    @property
    def speed_kph(self):
        return self.speed_mps * _MPS_TO_KPH

    @property
    def alt_ellipsoid(self):
        """h = H + N (MSL + geoid separation)."""
        return self.alt_msl + self.geoid_sep_m

    @property
    def has_motion(self):
        """Coarse motion flag (~0.3 kn threshold like the driver)."""
        return self.sog_kn >= 0.3

    # Friendly aliases
    @property
    def err_2sigma(self):
        return self.err_2sigma_m

    @property
    def mag_var(self):
        return self.mag_var_deg

    def __repr__(self):
        return (
            "GNSSFix(lat={:+.6f}, lon={:+.6f}, H={:.1f}m, h={:.1f}m, "
            "±2σ={:.1f}m, v={:.2f}kn/{:.1f}kph, COG={:.0f}°, t={})"
        ).format(
            self.lat, self.lon,
            self.alt_msl, self.alt_ellipsoid,
            self.err_2sigma_m, self.sog_kn, self.speed_kph,
            self.cog_deg, self.time_utc
        )


class GNSSSensor:
    """
    Simple interface mirroring your EnvSensor style.

    Example:
        gnss = GNSSSensor(rx_pin=1, tx_pin=2, uart_id=2)
        fix = gnss.read()
        if fix:
            print(fix.lat, fix.lon, fix.speed_mps)
    """

    def __init__(self, rx_pin, tx_pin, uart_id=2, baud=9600):
        self._drv = GNSSReceive(rx_pin, tx_pin, uart_id=uart_id, baud=baud)
        self._last = None

    def read(self):
        """
        Returns GNSSFix or None (when no valid fix yet).
        Mirrors GNSSReceive.getdata() and does no re-parsing.
        """
        lat, lon, alt, terr2s, sog_kn, cog_deg, mag_var, geo, ts = self._drv.getdata()

        # "No fix" heuristic = 0/0 coordinates
        if (lat == 0.0 and lon == 0.0):
            return None

        fix = GNSSFix(
            lat=lat,
            lon=lon,
            alt_msl=alt,
            err_2sigma_m=terr2s,
            sog_kn=sog_kn,
            cog_deg=cog_deg,
            mag_var_deg=mag_var,
            geoid_sep_m=geo,
            time_utc=ts,
        )
        self._last = fix
        return fix

    @property
    def last(self):
        """Return last valid fix (or None)."""
        return self._last

    # Optional pass-throughs if you find them handy:
    def is_stationary(self):
        return self._drv.is_stationary()

    def setrate(self, rate_hz, meas_per_nav):
        return self._drv.setrate(rate_hz, meas_per_nav)

    def modulesetup(self):
        return self._drv.modulesetup()
