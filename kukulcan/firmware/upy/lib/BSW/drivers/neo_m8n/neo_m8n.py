from machine import UART
import utime as time
import struct

class GNSSReceive:
    def __init__(self, rx_pin, tx_pin, uart_id=2, baud=9600):
        """
        Driver for u-blox NEO-M8 family (NMEA + a few UBX cfg cmds).

        Args:
            rx_pin: machine.Pin for ESP32 RX (gnss TX -> this pin)
            tx_pin: machine.Pin for ESP32 TX (gnss RX <- this pin)
            uart_id: UART port id (default 2 on ESP32)
            baud: UART baudrate (default 9600)
        """
        self.gnss = UART(uart_id, baudrate=baud, tx=tx_pin, rx=rx_pin)
        self.data = {}
        self.background_buffer = bytearray()
        # simple state for moving-average speed
        self._ma_sog = 0.0

    # ---------- helpers ----------
    @staticmethod
    def _fmt_time(hhmmss):
        """Return 'HH:MM:SS' from 'hhmmss[.sss]'; 0 if not available."""
        if not hhmmss:
            return 0
        s = hhmmss.split(".")[0]
        if len(s) < 6:
            return 0
        return s[:2] + ":" + s[2:4] + ":" + s[4:6]

    @staticmethod
    def _parse_latlon(lat_str, ns, lon_str, ew):
        # lat: ddmm.mmmm, lon: dddmm.mmmm
        if not lat_str or not lon_str:
            return None, None
        try:
            pos = lat_str.find(".") - 2
            lat_deg = int(lat_str[:pos]); lat_min = float(lat_str[pos:])
            lat = lat_deg + lat_min / 60.0
            if ns == "S":
                lat = -lat

            pos = lon_str.find(".") - 2
            lon_deg = int(lon_str[:pos]); lon_min = float(lon_str[pos:])
            lon = lon_deg + lon_min / 60.0
            if ew == "W":
                lon = -lon
            return lat, lon
        except:
            return None, None

    def _checksum(self, nmea_sentence):
        # normalize CRLF -> LF for stable checksum slice
        if nmea_sentence.endswith(b"\r\n"):
            nmea_sentence = nmea_sentence[:-2] + b"\n"
        checksum_pos = nmea_sentence.find(b'*')
        if checksum_pos < 0:
            return False
        checksum = 0
        for i in nmea_sentence[1:checksum_pos]:
            checksum ^= i
        checksum = ("%02X" % checksum).encode('utf-8')
        return nmea_sentence[checksum_pos+1:checksum_pos+3] == checksum

    def _ubx_checksum(self, ubx_packet):
        ck_a = ck_b = 0
        for byte in ubx_packet:
            ck_a = (ck_a + byte) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF
        return struct.pack("<B", ck_a), struct.pack("<B", ck_b)

    # ---------- buffered ingest ----------
    def update_buffer(self):
        if self.gnss.any():
            chunk = self.gnss.read()
            if chunk:
                self.background_buffer += chunk
                if len(self.background_buffer) > 512:
                    self.background_buffer = self.background_buffer[-512:]

    def _update_data(self):
        sentences_read = 0
        # pump quickly; do not block forever
        while sentences_read < 5:
            self.update_buffer()
            start_pos = self.background_buffer.find(b'$')
            if start_pos == -1:
                break
            end_pos = self.background_buffer.find(b'\n', start_pos)
            if end_pos == -1:
                break
            data_section = self.background_buffer[start_pos:end_pos+1]
            self.background_buffer = self.background_buffer[end_pos+1:]
            if self._checksum(data_section):
                s = data_section.decode('utf-8', 'ignore')
                sentence_header = s[3:6]
                self.data[sentence_header] = s
                sentences_read += 1

    # ---------- public API ----------
    def position(self):
        """
        Returns (lat, lon, horizontal_error_m, timestamp_str)
        - lat, lon in decimal degrees (N/E positive, S/W negative)
        - error from HDOP * 2.5 (1σ)
        - timestamp "hh:mm:ss" or 0 if unavailable
        """
        try:
            self._update_data()
        except UnicodeError:
            self._update_data()

        lat = lon = 0
        time_stamp = 0
        position_error = 0.0

        # Prefer RMC
        rmc = self.data.get("RMC")
        if rmc:
            f = rmc.split(",")
            time_stamp = self._fmt_time(f[1])
            if len(f) > 6 and f[2] == "A":
                lt, ln = self._parse_latlon(f[3], f[4], f[5], f[6])
                if lt is not None:
                    lat, lon = lt, ln

        # Fall back to GGA
        if (not lat and not lon) and self.data.get("GGA"):
            gga = self.data["GGA"].split(",")
            time_stamp = time_stamp or self._fmt_time(gga[1])
            if len(gga) > 6 and gga[6] != "0":  # fix quality nonzero
                lt, ln = self._parse_latlon(gga[2], gga[3], gga[4], gga[5])
                if lt is not None:
                    lat, lon = lt, ln

        # Last resort, GLL
        if (not lat and not lon) and self.data.get("GLL"):
            gll = self.data["GLL"].split(",")
            time_stamp = time_stamp or self._fmt_time(gll[5])
            if len(gll) > 6 and gll[6] == "A":
                lt, ln = self._parse_latlon(gll[1], gll[2], gll[3], gll[4])
                if lt is not None:
                    lat, lon = lt, ln

        # HDOP from GSA
        gsa = self.data.get("GSA")
        if gsa:
            try:
                # ... , PDOP, HDOP, VDOP*CS
                parts = gsa.split(",")
                hdop_str = parts[-2]
                position_error = float(hdop_str) * 2.5  # HDOP * UERE≈2.5 m (1σ)
            except:
                pass

        return lat or 0, lon or 0, position_error, time_stamp

    def velocity(self, data_needs_updating=True):
        """
        Returns (sog_kn, cog_deg, mag_var_deg, timestamp_str) as floats + string time
        """
        if data_needs_updating:
            try:
                self._update_data()
            except UnicodeError:
                self._update_data()

        rmc = self.data.get("RMC")
        if not rmc:
            return 0.0, 0.0, 0.0, 0
        f = rmc.split(",")
        ts = self._fmt_time(f[1]) if len(f) > 1 else 0
        if len(f) < 12 or f[2] != "A":
            return 0.0, 0.0, 0.0, ts

        try:
            sog = float(f[7]) if f[7] else 0.0  # knots
        except:
            sog = 0.0
        try:
            cog = float(f[8]) if f[8] else 0.0  # degrees
        except:
            cog = 0.0
        try:
            mag = float(f[10]) if f[10] else 0.0
            if f[11] == "W":
                mag = -mag
        except:
            mag = 0.0
        return sog, cog, mag, ts

    def altitude(self, data_needs_updating=True):
        """
        Returns (alt_m, geoid_sep_m, vertical_error_m, timestamp_str)
        - vertical_error ≈ VDOP * 5 (1σ)
        """
        if data_needs_updating:
            try:
                self._update_data()
            except UnicodeError:
                self._update_data()

        gga = self.data.get("GGA")
        if not gga:
            return 0.0, 0.0, 0.0, 0
        g = gga.split(",")
        ts = self._fmt_time(g[1]) if len(g) > 1 else 0
        gsa = self.data.get("GSA")

        if len(g) > 12 and g[6] != "0":  # valid fix
            try:
                alt = float(g[9]) if g[9] else 0.0
            except:
                alt = 0.0
            try:
                geo = float(g[11]) if g[11] else 0.0
            except:
                geo = 0.0

            verr = 0.0
            if gsa:
                try:
                    parts = gsa.split(",")
                    vdop_field = parts[-1]
                    vdop = float(gsa.split("*")[0])
                    verr = vdop * 5.0
                except:
                    pass
            return alt, geo, verr, ts
        return 0.0, 0.0, 0.0, ts

    # ---------- extra utilities ----------
    def _iso8601(self):
        """Return UTC ISO-8601 (YYYY-MM-DDTHH:MM:SSZ) using RMC date+time if present, else 0."""
        rmc = self.data.get("RMC")
        if not rmc:
            return 0
        f = rmc.split(",")
        t = self._fmt_time(f[1]) if len(f) > 1 else 0
        if not t or len(f) < 10:
            return 0
        d = f[9]  # ddmmyy
        if len(d) != 6:
            return 0
        dd, mm, yy = d[:2], d[2:4], d[4:6]
        # assume 2000-2079 for yy<80, else 1900s
        yyyy = "20" + yy if int(yy) < 80 else "19" + yy
        return f"{yyyy}-{mm}-{dd}T{t}Z"

    def speed_ms(self):
        """Speed over ground in m/s (converted from knots)."""
        sog_kn, _, _, _ = self.velocity(data_needs_updating=False)
        return sog_kn * 0.514444

    def heading_deg(self, min_kn=0.5):
        """Heading (COG) in degrees; returns 0 if speed below threshold (default 0.5 kn)."""
        sog_kn, cog_deg, _, _ = self.velocity(data_needs_updating=False)
        return cog_deg if sog_kn >= min_kn else 0.0

    def altitude_ellipsoid(self):
        """Ellipsoidal height h = H + N (MSL + geoid separation)."""
        alt_msl, geoid, _, _ = self.altitude(data_needs_updating=False)
        return alt_msl + geoid

    def stable_sog(self, alpha=0.2):
        """Simple EMA smoothing of SOG in knots."""
        sog, _, _, _ = self.velocity(False)
        self._ma_sog = alpha * sog + (1 - alpha) * self._ma_sog
        return self._ma_sog

    def is_stationary(self, kn_thresh=0.3):
        """True if smoothed SOG below threshold (knots)."""
        return self.stable_sog() < kn_thresh

    def getdata(self):
        """
        Returns:
          (lat, lon, alt_msl, total_error_2sigma_m, sog_kn, cog_deg, mag_var_deg, geoid_sep_m, timestamp_iso_or_time)

        - timestamp prefers ISO-8601 (from RMC date+time), falls back to "hh:mm:ss" or 0.
        """
        lat, lon, pos_err, t0 = self.position()
        sog, cog, mag, t1 = self.velocity(data_needs_updating=False)
        alt, geo, verr, t2 = self.altitude(data_needs_updating=False)

        # best available time
        timestamp = self._iso8601() or t0 or t1 or t2 or 0

        # combine 1σ horizontal + vertical into 3D, then scale ≈ 2σ
        total_error = 2.45 * ((pos_err * pos_err + verr * verr) ** 0.5)
        
        lat = round(lat, 8) if lat else 0.0
        lon = round(lon, 6) if lon else 0.0

        return lat, lon, alt, total_error, sog, cog, mag, geo, timestamp

    # ---------- UBX control ----------
    def _ubx_ack_nack(self, timeout_ms=1000):
        start = time.ticks_ms()
        ackdata = b''
        while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
            if self.gnss.any():
                chunk = self.gnss.read()
                if chunk:
                    ackdata += chunk
                    # UBX-ACK-ACK (0xB5 0x62 05 01) or NAK (05 00)
                    if b'\xb5\x62\x05\x01' in ackdata:
                        return True
                    if b'\xb5\x62\x05\x00' in ackdata:
                        return False
        return False  # timeout

    def gnss_stop(self):
        # UBX-CFG-RST: controlled GNSS stop
        packet = b'\x06\x04' + b'\x04\x00' + b'\x00\x00' + b'\x08' + b'\x00'
        ck_a, ck_b = self._ubx_checksum(packet)
        self.gnss.write(b'\xb5\x62' + packet + ck_a + ck_b)

    def gnss_start(self):
        # UBX-CFG-RST: controlled GNSS start
        packet = b'\x06\x04' + b'\x04\x00' + b'\x00\x00' + b'\x09' + b'\x00'
        ck_a, ck_b = self._ubx_checksum(packet)
        self.gnss.write(b'\xb5\x62' + packet + ck_a + ck_b)

    def setrate(self, rate, measurements_per_nav_solution):
        """
        Set nav solution output rate (Hz) and measurements per solution.
        Returns True on ACK, False on NAK/timeout.
        """
        measurement_time_delta_ms = int(1000 / rate)
        payload = (
            b'\x06\x08\x06\x00' +
            struct.pack("<H", measurement_time_delta_ms) +
            struct.pack("<H", measurements_per_nav_solution) +
            b'\x00\x00'
        )
        ck_a, ck_b = self._ubx_checksum(payload)
        packet = b'\xb5\x62' + payload + ck_a + ck_b
        self.gnss.write(packet)
        return self._ubx_ack_nack()

    def modulesetup(self):
        """
        One-time configuration & save.
        """
        # 1) Disable VTG (redundant)
        self.gnss.write(b'\xb5\x62\x06\x01\x03\x00\xF0\x05\x00\xff\x19')
        if not self._ubx_ack_nack():
            return False

        # 2) NAV5 (airborne <4g, 3D only, mask/thresholds)
        payload = (
            b'\x06\x24' + b'$\x00' + b'G\x08' + b'\x08' + b'\x02' +
            b'\x00\x00\x00\x00' + b'\x00\x00\x00\x00' + b'\x14' + b'\x00' +
            b'\x00\x00' + b'\x00\x00' + b'\x00\x00' + b'\x00\x00' +
            b'\x14' + b'\x00' + b'\x00' + b'\x00' + b'\x00' + b'\x01' + b'\x00' +
            b'\x00\x00\x00\x00\x00\x00\x00'
        )
        ck_a, ck_b = self._ubx_checksum(payload)
        self.gnss.write(b'\xb5\x62' + payload + ck_a + ck_b)
        if not self._ubx_ack_nack():
            return False

        # 3) NAVX5 (limits, AssistNow Autonomous, etc.)
        payload = (
            b'\x06\x23' + b'(\x00' + b'\x00\x00' + b'D@' + b'\x00\x00\x00\x00' +
            b'\x00\x00' + b'\x04' + b'<' + b'\x00' + b'\x00' + b'\x01' + b'\x00' +
            b'\x00' + b'\x00\x00' + b'\x00\x00\x00\x00\x00\x00' + b'\x00' + b'\x01' +
            b'\x00\x00' + b'\x14\x00' + b'\x00\x00\x00\x00' + b'\x00\x00\x00\x00' + b'\x00'
        )
        ck_a, ck_b = self._ubx_checksum(payload)
        self.gnss.write(b'\xb5\x62' + payload + ck_a + ck_b)
        if not self._ubx_ack_nack():
            return False

        # 4) GNSS selection (gps, SBAS, Galileo, BeiDou, GLONASS)
        payload = (
            b'\x06\x3e' + b'\x2c\x00' + b'\x00\x00\xff\x05' +
            b'\x00\x08\x10\x00\x00\x01\x00\x01' +  # gps
            b'\x01\x01\x03\x00\x00\x01\x00\x01' +  # SBAS
            b'\x02\x02\x08\x00\x00\x01\x00\x01' +  # Galileo
            b'\x03\x08\x0e\x00\x00\x01\x00\x01' +  # BeiDou
            b'\x06\x06\x0e\x00\x00\x01\x00\x01'    # GLONASS
        )
        ck_a, ck_b = self._ubx_checksum(payload)
        self.gnss.write(b'\xb5\x62' + payload + ck_a + ck_b)
        if not self._ubx_ack_nack():
            return False

        # 5) Interference/jamming monitor
        payload = b'\x06\x39' + b'\x08\x00' + b'\xadb\xadG' + b'\x00\x00#\x1e'
        ck_a, ck_b = self._ubx_checksum(payload)
        self.gnss.write(b'\xb5\x62' + payload + ck_a + ck_b)
        if not self._ubx_ack_nack():
            return False

        # 6) Save to flash (for M8N/J). For M8Q/M8M use BBR instead (change last byte to \x01).
        payload = b'\x06\x09' + b'\x0d\x00' + b'\x00\x00\x00\x00' + b'\x00\x00\x00\x1a' + b'\x00\x00\x00\x00' + b'\x02'
        ck_a, ck_b = self._ubx_checksum(payload)
        self.gnss.write(b'\xb5\x62' + payload + ck_a + ck_b)
        if not self._ubx_ack_nack():
            return False

        # 7) Soft reset to apply
        payload = b'\x06\x04' + b'\x04\x00' + b'\xff\xff' + b'\x00' + b'\x00'
        ck_a, ck_b = self._ubx_checksum(payload)
        self.gnss.write(b'\xb5\x62' + payload + ck_a + ck_b)
        return True

if __name__ == "__main__":
    # Example pins; adjust for your board
    gnss = GNSSReceive(rx_pin=1, tx_pin=2)

    flag = gnss.modulesetup()
    while not flag:
        flag = gnss.modulesetup()

    flag = gnss.setrate(2, 3)
    while not flag:
        flag = gnss.setrate(2, 3)

    while True:
        lat, lon, alt, terr, sog, cog, mag, geo, ts = gnss.getdata()
        # ISO if available, else hh:mm:ss
        print(ts, f"lat={lat:.7f}", f"lon={lon:.7f}", f"H={alt:.1f}m",
              f"h={alt+geo:.1f}m", f"2σ≈{terr:.1f}m",
              f"SOG={sog:.3f}kn", f"COG={cog:.1f}°",
              "stationary=" + str(gnss.is_stationary()))
        time.sleep_ms(250)
