from time import ticks_ms, ticks_diff
import gc

try:
    from machine import freq, reset_cause, PWRON_RESET, HARD_RESET, WDT_RESET, DEEPSLEEP_RESET, SOFT_RESET
except Exception:
    freq = None
    def reset_cause(): return None
    PWRON_RESET = HARD_RESET = WDT_RESET = DEEPSLEEP_RESET = SOFT_RESET = object()

try:
    import esp32
except Exception:
    esp32 = None

try:
    import network
except Exception:
    network = None
    
try:
    from esp32 import mcu_temperature as _mcu_temp
except Exception:
    _mcu_temp = None

try:
    from esp32 import raw_temperature as _raw_temp  # por compatibilidad con algunos builds
except Exception:
    _raw_temp = None



class SysSnapshot:
    __slots__ = ("cpu_mhz", "temp_c", "heap_free", "heap_total",
                 "rssi_dbm", "uptime_s", "reset_cause_str")

    def __init__(self, cpu_mhz, temp_c, heap_free, heap_total,
                 rssi_dbm, uptime_s, reset_cause_str):
        self.cpu_mhz = cpu_mhz
        self.temp_c = temp_c
        self.heap_free = heap_free
        self.heap_total = heap_total
        self.rssi_dbm = rssi_dbm
        self.uptime_s = uptime_s
        self.reset_cause_str = reset_cause_str

    @property
    def heap_k(self):
        if self.heap_free and self.heap_total:
            return f"{self.heap_free//1024}K/{self.heap_total//1024}K"
        return "-/-"

    @property
    def uptime_hhmm(self):
        s = int(self.uptime_s or 0)
        m, _ = divmod(s, 60)
        h, m = divmod(m, 60)
        return "%d:%02d" % (h, m)


class SysInfoSensor:
    _t0 = ticks_ms()

    def _reset_cause_str(self):
        try:
            rc = reset_cause()
            m = {
                PWRON_RESET: "PWRON",
                HARD_RESET: "HARD",
                WDT_RESET: "WDT",
                DEEPSLEEP_RESET: "DSLP",
                SOFT_RESET: "SOFT",
            }
            return m.get(rc, str(rc))
        except Exception:
            return None

    def _cpu_mhz(self):
        try:
            return int(freq() // 1_000_000)
        except Exception:
            return None

    def _temp_c(self):

        try:
            if _mcu_temp:
                val = float(_mcu_temp())
                if -40.0 <= val <= 150.0:
                    return val
        except Exception:
            pass


    def _wifi_rssi(self):
        try:
            if not network:
                return None
            sta = network.WLAN(network.STA_IF)
            if sta.active() and sta.isconnected():
                return int(sta.status('rssi'))
        except Exception:
            pass
        return None

    def read(self):
        try:
            free = gc.mem_free()
            alloc = gc.mem_alloc()
            total = free + alloc
        except Exception:
            free = total = None

        uptime_s = ticks_diff(ticks_ms(), self._t0) // 1000

        return SysSnapshot(
            cpu_mhz=self._cpu_mhz(),
            temp_c=self._temp_c(),
            heap_free=free,
            heap_total=total,
            rssi_dbm=self._wifi_rssi(),
            uptime_s=uptime_s,
            reset_cause_str=self._reset_cause_str(),
        )
