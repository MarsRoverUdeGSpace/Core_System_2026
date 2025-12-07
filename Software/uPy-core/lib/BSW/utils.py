import utime as time
from machine import Pin
from math import pow

SEA_LEVEL_HPA = 1013.25  # adjust to your location

class Button:
    """
    Active-low button with pull-up and IRQ debounce.
    Default: pin=3, pull-up enabled, 10 ms debounce.
    """
    def __init__(self, pin_no=3, debounce_ms=10, active_low=True):
        self._pin = Pin(pin_no, Pin.IN, Pin.PULL_UP)
        self._active_low = active_low
        self._debounce = debounce_ms
        self._last_ms = time.ticks_ms()
        self._flag = False
        self._armed = True
        self._pin.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=self._irq)

    def _level_is_pressed(self):
        v = self._pin.value()
        return (v == 0) if self._active_low else (v == 1)

    def _irq(self, _):
        now = time.ticks_ms()
        if time.ticks_diff(now, self._last_ms) < self._debounce:
            return  # ignore bounces within debounce window
        self._last_ms = now

        if self._level_is_pressed():
            # edge to pressed; only emit if armed
            if self._armed:
                self._flag = True    # one-shot event
                self._armed = False  # disarm until release
        else:
            # edge to released; re-arm
            self._armed = True

    def pressed(self):
        """Return True exactly once per full press->release."""
        if self._flag:
            self._flag = False
            return True
        return False

    def is_pressed(self):
        """Raw (debounced by edge window) level read."""
        return self._level_is_pressed()

def pressure_pa_to_alt_m(pressure_pa, sea_level_hpa=SEA_LEVEL_HPA):
    """Barometric altitude from pressure (Pa). Returns meters."""
    pressure_hpa = pressure_pa / 100.0
    return 44330.0 * (1.0 - pow((pressure_hpa / sea_level_hpa), 1.0 / 5.255))

def draw_lines(oled, blocks, oled_h=64):
    """Render multi-line text blocks (with \n) on SSD1306."""
    oled.fill(0)
    y = 0
    for block in blocks:
        for line in block.split("\n"):
            oled.text(line[:21], 0, y)
            y += 8
    oled.show()

def _pbm_read_header(f):
    # Expect "P4" then width height, allow '#' comments
    def tokens():
        for line in f:
            line = line.strip()
            if not line or line.startswith(b"#"):
                continue
            for tok in line.split():
                yield tok
    it = tokens()
    magic = next(it)
    if magic != b"P4":
        raise ValueError("PBM must be P4 (binary). Got: %r" % magic)
    w = int(next(it)); h = int(next(it))
    return w, h

def draw_pbm(oled, path, x0=0, y0=0, invert=False, center=False):
    # Draw binary PBM (P4) onto an SSD1306-style oled
    with open(path, "rb") as f:
        w, h = _pbm_read_header(f)
        ox, oy = x0, y0
        if center:
            ox = (oled.width  - w) // 2
            oy = (oled.height - h) // 2
        row_bytes = (w + 7) // 8
        y = 0
        while y < h:
            row = f.read(row_bytes)
            if not row or len(row) < row_bytes:
                break
            x = 0
            for byte in row:
                for bit in range(8):
                    if x >= w:
                        break
                    on = (byte & (0x80 >> bit)) != 0
                    if invert:
                        on = not on
                    oled.pixel(ox + x, oy + y, 1 if on else 0)
                    x += 1
            y += 1
    oled.show()


