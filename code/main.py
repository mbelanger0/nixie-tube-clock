"""
Nixie Tube Clock - MicroPython firmware for RP2350

Hardware (from schematic):
  RTC:  DS3231M on I2C0, GPIO16=SDA, GPIO17=SCL (3.3V pull-ups on board)
  Shift register chain: two HV5122 (U6 -> U7) driven via CD40109 level shifters
    GPIO10 -> SR_DATA_IN
    GPIO11 -> SR_CLK
    GPIO12 -> SR_STR   (strobe / latch)
    GPIO13 -> SR_OE    (output enable; HV5122 OE is ACTIVE HIGH)
  Six nixie tubes N1..N6:
    N1..N3 driven by U6 outputs HVOUT1..HVOUT30
    N4..N6 driven by U7 outputs HVOUT1..HVOUT30
  Within each 10-output block the cathode ordering (from the schematic) is
    HVOUT+0 = digit 0, HVOUT+1 = 9, +2 = 8, +3 = 7, ..., +9 = digit 1.

Display assignment (left-to-right): N1 N2 : N3 N4 : N5 N6  = HH MM SS.

Data flow and bit order:
  Bit 0 of the 64-bit frame corresponds to U6.HVOUT1 (first stage the MCU
  clocks into). Bit 63 corresponds to U7.HVOUT32. When the frame is packed
  big-endian and shifted out MSB-first, the MSB of byte 0 travels all the
  way through and lands at U7.HVOUT32, and the LSB of byte 7 ends at
  U6.HVOUT1 - which matches the definition above.

Note on SPI: the RP2350's hardware SPI1 uses GPIO10 for SCK and GPIO11 for
TX, but the board wires them the other way (GPIO10=data, GPIO11=clock), so
this script bit-bangs the shift register. 64 bits once per second is trivial.
"""

import machine
import time

# ---------------------------------------------------------------------------
# Pin assignments
# ---------------------------------------------------------------------------
PIN_SR_DATA = 10
PIN_SR_CLK = 11
PIN_SR_STR = 12
PIN_SR_OE = 13

PIN_I2C_SDA = 16
PIN_I2C_SCL = 17


# ---------------------------------------------------------------------------
# Nixie driver (two cascaded HV5122s)
# ---------------------------------------------------------------------------
class NixieDriver:
    # Offset within each tube's 10-output block for each digit 0..9.
    # Derived from the schematic: HVOUT(base+0)=0, (base+1)=9, (base+2)=8,
    # (base+3)=7, (base+4)=6, (base+5)=5, (base+6)=4, (base+7)=3,
    # (base+8)=2, (base+9)=1.
    _POS = (0, 9, 8, 7, 6, 5, 4, 3, 2, 1)

    def __init__(self, data_pin, clk_pin, str_pin, oe_pin):
        self._data = machine.Pin(data_pin, machine.Pin.OUT, value=0)
        self._clk = machine.Pin(clk_pin, machine.Pin.OUT, value=0)
        self._str = machine.Pin(str_pin, machine.Pin.OUT, value=0)
        # Keep outputs disabled (OE low) until we've clocked in real data,
        # so nothing random lights up at power-on.
        self._oe = machine.Pin(oe_pin, machine.Pin.OUT, value=0)

    def _bit_index(self, tube, digit):
        """Map (tube 1..6, digit 0..9) -> bit index 0..63 in the frame."""
        if not 1 <= tube <= 6:
            raise ValueError("tube must be 1..6")
        if not 0 <= digit <= 9:
            raise ValueError("digit must be 0..9")
        on_u7 = tube >= 4  # U7 carries tubes 4..6
        tube_in_chip = (tube - 1) % 3  # 0, 1, or 2
        hvout_zero_idx = tube_in_chip * 10 + self._POS[digit]  # 0..29
        return (32 if on_u7 else 0) + hvout_zero_idx

    def _shift_out(self, buf):
        """Clock out `buf` MSB-first to the HV5122 chain."""
        data = self._data
        clk = self._clk
        for byte in buf:
            b = byte
            for _ in range(8):
                data((b >> 7) & 1)
                clk(1)
                clk(0)
                b = (b << 1) & 0xFF

    def show_digits(self, d1, d2, d3, d4, d5, d6):
        """Light one digit per tube. Pass None to leave a tube dark."""
        frame = 0
        for tube, digit in enumerate((d1, d2, d3, d4, d5, d6), start=1):
            if digit is None:
                continue
            frame |= 1 << self._bit_index(tube, digit)

        buf = frame.to_bytes(8, "big")

        # Clock 64 bits in with strobe low, then pulse strobe high to latch.
        self._str(0)
        self._shift_out(buf)
        self._str(1)
        self._str(0)
        # Enable the high-voltage outputs (active HIGH on HV5122).
        # If all tubes stay dark (or all digits glow at once) after the
        # first successful update, this polarity is the first thing to
        # suspect - try flipping it.
        self._oe(1)

    def blank(self):
        """Extinguish all tubes (leaves shift-register contents intact)."""
        self._oe(0)

    def clear(self):
        """Latch an all-zero frame, then enable outputs (truly blank)."""
        self._str(0)
        self._shift_out(b"\x00" * 8)
        self._str(1)
        self._str(0)
        self._oe(1)


# ---------------------------------------------------------------------------
# DS3231M RTC driver (minimal)
# ---------------------------------------------------------------------------
class DS3231:
    ADDR = 0x68

    def __init__(self, i2c):
        self._i2c = i2c

    @staticmethod
    def _bcd2dec(b):
        return (b >> 4) * 10 + (b & 0x0F)

    @staticmethod
    def _dec2bcd(d):
        return ((d // 10) << 4) | (d % 10)

    def read_hms(self):
        """Return (hour, minute, second) in 24-hour format."""
        data = self._i2c.readfrom_mem(self.ADDR, 0x00, 3)
        sec = self._bcd2dec(data[0] & 0x7F)
        minute = self._bcd2dec(data[1] & 0x7F)
        hreg = data[2]
        if hreg & 0x40:
            # 12-hour mode
            hour = self._bcd2dec(hreg & 0x1F)
            pm = bool(hreg & 0x20)
            if pm and hour != 12:
                hour += 12
            elif not pm and hour == 12:
                hour = 0
        else:
            hour = self._bcd2dec(hreg & 0x3F)
        return hour, minute, sec

    def write_hms(self, hour, minute, second):
        """Set time in 24-hour format (clears any 12-hour flag)."""
        buf = bytes(
            [
                self._dec2bcd(second),
                self._dec2bcd(minute),
                self._dec2bcd(hour),  # upper bits clear -> 24-hour mode
            ]
        )
        self._i2c.writeto_mem(self.ADDR, 0x00, buf)


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
def run():
    i2c = machine.I2C(
        0,
        sda=machine.Pin(PIN_I2C_SDA),
        scl=machine.Pin(PIN_I2C_SCL),
        freq=400_000,
    )
    rtc = DS3231(i2c)

    # First-time setup: if the coin cell is fresh and the RTC has never
    # been set, uncomment, reflash, let it run once, then re-comment.
    # rtc.write_hms(12, 0, 0)

    nix = NixieDriver(PIN_SR_DATA, PIN_SR_CLK, PIN_SR_STR, PIN_SR_OE)

    last_s = -1
    while True:
        try:
            h, m, s = rtc.read_hms()
        except OSError:
            # Transient I2C glitch - retry on the next tick.
            time.sleep_ms(100)
            continue

        if s != last_s:
            last_s = s
            nix.show_digits(
                h // 10,
                h % 10,
                m // 10,
                m % 10,
                s // 10,
                s % 10,
            )

        # Poll a few times per second so we catch the rollover quickly.
        time.sleep_ms(50)


if __name__ == "__main__":
    run()
