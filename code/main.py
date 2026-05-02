"""
Nixie Tube Clock - CircuitPython firmware for RP2350

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

Display assignment (left-to-right): N6 N5 : N4 N3 : N2 N1  = HH MM SS.
  (Board has tubes mirrored from the original schematic labeling.)

Data flow and bit order:
  Bit 0 of the 64-bit frame corresponds to U6.HVOUT1 (first stage the MCU
  clocks into). Bit 63 corresponds to U7.HVOUT32. When the frame is packed
  big-endian and shifted out MSB-first, the MSB of byte 0 travels all the
  way through and lands at U7.HVOUT32, and the LSB of byte 7 ends at
  U6.HVOUT1 - which matches the definition above.

HV5122 control (from Microchip DS20005418B):
  STR LOW              -> all outputs forced ON (all NMOS conducting)
  STR HIGH + OE LOW    -> all outputs OFF (high impedance)
  STR HIGH + OE HIGH   -> outputs reflect shift register data
  Data shifts on the HIGH-TO-LOW (falling) edge of CLK.

  STR is held HIGH at all times. To avoid glitching during a shift, OE
  is driven LOW while clocking in new data, then set HIGH to display.

Note on SPI: the RP2350's hardware SPI1 uses GPIO10 for SCK and GPIO11 for
TX, but the board wires them the other way (GPIO10=data, GPIO11=clock), so
this script bit-bangs the shift register. 64 bits once per second is trivial.
"""

import board
import busio
import digitalio
import time

# ---------------------------------------------------------------------------
# Pin assignments
# ---------------------------------------------------------------------------
PIN_SR_DATA = board.GP10
PIN_SR_CLK = board.GP11
PIN_SR_STR = board.GP12
PIN_SR_OE = board.GP13

PIN_I2C_SDA = board.GP16
PIN_I2C_SCL = board.GP17

TUBE_NAMES = ("N1[S1]", "N2[S10]", "N3[M1]", "N4[M10]", "N5[H1]", "N6[H10]")


# ---------------------------------------------------------------------------
# Helper: configure a digital output pin
# ---------------------------------------------------------------------------
def _make_output(pin, initial_value=False):
    dio = digitalio.DigitalInOut(pin)
    dio.direction = digitalio.Direction.OUTPUT
    dio.value = initial_value
    return dio


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
        self._data = _make_output(data_pin, False)
        self._clk = _make_output(clk_pin, False)
        # STR must idle HIGH for normal operation. LOW forces all outputs ON.
        self._str = _make_output(str_pin, True)
        # OE LOW keeps all outputs off until we clock in real data.
        self._oe = _make_output(oe_pin, False)
        self._prev_frame = -1
        self._prev_digits = (None,) * 6
        self._update_count = 0
        print("[NIXIE] Driver initialized")
        print("[NIXIE]   DATA=GP10  CLK=GP11  STR=GP12  OE=GP13")
        print("[NIXIE]   STR=HIGH (normal), OE=LOW (outputs blanked)")

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
        """Clock out `buf` MSB-first to the HV5122 chain.

        Data is latched on the falling edge of CLK per the HV5122 datasheet.
        We set the data bit, raise CLK, then lower CLK. The data is stable
        well before the falling edge at CircuitPython speeds.
        """
        data = self._data
        clk = self._clk
        for byte in buf:
            b = byte
            for _ in range(8):
                data.value = bool((b >> 7) & 1)
                clk.value = True
                clk.value = False
                b = (b << 1) & 0xFF

    def show_digits(self, d1, d2, d3, d4, d5, d6):
        """Light one digit per tube. Pass None to leave a tube dark."""
        digits = (d1, d2, d3, d4, d5, d6)
        frame = 0
        for tube, digit in enumerate(digits, start=1):
            if digit is None:
                continue
            frame |= 1 << self._bit_index(tube, digit)

        buf = frame.to_bytes(8, "big")
        self._update_count += 1

        # Show which tubes changed since the last update.
        changed = []
        for i in range(6):
            if digits[i] != self._prev_digits[i]:
                old = "-" if self._prev_digits[i] is None else str(self._prev_digits[i])
                new = "-" if digits[i] is None else str(digits[i])
                changed.append("{}: {} -> {}".format(TUBE_NAMES[i], old, new))

        print("----------- update #{} -----------".format(self._update_count))
        print("[NIXIE] Digits:  {}{}:{}{}:{}{}".format(*digits))
        if changed:
            for c in changed:
                print("[NIXIE]   CHANGED  {}".format(c))
        else:
            print("[NIXIE]   (no tube changes)")

        # Frame-level debug: hex and per-chip breakdown.
        u6_bits = frame & 0xFFFFFFFF
        u7_bits = (frame >> 32) & 0xFFFFFFFF
        print("[NIXIE] Frame:   0x{:016x}".format(frame))
        print("[NIXIE]   U6 (N1-N3): 0x{:08x}".format(u6_bits))
        print("[NIXIE]   U7 (N4-N6): 0x{:08x}".format(u7_bits))

        if self._prev_frame >= 0:
            diff = frame ^ self._prev_frame
            if diff:
                print("[NIXIE]   Bit diff:    0x{:016x}  ({} bits toggled)".format(
                    diff, bin(diff).count("1")
                ))

        print("[NIXIE] Buf out: {}".format(
            " ".join("{:02x}".format(b) for b in buf)
        ))

        self._prev_frame = frame
        self._prev_digits = digits

        # Update sequence:
        #   1. OE LOW   - blank outputs while we shift in new data
        #   2. Shift 64 bits into the register chain (data latched on CLK fall)
        #   3. OE HIGH  - outputs now reflect the new shift register contents
        # STR stays HIGH throughout (LOW would force all outputs ON).
        self._oe.value = False
        self._shift_out(buf)
        self._oe.value = True

    def blank(self):
        """Extinguish all tubes (leaves shift-register contents intact)."""
        print("[NIXIE] Blanking outputs (OE -> LOW)")
        self._oe.value = False

    def clear(self):
        """Shift in an all-zero frame, then enable outputs (truly blank)."""
        self._oe.value = False
        self._shift_out(b"\x00" * 8)
        self._prev_frame = 0
        self._prev_digits = (None,) * 6
        print("[NIXIE] Cleared (zero frame shifted in, OE -> HIGH)")
        self._oe.value = True


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

    def _try_lock_with_timeout(self, timeout_ms=50):
        """Attempt to acquire the I2C lock with a timeout.
        Returns True if locked, False if timed out."""
        start = time.monotonic()
        timeout_s = timeout_ms / 1000.0
        while not self._i2c.try_lock():
            if time.monotonic() - start > timeout_s:
                return False
        return True

    def _read_register(self, reg, length):
        """Read `length` bytes starting at register `reg`."""
        buf = bytearray(length)
        if not self._try_lock_with_timeout():
            # Bus is stuck. Try to recover by deinit/reinit.
            try:
                self._i2c.unlock()
            except RuntimeError:
                pass
            raise OSError("I2C lock timeout (bus may be stuck)")
        try:
            self._i2c.writeto_then_readfrom(self.ADDR, bytes([reg]), buf)
        finally:
            self._i2c.unlock()
        return buf

    def _write_register(self, reg, data):
        """Write `data` bytes starting at register `reg`."""
        if not self._try_lock_with_timeout():
            try:
                self._i2c.unlock()
            except RuntimeError:
                pass
            raise OSError("I2C lock timeout (bus may be stuck)")
        try:
            self._i2c.writeto(self.ADDR, bytes([reg]) + data)
        finally:
            self._i2c.unlock()

    def read_hms(self):
        """Return (hour, minute, second) in 24-hour format."""
        data = self._read_register(0x00, 3)
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

    def read_status(self):
        """Read the DS3231 status register (0x0F) for diagnostics."""
        data = self._read_register(0x0F, 1)
        return data[0]

    def read_temperature(self):
        """Read the DS3231's internal temperature sensor (deg C)."""
        data = self._read_register(0x11, 2)
        temp_msb = data[0]
        temp_lsb = data[1]
        # MSB is signed integer part, upper 2 bits of LSB are 0.25 C steps.
        temp = temp_msb + ((temp_lsb >> 6) * 0.25)
        if temp_msb & 0x80:
            temp -= 256
        return temp

    def write_hms(self, hour, minute, second):
        """Set time in 24-hour format (clears any 12-hour flag)."""
        buf = bytes(
            [
                self._dec2bcd(second),
                self._dec2bcd(minute),
                self._dec2bcd(hour),  # upper bits clear -> 24-hour mode
            ]
        )
        self._write_register(0x00, buf)


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
def run():
    print()
    print("=" * 50)
    print("  NIXIE TUBE CLOCK  -  CircuitPython / RP2350")
    print("=" * 50)
    print()

    # --- I2C init and bus scan ---
    print("[INIT] Setting up I2C  (SDA=GP16, SCL=GP17, 400kHz)")
    i2c = busio.I2C(scl=PIN_I2C_SCL, sda=PIN_I2C_SDA, frequency=400_000)

    start_lock = time.monotonic()
    while not i2c.try_lock():
        if time.monotonic() - start_lock > 1.0:
            print("[I2C]  WARNING: Could not acquire I2C lock for bus scan")
            break
    else:
        try:
            devices = i2c.scan()
            print("[I2C]  Bus scan found {} device(s): {}".format(
                len(devices),
                ", ".join("0x{:02x}".format(d) for d in devices) if devices else "(none)"
            ))
            if 0x68 in devices:
                print("[I2C]  DS3231 detected at 0x68  [OK]")
            else:
                print("[I2C]  WARNING: DS3231 not found at 0x68!")
        finally:
            i2c.unlock()

    rtc = DS3231(i2c)

    # --- Read RTC status and temperature at startup ---
    try:
        status = rtc.read_status()
        osf = bool(status & 0x80)
        print("[RTC]  Status register: 0x{:02x}".format(status))
        if osf:
            print("[RTC]  WARNING: Oscillator Stop Flag is set!")
            print("[RTC]    The RTC may have lost power / time may be stale.")
        else:
            print("[RTC]  Oscillator running normally  [OK]")
    except OSError as e:
        print("[RTC]  Could not read status register:", e)

    try:
        temp_c = rtc.read_temperature()
        print("[RTC]  DS3231 temperature: {:.2f} C / {:.1f} F".format(
            temp_c, temp_c * 9.0 / 5.0 + 32.0
        ))
    except OSError as e:
        print("[RTC]  Could not read temperature:", e)

    try:
        h, m, s = rtc.read_hms()
        print("[RTC]  Initial time: {:02d}:{:02d}:{:02d}".format(h, m, s))
    except OSError as e:
        print("[RTC]  Could not read initial time:", e)

    # First-time setup: if the coin cell is fresh and the RTC has never
    # been set, uncomment, reflash, let it run once, then re-comment.
    # rtc.write_hms(12, 0, 0)

    print()
    print("[INIT] Setting up shift register outputs")
    nix = NixieDriver(PIN_SR_DATA, PIN_SR_CLK, PIN_SR_STR, PIN_SR_OE)

    print()
    print("[LOOP] Entering main loop (50 ms poll interval)")
    print()

    last_s = -1
    polls_since_update = 0
    i2c_errors = 0
    consecutive_errors = 0
    start_mono = time.monotonic()

    while True:
        try:
            h, m, s = rtc.read_hms()
            consecutive_errors = 0
        except OSError as e:
            i2c_errors += 1
            consecutive_errors += 1
            print("[ERR]  I2C read failed (#{} total, {} consecutive): {}".format(
                i2c_errors, consecutive_errors, e
            ))
            if consecutive_errors >= 10:
                # Bus is probably stuck. Reinit I2C to recover.
                print("[ERR]  Too many consecutive failures, reinitializing I2C bus...")
                try:
                    i2c.deinit()
                except Exception:
                    pass
                time.sleep(0.1)
                i2c = busio.I2C(scl=PIN_I2C_SCL, sda=PIN_I2C_SDA, frequency=400_000)
                rtc = DS3231(i2c)
                consecutive_errors = 0
                print("[ERR]  I2C bus reinitialized")
            time.sleep(0.1)
            continue

        polls_since_update += 1

        if s != last_s:
            uptime = time.monotonic() - start_mono
            print("[TIME] {:02d}:{:02d}:{:02d}  (uptime {:.0f}s, {} polls since last tick, {} i2c errors)".format(
                h, m, s, uptime, polls_since_update, i2c_errors
            ))
            polls_since_update = 0
            last_s = s
            # Tube order is mirrored on the board:
            #   N1=S1, N2=S10, N3=M1, N4=M10, N5=H1, N6=H10
            nix.show_digits(
                s % 10,   # N1: seconds ones
                s // 10,  # N2: seconds tens
                m % 10,   # N3: minutes ones
                m // 10,  # N4: minutes tens
                h % 10,   # N5: hours ones
                h // 10,  # N6: hours tens
            )
            print()

        # Poll a few times per second so we catch the rollover quickly.
        time.sleep(0.05)


run()