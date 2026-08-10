#!/usr/bin/env python3
"""
BQ25887 Battery Charger Monitor
Reads VBUS, IBUS, ICHG, VBAT, VCELLTOP, VCELLBOT, TDIE every 10 seconds.
Logs to a timestamped CSV file for later plotting.
Requires: smbus2  →  pip install smbus2
Usage:    python3 bq25887_monitor.py
"""

import time
import csv
import os
import smbus2

# ── Configuration ────────────────────────────────────────────────────────────
I2C_BUS     = 1        # /dev/i2c-1 on Raspberry Pi
BQ_ADDR     = 0x6A     # Fixed address on BQ25887
INTERVAL_S  = 10       # Polling interval in seconds

# CSV file — named with session start time, saved alongside this script
_session_ts = time.strftime("%Y%m%d_%H%M%S")
CSV_FILE    = os.path.join(os.path.dirname(__file__), f"bq25887_{_session_ts}.csv")

CSV_HEADERS = [
    "datetime", "elapsed_s",
    "vbus_mv", "ibus_ma", "ichg_ma",
    "vbat_mv", "vcelltop_mv", "vcellbot_mv", "delta_cell_mv",
    "tdie_c",
    "power_good", "vbus_source", "chrg_status", "flags", "fault",
]

# ── Register addresses ────────────────────────────────────────────────────────
REG_ADC_CTRL   = 0x15  # ADC Control
REG_IBUS_H     = 0x17  # IBUS  MSB
REG_IBUS_L     = 0x18  # IBUS  LSB
REG_ICHG_H     = 0x19  # ICHG  MSB
REG_ICHG_L     = 0x1A  # ICHG  LSB
REG_VBUS_H     = 0x1B  # VBUS  MSB
REG_VBUS_L     = 0x1C  # VBUS  LSB
REG_VBAT_H     = 0x1D  # VBAT  MSB
REG_VBAT_L     = 0x1E  # VBAT  LSB
REG_VCELLTOP_H = 0x1F  # Top cell MSB
REG_VCELLTOP_L = 0x20  # Top cell LSB
REG_TDIE_H     = 0x23  # Die temp MSB
REG_TDIE_L     = 0x24  # Die temp LSB
REG_VCELLBOT_H = 0x26  # Bottom cell MSB
REG_VCELLBOT_L = 0x27  # Bottom cell LSB
REG_STATUS1    = 0x0B  # Charger Status 1
REG_STATUS2    = 0x0C  # Charger Status 2
REG_FAULT      = 0x0E  # Fault Status

# ── ADC Control bits ──────────────────────────────────────────────────────────
ADC_EN_BIT        = 0x80  # bit7: enable ADC
ADC_CONTINUOUS    = 0x00  # bit6=0: continuous conversion
ADC_SAMPLE_15BIT  = 0x00  # bits[5:4]=00: 15-bit effective resolution

# ── Charge status decode ──────────────────────────────────────────────────────
CHRG_STAT = {
    0b000: "Not charging",
    0b001: "Trickle charge",
    0b010: "Pre-charge",
    0b011: "Fast charge (CC)",
    0b100: "Taper charge (CV)",
    0b101: "Top-off timer",
    0b110: "Charge complete",
    0b111: "Reserved",
}

VBUS_STAT = {
    0b000: "No input",
    0b001: "USB host (500 mA)",
    0b010: "USB CDP (1.5 A)",
    0b011: "Adapter 3 A (PSEL low)",
    0b100: "Poor source (×7)",
    0b101: "Unknown adapter (500 mA)",
    0b110: "Non-standard adapter",
}


def read_reg(bus, reg):
    return bus.read_byte_data(BQ_ADDR, reg)


def read_word_signed(bus, reg_h, reg_l):
    """Read two consecutive registers and return a signed 16-bit integer."""
    msb = read_reg(bus, reg_h)
    lsb = read_reg(bus, reg_l)
    raw = (msb << 8) | lsb
    # Two's complement for 16-bit
    return raw if raw < 0x8000 else raw - 0x10000


def enable_adc(bus):
    # Disable watchdog timer (bits[5:4] = 00 in REG05)
    reg05 = read_reg(bus, 0x05)
    bus.write_byte_data(BQ_ADDR, 0x05, reg05 & 0b11001111)  # clear bits 5:4
    # Enable ADC continuous mode
    bus.write_byte_data(BQ_ADDR, REG_ADC_CTRL, ADC_EN_BIT | ADC_CONTINUOUS | ADC_SAMPLE_15BIT)
    """Enable ADC in continuous mode at 15-bit resolution."""
    val = ADC_EN_BIT | ADC_CONTINUOUS | ADC_SAMPLE_15BIT
    bus.write_byte_data(BQ_ADDR, REG_ADC_CTRL, val)
    # Verify ADC started (EN bit stays set in continuous mode)
    check = read_reg(bus, REG_ADC_CTRL)
    if not (check & ADC_EN_BIT):
        print("⚠  Warning: ADC_EN bit cleared immediately — check VBUS/VBAT levels.")


def decode_status(bus):
    """Return human-readable charge and power-good status strings."""
    s1 = read_reg(bus, REG_STATUS1)
    s2 = read_reg(bus, REG_STATUS2)

    chrg_code = s1 & 0x07
    chrg_str  = CHRG_STAT.get(chrg_code, f"Unknown (0b{chrg_code:03b})")

    vbus_code = (s2 >> 4) & 0x07
    vbus_str  = VBUS_STAT.get(vbus_code, f"Unknown (0b{vbus_code:03b})")

    pg        = "✓ Power good" if (s2 & 0x80) else "✗ Not power good"
    wd        = "  [WD expired]" if (s1 & 0x08) else ""
    iindpm    = "  [IINDPM]"     if (s1 & 0x40) else ""
    vindpm    = "  [VINDPM]"     if (s1 & 0x20) else ""
    treg      = "  [TREG]"       if (s1 & 0x10) else ""

    return pg, vbus_str, chrg_str, wd + iindpm + vindpm + treg


def decode_fault(bus):
    f = read_reg(bus, REG_FAULT)
    faults = []
    if f & 0x80: faults.append("VBUS OVP")
    if f & 0x40: faults.append("Thermal shutdown")
    if f & 0x10: faults.append("Safety timer expired")
    return ", ".join(faults) if faults else "None"


def read_all(bus):
    # ADC results: LSB = 1 mV or 1 mA, signed 16-bit
    vbus_mv     = read_word_signed(bus, REG_VBUS_H,     REG_VBUS_L)
    ibus_ma     = read_word_signed(bus, REG_IBUS_H,     REG_IBUS_L)
    ichg_ma     = read_word_signed(bus, REG_ICHG_H,     REG_ICHG_L)
    vbat_mv     = read_word_signed(bus, REG_VBAT_H,     REG_VBAT_L)
    vcelltop_mv = read_word_signed(bus, REG_VCELLTOP_H, REG_VCELLTOP_L)
    vcellbot_mv = read_word_signed(bus, REG_VCELLBOT_H, REG_VCELLBOT_L)

    # TDIE: LSB = 0.5 °C, signed 16-bit
    tdie_raw    = read_word_signed(bus, REG_TDIE_H, REG_TDIE_L)
    tdie_c      = tdie_raw * 0.5

    pg, vbus_stat, chrg_stat, flags = decode_status(bus)
    fault = decode_fault(bus)

    return {
        "VBUS":     vbus_mv,
        "IBUS":     ibus_ma,
        "ICHG":     ichg_ma,
        "VBAT":     vbat_mv,
        "VCELLTOP": vcelltop_mv,
        "VCELLBOT": vcellbot_mv,
        "TDIE":     tdie_c,
        "PG":       pg,
        "VBUS_SRC": vbus_stat,
        "CHRG":     chrg_stat,
        "FLAGS":    flags,
        "FAULT":    fault,
    }


def print_reading(d, n, elapsed):
    ts = time.strftime("%H:%M:%S")
    print(f"\n{'─'*52}")
    print(f" #{n:04d}  {ts}  (+{elapsed:.0f} s)  BQ25887")
    print(f"{'─'*52}")
    print(f"  {d['PG']}")
    print(f"  Source  : {d['VBUS_SRC']}")
    print(f"  Charge  : {d['CHRG']}")
    if d['FLAGS']:
        print(f"  Flags   : {d['FLAGS']}")
    if d['FAULT'] != "None":
        print(f"  ⚠ Fault : {d['FAULT']}")
    print(f"{'─'*52}")
    print(f"  VBUS    : {d['VBUS']:>6}  mV  ({d['VBUS']/1000:.3f} V)")
    print(f"  IBUS    : {d['IBUS']:>6}  mA  ({d['IBUS']/1000:.3f} A)")
    print(f"  ICHG    : {d['ICHG']:>6}  mA  ({d['ICHG']/1000:.3f} A)")
    print(f"  VBAT    : {d['VBAT']:>6}  mV  ({d['VBAT']/1000:.3f} V)")
    print(f"  Cell top: {d['VCELLTOP']:>6}  mV  ({d['VCELLTOP']/1000:.3f} V)")
    print(f"  Cell bot: {d['VCELLBOT']:>6}  mV  ({d['VCELLBOT']/1000:.3f} V)")
    print(f"  Δ cells : {d['VCELLTOP']-d['VCELLBOT']:>+6}  mV")
    print(f"  T die   : {d['TDIE']:>6.1f}  °C")


def write_csv_row(writer, d, elapsed):
    writer.writerow({
        "datetime":      time.strftime("%Y-%m-%d %H:%M:%S"),
        "elapsed_s":     f"{elapsed:.1f}",
        "vbus_mv":       d["VBUS"],
        "ibus_ma":       d["IBUS"],
        "ichg_ma":       d["ICHG"],
        "vbat_mv":       d["VBAT"],
        "vcelltop_mv":   d["VCELLTOP"],
        "vcellbot_mv":   d["VCELLBOT"],
        "delta_cell_mv": d["VCELLTOP"] - d["VCELLBOT"],
        "tdie_c":        f"{d['TDIE']:.1f}",
        "power_good":    1 if "✓" in d["PG"] else 0,
        "vbus_source":   d["VBUS_SRC"],
        "chrg_status":   d["CHRG"],
        "flags":         d["FLAGS"],
        "fault":         d["FAULT"],
    })


def main():
    print("BQ25887 Monitor — press Ctrl+C to stop")
    print(f"Bus: i2c-{I2C_BUS}, Address: 0x{BQ_ADDR:02X}, Interval: {INTERVAL_S} s")
    print(f"Logging to: {CSV_FILE}")

    start_time = time.monotonic()

    with smbus2.SMBus(I2C_BUS) as bus, \
         open(CSV_FILE, "w", newline="") as csvf:

        writer = csv.DictWriter(csvf, fieldnames=CSV_HEADERS)
        writer.writeheader()
        csvf.flush()

        # Enable continuous ADC conversion
        enable_adc(bus)
        # Give ADC one cycle to complete first conversion (~24 ms at 15-bit)
        time.sleep(0.1)

        n = 0
        while True:
            try:
                n += 1
                elapsed = time.monotonic() - start_time
                d = read_all(bus)
                print_reading(d, n, elapsed)
                write_csv_row(writer, d, elapsed)
                csvf.flush()          # ensure data hits disk after each row
            except OSError as e:
                print(f"⚠  I2C error: {e}")
            time.sleep(INTERVAL_S)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
