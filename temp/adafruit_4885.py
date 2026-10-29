# Temperature & Humidity Sensor Module
#
# Hardware---------------------------------------------------------------------
# Model: Adafruit 4885 (Sensirion SHT40, STEMMA QT / Qwiic)
# Power Supply: 3.3V - 5V
# Interface: I2C, address 0x44
# Accuracy: +/-0.2 C, +/-1.8 %RH
# Reference: https://www.adafruit.com/product/4885
#
# Connections------------------------------------------------------------------
# Raspberry Pi GPIO Pinout:
# Standard 40-pin header; GPIO numbers use BCM numbering.
# I2C must be enabled (sudo raspi-config -> Interface Options -> I2C).
#
# Module Pin    STEMMA QT wire    Pi Physical Pin    Function
# VIN           Red               1                  3.3V power
# GND           Black             6                  Ground
# SDA           Blue              3                  GPIO2 (I2C1 SDA)
# SCL           Yellow            5                  GPIO3 (I2C1 SCL)
#
# Check wiring with: i2cdetect -y 1   (sensor shows up at 0x44)
# -----------------------------------------------------------------------------
#
# Usage------------------------------------------------------------------------
#   uv run temp/adafruit_4885.py live [--interval 1.0] [--precision high]
#       Print temperature and humidity readings until Ctrl+C.
#
#   uv run temp/adafruit_4885.py live --csv temp/captures/room.csv
#       Same, and also append every reading to a CSV file.
# -----------------------------------------------------------------------------

import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path

from smbus2 import SMBus, i2c_msg

I2C_BUS = 1
I2C_ADDRESS = 0x44

CMD_SERIAL = 0x89
CMD_SOFT_RESET = 0x94
# Measurement command and the time it takes, per repeatability level.
MEASURE = {
    "high": (0xFD, 0.010),
    "medium": (0xF6, 0.005),
    "low": (0xE0, 0.002),
}


# Sensor -----------------------------------------------------------------------


def _crc8(data):
    """Sensirion CRC-8: polynomial 0x31, init 0xFF."""
    crc = 0xFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x31) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


class SHT40:
    def __init__(self, bus=I2C_BUS, address=I2C_ADDRESS):
        self.address = address
        self._bus = SMBus(bus)

    def _command(self, cmd, delay, length):
        """Send a one-byte command and read back `length` CRC-checked bytes."""
        self._bus.i2c_rdwr(i2c_msg.write(self.address, [cmd]))
        time.sleep(delay)
        read = i2c_msg.read(self.address, length)
        self._bus.i2c_rdwr(read)
        raw = list(read)
        words = []
        for i in range(0, length, 3):
            if _crc8(raw[i:i + 2]) != raw[i + 2]:
                raise IOError(f"CRC mismatch in response to 0x{cmd:02X}")
            words.append(raw[i] << 8 | raw[i + 1])
        return words

    def reset(self):
        self._bus.i2c_rdwr(i2c_msg.write(self.address, [CMD_SOFT_RESET]))
        time.sleep(0.001)

    def serial_number(self):
        hi, lo = self._command(CMD_SERIAL, 0.001, 6)
        return hi << 16 | lo

    def measure(self, precision="high"):
        """Return (temperature_c, humidity_percent)."""
        cmd, delay = MEASURE[precision]
        t_raw, rh_raw = self._command(cmd, delay, 6)
        temperature = -45 + 175 * t_raw / 65535
        humidity = -6 + 125 * rh_raw / 65535
        return temperature, min(max(humidity, 0.0), 100.0)

    def close(self):
        self._bus.close()


# Modes ------------------------------------------------------------------------


def run_live(sensor, interval, precision, csv_path):
    print(f"SHT40 at 0x{sensor.address:02X}, serial 0x{sensor.serial_number():08X}. "
          f"Reading every {interval}s, Ctrl+C to stop.")

    writer, csv_file = None, None
    if csv_path:
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        new_file = not csv_path.exists()
        csv_file = csv_path.open("a", newline="")
        writer = csv.writer(csv_file)
        if new_file:
            writer.writerow(["timestamp", "temperature_c", "humidity_percent"])
        print(f"Logging to {csv_path}")

    try:
        while True:
            start = time.monotonic()
            stamp = datetime.now()
            try:
                temperature, humidity = sensor.measure(precision)
            except OSError as err:
                print(f"[{stamp:%H:%M:%S}] read failed: {err}")
            else:
                print(f"[{stamp:%H:%M:%S}] {temperature:6.2f} C   {humidity:5.1f} %RH")
                if writer:
                    writer.writerow([stamp.isoformat(timespec="seconds"),
                                     f"{temperature:.2f}", f"{humidity:.2f}"])
                    csv_file.flush()
            time.sleep(max(0.0, interval - (time.monotonic() - start)))
    finally:
        if csv_file:
            csv_file.close()


def main():
    parser = argparse.ArgumentParser(description="Adafruit 4885 (SHT40) temperature & humidity sensor")
    parser.add_argument("--bus", type=int, default=I2C_BUS, help="I2C bus number (default: %(default)s)")
    parser.add_argument("--address", type=lambda v: int(v, 0), default=I2C_ADDRESS,
                        help="I2C address (default: 0x44)")
    sub = parser.add_subparsers(dest="mode", required=True)

    live = sub.add_parser("live", help="print readings continuously")
    live.add_argument("--interval", type=float, default=1.0, help="seconds between readings (default: %(default)s)")
    live.add_argument("--precision", choices=MEASURE, default="high", help="measurement repeatability (default: %(default)s)")
    live.add_argument("--csv", type=Path, help="also append readings to this CSV file")

    args = parser.parse_args()
    sensor = SHT40(args.bus, args.address)
    try:
        sensor.reset()
        run_live(sensor, args.interval, args.precision, args.csv)
    except KeyboardInterrupt:
        print()
    except OSError as err:
        print(f"Cannot talk to SHT40 at 0x{args.address:02X} on I2C bus {args.bus}: {err}\n"
              f"Check wiring and run: i2cdetect -y {args.bus}", file=sys.stderr)
        return 1
    finally:
        sensor.close()


if __name__ == "__main__":
    sys.exit(main())
