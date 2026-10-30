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
#
#   uv run temp/adafruit_4885.py plot [--interval 1.0] [--window 600] [--csv ...]
#       Live terminal chart of temperature (left axis) and humidity (right
#       axis) over time ('q' quits).
# -----------------------------------------------------------------------------

import argparse
import csv
import sys
import time
from collections import deque
from datetime import datetime
from pathlib import Path

from smbus2 import SMBus, i2c_msg
from textual.app import App, ComposeResult
from textual.containers import Grid
from textual.widgets import Footer, Header
from textual_hires_canvas import Canvas, TextAlign
from textual_plot import DurationFormatter, HiResMode, NumericAxisFormatter, PlotWidget
from textual_plot.plot_widget import Legend

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


class CSVLog:
    """Appends readings to a CSV file, writing the header for a new file."""

    def __init__(self, path):
        path.parent.mkdir(parents=True, exist_ok=True)
        new_file = not path.exists()
        self._file = path.open("a", newline="")
        self._writer = csv.writer(self._file)
        if new_file:
            self._writer.writerow(["timestamp", "temperature_c", "humidity_percent"])

    def write(self, stamp, temperature, humidity):
        self._writer.writerow([stamp.isoformat(timespec="seconds"),
                               f"{temperature:.2f}", f"{humidity:.2f}"])
        self._file.flush()

    def close(self):
        self._file.close()


def run_live(sensor, interval, precision, csv_path):
    print(f"SHT40 at 0x{sensor.address:02X}, serial 0x{sensor.serial_number():08X}. "
          f"Reading every {interval}s, Ctrl+C to stop.")

    log = CSVLog(csv_path) if csv_path else None
    if log:
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
                if log:
                    log.write(stamp, temperature, humidity)
            time.sleep(max(0.0, interval - (time.monotonic() - start)))
    finally:
        if log:
            log.close()


TEMPERATURE_STYLE = "orange1"
HUMIDITY_STYLE = "deep_sky_blue1"


def _padded_range(values, min_pad):
    lo, hi = min(values), max(values)
    pad = max((hi - lo) * 0.1, min_pad)
    return lo - pad, hi + pad


class DualAxisPlot(PlotWidget):
    """PlotWidget with a second, independently scaled y axis on the right.

    Right-axis data is mapped linearly onto the left axis range for drawing;
    the right margin shows ticks in the right axis' own units.
    """

    margin_right = 8

    def __init__(self, left_style, right_style, **kwargs):
        super().__init__(**kwargs)
        self._left_style = left_style
        self._right_style = right_style
        self._right_label = ""
        self._right_formatter = NumericAxisFormatter()
        self._map = (0.0, 1.0, 0.0, 1.0)  # left_lo, left_hi, right_lo, right_hi

    def compose(self) -> ComposeResult:
        with Grid():
            yield Canvas(1, 1, id="margin-top")
            yield Canvas(1, 1, id="margin-left")
            yield Canvas(1, 1, id="plot")
            yield Canvas(1, 1, id="margin-right")
            yield Canvas(1, 1, id="margin-bottom")
        yield Legend(id="legend")

    def _update_margin_sizes(self):
        grid = self.query_one(Grid)
        grid.styles.grid_size_columns = 3
        grid.styles.grid_columns = f"{self.margin_left} 1fr {self.margin_right}"
        grid.styles.grid_rows = f"{self.margin_top} 1fr {self.margin_bottom}"
        for margin in ("#margin-top", "#margin-bottom"):
            self.query_one(margin).styles.column_span = 3

    def set_right_ylabel(self, label):
        self._right_label = label

    def set_axis_ranges(self, left, right):
        """Fix both y ranges as (lo, hi) pairs; right values map onto left."""
        self._map = (*left, *right)
        self.set_ylimits(*left)

    def to_left(self, values):
        left_lo, left_hi, right_lo, right_hi = self._map
        scale = (left_hi - left_lo) / (right_hi - right_lo)
        return [left_lo + (v - right_lo) * scale for v in values]

    def _to_right(self, value):
        left_lo, left_hi, right_lo, right_hi = self._map
        return right_lo + (value - left_lo) * (right_hi - right_lo) / (left_hi - left_lo)

    def _render_y_ticks(self):
        canvas = self.query_one("#plot", Canvas)
        axis_style = str(self.get_component_rich_style("plot--axis"))
        axes = (
            (self.query_one("#margin-left", Canvas), self.margin_left, self._y_formatter,
             self._y_min, self._y_max, self._left_style, 0, (0, 0, 0, 2)),
            (self.query_one("#margin-right", Canvas), self.margin_right, self._right_formatter,
             self._to_right(self._y_min), self._to_right(self._y_max), self._right_style,
             self._scale_rectangle.right, (0, 2, 0, 0)),
        )
        for margin, width, formatter, lo, hi, style, axis_x, quad in axes:
            margin.reset()
            right_side = margin.id == "margin-right"
            ticks, labels = formatter.get_ticks_and_labels(lo, hi)
            for tick, label in zip(ticks, labels):
                if tick < lo or tick > hi:
                    continue
                value = self.to_left([tick])[0] if right_side else tick
                _, y = self.get_pixel_from_coordinate(0.0, value)
                if tick == lo:
                    y += 1
                pixel = self.combine_quad_with_pixel(quad, canvas, axis_x, y)
                canvas.set_pixel(axis_x, y, pixel, style=axis_style)
                label = label[: width - 1]
                if right_side:
                    margin.write_text(1, y, f"[{style}]{label}", TextAlign.LEFT)
                else:
                    margin.write_text(width - 2, y, f"[{style}]{label}", TextAlign.RIGHT)

    def _render_y_label(self):
        margin = self.query_one("#margin-top", Canvas)
        margin.write_text(self.margin_left - 2, 0, f"[bold {self._left_style}]{self._y_label}",
                          TextAlign.LEFT)
        margin.write_text(margin.size.width - 1, 0, f"[bold {self._right_style}]{self._right_label}",
                          TextAlign.RIGHT)


class PlotApp(App):
    """Rolling chart of temperature and humidity against elapsed time."""

    TITLE = "SHT40 temperature & humidity"
    CSS = "DualAxisPlot { height: 1fr; }"
    BINDINGS = [("q", "quit", "Quit"), ("c", "clear", "Clear")]

    def __init__(self, sensor, interval, precision, window, csv_path):
        super().__init__()
        self.sensor = sensor
        self.interval = interval
        self.precision = precision
        self.window = window
        self.csv_path = csv_path
        self.csv_log = None
        self.samples = deque()  # (elapsed_s, temperature_c, humidity_percent)
        self.start = time.monotonic()

    def compose(self) -> ComposeResult:
        yield Header()
        yield DualAxisPlot(TEMPERATURE_STYLE, HUMIDITY_STYLE)
        yield Footer()

    def on_mount(self):
        self.sub_title = (f"0x{self.sensor.address:02X}, serial 0x{self.sensor.serial_number():08X}, "
                          f"every {self.interval}s")
        if self.csv_path:
            self.csv_log = CSVLog(self.csv_path)
        plot = self.query_one(DualAxisPlot)
        plot.set_x_formatter(DurationFormatter())
        plot.set_xlabel("Elapsed")
        plot.set_ylabel("Temperature (C)")
        plot.set_right_ylabel("Humidity (%RH)")
        self.read_sensor()
        self.set_interval(self.interval, self.read_sensor)

    def on_unmount(self):
        if self.csv_log:
            self.csv_log.close()

    def read_sensor(self):
        stamp = datetime.now()
        try:
            temperature, humidity = self.sensor.measure(self.precision)
        except OSError as err:
            self.notify(f"read failed: {err}", severity="error")
            return
        if self.csv_log:
            self.csv_log.write(stamp, temperature, humidity)
        elapsed = time.monotonic() - self.start
        self.samples.append((elapsed, temperature, humidity))
        while self.samples[0][0] < elapsed - self.window:
            self.samples.popleft()
        self.title = f"{temperature:.2f} C   {humidity:.1f} %RH   [{stamp:%H:%M:%S}]"
        self.redraw()

    def redraw(self):
        plot = self.query_one(DualAxisPlot)
        plot.clear()
        if len(self.samples) < 2:
            return
        times = [s[0] for s in self.samples]
        temperatures = [s[1] for s in self.samples]
        humidities = [s[2] for s in self.samples]
        plot.set_axis_ranges(_padded_range(temperatures, 0.25), _padded_range(humidities, 1.0))
        plot.plot(times, temperatures, line_style=TEMPERATURE_STYLE, hires_mode=HiResMode.BRAILLE)
        plot.plot(times, plot.to_left(humidities), line_style=HUMIDITY_STYLE,
                  hires_mode=HiResMode.BRAILLE)
        plot.set_xlimits(max(0.0, times[-1] - self.window), times[-1])

    def action_clear(self):
        self.samples.clear()
        self.redraw()


def run_plot(sensor, interval, precision, window, csv_path):
    PlotApp(sensor, interval, precision, window, csv_path).run()


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

    plot = sub.add_parser("plot", help="chart readings over time in the terminal")
    plot.add_argument("--interval", type=float, default=1.0, help="seconds between readings (default: %(default)s)")
    plot.add_argument("--precision", choices=MEASURE, default="high", help="measurement repeatability (default: %(default)s)")
    plot.add_argument("--window", type=float, default=600, help="seconds of history shown (default: %(default)s)")
    plot.add_argument("--csv", type=Path, help="also append readings to this CSV file")

    args = parser.parse_args()
    sensor = SHT40(args.bus, args.address)
    try:
        sensor.reset()
        if args.mode == "live":
            run_live(sensor, args.interval, args.precision, args.csv)
        else:
            run_plot(sensor, args.interval, args.precision, args.window, args.csv)
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
