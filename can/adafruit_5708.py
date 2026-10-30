# CAN Bus Transceiver Module
#
# Hardware---------------------------------------------------------------------
# Model: Adafruit 5708 (CAN Pal, NXP TJA1051T/3)
# Power Supply: 3.3V (onboard charge pump makes the 5V for the bus side)
# Interface: CAN (CANH/CANL terminal block), logic-level RX/TX
# Termination: switchable 120 ohm on board
# Reference: https://www.adafruit.com/product/5708
#
# The CAN Pal is only a transceiver: it has no CAN controller. The Pi has no
# CAN controller either (same as the RP2350, which uses PIO/can2040), so this
# script decodes classic CAN 2.0A/B frames in software from timestamped RX
# edges. It is a passive listener: TX is held recessive and frames are never
# ACKed, so the bus needs at least two other nodes (or a sender that tolerates
# missing ACKs and keeps retransmitting). Works reliably up to ~125 kbit/s;
# higher bitrates depend on Pi load.
#
# Connections------------------------------------------------------------------
# Raspberry Pi GPIO Pinout:
# Standard 40-pin header; GPIO numbers use BCM numbering.
# Power the module from 3.3V only: VCC also sets the RX logic level, so 5V
# would put 5V on a Pi GPIO.
#
# Module Pin    Pi Physical Pin    Function
# VCC           1                  3.3V power
# GND           6                  Ground
# RX            15                 GPIO22 (transceiver output -> Pi)
# TX            13                 GPIO27 (Pi -> transceiver, held high)
# CANH / CANL   -                  CAN bus pair (terminal block)
# GND (block)   -                  Bus ground reference
# -----------------------------------------------------------------------------
#
# Usage------------------------------------------------------------------------
#   uv run can/adafruit_5708.py live [--bitrate 125000]
#       Print every received CAN frame until Ctrl+C.
#
#   uv run can/adafruit_5708.py live --csv can/captures/bus.csv
#       Same, and also append every frame to a CSV file.
#
#   uv run can/adafruit_5708.py detect [--frames 20]
#       Capture traffic and report which standard bitrate decodes it.
# -----------------------------------------------------------------------------

import argparse
import csv
import math
import queue
import sys
from datetime import datetime
from pathlib import Path

import lgpio

GPIO_CHIP = 0
RX_PIN = 22
TX_PIN = 27
BITRATE = 125_000
BITRATES = (10_000, 20_000, 50_000, 83_333, 100_000, 125_000, 250_000, 500_000, 800_000, 1_000_000)
# Recessive bits that separate frames: ACK delimiter + EOF + intermission is
# 11, while inside a frame stuffing limits runs to 5 (6 around the ACK slot).
IDLE_BITS = 10
CRC15_POLY = 0x4599


# Receiver ---------------------------------------------------------------------


class CANListener:
    """Collects RX edges as (tick_ns, level) bursts, one burst per frame."""

    def __init__(self, chip=GPIO_CHIP, rx_pin=RX_PIN, tx_pin=TX_PIN, bitrate=BITRATE):
        self.rx_pin = rx_pin
        self.tx_pin = tx_pin
        self.bitrate = bitrate
        self.bursts = queue.Queue()
        self._edges = []
        self._idle_ns = IDLE_BITS * 1e9 / bitrate
        self._handle = lgpio.gpiochip_open(chip)
        if tx_pin is not None:
            # Keep the transceiver recessive so we never disturb the bus.
            lgpio.gpio_claim_output(self._handle, tx_pin, 1)
        lgpio.gpio_claim_alert(self._handle, rx_pin, lgpio.BOTH_EDGES, lgpio.SET_PULL_UP)
        lgpio.gpio_set_watchdog_micros(self._handle, rx_pin, math.ceil(self._idle_ns / 1000))
        self._cb = lgpio.callback(self._handle, rx_pin, lgpio.BOTH_EDGES, self._on_edge)

    def _close_burst(self):
        if len(self._edges) > 1:
            self.bursts.put(self._edges)
        self._edges = []

    def _on_edge(self, chip, gpio, level, tick):
        if level == lgpio.TIMEOUT:
            self._close_burst()
            return
        if self._edges and tick - self._edges[-1][0] >= self._idle_ns:
            self._close_burst()
        # A frame starts with the falling edge of SOF; ignore stray rising edges.
        if self._edges or level == 0:
            self._edges.append((tick, level))

    def read(self, timeout=None):
        return self.bursts.get(timeout=timeout)

    def close(self):
        self._cb.cancel()
        lgpio.gpio_free(self._handle, self.rx_pin)
        if self.tx_pin is not None:
            lgpio.gpio_free(self._handle, self.tx_pin)
        lgpio.gpiochip_close(self._handle)


# Decoder ----------------------------------------------------------------------


class FrameError(ValueError):
    pass


def edges_to_bits(edges, bitrate):
    """Turn (tick_ns, level) edges into a bit list, resyncing on every edge."""
    bit_ns = 1e9 / bitrate
    bits = []
    for (t0, level), (t1, _) in zip(edges, edges[1:]):
        bits.extend([level] * max(1, round((t1 - t0) / bit_ns)))
    bits.extend([1] * IDLE_BITS)  # the bus is recessive after the last edge
    return bits


def _crc15(bits):
    crc = 0
    for bit in bits:
        feedback = bit ^ (crc >> 14)
        crc = (crc << 1) & 0x7FFF
        if feedback:
            crc ^= CRC15_POLY
    return crc


class _BitReader:
    """Reads bits while removing stuff bits (one inverted bit after five equal)."""

    def __init__(self, bits):
        self._bits = bits
        self._pos = 0
        self._last = None
        self._run = 0
        self.stuffing = True
        self.taken = []  # destuffed bits, for the CRC

    def _raw(self):
        if self._pos >= len(self._bits):
            raise FrameError("truncated frame")
        bit = self._bits[self._pos]
        self._pos += 1
        return bit

    def _skip_stuff_bit(self):
        bit = self._raw()
        if bit == self._last:
            raise FrameError("stuff error (error frame or wrong bitrate)")
        self._last, self._run = bit, 1

    def bit(self):
        if self.stuffing and self._run == 5:
            self._skip_stuff_bit()
        bit = self._raw()
        if bit == self._last:
            self._run += 1
        else:
            self._last, self._run = bit, 1
        self.taken.append(bit)
        return bit

    def read(self, count):
        value = 0
        for _ in range(count):
            value = value << 1 | self.bit()
        return value

    def end_stuffing(self):
        """Stuffing covers the CRC sequence too, including a trailing stuff bit."""
        if self._run == 5:
            self._skip_stuff_bit()
        self.stuffing = False


def decode_frame(bits):
    r = _BitReader(bits)
    if r.bit() != 0:
        raise FrameError("missing start of frame")
    ident = r.read(11)
    rtr = r.bit()  # SRR in an extended frame
    extended = r.bit()
    if extended:
        ident = ident << 18 | r.read(18)
        rtr = r.bit()
        fdf = r.bit()  # r1
        r.bit()  # r0
    else:
        fdf = r.bit()  # r0
    if fdf:
        raise FrameError("reserved bit set (CAN FD frame or wrong bitrate)")
    dlc = r.read(4)
    data = bytes(r.read(8) for _ in range(0 if rtr else min(dlc, 8)))
    expected = _crc15(r.taken)
    crc = r.read(15)
    r.end_stuffing()
    if crc != expected:
        raise FrameError(f"CRC mismatch (got 0x{crc:04X}, expected 0x{expected:04X})")
    if r.bit() != 1:
        raise FrameError("bad CRC delimiter")
    acked = r.bit() == 0
    return {"id": ident, "extended": bool(extended), "rtr": bool(rtr),
            "dlc": dlc, "data": data, "acked": acked}


def decode(edges, bitrate):
    return decode_frame(edges_to_bits(edges, bitrate))


def describe(frame):
    ident = f"{frame['id']:08X}" if frame["extended"] else f"     {frame['id']:03X}"
    payload = "remote request" if frame["rtr"] else frame["data"].hex(" ").upper()
    flags = "" if frame["acked"] else "  (no ACK)"
    return f"{ident}   [{frame['dlc']}]  {payload}{flags}"


# Modes ------------------------------------------------------------------------


def run_live(rx, csv_path):
    print(f"Listening on GPIO{rx.rx_pin} at {rx.bitrate} bit/s, Ctrl+C to stop.")

    writer, csv_file = None, None
    if csv_path:
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        new_file = not csv_path.exists()
        csv_file = csv_path.open("a", newline="")
        writer = csv.writer(csv_file)
        if new_file:
            writer.writerow(["timestamp", "id", "extended", "rtr", "dlc", "data", "acked"])
        print(f"Logging to {csv_path}")

    good = bad = 0
    try:
        while True:
            edges = rx.read()
            stamp = datetime.now()
            clock = stamp.strftime("%H:%M:%S.%f")[:-3]
            try:
                frame = decode(edges, rx.bitrate)
            except FrameError as err:
                bad += 1
                print(f"[{clock}] invalid frame: {err}")
                continue
            good += 1
            print(f"[{clock}] {describe(frame)}")
            if writer:
                writer.writerow([stamp.isoformat(timespec="milliseconds"), f"0x{frame['id']:X}",
                                 int(frame["extended"]), int(frame["rtr"]), frame["dlc"],
                                 frame["data"].hex(), int(frame["acked"])])
                csv_file.flush()
    finally:
        print(f"\n{good} frames decoded, {bad} invalid.")
        if csv_file:
            csv_file.close()


def run_detect(rx, count):
    print(f"Listening on GPIO{rx.rx_pin}, waiting for {count} bursts of traffic, Ctrl+C to stop early.")
    bursts = []
    try:
        while len(bursts) < count:
            bursts.append(rx.read())
            print(f"\r  captured {len(bursts)}/{count}", end="", flush=True)
    except KeyboardInterrupt:
        pass
    print()
    if not bursts:
        print("No traffic seen. Check wiring, termination and that the bus is active.")
        return 1

    results = []
    for bitrate in BITRATES:
        ok = 0
        for edges in bursts:
            try:
                decode(edges, bitrate)
                ok += 1
            except FrameError:
                pass
        results.append((ok, bitrate))
        print(f"  {bitrate:>9} bit/s: {ok}/{len(bursts)} valid")
    ok, bitrate = max(results)
    if ok == 0:
        print("No bitrate decoded any frame; it may be above what the Pi can sample.")
        return 1
    print(f"Best match: {bitrate} bit/s. Run: live --bitrate {bitrate}")


def main():
    parser = argparse.ArgumentParser(description="Adafruit 5708 (CAN Pal, TJA1051T/3) CAN bus listener")
    parser.add_argument("--chip", type=int, default=GPIO_CHIP, help="gpiochip number (default: %(default)s)")
    parser.add_argument("--rx", type=int, default=RX_PIN, help="BCM GPIO wired to module RX (default: %(default)s)")
    parser.add_argument("--tx", type=int, default=TX_PIN,
                        help="BCM GPIO wired to module TX, held recessive; -1 if unconnected (default: %(default)s)")
    sub = parser.add_subparsers(dest="mode", required=True)

    live = sub.add_parser("live", help="print received CAN frames")
    live.add_argument("--bitrate", type=int, default=BITRATE, help="bus bitrate in bit/s (default: %(default)s)")
    live.add_argument("--csv", type=Path, help="also append frames to this CSV file")

    detect = sub.add_parser("detect", help="find the bus bitrate from captured traffic")
    detect.add_argument("--frames", type=int, default=20, help="bursts to capture (default: %(default)s)")

    args = parser.parse_args()
    tx = None if args.tx < 0 else args.tx
    # Detect splits bursts at the slowest bitrate's idle time; decoding only
    # needs the first frame of each burst.
    bitrate = args.bitrate if args.mode == "live" else BITRATES[0]
    rx = CANListener(args.chip, args.rx, tx, bitrate)
    try:
        if args.mode == "live":
            run_live(rx, args.csv)
        else:
            return run_detect(rx, args.frames)
    except KeyboardInterrupt:
        pass
    finally:
        rx.close()


if __name__ == "__main__":
    sys.exit(main())
