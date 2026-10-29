# Digital IR Receiver Module
#
# Hardware---------------------------------------------------------------------
# Model: DFR0094
# Power Supply: 5V
# IR RX Frequency: 38KHz
# Reference: https://wiki.dfrobot.com/dfr0094/
#
# Connections------------------------------------------------------------------
# Raspberry Pi GPIO Pinout:
# Standard 40-pin header; GPIO numbers use BCM numbering.
#
# Module Pin    Pi Physical Pin    Function
# VCC / +       2                  5V power
# GND / -       6                  Ground
# Signal        11                 GPIO17
# -----------------------------------------------------------------------------
#
# Usage------------------------------------------------------------------------
#   uv run ir-rx/df_robot_dfr0094.py live [--raw]
#       Print every received IR frame (decoded protocol + optional raw timings).
#
#   uv run ir-rx/df_robot_dfr0094.py capture --device "Living room TV"
#       Interactive: press a remote button, type the key name, repeat.
#       Keys are saved to ir-rx/captures/<device>.json (appended if it exists).
#
# Capture file format (consumed by RP2350-POE-ETH production firmware):
#   {
#     "device": "Living room TV",
#     "receiver": "DFR0094", "carrier_hz": 38000,
#     "keys": {
#       "power": {
#         "protocol": "NEC", "address": 4, "command": 8, "code": "0xF708FB04",
#         "bits": 32, "raw_us": [9012, 4475, 580, 545, ...]
#       }
#     }
#   }
#   raw_us alternates mark (carrier on) / space (carrier off), starting with a
#   mark, in microseconds. "code" is the 32-bit frame as sent, LSB first.
# -----------------------------------------------------------------------------

import argparse
import json
import queue
import re
import sys
from datetime import datetime, timezone
from pathlib import Path

import lgpio

GPIO_CHIP = 0
GPIO_PIN = 17
FRAME_GAP_US = 15_000  # silence that ends a frame
GLITCH_US = 80  # ignore pulses shorter than this
TOLERANCE = 0.3  # +/- 30% timing tolerance for decoding
CAPTURES_DIR = Path(__file__).parent / "captures"


# Receiver ---------------------------------------------------------------------


class IRReceiver:
    """Collects mark/space durations from the active-low demodulated output."""

    def __init__(self, chip=GPIO_CHIP, pin=GPIO_PIN):
        self.pin = pin
        self.frames = queue.Queue()
        self._pulses = []
        self._last_tick = None
        self._merge_next = False
        self._handle = lgpio.gpiochip_open(chip)
        lgpio.gpio_claim_alert(self._handle, pin, lgpio.BOTH_EDGES, lgpio.SET_PULL_UP)
        lgpio.gpio_set_watchdog_micros(self._handle, pin, FRAME_GAP_US)
        self._cb = lgpio.callback(self._handle, pin, lgpio.BOTH_EDGES, self._on_edge)

    def _on_edge(self, chip, gpio, level, tick):
        if level == lgpio.TIMEOUT:
            if len(self._pulses) > 1:
                self.frames.put(self._pulses)
            self._pulses = []
            self._last_tick = None
            self._merge_next = False
            return

        if self._last_tick is None:
            # First falling edge (carrier start) opens a frame.
            if level == 0:
                self._last_tick = tick
            return

        duration = (tick - self._last_tick) // 1000  # ns -> us
        self._last_tick = tick
        if self._merge_next:
            self._pulses[-1] += duration
            self._merge_next = False
            return
        if duration < GLITCH_US and self._pulses:
            # A spike splits one pulse in two: fold it and the next part back.
            self._pulses[-1] += duration
            self._merge_next = True
            return
        # Rising edge ends a mark (low), falling edge ends a space (high).
        self._pulses.append(duration)

    def flush(self):
        while not self.frames.empty():
            self.frames.get_nowait()

    def read(self, timeout=None):
        return self.frames.get(timeout=timeout)

    def close(self):
        self._cb.cancel()
        lgpio.gpio_free(self._handle, self.pin)
        lgpio.gpiochip_close(self._handle)


# Decoders ---------------------------------------------------------------------


def _near(value, target):
    return abs(value - target) <= target * TOLERANCE


def _pulse_distance_bits(pulses, start, count, mark, zero, one):
    """Decode mark+space pairs where the space length carries the bit, LSB first."""
    value = 0
    for i in range(count):
        m, s = pulses[start + 2 * i], pulses[start + 2 * i + 1]
        if not _near(m, mark):
            return None
        if _near(s, one):
            value |= 1 << i
        elif not _near(s, zero):
            return None
    return value


def decode_nec(pulses):
    if len(pulses) < 2 or not _near(pulses[0], 9000):
        return None
    if _near(pulses[1], 2250) and len(pulses) <= 4:
        return {"protocol": "NEC", "repeat": True}
    if not _near(pulses[1], 4500) or len(pulses) < 2 + 64:
        return None
    code = _pulse_distance_bits(pulses, 2, 32, 560, 560, 1690)
    if code is None:
        return None
    addr, addr_inv = code & 0xFF, (code >> 8) & 0xFF
    cmd, cmd_inv = (code >> 16) & 0xFF, (code >> 24) & 0xFF
    if cmd ^ cmd_inv != 0xFF:
        return None
    protocol = "NEC"
    if addr ^ addr_inv != 0xFF:
        protocol, addr = "NEC-EXT", code & 0xFFFF
    return {"protocol": protocol, "address": addr, "command": cmd,
            "code": f"0x{code:08X}", "bits": 32}


def decode_samsung(pulses):
    if len(pulses) < 2 + 64 or not (_near(pulses[0], 4500) and _near(pulses[1], 4500)):
        return None
    code = _pulse_distance_bits(pulses, 2, 32, 560, 560, 1690)
    if code is None:
        return None
    return {"protocol": "SAMSUNG", "address": code & 0xFFFF,
            "command": (code >> 16) & 0xFF, "code": f"0x{code:08X}", "bits": 32}


def decode_sony(pulses):
    if len(pulses) < 2 or not _near(pulses[0], 2400):
        return None
    # Bits: space 600 then mark 600 (0) or 1200 (1); final space is absent.
    marks = pulses[2::2]
    spaces = pulses[1::2]
    if len(marks) not in (12, 15, 20) or not all(_near(s, 600) for s in spaces):
        return None
    code = 0
    for i, m in enumerate(marks):
        if _near(m, 1200):
            code |= 1 << i
        elif not _near(m, 600):
            return None
    bits = len(marks)
    return {"protocol": f"SONY{bits}", "address": code >> 7,
            "command": code & 0x7F, "code": f"0x{code:0{(bits + 3) // 4}X}",
            "bits": bits}


DECODERS = (decode_nec, decode_samsung, decode_sony)


def decode(pulses):
    for decoder in DECODERS:
        result = decoder(pulses)
        if result:
            return result
    return {"protocol": "UNKNOWN"}


def describe(info, pulses):
    if info.get("repeat"):
        return f"{info['protocol']:<8} (repeat)"
    if info["protocol"] == "UNKNOWN":
        return f"UNKNOWN  {len(pulses)} pulses, lead {pulses[0]}/{pulses[1] if len(pulses) > 1 else '-'} us"
    return (f"{info['protocol']:<8} addr=0x{info['address']:04X} "
            f"cmd=0x{info['command']:02X} code={info['code']}")


# Modes ------------------------------------------------------------------------


def run_live(rx, show_raw):
    print(f"Listening on GPIO{rx.pin}. Press remote buttons, Ctrl+C to stop.")
    while True:
        pulses = rx.read()
        info = decode(pulses)
        stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        print(f"[{stamp}] {describe(info, pulses)}")
        if show_raw and not info.get("repeat"):
            print(f"           raw_us={pulses}")


def _slug(name):
    return re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_") or "device"


def _load(path, device):
    if path.exists():
        data = json.loads(path.read_text())
        print(f"Appending to {path} ({len(data.get('keys', {}))} keys already stored)")
        return data
    return {"device": device, "receiver": "DFR0094", "carrier_hz": 38000,
            "created": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "keys": {}}


def _save(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    data["updated"] = datetime.now(timezone.utc).isoformat(timespec="seconds")
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(data, indent=2) + "\n")
    tmp.replace(path)


def _wait_for_press(rx):
    """Return the first non-repeat frame."""
    rx.flush()
    while True:
        pulses = rx.read()
        info = decode(pulses)
        if not info.get("repeat"):
            return pulses, info


def run_capture(rx, device, out_path):
    data = _load(out_path, device)
    keys = data["keys"]
    print(f"Capturing device '{data['device']}' -> {out_path}")
    print("Press a remote button. Then type the key name "
          "(empty = discard and retry, 'q' = save and quit).")
    while True:
        print("\nWaiting for button...", flush=True)
        pulses, info = _wait_for_press(rx)
        print(f"  got: {describe(info, pulses)}")
        for name, entry in keys.items():
            if entry.get("code") and entry.get("code") == info.get("code"):
                print(f"  note: same code already stored as '{name}'")
        name = input("  key name: ").strip()
        if name.lower() == "q":
            break
        if not name:
            print("  discarded")
            continue
        if name in keys:
            if input(f"  '{name}' exists, overwrite? [y/N]: ").strip().lower() != "y":
                continue
        keys[name] = {**info, "raw_us": pulses}
        _save(out_path, data)
        print(f"  saved '{name}' ({len(keys)} keys total)")
    _save(out_path, data)
    print(f"Saved {len(keys)} keys to {out_path}")


def main():
    parser = argparse.ArgumentParser(description="DFR0094 IR receiver: live view and key capture")
    parser.add_argument("--chip", type=int, default=GPIO_CHIP, help="gpiochip number (default: %(default)s)")
    parser.add_argument("--pin", type=int, default=GPIO_PIN, help="BCM GPIO pin (default: %(default)s)")
    sub = parser.add_subparsers(dest="mode", required=True)

    live = sub.add_parser("live", help="print received IR frames")
    live.add_argument("--raw", action="store_true", help="also print raw mark/space timings")

    cap = sub.add_parser("capture", help="record remote keys to a JSON file")
    cap.add_argument("--device", help="remote/device name (prompted if omitted)")
    cap.add_argument("--out", type=Path, help="output file (default: captures/<device>.json)")

    args = parser.parse_args()
    rx = IRReceiver(args.chip, args.pin)
    try:
        if args.mode == "live":
            run_live(rx, args.raw)
        else:
            device = args.device or input("Device name: ").strip() or "device"
            out = args.out or CAPTURES_DIR / f"{_slug(device)}.json"
            run_capture(rx, device, out)
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        rx.close()


if __name__ == "__main__":
    sys.exit(main())
