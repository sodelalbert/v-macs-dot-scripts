# Waveshare 27876 Modbus POE ETH Relay (B)


# Device is comming with default IP address 192.168.1.200
# Add ptemporary address to your network interface to be able to access the device.
# On Mac: sudo ifconfig en0 alias 192.168.1.250 netmask 255.255.255.0
# Login to  http://192.168.1.200/ password: 123456
# Change IP address resolution to DHCP.

# References: https://www.waveshare.com/wiki/Modbus_POE_ETH_Relay_(B)?srsltid=AYCX9vKLJ_x7EHb22dVoK0zAQy5Kdi5DUDy5MyWZesXv4OCDUpjiJdhE


# http://192.168.50.116/ip_en.html

import socket
import time

HOST, PORT = "192.168.50.116", 4196
GREEN, RESET = "\033[1;92m", "\033[0m"


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc.to_bytes(2, "little")


def transact(pdu):
    with socket.create_connection((HOST, PORT), timeout=3) as s:
        s.sendall(pdu + crc16(pdu))
        resp = s.recv(256)
    if len(resp) < 5 or crc16(resp[:-2]) != resp[-2:]:
        raise IOError(f"Bad response: {resp.hex(' ')}")
    if resp[1] & 0x80:
        raise IOError(f"Modbus exception {resp[2]:#x}")
    return resp


def get_relays():
    mask = transact(bytes.fromhex("01 01 00 00 00 08"))[3]
    return {i + 1: bool(mask >> i & 1) for i in range(8)}


def set_relay(n, on):  # n = 1..8
    pdu = bytes([0x01, 0x05, 0x00, n - 1, 0xFF if on else 0x00, 0x00])
    resp = transact(pdu)
    if resp[:-2] != pdu:
        raise IOError(f"Unexpected echo: {resp.hex(' ')}")


def set_all(on):
    pdu = bytes([0x01, 0x05, 0x00, 0xFF, 0xFF if on else 0x00, 0x00])
    resp = transact(pdu)
    if resp[:-2] != pdu:
        raise IOError(f"Unexpected echo: {resp.hex(' ')}")


def print_relays():
    states = (
        f"{GREEN}{n}:ON{RESET}" if on else f"{n}:off"
        for n, on in get_relays().items()
    )
    print("Relays:", " ".join(states))


if __name__ == "__main__":
    set_all(False)
    print_relays()

    for n in range(1, 9):
        try:
            set_relay(n, True)
            print_relays()
            time.sleep(0.03)
        finally:
            set_relay(n, False)
    print_relays()

    set_all(True)
    print_relays()
    set_all(False)
    print_relays()