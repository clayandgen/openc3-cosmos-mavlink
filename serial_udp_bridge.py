#!/usr/bin/env python3
"""
Serial-to-UDP bridge for MAVLink.

Bridges a MAVLink serial connection (e.g. flight controller USB) to UDP so
that COSMOS can communicate with a real drone using the existing UDP-based
MAVLink plugin.

Data flow:
    Flight Controller <--USB Serial--> Bridge <--UDP--> COSMOS

    - Telemetry:  FC -> Serial -> Bridge -> UDP 127.0.0.1:14552 -> COSMOS
    - Commands:   COSMOS -> UDP 0.0.0.0:14560 -> Bridge -> Serial -> FC

Prerequisites:
    pip install pyserial

Usage:
    # Use defaults (auto-detect USB device at 115200 baud):
    python3 serial_udp_bridge.py

    # Specify serial port and baud rate:
    python3 serial_udp_bridge.py /dev/tty.usbmodem2101 115200

    # On Linux the device is typically /dev/ttyACM0 or /dev/ttyUSB0

Finding your serial port:
    macOS:  ls /dev/tty.usb*
    Linux:  ls /dev/ttyACM* /dev/ttyUSB*

Stop the bridge with Ctrl+C.
"""
import glob
import serial
import socket
import threading
import sys
import time

SERIAL_BAUD = 115200

# COSMOS reads from this port (we send telemetry here)
UDP_SEND_HOST = "127.0.0.1"
UDP_SEND_PORT = 14552

# COSMOS writes to this port (we receive commands here)
UDP_RECV_PORT = 14560


def find_serial_port():
    """Auto-detect a USB serial device."""
    patterns = ["/dev/tty.usbmodem*", "/dev/ttyACM*", "/dev/ttyUSB*"]
    for pattern in patterns:
        matches = sorted(glob.glob(pattern))
        if matches:
            return matches[0]
    return None


def serial_to_udp(ser, sock):
    """Read MAVLink packets from serial and forward to COSMOS via UDP."""
    count = 0
    while True:
        try:
            data = ser.read(256)
            if data:
                sock.sendto(data, (UDP_SEND_HOST, UDP_SEND_PORT))
                count += 1
                if count <= 5 or count % 100 == 0:
                    print(f"  [Serial->UDP] {len(data)} bytes (packet #{count})")
        except Exception as e:
            print(f"Serial read error: {e}")
            break


def udp_to_serial(ser, sock):
    """Read commands from COSMOS via UDP and forward to serial."""
    count = 0
    while True:
        try:
            data, addr = sock.recvfrom(4096)
            if data:
                ser.write(data)
                count += 1
                if count <= 5 or count % 100 == 0:
                    print(f"  [UDP->Serial] {len(data)} bytes from {addr} (packet #{count})")
        except Exception as e:
            print(f"UDP read error: {e}")
            break


def main():
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = find_serial_port()
        if port is None:
            print("ERROR: No USB serial device found.")
            print("  macOS:  ls /dev/tty.usb*")
            print("  Linux:  ls /dev/ttyACM* /dev/ttyUSB*")
            sys.exit(1)
        print(f"Auto-detected serial port: {port}")

    baud = int(sys.argv[2]) if len(sys.argv) > 2 else SERIAL_BAUD

    print(f"Opening serial: {port} @ {baud}")
    ser = serial.Serial(port, baud, timeout=0.1)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", UDP_RECV_PORT))

    print(f"Bridge running:")
    print(f"  Telemetry: Serial {port} -> UDP {UDP_SEND_HOST}:{UDP_SEND_PORT}")
    print(f"  Commands:  UDP 0.0.0.0:{UDP_RECV_PORT} -> Serial {port}")
    print(f"Press Ctrl+C to stop.")

    t1 = threading.Thread(target=serial_to_udp, args=(ser, sock), daemon=True)
    t2 = threading.Thread(target=udp_to_serial, args=(ser, sock), daemon=True)
    t1.start()
    t2.start()

    try:
        t1.join()
    except KeyboardInterrupt:
        print("\nStopping bridge.")
    finally:
        ser.close()
        sock.close()


if __name__ == "__main__":
    main()
