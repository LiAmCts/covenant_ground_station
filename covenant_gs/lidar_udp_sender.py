import socket
import struct
import time

import serial


SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 230400

# IMPORTANT:
# Mettre ici l'IP du PC Ground Station, pas l'IP de la Raspberry.
TARGET_IP = "192.168.1.103"
TARGET_PORT = 56000

READ_SIZE_BYTES = 512

PACKET_MAGIC = b"CVL1"
PROTOCOL_VERSION = 1
PACKET_TYPE_RAW_LIDAR = 1

# Format:
# magic        4 bytes  CVL1
# version      1 byte
# packet_type  1 byte
# sequence     4 bytes
# timestamp    8 bytes, monotonic_ns
# payload_len  2 bytes
HEADER_STRUCT = struct.Struct("!4sBBIQH")


def build_packet(sequence: int, payload: bytes) -> bytes:
    timestamp_ns = time.monotonic_ns()

    header = HEADER_STRUCT.pack(
        PACKET_MAGIC,
        PROTOCOL_VERSION,
        PACKET_TYPE_RAW_LIDAR,
        sequence,
        timestamp_ns,
        len(payload),
    )

    return header + payload


def main():
    target = (TARGET_IP, TARGET_PORT)

    print("COVENANT LiDAR UDP Sender")
    print(f"Serial port: {SERIAL_PORT}")
    print(f"Baudrate: {BAUDRATE}")
    print(f"Target: {TARGET_IP}:{TARGET_PORT}/UDP")
    print(f"Read size: {READ_SIZE_BYTES} bytes")
    print("Press Ctrl+C to stop.")
    print()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1_000_000)

    sequence = 0
    packets_sent = 0
    bytes_sent = 0

    last_status_time = time.time()
    last_status_packets = 0
    last_status_bytes = 0

    try:
        with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.05) as ser:
            print("LiDAR serial port opened.")
            print("Streaming raw LiDAR data over UDP...")
            print()

            while True:
                payload = ser.read(READ_SIZE_BYTES)

                if not payload:
                    continue

                packet = build_packet(sequence, payload)
                sock.sendto(packet, target)

                sequence = (sequence + 1) & 0xFFFFFFFF
                packets_sent += 1
                bytes_sent += len(payload)

                now = time.time()

                if now - last_status_time >= 1.0:
                    delta_time = now - last_status_time
                    delta_packets = packets_sent - last_status_packets
                    delta_bytes = bytes_sent - last_status_bytes

                    rate_kb_s = (delta_bytes / 1024.0) / delta_time
                    pps = delta_packets / delta_time

                    print(
                        f"packets={packets_sent:8d} | "
                        f"bytes={bytes_sent:10d} | "
                        f"rate={rate_kb_s:6.2f} KB/s | "
                        f"pps={pps:6.1f}"
                    )

                    last_status_time = now
                    last_status_packets = packets_sent
                    last_status_bytes = bytes_sent

    except KeyboardInterrupt:
        print()
        print("Stopped by user.")

    except serial.SerialException as error:
        print()
        print("Serial error:")
        print(error)

    finally:
        sock.close()
        print("UDP socket closed.")


if __name__ == "__main__":
    main()