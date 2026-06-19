import math
import serial
import struct
import time

import matplotlib.pyplot as plt
import numpy as np


PORT = "/dev/ttyUSB0"
BAUDRATE = 230400

FRAME_HEADER = 0x54
FRAME_VER_LEN = 0x2C
FRAME_LENGTH = 47
POINTS_PER_FRAME = 12

MIN_DISTANCE_M = 0.05
MAX_DISTANCE_M = 12.0

# Limite d'affichage en mètres
DISPLAY_RANGE_M = 4.0

# Nombre minimal de points avant d'afficher une révolution
MIN_POINTS_PER_SCAN = 250


def normalize_angle_delta(start_angle_deg: float, end_angle_deg: float) -> float:
    delta = end_angle_deg - start_angle_deg

    if delta < 0:
        delta += 360.0

    return delta


def parse_frame(frame: bytes):
    if len(frame) != FRAME_LENGTH:
        return None

    if frame[0] != FRAME_HEADER or frame[1] != FRAME_VER_LEN:
        return None

    speed = struct.unpack_from("<H", frame, 2)[0]
    start_angle_deg = struct.unpack_from("<H", frame, 4)[0] / 100.0

    points_raw = []
    offset = 6

    for _ in range(POINTS_PER_FRAME):
        distance_mm = struct.unpack_from("<H", frame, offset)[0]
        confidence = frame[offset + 2]
        offset += 3

        points_raw.append((distance_mm, confidence))

    end_angle_deg = struct.unpack_from("<H", frame, offset)[0] / 100.0
    offset += 2

    timestamp = struct.unpack_from("<H", frame, offset)[0]

    angle_delta = normalize_angle_delta(start_angle_deg, end_angle_deg)
    angle_step = angle_delta / (POINTS_PER_FRAME - 1)

    points = []

    for i, (distance_mm, confidence) in enumerate(points_raw):
        angle_deg = (start_angle_deg + i * angle_step) % 360.0
        distance_m = distance_mm / 1000.0

        if MIN_DISTANCE_M <= distance_m <= MAX_DISTANCE_M:
            points.append(
                {
                    "angle_deg": angle_deg,
                    "distance_m": distance_m,
                    "confidence": confidence,
                }
            )

    return {
        "speed": speed,
        "start_angle_deg": start_angle_deg,
        "end_angle_deg": end_angle_deg,
        "timestamp": timestamp,
        "points": points,
    }


def find_and_parse_frames(buffer: bytearray):
    decoded_frames = []

    while len(buffer) >= FRAME_LENGTH:
        try:
            header_index = buffer.index(FRAME_HEADER)
        except ValueError:
            buffer.clear()
            break

        if header_index > 0:
            del buffer[:header_index]

        if len(buffer) < FRAME_LENGTH:
            break

        if buffer[1] != FRAME_VER_LEN:
            del buffer[0]
            continue

        candidate = bytes(buffer[:FRAME_LENGTH])
        del buffer[:FRAME_LENGTH]

        parsed = parse_frame(candidate)

        if parsed is not None:
            decoded_frames.append(parsed)

    return decoded_frames


def polar_points_to_yz(points):
    y_values = []
    z_values = []

    for point in points:
        angle_rad = math.radians(point["angle_deg"])
        distance = point["distance_m"]

        # Interprétation du scan comme une coupe verticale Y/Z
        y = distance * math.cos(angle_rad)
        z = distance * math.sin(angle_rad)

        y_values.append(y)
        z_values.append(z)

    return np.array(y_values), np.array(z_values)


def setup_plot():
    plt.ion()

    fig, ax = plt.subplots()
    scatter = ax.scatter([], [], s=2)

    ax.set_title("COVENANT Ground Station - Live LiDAR 2D Slice")
    ax.set_xlabel("Y axis [m]")
    ax.set_ylabel("Z axis [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)

    ax.set_xlim(-DISPLAY_RANGE_M, DISPLAY_RANGE_M)
    ax.set_ylim(-DISPLAY_RANGE_M, DISPLAY_RANGE_M)

    return fig, ax, scatter


def update_plot(fig, scatter, y_values, z_values):
    if len(y_values) == 0:
        return

    points_2d = np.column_stack((y_values, z_values))
    scatter.set_offsets(points_2d)

    fig.canvas.draw()
    fig.canvas.flush_events()


def main():
    print("COVENANT Ground Station - Live 2D LiDAR Slice")
    print(f"Port: {PORT}")
    print(f"Baudrate: {BAUDRATE}")
    print("Press Ctrl+C to stop.")
    print()

    buffer = bytearray()

    current_scan_points = []
    previous_start_angle = None
    scan_count = 0
    frame_count = 0

    fig, ax, scatter = setup_plot()

    try:
        with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
            print("Serial port opened successfully.")
            print("Displaying live scan...")
            print()

            while True:
                data = ser.read(512)

                if not data:
                    continue

                buffer.extend(data)
                frames = find_and_parse_frames(buffer)

                for frame in frames:
                    frame_count += 1
                    start_angle = frame["start_angle_deg"]

                    # Détection simple d'une nouvelle révolution :
                    # quand l'angle repasse de ~359° à ~0°
                    if (
                        previous_start_angle is not None
                        and start_angle < previous_start_angle
                        and len(current_scan_points) >= MIN_POINTS_PER_SCAN
                    ):
                        scan_count += 1

                        y_values, z_values = polar_points_to_yz(current_scan_points)
                        update_plot(fig, scatter, y_values, z_values)

                        print(
                            f"scan={scan_count:5d} | "
                            f"frames={frame_count:6d} | "
                            f"points={len(current_scan_points):4d}"
                        )

                        current_scan_points = []

                    current_scan_points.extend(frame["points"])
                    previous_start_angle = start_angle

    except KeyboardInterrupt:
        print()
        print("Stopped by user.")

    except serial.SerialException as error:
        print("Serial error:")
        print(error)


if __name__ == "__main__":
    main()
