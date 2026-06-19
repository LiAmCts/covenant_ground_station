import math
import serial
import struct
import time


PORT = "/dev/ttyUSB0"
BAUDRATE = 230400

FRAME_HEADER = 0x54
FRAME_VER_LEN = 0x2C
FRAME_LENGTH = 47
POINTS_PER_FRAME = 12

DURATION_SECONDS = 10


def normalize_angle_delta(start_angle_deg: float, end_angle_deg: float) -> float:
    """
    Calcule l'écart angulaire entre deux angles, en gérant le passage 359° -> 0°.
    """
    delta = end_angle_deg - start_angle_deg

    if delta < 0:
        delta += 360.0

    return delta


def parse_frame(frame: bytes):
    """
    Décode une trame LDROBOT/LD19/STL typique de 47 octets.

    Structure attendue :
    - header        : 1 byte  = 0x54
    - ver_len       : 1 byte  = 0x2C
    - speed         : 2 bytes
    - start_angle   : 2 bytes, centi-degrés
    - 12 points     : distance uint16 mm + confidence uint8
    - end_angle     : 2 bytes, centi-degrés
    - timestamp     : 2 bytes
    - crc           : 1 byte
    """

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
    """
    Cherche les trames valides dans un buffer brut.
    Retourne les frames décodées.
    """

    decoded_frames = []

    while len(buffer) >= FRAME_LENGTH:
        # Cherche le header 0x54
        try:
            header_index = buffer.index(FRAME_HEADER)
        except ValueError:
            buffer.clear()
            break

        # Supprime ce qui est avant le header
        if header_index > 0:
            del buffer[:header_index]

        # Pas encore assez de données pour une frame complète
        if len(buffer) < FRAME_LENGTH:
            break

        # Vérifie le deuxième octet 0x2C
        if buffer[1] != FRAME_VER_LEN:
            del buffer[0]
            continue

        candidate = bytes(buffer[:FRAME_LENGTH])
        del buffer[:FRAME_LENGTH]

        parsed = parse_frame(candidate)

        if parsed is not None:
            decoded_frames.append(parsed)

    return decoded_frames


def main():
    print("COVENANT Ground Station - LiDAR Frame Decoder")
    print(f"Port: {PORT}")
    print(f"Baudrate: {BAUDRATE}")
    print(f"Duration: {DURATION_SECONDS} seconds")
    print()

    buffer = bytearray()
    total_frames = 0
    total_points = 0
    start_time = time.time()

    try:
        with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
            print("Serial port opened successfully.")
            print("Decoding LiDAR frames...")
            print()

            while time.time() - start_time < DURATION_SECONDS:
                data = ser.read(512)

                if not data:
                    continue

                buffer.extend(data)
                frames = find_and_parse_frames(buffer)

                for frame in frames:
                    total_frames += 1

                    valid_distances = [
                        p["distance_m"]
                        for p in frame["points"]
                        if 0.03 <= p["distance_m"] <= 12.0
                    ]

                    total_points += len(valid_distances)

                    if total_frames % 20 == 0 and valid_distances:
                        print(
                            f"frames={total_frames:5d} | "
                            f"angle={frame['start_angle_deg']:7.2f}° -> {frame['end_angle_deg']:7.2f}° | "
                            f"min={min(valid_distances):5.2f} m | "
                            f"max={max(valid_distances):5.2f} m | "
                            f"points={len(valid_distances):2d}"
                        )

    except serial.SerialException as error:
        print("Serial error:")
        print(error)
        return

    print()
    print(f"Total decoded frames: {total_frames}")
    print(f"Total valid points: {total_points}")

    if total_frames == 0:
        print("Result: no valid frame decoded.")
        print("Raw serial works, but the frame format may differ.")
    else:
        print("Result: LiDAR frames decoded successfully.")


if __name__ == "__main__":
    main()
