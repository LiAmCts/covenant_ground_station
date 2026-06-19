import math
import socket
import struct
import sys
import time

import numpy as np

from PySide6.QtCore import QObject, QThread, QTimer, Signal, Slot
from PySide6.QtWidgets import (
    QApplication,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QVBoxLayout,
    QWidget,
)

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


# ============================================================
# Network configuration
# ============================================================

PC_IP_DISPLAY = "192.168.1.103"

# 0.0.0.0 = listen on all network interfaces of the PC.
UDP_LISTEN_IP = "0.0.0.0"
UDP_LISTEN_PORT = 56000

UDP_RECV_SIZE_BYTES = 2048

# ============================================================
# UDP packet protocol from Raspberry Pi
# ============================================================

PACKET_MAGIC = b"CVL1"
PROTOCOL_VERSION = 1
PACKET_TYPE_RAW_LIDAR = 1

# Format:
# magic        4 bytes  CVL1
# version      1 byte
# packet_type  1 byte
# sequence     4 bytes
# timestamp    8 bytes, monotonic_ns from Raspberry
# payload_len  2 bytes
UDP_HEADER_STRUCT = struct.Struct("!4sBBIQH")
UDP_HEADER_SIZE = UDP_HEADER_STRUCT.size

# ============================================================
# LiDAR frame configuration
# ============================================================

FRAME_HEADER = 0x54
FRAME_VER_LEN = 0x2C
FRAME_LENGTH = 47
POINTS_PER_FRAME = 12

MIN_DISTANCE_M = 0.05
MAX_DISTANCE_M = 12.0

# ============================================================
# Mapping / display configuration
# ============================================================

SIMULATED_SPEED_X_M_S = 0.25

DISPLAY_UPDATE_MS = 400

Y_RANGE_M = 8.0
Z_RANGE_M = 5.0

GRID_SPACING_M = 5.0

# Points are kept in memory.
# Display is downsampled only if too many points are drawn.
MAX_DISPLAY_POINTS = 250_000


def normalize_angle_delta(start_angle_deg: float, end_angle_deg: float) -> float:
    delta = end_angle_deg - start_angle_deg

    if delta < 0:
        delta += 360.0

    return delta


def parse_lidar_frame(frame: bytes):
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
        "points": points,
    }


def find_and_parse_lidar_frames(buffer: bytearray):
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

        parsed = parse_lidar_frame(candidate)

        if parsed is not None:
            decoded_frames.append(parsed)

    return decoded_frames


def parse_udp_packet(packet: bytes):
    if len(packet) < UDP_HEADER_SIZE:
        return None

    try:
        (
            magic,
            version,
            packet_type,
            sequence,
            timestamp_ns,
            payload_len,
        ) = UDP_HEADER_STRUCT.unpack_from(packet, 0)
    except struct.error:
        return None

    if magic != PACKET_MAGIC:
        return None

    if version != PROTOCOL_VERSION:
        return None

    if packet_type != PACKET_TYPE_RAW_LIDAR:
        return None

    payload_start = UDP_HEADER_SIZE
    payload_end = payload_start + payload_len

    if len(packet) < payload_end:
        return None

    payload = packet[payload_start:payload_end]

    return {
        "sequence": sequence,
        "timestamp_ns": timestamp_ns,
        "payload": payload,
    }


def scan_points_to_xyz(scan_points, x_position_m: float):
    xyz_points = []

    for point in scan_points:
        angle_rad = math.radians(point["angle_deg"])
        distance = point["distance_m"]

        x = x_position_m
        y = distance * math.cos(angle_rad)
        z = distance * math.sin(angle_rad)

        xyz_points.append([x, y, z])

    return np.array(xyz_points, dtype=np.float64)


class UdpLidarWorker(QObject):
    scan_ready = Signal(object, dict)
    status_changed = Signal(str)
    error_occurred = Signal(str)
    finished = Signal()

    def __init__(self):
        super().__init__()
        self._running = False

    @Slot()
    def run(self):
        self._running = True

        lidar_buffer = bytearray()
        current_scan_points = []
        previous_start_angle = None

        scan_count = 0
        frame_count = 0
        total_received_points = 0

        udp_packets_received = 0
        udp_bytes_received = 0
        udp_packets_lost = 0
        last_sequence = None

        start_time = time.time()

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        try:
            sock.bind((UDP_LISTEN_IP, UDP_LISTEN_PORT))
            sock.settimeout(0.2)

            self.status_changed.emit("UDP ONLINE")

            while self._running:
                try:
                    packet, sender_address = sock.recvfrom(UDP_RECV_SIZE_BYTES)
                except socket.timeout:
                    continue

                parsed_packet = parse_udp_packet(packet)

                if parsed_packet is None:
                    continue

                sequence = parsed_packet["sequence"]
                payload = parsed_packet["payload"]

                udp_packets_received += 1
                udp_bytes_received += len(payload)

                if last_sequence is not None:
                    expected_sequence = (last_sequence + 1) & 0xFFFFFFFF

                    if sequence != expected_sequence:
                        if sequence > expected_sequence:
                            udp_packets_lost += sequence - expected_sequence
                        else:
                            udp_packets_lost += 1

                last_sequence = sequence

                lidar_buffer.extend(payload)
                frames = find_and_parse_lidar_frames(lidar_buffer)

                for frame in frames:
                    frame_count += 1
                    start_angle = frame["start_angle_deg"]

                    new_scan_detected = (
                        previous_start_angle is not None
                        and start_angle < previous_start_angle
                        and len(current_scan_points) > 100
                    )

                    if new_scan_detected:
                        scan_count += 1

                        elapsed_time_s = time.time() - start_time
                        x_position_m = elapsed_time_s * SIMULATED_SPEED_X_M_S

                        xyz_points = scan_points_to_xyz(
                            current_scan_points,
                            x_position_m,
                        )

                        total_received_points += len(xyz_points)

                        stats = {
                            "scan_count": scan_count,
                            "frame_count": frame_count,
                            "last_scan_points": len(xyz_points),
                            "total_received_points": total_received_points,
                            "x_position_m": x_position_m,
                            "speed_x_m_s": SIMULATED_SPEED_X_M_S,
                            "elapsed_time_s": elapsed_time_s,
                            "udp_packets_received": udp_packets_received,
                            "udp_bytes_received": udp_bytes_received,
                            "udp_packets_lost": udp_packets_lost,
                            "last_sender": f"{sender_address[0]}:{sender_address[1]}",
                        }

                        self.scan_ready.emit(xyz_points, stats)
                        current_scan_points = []

                    current_scan_points.extend(frame["points"])
                    previous_start_angle = start_angle

        except OSError as error:
            self.error_occurred.emit(str(error))

        finally:
            sock.close()
            self.status_changed.emit("OFFLINE")
            self.finished.emit()

    def stop(self):
        self._running = False


class CovenantGroundStationUdpV3(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("COVENANT Ground Station - UDP LiDAR Mapping")
        self.resize(1500, 850)

        self.thread = None
        self.worker = None

        self.map_points = np.empty((0, 3), dtype=np.float64)
        self.trajectory_points = np.empty((0, 3), dtype=np.float64)

        self.latest_stats = {
            "scan_count": 0,
            "frame_count": 0,
            "last_scan_points": 0,
            "total_received_points": 0,
            "x_position_m": 0.0,
            "speed_x_m_s": SIMULATED_SPEED_X_M_S,
            "elapsed_time_s": 0.0,
            "udp_packets_received": 0,
            "udp_bytes_received": 0,
            "udp_packets_lost": 0,
            "last_sender": "-",
        }

        self._build_ui()

        self.display_timer = QTimer(self)
        self.display_timer.timeout.connect(self.update_visual)
        self.display_timer.start(DISPLAY_UPDATE_MS)

        QTimer.singleShot(400, self.start_udp_receiver)

    def _build_ui(self):
        self.setStyleSheet(
            """
            QMainWindow {
                background-color: #030508;
            }

            QWidget {
                background-color: #030508;
                color: #EAF2FF;
                font-family: Arial;
            }

            QLabel {
                color: #EAF2FF;
            }

            QFrame#SidePanel {
                background-color: #070C12;
                border: 1px solid #172334;
                border-radius: 16px;
            }

            QLabel#Title {
                font-size: 28px;
                font-weight: 900;
                letter-spacing: 2px;
                color: #FFFFFF;
            }

            QLabel#Subtitle {
                font-size: 12px;
                color: #7F95B2;
                letter-spacing: 1px;
            }

            QLabel#SectionTitle {
                font-size: 11px;
                color: #5F748F;
                font-weight: 800;
                letter-spacing: 1px;
            }

            QLabel#StatName {
                font-size: 11px;
                color: #7186A1;
            }

            QLabel#StatValue {
                font-size: 18px;
                font-weight: 800;
                color: #FFFFFF;
            }

            QLabel#Online {
                font-size: 19px;
                font-weight: 900;
                color: #52F0A2;
            }

            QLabel#Offline {
                font-size: 19px;
                font-weight: 900;
                color: #FF405D;
            }
            """
        )

        root = QWidget()
        layout = QHBoxLayout(root)
        layout.setContentsMargins(18, 18, 18, 18)
        layout.setSpacing(18)

        plot_panel = self._build_plot_panel()
        side_panel = self._build_side_panel()

        layout.addWidget(plot_panel, 5)
        layout.addWidget(side_panel, 1)

        self.setCentralWidget(root)

    def _build_plot_panel(self):
        panel = QFrame()
        panel.setStyleSheet(
            """
            QFrame {
                background-color: #030508;
                border: 1px solid #101824;
                border-radius: 18px;
            }
            """
        )

        layout = QVBoxLayout(panel)
        layout.setContentsMargins(8, 8, 8, 8)

        self.figure = Figure(facecolor="#030508")
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setStyleSheet("background-color: #030508;")

        self.ax = self.figure.add_subplot(111, projection="3d")
        self.ax.set_facecolor("#030508")

        layout.addWidget(self.canvas)

        return panel

    def _build_side_panel(self):
        panel = QFrame()
        panel.setObjectName("SidePanel")

        layout = QVBoxLayout(panel)
        layout.setContentsMargins(24, 26, 24, 26)
        layout.setSpacing(14)

        title = QLabel("COVENANT")
        title.setObjectName("Title")
        layout.addWidget(title)

        subtitle = QLabel("GROUND STATION / UDP LIDAR MAPPING")
        subtitle.setObjectName("Subtitle")
        layout.addWidget(subtitle)

        layout.addSpacing(24)

        layout.addWidget(self._section("CONNECTION"))

        self.status_value = QLabel("IDLE")
        self.status_value.setObjectName("Offline")
        layout.addWidget(self.status_value)

        self.pc_ip_value = self._stat_value(PC_IP_DISPLAY)
        self.udp_port_value = self._stat_value(str(UDP_LISTEN_PORT))
        self.sender_value = self._stat_value("-")

        layout.addLayout(self._stat_row("PC IP", self.pc_ip_value))
        layout.addLayout(self._stat_row("UDP PORT", self.udp_port_value))
        layout.addLayout(self._stat_row("SENDER", self.sender_value))

        layout.addSpacing(22)

        layout.addWidget(self._section("MOTION"))

        self.distance_value = self._stat_value("0.00 m")
        self.speed_value = self._stat_value(f"{SIMULATED_SPEED_X_M_S:.2f} m/s")

        layout.addLayout(self._stat_row("TOTAL DISTANCE", self.distance_value))
        layout.addLayout(self._stat_row("SIM SPEED", self.speed_value))

        layout.addSpacing(22)

        layout.addWidget(self._section("LIDAR STREAM"))

        self.points_value = self._stat_value("0")
        self.displayed_points_value = self._stat_value("0")
        self.scan_value = self._stat_value("0")
        self.udp_packets_value = self._stat_value("0")
        self.udp_lost_value = self._stat_value("0")

        layout.addLayout(self._stat_row("POINTS RECEIVED", self.points_value))
        layout.addLayout(self._stat_row("POINTS DISPLAYED", self.displayed_points_value))
        layout.addLayout(self._stat_row("SCANS", self.scan_value))
        layout.addLayout(self._stat_row("UDP PACKETS", self.udp_packets_value))
        layout.addLayout(self._stat_row("UDP LOST", self.udp_lost_value))

        layout.addStretch()

        footer = QLabel(
            "Input: Raspberry Pi UDP stream\n"
            "Mapping: PC Ground Station\n"
            "Motion: simulated X"
        )
        footer.setStyleSheet("font-size: 11px; color: #4D5D72;")
        layout.addWidget(footer)

        return panel

    def _section(self, text):
        label = QLabel(text)
        label.setObjectName("SectionTitle")
        return label

    def _stat_value(self, text):
        label = QLabel(text)
        label.setObjectName("StatValue")
        return label

    def _stat_row(self, name, value_label):
        layout = QGridLayout()

        name_label = QLabel(name)
        name_label.setObjectName("StatName")

        layout.addWidget(name_label, 0, 0)
        layout.addWidget(value_label, 0, 1)

        return layout

    def start_udp_receiver(self):
        if self.thread is not None:
            return

        self.thread = QThread()
        self.worker = UdpLidarWorker()
        self.worker.moveToThread(self.thread)

        self.thread.started.connect(self.worker.run)
        self.worker.scan_ready.connect(self.on_scan_ready)
        self.worker.status_changed.connect(self.on_status_changed)
        self.worker.error_occurred.connect(self.on_error)
        self.worker.finished.connect(self.thread.quit)
        self.worker.finished.connect(self.worker.deleteLater)
        self.thread.finished.connect(self.on_thread_finished)

        self.thread.start()

    def stop_udp_receiver(self):
        if self.worker is not None:
            self.worker.stop()

    def on_scan_ready(self, xyz_points, stats):
        if len(xyz_points) > 0:
            self.map_points = np.vstack((self.map_points, xyz_points))

        x_position_m = stats["x_position_m"]
        drone_position = np.array([[x_position_m, 0.0, 0.0]], dtype=np.float64)

        if len(self.trajectory_points) == 0:
            self.trajectory_points = drone_position
        else:
            last_x = self.trajectory_points[-1, 0]
            if abs(x_position_m - last_x) >= 0.10:
                self.trajectory_points = np.vstack((self.trajectory_points, drone_position))

        self.latest_stats = stats
        self.update_stats_panel()

    def on_status_changed(self, status):
        self.status_value.setText(status)

        if "ONLINE" in status:
            self.status_value.setObjectName("Online")
        else:
            self.status_value.setObjectName("Offline")

        self.status_value.style().unpolish(self.status_value)
        self.status_value.style().polish(self.status_value)

    def on_error(self, error):
        self.status_value.setText("ERROR")
        self.status_value.setObjectName("Offline")
        self.status_value.style().unpolish(self.status_value)
        self.status_value.style().polish(self.status_value)

        print("UDP receiver error:")
        print(error)

    def on_thread_finished(self):
        self.thread.deleteLater()
        self.thread = None
        self.worker = None

    def update_stats_panel(self):
        self.distance_value.setText(f"{self.latest_stats['x_position_m']:.2f} m")
        self.speed_value.setText(f"{self.latest_stats['speed_x_m_s']:.2f} m/s")
        self.points_value.setText(str(self.latest_stats["total_received_points"]))
        self.scan_value.setText(str(self.latest_stats["scan_count"]))
        self.udp_packets_value.setText(str(self.latest_stats["udp_packets_received"]))
        self.udp_lost_value.setText(str(self.latest_stats["udp_packets_lost"]))
        self.sender_value.setText(self.latest_stats["last_sender"])

        displayed_count = min(len(self.map_points), MAX_DISPLAY_POINTS)
        self.displayed_points_value.setText(str(displayed_count))

    def get_points_for_display(self):
        if len(self.map_points) <= MAX_DISPLAY_POINTS:
            return self.map_points

        step = math.ceil(len(self.map_points) / MAX_DISPLAY_POINTS)
        return self.map_points[::step]

    def update_visual(self):
        self.ax.clear()

        self.ax.set_facecolor("#030508")
        self.figure.patch.set_facecolor("#030508")

        current_x = self.latest_stats["x_position_m"]
        x_max = self.compute_x_max(current_x)

        self.ax.set_xlim(0.0, x_max)
        self.ax.set_ylim(-Y_RANGE_M, Y_RANGE_M)
        self.ax.set_zlim(-Z_RANGE_M, Z_RANGE_M)

        self.ax.set_axis_off()
        self.ax.view_init(elev=24, azim=-62)

        self.draw_measure_grid(x_max)
        self.draw_point_cloud(x_max)
        self.draw_trajectory()
        self.draw_drone_ball(current_x)
        self.draw_overlay(x_max)

        self.canvas.draw_idle()

    def compute_x_max(self, current_x):
        minimum_x_max = 20.0
        dynamic_x_max = current_x + GRID_SPACING_M
        return max(minimum_x_max, dynamic_x_max)

    def draw_point_cloud(self, x_max):
        display_points = self.get_points_for_display()

        if len(display_points) == 0:
            return

        x = display_points[:, 0]
        y = display_points[:, 1]
        z = display_points[:, 2]

        self.ax.scatter(
            x,
            y,
            z,
            s=1.2,
            c=x,
            cmap="plasma",
            alpha=0.95,
            vmin=0.0,
            vmax=max(1.0, x_max),
            depthshade=False,
        )

    def draw_trajectory(self):
        if len(self.trajectory_points) < 2:
            return

        x = self.trajectory_points[:, 0]
        y = self.trajectory_points[:, 1]
        z = self.trajectory_points[:, 2]

        self.ax.plot(
            x,
            y,
            z,
            color="#FF2035",
            linewidth=2.5,
            alpha=0.95,
        )

    def draw_drone_ball(self, current_x):
        self.ax.scatter(
            [current_x],
            [0.0],
            [0.0],
            s=120,
            color="#FF2035",
            alpha=1.0,
            depthshade=False,
        )

    def draw_measure_grid(self, x_max):
        grid_color = "#172333"
        center_line_color = "#26384F"
        label_color = "#607891"

        x_ticks = np.arange(0.0, x_max + GRID_SPACING_M, GRID_SPACING_M)

        for x in x_ticks:
            self.ax.plot(
                [x, x],
                [-Y_RANGE_M, Y_RANGE_M],
                [0.0, 0.0],
                color=grid_color,
                linewidth=0.8,
                alpha=0.6,
            )

            self.ax.text(
                x,
                -Y_RANGE_M - 0.65,
                0.0,
                f"{int(x)} m",
                color=label_color,
                fontsize=8,
            )

        y_ticks = np.arange(-Y_RANGE_M, Y_RANGE_M + GRID_SPACING_M, GRID_SPACING_M)

        for y in y_ticks:
            self.ax.plot(
                [0.0, x_max],
                [y, y],
                [0.0, 0.0],
                color=grid_color,
                linewidth=0.8,
                alpha=0.45,
            )

        self.ax.plot(
            [0.0, x_max],
            [0.0, 0.0],
            [0.0, 0.0],
            color=center_line_color,
            linewidth=1.3,
            alpha=0.85,
        )

    def draw_overlay(self, x_max):
        self.ax.text2D(
            0.03,
            0.95,
            "COVENANT GROUND STATION",
            transform=self.ax.transAxes,
            color="#FFFFFF",
            fontsize=16,
            fontweight="bold",
        )

        self.ax.text2D(
            0.03,
            0.91,
            f"UDP LIDAR MAPPING  |  SCALE {int(GRID_SPACING_M)} m  |  RANGE {x_max:.0f} m",
            transform=self.ax.transAxes,
            color="#7F95B2",
            fontsize=10,
        )

    def closeEvent(self, event):
        self.stop_udp_receiver()

        if self.thread is not None:
            self.thread.quit()
            self.thread.wait(1000)

        event.accept()


def main():
    app = QApplication(sys.argv)
    window = CovenantGroundStationUdpV3()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
