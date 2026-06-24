#!/usr/bin/env python3

import json
import math
import socket
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu


TARGET_IP = "192.168.1.103"   # IP du téléphone Android
TARGET_PORT = 56010

SEND_HZ = 10.0

BATTERY_TOPIC = "/ap/battery"
IMU_TOPIC = "/ap/imu/experimental/data"


def quaternion_to_euler_deg(x, y, z, w):
    """
    Converts quaternion to roll, pitch, yaw in degrees.
    The sign of yaw may need to be inverted later depending on the map convention.
    """

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)

    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (
        math.degrees(roll),
        math.degrees(pitch),
        math.degrees(yaw),
    )


def normalize_angle_deg(angle):
    while angle > 180.0:
        angle -= 360.0

    while angle < -180.0:
        angle += 360.0

    return angle


class CovenantBatteryImuBridge(Node):
    def __init__(self):
        super().__init__("covenant_battery_imu_udp_bridge")

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.last_battery = None
        self.last_imu = None

        self.initial_yaw_deg = None
        self.last_send_wall_time = 0.0

        self.create_subscription(
            BatteryState,
            BATTERY_TOPIC,
            self.on_battery,
            10,
        )

        self.create_subscription(
            Imu,
            IMU_TOPIC,
            self.on_imu,
            50,
        )

        self.timer = self.create_timer(
            1.0 / SEND_HZ,
            self.send_payload,
        )

        self.get_logger().info(f"Listening battery topic: {BATTERY_TOPIC}")
        self.get_logger().info(f"Listening IMU topic: {IMU_TOPIC}")
        self.get_logger().info(f"Sending UDP telemetry to {TARGET_IP}:{TARGET_PORT}")

    def on_battery(self, msg):
        self.last_battery = msg

    def on_imu(self, msg):
        self.last_imu = msg

    def send_payload(self):
        now = time.time()

        payload = {
            "source": "covenant_ros_bridge",
            "bridge_time": now,
            "status_available": False,
            "flight_mode": None,
            "armed": None,
            "health": None,
        }

        if self.last_battery is not None:
            voltage = float(self.last_battery.voltage)
            current = float(self.last_battery.current)
            percentage = float(self.last_battery.percentage)

            battery_valid = voltage > 3.0

            if 0.0 <= percentage <= 1.0:
                battery_percent = percentage * 100.0
            else:
                battery_percent = percentage

            payload.update(
                {
                    "battery_present": bool(self.last_battery.present),
                    "battery_valid": battery_valid,
                    "battery_percent": round(battery_percent, 1),
                    "voltage_v": round(voltage, 3),
                    "current_a": round(current, 3),
                    "battery_status_code": int(self.last_battery.power_supply_status),
                    "battery_health_code": int(self.last_battery.power_supply_health),
                }
            )

        if self.last_imu is not None:
            q = self.last_imu.orientation

            roll_deg, pitch_deg, yaw_deg = quaternion_to_euler_deg(
                float(q.x),
                float(q.y),
                float(q.z),
                float(q.w),
            )

            if self.initial_yaw_deg is None:
                self.initial_yaw_deg = yaw_deg

            yaw_relative_deg = normalize_angle_deg(yaw_deg - self.initial_yaw_deg)

            gyro = self.last_imu.angular_velocity
            accel = self.last_imu.linear_acceleration

            payload.update(
                {
                    "imu_frame_id": str(self.last_imu.header.frame_id),
                    "orientation_qx": round(float(q.x), 6),
                    "orientation_qy": round(float(q.y), 6),
                    "orientation_qz": round(float(q.z), 6),
                    "orientation_qw": round(float(q.w), 6),
                    "roll_deg": round(roll_deg, 2),
                    "pitch_deg": round(pitch_deg, 2),
                    "yaw_deg": round(yaw_deg, 2),
                    "yaw_relative_deg": round(yaw_relative_deg, 2),
                    "gyro_x_rad_s": round(float(gyro.x), 6),
                    "gyro_y_rad_s": round(float(gyro.y), 6),
                    "gyro_z_rad_s": round(float(gyro.z), 6),
                    "accel_x_m_s2": round(float(accel.x), 4),
                    "accel_y_m_s2": round(float(accel.y), 4),
                    "accel_z_m_s2": round(float(accel.z), 4),
                }
            )

        encoded = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        self.sock.sendto(encoded, (TARGET_IP, TARGET_PORT))

        if now - self.last_send_wall_time > 2.0:
            self.last_send_wall_time = now
            self.get_logger().info(json.dumps(payload, indent=2))


def main():
    rclpy.init()

    node = CovenantBatteryImuBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
