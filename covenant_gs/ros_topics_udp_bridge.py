#!/usr/bin/env python3

import json
import math
import socket
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSDurabilityPolicy

from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped


TARGET_IP = "192.168.1.102"   # IP actuelle du téléphone Android
TARGET_PORT = 56010

SEND_HZ = 10.0

BATTERY_TOPIC = "/ap/battery"
IMU_TOPIC = "/ap/imu/experimental/data"
POSE_TOPIC = "/ap/pose/filtered"
TWIST_TOPIC = "/ap/twist/filtered"


def make_sensor_qos():
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


def quaternion_to_euler_deg(x, y, z, w):
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


def safe_float(value, digits=3):
    try:
        value = float(value)

        if math.isnan(value):
            return None

        return round(value, digits)
    except Exception:
        return None


class CovenantRosTopicsUdpBridge(Node):
    def __init__(self):
        super().__init__("covenant_ros_topics_udp_bridge")

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.last_battery = None
        self.last_imu = None
        self.last_pose = None
        self.last_twist = None

        self.last_battery_wall_time = 0.0
        self.last_imu_wall_time = 0.0
        self.last_pose_wall_time = 0.0
        self.last_twist_wall_time = 0.0

        self.battery_count = 0
        self.imu_count = 0
        self.pose_count = 0
        self.twist_count = 0

        self.initial_imu_yaw_deg = None
        self.initial_pose_yaw_deg = None

        sensor_qos = make_sensor_qos()

        self.create_subscription(
            BatteryState,
            BATTERY_TOPIC,
            self.on_battery,
            sensor_qos,
        )

        self.create_subscription(
            Imu,
            IMU_TOPIC,
            self.on_imu,
            sensor_qos,
        )

        self.create_subscription(
            PoseStamped,
            POSE_TOPIC,
            self.on_pose,
            sensor_qos,
        )

        self.create_subscription(
            TwistStamped,
            TWIST_TOPIC,
            self.on_twist,
            sensor_qos,
        )

        self.timer = self.create_timer(
            1.0 / SEND_HZ,
            self.send_payload,
        )

    def on_battery(self, msg):
        self.last_battery = msg
        self.last_battery_wall_time = time.time()
        self.battery_count += 1

    def on_imu(self, msg):
        self.last_imu = msg
        self.last_imu_wall_time = time.time()
        self.imu_count += 1

    def on_pose(self, msg):
        self.last_pose = msg
        self.last_pose_wall_time = time.time()
        self.pose_count += 1

    def on_twist(self, msg):
        self.last_twist = msg
        self.last_twist_wall_time = time.time()
        self.twist_count += 1

    def send_payload(self):
        now = time.time()

        payload = {
            "source": "covenant_ros_bridge",
            "bridge_time": now,

            "status_available": False,
            "flight_mode": None,
            "armed": None,
            "health": None,

            "battery_received": self.last_battery is not None,
            "imu_received": self.last_imu is not None,
            "pose_received": self.last_pose is not None,
            "twist_received": self.last_twist is not None,

            "battery_count": self.battery_count,
            "imu_count": self.imu_count,
            "pose_count": self.pose_count,
            "twist_count": self.twist_count,
        }

        imu_yaw_relative_deg = None
        pose_yaw_relative_deg = None

        if self.last_battery is not None:
            voltage = float(self.last_battery.voltage)
            current = float(self.last_battery.current)
            percentage = float(self.last_battery.percentage)

            if 0.0 <= percentage <= 1.0:
                battery_percent = percentage * 100.0
            else:
                battery_percent = percentage

            payload.update(
                {
                    "battery_present": bool(self.last_battery.present),
                    "battery_valid": voltage > 3.0,
                    "battery_age_ms": int((now - self.last_battery_wall_time) * 1000.0),

                    "battery_percent": round(battery_percent, 1),
                    "voltage_v": round(voltage, 3),
                    "current_a": round(current, 3),
                    "temperature_c": safe_float(self.last_battery.temperature),

                    "battery_status_code": int(self.last_battery.power_supply_status),
                    "battery_health_code": int(self.last_battery.power_supply_health),
                    "battery_technology_code": int(self.last_battery.power_supply_technology),
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

            if self.initial_imu_yaw_deg is None:
                self.initial_imu_yaw_deg = yaw_deg

            imu_yaw_relative_deg = normalize_angle_deg(yaw_deg - self.initial_imu_yaw_deg)

            gyro = self.last_imu.angular_velocity
            accel = self.last_imu.linear_acceleration

            payload.update(
                {
                    "imu_frame_id": str(self.last_imu.header.frame_id),
                    "imu_age_ms": int((now - self.last_imu_wall_time) * 1000.0),

                    "orientation_qx": round(float(q.x), 6),
                    "orientation_qy": round(float(q.y), 6),
                    "orientation_qz": round(float(q.z), 6),
                    "orientation_qw": round(float(q.w), 6),

                    "roll_deg": round(roll_deg, 2),
                    "pitch_deg": round(pitch_deg, 2),
                    "yaw_deg": round(yaw_deg, 2),
                    "yaw_relative_deg": round(imu_yaw_relative_deg, 2),

                    "gyro_x_rad_s": round(float(gyro.x), 6),
                    "gyro_y_rad_s": round(float(gyro.y), 6),
                    "gyro_z_rad_s": round(float(gyro.z), 6),

                    "accel_x_m_s2": round(float(accel.x), 4),
                    "accel_y_m_s2": round(float(accel.y), 4),
                    "accel_z_m_s2": round(float(accel.z), 4),
                }
            )

        if self.last_pose is not None:
            p = self.last_pose.pose.position
            q = self.last_pose.pose.orientation

            pose_roll_deg, pose_pitch_deg, pose_yaw_deg = quaternion_to_euler_deg(
                float(q.x),
                float(q.y),
                float(q.z),
                float(q.w),
            )

            if self.initial_pose_yaw_deg is None:
                self.initial_pose_yaw_deg = pose_yaw_deg

            pose_yaw_relative_deg = normalize_angle_deg(
                pose_yaw_deg - self.initial_pose_yaw_deg
            )

            payload.update(
                {
                    "pose_frame_id": str(self.last_pose.header.frame_id),
                    "pose_age_ms": int((now - self.last_pose_wall_time) * 1000.0),

                    "position_x_m": round(float(p.x), 3),
                    "position_y_m": round(float(p.y), 3),
                    "position_z_m": round(float(p.z), 3),
                    "altitude_m": round(float(p.z), 3),

                    "pose_roll_deg": round(pose_roll_deg, 2),
                    "pose_pitch_deg": round(pose_pitch_deg, 2),
                    "pose_yaw_deg": round(pose_yaw_deg, 2),
                    "pose_yaw_relative_deg": round(pose_yaw_relative_deg, 2),
                }
            )

        if self.last_twist is not None:
            linear = self.last_twist.twist.linear
            angular = self.last_twist.twist.angular

            vx = float(linear.x)
            vy = float(linear.y)
            vz = float(linear.z)

            velocity_mps = math.sqrt(vx * vx + vy * vy + vz * vz)

            payload.update(
                {
                    "twist_frame_id": str(self.last_twist.header.frame_id),
                    "twist_age_ms": int((now - self.last_twist_wall_time) * 1000.0),

                    "velocity_x_mps": round(vx, 4),
                    "velocity_y_mps": round(vy, 4),
                    "velocity_z_mps": round(vz, 4),
                    "velocity_mps": round(velocity_mps, 4),

                    "angular_x_rad_s": round(float(angular.x), 6),
                    "angular_y_rad_s": round(float(angular.y), 6),
                    "angular_z_rad_s": round(float(angular.z), 6),
                }
            )

        if pose_yaw_relative_deg is not None:
            payload["map_yaw_source"] = "pose"
            payload["map_yaw_relative_deg"] = round(pose_yaw_relative_deg, 2)
        elif imu_yaw_relative_deg is not None:
            payload["map_yaw_source"] = "imu"
            payload["map_yaw_relative_deg"] = round(imu_yaw_relative_deg, 2)
        else:
            payload["map_yaw_source"] = None
            payload["map_yaw_relative_deg"] = None

        encoded = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        self.sock.sendto(encoded, (TARGET_IP, TARGET_PORT))


def main():
    rclpy.init()

    node = CovenantRosTopicsUdpBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()