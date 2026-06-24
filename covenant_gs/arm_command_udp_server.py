#!/usr/bin/env python3

import socket

import rclpy
from rclpy.node import Node

from ardupilot_msgs.srv import ArmMotors


UDP_LISTEN_IP = "0.0.0.0"
UDP_LISTEN_PORT = 56020

ARM_SERVICE_NAME = "/ap/arm_motors"


class ArmCommandUdpServer(Node):
    def __init__(self):
        super().__init__("covenant_arm_command_udp_server")

        self.client = self.create_client(ArmMotors, ARM_SERVICE_NAME)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((UDP_LISTEN_IP, UDP_LISTEN_PORT))
        self.sock.setblocking(False)

        self.timer = self.create_timer(0.05, self.poll_udp)

    def poll_udp(self):
        while True:
            try:
                data, _addr = self.sock.recvfrom(256)
            except BlockingIOError:
                return
            except Exception:
                return

            payload = data.decode("utf-8", errors="ignore").strip()

            if payload == "1":
                self.call_arm_service(True)
            elif payload == "0":
                self.call_arm_service(False)

    def call_arm_service(self, arm: bool):
        if not self.client.service_is_ready():
            if not self.client.wait_for_service(timeout_sec=0.5):
                return

        request = ArmMotors.Request()
        request.arm = arm

        self.client.call_async(request)


def main():
    rclpy.init()

    node = ArmCommandUdpServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()