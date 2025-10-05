import rclpy
from rclpy.node import Node
import socket
import struct
import math
import os
from geometry_msgs.msg import Twist

CMD_HOST = str(os.getenv("CMD_HOST", "127.0.0.1"))
CMD_PORT = int(os.getenv("CMD_PORT", "5555"))


class Send_Vel(Node):
    def __init__(self):
        super().__init__('send_vel')

        self.v = 0.0
        self.w = 0.0

        self.sock_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.subscription = self.create_subscription(Twist, '/cmd_vel_nav', self.listener_callback, 10)
        self.subscription

        # self.timer = self.create_timer(0.1, self.start_build_map)

    def listener_callback(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z
        self.send_cmd()

    def send_cmd(self):
        # self.get_logger().info(f"send_cmd v={v:.2f}, w={w:.2f}")
        packet = struct.pack("<2f", self.v, self.w)
        self.sock_cmd.sendto(packet, (CMD_HOST, CMD_PORT))

    def start_build_map(self):
        t = self.get_clock().now().nanoseconds / 1e9
        self.v = 0.3*math.sin(0.5*t)
        self.w = 0.1
        self.send_cmd()


def main(args=None):
    rclpy.init(args=args)
    node = Send_Vel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()