import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import socket
import struct
import math
import os

TEL_HOST = str(os.getenv("TEL_HOST", "0.0.0.0"))
TEL_PORT = int(os.getenv("TEL_PORT", "5600"))
PROTO = str(os.getenv("PROTO", "tcp"))

LIDAR_X = 0.0
LIDAR_Y = 0.0
# LIDAR_Z = 0.031
LIDAR_Z = 0.0
LIDAR_YAW = 0.0


class TelemetryBridge(Node):
    def __init__(self):
        super().__init__('telemetry_bridge')

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu', 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        self.publish_static_lidar_transform()

        if PROTO == "udp":
            self.sock_tel = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock_tel.bind((TEL_HOST, TEL_PORT))
        else:
            self.sock_tel = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock_tel.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock_tel.bind((TEL_HOST, TEL_PORT))
            self.sock_tel.listen(1)
            self.get_logger().info(f"Waiting for TCP telemetry on {TEL_HOST}:{TEL_PORT}...")
            conn, _ = self.sock_tel.accept()
            self.sock_tel = conn
            self.get_logger().info("Connected to Webots telemetry")

        self.timer = self.create_timer(0.033333333333, self.read_telemetry)

        self.scan_msg = LaserScan()
        self.scan_msg.header.frame_id = 'lidar_link'
        self.scan_msg.angle_min = -math.pi/4
        self.scan_msg.angle_max = math.pi/4
        self.scan_msg.time_increment = 0.0
        self.scan_msg.scan_time = 0.1
        self.scan_msg.range_min = 0.2
        self.scan_msg.range_max = 7.5

    def publish_static_lidar_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'lidar_link'
        t.transform.translation.x = LIDAR_X
        t.transform.translation.y = LIDAR_Y
        t.transform.translation.z = LIDAR_Z
        q = self.euler_to_quaternion(0, 0, LIDAR_YAW)
        t.transform.rotation = q
        self.static_tf_broadcaster.sendTransform(t)


    def recv_all(self, size):
        buf = b""
        while len(buf) < size:
            chunk = self.sock_tel.recv(size - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf

    def read_telemetry(self):
        try:
            if PROTO == "udp":
                data, _ = self.sock_tel.recvfrom(65535)
            else:
                size_bytes = self.sock_tel.recv(4)
                if not size_bytes:
                    return
                size = struct.unpack("<I", size_bytes)[0]
                data = self.recv_all(size)
                if not data or not data.startswith(b"WBTG"):
                    return

            if len(data) < 4 + 9*4 + 4:
                return

            header_size = 4 + 9 * 4
            odom_x, odom_y, odom_th, vx, vy, vth, wx, wy, wz = struct.unpack("<9f", data[4:header_size])
            n = struct.unpack("<I", data[header_size:header_size + 4])[0]
            ranges = []
            if n > 0 and len(data) >= header_size + 4 + 4 * n:
                ranges = struct.unpack(f"<{n}f", data[header_size + 4:header_size + 4 + 4 * n])
            # self.get_logger().info(f"len(ranges) = {len(ranges)}")
            # это делается в параметрах slam_toolbox
            # ranges = ranges[::3]
            filtered_ranges = []
            for r in ranges:
                if r > self.scan_msg.range_max or r < self.scan_msg.range_min:
                    filtered_ranges.append(float('inf'))
                else:
                    filtered_ranges.append(r)


            now = self.get_clock().now().to_msg()

            # === Odometry ===
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x = odom_x
            odom.pose.pose.position.y = odom_y
            q = self.euler_to_quaternion(0, 0, odom_th)
            odom.pose.pose.orientation = q
            odom.twist.twist.linear.x = vx
            odom.twist.twist.linear.y = vy
            odom.twist.twist.angular.z = vth
            self.odom_pub.publish(odom)

            # === TF: odom -> base_link ===
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = odom_x
            t.transform.translation.y = odom_y
            t.transform.translation.z = 0.0
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)
            # self.publish_static_lidar_transform()

            # === LiDAR ===
            if filtered_ranges:
                self.scan_msg.header.stamp = now
                self.scan_msg.ranges = list(filtered_ranges)
                self.scan_msg.intensities = [0.0] * len(filtered_ranges)
                self.scan_msg.angle_increment = (math.pi/2) / (len(filtered_ranges) - 1)
                self.scan_pub.publish(self.scan_msg)

            # === IMU ===
            imu = Imu()
            imu.header.stamp = now
            imu.header.frame_id = 'imu_link'
            imu.angular_velocity.x = wx
            imu.angular_velocity.y = wy
            imu.angular_velocity.z = wz
            self.imu_pub.publish(imu)

        except Exception as e:
            self.get_logger().warn(f"Parse error: {e}")

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()