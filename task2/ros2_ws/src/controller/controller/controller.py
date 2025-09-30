#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose, Twist, Quaternion
import tf_transformations
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from math import pi
import math
import os
import socket
import struct
import time

class Controller(Node):
    def __init__(self):
        ################
        # ROS2
        ################
        # Инициализация
        super().__init__('controller')
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_th = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        self.wx = 0.0
        self.wy = 0.0
        self.wz = 0.0
        self.ranges = []

        # Создание издателей (паблишеров)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_lidar = self.create_publisher(LaserScan, '/scan', 10)
        # TF broadcaster (odom->base_link)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer_tf = self.create_timer(0.05, self.publish_odom_tf)  # 20 Гц

        # Таймер отправки сообщений
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.publish_message)

        # Сообщения
        # Одометрия
        self.odom_msg = Odometry()
        self.odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_link" # система координат робота
        self.odom_msg.pose.pose.position.x = self.odom_x
        self.odom_msg.pose.pose.position.y = self.odom_y
        q = tf_transformations.quaternion_from_euler(0, 0, self.odom_th)
        self.odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        # Лидар
        self.lidar_msg = LaserScan()
        self.lidar_msg.header.stamp = self.get_clock().now().to_msg()
        self.lidar_msg.header.frame_id = "base_link" # привяжем к системе координат робота
        self.lidar_msg.angle_min = -pi/4
        self.lidar_msg.angle_max = pi/4
        # self.lidar_msg.angle_increment = 0.01
        self.lidar_msg.range_min = 0.200000
        self.lidar_msg.range_max = 12.000000
        # self.time_prev = self.get_clock().now().to_msg()

        ###################
        # demo.py
        ###################
        global CMD_HOST, CMD_PORT, TEL_HOST, TEL_PORT, PROTO
        global SAFE_DIST, CRASH_DIST, RECOVER_TIME, TURN_TIME, FORWARD_SPEED
        CMD_HOST  = str(os.getenv("CMD_HOST", "127.0.0.1"))
        CMD_PORT  = int(os.getenv("CMD_PORT", "5555"))
        TEL_HOST  = str(os.getenv("TEL_HOST", "0.0.0.0"))
        TEL_PORT  = int(os.getenv("TEL_PORT", "5600"))
        PROTO     = str(os.getenv("PROTO", "tcp"))

        # === Параметры ===
        SAFE_DIST = 0.5
        CRASH_DIST = 0.3
        RECOVER_TIME = 1               # время отката назад
        TURN_TIME = 0.6                # базовое время для поворота
        FORWARD_SPEED = 0.6            # м/с

        self.sock_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        if PROTO == "udp":
            self.sock_tel = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock_tel.bind((TEL_HOST, TEL_PORT))
        else:
            self.sock_tel = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock_tel.bind((TEL_HOST, TEL_PORT))
            self.sock_tel.listen(1)
            self.get_logger().info(f"[client] waiting for telemetry TCP on {TEL_HOST}:{TEL_PORT}...")
            conn, _ = self.sock_tel.accept()
            self.sock_tel = conn
            self.get_logger().info("[client] connected to udp_diff telemetry")

        # Таймер выполнения основной программы
        timer_period_main = 0.1
        self.timer_main = self.create_timer(timer_period_main, self.control)

    def send_cmd(self, v: float, w: float):
        self.get_logger().info(f"send_cmd v={v:.2f}, w={w:.2f}")
        packet = struct.pack("<2f", v, w)
        self.sock_cmd.sendto(packet, (CMD_HOST, CMD_PORT))


    def recv_all(self, sock, size):
        buf = b""
        while len(buf) < size:
            chunk = sock.recv(size - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf


    def recv_tel(self):
        if PROTO == "udp":
            data, _ = self.sock_tel.recvfrom(65535)
        else:
            size_bytes = self.sock_tel.recv(4)
            if not size_bytes:
                return None
            size = struct.unpack("<I", size_bytes)[0]
            data = self.recv_all(self.sock_tel, size)

        if not data.startswith(b"WBTG"):  # ВНИМАНИЕ! Было: WBT2
            self.get_logger().info('WBT2 -> WBTG')
            return None

        # теперь в пакете: 9 float после "WBTG" (36 байт)
        # "<6f" -> "<9f"
        header_size = 4 + 9 * 4
        odom_x, odom_y, odom_th, vx, vy, vth, wx, wy, wz = struct.unpack("<9f", data[4:header_size])
        n = struct.unpack("<I", data[header_size:header_size + 4])[0]
        ranges = []
        if n > 0:
            ranges = struct.unpack(f"<{n}f", data[header_size + 4:header_size + 4 + 4 * n])

        return odom_x, odom_y, odom_th, (vx, vy, vth), (wx, wy, wz), ranges


    def publish_message(self):
        # Одометрия
        self.odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg.pose.pose.position.x = self.odom_x
        self.odom_msg.pose.pose.position.y = self.odom_y
        q = tf_transformations.quaternion_from_euler(0, 0, self.odom_th)
        self.odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        # Линейные скорости
        self.odom_msg.twist.twist.linear.x = self.vx
        self.odom_msg.twist.twist.linear.y = self.vy
        self.odom_msg.twist.twist.linear.z = self.vth
        self.odom_msg.twist.twist.angular.x = self.wx
        self.odom_msg.twist.twist.angular.y = self.wy
        self.odom_msg.twist.twist.angular.z = self.wz
        self.pub_odom.publish(self.odom_msg)

        # Лидар
        self.lidar_msg.header.stamp = self.get_clock().now().to_msg()
        self.lidar_msg.angle_increment = pi/2/(len(self.ranges) - 1)
        ################
        # необязательно
        # time_now = self.get_clock().now().to_msg()
        # self.lidar_msg.scan_time = time_now - self.time_prev
        # self.time_prev = time_now
        # self.lidar_msg.time_increment = self.lidar_msg.scan_time / len(self.ranges)
        ################
        self.lidar_msg.ranges = self.ranges
        self.pub_lidar.publish(self.lidar_msg)

    def publish_odom_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.odom_x
        t.transform.translation.y = self.odom_y
        t.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, self.odom_th)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

    def control(self):
        tel = self.recv_tel()
        if not tel:
            return
        x, y, th, vel, gyro, ranges = tel
        vx, vy, vth = vel
        wx, wy, wz = gyro

        self.odom_x = x
        self.odom_y = y
        self.odom_th = th
        self.vx = vx
        self.vy = vy
        self.vth = vth
        self.wx = wx
        self.wy = wy
        self.wz = wz
        
        self.ranges = ranges

        self.send_cmd(-0.15, 0)

        self.get_logger().info(f"[client] pos=({x:.2f},{y:.2f},θ={math.degrees(th):.1f}°) "
              f"vel=({vx:.2f},{vy:.2f},{vth:.2f}) gyro=({wx:.2f},{wy:.2f},{wz:.2f}) ")





def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    
    # По завершении
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()