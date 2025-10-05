import rclpy
from rclpy.node import Node
import os
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Header

CMD_HOST = str(os.getenv("CMD_HOST", "127.0.0.1"))
CMD_PORT = int(os.getenv("CMD_PORT", "5555"))


class AMCL_Init(Node):
    def __init__(self):
        super().__init__('amcl_init')

        # Инициализация локализации amcl
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        # Публикация начального положения (1 раз) с задержкой
        timer_period = 10.0
        self.timer = self.create_timer(timer_period, self.publish_initial_pose)

    def publish_initial_pose(self):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"

        x = 0.5
        y = 0.5
        yaw = 0.0

        from math import sin, cos
        qz = sin(yaw / 2.0)
        qw = cos(yaw / 2.0)

        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.position.z = 0.0
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = qz
        pose_msg.pose.pose.orientation.w = qw

        # Ковариация
        pose_msg.pose.covariance[0] = 0.25  # x
        pose_msg.pose.covariance[7] = 0.25  # y
        pose_msg.pose.covariance[35] = 0.06853891909122467 # yaw

        self.publisher_.publish(pose_msg)
        self.get_logger().info(f'Published initial pose: x={x}, y={y}, yaw={yaw:.2f} rad')
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = AMCL_Init()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()