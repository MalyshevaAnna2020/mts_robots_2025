import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import math
from geometry_msgs.msg import Quaternion


class NavigationGoalWithRetry(Node):
    def __init__(self):
        super().__init__('navigation_goal_with_retry')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Параметры
        self.retry_delay_sec = 2.0  # секунды между попытками
        
        self.goal_x = 4.0
        self.goal_y = 4.0
        self.goal_theta = math.pi / 2

    def send_goal(self):
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Сервер навигации недоступен!')
            self.retry_timer = self.create_timer(self.retry_delay_sec, self.send_goal)
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.goal_x
        goal_msg.pose.pose.position.y = self.goal_y
        goal_msg.pose.pose.orientation = self.yaw_to_quaternion(self.goal_theta)

        self.get_logger().info(f'Отправка цели ({self.goal_x}, {self.goal_y})')
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        distance = feedback_msg.feedback.distance_remaining
        self.get_logger().debug(f'Осталось: {distance:.2f} м')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Цель отклонена сервером.')
            self.handle_failure()
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # # Статусы (из nav2_msgs/action/NavigateToPose.idl)
        # enum GoalStatus {
        #     UNKNOWN = 0,
        #     ACCEPTED = 1,
        #     EXECUTING = 2,
        #     CANCELING = 3,
        #     SUCCEEDED = 4,
        #     CANCELED = 5,
        #     ABORTED = 6
        # };
        status = future.result().status
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Навигация успешно завершена!')
            rclpy.shutdown()
        else:
            self.get_logger().error(f'Навигация не удалась. Статус: {status}')
            self.handle_failure()

    def handle_failure(self):
        self.get_logger().info(f'Ждём {self.retry_delay_sec} сек перед повторной попыткой...')
        # Создаём таймер для повторной отправки
        self.retry_timer = self.create_timer(self.retry_delay_sec, self.retry_send_goal)
        

    def retry_send_goal(self):
        self.retry_timer.cancel()  # отменяем таймер
        self.send_goal()           # повторная отправка

    @staticmethod
    def yaw_to_quaternion(yaw):
        return Quaternion(z=math.sin(yaw / 2), w=math.cos(yaw / 2))


def main(args=None):
    rclpy.init(args=args)
    node = NavigationGoalWithRetry()
    
    # Первая отправка
    node.send_goal()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
