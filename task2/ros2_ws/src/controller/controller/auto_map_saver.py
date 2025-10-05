import rclpy
from rclpy.node import Node
from slam_toolbox.srv import SaveMap
from datetime import datetime
import os

class AutoMapSaver(Node):
    def __init__(self):
        super().__init__('auto_map_saver')

        self.timer = self.create_timer(1.0, self.save_map_callback)

    def save_map_callback(self):
        os.system("ros2 run nav2_map_server map_saver_cli -f map > /dev/null 2>&1")
        

def main(args=None):
    rclpy.init(args=args)
    node = AutoMapSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()