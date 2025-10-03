from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import ExecuteProcess
from launch.actions import TimerAction

def generate_launch_description():
    pkg_controller = get_package_share_directory('controller')
    params_file = os.path.join(pkg_controller, 'resource', 'slam_toolbox_params_real.yaml')

    return LaunchDescription([
        # Передача телеметрии
        Node(
            package='controller',
            executable='telemetry_bridge',
            name='telemetry_bridge',
            output='screen',
        ),

        # SLAM
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node', # 'sync_slam_toolbox_node'
            name='slam_toolbox',
            output='screen',
            parameters=[params_file],
            remappings=[
                ('/scan', '/scan'),
                ('/odom', '/odom'),
            ]
        ),

        # Активация SLAM Toolbox (автоматически)
        TimerAction(
            period=2.0,  # ждём 2 секунды после запуска
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
                    output='screen'
                ),
            ]
        ),
        TimerAction(
            period=4.0,  # ждём еще 2 секунды после запуска
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
                    output='screen'
                )
            ]
        )
    ])