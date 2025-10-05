from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import TimerAction, ExecuteProcess, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_controller = get_package_share_directory('controller')

    # Параметры SLAM
    params_file_slam = os.path.join(pkg_controller, 'resource', 'slam_toolbox_params_real.yaml')

    # Параметры nav2_bringup
    use_sim_time = 'false'
    params_file_nav = os.path.join(pkg_controller, 'resource', 'nav2_params.yaml')

    return LaunchDescription([
        # Передача телеметрии
        Node(
            package='controller',
            executable='telemetry_bridge',
            name='telemetry_bridge',
            output='screen',
        ),

        # Передача управления
        Node(
            package='controller',
            executable='send_vel',
            name='send_vel',
            output='screen',
        ),

        # Начальное положение робота
        Node(
            package='controller',
            executable='amcl_init',
            name='amcl_init',
            output='screen',
        ),

        # Отправка цели (goal)
        Node(
            package='controller',
            executable='navigation_goal_sender',
            name='navigation_goal_sender',
            output='screen',
        ),

        # SLAM
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node', # 'sync_slam_toolbox_node'
            name='slam_toolbox',
            output='screen',
            parameters=[params_file_slam],
            remappings=[
                ('/scan', '/scan'),
                ('/odom', '/odom'),
            ]
        ),

        # Активация SLAM Toolbox (автоматически)
        TimerAction(
            period=2.0,  # ждём 2 секунды после запуска
            # configure
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
                    output='screen'
                ),
            ]
        ),
        TimerAction(
            period=4.0,  # ждём еще 2 секунды после запуска
            # activate (после configure)
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
                    output='screen'
                )
            ]
        ),

        # Основной стек Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_controller, 'launch', 'navigation.py'])
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file_nav
            }.items()
        ),
    ])