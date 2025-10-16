from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import TimerAction, ExecuteProcess, IncludeLaunchDescription, Shutdown
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
    config_file_path = os.path.join(pkg_controller, "resource", "odom_to_tf.yaml")

    return LaunchDescription([
        # Передача телеметрии
        Node(
            package='controller',
            executable='telemetry_bridge',
            name='telemetry_bridge',
            output='screen',
            # on_exit=Shutdown()
        ),

        # Передача управления
        Node(
            package='controller',
            executable='send_vel',
            name='send_vel',
            output='screen',
            # on_exit=Shutdown()
        ),

        # Начальное положение робота
        Node(
            package='controller',
            executable='amcl_init',
            name='amcl_init',
            output='screen',
            # on_exit=Shutdown()
        ),

        # Отправка цели (goal)
        Node(
            package='controller',
            executable='navigation_goal_sender',
            name='navigation_goal_sender',
            output='screen',
            # on_exit=Shutdown()
        ),

        # # Перед тем, как использовать следующие 2 узла,
        # # надо убрать одометрию и tf в telemetry_bridge.py
        # # Одометрия из скана лидара
        # Node(
        #     package='lidar_odometry',
        #     executable='lidar_odometry_node',
        #     name='lidar_odometry_node',
        #     output='screen'
        # ),

        # # Преобразование одометрии в tf
        # Node(
        #     package="odom_to_tf_ros2",
        #     executable="odom_to_tf",
        #     name="odom_to_tf",
        #     output="screen",
        #     parameters=[config_file_path],
        #     remappings=[],
        # ),

        # SLAM
        # # Cartographer
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         PathJoinSubstitution([pkg_controller, 'launch', 'cartographer.py'])
        #     ),
        # ),
        # slam_toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_controller, 'launch', 'slam.py'])
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': params_file_slam,
                'use_lifecycle_manager': 'false',
            }.items()
        ),
        # Node(
        #     package='slam_toolbox',
        #     executable='async_slam_toolbox_node', # 'sync_slam_toolbox_node'
        #     name='slam_toolbox',
        #     output='screen',
        #     parameters=[params_file_slam],
        #     remappings=[
        #         ('/scan', '/scan'),
        #         ('/odom', '/odom'),
        #     ]
        # ),

        # # Активация SLAM Toolbox (автоматически)
        # TimerAction(
        #     period=2.0,  # ждём 2 секунды после запуска
        #     # configure
        #     actions=[
        #         ExecuteProcess(
        #             cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
        #             output='screen'
        #         ),
        #     ]
        # ),
        # TimerAction(
        #     period=4.0,  # ждём еще 2 секунды после запуска
        #     # activate (после configure)
        #     actions=[
        #         ExecuteProcess(
        #             cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
        #             output='screen'
        #         )
        #     ]
        # ),

        # Основной стек Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_controller, 'launch', 'navigation.py'])
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file_nav,
                'autostart': 'true',
                # 'use_respawn': 'true',
            }.items()
        ),
    ])