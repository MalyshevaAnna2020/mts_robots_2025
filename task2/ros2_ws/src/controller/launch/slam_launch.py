from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Устанавливаем параметры по умолчанию
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = LaunchConfiguration('params_file')

    # Аргументы запуска
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Путь расположения файла конфигурации
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('controller'),
            'config',
            'params.yaml'
        ]),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    # Узел SLAM Toolbox
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='lifelong_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/scan', '/scan'),
            ('/odom', '/odom')
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        slam_toolbox_node
    ])