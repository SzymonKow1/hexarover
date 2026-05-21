import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    bringup_pkg = get_package_share_directory('hexarover_bringup')
    sllidar_pkg = get_package_share_directory('sllidar_ros2')

    arg_rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Czy otworzyć RViz2?'
    )

    # 1. LIDAR
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sllidar_pkg, 'launch', 'sllidar_a2m12_launch .py')
        ),
        launch_arguments={'serial_port': '/dev/lidar'}.items()
    )

    # 2. KAMERA
    video_publisher_node = Node(
        package='hexarover_vision',
        executable='video_publisher',
        name='video_publisher',
        output='screen',
    )

    # 3. VISION NODE – YOLO + fuzja LiDAR
    # publikuje: /human_angle, /human_distance, /human_marker, /camera_fov, ...
    vision_node = Node(
        package='hexarover_vision',
        executable='vision_node',
        name='vision_node',
        output='screen',
    )

    # 4. FOLLOWER NODE – PID, subskrybuje /human_angle i /human_distance
    # publikuje /cmd_vel
    follower_node = Node(
        package='hexarover_vision',
        executable='follower_node',
        name='follower_node',
        output='screen',
    )

    # 5. CYTRON DRIVER – subskrybuje /cmd_vel, steruje silnikami przez UART
    cytron_node = Node(
        package='cytron_driver',
        executable='cytron_node',
        name='cytron_node',
        output='screen',
    )

    # 6. STATIC TF: laser → lidar_link
    tf_laser_to_lidar_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_laser_to_lidar_link',
        arguments=['0', '0', '0', '4.7124', '0', '0', 'laser', 'lidar_link'],
        output='screen',
    )

    # 7. RVIZ2
    rviz_config = os.path.join(bringup_pkg, 'rviz', 'vision_dev.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        arg_rviz,
        lidar_launch,
        video_publisher_node,
        vision_node,
        follower_node,
        cytron_node,
        tf_laser_to_lidar_link,
        rviz_node,
    ])