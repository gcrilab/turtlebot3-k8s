#!/usr/bin/env python3
"""Launch TurtleBot3 bringup together with a USB camera (v4l2_camera)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    camera_device = LaunchConfiguration('camera_device', default='/dev/video0')
    camera_width = LaunchConfiguration('camera_width', default='640')
    camera_height = LaunchConfiguration('camera_height', default='480')

    # TurtleBot3 robot bringup (OpenCR + LiDAR)
    turtlebot3_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_bringup'),
                'launch',
                'robot.launch.py',
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # USB camera via v4l2_camera
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        output='screen',
        parameters=[{
            'video_device': camera_device,
            'image_size': [camera_width, camera_height],
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation clock'),
        DeclareLaunchArgument('camera_device', default_value='/dev/video0',
                              description='V4L2 camera device node'),
        DeclareLaunchArgument('camera_width', default_value='640',
                              description='Camera capture width in pixels'),
        DeclareLaunchArgument('camera_height', default_value='480',
                              description='Camera capture height in pixels'),
        turtlebot3_bringup,
        camera_node,
    ])
