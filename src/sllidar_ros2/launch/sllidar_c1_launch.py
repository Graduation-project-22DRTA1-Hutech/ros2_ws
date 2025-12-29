#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        # ===== LIDAR PARAMS =====
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0'
        ),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value='460800'
        ),

        DeclareLaunchArgument(
            'frame_id',
            default_value='laser'
        ),

        # ===== SL LIDAR NODE =====
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': LaunchConfiguration('serial_port'),
                'serial_baudrate': LaunchConfiguration('serial_baudrate'),
                'frame_id': LaunchConfiguration('frame_id'),
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard'
            }]
        ),

        # ===== STATIC TF: base_link -> laser =====
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_tf',
            arguments=[
                '0', '0', '0.15',   # x y z (chỉnh theo robot)
                '0', '0', '0',      # roll pitch yaw
                'base_link',
                'laser'
            ]
        ),
    ])

