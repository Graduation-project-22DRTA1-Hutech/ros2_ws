from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        # ================= micro-ROS agent =================
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['serial', '--dev', '/dev/ttyUSB0']
        ),

        # ================= Joystick / command =================
        Node(
            package='mecanum_joystick',
            executable='freq_publisher',
            name='freq_publisher',
            output='screen'
        ),

        # ================= LiDAR =================
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_c1',
            output='screen',
            parameters=[{
                'frame_id': 'laser',
                'scan_mode': 'Standard',
                'channel_type': 'serial',
                'serial_port': '/dev/ttyUSB1',
                'serial_baudrate': 460800
            }]
        ),

        # ================= Static TF: base_link -> laser =================
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_laser',
            arguments=['0', '0', '0.15', '0', '0', '0', 'base_link', 'laser']
        ),

        # ================= Static TF: base_link -> imu =================
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_imu',
            arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_link']
        ),

        # ================= Wheel odometry (KHÔNG publish TF) =================
	Node(
	    package='odom_to_tf_py',
	    executable='wheel_odom_node',
	    name='wheel_odom',
	    output='screen'
	),
		

        # ================= IMU filter =================
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            remappings=[
                ('imu/data_raw', '/imu/raw'),
                ('imu/data', '/imu/data')
            ]
        ),

        # ================= EKF robot_localization =================
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=['/home/robot/Desktop/ekf.yaml']
        ),

        # ================= SLAM Toolbox =================
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=['/home/robot/Desktop/slam_config.yaml']
        ),
    ])
