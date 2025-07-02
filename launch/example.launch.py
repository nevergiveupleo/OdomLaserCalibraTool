from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='laser_odom_calibration',
            executable='io_test',
            name='calib',
            output='screen',
            parameters=[
                {'file_path': '/home/mark/calibration_ws/src/2d_lidar_odom_calibration/'},
                {'bagname': 'rosbag2_2025_07_02-14_43_39'},
                {'laser_topic': '/scan'},
                {'odom_topic': '/odom'},
            ],
        )
    ])
