from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()
    file_path_param = DeclareLaunchArgument('file_path', default_value='$HOME/calibration_ws/src/2d_lidar_odom_calibration/')
    ld.add_action(file_path_param)
    bag_name_param = DeclareLaunchArgument('bag_name', default_value='rosbag2_2025_07_03-14_23_40')
    ld.add_action(bag_name_param)
    laser_topic_param = DeclareLaunchArgument('laser_topic', default_value='/scan')
    ld.add_action(laser_topic_param)
    odom_topic_param = DeclareLaunchArgument('odom_topic', default_value='/odom')
    ld.add_action(odom_topic_param)
    
    cali_node = Node(
                package='laser_odom_calibration',
                executable='io_test',
                name='calib',
                output='screen',
                parameters=[
                    {'file_path': LaunchConfiguration('file_path')},
                    {'bagname': LaunchConfiguration('bag_name')},
                    {'laser_topic': LaunchConfiguration('laser_topic')},
                    {'odom_topic': LaunchConfiguration('odom_topic')},
                ],
    )
    ld.add_action(cali_node)
    return ld