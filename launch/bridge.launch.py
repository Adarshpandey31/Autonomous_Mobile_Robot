from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_name = 'waiter_b'

    # Paths
    pkg_share = get_package_share_directory(package_name)
    
    bridge_port = LaunchConfiguration('bridge_port', default='/dev/ttyUSB1')
    bridge_baud = LaunchConfiguration('bridge_baud', default='115200')
    
    lidar_port = LaunchConfiguration('lidar_port', default='/dev/ttyUSB0')
    lidar_baud = LaunchConfiguration('lidar_baud', default='115200')

    return LaunchDescription([
        Node(
            package=package_name,
            executable='arduino_bridge',
            name='arduino_bridge_node',
            output='screen',
            parameters=[{
                'port': bridge_port,
                'baudrate': bridge_baud
            }]
        ),
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            parameters=[{
                'serial_port': lidar_port,
                'serial_baudrate': lidar_baud,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True
            }],
            output='screen'
        )
    ])