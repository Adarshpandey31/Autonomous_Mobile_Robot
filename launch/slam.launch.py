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

    urdf_path = os.path.join(pkg_share, 'urdf', 'waiter_bot.urdf.xacro')

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
                'frame_id': 'laser_frame',
                'inverted': False,
                'angle_compensate': True
            }],
            output='screen',
            remappings=[('/scan', '/scan_raw')],
        ),
        Node(
            package=package_name,
            executable='lidar_filter',
            name='lidar_filter_node',
            output='screen',
            parameters=[{
                'input_topic': '/scan_raw',
                'output_topic': '/scan',
                'lower_threshold': 0.40,
                'upper_threshold': 8.0,
                'min_angle': -90.0,
                'max_angle': 90.0
            }]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_path])
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([pkg_share, 'config', 'view.rviz'])]
        ),
        Node(
            package='waiter_b',
            executable='dead_reckon',
            name='dead_reckon_node',
            parameters=[{
                'linear_scale': 0.67141,
                'angular_scale': 0.53047,
                'linear_bias': 0.0,
                'angular_bias': 0.0,
                'publish_rate': 50.0,
               'frame_id': 'odom',
                'child_frame_id': 'base_link'
            }],
            output='screen'
        ),
        # Node(
        #     package='slam_toolbox',
        #     executable='async_slam_toolbox_node',
        #     name='slam_toolbox',
        #     parameters=[{
        #         'use_sim_time': False,
        #         'map_frame': 'map',
        #         'odom_frame': 'odom',
        #         'base_frame': 'base_link',
        #         'scan_topic': 'scan',
        #         'mode': 'mapping',
        #     }],
        #     remappings=[
        #         ('scan', '/scan')   # your filtered scan
        #     ]
        # )
    ])