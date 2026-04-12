from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[
                {"port": 9090},
                {"address": "0.0.0.0"},  # expose to LAN
                {"use_compression": False}
            ],
            output='screen'
        ),
        Node(
            package=package_name,
            executable='arduino_bridge',
            name='arduino_bridge_node',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB1',
                'baudrate': 115200
            }]
        ),
    ])
    
