from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='serial_ping_pkg',
            executable='informed_follower_node',
            name='informed_follower_node',
            output='screen',
            parameters=[
                {'serial.port': '/dev/ttyUSB0'},
                {'serial.port_fallback': '/dev/ttyUSB1'},
                {'serial.baudrate': 9600},
                {'leader_name': 'leader'},
            ]
        )
    ])
