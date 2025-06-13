from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='serial_ping_pkg',
            executable='informed_leader_node',
            name='informed_leader_node',
            output='screen',
            parameters=[
                {'serial.port': '/dev/ttyUSB0'},
                {'serial.port_fallback': '/dev/ttyUSB1'},
                {'serial.baudrate': 9600},
                {'robot_name': 'leader'},
                {'ping_command': 'PING'},
                {'sound_velocity': 1500.0},
                {'timeout_threshold': 2.0},
            ]
        )
    ])
