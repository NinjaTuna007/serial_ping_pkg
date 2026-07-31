"""Launch the OWTT follower (receiver) node.

The serial link to the Teensy is owned by the succorfish_driver node (use its
"teensy" profile); launch it separately or set start_driver:=true here for a
single-node run.

robot_name selects the sound-velocity mapping: 'lolo' keeps the legacy
svs_interfaces/msg/SVS feed (/lolo/sensors/svs); any other name uses the stick
convention (std_msgs/msg/Float64 on /<robot_name>/smarc/sound_velocity).
Explicit sound_velocity_* arguments always win. The follower node itself stays
unnamespaced (its /<leader>/... outputs are absolute by design).
"""
import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_name = LaunchConfiguration('robot_name')
    is_lolo = PythonExpression(["'", robot_name, "' == 'lolo'"])

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'start_driver', default_value='false',
            description='Also launch succorfish_driver in the same namespace'),
        DeclareLaunchArgument(
            'driver_profile', default_value='teensy',
            description='Driver profile: teensy (115200/ttyACM) or succorfish (9600/ttyUSB)'),
        DeclareLaunchArgument(
            'driver_namespace', default_value='',
            description='Namespace for the driver (must match this node\'s namespace)'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('succorfish_driver'),
                    'launch',
                    'succorfish_driver.launch.py',
                ]),
            ]),
            condition=IfCondition(LaunchConfiguration('start_driver')),
            launch_arguments={
                'profile': LaunchConfiguration('driver_profile'),
                'namespace': LaunchConfiguration('driver_namespace'),
            }.items(),
        ),

        # OWTT range conversion
        DeclareLaunchArgument('owtt_delta_prefix', default_value='#I',
                              description='Teensy OWTT delta line prefix'),
        DeclareLaunchArgument('owtt_offset_us', default_value='79500.0',
                              description='Constant offset subtracted from delta (microseconds)'),
        DeclareLaunchArgument('default_sound_velocity', default_value='1500.0',
                              description='Fallback sound speed (m/s)'),

        # Sound-velocity source mapping (see module docstring)
        DeclareLaunchArgument(
            'robot_name', default_value='lolo',
            description='Vehicle name for the sound-velocity topic mapping'),
        DeclareLaunchArgument(
            'sound_velocity_topic',
            default_value=IfElseSubstitution(
                is_lolo, '/lolo/sensors/svs',
                ['/', robot_name, '/smarc/sound_velocity']),
            description='Live sound-velocity topic'),
        DeclareLaunchArgument(
            'sound_velocity_msg_type',
            default_value=IfElseSubstitution(
                is_lolo, 'svs_interfaces/msg/SVS', 'std_msgs/msg/Float64'),
            description='Sound-velocity msg type'),
        DeclareLaunchArgument(
            'sound_velocity_field',
            default_value=IfElseSubstitution(is_lolo, 'svs', 'data'),
            description='Field on the msg holding m/s'),
        DeclareLaunchArgument('leader_gps_msg_type', default_value='GeoPoint',
                              description='GeoPoint | NavSatFix'),

        # Leader name <-> modem id mapping (two leaders)
        DeclareLaunchArgument('leader1_name', default_value='leader1',
                              description='Name for the first leader'),
        DeclareLaunchArgument('leader1_modem_id', default_value='007',
                              description='Modem id of the first leader'),
        DeclareLaunchArgument('leader2_name', default_value='leader2',
                              description='Name for the second leader'),
        DeclareLaunchArgument('leader2_modem_id', default_value='111',
                              description='Modem id of the second leader'),
        DeclareLaunchArgument(
            'own_modem_id', default_value='101',
            description="This receiver's own modem address (for the $Y config command)"),
        DeclareLaunchArgument(
            'mode', default_value='receiver',
            description='receiver | wire (wire = passive, Teensy transparent)'),
        DeclareLaunchArgument('command_terminator', default_value='\r\n',
                              description='Terminator appended to Teensy commands'),
        DeclareLaunchArgument(
            'ignore_pps_after_s', default_value='0',
            description='Holdover experiment: seconds since Teensy boot after which it '
                        'ignores PPS and free-runs on the OCXO (0 = never; '
                        'normal sticks keep 0)'),

        Node(
            package='serial_ping_pkg',
            executable='owtt_follower_node',
            name='owtt_follower_node',
            output='screen',
            parameters=[{
                'owtt.delta_prefix': LaunchConfiguration('owtt_delta_prefix'),
                'owtt.offset_us': LaunchConfiguration('owtt_offset_us'),
                'owtt.default_sound_velocity': LaunchConfiguration('default_sound_velocity'),
                'owtt.sound_velocity_topic': LaunchConfiguration('sound_velocity_topic'),
                'owtt.sound_velocity_msg_type': LaunchConfiguration('sound_velocity_msg_type'),
                'owtt.sound_velocity_field': LaunchConfiguration('sound_velocity_field'),
                'follower.leader_gps_msg_type': LaunchConfiguration('leader_gps_msg_type'),
                'follower.leader1_name': LaunchConfiguration('leader1_name'),
                'follower.leader1_modem_id': LaunchConfiguration('leader1_modem_id'),
                'follower.leader2_name': LaunchConfiguration('leader2_name'),
                'follower.leader2_modem_id': LaunchConfiguration('leader2_modem_id'),
                'teensy.own_modem_id': LaunchConfiguration('own_modem_id'),
                'teensy.mode': LaunchConfiguration('mode'),
                'teensy.command_terminator': LaunchConfiguration('command_terminator'),
                'teensy.ignore_pps_after_s': LaunchConfiguration('ignore_pps_after_s'),
            }],
        ),
    ])
