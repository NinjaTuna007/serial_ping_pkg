"""TWTT informed-leader node.

Two-way-travel-time "informed" leader: pings the follower over the Succorfish
modem, converts the reply travel time to a range, then transmits its own latest
GPS fix together with that distance as a ``$B`` frame so the follower can
reconstruct both where the leader is and how far away it is. Runs on a fixed
timer, or (in slave mode) only after hearing a ``#B`` broadcast.

The modem link is owned by the ``succorfish_driver`` node; this node never opens
a serial port. Pings go out as ``SendCommand`` requests and the ``$B`` position
frame is published to the driver's ``TX`` topic.

Key parameters (defaults from ``config/tuper_twtt/tuper_twtt_config.yaml``):
- ``robot_name`` -- builds the default GPS topic.
- ``leader_gps_topic`` / ``leader_gps_msg_type`` -- own GPS source (``GeoPoint`` or ``NavSatFix``).
- ``ping_command`` -- command used to ping the follower (e.g. ``$P111``).
- ``sound_velocity`` -- m/s used for travel-time -> range.
- ``timeout_threshold`` -- ping reply timeout (s); ``timer_period`` -- seconds between pings.
- ``is_slave`` -- if true, ping only after a ``#B`` broadcast instead of on a timer.

Published topics: none on ROS (the ``$B`` frame goes out over the driver's ``TX`` topic).

Subscribed topics:
- ``leader_gps_topic`` (``geographic_msgs/GeoPoint`` or ``sensor_msgs/NavSatFix``) -- own GPS.
- the driver's ``RX`` topic (in slave mode) -- to hear ``#B`` broadcasts.
"""
from rclpy.node import Node

from serial_ping_pkg.common.spin import run_simple_node
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix
from serial_ping_pkg.utils import load_yaml_config
from serial_ping_pkg.common.driver_client import DriverClient
from serial_ping_pkg.common.ping_protocol import travel_time_to_distance
from serial_ping_pkg.tuper_twtt.leader_protocol import (
    build_position_broadcast,
)

# Modem reply carries the travel time as ``T`` followed by the tick count.
PING_REPLY_REGEX = r'T\d{5,}'


class InformedLeaderNode(Node):
    def __init__(self):
        super().__init__('twtt_leader_node')

        config = load_yaml_config('serial_ping_pkg', 'tuper_twtt/tuper_twtt_config.yaml')
        self.declare_parameter('robot_name', 'leader')
        self.declare_parameter('sound_velocity', config['serial'].get('sound_velocity', 1500.0))
        self.declare_parameter('timeout_threshold', config['serial'].get('timeout_threshold', 2.0))
        self.declare_parameter('timer_period', config.get('timer_period', 3.0))

        # Initialize robot_name before using it
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.declare_parameter('leader_gps_topic', config.get('leader_gps_topic', f'/{self.robot_name}/smarc/latlon'))
        self.declare_parameter('leader_gps_msg_type', config.get('leader_gps_msg_type', 'GeoPoint'))
        self.declare_parameter('ping_command', config.get('ping_command', config['ping_command']))

        # check from launch parameters if this leader is a "slave"
        self.declare_parameter('is_slave', False)
        self.is_slave = self.get_parameter('is_slave').get_parameter_value().bool_value

        self.leader_gps_msg_type = self.get_parameter('leader_gps_msg_type').get_parameter_value().string_value
        self.ping_command = self.get_parameter('ping_command').get_parameter_value().string_value
        self.sound_velocity = self.get_parameter('sound_velocity').get_parameter_value().double_value
        self.timeout_threshold = self.get_parameter('timeout_threshold').get_parameter_value().double_value
        self.leader_gps_topic = self.get_parameter('leader_gps_topic').get_parameter_value().string_value
        self.timer_period = self.get_parameter('timer_period').get_parameter_value().double_value

        # Dynamically import the message type
        if self.leader_gps_msg_type == 'GeoPoint':
            self.LeaderMsgType = GeoPoint
        elif self.leader_gps_msg_type == 'NavSatFix':
            self.LeaderMsgType = NavSatFix
        else:
            raise ValueError(f"Unsupported message type: {self.leader_gps_msg_type}")

        self.latest_lat = -1
        self.latest_lon = -1
        self._inflight = False
        self.create_subscription(
            self.LeaderMsgType,
            self.leader_gps_topic,
            self.gps_callback,
            10
        )

        # Talk to the modem through the succorfish_driver. In slave mode we also
        # listen to inbound lines so a #B broadcast can trigger a ping.
        self.driver = DriverClient(
            self, on_line=(self._on_rx_line if self.is_slave else None))

        self.get_logger().info(
            f"Initialized InformedLeaderNode with GPS topic: {self.leader_gps_topic} "
            f"and message type: {self.leader_gps_msg_type}")
        if not self.is_slave:
            self.timer = self.create_timer(self.timer_period, self.ping_follower)
        else:
            self.get_logger().info('Slave leader listening for #B broadcasts...')

    def gps_callback(self, msg):
        self.latest_lat = msg.latitude
        self.latest_lon = msg.longitude

    def _on_rx_line(self, line):
        line = line.strip()
        if line.startswith('#B'):
            self.get_logger().info('Received broadcast: ' + line)
            self.ping_follower()

    def ping_follower(self):
        if self._inflight:
            return
        self.get_logger().info(f"Sending ping command: {self.ping_command}")
        self._inflight = True
        self.driver.request(
            self.ping_command,
            expect_regex=PING_REPLY_REGEX,
            timeout=self.timeout_threshold,
            on_result=self._on_ping_result,
        )

    def _on_ping_result(self, resp):
        self._inflight = False
        if resp is None or not resp.success:
            self.get_logger().error("Ping timed out")
            return

        self.get_logger().info(f"Combined Response: {resp.matched_line}")
        try:
            response_str = resp.matched_line.split("T")[1]
            if '$' in response_str:
                self.get_logger().warn("Timeout or malformed response received, ignoring")
                return
            dist = travel_time_to_distance(float(response_str), self.sound_velocity)
            self.get_logger().info(f"Distance: {dist:.3f} meters")
            dist = round(dist, 3)
            if self.latest_lat == -1 or self.latest_lon == -1:
                self.get_logger().error(
                    "Latest GPS position not available, cannot send position+distance")
                return
            msg_str = build_position_broadcast(self.latest_lat, self.latest_lon, dist)
            self.get_logger().info(f"Sending position+distance: {msg_str}")
            self.driver.write(msg_str)
            self.get_logger().info(f"Sent position+distance: {msg_str}")
        except (IndexError, ValueError) as e:
            self.get_logger().error(f"Failed to parse response: {e}")


def main(args=None):
    run_simple_node(InformedLeaderNode, args=args)

if __name__ == '__main__':
    main()
