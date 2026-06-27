#!/usr/bin/env python3
"""Single-target periodic ping / ranging node.

General-purpose two-way ranging utility that tracks one robot's position and,
on a fixed timer, pings a single target modem, converts the round-trip travel
time to a range, and publishes it. Position tracking and ranging are decoupled:
a missing position never blocks a ping.

The modem link is owned by the ``succorfish_driver`` node; this node never opens
a serial port. Each ping is issued as a ``SendCommand`` request over the
driver's ROS interface.

Key parameters (defaults from ``config/common/single_target_ping_config.yaml``):
- ``serial.timeout_threshold`` -- per-ping reply timeout (s).
- ``robot_name`` -- builds the subscribed/published topic names.
- ``ping_command`` -- ping command string (e.g. ``$P001``).
- ``timer_period`` -- seconds between pings.
- ``sound_velocity`` -- m/s used for travel-time -> range.
- ``distance_topic_suffix`` -- suffix for the published distance topic.

Published topics:
- ``/<robot_name>/<distance_topic_suffix>`` (``std_msgs/Float32``) -- range (m).

Subscribed topics:
- ``/<robot_name>/smarc/latlon`` (``geographic_msgs/GeoPoint``) -- own position (cached only).
"""
from rclpy.node import Node

from serial_ping_pkg.common.spin import run_simple_node
from geographic_msgs.msg import GeoPoint
from std_msgs.msg import Float32
from serial_ping_pkg.utils import load_yaml_config
from serial_ping_pkg.common.driver_client import DriverClient
from serial_ping_pkg.common.ping_protocol import parse_ping_distance

# Modem reply carries the travel time as ``T`` followed by the tick count.
PING_REPLY_REGEX = r'T\d{5,}'


class SingleTargetPingNode(Node):
    def __init__(self):
        super().__init__('single_target_ping_node')

        # Load config from YAML and declare parameters
        config = load_yaml_config('serial_ping_pkg', 'common/single_target_ping_config.yaml')

        # Serial timing parameter (port/baud now live with succorfish_driver)
        self.declare_parameter('serial.timeout_threshold', config['serial']['timeout_threshold'])

        # Single target ping parameters
        self.declare_parameter('robot_name', config['single_target_ping']['robot_name'])
        self.declare_parameter('ping_command', config['single_target_ping']['ping_command'])
        self.declare_parameter('timer_period', config['single_target_ping']['timer_period'])
        self.declare_parameter('sound_velocity', config['single_target_ping']['sound_velocity'])
        self.declare_parameter('distance_topic_suffix', config['single_target_ping']['distance_topic_suffix'])

        # Get parameter values
        self.timeout_threshold = self.get_parameter('serial.timeout_threshold').get_parameter_value().double_value
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.ping_command = self.get_parameter('ping_command').get_parameter_value().string_value
        self.timer_period = self.get_parameter('timer_period').get_parameter_value().double_value
        self.sound_velocity = self.get_parameter('sound_velocity').get_parameter_value().double_value
        self.distance_topic_suffix = self.get_parameter('distance_topic_suffix').get_parameter_value().string_value

        # Talk to the modem through the succorfish_driver (no direct serial).
        self.driver = DriverClient(self)
        self._inflight = False

        # Position tracking
        self.current_position = None

        # Subscribe to position topic
        position_topic = f"/{self.robot_name}/smarc/latlon"
        self.get_logger().info(f"Subscribing to position topic: {position_topic}")
        self.position_subscription = self.create_subscription(
            GeoPoint,
            position_topic,
            self.position_callback,
            10
        )

        # Create distance publisher
        distance_topic = f"/{self.robot_name}/{self.distance_topic_suffix}"
        self.get_logger().info(f"Publishing distance to: {distance_topic}")
        self.distance_publisher = self.create_publisher(Float32, distance_topic, 10)

        # Create timer for periodic pinging
        self.ping_timer = self.create_timer(self.timer_period, self.ping_callback)

        self.get_logger().info("Single Target Ping Node initialized:")
        self.get_logger().info(f"  - Robot name: {self.robot_name}")
        self.get_logger().info(f"  - Ping command: {self.ping_command}")
        self.get_logger().info(f"  - Timer period: {self.timer_period}s")
        self.get_logger().info(f"  - Sound velocity: {self.sound_velocity} m/s")

    def position_callback(self, msg):
        """Callback to update current position from latlon topic"""
        self.current_position = msg
        self.get_logger().debug(f"Position updated: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}")

    def ping_callback(self):
        """Timer callback to periodically ping the target"""
        if self._inflight:
            return

        # Log current position status for debugging, but don't block pinging
        if self.current_position is None:
            self.get_logger().debug("No position data available yet, but continuing with ping")
        else:
            self.get_logger().debug(
                f"Current position: lat={self.current_position.latitude:.6f}, "
                f"lon={self.current_position.longitude:.6f}")

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
            self.get_logger().error("Ping timed out; not publishing distance")
            return

        self.get_logger().info(f"Ping response: {resp.matched_line}")
        try:
            dist = parse_ping_distance(resp.matched_line, self.sound_velocity)
            self.get_logger().info(f"Calculated distance: {dist:.3f} meters")
            msg = Float32()
            msg.data = dist
            self.distance_publisher.publish(msg)
        except (IndexError, ValueError) as e:
            self.get_logger().error(f"Failed to parse ping response: {e}")


def main(args=None):
    run_simple_node(SingleTargetPingNode, args=args)

if __name__ == '__main__':
    main()
