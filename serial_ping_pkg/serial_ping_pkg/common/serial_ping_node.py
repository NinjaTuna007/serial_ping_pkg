"""Two-leader alternating ping / ranging node.

General-purpose two-way ranging utility: on a 0.5 s timer it alternates pinging
two configured modem addresses (``serial.leader1`` / ``serial.leader2``),
measures the round-trip travel time off the reply, converts it to a range with
``serial.sound_velocity``, and publishes each leader's distance separately.

The modem link is owned by the ``succorfish_driver`` node; this node never opens
a serial port. Each ping is issued as a ``SendCommand`` request (write the ping,
wait for a ``T<ticks>`` reply within ``timeout_threshold``) over the driver's
ROS interface.

Key parameters (defaults from ``config/common/serial_config.yaml``):
- ``serial.leader1`` / ``serial.leader2`` -- the two ping command strings (e.g. ``$P001``).
- ``serial.sound_velocity`` -- m/s used for travel-time -> range.
- ``serial.timeout_threshold`` -- per-ping reply timeout (s).

Published topics:
- ``leader1/distance`` (``std_msgs/Float32``) -- range to leader 1 (m).
- ``leader2/distance`` (``std_msgs/Float32``) -- range to leader 2 (m).

Subscribed topics: none directly (replies arrive via the driver's RX/service).
"""
from rclpy.node import Node

from serial_ping_pkg.common.spin import run_simple_node
from std_msgs.msg import Float32
from serial_ping_pkg.utils import load_yaml_config
from serial_ping_pkg.common.driver_client import DriverClient
from serial_ping_pkg.common.ping_protocol import parse_ping_distance

# A modem reply carries the travel time as ``T`` followed by the tick count; the
# legacy node gated on a ``T`` with >=5 trailing chars, so match that here.
PING_REPLY_REGEX = r'T\d{5,}'


class SerialPingNode(Node):
    def __init__(self):
        super().__init__('serial_ping_node')

        # Load config from YAML and declare parameters
        config = load_yaml_config('serial_ping_pkg', 'common/serial_config.yaml')
        self.declare_parameter('serial.leader1', config['serial']['leader1'])
        self.declare_parameter('serial.leader2', config['serial']['leader2'])
        self.declare_parameter('serial.sound_velocity', config['serial']['sound_velocity'])
        self.declare_parameter('serial.timeout_threshold', config['serial']['timeout_threshold'])

        self.leader1_ping_command = self.get_parameter('serial.leader1').get_parameter_value().string_value
        self.leader2_ping_command = self.get_parameter('serial.leader2').get_parameter_value().string_value
        self.sound_velocity = self.get_parameter('serial.sound_velocity').get_parameter_value().double_value
        self.timeout_threshold = self.get_parameter('serial.timeout_threshold').get_parameter_value().double_value

        self.last_pinged = None
        # One ping in flight at a time, mirroring the original blocking loop.
        self._inflight = False

        # Talk to the modem through the succorfish_driver (no direct serial).
        self.driver = DriverClient(self)

        # Create publishers
        self.distance_1_pub = self.create_publisher(Float32, 'leader1/distance', 10)
        self.distance_2_pub = self.create_publisher(Float32, 'leader2/distance', 10)

        # Timer to run every 0.5 seconds
        self.timer = self.create_timer(0.5, self.alternate_leader_ping)

    def alternate_leader_ping(self):
        if self._inflight:
            # Previous ping has not resolved yet; skip this tick.
            return

        if self.last_pinged is None or self.last_pinged == 2:
            command = self.leader1_ping_command
            leader = 1
        else:
            command = self.leader2_ping_command
            leader = 2
        self.last_pinged = leader

        self.get_logger().info(f"Sending command: {command}")
        self._inflight = True
        self.driver.request(
            command,
            expect_regex=PING_REPLY_REGEX,
            timeout=self.timeout_threshold,
            on_result=lambda resp, leader=leader: self._on_ping_result(leader, resp),
        )

    def _on_ping_result(self, leader, resp):
        self._inflight = False
        if resp is None or not resp.success:
            self.get_logger().error(
                f"Ping timed out; not publishing any distance to Leader {leader}")
            return

        self.get_logger().info(f"Combined Response: {resp.matched_line}")
        try:
            dist = parse_ping_distance(resp.matched_line, self.sound_velocity)
            self.get_logger().info(f"Distance: {dist:.3f} meters")
            msg = Float32()
            msg.data = dist
            if leader == 1:
                self.distance_1_pub.publish(msg)
            else:
                self.distance_2_pub.publish(msg)
        except (IndexError, ValueError) as e:
            self.get_logger().error(f"Failed to parse response: {e}")


def main(args=None):
    run_simple_node(SerialPingNode, args=args)

if __name__ == '__main__':
    main()
