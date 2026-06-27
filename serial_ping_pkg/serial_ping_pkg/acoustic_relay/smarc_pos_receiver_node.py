"""Acoustic-relay position receiver node.

Reads ``#B<modem-id(3)><num_chars(2)><data>`` broadcast frames off the modem,
decodes them via ``acoustic_relay.pos_protocol.parse_pos_broadcast``, and
republishes each sending robot's position (and depth/heading when present) onto
its own ROS topic. The robot table is supplied as index-aligned parallel arrays
so it is fully overridable from the launch CLI.

Key parameters (defaults from ``config/acoustic_relay/acoustic_relay_config.yaml``):
- ``robots.names`` / ``robots.modem_ids`` / ``robots.topics`` -- parallel arrays
  (``names[i]`` <-> ``modem_ids[i]`` <-> ``topics[i]``); modem ids are ints,
  zero-padded to 3 digits on use; a blank topic falls back to
  ``/relay_<name>/smarc/latlon``.

The modem link is owned by the ``succorfish_driver`` node; this node never opens
a serial port -- inbound lines arrive via the driver's ``RX`` topic.

Published topics (per configured robot):
- ``<topics[i]>`` (``geographic_msgs/GeoPoint``) -- received position.
- ``/relay_<name>/smarc/depth`` and ``/relay_<name>/smarc/heading``
  (``std_msgs/Float32``) -- when the frame carries them.

Subscribed topics:
- the driver's ``RX`` topic (``succorfish_msgs/SerialLine``) -- inbound modem lines.
"""

from rclpy.node import Node

from serial_ping_pkg.common.spin import run_simple_node
from geographic_msgs.msg import GeoPoint
from serial_ping_pkg.utils import load_yaml_config
from serial_ping_pkg.common.driver_client import DriverClient
from serial_ping_pkg.acoustic_relay.pos_protocol import parse_pos_broadcast

class SmarcPosReceiverNode(Node):
    def __init__(self):
        super().__init__('smarc_pos_receiver_node')

        # Load config from YAML (used only as defaults for the declared params)
        config = load_yaml_config('serial_ping_pkg', 'acoustic_relay/acoustic_relay_config.yaml')
        pos_receiver_cfg = config.get('pos_receiver', {})

        # Robot table as index-aligned parallel arrays so it is fully overridable
        # from the launch CLI (names[i] <-> modem_ids[i] <-> topics[i]). Modem ids
        # are integers and zero-padded to three digits on use; a blank topic falls
        # back to /relay_<name>/smarc/latlon.
        default_names = list(pos_receiver_cfg.get('names', ['lolo', 'sam']))
        default_modem_ids = [int(m) for m in pos_receiver_cfg.get('modem_ids', [7, 111])]
        default_topics = list(pos_receiver_cfg.get('topics', [])) or [
            f"/relay_{name}/smarc/latlon" for name in default_names
        ]
        self.declare_parameter('robots.names', default_names)
        self.declare_parameter('robots.modem_ids', default_modem_ids)
        self.declare_parameter('robots.topics', default_topics)

        # Build robot configuration from the parallel-array parameters.
        names = list(self.get_parameter('robots.names').get_parameter_value().string_array_value)
        modem_ids = list(self.get_parameter('robots.modem_ids').get_parameter_value().integer_array_value)
        topics = list(self.get_parameter('robots.topics').get_parameter_value().string_array_value)

        if len(names) != len(modem_ids):
            self.get_logger().error(
                f"robots.names ({len(names)}) and robots.modem_ids ({len(modem_ids)}) "
                "must be the same length; no robots configured."
            )
            names, modem_ids = [], []

        self.robot_config = {}
        self.robot_publishers = {}
        for i, robot_name in enumerate(names):
            modem_id = str(modem_ids[i]).zfill(3)
            topic = topics[i] if i < len(topics) and topics[i] else f"/relay_{robot_name}/smarc/latlon"
            self.robot_config[modem_id] = {'name': robot_name, 'topic': topic}
            self.robot_publishers[modem_id] = self.create_publisher(GeoPoint, topic, 10)
            self.get_logger().info(f"Configured robot {robot_name} with modem ID {modem_id} -> topic {topic}")

        # Receive modem lines from the succorfish_driver (no direct serial).
        self.driver = DriverClient(self, on_line=self.parse_message)

    def parse_message(self, line):
        line = line.strip()
        if not line:
            return
        self.get_logger().info(f"Received line: {line}")
        # Expecting: #B<modem-id(3)><num_chars(2)><data>
        parsed = parse_pos_broadcast(line)
        if parsed is None:
            self.get_logger().warn(f"Invalid or malformed position broadcast: {line}")
            return
        modem_id, lat, lon, depth, heading = parsed
        try:
            msg = GeoPoint()
            msg.latitude = lat
            msg.longitude = lon
            msg.altitude = 0.0

            # Use dynamic robot configuration
            if modem_id in self.robot_publishers:
                robot_info = self.robot_config[modem_id]
                self.robot_publishers[modem_id].publish(msg)
                self.get_logger().info(
                    f"[{self.get_name()}] Published GeoPoint to {robot_info['topic']} from modem {modem_id} ({robot_info['name']}): lat={msg.latitude}, lon={msg.longitude}"
                )
                # Publish depth if present
                if depth is not None:
                    try:
                        depth_val = depth
                        depth_topic = f"/relay_{robot_info['name']}/smarc/depth"
                        depth_pub = self.create_publisher(__import__('std_msgs.msg', fromlist=['Float32']).Float32, depth_topic, 10)
                        from std_msgs.msg import Float32
                        depth_msg = Float32()
                        depth_msg.data = depth_val
                        depth_pub.publish(depth_msg)
                        self.get_logger().info(f"Published depth {depth_val} to {depth_topic}")
                    except Exception as e:
                        self.get_logger().warn(f"Failed to parse/publish depth: {e}")
                # Publish heading if present
                if heading is not None:
                    try:
                        heading_val = heading
                        heading_topic = f"/relay_{robot_info['name']}/smarc/heading"
                        heading_pub = self.create_publisher(__import__('std_msgs.msg', fromlist=['Float32']).Float32, heading_topic, 10)
                        from std_msgs.msg import Float32
                        heading_msg = Float32()
                        heading_msg.data = float(heading_val)
                        heading_pub.publish(heading_msg)
                        self.get_logger().info(f"Published heading {heading_val} to {heading_topic}")
                    except Exception as e:
                        self.get_logger().warn(f"Failed to parse/publish heading: {e}")
            else:
                self.get_logger().warn(f"Unknown modem_id {modem_id}, not publishing.")

        except Exception as e:
            self.get_logger().error(f"Failed to parse data '{line}': {e}")

def main(args=None):
    run_simple_node(SmarcPosReceiverNode, args=args)

if __name__ == '__main__':
    main()
