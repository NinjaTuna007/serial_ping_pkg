"""Acoustic-relay position broadcaster node.

Subscribes to one robot's SMARC navigation topics (lat/lon, depth, heading) and
broadcasts a compact ``$B`` position frame over the acoustic modem on each fresh
latlon update (rate-limited to once every ~3 s). The frame is built by
``acoustic_relay.pos_protocol.build_pos_broadcast`` and is decoded on the other
side by ``smarc_pos_receiver_node``.

Key parameters (serial defaults from ``config/acoustic_relay/acoustic_relay_config.yaml``):
- ``serial.port`` / ``serial.port_fallback`` / ``serial.baudrate`` -- modem link.
- ``robot_name`` -- builds the subscribed topic names (default ``lolo``).

Published topics: none (output goes out over the serial modem).

Subscribed topics:
- ``/<robot_name>/smarc/latlon`` (``geographic_msgs/GeoPoint``) -- triggers a broadcast.
- ``/<robot_name>/smarc/depth`` (``std_msgs/Float32``) -- latest depth, cached.
- ``/<robot_name>/smarc/heading`` (``std_msgs/Float32``) -- latest heading, cached.
"""
import rclpy
from rclpy.node import Node
from geographic_msgs.msg import GeoPoint
from std_msgs.msg import Float32
from serial_ping_pkg.utils import load_yaml_config, init_serial
from serial_ping_pkg.acoustic_relay.pos_protocol import build_pos_broadcast

class SmarcPosBroadcastNode(Node):
    def __init__(self):
        super().__init__('smarc_pos_broadcast_node')

        # Declare parameters with defaults from YAML
        config = load_yaml_config('serial_ping_pkg', 'acoustic_relay/acoustic_relay_config.yaml')
        self.declare_parameter('serial.port', config['serial']['port'])
        self.declare_parameter('serial.port_fallback', config['serial']['port_fallback'])
        self.declare_parameter('serial.baudrate', config['serial']['baudrate'])
        self.declare_parameter('robot_name', 'lolo')

        self.port = self.get_parameter('serial.port').get_parameter_value().string_value
        self.port_fallback = self.get_parameter('serial.port_fallback').get_parameter_value().string_value
        self.baudrate = self.get_parameter('serial.baudrate').get_parameter_value().integer_value
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value

        self.ser = init_serial(self.port, self.port_fallback, self.baudrate, self.get_logger())
        if self.ser is None:
            rclpy.shutdown()
            return

        # Build topic dynamically based on robot name
        input_topic = f"/{self.robot_name}/smarc/latlon"
        self.get_logger().info(f"Subscribing to {input_topic}")
        self.last_msg_time = self.get_clock().now()
        self.subscription = self.create_subscription(
            GeoPoint,
            input_topic,
            self.latlon_callback,
            10
        )

        # Subscribe to depth topic
        depth_topic = f"/{self.robot_name}/smarc/depth"
        self.latest_depth = 0.0
        self.create_subscription(
            Float32,
            depth_topic,
            self.depth_callback,
            10
        )
        self.get_logger().info(f"Subscribing to {depth_topic} for depth")

        # Subscribe to heading topic
        heading_topic = f"/{self.robot_name}/smarc/heading"
        self.latest_heading = 0  # integer degrees
        self.create_subscription(
            Float32,
            heading_topic,
            self.heading_callback,
            10
        )
        self.get_logger().info(f"Subscribing to {heading_topic} for heading")

    def depth_callback(self, msg):
        self.latest_depth = msg.data

    def heading_callback(self, msg):
        self.latest_heading = int(round(msg.data))

    def latlon_callback(self, msg):
        current_time = self.get_clock().now()
        if (current_time - self.last_msg_time).nanoseconds < 3e9:
            return
        self.last_msg_time = current_time
        out_str = build_pos_broadcast(
            msg.latitude, msg.longitude, self.latest_depth, self.latest_heading)
        try:
            self.ser.write((out_str + "\r\n").encode())
            self.get_logger().info(f"Broadcasted: {out_str}")
        except Exception as e:
            self.get_logger().error(f"Serial communication error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SmarcPosBroadcastNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
