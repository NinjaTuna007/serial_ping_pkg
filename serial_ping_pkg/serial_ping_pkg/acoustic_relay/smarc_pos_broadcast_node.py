"""Acoustic-relay position broadcaster node.

Subscribes to one robot's SMARC navigation topics (lat/lon, depth, heading) and
broadcasts a compact ``$B`` position frame over the acoustic modem on each fresh
latlon update (rate-limited to once every ~3 s). The frame is built by
``acoustic_relay.pos_protocol.build_pos_broadcast`` and is decoded on the other
side by ``smarc_pos_receiver_node``.

The modem link is owned by the ``succorfish_driver`` node; this node never opens
a serial port -- it publishes each frame to the driver's ``TX`` topic.

Key parameters:
- ``robot_name`` -- builds the subscribed topic names (default ``lolo``).

Published topics: none on ROS (frames go out over the driver's ``TX`` topic).

Subscribed topics:
- ``/<robot_name>/smarc/latlon`` (``geographic_msgs/GeoPoint``) -- triggers a broadcast.
- ``/<robot_name>/smarc/depth`` (``std_msgs/Float32``) -- latest depth, cached.
- ``/<robot_name>/smarc/heading`` (``std_msgs/Float32``) -- latest heading, cached.
"""
from rclpy.node import Node

from serial_ping_pkg.common.spin import run_simple_node
from geographic_msgs.msg import GeoPoint
from std_msgs.msg import Float32
from serial_ping_pkg.common.driver_client import DriverClient
from serial_ping_pkg.acoustic_relay.pos_protocol import build_pos_broadcast

class SmarcPosBroadcastNode(Node):
    def __init__(self):
        super().__init__('smarc_pos_broadcast_node')

        self.declare_parameter('robot_name', 'lolo')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value

        # Broadcast frames through the succorfish_driver (no direct serial).
        self.driver = DriverClient(self)

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
            self.driver.write(out_str)
            self.get_logger().info(f"Broadcasted: {out_str}")
        except Exception as e:
            self.get_logger().error(f"Failed to broadcast: {e}")

def main(args=None):
    run_simple_node(SmarcPosBroadcastNode, args=args)

if __name__ == '__main__':
    main()
