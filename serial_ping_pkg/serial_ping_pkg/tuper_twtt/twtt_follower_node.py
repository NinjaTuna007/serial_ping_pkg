"""TWTT informed-follower node.

Counterpart to ``twtt_leader_node``: reads the modem, decodes the
``lat,lon,distance`` frames the leader transmits (via
``tuper_twtt.leader_protocol.parse_position_distance``), and republishes the
reconstructed leader position and range into the ROS graph.

Key parameters (defaults from ``config/tuper_twtt/tuper_twtt_config.yaml``):
- ``leader_name`` -- builds the published topic names.
- ``leader_gps_topic`` / ``leader_gps_msg_type`` -- output position topic + type
  (``GeoPoint`` or ``NavSatFix``).

The modem link is owned by the ``succorfish_driver`` node; this node never opens
a serial port -- inbound lines arrive via the driver's ``RX`` topic.

Published topics:
- ``leader_gps_topic`` (``geographic_msgs/GeoPoint`` or ``sensor_msgs/NavSatFix``) -- reconstructed leader position.
- ``/<leader_name>/distance`` (``std_msgs/Float32``) -- leader<->follower range (m).

Subscribed topics:
- the driver's ``RX`` topic (``succorfish_msgs/SerialLine``) -- inbound modem lines.
"""
from rclpy.node import Node

from serial_ping_pkg.common.spin import run_simple_node
from geographic_msgs.msg import GeoPoint
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
from serial_ping_pkg.common.driver_client import DriverClient
from serial_ping_pkg.tuper_twtt.leader_protocol import (
    parse_position_distance,
)

class InformedFollowerNode(Node):
    def __init__(self):
        super().__init__('twtt_follower_node')

        self.declare_parameter('leader_name', 'leader')
        self.leader_name = self.get_parameter('leader_name').get_parameter_value().string_value

        # Parameters for leader's GPS topic and message type
        self.declare_parameter('leader_gps_topic', f'/{self.leader_name}/smarc/latlon')
        self.declare_parameter('leader_gps_msg_type', 'GeoPoint')
        self.leader_gps_topic = self.get_parameter('leader_gps_topic').get_parameter_value().string_value
        self.leader_gps_msg_type = self.get_parameter('leader_gps_msg_type').get_parameter_value().string_value

        # Dynamically import the message type
        if self.leader_gps_msg_type == 'GeoPoint':
            self.LeaderMsgType = GeoPoint
        elif self.leader_gps_msg_type == 'NavSatFix':
            self.LeaderMsgType = NavSatFix
        else:
            raise ValueError(f"Unsupported message type: {self.leader_gps_msg_type}")

        # Publishers for leader's position and distance
        self.leader_pos_pub = self.create_publisher(self.LeaderMsgType, self.leader_gps_topic, 10)
        self.leader_dist_pub = self.create_publisher(Float32, f"/{self.leader_name}/distance", 10)

        # Receive modem lines from the succorfish_driver (no direct serial).
        self.driver = DriverClient(self, on_line=self.parse_message)

    def parse_message(self, line):
        line = line.strip()
        if not line:
            return
        self.get_logger().info(f"Received line: {line}")
        # Expecting: <lat>,<lon>,<distance>
        parsed = parse_position_distance(line)
        if parsed is None:
            self.get_logger().warn(f"Malformed message: {line}")
            return
        try:
            lat, lon, dist = parsed
            if self.leader_gps_msg_type == 'GeoPoint':
                pos_msg = GeoPoint()
                pos_msg.latitude = lat
                pos_msg.longitude = lon
                pos_msg.altitude = 0.0
            elif self.leader_gps_msg_type == 'NavSatFix':
                pos_msg = NavSatFix()
                pos_msg.latitude = lat
                pos_msg.longitude = lon
                pos_msg.altitude = 0.0
            self.leader_pos_pub.publish(pos_msg)
            dist_msg = Float32()
            dist_msg.data = dist
            self.leader_dist_pub.publish(dist_msg)
            self.get_logger().info(f"Published leader pos and distance {dist}")
        except Exception as e:
            self.get_logger().warn(f"Failed to parse/publish: {e}")

def main(args=None):
    run_simple_node(InformedFollowerNode, args=args)

if __name__ == '__main__':
    main()
