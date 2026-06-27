"""General-purpose leader-broadcast receiver node.

Reads leader broadcast frames off the modem, decodes ``leader_id,lat,lon,dist``
via ``common.ping_protocol.parse_leader_broadcast``, and republishes each
leader's GPS fix and range into the ROS graph. Publishers are created lazily,
one pair per leader id seen on the wire.

The modem link is owned by the ``succorfish_driver`` node; this node never opens
a serial port. Inbound lines arrive via the driver's ``RX`` topic.

Published topics (per leader id seen):
- ``/leader<id>/gps`` (``sensor_msgs/NavSatFix``) -- reconstructed leader position.
- ``/leader<id>/distance`` (``std_msgs/Float32``) -- range to that leader (m).

Subscribed topics:
- the driver's ``RX`` topic (``succorfish_msgs/SerialLine``) -- inbound modem lines.
"""
from rclpy.node import Node

from serial_ping_pkg.common.spin import run_simple_node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from serial_ping_pkg.common.driver_client import DriverClient
from serial_ping_pkg.common.ping_protocol import parse_leader_broadcast


class SerialBroadcastReceiver(Node):
    def __init__(self):
        super().__init__('serial_broadcast_receiver')

        # Receive modem lines from the succorfish_driver (no direct serial).
        self.driver = DriverClient(self, on_line=self.parse_message)

        self.gps_pubs = {}   # Key: leader ID, Value: GPS publisher
        self.dist_pubs = {}  # Key: leader ID, Value: Distance publisher

    def parse_message(self, line):
        line = line.strip()
        if not line:
            return
        self.get_logger().info(f"Received line: {line}")

        parsed = parse_leader_broadcast(line)
        if parsed is None:
            self.get_logger().warn(f"Invalid or malformed leader broadcast: {line}")
            return

        try:
            leader_id, lat, lon, dist = parsed

            gps_topic = f'/leader{leader_id}/gps'
            dist_topic = f'/leader{leader_id}/distance'

            # Create publishers if not already created
            if leader_id not in self.gps_pubs:
                self.gps_pubs[leader_id] = self.create_publisher(NavSatFix, gps_topic, 10)
                self.dist_pubs[leader_id] = self.create_publisher(Float32, dist_topic, 10)
                self.get_logger().info(f"Created publishers for leader {leader_id}")

            # Publish GPS
            gps_msg = NavSatFix()
            gps_msg.header.stamp = self.get_clock().now().to_msg()
            gps_msg.header.frame_id = 'gps'
            gps_msg.latitude = lat
            gps_msg.longitude = lon
            gps_msg.altitude = 0.0
            self.gps_pubs[leader_id].publish(gps_msg)
            self.get_logger().info(f"Published GPS to {gps_topic}: lat={lat}, lon={lon}")

            # Publish distance
            dist_msg = Float32()
            dist_msg.data = dist
            self.dist_pubs[leader_id].publish(dist_msg)
            self.get_logger().info(f"Published distance to {dist_topic}: {dist}")

        except Exception as e:
            self.get_logger().error(f"Failed to parse line '{line}': {e}")


def main(args=None):
    run_simple_node(SerialBroadcastReceiver, args=args)


if __name__ == '__main__':
    main()
