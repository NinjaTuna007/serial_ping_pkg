"""General-purpose leader-broadcast receiver node.

Reads leader broadcast frames off the modem, decodes ``leader_id,lat,lon,dist``
via ``common.ping_protocol.parse_leader_broadcast``, and republishes each
leader's GPS fix and range into the ROS graph. Publishers are created lazily,
one pair per leader id seen on the wire.

Key parameters:
- ``port`` -- serial device (default ``/dev/ttyUSB0``).
- ``baudrate`` -- modem baudrate (default ``9600``).

Published topics (per leader id seen):
- ``/leader<id>/gps`` (``sensor_msgs/NavSatFix``) -- reconstructed leader position.
- ``/leader<id>/distance`` (``std_msgs/Float32``) -- range to that leader (m).

Subscribed topics: none (input arrives over the serial modem).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from serial_ping_pkg.common.ping_protocol import parse_leader_broadcast
import serial


class SerialBroadcastReceiver(Node):
    def __init__(self):
        super().__init__('serial_broadcast_receiver')

        # Declare and get 'port' and 'baudrate' parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 9600)
        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.buffer = ""
        self.timer = self.create_timer(0.5, self.read_serial)

        self.gps_pubs = {}   # Key: leader ID, Value: GPS publisher
        self.dist_pubs = {}  # Key: leader ID, Value: Distance publisher

    def read_serial(self):
        try:
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                self.buffer += data
                while '\r\n' in self.buffer:
                    line, self.buffer = self.buffer.split('\r\n', 1)
                    self.parse_message(line.strip())
        except Exception as e:
            self.get_logger().error(f"Serial read error: {e}")

    def parse_message(self, line):
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
    rclpy.init(args=args)
    node = SerialBroadcastReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        node.ser.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()