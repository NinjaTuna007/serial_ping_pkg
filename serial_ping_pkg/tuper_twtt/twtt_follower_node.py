"""TWTT informed-follower node.

Counterpart to ``twtt_leader_node``: reads the modem, decodes the
``lat,lon,distance`` frames the leader transmits (via
``tuper_twtt.leader_protocol.parse_position_distance``), and republishes the
reconstructed leader position and range into the ROS graph.

Key parameters (defaults from ``config/tuper_twtt/tuper_twtt_config.yaml``):
- ``serial.port`` / ``serial.port_fallback`` / ``serial.baudrate`` -- modem link.
- ``leader_name`` -- builds the published topic names.
- ``leader_gps_topic`` / ``leader_gps_msg_type`` -- output position topic + type
  (``GeoPoint`` or ``NavSatFix``).

Published topics:
- ``leader_gps_topic`` (``geographic_msgs/GeoPoint`` or ``sensor_msgs/NavSatFix``) -- reconstructed leader position.
- ``/<leader_name>/distance`` (``std_msgs/Float32``) -- leader<->follower range (m).

Subscribed topics: none (input arrives over the serial modem).
"""
import rclpy
from rclpy.node import Node
from geographic_msgs.msg import GeoPoint
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
from serial_ping_pkg.utils import load_yaml_config, init_serial
from serial_ping_pkg.tuper_twtt.leader_protocol import (
    parse_position_distance,
)

class InformedFollowerNode(Node):
    def __init__(self):
        super().__init__('twtt_follower_node')

        config = load_yaml_config('serial_ping_pkg', 'tuper_twtt/tuper_twtt_config.yaml')
        self.declare_parameter('serial.port', config['serial']['port'])
        self.declare_parameter('serial.port_fallback', config['serial']['port_fallback'])
        self.declare_parameter('serial.baudrate', config['serial']['baudrate'])
        self.declare_parameter('leader_name', 'leader')

        self.port = self.get_parameter('serial.port').get_parameter_value().string_value
        self.port_fallback = self.get_parameter('serial.port_fallback').get_parameter_value().string_value
        self.baudrate = self.get_parameter('serial.baudrate').get_parameter_value().integer_value
        self.leader_name = self.get_parameter('leader_name').get_parameter_value().string_value

        self.ser = init_serial(self.port, self.port_fallback, self.baudrate, self.get_logger())
        if self.ser is None:
            rclpy.shutdown()
            return

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

        self.timer = self.create_timer(0.5, self.read_serial)
        self.buffer = ""

    def read_serial(self):
        try:
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                self.buffer += data
                while '\r\n' in self.buffer:
                    line, self.buffer = self.buffer.split('\r\n', 1)
                    self.parse_message(line)
        except Exception as e:
            self.get_logger().error(f"Serial communication error: {e}")

    def parse_message(self, line):
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
    rclpy.init(args=args)
    node = InformedFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
