# Message format: #B<modem-id(3)><num_chars(2)><data>\r\n

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geographic_msgs.msg import GeoPoint
import serial
import yaml
import os

class SmarcPosReceiverNode(Node):
    def __init__(self):
        super().__init__('smarc_pos_receiver_node')

        # Load config file
        config = self.load_config()
        self.port = config['serial']['port']
        self.port_fallback = config['serial']['port_fallback']
        self.baudrate = config['serial']['baudrate']

        # Initialize serial
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            if self.ser.is_open:
                self.get_logger().info(f"Serial port {self.port} opened at {self.baudrate} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {self.port}: {e}")
            self.get_logger().info(f"Trying fallback port: {self.port_fallback}")
            try:
                self.ser = serial.Serial(self.port_fallback, self.baudrate, timeout=1)
                if self.ser.is_open:
                    self.get_logger().info(f"Serial port {self.port_fallback} opened at {self.baudrate} baud")
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to open fallback serial port {self.port_fallback}: {e}")
                self.get_logger().error("No serial port available. Exiting node.")
                rclpy.shutdown()
                return

        # Publishers for GeoPoint
        self.lolo_pub = self.create_publisher(GeoPoint, '/lolo/smarc_latlon', 10)
        self.sam_pub = self.create_publisher(GeoPoint, '/sam/smarc_latlon', 10)

        # Timer to check serial port regularly
        self.timer = self.create_timer(0.5, self.read_serial)

        # Buffer for incomplete lines
        self.buffer = ""

    def load_config(self):
        config_path = os.path.join(
            get_package_share_directory('serial_ping_pkg'),
            'config',
            'serial_config.yaml'
        )
        with open(config_path, 'r') as file:
            return yaml.safe_load(file)

    def read_serial(self):
        try:
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                self.buffer += data
                while '\r\n' in self.buffer:
                    line, self.buffer = self.buffer.split('\r\n', 1)
                    self.parse_message(line)
        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")

    def parse_message(self, line):
        # Expecting: #B<modem-id(3)><num_chars(2)><data>
        if not line.startswith('#B'):
            return
        if len(line) < 7:  # #B + 3 (id) + 2 (len) + at least 1 char data
            self.get_logger().warn(f"Line too short: {line}")
            return
        modem_id = line[2:5]
        num_chars_str = line[5:7]
        try:
            num_chars = int(num_chars_str)
        except ValueError:
            self.get_logger().warn(f"Invalid num_chars: {num_chars_str} in line: {line}")
            return
        data = line[7:7+num_chars]
        if len(data) != num_chars:
            self.get_logger().warn(f"Data length mismatch in line: {line}")
            return
        # Data expected as <lat>,<lon>
        try:
            lat_str, lon_str = data.split(',')
            msg = GeoPoint()
            msg.latitude = float(lat_str)
            msg.longitude = float(lon_str)
            msg.altitude = 0.0

            if modem_id == "007":
                self.lolo_pub.publish(msg)
                self.get_logger().info(f"Published GeoPoint to /lolo/smarc_latlon from modem {modem_id}: lat={msg.latitude}, lon={msg.longitude}")
            elif modem_id == "069":
                self.sam_pub.publish(msg)
                self.get_logger().info(f"Published GeoPoint to /sam/smarc_latlon from modem {modem_id}: lat={msg.latitude}, lon={msg.longitude}")
            else:
                self.get_logger().warn(f"Unknown modem_id {modem_id}, not publishing.")

        except Exception as e:
            self.get_logger().error(f"Failed to parse data '{data}': {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SmarcPosReceiverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()