import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geographic_msgs.msg import GeoPoint
import serial
import yaml
import os

class SmarcPosBroadcastNode(Node):
    def __init__(self):
        super().__init__('smarc_pos_broadcast_node')

        # Load config file
        config = self.load_config()
        self.port = config['serial']['port']
        self.port_fallback = config['serial']['port_fallback']
        self.baudrate = config['serial']['baudrate']
        self.last_msg_time = self.get_clock().now()

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

        # Subscribe to GeoPoint topic
        self.subscription = self.create_subscription(
            GeoPoint,
            '/lolo/smarc/latlon',
            self.latlon_callback,
            10
        )

    def load_config(self):
        config_path = os.path.join(
            get_package_share_directory('serial_ping_pkg'),
            'config',
            'serial_config.yaml'
        )
        with open(config_path, 'r') as file:
            return yaml.safe_load(file)

    def latlon_callback(self, msg):

        # timeout to avoid flooding the serial port
        current_time = self.get_clock().now()
        if (current_time - self.last_msg_time).nanoseconds < 3e9:
            # self.get_logger().info("Ignoring message to avoid flooding serial port")
            return

        # Update last message time
        self.last_msg_time = current_time

        # Format: <lat,lon> in degrees, double precision
        lat = msg.latitude
        lon = msg.longitude
        data = f"{lat:.8f},{lon:.8f}"
        n_chars = len(data)
        out_str = f"$B{n_chars}{data}"
        try:
            self.ser.write(out_str.encode())
            self.get_logger().info(f"Broadcasted: {out_str}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SmarcPosBroadcastNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()