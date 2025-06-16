# Message format: #B<modem-id(3)><num_chars(2)><data>\r\n

import rclpy
from rclpy.node import Node
from geographic_msgs.msg import GeoPoint
from serial_ping_pkg.utils import init_serial, load_yaml_config

class SmarcPosReceiverNode(Node):
    def __init__(self):
        super().__init__('smarc_pos_receiver_node')

        # Load config from YAML
        config = load_yaml_config('serial_ping_pkg', 'serial_config.yaml')
        pos_receiver_cfg = config.get('pos_receiver', {})
        serial_cfg = config.get('serial', {})

        self.port = serial_cfg.get('port', '/dev/ttyUSB0')
        self.port_fallback = serial_cfg.get('port_fallback', '/dev/ttyUSB1')
        self.baudrate = serial_cfg.get('baudrate', 9600)

        self.ser = init_serial(self.port, self.port_fallback, self.baudrate, self.get_logger())
        if self.ser is None:
            rclpy.shutdown()
            return

        # Build robot configuration from config file
        self.robot_config = {}
        self.robot_publishers = {}
        robots = pos_receiver_cfg.get('robots', {})
        for robot_name, robot_info in robots.items():
            modem_id = str(robot_info.get('modem_id')).zfill(3)
            topic = robot_info.get('topic', f"/relay_{robot_name}/smarc/latlon")
            self.robot_config[modem_id] = {'name': robot_name, 'topic': topic}
            self.robot_publishers[modem_id] = self.create_publisher(GeoPoint, topic, 10)
            self.get_logger().info(f"Configured robot {robot_name} with modem ID {modem_id} -> topic {topic}")

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
        # Expecting: #B<modem-id(3)><num_chars(2)><data>
        if not line.startswith('#B'):
            return
        if len(line) < 7:
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
        try:
            # Split data into lat, lon, depth, heading
            parts = data.split(',')
            lat_str, lon_str = parts[0], parts[1]
            depth_str = None
            heading_str = None
            if len(parts) == 4:
                depth_str = parts[2]
                heading_str = parts[3]
            elif len(parts) == 3:
                # If only depth or heading is present, assume it's depth
                depth_str = parts[2]
            msg = GeoPoint()
            msg.latitude = float(lat_str)
            msg.longitude = float(lon_str)
            msg.altitude = 0.0

            # Use dynamic robot configuration
            if modem_id in self.robot_publishers:
                robot_info = self.robot_config[modem_id]
                self.robot_publishers[modem_id].publish(msg)
                self.get_logger().info(
                    f"[{self.get_name()}] Published GeoPoint to {robot_info['topic']} from modem {modem_id} ({robot_info['name']}): lat={msg.latitude}, lon={msg.longitude}"
                )
                # Publish depth if present
                if depth_str is not None:
                    try:
                        depth_val = float(depth_str)
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
                if heading_str is not None:
                    try:
                        heading_val = int(heading_str)
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
            self.get_logger().error(f"Failed to parse data '{data}': {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SmarcPosReceiverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()