# Message format: #B<modem-id(3)><num_chars(2)><data>\r\n

import rclpy
from rclpy.node import Node
from geographic_msgs.msg import GeoPoint
from serial_ping_pkg.utils import init_serial

class SmarcPosReceiverNode(Node):
    def __init__(self):
        super().__init__('smarc_pos_receiver_node')

        # Declare parameters with defaults - no config file dependency
        self.declare_parameter('serial.port', '/dev/ttyUSB0')
        self.declare_parameter('serial.port_fallback', '/dev/ttyUSB1')
        self.declare_parameter('serial.baudrate', 9600)
        
        # Robot configuration parameters - allow any type for modem_id
        self.declare_parameter('robots.lolo.modem_id')
        self.declare_parameter('robots.sam.modem_id')
        self.declare_parameter('robots.lolo.name', 'lolo')
        self.declare_parameter('robots.sam.name', 'sam')

        self.port = self.get_parameter('serial.port').get_parameter_value().string_value
        self.port_fallback = self.get_parameter('serial.port_fallback').get_parameter_value().string_value
        self.baudrate = self.get_parameter('serial.baudrate').get_parameter_value().integer_value

        self.ser = init_serial(self.port, self.port_fallback, self.baudrate, self.get_logger())
        if self.ser is None:
            rclpy.shutdown()
            return

        # Build robot configuration dynamically
        self.robot_config = {}
        self.robot_publishers = {}
        
        # Configure lolo - handle both string and numeric parameter types
        lolo_modem_param = self.get_parameter('robots.lolo.modem_id').get_parameter_value()
        if lolo_modem_param.type == 1:  # STRING
            lolo_modem_id = lolo_modem_param.string_value
        elif lolo_modem_param.type == 2:  # INTEGER  
            lolo_modem_id = f"{lolo_modem_param.integer_value:03d}"
        elif lolo_modem_param.type == 3:  # DOUBLE
            lolo_modem_id = f"{int(lolo_modem_param.double_value):03d}"
        else:
            lolo_modem_id = str(lolo_modem_param.string_value)
            
        lolo_name = self.get_parameter('robots.lolo.name').get_parameter_value().string_value
        lolo_topic = f"/relay_{lolo_name}/smarc/latlon"
        self.robot_config[lolo_modem_id] = {'name': lolo_name, 'topic': lolo_topic}
        self.robot_publishers[lolo_modem_id] = self.create_publisher(GeoPoint, lolo_topic, 10)
        self.get_logger().info(f"Configured robot {lolo_name} with modem ID {lolo_modem_id} -> topic {lolo_topic}")
        
        # Configure sam - handle both string and numeric parameter types
        sam_modem_param = self.get_parameter('robots.sam.modem_id').get_parameter_value()
        if sam_modem_param.type == 1:  # STRING
            sam_modem_id = sam_modem_param.string_value
        elif sam_modem_param.type == 2:  # INTEGER
            sam_modem_id = f"{sam_modem_param.integer_value:03d}"
        elif sam_modem_param.type == 3:  # DOUBLE
            sam_modem_id = f"{int(sam_modem_param.double_value):03d}"
        else:
            sam_modem_id = str(sam_modem_param.string_value)
            
        sam_name = self.get_parameter('robots.sam.name').get_parameter_value().string_value
        sam_topic = f"/relay_{sam_name}/smarc/latlon"
        self.robot_config[sam_modem_id] = {'name': sam_name, 'topic': sam_topic}
        self.robot_publishers[sam_modem_id] = self.create_publisher(GeoPoint, sam_topic, 10)
        self.get_logger().info(f"Configured robot {sam_name} with modem ID {sam_modem_id} -> topic {sam_topic}")

        # Timer to check serial port regularly
        self.timer = self.create_timer(0.5, self.read_serial)

        # Buffer for incomplete lines
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