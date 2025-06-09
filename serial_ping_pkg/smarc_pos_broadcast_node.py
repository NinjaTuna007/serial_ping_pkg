import rclpy
from rclpy.node import Node
from geographic_msgs.msg import GeoPoint
from std_msgs.msg import Float32
from serial_ping_pkg.utils import load_yaml_config, init_serial

class SmarcPosBroadcastNode(Node):
    def __init__(self):
        super().__init__('smarc_pos_broadcast_node')

        # Declare parameters with defaults from YAML
        config = load_yaml_config('serial_ping_pkg', 'serial_config.yaml')
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
        lat = msg.latitude
        lon = msg.longitude
        data = f"{lat:.8f},{lon:.8f}"
        # Format depth as 5 chars: ,DDD.D
        depth_val = max(0.0, min(self.latest_depth, 999.9))
        depth_str = f",{depth_val:05.1f}"
        # Format heading as 3 chars: ,DDD
        heading_val = max(0, min(self.latest_heading, 359))
        heading_str = f",{heading_val:03d}"
        n_chars = len(data) + len(depth_str) + len(heading_str)
        out_str = f"$B{n_chars}{data}{depth_str}{heading_str}"
        try:
            self.ser.write(out_str.encode())
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