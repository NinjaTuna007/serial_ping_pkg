import rclpy
from rclpy.node import Node
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from serial_ping_pkg.utils import load_yaml_config, init_serial
import time

class InformedLeaderNode(Node):
    def __init__(self):
        super().__init__('informed_leader_node')

        config = load_yaml_config('serial_ping_pkg', 'serial_config.yaml')
        self.declare_parameter('serial.port', config['serial']['port'])
        self.declare_parameter('serial.port_fallback', config['serial']['port_fallback'])
        self.declare_parameter('serial.baudrate', config['serial']['baudrate'])
        self.declare_parameter('robot_name', 'leader')
        self.declare_parameter('ping_command', 'PING')
        self.declare_parameter('sound_velocity', 1500.0)
        self.declare_parameter('timeout_threshold', 2.0)
        self.declare_parameter('leader_gps_topic', f'/{self.robot_name}/smarc/latlon')
        self.declare_parameter('leader_gps_msg_type', 'GeoPoint')
        self.leader_gps_msg_type = self.get_parameter('leader_gps_msg_type').get_parameter_value().string_value

        self.port = self.get_parameter('serial.port').get_parameter_value().string_value
        self.port_fallback = self.get_parameter('serial.port_fallback').get_parameter_value().string_value
        self.baudrate = self.get_parameter('serial.baudrate').get_parameter_value().integer_value
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.ping_command = self.get_parameter('ping_command').get_parameter_value().string_value
        self.sound_velocity = self.get_parameter('sound_velocity').get_parameter_value().double_value
        self.timeout_threshold = self.get_parameter('timeout_threshold').get_parameter_value().double_value
        self.leader_gps_topic = self.get_parameter('leader_gps_topic').get_parameter_value().string_value

        self.ser = init_serial(self.port, self.port_fallback, self.baudrate, self.get_logger())
        if self.ser is None:
            rclpy.shutdown()
            return

        # Dynamically import the message type
        if self.leader_gps_msg_type == 'GeoPoint':
            self.LeaderMsgType = GeoPoint
        elif self.leader_gps_msg_type == 'NavSatFix':
            self.LeaderMsgType = NavSatFix
        else:
            raise ValueError(f"Unsupported message type: {self.leader_gps_msg_type}")

        self.latest_lat = 0.0
        self.latest_lon = 0.0
        self.create_subscription(
            self.LeaderMsgType,
            self.leader_gps_topic,
            self.gps_callback,
            10
        )

        self.timer = self.create_timer(1.0, self.ping_follower)

    def gps_callback(self, msg):
        if self.leader_gps_msg_type == 'GeoPoint':
            self.latest_lat = msg.latitude
            self.latest_lon = msg.longitude
        elif self.leader_gps_msg_type == 'NavSatFix':
            self.latest_lat = msg.latitude
            self.latest_lon = msg.longitude
        # Add more types as needed

    def ping_follower(self):
        self.get_logger().info(f"Sending ping command: {self.ping_command}")
        try:
            ping_time = time.time()
            self.ser.write(self.ping_command.encode())
            read_so_far = ""
            while True:
                if time.time() - ping_time > self.timeout_threshold:
                    self.get_logger().error("Ping timed out")
                    return
                response = self.ser.read(self.ser.in_waiting or 1)
                read_now = response.decode('utf-8').strip()
                read_so_far += read_now
                if "T" in read_so_far:
                    if len(read_so_far.split("T")[1]) >= 5:
                        break
            self.get_logger().info(f"Combined Response: {read_so_far}")
            try:
                response_str = read_so_far.split("T")[1]
                dist = 0.00003125 * self.sound_velocity * float(response_str)
                self.get_logger().info(f"Distance: {dist:.3f} meters")
                # Construct message: <lat>,<lon>,<distance>
                msg_str = f"{self.latest_lat:.8f},{self.latest_lon:.8f},{dist:.2f}\r\n"
                self.ser.write(msg_str.encode())
                self.get_logger().info(f"Sent position+distance: {msg_str.strip()}")
            except (IndexError, ValueError) as e:
                self.get_logger().error(f"Failed to parse response: {e}")
        except Exception as e:
            self.get_logger().error(f"Serial communication error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = InformedLeaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
