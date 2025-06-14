import rclpy
from rclpy.node import Node
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from serial_ping_pkg.utils import load_yaml_config, init_serial
import time
import threading

class InformedLeaderNode(Node):
    def __init__(self):
        super().__init__('informed_leader_node')

        config = load_yaml_config('serial_ping_pkg', 'serial_config.yaml')
        self.declare_parameter('serial.port', config['serial']['port'])
        self.declare_parameter('serial.port_fallback', config['serial']['port_fallback'])
        self.declare_parameter('serial.baudrate', config['serial']['baudrate'])
        self.declare_parameter('robot_name', 'leader')
        self.declare_parameter('sound_velocity', config['serial'].get('sound_velocity', 1500.0))
        self.declare_parameter('timeout_threshold', config['serial'].get('timeout_threshold', 2.0))
        self.declare_parameter('timer_period', config.get('timer_period', 3.0))

        # Initialize robot_name before using it
        self.robot_name = self.get_parameter( 'robot_name').get_parameter_value().string_value
        self.declare_parameter('leader_gps_topic', config.get('leader_gps_topic', f'/{self.robot_name}/smarc/latlon'))
        self.declare_parameter('leader_gps_msg_type', config.get('leader_gps_msg_type', 'GeoPoint'))
        self.declare_parameter('ping_command', config.get('ping_command', config['ping_command']))

        # check from launch parameters if this leader is a "slave"
        self.declare_parameter('is_slave', False)
        self.is_slave = self.get_parameter('is_slave').get_parameter_value().bool_value


        self.leader_gps_msg_type = self.get_parameter('leader_gps_msg_type').get_parameter_value().string_value
        self.ping_command = self.get_parameter('ping_command').get_parameter_value().string_value
        self.port = self.get_parameter('serial.port').get_parameter_value().string_value
        self.port_fallback = self.get_parameter('serial.port_fallback').get_parameter_value().string_value
        self.baudrate = self.get_parameter('serial.baudrate').get_parameter_value().integer_value
        self.sound_velocity = self.get_parameter('sound_velocity').get_parameter_value().double_value
        self.timeout_threshold = self.get_parameter('timeout_threshold').get_parameter_value().double_value
        self.leader_gps_topic = self.get_parameter('leader_gps_topic').get_parameter_value().string_value
        self.timer_period = self.get_parameter('timer_period').get_parameter_value().double_value

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

        self.latest_lat = -1
        self.latest_lon = -1
        self.create_subscription(
            self.LeaderMsgType,
            self.leader_gps_topic,
            self.gps_callback,
            10
        )

        # log
        self.get_logger().info(f"Initialized InformedLeaderNode with GPS topic: {self.leader_gps_topic} and message type: {self.leader_gps_msg_type}")
        if not self.is_slave:
            self.timer = self.create_timer(self.timer_period, self.ping_follower)
        else:
            def listen_for_broadcast():
                self.get_logger().info('Slave leader listening for #B broadcasts...')
                buffer = ""
                while rclpy.ok():
                    try:
                        data = self.ser.read(self.ser.in_waiting or 1)
                        msg = data.decode('utf-8', errors='ignore')
                        if msg:
                            buffer += msg
                            self.get_logger().debug(f'Slave received serial: {msg.strip()}')
                            # Process each line in buffer
                            while '\n' in buffer:
                                line, buffer = buffer.split('\n', 1)
                                line = line.strip()
                                if line.startswith('#B'):
                                    self.get_logger().info('Received broadcast: ' + line)
                                    time.sleep(1)
                                    self.ping_follower()
                    except Exception as e:
                        self.get_logger().error(f"Error in slave listen thread: {e}")
                        time.sleep(0.1)
            self.listen_thread = threading.Thread(target=listen_for_broadcast, daemon=True)
            self.listen_thread.start()

        self.failure_count = 0  # Track consecutive failures
        self.max_failures = 1   # Reset serial after this many failures

    def gps_callback(self, msg):
        if self.leader_gps_msg_type == 'GeoPoint':
            self.latest_lat = msg.latitude
            self.latest_lon = msg.longitude
        elif self.leader_gps_msg_type == 'NavSatFix':
            self.latest_lat = msg.latitude
            self.latest_lon = msg.longitude
        # Add more types as needed

    def reset_serial(self):
        try:
            self.get_logger().warn("Resetting serial port due to repeated failures...")
            self.ser.close()
            time.sleep(1)
            self.ser = init_serial(self.port, self.port_fallback, self.baudrate, self.get_logger())
            if self.ser is not None:
                self.get_logger().info("Serial port re-opened successfully.")
            else:
                self.get_logger().error("Failed to re-open serial port!")
        except Exception as e:
            self.get_logger().error(f"Exception during serial reset: {e}")

    def ping_follower(self):
        self.get_logger().info(f"Sending ping command: {self.ping_command}")
        try:
            ping_time = time.time()
            self.ser.write(self.ping_command.encode())
            read_so_far = ""
            while True:
                if time.time() - ping_time > self.timeout_threshold:
                    self.get_logger().error("Ping timed out")
                    try:
                        if hasattr(self.ser, 'reset_input_buffer'):
                            self.ser.reset_input_buffer()
                        if hasattr(self.ser, 'reset_output_buffer'):
                            self.ser.reset_output_buffer()
                        self.get_logger().warn("Serial buffers cleared after timeout.")
                    except Exception as e:
                        self.get_logger().error(f"Failed to clear serial buffers: {e}")
                    time.sleep(0.5)
                    self.failure_count += 1
                    if self.failure_count >= self.max_failures:
                        self.reset_serial()
                        self.failure_count = 0
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
                if '$' in response_str:
                    self.get_logger().warn("Timeout or malformed response received, ignoring")
                    try:
                        if hasattr(self.ser, 'reset_input_buffer'):
                            self.ser.reset_input_buffer()
                        if hasattr(self.ser, 'reset_output_buffer'):
                            self.ser.reset_output_buffer()
                        self.get_logger().warn("Serial buffers cleared after malformed response.")
                    except Exception as e:
                        self.get_logger().error(f"Failed to clear serial buffers: {e}")
                    self.failure_count += 1
                    if self.failure_count >= self.max_failures:
                        self.reset_serial()
                        self.failure_count = 0
                    return
                dist = 0.00003125 * self.sound_velocity * float(response_str)
                self.get_logger().info(f"Distance: {dist:.3f} meters")
                dist = round(dist, 3)
                if self.latest_lat == -1 or self.latest_lon == -1:
                    self.get_logger().error("Latest GPS position not available, cannot send position+distance")
                    return
                msg_str = f"$B{len(str(self.latest_lat)) + len(str(self.latest_lon)) + len(str(dist)) + 2}{self.latest_lat},{self.latest_lon},{dist}\r\n"
                self.get_logger().info(f"Sending position+distance: {msg_str.strip()}")
                if hasattr(self.ser, 'reset_input_buffer'):
                    self.ser.reset_input_buffer()
                if hasattr(self.ser, 'reset_output_buffer'):
                    self.ser.reset_output_buffer()
                self.get_logger().info("Serial buffers cleared before sending position+distance.")
                self.ser.write(msg_str.encode())
                self.get_logger().info(f"Sent position+distance: {msg_str.strip()}")
                self.failure_count = 0  # Reset failure count on success
            except (IndexError, ValueError) as e:
                self.get_logger().error(f"Failed to parse response: {e}")
                self.failure_count += 1
                if self.failure_count >= self.max_failures:
                    self.reset_serial()
                    self.failure_count = 0
        except Exception as e:
            self.get_logger().error(f"Serial communication error: {e}")
            self.failure_count += 1
            if self.failure_count >= self.max_failures:
                self.reset_serial()
                self.failure_count = 0

def main(args=None):
    rclpy.init(args=args)
    node = InformedLeaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
