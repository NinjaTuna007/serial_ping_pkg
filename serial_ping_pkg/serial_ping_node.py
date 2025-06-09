import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from serial_ping_pkg.utils import load_yaml_config, init_serial
import time

class SerialPingNode(Node):
    def __init__(self):
        super().__init__('serial_ping_node')

        # Load config from YAML and declare parameters
        config = load_yaml_config('serial_ping_pkg', 'serial_config.yaml')
        self.declare_parameter('serial.port', config['serial']['port'])
        self.declare_parameter('serial.port_fallback', config['serial']['port_fallback'])
        self.declare_parameter('serial.baudrate', config['serial']['baudrate'])
        self.declare_parameter('serial.leader1', config['serial']['leader1'])
        self.declare_parameter('serial.leader2', config['serial']['leader2'])
        self.declare_parameter('serial.sound_velocity', config['serial']['sound_velocity'])
        self.declare_parameter('serial.timeout_threshold', config['serial']['timeout_threshold'])

        self.port = self.get_parameter('serial.port').get_parameter_value().string_value
        self.port_fallback = self.get_parameter('serial.port_fallback').get_parameter_value().string_value
        self.baudrate = self.get_parameter('serial.baudrate').get_parameter_value().integer_value
        self.leader1_ping_command = self.get_parameter('serial.leader1').get_parameter_value().string_value
        self.leader2_ping_command = self.get_parameter('serial.leader2').get_parameter_value().string_value
        self.sound_velocity = self.get_parameter('serial.sound_velocity').get_parameter_value().double_value
        self.timeout_threshold = self.get_parameter('serial.timeout_threshold').get_parameter_value().double_value

        self.last_pinged = None
        self.ping_time = None
        self.timed_out = False

        # Initialize serial using utils
        self.ser = init_serial(self.port, self.port_fallback, self.baudrate, self.get_logger())
        if self.ser is None:
            rclpy.shutdown()
            return

        # Create publishers
        self.distance_1_pub = self.create_publisher(Float32, 'leader1/distance', 10)
        self.distance_2_pub = self.create_publisher(Float32, 'leader2/distance', 10)

        # Timer to run every 0.5 seconds
        self.timer = self.create_timer(0.5, self.alternate_leader_ping)

    def alternate_leader_ping(self):
        if self.last_pinged is None:
            self.command = self.leader1_ping_command
            self.last_pinged = 1
        elif self.last_pinged == 1:
            self.command = self.leader2_ping_command
            self.last_pinged = 2
        else:
            self.command = self.leader1_ping_command
            self.last_pinged = 1

        self.get_logger().info(f"Sending command: {self.command}")

        try:
            self.ping_time = time.time()
            self.ser.write(self.command.encode())
            read_so_far = ""
            while True:
                if time.time() - self.ping_time > self.timeout_threshold:
                    self.get_logger().error("Ping timed out")
                    self.timed_out = True
                    break

                response = self.ser.read(self.ser.in_waiting or 1)
                read_now = response.decode('utf-8').strip()
                read_so_far += read_now

                if "T" in read_so_far:
                    if len(read_so_far.split("T")[1]) >= 5:
                        break

            self.get_logger().info(f"Combined Response: {read_so_far}")

            if self.timed_out:
                if self.last_pinged == 1:
                    self.get_logger().error("Not publishing any distance to Leader 1")
                else:
                    self.get_logger().error("Not publishing any distance to Leader 2")
                self.timed_out = False
                return

            try:
                response_str = read_so_far.split("T")[1]
                dist = 0.00003125 * self.sound_velocity * float(response_str)
                self.get_logger().info(f"Distance: {dist:.3f} meters")

                msg = Float32()
                msg.data = dist
                if self.last_pinged == 1:
                    self.distance_1_pub.publish(msg)
                else:
                    self.distance_2_pub.publish(msg)

            except (IndexError, ValueError) as e:
                self.get_logger().error(f"Failed to parse response: {e}")

        except Exception as e:
            self.get_logger().error(f"Serial communication error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialPingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
